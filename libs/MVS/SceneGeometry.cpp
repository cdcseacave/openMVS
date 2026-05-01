/*
* SceneGeometry.cpp
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

// Geometric / KD-tree-based Scene methods moved out of Scene.cpp to isolate the
// CGAL and nanoflann template-instantiation cost into a smaller TU (see the
// comment in libs/MVS/CMakeLists.txt).
//
// Contains:
//   * Scene::EstimatePointCloudNormals (uses lmmin + ZNCC refinement, no CGAL/nanoflann)
//   * Scene::EstimateSparseSurface     (nanoflann KD-tree)
//   * Scene::CropToROI                 (no CGAL directly; relies on helpers below)
//   * Scene::ROIPointWeights           (calls ComputeMeanDistanceToClosestN -> CGAL K-NN)
//   * file-scope helpers used by ROIPointWeights

#include "Common.h"
#include "Scene.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Kd_tree.h>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define SCENE_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

// estimate normals for the point-cloud using the views per point
bool Scene::EstimatePointCloudNormals(bool bRefine)
{
	if (!pointcloud.IsValid() || images.empty())
		return false; // no views available
	if (pointcloud.normals.size() == pointcloud.points.size())
		return true; // normals already estimated
	pointcloud.normals.resize(pointcloud.points.size());
	// estimate normals using the views per point
	#ifdef SCENE_USE_OPENMP
	#pragma omp parallel for
	for (int64_t _ID=0; _ID<(int64_t)pointcloud.points.size(); ++_ID) {
		const IIndex ID(static_cast<IIndex>(_ID));
	#else
	FOREACH(ID, pointcloud.points) {
	#endif
		const PointCloud::Point& point = pointcloud.points[ID];
		const PointCloud::ViewArr& views = pointcloud.pointViews[ID];
		ASSERT(views.size() >= 2);
		// compute the normal as the average over the viewing directions
		Point3 viewDirSum(Point3::ZERO);
		FOREACH(viewIdx, views) {
			const Image& imageData = images[views[viewIdx]];
			ASSERT(imageData.IsValid());
			const Point3 viewDir = normalized(imageData.camera.C - Cast<REAL>(point));
			viewDirSum += viewDir;
		}
		pointcloud.normals[ID] = normalized(viewDirSum);
	}
	if (!bRefine)
		return true; // coarse normals estimated, but skip refinement

	// Refine normals using ZNCC correlation between the point views
	// for each point, the depth is known, and we have a coarse normal estimate;
	// choose the target view as the view with the best score, score computed as
	// score = exp(-0.5 * (angle(viewDir, normal) / sigma)^2) / Camera::GetFootprintWorld(depth);
	// using the homography matrix given by the plane define by the point and the normal,
	// project each pixel from the target view patch to every reference view and compute the ZNCC score;
	// use gradient descent to refine the normal estimate, keeping the depth constant.

	// Load images
	// TODO: replace with images cache
	bool bImagesReloaded(false);
	#ifdef SCENE_USE_OPENMP
	#pragma omp parallel for
	for (int64_t _idx=0; _idx<(int64_t)images.size(); ++_idx) {
		const IIndex idx(static_cast<IIndex>(_idx));
	#else
	FOREACH(idx, images) {
	#endif
		Image& imageData = images[idx];
		if (!imageData.IsValid())
			continue;
		if (imageData.image.empty())
			bImagesReloaded = true; // need to reload images
		if (!imageData.ReloadImage(1024)) {
			DEBUG("error: cannot reload image '%s'", imageData.name.c_str());
			exit(EXIT_FAILURE);
		}
		imageData.UpdateCamera(platforms);
	}

	// Refine normals using lmmin optimization with ZNCC correlation
	constexpr int patchRadius = 3; // Half-size of the patch window
	constexpr int patchSize = patchRadius * 2 + 1;
	constexpr int nTexels = patchSize * patchSize;
	constexpr float sigmaAngle = FD2R(15.f); // 15 degrees sigma for angle weighting
	constexpr float sigmaAngleInv = -1.f / (2.f * SQUARE(sigmaAngle));
	typedef Sampler::Linear<float> Sampler;
	const Sampler sampler;
	typedef RobustNorm::Cauchy<double> RobustNormFunc;
	const RobustNormFunc robust(0.7);

	// Define optimization data structure
	struct NormalOptimizationData {
		const PointCloud::Point& point;
		const PointCloud::ViewArr& views;
		const ImageArr& images;
		IIndex targetViewIdx;
		Point2f targetProjection;
		std::array<float,nTexels> targetPatch;
		double targetVariance;
		const Sampler& sampler;
		const RobustNormFunc& robust;

		NormalOptimizationData(const PointCloud::Point& _point, const PointCloud::ViewArr& _views,
							   const ImageArr& _images, IIndex _targetViewIdx, const Point2f& _targetProjection,
							   const std::array<float,nTexels>& _targetPatch,
							   double _targetVariance, const Sampler& _sampler, const RobustNormFunc& _robust)
			: point(_point), views(_views), images(_images), targetViewIdx(_targetViewIdx),
			  targetProjection(_targetProjection), targetPatch(_targetPatch),
			  targetVariance(_targetVariance), sampler(_sampler), robust(_robust) {}

		static void Residuals(const double* x, int nPoints, const void* pData, double* fvec, double* fjac, int* /*info*/) {
			const NormalOptimizationData& data = *reinterpret_cast<const NormalOptimizationData*>(pData);
			ASSERT(fjac == NULL); // We don't provide Jacobian, let lmmin compute it numerically
			// Convert spherical coordinates to normal vector
			Point3 normal;
			Dir2Normal(*reinterpret_cast<const Point2d*>(x), normal);
			// Ensure normal points toward target camera
			const Camera& targetCamera = data.images[data.views[data.targetViewIdx]].camera;
			const Point3 viewDir = normalized(targetCamera.C - Cast<REAL>(data.point));
			if (normal.dot(viewDir) < 0)
				normal = -normal;
			const Plane plane(normal, Cast<REAL>(data.point));
			// Compute ZNCC residuals for each reference view
			FOREACH(refViewIdx, data.views) {
				if (refViewIdx == data.targetViewIdx) {
					fvec[refViewIdx] = 0; // zero residual if target view
					continue;
				}
				const Image& refImage = data.images[data.views[refViewIdx]];
				ASSERT(refImage.IsValid() && !refImage.image.empty());
				// Sample reference patch using plane projection
				std::array<float,nTexels> refPatch;
				int validTexels = 0;
				double refMean = 0.f;
				for (int dy = -patchRadius; dy <= patchRadius; ++dy) {
					for (int dx = -patchRadius; dx <= patchRadius; ++dx) {
						const Point2f targetPos = data.targetProjection + Point2f(dx, dy);
						// Back-project target pixel to 3D using the refined normal
						const Ray3 ray(targetCamera.C, normalized(targetCamera.RayPoint(Cast<REAL>(targetPos))));
						// Intersect ray with plane to get 3D point
						Point3::EVec X3D;
						if (!ray.Intersects(plane, false, NULL, &X3D))
							continue;
						// Project 3D point to reference image
						const Point2f refPos = refImage.camera.TransformPointW2I(Point3(X3D));
						if (refImage.image.isInsideWithBorder<float,1>(refPos)) {
							const Pixel32F pixelValue = refImage.image.sample<Sampler,Pixel32F>(data.sampler, refPos);
							const float intensity = pixelValue.r * 0.299f + pixelValue.g * 0.587f + pixelValue.b * 0.114f;
							refPatch[validTexels++] = intensity;
							refMean += intensity;
						}
					}
				}
				if (validTexels < nTexels) {
					fvec[refViewIdx] = 0.9; // no valid texture, large residual
					continue;
				}
				refMean /= nTexels;
				// Compute reference patch variance and ZNCC
				double refVariance(0), correlation(0);
				for (int i = 0; i < nTexels; ++i) {
					const double refDiff = static_cast<double>(refPatch[i]) - refMean;
					refVariance += refDiff * refDiff;
					correlation += static_cast<double>(data.targetPatch[i]) * refDiff;
				}
				// Set residuals
				if (refVariance > 1e-8) {
					const double zncc = CLAMP(correlation / SQRT(data.targetVariance * refVariance), -1.0, 1.0);
					fvec[refViewIdx] = data.robust(1.0 - zncc); // maximize ZNCC, so minimize negative ZNCC
				} else {
					fvec[refViewIdx] = 0.9; // no valid texture, large residual
				}
			}
		}
	};

	#ifdef SCENE_USE_OPENMP
	#pragma omp parallel for
	for (int64_t _ID=0; _ID<(int64_t)pointcloud.points.size(); ++_ID) {
		const IIndex ID(static_cast<IIndex>(_ID));
	#else
	FOREACH(ID, pointcloud.points) {
	#endif
		const PointCloud::Point& point = pointcloud.points[ID];
		const PointCloud::ViewArr& views = pointcloud.pointViews[ID];
		Point3f& normal = pointcloud.normals[ID];
		// Find the best target view based on angle and footprint
		IIndex bestTargetIdx = NO_ID;
		float bestScore = -1.f;
		Point2f bestProjection;
		FOREACH(viewIdx, views) {
			const Image& imageData = images[views[viewIdx]];
			ASSERT(imageData.IsValid() && !imageData.image.empty());
			const Camera& camera = imageData.camera;
			const Point3f viewDir = normalized(camera.C - Cast<REAL>(point));
			// Project point to image
			const auto [projection, depth] = camera.ProjectPointP(point);
			if (depth <= 0 || !imageData.image.isInsideWithBorder<float>(projection, patchRadius))
				continue;
			// Compute view score: angle compatibility and footprint
			const float angle = ACOS(ComputeAngleN(normal.ptr(), viewDir.ptr()));
			const float angleWeight = EXP(SQUARE(angle) * sigmaAngleInv);
			const float footprint = camera.GetFootprintImage(depth);
			const float score = angleWeight / footprint;
			if (score > bestScore) {
				bestScore = score;
				bestTargetIdx = viewIdx;
				bestProjection = projection;
			}
		}
		if (bestTargetIdx == NO_ID)
			continue;
		const Image& targetImage = images[views[bestTargetIdx]];

		// Extract target patch - convert to grayscale intensities
		std::array<float,nTexels> targetPatch;
		double targetMean(0);
		int validTexels = 0;
		for (int dy = -patchRadius; dy <= patchRadius; ++dy) {
			for (int dx = -patchRadius; dx <= patchRadius; ++dx) {
				const Point2f samplePos = bestProjection + Point2f(dx, dy);
				if (targetImage.image.isInsideWithBorder<float,1>(samplePos)) {
					const Pixel32F pixelValue = targetImage.image.sample<Sampler,Pixel32F>(sampler, samplePos);
					const float intensity = pixelValue.r * 0.299f + pixelValue.g * 0.587f + pixelValue.b * 0.114f;
					targetPatch[validTexels++] = intensity;
					targetMean += intensity;
				}
			}
		}
		if (validTexels < nTexels)
			continue;
		targetMean /= nTexels;

		// Compute target patch variance
		double targetVariance(0);
		for (int i = 0; i < validTexels; ++i) {
			const double diff = static_cast<double>(targetPatch[i]) - targetMean;
			targetPatch[i] = diff; // Store normalized values
			targetVariance += diff * diff;
		}
		if (targetVariance < 1e-6) // Skip texture-less patches
			continue;

		// Create optimization data
		Point2d paramN;
		Normal2Dir(normal, paramN); // Convert normal to spherical coordinates
		NormalOptimizationData optData(point, views, images, bestTargetIdx, bestProjection,
									   targetPatch, targetVariance, sampler, robust);
		// Setup and run lmmin optimization
		constexpr int numParams(2);
		lm_control_struct control{1.e-6, 1.e-7, 1.e-8, 1.e-7, 100.0, 100}; // similar to lm_control_float
		lm_status_struct status;
		lmmin(numParams, paramN.ptr(), views.size(), &optData, NormalOptimizationData::Residuals, &control, &status);
		// Check if optimization succeeded and update normal
		if (status.info < 4) {
			// Convert optimized spherical coordinates back to normal vector
			Dir2Normal(paramN, normal);
			// Ensure normal points toward target camera
			const Point3f viewDir = normalized(targetImage.camera.C - Cast<REAL>(point));
			if (normal.dot(viewDir) < 0)
				normal = -normal; // Flip normal if it points away from target camera
		}
		// Note: If optimization fails, keep the original normal estimate
	}

	if (bImagesReloaded) {
		// Release images
		for (Image& imageData: images)
			imageData.ReleaseImage();
	}
	return true;
} // EstimatePointCloudNormals
/*----------------------------------------------------------------*/

// Build an approximate surface from the sparse point cloud by creating
// an oriented square (as two triangles) centered at each point, aligned by its normal.
// The square size is estimated from local neighbor spacing using a KD-tree (nanoflann).
// - kNeighbors: number of neighbors used to estimate spacing (>=3)
// - sizeScale: scales the median neighbor distance to get the square side (typ. 0.8-1.0)
// - normalAngleMax: only neighbors with normals within this angle are considered (radians)
namespace {
// nanoflann adaptor for PointCloud::points (3D float)
struct PointCloudAdaptor3f {
	const MVS::PointCloud::Point* pts; size_t n;
	inline PointCloudAdaptor3f(const MVS::PointCloud::Point* p, size_t _n): pts(p), n(_n) {}
	inline size_t kdtree_get_point_count() const { return n; }
	inline float kdtree_get_pt(const size_t idx, int dim) const { return pts[idx][dim]; }
	template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};
} // anonymous namespace
bool Scene::EstimateSparseSurface(unsigned kNeighbors, float sizeScale, float normalAngleMax)
{
	// Ensure normals exist
	mesh.Release();
	if (pointcloud.normals.size() != pointcloud.points.size() && !EstimatePointCloudNormals())
		return false;

	// Build KD-tree over sparse points
	PointCloudAdaptor3f adaptor(pointcloud.points.data(), pointcloud.points.size());
	using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor3f>, PointCloudAdaptor3f, 3>;
	KDTree kdtree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams());
	kdtree.buildIndex();

	const uint32_t N = pointcloud.points.size();
	const unsigned k = MAXF(3u, kNeighbors);
	const float cosMax = COS(normalAngleMax);
	const nanoflann::SearchParameters searchParams(0, false);

	// Compute per-point square half-size using median neighbor distance from co-planar neighbors
	std::vector<float> halfSizes(N);
	std::vector<size_t> idxs(k+1);
	std::vector<float> dists(k+1);
	for (uint32_t i = 0; i < N; ++i) {
		nanoflann::KNNResultSet<float> rs(k+1);
		rs.init(idxs.data(), dists.data());
		kdtree.findNeighbors(rs, pointcloud.points[i].ptr(), searchParams);
		// Collect neighbor distances that are roughly co-planar (normal-aligned)
		FloatArr neighDists(0, k);
		const Point3f& n0 = pointcloud.normals[i];
		for (size_t j = 0; j < rs.size(); ++j) {
			const float dSq = dists[j];
			if (dSq <= 0)
				continue; // skip self
			if (normalAngleMax > 0) {
				// keep neighbors with similar surface orientation
				const size_t ni = idxs[j];
				const Point3f& nj = pointcloud.normals[ni];
				const float cosang = ComputeAngleN(n0.ptr(), nj.ptr());
				if (cosang < cosMax)
					continue;
			}
			neighDists.push_back(dSq);
		}
		// median
		const float median = neighDists.size() < 2 ? 0.f : SQRT(neighDists.GetMedian());
		halfSizes[i] = 0.5f * sizeScale * median; // half of side length
	}

	// Skip points with zero half-size or too large half-size
	const auto EstimateMaxHalfSize = [](const std::vector<float>& halfSizes) -> float {
		// Create a copy of halfSizes excluding zero values
		FloatArr nonZeroHalfSizes;
		nonZeroHalfSizes.reserve(halfSizes.size());
		for (float h : halfSizes)
			if (h > 0)
				nonZeroHalfSizes.push_back(h);
		const std::pair<float,float> th(ComputeX84Threshold<float,float>(nonZeroHalfSizes, 7.f));
		return th.first+th.second;
	};
	const float maxHalfSize = EstimateMaxHalfSize(halfSizes);
	uint32_t nValid = 0;
	for (float& h : halfSizes) {
		if (h > 0 && h < maxHalfSize)
			++nValid;
		else
			h = 0;
	}
	if (nValid == 0)
		return false;

	// Allocate mesh: 4 vertices and 2 faces per valid point
	mesh.vertices.resize(nValid * 4);
	mesh.faces.resize(nValid * 2);

	// Build orthonormal frame and write vertices/faces
	auto BuildFrame = [](const Point3f& n, Point3f& u, Point3f& v) {
		// robust tangent basis from normal
		Point3f a = ABS(n.x) > ABS(n.z) ? Point3f(-n.y, n.x, 0.f) : Point3f(0.f, -n.z, n.y);
		u = normalized(a);
		v = normalized(n.cross(u));
	};

	uint32_t outIdx = 0; // quad index
	#ifdef SCENE_USE_OPENMP
	// To allow parallel fill, compute a mapping from input point index to output quad index
	std::vector<uint32_t> quadIndex(N);
	for (uint32_t i = 0; i < N; ++i)
		if (halfSizes[i] > 0)
			quadIndex[i] = outIdx++;
	#pragma omp parallel for schedule(static)
	for (int64_t _i = 0; _i < (int64_t)N; ++_i) {
		const uint32_t i = (uint32_t)_i;
		if (halfSizes[i] <= 0)
			continue;
		const uint32_t qi = quadIndex[i];
	#else
	for (uint32_t i = 0; i < N; ++i) {
		if (halfSizes[i] <= 0)
			continue;
		const uint32_t qi = outIdx++;
	#endif
		const Mesh::VIndex vbase(qi * 4);
		const Mesh::FIndex fbase(qi * 2);
		const Point3f& p = pointcloud.points[i];
		const Point3f& n = pointcloud.normals[i];
		Point3f u, v;
		BuildFrame(n, u, v);
		const float h = halfSizes[i];
		// Square corners in plane
		mesh.vertices[vbase + 0] = p + (-u - v) * h;
		mesh.vertices[vbase + 1] = p + ( u - v) * h;
		mesh.vertices[vbase + 2] = p + ( u + v) * h;
		mesh.vertices[vbase + 3] = p + (-u + v) * h;
		// Two triangles: (0,1,2) and (0,2,3)
		mesh.faces[fbase + 0] = Mesh::Face(vbase + 0, vbase + 1, vbase + 2);
		mesh.faces[fbase + 1] = Mesh::Face(vbase + 0, vbase + 2, vbase + 3);
	}
	return true;
}
/*----------------------------------------------------------------*/

// remove all points outside the given bounding-box and keep only the cameras that see the remaining points
//  - minNumPoints: minimum number of points to keep the camera
Scene& Scene::CropToROI(const OBB3f& obb, unsigned minNumPoints)
{
	ASSERT(obb.IsValid());
	// remove geometry outside the ROI
	if (!pointcloud.IsEmpty())
		pointcloud.RemovePointsOutside(obb);
	if (!mesh.IsEmpty())
		mesh.RemoveFacesOutside(obb);
	// remove cameras that do not see any points
	if (minNumPoints == 0 || !pointcloud.IsValid())
		return *this;
	UnsignedArr visibility(images.size());
	visibility.Memset(0);
	for (const PointCloud::ViewArr& views: pointcloud.pointViews) {
		for (const PointCloud::View& idxImage: views) {
			const Image& imageData = images[idxImage];
			if (!imageData.IsValid())
				continue;
			++visibility[idxImage];
		}
	}
	IIndexArr idxImages;
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		if (visibility[idxImage] >= minNumPoints)
			idxImages.emplace_back(idxImage);
	}
	return *this = SubScene(idxImages);
}


namespace {

void MinMaxScale(FloatArr &arr) {
	if (arr.empty())
		return;
	const auto [minVal, maxVal] = arr.GetMinMax();
	const float range = maxVal - minVal;
	if (range == 0.0f)
		return;
	for (size_t i = 0; i < arr.size(); ++i) {
		arr[i] = (arr[i] - minVal) / range;
	}
}

// Winsorize a vector in place: limits values below the lower percentile and above the upper percentile
void Winsorize(FloatArr& data, float lower_percentile, float upper_percentile) {
	if (data.empty() || lower_percentile < 0.0 || upper_percentile > 100.0 || lower_percentile > upper_percentile) {
		throw std::invalid_argument("Invalid input or percentile range");
	}

	FloatArr sorted_data(data);
	std::sort(sorted_data.begin(), sorted_data.end());

	size_t n = sorted_data.size();
	size_t lower_index = static_cast<size_t>(lower_percentile / 100.0 * (n - 1));
	size_t upper_index = static_cast<size_t>(upper_percentile / 100.0 * (n - 1));

	float lower_value = sorted_data[lower_index];
	float upper_value = sorted_data[upper_index];

	for (auto& value : data) {
		if (value < lower_value) {
			value = lower_value;
		} else if (value > upper_value) {
			value = upper_value;
		}
	}
}

float RadialWeight2D(int width, int height, int x, int y, float alpha=2) {
	float x_center = (width - 1) * 0.5f;
	float y_center = (height - 1) * 0.5f;

	float R = std::sqrt(x_center * x_center + y_center * y_center);

	float dx = x - x_center;
	float dy = y - y_center;
	float distance = std::sqrt(dx * dx + dy * dy);

	float r = distance / R;

	float weight = 1.0f - std::pow(r, alpha);
	return (weight > 0.0f) ? weight : 0.0f;
}

FloatArr ComputeMeanDistanceToClosestN(const PointCloud::PointArr &pts, int numberOfNeighbors) {
	FloatArr meanDistances(pts.size());
	meanDistances.MemsetValue(0);

	typedef CGAL::Simple_cartesian<double>				 K;
	typedef CGAL::Search_traits_3<K>					   TreeTraits;
	typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> K_neighbor_search;
	typedef K_neighbor_search::Tree						Tree;

	std::vector<K::Point_3> cgalPoints;
	cgalPoints.reserve(pts.size());
	// Convert each 3D point to a CGAL point
	for (const auto &p: pts)
		cgalPoints.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
	// Build a KD-tree for neighbor searches
	Tree tree(cgalPoints.begin(), cgalPoints.end());
	// For each point, find its N nearest *other* points and average their distance;
	// query for N+1 neighbors and skip distance-0 hits (the query point itself,
	// plus any coincident duplicates) so the mean isn't biased downward by self
	FOREACH(i, cgalPoints) {
		K_neighbor_search search(tree, cgalPoints[i], numberOfNeighbors + 1);
		double sumDist = 0;
		int count = 0;
		for (const auto& result : search) {
			const double distSq = result.second; // result is std::pair<Point_3, double>
			if (distSq <= 0)
				continue; // skip self / coincident duplicates
			sumDist += SQRT(distSq);
			if (++count >= numberOfNeighbors)
				break;
		}
		if (count > 0)
			meanDistances[i] = static_cast<float>(sumDist / static_cast<double>(count));
	}
	return meanDistances;
}

} // anonymous namespace

// Compute a weight for each point in the scene point cloud based on:
//  - proximity to image center
//  - depth from camera
//  - number of views observing the point
//  - mean distance to closest neighbors in the point cloud
FloatArr Scene::ROIPointWeights() const {
	const int numberOfNeighbors = 16;
	const float meanNeighborDistanceWLambda = 0.25f;
	const float imageCenterWLambda = 0.25f;
	const float numberOfViewsWLambda = 0;
	const float depthWLambda = 1.f - meanNeighborDistanceWLambda - imageCenterWLambda - numberOfViewsWLambda;

	FloatArr imageCenterWeights(pointcloud.points.size());
	FloatArr depthWeights(pointcloud.points.size());
	FloatArr numberOfViewsWeights(pointcloud.points.size());
	FloatArr meanDistanceToClosestN(pointcloud.points.size());
	imageCenterWeights.MemsetValue(0);
	depthWeights.MemsetValue(0);

	FloatArr pointcloudMeanDistanceToClosestN = ComputeMeanDistanceToClosestN(pointcloud.points, numberOfNeighbors);
	FloatArr pointWeights(pointcloud.points.size());
	FOREACH(idxPoint, pointcloud.points) {
		const PointCloud::ViewArr &views = pointcloud.pointViews[idxPoint];
		numberOfViewsWeights[idxPoint] = views.size();
		const float meanDistanceWeight = 1.0f / (1.0f + pointcloudMeanDistanceToClosestN[idxPoint]);
		meanDistanceToClosestN[idxPoint] = meanDistanceWeight;
		FOREACH(idxView, views) {
			int idxImage = views[idxView];
			const Image &image = images[idxImage];
			if (!image.IsValid())
				continue;
			const Point3f &X(pointcloud.points[idxPoint]);
			const Point3 camX(image.camera.TransformPointW2C(Cast<REAL>(X)));
			const Point2i pt(ROUND2INT(image.camera.TransformPointC2I(camX)));
			if (!Image8U::isInside(pt, image.GetSize()))
				continue;
			const float depthWeight = 1.0f / (1.0f + camX.z);
			depthWeights[idxPoint] += depthWeight;
			const float imageCenterWeight = RadialWeight2D(image.width, image.height, pt.x, pt.y, 2.0f);
			imageCenterWeights[idxPoint] += imageCenterWeight;
		}
	}
	for (size_t i = 0; i < pointcloud.points.size(); ++i) {
		depthWeights[i] /= numberOfViewsWeights[i];
		imageCenterWeights[i] /= numberOfViewsWeights[i];
	}

	// Set top 10% and bottom 10% to 10th and 90th quantile, respectively
	Winsorize(imageCenterWeights, 10.f, 90.f);
	Winsorize(depthWeights, 10.f, 90.f);
	Winsorize(meanDistanceToClosestN, 10.f, 90.f);

	MinMaxScale(imageCenterWeights);
	MinMaxScale(depthWeights);
	MinMaxScale(numberOfViewsWeights);
	MinMaxScale(meanDistanceToClosestN);

	for (size_t i = 0; i < pointcloud.points.size(); ++i) {
		pointWeights[i] = imageCenterWLambda * imageCenterWeights[i] +
						  depthWLambda * depthWeights[i] +
						  numberOfViewsWLambda * numberOfViewsWeights[i] +
						  meanNeighborDistanceWLambda * meanDistanceToClosestN[i];
	}

	return pointWeights;
}
/*----------------------------------------------------------------*/
