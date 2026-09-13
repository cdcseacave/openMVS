/*
* SceneNeighbors.cpp
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

// View-neighbor selection and the point-cloud sampling that feeds it:
// SampleMeshWithVisibility, ExportMeshToDepthMaps, EstimateNeighborViewsPointCloud,
// SelectNeighborViews, FilterNeighborViews.

#include "Common.h"
#include "Scene.h"


using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define SCENE_USE_OPENMP
#endif

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Scene   "));


// compute point-cloud with visibility info from the existing mesh
//  - sampling: sampling density per squared unit area (if >0), or
//              absolute number of points (if <0), or
//              use existing vertices as samples (if ==0)
void Scene::SampleMeshWithVisibility(REAL sampling, unsigned maxResolution)
{
	ASSERT(!mesh.IsEmpty());
	pointcloud.Release();
	if (sampling < 0) {
		// absolute number of points
		mesh.SamplePoints(ROUND2INT<unsigned>(-sampling), pointcloud);
	} else if (sampling > 0) {
		// sampling density per squared unit area
		mesh.SamplePoints(sampling, pointcloud);
	} else {
		// use existing vertices as samples
		pointcloud.points.Join(mesh.vertices.data(), mesh.vertices.size());
	}
	pointcloud.pointViews.resize(pointcloud.points.size());
	// compute visibility for each point by projecting the mesh onto each image
	constexpr Depth thFrontDepth(0.985f);
	#ifdef SCENE_USE_OPENMP
	#pragma omp parallel for
	for (int64_t _ID=0; _ID<images.size(); ++_ID) {
		const IIndex ID(static_cast<IIndex>(_ID));
	#else
	FOREACH(ID, images) {
	#endif
		const Image& imageData = images[ID];
		unsigned level(0);
		const unsigned nMaxResolution(Image8U::computeMaxResolution(imageData.width, imageData.height, level, 0, maxResolution));
		const REAL scale(imageData.width > imageData.height ? (REAL)nMaxResolution/imageData.width : (REAL)nMaxResolution/imageData.height);
		const cv::Size scaledSize(Image8U::computeResize(imageData.GetSize(), scale));
		const Camera camera(imageData.GetCamera(platforms, scaledSize));
		DepthMap depthMap(scaledSize);
		mesh.Project(camera, depthMap);
		FOREACH(idxPoint, pointcloud.points) {
			const Point3f xz(camera.TransformPointW2I3(Cast<REAL>(pointcloud.points[idxPoint])));
			if (xz.z <= 0)
				continue;
			const Point2f x(xz.x, xz.y);
			if (depthMap.isInsideWithBorder<float,1>(x) && xz.z * thFrontDepth < depthMap(ROUND2INT(x))) {
				#ifdef SCENE_USE_OPENMP
				#pragma omp critical
				#endif
				pointcloud.pointViews[idxPoint].emplace_back(ID);
			}
		}
	}
	// remove points with less than 2 views
	RFOREACH(idx, pointcloud.points) {
		if (pointcloud.pointViews[idx].size() < 2)
			pointcloud.RemovePoint(idx);
		#ifdef SCENE_USE_OPENMP
		else
			pointcloud.pointViews[idx].Sort();
		#endif
	}
	DEBUG_EXTRA("Sampled mesh with visibility info: %u points from %f %s",
		pointcloud.points.size(),
		sampling < 0 ? -sampling : sampling > 0 ? sampling : REAL(mesh.vertices.size()),
		sampling < 0 ? "samples" : sampling > 0 ? "sampling" : "vertices");
} // SampleMeshWithVisibility
/*----------------------------------------------------------------*/

bool Scene::ExportMeshToDepthMaps(const String& baseName)
{
	ASSERT(!images.empty() && !mesh.IsEmpty());
	const String ext(Util::getFileExt(baseName).ToLower());
	const int nType(ext == _T(".dmap") ? 2 : (ext == _T(".pfm") ? 1 : 0));
	if (nType == 2)
		mesh.ComputeNormalVertices();
	DepthMap depthMap;
	NormalMap normalMap;
	#ifdef SCENE_USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for private(depthMap, normalMap) schedule(dynamic)
	for (int _i=0; _i<(int)images.size(); ++_i) {
		#pragma omp flush (bAbort)
		if (bAbort)
			continue;
		const IIndex idxImage((IIndex)_i);
	#else
	FOREACH(idxImage, images) {
	#endif
		Image& image = images[idxImage];
		if (!image.IsValid())
			continue;
		const unsigned imageSize(image.RecomputeMaxResolution(OPTDENSE::nResolutionLevel, OPTDENSE::nMinResolution, OPTDENSE::nMaxResolution));
		image.ResizeImage(imageSize);
		image.UpdateCamera(platforms);
		depthMap.create(image.GetSize());
		if (nType == 2)
			mesh.Project(image.camera, depthMap, normalMap);
		else
			mesh.Project(image.camera, depthMap);
		const String fileName(Util::insertBeforeFileExt(baseName, String::FormatString("%04u", image.ID)));
		if ((nType == 2 && ![&]() {
				IIndexArr IDs(0, image.neighbors.size()+1);
				IDs.push_back(idxImage);
				for (const ViewScore& neighbor: image.neighbors)
					IDs.push_back(neighbor.ID);
				return ExportDepthDataRaw(fileName, image.name, IDs, image.GetSize(), image.camera.K, image.camera.R, image.camera.C, 0.001f, FLT_MAX, depthMap, normalMap, ConfidenceMap(), ViewsMap());
			} ()) ||
			(nType == 1 && !depthMap.Save(fileName)) ||
			(nType == 0 && !ExportDepthMap(fileName, depthMap)))
		{
			#ifdef SCENE_USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			continue;
			#else
			return false;
			#endif
		}
	}
	#ifdef SCENE_USE_OPENMP
	if (bAbort)
		return false;
	#endif
	return true;
} // ExportMeshToDepthMaps
/*----------------------------------------------------------------*/
// create a virtual point-cloud to be used to initialize the neighbor view
// from image pair points at the intersection of the viewing directions
bool Scene::EstimateNeighborViewsPointCloud(unsigned maxResolution)
{
	constexpr Depth minPercentDepthPerturb(0.3f);
	constexpr Depth maxPercentDepthPerturb(1.3f);
	const auto ProjectGridToImage = [&](IIndex idI, IIndex idJ, Depth depth) {
		const Depth minDepthPerturb(depth * minPercentDepthPerturb);
		const Depth maxDepthPerturb(depth * maxPercentDepthPerturb);
		const Image& imageData = images[idI];
		const Image& imageData2 = images[idJ];
		const float stepW((float)imageData.width / maxResolution);
		const float stepH((float)imageData.height / maxResolution);
		for (unsigned r = 0; r < maxResolution; ++r) {
			for (unsigned c = 0; c < maxResolution; ++c) {
				const Point2f x(c*stepW + stepW/2, r*stepH + stepH/2);
				const Depth depthPerturb(randomRange(minDepthPerturb, maxDepthPerturb));
				const Point3 X(imageData.camera.TransformPointI2W(Point3(x.x, x.y, depthPerturb)));
				const Point3 X2(imageData2.camera.TransformPointW2C(X));
				if (X2.z < 0)
					continue;
				const Point2f x2(imageData2.camera.TransformPointC2I(X2));
				if (!Image8U::isInside(x2, imageData2.GetSize()))
					continue;
				pointcloud.points.emplace_back(X);
				pointcloud.pointViews.emplace_back(idI < idJ ? PointCloud::ViewArr{idI, idJ} : PointCloud::ViewArr{idJ, idI});
			}
		}
	};
	pointcloud.Release();
	FOREACH(i, images) {
		const Image& imageData = images[i];
		if (!imageData.IsValid())
			continue;
		FOREACH(j, images) {
			if (i == j)
				continue;
			const Image& imageData2 = images[j];
			Point3 X;
			TriangulatePoint3D(
				imageData.camera.K, imageData2.camera.K,
				imageData.camera.R, imageData2.camera.R,
				imageData.camera.C, imageData2.camera.C,
				Point2::ZERO, Point2::ZERO, X);
			const Depth depth((Depth)imageData.camera.PointDepth(X));
			const Depth depth2((Depth)imageData2.camera.PointDepth(X));
			if (depth <= 0 || depth2 <= 0)
				continue;
			ProjectGridToImage(i, j, depth);
			ProjectGridToImage(j, i, depth2);
		}
	}
	return true;
} // EstimateNeighborViewsPointCloud
/*----------------------------------------------------------------*/

// compute visibility for the reference image
// and select the best views for reconstructing the dense point-cloud;
// extract also all 3D points seen by the reference image;
// (inspired by: "Multi-View Stereo for Community Photo Collections", Goesele, 2007)
//  - fWeightPointInsideROI: 0 - ignore ROI, between 0 and 1 - weight inside ROI points, 1 - consider only ROI points
bool Scene::SelectNeighborViews(uint32_t ID, IndexArr& points, unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle, float fWeightPointInsideROI)
{
	ASSERT(points.empty());

	// extract the estimated 3D points and the corresponding 2D projections for the reference image
	Image& imageData = images[ID];
	ASSERT(imageData.IsValid());
	ViewScoreArr& neighbors = imageData.neighbors;
	ASSERT(neighbors.empty());
	struct Score {
		float score;
		float avgAngle;
		uint32_t points;
	};
	CLISTDEF0(Score) scores(images.size());
	scores.Memset(0);
	// the footprint ratio of every shared observation, per view: a view's scale is their trimmed
	// mean, as one mis-triangulated point next to a camera center has a ratio of millions, enough
	// to drag a plain mean -- and with it the size the view's image is resampled to -- anywhere
	struct ScaleRatio {
		uint32_t view;
		float ratio;
	};
	CLISTDEF0(ScaleRatio) scaleRatios;
	if (nMinPointViews > nCalibratedImages)
		nMinPointViews = nCalibratedImages;
	unsigned nPoints = 0;
	imageData.avgDepth = 0;
	ASSERT(fWeightPointInsideROI >= 0 && fWeightPointInsideROI <= 1);
	const bool bCheckInsideROI(fWeightPointInsideROI > 0 && IsBounded());
	const float fWeightPointOutsideROI(bCheckInsideROI ? 1.f - fWeightPointInsideROI : 1.f);
	const float sigmaAngleSmall(-1.f/(2.f*SQUARE(fOptimAngle*0.38f)));
	const float sigmaAngleLarge(-1.f/(2.f*SQUARE(fOptimAngle*0.7f)));
	FOREACH(idx, pointcloud.points) {
		const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
		ASSERT(views.IsSorted());
		if (views.FindFirst(ID) == PointCloud::ViewArr::NO_INDEX)
			continue;
		const PointCloud::Point& point = pointcloud.points[idx];
		const Depth depth((float)imageData.camera.PointDepth(point));
		ASSERT(depth > 0);
		if (depth <= 0)
			continue;
		// store this point
		if (views.size() >= nMinPointViews)
			points.push_back((uint32_t)idx);
		const float wROI(bCheckInsideROI && obb.Intersects(point) ? fWeightPointInsideROI : fWeightPointOutsideROI);
		if (wROI <= 0)
			continue;
		imageData.avgDepth += depth;
		++nPoints;
		// score shared views
		const Point3f V1(imageData.camera.C - Cast<REAL>(point));
		const float footprint1(imageData.camera.GetFootprintImage(depth));
		for (const PointCloud::View& view: views) {
			if (view == ID)
				continue;
			const Image& imageData2 = images[view];
			const Depth depth2((float)imageData2.camera.PointDepth(point));
			ASSERT(depth2 > 0);
			if (depth2 <= 0)
				continue;
			const Point3f V2(imageData2.camera.C - Cast<REAL>(point));
			const float fAngle(ACOS(ComputeAngle(V1.ptr(), V2.ptr())));
			const float wAngle(EXP(SQUARE(fAngle-fOptimAngle)*(fAngle<fOptimAngle?sigmaAngleSmall:sigmaAngleLarge)));
			const float footprint2(imageData2.camera.GetFootprintImage(depth2));
			const float fScaleRatio(footprint1/footprint2);
			if (!ISFINITE(fScaleRatio))
				continue;
			float wScale;
			if (fScaleRatio > 1.6f)
				wScale = SQUARE(1.6f/fScaleRatio);
			else if (fScaleRatio >= 1.f)
				wScale = 1.f;
			else
				wScale = SQUARE(fScaleRatio);
			Score& score = scores[view];
			score.score += MAXF(wAngle,0.1f) * wScale * wROI;
			scaleRatios.push_back(ScaleRatio{view, fScaleRatio});
			score.avgAngle += fAngle;
			++score.points;
		}
	}
	if(nPoints > 3)
		imageData.avgDepth /= nPoints;
	// bucket the ratios by view, a counting sort on the per-view counts already known, so each
	// neighbor's scale below -- the trimmed mean of its ratios, dropping the lowest and highest 10%,
	// at least one each so a view sharing only a handful of points still loses its outlier -- stays
	// linear in the number of shared observations; afterwards ends[v] is one past view v's bucket
	FloatArr ratios;
	ratios.resize(scaleRatios.size());
	UnsignedArr ends(scores.size());
	unsigned offset(0);
	FOREACH(v, scores) {
		ends[v] = offset;
		offset += scores[v].points;
	}
	ASSERT(offset == scaleRatios.size());
	for (const ScaleRatio& r: scaleRatios)
		ratios[ends[r.view]++] = r.ratio;

	// select best neighborViews
	if (neighbors.empty()) {
		Point2fArr projs(0, points.size());
		FOREACH(IDB, images) {
			const Image& imageDataB = images[IDB];
			if (!imageDataB.IsValid())
				continue;
			const Score& score = scores[IDB];
			if (score.points < 3)
				continue;
			ASSERT(ID != IDB);
			// compute how well the matched features are spread out (image covered area)
			const Point2f boundsA(imageData.GetSize());
			const Point2f boundsB(imageDataB.GetSize());
			ASSERT(projs.empty());
			for (uint32_t idx: points) {
				const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
				ASSERT(views.IsSorted());
				ASSERT(views.FindFirst(ID) != PointCloud::ViewArr::NO_INDEX);
				if (views.FindFirst(IDB) == PointCloud::ViewArr::NO_INDEX)
					continue;
				const PointCloud::Point& point = pointcloud.points[idx];
				Point2f ptB = std::get<0>(imageDataB.camera.ProjectPointP(point));
				if (!imageDataB.camera.IsInside(ptB, boundsB))
					continue;
				Point2f& ptA = projs.emplace_back(std::get<0>(imageData.camera.ProjectPointP(point)));
				if (!imageData.camera.IsInside(ptA, boundsA))
					projs.RemoveLast();
			}
			ASSERT(projs.size() <= score.points);
			if (projs.empty())
				continue;
			const float area(ComputeCoveredArea<float,2,16,false>((const float*)projs.data(), projs.size(), boundsA.ptr()));
			projs.Empty();
			// store image score
			ViewScore& neighbor = neighbors.AddEmpty();
			neighbor.ID = IDB;
			neighbor.points = score.points;
			neighbor.scale = FloatArr::GetTrimmedMean(ratios.data()+ends[IDB]-score.points, ratios.data()+ends[IDB], 0.1f, 0.1f, 1);
			neighbor.angle = score.avgAngle/score.points;
			neighbor.area = area;
			neighbor.score = score.score*MAXF(area,0.01f);
		}
		neighbors.Sort([](const ViewScore& i, const ViewScore& j) {
			return i.score > j.score;
		});
		#if TD_VERBOSE != TD_VERBOSE_OFF
		// print neighbor views
		if (VERBOSITY_LEVEL > 2) {
			String msg;
			FOREACH(n, neighbors)
				msg += String::FormatString(" %3u(%upts,%.2fscl)", neighbors[n].ID, neighbors[n].points, neighbors[n].scale);
			VERBOSE("Reference image %3u sees %u views:%s (%u shared points)", ID, neighbors.size(), msg.c_str(), nPoints);
		}
		#endif
	}
	if (points.size() <= 3 || neighbors.size() < MINF(nMinViews,nCalibratedImages-1)) {
		DEBUG_EXTRA("error: reference image %3u has not enough images in view", ID);
		return false;
	}
	return true;
} // SelectNeighborViews

void Scene::SelectNeighborViews(unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle, float fWeightPointInsideROI)
{
	#ifdef SCENE_USE_OPENMP
	for (int_t ID=0; ID<(int_t)images.size(); ++ID) {
		const IIndex idxImage((IIndex)ID);
	#else
	FOREACH(idxImage, images) {
	#endif
		// select image neighbors
		IndexArr points;
		SelectNeighborViews(idxImage, points, nMinViews, nMinPointViews, fOptimAngle, fWeightPointInsideROI);
	}
} // SelectNeighborViews
/*----------------------------------------------------------------*/


// keep only the best neighbors for the reference image
bool Scene::FilterNeighborViews(ViewScoreArr& neighbors, float fMinArea, float fMinScale, float fMaxScale, float fMinAngle, float fMaxAngle, unsigned nMaxViews)
{
	// remove invalid neighbor views
	const unsigned nMinViews(MAXF(4u, nMaxViews*3/4));
	RFOREACH(n, neighbors) {
		const ViewScore& neighbor = neighbors[n];
		if (neighbors.size() > nMinViews &&
			(neighbor.area < fMinArea ||
			 !ISINSIDE(neighbor.scale, fMinScale, fMaxScale) ||
			 !ISINSIDE(neighbor.angle, fMinAngle, fMaxAngle)))
			neighbors.RemoveAtMove(n);
	}
	if (neighbors.size() > nMaxViews)
		neighbors.resize(nMaxViews);
	// a view kept despite an out-of-range scale (too few views to drop any) is still resampled by
	// it: bound the scale, so no neighbor image is ever resized past what the filter allows
	for (ViewScore& neighbor: neighbors)
		neighbor.scale = CLAMP(neighbor.scale, fMinScale, fMaxScale);
	return !neighbors.empty();
} // FilterNeighborViews
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
