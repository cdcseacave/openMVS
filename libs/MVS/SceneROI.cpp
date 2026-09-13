/*
* SceneROI.cpp
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

// Region-of-interest estimation: EstimateGravityDirection, EstimateROI and their
// file-scope helpers (ClampedSampleStep, UpSignScore, WeightedQuantiles, GrowBound).

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


// fixed-stride subsampling step bounding the samples taken from the cloud: robust
// aggregates are preserved while the cost stops growing with the cloud size
static size_t ClampedSampleStep(const PointCloud& pointcloud, size_t maxNumSamples)
{
	return MAXF<size_t>(1, pointcloud.points.size()/maxNumSamples);
}

// score the plausibility that the given direction points upward, based on near-universal
// facts of photogrammetric captures; when the mean image-down direction is coherent and
// informative it decides alone (photos are rarely taken upside-down), otherwise fall back
// to: cameras sit above the scene, and cameras look (somewhat downward) toward the scene;
// returns >0 if the direction points up, <0 if it points down, ~0 if undecidable;
// if pSeparation is given, it receives the camera-to-scene centroid separation along the
// direction relative to the scene spread (near zero when perpendicular to the true vertical)
static float UpSignScore(const Eigen::Vector3f& g, const ImageArr& images, const PointCloud& pointcloud, float* pSeparation = NULL)
{
	if (pSeparation)
		*pSeparation = 0;
	Eigen::Vector3f meanC(Eigen::Vector3f::Zero()), meanFwd(Eigen::Vector3f::Zero()), meanDown(Eigen::Vector3f::Zero());
	size_t numCameras(0);
	for (const Image& image: images) {
		if (!image.IsValid())
			continue;
		meanC += Eigen::Vector3f(Cast<float>(image.camera.C));
		meanFwd += Eigen::Vector3f(Cast<float>(Point3(image.camera.R.row(2))));
		meanDown += Eigen::Vector3f(Cast<float>(Point3(image.camera.R.row(1))));
		++numCameras;
	}
	if (numCameras == 0)
		return 0;
	meanC /= (float)numCameras;
	// center of the scene points and their spread along g: median and MAD, as this
	// runs on the raw sparse cloud, before the outliers ROI estimation removes are
	// gone, and a mean/variance lets a few gross outliers collapse the separation
	float sAbove(0);
	bool hasAbove(false);
	if (pointcloud.IsValid() && pointcloud.points.size() >= 100) {
		const size_t sampleStep(ClampedSampleStep(pointcloud, 10000));
		FloatArr projs(0, (IDX)(pointcloud.points.size()/sampleStep+1));
		for (size_t i = 0; i < pointcloud.points.size(); i += sampleStep)
			projs.push_back(g.dot(Eigen::Vector3f(pointcloud.points[(IDX)i])));
		const float center(projs.GetMedian());
		for (float& proj: projs)
			proj = ABS(proj - center);
		const float spread(projs.GetMedian() * 1.4826f); // MAD scaled to match a sigma
		if (spread > 0) {
			sAbove = (meanC.dot(g) - center) / spread;
			hasAbove = true;
			if (pSeparation)
				*pSeparation = ABS(sAbove);
		}
	}
	// the mean image-down direction decides alone when coherent and not (near) perpendicular
	const float downCoherence(meanDown.norm()/(float)numCameras);
	const float downScore(downCoherence > 0 ? -meanDown.normalized().dot(g) : 0.f);
	if (downCoherence > 0.7f && ABS(downScore) > 0.25f)
		return downScore;
	float score(0);
	if (hasAbove)
		score += CLAMP(sAbove, -1.f, 1.f);
	score += -meanFwd.dot(g)/(float)numCameras * 0.5f;
	if (ABS(score) < 0.1f)
		return downScore; // last resort, however weak
	return score;
}

// estimate the gravity (up) direction of the scene by combining two independent cues:
//  - camera roll consensus: photos are typically taken with (near) zero roll, so gravity is
//    perpendicular to the image x-axis of most cameras (or their y-axis if in portrait
//    orientation); solved robustly by scoring candidate directions sampled from pairs of
//    camera x-axes, then refined on the supporting cameras
//  - ground plane: the dominant RANSAC plane of the sparse point-cloud, considered only if
//    well supported and with most of the scene on the camera side (a boundary, as a ground is)
// the two cues are cross-checked: agreement confirms the estimate (the camera gravity is then
// used, being immune to terrain slope), a near-perpendicular plane is discarded as a facade,
// and when the cameras are inconclusive (e.g. all sharing one heading) only a dominant
// ground-like plane is trusted; an ill-conditioned camera consensus (e.g. a single flight
// heading, where the support test cannot tell true gravity from the shared image-down axis)
// is accepted only if externally validated by gravity separating the cameras from the scene;
// returns false (leaving up unchanged) if no confident estimate is found
bool Scene::EstimateGravityDirection(Point3f& up) const
{
	const float maxSinRoll(SIN(D2R(12.f))); // max |sin(roll)| for a camera to support a candidate
	const float minCosConfirm(COS(D2R(20.f))); // min cos(angle) for the two cues to confirm each other
	// collect the world-space image axes of the valid cameras
	std::vector<Eigen::Vector3f> xAxes, yAxes;
	for (const Image& image: images) {
		if (!image.IsValid())
			continue;
		xAxes.emplace_back(Cast<float>(Point3(image.camera.R.row(0))));
		yAxes.emplace_back(Cast<float>(Point3(image.camera.R.row(1))));
	}
	const size_t numCameras(xAxes.size());
	if (numCameras < 4)
		return false;
	Eigen::Vector3f meanDown(Eigen::Vector3f::Zero());
	for (const Eigen::Vector3f& y: yAxes)
		meanDown += y;
	// a camera supports a gravity candidate if the candidate is (nearly) perpendicular
	// to the camera horizontal axis: x in landscape, y in portrait orientation
	const auto SupportsCandidate = [&](const Eigen::Vector3f& g, size_t i) {
		return MINF(ABS(g.dot(xAxes[i])), ABS(g.dot(yAxes[i]))) <= maxSinRoll;
	};
	const auto CountSupport = [&](const Eigen::Vector3f& g) {
		size_t numSupport(0);
		for (size_t i = 0; i < numCameras; ++i)
			if (SupportsCandidate(g, i))
				++numSupport;
		return numSupport;
	};
	// camera cue: candidates from pairs of camera x-axes (y-axes for portrait orientation)
	// with different heading (the cross product of two zero-roll horizontal axes is
	// vertical), plus the mean image-down direction (exact for level photos)
	std::vector<Eigen::Vector3f> candidates;
	if (meanDown.norm() > 0.5f*numCameras)
		candidates.emplace_back(meanDown.normalized());
	// geo-aligned scenes carry the vertical for free: the stored transform maps the
	// absolute (ENU, +Z up) frame to the local one, so the local vertical is its third
	// column; only a candidate though, it still needs camera support to be selected
	if (HasTransform()) {
		const Eigen::Vector3f enuUp(
			(float)transform(0,2), (float)transform(1,2), (float)transform(2,2));
		const float norm(enuUp.norm());
		if (norm > FLT_EPSILON)
			candidates.emplace_back(enuUp/norm);
	}
	const size_t stride(MAXF<size_t>(1, numCameras/16));
	for (const std::vector<Eigen::Vector3f>* axes: {&xAxes, &yAxes}) {
		for (size_t offset = stride; offset <= numCameras/2; offset += stride) {
			for (size_t i = 0; i < numCameras; i += stride) {
				const Eigen::Vector3f cand((*axes)[i].cross((*axes)[(i+offset)%numCameras]));
				const float norm(cand.norm());
				if (norm > 0.2f) // skip near-parallel pairs
					candidates.emplace_back(cand/norm);
			}
		}
	}
	size_t bestSupport(0);
	Eigen::Vector3f gCam(Eigen::Vector3f::Zero());
	for (const Eigen::Vector3f& cand: candidates) {
		const size_t support(CountSupport(cand));
		if (bestSupport < support) {
			bestSupport = support;
			gCam = cand;
		}
	}
	bool camConditioned(false);
	if (bestSupport > 0) {
		// refine: gravity minimizes the sum of squared dot products with the
		// supporting cameras' horizontal axes (smallest eigenvector); the solution
		// is well conditioned only if the horizontal axes span multiple headings
		Eigen::Matrix3f M(Eigen::Matrix3f::Zero());
		for (size_t i = 0; i < numCameras; ++i) {
			if (!SupportsCandidate(gCam, i))
				continue;
			const Eigen::Vector3f& h(ABS(gCam.dot(xAxes[i])) <= ABS(gCam.dot(yAxes[i])) ? xAxes[i] : yAxes[i]);
			M += h*h.transpose();
		}
		const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(M);
		if (es.info() == Eigen::Success) {
			camConditioned = es.eigenvalues()(1) > 0.03f*es.eigenvalues().sum();
			const Eigen::Vector3f refined(es.eigenvectors().col(0));
			const size_t refinedSupport(CountSupport(refined));
			if (refinedSupport > bestSupport || (camConditioned && refinedSupport == bestSupport)) {
				bestSupport = refinedSupport;
				gCam = refined;
			}
		}
	}
	const float camConfidence((float)bestSupport/numCameras);
	// ground plane cue from the sparse point-cloud
	Eigen::Vector3f gPlane(Eigen::Vector3f::Zero());
	float planeSupport(0);
	if (pointcloud.IsValid() && pointcloud.points.size() >= 100) {
		Point3fArr samples;
		const size_t sampleStep(ClampedSampleStep(pointcloud, 20000));
		for (size_t i = 0; i < pointcloud.points.size(); i += sampleStep)
			samples.emplace_back(pointcloud.points[(IDX)i]);
		Planef plane;
		double maxThreshold(DBL_MAX);
		const unsigned numInliers(EstimatePlane(samples, plane, maxThreshold, NULL, 256));
		if (numInliers >= samples.size()/5) {
			// orient the normal such that the cameras lie on its positive side
			float camSide(0);
			for (const Image& image: images)
				if (image.IsValid())
					camSide += plane.Distance(Cast<float>(image.camera.C));
			if (camSide < 0)
				plane.Negate();
			// ground-like: the scene lies (almost) entirely on the camera side of the plane
			const float distTolerance(3*(float)maxThreshold);
			size_t numAbove(0);
			for (const Point3f& X: samples)
				if (plane.Distance(X) >= -distTolerance)
					++numAbove;
			if ((float)numAbove/samples.size() >= 0.85f) {
				gPlane = plane.m_vN;
				planeSupport = (float)numInliers/samples.size();
			}
		}
	}
	// arbitrate between the two cues
	const bool hasCam(camConfidence >= 0.6f);
	const bool hasPlane(planeSupport > 0);
	// an ill-conditioned camera solution can still be validated externally: gravity must
	// separate the cameras from the scene they photograph, while the degenerate solution
	// (the shared image-down axis of a single-heading capture) is perpendicular to it
	float camSeparation(0);
	const float camScore(hasCam ? UpSignScore(gCam, images, pointcloud, &camSeparation) : 0.f);
	const bool camPlausible(camConditioned || camSeparation > 0.2f);
	Eigen::Vector3f g;
	bool bPlaneOriented(false); // set if g comes from the plane cue, whose sign is already fixed
	if (hasCam && hasPlane) {
		const float cosAngle(ABS(gCam.dot(gPlane)));
		if (cosAngle >= minCosConfirm) {
			// the cues confirm each other: use the camera gravity, immune to terrain slope,
			// unless it is ill-conditioned (e.g. a single flight heading), in which case the
			// ground plane normal is the only reliable anchor
			if (camConditioned) {
				g = gCam;
			} else {
				g = gPlane;
				bPlaneOriented = true;
			}
			DEBUG_ULTIMATE("Gravity direction confirmed by cameras (%.0f%% support) and ground plane (%.0f%% inliers): %.1f deg apart",
				camConfidence*100, planeSupport*100, R2D(ACOS(MINF(cosAngle, 1.f))));
		} else if (cosAngle <= SIN(D2R(20.f))) {
			// near-perpendicular dominant plane: a facade, not the ground; but only a
			// validated camera estimate may overrule it
			if (!camPlausible) {
				DEBUG_ULTIMATE("error: ill-conditioned camera gravity cue and the dominant plane is vertical");
				return false;
			}
			g = gCam;
			DEBUG_ULTIMATE("Gravity direction set from cameras (%.0f%% support); dominant plane discarded as vertical", camConfidence*100);
		} else if (camConditioned) {
			// conflicting plane (e.g. sloped terrain): trust the well-conditioned camera consensus
			g = gCam;
			DEBUG_ULTIMATE("Gravity direction set from cameras (%.0f%% support); dominant plane conflicts (%.1f deg apart)",
				camConfidence*100, R2D(ACOS(MINF(cosAngle, 1.f))));
		} else {
			DEBUG_ULTIMATE("error: camera and ground-plane gravity cues conflict (%.1f deg apart)", R2D(ACOS(MINF(cosAngle, 1.f))));
			return false;
		}
	} else if (hasCam) {
		if (!camPlausible) {
			DEBUG_ULTIMATE("error: ill-conditioned camera gravity cue (%.0f%% support) and no ground plane to validate it", camConfidence*100);
			return false;
		}
		g = gCam;
		DEBUG_ULTIMATE("Gravity direction set from cameras (%.0f%% support); no ground plane found", camConfidence*100);
	} else if (planeSupport >= 0.5f) {
		// cameras are inconclusive: accept only a dominant ground-like plane
		g = gPlane;
		bPlaneOriented = true;
		DEBUG_ULTIMATE("Gravity direction set from the ground plane (%.0f%% inliers); cameras inconclusive (%.0f%% support)",
			planeSupport*100, camConfidence*100);
	} else {
		DEBUG_ULTIMATE("error: no confident gravity direction estimate (cameras %.0f%%, plane %.0f%%)",
			camConfidence*100, planeSupport*100);
		return false;
	}
	// orient the direction upward; the plane normal is already oriented toward the cameras,
	// a camera-sourced direction has an arbitrary sign and is oriented by scene geometry
	if (!bPlaneOriented && camScore < 0)
		g = -g;
	up = Point3f(g.x(), g.y(), g.z());
	return true;
} // EstimateGravityDirection
/*----------------------------------------------------------------*/

// interval covering the given samples with a weight fraction `tail` trimmed at each end;
// at least one sample (and proportionally more for larger sets) is always trimmed per end,
// so that a single extreme outlier cannot set the bound when the set is small
static std::pair<float,float> WeightedQuantiles(std::vector<std::pair<float,float>>& valueWeights, float tail)
{
	ASSERT(!valueWeights.empty());
	std::sort(valueWeights.begin(), valueWeights.end());
	double totalWeight(0);
	for (const auto& vw: valueWeights)
		totalWeight += vw.second;
	const double tailWeight(totalWeight*tail);
	const size_t minTrim(MAXF<size_t>(1, (size_t)((float)valueWeights.size()*tail)));
	std::pair<float,float> interval;
	double cumWeight(0);
	for (size_t i = 0; i < valueWeights.size(); ++i) {
		interval.first = valueWeights[i].first;
		if ((cumWeight += valueWeights[i].second) >= tailWeight && i >= minTrim)
			break;
	}
	cumWeight = 0;
	for (size_t i = valueWeights.size(); i-- > 0; ) {
		interval.second = valueWeights[i].first;
		if ((cumWeight += valueWeights[i].second) >= tailWeight && valueWeights.size()-1-i >= minTrim)
			break;
	}
	return interval;
}

// extend the given upper bound over the candidate coordinates beyond it, for as long as
// the gaps between consecutive points stay small (gap-connectivity); when bDropIsolated,
// candidates with no other candidate within maxGap cannot extend the bound (a lone
// floater is noise, real structure has local support)
static float GrowBound(FloatArr& coords, float bound, float maxGap, bool bDropIsolated = false)
{
	coords.Sort();
	if (bDropIsolated) {
		FloatArr supported(0, coords.size());
		for (IDX i = 0; i < coords.size(); ++i) {
			const bool nearPrev(i > 0 && coords[i]-coords[i-1] <= maxGap);
			const bool nearNext(i+1 < coords.size() && coords[i+1]-coords[i] <= maxGap);
			if (nearPrev || nearNext)
				supported.push_back(coords[i]);
		}
		coords.Swap(supported);
	}
	for (float c: coords) {
		if (c - bound > maxGap)
			break;
		if (c > bound)
			bound = c;
	}
	return bound;
}

// estimate the region-of-interest (ROI) based on the known poses and sparse point-cloud
//  - scaleROI: ROI scale factor, multipled after computation
//  - upAxis: indicates the gravity direction (0 for x, 1 for y, 2 for z, -1 for auto-detect)
bool Scene::EstimateROI(float scaleROI, int upAxis)
{
	if (!pointcloud.IsValid() || pointcloud.points.size() < 100 || images.size() < 4)
		return false;
	// work on a bounded uniform subsample of the cloud: all the statistics below are
	// robust aggregates that a fixed-stride subset preserves, while the cost of the
	// weighting (K-NN, reprojections) stops growing with the cloud size
	UnsignedArr sampleIndices;
	{
		const size_t numPoints(pointcloud.points.size());
		const size_t sampleStep(ClampedSampleStep(pointcloud, 200000));
		sampleIndices.Reserve(numPoints/sampleStep+1);
		for (size_t i = 0; i < numPoints; i += sampleStep)
			sampleIndices.push_back((unsigned)i);
	}
	// determine the up direction first, as both the tower detection and the box fit
	// depend on it: user-provided axis, otherwise estimated from cameras and geometry
	Eigen::Vector3f up;
	bool bHasUp(false);
	if (upAxis >= 0) {
		ASSERT(upAxis < 3);
		up = Eigen::Vector3f::Unit(upAxis);
		// the user picks only the axis; orient its sign from the scene geometry so that
		// up points skyward regardless of the world coordinate convention (e.g. -Y up)
		if (UpSignScore(up, images, pointcloud) < 0)
			up = -up;
		bHasUp = true;
	} else {
		Point3f estimatedUp;
		if (EstimateGravityDirection(estimatedUp)) {
			up = Eigen::Vector3f(estimatedUp.x, estimatedUp.y, estimatedUp.z);
			bHasUp = true;
		}
	}
	float medianNeighborDistance(0);
	FloatArr pointWeights = ROIPointWeights(sampleIndices, medianNeighborDistance);
	// compute threshold using robust statistics
	const auto [median, trustRegionSize] = ComputeX84Threshold(pointWeights.data(), pointWeights.size(), 0.7f);
	Line3f camCenterLine;
	const Point3f upPoint(bHasUp ? Point3f(up.x(), up.y(), up.z()) : Point3f());
	const bool isTower(ComputeCenterLine(camCenterLine, bHasUp ? &upPoint : NULL));
	float threshold = isTower ? (median + 2*trustRegionSize) : (median - trustRegionSize / 2);
	// clamp the threshold such that the retained fraction stays inside a sane band,
	// guarding against weight distributions whose shape defeats the X84 assumptions
	{
		const float minFrac(isTower ? 0.08f : 0.25f);
		const float maxFrac(isTower ? 0.40f : 0.90f);
		size_t numAbove(0);
		for (float weight: pointWeights)
			if (weight > threshold)
				++numAbove;
		const float frac((float)numAbove/(float)pointWeights.size());
		if (frac < minFrac || frac > maxFrac) {
			const float targetFrac(frac < minFrac ? minFrac : maxFrac);
			FloatArr sorted(pointWeights);
			threshold = sorted.GetNth((IDX)((1.f-targetFrac)*(float)(sorted.size()-1)));
		}
	}
	DEBUG_ULTIMATE("ROI threshold median: %f, trust region size: %f, threshold: %f", median, trustRegionSize, threshold);
	// keep only points above the threshold
	std::vector<Eigen::Vector3f> points;
	points.reserve(sampleIndices.size());
	FloatArr weights(0, sampleIndices.size());
	FOREACH(i, sampleIndices)
		if (pointWeights[i] > threshold) {
			points.emplace_back(Cast<float>(pointcloud.points[sampleIndices[i]]));
			weights.push_back(pointWeights[i]);
		}
	if (points.size() < 30) {
		VERBOSE("error: ROI estimation failed: too few points above the weight threshold (%u)", (unsigned)points.size());
		return false;
	}
	const float looseThreshold(isTower ? median : median - 2*trustRegionSize);
	// derive the core bounds for the given orientation, grow them over gap-connected
	// structure, and apply the margin; upAligned indicates the local z axis is the up
	// direction (enabling the weight-free vertical growth)
	const auto FitBounds = [&](const OBB3f::MATRIX& R, bool upAligned) {
		// support-driven bounds: per-axis core interval from the weighted quantiles of the
		// high-confidence points (robust to isolated outliers)
		Eigen::Vector3f lo, hi;
		{
			std::vector<std::pair<float,float>> valueWeights(points.size());
			for (int a = 0; a < 3; ++a) {
				for (size_t i = 0; i < points.size(); ++i)
					valueWeights[i] = std::make_pair(R.row(a).dot(points[i]), weights[i]);
				const std::pair<float,float> interval(WeightedQuantiles(valueWeights, 0.01f));
				lo[a] = interval.first;
				hi[a] = interval.second;
			}
		}
		// grow the bounds to include gap-connected structure that scored below the threshold:
		// vertically inside the core footprint without any weight requirement in either
		// direction (e.g. a tower top or a pit floor, penalized by all weight cues for being
		// far from the cameras and sparse), and horizontally inside the core interval of the
		// other axes for moderate-confidence points (e.g. the scene periphery), stopping where
		// the point support breaks; each axis uses a gap tolerance derived from its own extent
		// (floored by the cloud spacing) so thin axes are not inflated by the large ones, and
		// the pass is iterated once more with the grown gates so corner-adjacent structure can
		// follow; the weight-free vertical growth considers the full cloud (its candidates
		// need no weights) but drops isolated candidates, so single floaters cannot extend it
		const Eigen::Vector3f coreLo(lo), coreHi(hi);
		// the rotation is fixed for the whole fit, so project the moderate-confidence
		// candidates once: the two growth iterations differ only in their gates
		std::vector<Eigen::Vector3f> candidates;
		candidates.reserve(sampleIndices.size());
		FOREACH(i, sampleIndices)
			if (pointWeights[i] > looseThreshold)
				candidates.emplace_back(R * Eigen::Vector3f(Cast<float>(pointcloud.points[sampleIndices[i]])));
		for (int iter = 0; iter < 2; ++iter) {
			Eigen::Vector3f maxGap;
			for (int a = 0; a < 3; ++a)
				maxGap[a] = MAXF((hi[a]-lo[a])*0.05f, medianNeighborDistance*2);
			const Eigen::Vector3f gateLo(lo), gateHi(hi);
			const auto InsideGate = [&](const Eigen::Vector3f& q, int b, int c) {
				return q[b] >= gateLo[b] && q[b] <= gateHi[b] && q[c] >= gateLo[c] && q[c] <= gateHi[c];
			};
			FloatArr grow[3][2]; // per axis: candidates beyond hi, beyond lo (negated)
			for (const Eigen::Vector3f& q: candidates) {
				for (int a = 0; a < 3; ++a) {
					if (upAligned && a == 2)
						continue; // covered by the weight-free pass below, which sees the full cloud
					if (!InsideGate(q, (a+1)%3, (a+2)%3))
						continue;
					if (q[a] > gateHi[a])
						grow[a][0].push_back(q[a]);
					else if (q[a] < gateLo[a])
						grow[a][1].push_back(-q[a]);
				}
			}
			if (upAligned) {
				FOREACH(i, pointcloud.points) {
					const Eigen::Vector3f q(R * Eigen::Vector3f(Cast<float>(pointcloud.points[i])));
					if (!InsideGate(q, 0, 1))
						continue;
					if (q[2] > gateHi[2])
						grow[2][0].push_back(q[2]);
					else if (q[2] < gateLo[2])
						grow[2][1].push_back(-q[2]);
				}
			}
			for (int a = 0; a < 3; ++a) {
				const bool bDropIsolated(upAligned && a == 2);
				hi[a] = GrowBound(grow[a][0], hi[a], maxGap[a], bDropIsolated);
				lo[a] = -GrowBound(grow[a][1], -lo[a], maxGap[a], bDropIsolated);
			}
		}
		DEBUG_ULTIMATE("ROI bounds grown per axis by (%.2f %.2f %.2f) up / (%.2f %.2f %.2f) down",
			hi[0]-coreHi[0], hi[1]-coreHi[1], hi[2]-coreHi[2], coreLo[0]-lo[0], coreLo[1]-lo[1], coreLo[2]-lo[2]);
		OBB3f b;
		b.m_rot = R;
		b.m_pos = R.transpose() * ((lo+hi)*0.5f);
		b.m_ext = (hi-lo)*0.5f;
		// enlarge the box, multiplicatively with an absolute margin floor so that thin axes
		// (e.g. the height of a flat aerial scene) also receive a usable margin
		if (scaleROI >= 1) {
			const float meanExt(b.m_ext.mean());
			for (int a = 0; a < 3; ++a)
				b.m_ext[a] += (scaleROI-1)*MAXF(b.m_ext[a], meanExt);
		} else
			b.EnlargePercent(scaleROI);
		return b;
	};
	// weight fraction of the (sampled) cloud contained by the box
	const auto ComputeCoverage = [&](const OBB3f& b) {
		double insideWeight(0), totalWeight(0);
		FOREACH(i, sampleIndices) {
			const float w(pointWeights[i]);
			totalWeight += w;
			if (b.Intersects(Eigen::Vector3f(Cast<float>(pointcloud.points[sampleIndices[i]]))))
				insideWeight += w;
		}
		return totalWeight > 0 ? (float)(insideWeight/totalWeight) : 0.f;
	};
	// fit the box orientation and bounds, self-checking that the box contains the bulk of
	// the total point weight; on failure fall back to progressively simpler orientations
	// before giving up (a mis-estimated up or an over-tightened fit should degrade to a
	// usable box, not to a silently unbounded scene)
	OBB3f box;
	if (bHasUp) {
		// up-axis aligned box, in-plane orientation minimizing the footprint area
		box.Set(points.data(), points.size(), up);
	} else {
		// unconstrained covariance-based box
		box.Set(points.data(), points.size());
	}
	const char* fitName(bHasUp ? "up-aligned" : "covariance");
	OBB3f roi(FitBounds(OBB3f::MATRIX(box.m_rot), bHasUp));
	float coverage(ComputeCoverage(roi));
	if (coverage < 0.7f && bHasUp) {
		OBB3f boxCov;
		boxCov.Set(points.data(), points.size());
		const OBB3f cand(FitBounds(OBB3f::MATRIX(boxCov.m_rot), false));
		const float candCoverage(ComputeCoverage(cand));
		DEBUG_ULTIMATE("ROI up-aligned fit covers only %.0f%% of the point-cloud weight; covariance fallback covers %.0f%%", coverage*100, candCoverage*100);
		if (candCoverage > coverage) {
			roi = cand;
			coverage = candCoverage;
			fitName = "covariance";
		}
	}
	if (coverage < 0.7f) {
		const OBB3f cand(FitBounds(OBB3f::MATRIX(OBB3f::MATRIX::Identity()), false));
		const float candCoverage(ComputeCoverage(cand));
		DEBUG_ULTIMATE("ROI %s fit covers only %.0f%% of the point-cloud weight; axis-aligned fallback covers %.0f%%", fitName, coverage*100, candCoverage*100);
		if (candCoverage > coverage) {
			roi = cand;
			coverage = candCoverage;
			fitName = "axis-aligned";
		}
	}
	if (coverage < 0.7f) {
		VERBOSE("error: ROI estimation failed: the box covers only %.0f%% of the point-cloud weight", coverage*100);
		return false;
	}
	obb = roi;
	VERBOSE("ROI estimated with position (%f,%f,%f) and extent (%f,%f,%f): scale %f, up %s, %s fit, weight coverage %.0f%%",
			obb.m_pos[0], obb.m_pos[1], obb.m_pos[2], obb.m_ext[0], obb.m_ext[1], obb.m_ext[2], scaleROI,
			bHasUp ? String::FormatString("(%.3f,%.3f,%.3f)", up.x(), up.y(), up.z()).c_str() : "unconstrained",
			fitName, coverage*100);
	return true;
} // EstimateROI
/*----------------------------------------------------------------*/

// compute the average distance between cameras and scene (or ROI if specified and exists):
//  - depthPercentile: percentile of closest points to consider for each image (0-1)
//  - bForceRecompute: force recomputation even if already available for each image
//  - bUseROI: use the ROI if it exists, otherwise use the entire scene
// return the average depth over all images
float Scene::ComputeDistanceCameras2Scene(float depthPercentile, bool bForceRecompute, bool bUseROI)
{
	// for each image, compute the average distance between the camera and the scene points it sees;
	// the average is computed on the Nth percentile of the closest points
	const OBB3f* pObb = bUseROI && IsBounded() ? &obb : NULL;
	// fall back to camera-frustum visibility when per-point views are missing
	const bool bHasViews = !pointcloud.pointViews.empty();
	REAL sumDepth = 0;
	unsigned nImages = 0;
	#ifdef SCENE_USE_OPENMP
	#pragma omp parallel for reduction(+:sumDepth,nImages) //schedule(dynamic)
	for (int64_t _idx=0; _idx<(int64_t)images.size(); ++_idx) {
		const IIndex idx(static_cast<IIndex>(_idx));
	#else
	FOREACH(idx, images) {
	#endif
		Image& imageData = images[idx];
		if (!imageData.IsValid())
			continue;
		if (bForceRecompute || imageData.avgDepth <= 0) {
			// recompute average depth
			FloatArr depths;
			if (bHasViews) {
				FOREACH(idxPoint, pointcloud.points) {
					const PointCloud::ViewArr& views = pointcloud.pointViews[idxPoint];
					for (PointCloud::View idxView: views) {
						if (idxView != idx)
							continue;
						const Point3f& point = pointcloud.points[idxPoint];
						if (!pObb || pObb->Intersects(point))
							depths.emplace_back(imageData.camera.PointDepth(point));
						break;
					}
				}
			} else {
				const Point2f imageSize(imageData.GetSize());
				FOREACH(idxPoint, pointcloud.points) {
					const Point3f& point = pointcloud.points[idxPoint];
					if (pObb && !pObb->Intersects(point))
						continue;
					const auto [proj, depth] = imageData.camera.ProjectPointP(point);
					if (depth > 0 && imageData.camera.IsInside(proj, imageSize))
						depths.emplace_back(depth);
				}
			}
			if (depths.empty()) {
				imageData.avgDepth = 0;
				continue;
			}
			imageData.avgDepth = depths.GetNth(ROUND2INT<IDX>((depths.size()-1) * depthPercentile));
		}
		sumDepth += static_cast<REAL>(imageData.avgDepth);
		++nImages;
	}
	return nImages == 0 ? 0.f : static_cast<float>(sumDepth / nImages);
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
