/*
 * StarInitializer.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "StarInitializer.h"
#include "Scene.h"
#include "Triangulation.h"
#include "BundleAdjustment.h"
#include "GlobalRotationAveraging.h"
#include "GlobalScaleAveraging.h"

using namespace SFM;

// S T R U C T S ///////////////////////////////////////////////////

IIndex StarInitializer::SelectReferenceView(const Scene& scene)
{
	// Select view with highest connectivity (most matches)
	IIndex bestView = NO_ID;
	unsigned maxDegree = 0;

	// Count connections per view
	UnsignedArr degree(scene.images.size());
	degree.Memset(0);
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		degree[pair.ID1] += pair.GetNumFilteredInliers();
		degree[pair.ID2] += pair.GetNumFilteredInliers();
	}

	// Find view with max degree
	FOREACH(i, scene.images) {
		if (degree[i] > maxDegree) {
			maxDegree = degree[i];
			bestView = i;
		}
	}
	if (bestView == NO_ID) {
		VERBOSE("error: no valid reference view found");
		return NO_ID;
	}
	VERBOSE("Selected reference view %u with %u connections", bestView, maxDegree);
	return bestView;
}

bool StarInitializer::EstimateGlobalScale(
	Scene& scene,
	IIndex refViewID,
	const IIndexArr& connectedViews)
{
	if (connectedViews.empty())
		return true;

	// Set of views involved in the optimization (Ref + Connected)
	std::unordered_set<uint32_t> validViews;
	validViews.insert(refViewID);
	validViews.insert(connectedViews.begin(), connectedViews.end());

	// Map: ViewID -> { FeatureID -> List of (PairIndex, DistanceFromView) }
	// This stores the distance of a triangulated point from a specific view, derived from a specific pair
	typedef std::pair<uint32_t, double> PairDist; // PairIndex, Distance
	typedef CLISTDEF0(PairDist) PairDistArr;
	typedef std::unordered_map<uint32_t, PairDistArr> FeatureDistMap; // FeatureID -> List
	std::unordered_map<uint32_t, FeatureDistMap> viewFeatureDists; // ViewID -> FeatureDistMap

	// Track if any valid pair contributes to scale estimation
	bool hasValidPairs = false;

	// 1. Iterate all pairs in the scene
	const REAL maxCosAngle = COS(D2R(0.5f));
	const REAL reprojPixelThreshold = 6;
	FOREACH(pairIdx, scene.pairs) {
		// Only consider pairs where both views are in our set
		const ImagePair& pair = scene.pairs[pairIdx];
		if (validViews.find(pair.ID1) == validViews.end() ||
			validViews.find(pair.ID2) == validViews.end())
			continue;
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		// Triangulate all matches
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		const Camera& cam1 = *img1.pCamera;
		const Camera& cam2 = *img2.pCamera;
		const Pose3D& relPose = pair.relativePose.value();
		// Angular reprojection threshold (works for both pinhole and spherical; pixel metric breaks on equirectangular)
		const REAL cosReprojAngle = COS(REAL(0.5) * (cam1.PixelErrorToAngular(reprojPixelThreshold) + cam2.PixelErrorToAngular(reprojPixelThreshold)));
		for (const DMatch& match : pair.matches) {
			const Point2f& pt1 = img1.keypoints[match.queryIdx].pt;
			const Point2f& pt2 = img2.keypoints[match.trainIdx].pt;
			// Observed unit bearings (works for both camera types)
			const Point3 b1 = cam1.UnprojectNormalized(Cast<REAL>(pt1));
			const Point3 b2 = cam2.UnprojectNormalized(Cast<REAL>(pt2));
			// Triangulate in Camera 1 frame (midpoint on unit bearings)
			Point3 Xlocal;
			if (!TriangulatePoint3D(relPose.R, relPose.C, b1, b2, Xlocal))
				continue;
			// Cheirality + angular reprojection in Cam1
			const REAL nX = norm(Xlocal);
			if (nX < ZEROTOLERANCE<REAL>())
				continue;
			const REAL cosErr1 = b1.dot(Xlocal / nX);
			if (cosErr1 < cosReprojAngle)
				continue;
			// Cheirality + angular reprojection in Cam2
			const Point3 Xcam2 = relPose.TransformPointW2C(Xlocal);
			const REAL nXc2 = norm(Xcam2);
			if (nXc2 < ZEROTOLERANCE<REAL>())
				continue;
			const REAL cosErr2 = b2.dot(Xcam2 / nXc2);
			if (cosErr2 < cosReprojAngle)
				continue;
			// Angle check
			const Point3 V1 = Xlocal; // ray from C1 to X
			const Point3 V2 = Xlocal - relPose.C; // ray from C2 to X
			REAL cosAngle = ComputeAngle(V1.ptr(), V2.ptr());
			if (cosAngle > maxCosAngle)
				continue;
			// Compute distances from camera centers
			// Dist from Cam 1 (Origin): |Xlocal - 0|
			double dist1 = cv::norm(Xlocal);
			// Dist from Cam 2 (relPose.C): |Xlocal - relPose.C|
			// Note: relPose.C is the position of Cam 2 in Cam 1 frame
			double dist2 = cv::norm(Xlocal - relPose.C);
			// Store distances
			// View1 (ID1), Feature (queryIdx)
			PairDistArr& arr1 = viewFeatureDists[pair.ID1][match.queryIdx];
			if (arr1.FindFunc([pairIdx](const auto& elem) { return elem.first == pairIdx; }) == PairDistArr::NO_INDEX)
				arr1.emplace_back(pairIdx, dist1); // should not happen
			// View2 (ID2), Feature (trainIdx)
			PairDistArr& arr2 = viewFeatureDists[pair.ID2][match.trainIdx];
			if (arr2.FindFunc([pairIdx](const auto& elem) { return elem.first == pairIdx; }) == PairDistArr::NO_INDEX)
				arr2.emplace_back(pairIdx, dist2); // possible if no cross-checking matching was used
		}
		hasValidPairs = true;
	}
	if (!hasValidPairs)
		return true;

	// 2. Build system of equations
	// Variables: log(Scale_p) for each active pair p
	// Equations: log(S_p) - log(S_q) = log(dist_q / dist_p)
	// For each view and feature, if we have measurements from multiple pairs, they should agree
	std::vector<ScalePair> scalePairs;
	std::unordered_set<uint32_t> constrainedPairs;

	// Map: PairIdx1 -> { PairIdx2 -> List of log-ratios }
	std::map<uint32_t, std::map<uint32_t, DoubleArr>> pairRatios;
	for (const auto& viewIt : viewFeatureDists) {
		for (const auto& featIt : viewIt.second) {
			const PairDistArr& obs = featIt.second;
			if (obs.size() < 2)
				continue;
			// We have multiple pairs observing the same feature from the same view.
			// For each pair (i, j), S_i * dist_i = S_j * dist_j => S_i / S_j = dist_j / dist_i
			// We can link all to the first one, or all pairs.
			// Linking to the first one is sufficient (spanning tree).
			// Or all combinations for robustness.
			// Let's do all combinations (n*(n-1)/2) but n is usually small (2-4).
			for (size_t i = 0; i + 1 < obs.size(); ++i) {
				for (size_t j = i + 1; j < obs.size(); ++j) {
					uint32_t pairIdx1 = obs[i].first;
					double dist1 = obs[i].second;
					uint32_t pairIdx2 = obs[j].first;
					double dist2 = obs[j].second;
					ASSERT(pairIdx1 != pairIdx2);
					// Ensure pairIdx1 < pairIdx2 for consistent storage
					if (pairIdx1 > pairIdx2) {
						std::swap(pairIdx1, pairIdx2);
						std::swap(dist1, dist2);
					}
					// log(S1) - log(S2) = log(dist2 / dist1)
					double ratio = dist2 / dist1;
					pairRatios[pairIdx1][pairIdx2].push_back(ratio);
				}
			}
		}
	}
	// Compute the mean of the ratios for each pair of image pairs
	for (auto& it1 : pairRatios) {
		uint32_t pairIdx1 = it1.first;
		for (auto& it2 : it1.second) {
			uint32_t pairIdx2 = it2.first;
			ASSERT(pairIdx1 < pairIdx2);
			DoubleArr& ratios = it2.second;
			if (ratios.size() < 15) // Require minimum common points
				continue;
			ratios.Sort();
			size_t num = ratios.size();
			size_t start = num / 10;
			size_t end = num - start;
			ASSERT(end > start);
			double sum = std::accumulate(ratios.begin() + start, ratios.begin() + end, 0.0);
			double numValid = static_cast<double>(end - start);
			double meanRatio = sum / numValid;
			scalePairs.emplace_back(pairIdx2, pairIdx1, meanRatio, (float)SQRT(numValid));
			constrainedPairs.insert(pairIdx1);
			constrainedPairs.insert(pairIdx2);
		}
	}
	if (scalePairs.empty()) {
		VERBOSE("warning: no overlapping constraints for global scale");
		return false;
	}

	// 3. Solve system using shared global scale averaging
	// Fix one pair scale to 1.0: choose the pair connected to Ref with most matches.
	uint32_t fixedPairIdx = NO_ID;
	unsigned maxMatches = 0;
	for (const uint32_t pairIdx : constrainedPairs) {
		const ImagePair& pair = scene.pairs[pairIdx];
		// Check if connected to Ref
		if (pair.ID1 == refViewID || pair.ID2 == refViewID) {
			if (pair.GetNumInliers() > maxMatches) {
				maxMatches = pair.GetNumInliers();
				fixedPairIdx = pairIdx;
			}
		}
	}

	std::vector<REAL> pairScales;
	GlobalScaleEstimator scaleEstimator;
	if (!scaleEstimator.EstimateScales(scalePairs, (uint32_t)scene.pairs.size(), fixedPairIdx, pairScales)) {
		VERBOSE("warning: failed to estimate global scale factors");
		return false;
	}

	// 4. Update relative poses
	for (const uint32_t pairIdx : constrainedPairs) {
		const double scale = pairScales[pairIdx];
		ImagePair& pair = scene.pairs[pairIdx];
		pair.relativePose.value().C *= scale;
		DEBUG_ULTIMATE("Pair (%u,%u) scale: %.4f", pair.ID1, pair.ID2, scale);
	}

	// 5. Update absolute poses of connected views
	// We assume Ref is at Identity.
	// We update neighbors of Ref using the updated relative poses.
	// Note: We only update views that are directly connected to Ref in the star graph.
	// Other views (if any in connectedViews but not directly connected? Star implies direct connection)
	// StarInitializer::Initialize ensures connectedViews are directly connected.
	for (uint32_t viewID : connectedViews) {
		const ImagePair* pPair(refViewID < viewID ? scene.FindPair(refViewID, viewID) : scene.FindPair(viewID, refViewID));
		if (!pPair || !pPair->relativePose.has_value())
			continue;
		Image& img = scene.images[viewID];
		// Re-compute absolute pose from Ref (Identity) and scaled RelativePose
		if (pPair->ID1 == refViewID) {
			// T_view_ref = RelPose
			// ViewPose = RelPose * RefPose(Identity) = RelPose
			static_cast<Pose3D&>(img) = pPair->relativePose.value();
		} else {
			// T_ref_view = RelPose
			// ViewPose = RelPose^-1 * RefPose(Identity)
			static_cast<Pose3D&>(img) = pPair->relativePose.value().Inverse();
		}
	}
	return true;
}

bool StarInitializer::Initialize(
	Scene& scene,
	const StarInitConfig& config)
{
	TD_TIMER_START();
	ASSERT(!scene.IsEmpty() && !scene.pairs.empty() && !scene.tracks.empty())

	// 1. Select reference view (highest connectivity)
	const IIndex refID = SelectReferenceView(scene);
	if (refID == NO_ID)
		return false;
	// Set reference view to identity pose
	Image& refImg = scene.images[refID];
	reinterpret_cast<Pose3D&>(refImg) = Pose3D::Identity();
	DEBUG("Reference view %u set to identity pose", refID);

	// 2. Register connected views from relative poses
	struct ConnectedView {
		IIndex viewID; // ID of the connected target view
		unsigned numInliers; // number of inliers in the pair
		Pose3D relPose; // relative pose to reference
	};
	CLISTDEF0IDX(ConnectedView, IIndex) candidates;
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		IIndex sourceID = NO_ID, targetID = NO_ID;
		// Determine which view is the reference
		if (pair.ID1 == refID) {
			sourceID = refID;
			targetID = pair.ID2;
		} else if (pair.ID2 == refID) {
			sourceID = refID;
			targetID = pair.ID1;
		} else {
			continue; // neither view is reference
		}
		Image& targetImg = scene.images[targetID];
		if (targetImg.HasPose())
			continue; // already registered
		// Convert relative pose to absolute
		// Relative: T_target_source = [R|t]
		// Absolute: target pose relative to source (which is identity)
		// If source is ID2, we need inverse transform
		Pose3D pose(sourceID == pair.ID1 ? pair.relativePose.value() : pair.relativePose->Inverse());
		candidates.push_back({targetID, pair.GetNumFilteredInliers(), pose});
	}
	if (candidates.size() < config.minViews-1) {
		VERBOSE("error: insufficient initial views (%u < %u)",
			candidates.size(), config.minViews-1);
		return false;
	}
	// Sort candidates by number of inliers (descending)
	candidates.Sort([](const ConnectedView& a, const ConnectedView& b) {
		return a.numInliers > b.numInliers;
	});
	// Filter candidates by minimum inliers per view
	if (config.ratioInliersFilter > 0) {
		// Compute robust threshold top 3 views
		unsigned avgInliers = 0;
		const IIndex numCandidates = candidates.size();
		IIndex count = MINF(3u, numCandidates);
		for (unsigned i = 0; i < count; ++i)
			avgInliers += candidates[i].numInliers;
		avgInliers /= count;
		const unsigned thInliers = ROUND2INT<unsigned>(avgInliers * config.ratioInliersFilter);
		IIndex size = numCandidates;
		while (size-- > 0 && candidates[size].numInliers < thInliers);
		candidates.resize(size + 1);
		DEBUG_EXTRA("Filtered connected views by inlier matches: kept %u/%u views with at least %u inliers",
			candidates.size(), numCandidates, thInliers);
	}
	// Keep only up to maxViews
	if (candidates.size() > config.maxViews)
		candidates.resize(config.maxViews);
	// Set absolute poses for connected views
	IIndexArr connectedViews(0, candidates.size());
	for (const ConnectedView& cv : candidates) {
		static_cast<Pose3D&>(scene.images[cv.viewID]) = cv.relPose;
		connectedViews.push_back(cv.viewID);
		DEBUG_ULTIMATE("Registered view %u (%u matches)", cv.viewID, cv.numInliers);
	}
	DEBUG("Registered %u views from relative poses", candidates.size());

	// 2.a. Finetune rotations with global rotation averaging (optional)
	if (config.globalRotations) {
		DEBUG("Refining initial rotations with global rotation averaging");
		GlobalRotationEstimatorOptions rotOptions;
		rotOptions.skipInitialization = true; // rotations are already initialized
		GlobalRotationEstimator rotEstimator(rotOptions);
		if (!rotEstimator.EstimateRotations(scene)) {
			VERBOSE("error: initializer global rotation averaging failed");
			return false;
		}
	}

	// 3. Estimate global scale
	if (!EstimateGlobalScale(scene, refID, connectedViews)) {
		VERBOSE("error: global scale estimation failed (proceeding without scaling)");
		return false;
	}

	// 4. Triangulate initial points
	TriangulateTracks(scene, false, config.maxReprojError, config.minAngleThreshold);
	if (scene.status.nTracks < 100) {
		VERBOSE("error: insufficient triangulated tracks (%u)", scene.status.nTracks);
		return false;
	}

	// 5. Mini bundle adjustment (refine initial reconstruction)
	BAConfig baConfig;
	baConfig.maxIterations = 10; // default mini BA iterations
	if (!BundleAdjustment::Adjust(scene, baConfig)) {
		VERBOSE("error: mini bundle adjustment failed");
		return false;
	}

	// 6. Update tracks after BA
	float maxReprojError = MAXF(config.maxReprojError-1, 1.f);
	float minAngleThreshold = MINF(config.minAngleThreshold+0.5f, 3.f);
	TriangulateTracks(scene, true, maxReprojError, minAngleThreshold);
	FilterTracks(scene, maxReprojError, minAngleThreshold);
	if (scene.status.nTracks < 100) {
		VERBOSE("error: insufficient triangulated tracks after BA (%u)", scene.status.nTracks);
		return false;
	}

	// 7. Bundle adjustment with intrinsics refinement
	baConfig.RefineMainIntrinsics();
	baConfig.maxIterations = 25;
	if (!BundleAdjustment::Adjust(scene, baConfig)) {
		VERBOSE("error: bundle adjustment with intrinsics refinement failed");
		return false;
	}

	// 8. Update tracks and stats after BA
	maxReprojError = MAXF(config.maxReprojError-2, 1.f);
	minAngleThreshold = MINF(config.minAngleThreshold+1.f, 3.f);
	TriangulateTracks(scene, true, maxReprojError, minAngleThreshold);
	FilterTracks(scene, maxReprojError, minAngleThreshold);
	scene.status.nCalibratedImages = connectedViews.size() + 1;
	scene.status.nState.set(Scene::Status::STATE::CALIBRATED);
	VERBOSE("Star initialization complete: %u views, %u tracks (%s)",
		scene.status.nCalibratedImages, scene.status.nTracks, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/
