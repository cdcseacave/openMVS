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
#include <numeric>

using namespace SFM;

// S T R U C T S ///////////////////////////////////////////////////

// The star evidence of the pairs that join the reference view to another view: the pair's weighted
// inliers times the share of its inliers that triangulate at its own relative pose with a ray angle
// of at least minAngle degrees, the inliers that measure a distance (a pair of far points fixes no
// distance, whatever its inlier count; its unit baseline only guesses one); 0 for every other pair
// and for a pair without a pose or a valid weight, the star does not use them
static UnsignedArr ComputeStarEvidence(const Scene& scene, IIndex refID, float minAngle)
{
	UnsignedArr evidence(scene.pairs.size());
	evidence.Memset(0);
	#pragma omp parallel for schedule(dynamic)
	for (int_t i = 0; i < (int_t)scene.pairs.size(); ++i) {
		const ImagePair& pair = scene.pairs[i];
		if (pair.ID1 != refID && pair.ID2 != refID)
			continue;
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		const float share = pair.ComputeShareAboveAngle(scene.images[pair.ID1], scene.images[pair.ID2], minAngle);
		evidence[i] = ROUND2INT<unsigned>(pair.GetNumWeightedInliers() * share);
	}
	return evidence;
}

IIndex StarInitializer::SelectReferenceView(const Scene& scene, const IIndexArr& seedViews, unsigned minPairs)
{
	// weighted inliers and valid pairs per view, dense supplement discounted and included: a
	// dense-only pair is a real connection of both its images, and the star initializer must see
	// the same graph the weights that let it through were computed on
	UnsignedArr degree(scene.images.size()), numPairs(scene.images.size());
	degree.Memset(0);
	numPairs.Memset(0);
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		degree[pair.ID1] += pair.GetNumWeightedInliers();
		degree[pair.ID2] += pair.GetNumWeightedInliers();
		++numPairs[pair.ID1];
		++numPairs[pair.ID2];
	}
	// the heaviest of the given views with at least minArms valid pairs: the star built around the
	// reference view has one arm per valid pair and the initializer refuses a star smaller than
	// its minimum, so a heavier view with fewer arms is no use -- on a sparse kept graph (a chain
	// of 19 images and 20 pairs) the heaviest seed view had two
	const auto Heaviest = [&degree, &numPairs](const auto& views, unsigned minArms, IIndex& bestView) {
		unsigned maxDegree = 0;
		for (IIndex i : views) {
			if (numPairs[i] >= minArms && degree[i] > maxDegree) {
				maxDegree = degree[i];
				bestView = i;
			}
		}
		return maxDegree;
	};
	const unsigned minArms = MAXF(minPairs, 1u);
	IIndex bestView = NO_ID;
	unsigned maxDegree = Heaviest(seedViews, minArms, bestView);
	if (maxDegree > 0) {
		VERBOSE("Selected reference view %u with %u connections over %u pairs among %u seed views",
			bestView, maxDegree, numPairs[bestView], seedViews.size());
		return bestView;
	}
	if (!seedViews.empty())
		VERBOSE("warning: none of the %u seed views has %u valid pairs, choosing the reference view among every image",
			seedViews.size(), minArms);
	std::vector<IIndex> every(scene.images.size());
	std::iota(every.begin(), every.end(), IIndex(0));
	maxDegree = Heaviest(every, minArms, bestView);
	if (maxDegree == 0) // no image has enough pairs: the heaviest image, and the caller reports the shortfall
		maxDegree = Heaviest(every, 1u, bestView);
	if (bestView == NO_ID) {
		VERBOSE("error: no valid reference view found");
		return NO_ID;
	}
	VERBOSE("Selected reference view %u with %u connections over %u pairs", bestView, maxDegree, numPairs[bestView]);
	return bestView;
}

std::vector<ScalePair> SFM::ComputePairScales(const Scene& scene, const IIndexArr& pairIndices)
{
	// 1. Triangulate the matches of each pair in its unit frame (in parallel): the distances of every
	// point that passes the checks from the pair's two cameras, with its feature in each view
	struct PairPoint { uint32_t feature1, feature2; double dist1, dist2; };
	std::vector<std::vector<PairPoint>> pairPoints(pairIndices.size());
	const REAL maxCosAngle = COS(D2R(0.5));
	const REAL reprojPixelThreshold = 6;
	#pragma omp parallel for schedule(dynamic, 16)
	for (int_t k = 0; k < (int_t)pairIndices.size(); ++k) {
		const ImagePair& pair = scene.pairs[pairIndices[k]];
		ASSERT(pair.relativePose.has_value());
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		const Camera& cam1 = *img1.pCamera;
		const Camera& cam2 = *img2.pCamera;
		const Pose3D& relPose = pair.relativePose.value();
		// Angular reprojection threshold (works for both pinhole and spherical; pixel metric breaks on equirectangular)
		const REAL cosReprojAngle = COS(REAL(0.5) * (cam1.PixelErrorToAngular(reprojPixelThreshold) + cam2.PixelErrorToAngular(reprojPixelThreshold)));
		std::vector<PairPoint>& points = pairPoints[k];
		points.reserve(pair.matches.size());
		for (const DMatch& match : pair.matches) {
			// Observed unit bearings (works for both camera types)
			const Point3 b1 = cam1.UnprojectNormalized(Cast<REAL>(img1.keypoints[match.queryIdx].pt));
			const Point3 b2 = cam2.UnprojectNormalized(Cast<REAL>(img2.keypoints[match.trainIdx].pt));
			// Triangulate in Camera 1 frame (midpoint on unit bearings)
			Point3 Xlocal;
			if (!TriangulatePoint3D(relPose.R, relPose.C, b1, b2, Xlocal))
				continue;
			// Cheirality + angular reprojection in Cam1
			const REAL nX = norm(Xlocal);
			if (nX < ZEROTOLERANCE<REAL>() || b1.dot(Xlocal / nX) < cosReprojAngle)
				continue;
			// Cheirality + angular reprojection in Cam2
			const Point3 Xcam2 = relPose.TransformPointW2C(Xlocal);
			const REAL nXc2 = norm(Xcam2);
			if (nXc2 < ZEROTOLERANCE<REAL>() || b2.dot(Xcam2 / nXc2) < cosReprojAngle)
				continue;
			// Angle check between the rays from C1 and from C2 (relPose.C is Cam 2's position in Cam 1's frame)
			const Point3 V1 = Xlocal;
			const Point3 V2 = Xlocal - relPose.C;
			if (ComputeAngle(V1.ptr(), V2.ptr()) > maxCosAngle)
				continue;
			points.push_back({(uint32_t)match.queryIdx, (uint32_t)match.trainIdx, cv::norm(Xlocal), cv::norm(V2)});
		}
	}

	// 2. Build system of equations
	// Variables: log(Scale_p) for each active pair p
	// Equations: log(S_p) - log(S_q) = log(dist_q / dist_p)
	// For each view and feature, if we have measurements from multiple pairs, they should agree.
	// Two pairs share at most one view, so each pair of pairs gets all its ratios from one view and
	// the views are independent (in parallel): per view, the distances of every pair that triangulated
	// each of its features, then the ratios of every two pairs, grouped per pair of pairs
	struct FeatureDist { uint32_t feature, pair, order; double dist; }; // pair = index in the scene's pairs
	struct Ratio { uint32_t pair1, pair2; double ratio; }; // pair1 < pair2
	struct Equation { uint32_t pair1, pair2; double meanRatio; float weight; };
	std::vector<std::vector<std::pair<uint32_t, bool>>> pairsOfView(scene.images.size()); // index in pairIndices, view is ID2
	FOREACH(k, pairIndices) {
		const ImagePair& pair = scene.pairs[pairIndices[k]];
		pairsOfView[pair.ID1].emplace_back((uint32_t)k, false);
		pairsOfView[pair.ID2].emplace_back((uint32_t)k, true);
	}
	std::vector<std::vector<Equation>> equationsOfView(scene.images.size());
	#pragma omp parallel for schedule(dynamic, 1)
	for (int_t v = 0; v < (int_t)pairsOfView.size(); ++v) {
		if (pairsOfView[v].size() < 2)
			continue;
		std::vector<FeatureDist> dists;
		for (const auto& [k, isSecond] : pairsOfView[v]) {
			const std::vector<PairPoint>& points = pairPoints[k];
			FOREACH(i, points)
				dists.push_back(isSecond ?
					FeatureDist{points[i].feature2, pairIndices[k], (uint32_t)i, points[i].dist2} :
					FeatureDist{points[i].feature1, pairIndices[k], (uint32_t)i, points[i].dist1});
		}
		// a feature a pair lists twice (no cross-checking in the matching) keeps its first distance
		std::sort(dists.begin(), dists.end(), [](const FeatureDist& a, const FeatureDist& b) {
			return a.feature != b.feature ? a.feature < b.feature : (a.pair != b.pair ? a.pair < b.pair : a.order < b.order);
		});
		std::vector<Ratio> ratios;
		std::vector<const FeatureDist*> features; // the first distance of each pair of one feature
		for (size_t i = 0; i < dists.size(); ) {
			features.clear();
			for (; i < dists.size() && (features.empty() || dists[i].feature == features.front()->feature); ++i)
				if (features.empty() || features.back()->pair != dists[i].pair)
					features.push_back(&dists[i]);
			// S_1 / S_2 = dist_2 / dist_1 for every two pairs that triangulated the feature (pair 1 < pair 2)
			for (size_t a = 0; a + 1 < features.size(); ++a)
				for (size_t b = a + 1; b < features.size(); ++b)
					ratios.push_back({features[a]->pair, features[b]->pair, features[b]->dist / features[a]->dist});
		}
		std::sort(ratios.begin(), ratios.end(), [](const Ratio& a, const Ratio& b) {
			return a.pair1 != b.pair1 ? a.pair1 < b.pair1 : (a.pair2 != b.pair2 ? a.pair2 < b.pair2 : a.ratio < b.ratio);
		});
		// the 10%-trimmed mean of the ratios of each pair of pairs
		for (size_t i = 0; i < ratios.size(); ) {
			size_t end = i;
			while (end < ratios.size() && ratios[end].pair1 == ratios[i].pair1 && ratios[end].pair2 == ratios[i].pair2)
				++end;
			const size_t num = end - i;
			if (num >= 15) { // Require minimum common points
				const size_t trim = num / 10;
				double sum = 0;
				for (size_t j = i + trim; j < end - trim; ++j)
					sum += ratios[j].ratio;
				const double numValid = static_cast<double>(num - 2 * trim);
				equationsOfView[v].push_back({ratios[i].pair1, ratios[i].pair2, sum / numValid, (float)SQRT(numValid)});
			}
			i = end;
		}
	}
	std::vector<Equation> equations;
	for (const std::vector<Equation>& viewEquations : equationsOfView)
		equations.insert(equations.end(), viewEquations.begin(), viewEquations.end());
	std::sort(equations.begin(), equations.end(), [](const Equation& a, const Equation& b) {
		return a.pair1 != b.pair1 ? a.pair1 < b.pair1 : a.pair2 < b.pair2;
	});
	std::vector<ScalePair> scalePairs;
	scalePairs.reserve(equations.size());
	for (const Equation& equation : equations)
		scalePairs.emplace_back(equation.pair2, equation.pair1, equation.meanRatio, equation.weight);
	return scalePairs;
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

	// The pairs joining two of them with a usable relative pose
	IIndexArr pairIndices;
	FOREACH(pairIdx, scene.pairs) {
		const ImagePair& pair = scene.pairs[pairIdx];
		if (validViews.find(pair.ID1) == validViews.end() ||
			validViews.find(pair.ID2) == validViews.end())
			continue;
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		pairIndices.push_back((IIndex)pairIdx);
	}
	if (pairIndices.empty())
		return true;

	// 1-2. The pair-scale equations
	const std::vector<ScalePair> scalePairs = ComputePairScales(scene, pairIndices);
	std::unordered_set<uint32_t> constrainedPairs;
	for (const ScalePair& scalePair : scalePairs) {
		constrainedPairs.insert(scalePair.idxB); // pairIdx1 first, as the equations were built
		constrainedPairs.insert(scalePair.idxA);
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
	const IIndex refID = SelectReferenceView(scene, config.seedViews, config.minViews > 0 ? config.minViews - 1 : 0);
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
		unsigned numEvidence; // the inliers of the pair that fix a distance (see ComputeStarEvidence)
		Pose3D relPose; // relative pose to reference
	};
	CLISTDEF0IDX(ConnectedView, IIndex) candidates;
	const UnsignedArr pairEvidence(ComputeStarEvidence(scene, refID, config.evidenceMinAngle));
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		if (pairEvidence[idxPair] == 0)
			continue; // no inlier fixes a distance (or the pair does not touch the reference): the view is left to the resection
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
		candidates.push_back({targetID, pair.GetNumWeightedInliers(), pairEvidence[idxPair], pose});
	}
	if (candidates.size() < config.minViews-1) {
		VERBOSE("error: insufficient initial views (%u < %u)",
			candidates.size(), config.minViews-1);
		return false;
	}
	// Sort candidates by evidence (descending)
	candidates.Sort([](const ConnectedView& a, const ConnectedView& b) {
		return a.numEvidence > b.numEvidence;
	});
	// Filter candidates by minimum evidence per view
	if (config.ratioInliersFilter > 0) {
		// Compute robust threshold top 3 views: by their inliers, the minimum a view's evidence must pass
		UnsignedArr inliers(candidates.size());
		FOREACH(i, candidates)
			inliers[i] = candidates[i].numInliers;
		inliers.Sort([](unsigned a, unsigned b) { return a > b; });
		unsigned avgInliers = 0;
		const IIndex numCandidates = candidates.size();
		IIndex count = MINF(3u, numCandidates);
		for (unsigned i = 0; i < count; ++i)
			avgInliers += inliers[i];
		avgInliers /= count;
		const unsigned thInliers = ROUND2INT<unsigned>(avgInliers * config.ratioInliersFilter);
		IIndex size = numCandidates;
		while (size-- > 0 && candidates[size].numEvidence < thInliers);
		candidates.resize(size + 1);
		DEBUG_EXTRA("Filtered connected views by inlier matches: kept %u/%u views with at least %u inliers of evidence",
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
	BAConfig baConfig = config.baConfig;
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

	// 7. Bundle adjustment with focal-length refinement: distortion is left to the resection's
	// global bundle adjustments once the model is large enough to constrain it
	// (ResectionConfig::minRefineExtIntrs and the final one, both of which already free it) --
	// on a star of a handful of views it is unconstrained, and a converged focal can still be
	// bent by a solve free to also absorb error into k1-k3. A forced or otherwise known focal
	// (config.refineFocalLength == false) is left untouched: the bundle adjustment below runs
	// with the intrinsics flags baConfig already carries (none, so all of them fixed).
	if (config.refineFocalLength)
		baConfig.refineFocalLength = true;
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
