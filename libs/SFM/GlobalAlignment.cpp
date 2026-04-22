/*
 * GlobalAlignment.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "GlobalAlignment.h"
#include "GlobalRotationAveraging.h"
#include "GlobalScaleAveraging.h"
#include "GlobalTranslationAveraging.h"
#include "Scene.h"
#include "SimilarityTransform.h"
#include "Track.h"
#include "Triangulation.h"
#include "InterfaceMVS.h"
#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#pragma pop_macro("VERBOSE")
#pragma pop_macro("LOG")

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)

// enable to export intermediate aligned sub-scenes for debugging
#define GLOBALALIGNMENT_DEBUG 0


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("GlbAlign"));


GlobalAlignment::GlobalAlignment(Scene& _scene, const GlobalAlignmentConfig& _config)
	: scene(_scene), config(_config)
{
}

void GlobalAlignment::BuildGlobalToLocalMap(const std::vector<IIndexArr>& localToGlobals)
{
	globalToLocal.clear();
	for (uint32_t sceneIdx = 0; sceneIdx < localToGlobals.size(); ++sceneIdx) {
		const IIndexArr& mapping = localToGlobals[sceneIdx];
		for (IIndex localID = 0; localID < mapping.size(); ++localID) {
			const IIndex globalID = mapping[localID];
			if (globalID == NO_ID)
				continue;
			MAYBEUNUSED const auto [it, inserted] = globalToLocal.emplace(globalID, std::make_pair(sceneIdx, localID));
			ASSERT(inserted, "global image %u appears in multiple sub-scenes (%u:%u and %u:%u)",
				globalID, it->second.first, it->second.second, sceneIdx, localID);
		}
	}
}

bool GlobalAlignment::MergeScenes(std::vector<Scene>& subScenes, const std::vector<IIndexArr>& localToGlobals)
{
	TD_TIMER_STARTD();

	ASSERT(!subScenes.empty());
	ASSERT(subScenes.size() == localToGlobals.size());
	const uint32_t numSubScenes = (uint32_t)subScenes.size();
	VERBOSE("Merging %u sub-scenes into global scene", numSubScenes);

	#if GLOBALALIGNMENT_DEBUG
	// Export sub-scenes before alignment for debugging
	FOREACH(i, subScenes)
		subScenes[i].ExportPLY(String::FormatString("subscene_%u.ply", i));
	#endif

	BuildGlobalToLocalMap(localToGlobals);

	// Stage 1: Estimate relative poses between connected sub-scenes
	std::vector<ScenePair> scenePairs;
	if (!EstimateRelativePoses(subScenes, scenePairs)) {
		VERBOSE("error: failed to estimate relative poses");
		return false;
	}

	// If only one sub-scene or no connections, just copy directly
	if (numSubScenes == 1 || scenePairs.empty()) {
		VERBOSE("Single sub-scene or no connections, copying directly");
		for (uint32_t sceneIdx = 0; sceneIdx < numSubScenes; ++sceneIdx) {
			MergeSingleScene(subScenes[sceneIdx], localToGlobals[sceneIdx]);
		}
		DEBUG("Single-scene merge completed (%s)", TD_TIMER_GET_FMT().c_str());
		return true;
	}

	// Stage 2: Rotation averaging
	std::vector<Point3d> globalRotations;
	if (!EstimateGlobalRotations(scenePairs, numSubScenes, globalRotations)) {
		VERBOSE("error: failed to estimate global rotations");
		return false;
	}

	// Stage 3: Scale averaging
	std::vector<REAL> globalScales;
	if (!EstimateGlobalScales(scenePairs, numSubScenes, globalScales)) {
		VERBOSE("error: failed to estimate global scales");
		return false;
	}

	// Stage 4: Translation averaging
	std::vector<Point3> globalTranslations;
	if (!EstimateGlobalTranslations(scenePairs, globalRotations, globalScales, numSubScenes, globalTranslations)) {
		VERBOSE("error: failed to estimate global translations");
		return false;
	}

	// Stage 5: Merge sub-scenes with global transforms
	if (!MergeTransformedScenes(subScenes, localToGlobals, globalRotations, globalScales, globalTranslations)) {
		VERBOSE("error: failed to merge transformed scenes");
		return false;
	}

	#if GLOBALALIGNMENT_DEBUG
	// Export merged scene for debugging
	ExportMVS(MAKE_PATH("scene_merged_reconstruction.mvs"), scene);
	#endif

	DEBUG("Global alignment completed: merged %u sub-scenes (%s)",
		numSubScenes, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


bool GlobalAlignment::EstimateRelativePoses(
	const std::vector<Scene>& subScenes,
	std::vector<ScenePair>& scenePairs)
{
	ASSERT(!globalToLocal.empty());
	const uint32_t numSubScenes = (uint32_t)subScenes.size();

	// Per-sub-scene cache: PairIdx(localImageID, featureID) -> 3D inlier-track position.
	// Only observations belonging to inlier tracks are indexed; outliers are excluded
	// because their triangulated positions are unreliable.
	std::vector<std::unordered_map<PairIdx, Point3>> sceneObsToTrackPos(numSubScenes);
	for (uint32_t sceneIdx = 0; sceneIdx < numSubScenes; ++sceneIdx) {
		const Scene& subScene = subScenes[sceneIdx];
		size_t numInlierObservations = 0;
		for (const Track& track : subScene.tracks)
			if (track.IsInlier())
				numInlierObservations += track.GetNumInliers();
		auto& obsToTrackPos = sceneObsToTrackPos[sceneIdx];
		obsToTrackPos.reserve(numInlierObservations);
		for (const Track& track : subScene.tracks) {
			if (track.IsInlier())
				for (const Observation& obs : track)
					obsToTrackPos.emplace(PairIdx(obs.imageID, obs.featureID), track.position);
		}
	}

	// Group cross-sub-scene image pairs by sub-scene pair. Each link remembers which
	// side of the image pair corresponds to sub-scene A vs B so feature indices
	// (queryIdx/trainIdx) can be mapped consistently.
	struct PairLink {
		const ImagePair* pair;
		IIndex localIdA;
		IIndex localIdB;
		bool aIsQuery; // true if pair.ID1 belongs to sub-scene A (i.e. uses queryIdx)
	};
	std::unordered_map<PairIdx, CLISTDEF0IDX(PairLink, uint32_t)> linksByScenePair;
	linksByScenePair.reserve(numSubScenes * 2);
	for (const ImagePair& pair : scene.pairs) {
		if (pair.GetNumFilteredInliers() < config.minCommonTracks)
			continue;
		auto it1 = globalToLocal.find(pair.ID1);
		auto it2 = globalToLocal.find(pair.ID2);
		if (it1 == globalToLocal.end() || it2 == globalToLocal.end())
			continue;
		const uint32_t scene1 = it1->second.first;
		const uint32_t scene2 = it2->second.first;
		if (scene1 == scene2)
			continue;
		PairLink link;
		link.pair = &pair;
		if (scene1 < scene2) {
			link.localIdA = it1->second.second;
			link.localIdB = it2->second.second;
			link.aIsQuery = true;
		} else {
			link.localIdA = it2->second.second;
			link.localIdB = it1->second.second;
			link.aIsQuery = false;
		}
		linksByScenePair[MakePairIdx(scene1, scene2)].emplace_back(link);
	}

	scenePairs.clear();
	unsigned numEstimated = 0;
	unsigned numSkippedPairs = 0;
	for (const auto& [pairIdx, links] : linksByScenePair) {
		const Scene& subSceneA = subScenes[pairIdx.i];
		const Scene& subSceneB = subScenes[pairIdx.j];
		const auto& obsToTrackPosA = sceneObsToTrackPos[pairIdx.i];
		const auto& obsToTrackPosB = sceneObsToTrackPos[pairIdx.j];

		// Collect 3D-3D correspondences: for each cross-sub-scene match whose
		// endpoints both lie on an existing inlier track, push the two 3D
		// positions (each in its own sub-scene's local frame).
		Point3Arr srcPoints, dstPoints;
		for (const PairLink& link : links) {
			ASSERT(link.localIdA < subSceneA.images.size() && link.localIdB < subSceneB.images.size());
			const Image& imgA = subSceneA.images[link.localIdA];
			const Image& imgB = subSceneB.images[link.localIdB];
			if (!imgA.IsValid() || !imgB.IsValid())
				continue;

			const unsigned numInliers = link.pair->GetNumFilteredInliers();
			for (unsigned i = 0; i < numInliers; ++i) {
				const DMatch& match = link.pair->matches[i];
				const uint32_t featureA = link.aIsQuery ? match.queryIdx : match.trainIdx;
				const uint32_t featureB = link.aIsQuery ? match.trainIdx : match.queryIdx;
				const auto itA = obsToTrackPosA.find(PairIdx(link.localIdA, featureA));
				if (itA == obsToTrackPosA.end())
					continue;
				const auto itB = obsToTrackPosB.find(PairIdx(link.localIdB, featureB));
				if (itB == obsToTrackPosB.end())
					continue;
				srcPoints.emplace_back(itA->second);
				dstPoints.emplace_back(itB->second);
			}
		}

		if (srcPoints.size() < config.minCommonTracks) {
			DEBUG_ULTIMATE("Sub-scene pair (%u, %u): skipped (only %u 3D-3D correspondences, need >= %u)",
				pairIdx.i, pairIdx.j, (unsigned)srcPoints.size(), config.minCommonTracks);
			++numSkippedPairs;
			continue;
		}

		// Characteristic length scale for the RANSAC threshold: 0.1% of the source
		// point cloud's bounding-box diagonal. Using a relative scale keeps the
		// criterion invariant to each sub-scene's arbitrary units.
		AABB3 srcBbox(true);
		for (const Point3& p : srcPoints)
			srcBbox.InsertFull(p);
		if (srcBbox.IsEmpty()) {
			++numSkippedPairs;
			continue;
		}
		const double threshold = 0.001 * srcBbox.GetSize().norm();

		Transform T;
		const unsigned numInliers = EstimateSimilarityTransform(srcPoints, dstPoints, T, threshold);
		if (numInliers == 0) {
			DEBUG_ULTIMATE("warning: sub-scene pair (%u, %u): Sim(3) RANSAC failed (%u correspondences)",
				pairIdx.i, pairIdx.j, (unsigned)srcPoints.size());
			++numSkippedPairs;
			continue;
		}
		if (numInliers < config.minCommonTracks) {
			DEBUG_ULTIMATE("warning: sub-scene pair (%u, %u): skipped (too few inliers %u/%u)",
				pairIdx.i, pairIdx.j, numInliers, (unsigned)srcPoints.size());
			++numSkippedPairs;
			continue;
		}
		const double inlierRatio = (double)numInliers / (double)srcPoints.size();
		if (inlierRatio < 0.3) {
			DEBUG_ULTIMATE("warning: sub-scene pair (%u, %u): skipped (low inlier ratio %.1f%% = %u/%u)",
				pairIdx.i, pairIdx.j, inlierRatio * 100.0, numInliers, (unsigned)srcPoints.size());
			++numSkippedPairs;
			continue;
		}

		ScenePair sp;
		sp.sceneA = pairIdx.i;
		sp.sceneB = pairIdx.j;
		sp.relativeTransform = T;
		sp.numInliers = numInliers;
		scenePairs.push_back(sp);
		++numEstimated;
		DEBUG_ULTIMATE("Sub-scene pair (%u, %u) Sim(3): scale=%.4g, inliers %u/%u (%.1f%%)",
			pairIdx.i, pairIdx.j, T.scale, numInliers, (unsigned)srcPoints.size(), inlierRatio * 100.0);
	}

	std::sort(scenePairs.begin(), scenePairs.end(), [](const ScenePair& a, const ScenePair& b) {
		return a.sceneA < b.sceneA || (a.sceneA == b.sceneA && a.sceneB < b.sceneB);
	});

	DEBUG("Estimated %u relative Sim(3) transforms between sub-scenes (%u skipped)",
		numEstimated, numSkippedPairs);
	return !scenePairs.empty();
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateGlobalRotations(
	const std::vector<ScenePair>& scenePairs,
	const uint32_t numSubScenes,
	std::vector<Point3d>& globalRotations)
{
	// Convert scene pairs to rotation pairs
	std::vector<RotationPair> rotationPairs;
	rotationPairs.reserve(scenePairs.size());

	for (const ScenePair& sp : scenePairs) {
		RotationPair rp;
		rp.idxA = sp.sceneA;
		rp.idxB = sp.sceneB;
		rp.relativeRotation = sp.relativeTransform.R;
		rp.weight = (float)sp.numInliers;
		rotationPairs.push_back(rp);
	}

	// Use global rotation estimator
	GlobalRotationEstimatorOptions rotOptions;
	rotOptions.skipInitialization = false;
	rotOptions.useWeight = true;

	GlobalRotationEstimator rotEstimator(rotOptions);
	if (!rotEstimator.EstimateRotations(rotationPairs, numSubScenes, globalRotations)) {
		VERBOSE("error: rotation averaging failed");
		return false;
	}

	// Filter relative rotations inconsistent with global estimates and re-solve
	const unsigned numFiltered = GlobalRotationEstimator::FilterRelativeRotations(globalRotations, rotationPairs);
	if (numFiltered > 0) {
		DEBUG("Re-estimating global rotations after filtering %u pairs", numFiltered);
		globalRotations.clear();
		if (!rotEstimator.EstimateRotations(rotationPairs, numSubScenes, globalRotations)) {
			VERBOSE("error: rotation averaging failed after filtering");
			return false;
		}
	}

	DEBUG("Estimated %u global rotations", (unsigned)globalRotations.size());
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateGlobalScales(
	const std::vector<ScenePair>& scenePairs,
	const uint32_t numSubScenes,
	std::vector<REAL>& globalScales)
{
	// Each relativeTransform satisfies p_B = (s_A/s_B) * R * p_A + t, so its
	// scale field is s_A/s_B. ScalePair expects the ratio in the opposite
	// direction (s_B/s_A), hence the reciprocal below.
	std::vector<ScalePair> scalePairs;
	scalePairs.reserve(scenePairs.size());
	for (const ScenePair& sp : scenePairs) {
		if (sp.relativeTransform.scale <= 0)
			continue;
		ScalePair scalePair;
		scalePair.idxA = sp.sceneA;
		scalePair.idxB = sp.sceneB;
		scalePair.scaleRatio = REAL(1) / sp.relativeTransform.scale;
		scalePair.weight = (float)sp.numInliers;
		scalePairs.push_back(scalePair);
	}

	if (scalePairs.empty()) {
		// No scale information, use unit scales
		VERBOSE("warning: no scale pairs found, using unit scales");
		globalScales.resize(numSubScenes, REAL(1));
		return true;
	}

	// Estimate global scales
	GlobalScaleEstimator scaleEstimator;
	if (!scaleEstimator.EstimateScales(scalePairs, numSubScenes, globalScales)) {
		VERBOSE("error: scale averaging failed");
		return false;
	}

	DEBUG("Estimated %u global scales from %u pairs",
		(unsigned)globalScales.size(), (unsigned)scalePairs.size());
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateGlobalTranslations(
	const std::vector<ScenePair>& scenePairs,
	const std::vector<Point3d>& globalRotations,
	const std::vector<REAL>& globalScales,
	const uint32_t numSubScenes,
	std::vector<Point3>& globalTranslations)
{
	// Convert scene pairs to translation pairs
	std::vector<TranslationPair> translationPairs;
	translationPairs.reserve(scenePairs.size());

	for (const ScenePair& sp : scenePairs) {
		// Rotation averaging produces R_i mapping global→local, use transpose for local→global.
		const RMatrix RA(globalRotations[sp.sceneA]);
		const REAL sA = globalScales[sp.sceneA];

		// C_{BA}: position of scene B's origin expressed in scene A's local frame.
		// The relativeTransform satisfies p_B = scale * R * p_A + t (A -> B), so the
		// inverse maps B's origin (0 in B) to  -(1/scale) * R^T * t  in A.
		const Transform& T = sp.relativeTransform;
		const Point3 relT_local = (T.R.t() * T.t) * (-REAL(1) / T.scale);

		// Transform to global frame: t_B - t_A = s_A * R_A^T * C_{BA}
		const Point3 relT_global = sA * (RA.t() * relT_local);

		TranslationPair tp;
		tp.idxA = sp.sceneA;
		tp.idxB = sp.sceneB;
		tp.relativeTranslation = relT_global;
		tp.weight = (float)sp.numInliers;
		translationPairs.push_back(tp);
	}

	// Estimate global translations
	GlobalTranslationEstimator translationEstimator;
	if (!translationEstimator.EstimateTranslations(translationPairs, numSubScenes, globalTranslations)) {
		VERBOSE("error: translation averaging failed");
		return false;
	}

	DEBUG("Estimated %u global translations", (unsigned)globalTranslations.size());
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::MergeTransformedScenes(
	std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	const std::vector<Point3d>& globalRotations,
	const std::vector<REAL>& globalScales,
	const std::vector<Point3>& globalTranslations)
{
	// Track per-camera accumulation counts; destination cameras accumulate directly
	std::unordered_map<Camera*, unsigned> cameraAccumCount;

	// Transform each sub-scene and merge into global scene
	scene.status.nCalibratedImages = 0;
	FOREACH(sceneIdx, subScenes) {
		Scene& subScene = subScenes[sceneIdx];

		// Build similarity transform for this sub-scene
		// Rotation averaging produces R_i mapping global→local (same convention as Image.R),
		// but the similarity transform needs local→global, so transpose
		SEACAVE::Transform T;
		T.R = RMatrix(globalRotations[sceneIdx]).t();
		T.scale = globalScales[sceneIdx];
		T.t = globalTranslations[sceneIdx];

		// Apply transform to sub-scene
		subScene.Transform(T);

		#if GLOBALALIGNMENT_DEBUG
		// Export aligned sub-scene for debugging
		subScene.ExportPLY(String::FormatString("subscene_%u_aligned.ply", sceneIdx));
		#endif

		// Accumulate intrinsics from sub-scene cameras into destination cameras
		const IIndexArr& localToGlobal = localToGlobals[sceneIdx];
		for (IIndex localID = 0; localID < subScene.images.size(); ++localID) {
			const IIndex globalID = localToGlobal[localID];
			if (globalID == NO_ID || globalID >= scene.images.size())
				continue;
			const Image& srcImg = subScene.images[localID];
			Image& dstImg = scene.images[globalID];
			if (!srcImg.IsValid() || !srcImg.HasCamera() || !dstImg.HasCamera())
				continue;
			auto [it, inserted] = cameraAccumCount.emplace(dstImg.pCamera, 0);
			if (inserted)
				dstImg.pCamera->ResetIntrinsics();
			dstImg.pCamera->AccumulateIntrinsics(*srcImg.pCamera);
			++it->second;
		}

		// Merge into global scene
		MergeSingleScene(subScene, localToGlobal);
	}

	// Finalize intrinsics averaging
	for (const auto& [cam, count] : cameraAccumCount) {
		if (count > 0) {
			cam->ScaleIntrinsics(REAL(1) / count);
			DEBUG_EXTRA("Camera intrinsics averaged (%u sub-scenes): %s", count, cam->GetIntrinsicsString().c_str());
		}
	}

	// Merge sub-scene tracks and connect them via cross-sub-scene pairs
	MergeTracksWithCrossSubScenePairs();
	FilterTracks(scene, 16.f, 0.5f);

	DEBUG("Merged %u transformed sub-scenes (%u tracks, %u calibrated images)",
		(unsigned)subScenes.size(), (unsigned)scene.tracks.size(), scene.status.nCalibratedImages);
	return true;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::MergeSingleScene(Scene& subScene, const IIndexArr& localToGlobal)
{
	// Copy image poses and move back keypoints/descriptors
	// (keypoints/descriptors were moved to sub-scenes during ExtractSubScene to save memory)
	for (IIndex localID = 0; localID < subScene.images.size(); ++localID) {
		const IIndex globalID = localToGlobal[localID];
		if (globalID == NO_ID || globalID >= scene.images.size())
			continue;

		Image& srcImg = subScene.images[localID];
		Image& dstImg = scene.images[globalID];

		if (srcImg.IsValid()) {
			if (!dstImg.IsValid())
				++scene.status.nCalibratedImages;
			dstImg.R = srcImg.R;
			dstImg.C = srcImg.C;
		}
		// Move keypoints/descriptors back from sub-scene to global scene
		if (srcImg.HasFeatures() && !dstImg.HasFeatures()) {
			dstImg.keypoints = std::move(srcImg.keypoints);
			dstImg.descriptors = std::move(srcImg.descriptors);
		}
	}

	// Remap and merge image-pairs from local sub-scene into global scene.
	// Ordering invariant: localToGlobal is sorted by global ID, so
	// localID1 < localID2 implies globalID1 < globalID2.
	for (ImagePair& srcPair : subScene.pairs) {
		if (srcPair.ID1 >= localToGlobal.size() || srcPair.ID2 >= localToGlobal.size())
			continue;
		IIndex globalID1 = localToGlobal[srcPair.ID1];
		IIndex globalID2 = localToGlobal[srcPair.ID2];
		if (globalID1 == NO_ID || globalID2 == NO_ID)
			continue;
		ASSERT(globalID1 < globalID2);
		srcPair.ID1 = globalID1;
		srcPair.ID2 = globalID2;
		ASSERT(scene.FindPair(srcPair.ID1, srcPair.ID2) == NULL);
		scene.pairs.emplace_back(std::move(srcPair));
	}

	// Merge tracks and colors together to keep indices aligned
	const bool hasColors = !subScene.colors.empty() && subScene.colors.size() == subScene.tracks.size();
	scene.tracks.reserve(scene.tracks.size() + subScene.tracks.size());
	if (hasColors)
		scene.colors.reserve(scene.colors.size() + subScene.tracks.size());

	FOREACH(srcIdx, subScene.tracks) {
		const Track& srcTrack = subScene.tracks[srcIdx];
		if (!srcTrack.IsValid())
			continue;

		Track dstTrack = srcTrack;

		// Remap observation image IDs from local to global
		for (Observation& obs : dstTrack.observations) {
			if (obs.imageID < localToGlobal.size()) {
				const IIndex globalID = localToGlobal[obs.imageID];
				if (globalID != NO_ID)
					obs.imageID = globalID;
				else
					obs.imageID = NO_ID; // invalid observation
			}
		}

		// Remove invalid observations (decrement numInliers if an inlier was removed)
		for (IIndex i = dstTrack.observations.size(); i-- > 0; ) {
			if (dstTrack.observations[i].imageID == NO_ID) {
				if (i < dstTrack.numInliers)
					--dstTrack.numInliers;
				dstTrack.observations.RemoveAtMove(i);
			}
		}

		if (!dstTrack.IsValid())
			continue;

		if (dstTrack.IsInlier())
			++scene.status.nTracks;
		scene.tracks.push_back(dstTrack);
		if (hasColors)
			scene.colors.push_back(subScene.colors[srcIdx]);
	}
}
/*----------------------------------------------------------------*/


void GlobalAlignment::MergeTracksWithCrossSubScenePairs()
{
	// Per-root metadata for union-find: 3D position, inlier count, image set
	struct RootMeta {
		Point3 position{Point3::ZERO};
		uint32_t numInliers{0};
		std::unique_ptr<std::unordered_map<IIndex, uint16_t>> imageCount;
		bool hasPosition{false};

		void InitImageCount() {
			if (!imageCount)
				imageCount = std::make_unique<std::unordered_map<IIndex, uint16_t>>();
		}
	};

	// Phase 1: Build featureOffsets and initialize DisjointSet from existing tracks
	//
	// Each feature across all images gets a unique global ID via featureOffsets:
	//   globalID = featureOffsets[imageID] + featureID
	// We then seed the union-find by unioning all observations within each
	// sub-scene track into a single set. This preserves the track structure
	// from each independently-reconstructed sub-scene.

	// Compute feature offsets for O(1) global ID lookup (same as BuildTracks)
	Unsigned32Arr featureOffsets(0, scene.images.size() + 1);
	uint32_t totalFeatures = 0;
	for (const Image& img : scene.images) {
		featureOffsets.push_back(totalFeatures);
		totalFeatures += (uint32_t)img.keypoints.size();
	}
	featureOffsets.push_back(totalFeatures); // sentinel
	if (totalFeatures == 0) {
		DEBUG("warning: no features for track merging");
		return;
	}

	DisjointSet<uint32_t> ds(totalFeatures);
	std::vector<RootMeta> rootMeta(totalFeatures);
	std::vector<bool> featureCounted(totalFeatures, false);

	// Initialize union-find sets from existing sub-scene tracks.
	// config.mergeTrackInliersOnly controls whether we seed with only inlier
	// observations (first numInliers entries, which passed reprojection filtering)
	// or all observations including outliers.
	const bool useOnlyInliers = config.mergeTrackInliersOnly;
	AABB3 bbox(true);
	for (const Track& track : scene.tracks) {
		if (!track.IsValid())
			continue;
		const uint32_t numObs = useOnlyInliers
			? (uint32_t)track.numInliers
			: (uint32_t)track.observations.size();
		if (numObs < 2)
			continue;
		// Union selected observations into one set
		uint32_t firstGid = NO_ID;
		for (uint32_t i = 0; i < numObs; ++i) {
			const Observation& obs = track.observations[i];
			if (obs.imageID >= scene.images.size())
				continue;
			if (obs.featureID >= scene.images[obs.imageID].keypoints.size())
				continue;
			const uint32_t gid = featureOffsets[obs.imageID] + obs.featureID;
			featureCounted[gid] = true;
			if (firstGid == NO_ID)
				firstGid = gid;
			else
				ds.Union(firstGid, gid);
		}
		if (firstGid == NO_ID)
			continue;
		// Store metadata at root
		const uint32_t root = ds.Find(firstGid);
		RootMeta& meta = rootMeta[root];
		meta.position = track.position;
		meta.numInliers = track.numInliers;
		meta.hasPosition = track.IsInlier();
		meta.InitImageCount();
		for (uint32_t i = 0; i < numObs; ++i) {
			const Observation& obs = track.observations[i];
			if (obs.imageID < scene.images.size() &&
				obs.featureID < scene.images[obs.imageID].keypoints.size())
				++((*meta.imageCount)[obs.imageID]);
		}
		if (meta.hasPosition)
			bbox.InsertFull(track.position);
	}

	// Compute proximity threshold from scene bounding box
	const REAL proximityThreshold = bbox.IsEmpty() ? REAL(0) : REAL(0.02) * bbox.GetSize().norm();

	// Phase 2: Process ONLY cross-sub-scene pairs (connecting pairs) to merge
	// tracks across sub-scene boundaries.
	//
	// A pair is a "connecting pair" if its two images belong to different sub-scenes.
	// Intra-sub-scene pairs are skipped: their tracks are already correctly formed
	// by BuildTracks during sub-scene reconstruction. Re-processing them here would
	// over-merge tracks within a sub-scene (because outlier observations removed
	// during reconstruction can lift the duplicate-image guard that originally kept
	// the tracks separate), bloating image sets and blocking legitimate cross-sub-scene
	// connections via the duplicate-image guard.
	unsigned numMerged = 0, numRejectedProximity = 0, numRejectedDupImage = 0, numNewPairTracks = 0;
	unsigned numCrossScenePairs = 0;

	// Ensure root has metadata and feature is counted exactly once;
	// new features from cross-sub-scene pairs are counted as additional observations
	// but do NOT increment numInliers (these are unvalidated matches, not verified inliers)
	auto AccumulateFeature = [&](uint32_t gid, IIndex imgID) {
		if (featureCounted[gid])
			return;
		featureCounted[gid] = true;
		const uint32_t root = ds.Find(gid);
		RootMeta& meta = rootMeta[root];
		meta.InitImageCount();
		++((*meta.imageCount)[imgID]);
	};

	for (const ImagePair& pair : scene.pairs) {
		if (!pair.HasMatches())
			continue;
		ASSERT(pair.ID1 < scene.images.size() && pair.ID2 < scene.images.size());
		ASSERT(scene.images[pair.ID1].HasFeatures() && scene.images[pair.ID2].HasFeatures());
		// Filter: only process cross-sub-scene pairs.
		// Both images must be in globalToLocal (assigned to a sub-scene)
		// and must belong to different sub-scenes.
		const auto itA = globalToLocal.find(pair.ID1);
		const auto itB = globalToLocal.find(pair.ID2);
		if (itA == globalToLocal.end() || itB == globalToLocal.end())
			continue;
		if (itA->second.first == itB->second.first)
			continue; // same sub-scene, skip
		++numCrossScenePairs;

		const uint32_t offset1 = featureOffsets[pair.ID1];
		const uint32_t offset2 = featureOffsets[pair.ID2];
		FOREACHRAW(i, pair.GetNumFilteredInliers()) {
			const DMatch& match = pair.matches[i];
			if ((unsigned)match.queryIdx >= scene.images[pair.ID1].keypoints.size() ||
				(unsigned)match.trainIdx >= scene.images[pair.ID2].keypoints.size())
				continue;
			const uint32_t gid1 = offset1 + match.queryIdx;
			const uint32_t gid2 = offset2 + match.trainIdx;

			// Ensure both features are counted in their root metadata
			AccumulateFeature(gid1, pair.ID1);
			AccumulateFeature(gid2, pair.ID2);

			// Attempt union with image-uniqueness and 3D proximity guards
			ds.UnionIf(gid1, gid2,
				[&](uint32_t rootDst, uint32_t rootSrc) -> bool {
					RootMeta& metaDst = rootMeta[rootDst];
					RootMeta& metaSrc = rootMeta[rootSrc];
					ASSERT(metaDst.imageCount && metaSrc.imageCount);
					// Guard 1: reject if merging would create duplicate image observations
					for (const auto& [imgID, count] : *metaSrc.imageCount) {
						if (metaDst.imageCount->count(imgID)) {
							++numRejectedDupImage;
							return false;
						}
					}
					// Guard 2: if both sides have triangulated 3D positions,
					// reject if they are too far apart (indicates false match)
					if (metaDst.hasPosition && metaSrc.hasPosition && proximityThreshold > 0) {
						if (norm(metaDst.position - metaSrc.position) > proximityThreshold) {
							++numRejectedProximity;
							return false;
						}
					}
					// Merge metadata: weighted-average 3D positions, merge image sets
					if (metaDst.hasPosition && metaSrc.hasPosition) {
						const REAL wDst = (REAL)metaDst.numInliers;
						const REAL wSrc = (REAL)metaSrc.numInliers;
						metaDst.position = (metaDst.position * wDst + metaSrc.position * wSrc) / (wDst + wSrc);
					} else if (metaSrc.hasPosition) {
						metaDst.position = metaSrc.position;
						metaDst.hasPosition = true;
					}
					metaDst.numInliers += metaSrc.numInliers;
					for (const auto& [imgID, count] : *metaSrc.imageCount)
						(*metaDst.imageCount)[imgID] += count;
					metaSrc.imageCount.reset();
					++numMerged;
					return true;
				}
			);
		}
	}

	// Phase 3: Assemble final tracks grouped by union-find root
	std::unordered_map<uint32_t, ObservationArr> trackGroups;
	FOREACH(imgID, scene.images) {
		const Image& img = scene.images[imgID];
		const uint32_t offset = featureOffsets[imgID];
		for (uint32_t fid = 0; fid < (uint32_t)img.keypoints.size(); ++fid) {
			const uint32_t gid = offset + fid;
			const uint32_t root = ds.Find(gid);
			// Only include features that belong to a set with metadata (i.e., part of a track)
			if (!rootMeta[root].imageCount)
				continue;
			trackGroups[root].emplace_back(imgID, fid);
		}
	}

	scene.tracks.Release();
	scene.colors.Release(); // colors indexed in parallel with tracks; must be rebuilt
	scene.status.nTracks = 0;
	scene.tracks.reserve(trackGroups.size());

	for (auto& [root, observations] : trackGroups) {
		if (observations.size() < 2)
			continue;
		observations.Sort();
		Track track;
		track.observations = std::move(observations);
		const RootMeta& meta = rootMeta[root];
		if (meta.hasPosition) {
			// Use averaged 3D position from merged sub-scene tracks;
			// numInliers from accumulated metadata (original inliers + cross-sub-scene additions)
			track.position = meta.position;
			track.numInliers = (uint8_t)MINF(MINF(meta.numInliers, (uint32_t)track.observations.size()), 255u);
			++scene.status.nTracks;
		} else {
			// New track without 3D: triangulate
			if (TriangulateSkewLLS(track, scene.images) >= 2) {
				++scene.status.nTracks;
				++numNewPairTracks;
			}
			// tracks with failed triangulation kept with numInliers=0,
			// excluded from BA until next triangulation attempt
		}
		scene.tracks.emplace_back(std::move(track));
	}

	DEBUG("Track merge: %u/%u tracks, %u cross-sub-scene merges from %u connecting pairs, "
		"%u new from pairs, %u rejected by proximity, %u rejected by duplicate image",
		scene.status.nTracks, scene.tracks.size(), numMerged, numCrossScenePairs,
		numNewPairTracks, numRejectedProximity, numRejectedDupImage);
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
