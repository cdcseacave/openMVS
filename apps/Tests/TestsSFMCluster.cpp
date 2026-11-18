/*
 * TestsSFMCluster.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
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

#include "../../libs/SFM.h"
#include "../../libs/SFM/GlobalRotationAveraging.h"
#include "../../libs/SFM/GlobalScaleAveraging.h"
#include "../../libs/SFM/GlobalTranslationAveraging.h"
#include "../../libs/Math/DisjointSet.h"
#include "../../libs/SFM/RobustAveraging.h"
#include "../../libs/SFM/PairsWeighting.h"
#include "../../libs/SFM/ViewGraphCalibrator.h"
#include "../../libs/SFM/BundleAdjustment.h"
#include "../../libs/SFM/SceneCluster.h"
#include "../../libs/SFM/SceneBlocks.h"
#include "../../libs/SFM/GlobalAlignment.h"
#include "../../libs/SFM/PoseLink.h"
#include "../../libs/SFM/Link2D3D.h"
#include "../../libs/SFM/Triangulation.h"
#include "../../libs/SFM/MatchGeometric.h"
#include "../../libs/SFM/LensFit.h"
#include "../../libs/SFM/RoMa2Matcher.h"
#include "../../libs/SFM/FlexGlueMatcher.h"
#include "../../libs/SFM/SphereCubeMap.h"
#include "../../libs/SFM/InterfaceMVS.h"
#include "../../libs/MVS.h"
#include <nlohmann/json.hpp>
#include "Tests.h"
#include "TestsSFMFramework.h"
#include <filesystem>
#include <chrono>
#include <random>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestSFM "));

namespace SFM {

// Test 1: Single cluster passthrough and disabled clustering
bool SceneClusterSingleClusterTest()
{
	TD_TIMER_START();

	// Sub-test A: nViews <= maxViewsPerCluster → no split
	{
		Scene scene;
		SceneConfig cfg;
		cfg.numImages = 8;
		cfg.numPoints = 60;
		cfg.generatePairs = true;
		cfg.generateDescriptors = true;
		GenerateTestScene(scene, cfg);
		ComputePairsWeights(scene);

		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = 10; // 8 <= 10, no split
		SceneCluster cluster(scene, clusterCfg);
		std::vector<IIndexArr> localToGlobals;
		std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

		if (subScenes.size() != 1) {
			VERBOSE("SceneClusterSingleClusterTest FAILED: expected 1 sub-scene, got %u", (unsigned)subScenes.size());
			return false;
		}
		// The scene was std::move'd into subScenes[0]
		if (subScenes[0].images.size() != 8) {
			VERBOSE("SceneClusterSingleClusterTest FAILED: expected 8 images, got %u", (unsigned)subScenes[0].images.size());
			return false;
		}
	}

	// Sub-test B: maxViewsPerCluster == 0 → disabled
	{
		Scene scene;
		SceneConfig cfg;
		cfg.numImages = 8;
		cfg.numPoints = 60;
		cfg.generatePairs = true;
		cfg.generateDescriptors = true;
		GenerateTestScene(scene, cfg);
		ComputePairsWeights(scene);

		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = 0;
		SceneCluster cluster(scene, clusterCfg);
		std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition());

		if (subScenes.size() != 1) {
			VERBOSE("SceneClusterSingleClusterTest FAILED: disabled clustering should return 1 sub-scene, got %u", (unsigned)subScenes.size());
			return false;
		}
	}

	VERBOSE("SceneClusterSingleClusterTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 2: Size constraints and coverage
bool SceneClusterSizeConstraintsTest()
{
	TD_TIMER_START();

	Scene scene;
	GenerateTwoClusterScene(scene, 15, 15, 4, 40);

	const unsigned totalImages = (unsigned)scene.images.size();
	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 18;
	clusterCfg.targetViewsPerCluster = 12;
	clusterCfg.minViewsPerCluster = 5;
	clusterCfg.maxOverCapacity = 5;

	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	if (subScenes.size() < 2) {
		VERBOSE("SceneClusterSizeConstraintsTest FAILED: expected >= 2 sub-scenes, got %u", (unsigned)subScenes.size());
		return false;
	}

	// Verify size constraints
	for (unsigned s = 0; s < subScenes.size(); ++s) {
		const unsigned sz = (unsigned)subScenes[s].images.size();
		if (sz > clusterCfg.maxViewsPerCluster + clusterCfg.maxOverCapacity) {
			VERBOSE("SceneClusterSizeConstraintsTest FAILED: sub-scene %u has %u images > max %u",
				s, sz, clusterCfg.maxViewsPerCluster + clusterCfg.maxOverCapacity);
			return false;
		}
	}

	// Verify every image appears exactly once
	std::vector<int> imageCounts(totalImages, 0);
	for (unsigned s = 0; s < localToGlobals.size(); ++s) {
		for (IIndex globalID : localToGlobals[s]) {
			if (globalID < totalImages)
				++imageCounts[globalID];
		}
	}
	unsigned missingImages = 0;
	for (unsigned i = 0; i < totalImages; ++i) {
		if (imageCounts[i] > 1) {
			VERBOSE("SceneClusterSizeConstraintsTest FAILED: image %u in %d sub-scenes", i, imageCounts[i]);
			return false;
		}
		if (imageCounts[i] == 0)
			++missingImages;
	}
	// Allow a few images to be dropped (undersized clusters)
	if (missingImages > 3) {
		VERBOSE("SceneClusterSizeConstraintsTest FAILED: %u missing images (> 3 allowed)", missingImages);
		return false;
	}

	VERBOSE("SceneClusterSizeConstraintsTest PASSED: %u sub-scenes, %u missing images (%s)",
		(unsigned)subScenes.size(), missingImages, TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 3: Disconnected components get split into separate clusters
bool SceneClusterDisconnectedComponentsTest()
{
	TD_TIMER_START();

	// Create 20 images in two disconnected groups
	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 20;
	cfg.numPoints = 80;
	cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	cfg.rotationAngleStep = 18.0; // 360/20
	cfg.generateDescriptors = true;
	cfg.generatePairs = true;
	GenerateTestScene(scene, cfg);
	ComputePairsWeights(scene);

	// Remove all cross-group pairs (group A: 0-9, group B: 10-19)
	RFOREACH(i, scene.pairs) {
		const ImagePair& pair = scene.pairs[i];
		const bool aInFirst = pair.ID1 < 10;
		const bool bInFirst = pair.ID2 < 10;
		if (aInFirst != bInFirst) {
			scene.pairs.RemoveAtMove(i);
		}
	}
	// the two groups were never seen together, so no track may span them either: the split rebuilds
	// them from the pairs that remain
	scene.tracks.Release();

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 15; // Smaller than 20 to force split attempt
	clusterCfg.minViewsPerCluster = 5;

	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	if (subScenes.size() < 2) {
		VERBOSE("SceneClusterDisconnectedComponentsTest FAILED: expected >= 2 sub-scenes, got %u", (unsigned)subScenes.size());
		return false;
	}

	// Verify the two groups are separated: check no sub-scene mixes images from both groups
	for (unsigned s = 0; s < localToGlobals.size(); ++s) {
		bool hasFirst = false, hasSecond = false;
		for (IIndex gid : localToGlobals[s]) {
			if (gid < 10) hasFirst = true;
			else hasSecond = true;
		}
		if (hasFirst && hasSecond) {
			VERBOSE("SceneClusterDisconnectedComponentsTest FAILED: sub-scene %u mixes disconnected groups", s);
			return false;
		}
	}

	VERBOSE("SceneClusterDisconnectedComponentsTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 4: Memory protocol — keypoints MOVED, cross-pairs LEFT
bool SceneClusterMemoryProtocolTest()
{
	TD_TIMER_START();

	Scene scene;
	GenerateTwoClusterScene(scene, 12, 12, 4, 40, 80);

	// Record pre-split state
	const unsigned totalImages = (unsigned)scene.images.size();
	std::vector<size_t> origKeypointCounts(totalImages);
	for (unsigned i = 0; i < totalImages; ++i)
		origKeypointCounts[i] = scene.images[i].keypoints.size();
	const unsigned origPairCount = (unsigned)scene.pairs.size();

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 14;
	clusterCfg.minViewsPerCluster = 5;
	// the sub-scenes are this fixture, and its handful of points makes every seam look thin to a rule
	// written for a capture's tracks: no cluster is merged away here for want of strong neighbours
	clusterCfg.minClusterDegree = 0;

	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	if (subScenes.size() < 2) {
		VERBOSE("SceneClusterMemoryProtocolTest FAILED: expected >= 2 sub-scenes, got %u", (unsigned)subScenes.size());
		return false;
	}

	// Build set of assigned global images
	std::set<IIndex> assignedImages;
	for (const IIndexArr& mapping : localToGlobals)
		for (IIndex gid : mapping)
			assignedImages.insert(gid);

	// Check 1: Global images have empty keypoints (for assigned images)
	for (IIndex gid : assignedImages) {
		if (!scene.images[gid].keypoints.empty()) {
			VERBOSE("SceneClusterMemoryProtocolTest FAILED: global image %u still has keypoints after split", gid);
			return false;
		}
	}

	// Check 2: Sub-scene images have non-empty keypoints
	for (unsigned s = 0; s < subScenes.size(); ++s) {
		for (const Image& img : subScenes[s].images) {
			if (img.keypoints.empty()) {
				VERBOSE("SceneClusterMemoryProtocolTest FAILED: sub-scene %u has image with empty keypoints", s);
				return false;
			}
		}
	}

	// Check 3: Global scene retains only cross-sub-scene pairs
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.HasMatches())
			continue;
		// Both images must belong to different sub-scenes
		int sceneA = -1, sceneB = -1;
		for (unsigned s = 0; s < localToGlobals.size(); ++s) {
			for (IIndex gid : localToGlobals[s]) {
				if (gid == pair.ID1) sceneA = (int)s;
				if (gid == pair.ID2) sceneB = (int)s;
			}
		}
		if (sceneA == sceneB && sceneA != -1) {
			VERBOSE("SceneClusterMemoryProtocolTest FAILED: intra-cluster pair (%u,%u) remains in global", pair.ID1, pair.ID2);
			return false;
		}
	}

	// Check 4: Total pair count conservation
	unsigned subScenePairCount = 0;
	for (const Scene& sub : subScenes)
		subScenePairCount += (unsigned)sub.pairs.size();
	unsigned globalPairCount = 0;
	for (const ImagePair& p : scene.pairs)
		if (p.HasMatches())
			++globalPairCount;
	if (subScenePairCount + globalPairCount != origPairCount) {
		VERBOSE("SceneClusterMemoryProtocolTest FAILED: pair count mismatch: %u + %u != %u",
			subScenePairCount, globalPairCount, origPairCount);
		return false;
	}

	VERBOSE("SceneClusterMemoryProtocolTest PASSED: %u sub-scenes, %u cross-pairs remain (%s)",
		(unsigned)subScenes.size(), globalPairCount, TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 5: ID remapping consistency
bool SceneClusterIDRemappingTest()
{
	TD_TIMER_START();

	Scene scene;
	GenerateTwoClusterScene(scene, 12, 12, 4, 40, 80);

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 14;
	clusterCfg.minViewsPerCluster = 5;
	// the sub-scenes are this fixture, and its handful of points makes every seam look thin to a rule
	// written for a capture's tracks: no cluster is merged away here for want of strong neighbours
	clusterCfg.minClusterDegree = 0;

	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	if (subScenes.size() < 2) {
		VERBOSE("SceneClusterIDRemappingTest FAILED: expected >= 2 sub-scenes");
		return false;
	}

	// Check 1: localToGlobal maps to valid global IDs, no duplicates across sub-scenes
	std::set<IIndex> allGlobalIDs;
	for (unsigned s = 0; s < localToGlobals.size(); ++s) {
		for (IIndex localID = 0; localID < localToGlobals[s].size(); ++localID) {
			const IIndex globalID = localToGlobals[s][localID];
			if (globalID >= scene.images.size()) {
				VERBOSE("SceneClusterIDRemappingTest FAILED: invalid global ID %u in sub-scene %u", globalID, s);
				return false;
			}
			if (allGlobalIDs.count(globalID)) {
				VERBOSE("SceneClusterIDRemappingTest FAILED: global ID %u in multiple sub-scenes", globalID);
				return false;
			}
			allGlobalIDs.insert(globalID);
		}
	}

	// Check 2: Track observations use valid local IDs
	for (unsigned s = 0; s < subScenes.size(); ++s) {
		const Scene& sub = subScenes[s];
		for (const Track& track : sub.tracks) {
			for (const Observation& obs : track) {
				if (obs.imageID >= sub.images.size()) {
					VERBOSE("SceneClusterIDRemappingTest FAILED: track obs imageID %u >= %u in sub-scene %u",
						obs.imageID, (unsigned)sub.images.size(), s);
					return false;
				}
				if (obs.featureID >= sub.images[obs.imageID].keypoints.size()) {
					VERBOSE("SceneClusterIDRemappingTest FAILED: track obs featureID %u >= %u in sub-scene %u",
						obs.featureID, (unsigned)sub.images[obs.imageID].keypoints.size(), s);
					return false;
				}
			}
		}
	}

	// Check 3: Sub-scene pair IDs are valid local indices
	for (unsigned s = 0; s < subScenes.size(); ++s) {
		const Scene& sub = subScenes[s];
		for (const ImagePair& pair : sub.pairs) {
			if (pair.ID1 >= sub.images.size() || pair.ID2 >= sub.images.size()) {
				VERBOSE("SceneClusterIDRemappingTest FAILED: pair (%u,%u) exceeds image count %u in sub-scene %u",
					pair.ID1, pair.ID2, (unsigned)sub.images.size(), s);
				return false;
			}
			if (pair.ID1 >= pair.ID2) {
				VERBOSE("SceneClusterIDRemappingTest FAILED: pair (%u,%u) not ordered in sub-scene %u",
					pair.ID1, pair.ID2, s);
				return false;
			}
		}
	}

	VERBOSE("SceneClusterIDRemappingTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 6: Small clusters are rescued/absorbed
bool SceneClusterSmallClusterRescueTest()
{
	TD_TIMER_START();

	// Create scene: 12 strongly connected + 10 strongly connected + 3 weakly connected to cluster A
	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 25;
	cfg.numPoints = 100;
	cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	cfg.rotationAngleStep = 360.0 / 25;
	cfg.generateDescriptors = true;
	cfg.generatePairs = true;
	GenerateTestScene(scene, cfg);
	ComputePairsWeights(scene);

	// Structure: A=[0,12), B=[12,22), weak=[22,25)
	// Remove cross-group pairs except: weak images connect only to A (with low weight)
	RFOREACH(i, scene.pairs) {
		ImagePair& pair = scene.pairs[i];
		const bool id1InA = pair.ID1 < 12;
		const bool id2InA = pair.ID2 < 12;
		const bool id1InB = pair.ID1 >= 12 && pair.ID1 < 22;
		const bool id2InB = pair.ID2 >= 12 && pair.ID2 < 22;
		const bool id1InW = pair.ID1 >= 22;
		const bool id2InW = pair.ID2 >= 22;

		const bool intraA = id1InA && id2InA;
		const bool intraB = id1InB && id2InB;
		const bool weakToA = (id1InW && id2InA) || (id1InA && id2InW);
		const bool intraW = id1InW && id2InW;

		if (intraA || intraB) {
			pair.weightSpatial = 10.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 10.f;
			pair.weightTriplet = 10.f;
		} else if (weakToA) {
			pair.weightSpatial = 2.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 2.f;
			pair.weightTriplet = 0.f;
		} else if (intraW) {
			pair.weightSpatial = 1.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 1.f;
			pair.weightTriplet = 0.f;
		} else {
			// Remove other cross-group pairs
			scene.pairs.RemoveAtMove(i);
		}
	}

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 15;
	clusterCfg.targetViewsPerCluster = 10;
	clusterCfg.minViewsPerCluster = 5;
	clusterCfg.maxOverCapacity = 5;

	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	// Verify no output cluster has fewer than minViewsPerCluster
	for (unsigned s = 0; s < subScenes.size(); ++s) {
		if (subScenes[s].images.size() < clusterCfg.minViewsPerCluster) {
			VERBOSE("SceneClusterSmallClusterRescueTest FAILED: sub-scene %u has %u images < min %u",
				s, (unsigned)subScenes[s].images.size(), clusterCfg.minViewsPerCluster);
			return false;
		}
	}

	// Verify the 3 weak images are assigned (not dropped)
	std::set<IIndex> allAssigned;
	for (const IIndexArr& mapping : localToGlobals)
		for (IIndex gid : mapping)
			allAssigned.insert(gid);
	unsigned weakAssigned = 0;
	for (unsigned w = 22; w < 25; ++w)
		if (allAssigned.count(w))
			++weakAssigned;
	if (weakAssigned < 3) {
		VERBOSE("SceneClusterSmallClusterRescueTest FAILED: only %u/3 weak images rescued", weakAssigned);
		return false;
	}

	VERBOSE("SceneClusterSmallClusterRescueTest PASSED: %u sub-scenes, all weak images rescued (%s)",
		(unsigned)subScenes.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

// Two established clusters and a small, cohesive community: A holds 10 images at the target and B
// 11, one past it; W's 4 images are joined to one another by 6 pairs of weight 50, to B by four
// pairs of weight 20 and to A by a single pair of weight 4. The greedy merge cannot give W to B
// (B is past the target), and used to give it to A over that one pair because the coupling test
// skipped a side under the floor. The interface to A is 40 in the graph's integer weights against
// W's own 3000, far under minClusterCoupling, so W stays apart and the small-cluster pass then
// puts it with B, the neighbour it shares the most weight with.
bool SceneClusterSubFloorCouplingTest()
{
	TD_TIMER_START();

	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 25;
	cfg.numPoints = 0;
	cfg.generatePairs = false;
	cfg.generateDescriptors = false;
	cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	cfg.rotationAngleStep = 360.0 / 25;
	GenerateTestScene(scene, cfg);
	scene.pairs.clear();
	scene.tracks.clear();

	// a pair whose composite weight is exactly the given number: unit quality factors, triplet 0.75 (0.25 + 0.75 = 1)
	const auto AddPair = [&scene](IIndex a, IIndex b, float weight) {
		ImagePair pair(a, b);
		pair.weightedInliers = weight;
		pair.weightSpatial = 1.f;
		pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
		pair.weightConnectivity = 1.f;
		pair.weightTriplet = 0.75f;
		scene.pairs.emplace_back(std::move(pair));
	};
	// A = [0,10): a hub at 0 and a chain, so the greedy assembles it one image at a time
	for (IIndex k = 1; k < 10; ++k) AddPair(0, k, 200.f);
	for (IIndex k = 1; k + 1 < 10; ++k) AddPair(k, k + 1, 100.f);
	// B = [10,21): the same shape around a hub at 10, one image more than the target
	for (IIndex k = 11; k < 21; ++k) AddPair(10, k, 200.f);
	for (IIndex k = 11; k + 1 < 21; ++k) AddPair(k, k + 1, 100.f);
	// W = [21,25): every pair among them
	for (IIndex a = 21; a < 25; ++a) for (IIndex b = a + 1; b < 25; ++b) AddPair(a, b, 50.f);
	// W's interfaces
	for (IIndex k = 0; k < 4; ++k) AddPair(IIndex(21 + k), IIndex(10 + k), 20.f);
	AddPair(21, 9, 4.f);

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 15;
	clusterCfg.targetViewsPerCluster = 10;
	clusterCfg.minViewsPerCluster = 5;
	clusterCfg.maxOverCapacity = 5;
	clusterCfg.minClusterDegree = 0; // no tracks here, so the seam rules would merge every leaf away

	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	std::vector<int> subSceneOf(25, -1);
	FOREACH(s, localToGlobals)
		for (IIndex gid : localToGlobals[s])
			subSceneOf[gid] = (int)s;
	for (IIndex w = 21; w < 25; ++w) {
		if (subSceneOf[w] < 0 || subSceneOf[w] != subSceneOf[10] || subSceneOf[w] == subSceneOf[0]) {
			VERBOSE("SceneClusterSubFloorCouplingTest FAILED: image %u landed in sub-scene %d while B's hub is in %d and A's in %d; "
				"a community joined to a cluster by a pair carrying a fraction of its own cohesion must not be absorbed by it",
				w, subSceneOf[w], subSceneOf[10], subSceneOf[0]);
			return false;
		}
	}
	if (subSceneOf[0] == subSceneOf[10]) {
		VERBOSE("SceneClusterSubFloorCouplingTest FAILED: A and B ended in one sub-scene, so the fixture does not exercise the merge");
		return false;
	}

	VERBOSE("SceneClusterSubFloorCouplingTest PASSED: the community under the floor went to the cluster it shares its weight with, "
		"not to the one a thin pair joined it to (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// A small, cohesive community no neighbour is coupled to stands alone when the merge could place
// it, and is absorbed as before when it could not. A (10 images at the target) and B (11) as in the
// sub-floor coupling fixture; W's 4 images are joined to one another by 6 pairs of weight 50 (3000
// in the graph's integer weights) and to the rest by one pair of 6 to B and one of 4 to A: its
// heaviest interface, 60, is 2% of its own cohesion, so the greedy merge refuses it everywhere.
// With 80 tracks seen by three W cameras and three B cameras -- a strong seam by the merge's own
// bars (75 tracks, 3 cameras a side with 30) -- W is a sub-scene of its own; with no tracks it goes
// to B, the neighbour it shares the most weight with, as every cluster under the floor used to.
bool SceneClusterStandaloneCommunityTest()
{
	TD_TIMER_START();

	const auto BuildScene = [](Scene& scene, bool withSeamTracks) {
		SceneConfig cfg;
		cfg.numImages = 25;
		cfg.numPoints = 0;
		cfg.generatePairs = false;
		cfg.generateDescriptors = false;
		cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
		cfg.rotationAngleStep = 360.0 / 25;
		GenerateTestScene(scene, cfg);
		scene.pairs.clear();
		scene.tracks.clear();
		// a pair whose composite weight is exactly the given number: unit quality factors, triplet 0.75 (0.25 + 0.75 = 1)
		const auto AddPair = [&scene](IIndex a, IIndex b, float weight) {
			ImagePair pair(a, b);
			pair.weightedInliers = weight;
			pair.weightSpatial = 1.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 1.f;
			pair.weightTriplet = 0.75f;
			scene.pairs.emplace_back(std::move(pair));
		};
		// A = [0,10): a hub at 0 and a chain; B = [10,21): the same around a hub at 10
		for (IIndex k = 1; k < 10; ++k) AddPair(0, k, 200.f);
		for (IIndex k = 1; k + 1 < 10; ++k) AddPair(k, k + 1, 100.f);
		for (IIndex k = 11; k < 21; ++k) AddPair(10, k, 200.f);
		for (IIndex k = 11; k + 1 < 21; ++k) AddPair(k, k + 1, 100.f);
		// W = [21,25): every pair among them, and two thin pairs out
		for (IIndex a = 21; a < 25; ++a) for (IIndex b = a + 1; b < 25; ++b) AddPair(a, b, 50.f);
		AddPair(21, 10, 6.f);
		AddPair(21, 9, 4.f);
		if (withSeamTracks) {
			// 80 tracks each seen by W's 21, 22, 23 and B's 10, 11, 12: seam-usable on both sides,
			// 80 >= 75 tracks, three cameras a side with >= 30 (Partition builds tracks only when
			// the scene has none, so these are the seam's)
			for (uint32_t t = 0; t < 80; ++t) {
				Track& track = scene.tracks.emplace_back();
				for (uint32_t img : {21u, 22u, 23u, 10u, 11u, 12u})
					track.observations.emplace_back(img, t);
				track.numInliers = (uint8_t)track.observations.size();
			}
		}
	};

	for (unsigned c = 0; c < 2; ++c) {
		const bool withSeamTracks = c == 0;
		Scene scene;
		BuildScene(scene, withSeamTracks);
		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = 15;
		clusterCfg.targetViewsPerCluster = 10;
		clusterCfg.minViewsPerCluster = 5;
		clusterCfg.maxOverCapacity = 5;
		SceneCluster cluster(scene, clusterCfg);
		std::vector<IIndexArr> localToGlobals;
		std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

		std::vector<int> subSceneOf(25, -1);
		FOREACH(s, localToGlobals)
			for (IIndex gid : localToGlobals[s])
				subSceneOf[gid] = (int)s;
		if (subSceneOf[0] < 0 || subSceneOf[10] < 0 || subSceneOf[0] == subSceneOf[10]) {
			VERBOSE("SceneClusterStandaloneCommunityTest FAILED: A and B must be two sub-scenes (%d, %d), the fixture does not exercise the small-cluster pass otherwise", subSceneOf[0], subSceneOf[10]);
			return false;
		}
		for (IIndex w = 22; w < 25; ++w) {
			if (subSceneOf[w] != subSceneOf[21]) {
				VERBOSE("SceneClusterStandaloneCommunityTest FAILED: W was split, image %u in sub-scene %d and 21 in %d", w, subSceneOf[w], subSceneOf[21]);
				return false;
			}
		}
		if (withSeamTracks) {
			if (subSceneOf[21] < 0 || subSceneOf[21] == subSceneOf[10] || subSceneOf[21] == subSceneOf[0]) {
				VERBOSE("SceneClusterStandaloneCommunityTest FAILED: W landed in sub-scene %d (B's hub in %d, A's in %d) although no neighbour is coupled to it and its seam to B is strong; it must stand alone", subSceneOf[21], subSceneOf[10], subSceneOf[0]);
				return false;
			}
		} else if (subSceneOf[21] != subSceneOf[10]) {
			VERBOSE("SceneClusterStandaloneCommunityTest FAILED: W landed in sub-scene %d while B's hub is in %d; with no seam tracks the merge could not place it, so it goes to the neighbour it shares the most weight with", subSceneOf[21], subSceneOf[10]);
			return false;
		}
	}

	VERBOSE("SceneClusterStandaloneCommunityTest PASSED: a community no neighbour is coupled to stands alone when it has a strong seam, and is absorbed when it has none (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// A and B as in the standalone-community fixture but both filled to the cap with no slack
// (maxOverCapacity 0): A = [0,11) around hub 0, B = [11,22) around hub 11. W = [22,26) is joined to
// one another by six pairs of weight 50 (3000 in the graph's integer weights, so the hub/chain
// edges that assemble A and B settle first) and to B by two pairs of weight 30 (600, well past
// minClusterCoupling of W's own cohesion: solidly coupled, not a standalone community), with 80
// tracks seen by W's 22, 23, 24 and B's 11, 12, 13 giving that pair a strong seam by the merge's own
// bars. Neither the small-cluster pass nor the orphan rescue can place W (B is already at the cap),
// so W must come out of the split as no sub-scene at all -- it used to come out as a third one
// whenever B's cluster had already been extracted by the time the sub-scene builder looked at W
// again, because the coupling test it re-ran then no longer saw the interface to B and read the
// same strong seam as if no neighbour were coupled to it.
bool SceneClusterCoupledOrphanSkippedTest()
{
	TD_TIMER_START();

	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 26;
	cfg.numPoints = 0;
	cfg.generatePairs = false;
	cfg.generateDescriptors = false;
	cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	cfg.rotationAngleStep = 360.0 / 26;
	GenerateTestScene(scene, cfg);
	scene.pairs.clear();
	scene.tracks.clear();

	// a pair whose composite weight is exactly the given number: unit quality factors, triplet 0.75 (0.25 + 0.75 = 1)
	const auto AddPair = [&scene](IIndex a, IIndex b, float weight) {
		ImagePair pair(a, b);
		pair.weightedInliers = weight;
		pair.weightSpatial = 1.f;
		pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
		pair.weightConnectivity = 1.f;
		pair.weightTriplet = 0.75f;
		scene.pairs.emplace_back(std::move(pair));
	};
	// A = [0,11): a hub at 0 and a chain, so the greedy assembles it one image at a time
	for (IIndex k = 1; k < 11; ++k) AddPair(0, k, 200.f);
	for (IIndex k = 1; k + 1 < 11; ++k) AddPair(k, k + 1, 100.f);
	// B = [11,22): the same shape around a hub at 11
	for (IIndex k = 12; k < 22; ++k) AddPair(11, k, 200.f);
	for (IIndex k = 12; k + 1 < 22; ++k) AddPair(k, k + 1, 100.f);
	// W = [22,26): every pair among them, and two pairs into B lighter than the hub/chain edges so
	// A and B assemble first, but well above the coupling floor
	for (IIndex a = 22; a < 26; ++a) for (IIndex b = a + 1; b < 26; ++b) AddPair(a, b, 50.f);
	AddPair(22, 11, 30.f);
	AddPair(23, 12, 30.f);

	// 80 tracks each seen by W's 22, 23, 24 and B's 11, 12, 13: seam-usable on both sides, well past
	// the merge's strong-seam bars (75 tracks, 3 cameras a side with >= 30)
	for (uint32_t t = 0; t < 80; ++t) {
		Track& track = scene.tracks.emplace_back();
		for (uint32_t img : {22u, 23u, 24u, 11u, 12u, 13u})
			track.observations.emplace_back(img, t);
		track.numInliers = (uint8_t)track.observations.size();
	}

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 11;
	clusterCfg.targetViewsPerCluster = 10;
	clusterCfg.minViewsPerCluster = 5;
	clusterCfg.maxOverCapacity = 0;

	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	std::vector<int> subSceneOf(26, -1);
	FOREACH(s, localToGlobals)
		for (IIndex gid : localToGlobals[s])
			subSceneOf[gid] = (int)s;

	if (subSceneOf[0] < 0 || subSceneOf[11] < 0 || subSceneOf[0] == subSceneOf[11]) {
		VERBOSE("SceneClusterCoupledOrphanSkippedTest FAILED: A and B must be two sub-scenes (%d, %d), the fixture does not exercise the floor rule otherwise", subSceneOf[0], subSceneOf[11]);
		return false;
	}
	const int wScene = subSceneOf[22];
	for (IIndex w = 23; w < 26; ++w) {
		if (subSceneOf[w] != wScene) {
			VERBOSE("SceneClusterCoupledOrphanSkippedTest FAILED: W was split, image %u in sub-scene %d and 22 in %d", w, subSceneOf[w], wScene);
			return false;
		}
	}
	if (wScene != -1) {
		VERBOSE("SceneClusterCoupledOrphanSkippedTest FAILED: W came out as sub-scene %d although it is coupled to B (sub-scene %d) and B has no room for it; a coupled cluster under the floor must be skipped, not mistaken for a standalone community",
			wScene, subSceneOf[11]);
		return false;
	}

	VERBOSE("SceneClusterCoupledOrphanSkippedTest PASSED: a coupled cluster under the floor that no neighbour can take is skipped whatever its place in the order (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// The balance pass (RefineClustersBalance) never takes an image whose departure leaves part of its
// cluster hanging under the coupling floor: X holds a 10-image body, a 5-image stretch and the one
// image bridging them, which leans toward the smaller cluster Y (affinity 1.5, past the pass's 0.7);
// moving it would leave the stretch on the body by a pair of weight 4 against its own 550. It stays,
// and so does the stretch, where moving the bridge would strand it.
bool SceneClusterBalanceCouplingTest()
{
	TD_TIMER_START();

	// images: body 0-9, bridge 10, stretch 11-15 (cluster X, 16 images); Y 16-23 (8 images)
	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 24;
	cfg.numPoints = 0;
	cfg.generatePairs = false;
	cfg.generateDescriptors = false;
	cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	cfg.rotationAngleStep = 360.0 / 24;
	GenerateTestScene(scene, cfg);
	scene.pairs.clear();
	scene.tracks.clear();

	// a pair whose composite weight is exactly the given number: unit quality factors, triplet 0.75 (0.25 + 0.75 = 1)
	const auto AddPair = [&scene](IIndex a, IIndex b, float weight) {
		ImagePair pair(a, b);
		pair.weightedInliers = weight;
		pair.weightSpatial = 1.f;
		pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
		pair.weightConnectivity = 1.f;
		pair.weightTriplet = 0.75f;
		scene.pairs.emplace_back(std::move(pair));
	};
	const auto AddChain = [&](IIndex first, IIndex last) {
		for (IIndex k = first; k < last; ++k) AddPair(k, k + 1, 100.f);
		for (IIndex k = first; k + 1 < last; ++k) AddPair(k, k + 2, 50.f);
	};
	AddChain(0, 9);                     // the body
	AddChain(11, 15);                   // the stretch: internal weight 4 x 100 + 3 x 50 = 550
	AddChain(16, 23);                   // Y
	AddPair(9, 10, 100.f); AddPair(8, 10, 50.f);    // the bridge holds the body: 150
	AddPair(10, 11, 100.f); AddPair(10, 12, 50.f);  // and the stretch: 150
	AddPair(9, 11, 4.f);                            // the stretch's own link to the body, under the floor
	AddPair(10, 16, 300.f); AddPair(10, 17, 150.f); // and leans toward Y: 450 against its 300 in X

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 20;
	clusterCfg.targetViewsPerCluster = 16;
	clusterCfg.minViewsPerCluster = 4;
	clusterCfg.minWaistSideViews = 3;
	clusterCfg.minClusterDegree = 0;
	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> clusters(2);
	for (IIndex i = 0; i <= 15; ++i) clusters[0].push_back(i);
	for (IIndex i = 16; i <= 23; ++i) clusters[1].push_back(i);
	cluster.RefineClustersBalance(clusters);

	const auto Holds = [&](size_t c, IIndex i) { return std::find(clusters[c].begin(), clusters[c].end(), i) != clusters[c].end(); };
	for (IIndex i = 0; i <= 15; ++i) {
		if (!Holds(0, i)) {
			VERBOSE("SceneClusterBalanceCouplingTest FAILED: image %u left X, %s", i,
				i == 10 ? "the bridge whose departure strands the stretch under the floor" : "an image the fixture never offers Y");
			return false;
		}
	}
	VERBOSE("SceneClusterBalanceCouplingTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// The thin-waist check (RefineClustersSplitThinWaist) weighs every prefix cut along the Fiedler order:
// a 4-image stretch hanging on its 12-image cluster C by a pair of weight 5 (coupling 0.0125), too
// small for a sub-scene of its own and heavily coupled to the cluster D (250 against its own 400), moves
// whole to D; with no neighbour holding it at the floor it stays where it is; and an 8-image side
// under-coupled to its cluster, large enough to stand, is split off as before
bool SceneClusterThinStretchTest()
{
	TD_TIMER_START();

	struct Variant { const char* name; unsigned stretch; bool linkedToD; };
	for (const Variant& variant : {Variant{"a stretch D holds", 4, true}, Variant{"a stretch nothing holds", 4, false}, Variant{"a side that can stand", 8, true}}) {
		// C: body 0-11 and the stretch 12..; D: the 12 images after it
		const IIndex firstD = 12 + variant.stretch, numImages = firstD + 12;
		Scene scene;
		SceneConfig cfg;
		cfg.numImages = numImages;
		cfg.numPoints = 0;
		cfg.generatePairs = false;
		cfg.generateDescriptors = false;
		cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
		cfg.rotationAngleStep = 360.0 / numImages;
		GenerateTestScene(scene, cfg);
		scene.pairs.clear();
		scene.tracks.clear();

		// a pair whose composite weight is exactly the given number: unit quality factors, triplet 0.75 (0.25 + 0.75 = 1)
		const auto AddPair = [&scene](IIndex a, IIndex b, float weight) {
			ImagePair pair(a, b);
			pair.weightedInliers = weight;
			pair.weightSpatial = 1.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 1.f;
			pair.weightTriplet = 0.75f;
			scene.pairs.emplace_back(std::move(pair));
		};
		const auto AddChain = [&](IIndex first, IIndex last) {
			for (IIndex k = first; k < last; ++k) AddPair(k, k + 1, 100.f);
			for (IIndex k = first; k + 1 < last; ++k) AddPair(k, k + 2, 50.f);
		};
		AddChain(0, 11);
		AddChain(12, firstD - 1);
		AddChain(firstD, numImages - 1);
		AddPair(11, 12, 5.f); // the stretch's only link to its body
		if (variant.linkedToD) {
			AddPair(12, firstD, 100.f); AddPair(13, firstD + 1, 100.f); AddPair(14, firstD + 2, 50.f);
		}

		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = 24;
		clusterCfg.minViewsPerCluster = 6;
		clusterCfg.maxOverCapacity = 5;
		clusterCfg.minWaistSideViews = 3;
		clusterCfg.minClusterDegree = 0;
		SceneCluster cluster(scene, clusterCfg);
		std::vector<IIndexArr> clusters(2);
		for (IIndex i = 0; i < firstD; ++i) clusters[0].push_back(i);
		for (IIndex i = firstD; i < numImages; ++i) clusters[1].push_back(i);
		cluster.RefineClustersSplitThinWaist(clusters);

		// the cluster holding image 12 (the stretch's first), image 0 (the body) and image firstD (D)
		const auto ClusterOf = [&](IIndex img) {
			for (size_t ci = 0; ci < clusters.size(); ++ci)
				if (std::find(clusters[ci].begin(), clusters[ci].end(), img) != clusters[ci].end())
					return (int)ci;
			return -1;
		};
		unsigned numClusters = 0;
		for (const IIndexArr& c : clusters)
			if (!c.empty()) ++numClusters;
		const int cBody = ClusterOf(0), cStretch = ClusterOf(12), cD = ClusterOf(firstD);
		bool ok = true;
		for (IIndex i = 12; i < firstD; ++i)
			if (ClusterOf(i) != cStretch)
				ok = false; // the stretch split instead of moving or staying whole
		if (ok) {
			if (variant.stretch >= clusterCfg.minViewsPerCluster)
				// large enough to stand on its own: split off as its own cluster
				ok = numClusters == 3 && cBody != cStretch && cStretch != cD && cBody != cD;
			else if (variant.linkedToD)
				// too small to stand, held more by D than by its own cluster: moves whole to D
				ok = numClusters == 2 && cStretch == cD && cBody != cStretch;
			else
				// too small to stand and no neighbour holds it better: stays in place
				ok = numClusters == 2 && cStretch == cBody && cBody != cD;
		}
		if (!ok) {
			VERBOSE("SceneClusterThinStretchTest FAILED: %s, %u clusters, image 0 in %d, image 12 in %d, image %u in %d",
				variant.name, numClusters, cBody, cStretch, firstD, cD);
			return false;
		}
	}

	VERBOSE("SceneClusterThinStretchTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// ===============================================================================
// Clustering for the merge: cluster sizes aimed at it, and cuts whose seams it can read
// ===============================================================================

// A chain of images that see the same points as their neighbours: images 0..numImages-1, with
// `tracksPerStart` tracks starting at every image i and observed by the images i..i+lengthOf(i)-1
// (clipped at the last image; no track starts at i when lengthOf(i) is 0), a pair between every two
// images a track joins, matched through those very tracks. The poses and the keypoint positions are
// dummies: clustering reads the pairs and the tracks alone.
static void BuildChainScene(unsigned numImages, unsigned tracksPerStart,
	const std::function<unsigned(unsigned)>& lengthOf, Scene& scene)
{
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), REAL(400), REAL(400), REAL(320), REAL(240)));
	for (unsigned i = 0; i < numImages; ++i)
		scene.images.emplace_back((IIndex)i, String(), Pose3D(), 0, scene.cameras[0]);
	scene.status.nCalibratedImages = scene.images.size();

	// the tracks, one keypoint per observation, spread over the frame so that every pair carries the
	// same coverage into the weighting
	std::mt19937 rng(11);
	std::uniform_real_distribution<REAL> xDist(10, 630), yDist(10, 470);
	for (unsigned start = 0; start < numImages; ++start) {
		const unsigned length = lengthOf(start);
		if (length == 0)
			continue;
		const unsigned last = MINF(start + length, numImages);
		for (unsigned t = 0; t < tracksPerStart; ++t) {
			Track track(Point3(0, 0, 1));
			for (unsigned i = start; i < last; ++i) {
				Image& img = scene.images[i];
				track.observations.emplace_back(i, (uint32_t)img.keypoints.size());
				img.keypoints.emplace_back(Point2(xDist(rng), yDist(rng)), 0.f, 0.f, 10.f);
			}
			track.numInliers = (uint8_t)MINF(track.observations.size(), (IIndex)UINT8_MAX);
			scene.tracks.emplace_back(std::move(track));
		}
	}

	// a pair per image couple the tracks join, matched through those tracks
	std::unordered_map<PairIdx::PairIndex, uint32_t> pairOf;
	for (const Track& track : scene.tracks) {
		FOREACH(i, track.observations) {
			for (IIndex j = i + 1; j < track.observations.size(); ++j) {
				const Observation& first = track.observations[i];
				const Observation& second = track.observations[j];
				const PairIdx images(first.imageID, second.imageID);
				const auto ret = pairOf.emplace(images.idx, (uint32_t)scene.pairs.size());
				if (ret.second)
					scene.pairs.emplace_back(images.i, images.j);
				scene.pairs[ret.first->second].matches.emplace_back(first.featureID, second.featureID);
			}
		}
	}
	scene.status.nTracks = scene.tracks.size();
	scene.status.nState.set(Scene::Status::STATE::FEATURES_EXTRACTED);
	scene.status.nState.set(Scene::Status::STATE::MATCHED);
	ComputePairsWeights(scene);
}

// Four hundred images walking a whole ring: the sub-scenes must come out sized for the merge -- none
// under the floor, none over the ceiling and its slack, half of them around the target -- and every
// two that touch must share a seam the merge can register
bool SceneClusterTargetSizeTest()
{
	TD_TIMER_START();

	const RingSceneConfig cfg{40, 10};
	Scene scene;
	std::vector<IIndexArr> blocks;
	std::vector<Pose3D> gtPoses;
	GenerateRingScene(cfg, scene, blocks, gtPoses);
	const unsigned numImages = (unsigned)scene.images.size();

	const ClusterConfig clusterCfg;
	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	const std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	if (subScenes.size() < 3) {
		VERBOSE("SceneClusterTargetSizeTest FAILED: expected >= 3 sub-scenes, got %u", (unsigned)subScenes.size());
		return false;
	}
	std::vector<REAL> sizes;
	for (unsigned s = 0; s < subScenes.size(); ++s) {
		const unsigned size = (unsigned)subScenes[s].images.size();
		if (size < clusterCfg.minViewsPerCluster || size > clusterCfg.maxViewsPerCluster + clusterCfg.maxOverCapacity) {
			VERBOSE("SceneClusterTargetSizeTest FAILED: sub-scene %u has %u images, outside [%u, %u]",
				s, size, clusterCfg.minViewsPerCluster, clusterCfg.maxViewsPerCluster + clusterCfg.maxOverCapacity);
			return false;
		}
		sizes.push_back((REAL)size);
	}
	const REAL median = Median(sizes);
	const REAL medianLow = (REAL)clusterCfg.targetViewsPerCluster * REAL(0.7);
	const REAL medianHigh = (REAL)clusterCfg.targetViewsPerCluster * REAL(1.3);
	if (median < medianLow || median > medianHigh) {
		VERBOSE("SceneClusterTargetSizeTest FAILED: median sub-scene size %.0f outside [%.0f, %.0f]", median, medianLow, medianHigh);
		return false;
	}

	// every image in exactly one sub-scene
	std::vector<unsigned> numAssignments(numImages, 0);
	for (const IIndexArr& localToGlobal : localToGlobals)
		for (IIndex globalID : localToGlobal)
			++numAssignments[globalID];
	for (unsigned i = 0; i < numImages; ++i) {
		if (numAssignments[i] != 1) {
			VERBOSE("SceneClusterTargetSizeTest FAILED: image %u assigned to %u sub-scenes", i, numAssignments[i]);
			return false;
		}
	}

	// every seam between two sub-scenes usable by the merge, by the rule the clustering applies
	const SeamTrackStatsMap seams = cluster.ComputeSeamTrackStats(localToGlobals);
	for (const auto& [pair, seam] : seams) {
		if (!IsStrongSeam(seam, clusterCfg)) {
			VERBOSE("SceneClusterTargetSizeTest FAILED: sub-scenes (%u, %u) share %u seam-usable tracks over %u/%u cameras",
				pair.first, pair.second, seam.first.usable,
				seam.first.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks),
				seam.second.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks));
			return false;
		}
	}

	VERBOSE("SceneClusterTargetSizeTest PASSED: %u images in %u sub-scenes, median %.0f images, %u seams (%s)",
		numImages, (unsigned)subScenes.size(), median, (unsigned)seams.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

// The seam-usable tracks of a cut, counted as the merge counts them: a track one side triangulates
// and the other sees, credited to every camera that observed it
bool SceneClusterSeamTracksTest()
{
	TD_TIMER_START();

	const ClusterConfig clusterCfg;
	std::vector<IIndexArr> clusters(2);
	for (IIndex i = 0; i < 12; ++i)
		clusters[i < 6 ? 0 : 1].push_back(i);

	// three-view tracks: only the two starting at 4 and 5 cross the cut between 5 and 6
	{
		Scene scene;
		BuildChainScene(12, 20, [](unsigned i) { return i <= 9 ? 3u : 0u; }, scene);
		SceneCluster cluster(scene, clusterCfg);
		const SeamTrackStatsMap seams = cluster.ComputeSeamTrackStats(clusters);
		if (seams.size() != 1) {
			VERBOSE("SceneClusterSeamTracksTest FAILED: expected one seam, got %u", (unsigned)seams.size());
			return false;
		}
		const std::pair<SeamTrackStats, SeamTrackStats>& seam = seams.begin()->second;
		if (seam.first.usable != 40 || seam.second.usable != 40) {
			VERBOSE("SceneClusterSeamTracksTest FAILED: %u/%u seam-usable tracks, expected 40",
				seam.first.usable, seam.second.usable);
			return false;
		}
		const std::pair<IIndex, unsigned> expected[4] = {{5, 40}, {4, 20}, {6, 40}, {7, 20}};
		for (unsigned k = 0; k < 4; ++k) {
			const SeamTrackStats& stats = k < 2 ? seam.first : seam.second;
			const auto it = stats.perCamera.find(expected[k].first);
			if (stats.perCamera.size() != 2 || it == stats.perCamera.end() || it->second != expected[k].second) {
				VERBOSE("SceneClusterSeamTracksTest FAILED: image %u carries %u seam-usable tracks of %u cameras, expected %u",
					expected[k].first, it == stats.perCamera.end() ? 0 : it->second,
					(unsigned)stats.perCamera.size(), expected[k].second);
				return false;
			}
		}
		if (seam.first.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks) != 1 ||
			seam.second.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks) != 1) {
			VERBOSE("SceneClusterSeamTracksTest FAILED: %u/%u cameras carrying minSeamCameraTracks, expected 1/1",
				seam.first.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks),
				seam.second.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks));
			return false;
		}
	}

	// five-view tracks reach across the cut from further away, and the seam spreads over the cameras
	// the merge needs
	{
		Scene scene;
		BuildChainScene(12, 20, [](unsigned i) { return i <= 7 ? 5u : 0u; }, scene);
		SceneCluster cluster(scene, clusterCfg);
		const SeamTrackStatsMap seams = cluster.ComputeSeamTrackStats(clusters);
		if (seams.size() != 1) {
			VERBOSE("SceneClusterSeamTracksTest FAILED: expected one seam, got %u", (unsigned)seams.size());
			return false;
		}
		const std::pair<SeamTrackStats, SeamTrackStats>& seam = seams.begin()->second;
		const unsigned cameras[2] = {seam.first.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks),
			seam.second.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks)};
		if (cameras[0] < clusterCfg.minSeamCameras || cameras[1] < clusterCfg.minSeamCameras) {
			VERBOSE("SceneClusterSeamTracksTest FAILED: %u/%u cameras carrying minSeamCameraTracks, expected >= %u each",
				cameras[0], cameras[1], clusterCfg.minSeamCameras);
			return false;
		}
	}

	VERBOSE("SceneClusterSeamTracksTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// A tail joined to the rest of the capture by a handful of short tracks: it is big enough to keep,
// but no seam of it can be registered, so it must not come out as a sub-scene of its own
bool SceneClusterLeafMergeTest()
{
	TD_TIMER_START();

	// images 0..259 a chain of five-view tracks, images 260..304 a tail of the same, and between
	// them only the three-view tracks starting at 255..259 -- two of which cross the cut
	Scene scene;
	BuildChainScene(305, 20, [](unsigned i) { return i >= 255 && i <= 259 ? 3u : 5u; }, scene);

	const ClusterConfig clusterCfg;
	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	const std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);

	for (unsigned s = 0; s < subScenes.size(); ++s) {
		if (subScenes[s].images.size() < clusterCfg.minViewsPerCluster) {
			VERBOSE("SceneClusterLeafMergeTest FAILED: sub-scene %u has %u images < %u",
				s, (unsigned)subScenes[s].images.size(), clusterCfg.minViewsPerCluster);
			return false;
		}
	}

	// the tail sits with the end of the chain, and that sub-scene reaches back into the chain
	// proper: a tail left to itself is the block the merge would have to leave unplaced
	unsigned subSceneOfEnd = NO_ID;
	for (unsigned s = 0; s < localToGlobals.size(); ++s)
		for (IIndex globalID : localToGlobals[s])
			if (globalID == 259)
				subSceneOfEnd = s;
	if (subSceneOfEnd == NO_ID) {
		VERBOSE("SceneClusterLeafMergeTest FAILED: image 259 left out of the sub-scenes");
		return false;
	}
	std::set<IIndex> withEnd(localToGlobals[subSceneOfEnd].begin(), localToGlobals[subSceneOfEnd].end());
	for (IIndex globalID = 260; globalID < 305; ++globalID) {
		if (withEnd.count(globalID) == 0) {
			VERBOSE("SceneClusterLeafMergeTest FAILED: image %u is not in the sub-scene of image 259 (%u images)",
				globalID, (unsigned)withEnd.size());
			return false;
		}
	}
	if (withEnd.count(250) == 0) {
		VERBOSE("SceneClusterLeafMergeTest FAILED: the tail and image 259 are a sub-scene of their own (%u images)",
			(unsigned)withEnd.size());
		return false;
	}

	VERBOSE("SceneClusterLeafMergeTest PASSED: %u sub-scenes, the tail merged into a %u-image one (%s)",
		(unsigned)subScenes.size(), (unsigned)withEnd.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

} // namespace SFM
