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

// The clustering of a view graph into sub-scenes (SceneCluster::Partition, ExtractBlocks), on generated scenes:
// (1) a scene within the view limit, and one with the limit disabled (0), stay a single sub-scene holding every image;
// (2) the size constraints on two clusters of 15 images: no sub-scene past the maximum and its slack, no image in two
//     sub-scenes and at most three dropped as undersized;
// (3) two groups with no pair between them (and no track across) never share a sub-scene;
// (4) on two clusters of 12, the memory protocol (the keypoints of the assigned images moved to the sub-scenes, only the
//     cross-sub-scene pairs left in the global scene, the pairs conserved) and the ID remapping of the same sub-scenes
//     (the local to global maps hold valid IDs once, the tracks' observations and the pairs of a sub-scene index its own
//     images and keypoints, pairs ordered).
bool SceneClusterBasicsTest()
{
	TestScope scope("SceneClusterBasicsTest");

	// 1. a single sub-scene: within the limit, and with the limit off
	for (const unsigned maxViews : {10u, 0u}) {
		Scene scene;
		SceneConfig cfg;
		cfg.numImages = 8;
		cfg.numPoints = 60;
		cfg.generatePairs = true;
		cfg.generateDescriptors = true;
		GenerateTestScene(scene, cfg);
		ComputePairsWeights(scene);
		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = maxViews; // 8 <= 10: no split; 0: clustering disabled
		SceneCluster cluster(scene, clusterCfg);
		const std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition());
		TEST_CHECK(subScenes.size() == 1, "maxViewsPerCluster %u: expected 1 sub-scene, got %u", maxViews, (unsigned)subScenes.size());
		TEST_CHECK(subScenes[0].images.size() == 8, "maxViewsPerCluster %u: expected 8 images, got %u", maxViews, (unsigned)subScenes[0].images.size());
	}

	// 2. size constraints and coverage
	{
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
		const std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);
		TEST_CHECK(subScenes.size() >= 2, "size constraints: expected >= 2 sub-scenes, got %u", (unsigned)subScenes.size());
		for (unsigned s = 0; s < subScenes.size(); ++s)
			TEST_CHECK(subScenes[s].images.size() <= clusterCfg.maxViewsPerCluster + clusterCfg.maxOverCapacity,
				"size constraints: sub-scene %u has %u images > max %u", s, (unsigned)subScenes[s].images.size(),
				clusterCfg.maxViewsPerCluster + clusterCfg.maxOverCapacity);
		std::vector<int> imageCounts(totalImages, 0);
		for (const IIndexArr& localToGlobal : localToGlobals)
			for (IIndex globalID : localToGlobal)
				if (globalID < totalImages)
					++imageCounts[globalID];
		unsigned missingImages = 0;
		for (unsigned i = 0; i < totalImages; ++i) {
			TEST_CHECK(imageCounts[i] <= 1, "size constraints: image %u in %d sub-scenes", i, imageCounts[i]);
			missingImages += imageCounts[i] == 0;
		}
		TEST_CHECK(missingImages <= 3, "size constraints: %u missing images, at most 3 may be dropped as undersized", missingImages);
	}

	// 3. disconnected components get split into separate clusters: 20 images in two groups with no pair between them
	{
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
		RFOREACH(i, scene.pairs)
			if ((scene.pairs[i].ID1 < 10) != (scene.pairs[i].ID2 < 10))
				scene.pairs.RemoveAtMove(i); // across the groups A (0-9) and B (10-19)
		// the two groups were never seen together, so no track may span them either: the split rebuilds
		// them from the pairs that remain
		scene.tracks.Release();
		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = 15; // smaller than 20 to force the split
		clusterCfg.minViewsPerCluster = 5;
		SceneCluster cluster(scene, clusterCfg);
		std::vector<IIndexArr> localToGlobals;
		const std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);
		TEST_CHECK(subScenes.size() >= 2, "disconnected groups: expected >= 2 sub-scenes, got %u", (unsigned)subScenes.size());
		for (unsigned s = 0; s < localToGlobals.size(); ++s) {
			bool hasFirst = false, hasSecond = false;
			for (IIndex gid : localToGlobals[s])
				(gid < 10 ? hasFirst : hasSecond) = true;
			TEST_CHECK(!(hasFirst && hasSecond), "disconnected groups: sub-scene %u mixes the groups", s);
		}
	}

	// 4. memory protocol (keypoints MOVED, cross-pairs LEFT) and ID remapping, on two clusters of 12
	Scene scene;
	GenerateTwoClusterScene(scene, 12, 12, 4, 40, 80);
	const unsigned totalImages = (unsigned)scene.images.size();
	const unsigned origPairCount = (unsigned)scene.pairs.size();
	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 14;
	clusterCfg.minViewsPerCluster = 5;
	// the sub-scenes are this fixture, and its handful of points makes every seam look thin to a rule
	// written for a capture's tracks: no cluster is merged away here for want of strong neighbours
	clusterCfg.minClusterDegree = 0;
	SceneCluster cluster(scene, clusterCfg);
	std::vector<IIndexArr> localToGlobals;
	const std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);
	TEST_CHECK(subScenes.size() >= 2, "memory protocol: expected >= 2 sub-scenes, got %u", (unsigned)subScenes.size());
	std::vector<int> subSceneOf(totalImages, -1);
	FOREACH(s, localToGlobals)
		for (const IIndex gid : localToGlobals[s]) {
			TEST_CHECK(gid < totalImages, "ID remapping: invalid global ID %u in sub-scene %u", gid, (unsigned)s);
			TEST_CHECK(subSceneOf[gid] < 0, "ID remapping: global ID %u in multiple sub-scenes", gid);
			subSceneOf[gid] = (int)s;
			TEST_CHECK(scene.images[gid].keypoints.empty(), "memory protocol: global image %u still has keypoints after the split", gid);
		}
	for (unsigned s = 0; s < subScenes.size(); ++s) {
		const Scene& sub = subScenes[s];
		for (const Image& img : sub.images)
			TEST_CHECK(!img.keypoints.empty(), "memory protocol: sub-scene %u has an image with empty keypoints", s);
		for (const Track& track : sub.tracks)
			for (const Observation& obs : track) {
				TEST_CHECK(obs.imageID < sub.images.size(), "ID remapping: track obs imageID %u >= %u in sub-scene %u", obs.imageID, (unsigned)sub.images.size(), s);
				TEST_CHECK(obs.featureID < sub.images[obs.imageID].keypoints.size(), "ID remapping: track obs featureID %u >= %u in sub-scene %u",
					obs.featureID, (unsigned)sub.images[obs.imageID].keypoints.size(), s);
			}
		for (const ImagePair& pair : sub.pairs) {
			TEST_CHECK(pair.ID1 < sub.images.size() && pair.ID2 < sub.images.size(), "ID remapping: pair (%u,%u) exceeds image count %u in sub-scene %u",
				pair.ID1, pair.ID2, (unsigned)sub.images.size(), s);
			TEST_CHECK(pair.ID1 < pair.ID2, "ID remapping: pair (%u,%u) not ordered in sub-scene %u", pair.ID1, pair.ID2, s);
		}
	}
	// the global scene retains only the cross-sub-scene pairs, and no pair is lost
	unsigned subScenePairCount = 0, globalPairCount = 0;
	for (const Scene& sub : subScenes)
		subScenePairCount += (unsigned)sub.pairs.size();
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.HasMatches())
			continue;
		++globalPairCount;
		TEST_CHECK(subSceneOf[pair.ID1] != subSceneOf[pair.ID2] || subSceneOf[pair.ID1] < 0, "memory protocol: intra-cluster pair (%u,%u) remains in the global scene", pair.ID1, pair.ID2);
	}
	TEST_CHECK(subScenePairCount + globalPairCount == origPairCount, "memory protocol: pair count mismatch: %u + %u != %u", subScenePairCount, globalPairCount, origPairCount);
	return scope.Pass("single sub-scene, size constraints, disconnected groups; memory protocol and ID remapping on %u sub-scenes, %u cross-pairs left", (unsigned)subScenes.size(), globalPairCount);
}
/*----------------------------------------------------------------*/

// The small-cluster pass of SceneCluster::Partition, on view graphs of two established clusters A and B and a small community W.
// A and B are built as a hub and a chain (so the greedy assembles them one image at a time), W as a clique of weight 50 (3000 in
// the graph's integer weights, so the hub/chain edges settle first); cluster bounds 15/10/5 with 5 over capacity unless said.
// 1. rescue: on a generated 25-image scene with its computed weights, A = [0,12) and B = [12,22) strong, the three images of W
//    joined weakly to A and to one another: no sub-scene under the minimum, every image of W assigned;
// 2. sub-floor coupling: A 10 images at the target, B 11 (one past it), W joined to B by four pairs of 20 and to A by one of 4:
//    the greedy cannot give W to B (past the target) and used to give it to A over that one pair because the coupling test
//    skipped a side under the floor; the interface to A, 40, is far under minClusterCoupling of W's 3000, so W stays apart and
//    the pass puts it with B, the neighbour it shares the most weight with;
// 3. standalone community: W joined to B by one pair of 6 and to A by one of 4, 2% of its cohesion, so no neighbour is coupled
//    to it: with 80 tracks seen by three W and three B cameras (a strong seam by the merge's bars: 75 tracks, 3 cameras a side
//    with 30) it is a sub-scene of its own, with none it goes to B;
// 4. coupled orphan: A [0,11) and B [11,22) at the cap with no slack (11 views, 0 over capacity), W [22,26) coupled to B by two
//    pairs of 30 (600, well past minClusterCoupling) with the same strong seam: neither the pass nor the orphan rescue can place
//    it, so W is no sub-scene at all -- it used to come out as a third whenever B had been extracted before the sub-scene builder
//    looked at W again, the coupling test then no longer seeing the interface to B.
bool SceneClusterSmallClusterPassTest()
{
	TestScope scope("SceneClusterSmallClusterPassTest");
	const auto Bounds = [](unsigned maxViews, unsigned overCapacity, bool everyCluster) {
		ClusterConfig cfg;
		cfg.maxViewsPerCluster = maxViews;
		cfg.targetViewsPerCluster = 10;
		cfg.minViewsPerCluster = 5;
		cfg.maxOverCapacity = overCapacity;
		if (everyCluster)
			cfg.minClusterDegree = 0; // no tracks, so the seam rules would merge every leaf away
		return cfg;
	};
	// per image, the sub-scene the partition put it in (-1: none)
	const auto SubSceneOf = [](Scene& scene, const ClusterConfig& cfg) {
		std::vector<IIndexArr> localToGlobals;
		ExtractBlocks(scene, SceneCluster(scene, cfg).Partition(), &localToGlobals);
		std::vector<int> subSceneOf(scene.images.size(), -1);
		FOREACH(s, localToGlobals)
			for (IIndex gid : localToGlobals[s])
				subSceneOf[gid] = (int)s;
		return subSceneOf;
	};
	// A = [0, endA] around hub 0, B = [endA+1, endB] around its first image
	const auto TwoClusters = [](PairGraph& g, IIndex endA, IIndex endB) {
		g.Hub(0, endA, 200); g.Chain(1, endA, 100);
		g.Hub(endA + 1, endB, 200); g.Chain(endA + 2, endB, 100);
	};
	// 80 tracks each seen by the given six images (Partition builds tracks only when the scene has none, so these are the seam's)
	const auto SeamTracks = [](Scene& scene, std::initializer_list<uint32_t> images) {
		for (uint32_t t = 0; t < 80; ++t) {
			Track& track = scene.tracks.emplace_back();
			for (uint32_t img : images)
				track.observations.emplace_back(img, t);
			track.numInliers = (uint8_t)track.observations.size();
		}
	};

	// 1. rescue
	{
		Scene scene;
		SceneConfig cfg;
		cfg.numImages = 25;
		cfg.numPoints = 100;
		cfg.rotationAngleStep = 360.0 / 25;
		cfg.generateDescriptors = true;
		cfg.generatePairs = true;
		GenerateTestScene(scene, cfg);
		ComputePairsWeights(scene);
		const auto Group = [](IIndex id) { return id < 12 ? 0 : id < 22 ? 1 : 2; }; // A, B, W
		RFOREACH(i, scene.pairs) {
			ImagePair& pair = scene.pairs[i];
			const int g1 = Group(pair.ID1), g2 = Group(pair.ID2);
			const float weight = g1 == g2 ? (g1 == 2 ? 1.f : 10.f) : (g1 == 2 || g2 == 2) && (g1 == 0 || g2 == 0) ? 2.f : 0.f;
			if (weight == 0.f) {
				scene.pairs.RemoveAtMove(i); // B to W and A to B
				continue;
			}
			pair.weightSpatial = pair.weightConnectivity = weight;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightTriplet = weight == 10.f ? 10.f : 0.f;
		}
		const std::vector<int> subSceneOf = SubSceneOf(scene, Bounds(15, 5, false));
		std::map<int, unsigned> sizes;
		for (int sub : subSceneOf)
			++sizes[sub];
		for (const auto& [sub, size] : sizes)
			TEST_CHECK(sub < 0 || size >= 5, "rescue: sub-scene %d has %u images, under the minimum of 5", sub, size);
		TEST_CHECK(subSceneOf[22] >= 0 && subSceneOf[23] >= 0 && subSceneOf[24] >= 0, "rescue: the weak images were not all assigned "
			"(sub-scenes %d, %d, %d)", subSceneOf[22], subSceneOf[23], subSceneOf[24]);
	}

	// 2. sub-floor coupling
	{
		Scene scene;
		PairGraph g(scene, 25);
		TwoClusters(g, 9, 20);
		g.Clique(21, 24, 50);
		for (IIndex k = 0; k < 4; ++k)
			g.Pair(21 + k, 10 + k, 20);
		g.Pair(21, 9, 4);
		const std::vector<int> subSceneOf = SubSceneOf(scene, Bounds(15, 5, true));
		TEST_CHECK(subSceneOf[0] != subSceneOf[10], "sub-floor coupling: A and B ended in one sub-scene, the fixture does not exercise the merge");
		for (IIndex w = 21; w < 25; ++w)
			TEST_CHECK(subSceneOf[w] >= 0 && subSceneOf[w] == subSceneOf[10], "sub-floor coupling: image %u landed in sub-scene %d while B's "
				"hub is in %d and A's in %d; a community joined to a cluster by a pair carrying a fraction of its own cohesion must not be "
				"absorbed by it", w, subSceneOf[w], subSceneOf[10], subSceneOf[0]);
	}

	// 3. standalone community, with and without its seam tracks
	for (bool seam : {true, false}) {
		Scene scene;
		PairGraph g(scene, 25);
		TwoClusters(g, 9, 20);
		g.Clique(21, 24, 50);
		g.Pair(21, 10, 6);
		g.Pair(21, 9, 4);
		if (seam)
			SeamTracks(scene, {21, 22, 23, 10, 11, 12});
		const std::vector<int> subSceneOf = SubSceneOf(scene, Bounds(15, 5, false));
		TEST_CHECK(subSceneOf[0] >= 0 && subSceneOf[10] >= 0 && subSceneOf[0] != subSceneOf[10], "standalone community: A and B must be "
			"two sub-scenes (%d, %d), the fixture does not exercise the small-cluster pass otherwise", subSceneOf[0], subSceneOf[10]);
		for (IIndex w = 22; w < 25; ++w)
			TEST_CHECK(subSceneOf[w] == subSceneOf[21], "standalone community: W was split, image %u in sub-scene %d and 21 in %d", w,
				subSceneOf[w], subSceneOf[21]);
		if (seam)
			TEST_CHECK(subSceneOf[21] >= 0 && subSceneOf[21] != subSceneOf[10] && subSceneOf[21] != subSceneOf[0], "standalone community: "
				"W landed in sub-scene %d (B's hub in %d, A's in %d) although no neighbour is coupled to it and its seam to B is strong; it "
				"must stand alone", subSceneOf[21], subSceneOf[10], subSceneOf[0]);
		else
			TEST_CHECK(subSceneOf[21] == subSceneOf[10], "standalone community: W landed in sub-scene %d while B's hub is in %d; with no "
				"seam tracks the merge could not place it, so it goes to the neighbour it shares the most weight with", subSceneOf[21], subSceneOf[10]);
	}

	// 4. coupled orphan
	{
		Scene scene;
		PairGraph g(scene, 26);
		TwoClusters(g, 10, 21);
		g.Clique(22, 25, 50);
		g.Pair(22, 11, 30);
		g.Pair(23, 12, 30);
		SeamTracks(scene, {22, 23, 24, 11, 12, 13});
		const std::vector<int> subSceneOf = SubSceneOf(scene, Bounds(11, 0, false));
		TEST_CHECK(subSceneOf[0] >= 0 && subSceneOf[11] >= 0 && subSceneOf[0] != subSceneOf[11], "coupled orphan: A and B must be two "
			"sub-scenes (%d, %d), the fixture does not exercise the floor rule otherwise", subSceneOf[0], subSceneOf[11]);
		for (IIndex w = 23; w < 26; ++w)
			TEST_CHECK(subSceneOf[w] == subSceneOf[22], "coupled orphan: W was split, image %u in sub-scene %d and 22 in %d", w,
				subSceneOf[w], subSceneOf[22]);
		TEST_CHECK(subSceneOf[22] == -1, "coupled orphan: W came out as sub-scene %d although it is coupled to B (sub-scene %d) and B has "
			"no room for it; a coupled cluster under the floor must be skipped, not mistaken for a standalone community", subSceneOf[22],
			subSceneOf[11]);
	}
	return scope.Pass("weak images rescued; a community under the floor goes to the cluster it shares its weight with, stands alone "
		"with a strong seam and no coupled neighbour, and is skipped when coupled to a full cluster");
}
/*----------------------------------------------------------------*/

// The refinement passes of the clustering, on hand-made view graphs (PairGraph: every pair weighs exactly its number, the
// bodies are chains of (k,k+1) at 100 and (k,k+2) at 50):
// (1) the balance pass (RefineClustersBalance) never takes an image whose departure leaves part of its cluster hanging
//     under the coupling floor: X holds a 10-image body, a 5-image stretch and the one image bridging them, which leans
//     toward the smaller cluster Y (affinity 1.5, past the pass's 0.7); moving it would leave the stretch on the body by a
//     pair of weight 4 against its own 550. It stays, and so does the stretch, where moving the bridge would strand it;
// (2) the thin-waist check (RefineClustersSplitThinWaist) weighs every prefix cut along the Fiedler order: a 4-image
//     stretch hanging on its 12-image cluster C by a pair of weight 5 (coupling 0.0125), too small for a sub-scene of its
//     own and heavily coupled to the cluster D (250 against its own 400), moves whole to D; with no neighbour holding it at
//     the floor it stays where it is; and an 8-image side under-coupled to its cluster, large enough to stand, is split
//     off as before.
bool SceneClusterRefinePassesTest()
{
	TestScope scope("SceneClusterRefinePassesTest");
	// the body of images [first, last]: a chain of 100 and the pairs two apart at 50
	const auto Body = [](PairGraph& g, IIndex first, IIndex last) {
		g.Chain(first, last, 100.f);
		for (IIndex k = first; k + 1 < last; ++k)
			g.Pair(k, k + 2, 50.f);
	};
	// the images [first, last) as one cluster
	const auto Range = [](IIndex first, IIndex last) {
		IIndexArr cluster;
		for (IIndex i = first; i < last; ++i)
			cluster.push_back(i);
		return cluster;
	};
	const auto Holds = [](const IIndexArr& cluster, IIndex i) { return std::find(cluster.begin(), cluster.end(), i) != cluster.end(); };

	// 1. balance coupling: images body 0-9, bridge 10, stretch 11-15 (cluster X, 16 images); Y 16-23 (8 images)
	{
		Scene scene;
		PairGraph g(scene, 24);
		Body(g, 0, 9);                      // the body
		Body(g, 11, 15);                    // the stretch: internal weight 4 x 100 + 3 x 50 = 550
		Body(g, 16, 23);                    // Y
		g.Pair(9, 10, 100.f); g.Pair(8, 10, 50.f);    // the bridge holds the body: 150
		g.Pair(10, 11, 100.f); g.Pair(10, 12, 50.f);  // and the stretch: 150
		g.Pair(9, 11, 4.f);                           // the stretch's own link to the body, under the floor
		g.Pair(10, 16, 300.f); g.Pair(10, 17, 150.f); // and leans toward Y: 450 against its 300 in X
		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = 20;
		clusterCfg.targetViewsPerCluster = 16;
		clusterCfg.minViewsPerCluster = 4;
		clusterCfg.minWaistSideViews = 3;
		clusterCfg.minClusterDegree = 0;
		SceneCluster cluster(scene, clusterCfg);
		std::vector<IIndexArr> clusters{Range(0, 16), Range(16, 24)};
		cluster.RefineClustersBalance(clusters);
		for (IIndex i = 0; i <= 15; ++i)
			TEST_CHECK(Holds(clusters[0], i), "balance coupling: image %u left X, %s", i,
				i == 10 ? "the bridge whose departure strands the stretch under the floor" : "an image the fixture never offers Y");
	}

	// 2. thin stretch: C is the body 0-11 and the stretch 12..; D the 12 images after it
	struct Variant { const char* name; unsigned stretch; bool linkedToD; };
	for (const Variant& variant : {Variant{"a stretch D holds", 4, true}, Variant{"a stretch nothing holds", 4, false}, Variant{"a side that can stand", 8, true}}) {
		const IIndex firstD = 12 + variant.stretch, numImages = firstD + 12;
		Scene scene;
		PairGraph g(scene, numImages);
		Body(g, 0, 11);
		Body(g, 12, firstD - 1);
		Body(g, firstD, numImages - 1);
		g.Pair(11, 12, 5.f); // the stretch's only link to its body
		if (variant.linkedToD) {
			g.Pair(12, firstD, 100.f); g.Pair(13, firstD + 1, 100.f); g.Pair(14, firstD + 2, 50.f);
		}
		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = 24;
		clusterCfg.minViewsPerCluster = 6;
		clusterCfg.maxOverCapacity = 5;
		clusterCfg.minWaistSideViews = 3;
		clusterCfg.minClusterDegree = 0;
		SceneCluster cluster(scene, clusterCfg);
		std::vector<IIndexArr> clusters{Range(0, firstD), Range(firstD, numImages)};
		cluster.RefineClustersSplitThinWaist(clusters);

		// the cluster holding image 12 (the stretch's first), image 0 (the body) and image firstD (D)
		const auto ClusterOf = [&](IIndex img) {
			FOREACH(ci, clusters)
				if (Holds(clusters[ci], img))
					return (int)ci;
			return -1;
		};
		unsigned numClusters = 0;
		for (const IIndexArr& c : clusters)
			numClusters += !c.empty();
		const int cBody = ClusterOf(0), cStretch = ClusterOf(12), cD = ClusterOf(firstD);
		for (IIndex i = 12; i < firstD; ++i)
			TEST_CHECK(ClusterOf(i) == cStretch, "thin stretch, %s: the stretch split instead of moving or staying whole (image %u in %d, image 12 in %d)",
				variant.name, i, ClusterOf(i), cStretch);
		bool ok;
		if (variant.stretch >= clusterCfg.minViewsPerCluster) // large enough to stand on its own: split off as its own cluster
			ok = numClusters == 3 && cBody != cStretch && cStretch != cD && cBody != cD;
		else if (variant.linkedToD) // too small to stand, held more by D than by its own cluster: moves whole to D
			ok = numClusters == 2 && cStretch == cD && cBody != cStretch;
		else // too small to stand and no neighbour holds it better: stays in place
			ok = numClusters == 2 && cStretch == cBody && cBody != cD;
		TEST_CHECK(ok, "thin stretch, %s: %u clusters, image 0 in %d, image 12 in %d, image %u in %d", variant.name, numClusters, cBody, cStretch, firstD, cD);
	}
	return scope.Pass("the balance pass keeps the bridge and the stretch; the thin-waist check moves a held stretch whole, leaves an unheld one, splits a side that can stand");
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

// Clustering for the merge:
// (1) four hundred images walking a whole ring (40 blocks of 10): the sub-scenes must come out sized for the merge -- none under the floor, none over
//     the ceiling and its slack, half of them around the target, every image in exactly one -- and every two that touch must
//     share a seam the merge can register;
// (2) the seam-usable tracks of a cut, counted as the merge counts them: a track one side triangulates and the other sees,
//     credited to every camera that observed it (three-view tracks: only the two starting at 4 and 5 cross the cut between 5
//     and 6; five-view tracks reach across it from further away and spread the seam over the cameras the merge needs), on
//     chains of images that see their neighbours' tracks (BuildChainScene);
// (3) a tail joined to the rest of the capture by a handful of short tracks: it is big enough to keep, but no seam of it can
//     be registered, so it must not come out as a sub-scene of its own.
bool SceneClusterSeamTest()
{
	TestScope scope("SceneClusterSeamTest");

	// 1. target size
	unsigned numSubScenes;
	REAL median;
	{
		const RingSceneConfig ringCfg{40, 10};
		Scene scene;
		std::vector<IIndexArr> blocks;
		std::vector<Pose3D> gtPoses;
		GenerateRingScene(ringCfg, scene, blocks, gtPoses);
		const unsigned numImages = (unsigned)scene.images.size();
		const ClusterConfig clusterCfg;
		SceneCluster cluster(scene, clusterCfg);
		std::vector<IIndexArr> localToGlobals;
		const std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);
		numSubScenes = (unsigned)subScenes.size();
		TEST_CHECK(numSubScenes >= 3, "target size: expected >= 3 sub-scenes, got %u", numSubScenes);
		std::vector<REAL> sizes;
		for (unsigned s = 0; s < subScenes.size(); ++s) {
			const unsigned size = (unsigned)subScenes[s].images.size();
			TEST_CHECK(size >= clusterCfg.minViewsPerCluster && size <= clusterCfg.maxViewsPerCluster + clusterCfg.maxOverCapacity,
				"target size: sub-scene %u has %u images, outside [%u, %u]", s, size, clusterCfg.minViewsPerCluster,
				clusterCfg.maxViewsPerCluster + clusterCfg.maxOverCapacity);
			sizes.push_back((REAL)size);
		}
		median = Median(sizes);
		const REAL medianLow = (REAL)clusterCfg.targetViewsPerCluster * REAL(0.7);
		const REAL medianHigh = (REAL)clusterCfg.targetViewsPerCluster * REAL(1.3);
		TEST_CHECK(median >= medianLow && median <= medianHigh, "target size: median sub-scene size %.0f outside [%.0f, %.0f]", median, medianLow, medianHigh);
		// every image in exactly one sub-scene
		std::vector<unsigned> numAssignments(numImages, 0);
		for (const IIndexArr& localToGlobal : localToGlobals)
			for (IIndex globalID : localToGlobal)
				++numAssignments[globalID];
		for (unsigned i = 0; i < numImages; ++i)
			TEST_CHECK(numAssignments[i] == 1, "target size: image %u assigned to %u sub-scenes", i, numAssignments[i]);
		// every seam between two sub-scenes usable by the merge, by the rule the clustering applies
		const SeamTrackStatsMap seams = cluster.ComputeSeamTrackStats(localToGlobals);
		for (const auto& [pair, seam] : seams)
			TEST_CHECK(IsStrongSeam(seam, clusterCfg), "target size: sub-scenes (%u, %u) share %u seam-usable tracks over %u/%u cameras",
				pair.first, pair.second, seam.first.usable, seam.first.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks),
				seam.second.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks));
	}

	// 2. seam tracks: twelve images cut between 5 and 6
	{
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
			TEST_CHECK(seams.size() == 1, "seam tracks: expected one seam, got %u", (unsigned)seams.size());
			const std::pair<SeamTrackStats, SeamTrackStats>& seam = seams.begin()->second;
			TEST_CHECK(seam.first.usable == 40 && seam.second.usable == 40, "seam tracks: %u/%u seam-usable tracks, expected 40", seam.first.usable, seam.second.usable);
			const std::pair<IIndex, unsigned> expected[4] = {{5, 40}, {4, 20}, {6, 40}, {7, 20}};
			for (unsigned k = 0; k < 4; ++k) {
				const SeamTrackStats& stats = k < 2 ? seam.first : seam.second;
				const auto it = stats.perCamera.find(expected[k].first);
				TEST_CHECK(stats.perCamera.size() == 2 && it != stats.perCamera.end() && it->second == expected[k].second,
					"seam tracks: image %u carries %u seam-usable tracks of %u cameras, expected %u", expected[k].first,
					it == stats.perCamera.end() ? 0 : it->second, (unsigned)stats.perCamera.size(), expected[k].second);
			}
			TEST_CHECK(seam.first.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks) == 1 && seam.second.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks) == 1,
				"seam tracks: %u/%u cameras carrying minSeamCameraTracks, expected 1/1", seam.first.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks),
				seam.second.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks));
		}
		// five-view tracks reach across the cut from further away, and the seam spreads over the cameras the merge needs
		{
			Scene scene;
			BuildChainScene(12, 20, [](unsigned i) { return i <= 7 ? 5u : 0u; }, scene);
			SceneCluster cluster(scene, clusterCfg);
			const SeamTrackStatsMap seams = cluster.ComputeSeamTrackStats(clusters);
			TEST_CHECK(seams.size() == 1, "seam tracks: expected one seam, got %u", (unsigned)seams.size());
			const std::pair<SeamTrackStats, SeamTrackStats>& seam = seams.begin()->second;
			const unsigned cameras[2] = {seam.first.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks), seam.second.CamerasWithAtLeast(clusterCfg.minSeamCameraTracks)};
			TEST_CHECK(cameras[0] >= clusterCfg.minSeamCameras && cameras[1] >= clusterCfg.minSeamCameras,
				"seam tracks: %u/%u cameras carrying minSeamCameraTracks, expected >= %u each", cameras[0], cameras[1], clusterCfg.minSeamCameras);
		}
	}

	// 3. leaf merge: images 0..259 a chain of five-view tracks, images 260..319 a tail of the same (60, over the floor of 53, so only the leaf rule can place it), and between them only the
	// three-view tracks starting at 255..259 -- two of which cross the cut
	unsigned tailSubSceneSize;
	{
		Scene scene;
		BuildChainScene(320, 20, [](unsigned i) { return i >= 255 && i <= 259 ? 3u : 5u; }, scene);
		const ClusterConfig clusterCfg;
		SceneCluster cluster(scene, clusterCfg);
		std::vector<IIndexArr> localToGlobals;
		const std::vector<Scene> subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);
		for (unsigned s = 0; s < subScenes.size(); ++s)
			TEST_CHECK(subScenes[s].images.size() >= clusterCfg.minViewsPerCluster, "leaf merge: sub-scene %u has %u images < %u",
				s, (unsigned)subScenes[s].images.size(), clusterCfg.minViewsPerCluster);
		// the tail sits with the end of the chain, and that sub-scene reaches back into the chain proper: a tail left to
		// itself is the block the merge would have to leave unplaced
		unsigned subSceneOfEnd = NO_ID;
		FOREACH(s, localToGlobals)
			for (IIndex globalID : localToGlobals[s])
				if (globalID == 259)
					subSceneOfEnd = (unsigned)s;
		TEST_CHECK(subSceneOfEnd != NO_ID, "leaf merge: image 259 left out of the sub-scenes");
		const std::set<IIndex> withEnd(localToGlobals[subSceneOfEnd].begin(), localToGlobals[subSceneOfEnd].end());
		for (IIndex globalID = 260; globalID < 320; ++globalID)
			TEST_CHECK(withEnd.count(globalID) != 0, "leaf merge: image %u is not in the sub-scene of image 259 (%u images)", globalID, (unsigned)withEnd.size());
		TEST_CHECK(withEnd.count(250) != 0, "leaf merge: the tail and image 259 are a sub-scene of their own (%u images)", (unsigned)withEnd.size());
		tailSubSceneSize = (unsigned)withEnd.size();
	}
	return scope.Pass("%u sub-scenes of median %.0f images on 400, the seam tracks counted as the merge does, the tail merged into a %u-image sub-scene",
		numSubScenes, median, tailSubSceneSize);
}

} // namespace SFM
