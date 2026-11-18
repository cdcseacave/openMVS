/*
 * TestsSFMFeatures.cpp
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

// VocabularyTree save/load roundtrip test
bool VocabularyTreeTest()
{
	TD_TIMER_START();

	// Helper to fill one image with descriptors around provided center
	std::mt19937 rng(123);
	auto makeQuantizedDesc = [&rng](cv::Mat& dst, const std::vector<uint8_t>& center, unsigned nRows) {
		dst.create((int)nRows, (int)center.size(), CV_8U);
		std::normal_distribution<float> noise(0.f, 8.f);
		for (unsigned r = 0; r < nRows; ++r) {
			uint8_t* row = dst.ptr<uint8_t>((int)r);
			for (size_t c = 0; c < center.size(); ++c) {
				int v = ROUND2INT(center[c] + noise(rng));
				row[c] = (uint8_t)CLAMP(v, 0, 255);
			}
		}
	};
	// Helper for binary descriptors around prototype (flip few bits)
	auto makeBinaryDesc = [&rng](cv::Mat& dst, const std::vector<uint8_t>& proto, unsigned nRows, int flipsPerDesc) {
		dst.create((int)nRows, (int)proto.size(), CV_8U);
		std::uniform_int_distribution<int> bitPos(0, (int)proto.size() * 8 - 1);
		for (unsigned r = 0; r < nRows; ++r) {
			uint8_t* row = dst.ptr<uint8_t>((int)r);
			std::memcpy(row, proto.data(), proto.size());
			for (int f = 0; f < flipsPerDesc; ++f) {
				int b = bitPos(rng);
				int byte = b / 8, bit = b % 8;
				row[byte] ^= (1u << bit);
			}
		}
	};

	// --- Subtest 1: Quantized RootSIFT-like (CV_8U, L2) ---
	{
		Scene scene;
		const size_t numImages = 5;
		const size_t numDescriptorsPerImage = 300;
		const int descriptorDim = 128;
		// Create two clusters; images 0&1 near C0, 2&3 near C1, 4 mixed
		std::vector<uint8_t> C0(descriptorDim, 60), C1(descriptorDim, 200);
		for (size_t i = 0; i < numImages; ++i) {
			Image& img = scene.images.emplace_back((IIndex)i, "");
			img.keypoints.resize(numDescriptorsPerImage);
			if (i < 2)
				makeQuantizedDesc(img.descriptors, C0, (unsigned)numDescriptorsPerImage);
			else if (i < 4)
				makeQuantizedDesc(img.descriptors, C1, (unsigned)numDescriptorsPerImage);
			else {
				cv::Mat A, B;
				makeQuantizedDesc(A, C0, (unsigned)(numDescriptorsPerImage / 2));
				makeQuantizedDesc(B, C1, (unsigned)(numDescriptorsPerImage - numDescriptorsPerImage / 2));
				cv::vconcat(A, B, img.descriptors);
			}
		}
		VocabularyTree vocab;
		VocabularyTree::Config cfg;
		cfg.descriptorsAreBinary = false;
		cfg.K = 8;
		cfg.L = 5;
		cfg.maxKMeansIters = 8;
		if (!vocab.Build(scene, cfg)) {
			VERBOSE("VocabularyTreeTest: Build QFLOAT failed");
			return false;
		}
		// Query image 0 should have image 1 as top-2
		auto res0 = vocab.Query(scene.images[0], 3, 0.f);
		if (res0.empty()) {
			VERBOSE("VocabularyTreeTest: Query QFLOAT returned empty");
			return false;
		}
		bool found01 = false;
		for (auto& p : res0)
			if (p.first == 1)
				found01 = true;
		if (!found01) {
			VERBOSE("VocabularyTreeTest: expected img1 among top for img0");
			return false;
		}
		// Save / Release / Load-only should not have postings (queries empty)
		const String savePath = MAKE_PATH("vocab_q.bin");
		if (!vocab.Save(savePath)) {
			VERBOSE("VocabularyTreeTest: Save QFLOAT failed");
			return false;
		}
		vocab.Release();
		if (!vocab.Load(savePath)) {
			VERBOSE("VocabularyTreeTest: Load QFLOAT failed");
			return false;
		}
		auto resEmpty = vocab.Query(scene.images[0], 3, 0.f);
		if (!resEmpty.empty()) {
			VERBOSE("VocabularyTreeTest: Loaded tree without DB should return empty");
			return false;
		}
		// Index the current scene using the loaded tree
		if (!vocab.Index(scene)) {
			VERBOSE("VocabularyTreeTest: Index-after-load QFLOAT failed");
			return false;
		}
		auto res0b = vocab.Query(scene.images[0], 2, 0.f);
		bool found01b = false;
		for (auto& p : res0b)
			if (p.first == 1)
				found01b = true;
		if (!found01b) {
			VERBOSE("VocabularyTreeTest: img1 not found after reload");
			return false;
		}
		File::deleteFile(savePath);
	}

	// --- Subtest 2: Binary (CV_8U, Hamming) ---
	{
		Scene scene;
		const size_t numImages = 5;
		const size_t numDescriptorsPerImage = 300;
		const int descriptorBytes = 32; // 256-bit ORB-like
		std::vector<uint8_t> P0(descriptorBytes, 0x0F), P1(descriptorBytes, 0xF0);
		for (size_t i = 0; i < numImages; ++i) {
			Image& img = scene.images.emplace_back((IIndex)i, "");
			img.keypoints.resize(numDescriptorsPerImage);
			if (i < 2)
				makeBinaryDesc(img.descriptors, P0, (unsigned)numDescriptorsPerImage, 8);
			else if (i < 4)
				makeBinaryDesc(img.descriptors, P1, (unsigned)numDescriptorsPerImage, 8);
			else {
				cv::Mat A, B;
				makeBinaryDesc(A, P0, (unsigned)(numDescriptorsPerImage / 2), 8);
				makeBinaryDesc(B, P1, (unsigned)(numDescriptorsPerImage - numDescriptorsPerImage / 2), 8);
				cv::vconcat(A, B, img.descriptors);
			}
		}
		VocabularyTree vocab;
		VocabularyTree::Config cfg;
		cfg.descriptorsAreBinary = true;
		cfg.K = 8;
		cfg.L = 5;
		cfg.maxKMeansIters = 8;
		if (!vocab.Build(scene, cfg)) {
			VERBOSE("VocabularyTreeTest: Build BINARY failed");
			return false;
		}
		auto r0 = vocab.Query(scene.images[0], 3, 0.f);
		bool found1 = false;
		for (auto& p : r0)
			if (p.first == 1)
				found1 = true;
		if (!found1) {
			VERBOSE("VocabularyTreeTest: expected img1 among top (binary)");
			return false;
		}
		const String savePath = MAKE_PATH("vocab_b.bin");
		if (!vocab.Save(savePath)) {
			VERBOSE("VocabularyTreeTest: Save BINARY failed");
			return false;
		}
		vocab.Release();
		if (!vocab.Load(savePath)) {
			VERBOSE("VocabularyTreeTest: Load BINARY failed");
			return false;
		}
		if (!vocab.Index(scene)) {
			VERBOSE("VocabularyTreeTest: Index-after-load BINARY failed");
			return false;
		}
		auto r0b = vocab.Query(scene.images[0], 3, 0.f);
		bool found1b = false;
		for (auto& p : r0b)
			if (p.first == 1)
				found1b = true;
		if (!found1b) {
			VERBOSE("VocabularyTreeTest: img1 not found after reload (binary)");
			return false;
		}
		File::deleteFile(savePath);
	}

	VERBOSE("VocabularyTreeTest: All tests passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Pair assembly and storage (AssemblePairDense, StorePairDense) on the exact geometry of two
// pinhole cameras: the union of the guided matches and the dense fill is fitted once and splits
// into the pair's sparse and dense segments, the fill is drawn only where the guided matches are
// not, a pair with no guided match at all is still assembled as a dense-only pair, a fill too small
// to fit leaves the verdict's geometry untouched, and the store appends the dense keypoints past
// each image's described prefix with indices that depend only on what the images already carry
bool BuildTracksConflictTest()
{
	TD_TIMER_START();

	// A scene of one-keypoint images (image A gets two) with weighted pairs holding one match each;
	// the pairs are appended in decreasing weight, the order BuildTracks receives them in, and
	// given with ID1 < ID2, the order ImagePair keeps (its constructor swaps the IDs, not the match).
	struct Link { IIndex ID1, ID2; int q, t; float weight; };
	const auto Build = [](const std::vector<std::vector<Point2f>>& keypoints, const std::vector<Link>& links, Scene& scene) {
		FOREACH(i, keypoints) {
			Image& img = scene.images.emplace_back((IIndex)i, String::FormatString("%u.jpg", i));
			for (const Point2f& pt : keypoints[i])
				img.keypoints.emplace_back(pt.x, pt.y, 3.f, -1.f, 0.1f);
		}
		for (const Link& link : links) {
			ASSERT(link.ID1 < link.ID2);
			ImagePair& pair = scene.pairs.emplace_back(link.ID1, link.ID2);
			pair.matches.emplace_back(link.q, link.t);
			pair.numFilteredInliers = 1;
			// W = numInliers * cbrt(weightSpatial * overlapArea^2 * weightConnectivity) * (0.25 + weightTriplet)
			pair.weightConnectivity = 1.f;
			pair.weightTriplet = 0.75f;
			pair.weightSpatial = POWI(link.weight, 3);
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			ASSERT(ISEQUAL(pair.GetCompositeWeight(), link.weight));
		}
	};
	// the tracks as sorted lists of (image, feature), sorted
	typedef std::vector<std::vector<std::pair<IIndex, uint32_t>>> Tracks;
	const auto TracksOf = [](const Scene& scene) {
		Tracks tracks;
		for (const Track& track : scene.tracks) {
			std::vector<std::pair<IIndex, uint32_t>> obs;
			for (const Observation& o : track.observations)
				obs.emplace_back(o.imageID, o.featureID);
			std::sort(obs.begin(), obs.end());
			tracks.push_back(obs);
		}
		std::sort(tracks.begin(), tracks.end());
		return tracks;
	};
	const auto Report = [](const char* what, const Tracks& tracks) {
		VERBOSE("BuildTracksConflictTest FAILED: %s (%u tracks)", what, (unsigned)tracks.size());
		FOREACH(k, tracks) {
			String s;
			for (const auto& [img, feat] : tracks[k])
				s += String::FormatString(" (%u,%u)", img, feat);
			VERBOSE("BuildTracksConflictTest:   track %u:%s", k, s.c_str());
		}
	};

	// Image A holds a1 (keypoint 0) and a2 (keypoint 1), far apart; B, C, D one keypoint each.
	// The links, heaviest first: a2-c (6), b-c (5), a1-b (4), a1-d (4), b-d (4). The plain union
	// joins all five keypoints and image A is in twice. The triangle a1-b-d corroborates a1-b, the
	// chain b-c-a2 has no triangle: on the path a1-b-c-a2 the least-supported links are b-c and c-a2
	// (no triangle), the lighter of them b-c is cut, and b stays with the corroborated side.
	const std::vector<std::vector<Point2f>> keypointsA = {
		{Point2f(10.f, 10.f), Point2f(100.f, 100.f)}, {Point2f(20.f, 20.f)}, {Point2f(30.f, 30.f)}, {Point2f(40.f, 40.f)}};
	const std::vector<Link> linksA = {
		{0, 2, 1, 0, 6.f}, {1, 2, 0, 0, 5.f}, {0, 1, 0, 0, 4.f}, {0, 3, 0, 0, 4.f}, {1, 3, 0, 0, 4.f}};
	const Tracks cutA = {{{0, 0}, {1, 0}, {3, 0}}, {{0, 1}, {2, 0}}};
	// the veto in pair order takes a2-c and b-c first, then refuses a1-b and b-d: b stays with a2
	const Tracks vetoA = {{{0, 0}, {3, 0}}, {{0, 1}, {1, 0}, {2, 0}}};
	{
		Scene scene;
		Build(keypointsA, linksA, scene);
		BuildTracks(scene, 0.f);
		if (TracksOf(scene) != cutA) {
			Report("the cut keeps b on the corroborated side", TracksOf(scene));
			return false;
		}
	}
	{
		Scene scene;
		Build(keypointsA, linksA, scene);
		TrackConflictConfig conflict;
		conflict.cut = false;
		BuildTracks(scene, 0.f, conflict);
		if (TracksOf(scene) != vetoA) {
			Report("without the cut the veto keeps the first arrival", TracksOf(scene));
			return false;
		}
	}
	{
		Scene scene;
		Build(keypointsA, linksA, scene);
		TrackConflictConfig conflict;
		conflict.maxComponentSize = 4; // five keypoints: too large for the cut
		BuildTracks(scene, 0.f, conflict);
		if (TracksOf(scene) != vetoA) {
			Report("a component above the size bar takes the veto", TracksOf(scene));
			return false;
		}
	}

	// a1 and a2 two pixels apart, linked a1-b, a2-c, b-c: one feature seen twice. Within the merge
	// distance the component is one track and a1, first of two equally linked keypoints, stands
	// for image A; with the merge off the conflict is cut: all links equally unsupported and
	// weighted, the earliest, a1-b, goes, and a1 alone is no track.
	const std::vector<std::vector<Point2f>> keypointsB = {
		{Point2f(10.f, 10.f), Point2f(12.f, 10.f)}, {Point2f(20.f, 20.f)}, {Point2f(30.f, 30.f)}};
	const std::vector<Link> linksB = {{0, 1, 0, 0, 3.f}, {0, 2, 1, 0, 3.f}, {1, 2, 0, 0, 3.f}};
	{
		Scene scene;
		Build(keypointsB, linksB, scene);
		BuildTracks(scene, 0.f);
		const Tracks expected = {{{0, 0}, {1, 0}, {2, 0}}};
		if (TracksOf(scene) != expected) {
			Report("two keypoints within the merge distance are one feature", TracksOf(scene));
			return false;
		}
	}
	{
		Scene scene;
		Build(keypointsB, linksB, scene);
		TrackConflictConfig conflict;
		conflict.mergeDistance = 0.f;
		BuildTracks(scene, 0.f, conflict);
		const Tracks expected = {{{0, 1}, {1, 0}, {2, 0}}};
		if (TracksOf(scene) != expected) {
			Report("with the merge off the near duplicates are cut apart", TracksOf(scene));
			return false;
		}
	}
	{
		Scene scene;
		Build(keypointsB, linksB, scene);
		TrackConflictConfig conflict;
		conflict.cut = false;
		BuildTracks(scene, 0.f, conflict);
		const Tracks expected = {{{0, 0}, {1, 0}, {2, 0}}};
		if (TracksOf(scene) != expected) {
			Report("the veto merges two keypoints within the merge distance too", TracksOf(scene));
			return false;
		}
	}

	// Three keypoints of image A: a1 and a2 (keypoints 0, 1) 2.9 px apart, a3 (keypoint 2) 2.9 px
	// from a1 and 5.8 from a2. The links, heaviest first: a3-b, a1-c, a2-d, c-d, b-c. The veto
	// merges a1 and a2 through c-d, then refuses b-c: a3 is one feature with a1 but not with a2.
	// The cut sees the same three in one component, and b-c, unsupported and lightest on the path
	// from a3 to a2, goes. Either way a1, the first of two equally linked, stands for A.
	const std::vector<std::vector<Point2f>> keypointsD = {
		{Point2f(10.f, 10.f), Point2f(12.9f, 10.f), Point2f(7.1f, 10.f)}, {Point2f(20.f, 20.f)}, {Point2f(30.f, 30.f)}, {Point2f(40.f, 40.f)}};
	const std::vector<Link> linksD = {
		{0, 1, 2, 0, 6.f}, {0, 2, 0, 0, 5.f}, {0, 3, 1, 0, 5.f}, {2, 3, 0, 0, 4.f}, {1, 2, 0, 0, 3.f}};
	const Tracks tracksD = {{{0, 0}, {2, 0}, {3, 0}}, {{0, 2}, {1, 0}}};
	for (const bool cut : {true, false}) {
		Scene scene;
		Build(keypointsD, linksD, scene);
		TrackConflictConfig conflict;
		conflict.cut = cut;
		BuildTracks(scene, 0.f, conflict);
		if (TracksOf(scene) != tracksD) {
			Report(cut ? "the cut merges one feature and not its far neighbour" : "the veto merges one feature and not its far neighbour", TracksOf(scene));
			return false;
		}
	}

	// a chain a-b-c without a conflict is one track of three, under either rule
	const std::vector<std::vector<Point2f>> keypointsC = {{Point2f(10.f, 10.f)}, {Point2f(20.f, 20.f)}, {Point2f(30.f, 30.f)}};
	const std::vector<Link> linksC = {{0, 1, 0, 0, 3.f}, {1, 2, 0, 0, 3.f}};
	{
		Scene scene;
		Build(keypointsC, linksC, scene);
		BuildTracks(scene, 0.f);
		const Tracks expected = {{{0, 0}, {1, 0}, {2, 0}}};
		if (TracksOf(scene) != expected) {
			Report("a conflict-free chain is one track", TracksOf(scene));
			return false;
		}
	}

	VERBOSE("BuildTracksConflictTest passed: %s", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Global-descriptor retrieval test: cosine ranking of the per-image descriptors, its
// deterministic tie order, PairsMatcher's RETRIEVAL-mode pair selection over the same
// descriptors, the rankings CSV export, and the .sfm round-trip of the descriptors
bool GlobalDescriptorsQueryTest()
{
	TD_TIMER_START();

	// three clusters of four images, each cluster occupying its own eighth of the descriptor
	Scene scene;
	const int D = 64;
	std::mt19937 rng(11);
	std::normal_distribution<float> noise(0.f, 0.05f);
	for (IIndex i = 0; i < 12; ++i) {
		Image& img = scene.images.emplace_back(i, String::FormatString("%02u.jpg", i));
		img.keypoints.resize(10);
		img.descriptors = cv::Mat::zeros(10, 32, CV_8U); // PairsMatcher expects descriptors to exist
		img.globalDescriptor.create(1, D, CV_32F);
		for (int c = 0; c < D; ++c)
			img.globalDescriptor.at<float>(c) = (c/8 == (int)(i/4) ? 1.f : 0.f) + noise(rng);
		img.globalDescriptor /= cv::norm(img.globalDescriptor);
	}
	scene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);

	// the top-3 neighbors of an image all belong to its own cluster
	GlobalDescriptors index;
	if (!index.Build(scene) || index.Dim() != D || index.Size() != 12) {
		VERBOSE("GlobalDescriptorsQueryTest FAILED: build");
		return false;
	}
	for (const auto& [id, score] : index.Query(5, 3)) {
		if (id / 4 != 1 || id == 5) {
			VERBOSE("GlobalDescriptorsQueryTest FAILED: image %u ranked for image 5", id);
			return false;
		}
	}

	// identical descriptors rank by ascending image ID, and two queries agree
	scene.images[9].globalDescriptor = scene.images[8].globalDescriptor.clone();
	scene.images[10].globalDescriptor = scene.images[8].globalDescriptor.clone();
	GlobalDescriptors index2;
	index2.Build(scene);
	const auto tied = index2.Query(11, 3);
	if (tied.size() != 3 || tied[0].first != 8 || tied[1].first != 9 || tied[2].first != 10 || index2.Query(11, 3) != tied) {
		VERBOSE("GlobalDescriptorsQueryTest FAILED: tie order");
		return false;
	}

	// RETRIEVAL mode ranks the pair selection through these same global descriptors, and still
	// returns a connected view graph, joined by at most two cross-cluster bridges
	MatchConfig matchCfg;
	matchCfg.mode = MatchConfig::RETRIEVAL;
	matchCfg.maxPairsPerImage = 3;
	PairsMatcher matcher(scene, matchCfg);
	const PairIdxArr pairs = matcher.CollectRetrievalPairs(2);
	DisjointSet<IIndex> components(12);
	unsigned numCross = 0;
	for (const PairIdx& p : pairs) {
		components.Union(p.i, p.j);
		numCross += (p.i/4 != p.j/4);
	}
	for (IIndex i = 1; i < 12; ++i) {
		if (components.Find(i) != components.Find(0)) {
			VERBOSE("GlobalDescriptorsQueryTest FAILED: view graph split");
			return false;
		}
	}
	if (numCross > 2) {
		VERBOSE("GlobalDescriptorsQueryTest FAILED: %u cross-cluster pairs", numCross);
		return false;
	}

	// the .sfm round-trip of the descriptors
	ScopedTempDir tmp("GlobalDescriptorsQueryTest");
	if (!tmp.IsValid())
		return false;
	if (!scene.Save(tmp("scene.sfm")))
		return false;
	Scene loaded;
	if (!loaded.Load(tmp("scene.sfm")) || !loaded.images[3].HasGlobalDescriptor() ||
		cv::norm(loaded.images[3].globalDescriptor - scene.images[3].globalDescriptor) > 1e-6) {
		VERBOSE("GlobalDescriptorsQueryTest FAILED: serialization");
		return false;
	}

	VERBOSE("GlobalDescriptorsQueryTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Pose-guided selection must still produce candidate pairs for images absent from the pose file.
bool KnownPosePairSelectionTest()
{
	Scene scene;
	SceneConfig sceneCfg;
	sceneCfg.numImages = 6;
	sceneCfg.numPoints = 120;
	sceneCfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	sceneCfg.rotationAngleStep = 60;
	sceneCfg.generateDescriptors = true;
	GenerateTestScene(scene, sceneCfg);
	const IIndex unposedImage = scene.images.size() - 1;
	const IIndex unposedImageID = scene.images[unposedImage].ID;
	scene.images[unposedImage].InvalidatePose();

	MatchConfig matchCfg;
	matchCfg.descriptorsAreBinary = sceneCfg.binaryDescriptors;
	matchCfg.maxPairsPerImage = 4;
	PairsMatcher matcher(scene, matchCfg);
	const PairIdxArr pairs = matcher.CollectKnownPosePairs(3);
	bool coversUnposedImage = false;
	std::unordered_set<PairIdx::PairIndex> uniquePairs;
	for (const PairIdx& pair : pairs) {
		if (!uniquePairs.emplace(pair.idx).second) {
			VERBOSE("KnownPosePairSelectionTest FAILED: duplicate pair (%u, %u)", pair.i, pair.j);
			return false;
		}
		coversUnposedImage = coversUnposedImage || pair.i == unposedImageID || pair.j == unposedImageID;
	}
	if (!coversUnposedImage) {
		VERBOSE("KnownPosePairSelectionTest FAILED: no candidate covers the unposed image");
		return false;
	}

	VERBOSE("KnownPosePairSelectionTest PASSED (%u candidates)", (unsigned)pairs.size());
	return true;
}

// PairsMatcher::AddSequentialPairs unit test: the (i, i+k) pairs it appends to an existing
// candidate list for k = 1..overlap (no wrap-around), never duplicating a pair already present,
// and adding nothing at overlap 0.
bool SequentialPairsPriorTest()
{
	Scene scene;
	SceneConfig sceneCfg;
	sceneCfg.numImages = 6;
	sceneCfg.numPoints = 0;
	GenerateTestScene(scene, sceneCfg);

	// (i, i+k) for k = 1..3, no wrap-around, on 6 images
	const std::vector<std::pair<IIndex,IIndex>> expectedPairs = {
		{0,1}, {0,2}, {0,3},
		{1,2}, {1,3}, {1,4},
		{2,3}, {2,4}, {2,5},
		{3,4}, {3,5},
		{4,5}
	};
	const auto ToSet = [](const PairIdxArr& pairs) {
		std::unordered_set<PairIdx::PairIndex> set;
		for (const PairIdx& pair : pairs)
			set.emplace(pair.idx);
		return set;
	};
	const auto HasAllExpected = [&expectedPairs](const std::unordered_set<PairIdx::PairIndex>& set) {
		for (const auto& [i, j] : expectedPairs)
			if (set.find(MakePairIdx(i, j).idx) == set.end())
				return false;
		return true;
	};

	// Case 1: an empty candidate list gains exactly the expected pairs
	PairIdxArr pairs;
	unsigned numAdded = PairsMatcher::AddSequentialPairs(scene, 3, pairs);
	if (numAdded != expectedPairs.size() || pairs.size() != expectedPairs.size() || !HasAllExpected(ToSet(pairs))) {
		VERBOSE("SequentialPairsPriorTest FAILED: empty list gained %u pairs (%u total), expected %u",
			numAdded, (unsigned)pairs.size(), (unsigned)expectedPairs.size());
		return false;
	}

	// Case 2: a list already holding (0,1) and (2,3) gains the rest and holds no duplicates
	PairIdxArr partial;
	partial.emplace_back(MakePairIdx(0, 1));
	partial.emplace_back(MakePairIdx(2, 3));
	numAdded = PairsMatcher::AddSequentialPairs(scene, 3, partial);
	const std::unordered_set<PairIdx::PairIndex> partialSet = ToSet(partial);
	if (numAdded != expectedPairs.size() - 2 || partial.size() != expectedPairs.size() ||
		partialSet.size() != partial.size() || !HasAllExpected(partialSet)) {
		VERBOSE("SequentialPairsPriorTest FAILED: partial list gained %u pairs (%u total, %u unique), expected %u (%u total)",
			numAdded, (unsigned)partial.size(), (unsigned)partialSet.size(),
			(unsigned)expectedPairs.size() - 2, (unsigned)expectedPairs.size());
		return false;
	}

	// Case 3: overlap 0 adds nothing
	PairIdxArr none;
	numAdded = PairsMatcher::AddSequentialPairs(scene, 0, none);
	if (numAdded != 0 || !none.empty()) {
		VERBOSE("SequentialPairsPriorTest FAILED: overlap 0 added %u pairs", numAdded);
		return false;
	}

	VERBOSE("SequentialPairsPriorTest PASSED (%u pairs for overlap 3 on %u images)",
		(unsigned)expectedPairs.size(), (unsigned)scene.images.size());
	return true;
}

// ===============================================================================
// Spherical bottom band of the feature extraction (FeatureExtractionConfig::sphericalBottomBand):
// a textured equirectangular image gives keypoints all over it with the band off, none in the
// bottom 20% of rows with the default band and as many above it as before (the band removes
// keypoints, it does not move the budget); a perspective image of the same texture is unaffected
// by the option (same keypoints, same descriptors).
// ===============================================================================
bool SphericalBottomBandTest()
{
	VERBOSE("\n=== SphericalBottomBandTest: features in the bottom rows of a spherical image ===");
	constexpr int width = 2048, height = 1024;
	// blurred noise, fine enough for SIFT blobs everywhere
	cv::RNG rng(7);
	cv::Mat noise(height, width, CV_8UC3);
	rng.fill(noise, cv::RNG::UNIFORM, 0, 256);
	cv::Mat texture;
	cv::GaussianBlur(noise, texture, cv::Size(0, 0), 2.5);
	cv::normalize(texture, texture, 0, 255, cv::NORM_MINMAX);

	FeatureExtractionConfig cfg;
	cfg.detectorType = FeatureType::SIFT;
	cfg.maxFeaturesPerCell = 200;
	cfg.minFeaturesPerCell = 100;
	cfg.cubemapFaceSize = 512;
	ASSERT(cfg.sphericalBottomBand == 0.2f); // on by default

	auto Extract = [&](float band, bool spherical, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
		Scene scene;
		std::unique_ptr<Camera> camera;
		if (spherical)
			camera = std::make_unique<SphericalCamera>(cv::Size(width, height));
		else
			camera = std::make_unique<PinholeCamera>(cv::Size(width, height), 1000., 1000., width / 2., height / 2.);
		Image image;
		image.ID = 0;
		image.cameraID = 0; // a valid camera ID, the camera is owned here and not freed by the image
		image.pCamera = camera.get();
		if (spherical)
			image.pixels = texture.clone();
		else
			cv::cvtColor(texture, image.pixels, cv::COLOR_BGR2GRAY);
		cfg.sphericalBottomBand = band;
		FeaturesExtractor extractor(scene, cfg);
		cv::Ptr<cv::Feature2D> detector;
		if (!extractor.ExtractImage(image, detector, true))
			return false;
		keypoints = image.keypoints;
		descriptors = image.descriptors.clone();
		return true;
	};

	std::vector<cv::KeyPoint> kpOff, kpOn;
	cv::Mat descOff, descOn;
	if (!Extract(0.f, true, kpOff, descOff) || !Extract(0.2f, true, kpOn, descOn)) {
		VERBOSE("SphericalBottomBandTest FAILED: the spherical extraction failed");
		return false;
	}
	const float bandRow = 0.8f * height;
	size_t offBelow = 0, offAbove = 0, onBelow = 0, onAbove = 0;
	for (const cv::KeyPoint& kp : kpOff)
		(kp.pt.y > bandRow ? offBelow : offAbove)++;
	for (const cv::KeyPoint& kp : kpOn)
		(kp.pt.y > bandRow ? onBelow : onAbove)++;
	VERBOSE("  band 0: %u keypoints above row %g, %u below; band 0.2: %u above, %u below",
		(unsigned)offAbove, bandRow, (unsigned)offBelow, (unsigned)onAbove, (unsigned)onBelow);
	if (offBelow < 100) {
		VERBOSE("SphericalBottomBandTest FAILED: the band-0 extraction has only %u keypoints in the bottom 20%% of rows, the test image is not textured enough", (unsigned)offBelow);
		return false;
	}
	if (onBelow != 0) {
		VERBOSE("SphericalBottomBandTest FAILED: %u keypoints in the discarded bottom band", (unsigned)onBelow);
		return false;
	}
	// the band masks keypoints one by one after the faces' detection, which it does not touch: the rows above the band keep
	// every keypoint (the angular NMS across the face seams could only differ where a masked keypoint lies within
	// cubemapDedupAngleDeg, 0.25 degrees or a row or two of the image, of one above)
	if (onAbove != offAbove) {
		VERBOSE("SphericalBottomBandTest FAILED: the rows above the band hold %u keypoints, %u without the band", (unsigned)onAbove, (unsigned)offAbove);
		return false;
	}
	if ((int)onAbove != descOn.rows) {
		VERBOSE("SphericalBottomBandTest FAILED: %u keypoints but %d descriptor rows", (unsigned)onAbove, descOn.rows);
		return false;
	}

	std::vector<cv::KeyPoint> kpP0, kpP2;
	cv::Mat descP0, descP2;
	if (!Extract(0.f, false, kpP0, descP0) || !Extract(0.2f, false, kpP2, descP2)) {
		VERBOSE("SphericalBottomBandTest FAILED: the perspective extraction failed");
		return false;
	}
	if (kpP0.empty() || kpP0.size() != kpP2.size() || cv::norm(descP0, descP2, cv::NORM_INF) != 0) {
		VERBOSE("SphericalBottomBandTest FAILED: the band changed the perspective extraction (%u vs %u keypoints)", (unsigned)kpP0.size(), (unsigned)kpP2.size());
		return false;
	}
	for (size_t i = 0; i < kpP0.size(); ++i)
		if (kpP0[i].pt != kpP2[i].pt) {
			VERBOSE("SphericalBottomBandTest FAILED: the band moved perspective keypoint %u", (unsigned)i);
			return false;
		}
	VERBOSE("SphericalBottomBandTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

// the detector words: the four accepted ones in any letter case, any other word is NONE
bool FeatureTypeNameTest()
{
	VERBOSE("\n=== FeatureTypeNameTest: detector words in any letter case ===");
	const std::pair<const char*, FeatureType> accepted[] = {
		{"AKAZE", FeatureType::AKAZE}, {"akaze", FeatureType::AKAZE},
		{"ORB", FeatureType::ORB}, {"Orb", FeatureType::ORB},
		{"SIFT", FeatureType::SIFT}, {"sift", FeatureType::SIFT},
		{"SIFTGPU", FeatureType::SIFTGPU}, {"SiftGpu", FeatureType::SIFTGPU}};
	for (const auto& a : accepted)
		if (FeatureTypeFromString(a.first) != a.second) {
			VERBOSE("FeatureTypeNameTest FAILED: '%s' is not read as %s", a.first, FeatureTypeToString(a.second).c_str());
			return false;
		}
	for (const char* word : {"", "SURF", "SIFT GPU", "siftgpu2"})
		if (FeatureTypeFromString(word) != FeatureType::NONE) {
			VERBOSE("FeatureTypeNameTest FAILED: '%s' is accepted", word);
			return false;
		}
	VERBOSE("FeatureTypeNameTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

// ===============================================================================
// PairsMatcher spherical relative pose test: end-to-end integration test for
// the PairsMatcher -> poselib::estimate_relative_pose_bearings path. Exercises
// RANSAC scoring, cheirality-off behavior for spherical, and the Sampson-on-sphere
// Jacobian in refine_relpose_bearing.
// ===============================================================================
bool PairsMatcherSphericalTest()
{
	VERBOSE("\n=== PairsMatcherSphericalTest: spherical relative pose via bearings ===");

	Scene scene;
	Pose3D poseRelGT;
	BuildSphericalTwoViewScene(scene, poseRelGT);
	VERBOSE("Built scene: %u images, %u tracks", (unsigned)scene.images.size(), (unsigned)scene.tracks.size());

	// Populate matches for the pair from the ground-truth track observations.
	ImagePair pair(0, 1);
	for (const Track& track : scene.tracks) {
		uint32_t feat0 = NO_ID, feat1 = NO_ID;
		for (const auto& obs : track.observations) {
			if (obs.imageID == 0) feat0 = obs.featureID;
			else if (obs.imageID == 1) feat1 = obs.featureID;
		}
		if (feat0 != NO_ID && feat1 != NO_ID)
			pair.matches.emplace_back(feat0, feat1);
	}
	VERBOSE("Built pair with %u matches", pair.GetNumMatches());

	// Run geometric verification via MatchGeometric (the "calibrated" branch of
	// PairsMatcher::MatchPair). This is the site that was rewritten in Phase 3.
	MatchConfig matchCfg;
	matchCfg.minMatches = 8;
	matchCfg.maxEpipolarError = 5.f;
	PairsMatcher matcher(scene, matchCfg);
	if (!matcher.MatchPair(scene.images[0], scene.images[1], pair)) {
		VERBOSE("PairsMatcherSphericalTest FAILED: MatchPair returned false");
		return false;
	}
	if (!pair.relativePose.has_value()) {
		VERBOSE("PairsMatcherSphericalTest FAILED: pair has no relative pose after MatchPair");
		return false;
	}

	const Pose3D& poseRelRecovered = pair.relativePose.value();
	const REAL angleErr = R2D(ACOS(ComputeAngle(poseRelRecovered.R, poseRelGT.R)));

	// Translation is recovered up to scale; check direction similarity.
	const Point3 tRecovered = poseRelRecovered.GetT();
	const Point3 tGT = poseRelGT.GetT();
	const Point3 tGTnorm = normalized(tGT);
	const Point3 tRecNorm = normalized(tRecovered);
	const REAL tSim = ABS(tGTnorm.dot(tRecNorm));

	VERBOSE("PairsMatcherSphericalTest: matches=%u, inliers=%u, rotation err=%.4f deg, t similarity=%.4f",
	        pair.GetNumMatches(), pair.GetNumInliers(), angleErr, tSim);

	if (angleErr > REAL(0.5)) {
		VERBOSE("PairsMatcherSphericalTest FAILED: rotation error %.4f deg > 0.5 deg", angleErr);
		return false;
	}
	if (tSim < REAL(0.99)) {
		VERBOSE("PairsMatcherSphericalTest FAILED: translation similarity %.4f < 0.99", tSim);
		return false;
	}

	VERBOSE("PairsMatcherSphericalTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

// Note: the bearing-vector absolute pose (PnP) is tested directly in the
// PoseLib test suite (ports/poselib/source/tests/optim_bearing_test.cc) —
// specifically test_estimate_absolute_pose_bearings and
// test_bearing_absolute_pose_jacobian. We keep PairsMatcherSphericalTest as
// an OpenMVS integration test because it exercises PairsMatcher::MatchPair
// (geometric verification orchestration), which is OpenMVS-specific.


// ===============================================================================
// MatchFeaturesGeometric spherical test: exercises the tracked-point guided
// matching pipeline (used by KeyframeExtractor on 360° video). Tests the
// post-RANSAC epipolar-constrained descriptor filtering step which cannot
// use F-matrices for spherical pairs. Before Phase 4 this path would
// throw bad_optional_access on pair.F.value() for any pair where at least
// one camera is spherical.
// ===============================================================================
bool MatchGeometricSphericalTest()
{
	VERBOSE("\n=== MatchGeometricSphericalTest: tracked-guided matching on spherical pair ===");

	Scene scene;
	Pose3D poseRelGT;
	BuildSphericalTwoViewScene(scene, poseRelGT);
	Image& img0 = scene.images[0];
	Image& img1 = scene.images[1];
	VERBOSE("Built scene: %u images, %u tracks, img0.kpts=%u, img1.kpts=%u",
	        (unsigned)scene.images.size(), (unsigned)scene.tracks.size(),
	        (unsigned)img0.keypoints.size(), (unsigned)img1.keypoints.size());

	// Build tracked-point arrays indexed by img0's keypoint index (MatchFeaturesGeometric's
	// convention). For each img0 feature i, find the track that owns it and look up the
	// corresponding img1 feature; set trackedPoints2[i] to that img1 keypoint position.
	const size_t N0 = img0.keypoints.size();
	std::vector<Point2f> trackedPoints1(N0);
	std::vector<Point2f> trackedPoints2(N0, Point2f(0.f, 0.f));
	std::vector<uchar> trackStatus(N0, 0);
	for (size_t i = 0; i < N0; ++i)
		trackedPoints1[i] = img0.keypoints[i].pt;

	// Map img0.featID -> img1.featID via tracks. Since BuildSphericalTwoViewScene
	// rejects tracks with < 2 observations, every surviving track for a 2-view
	// scene has exactly one observation per image.
	std::vector<uint32_t> feat0ToFeat1(N0, NO_ID);
	for (const Track& t : scene.tracks) {
		uint32_t feat0 = NO_ID, feat1 = NO_ID;
		for (const auto& obs : t.observations) {
			if (obs.imageID == 0) feat0 = obs.featureID;
			else if (obs.imageID == 1) feat1 = obs.featureID;
		}
		if (feat0 != NO_ID && feat1 != NO_ID) {
			ASSERT(feat0 < N0);
			feat0ToFeat1[feat0] = feat1;
			trackedPoints2[feat0] = img1.keypoints[feat1].pt;
			trackStatus[feat0] = 1;
		}
	}
	size_t numTracked = 0;
	for (uchar s : trackStatus)
		if (s) ++numTracked;
	VERBOSE("MatchGeometricSphericalTest: %zu tracked correspondences", numTracked);
	if (numTracked < 50) {
		VERBOSE("MatchGeometricSphericalTest FAILED: only %zu tracked correspondences (need >= 50)", numTracked);
		return false;
	}

	// Synthesize unique 256-bit binary descriptors per track. Paired img0/img1
	// keypoints share the same descriptor (Hamming distance 0) so descriptor
	// matching always prefers the ground-truth pair as the closest candidate.
	// Different tracks get pseudo-random distinct patterns (high Hamming distance).
	const int descBytes = 32;
	img0.descriptors.create((int)N0, descBytes, CV_8U);
	img1.descriptors.create((int)img1.keypoints.size(), descBytes, CV_8U);
	img0.descriptors.setTo(cv::Scalar::all(0));
	img1.descriptors.setTo(cv::Scalar::all(0));
	for (uint32_t feat0 = 0; feat0 < N0; ++feat0) {
		const uint32_t feat1 = feat0ToFeat1[feat0];
		if (feat1 == NO_ID)
			continue;
		std::mt19937 descRng(0xDEADBEEFu ^ feat0);
		for (int b = 0; b < descBytes; ++b) {
			const uint8_t byte = (uint8_t)(descRng() & 0xFF);
			img0.descriptors.at<uint8_t>((int)feat0, b) = byte;
			img1.descriptors.at<uint8_t>((int)feat1, b) = byte;
		}
	}

	// Record which matches span the back hemisphere — these are the features
	// that the pre-Phase-4 (F-matrix) code would have lost, because the
	// fundamental matrix is not geometrically meaningful for spherical pairs
	// and pair.F is empty so the .value() call throws before reaching them.
	size_t numBackHemisphereMatches = 0;
	for (uint32_t feat0 = 0; feat0 < N0; ++feat0) {
		const uint32_t feat1 = feat0ToFeat1[feat0];
		if (feat1 == NO_ID)
			continue;
		const Point3 b0 = img0.pCamera->UnprojectNormalized(Cast<REAL>(img0.keypoints[feat0].pt));
		const Point3 b1 = img1.pCamera->UnprojectNormalized(Cast<REAL>(img1.keypoints[feat1].pt));
		if (b0.z < 0 || b1.z < 0)
			++numBackHemisphereMatches;
	}
	VERBOSE("MatchGeometricSphericalTest: %zu back-hemisphere matches in scene", numBackHemisphereMatches);
	if (numBackHemisphereMatches < 20) {
		VERBOSE("MatchGeometricSphericalTest FAILED: scene has only %zu back-hemisphere matches (need >= 20 to be a meaningful test)", numBackHemisphereMatches);
		return false;
	}

	// Run MatchFeaturesGeometric — the routine KeyframeExtractor calls for every
	// consecutive video keyframe pair. Uses trackedPoints to bootstrap GeometricFilter,
	// then filters descriptor candidates by epipolar distance.
	MatchConfig matchCfg;
	matchCfg.minMatches = 30;
	matchCfg.maxEpipolarError = 5.f;
	matchCfg.matchRatio = 0.9f;
	matchCfg.descriptorsAreBinary = true;
	matchCfg.minTriangulationAngle = 0.f;
	matchCfg.reprojThreshold = 0.f;
	matchCfg.epipoleFilterThreshold = 0.f;
	PairsMatcher matcher(scene, matchCfg);

	ImagePair pair(0, 1);
	const bool geometryEstimated = MatchFeaturesGeometric(
		matcher, img0, img1, trackedPoints1, trackedPoints2, trackStatus, pair, 2.f);

	if (!geometryEstimated) {
		VERBOSE("MatchGeometricSphericalTest FAILED: MatchFeaturesGeometric reported fallback (no geometry estimated)");
		return false;
	}
	if (pair.F.has_value()) {
		VERBOSE("MatchGeometricSphericalTest FAILED: pair.F should be absent for spherical pair, got a value");
		return false;
	}
	if (!pair.E.has_value()) {
		VERBOSE("MatchGeometricSphericalTest FAILED: pair.E missing");
		return false;
	}
	if (!pair.relativePose.has_value()) {
		VERBOSE("MatchGeometricSphericalTest FAILED: pair.relativePose missing");
		return false;
	}

	const Pose3D& poseRec = pair.relativePose.value();
	const REAL angleErr = R2D(ACOS(ComputeAngle(poseRec.R, poseRelGT.R)));
	const Point3 tSimDir = normalized(poseRec.GetT()).dot(normalized(poseRelGT.GetT())) > 0 ? Point3(1,0,0) : Point3(-1,0,0);
	const REAL tSim = ABS(normalized(poseRec.GetT()).dot(normalized(poseRelGT.GetT())));
	(void)tSimDir;

	const unsigned numMatches = pair.GetNumMatches();
	const unsigned numInliers = pair.GetNumInliers();
	VERBOSE("MatchGeometricSphericalTest: matches=%u, inliers=%u, rotation err=%.4f deg, t similarity=%.4f",
	        numMatches, numInliers, angleErr, tSim);

	if (angleErr > REAL(0.5)) {
		VERBOSE("MatchGeometricSphericalTest FAILED: rotation error %.4f deg > 0.5 deg", angleErr);
		return false;
	}
	if (tSim < REAL(0.99)) {
		VERBOSE("MatchGeometricSphericalTest FAILED: translation similarity %.4f < 0.99", tSim);
		return false;
	}
	// Expect at least 80% of tracked correspondences to survive the full pipeline
	// (geometric filter + descriptor filter + epipolar filter).
	const size_t minExpectedInliers = (numTracked * 8) / 10;
	if (numInliers < minExpectedInliers) {
		VERBOSE("MatchGeometricSphericalTest FAILED: only %u inliers < expected %zu (80%% of %zu tracked)",
		        numInliers, minExpectedInliers, numTracked);
		return false;
	}

	// Critical: at least some of the inliers must span the back hemisphere, to
	// prove the angular epipolar filter actually admits z<0 bearings.
	size_t inlierBackHemisphere = 0;
	for (unsigned k = 0; k < numInliers; ++k) {
		const DMatch& m = pair.matches[k];
		const Point3 b0 = img0.pCamera->UnprojectNormalized(Cast<REAL>(img0.keypoints[m.queryIdx].pt));
		const Point3 b1 = img1.pCamera->UnprojectNormalized(Cast<REAL>(img1.keypoints[m.trainIdx].pt));
		if (b0.z < 0 || b1.z < 0)
			++inlierBackHemisphere;
	}
	VERBOSE("MatchGeometricSphericalTest: %zu/%u back-hemisphere inliers", inlierBackHemisphere, numInliers);
	if (inlierBackHemisphere < 10) {
		VERBOSE("MatchGeometricSphericalTest FAILED: only %zu back-hemisphere inliers — epipolar filter is rejecting z<0 bearings",
		        inlierBackHemisphere);
		return false;
	}

	VERBOSE("MatchGeometricSphericalTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

// Two-view geometry test: PairsMatcher and ImagePair matrix operations
bool TwoViewTest()
{
	TD_TIMER_START();
	std::mt19937 rng(123);

	// Generate synthetic scene
	Scene sceneGT, scene;
	SceneConfig cfg;
	std::uniform_real_distribution<REAL> kDist(-0.2, 0.2);
	cfg.cameras.front().k1 = kDist(rng);
	cfg.cameras.front().k2 = kDist(rng);
	if (cfg.cameras.front().k1 * cfg.cameras.front().k2 > 0)
		cfg.cameras.front().k2 *= -1; // ensure k1 and k2 have different signs
	cfg.numImages = 2;
	cfg.numPoints = 600;
	cfg.poseMode = SceneConfig::RANDOM_POSES;
	cfg.generateDescriptors = true;
	cfg.perturbOptions = SceneConfig::PERTURB_KEYPOINTS;
	GenerateTestScene(sceneGT, cfg, &scene);
	// Get relative pose
	const Pose3D pose_rel = scene.images[1] / scene.images[0];
	const PinholeCamera& camGT = *static_cast<PinholeCamera*>(sceneGT.images[0].pCamera);
	PinholeCamera& cam = *static_cast<PinholeCamera*>(scene.images[0].pCamera);
	const Matrix3x3 K = cam.GetK();
	cam.trustIntrinsics = true; // estimate relative pose during matching

	// Test camera distortion projection/unprojection
	{
		Point2 pt_dist(120.f, 40.f);
		REAL depth = 5;
		Point3 X = scene.images[0].UnprojectPoint(pt_dist, depth);
		const auto [pt_proj, valid] = scene.images[0].ProjectPoint(X);
		if (!valid) {
			VERBOSE("TwoViewTest: Distortion projection/unprojection invalid projection");
			return false;
		}
		REAL dist_err = norm(pt_proj - pt_dist);
		if (dist_err > 1e-4) {
			VERBOSE("TwoViewTest: Distortion projection/unprojection error too large (%.6f)", dist_err);
			return false;
		}
	}

	// Test ImagePair matrix operations
	{
		// Test ComposeEssentialMatrix
		const Matrix3x3 E_composed = ImagePair::ComposeEssentialMatrix(pose_rel);
		// Test DecomposeEssentialMatrix (returns one of 4 solutions, needs cheirality check)
		const Pose3D pose_decomposed = ImagePair::DecomposeEssentialMatrix(E_composed);

		// Test RecoverPose with actual point correspondences (unit bearings, each through its own camera)
		std::vector<Eigen::Vector3d> bearings1, bearings2;
		for (const Track& track : scene.tracks) {
			bearings1.emplace_back(scene.images[0].pCamera->UnprojectNormalized(Cast<REAL>(scene.images[0].keypoints[track.observations[0].featureID].pt)));
			bearings2.emplace_back(scene.images[1].pCamera->UnprojectNormalized(Cast<REAL>(scene.images[1].keypoints[track.observations[1].featureID].pt)));
		}
		Pose3D pose_recovered;
		int numInliers = ImagePair::RecoverPose(E_composed, bearings1, bearings2, pose_recovered);
		if (numInliers < 8) {
			VERBOSE("TwoViewTest: RecoverPose returned insufficient inliers (%d)", numInliers);
			return false;
		}

		// Verify decomposed rotation is close to recovered pose
		const REAL angle_decomp_err = ACOS(ComputeAngle(pose_decomposed.R, pose_recovered.R));
		if (angle_decomp_err > 0.1) {
			VERBOSE("TwoViewTest: DecomposeEssentialMatrix rotation error too large (%.4f rad)", angle_decomp_err);
		}

		// Verify recovered rotation is close to ground truth
		const REAL angle_err = ACOS(ComputeAngle(pose_recovered.R, pose_rel.R));
		if (angle_err > 0.1) {
			VERBOSE("TwoViewTest: rotation error too large (%.4f rad)", angle_err);
			return false;
		}
		// Verify recovered translation direction (up to scale and sign)
		const Point3 t_recovered = pose_recovered.GetT();
		const Point3 t_normalized = normalized(pose_rel.GetT());
		const Point3 t_recovered_normalized = normalized(t_recovered);
		const REAL t_similarity = ABS(t_normalized.dot(t_recovered_normalized));
		if (t_similarity < 0.95) {
			VERBOSE("TwoViewTest: translation direction error too large (similarity=%.4f)", t_similarity);
			return false;
		}

		// Test ComposeFundamentalMatrix
		const Matrix3x3 F_composed = ImagePair::ComposeFundamentalMatrix(E_composed, K, K);
		// Test DecomposeFundamentalMatrix
		const Matrix3x3 E_from_F = ImagePair::DecomposeFundamentalMatrix(F_composed, K, K);
		// Verify E and E_from_F are equivalent (up to scale)
		const Matrix3x3 E_from_F_normalized = E_from_F / cv::norm(E_from_F);
		const Matrix3x3 E_normalized = E_composed / cv::norm(E_composed);
		const REAL e_diff = FrobeniusNorm(E_from_F_normalized, E_normalized);
		if (e_diff > 0.01) {
			VERBOSE("TwoViewTest: F->E decomposition error (%.6f)", e_diff);
			return false;
		}

		DEBUG_EXTRA("Matrix operations verified: angle_err=%.4f rad, t_similarity=%.4f, e_diff=%.6f",
		            angle_err, t_similarity, e_diff);
	}

	// Test PairsMatcher::MatchPair
	const float maxEpipolarError = 5.f;
	ImagePair pair;
	{
		MatchConfig config;
		config.descriptorsAreBinary = cfg.binaryDescriptors;
		config.minMatches = 8;
		config.maxEpipolarError = maxEpipolarError;
		PairsMatcher matcher(scene, config);
		if (!matcher.MatchPair(scene.images[0], scene.images[1], pair)) {
			VERBOSE("TwoViewTest: PairsMatcher::MatchPair failed");
			return false;
		}
		if (!pair.HasMatches()) {
			VERBOSE("TwoViewTest: pair has no matches after MatchPair");
			return false;
		}
		if (!pair.HasGeometricVerification()) {
			VERBOSE("TwoViewTest: pair has no geometric verification");
			return false;
		}
		if (!pair.relativePose.has_value()) {
			VERBOSE("TwoViewTest: pair has no relative pose");
			return false;
		}
		// Verify matched relative pose
		const Pose3D& recovered_pose = pair.relativePose.value();
		const REAL angle_err = ACOS(ComputeAngle(recovered_pose.R, pose_rel.R));
		if (angle_err > 0.5) {
			VERBOSE("TwoViewTest: matched rotation error too large (%.4f rad)", angle_err);
			return false;
		}
		const Point3 t_est = recovered_pose.GetT();
		const Point3 t_normalized = normalized(pose_rel.GetT());
		const Point3 t_est_normalized = normalized(t_est);
		const REAL t_similarity = ABS(t_normalized.dot(t_est_normalized));
		if (t_similarity < 0.95) {
			VERBOSE("TwoViewTest: matched translation error too large (similarity=%.4f)", t_similarity);
			return false;
		}
		VERBOSE("TwoViewTest: PairsMatcher found %u matches, %u inliers (angle_err=%.4f rad, t_sim=%.4f)",
		        pair.GetNumMatches(), pair.GetNumInliers(), angle_err, t_similarity);
	}

	// Test CheckEpipolarInliers
	{
		ASSERT(pair.relativePose.has_value());
		const size_t numInliersRelativePose = pair.CheckEpipolarInliers(scene.images[0], scene.images[1], maxEpipolarError);
		if (numInliersRelativePose != pair.GetNumInliers()) {
			VERBOSE("TwoViewTest: CheckEpipolarInliers inconsistent with PairsMatcher for relative pose (%u vs %u)",
			        numInliersRelativePose, pair.GetNumInliers());
			return false;
		}
		pair.relativePose.reset(); // Remove relative pose to test E case
		ASSERT(pair.E.has_value());
		const size_t numInliersE = pair.CheckEpipolarInliers(scene.images[0], scene.images[1], maxEpipolarError);
		if (numInliersE != pair.GetNumInliers()) {
			VERBOSE("TwoViewTest: CheckEpipolarInliers inconsistent with PairsMatcher for essential matrix (%u vs %u)",
			        numInliersE, pair.GetNumInliers());
			return false;
		}
		pair.E.reset(); // Remove E to test F case
		ASSERT(pair.F.has_value());
		const size_t numInliersF = pair.CheckEpipolarInliers(scene.images[0], scene.images[1], maxEpipolarError+2);
		if (numInliersF+50 < pair.GetNumInliers()) { // allow small tolerance
			VERBOSE("TwoViewTest: CheckEpipolarInliers inconsistent with PairsMatcher for fundamental matrix (%u vs %u)",
			        numInliersF, pair.GetNumInliers());
			return false;
		}
	}

	// Test RelativePoseRefine::RefineTwoViewCalibration
	{
		// Create a copy of the camera and pose to refine
		Pose3D pose_rel_refined = pose_rel;

		// Perturb the intrinsics and pose slightly to simulate estimation error
		cam.fx = cam.fy *= 1.03;  // 3% error in focal length
		cam.k1 = cam.k2 = 0;  // large distortion error
		DEBUG("TwoViewTest: Ground truth camera intrinsics: f=%f, cx=%f, cy=%f, k1=%f, k2=%f",
			camGT.fx, camGT.cx, camGT.cy, camGT.k1, camGT.k2);
		DEBUG("TwoViewTest: Distorted camera intrinsics: f=%f, cx=%f, cy=%f, k1=%f, k2=%f",
			cam.fx, cam.cx, cam.cy, cam.k1, cam.k2);

		// Refine calibration
		RelativePoseRefine::Config refine_cfg;
		#ifndef _RELEASE
		refine_cfg.verbose = true;
		#endif
		refine_cfg.robustThreshold = 1.0;
		RelativePoseRefine::Result refine_result;
		const bool refined = RelativePoseRefine::RefineTwoViewCalibration(
			scene.images[0].keypoints, scene.images[1].keypoints, pair.matches,
			cam, pose_rel_refined,
			refine_cfg, &refine_result);
		if (!refined) {
			VERBOSE("TwoViewTest: RelativePoseRefine::RefineTwoViewCalibration failed");
			return false;
		}
		DEBUG("TwoViewTest: Refined camera intrinsics: f=%f, cx=%f, cy=%f, k1=%f, k2=%f",
			cam.fx, cam.cx, cam.cy, cam.k1, cam.k2);

		// Verify refined intrinsics are close to ground truth
		const REAL fx_error = ABS(cam.fx - camGT.fx) / camGT.fx;
		const REAL k1_error = ABS(cam.k1 - camGT.k1);
		const REAL k2_error = ABS(cam.k2 - camGT.k2);
		if (fx_error > 0.05) {  // 5% tolerance
			VERBOSE("TwoViewTest: refined focal length error too large (%.4f%%)", fx_error * 100);
			return false;
		}
		if (k1_error > 0.1 || k2_error > 0.1) {
			VERBOSE("TwoViewTest: refined distortion error too large (k1_err=%.6f, k2_err=%.6f)", k1_error, k2_error);
			return false;
		}

		// Verify refined pose is close to ground truth
		const REAL refined_angle_err = ACOS(ComputeAngle(pose_rel_refined.R, pose_rel.R));
		const Point3 t_refined = normalized(pose_rel_refined.GetT());
		const Point3 t_gt = normalized(pose_rel.GetT());
		const REAL refined_t_similarity = ABS(t_refined.dot(t_gt));
		if (refined_angle_err > 0.5) {
			VERBOSE("TwoViewTest: refined rotation error too large (%.4f rad)", refined_angle_err);
			return false;
		}
		if (refined_t_similarity < 0.95) {
			VERBOSE("TwoViewTest: refined translation error too large (similarity=%.4f)", refined_t_similarity);
			return false;
		}

		VERBOSE("TwoViewTest: RefineTwoViewCalibration: cost %.6f -> %.6f, fx_err=%.2f%%, angle_err=%.4f rad, t_sim=%.4f",
		        refine_result.initialCost, refine_result.finalCost, fx_error * 100, refined_angle_err, refined_t_similarity);
	}

	VERBOSE("TwoViewTest: All tests passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The relative pose recovered from the fundamental matrix (GeometricFilter's 7-DoF F branch over trusted
// intrinsics, then DecomposeFundamentalToPose) for two cameras of different intrinsics and image sizes, and for
// two cameras of one size: the true rotation and translation direction, and nearly every match in front of both
bool FundamentalPoseTwoCamerasTest()
{
	TD_TIMER_START();
	const auto RunCase = [](LPCSTR name, const SceneConfig::CameraSpec& spec2) {
		SceneConfig cfg;
		cfg.cameras = {SceneConfig::CameraSpec{SceneConfig::PINHOLE, 640, 480, 500.0, 330.0, 235.0}, spec2};
		cfg.numImages = 2;
		cfg.numPoints = 400;
		cfg.poseMode = SceneConfig::RANDOM_POSES;
		cfg.generatePairs = true;
		cfg.perturbOptions = SceneConfig::PERTURB_KEYPOINTS;
		Scene sceneGT, scene;
		GenerateTestScene(sceneGT, cfg, &scene);
		ImagePair& pair = scene.pairs.front();
		const Pose3D truth = pair.relativePose.value(); // image 1 relative to image 0, as generated
		const unsigned numTrue = pair.GetNumInliers();
		pair.relativePose.reset();
		pair.E.reset();
		pair.F.reset();
		MatchConfig config;
		config.forceFundamental = true; // the 7-DoF F over the fixture's trusted intrinsics, then its decomposition
		PairsMatcher matcher(scene, config);
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		if (PairsMatcher::SelectGeometryBranch(config, img1, img2) != PairsMatcher::GeometryBranch::FUNDAMENTAL) {
			VERBOSE("FundamentalPoseTwoCamerasTest FAILED (%s): the pair did not take the fundamental branch", name);
			return false;
		}
		if (!matcher.GeometricFilter(img1, img2, pair) || !pair.relativePose.has_value()) {
			VERBOSE("FundamentalPoseTwoCamerasTest FAILED (%s): images %dx%d and %dx%d, no relative pose from F (%s)", name,
				img1.GetWidth(), img1.GetHeight(), img2.GetWidth(), img2.GetHeight(), pair.HasMatches() ? "matches kept" : "matches invalidated");
			return false;
		}
		const REAL rotErr = R2D(ACOS(ComputeAngle(pair.relativePose->R, truth.R)));
		const REAL dirErr = R2D(ACOS(CLAMP(normalized(pair.relativePose->GetT()).dot(normalized(truth.GetT())), REAL(-1), REAL(1))));
		const auto [bearings1, bearings2] = pair.GetMatchedBearings(img1, img2, true);
		Pose3D pose;
		const unsigned numInFront = ImagePair::RecoverPose(pair.E.value(), bearings1, bearings2, pose);
		VERBOSE("FundamentalPoseTwoCamerasTest (%s): images %dx%d f=%g and %dx%d f=%g: rotation error %.3f deg, direction error %.3f deg, "
			"%u of %u true matches verified, %u in front of both cameras, %u after the strict filter", name,
			img1.GetWidth(), img1.GetHeight(), img1.GetK()(0,0), img2.GetWidth(), img2.GetHeight(), img2.GetK()(0,0),
			rotErr, dirErr, pair.GetNumInliers(), numTrue, numInFront, pair.GetNumFilteredInliers());
		if (rotErr > 1.0 || dirErr > 2.0) {
			VERBOSE("FundamentalPoseTwoCamerasTest FAILED (%s): the pose from F is off the truth (bars 1 and 2 deg)", name);
			return false;
		}
		if (pair.GetNumInliers() < numTrue * 95 / 100 || numInFront < numTrue * 95 / 100) {
			VERBOSE("FundamentalPoseTwoCamerasTest FAILED (%s): fewer than 95%% of the %u true matches verified or in front", name, numTrue);
			return false;
		}
		return true;
	};
	// a second camera of another focal, principal point and image size
	if (!RunCase("different sizes", SceneConfig::CameraSpec{SceneConfig::PINHOLE, 1600, 900, 1300.0, 790.0, 460.0}))
		return false;
	// a second camera of the same size and intrinsics (the case the pose was recovered for before)
	if (!RunCase("same size", SceneConfig::CameraSpec{SceneConfig::PINHOLE, 640, 480, 500.0, 330.0, 235.0}))
		return false;
	VERBOSE("FundamentalPoseTwoCamerasTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// FeatureType::DEFAULT is the detector of a default-constructed FeatureExtractionConfig and KeyframeConfig, and a
// default-constructed MatchConfig carries the descriptor kind and the thresholds DefaultsForFeatureType gives that
// detector (the ones the command line applies for it), so a library or Python caller that sets nothing gets the
// pipeline the application runs; the thresholds of each detector are the documented ones
bool DefaultDetectorTest()
{
	VERBOSE("\n=== DefaultDetectorTest: one default detector for the library and the application ===");
	if (FeatureType::DEFAULT != FeatureType::SIFTGPU && FeatureType::DEFAULT != FeatureType::SIFT) {
		VERBOSE("DefaultDetectorTest FAILED: the default detector is neither SIFTGPU nor SIFT");
		return false;
	}
	if (FeatureExtractionConfig().detectorType != FeatureType::DEFAULT || KeyframeConfig().detectorType != FeatureType::DEFAULT) {
		VERBOSE("DefaultDetectorTest FAILED: a configuration does not start from the default detector");
		return false;
	}
	const MatchConfig defaults;
	const MatchConfig expected(MatchConfig().DefaultsForFeatureType(FeatureType::DEFAULT));
	if (defaults.descriptorsAreBinary != expected.descriptorsAreBinary || defaults.maxDescriptorsPerImage != expected.maxDescriptorsPerImage ||
		defaults.matchDistance != expected.matchDistance || defaults.matchRatio != expected.matchRatio || defaults.minMatches != expected.minMatches ||
		defaults.descriptorsAreBinary || defaults.maxDescriptorsPerImage != 1000 || defaults.matchRatio != 0.8f || defaults.minMatches != 15) {
		VERBOSE("DefaultDetectorTest FAILED: the default MatchConfig does not carry the SIFT thresholds of the default detector");
		return false;
	}
	// the thresholds of every detector, named
	struct Expected { FeatureType type; bool binary; unsigned maxDescriptors; float distance; float ratio; unsigned minMatches; };
	const Expected table[] = {
		{FeatureType::AKAZE, true, 2000, 100.f, 0.9f, 50}, {FeatureType::ORB, true, 2000, 64.f, 0.9f, 50},
		{FeatureType::SIFT, false, 1000, FLT_MAX, 0.8f, 15}, {FeatureType::SIFTGPU, false, 1000, FLT_MAX, 0.8f, 15}};
	for (const Expected& e: table) {
		const MatchConfig cfg(MatchConfig().DefaultsForFeatureType(e.type));
		if (cfg.descriptorsAreBinary != e.binary || cfg.maxDescriptorsPerImage != e.maxDescriptors || cfg.matchDistance != e.distance ||
			cfg.matchRatio != e.ratio || cfg.minMatches != e.minMatches) {
			VERBOSE("DefaultDetectorTest FAILED: wrong thresholds for %s", FeatureTypeToString(e.type).c_str());
			return false;
		}
	}
	VERBOSE("DefaultDetectorTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

// PairsMatcher sequential mode test
bool PairMatcherTest()
{
	TD_TIMER_START();
	VERBOSE("--- PairsMatcher Sequential Mode Test ---");

	Scene scene;
	// Generate mock scene with 5 images and guaranteed matches
	SceneConfig scfg;
	scfg.numImages = 5;
	scfg.generateDescriptors = true;
	scfg.numPoints = 100; // Ensure enough points for MinMatches (default 15)
	GenerateTestScene(scene, scfg);

	// Configure sequential matching with overlap 2
	MatchConfig mcfg;
	mcfg.mode = MatchConfig::SEQUENTIAL;
	mcfg.matchSequenceOverlap = 2;
	mcfg.maxEpipolarError = 0; // Disable geometric verification for simplicity (rely on descriptor matches)
	mcfg.minMatches = 10;
	mcfg.matchDistance = FLT_MAX; // Large distance to avoid filtering
	mcfg.descriptorsAreBinary = scfg.binaryDescriptors;

	PairsMatcher matcher(scene, mcfg);
	bool bFatal;
	unsigned numPairs = matcher.Match(bFatal);
	VERBOSE("Matched %u pairs", numPairs);
	if (bFatal) {
		VERBOSE("PairMatcherTest FAILED: matching reported a fatal round failure");
		return false;
	}

	// Check coverage: pairs (i, i+1) and (i, i+2) should exist
	// 5 images (0,1,2,3,4)
	// (0,1), (0,2)
	// (1,2), (1,3)
	// (2,3), (2,4)
	// (3,4), (0,3)
	// (0,4), (1,4)
	// Total 10 pairs
	std::set<uint64_t> expectedPairs;
	const auto AddPair = [&](IIndex A, IIndex B) { expectedPairs.insert(MakePairIdx(A, B).idx); };
	AddPair(0, 1); AddPair(0, 2);
	AddPair(1, 2); AddPair(1, 3);
	AddPair(2, 3); AddPair(2, 4);
	AddPair(3, 4); AddPair(0, 3);
	AddPair(0, 4); AddPair(1, 4);
	if (numPairs != expectedPairs.size()) {
		VERBOSE("PairMatcherTest FAILED: expected %u pairs, got %u", (unsigned)expectedPairs.size(), numPairs);
		return false;
	}

	for (const ImagePair& p : scene.pairs) {
		uint64_t idx = MakePairIdx(p.ID1, p.ID2).idx;
		if (expectedPairs.count(idx) == 0) {
			VERBOSE("PairMatcherTest FAILED: unexpected pair (%u, %u)", p.ID1, p.ID2);
			return false;
		}
		expectedPairs.erase(idx);
	}
	if (!expectedPairs.empty()) {
		VERBOSE("PairMatcherTest FAILED: missing %u expected pairs", (unsigned)expectedPairs.size());
		return false;
	}

	VERBOSE("PairMatcherTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Scene::MatchPairs() must fail the stage rather than report success when matching leaves the
// view graph empty over two or more images -- the state a fatal matching failure leaves behind,
// indistinguishable by return value alone from a scene whose candidates were already all matched.
// A device-allocation failure can not be provoked from a unit test, but the observable state it
// leaves behind can: a scene whose images share no content rejects every candidate on plain
// insufficient matches (an ordinary, non-fatal outcome) and reaches the exact same empty graph.
bool MatchPairsFailureTest()
{
	TD_TIMER_START();
	VERBOSE("--- MatchPairs Failure Signal Test ---");

	// Two images with keypoints but nothing in common: every candidate pair falls under
	// minMatches, so nothing is ever stored and the view graph comes out empty.
	Scene scene;
	SceneConfig scfg;
	scfg.numImages = 3;
	scfg.numPoints = 0;
	scfg.generateDescriptors = true;
	GenerateTestScene(scene, scfg);

	MatchConfig matchCfg;
	matchCfg.mode = MatchConfig::EXHAUSTIVE;
	matchCfg.maxEpipolarError = 0;
	matchCfg.descriptorsAreBinary = scfg.binaryDescriptors;

	if (scene.MatchPairs(matchCfg)) {
		VERBOSE("MatchPairsFailureTest FAILED: MatchPairs() reported success with an empty view graph over %u images",
			(unsigned)scene.images.size());
		return false;
	}
	if (!scene.pairs.empty()) {
		VERBOSE("MatchPairsFailureTest FAILED: %u unexpected pairs stored", (unsigned)scene.pairs.size());
		return false;
	}
	if (scene.status.nState.isSet(Scene::Status::STATE::MATCHED)) {
		VERBOSE("MatchPairsFailureTest FAILED: MATCHED state set despite the reported failure");
		return false;
	}

	// A single image legitimately produces no pairs (nothing to match), and that must stay
	// a no-op, not a reported failure.
	Scene single;
	SceneConfig singleCfg;
	singleCfg.numImages = 1;
	singleCfg.numPoints = 0;
	singleCfg.generateDescriptors = true;
	GenerateTestScene(single, singleCfg);
	if (!single.MatchPairs(matchCfg)) {
		VERBOSE("MatchPairsFailureTest FAILED: a single-image scene must not be reported as a matching failure");
		return false;
	}

	VERBOSE("MatchPairsFailureTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// PairsMatcher::Match() reports a fatal round failure apart from the pair count, because the count
// cannot express it: a round legitimately stores nothing, and a round that fails AFTER an earlier
// one stored pairs leaves a view graph that is truncated rather than empty, which is precisely what
// Scene::MatchPairs' own empty-graph check can not see.
// What this test pins is the half of that contract a unit fixture can reach: the signal must stay
// CLEAR for every non-fatal outcome, so it can not be conflated with "stored nothing". Provoking a
// genuinely fatal round is out of reach here -- the only two are a SiftMatchGPU coordinator that
// fails to initialise (SiftGPU + CUDA build, real device) and a device-slot-pool failure inside
// MatchPairsDense (a loaded RoMa v2 model) -- so the stage-failure side is left to the run-time
// behaviour Scene::MatchPairs implements and MatchPairsFailureTest pins for the empty-graph case.
bool MatchRoundFatalSignalTest()
{
	TD_TIMER_START();
	VERBOSE("--- PairsMatcher Fatal Round Signal Test ---");

	// three images with keypoints but nothing in common: every candidate pair falls under
	// minMatches, so the round completes having stored nothing. An ordinary outcome, and the one a
	// signal derived from the count ("no pairs, so it must have failed") would get wrong
	Scene scene;
	SceneConfig scfg;
	scfg.numImages = 3;
	scfg.numPoints = 0;
	scfg.generateDescriptors = true;
	GenerateTestScene(scene, scfg);

	MatchConfig matchCfg;
	matchCfg.mode = MatchConfig::EXHAUSTIVE;
	matchCfg.maxEpipolarError = 0;
	matchCfg.descriptorsAreBinary = scfg.binaryDescriptors;

	PairsMatcher matcher(scene, matchCfg);
	bool bFatal = true;
	const unsigned numPairs = matcher.Match(bFatal);
	if (numPairs != 0 || !scene.pairs.empty()) {
		VERBOSE("MatchRoundFatalSignalTest FAILED: %u pairs matched among three images with nothing in common", numPairs);
		return false;
	}
	if (bFatal) {
		VERBOSE("MatchRoundFatalSignalTest FAILED: a round that stored nothing is reported as a fatal failure");
		return false;
	}

	// a scene too small to hold a single pair: Match() refuses it, and that refusal must not read as
	// a fatal round either -- Scene::MatchPairs lets a single-image scene through on exactly that
	// (MatchPairsFailureTest pins the stage side of it)
	Scene single;
	SceneConfig singleCfg;
	singleCfg.numImages = 1;
	singleCfg.numPoints = 0;
	singleCfg.generateDescriptors = true;
	GenerateTestScene(single, singleCfg);
	PairsMatcher singleMatcher(single, matchCfg);
	bFatal = true;
	if (singleMatcher.Match(bFatal) != 0 || bFatal) {
		VERBOSE("MatchRoundFatalSignalTest FAILED: a single-image scene is reported as a fatal round failure");
		return false;
	}

	VERBOSE("MatchRoundFatalSignalTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool PreMatchTest()
{
	TD_TIMER_START();
	VERBOSE("--- PairsMatcher Pre-Matching Test ---");

	Scene scene;
	// Generate mock camera
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), 1000, 1000, 320, 240));
	// Generate mock scene with 3 images
	scene.images.resize(3);
	for (int i = 0; i < 3; ++i) {
		scene.images[i].ID = i;
		scene.images[i].fileName = "img" + std::to_string(i) + ".jpg";
		scene.images[i].pCamera = scene.cameras[0];
		scene.images[i].cameraID = 0;
		scene.images[i].keypoints.resize(20);
		scene.images[i].descriptors.create(20, 128, CV_8U);
		// Fill with random noise first
		cv::randu(scene.images[i].descriptors, cv::Scalar(0), cv::Scalar(255));
	}

	// Make Img0 and Img1 matches (share 15 descriptors)
	// Make Img0 and Img2 weak matches (share 2 descriptors)
	for (int i = 0; i < 15; ++i) {
		// Common pattern for 0-1
		for (int k = 0; k < 128; ++k) {
			uint8_t val = (uint8_t)(i * 10 + k);
			scene.images[0].descriptors.at<uint8_t>(i, k) = val;
			scene.images[1].descriptors.at<uint8_t>(i, k) = val;
		}
	}
	for (int i = 0; i < 2; ++i) {
		// Common pattern for 0-2 (different from above)
		for (int k = 0; k < 128; ++k) {
			uint8_t val = (uint8_t)(200 + i * 10 + k);
			scene.images[0].descriptors.at<uint8_t>(18+i, k) = val; // Use last slots of 0
			scene.images[2].descriptors.at<uint8_t>(i, k) = val;
		}
	}

	MatchConfig mcfg;
	mcfg.mode = MatchConfig::EXHAUSTIVE;
	mcfg.preMatchThreshold = 5; // Require at least 5 matches
	mcfg.descriptorsAreBinary = false; // Our noise generation is simple bytes (SIFT-like)
	mcfg.minMatches = 15; // Require at least 15 matches to keep pair
	mcfg.maxEpipolarError = 0; // Disable geometric verification (no cameras)

	PairsMatcher matcher(scene, mcfg);
	bool bFatal;
	unsigned numPairs = matcher.Match(bFatal);

	VERBOSE("Matched %u pairs", numPairs);
	if (bFatal) {
		VERBOSE("PreMatchTest FAILED: matching reported a fatal round failure");
		return false;
	}

	// Pair (0,1) needs >= 5 matches -> should exist
	// Pair (0,2) needs >= 5 matches (has 2) -> should be filtered out
	// Pair (1,2) -> random noise -> likely 0 matches -> filtered out

	bool pair01 = (scene.FindPair(0, 1) != nullptr);
	bool pair02 = (scene.FindPair(0, 2) != nullptr);

	if (!pair01) {
		VERBOSE("PreMatchTest FAILED: expected pair (0,1) to be kept");
		return false;
	}
	if (pair02) {
		VERBOSE("PreMatchTest FAILED: expected pair (0,2) to be filtered (weak matches)");
		return false;
	}

	VERBOSE("PreMatchTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// RETRIEVAL match-mode test: candidate selection ranks purely by the global descriptors --
// VOCABULARY and RETRIEVAL name two backends and neither consults a gate to borrow the
// other's -- a missing descriptor is a hard error rather than a vocabulary-tree fallback, and
// the mode dispatches correctly end-to-end through Match()
bool RetrievalModeTest()
{
	TD_TIMER_START();

	// three clusters of four images, same construction as GlobalDescriptorsQueryTest
	Scene scene;
	const int D = 64;
	std::mt19937 rng(31);
	std::normal_distribution<float> noise(0.f, 0.05f);
	for (IIndex i = 0; i < 12; ++i) {
		Image& img = scene.images.emplace_back(i, String::FormatString("%02u.jpg", i));
		img.keypoints.resize(10);
		img.descriptors = cv::Mat::zeros(10, 32, CV_8U); // PairsMatcher expects descriptors to exist
		img.globalDescriptor.create(1, D, CV_32F);
		for (int c = 0; c < D; ++c)
			img.globalDescriptor.at<float>(c) = (c/8 == (int)(i/4) ? 1.f : 0.f) + noise(rng);
		img.globalDescriptor /= cv::norm(img.globalDescriptor);
	}
	scene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);

	MatchConfig matchCfg;
	matchCfg.mode = MatchConfig::RETRIEVAL;
	matchCfg.maxPairsPerImage = 3;

	// RETRIEVAL ranks by the descriptors regardless of the DenseMatchConfig passed to SetDense: the
	// mode is its own opt-in and never consults a gate (design decision 10)
	PairsMatcher matcher(scene, matchCfg);
	matcher.SetDense(NULL, DenseMatchConfig());
	const PairIdxArr pairs = matcher.CollectRetrievalPairs(2);
	if (pairs.empty()) {
		VERBOSE("RetrievalModeTest FAILED: RETRIEVAL produced no pairs despite a fully described scene");
		return false;
	}
	DisjointSet<IIndex> components(12);
	unsigned numCross = 0;
	for (const PairIdx& p : pairs) {
		components.Union(p.i, p.j);
		numCross += (p.i/4 != p.j/4);
	}
	for (IIndex i = 1; i < 12; ++i) {
		if (components.Find(i) != components.Find(0)) {
			VERBOSE("RetrievalModeTest FAILED: view graph split");
			return false;
		}
	}
	if (numCross > 2) {
		VERBOSE("RetrievalModeTest FAILED: %u cross-cluster pairs", numCross);
		return false;
	}

	// a scene missing a descriptor on one image is a hard, actionable error: no pairs, no
	// crash — and, by construction (PairsMatcher::Match/EnsureGlobalDescriptorsIndex), no
	// vocabulary tree or PreMatch ever runs to paper over it
	Scene brokenScene;
	for (IIndex i = 0; i < 4; ++i) {
		Image& img = brokenScene.images.emplace_back(i, String::FormatString("%02u.jpg", i));
		img.keypoints.resize(10);
		img.descriptors = cv::Mat::zeros(10, 32, CV_8U);
		if (i != 2) { // image 2 is missing its global descriptor
			img.globalDescriptor.create(1, D, CV_32F);
			img.globalDescriptor.setTo(1.f / std::sqrt((float)D));
		}
	}
	PairsMatcher brokenMatcher(brokenScene, matchCfg);
	brokenMatcher.SetDense(NULL, DenseMatchConfig());
	if (!brokenMatcher.CollectRetrievalPairs(2).empty()) {
		VERBOSE("RetrievalModeTest FAILED: a missing descriptor should error out, not silently produce pairs");
		return false;
	}

	// end-to-end through Match(): a small circular-arrangement scene where every pair has real
	// covisible matches (PairMatcherTest's setup), so whichever subset RETRIEVAL selects still
	// matches for real; the budget is >= 10 so verification feedback also engages
	Scene e2eScene;
	SceneConfig scfg;
	scfg.numImages = 5;
	scfg.generateDescriptors = true;
	scfg.numPoints = 100;
	GenerateTestScene(e2eScene, scfg);
	FOREACH(i, e2eScene.images) {
		Image& img = e2eScene.images[i];
		img.globalDescriptor.create(1, D, CV_32F);
		for (int c = 0; c < D; ++c)
			img.globalDescriptor.at<float>(c) = (c/8 == (int)(i/2) ? 1.f : 0.f) + noise(rng);
		img.globalDescriptor /= cv::norm(img.globalDescriptor);
	}
	e2eScene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);
	MatchConfig e2eCfg;
	e2eCfg.mode = MatchConfig::RETRIEVAL;
	e2eCfg.maxPairsPerImage = 20; // >= 10 so verification feedback engages
	e2eCfg.maxEpipolarError = 0; // rely on descriptor matches only, as PairMatcherTest does
	e2eCfg.minMatches = 10;
	e2eCfg.matchDistance = FLT_MAX;
	e2eCfg.descriptorsAreBinary = scfg.binaryDescriptors;
	PairsMatcher e2eMatcher(e2eScene, e2eCfg);
	bool bFatal;
	const unsigned numMatched = e2eMatcher.Match(bFatal);
	if (numMatched == 0 || bFatal) {
		VERBOSE("RetrievalModeTest FAILED: end-to-end RETRIEVAL match produced no pairs%s",
			bFatal ? " (fatal round failure)" : "");
		return false;
	}

	VERBOSE("RetrievalModeTest PASSED: %u candidate pairs (%u cross-cluster), %u end-to-end matched pairs (%s)",
		(unsigned)pairs.size(), numCross, numMatched, TD_TIMER_GET_FMT().c_str());
	return true;
}

// VOCABULARY/RETRIEVAL backend-isolation test: a 12-image scene deliberately given two
// disagreeing truths -- the global descriptors cluster {0-3}{4-7}{8-11} (i/4), exactly as in
// RetrievalModeTest, while the local (binary) descriptors cluster {0,4,8}{1,5,9}{2,6,10}{3,7,11}
// (i%4) via four well-separated Hamming prototypes. VOCABULARY's matcher opts into ROMA2
// retrieval (enabled=true) exactly as the deleted substitution required, so if its candidate
// pairs still followed the i/4 partition, that could only mean the deleted global-descriptor
// substitution (or some new equivalent of it) is back. Asserted on the produced pair sets
// themselves, never on log text.
bool VocabularyIgnoresGlobalDescriptorsTest()
{
	TD_TIMER_START();

	Scene scene;
	const int D = 64;
	const int descriptorBytes = 32; // 256-bit ORB-like
	const unsigned numDescriptorsPerImage = 300;
	std::mt19937 rng(41);
	std::normal_distribution<float> noise(0.f, 0.05f);
	std::uniform_int_distribution<int> bitPos(0, descriptorBytes * 8 - 1);
	// four local-descriptor prototypes, pairwise far apart in Hamming distance
	std::vector<std::vector<uint8_t>> localProto(4);
	for (int c = 0; c < 4; ++c)
		localProto[c].assign(descriptorBytes, (uint8_t)(0x11 * (c + 1)));
	for (IIndex i = 0; i < 12; ++i) {
		Image& img = scene.images.emplace_back(i, String::FormatString("%02u.jpg", i));
		img.keypoints.resize(numDescriptorsPerImage);
		img.descriptors.create((int)numDescriptorsPerImage, descriptorBytes, CV_8U);
		const std::vector<uint8_t>& proto = localProto[i % 4]; // local clustering: i%4
		for (unsigned r = 0; r < numDescriptorsPerImage; ++r) {
			uint8_t* row = img.descriptors.ptr<uint8_t>((int)r);
			std::memcpy(row, proto.data(), proto.size());
			for (int f = 0; f < 8; ++f) { // a handful of flipped bits, well inside the inter-prototype gap
				const int b = bitPos(rng);
				row[b / 8] ^= (uint8_t)(1u << (b % 8));
			}
		}
		img.globalDescriptor.create(1, D, CV_32F);
		for (int c = 0; c < D; ++c) // global clustering: i/4 (same construction as RetrievalModeTest)
			img.globalDescriptor.at<float>(c) = (c/8 == (int)(i/4) ? 1.f : 0.f) + noise(rng);
		img.globalDescriptor /= cv::norm(img.globalDescriptor);
	}
	scene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);

	MatchConfig matchCfg;
	matchCfg.mode = MatchConfig::VOCABULARY;
	matchCfg.descriptorsAreBinary = true;
	matchCfg.maxPairsPerImage = 3;
	PairsMatcher matcher(scene, matchCfg);
	// opt into ROMA2 (enabled=true) exactly as the deleted substitution required: under the old
	// code this alone was enough to take VOCABULARY over with the global descriptors, no model
	// needed
	DenseMatchConfig denseCfg;
	denseCfg.enabled = true;
	matcher.SetDense(NULL, denseCfg);
	const PairIdxArr vocabPairs = matcher.CollectVocabularyPairs(2);
	if (vocabPairs.empty()) {
		VERBOSE("VocabularyIgnoresGlobalDescriptorsTest FAILED: VOCABULARY produced no candidate pairs");
		return false;
	}

	MatchConfig retrievalCfg = matchCfg;
	retrievalCfg.mode = MatchConfig::RETRIEVAL;
	PairsMatcher retrievalMatcher(scene, retrievalCfg);
	const PairIdxArr retrievalPairs = retrievalMatcher.CollectRetrievalPairs(2);
	if (retrievalPairs.empty()) {
		VERBOSE("VocabularyIgnoresGlobalDescriptorsTest FAILED: RETRIEVAL produced no candidate pairs");
		return false;
	}

	std::set<uint64_t> vocabSet, retrievalSet;
	for (const PairIdx& p : vocabPairs) vocabSet.insert(p.idx);
	for (const PairIdx& p : retrievalPairs) retrievalSet.insert(p.idx);
	if (vocabSet == retrievalSet) {
		VERBOSE("VocabularyIgnoresGlobalDescriptorsTest FAILED: VOCABULARY and RETRIEVAL selected the identical "
			"pair set on a scene built so the local and global clusterings disagree -- the global descriptors "
			"are still leaking into VOCABULARY's ranking");
		return false;
	}

	// stronger: VOCABULARY's own pairs must actually follow the local (i%4, 4 clusters of 3)
	// clustering it was given, not merely "differ from RETRIEVAL" for some unrelated reason.
	// CollectFusedRetrievalPairs bridges any connected components its mutual-top-K selection
	// left disjoint (see its step 4), so up to numClusters-1 pairs are legitimately cross-cluster;
	// every remaining pair must stay inside its local cluster
	unsigned vocabCross = 0;
	for (const PairIdx& p : vocabPairs)
		vocabCross += (p.i % 4 != p.j % 4);
	if (vocabCross > 3) {
		VERBOSE("VocabularyIgnoresGlobalDescriptorsTest FAILED: %u/%u VOCABULARY pairs cross the local "
			"descriptor clustering (more than the 3 connectivity bridges 4 clusters can need)",
			vocabCross, (unsigned)vocabPairs.size());
		return false;
	}
	// and RETRIEVAL's pairs must follow the global (i/4, 3 clusters of 4) clustering it was
	// given, up to the 2 connectivity bridges 3 clusters can need
	unsigned retrievalCross = 0;
	for (const PairIdx& p : retrievalPairs)
		retrievalCross += (p.i / 4 != p.j / 4);
	if (retrievalCross > 2) {
		VERBOSE("VocabularyIgnoresGlobalDescriptorsTest FAILED: %u/%u RETRIEVAL pairs cross the global "
			"descriptor clustering (more than the 2 connectivity bridges 3 clusters can need)",
			retrievalCross, (unsigned)retrievalPairs.size());
		return false;
	}

	VERBOSE("VocabularyIgnoresGlobalDescriptorsTest PASSED: %u VOCABULARY pairs (local clustering), "
		"%u RETRIEVAL pairs (global clustering) (%s)",
		(unsigned)vocabPairs.size(), (unsigned)retrievalPairs.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

} // namespace SFM
