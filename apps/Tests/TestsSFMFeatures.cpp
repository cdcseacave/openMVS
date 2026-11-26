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
#include "../../libs/Math/Camera/CameraUndistort.h"
#if defined(_USE_SIFTGPU) && defined(SIFTGPU_CUDA)
#include <siftgpu/SiftGPU.h>
#endif
#include "../../libs/SFM/InterfaceMVS.h"
#include "../../libs/MVS.h"
#include <nlohmann/json.hpp>
#include "Tests.h"
#include "TestsSFMFramework.h"
#include <filesystem>
#include <chrono>
#include <random>
#include <numeric>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestSFM "));

namespace SFM {

// The vocabulary tree on five images of two descriptor clusters (images 0 and 1 near the first, 2 and 3 near the second, 4 a
// mix of both), for quantized RootSIFT-like descriptors (CV_8U, L2) and for binary ones (Hamming): built, the twin of image 0
// ranks among its top; saved, released and loaded, the tree holds no postings (queries are empty) until the scene is indexed
// again, after which the twin ranks again
bool VocabularyTreeTest()
{
	TestScope scope("VocabularyTreeTest");
	const ScopedTempDir tmpDir(_T("VocabularyTreeTest"));
	TEST_CHECK(tmpDir.IsValid(), "no temporary folder");
	constexpr unsigned numImages = 5, numDescriptorsPerImage = 300;
	std::mt19937 rng(123);
	// rows of descriptors around the center of one of the two clusters
	const auto QuantizedDescriptors = [&rng](cv::Mat& dst, unsigned cluster, unsigned nRows) {
		const std::vector<uint8_t> center(128, cluster ? 200 : 60);
		dst.create((int)nRows, (int)center.size(), CV_8U);
		std::normal_distribution<float> noise(0.f, 8.f);
		for (unsigned r = 0; r < nRows; ++r) {
			uint8_t* row = dst.ptr<uint8_t>((int)r);
			for (size_t c = 0; c < center.size(); ++c)
				row[c] = (uint8_t)CLAMP(ROUND2INT(center[c] + noise(rng)), 0, 255);
		}
	};
	// binary descriptors around the prototype of one of the two clusters (a few flipped bits)
	const auto BinaryDescriptors = [&rng](cv::Mat& dst, unsigned cluster, unsigned nRows) {
		const std::vector<uint8_t> proto(32, cluster ? 0xF0 : 0x0F); // 256-bit ORB-like
		dst.create((int)nRows, (int)proto.size(), CV_8U);
		std::uniform_int_distribution<int> bitPos(0, (int)proto.size() * 8 - 1);
		for (unsigned r = 0; r < nRows; ++r) {
			uint8_t* row = dst.ptr<uint8_t>((int)r);
			std::memcpy(row, proto.data(), proto.size());
			for (int f = 0; f < 8; ++f) {
				const int b = bitPos(rng);
				row[b / 8] ^= (uint8_t)(1u << (b % 8));
			}
		}
	};
	const auto RankedImage1 = [](const VocabularyTree& vocab, const Image& query) {
		for (const auto& p : vocab.Query(query, 3, 0.f))
			if (p.first == 1)
				return true;
		return false;
	};
	for (const bool binary : {false, true}) {
		const char* const name = binary ? "binary" : "quantized";
		const std::function<void(cv::Mat&, unsigned, unsigned)> describe = binary ?
			std::function<void(cv::Mat&, unsigned, unsigned)>(BinaryDescriptors) : QuantizedDescriptors;
		Scene scene;
		for (unsigned i = 0; i < numImages; ++i) {
			Image& img = scene.images.emplace_back((IIndex)i, "");
			img.keypoints.resize(numDescriptorsPerImage);
			if (i < 4)
				describe(img.descriptors, i / 2, numDescriptorsPerImage);
			else {
				cv::Mat A, B;
				describe(A, 0, numDescriptorsPerImage / 2);
				describe(B, 1, numDescriptorsPerImage - numDescriptorsPerImage / 2);
				cv::vconcat(A, B, img.descriptors);
			}
		}
		VocabularyTree vocab;
		VocabularyTree::Config cfg;
		cfg.descriptorsAreBinary = binary;
		cfg.K = 8;
		cfg.L = 5;
		cfg.maxKMeansIters = 8;
		TEST_CHECK(vocab.Build(scene, cfg), "%s: Build failed", name);
		TEST_CHECK(!vocab.Query(scene.images[0], 3, 0.f).empty(), "%s: Query returned empty", name);
		TEST_CHECK(RankedImage1(vocab, scene.images[0]), "%s: expected img1 among the top for img0", name);
		const String savePath = tmpDir(binary ? _T("vocab_b.bin") : _T("vocab_q.bin"));
		TEST_CHECK(vocab.Save(savePath), "%s: Save failed", name);
		vocab.Release();
		TEST_CHECK(vocab.Load(savePath), "%s: Load failed", name);
		TEST_CHECK(vocab.Query(scene.images[0], 3, 0.f).empty(), "%s: a loaded tree without its database should return empty", name);
		TEST_CHECK(vocab.Index(scene), "%s: Index after load failed", name);
		TEST_CHECK(RankedImage1(vocab, scene.images[0]), "%s: img1 not found after the reload", name);
	}
	return scope.Pass("quantized and binary trees: built, saved, loaded and indexed again, the twin of image 0 ranks each time");
}

// BuildTracks on components holding one image twice (TrackConflictConfig): the cut removes the least-supported link on the
// path between the two keypoints and keeps the corroborated side where the pair-order veto keeps whichever arrived first,
// two keypoints within the merge distance are one feature and the one with more links stays, a component above the size
// bar takes the veto, the cut can be switched off, and a conflict-free scene is untouched
bool BuildTracksConflictTest()
{
	TestScope scope("BuildTracksConflictTest");
	// A scene of one-keypoint images (image A gets two) with weighted pairs holding one match each;
	// the pairs are appended in decreasing weight, the order BuildTracks receives them in, and
	// given with ID1 < ID2, the order ImagePair keeps (its constructor swaps the IDs, not the match).
	struct Link { IIndex ID1, ID2; int q, t; float weight; };
	typedef std::vector<std::vector<Point2f>> Keypoints;
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
	const auto Describe = [](const Tracks& tracks) {
		String s = String::FormatString("%u tracks:", (unsigned)tracks.size());
		for (const auto& track : tracks) {
			s += " [";
			for (const auto& [img, feat] : track)
				s += String::FormatString(" (%u,%u)", img, feat);
			s += " ]";
		}
		return s;
	};
	struct Case {
		const char* what;
		const Keypoints& keypoints;
		const std::vector<Link>& links;
		std::function<void(TrackConflictConfig&)> configure;
		Tracks expected;
	};
	const auto RunCase = [&](const Case& c) -> bool {
		Scene scene;
		FOREACH(i, c.keypoints) {
			Image& img = scene.images.emplace_back((IIndex)i, String::FormatString("%u.jpg", i));
			for (const Point2f& pt : c.keypoints[i])
				img.keypoints.emplace_back(pt.x, pt.y, 3.f, -1.f, 0.1f);
		}
		for (const Link& link : c.links) {
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
		TrackConflictConfig conflict;
		c.configure(conflict);
		BuildTracks(scene, 0.f, conflict);
		const Tracks got = TracksOf(scene);
		TEST_CHECK(got == c.expected, "%s: got %s, expected %s", c.what, Describe(got).c_str(), Describe(c.expected).c_str());
		return true;
	};
	const auto Default = [](TrackConflictConfig&) {};

	// Image A holds a1 (keypoint 0) and a2 (keypoint 1), far apart; B, C, D one keypoint each.
	// The links, heaviest first: a2-c (6), b-c (5), a1-b (4), a1-d (4), b-d (4). The plain union
	// joins all five keypoints and image A is in twice. The triangle a1-b-d corroborates a1-b, the
	// chain b-c-a2 has no triangle: on the path a1-b-c-a2 the least-supported links are b-c and c-a2
	// (no triangle), the lighter of them b-c is cut, and b stays with the corroborated side.
	const Keypoints keypointsA = {{Point2f(10.f, 10.f), Point2f(100.f, 100.f)}, {Point2f(20.f, 20.f)}, {Point2f(30.f, 30.f)}, {Point2f(40.f, 40.f)}};
	const std::vector<Link> linksA = {{0, 2, 1, 0, 6.f}, {1, 2, 0, 0, 5.f}, {0, 1, 0, 0, 4.f}, {0, 3, 0, 0, 4.f}, {1, 3, 0, 0, 4.f}};
	// the veto in pair order takes a2-c and b-c first, then refuses a1-b and b-d: b stays with a2
	const Tracks vetoA = {{{0, 0}, {3, 0}}, {{0, 1}, {1, 0}, {2, 0}}};
	// a1 and a2 two pixels apart, linked a1-b, a2-c, b-c: one feature seen twice. Within the merge
	// distance the component is one track and a1, first of two equally linked keypoints, stands
	// for image A; with the merge off the conflict is cut: all links equally unsupported and
	// weighted, the earliest, a1-b, goes, and a1 alone is no track.
	const Keypoints keypointsB = {{Point2f(10.f, 10.f), Point2f(12.f, 10.f)}, {Point2f(20.f, 20.f)}, {Point2f(30.f, 30.f)}};
	const std::vector<Link> linksB = {{0, 1, 0, 0, 3.f}, {0, 2, 1, 0, 3.f}, {1, 2, 0, 0, 3.f}};
	const Tracks mergedB = {{{0, 0}, {1, 0}, {2, 0}}};
	// Three keypoints of image A: a1 and a2 (keypoints 0, 1) 2.9 px apart, a3 (keypoint 2) 2.9 px
	// from a1 and 5.8 from a2. The links, heaviest first: a3-b, a1-c, a2-d, c-d, b-c. The veto
	// merges a1 and a2 through c-d, then refuses b-c: a3 is one feature with a1 but not with a2.
	// The cut sees the same three in one component, and b-c, unsupported and lightest on the path
	// from a3 to a2, goes. Either way a1, the first of two equally linked, stands for A.
	const Keypoints keypointsD = {{Point2f(10.f, 10.f), Point2f(12.9f, 10.f), Point2f(7.1f, 10.f)}, {Point2f(20.f, 20.f)}, {Point2f(30.f, 30.f)}, {Point2f(40.f, 40.f)}};
	const std::vector<Link> linksD = {{0, 1, 2, 0, 6.f}, {0, 2, 0, 0, 5.f}, {0, 3, 1, 0, 5.f}, {2, 3, 0, 0, 4.f}, {1, 2, 0, 0, 3.f}};
	const Tracks tracksD = {{{0, 0}, {2, 0}, {3, 0}}, {{0, 2}, {1, 0}}};
	// a chain a-b-c without a conflict is one track of three, under either rule
	const Keypoints keypointsC = {{Point2f(10.f, 10.f)}, {Point2f(20.f, 20.f)}, {Point2f(30.f, 30.f)}};
	const std::vector<Link> linksC = {{0, 1, 0, 0, 3.f}, {1, 2, 0, 0, 3.f}};

	const Case cases[] = {
		{"the cut keeps b on the corroborated side", keypointsA, linksA, Default, {{{0, 0}, {1, 0}, {3, 0}}, {{0, 1}, {2, 0}}}},
		{"without the cut the veto keeps the first arrival", keypointsA, linksA, [](TrackConflictConfig& c) { c.cut = false; }, vetoA},
		{"a component above the size bar (five keypoints against 4) takes the veto", keypointsA, linksA, [](TrackConflictConfig& c) { c.maxComponentSize = 4; }, vetoA},
		{"two keypoints within the merge distance are one feature", keypointsB, linksB, Default, mergedB},
		{"with the merge off the near duplicates are cut apart", keypointsB, linksB, [](TrackConflictConfig& c) { c.mergeDistance = 0.f; }, {{{0, 1}, {1, 0}, {2, 0}}}},
		{"the veto merges two keypoints within the merge distance too", keypointsB, linksB, [](TrackConflictConfig& c) { c.cut = false; }, mergedB},
		{"the cut merges one feature and not its far neighbour", keypointsD, linksD, Default, tracksD},
		{"the veto merges one feature and not its far neighbour", keypointsD, linksD, [](TrackConflictConfig& c) { c.cut = false; }, tracksD},
		{"a conflict-free chain is one track", keypointsC, linksC, Default, mergedB},
	};
	for (const Case& c : cases)
		if (!RunCase(c))
			return false;
	return scope.Pass("%u conflict cases: the cut and the veto, the merge distance, the size bar, and a conflict-free chain", (unsigned)(sizeof(cases)/sizeof(cases[0])));
}

// Twelve images, the scene of the retrieval tests, that hold two disagreeing truths: the global descriptors cluster {0-3}
// {4-7}{8-11} (i/4, each cluster occupying its own eighth of the descriptor), the local ones, 300 binary descriptors of four
// well-separated Hamming prototypes with a handful of flipped bits, cluster {0,4,8}{1,5,9}{2,6,10}{3,7,11} (i%4)
static void BuildClusterScene(Scene& scene, uint32_t seed)
{
	constexpr int D = 64, descriptorBytes = 32, numDescriptorsPerImage = 300;
	std::mt19937 rng(seed);
	std::normal_distribution<float> noise(0.f, 0.05f);
	std::uniform_int_distribution<int> bitPos(0, descriptorBytes * 8 - 1);
	for (IIndex i = 0; i < 12; ++i) {
		Image& img = scene.images.emplace_back(i, String::FormatString("%02u.jpg", i));
		img.keypoints.resize(numDescriptorsPerImage);
		img.descriptors.create(numDescriptorsPerImage, descriptorBytes, CV_8U);
		for (int r = 0; r < numDescriptorsPerImage; ++r) {
			uint8_t* row = img.descriptors.ptr<uint8_t>(r);
			std::memset(row, 0x11 * (int)(i % 4 + 1), descriptorBytes); // the prototype of local cluster i%4
			for (int f = 0; f < 8; ++f) { // flipped bits, well inside the inter-prototype gap
				const int b = bitPos(rng);
				row[b / 8] ^= (uint8_t)(1u << (b % 8));
			}
		}
		img.globalDescriptor.create(1, D, CV_32F);
		for (int c = 0; c < D; ++c)
			img.globalDescriptor.at<float>(c) = (c/8 == (int)(i/4) ? 1.f : 0.f) + noise(rng);
		img.globalDescriptor /= cv::norm(img.globalDescriptor);
	}
	scene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);
}

// A small circular arrangement where every pair has real covisible matches, so whichever subset a match mode selects still
// matches for real (the pair-selection and retrieval tests that run Match())
static void BuildCovisibleScene(Scene& scene)
{
	SceneConfig scfg;
	scfg.numImages = 5;
	scfg.generateDescriptors = true;
	scfg.numPoints = 100; // enough points for the default minMatches of 15
	GenerateTestScene(scene, scfg);
}

// Global-descriptor retrieval, on the twelve images of BuildClusterScene: the cosine ranking of the per-image descriptors and
// its deterministic tie order, PairsMatcher's RETRIEVAL-mode pair selection over the same descriptors (a connected view
// graph joined by at most two cross-cluster bridges, whatever the DenseMatchConfig passed to SetDense: the mode is its own
// opt-in and never consults a gate), a missing descriptor a hard error rather than a vocabulary-tree fallback, the mode
// dispatching end-to-end through Match(), VOCABULARY and RETRIEVAL naming two backends that neither borrows from the other
// (on this scene the pair sets follow the local and the global clustering respectively), and the .sfm round-trip of the descriptors
bool RetrievalTest()
{
	TestScope scope("RetrievalTest");
	Scene scene;
	BuildClusterScene(scene, 41);

	// the top-3 neighbors of an image all belong to its own cluster
	GlobalDescriptors index;
	TEST_CHECK(index.Build(scene) && index.Dim() == 64 && index.Size() == 12, "building the global descriptor index");
	for (const auto& [id, score] : index.Query(5, 3))
		TEST_CHECK(id / 4 == 1 && id != 5, "image %u ranked for image 5", id);

	// RETRIEVAL ranks the pair selection through these same global descriptors and still returns a connected view graph,
	// joined by at most two cross-cluster bridges
	MatchConfig retrievalCfg;
	retrievalCfg.mode = MatchConfig::RETRIEVAL;
	retrievalCfg.descriptorsAreBinary = true;
	retrievalCfg.maxPairsPerImage = 3;
	const auto CheckConnected = [&](const PairIdxArr& pairs, const char* what) -> bool {
		TEST_CHECK(!pairs.empty(), "%s produced no pairs despite a fully described scene", what);
		DisjointSet<IIndex> components(12);
		unsigned numCross = 0;
		for (const PairIdx& p : pairs) {
			components.Union(p.i, p.j);
			numCross += (p.i/4 != p.j/4);
		}
		for (IIndex i = 1; i < 12; ++i)
			TEST_CHECK(components.Find(i) == components.Find(0), "%s: the view graph split", what);
		TEST_CHECK(numCross <= 2, "%s: %u cross-cluster pairs", what, numCross);
		return true;
	};
	PairsMatcher retrievalMatcher(scene, retrievalCfg);
	const PairIdxArr retrievalPairs = retrievalMatcher.CollectRetrievalPairs(2);
	if (!CheckConnected(retrievalPairs, "RETRIEVAL"))
		return false;
	PairsMatcher denseMatcher(scene, retrievalCfg);
	denseMatcher.SetDense(NULL, DenseMatchConfig());
	if (!CheckConnected(denseMatcher.CollectRetrievalPairs(2), "RETRIEVAL with a dense configuration"))
		return false;

	// a scene missing a descriptor on one image is a hard, actionable error: no pairs, no crash -- and, by construction
	// (PairsMatcher::Match/EnsureGlobalDescriptorsIndex), no vocabulary tree or PreMatch ever runs to paper over it
	{
		Scene brokenScene;
		for (IIndex i = 0; i < 4; ++i) {
			Image& img = brokenScene.images.emplace_back(i, String::FormatString("%02u.jpg", i));
			img.keypoints.resize(10);
			img.descriptors = cv::Mat::zeros(10, 32, CV_8U);
			if (i != 2) { // image 2 is missing its global descriptor
				img.globalDescriptor.create(1, 64, CV_32F);
				img.globalDescriptor.setTo(1.f / std::sqrt(64.f));
			}
		}
		PairsMatcher brokenMatcher(brokenScene, retrievalCfg);
		brokenMatcher.SetDense(NULL, DenseMatchConfig());
		TEST_CHECK(brokenMatcher.CollectRetrievalPairs(2).empty(), "a missing descriptor should error out, not silently produce pairs");
	}

	// end-to-end through Match(), on a scene where every pair has real covisible matches; the budget is >= 10 so the
	// verification feedback also engages
	unsigned numMatched = 0;
	{
		Scene e2eScene;
		BuildCovisibleScene(e2eScene);
		std::mt19937 rng(31);
		std::normal_distribution<float> noise(0.f, 0.05f);
		FOREACH(i, e2eScene.images) {
			Image& img = e2eScene.images[i];
			img.globalDescriptor.create(1, 64, CV_32F);
			for (int c = 0; c < 64; ++c)
				img.globalDescriptor.at<float>(c) = (c/8 == (int)(i/2) ? 1.f : 0.f) + noise(rng);
			img.globalDescriptor /= cv::norm(img.globalDescriptor);
		}
		e2eScene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);
		MatchConfig e2eCfg;
		e2eCfg.mode = MatchConfig::RETRIEVAL;
		e2eCfg.maxPairsPerImage = 20; // >= 10 so verification feedback engages
		e2eCfg.maxEpipolarError = 0; // rely on descriptor matches only, as PairSelectionTest does
		e2eCfg.minMatches = 10;
		e2eCfg.matchDistance = FLT_MAX;
		PairsMatcher e2eMatcher(e2eScene, e2eCfg);
		bool bFatal;
		numMatched = e2eMatcher.Match(bFatal);
		TEST_CHECK(numMatched != 0 && !bFatal, "end-to-end RETRIEVAL match produced no pairs%s", bFatal ? " (fatal round failure)" : "");
	}

	// VOCABULARY's matcher opts into ROMA2 retrieval (enabled=true) exactly as the deleted global-descriptor substitution
	// required: if its candidate pairs followed the global clustering (i/4), that could only mean the substitution (or a new
	// equivalent of it) is back. Asserted on the produced pair sets themselves, never on log text.
	MatchConfig vocabCfg = retrievalCfg;
	vocabCfg.mode = MatchConfig::VOCABULARY;
	PairsMatcher vocabMatcher(scene, vocabCfg);
	DenseMatchConfig denseCfg;
	denseCfg.enabled = true;
	vocabMatcher.SetDense(NULL, denseCfg);
	const PairIdxArr vocabPairs = vocabMatcher.CollectVocabularyPairs(2);
	TEST_CHECK(!vocabPairs.empty(), "VOCABULARY produced no candidate pairs");
	std::set<uint64_t> vocabSet, retrievalSet;
	for (const PairIdx& p : vocabPairs) vocabSet.insert(p.idx);
	for (const PairIdx& p : retrievalPairs) retrievalSet.insert(p.idx);
	TEST_CHECK(vocabSet != retrievalSet, "VOCABULARY and RETRIEVAL selected the identical pair set on a scene built so the local and global "
		"clusterings disagree -- the global descriptors are still leaking into VOCABULARY's ranking");
	// stronger: VOCABULARY's own pairs must actually follow the local (i%4, 4 clusters of 3) clustering it was given, not merely
	// "differ from RETRIEVAL" for some unrelated reason. CollectFusedRetrievalPairs bridges any connected components its
	// mutual-top-K selection left disjoint (see its step 4), so up to numClusters-1 pairs are legitimately cross-cluster; every
	// remaining pair must stay inside its local cluster
	unsigned vocabCross = 0;
	for (const PairIdx& p : vocabPairs)
		vocabCross += (p.i % 4 != p.j % 4);
	TEST_CHECK(vocabCross <= 3, "%u/%u VOCABULARY pairs cross the local descriptor clustering (more than the 3 connectivity bridges 4 clusters can need)",
		vocabCross, (unsigned)vocabPairs.size());
	// (and RETRIEVAL's pairs follow the global clustering, up to the 2 bridges 3 clusters can need: CheckConnected above)

	// identical descriptors rank by ascending image ID, and two queries agree
	scene.images[9].globalDescriptor = scene.images[8].globalDescriptor.clone();
	scene.images[10].globalDescriptor = scene.images[8].globalDescriptor.clone();
	GlobalDescriptors index2;
	index2.Build(scene);
	const auto tied = index2.Query(11, 3);
	TEST_CHECK(tied.size() == 3 && tied[0].first == 8 && tied[1].first == 9 && tied[2].first == 10 && index2.Query(11, 3) == tied, "tie order");

	// the .sfm round-trip of the descriptors
	const ScopedTempDir tmp(_T("RetrievalTest"));
	TEST_CHECK(tmp.IsValid() && scene.Save(tmp("scene.sfm")), "saving the scene");
	Scene loaded;
	TEST_CHECK(loaded.Load(tmp("scene.sfm")) && loaded.images[3].HasGlobalDescriptor() &&
		cv::norm(loaded.images[3].globalDescriptor - scene.images[3].globalDescriptor) <= 1e-6, "serialization of the global descriptors");
	return scope.Pass("cosine ranking, tie order, RETRIEVAL pairs connected with at most 2 bridges (%u end-to-end matched pairs), a missing descriptor refused, "
		"VOCABULARY on the local clustering (%u cross), the .sfm round-trip", numMatched, vocabCross);
}

// The name breaks of an image sequence (directory, name prefix, falling or jumping frame numbers, names without a
// number, 19 and 20 digits, the median rise per run of one prefix, a gap of 0) and the neighbours they leave (SequenceNeighbours)
bool SequenceNeighboursTest()
{
	TestScope scope("SequenceNeighboursTest");
	using Breaks = std::vector<bool>;
	const auto BreaksOf = [](std::initializer_list<const char*> names, unsigned maxGap) {
		std::vector<String> fileNames;
		for (const char* name : names)
			fileNames.emplace_back(name);
		return ComputeNameBreaks(fileNames, maxGap);
	};

	// 1. a 4x gap is no break, a 40x gap, a falling number and a prefix change are
	const std::initializer_list<const char*> sequence = {"c1_0002.jpg", "c1_0004.jpg", "c1_0006.jpg", "c1_0014.jpg",
		"c1_0094.jpg", "c1_0096.jpg", "c1_0090.jpg", "c2_0092.jpg", "c2_0094.jpg"};
	TEST_CHECK(BreaksOf(sequence, 10) == (Breaks{0,0,0,1,0,1,1,0}), "the breaks of the interrupted sequence are wrong");

	// a gap is measured in the usual step of the numbering: frames numbered by hundreds have no break, a frame
	// 20 steps on does
	TEST_CHECK(BreaksOf({"f_0100.jpg", "f_0200.jpg", "f_0300.jpg", "f_2300.jpg"}, 10) == (Breaks{0,0,1}),
		"the gap is not read against the median rise");

	// 2. names without a break, or without a number
	TEST_CHECK(BreaksOf({"000002.png", "000004.png", "000008.png"}, 10) == (Breaks{0,0}), "pure numeric stems: a break");
	TEST_CHECK(BreaksOf({"left.jpg", "right.jpg"}, 10) == (Breaks{0}), "stems without a number: a break");
	TEST_CHECK(BreaksOf({"c_0001.jpg", "photo_12345678901234567890.jpg", "c_0002.jpg"}, 10) == (Breaks{0,0}),
		"a 20-digit stem makes a break with a neighbour");
	// 19 digits are a number (nanosecond-epoch stems), 20 are not: the same jump is a break by the gap rule, then none
	TEST_CHECK(BreaksOf({"1403636579763555584.png", "1403636579763555585.png", "1403636579763555586.png", "1403636579763555686.png"}, 10) ==
		(Breaks{0,0,1}), "19-digit stems: the jump of 100 steps is no break");
	TEST_CHECK(BreaksOf({"14036365797635555840.png", "14036365797635555850.png", "14036365797635555860.png", "14036365797635556860.png"}, 10) ==
		(Breaks{0,0,0}), "20-digit stems: a break");

	// the median rise is taken per run of one prefix: a clip stepping by 100 after one stepping by 1 has no break of its own
	TEST_CHECK(BreaksOf({"a_0001.jpg", "a_0002.jpg", "a_0003.jpg", "a_0004.jpg", "a_0005.jpg", "a_0006.jpg",
		"b_0100.jpg", "b_0200.jpg", "b_0300.jpg"}, 10) == (Breaks{0,0,0,0,0,1,0,0}), "two clips numbered at different steps: "
		"the breaks are not the prefix change alone");
	// a falling number is a break and keeps the run: its rises still count in the run's median
	// (rises 10, 10, 10, 1, 1, 100: a median of 10, so the rise of 100 is no break; a run of its own after the fall would
	// have a median of 1 and break it)
	TEST_CHECK(BreaksOf({"a_0010.jpg", "a_0020.jpg", "a_0030.jpg", "a_0040.jpg", "a_0001.jpg", "a_0002.jpg", "a_0003.jpg",
		"a_0103.jpg"}, 10) == (Breaks{0,0,0,1,0,0,0}), "a falling number: the breaks of the run are wrong");

	// a gap of 0: the rises give no break (names that are clock times), the prefix and the falling number still do
	const std::initializer_list<const char*> clock = {"IMG_20261125_103058.jpg", "IMG_20261125_103059.jpg", "IMG_20261125_103100.jpg",
		"IMG_20261125_103101.jpg"};
	TEST_CHECK(BreaksOf(clock, 10) == (Breaks{0,1,0}), "clock-time names at a gap of 10: the minute rollover is no break");
	TEST_CHECK(BreaksOf(clock, 0) == (Breaks{0,0,0}), "clock-time names at a gap of 0: a break");
	TEST_CHECK(BreaksOf({"a_0001.jpg", "a_0005.jpg", "a_0003.jpg", "b_0004.jpg", "b_9000.jpg"}, 0) == (Breaks{0,1,1,0}),
		"a gap of 0: the prefix and falling-number breaks are wrong");
	TEST_CHECK(BreaksOf({"only.jpg"}, 10).empty(), "one name: breaks");
	TEST_CHECK(BreaksOf({}, 10).empty(), "no name: breaks");

	// 3. the directory is part of the name
	TEST_CHECK(BreaksOf({"a/0001.jpg", "a/0002.jpg", "b/0003.jpg"}, 10) == (Breaks{0,1}), "no break at the change of directory");

	// 4. the neighbours: at most 3 apart, none across a break
	{
		const SequenceNeighbours neighbours({0,0,0,1,0,0}, 3);
		TEST_CHECK(neighbours.Overlap() == 3 && neighbours.NumBreaks() == 1, "the overlap or the number of breaks is not kept");
		TEST_CHECK(neighbours.AreNeighbours(0, 3) && neighbours.AreNeighbours(1, 3) && neighbours.AreNeighbours(4, 6),
			"neighbours not found");
		TEST_CHECK(!neighbours.AreNeighbours(0, 4), "positions 4 apart are neighbours");
		TEST_CHECK(!neighbours.AreNeighbours(3, 4) && !neighbours.AreNeighbours(2, 5), "neighbours across a break");
		TEST_CHECK(!neighbours.AreNeighbours(2, 2), "an image is its own neighbour");
		// the positions past the flags have no break
		TEST_CHECK(neighbours.AreNeighbours(6, 8) && neighbours.AreNeighbours(7, 9) && !neighbours.AreNeighbours(3, 8), "the neighbours past the breaks are wrong");
		for (IIndex a = 0; a < 8; ++a)
			for (IIndex b = 0; b < 8; ++b)
				TEST_CHECK(neighbours.AreNeighbours(a, b) == neighbours.AreNeighbours(b, a),
					"AreNeighbours(%u, %u) is not symmetric", a, b);
		const SequenceNeighbours none({0,0,0}, 0);
		for (IIndex a = 0; a < 4; ++a)
			for (IIndex b = 0; b < 4; ++b)
				TEST_CHECK(!none.AreNeighbours(a, b), "overlap 0: (%u, %u) are neighbours", a, b);
	}

	// 5. built from a scene and a configuration: the breaks of its file names in a sequence run, none in any other
	{
		Scene scene;
		const char* names[] = {"c1_0002.jpg", "c1_0004.jpg", "c1_0094.jpg", "c1_0096.jpg", "c2_0001.jpg"};
		std::vector<String> fileNames;
		for (IIndex i = 0; i < 5; ++i) {
			scene.images.emplace_back(i, names[i]);
			fileNames.emplace_back(names[i]);
		}
		MatchConfig cfg;
		cfg.matchSequenceOverlap = 2;
		cfg.sequenceMaxGap = 10;
		TEST_CHECK(!cfg.IsImageSequence() && !SequenceNeighbours::FromConfig(scene, cfg).has_value(), "the default configuration is a sequence run");
		cfg.imageSequence = true;
		const std::optional<SequenceNeighbours> neighbours = SequenceNeighbours::FromConfig(scene, cfg);
		TEST_CHECK(cfg.IsImageSequence() && neighbours.has_value(), "no neighbours with the option");
		const Breaks breaks = ComputeNameBreaks(fileNames, 10);
		TEST_CHECK(breaks == (Breaks{0,1,0,1}) && neighbours->NumBreaks() == 2 && neighbours->Overlap() == 2, "the scene's breaks are wrong");
		for (IIndex a = 0; a < 5; ++a)
			for (IIndex b = a + 1; b < 5; ++b)
				TEST_CHECK(neighbours->AreNeighbours(a, b) == (b - a <= 2 && std::none_of(breaks.begin() + a, breaks.begin() + b, [](bool isBreak) { return isBreak; })),
					"the scene's neighbours are wrong: (%u, %u)", a, b);
		cfg.sequenceMaxGap = 0;
		TEST_CHECK(SequenceNeighbours::FromConfig(scene, cfg)->NumBreaks() == 1, "the scene's breaks at a gap of 0 are wrong");
	}

	return scope.Pass("name breaks and neighbours: gap, number, prefix and directory rules, 19 digits, the median per run, a gap of 0");
}

// The four ways a candidate pair is made or pruned: the known-pose selection must still produce candidates for an image
// absent from the pose file; PairsMatcher::AddSequentialPairs appends the (i, i+k) pairs for k = 1..overlap (no wrap-around)
// to an existing candidate list, never duplicating a pair already present, and adds nothing at overlap 0; the SEQUENTIAL
// mode matches exactly the (i, i+1) and (i, i+2) pairs; and the pre-match threshold keeps a pair with enough shared descriptors
// and drops the weak ones
bool PairSelectionTest()
{
	TestScope scope("PairSelectionTest");
	Scene scene;
	SceneConfig sceneCfg;
	sceneCfg.numImages = 6;
	sceneCfg.numPoints = 120;
	sceneCfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	sceneCfg.rotationAngleStep = 60;
	sceneCfg.generateDescriptors = true;
	GenerateTestScene(scene, sceneCfg);

	// 1. known poses, the last image unposed
	{
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
			TEST_CHECK(uniquePairs.emplace(pair.idx).second, "duplicate pair (%u, %u)", pair.i, pair.j);
			coversUnposedImage = coversUnposedImage || pair.i == unposedImageID || pair.j == unposedImageID;
		}
		TEST_CHECK(coversUnposedImage, "no candidate covers the unposed image");
	}

	// 2. the sequential prior: (i, i+k) for k = 1..3, no wrap-around, on 6 images
	{
		const std::vector<std::pair<IIndex,IIndex>> expectedPairs = {
			{0,1}, {0,2}, {0,3}, {1,2}, {1,3}, {1,4}, {2,3}, {2,4}, {2,5}, {3,4}, {3,5}, {4,5}};
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
		// an empty candidate list gains exactly the expected pairs
		PairIdxArr pairs;
		unsigned numAdded = PairsMatcher::AddSequentialPairs(scene, 3, pairs);
		TEST_CHECK(numAdded == expectedPairs.size() && pairs.size() == expectedPairs.size() && HasAllExpected(ToSet(pairs)),
			"an empty list gained %u pairs (%u total), expected %u", numAdded, (unsigned)pairs.size(), (unsigned)expectedPairs.size());
		// a list already holding (0,1) and (2,3) gains the rest and holds no duplicates
		PairIdxArr partial;
		partial.emplace_back(MakePairIdx(0, 1));
		partial.emplace_back(MakePairIdx(2, 3));
		numAdded = PairsMatcher::AddSequentialPairs(scene, 3, partial);
		const std::unordered_set<PairIdx::PairIndex> partialSet = ToSet(partial);
		TEST_CHECK(numAdded == expectedPairs.size() - 2 && partial.size() == expectedPairs.size() && partialSet.size() == partial.size() &&
			HasAllExpected(partialSet), "a partial list gained %u pairs (%u total, %u unique), expected %u (%u total)", numAdded,
			(unsigned)partial.size(), (unsigned)partialSet.size(), (unsigned)expectedPairs.size() - 2, (unsigned)expectedPairs.size());
		// overlap 0 adds nothing
		PairIdxArr none;
		numAdded = PairsMatcher::AddSequentialPairs(scene, 0, none);
		TEST_CHECK(numAdded == 0 && none.empty(), "overlap 0 added %u pairs", numAdded);
	}

	// 3. the sequential match mode on 5 images with guaranteed matches and overlap 2: the pairs (i, i+1) and (i, i+2)
	// (0,1) (0,2) (1,2) (1,3) (2,3) (2,4) (3,4), and the (0,3), (0,4), (1,4) of the all-pairs budget: 10 in all
	{
		Scene covisible;
		BuildCovisibleScene(covisible);
		MatchConfig mcfg;
		mcfg.mode = MatchConfig::SEQUENTIAL;
		mcfg.matchSequenceOverlap = 2;
		mcfg.maxEpipolarError = 0; // disable geometric verification for simplicity (rely on descriptor matches)
		mcfg.minMatches = 10;
		mcfg.matchDistance = FLT_MAX; // large distance to avoid filtering
		PairsMatcher matcher(covisible, mcfg);
		bool bFatal;
		const unsigned numPairs = matcher.Match(bFatal);
		TEST_CHECK(!bFatal, "matching reported a fatal round failure");
		std::set<uint64_t> expectedPairs;
		for (const auto& [a, b] : std::vector<std::pair<IIndex, IIndex>>{{0, 1}, {0, 2}, {1, 2}, {1, 3}, {2, 3}, {2, 4}, {3, 4}, {0, 3}, {0, 4}, {1, 4}})
			expectedPairs.insert(MakePairIdx(a, b).idx);
		TEST_CHECK(numPairs == expectedPairs.size(), "expected %u pairs, got %u", (unsigned)expectedPairs.size(), numPairs);
		for (const ImagePair& p : covisible.pairs) {
			const uint64_t idx = MakePairIdx(p.ID1, p.ID2).idx;
			TEST_CHECK(expectedPairs.count(idx) != 0, "unexpected pair (%u, %u)", p.ID1, p.ID2);
			expectedPairs.erase(idx);
		}
		TEST_CHECK(expectedPairs.empty(), "missing %u expected pairs", (unsigned)expectedPairs.size());
	}

	// 4. the pre-match threshold: image 0 shares 15 descriptors with image 1, and 2 with image 2; image 1 and 2 share none
	{
		Scene preMatch;
		preMatch.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), 1000, 1000, 320, 240));
		preMatch.images.resize(3);
		for (int i = 0; i < 3; ++i) {
			preMatch.images[i].ID = i;
			preMatch.images[i].fileName = "img" + std::to_string(i) + ".jpg";
			preMatch.images[i].pCamera = preMatch.cameras[0];
			preMatch.images[i].cameraID = 0;
			preMatch.images[i].keypoints.resize(20);
			preMatch.images[i].descriptors.create(20, 128, CV_8U);
			cv::randu(preMatch.images[i].descriptors, cv::Scalar(0), cv::Scalar(255)); // random noise first
		}
		for (int i = 0; i < 15; ++i) { // the common pattern of 0-1
			for (int k = 0; k < 128; ++k) {
				const uint8_t val = (uint8_t)(i * 10 + k);
				preMatch.images[0].descriptors.at<uint8_t>(i, k) = val;
				preMatch.images[1].descriptors.at<uint8_t>(i, k) = val;
			}
		}
		for (int i = 0; i < 2; ++i) { // a weak common pattern of 0-2, in the last slots of image 0
			for (int k = 0; k < 128; ++k) {
				const uint8_t val = (uint8_t)(200 + i * 10 + k);
				preMatch.images[0].descriptors.at<uint8_t>(18+i, k) = val;
				preMatch.images[2].descriptors.at<uint8_t>(i, k) = val;
			}
		}
		MatchConfig mcfg;
		mcfg.mode = MatchConfig::EXHAUSTIVE;
		mcfg.preMatchThreshold = 5; // require at least 5 matches
		mcfg.descriptorsAreBinary = false; // the noise is simple bytes (SIFT-like)
		mcfg.minMatches = 15; // require at least 15 matches to keep a pair
		mcfg.maxEpipolarError = 0; // disable geometric verification (no cameras)
		PairsMatcher matcher(preMatch, mcfg);
		bool bFatal;
		matcher.Match(bFatal);
		TEST_CHECK(!bFatal, "pre-matching reported a fatal round failure");
		// pair (0,1) needs >= 5 matches and has 15; pair (0,2) has 2 and is filtered; pair (1,2) is noise
		TEST_CHECK(preMatch.FindPair(0, 1) != nullptr, "expected pair (0,1) to be kept");
		TEST_CHECK(preMatch.FindPair(0, 2) == nullptr, "expected pair (0,2) to be filtered (weak matches)");
	}
	return scope.Pass("known-pose candidates cover the unposed image, the sequential prior adds exactly the expected pairs, the sequential mode matches them, "
		"and the pre-match threshold keeps the strong pair only");
}

// Ten posed frames of an interrupted sequence, a_0001..a_0005 then b_0006..b_0010 (a name break between positions 4 and 5),
// where every pair matches (30 descriptors shared by all the images) but the two halves share nothing else: 60 more
// descriptors are shared within a half only, and the global descriptors of the halves are orthogonal. So every candidate
// pair a match mode attempts is stored, and the visual rankings keep to the halves
static void BuildInterruptedSequenceScene(Scene& scene)
{
	SceneConfig scfg;
	scfg.numImages = 10;
	scfg.numPoints = 0; // the posed cameras only, the features are set below
	GenerateTestScene(scene, scfg);
	constexpr int numShared = 30, numHalf = 60, numOwn = 20, numRows = numShared + numHalf + numOwn, D = 64;
	cv::RNG rng(17);
	cv::Mat shared(numShared, 128, CV_8U), halves[2] = {cv::Mat(numHalf, 128, CV_8U), cv::Mat(numHalf, 128, CV_8U)};
	rng.fill(shared, cv::RNG::UNIFORM, 0, 256);
	rng.fill(halves[0], cv::RNG::UNIFORM, 0, 256);
	rng.fill(halves[1], cv::RNG::UNIFORM, 0, 256);
	FOREACH(i, scene.images) {
		Image& img = scene.images[i];
		const int half = i < 5 ? 0 : 1;
		img.fileName = String::FormatString("%c_%04u.jpg", half ? 'b' : 'a', i + 1);
		img.descriptors.create(numRows, 128, CV_8U);
		shared.copyTo(img.descriptors.rowRange(0, numShared));
		halves[half].copyTo(img.descriptors.rowRange(numShared, numShared + numHalf));
		cv::Mat own = img.descriptors.rowRange(numShared + numHalf, numRows);
		rng.fill(own, cv::RNG::UNIFORM, 0, 256);
		img.keypoints.clear();
		for (int r = 0; r < numRows; ++r)
			img.keypoints.emplace_back(cv::Point2f(10.f + (r % 20) * 25.f, 10.f + (r / 20) * 40.f), 10.f);
		img.globalDescriptor.create(1, D, CV_32F);
		for (int c = 0; c < D; ++c)
			img.globalDescriptor.at<float>(c) = (c / (D / 2) == half ? 1.f : 0.f) + (float)rng.gaussian(0.05);
		img.globalDescriptor /= cv::norm(img.globalDescriptor);
	}
	scene.status.nFeaturesType = FeatureType::SIFT;
	scene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);
}

// --image-sequence: the neighbour pairs of the sequence are always matched. PairsMatcher::AddNeighbourPairs appends the
// missing ones (none across the name break), the pre-match filter never removes one (one the scene already holds verified
// leaves the list, held once), and every match mode attempts all of them while adding nothing else, and none after a
// selection that failed; with the option off every mode's candidates are today's (the sequential prior of the retrieval
// modes included, which does cross the break)
bool SequenceForcedMatchTest()
{
	TestScope scope("SequenceForcedMatchTest");
	using PairSet = std::set<uint64_t>;
	constexpr IIndex numImages = 10;
	const auto ToSet = [](const PairIdxArr& pairs) {
		PairSet set;
		for (const PairIdx& pair : pairs)
			set.insert(pair.idx);
		return set;
	};
	const auto Spans = [](uint64_t pairIndex) {
		const PairIdx pair(pairIndex);
		return (pair.i < 5) != (pair.j < 5);
	};
	const auto CountSpanning = [&Spans](const PairSet& set) {
		return (unsigned)std::count_if(set.begin(), set.end(), Spans);
	};
	PairIdxArr allPairs;
	for (IIndex a = 0; a < numImages; ++a)
		for (IIndex b = a + 1; b < numImages; ++b)
			allPairs.emplace_back(MakePairIdx(a, b));

	// the configuration of every Match run below: at most 2 pairs per image keeps VOCABULARY itself on 10 images and the
	// verification-feedback round off (it needs at least 10), so the candidates of a mode are those of its first round:
	// the top 2*8/5 = 3 of the retrieval modes and the top 2*4/3 = 2 of the known poses
	const auto Config = [](MatchConfig::MatchMode mode, bool imageSequence) {
		MatchConfig cfg;
		cfg.mode = mode;
		cfg.imageSequence = imageSequence;
		cfg.maxPairsPerImage = 2;
		cfg.maxEpipolarError = 0; // rely on descriptor matches only, as PairSelectionTest does
		cfg.minMatches = 10;
		cfg.matchDistance = FLT_MAX;
		return cfg;
	};

	Scene scene;
	BuildInterruptedSequenceScene(scene);
	const SequenceNeighbours neighbours = SequenceNeighbours::FromConfig(scene, Config(MatchConfig::EXHAUSTIVE, true)).value();
	TEST_CHECK(neighbours.NumBreaks() == 1 && neighbours.Overlap() == 3 && neighbours.AreNeighbours(3, 4) && !neighbours.AreNeighbours(4, 5) && neighbours.AreNeighbours(5, 6),
		"the fixture's break is not between positions 4 and 5");
	// the neighbour pairs: (a, b), b - a <= 3, both on one side of the break: 9 on each side
	PairSet expectedNeighbours;
	for (IIndex a = 0; a < numImages; ++a)
		for (IIndex b = a + 1; b <= a + 3 && b < numImages; ++b)
			if ((a < 5) == (b < 5))
				expectedNeighbours.insert(MakePairIdx(a, b).idx);
	ASSERT(expectedNeighbours.size() == 18);

	// 1. AddNeighbourPairs appends exactly the missing neighbour pairs
	{
		PairIdxArr pairs;
		unsigned numAdded = PairsMatcher::AddNeighbourPairs(scene, neighbours, pairs);
		TEST_CHECK(numAdded == 18 && pairs.size() == 18 && ToSet(pairs) == expectedNeighbours,
			"an empty list gained %u pairs (%u total), expected the 18 neighbour pairs", numAdded, (unsigned)pairs.size());
		numAdded = PairsMatcher::AddNeighbourPairs(scene, neighbours, pairs);
		TEST_CHECK(numAdded == 0 && pairs.size() == 18, "a list holding every neighbour pair gained %u pairs", numAdded);
		PairIdxArr partial;
		partial.emplace_back(MakePairIdx(0, 1));
		partial.emplace_back(MakePairIdx(0, 9));
		numAdded = PairsMatcher::AddNeighbourPairs(scene, neighbours, partial);
		PairSet expectedPartial = expectedNeighbours;
		expectedPartial.insert(MakePairIdx(0, 9).idx);
		TEST_CHECK(numAdded == 17 && partial.size() == 19 && ToSet(partial) == expectedPartial,
			"a list holding (0,1) and (0,9) gained %u pairs (%u total), expected 17 (19 total, both kept)", numAdded, (unsigned)partial.size());
	}

	// the pairs Match stores on a fresh fixture: every attempted pair matches, so these are the attempted candidates
	const auto Matched = [](const MatchConfig& cfg, bool& bFatal) {
		Scene fresh;
		BuildInterruptedSequenceScene(fresh);
		PairsMatcher matcher(fresh, cfg);
		matcher.Match(bFatal);
		PairSet stored;
		for (const ImagePair& pair : fresh.pairs)
			stored.insert(MakePairIdx(pair.ID1, pair.ID2).idx);
		return stored;
	};

	// 2. PreMatch at a threshold no pair can reach keeps exactly the neighbour pairs in a sequence run, and nothing in another
	{
		MatchConfig cfg = Config(MatchConfig::VOCABULARY, true);
		cfg.preMatchThreshold = 100000;
		PairsMatcher matcher(scene, cfg);
		TEST_CHECK(!matcher.CollectVocabularyPairs(3).empty(), "the vocabulary tree PreMatch reads was not built");
		PairIdxArr kept = allPairs;
		matcher.PreMatch(kept);
		TEST_CHECK(kept.size() == 18 && ToSet(kept) == expectedNeighbours,
			"PreMatch of a sequence run kept %u pairs, expected the 18 neighbour pairs", (unsigned)kept.size());
		MatchConfig plainCfg = cfg;
		plainCfg.imageSequence = false;
		PairsMatcher plainMatcher(scene, plainCfg);
		TEST_CHECK(!plainMatcher.CollectVocabularyPairs(3).empty(), "the vocabulary tree PreMatch reads was not built");
		PairIdxArr none = allPairs;
		plainMatcher.PreMatch(none);
		TEST_CHECK(none.empty(), "PreMatch without the option kept %u pairs", (unsigned)none.size());
		TEST_CHECK(scene.pairs.empty(), "PreMatch stored %u pairs", (unsigned)scene.pairs.size());
		// a neighbour pair the scene already holds verified is done, not filtered: with the geometric verification on
		// (which stores what a pre-match verifies) it leaves the list, is held once, and the other neighbours stay
		{
			Scene verified;
			BuildInterruptedSequenceScene(verified);
			ImagePair& pair = verified.pairs.emplace_back(1, 2);
			pair.matches.resize(30);
			pair.F = Matrix3x3::IDENTITY;
			MatchConfig verifyCfg = cfg;
			verifyCfg.maxEpipolarError = 4.f;
			PairsMatcher verifyMatcher(verified, verifyCfg);
			TEST_CHECK(!verifyMatcher.CollectVocabularyPairs(3).empty(), "the vocabulary tree PreMatch reads was not built");
			PairIdxArr toMatch = allPairs;
			verifyMatcher.PreMatch(toMatch);
			PairSet expectedLeft = expectedNeighbours;
			expectedLeft.erase(MakePairIdx(1, 2).idx);
			TEST_CHECK(toMatch.size() == 17 && ToSet(toMatch) == expectedLeft,
				"PreMatch kept %u pairs, expected the 17 neighbour pairs the scene does not hold verified", (unsigned)toMatch.size());
			TEST_CHECK(verified.pairs.size() == 1 && verified.FindPair(1, 2) == &verified.pairs[0],
				"the verified neighbour pair is held %u times among %u pairs", (unsigned)std::count_if(verified.pairs.begin(), verified.pairs.end(),
				[](const ImagePair& p) { return MakePairIdx(p.ID1, p.ID2).idx == MakePairIdx(1, 2).idx; }), (unsigned)verified.pairs.size());
		}
		// and through Match: the vocabulary mode pre-matches its candidates with the neighbours of the option
		bool bFatal;
		const PairSet stored = Matched(cfg, bFatal);
		TEST_CHECK(!bFatal && stored == expectedNeighbours,
			"VOCABULARY pre-matching at an unreachable threshold stored %u pairs, expected the 18 neighbour pairs", (unsigned)stored.size());
		cfg.imageSequence = false;
		TEST_CHECK(Matched(cfg, bFatal).empty() && !bFatal, "VOCABULARY pre-matching at an unreachable threshold stored pairs without the option");
	}

	// 3. every match mode: the option attempts every neighbour pair and adds nothing else, in particular no pair across the
	// break; off, the candidates are today's: the mode's own and, in VOCABULARY and RETRIEVAL, the sequential prior
	PairSet priorPairs; {
		PairIdxArr prior;
		PairsMatcher::AddSequentialPairs(scene, 3, prior);
		priorPairs = ToSet(prior);
	}
	unsigned numModes = 0;
	for (const MatchConfig::MatchMode mode : {MatchConfig::EXHAUSTIVE, MatchConfig::VOCABULARY, MatchConfig::SEQUENTIAL,
		MatchConfig::RETRIEVAL, MatchConfig::KNOWN_POSES})
	{
		const char* const name = mode == MatchConfig::EXHAUSTIVE ? "EXHAUSTIVE" : mode == MatchConfig::VOCABULARY ? "VOCABULARY" :
			mode == MatchConfig::SEQUENTIAL ? "SEQUENTIAL" : mode == MatchConfig::RETRIEVAL ? "RETRIEVAL" : "KNOWN_POSES";
		// the mode's own candidates, from the call Match makes on the same fixture
		PairSet own; {
			Scene fresh;
			BuildInterruptedSequenceScene(fresh);
			PairsMatcher matcher(fresh, Config(mode, false));
			switch (mode) {
			case MatchConfig::EXHAUSTIVE: own = ToSet(allPairs); break;
			case MatchConfig::VOCABULARY: own = ToSet(matcher.CollectVocabularyPairs(3)); break;
			case MatchConfig::RETRIEVAL: own = ToSet(matcher.CollectRetrievalPairs(3)); break;
			case MatchConfig::KNOWN_POSES: own = ToSet(matcher.CollectKnownPosePairs(2)); break;
			default: // SEQUENTIAL: (i, i+k) for k = 1..3 with the wrap-around of 10 >= 2*3 images
				for (IIndex i = 0; i < numImages; ++i)
					for (IIndex k = 1; k <= 3; ++k)
						own.insert(MakePairIdx(i, (i + k) % numImages).idx);
			}
		}
		TEST_CHECK(!own.empty(), "%s: no candidates of its own", name);
		const bool withPrior = mode == MatchConfig::VOCABULARY || mode == MatchConfig::RETRIEVAL;
		bool bFatal;
		// on: every neighbour pair is attempted, and nothing but the mode's own candidates besides
		const PairSet on = Matched(Config(mode, true), bFatal);
		TEST_CHECK(!bFatal, "%s: a fatal round failure with the option", name);
		// cannot fail for EXHAUSTIVE and SEQUENTIAL: their own candidates hold every neighbour pair
		for (const uint64_t pairIndex : expectedNeighbours)
			TEST_CHECK(on.count(pairIndex), "%s: the neighbour pair (%u, %u) was not attempted", name, PairIdx(pairIndex).i, PairIdx(pairIndex).j);
		PairSet expectedOn = own;
		expectedOn.insert(expectedNeighbours.begin(), expectedNeighbours.end());
		TEST_CHECK(on == expectedOn, "%s: the option attempted %u pairs, expected the %u of the mode and the neighbours", name,
			(unsigned)on.size(), (unsigned)expectedOn.size());
		// cannot fail for EXHAUSTIVE and SEQUENTIAL either: every pair across the break they attempt is their own
		for (const uint64_t pairIndex : on)
			TEST_CHECK(!Spans(pairIndex) || own.count(pairIndex), "%s: the option added (%u, %u), across the break", name,
				PairIdx(pairIndex).i, PairIdx(pairIndex).j);
		// off: today's candidates, whatever the sequence options left unread
		const PairSet off = Matched(Config(mode, false), bFatal);
		TEST_CHECK(!bFatal, "%s: a fatal round failure without the option", name);
		PairSet expectedOff = own;
		if (withPrior)
			expectedOff.insert(priorPairs.begin(), priorPairs.end());
		TEST_CHECK(off == expectedOff, "%s: %u pairs attempted without the option, expected today's %u", name,
			(unsigned)off.size(), (unsigned)expectedOff.size());
		MatchConfig unread = Config(mode, false);
		unread.sequenceMaxGap = 1;
		TEST_CHECK(Matched(unread, bFatal) == off && !bFatal, "%s: the sequence options changed the candidates with the option off", name);
		// the halves share nothing, so the visual rankings cross the break at most by their connectivity bridge, where the
		// prior crosses it six times: the option is seen not to add those
		if (withPrior)
			TEST_CHECK(CountSpanning(own) <= 1 && CountSpanning(on) < CountSpanning(off), "%s: %u pairs across the break with the option, "
				"%u without (%u of the mode's own)", name, CountSpanning(on), CountSpanning(off), CountSpanning(own));
		++numModes;
	}

	// 4. a selection that failed (RETRIEVAL with a global descriptor missing, an error) ends the stage with the option as
	// without: nothing is matched, Match returns 0 and the error is logged, the neighbour pairs are not matched alone
	{
		const auto MatchedWithoutDescriptor = [](const MatchConfig& cfg, bool& bFatal, unsigned& numMatched, bool& logged) {
			Scene fresh;
			BuildInterruptedSequenceScene(fresh);
			fresh.images[7].globalDescriptor.release();
			PairsMatcher matcher(fresh, cfg);
			LogCapture log;
			numMatched = matcher.Match(bFatal);
			logged = log.Contains("error: retrieval produced no candidate pairs");
			PairSet stored;
			for (const ImagePair& pair : fresh.pairs)
				stored.insert(MakePairIdx(pair.ID1, pair.ID2).idx);
			return stored;
		};
		bool bFatal, logged;
		unsigned numMatched;
		const PairSet on = MatchedWithoutDescriptor(Config(MatchConfig::RETRIEVAL, true), bFatal, numMatched, logged);
		TEST_CHECK(!bFatal && on.empty() && numMatched == 0 && logged, "a failed RETRIEVAL selection with the option matched %u pairs "
			"(Match returned %u, the error %slogged), expected none, as without the option", (unsigned)on.size(), numMatched, logged ? "" : "not ");
		const PairSet off = MatchedWithoutDescriptor(Config(MatchConfig::RETRIEVAL, false), bFatal, numMatched, logged);
		TEST_CHECK(!bFatal && off.empty() && numMatched == 0 && logged, "a failed RETRIEVAL selection without the option matched %u pairs "
			"(Match returned %u, the error %slogged)", (unsigned)off.size(), numMatched, logged ? "" : "not ");
	}
	return scope.Pass("the 18 neighbour pairs appended once, kept by PreMatch, attempted by %u match modes with nothing across the break, "
		"none after a failed selection; today's candidates with the option off", numModes);
}

// Scene::MatchPairs() must fail the stage rather than report success when matching leaves the view graph empty over two or
// more images -- the state a fatal matching failure leaves behind, indistinguishable by return value alone from a scene whose
// candidates were already all matched. A device-allocation failure can not be provoked from a unit test, but the observable
// state it leaves behind can: a scene whose images share no content rejects every candidate on plain insufficient matches
// (an ordinary, non-fatal outcome) and reaches the exact same empty graph. And PairsMatcher::Match() reports a fatal round
// failure apart from the pair count, because the count cannot express it: a round legitimately stores nothing, and a round
// that fails AFTER an earlier one stored pairs leaves a view graph that is truncated rather than empty, which is precisely
// what Scene::MatchPairs' own empty-graph check can not see. What is pinned here of that contract is the half a unit fixture
// can reach: the signal must stay CLEAR for every non-fatal outcome, so it can not be conflated with "stored nothing".
// Provoking a genuinely fatal round is out of reach here -- the only two are a SiftMatchGPU coordinator that fails to
// initialise (SiftGPU + CUDA build, real device) and a device-slot-pool failure inside MatchPairsDense (a loaded RoMa v2
// model) -- so the stage-failure side is left to the run-time behaviour Scene::MatchPairs implements.
bool MatchOutcomeSignalsTest()
{
	TestScope scope("MatchOutcomeSignalsTest");
	// images with keypoints but nothing in common: every candidate pair falls under minMatches, so nothing is ever stored and
	// the view graph comes out empty
	const auto BuildScene = [](unsigned numImages) {
		Scene scene;
		SceneConfig scfg;
		scfg.numImages = numImages;
		scfg.numPoints = 0;
		scfg.generateDescriptors = true;
		GenerateTestScene(scene, scfg);
		return scene;
	};
	MatchConfig matchCfg;
	matchCfg.mode = MatchConfig::EXHAUSTIVE;
	matchCfg.maxEpipolarError = 0;
	matchCfg.descriptorsAreBinary = SceneConfig().binaryDescriptors;

	// the stage fails on the empty graph over three images
	{
		Scene scene = BuildScene(3);
		TEST_CHECK(!scene.MatchPairs(matchCfg), "MatchPairs() reported success with an empty view graph over %u images", (unsigned)scene.images.size());
		TEST_CHECK(scene.pairs.empty(), "%u unexpected pairs stored", (unsigned)scene.pairs.size());
		TEST_CHECK(!scene.status.nState.isSet(Scene::Status::STATE::MATCHED), "MATCHED state set despite the reported failure");
	}
	// a round that stored nothing is not a fatal failure; the pair count cannot say so
	{
		Scene scene = BuildScene(3);
		PairsMatcher matcher(scene, matchCfg);
		bool bFatal = true;
		const unsigned numPairs = matcher.Match(bFatal);
		TEST_CHECK(numPairs == 0 && scene.pairs.empty(), "%u pairs matched among three images with nothing in common", numPairs);
		TEST_CHECK(!bFatal, "a round that stored nothing is reported as a fatal failure");
	}
	// a single image legitimately produces no pairs (nothing to match): a no-op for the stage and no fatal round either
	{
		Scene single = BuildScene(1);
		TEST_CHECK(single.MatchPairs(matchCfg), "a single-image scene must not be reported as a matching failure");
		Scene singleRound = BuildScene(1);
		PairsMatcher singleMatcher(singleRound, matchCfg);
		bool bFatal = true;
		TEST_CHECK(singleMatcher.Match(bFatal) == 0 && !bFatal, "a single-image scene is reported as a fatal round failure");
	}
	return scope.Pass("an empty view graph over three images fails the stage and is no fatal round, a single image is neither");
}

// Spherical bottom band of the feature extraction (FeatureExtractionConfig::sphericalBottomBand): a textured equirectangular
// image gives keypoints all over it with the band off, none in the bottom 20% of rows with the default band and as many above
// it as before (the band removes keypoints, it does not move the budget); a perspective image of the same texture is unaffected
// by the option (same keypoints, same descriptors).
bool SphericalBottomBandTest()
{
	TestScope scope("SphericalBottomBandTest");
	constexpr int width = 1024, height = 512;
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
	cfg.cubemapFaceSize = 256;
	ASSERT(cfg.sphericalBottomBand == 0.2f); // on by default

	const auto Extract = [&](float band, bool spherical, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
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
	TEST_CHECK(Extract(0.f, true, kpOff, descOff) && Extract(0.2f, true, kpOn, descOn), "the spherical extraction failed");
	const float bandRow = 0.8f * height;
	size_t offBelow = 0, offAbove = 0, onBelow = 0, onAbove = 0;
	for (const cv::KeyPoint& kp : kpOff)
		(kp.pt.y > bandRow ? offBelow : offAbove)++;
	for (const cv::KeyPoint& kp : kpOn)
		(kp.pt.y > bandRow ? onBelow : onAbove)++;
	TEST_CHECK(offBelow >= 100, "the band-0 extraction has only %u keypoints in the bottom 20%% of rows, the test image is not textured enough", (unsigned)offBelow);
	TEST_CHECK(onBelow == 0, "%u keypoints in the discarded bottom band", (unsigned)onBelow);
	// the band masks keypoints one by one after the faces' detection, which it does not touch: the rows above the band keep
	// every keypoint (the angular NMS across the face seams could only differ where a masked keypoint lies within
	// cubemapDedupAngleDeg, 0.25 degrees or a row or two of the image, of one above)
	TEST_CHECK(onAbove == offAbove, "the rows above the band hold %u keypoints, %u without the band", (unsigned)onAbove, (unsigned)offAbove);
	TEST_CHECK((int)onAbove == descOn.rows, "%u keypoints but %d descriptor rows", (unsigned)onAbove, descOn.rows);

	std::vector<cv::KeyPoint> kpP0, kpP2;
	cv::Mat descP0, descP2;
	TEST_CHECK(Extract(0.f, false, kpP0, descP0) && Extract(0.2f, false, kpP2, descP2), "the perspective extraction failed");
	TEST_CHECK(!kpP0.empty() && kpP0.size() == kpP2.size() && cv::norm(descP0, descP2, cv::NORM_INF) == 0,
		"the band changed the perspective extraction (%u vs %u keypoints)", (unsigned)kpP0.size(), (unsigned)kpP2.size());
	for (size_t i = 0; i < kpP0.size(); ++i)
		TEST_CHECK(kpP0[i].pt == kpP2[i].pt, "the band moved perspective keypoint %u", (unsigned)i);
	return scope.Pass("band 0: %u keypoints above row %g and %u below; band 0.2: %u above, none below; the perspective extraction is unchanged",
		(unsigned)offAbove, bandRow, (unsigned)offBelow, (unsigned)onAbove);
}
/*----------------------------------------------------------------*/

// The detector words and the defaults of the detector configuration. The four accepted words (AKAZE, ORB, SIFT, SIFTGPU) are
// read in any letter case, any other word is NONE. FeatureType::DEFAULT is the detector of a default-constructed
// FeatureExtractionConfig and KeyframeConfig, and a default-constructed MatchConfig carries the descriptor kind and the
// thresholds DefaultsForFeatureType gives that detector (the ones the command line applies for it), so a library or Python
// caller that sets nothing gets the pipeline the application runs; the thresholds of each detector are the documented ones
bool DetectorConfigTest()
{
	TestScope scope("DetectorConfigTest");
	const std::pair<const char*, FeatureType> accepted[] = {
		{"AKAZE", FeatureType::AKAZE}, {"akaze", FeatureType::AKAZE},
		{"ORB", FeatureType::ORB}, {"Orb", FeatureType::ORB},
		{"SIFT", FeatureType::SIFT}, {"sift", FeatureType::SIFT},
		{"SIFTGPU", FeatureType::SIFTGPU}, {"SiftGpu", FeatureType::SIFTGPU}};
	for (const auto& a : accepted)
		TEST_CHECK(FeatureTypeFromString(a.first) == a.second, "'%s' is not read as %s", a.first, FeatureTypeToString(a.second).c_str());
	for (const char* word : {"", "SURF", "SIFT GPU", "siftgpu2"})
		TEST_CHECK(FeatureTypeFromString(word) == FeatureType::NONE, "'%s' is accepted", word);

	TEST_CHECK(FeatureType::DEFAULT == FeatureType::SIFTGPU || FeatureType::DEFAULT == FeatureType::SIFT, "the default detector is neither SIFTGPU nor SIFT");
	TEST_CHECK(FeatureExtractionConfig().detectorType == FeatureType::DEFAULT && KeyframeConfig().detectorType == FeatureType::DEFAULT,
		"a configuration does not start from the default detector");
	const MatchConfig defaults;
	const MatchConfig expected(MatchConfig().DefaultsForFeatureType(FeatureType::DEFAULT));
	TEST_CHECK(defaults.descriptorsAreBinary == expected.descriptorsAreBinary && defaults.maxDescriptorsPerImage == expected.maxDescriptorsPerImage &&
		defaults.matchDistance == expected.matchDistance && defaults.matchRatio == expected.matchRatio && defaults.minMatches == expected.minMatches &&
		!defaults.descriptorsAreBinary && defaults.maxDescriptorsPerImage == 1000 && defaults.matchRatio == 0.8f && defaults.minMatches == 15,
		"the default MatchConfig does not carry the SIFT thresholds of the default detector");
	// the thresholds of every detector, named
	struct Expected { FeatureType type; bool binary; unsigned maxDescriptors; float distance; float ratio; unsigned minMatches; };
	const Expected table[] = {
		{FeatureType::AKAZE, true, 2000, 100.f, 0.9f, 50}, {FeatureType::ORB, true, 2000, 64.f, 0.9f, 50},
		{FeatureType::SIFT, false, 1000, FLT_MAX, 0.8f, 15}, {FeatureType::SIFTGPU, false, 1000, FLT_MAX, 0.8f, 15}};
	for (const Expected& e: table) {
		const MatchConfig cfg(MatchConfig().DefaultsForFeatureType(e.type));
		TEST_CHECK(cfg.descriptorsAreBinary == e.binary && cfg.maxDescriptorsPerImage == e.maxDescriptors && cfg.matchDistance == e.distance &&
			cfg.matchRatio == e.ratio && cfg.minMatches == e.minMatches, "wrong thresholds for %s", FeatureTypeToString(e.type).c_str());
	}
	return scope.Pass("the four detector words in any case, one default detector for the library and the application, and the thresholds of each detector");
}
/*----------------------------------------------------------------*/

// The geometry of a pair of panoramas (bearing vectors, no fundamental matrix) on one sphere scene (MakeSphereScene: two
// panoramas 1.08 apart, 120 points around them, about half of each camera's matches behind it):
// 1. PairsMatcher::MatchPair, the calibrated branch through poselib's bearing-vector relative pose (RANSAC scoring without
//    cheirality, the Sampson error on the sphere), on the matches the tracks give: rotation within 0.5 degrees, translation
//    direction within acos(0.99);
// 2. MatchFeaturesGeometric, the tracked-point guided matching KeyframeExtractor runs on consecutive 360-degree keyframes:
//    bootstrapped from the tracked points, it filters descriptor candidates by the angular epipolar distance, with no F
//    (reading pair.F.value() there once threw for every spherical pair). It must estimate the geometry, leave F empty, keep
//    80% of the tracked correspondences, and admit back-hemisphere bearings (z < 0) among its inliers.
// The bearing-vector absolute pose is tested in PoseLib's own suite (test_estimate_absolute_pose_bearings).
bool SphericalPairGeometryTest()
{
	TestScope scope("SphericalPairGeometryTest");
	Scene scene(MakeSphereScene({Point3(-0.5, 0, 0), Point3(0.5, 0, 0.2)}, 120, 2027));
	Image& img0 = scene.images[0];
	Image& img1 = scene.images[1];
	const Pose3D poseRelGT(img1 / img0);
	// per keypoint of image 0, the keypoint of image 1 its track holds (every track of two panoramas holds both)
	std::vector<uint32_t> feat0ToFeat1(img0.keypoints.size(), NO_ID);
	for (const Track& track : scene.tracks) {
		ASSERT(track.observations.size() == 2 && track.observations[0].imageID == 0);
		feat0ToFeat1[track.observations[0].featureID] = track.observations[1].featureID;
	}
	const auto IsBack = [&](uint32_t feat0, uint32_t feat1) {
		return img0.pCamera->UnprojectNormalized(Cast<REAL>(img0.keypoints[feat0].pt)).z < 0 ||
			img1.pCamera->UnprojectNormalized(Cast<REAL>(img1.keypoints[feat1].pt)).z < 0;
	};
	// the recovered relative pose against the truth: rotation error in degrees, |cos| of the translation directions
	const auto PoseErrors = [&](const ImagePair& pair) {
		const Pose3D& pose = pair.relativePose.value();
		return std::make_pair(RotationErrorDeg(pose.R, poseRelGT.R), ABS(normalized(pose.GetT()).dot(normalized(poseRelGT.GetT()))));
	};

	// 1. MatchPair on the matches of the tracks
	{
		ImagePair pair(0, 1);
		FOREACH(feat0, feat0ToFeat1)
			if (feat0ToFeat1[feat0] != NO_ID)
				pair.matches.emplace_back((uint32_t)feat0, feat0ToFeat1[feat0]);
		MatchConfig matchCfg;
		matchCfg.minMatches = 8;
		matchCfg.maxEpipolarError = 5.f;
		PairsMatcher matcher(scene, matchCfg);
		TEST_CHECK(matcher.MatchPair(img0, img1, pair) && pair.relativePose.has_value(), "MatchPair found no relative pose (%u matches)",
			pair.GetNumMatches());
		const auto [rotErr, tSim] = PoseErrors(pair);
		TEST_CHECK(rotErr <= 0.5 && tSim >= 0.99, "MatchPair: rotation %.4f deg off (0.5 at most), translation similarity %.4f (0.99 at least)",
			rotErr, tSim);
	}

	// 2. MatchFeaturesGeometric bootstrapped from the tracked points, every track a unique 256-bit descriptor shared by its
	// two keypoints (Hamming distance 0, so the descriptor matching prefers the true pair)
	const size_t N0 = img0.keypoints.size();
	std::vector<Point2f> trackedPoints1(N0), trackedPoints2(N0, Point2f(0.f, 0.f));
	std::vector<uchar> trackStatus(N0, 0);
	constexpr int descBytes = 32;
	img0.descriptors = cv::Mat::zeros((int)N0, descBytes, CV_8U);
	img1.descriptors = cv::Mat::zeros((int)img1.keypoints.size(), descBytes, CV_8U);
	size_t numTracked = 0, numBack = 0;
	for (uint32_t feat0 = 0; feat0 < N0; ++feat0) {
		trackedPoints1[feat0] = img0.keypoints[feat0].pt;
		const uint32_t feat1 = feat0ToFeat1[feat0];
		if (feat1 == NO_ID)
			continue;
		trackedPoints2[feat0] = img1.keypoints[feat1].pt;
		trackStatus[feat0] = 1;
		++numTracked;
		if (IsBack(feat0, feat1))
			++numBack;
		std::mt19937 descRng(0xDEADBEEFu ^ feat0);
		for (int b = 0; b < descBytes; ++b)
			img0.descriptors.at<uint8_t>((int)feat0, b) = img1.descriptors.at<uint8_t>((int)feat1, b) = (uint8_t)(descRng() & 0xFF);
	}
	TEST_CHECK(numTracked >= 50 && numBack >= 20, "the scene holds %zu tracked correspondences (50 needed), %zu of them behind a camera "
		"(20 needed to make the test meaningful)", numTracked, numBack);
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
	TEST_CHECK(MatchFeaturesGeometric(matcher, img0, img1, trackedPoints1, trackedPoints2, trackStatus, pair, 2.f),
		"MatchFeaturesGeometric reported its fallback (no geometry estimated)");
	TEST_CHECK(!pair.F.has_value() && pair.E.has_value() && pair.relativePose.has_value(),
		"a spherical pair must carry E and a relative pose and no F (F %s, E %s, pose %s)", pair.F.has_value() ? "set" : "empty",
		pair.E.has_value() ? "set" : "empty", pair.relativePose.has_value() ? "set" : "empty");
	const auto [rotErr, tSim] = PoseErrors(pair);
	TEST_CHECK(rotErr <= 0.5 && tSim >= 0.99, "MatchFeaturesGeometric: rotation %.4f deg off (0.5 at most), translation similarity %.4f "
		"(0.99 at least)", rotErr, tSim);
	const unsigned numInliers = pair.GetNumInliers();
	TEST_CHECK(numInliers >= numTracked * 8 / 10, "only %u inliers, under 80%% of the %zu tracked", numInliers, numTracked);
	size_t numBackInliers = 0;
	for (unsigned k = 0; k < numInliers; ++k)
		if (IsBack(pair.matches[k].queryIdx, pair.matches[k].trainIdx))
			++numBackInliers;
	TEST_CHECK(numBackInliers >= 10, "only %zu of %u inliers behind a camera: the epipolar filter rejects z < 0 bearings", numBackInliers, numInliers);
	return scope.Pass("MatchPair and MatchFeaturesGeometric recover the panoramas' relative pose; %u of %zu tracked kept, %zu behind a camera",
		numInliers, numTracked, numBackInliers);
}
/*----------------------------------------------------------------*/

// The two-view geometry of PairsMatcher and ImagePair on one synthetic pair of pinhole cameras with distortion (600 points,
// perturbed keypoints): the distortion projection round trip, the ImagePair matrix operations (compose and decompose the
// essential matrix, the pose recovered from point correspondences, F -> E), PairsMatcher::MatchPair recovering the true relative
// pose, CheckEpipolarInliers agreeing with it from the pose, from E and from F, and RelativePoseRefine::RefineTwoViewCalibration
// recovering a 3% focal error and lost distortion; then the relative pose recovered from the fundamental matrix
// (GeometricFilter's 7-DoF F branch over trusted intrinsics, then DecomposeFundamentalToPose) for two cameras of different
// intrinsics and image sizes, and for two cameras of one size: the true rotation and translation direction, and nearly every
// match in front of both cameras
bool TwoViewGeometryTest()
{
	TestScope scope("TwoViewGeometryTest");
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

	// camera distortion projection/unprojection
	{
		const Point2 pt_dist(120.f, 40.f);
		const Point3 X = scene.images[0].UnprojectPoint(pt_dist, 5);
		const auto [pt_proj, valid] = scene.images[0].ProjectPoint(X);
		TEST_CHECK(valid, "distortion projection/unprojection invalid projection");
		TEST_CHECK(norm(pt_proj - pt_dist) <= 1e-4, "distortion projection/unprojection error too large (%.6f)", norm(pt_proj - pt_dist));
	}

	// ImagePair matrix operations
	REAL angle_decomp_err = 0;
	{
		const Matrix3x3 E_composed = ImagePair::ComposeEssentialMatrix(pose_rel);
		// DecomposeEssentialMatrix returns one of 4 solutions, needing a cheirality check
		const Pose3D pose_decomposed = ImagePair::DecomposeEssentialMatrix(E_composed);

		// RecoverPose with actual point correspondences (unit bearings, each through its own camera)
		std::vector<Eigen::Vector3d> bearings1, bearings2;
		for (const Track& track : scene.tracks) {
			bearings1.emplace_back(scene.images[0].pCamera->UnprojectNormalized(Cast<REAL>(scene.images[0].keypoints[track.observations[0].featureID].pt)));
			bearings2.emplace_back(scene.images[1].pCamera->UnprojectNormalized(Cast<REAL>(scene.images[1].keypoints[track.observations[1].featureID].pt)));
		}
		Pose3D pose_recovered;
		const int numInliers = ImagePair::RecoverPose(E_composed, bearings1, bearings2, pose_recovered);
		TEST_CHECK(numInliers >= 8, "RecoverPose returned insufficient inliers (%d)", numInliers);
		// the decomposed rotation is close to the recovered pose
		angle_decomp_err = ACOS(ComputeAngle(pose_decomposed.R, pose_recovered.R));
		TEST_CHECK(angle_decomp_err <= 0.1, "DecomposeEssentialMatrix rotation error too large (%.4f rad)", angle_decomp_err);
		// the recovered rotation is close to the ground truth
		const REAL angle_err = ACOS(ComputeAngle(pose_recovered.R, pose_rel.R));
		TEST_CHECK(angle_err <= 0.1, "rotation error too large (%.4f rad)", angle_err);
		// the recovered translation direction (up to scale and sign)
		const REAL t_similarity = ABS(normalized(pose_rel.GetT()).dot(normalized(pose_recovered.GetT())));
		TEST_CHECK(t_similarity >= 0.95, "translation direction error too large (similarity=%.4f)", t_similarity);
		// F composed from E and decomposed again is the same E up to scale
		const Matrix3x3 F_composed = ImagePair::ComposeFundamentalMatrix(E_composed, K, K);
		const Matrix3x3 E_from_F = ImagePair::DecomposeFundamentalMatrix(F_composed, K, K);
		const Matrix3x3 E_from_F_normalized = E_from_F / cv::norm(E_from_F);
		const Matrix3x3 E_normalized = E_composed / cv::norm(E_composed);
		const REAL e_diff = FrobeniusNorm(E_from_F_normalized, E_normalized);
		TEST_CHECK(e_diff <= 0.01, "F->E decomposition error (%.6f)", e_diff);
	}

	// PairsMatcher::MatchPair
	const float maxEpipolarError = 5.f;
	ImagePair pair;
	{
		MatchConfig config;
		config.descriptorsAreBinary = cfg.binaryDescriptors;
		config.minMatches = 8;
		config.maxEpipolarError = maxEpipolarError;
		PairsMatcher matcher(scene, config);
		TEST_CHECK(matcher.MatchPair(scene.images[0], scene.images[1], pair), "PairsMatcher::MatchPair failed");
		TEST_CHECK(pair.HasMatches(), "the pair has no matches after MatchPair");
		TEST_CHECK(pair.HasGeometricVerification(), "the pair has no geometric verification");
		TEST_CHECK(pair.relativePose.has_value(), "the pair has no relative pose");
		const Pose3D& recovered_pose = pair.relativePose.value();
		const REAL angle_err = ACOS(ComputeAngle(recovered_pose.R, pose_rel.R));
		TEST_CHECK(angle_err <= 0.5, "matched rotation error too large (%.4f rad)", angle_err);
		const REAL t_similarity = ABS(normalized(pose_rel.GetT()).dot(normalized(recovered_pose.GetT())));
		TEST_CHECK(t_similarity >= 0.95, "matched translation error too large (similarity=%.4f)", t_similarity);
	}

	// CheckEpipolarInliers from the relative pose, from E and from F
	{
		const size_t numInliersRelativePose = pair.CheckEpipolarInliers(scene.images[0], scene.images[1], maxEpipolarError);
		TEST_CHECK(numInliersRelativePose == pair.GetNumInliers(), "CheckEpipolarInliers inconsistent with PairsMatcher for the relative pose (%u vs %u)",
			(unsigned)numInliersRelativePose, pair.GetNumInliers());
		pair.relativePose.reset(); // remove the relative pose to test the E case
		ASSERT(pair.E.has_value());
		const size_t numInliersE = pair.CheckEpipolarInliers(scene.images[0], scene.images[1], maxEpipolarError);
		TEST_CHECK(numInliersE == pair.GetNumInliers(), "CheckEpipolarInliers inconsistent with PairsMatcher for the essential matrix (%u vs %u)",
			(unsigned)numInliersE, pair.GetNumInliers());
		pair.E.reset(); // remove E to test the F case
		ASSERT(pair.F.has_value());
		const size_t numInliersF = pair.CheckEpipolarInliers(scene.images[0], scene.images[1], maxEpipolarError+2);
		TEST_CHECK(numInliersF+50 >= pair.GetNumInliers(), "CheckEpipolarInliers inconsistent with PairsMatcher for the fundamental matrix (%u vs %u)", // allow small tolerance
			(unsigned)numInliersF, pair.GetNumInliers());
	}

	// RelativePoseRefine::RefineTwoViewCalibration from a 3% focal error and lost distortion
	{
		Pose3D pose_rel_refined = pose_rel;
		cam.fx = cam.fy *= 1.03;
		cam.k1 = cam.k2 = 0;
		RelativePoseRefine::Config refine_cfg;
		refine_cfg.robustThreshold = 1.0;
		RelativePoseRefine::Result refine_result;
		TEST_CHECK(RelativePoseRefine::RefineTwoViewCalibration(scene.images[0].keypoints, scene.images[1].keypoints, pair.matches,
			cam, pose_rel_refined, refine_cfg, &refine_result), "RelativePoseRefine::RefineTwoViewCalibration failed");
		const REAL fx_error = ABS(cam.fx - camGT.fx) / camGT.fx;
		const REAL k1_error = ABS(cam.k1 - camGT.k1);
		const REAL k2_error = ABS(cam.k2 - camGT.k2);
		TEST_CHECK(fx_error <= 0.05, "refined focal length error too large (%.4f%%)", fx_error * 100); // 5% tolerance
		TEST_CHECK(k1_error <= 0.1 && k2_error <= 0.1, "refined distortion error too large (k1_err=%.6f, k2_err=%.6f)", k1_error, k2_error);
		const REAL refined_angle_err = ACOS(ComputeAngle(pose_rel_refined.R, pose_rel.R));
		const REAL refined_t_similarity = ABS(normalized(pose_rel_refined.GetT()).dot(normalized(pose_rel.GetT())));
		TEST_CHECK(refined_angle_err <= 0.5, "refined rotation error too large (%.4f rad)", refined_angle_err);
		TEST_CHECK(refined_t_similarity >= 0.95, "refined translation error too large (similarity=%.4f)", refined_t_similarity);
	}

	// the pose from F, for a second camera of another focal, principal point and image size, and for one of the same
	const auto FundamentalPoseCase = [&](LPCSTR name, const SceneConfig::CameraSpec& spec2) -> bool {
		SceneConfig fcfg;
		fcfg.cameras = {SceneConfig::CameraSpec{SceneConfig::PINHOLE, 640, 480, 500.0, 330.0, 235.0}, spec2};
		fcfg.numImages = 2;
		fcfg.numPoints = 400;
		fcfg.poseMode = SceneConfig::RANDOM_POSES;
		fcfg.generatePairs = true;
		fcfg.perturbOptions = SceneConfig::PERTURB_KEYPOINTS;
		Scene fsceneGT, fscene;
		GenerateTestScene(fsceneGT, fcfg, &fscene);
		ImagePair& fpair = fscene.pairs.front();
		const Pose3D truth = fpair.relativePose.value(); // image 1 relative to image 0, as generated
		const unsigned numTrue = fpair.GetNumInliers();
		fpair.relativePose.reset();
		fpair.E.reset();
		fpair.F.reset();
		MatchConfig config;
		config.forceFundamental = true; // the 7-DoF F over the fixture's trusted intrinsics, then its decomposition
		PairsMatcher matcher(fscene, config);
		const Image& img1 = fscene.images[fpair.ID1];
		const Image& img2 = fscene.images[fpair.ID2];
		TEST_CHECK(PairsMatcher::SelectGeometryBranch(config, img1, img2) == PairsMatcher::GeometryBranch::FUNDAMENTAL, "%s: the pair did not take the fundamental branch", name);
		TEST_CHECK(matcher.GeometricFilter(img1, img2, fpair) && fpair.relativePose.has_value(), "%s: images %dx%d and %dx%d, no relative pose from F (%s)", name,
			img1.GetWidth(), img1.GetHeight(), img2.GetWidth(), img2.GetHeight(), fpair.HasMatches() ? "matches kept" : "matches invalidated");
		const REAL rotErr = RotationErrorDeg(fpair.relativePose->R, truth.R);
		const REAL dirErr = R2D(ACOS(CLAMP(normalized(fpair.relativePose->GetT()).dot(normalized(truth.GetT())), REAL(-1), REAL(1))));
		const auto [bearings1, bearings2] = fpair.GetMatchedBearings(img1, img2, true);
		Pose3D pose;
		const unsigned numInFront = ImagePair::RecoverPose(fpair.E.value(), bearings1, bearings2, pose);
		TEST_CHECK(rotErr <= 1.0 && dirErr <= 2.0, "%s: the pose from F is off the truth by %.3f deg in rotation and %.3f deg in direction (bars 1 and 2 deg)", name, rotErr, dirErr);
		TEST_CHECK(fpair.GetNumInliers() >= numTrue * 95 / 100 && numInFront >= numTrue * 95 / 100,
			"%s: %u of the %u true matches verified and %u in front of both cameras, 95%% of them expected", name, fpair.GetNumInliers(), numTrue, numInFront);
		return true;
	};
	if (!FundamentalPoseCase("different sizes", SceneConfig::CameraSpec{SceneConfig::PINHOLE, 1600, 900, 1300.0, 790.0, 460.0}) ||
		!FundamentalPoseCase("same size", SceneConfig::CameraSpec{SceneConfig::PINHOLE, 640, 480, 500.0, 330.0, 235.0}))
		return false;
	return scope.Pass("projection round trip, essential and fundamental matrix operations (the decomposed rotation %.4f rad from the recovered one), "
		"MatchPair, CheckEpipolarInliers, the two-view calibration refinement, and the pose from F for cameras of different and of equal sizes",
		angle_decomp_err);
}
/*----------------------------------------------------------------*/

// SiftGPU's CUDA matcher on a pair too large for one product of descriptors (SiftMatchCU::GetTiledSiftMatch): 70000 x 60000
// features, past the old fixed max_sift of 32768 (features beyond it were dropped) and past the 46340 at which the full
// int-indexed product overflows. Random unit descriptors (scaled to 512, as SiftGPU wants them) never match; planted copies
// (small noise) spread over the whole of both images, in every tile of rows, must all come back as matches. Twin rows in two
// different tiles with the same column test the merge of each column's best and second best over the tiles: without the
// mutual check both twins match the column; with it the column fails its ratio test and neither does. A small pair (one
// tile, the untiled path) gives its planted matches too. Skipped without a CUDA device.
bool SiftGPUTiledMatchTest()
{
	TestScope scope("SiftGPUTiledMatchTest");
	#if defined(_USE_SIFTGPU) && defined(SIFTGPU_CUDA)
	const int num1 = 70000, num2 = 60000;
	std::unique_ptr<SiftMatchGPU> gpu(CreateNewSiftMatchGPU(num1));
	TEST_CHECK(gpu != nullptr, "CreateNewSiftMatchGPU failed");
	gpu->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA);
	if (!gpu->CreateContextGL())
		return scope.Pass("skipped: no CUDA device for SiftMatchGPU");
	TEST_CHECK(gpu->Allocate(num1, 1), "Allocate(%d) failed", num1);
	TEST_CHECK(gpu->GetMaxSift() >= num1, "max_sift %d below %d", gpu->GetMaxSift(), num1);
	std::mt19937 rng(7);
	std::normal_distribution<float> gauss(0.f, 1.f);
	const auto Random = [&](uint8_t* d) {
		float v[128], n = 0;
		for (int k = 0; k < 128; ++k) { v[k] = std::abs(gauss(rng)); n += v[k] * v[k]; }
		n = 512.f / std::sqrt(n);
		for (int k = 0; k < 128; ++k) d[k] = (uint8_t)std::min(255.f, std::round(v[k] * n));
	};
	std::vector<uint8_t> d1((size_t)num1 * 128), d2((size_t)num2 * 128);
	for (int i = 0; i < num1; ++i) Random(&d1[(size_t)i * 128]);
	for (int j = 0; j < num2; ++j) Random(&d2[(size_t)j * 128]);
	// planted pairs: row i -> column j, spread over both images
	std::vector<std::pair<int,int>> planted;
	std::uniform_int_distribution<int> jitter(-2, 2);
	const auto Copy = [&](const uint8_t* src, uint8_t* dst) {
		for (int k = 0; k < 128; ++k) dst[k] = (uint8_t)std::clamp((int)src[k] + jitter(rng), 0, 255);
	};
	for (int t = 0; t < 600; ++t) {
		const int i = (int)((int64_t)t * (num1 - 1) / 599), j = (int)((int64_t)(599 - t) * (num2 - 1) / 599);
		Copy(&d1[(size_t)i * 128], &d2[(size_t)j * 128]);
		planted.emplace_back(i, j);
	}
	// twins: rows 1001 and 65001 (different tiles) copies of one descriptor, both close to column 30001
	const int twinA = 1001, twinB = 65001, twinCol = 30001;
	Copy(&d2[(size_t)twinCol * 128], &d1[(size_t)twinA * 128]);
	Copy(&d2[(size_t)twinCol * 128], &d1[(size_t)twinB * 128]);
	gpu->SetDescriptors(0, num1, d1.data());
	gpu->SetDescriptors(1, num2, d2.data());
	const auto Match = [&](int mbm, int n1) {
		std::vector<uint32_t> buf((size_t)n1 * 2);
		const int n = gpu->GetSiftMatch(n1, reinterpret_cast<uint32_t (*)[2]>(buf.data()), 0.7f, 0.8f, mbm);
		std::set<std::pair<int,int>> got;
		for (int k = 0; k < n; ++k) got.emplace((int)buf[2 * k], (int)buf[2 * k + 1]);
		return std::make_pair(n, got);
	};
	for (int mbm = 0; mbm < 2; ++mbm) {
		const auto [n, got] = Match(mbm, num1);
		TEST_CHECK(n >= 0, "GetSiftMatch reported a device error (mbm %d)", mbm);
		unsigned found = 0;
		for (const auto& p: planted) found += got.count(p);
		TEST_CHECK(found == planted.size(), "mbm %d: %u of %u planted matches found", mbm, found, (unsigned)planted.size());
		const bool a = got.count({twinA, twinCol}) != 0, b = got.count({twinB, twinCol}) != 0;
		TEST_CHECK(mbm ? (!a && !b) : (a && b), "mbm %d: twins matched %d %d", mbm, a, b);
		TEST_CHECK(got.size() == planted.size() + (mbm ? 0 : 2), "mbm %d: %u matches, expected %u", mbm, (unsigned)got.size(),
			(unsigned)(planted.size() + (mbm ? 0 : 2)));
	}
	// a small pair, matched in one product: 300 planted pairs among 3000 x 3000 features, the columns shuffled
	const int small = 3000, numSmallPlanted = 300;
	std::vector<uint8_t> s1((size_t)small * 128), s2((size_t)small * 128);
	for (int k = 0; k < small; ++k) { Random(&s1[(size_t)k * 128]); Random(&s2[(size_t)k * 128]); }
	std::vector<int> perm(small);
	std::iota(perm.begin(), perm.end(), 0);
	std::shuffle(perm.begin(), perm.end(), rng);
	std::set<std::pair<int,int>> smallPlanted;
	for (int k = 0; k < numSmallPlanted; ++k) {
		memcpy(&s1[(size_t)k * 128], &d1[(size_t)planted[k].first * 128], 128);
		memcpy(&s2[(size_t)perm[k] * 128], &d2[(size_t)planted[k].second * 128], 128);
		smallPlanted.emplace(k, perm[k]);
	}
	gpu->SetDescriptors(0, small, s1.data());
	gpu->SetDescriptors(1, small, s2.data());
	const auto [ns, gotSmall] = Match(1, small);
	TEST_CHECK(ns >= 0 && gotSmall == smallPlanted, "small pair: %u matches, %u planted", (unsigned)gotSmall.size(), (unsigned)smallPlanted.size());
	return scope.Pass("70000 x 60000 features: all %u planted matches with and without the mutual check, twins across tiles resolved",
		(unsigned)planted.size());
	#else
	return scope.Pass("skipped: built without SiftGPU CUDA");
	#endif
}
/*----------------------------------------------------------------*/

} // namespace SFM
