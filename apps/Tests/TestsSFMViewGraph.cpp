/*
 * TestsSFMViewGraph.cpp
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
#include "../../libs/SFM/SequenceChain.h"
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

namespace {

// two pinhole cameras 1 unit apart along X, both looking down +Z
const Point3 TWO_CAMERA_CENTRES[2] = {Point3(-0.5, 0.0, 0.0), Point3(0.5, 0.0, 0.0)};

void BuildTwoCameraScene(Scene& scene)
{
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), 500, 500, 319.5, 239.5));
	for (unsigned k = 0; k < 2; ++k) {
		Pose3D pose;
		pose.C = TWO_CAMERA_CENTRES[k];
		pose.R = Matrix3x3::IDENTITY;
		scene.images.emplace_back((IIndex)k, String::FormatString("%u.jpg", k), pose, 0, scene.cameras[0]);
	}
	scene.status.nCalibratedImages = scene.images.size();
}

using PairIds = std::pair<IIndex,IIndex>;
using PairSet = std::set<PairIds>;

// the pairs a scene holds, by their two images
PairSet KeptPairs(const Scene& scene)
{
	PairSet kept;
	for (const ImagePair& pair : scene.pairs)
		kept.emplace(pair.ID1, pair.ID2);
	return kept;
}
// the pairs of a set that are not in another
PairSet Without(const PairSet& pairs, const PairSet& removed)
{
	PairSet left;
	std::set_difference(pairs.begin(), pairs.end(), removed.begin(), removed.end(), std::inserter(left, left.end()));
	return left;
}

// turns a pair's baseline to the opposite direction; its rotation and its matches stay
void FlipBaseline(ImagePair& pair)
{
	pair.relativePose->C = -pair.relativePose->C;
}

// The contradiction cut as its tests run it: the minimum pair weight 3, the default weighting, every pair unscored
// unless the scores are given
constexpr float kCutMinPairWeight = 3.f;
const float kDefaultDirectionBar = ReconstructionConfig().maxPairDirectionResidual;
unsigned RunCut(Scene& scene, float maxDirectionResidual, bool repair = false, const SequenceNeighbours* neighbours = nullptr,
	SequenceFilterStats* stats = nullptr, const std::vector<float>* scores = nullptr)
{
	const std::vector<float> unscored(scene.pairs.size(), TripletScores::unscored);
	return FilterPairsByContradiction(scene, scores ? *scores : unscored, kCutMinPairWeight, maxDirectionResidual, PairsWeightingConfig(), repair, neighbours, stats);
}

// The arc of 16 cameras (RingConfig(300)) with its exact relative poses, weighed, every ray angle then 10 degrees;
// pSceneGT: the scene it is generated from, which holds the tracks
void BuildArc(Scene& scene, Scene* pSceneGT = nullptr)
{
	Scene sceneGT;
	GenerateTestScene(pSceneGT ? *pSceneGT : sceneGT, RingConfig(300), &scene);
	ComputePairsWeights(scene);
	for (ImagePair& pair : scene.pairs)
		pair.meanRayAngle = D2R(10.f);
}

// The feature of each point of a list in each image: featureOf[image][point], NO_ID where the image does not see it
using FeatureMap = std::vector<std::vector<uint32_t>>;

// does the image see the point, at least 10 pixels inside its frame, and where
bool SeesPoint(const Image& image, const Point3& X, Point2& proj)
{
	bool valid;
	std::tie(proj, valid) = image.ProjectPoint(X);
	return valid && proj.x >= 10 && proj.y >= 10 && proj.x <= image.GetSize().width - 10 && proj.y <= image.GetSize().height - 10;
}

// gives every image of the scene a keypoint for each of the points it sees
FeatureMap ProjectPoints(Scene& scene, const std::vector<Point3>& points)
{
	FeatureMap featureOf(scene.images.size());
	for (const Point3& X : points)
		for (Image& image : scene.images) {
			Point2 proj;
			const bool seen = SeesPoint(image, X, proj);
			featureOf[image.ID].push_back(seen ? (uint32_t)image.keypoints.size() : NO_ID);
			if (seen)
				image.keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
		}
	return featureOf;
}

// a camera like the image's, with its rotation, standing at C
Image StandingAt(const Image& image, const Point3& C)
{
	return Image(image.ID, "", Pose3D(image.R, C), image.cameraID, image.pCamera);
}

// Gives a pair the exact pose of the camera `second` (its second image, or a camera standing in for it) against its
// first image, with E and F, the baseline at `length`: a length of the pair's own, as a two-view pose has, which
// only the solved scales relate to another pair's. A camera at the first image's centre has no baseline: the pose
// takes a direction that means nothing. Returns whether the two stand apart.
bool SetExactPose(const Scene& scene, ImagePair& pair, const Image& second, REAL length)
{
	const Image& first = scene.images[pair.ID1];
	Pose3D pose = second / first;
	const bool apart = Point3(second.C) != Point3(first.C);
	pose.C = apart ? CMatrix(pose.C * (length / norm(pose.C))) : CMatrix(0, length, 0);
	pair.relativePose = pose;
	pair.E = ImagePair::ComposeEssentialMatrix(pose);
	pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), first.GetK(), scene.images[pair.ID2].GetK());
	return apart;
}

// Replaces a pair's matches by those its first image has with the camera `viewer` standing in for its second: every
// point the first image has a feature of (featureOfFirst, per point) joined to a new keypoint of the second image,
// where the viewer sees the point
void MatchAsSeenFrom(Scene& scene, ImagePair& pair, const Image& viewer, const std::vector<Point3>& points, const std::vector<uint32_t>& featureOfFirst)
{
	ASSERT(featureOfFirst.size() == points.size());
	Image& second = scene.images[pair.ID2];
	pair.matches.clear();
	FOREACH(p, points) {
		Point2 proj;
		if (featureOfFirst[p] == NO_ID || !SeesPoint(viewer, points[p], proj))
			continue;
		pair.matches.emplace_back(featureOfFirst[p], (uint32_t)second.keypoints.size());
		second.keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
	}
}

// A strip of cameras with real features, what the tests of the sequence chain walk: a rig of one image per pose
// (MakeRig: 640x480 at a focal of 400) with a keypoint for every point it sees, and the pairs AddPair adds
struct Strip {
	Scene scene;
	std::vector<Point3> points;
	FeatureMap featureOf;

	Strip(const std::vector<Pose3D>& poses, const std::vector<Point3>& _points)
		: scene(MakeRig(cv::Size(640, 480), 400, poses)), points(_points), featureOf(ProjectPoints(scene, points)) {}

	// Adds the pair (a,b) with the exact pose of the image `standIn` against a, the baseline at `length`
	// (SetExactPose), and 10 degrees of ray angle, none when the two stand at one place. Its matches join the feature
	// of each point in a to the feature in b of the point `pointShift` further in the list. A true pair is its own
	// stand-in at a shift of 0; a look-alike takes the pose of another image and, of b, the copies of what that
	// image sees.
	ImagePair& AddPair(IIndex a, IIndex b, REAL length, IIndex standIn, int pointShift = 0)
	{
		ASSERT(a < b && b < scene.images.size() && standIn < scene.images.size());
		ImagePair& pair = scene.pairs.emplace_back(a, b);
		for (int p = MAXF(0, -pointShift), end = (int)points.size() - MAXF(0, pointShift); p < end; ++p)
			if (featureOf[a][p] != NO_ID && featureOf[b][p + pointShift] != NO_ID)
				pair.matches.emplace_back(featureOf[a][p], featureOf[b][p + pointShift]);
		pair.meanRayAngle = SetExactPose(scene, pair, scene.images[standIn], length) ? D2R(10.f) : 0.f;
		return pair;
	}
	ImagePair& AddPair(IIndex a, IIndex b, REAL length) { return AddPair(a, b, length, b); }
};
} // namespace

// The pair weights (ComputePairsWeights and the composite they feed, ImagePair::GetCompositeWeight):
// (1) the terms of the weight: a pair of spread matches outweighs a clumped one in spatial coverage and in connectivity,
//     and the triplets of a bridge pair raise its triplet weight above the pairs that sit in one triangle;
// (2) the composite of a hand-set pair equals its formula term by term: the weighted inliers capped at 1000, times the
//     coverage in full, times the cube root of the angle term times the connectivity, times 0.25 plus the triangle
//     consistency; weightSpatial holds coverage x angle;
// (3) a pair's dense inliers count as evidence in proportion to the matcher's frame target, not up to a cap: 10 sparse and
//     400 dense inliers weigh 10 + 25 * 400/2000 = 15 at the default of 25 frame inliers, and 110 at 500, the weight the old
//     discount of a quarter per dense inlier gave. Two dense-only pairs of one image keep their order (1600 dense outweighs
//     400 -- a cap flattened them to the same weight), and the connectivity reads the dense matches at the same scale as the
//     evidence, so a dense-only pair and a sparse-only pair that are each their images' only pair carry the same connectivity;
// (4) a dense supplement match is evidence about its pair, discounted for the precision of a warp-sampled position -- not
//     full evidence, and not none. Each term of the weight is checked on one supplemented pair, since each reads a different
//     segment for a different reason:
//  - the ANGLE term, track-forming: FilterMatches' meanRayAngle is the median over the sparse matches AND the supplement. A
//    ray angle is a geometric quantity a warp-sampled position measures as well as a sub-pixel one, and ComputeIntrinsicWeight
//    multiplies ComputeAngleBaselineWeight(meanRayAngle) into weightSpatial -- it is the ONE term that can demote a degenerate
//    baseline, and leaving it sparse hands a pair whose evidence is dense the function's maximum (1.0, not a neutral value).
//  - the AREA term, track-forming: coverage asks where the pair has correspondences, and a dense draw covers what it was
//    drawn over. A dense-only pair would otherwise score no area at all.
//  - the MAGNITUDE, discounted: sparse + w x dense (GetNumWeightedInliers), so a stratified draw cannot re-rank the view graph
//    by sheer count.
//  - the VALIDITY FLOOR, track-forming: ComputeIntrinsicWeight's minimum-support bar returns 0 below it, which zeroes
//    weightSpatial, hence GetCompositeWeight(), hence BuildTracks' minPairWeight cut -- so a floor read from the sparse count
//    alone makes a supplemented pair contribute NO tracks at all, sparse or dense, on exactly the weak pairs supplementation
//    exists to serve.
bool PairWeightTest()
{
	TestScope scope("PairWeightTest");

	// 1. the terms: pairs (0,1) spread, (1,2) clumped and fewer, the bridge (0,2) in the triangles 0-1-2 and 0-2-3 while (0,1)
	// and (1,2) sit in the first only ((1,3) is absent), and the pairs (0,3), (2,3); the relative poses are all identity, so
	// every loop closes perfectly
	{
		Scene scene;
		PairGraph g(scene, 4);
		g.Matched(0, 1, 64).relativePose = Pose3D::Identity();             // spread over 64 cells
		g.Matched(1, 2, 40, true, 1, 1).relativePose = Pose3D::Identity(); // clumped in one cell
		for (const auto& ids : {std::make_pair(0, 2), std::make_pair(0, 3), std::make_pair(2, 3)})
			g.Matched(ids.first, ids.second, 20, true, 1, 1).relativePose = Pose3D::Identity();
		ComputePairsWeights(scene, PairsWeightingConfig());
		const ImagePair& p01 = *scene.FindPair(0, 1);
		const ImagePair& p12 = *scene.FindPair(1, 2);
		const ImagePair& p02 = *scene.FindPair(0, 2);
		TEST_CHECK(p01.weightSpatial > p12.weightSpatial, "spread matches weigh %.4f in spatial coverage, clumped %.4f", p01.weightSpatial, p12.weightSpatial);
		TEST_CHECK(p01.weightConnectivity > p12.weightConnectivity, "spread matches weigh %.4f in connectivity, clumped %.4f", p01.weightConnectivity, p12.weightConnectivity);
		#ifdef _USE_BOOST
		TEST_CHECK(p01.weightTriplet > 0.f && p12.weightTriplet > 0.f && p02.weightTriplet > p01.weightTriplet,
			"triplet weights (0,1) %.4f, (1,2) %.4f, bridge (0,2) %.4f: every valid triplet weighs and the bridge in two triangles weighs most",
			p01.weightTriplet, p12.weightTriplet, p02.weightTriplet);
		#endif
	}

	// 2. the composite weight of hand-set terms
	{
		struct Case { float inliers, coverage, angle, connectivity, triplet; };
		const Case cases[] = {
			{ 400.f, 0.5f, 0.8f, 0.6f, 0.35f },
			{ 1600.f, 0.2f, 0.3f, 1.f, 0.f },  // past the cap: counts 1000
			{ 50.f, 0.9f, 1.f, 0.05f, 1.f },
		};
		for (const Case& c : cases) {
			ImagePair pair(0, 1);
			pair.weightedInliers = c.inliers;
			pair.overlapArea = c.coverage;
			pair.weightSpatial = c.coverage * c.angle;
			pair.weightConnectivity = c.connectivity;
			pair.weightTriplet = c.triplet;
			const double expected = MINF((double)c.inliers, 1000.0) * c.coverage * std::cbrt((double)c.angle * c.connectivity) * (0.25 + c.triplet);
			TEST_CHECK(ABS(pair.GetCompositeWeight() - expected) <= 1e-4 * expected, "inliers %g, coverage %g, angle %g, connectivity %g, triangles %g weigh %g, expected %g",
				c.inliers, c.coverage, c.angle, c.connectivity, c.triplet, pair.GetCompositeWeight(), expected);
		}
	}

	// 3. the dense evidence in proportion to the frame target
	{
		constexpr unsigned NUM_SPARSE = 10, NUM_DENSE = 400;
		// numImages images of numKeypoints keypoints spread over the frame (the weighting reads the pair's partition, not the
		// images' described boundary, so none is set)
		const auto BuildImages = [](Scene& scene, IIndex numImages, unsigned numKeypoints) {
			scene.cameras.emplace_back(new PinholeCamera(cv::Size(100, 100), REAL(100), REAL(100), REAL(50), REAL(50)));
			for (IIndex k = 0; k < numImages; ++k) {
				Image& im = scene.images.emplace_back(k, String::FormatString("%u.jpg", k));
				im.cameraID = 0;
				im.pCamera = scene.cameras[0];
				for (unsigned i = 0; i < numKeypoints; ++i)
					im.keypoints.emplace_back(cv::Point2f(5.f + (i % 20) * 4.5f, 5.f + ((i / 20) % 20) * 4.5f), 3.f);
			}
		};
		// a pair of numSparse descriptor inliers followed by numDense dense ones, every match an inlier
		const auto AddPair = [](Scene& scene, IIndex a, IIndex b, unsigned numSparse, unsigned numDense) {
			ImagePair& pair = scene.pairs.emplace_back(a, b);
			for (uint32_t i = 0; i < numSparse + numDense; ++i)
				pair.matches.emplace_back(i, i);
			pair.numFilteredInliers = (int)numSparse;
			pair.numDenseInliers = (int)numDense;
			pair.meanRayAngle = (float)D2R(15.f); // the angle the baseline weight rates best
		};
		// the magnitude: proportional to the frame target
		struct Case { float frameInliers; unsigned expectedWeightedInliers; };
		const Case cases[] = {
			{ 25.f, NUM_SPARSE + 5u },                       // 25 * 400/2000
			{ 500.f, NUM_SPARSE + NUM_DENSE / 4u },          // a quarter per dense inlier: the old discount
		};
		float compositeAt[2];
		for (unsigned c = 0; c < 2; ++c) {
			Scene scene;
			BuildImages(scene, 2, NUM_SPARSE + NUM_DENSE);
			AddPair(scene, 0, 1, NUM_SPARSE, NUM_DENSE);
			PairsWeightingConfig cfg;
			cfg.denseFrameInliers = cases[c].frameInliers;
			cfg.denseMatchesPerFrame = 2000;
			ComputePairsWeights(scene, cfg);
			const ImagePair& pair = scene.pairs[0];
			TEST_CHECK(pair.GetNumWeightedInliers() == cases[c].expectedWeightedInliers, "%u sparse + %u dense inliers weigh %u at %.0f frame inliers, expected %u",
				NUM_SPARSE, NUM_DENSE, pair.GetNumWeightedInliers(), cases[c].frameInliers, cases[c].expectedWeightedInliers);
			compositeAt[c] = pair.GetCompositeWeight();
		}
		TEST_CHECK(compositeAt[0] > 0.f && compositeAt[0] < compositeAt[1], "the composite weight is %.2f at 25 frame inliers and %.2f at 500; "
			"the scaled pair must stay valid and weigh less", compositeAt[0], compositeAt[1]);
		{ // the order: image 0 sees image 1 through 1600 dense-only inliers and image 2 through 400
			Scene scene;
			BuildImages(scene, 3, 1600);
			AddPair(scene, 0, 1, 0, 1600);
			AddPair(scene, 0, 2, 0, 400);
			ComputePairsWeights(scene, PairsWeightingConfig());
			const float w01 = scene.pairs[0].GetCompositeWeight(), w02 = scene.pairs[1].GetCompositeWeight();
			TEST_CHECK(w02 > 0.f && w01 > w02, "1600 dense-only inliers weigh %.2f against %.2f for 400; the stronger dense-only pair of an image must outweigh the weaker one", w01, w02);
		}
		{ // the connectivity: a sparse-only pair and a dense-only pair, each its images' only pair, carry the same connectivity weight (1)
			Scene scene;
			BuildImages(scene, 4, 2000);
			AddPair(scene, 0, 1, 100, 0);
			AddPair(scene, 2, 3, 0, 2000);
			ComputePairsWeights(scene, PairsWeightingConfig());
			const float cSparse = scene.pairs[0].weightConnectivity, cDense = scene.pairs[1].weightConnectivity;
			TEST_CHECK(ISEQUAL(cSparse, cDense) && ISEQUAL(cSparse, 1.f), "connectivity %.4f for the sparse-only pair and %.4f for the dense-only one; "
				"both are their images' only pair, so both must be 1", cSparse, cDense);
		}
	}

	// 4. the supplement of one pair: the two cameras and 5 sparse (far, Z = 10) and 15 dense (near, Z = 1) points
	Scene scene;
	BuildTwoCameraScene(scene);
	// the descriptor evidence: 5 far points (Z = 10), spread across the frame, so their triangulation angle is ~5 deg and their
	// grid occupancy is 5 cells in each image
	std::vector<Point3> sparsePts, densePts;
	for (unsigned k = 0; k < 5; ++k)
		sparsePts.emplace_back(REAL(k) * 2.0 - 4.0, REAL(k) * 0.8 - 1.6, 10.0);
	// the supplement: 15 near points (Z = 1) clustered in the middle of the overlap, i.e. a ten times larger triangulation angle
	// (~53 deg) and three quarters of the accepted matches, so a median taken over all of them lands nowhere near the sparse one
	for (unsigned k = 0; k < 15; ++k)
		densePts.emplace_back(REAL(k) * 0.014 - 0.098, REAL(k) * 0.014 - 0.098, 1.0);
	for (unsigned k = 0; k < 2; ++k) {
		Image& img = scene.images[k];
		for (const Point3& X : sparsePts) {
			const auto [proj, valid] = img.ProjectPoint(X);
			TEST_CHECK(valid && Image8U::isInside(proj, img.GetSize()), "sparse point outside image %u", k);
			img.keypoints.emplace_back(Cast<float>(proj), 4.f, -1.f, 0.05f);
		}
		img.CloseDescribedKeypoints();
		for (const Point3& X : densePts) {
			const auto [proj, valid] = img.ProjectPoint(X);
			TEST_CHECK(valid && Image8U::isInside(proj, img.GetSize()), "dense point outside image %u", k);
			img.keypoints.push_back(Image::MakeDenseKeypoint(Cast<float>(proj), 0.9f, 6.f));
		}
	}
	ImagePair& pair = scene.pairs.emplace_back(0u, 1u);
	pair.relativePose = scene.images[1] / scene.images[0];
	for (uint32_t k = 0; k < (uint32_t)(sparsePts.size() + densePts.size()); ++k)
		pair.matches.emplace_back(k, k);

	// the median cosine over the TRACK-FORMING matches, sparse and dense together, derived here from the 3D points rather than
	// from the code under test: FloatArr::GetMedian takes the element at index size/2 after nth_element, and the cosine is
	// monotone in the angle. The sparse-only median is derived alongside it, so the check below can state that the two really
	// do differ -- the fixture's dense points sit ten times closer than its sparse ones for exactly that reason.
	const auto RayCosines = [](const std::vector<Point3>& pts, std::vector<REAL>& cosines) {
		for (const Point3& X : pts) {
			const Point3 V1 = X - TWO_CAMERA_CENTRES[0], V2 = X - TWO_CAMERA_CENTRES[1];
			cosines.push_back(V1.dot(V2) / (norm(V1) * norm(V2)));
		}
	};
	std::vector<REAL> sparseCos, allCos;
	RayCosines(sparsePts, sparseCos);
	RayCosines(sparsePts, allCos);
	RayCosines(densePts, allCos);
	std::sort(sparseCos.begin(), sparseCos.end());
	std::sort(allCos.begin(), allCos.end());
	const REAL expectedSparseAngle = ACOS(sparseCos[sparseCos.size() >> 1]);
	const REAL expectedAngle = ACOS(allCos[allCos.size() >> 1]);

	const unsigned numSparse = pair.FilterMatches(scene.images[0], scene.images[1], 0.5f, 6.f, 0.f);
	TEST_CHECK(numSparse == 5 && pair.GetNumFilteredInliers() == 5 && pair.GetNumDenseInliers() == 15 && pair.GetNumTrackFormingMatches() == 20,
		"partition after FilterMatches is %u sparse, %u dense (returned %u)", pair.GetNumFilteredInliers(), pair.GetNumDenseInliers(), numSparse);
	TEST_CHECK(ABS(REAL(pair.meanRayAngle) - expectedAngle) <= 1e-3 && ABS(expectedAngle - expectedSparseAngle) >= D2R(REAL(1)),
		"meanRayAngle is %.4f deg, the track-forming median is %.4f deg (the sparse-only median is %.4f deg) -- the dense supplement must vote on the pair's angle statistic",
		R2D(pair.meanRayAngle), R2D(expectedAngle), R2D(expectedSparseAngle));

	// The floor: 5 sparse matches are under the default minInliers = 15, 20 track-forming ones are not, so the pair must come
	// out with a non-zero weight instead of being hard-zeroed.
	ComputePairsWeights(scene);
	const ImagePair& weighted = *scene.FindPair(0, 1);
	TEST_CHECK(weighted.weightSpatial > 0.f, "weightSpatial is 0 on a supplemented pair with 5 sparse and 15 dense matches -- the validity floor "
		"is reading the sparse count, so this pair contributes no tracks");
	// The AREA score runs over the TRACK-FORMING matches, supplement included: it asks where the pair has correspondences, and
	// the supplement covers what it was drawn over. Recomputed here from the projections themselves -- the 5 sparse points occupy
	// 5 of the 10x10 cells of each image and the 15 clustered dense ones add a few more -- so the check states the rule rather
	// than a number, and asserts the union really is larger than the sparse count alone.
	const auto Occupancy = [](const Image& img, unsigned numPoints) {
		std::vector<bool> grid(100, false);
		for (unsigned k = 0; k < numPoints; ++k) {
			const cv::Point2f& p = img.keypoints[k].pt;
			grid[MINF((int)(p.y/(float)img.GetHeight()*10.f), 9)*10 + MINF((int)(p.x/(float)img.GetWidth()*10.f), 9)] = true;
		}
		return (unsigned)std::count(grid.begin(), grid.end(), true);
	};
	const unsigned occupiedTrackForming = MINF(Occupancy(scene.images[0], 20), Occupancy(scene.images[1], 20));
	const unsigned occupiedSparse = MINF(Occupancy(scene.images[0], 5), Occupancy(scene.images[1], 5));
	const float expectedSpatial = (float)occupiedTrackForming/100.f * ImagePair::ComputeAngleBaselineWeight(R2D(pair.meanRayAngle));
	TEST_CHECK(occupiedTrackForming > occupiedSparse && ABS(weighted.weightSpatial - expectedSpatial) <= 1e-5f,
		"weightSpatial is %.6f, expected %.6f (%u/100 track-forming grid occupancy, against %u/100 sparse, x the sparse angle score)",
		weighted.weightSpatial, expectedSpatial, occupiedTrackForming, occupiedSparse);
	// The MAGNITUDE the area score multiplies is where the supplement is scaled instead of ignored: the pair's evidence is
	// 5 sparse + 50/2000 x 15 dense = 5.375 at the default frame scale, more than the sparse count alone and far less than the
	// twenty a full count would give, so a coverage-maximising draw cannot re-rank the graph by sheer count, and a dense-only
	// pair is still not a zero.
	TEST_CHECK(ABS(weighted.weightedInliers - 5.375f) <= 1e-5f && weighted.GetNumWeightedInliers() == 5 && weighted.GetNumFilteredInliers() == 5,
		"the pair's evidence is %.4f (%u rounded) with %u sparse, expected 5 + 50/2000 x 15 = 5.375 (5)",
		weighted.weightedInliers, weighted.GetNumWeightedInliers(), weighted.GetNumFilteredInliers());

	// THE DEGENERATE BASELINE OF A DENSE-ONLY PAIR. The angle term is the only factor of weightSpatial that can demote a pair for
	// a bad baseline, and a pair whose evidence is entirely dense used to be structurally unable to reach it: no sparse match
	// meant no meanRayAngle, and ComputeAngleBaselineWeight scores an unmeasured baseline at its MAXIMUM. Two dense-only pairs
	// built through AppendDenseMatches -- the path the matcher uses -- differing in NOTHING but the depth of what they see: the
	// same eighty image positions, so the grid occupancy and hence the area score are identical by construction, and the same
	// 1-unit baseline. The far one must come out strictly weaker, and strictly below the area score it would have carried with
	// an unmeasured baseline.
	const auto DenseOnlyWeight = [&](REAL depth, float& weightSpatial, float& rayAngleDeg) -> bool {
		Scene denseScene;
		BuildTwoCameraScene(denseScene);
		// eighty points spread over the frame at a fixed DEPTH: one weighted inlier at the default frame scale (twenty would be
		// a quarter and round away as no evidence, and a real dense-only pair carries hundreds): a pixel of image 0
		// back-projected to the bearing that reaches that depth, so the A-side positions are the same whatever the depth
		std::vector<Point2f> pointsA, pointsB;
		std::vector<float> confidences;
		for (unsigned gy = 0; gy < 8; ++gy) {
			for (unsigned gx = 0; gx < 10; ++gx) {
				const Point2f ptA(200.f + 40.f*(float)gx, 60.f + 50.f*(float)gy);
				const Point3 bearing = denseScene.cameras[0]->UnprojectNormalized(Cast<REAL>(ptA));
				const Point3 X = TWO_CAMERA_CENTRES[0] + bearing*(depth/bearing.z);
				const auto [proj, valid] = denseScene.images[1].ProjectPoint(X);
				TEST_CHECK(valid && Image8U::isInside(Cast<float>(proj), denseScene.images[1].GetSize()), "the dense-only point of depth %g is outside image 1", depth);
				pointsA.push_back(ptA);
				pointsB.push_back(Cast<float>(proj));
				confidences.push_back(0.9f);
			}
		}
		ImagePair& densePair = denseScene.pairs.emplace_back(0u, 1u);
		densePair.relativePose = denseScene.images[1] / denseScene.images[0];
		// the dense-only construction: no sparse match ever, the whole match set appended here
		TEST_CHECK(AppendDenseMatches(denseScene, densePair, pointsA, pointsB, confidences, cv::Size(160, 160)) == pointsA.size() &&
			densePair.GetNumFilteredInliers() == 0 && densePair.GetNumDenseInliers() == pointsA.size(), "the dense-only pair of depth %g did not take its %zu matches", depth, pointsA.size());
		ComputePairsWeights(denseScene);
		weightSpatial = densePair.weightSpatial;
		rayAngleDeg = (float)R2D(densePair.meanRayAngle);
		return true;
	};
	float farSpatial = 0.f, nearSpatial = 0.f, farAngleDeg = 0.f, nearAngleDeg = 0.f;
	TEST_CHECK(DenseOnlyWeight(REAL(200), farSpatial, farAngleDeg) && DenseOnlyWeight(REAL(4), nearSpatial, nearAngleDeg), "could not build the two dense-only fixtures");
	// the two really are a weak and a wide baseline, measured off the dense matches alone
	TEST_CHECK(farAngleDeg > 0.f && farAngleDeg <= 1.f && nearAngleDeg >= 10.f, "the dense-only pairs measured %.4f deg and %.4f deg, expected a sub-degree and a wide "
		"baseline -- a dense-only pair is not measuring its baseline at all", farAngleDeg, nearAngleDeg);
	// ...and the weak one is demoted for it, both against the wide one and against the score an unmeasured baseline would have
	// handed it (the area alone, since that factor would be 1.0)
	const float areaOnly = nearSpatial/ImagePair::ComputeAngleBaselineWeight(nearAngleDeg);
	TEST_CHECK(farSpatial < nearSpatial && farSpatial < areaOnly*0.9f, "the %.4f-deg dense-only pair scores %.6f against %.6f for the %.4f-deg one (area alone would be %.6f) "
		"-- a degenerate baseline is not demoted", farAngleDeg, farSpatial, nearSpatial, nearAngleDeg, areaOnly);
	return scope.Pass("the terms of the weight, the composite formula, the dense evidence at the frame target and the supplement's angle, area, magnitude and floor");
}

// The camera-triplet view-graph filter of Manam & Govindu (CVPR 2024): the scores, the yield rule and both modes on
// hand-computed graphs (PairGraph: a pair carries an inlier count, its matches' coverage and a verification flag, so neither
// descriptors nor real images are needed).
bool TripletFilterTest()
{
	TestScope scope("TripletFilterTest");
	constexpr float eps = 1e-6f;
	struct Spec { IIndex idA, idB; unsigned numInliers; bool verified; };
	const auto Build = [](Scene& scene, IIndex numImages, const std::vector<Spec>& specs) {
		PairGraph g(scene, numImages);
		for (const Spec& spec : specs)
			g.Matched(spec.idA, spec.idB, spec.numInliers, spec.verified);
	};

	// 1. The scores: an 8-node graph with two triplet-graph components, triangles {0,1,2} and
	// {1,2,3} sharing edge (1,2), plus the isolated triangle {5,6,7}; G_LCT is the four nodes and
	// five edges of the first component, scored by the mean over its triangles of s_ij / max s_kl.
	// A 1-cell grid gives every pair full coverage, so strength = inlier count.
	PairsWeightingConfig cellCfg; cellCfg.gridSize = 1;
	const std::vector<Spec> pairSpecs = {
		{0,1,100,true}, {0,2,100,true}, {1,2,70,true}, {1,3,50,true}, {2,3,40,true},
		{3,4,30,true}, {3,5,20,true}, {5,6,10,true}, {6,7,10,true}, {5,7,10,true}
	};
	Scene scene; Build(scene, 8, pairSpecs);
	const TripletScores scores03 = ComputeTripletScores(scene, 0.3f, 0.f, cellCfg);
	TEST_CHECK(scores03.numTriplets == 3 && scores03.numTripletComponents == 2 && scores03.numScoredPairs == 5 && scores03.numNodes == 4 && scores03.maxDegree == 3,
		"statistics %zu triplets in %u components, %u scored pairs, %u nodes, max degree %u (expected 3, 2, 5, 4, 3)",
		scores03.numTriplets, scores03.numTripletComponents, scores03.numScoredPairs, scores03.numNodes, scores03.maxDegree);
	const float expectedScores[] = {1.f, 1.f, 0.85f, 50.f/70.f, 40.f/70.f, -1.f, -1.f, -1.f, -1.f, -1.f};
	FOREACH(i, scores03.scores)
		TEST_CHECK(ABS(scores03.scores[i] - expectedScores[i]) <= eps, "pair %u (%u,%u) scored %g, expected %g", (unsigned)i, scene.pairs[i].ID1, scene.pairs[i].ID2, scores03.scores[i], expectedScores[i]);
	// tau = m(1 - d_max/|V|) + d_max/|V| = m*0.25 + 0.75: 0.825 at m=0.3, 0.9 at m=0.6, and the
	// scores at m=0.6 (spot-checked) do not depend on m.
	const TripletScores scores06 = ComputeTripletScores(scene, 0.6f, 0.f, cellCfg);
	TEST_CHECK(ABS(scores03.tau - 0.825f) <= eps && ABS(scores06.tau - 0.9f) <= eps && ABS(scores06.scores[2] - expectedScores[2]) <= eps && ABS(scores06.scores[3] - expectedScores[3]) <= eps,
		"tau %g at m=0.3 (expected 0.825), %g at m=0.6 (expected 0.9); scores at m=0.6 %g %g (expected %g %g)",
		scores03.tau, scores06.tau, scores06.scores[2], scores06.scores[3], expectedScores[2], expectedScores[3]);
	// A weaker duplicate of (0,1) collapses onto the same edge and shares its score; an unverified
	// pair and a self-pair are never edges (each would close a new triangle if it were one, so the
	// triplet count staying at 3 proves they were left out), and a pair without an inlier has no
	// strength to score and stays unscored.
	std::vector<Spec> dupSpecs(pairSpecs);
	dupSpecs.insert(dupSpecs.end(), {Spec{0, 1, 10, true}, Spec{0, 3, 25, false}, Spec{2, 4, 0, true}, Spec{3, 3, 30, true}});
	Scene sceneDup; Build(sceneDup, 8, dupSpecs);
	const TripletScores scoresDup = ComputeTripletScores(sceneDup, 0.3f, 0.f, cellCfg);
	TEST_CHECK(scoresDup.numTriplets == 3 && scoresDup.numScoredPairs == 6 && scoresDup.numNodes == 4 && scoresDup.maxDegree == 3 &&
		ABS(scoresDup.scores[10] - scoresDup.scores[0]) <= eps && scoresDup.scores[11] == -1.f && scoresDup.scores[12] == -1.f && scoresDup.scores[13] == -1.f,
		"duplicate scene %zu triplets, %u scored, %u nodes, max degree %u; duplicate %g vs %g, unverified/no-inlier/self-pair %g %g %g "
		"(expected 3, 6, 4, 3; equal; all -1)", scoresDup.numTriplets, scoresDup.numScoredPairs, scoresDup.numNodes, scoresDup.maxDegree,
		scoresDup.scores[10], scoresDup.scores[0], scoresDup.scores[11], scoresDup.scores[12], scoresDup.scores[13]);

	// 2. The coverage discount: three images, a triangle whose (0,2) edge touches only 10 of the
	// 100 cells of a 10x10 grid over image 0 while (0,1) and (1,2) touch every cell, so (0,2)
	// scores 90/600 = 0.15 (a tenth of its raw 900 inliers); on a 2x2 grid the same ten cells are
	// its whole top row, half the coarse grid, so it scores 450/600 = 0.75 instead.
	PairsWeightingConfig weightingCfg; // default gridSize (10)
	Scene sceneCov;
	{
		PairGraph g(sceneCov, 3);
		g.Matched(0, 1, 600); g.Matched(1, 2, 600); g.Matched(0, 2, 900, true, 10, 100);
	}
	const TripletScores covScores = ComputeTripletScores(sceneCov, 0.f, 0.f, weightingCfg);
	PairsWeightingConfig coarseCfg; coarseCfg.gridSize = 2;
	const TripletScores coarseScores = ComputeTripletScores(sceneCov, 0.f, 0.f, coarseCfg);
	TEST_CHECK(covScores.numTriplets == 1 && covScores.numScoredPairs == 3 && ISEQUAL(covScores.scores[0], 1.f) && ISEQUAL(covScores.scores[1], 1.f) &&
		ISEQUAL(covScores.scores[2], 0.15f) && ISEQUAL(coarseScores.scores[2], 0.75f),
		"coverage scene %zu triplets, %u scored, scores %g %g %g on the default grid, %g on 2x2 (expected 1, 3, 1 1 0.15, 0.75)",
		covScores.numTriplets, covScores.numScoredPairs, covScores.scores[0], covScores.scores[1], covScores.scores[2], coarseScores.scores[2]);
	// The cutting rule at m = 0.5: G_LCT has 3 nodes and max degree 2, so tau = 0.5(1-2/3)+2/3 =
	// 0.833, applied as given. secondFaceScore (0.75, above minScore) is checked too and the
	// descent always runs when the ceiling shatters the graph; reading the source, neither changes
	// anything on three nodes (the two 1.0 edges hold every node in one piece at both ceilings), so
	// only the doppelganger (0,2) goes.
	TripletFilterConfig covCfg; covCfg.enabled = true; covCfg.cut = true; covCfg.minScore = 0.5f; covCfg.minYield = 0.f;
	const unsigned covRemoved = FilterPairsByTriplets(sceneCov, covCfg, weightingCfg);
	const std::set<std::pair<IIndex,IIndex>> covExpected{{0,1}, {1,2}};
	TEST_CHECK(covRemoved == 1 && KeptPairs(sceneCov) == covExpected, "coverage scene removed %u pairs; expected the doppelganger (0,2) alone, (0,1) and (1,2) kept", covRemoved);

	// 3. The yield rule: a 15-image walk in five 3-image copies (0-2, 3-5, ..., 12-14); consecutive
	// images share 1000 inliers at 2 degrees, images two apart 600 at 4 degrees, and the first
	// image of every copy pairs with the first image of every other copy on 300 inliers at 1
	// degree -- a near-duplicate viewpoint delivering far fewer inliers than a real pair at that
	// ray angle, so a triangle built only from those look-alike pairs gives no evidence.
	const auto BuildWalk = [](Scene& s) {
		PairGraph g(s, 15);
		const auto add = [&g](IIndex a, IIndex b, unsigned numInliers, float rayAngleDeg) {
			g.Matched(a, b, numInliers).meanRayAngle = (float)D2R(rayAngleDeg);
		};
		for (IIndex i = 0; i + 1 < 15; ++i) add(i, i + 1, 1000, 2.f);
		for (IIndex i = 0; i + 2 < 15; ++i) add(i, i + 2, 600, 4.f);
		for (IIndex a = 0; a < 15; a += 3)
			for (IIndex b = a + 3; b < 15; b += 3)
				add(a, b, 300, 1.f);
	};
	Scene walk; BuildWalk(walk);
	const unsigned idx01 = 0, idx02 = 14, idx03 = 27, idx06 = 28; // in order of insertion
	const TripletFilterConfig yieldDefaults;
	const TripletScores yieldScores = ComputeTripletScores(walk, 0.f, yieldDefaults.minYield, weightingCfg);
	const TripletScores yieldOff = ComputeTripletScores(walk, 0.f, 0.f, weightingCfg); // rule off: look-alike triangles count like any other
	TEST_CHECK(yieldScores.numTriplets == 31 && yieldScores.numDoppelgangerTriplets == 10 && yieldScores.numScoredPairs == 37 &&
		ISEQUAL(yieldScores.scores[idx01], 1.f) && ISEQUAL(yieldScores.scores[idx02], 0.6f) && ISEQUAL(yieldScores.scores[idx03], 0.12f) && ISEQUAL(yieldScores.scores[idx06], 0.f) &&
		yieldOff.numDoppelgangerTriplets == 0 && ISEQUAL(yieldOff.scores[idx03], 0.72f) && ISEQUAL(yieldOff.scores[idx06], 1.f),
		"walk %zu triplets (%u doppelganger), %u scored; (0,1) %g (0,2) %g (0,3) %g (0,6) %g; rule off (0,3) %g (0,6) %g "
		"(expected 31 (10), 37; 1 0.6 0.12 0; 0.72 1)", yieldScores.numTriplets, yieldScores.numDoppelgangerTriplets, yieldScores.numScoredPairs,
		yieldScores.scores[idx01], yieldScores.scores[idx02], yieldScores.scores[idx03], yieldScores.scores[idx06], yieldOff.scores[idx03], yieldOff.scores[idx06]);
	// A ray angle that is not finite has no measurable geometry: giving (0,6) a NaN angle takes it
	// out of every envelope bin, so its own yield reverts to 1 and the three doppelganger triangles
	// it took part in -- (0,3,6), (0,6,9), (0,6,12) -- stop being doppelgangers, leaving 7; (0,3)'s
	// triangle with 6 now scores 1 instead of 0, raising its mean from 0.6 to 1.6 over 5 triplets, 0.32.
	Scene walkNaN; BuildWalk(walkNaN);
	walkNaN.pairs[idx06].meanRayAngle = std::numeric_limits<float>::quiet_NaN();
	const TripletScores nanScores = ComputeTripletScores(walkNaN, 0.f, yieldDefaults.minYield, weightingCfg);
	TEST_CHECK(nanScores.numDoppelgangerTriplets == 7 && ISEQUAL(nanScores.scores[idx03], 0.32f) && ISEQUAL(nanScores.scores[idx01], 1.f) && ISEQUAL(nanScores.scores[idx02], 0.6f),
		"NaN ray angle on (0,6), %u doppelganger triplets; (0,1) %g (0,2) %g (0,3) %g (expected 7; 1 0.6 0.32)",
		nanScores.numDoppelgangerTriplets, nanScores.scores[idx01], nanScores.scores[idx02], nanScores.scores[idx03]);
	// The cutting rule at m = 0.6: G_LCT has 15 nodes and max degree 8 (image 3), so tau =
	// 0.6(1-8/15)+8/15 = 0.813; the fourteen consecutive pairs alone score 1 and already hold all
	// 15 images together, so the ceiling applies as given and only they survive.
	TripletFilterConfig yieldFilterCfg; yieldFilterCfg.enabled = true; yieldFilterCfg.cut = true; yieldFilterCfg.minScore = 0.6f;
	const unsigned yieldRemoved = FilterPairsByTriplets(walk, yieldFilterCfg, weightingCfg);
	std::set<std::pair<IIndex,IIndex>> yieldExpected;
	for (IIndex i = 0; i + 1 < 15; ++i) yieldExpected.emplace(i, i + 1);
	TEST_CHECK(yieldRemoved == 23 && KeptPairs(walk) == yieldExpected, "walk removed %u pairs; expected 23, keeping only the 14 consecutive pairs", yieldRemoved);

	// 4. The cutting rule and the default mode on two rooms of 60 and 30 images (consecutive pairs 1000 inliers, two apart
	// 600) joined by three weak pairs, (59,60)=60, (58,60)=50, (59,61)=50: no image holds more than
	// four pairs, so G_LCT has |V|=90, d_max=4 and tau = 0.3(1-4/90)+4/90 = 0.331, below which only
	// the three bridges sit.
	PairsWeightingConfig roomsCfg; roomsCfg.gridSize = 1;
	const auto BuildRooms = [](Scene& s) {
		PairGraph g(s, 90);
		for (IIndex i = 0; i + 1 < 60; ++i) g.Matched(i, i + 1, 1000);
		for (IIndex i = 0; i + 2 < 60; ++i) g.Matched(i, i + 2, 600);
		for (IIndex i = 60; i + 1 < 90; ++i) g.Matched(i, i + 1, 1000);
		for (IIndex i = 60; i + 2 < 90; ++i) g.Matched(i, i + 2, 600);
		g.Matched(59, 60, 60); g.Matched(58, 60, 50); g.Matched(59, 61, 50);
	};
	{
		// The cutting rule removes all three bridges: the larger room's 60 images seed the reconstruction, and the
		// scores handed over are those of the 174 pairs left.
		Scene rooms; BuildRooms(rooms);
		TripletFilterConfig cutCfg; cutCfg.enabled = true; cutCfg.cut = true; cutCfg.minYield = 0.f;
		IIndexArr seeds;
		std::vector<float> scores;
		const unsigned removed = FilterPairsByTriplets(rooms, cutCfg, roomsCfg, &seeds, &scores);
		TEST_CHECK(removed == 3 && KeptPairs(rooms).size() == 174 && seeds.size() == 60 && seeds.front() == 0 && seeds.back() == 59,
			"rooms cutting rule removed %u pairs, %u seeds; expected 3 removed, 174 kept, 60 seed images 0-59", removed, (unsigned)seeds.size());
		TEST_CHECK(scores.size() == 174 && scores == ComputeTripletScores(rooms, cutCfg.minScore, cutCfg.minYield, roomsCfg).scores,
			"rooms cutting rule handed over %zu scores; expected those of the 174 pairs left", scores.size());
	}
	{
		// The default mode removes nothing, whatever the graph could spare, and names the same seed views:
		// the larger room's 60 images; it hands over the scores it computed, one per pair, the bridges
		// below the ceiling among them.
		Scene rooms; BuildRooms(rooms);
		TripletFilterConfig defaultCfg; defaultCfg.enabled = true; defaultCfg.minYield = 0.f;
		IIndexArr seeds;
		std::vector<float> scores;
		const unsigned removed = FilterPairsByTriplets(rooms, defaultCfg, roomsCfg, &seeds, &scores);
		TEST_CHECK(removed == 0 && rooms.pairs.size() == 177 && seeds.size() == 60 && seeds.front() == 0 && seeds.back() == 59,
			"rooms default mode removed %u of 177 pairs, %u seeds; expected none removed, 60 seed images 0-59", removed, (unsigned)seeds.size());
		const TripletScores expected = ComputeTripletScores(rooms, defaultCfg.minScore, defaultCfg.minYield, roomsCfg);
		TEST_CHECK(scores.size() == 177 && scores == expected.scores && expected.numScoredPairs == 177 &&
			scores[174] < expected.tau && scores[175] < expected.tau && scores[176] < expected.tau,
			"rooms default mode handed over %zu scores (ceiling %g); expected the 177 scores of ComputeTripletScores with the three bridges below the ceiling",
			scores.size(), expected.tau);
	}
	{
		// A disabled filter removes nothing, reports no seed views and leaves every pair unscored.
		Scene rooms; BuildRooms(rooms);
		TripletFilterConfig offCfg; offCfg.enabled = false;
		IIndexArr seeds;
		std::vector<float> scores;
		const unsigned removed = FilterPairsByTriplets(rooms, offCfg, roomsCfg, &seeds, &scores);
		TEST_CHECK(removed == 0 && rooms.pairs.size() == 177 && seeds.empty() && scores == std::vector<float>(177, TripletScores::unscored),
			"disabled filter removed %u pairs (%u seeds, %zu scores); expected none, no seeds, 177 unscored", removed, (unsigned)seeds.size(), scores.size());
	}
	{
		// A scene without a verified pair has nothing to score: its pairs are all unscored.
		Scene bare;
		PairGraph g(bare, 3);
		g.Matched(0, 1, 100, false); g.Matched(1, 2, 100, false);
		std::vector<float> scores;
		const unsigned removed = FilterPairsByTriplets(bare, TripletFilterConfig(), roomsCfg, NULL, &scores);
		TEST_CHECK(removed == 0 && scores == std::vector<float>(2, TripletScores::unscored), "scene without a verified pair removed %u pairs, %zu scores; expected none and 2 unscored", removed, scores.size());
	}
	return scope.Pass("the scores, the coverage discount, the yield rule and both modes of the cut");
}

// The global rotation estimator (GlobalRotationEstimator):
// (1) on the 16-camera ring (RingConfig) with perturbed poses and pair poses, image 2 disconnected by zero weights, the
//     recovered rotations match the truth relative to image 0 within 5 degrees (the gauge is free);
// (2) the rotation averaging does not depend on the absolute scale of the pair weights: a graph of 40 rotations joined by
//     noisy pairs, a tenth of them wrong and light, with weights spread over three orders of magnitude, solved with its
//     weights as given and multiplied by 2^-7 and by 2^7 (powers of two, so the scaled weights are exact), gives
//     bit-identical rotations and removes the same pairs past the 12-degree bar. Without the estimator's median scaling the
//     L1 step's fixed-iteration ADMM sees a different problem at each scale and the rotations differ.
bool RotationAveragingTest()
{
	TestScope scope("RotationAveragingTest");

	// 1. the ring
	constexpr REAL toleranceDeg = 5.0;
	REAL maxAngleError = 0;
	{
		Scene sceneGT, scene;
		SceneConfig cfg = RingConfig(200);
		cfg.perturbOptions = SceneConfig::PERTURB_POSES | SceneConfig::PERTURB_PAIR_POSES; // perturb poses to test averaging
		GenerateTestScene(sceneGT, cfg, &scene);
		// remove poses to simulate unknown rotations
		for (Image& img : scene.images)
			img.InvalidatePose();
		ComputePairsWeights(scene);
		// disconnect image 2 by setting an artificial weight of 0 for all pairs involving it
		const uint32_t disconnectedImageId = 2;
		for (ImagePair& pair : scene.pairs)
			if (pair.ID1 == disconnectedImageId || pair.ID2 == disconnectedImageId)
				pair.InvalidateWeight();
		const GlobalRotationEstimatorOptions options;
		GlobalRotationEstimator estimator(options);
		TEST_CHECK(estimator.EstimateRotations(scene), "GlobalRotationEstimator::EstimateRotations failed on the ring");
		// the gauge: compare the rotations relative to the first image; the disconnected image stays invalid
		const RMatrix R0_gt = sceneGT.images[0].R;
		const RMatrix R0_est = scene.images[0].R;
		for (size_t i = 1; i < scene.images.size(); ++i) {
			if (i == disconnectedImageId)
				continue;
			const double angleError = RotationErrorDeg(scene.images[i].R * R0_est.t(), sceneGT.images[i].R * R0_gt.t());
			maxAngleError = MAXF(maxAngleError, angleError);
			TEST_CHECK(angleError <= toleranceDeg, "image %zu relative rotation error %.2f deg, tolerance %.2f deg", i, angleError, toleranceDeg);
		}
	}

	// 2. the weight scale
	constexpr uint32_t NUM_NODES = 40;
	std::mt19937 rng(20261112);
	std::uniform_real_distribution<double> uni(-1.0, 1.0);
	std::normal_distribution<double> noise(0.0, D2R(1.0));
	std::uniform_real_distribution<double> logWeight(0.0, 3.0);
	const auto RandomAxisAngle = [&](double maxAngle) {
		Point3 axis(uni(rng), uni(rng), uni(rng));
		axis *= maxAngle * ABS(uni(rng)) / norm(axis);
		return axis;
	};
	std::vector<RMatrix> truth(NUM_NODES);
	for (RMatrix& R : truth)
		R = RMatrix(RandomAxisAngle(M_PI * 0.9));
	std::vector<RotationPair> pairs;
	for (uint32_t a = 0; a < NUM_NODES; ++a) {
		for (uint32_t b = a + 1; b < NUM_NODES; ++b) {
			if (b - a > 3 && uni(rng) < 0.85)
				continue; // a band around the chain, a few chords
			RMatrix rel(truth[b] * truth[a].t());
			const bool wrong = uni(rng) < -0.8;
			if (wrong)
				rel = RMatrix(RandomAxisAngle(M_PI * 0.9)); // a wrong pair, weighing at most 10
			else
				rel = RMatrix(Point3(noise(rng), noise(rng), noise(rng))) * rel;
			pairs.emplace_back(a, b, Matrix3x3(rel), (float)POW(10.0, wrong ? logWeight(rng) / 3 : logWeight(rng)));
		}
	}
	const auto Solve = [&](float factor, std::vector<Point3>& rotations, unsigned& numFiltered) {
		std::vector<RotationPair> scaled(pairs);
		for (RotationPair& p : scaled)
			p.weight *= factor;
		const GlobalRotationEstimatorOptions options;
		GlobalRotationEstimator estimator(options);
		rotations.clear();
		if (!estimator.EstimateRotations(scaled, NUM_NODES, rotations))
			return false;
		numFiltered = GlobalRotationEstimator::FilterRelativeRotations(rotations, scaled, options.maxRelativeRotationAngle);
		return true;
	};
	std::vector<Point3> reference;
	unsigned referenceFiltered;
	TEST_CHECK(Solve(1.f, reference, referenceFiltered), "the estimator failed on the weights as given");
	for (const float factor : {1.f / 128.f, 128.f}) {
		std::vector<Point3> rotations;
		unsigned numFiltered;
		TEST_CHECK(Solve(factor, rotations, numFiltered), "the estimator failed on the weights times %g", factor);
		double maxDifference = 0;
		for (uint32_t i = 0; i < NUM_NODES; ++i)
			maxDifference = MAXF(maxDifference, (double)norm(rotations[i] - reference[i]));
		TEST_CHECK(maxDifference == 0 && numFiltered == referenceFiltered, "the weights times %g move the rotations by up to %g degrees and filter %u pairs against %u; "
			"the result must not depend on the scale of the weights", factor, R2D(maxDifference), numFiltered, referenceFiltered);
	}
	return scope.Pass("the ring within %.2f deg of the truth; %u pairs, %u past the bar, the same rotations at weights x1, x1/128 and x128",
		maxAngleError, (unsigned)pairs.size(), referenceFiltered);
}

// The contradiction cut (FilterPairsByContradiction).
// Its rotation stage, on hand-made view graphs (PairGraph) of pairs of a given relative rotation about the vertical axis and
// composite weight: (1) an eight-fold symmetric ring, the Palace case -- four look-alike pairs between opposite images claim
// the same view, heavier in composite weight than the true neighbour pairs, lighter once weighted by their triplet score --
// lands in the true basin from the score-weighted tree and removes exactly the look-alikes, while the same graph unscored
// starts in the symmetric mode; (2) a bridge, the only pair joining two consistent groups, is fitted exactly and never
// removed, however far its rotation is from anything; (3) a wrong pair whose inlier count is far past the weight cap weighs
// what the cap says (1000) before its triplet score scales it, and the ring of right pairs outvotes it; (4) on the 16-camera
// arc (RingConfig), the cross-face pairs of a four-fold symmetric roof, a quarter and a half turn off and too weak to be
// usable themselves, are judged against the rotations the other pairs give and removed, a pair a few degrees off, a loop's
// drift, stays; (5) the translation stage's bar takes both its ends.
// Its translation stage (bar above 0), on the arc with exact relative poses and every ray angle 10 degrees: (6) a pair whose
// baseline sign is flipped and one whose baseline is turned 120 degrees, the rotation exact, are removed at the default bar
// (45) and nothing else is; (7) the one pair joining two groups of cameras, its baseline flipped, is a bridge and is never
// removed; (8) a pair under 2 degrees of ray angle is never judged (the same pair at 10 degrees is removed); (9) with the bar
// 0 the cut is the rotation stage alone: a pair 30 degrees off in rotation goes at either bar, the flipped and the turned
// baselines only at 45; (10) with the repair on, the planted pairs whose matches still agree come back with the exact pose,
// none counted as removed, the weights final before it, and a pair whose matches agree with nothing stays removed; (11) a
// connected graph of fewer pairs than a scale component needs is not solved and removes nothing; (12) two components are
// averaged and solved apart, the flipped pair of the smaller judged by its own solve.
bool ContradictionCutTest()
{
	TestScope scope("ContradictionCutTest");
	// add a pair of the given relative rotation about the vertical axis (degrees) and composite weight
	// (inlier count at full quality; the matches, which nothing here reads, capped at 40)
	const auto addPair = [](PairGraph& g, IIndex a, IIndex b, REAL degrees, unsigned numInliers) {
		ImagePair& pair = g.Matched(a, b, MINF(numInliers, 40u));
		if (a > b) // the pair holds its smaller index first: the rotation from it is the inverse
			degrees = -degrees;
		pair.relativePose = Pose3D(RMatrix(Point3(0, D2R(degrees), 0)), CMatrix(1, 0, 0));
		pair.weightSpatial = pair.weightConnectivity = 1.f;
		pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
		pair.weightTriplet = 0.75f; // 0.25 + 0.75 = 1: the weight is the inlier count
		pair.weightedInliers = (float)numInliers;
	};
	constexpr float minPairWeight = kCutMinPairWeight;
	const PairsWeightingConfig weightingCfg;

	// (1) the octagon: the ring of eight neighbour pairs of 45 degrees (weight 20 each), and the four
	// opposite pairs claiming no rotation at all (weight 100 each), which a spanning tree by composite
	// weight takes for the truth
	const std::set<std::pair<IIndex,IIndex>> lookAlikes{{0,4}, {1,5}, {2,6}, {3,7}};
	const auto buildOctagon = [&](Scene& scene) {
		PairGraph g(scene, 8);
		for (IIndex k = 0; k < 8; ++k)
			addPair(g, k, (k + 1) % 8, k == 7 ? REAL(-315) : REAL(45), 20);
		for (const auto& ids : lookAlikes)
			addPair(g, ids.first, ids.second, 0, 100);
	};
	{
		Scene scene; buildOctagon(scene);
		std::vector<float> scores(scene.pairs.size(), 1.f);
		FOREACH(idxPair, scene.pairs)
			if (lookAlikes.count({scene.pairs[idxPair].ID1, scene.pairs[idxPair].ID2}))
				scores[idxPair] = 0.05f;
		const unsigned removed = FilterPairsByContradiction(scene, scores, minPairWeight, 0.f, weightingCfg);
		TEST_CHECK(!(removed != lookAlikes.size() || KeptPairs(scene).size() != 8), "the scored octagon removed %u pairs, expected %zu look-alikes", removed, lookAlikes.size());
		for (const auto& ids : lookAlikes)
			TEST_CHECK(!(KeptPairs(scene).count(ids)), "look-alike pair (%u,%u) kept", ids.first, ids.second);
	}
	{
		// unscored, the same graph starts in the symmetric mode and cuts the true ring instead
		Scene scene; buildOctagon(scene);
		const std::vector<float> scores(scene.pairs.size(), TripletScores::unscored);
		FilterPairsByContradiction(scene, scores, minPairWeight, 0.f, weightingCfg);
		bool allLookAlikes = true;
		for (const auto& ids : lookAlikes)
			allLookAlikes = allLookAlikes && !KeptPairs(scene).count(ids);
		TEST_CHECK(!(allLookAlikes), "the unscored octagon removed the look-alikes: the start no longer decides the basin");
	}

	// (2) a bridge: two rings of four images (true 90 degrees each) joined by the one pair (3,4), claiming
	// a rotation of 100 degrees; and an image hanging on one pair
	{
		Scene scene;
		PairGraph g(scene, 9);
		for (IIndex k = 0; k < 4; ++k) {
			addPair(g, k, (k + 1) % 4, k == 3 ? REAL(-270) : REAL(90), 200);
			addPair(g, 4 + k, 4 + (k + 1) % 4, k == 3 ? REAL(-270) : REAL(90), 200);
		}
		addPair(g, 3, 4, 100, 5);
		addPair(g, 8, 0, 179, 4);
		const size_t numPairs = scene.pairs.size();
		const std::vector<float> scores(numPairs, 1.f);
		TEST_CHECK(!(FilterPairsByContradiction(scene, scores, 0.f, 0.f, weightingCfg) != 0 || scene.pairs.size() != numPairs), "a bridge or a pendant pair was removed");
	}

	// (3) a capped-weight pair: eight right pairs of a ring (50000 inliers each, weight 1000 at the cap) against
	// one wrong pair of 400000 inliers (also 1000 at the cap) scoring 0.05
	{
		Scene scene;
		PairGraph g(scene, 8);
		for (IIndex k = 0; k < 8; ++k)
			addPair(g, k, (k + 1) % 8, k == 7 ? REAL(-315) : REAL(45), 50000);
		addPair(g, 0, 4, 0, 400000);
		TEST_CHECK(!(scene.pairs.back().GetCompositeWeight() != 1000.f), "the weight of a pair past the cap is %g, expected 1000", scene.pairs.back().GetCompositeWeight());
		// its score of 0.05 leaves it 50 against the ring's 1000; read at the raw count it would weigh 20000
		std::vector<float> scores(scene.pairs.size(), 1.f);
		scores.back() = 0.05f;
		const unsigned removed = FilterPairsByContradiction(scene, scores, minPairWeight, 0.f, weightingCfg);
		TEST_CHECK(!(removed != 1 || KeptPairs(scene).count({0, 4}) || KeptPairs(scene).size() != 8), "the capped wrong pair: %u pairs removed, expected only the wrong one", removed);
	}

	// (4) the tree check: the cross-face pairs of a four-fold roof (images 0-15 over a 240 degree arc), two a
	// quarter turn off and two a half turn off, weak as the cross-face pairs of the captures are (composite
	// weight under 6 against over 34 for every other pair); and a pair 8 degrees off, under the 12 degrees the
	// estimator allows, which stays. Once with the weak pairs in the averaging, outweighed, and once under the
	// minimum pair weight (10), so that they are not in it at all and are judged all the same.
	for (const float weightBar : {minPairWeight, 10.f}) {
		Scene sceneGT, scene;
		GenerateTestScene(sceneGT, RingConfig(200), &scene);
		ComputePairsWeights(scene);
		const RMatrix quarterTurn(Point3(0, D2R(REAL(90)), 0));
		const RMatrix halfTurn(Point3(0, D2R(REAL(180)), 0));
		const RMatrix drift(Point3(0, D2R(REAL(8)), 0));
		const std::set<std::pair<IIndex,IIndex>> crossFace{{0,8}, {1,9}, {3,12}, {5,14}};
		const std::pair<IIndex,IIndex> drifting(2, 11);
		const size_t numPairs = scene.pairs.size();
		for (ImagePair& pair : scene.pairs) {
			const std::pair<IIndex,IIndex> ids(pair.ID1, pair.ID2);
			const bool isCrossFace = crossFace.count(ids) != 0;
			if (!isCrossFace && ids != drifting)
				continue;
			pair.weightSpatial *= 1e-3f;
			pair.relativePose->R = (isCrossFace ? (pair.ID1 < 2 ? quarterTurn : halfTurn) : drift) * pair.relativePose->R;
		}
		const std::vector<float> scores(scene.pairs.size(), TripletScores::unscored);
		const unsigned removed = FilterPairsByContradiction(scene, scores, weightBar, 0.f, weightingCfg);
		const auto kept = KeptPairs(scene);
		TEST_CHECK(!(removed != crossFace.size() || scene.pairs.size() != numPairs - crossFace.size()), "the roof at minimum weight %g: removed %u pairs, expected %zu", weightBar, removed, crossFace.size());
		for (const auto& ids : crossFace)
			TEST_CHECK(!(kept.count(ids)), "the roof at minimum weight %g: cross-face pair (%u,%u) kept", weightBar, ids.first, ids.second);
		TEST_CHECK(!(!kept.count(drifting)), "the roof at minimum weight %g: the pair 8 degrees off was removed", weightBar);
	}

	// (5) the bar of the translation stage: 0 (no translation stage) and 180 (every direction is within it) are both
	// valid, and the rotation stage's answer is the same
	for (const float bar : {0.f, 180.f}) {
		Scene scene; buildOctagon(scene);
		std::vector<float> scores(scene.pairs.size(), 1.f);
		FOREACH(idxPair, scene.pairs)
			if (lookAlikes.count({scene.pairs[idxPair].ID1, scene.pairs[idxPair].ID2}))
				scores[idxPair] = 0.05f;
		const unsigned removed = FilterPairsByContradiction(scene, scores, minPairWeight, bar, weightingCfg);
		TEST_CHECK(!(removed != lookAlikes.size()), "bar %g: %u pairs removed, expected the %zu look-alikes", bar, removed, lookAlikes.size());
	}

	// the translation stage
	const float maxDirectionResidual = ReconstructionConfig().maxPairDirectionResidual;
	TEST_CHECK(!(!ISEQUAL(maxDirectionResidual, 45.f)), "the default bar of the translation stage is %g degrees, expected 45", maxDirectionResidual);
	const auto turn = [](ImagePair& pair) {
		pair.relativePose->C = RMatrix(Point3(D2R(REAL(70)), D2R(REAL(70)), D2R(REAL(70)))) * pair.relativePose->C;
	};
	{
		Scene scene; BuildArc(scene);
		const size_t numPairs = scene.pairs.size();
		ImagePair* pFlipped = scene.FindPair(3, 9);
		ImagePair* pTurned = scene.FindPair(2, 11);
		TEST_CHECK(!(pFlipped == NULL || pTurned == NULL), "the ring has no pair (3,9) or (2,11)");
		FlipBaseline(*pFlipped);
		turn(*pTurned);
		const unsigned removed = RunCut(scene, maxDirectionResidual);
		const auto kept = KeptPairs(scene);
		TEST_CHECK(!(removed != 2 || kept.size() != numPairs - 2 || kept.count({3, 9}) || kept.count({2, 11})), "the planted flipped and turned pairs: %u removed of %zu, expected exactly those two", removed, numPairs);
		Scene clean; BuildArc(clean);
		TEST_CHECK(!(RunCut(clean, maxDirectionResidual) != 0), "the ring with exact poses lost pairs");
	}
	{
		// two groups (images 0-7, 8-15) joined by the one pair (7,8), its baseline flipped
		Scene scene; BuildArc(scene);
		const IIndex half = 8;
		for (size_t idxPair = scene.pairs.size(); idxPair-- > 0; ) {
			const ImagePair& pair = scene.pairs[idxPair];
			if ((pair.ID1 < half) != (pair.ID2 < half) && !(pair.ID1 == 7 && pair.ID2 == 8))
				scene.pairs.RemoveAt(idxPair);
		}
		ImagePair* pBridge = scene.FindPair(7, 8);
		TEST_CHECK(!(pBridge == NULL), "the ring has no pair (7,8)");
		FlipBaseline(*pBridge);
		const unsigned removed = RunCut(scene, maxDirectionResidual);
		TEST_CHECK(!(removed != 0 || !KeptPairs(scene).count({7, 8})), "the bridge was removed (%u pairs)", removed);
	}
	for (const bool shortRay : {true, false}) {
		Scene scene; BuildArc(scene);
		ImagePair* pPair = scene.FindPair(4, 12);
		TEST_CHECK(!(pPair == NULL), "the ring has no pair (4,12)");
		FlipBaseline(*pPair);
		pPair->meanRayAngle = D2R(shortRay ? 1.5f : 10.f);
		const unsigned removed = RunCut(scene, maxDirectionResidual);
		TEST_CHECK(!(removed != (shortRay ? 0u : 1u)), "the flipped pair at %s ray angle: %u removed", shortRay ? "1.5 degrees of" : "10 degrees of", removed);
	}
	{
		// bar 0: the rotation stage alone. One more pair is 30 degrees off in rotation: it goes at either
		// bar; the flipped and the turned baselines go only when the translation stage runs
		const std::set<std::pair<IIndex,IIndex>> rotated{{6, 14}}, flippedAndTurned{{3, 9}, {2, 11}};
		for (const float bar : {maxDirectionResidual, 0.f}) {
			Scene scene; BuildArc(scene);
			const size_t numPairs = scene.pairs.size();
			ImagePair* pRotated = scene.FindPair(6, 14);
			ImagePair* pFlipped = scene.FindPair(3, 9);
			ImagePair* pTurned = scene.FindPair(2, 11);
			TEST_CHECK(!(pRotated == NULL || pFlipped == NULL || pTurned == NULL), "the ring has no pair (6,14), (3,9) or (2,11)");
			pRotated->relativePose->R = RMatrix(Point3(0, D2R(REAL(30)), 0)) * pRotated->relativePose->R;
			FlipBaseline(*pFlipped);
			turn(*pTurned);
			const unsigned removed = RunCut(scene, bar);
			const auto kept = KeptPairs(scene);
			std::set<std::pair<IIndex,IIndex>> expected(rotated);
			if (bar > 0.f)
				expected.insert(flippedAndTurned.begin(), flippedAndTurned.end());
			bool exact = removed == expected.size() && kept.size() == numPairs - expected.size();
			for (const auto& ids : expected)
				exact = exact && !kept.count(ids);
			for (const auto& ids : flippedAndTurned)
				exact = exact && (bar > 0.f || kept.count(ids));
			TEST_CHECK(!(!exact), "bar %g: %u pairs removed of %zu, expected the %zu pairs of the rotated%s", bar,
					removed, numPairs, expected.size(), bar > 0.f ? ", the flipped and the turned baselines" : " pair alone");
		}
	}
	{
		// the repair (the last argument of the cut): the flipped, the turned and the 30 degrees rotated pair of the ring come
		// back, none counted as removed, each with the rotation of the averaging and the baseline direction its matches
		// give, which are the exact ones; a flipped pair whose matches are shifted by one feature agrees with nothing and
		// stays removed
		const auto plant = [&](Scene& scene) {
			FlipBaseline(*scene.FindPair(3, 9));
			turn(*scene.FindPair(2, 11));
			ImagePair* pRotated = scene.FindPair(6, 14);
			pRotated->relativePose->R = RMatrix(Point3(0, D2R(REAL(30)), 0)) * pRotated->relativePose->R;
			ImagePair* pShifted = scene.FindPair(4, 12);
			FlipBaseline(*pShifted);
			for (size_t i = 0; i + 1 < pShifted->matches.size(); ++i)
				pShifted->matches[i].trainIdx = pShifted->matches[i + 1].trainIdx;
		};
		Scene scene; BuildArc(scene);
		const size_t numPairs = scene.pairs.size();
		const std::set<std::pair<IIndex,IIndex>> planted{{3, 9}, {2, 11}, {6, 14}};
		std::map<std::pair<IIndex,IIndex>, Pose3D> exact;
		for (const auto& [a, b] : planted)
			exact[{a, b}] = scene.FindPair(a, b)->relativePose.value();
		plant(scene);
		const unsigned removed = RunCut(scene, maxDirectionResidual, true);
		const auto kept = KeptPairs(scene);
		TEST_CHECK(!(removed != 1 || kept.size() != numPairs - 1 || kept.count({4, 12})), "the repair: %u pairs removed of %zu, expected only the pair with the shifted matches", removed, numPairs);
		for (const auto& [ids, pose] : exact) {
			const ImagePair& pair = *scene.FindPair(ids.first, ids.second);
			const REAL rotationError = R2D(ACOS(MINF(MAXF(ComputeAngle(pair.relativePose->R, pose.R), REAL(-1)), REAL(1))));
			const REAL directionError = R2D(ACOS(MINF(MAXF(pair.relativePose->C.dot(pose.C) / (norm(pair.relativePose->C) * norm(pose.C)), REAL(-1)), REAL(1))));
			TEST_CHECK(!(!(rotationError < 0.1 && directionError < 0.1) || !pair.E.has_value() || !pair.F.has_value()), "the repaired pair (%u,%u) is %g degrees off in rotation and %g in direction", ids.first, ids.second, rotationError, directionError);
		}
		// the weights are final before the repair: a repaired pair forms no triangle, so its triangle share is 0 and every
		// other pair weighs what it weighs in the graph without the repaired pairs
		Scene without; BuildArc(without);
		plant(without);
		RunCut(without, maxDirectionResidual);
		unsigned numReturned = 0;
		for (const ImagePair& pair : scene.pairs) {
			if (pair.returnedByRepair) {
				++numReturned;
				TEST_CHECK(!(pair.weightTriplet != 0.f), "the repaired pair (%u,%u) has the triangle share %g, expected 0", pair.ID1, pair.ID2, pair.weightTriplet);
				continue;
			}
			ImagePair* pOther = without.FindPair(pair.ID1, pair.ID2);
			TEST_CHECK(!(pOther == NULL || !ISEQUAL(pair.GetCompositeWeight(), pOther->GetCompositeWeight())), "the pair (%u,%u) weighs %g with the repair and %g without the repaired pairs", pair.ID1, pair.ID2,
					pair.GetCompositeWeight(), pOther ? pOther->GetCompositeWeight() : -1.f);
		}
		TEST_CHECK(!(numReturned != planted.size()), "%u repaired pairs flagged, expected %zu", numReturned, planted.size());
	}
	{
		// a connected graph of 5 pairs, a loop and a chord among them, one baseline flipped: under
		// minScaleComponent pairs nothing is solved, so nothing is judged
		Scene scene; BuildArc(scene);
		const std::set<std::pair<IIndex,IIndex>> graph{{0, 1}, {1, 2}, {2, 3}, {0, 3}, {0, 2}};
		for (size_t idxPair = scene.pairs.size(); idxPair-- > 0; )
			if (!graph.count({scene.pairs[idxPair].ID1, scene.pairs[idxPair].ID2}))
				scene.pairs.RemoveAt(idxPair);
		ImagePair* pFlipped = scene.FindPair(0, 2);
		TEST_CHECK(!(pFlipped == NULL || scene.pairs.size() != graph.size()), "the ring has not the 5 pairs of the small graph");
		FlipBaseline(*pFlipped);
		const unsigned removed = RunCut(scene, maxDirectionResidual);
		TEST_CHECK(!(removed != 0 || scene.pairs.size() != graph.size()), "the 5-pair graph lost %u pairs", removed);
	}
	{
		// two components: images 0-8 and 9-15 as two groups of cameras with every pair inside a group and none
		// between them, so the rotation stage averages two components (the second pass of its loop) and the
		// pair-scale graph falls into two components, each of more than minScaleComponent pairs; one baseline
		// of the smaller group is flipped. Only that pair goes, and the cut reports both solves and every pair
		// judged: the flipped pair is judged by the smaller group's own solve.
		Scene scene; BuildArc(scene);
		const IIndex firstOfSecond = 9;
		for (size_t idxPair = scene.pairs.size(); idxPair-- > 0; )
			if ((scene.pairs[idxPair].ID1 < firstOfSecond) != (scene.pairs[idxPair].ID2 < firstOfSecond))
				scene.pairs.RemoveAt(idxPair);
		const size_t numPairs = scene.pairs.size();
		ImagePair* pFlipped = scene.FindPair(10, 13);
		TEST_CHECK(!(pFlipped == NULL || numPairs != 9 * 8 / 2 + 7 * 6 / 2), "the two groups hold %zu pairs, or have no pair (10,13)", numPairs);
		FlipBaseline(*pFlipped);
		LogCapture log;
		const unsigned removed = RunCut(scene, maxDirectionResidual);
		const auto kept = KeptPairs(scene);
		TEST_CHECK(!(removed != 1 || kept.size() != numPairs - 1 || kept.count({10, 13})), "the two groups: %u pairs removed of %zu, expected only the flipped (10,13)", removed, numPairs);
		const String rotationReport = String::FormatString("Contradiction cut: removed 0/%zu pairs", numPairs);
		const String translationReport = String::FormatString("Translation cut: removed 1/%zu pairs", numPairs);
		TEST_CHECK(!(!log.Contains(rotationReport) || !log.Contains("of 2 usable components") ||
			!log.Contains(translationReport) || !log.Contains("centres of 2 of 2 scale components")), "the two groups: the cut did not report the two rotation components and the two solves "
				"judging every pair of the %zu (\"%s\", \"of 2 usable components\", \"%s\", \"centres of 2 of 2 scale components\")",
				numPairs, rotationReport.c_str(), translationReport.c_str());
	}
	return scope.Pass("the rotation stage on the octagon, the bridge, the capped pair and the roof; the translation stage on the "
		"flipped and turned baselines, the bridge, the short rays, bar 0, the repair, the small graph and two components");
}

// The walk of SequenceChainTest: 40 cameras 1 unit apart along X, each turned a few degrees about the vertical, looking
// sideways at points 5-15 units away; every pair up to 6 apart with its exact pose at a unit baseline, so the scales
// are the solve's. Image `standStill` stands at the centre of the one before it, and the images after `stillFrom` at
// its centre: a pair of two images at one place stands still.
constexpr IIndex kWalkImages = 40;
constexpr unsigned kWalkLinks = 39 + 38 + 37; // the neighbour pairs, at most 3 apart
static Strip MakeWalk(IIndex standStill = NO_ID, IIndex stillFrom = NO_ID)
{
	std::vector<Pose3D> poses;
	for (IIndex i = 0; i < kWalkImages; ++i)
		poses.emplace_back(RMatrix(Point3(0, D2R(REAL(4) * SIN(REAL(0.9) * i)), 0)), Point3(REAL(i == standStill ? i - 1 : MINF(i, stillFrom)), 0, 0));
	std::mt19937 rng(7);
	std::uniform_real_distribution<REAL> xDist(-6, 45), yDist(-2, 2), zDist(5, 15);
	std::vector<Point3> points(2000);
	for (Point3& X : points) {
		X.x = xDist(rng);
		X.y = yDist(rng);
		X.z = zDist(rng);
	}
	Strip walk(poses, points);
	for (IIndex i = 0; i + 1 < kWalkImages; ++i)
		for (IIndex j = i + 1; j < kWalkImages && j - i <= 6; ++j)
			walk.AddPair(i, j, 1);
	ComputePairsWeights(walk.scene);
	return walk;
}

// The chain as the cut solves it on a walk, from the walk's own rotations: the neighbours `overlap` apart with a name
// break after the image `breakAfter`, the pairs `isRemoved` names cut, the image `unrotated` in no rotation component
static SequenceChain SolveWalk(const Scene& scene, const std::function<bool(const ImagePair&)>& isRemoved = {}, IIndex breakAfter = NO_ID,
	IIndex unrotated = NO_ID, unsigned overlap = 3)
{
	const size_t numImages = scene.images.size();
	std::vector<Point3> rotations(numImages);
	std::vector<uint32_t> componentOf(numImages, 0);
	FOREACH(i, scene.images)
		rotations[i] = scene.images[i].R.GetRotationAxisAngle();
	if (unrotated != NO_ID) {
		rotations[unrotated] = Point3::INF;
		componentOf[unrotated] = NO_ID;
	}
	std::vector<PairState> states(scene.pairs.size(), PairState::KEPT);
	FOREACH(idxPair, scene.pairs)
		if (isRemoved && isRemoved(scene.pairs[idxPair]))
			states[idxPair] = PairState::CUT;
	std::vector<bool> breaks(numImages > 0 ? numImages - 1 : 0, false);
	if (breakAfter != NO_ID)
		breaks[breakAfter] = true;
	return SolveSequenceChain(scene, ComputeWorldBaselines(scene, rotations, componentOf), SequenceNeighbours(breaks, overlap), states);
}

// a chain in a message
static String ChainText(const SequenceChain& chain)
{
	return String::FormatString("%u segment(s) placing %u images by %u links, %u of them standing still", chain.numSegments, chain.NumPlaced(), chain.numLinks, chain.numStill);
}
// does the chain place `numPlaced` images in one segment by `numLinks` links, `numStill` of them standing still
static bool IsOneSegment(const SequenceChain& chain, unsigned numPlaced, unsigned numLinks, unsigned numStill = 0)
{
	return chain.numSegments == 1 && chain.NumPlaced() == numPlaced && chain.numLinks == numLinks && chain.numStill == numStill;
}
// is the chain two segments, the images before `firstOfSecond` and the rest, by `numLinks` links, each in one segment
static bool IsSplitAt(const SequenceChain& chain, const Scene& scene, IIndex firstOfSecond, unsigned numLinks)
{
	bool split = chain.numSegments == 2 && chain.NumPlaced() == scene.images.size() && chain.numLinks == numLinks && chain.localStep.size() == 2 &&
		chain.pathLength[firstOfSecond] == 0;
	FOREACH(i, chain.segmentOf)
		split = split && chain.segmentOf[i] == (i < firstOfSecond ? 0u : 1u);
	unsigned numPositive = 0;
	FOREACH(idxPair, scene.pairs) {
		if (chain.linkScale[idxPair] <= 0)
			continue;
		++numPositive;
		split = split && chain.segmentOf[scene.pairs[idxPair].ID1] == chain.segmentOf[scene.pairs[idxPair].ID2];
	}
	return split && numPositive == numLinks;
}
// does the chain place nothing, its vectors sized by image and pair all the same
static bool PlacesNothing(const SequenceChain& chain, const Scene& scene)
{
	return chain.numSegments == 0 && chain.NumPlaced() == 0 && chain.numLinks == 0 && chain.numStill == 0 && chain.localStep.empty() &&
		chain.segmentOf.size() == scene.images.size() && chain.linkScale.size() == scene.pairs.size() &&
		std::all_of(chain.linkScale.begin(), chain.linkScale.end(), [](REAL scale) { return scale == 0; });
}
// is image i placed exactly where image j is: in its segment, at its centre and path length
static bool IsPlacedAt(const SequenceChain& chain, IIndex i, IIndex j)
{
	return chain.segmentOf[i] != NO_ID && chain.segmentOf[i] == chain.segmentOf[j] && chain.centres[i] == chain.centres[j] &&
		chain.pathLength[i] == chain.pathLength[j];
}
// The scale of the least-squares fit s * c + t of the centres of a chain that places every image in one segment to
// the cameras' own (the segment's frame has the rotations' orientation), and the largest distance it leaves
static REAL FitToCameras(const SequenceChain& chain, const Scene& scene, REAL& scale)
{
	const size_t n = scene.images.size();
	Point3 meanChain(0, 0, 0), meanTrue(0, 0, 0);
	for (size_t i = 0; i < n; ++i) {
		meanChain += chain.centres[i] * (REAL(1) / n);
		meanTrue += Point3(scene.images[i].C) * (REAL(1) / n);
	}
	REAL num = 0, den = 0;
	for (size_t i = 0; i < n; ++i) {
		num += (chain.centres[i] - meanChain).dot(Point3(scene.images[i].C) - meanTrue);
		den += normSq(chain.centres[i] - meanChain);
	}
	scale = num / den;
	REAL maxDistance = 0;
	for (size_t i = 0; i < n; ++i)
		maxDistance = MAXF(maxDistance, norm(scale * (chain.centres[i] - meanChain) - (Point3(scene.images[i].C) - meanTrue)));
	return maxDistance;
}
constexpr REAL kChainTolerance = REAL(0.02); // of a length, in the tests of the chain

// The walk is one segment of its 40 images, within 2% of the cameras after the fit, its local step and every step of
// its path the fitted step; its links are exactly the 114 neighbour pairs, each as long as its true baseline
static bool ChainPlacesTheWalk(const TestScope& scope)
{
	const Strip walk = MakeWalk();
	const Scene& scene = walk.scene;
	const SequenceChain chain = SolveWalk(scene);
	TEST_CHECK(IsOneSegment(chain, kWalkImages, kWalkLinks), "the walk: %s, expected one segment of %u images by %u links", ChainText(chain).c_str(), kWalkImages, kWalkLinks);
	TEST_CHECK(chain.segmentOf.size() == kWalkImages && chain.centres.size() == kWalkImages && chain.pathLength.size() == kWalkImages &&
		chain.localStep.size() == 1 && chain.linkScale.size() == scene.pairs.size(), "the walk: the vectors are not sized by image, segment and pair");
	REAL scale;
	const REAL drift = FitToCameras(chain, scene, scale), step = REAL(1) / scale;
	TEST_CHECK(scale > 0 && drift <= kChainTolerance * (kWalkImages - 1), "the walk: a centre is %g from its camera after the fit, over %g of the walk's length %u",
		drift, kChainTolerance, kWalkImages - 1);
	TEST_CHECK(ABS(chain.localStep[0] - step) <= kChainTolerance * step && chain.pathLength[0] == 0, "the walk: the local step is %g, the fitted step %g; the path starts at %g",
		chain.localStep[0], step, chain.pathLength[0]);
	for (IIndex i = 1; i < kWalkImages; ++i)
		TEST_CHECK(ABS(chain.pathLength[i] - chain.pathLength[i - 1] - step) <= kChainTolerance * step, "the walk: the step of the path to image %u is %g, the fitted step %g",
			i, chain.pathLength[i] - chain.pathLength[i - 1], step);
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		// a link's length in the segment's frame is its scale times its baseline's: its true one, in fitted steps
		const REAL span = REAL(pair.ID2 - pair.ID1), length = chain.linkScale[idxPair] * norm(pair.relativePose->C) * scale;
		TEST_CHECK(span <= 3 ? ABS(length - span) <= kChainTolerance * span : length == 0, "the walk: the pair (%u,%u) is %g steps long in the chain, expected %s",
			pair.ID1, pair.ID2, length, span <= 3 ? "its true baseline, a link" : "0, no link");
	}
	return true;
}

// What splits the chain or takes a link out of it: a name break, or the pairs removed, across 19|20 give the segments
// 0-19 and 20-39 with no link between them; a neighbour pair without a baseline length, or without weight, is no
// link; an unrotated image is not placed; without neighbours (overlap 0) or without images nothing is placed
static bool ChainSplitsAndDropsLinks(const TestScope& scope)
{
	Strip walk = MakeWalk();
	Scene& scene = walk.scene;
	const auto acrossMiddle = [](const ImagePair& pair) { return pair.ID1 < 20 && pair.ID2 >= 20; };
	for (const bool byName : {true, false}) {
		const SequenceChain chain = byName ? SolveWalk(scene, {}, 19) : SolveWalk(scene, acrossMiddle);
		TEST_CHECK(IsSplitAt(chain, scene, 20, 2 * (19 + 18 + 17)), "%s across 19|20: %s, expected the segments 0-19 and 20-39 by 108 links, none across",
			byName ? "a name break" : "the pairs removed", ChainText(chain).c_str());
	}
	ImagePair& pair = *scene.FindPair(10, 11);
	const ImagePair intact(pair);
	for (const bool flat : {true, false}) {
		if (flat)
			pair.relativePose->C = CMatrix::ZERO;
		else
			pair.InvalidateWeight(); // the centres solve would skip it
		const SequenceChain chain = SolveWalk(scene);
		pair = intact;
		TEST_CHECK(IsOneSegment(chain, kWalkImages, kWalkLinks - 1) && chain.linkScale[&pair - &scene.pairs[0]] == 0, "the pair (10,11) without %s: %s, its scale %g; expected one link less, of all images",
			flat ? "a baseline length" : "weight", ChainText(chain).c_str(), chain.linkScale[&pair - &scene.pairs[0]]);
	}
	const IIndex last = kWalkImages - 1;
	const SequenceChain partial = SolveWalk(scene, {}, NO_ID, last);
	TEST_CHECK(IsOneSegment(partial, last, kWalkLinks - 3) && partial.segmentOf[last] == NO_ID, "the last image unrotated: %s, expected it not placed and its 3 links lost",
		ChainText(partial).c_str());
	const Scene empty;
	for (const Scene* pScene : std::initializer_list<const Scene*>{&scene, &empty}) {
		const SequenceChain chain = SolveWalk(*pScene, {}, NO_ID, NO_ID, 0);
		TEST_CHECK(PlacesNothing(chain, *pScene), "%s: %s, expected nothing placed", pScene == &empty ? "the empty scene" : "overlap 0", ChainText(chain).c_str());
	}
	return true;
}

// Frames standing still. A still pair is no link, and an image no link places stands where the nearest placed image
// of its still pairs stands (the lower on a tie), exactly, changing nothing else, whatever the order of the pairs
static bool ChainPlacesStillImages(const TestScope& scope)
{
	const IIndex last = kWalkImages - 1;
	{
		// one frame standing still in the walk (20 at 19's centre): its pairs 2 and 3 apart are links and place it
		const Strip walk = MakeWalk(20);
		const SequenceChain chain = SolveWalk(walk.scene);
		REAL scale;
		const REAL drift = FitToCameras(chain, walk.scene, scale);
		TEST_CHECK(IsOneSegment(chain, kWalkImages, kWalkLinks - 1) && chain.linkScale[walk.scene.FindPair(19, 20) - &walk.scene.pairs[0]] == 0 && scale > 0 &&
			drift <= kChainTolerance * last, "a frame standing still in the walk: %s, a centre %g from its camera after the fit; expected every image placed by links, the pair (19,20) none",
			ChainText(chain).c_str(), drift);
	}
	{
		// every pair under the near-duplicate ray angle: still pairs all, and no segment for an image to stand in
		Strip walk = MakeWalk();
		for (ImagePair& pair : walk.scene.pairs)
			pair.meanRayAngle = D2R(kNearDuplicateRayAngle * 0.75f);
		const SequenceChain chain = SolveWalk(walk.scene);
		TEST_CHECK(PlacesNothing(chain, walk.scene), "every pair under the near-duplicate angle: %s, expected nothing placed", ChainText(chain).c_str());
	}
	{
		// the walk ends with four cameras (36-39) at one place: 36, 37 and 38 are placed by their links to the images
		// before it, and 39, whose three neighbour pairs all stand still, where 38 stands
		Strip walk = MakeWalk(NO_ID, 36);
		Scene& scene = walk.scene;
		const auto ofLast = [&](const ImagePair& pair) { return pair.ID2 == last && pair.ID1 >= 36; };
		const SequenceChain chain = SolveWalk(scene);
		TEST_CHECK(IsOneSegment(chain, kWalkImages, kWalkLinks - 6, 1) && IsPlacedAt(chain, last, last - 1), "the walk ending still: %s, image 39 %g from image 38; expected it alone "
			"placed standing still, exactly at image 38, by 108 links", ChainText(chain).c_str(), norm(chain.centres[last] - chain.centres[last - 1]));
		FOREACH(idxPair, scene.pairs)
			TEST_CHECK(scene.pairs[idxPair].ID1 < 36 || chain.linkScale[idxPair] == 0, "the walk ending still: the still pair (%u,%u) is a link of scale %g",
				scene.pairs[idxPair].ID1, scene.pairs[idxPair].ID2, chain.linkScale[idxPair]);
		// placing it changes nothing else: the scene without the pairs of image 39 has the same local step, centres and paths
		Scene before(scene);
		RFOREACH(idxPair, before.pairs)
			if (before.pairs[idxPair].ID2 == last)
				before.pairs.RemoveAtMove(idxPair);
		const SequenceChain chainBefore = SolveWalk(before);
		bool same = IsOneSegment(chainBefore, last, kWalkLinks - 6) && chainBefore.segmentOf[last] == NO_ID && chain.localStep[0] == chainBefore.localStep[0];
		for (IIndex i = 0; i < last; ++i)
			same = same && chain.centres[i] == chainBefore.centres[i] && chain.pathLength[i] == chainBefore.pathLength[i];
		TEST_CHECK(same, "the walk ending still, without the pairs of image 39: %s, a local step of %.17g against %.17g; expected the same chain less image 39",
			ChainText(chainBefore).c_str(), chainBefore.localStep[0], chain.localStep[0]);
		// the pairs of the scene in the opposite order place it alike
		Scene reversed(scene);
		std::reverse(reversed.pairs.begin(), reversed.pairs.end());
		const SequenceChain chainReversed = SolveWalk(reversed);
		TEST_CHECK(IsOneSegment(chainReversed, kWalkImages, kWalkLinks - 6, 1) && IsPlacedAt(chainReversed, last, last - 1), "the walk ending still, its pairs reversed: %s, image 39 %g from image 38",
			ChainText(chainReversed).c_str(), norm(chainReversed.centres[last] - chainReversed.centres[last - 1]));
		// without its three still pairs, or unrotated, image 39 is not placed
		for (const bool removed : {true, false}) {
			const SequenceChain without = removed ? SolveWalk(scene, ofLast) : SolveWalk(scene, {}, NO_ID, last);
			TEST_CHECK(IsOneSegment(without, last, kWalkLinks - 6) && without.segmentOf[last] == NO_ID, "the walk ending still, image 39 %s: %s, expected it not placed",
				removed ? "without its still pairs" : "unrotated", ChainText(without).c_str());
		}
		// still by its baseline, not by its angle: at 10 degrees of ray angle and a baseline of no length, the pairs of
		// image 39 are still pairs as well
		for (ImagePair& pair : scene.pairs)
			if (ofLast(pair)) {
				pair.meanRayAngle = D2R(10.f);
				pair.relativePose->C = CMatrix::ZERO;
			}
		const SequenceChain flat = SolveWalk(scene);
		TEST_CHECK(IsOneSegment(flat, kWalkImages, kWalkLinks - 6, 1) && IsPlacedAt(flat, last, last - 1), "the pairs of image 39 with a baseline of no length: %s, expected it placed at image 38",
			ChainText(flat).c_str());
	}
	{
		// seven cameras (33-39) at one place: 33, 34 and 35 are placed by their links, 36, 37 and 38 where 35 stands,
		// the nearest placed image of each, and 39, whose neighbours 36-38 no link places, by a second pass
		const Strip walk = MakeWalk(NO_ID, 33);
		const SequenceChain chain = SolveWalk(walk.scene);
		bool placed = chain.numSegments == 1 && chain.NumPlaced() == kWalkImages && chain.numStill == 4;
		for (IIndex i = 36; i < kWalkImages; ++i)
			placed = placed && IsPlacedAt(chain, i, 35);
		TEST_CHECK(placed, "seven cameras at one place: %s, expected the images 36-39 standing still at image 35", ChainText(chain).c_str());
	}
	{
		// an image in the walk whose neighbour pairs are all still pairs (their ray angle under the near-duplicate
		// angle, the geometry untouched): 20 takes the nearest placed image, 19 of 19 and 21, the lower on a tie; 21
		// with its pair with 19 removed; 18, of 18 and 22, with that of 21 removed too
		Strip walk = MakeWalk();
		Scene& scene = walk.scene;
		for (ImagePair& pair : scene.pairs)
			if ((pair.ID1 == 20 || pair.ID2 == 20) && pair.ID2 - pair.ID1 <= 3)
				pair.meanRayAngle = D2R(kNearDuplicateRayAngle * 0.75f);
		struct Case { std::vector<IIndex> removedWith; IIndex expected; };
		for (const Case& c : {Case{{}, 19}, Case{{19}, 21}, Case{{19, 21}, 18}}) {
			const SequenceChain chain = SolveWalk(scene, [&c](const ImagePair& pair) {
				const IIndex other = pair.ID1 == 20 ? pair.ID2 : pair.ID2 == 20 ? pair.ID1 : NO_ID;
				return std::find(c.removedWith.begin(), c.removedWith.end(), other) != c.removedWith.end();
			});
			unsigned numAt = 0; // the images at whose centre and path image 20 is
			for (IIndex i = 0; i < kWalkImages; ++i)
				numAt += i != 20 && IsPlacedAt(chain, 20, i);
			TEST_CHECK(IsOneSegment(chain, kWalkImages, kWalkLinks - 6, 1) && numAt == 1 && IsPlacedAt(chain, 20, c.expected),
				"image 20 with still pairs to both sides, %zu of them removed: %s, placed where %u image(s) are, expected at image %u alone", c.removedWith.size(), ChainText(chain).c_str(), numAt, c.expected);
		}
	}
	return true;
}

// Two scale components sharing an image: the pairs across image 15 are removed and no pair-scale equation runs through
// it (the pairs ending at it keep their matches in the left half of its frame, the pairs starting at it in the right
// half). The larger component, 15-39, owns the image; the candidate links of the smaller that end at it are no links
static bool ChainGivesSharedImageToLargerComponent(const TestScope& scope)
{
	Strip walk = MakeWalk();
	Scene& scene = walk.scene;
	const IIndex shared = 15;
	for (ImagePair& pair : scene.pairs) {
		if (pair.ID1 != shared && pair.ID2 != shared)
			continue;
		const bool left = pair.ID2 == shared;
		pair.matches.erase(std::remove_if(pair.matches.begin(), pair.matches.end(), [&](const DMatch& match) {
			return (scene.images[shared].keypoints[left ? match.trainIdx : match.queryIdx].pt.x < 320.f) != left;
		}), pair.matches.end());
	}
	const SequenceChain chain = SolveWalk(scene, [&](const ImagePair& pair) { return pair.ID1 < shared && pair.ID2 > shared; });
	TEST_CHECK(IsSplitAt(chain, scene, shared, (14 + 13 + 12) + (24 + 23 + 22)), "two components sharing image 15: %s, expected the segments 0-14 and 15-39 by 108 links, none between them",
		ChainText(chain).c_str());
	return true;
}

// SolveSequenceChain on a walk of 40 cameras with real features (MakeWalk), the neighbours 3 apart:
//  - the walk is one segment within 2% of the cameras, its links the neighbour pairs at their true lengths;
//  - a name break or removed pairs split it, a pair without baseline length or weight is no link, an unrotated image
//    is not placed;
//  - an image no link places stands where the nearest placed image of its still pairs stands;
//  - an image two scale components place belongs to the larger.
bool SequenceChainTest()
{
	TestScope scope("SequenceChainTest");
	if (!ChainPlacesTheWalk(scope) || !ChainSplitsAndDropsLinks(scope) || !ChainPlacesStillImages(scope) || !ChainGivesSharedImageToLargerComponent(scope))
		return false;
	return scope.Pass("the walk of %u images in one segment by %u links; the breaks and the pairs that are no links; the still images; the shared image", kWalkImages, kWalkLinks);
}

// The periodic corridor of SequenceVetoTest: a pattern of 400 points repeated every 10 units along X and 60 cameras
// looking sideways at it, 30 walking out along +X one unit apart (out_0000..) and 30 walking back 2 units further
// from the points (back_0030..), so (i, 59 - i) are true revisits. A camera is turned 4 sin(2 pi x / 10) degrees about
// the vertical, so frames a period apart see the same picture; the images after `stillFrom` stand at its place.
// Its pairs, each with a baseline at a length of its own (so the lengths are the scales'):
//  - the true ones: every pair up to 6 apart, and every revisit;
//  - the look-alikes (i, i + 11) and (i, i + 21) of one way: the second image stands one or two periods from the image
//    after i, so the pair has the pose of (i, i + 1) and matches each point in i with its copy in the other: the
//    right rotation and a baseline of one step, between frames 11 and 21 apart;
//  - the near-duplicate look-alikes, when planted (PlantNearDuplicates).
constexpr IIndex kCorridorOut = 30, kCorridorImages = 2 * kCorridorOut;
constexpr unsigned kCorridorLinks = 59 + 58 + 57; // the neighbour pairs, at most 3 apart
constexpr int kPeriod = 10, kPatternPoints = 400;
struct Corridor : Strip {
	PairSet pairs, lookAlikes, nearDuplicates;
	IIndex stillFrom{NO_ID};

	using Strip::Strip;
	static Point3 Place(IIndex i, IIndex stillFrom)
	{
		const IIndex k = MINF(i, stillFrom);
		return k < kCorridorOut ? Point3(REAL(k), 0, 0) : Point3(REAL(kCorridorImages - 1 - k), 0, -2);
	}
	Point3 Place(IIndex i) const { return Place(i, stillFrom); }
	static REAL Length(IIndex a, IIndex b) { return REAL(0.5) + REAL(0.25) * ((3 * a + 5 * b) % 7); }
	PairSet TruePairs() const { return Without(Without(pairs, lookAlikes), nearDuplicates); }
};
static Corridor MakeCorridor(IIndex numCameras = kCorridorImages, bool lookAlikesBack = false, IIndex stillFrom = NO_ID)
{
	std::vector<Pose3D> poses;
	for (IIndex i = 0; i < numCameras; ++i) {
		const Point3 C = Corridor::Place(i, stillFrom);
		poses.emplace_back(RMatrix(Point3(0, D2R(REAL(4) * SIN(REAL(2) * M_PI * C.x / kPeriod)), 0)), C);
	}
	// the copies -2..4 of the pattern: point k of copy c is the point (c + 2) * kPatternPoints + k
	std::mt19937 rng(11);
	std::uniform_real_distribution<REAL> xDist(0, kPeriod), yDist(-2, 2), zDist(5, 15);
	std::vector<Point3> pattern(kPatternPoints), points;
	for (Point3& X : pattern) {
		X.x = xDist(rng);
		X.y = yDist(rng);
		X.z = zDist(rng);
	}
	for (int copy = -2; copy <= 4; ++copy)
		for (const Point3& X : pattern)
			points.emplace_back(X.x + REAL(kPeriod * copy), X.y, X.z);
	Corridor corridor(poses, points);
	corridor.stillFrom = stillFrom;
	for (IIndex i = 0; i < numCameras; ++i)
		corridor.scene.images[i].fileName = String::FormatString("%s_%04u.jpg", i < kCorridorOut ? "out" : "back", i);
	for (IIndex i = 0; i + 1 < numCameras; ++i)
		for (IIndex j = i + 1; j < numCameras; ++j) {
			const REAL shift = corridor.Place(j).x - corridor.Place(i + 1).x; // from the image after i to j
			const bool sameWay = j < kCorridorOut || (lookAlikesBack && i >= kCorridorOut);
			if (j - i <= 6 || i + j == kCorridorImages - 1)
				corridor.AddPair(i, j, Corridor::Length(i, j));
			else if (sameWay && (ABS(shift) == REAL(kPeriod) || ABS(shift) == REAL(2 * kPeriod))) {
				corridor.AddPair(i, j, Corridor::Length(i, j), i + 1, ROUND2INT(shift / kPeriod) * kPatternPoints);
				corridor.lookAlikes.emplace(i, j);
			} else
				continue;
			corridor.pairs.emplace(i, j);
		}
	ComputePairsWeights(corridor.scene);
	return corridor;
}
static SequenceNeighbours CorridorNeighbours()
{
	return SequenceNeighbours(std::vector<bool>(kCorridorImages - 1, false), 3);
}

// Adds the pair (a,b) as a camera of b's rotation standing `offset` along X from a matches a (MatchAsSeenFrom), with
// that camera's pose and the ray angle its matches measure. With b a whole number of periods from a, where the two
// have one rotation and see the same picture, it is the pair of b standing `offset` past those periods: a look-alike
// with a baseline of `offset`. At an offset of 0 the two see the same picture exactly: no parallax and no ray angle.
// The weights are left to be computed.
static ImagePair& AddBeside(Corridor& corridor, IIndex a, IIndex b, REAL offset)
{
	Scene& scene = corridor.scene;
	const Image beside = StandingAt(scene.images[b], Point3(scene.images[a].C) + Point3(offset, 0, 0));
	ImagePair& pair = scene.pairs.emplace_back(a, b);
	MatchAsSeenFrom(scene, pair, beside, corridor.points, corridor.featureOf[a]);
	pair.meanRayAngle = SetExactPose(scene, pair, beside, Corridor::Length(a, b)) ? pair.ComputeMeanRayAngle(scene.images[a], scene.images[b]) : 0.f;
	corridor.pairs.emplace(a, b);
	return pair;
}
// The near-duplicate look-alikes: every pair (i, i + 10) of the way out, its second image standing `offset` past the
// period, before it for an odd i
static void PlantNearDuplicates(Corridor& corridor, REAL offset)
{
	for (IIndex i = 0; i + kPeriod < kCorridorOut; ++i) {
		AddBeside(corridor, i, i + kPeriod, i % 2 == 0 ? offset : -offset);
		corridor.nearDuplicates.emplace(i, i + kPeriod);
	}
	ComputePairsWeights(corridor.scene);
}
constexpr REAL kNearOffset = REAL(0.25); // steps, the baseline of the near-duplicate look-alikes the tests plant

// Runs the contradiction cut on the scene with the neighbours (nullptr: without) and holds what the chain did, and the
// pairs the scene keeps, to the expected
static bool ExpectCut(const TestScope& scope, const char* what, Scene& scene, const SequenceNeighbours* neighbours, const SequenceFilterStats& expected,
	const PairSet& expectedKept, bool repair = false, float maxDirectionResidual = kDefaultDirectionBar, const std::vector<float>* scores = nullptr)
{
	const size_t numPairs = scene.pairs.size();
	SequenceFilterStats stats{7, 7, 7, 7, 7}; // not what any case expects: the cut fills every one
	const unsigned removed = RunCut(scene, maxDirectionResidual, repair, neighbours, &stats, scores);
	const PairSet kept = KeptPairs(scene);
	const auto text = [](const SequenceFilterStats& s) {
		return String::FormatString("%u segment(s) by %u links, %u of %u judged pairs vetoed, %u links protected", s.numSegments, s.numLinks, s.numVetoed, s.numJudged, s.numProtected);
	};
	TEST_CHECK(stats.numSegments == expected.numSegments && stats.numLinks == expected.numLinks && stats.numJudged == expected.numJudged &&
		stats.numVetoed == expected.numVetoed && stats.numProtected == expected.numProtected && kept == expectedKept && removed == numPairs - kept.size(),
		"%s: %s; %u of %zu pairs removed, %zu of them pairs to keep, and %zu kept that were to go; expected %s", what, text(stats).c_str(), removed, numPairs,
		Without(expectedKept, kept).size(), Without(kept, expectedKept).size(), text(expected).c_str());
	return true;
}

// Holds the lines the veto logs per vetoed pair (above verbosity 3; the images named <way>_<index>) to the corridor:
// one per look-alike and near-duplicate look-alike, each with the pair's true length, its distance from the chain and
// the chain's path, in steps: the line's own chain distance over the true one. A look-alike has a baseline of one
// step along the walk; a near-duplicate one of kNearOffset, back for an odd first image.
static bool ExpectVetoedLines(const TestScope& scope, const char* what, const String& log, const Corridor& corridor)
{
	const auto indexOf = [](const char* stem) { const char* number = strrchr(stem, '_'); return number == NULL ? NO_ID : (IIndex)atoi(number + 1); };
	unsigned numLines = 0;
	for (size_t pos = log.find("Vetoed pair: "); pos != String::npos; pos = log.find("Vetoed pair: ", pos + 1), ++numLines) {
		char stem1[64], stem2[64];
		double length, chain, path, discrepancy;
		TEST_CHECK(sscanf(log.c_str() + pos, "Vetoed pair: %63s %63s length %lf chain %lf path %lf discrepancy %lf", stem1, stem2, &length, &chain, &path, &discrepancy) == 6,
			"%s: the line %u of a vetoed pair does not read", what, numLines);
		const PairIds ids(indexOf(stem1), indexOf(stem2));
		const bool isNear = corridor.nearDuplicates.count(ids) != 0;
		TEST_CHECK(isNear || corridor.lookAlikes.count(ids), "%s: the line of a vetoed pair names (%u,%u), no look-alike", what, ids.first, ids.second);
		const REAL distance = norm(corridor.Place(ids.second) - corridor.Place(ids.first)), step = chain / distance;
		const REAL baseline = isNear ? (ids.first % 2 == 0 ? kNearOffset : -kNearOffset) : REAL(1);
		TEST_CHECK(ABS(length / step - ABS(baseline)) <= kChainTolerance * ABS(baseline) && ABS(discrepancy / step - (distance - baseline)) <= kChainTolerance * distance &&
			ABS(path / step - distance) <= kChainTolerance * distance, "%s: the vetoed pair (%u,%u) is %g steps long, %g off the chain on a path of %g; expected %g, %g and %g",
			what, ids.first, ids.second, length / step, discrepancy / step, path / step, ABS(baseline), distance - baseline, distance);
	}
	TEST_CHECK(numLines == corridor.lookAlikes.size() + corridor.nearDuplicates.size(), "%s: %u lines of vetoed pairs, expected %zu", what, numLines,
		corridor.lookAlikes.size() + corridor.nearDuplicates.size());
	return true;
}

// The rule by values (ChainVetoes), at a local step of 1: the tolerance is the larger of 8 local steps and 0.2 of the
// path, and only a pair shorter than a third of the chain distance is vetoed
static bool VetoRuleByValues(const TestScope& scope)
{
	struct Case { REAL discrepancy, pairLength, chainDistance, path; bool vetoed; const char* what; };
	const Case cases[] = {
		{10, 1, 11, 100, false, "10 off on a path of 100 (a tolerance of 20)"},
		{10, 1, 11, 20, true, "10 off on a path of 20 (a tolerance of 8)"},
		{7.9, 1, 9, 9, false, "7.9 off (a tolerance of 8)"},
		{30, 12, 30, 30, false, "a length of 12 on a chain distance of 30 (not short), 30 off"},
		{30, 9, 30, 30, true, "a length of 9 on a chain distance of 30, 30 off"},
		{1, 0.5, 0, 0, false, "two images at one centre (a chain distance of 0)"},
	};
	for (const Case& c : cases)
		TEST_CHECK(ChainVetoes(c.discrepancy, c.pairLength, c.chainDistance, 1, c.path) == c.vetoed, "the rule: %s is %svetoed", c.what, c.vetoed ? "not " : "");
	return true;
}

// ComputePairScales given anchors (the links, as the veto calls it) forms exactly the equations of the plain call
// that join an anchor and a pair that is none, with their values
static bool AnchoredPairScalesAreThePlainOnes(const TestScope& scope)
{
	const Corridor corridor = MakeCorridor();
	const Scene& scene = corridor.scene;
	IIndexArr all;
	std::vector<bool> anchors(scene.pairs.size());
	FOREACH(idxPair, scene.pairs) {
		all.push_back((IIndex)idxPair);
		anchors[idxPair] = scene.pairs[idxPair].ID2 - scene.pairs[idxPair].ID1 <= 3;
	}
	const std::vector<ScalePair> plain = ComputePairScales(scene, all), anchored = ComputePairScales(scene, all, &anchors);
	std::vector<ScalePair> expected;
	for (const ScalePair& equation : plain)
		if (anchors[equation.idxA] != anchors[equation.idxB])
			expected.push_back(equation);
	bool same = anchored.size() == expected.size();
	for (size_t i = 0; same && i < anchored.size(); ++i)
		same = anchored[i].idxA == expected[i].idxA && anchored[i].idxB == expected[i].idxB &&
			ABS(anchored[i].scaleRatio / expected[i].scaleRatio - 1) <= 1e-9 && anchored[i].weight == expected[i].weight;
	TEST_CHECK(same && !expected.empty() && expected.size() < plain.size(), "ComputePairScales with anchors: %zu equations, expected the %zu of the plain call's %zu that join an "
		"anchor and a pair that is none, each with its ratio and weight", anchored.size(), expected.size(), plain.size());
	return true;
}

// On the corridor the veto removes exactly the look-alikes, each a step long 10 or 20 steps off the chain, and judges
// every other pair that is no link, a neighbour pair of no weight among them. Without the neighbours the cut is the
// six-argument call, its stats zero, and keeps the look-alikes: their direction agrees with the centres
static bool VetoRemovesLookAlikes(const TestScope& scope)
{
	const SequenceNeighbours neighbours = CorridorNeighbours();
	Corridor corridor = MakeCorridor(), weightless(corridor), without(corridor), asBefore(corridor);
	const unsigned numJudged = (unsigned)corridor.pairs.size() - kCorridorLinks;
	TEST_CHECK(corridor.lookAlikes.size() == 19 + 9 && corridor.pairs.size() == kCorridorLinks + (56 + 55 + 54) + 27 + 28, "the corridor holds %zu pairs, %zu of them look-alikes",
		corridor.pairs.size(), corridor.lookAlikes.size());
	{
		LogCapture log(4); // the lines of the vetoed pairs
		if (!ExpectCut(scope, "the corridor", corridor.scene, &neighbours, {1, kCorridorLinks, numJudged, 28, 0}, corridor.TruePairs()) ||
			!ExpectVetoedLines(scope, "the corridor", log.text, corridor))
			return false;
		const String report = String::FormatString("Sequence chain veto: removed 28 of %u judged pairs", numJudged);
		TEST_CHECK(log.Contains(report) && !log.Contains("Translation cut: kept"), "the corridor: the cut did not report \"%s\", or reports a link it kept", report.c_str());
	}
	// a neighbour pair of no weight is no candidate link: judged like any other pair, and kept, being true
	weightless.scene.FindPair(10, 12)->InvalidateWeight();
	if (!ExpectCut(scope, "a neighbour pair of weight 0", weightless.scene, &neighbours, {1, kCorridorLinks - 1, numJudged + 1, 28, 0}, corridor.TruePairs()))
		return false;
	// without the neighbours: the result of the six-argument call
	const std::vector<float> scores(asBefore.scene.pairs.size(), TripletScores::unscored);
	FilterPairsByContradiction(asBefore.scene, scores, kCutMinPairWeight, kDefaultDirectionBar, PairsWeightingConfig(), false);
	const PairSet keptBefore = KeptPairs(asBefore.scene);
	TEST_CHECK(std::includes(keptBefore.begin(), keptBefore.end(), corridor.lookAlikes.begin(), corridor.lookAlikes.end()),
		"premise not met: without the neighbours the cut removes a look-alike (%zu of %zu pairs kept)", keptBefore.size(), corridor.pairs.size());
	return ExpectCut(scope, "without the neighbours", without.scene, nullptr, {}, keptBefore);
}

// The veto keeps to a segment. With a name break at the turn (29|30) the look-alikes of both ways are vetoed and the
// pairs across the turn, the revisits among them, are not judged. And a segment does not span two components of the
// rotation averaging, whose rotations share no frame: with the pairs joining the images 0-14 to the rest scoring 0,
// the veto judges the pairs inside a side alone
static bool VetoKeepsToASegment(const TestScope& scope)
{
	const auto numInside = [](const Corridor& corridor, IIndex firstOfSecond) { // the pairs inside a side that are no neighbours
		return (unsigned)std::count_if(corridor.pairs.begin(), corridor.pairs.end(), [&](const PairIds& ids) {
			return (ids.first < firstOfSecond) == (ids.second < firstOfSecond) && ids.second - ids.first > 3;
		});
	};
	{
		Corridor corridor = MakeCorridor(kCorridorImages, true);
		std::vector<bool> breaks(kCorridorImages - 1, false);
		breaks[kCorridorOut - 1] = true;
		const SequenceNeighbours neighbours(breaks, 3);
		TEST_CHECK(corridor.lookAlikes.size() == 2 * 28 && numInside(corridor, kCorridorOut) == 2 * (26 + 25 + 24 + 28), "the corridor with the look-alikes of the way back holds %zu "
			"look-alikes and %u pairs inside a way that are no neighbours", corridor.lookAlikes.size(), numInside(corridor, kCorridorOut));
		if (!ExpectCut(scope, "a break at the turn", corridor.scene, &neighbours, {2, 2 * (29 + 28 + 27), numInside(corridor, kCorridorOut), 2 * 28, 0}, corridor.TruePairs()))
			return false;
	}
	Corridor corridor = MakeCorridor();
	const IIndex firstOfSecond = 15;
	std::vector<float> scores(corridor.scene.pairs.size(), TripletScores::unscored);
	PairSet inside; // the look-alikes inside a side
	FOREACH(idxPair, corridor.scene.pairs) {
		const PairIds ids(corridor.scene.pairs[idxPair].ID1, corridor.scene.pairs[idxPair].ID2);
		if ((ids.first < firstOfSecond) != (ids.second < firstOfSecond))
			scores[idxPair] = 0.f;
		else if (corridor.lookAlikes.count(ids))
			inside.insert(ids);
	}
	const SequenceNeighbours neighbours = CorridorNeighbours();
	TEST_CHECK(inside.size() == 8, "the corridor holds %zu look-alikes inside the images 0-14 or 15-59, expected 8", inside.size());
	return ExpectCut(scope, "two rotation components", corridor.scene, &neighbours, {2, (14 + 13 + 12) + (44 + 43 + 42), numInside(corridor, firstOfSecond), 8, 0},
		Without(corridor.pairs, inside), false, kDefaultDirectionBar, &scores);
}

// The veto abstains, and the cut gives the result it gives without the neighbours, where no chain is solved
static bool VetoAbstainsWithoutAChain(const TestScope& scope)
{
	struct Case { IIndex numCameras; float maxDirectionResidual; unsigned overlap; const char* what; };
	const Case cases[] = {
		{5, kDefaultDirectionBar, 3, "5 cameras (9 candidate links, under the pairs a scale component needs)"},
		{kCorridorImages, 0.f, 3, "a bar of 0 (no translation stage)"},
		{kCorridorImages, kDefaultDirectionBar, 0, "overlap 0 (no neighbours)"},
		{1, kDefaultDirectionBar, 3, "one image"},
		{0, kDefaultDirectionBar, 3, "no image"},
	};
	for (const Case& c : cases) {
		Scene scene;
		if (c.numCameras > 0)
			scene = MakeCorridor(c.numCameras).scene;
		Scene plain(scene);
		RunCut(plain, c.maxDirectionResidual);
		MatchConfig cfg;
		cfg.imageSequence = true;
		cfg.matchSequenceOverlap = c.overlap;
		const SequenceNeighbours neighbours = SequenceNeighbours::FromConfig(scene, cfg).value();
		if (!ExpectCut(scope, c.what, scene, &neighbours, {}, KeptPairs(plain), false, c.maxDirectionResidual))
			return false;
	}
	return true;
}

// The repair is not offered a vetoed pair, and a vetoed pair makes no spot. On the corridor with its near-duplicate
// look-alikes, the repair on: the true pair (12,17), its baseline flipped, is removed by the translation stage and
// comes back, alone. The pair (5,25), a look-alike two periods on with its rotation 30 degrees off, is removed by the
// rotation stage and offered to the repair: its matches agree on the direction of the centres, no image holds a kept
// pair with both its images, and only the vetoed (5,15) and (15,25) would make its two images one spot
static bool VetoedPairsAreNotRepaired(const TestScope& scope)
{
	Corridor corridor = MakeCorridor();
	Scene& scene = corridor.scene;
	PlantNearDuplicates(corridor, kNearOffset);
	const PairIds turnedIds(5, 5 + 2 * kPeriod);
	ImagePair& turned = AddBeside(corridor, turnedIds.first, turnedIds.second, 1);
	turned.relativePose->R = RMatrix(Matrix3x3(RMatrix(Point3(0, D2R(REAL(30)), 0)) * turned.relativePose->R));
	const float turnedAngle = R2D(turned.meanRayAngle);
	FlipBaseline(*scene.FindPair(12, 17));
	ComputePairsWeights(scene);
	const PairSet expectedKept = Without(corridor.TruePairs(), {turnedIds});
	unsigned numCommon = 0; // the images holding a true pair with both images of the turned pair
	for (IIndex i = 0; i < kCorridorImages; ++i)
		numCommon += expectedKept.count({MINF(i, turnedIds.first), MAXF(i, turnedIds.first)}) && expectedKept.count({MINF(i, turnedIds.second), MAXF(i, turnedIds.second)});
	TEST_CHECK(turnedAngle >= kNearDuplicateRayAngle && numCommon == 0, "premise not met: the turned pair (5,25) has a ray angle of %g degrees, a near-duplicate under %g, "
		"or %u images hold a true pair with both its images", turnedAngle, kNearDuplicateRayAngle, numCommon);
	// judged: every pair that is no link but the turned one, cut before the veto, and the flipped one, which
	// triangulates nothing against its matches
	const SequenceNeighbours neighbours = CorridorNeighbours();
	if (!ExpectCut(scope, "the repair", scene, &neighbours, {1, kCorridorLinks, 220 + 20 - 1, 28 + 20, 0}, expectedKept, true))
		return false;
	const unsigned numReturned = (unsigned)std::count_if(scene.pairs.begin(), scene.pairs.end(), [](const ImagePair& pair) { return pair.returnedByRepair; });
	TEST_CHECK(numReturned == 1 && scene.FindPair(12, 17)->returnedByRepair, "the repair: %u pairs returned, expected the flipped (12,17) alone", numReturned);
	return true;
}

// The view-graph filter of a matched scene (a match-images-only Scene::Reconstruct) reads the sequence from the
// configuration and the names, out_* then back_*: a break at the turn. With the option, and in the SEQUENTIAL mode
// without it, the chain is two segments and exactly the look-alikes of both ways go; in another run they stay
static bool ReconstructReadsTheSequence(const TestScope& scope)
{
	using STATE = Scene::Status::STATE;
	const ScopedTempDir tmpDir(_T("SequenceVetoTest"));
	TEST_CHECK(tmpDir.IsValid(), "cannot create the temporary folder");
	Corridor corridor = MakeCorridor(kCorridorImages, true);
	corridor.scene.status.nState.set(STATE::FEATURES_EXTRACTED);
	corridor.scene.status.nState.set(STATE::MATCHED);
	const String matchedPath = tmpDir(_T("matched.sfm"));
	TEST_CHECK(corridor.scene.Save(matchedPath), "cannot save the matched corridor '%s'", matchedPath.c_str());
	ReconstructionConfig cfg;
	cfg.matchImagesOnly = true;
	cfg.matchCfg.lensFitEnabled = false; // the corridor's camera is exact: no lens to fit
	const String solved = String::FormatString("Sequence chain solve: 2 segment(s) placing %u images by %u links of", kCorridorImages, 2 * (29 + 28 + 27));
	struct Run { bool option; MatchConfig::MatchMode mode; bool isSequence; const char* what; };
	for (const Run& run : {Run{false, MatchConfig::VOCABULARY, false, "without the option"}, Run{true, MatchConfig::VOCABULARY, true, "with the option"},
		Run{false, MatchConfig::SEQUENTIAL, true, "in the SEQUENTIAL mode"}})
	{
		cfg.matchCfg.imageSequence = run.option;
		cfg.matchCfg.mode = run.mode;
		LogCapture log;
		Scene scene(2);
		TEST_CHECK(scene.Reconstruct(matchedPath, cfg) && scene.status.nState.isSet(STATE::PAIRS_FILTERED), "the match-images-only run of the matched corridor %s failed", run.what);
		const PairSet kept = KeptPairs(scene);
		if (run.isSequence)
			TEST_CHECK(log.Contains(solved) && kept == corridor.TruePairs(), "the view-graph filter %s keeps %zu pairs, expected the %zu that are no look-alikes, and \"%s\"",
				run.what, kept.size(), corridor.TruePairs().size(), solved.c_str());
		else
			TEST_CHECK(!log.Contains("Sequence chain") && Without(corridor.lookAlikes, kept).size() < corridor.lookAlikes.size(), "premise not met: the view-graph filter %s "
				"solves a chain, or keeps no look-alike", run.what);
	}
	return true;
}

// The protection of a link, on the arc of the contradiction cut's tests: the neighbour pair (7,8) with its baseline
// flipped. With matches that support the flipped pose (new features of image 8, where a camera at its centre mirrored
// about image 7's sees the points) it is a link: removed without the neighbours, kept with them, the one link the
// translation stage would have removed. Flipped against its true matches it triangulates nothing: it has no scale,
// is no link, and is removed with the neighbours as without
static bool FlippedLinkIsProtected(const TestScope& scope)
{
	const auto buildArc = [](Scene& scene, bool supported) {
		Scene sceneGT;
		BuildArc(scene, &sceneGT);
		ImagePair& pair = *scene.FindPair(7, 8);
		if (supported) {
			const Image mirrored = StandingAt(scene.images[8], Point3(scene.images[7].C) * REAL(2) - Point3(scene.images[8].C));
			std::vector<Point3> points;
			std::vector<uint32_t> featureOfFirst;
			for (const Track& track : sceneGT.tracks) {
				const auto seen = std::find_if(track.observations.begin(), track.observations.end(), [](const Observation& obs) { return obs.imageID == 7; });
				points.push_back(track.position);
				featureOfFirst.push_back(seen == track.observations.end() ? NO_ID : seen->featureID);
			}
			MatchAsSeenFrom(scene, pair, mirrored, points, featureOfFirst);
			SetExactPose(scene, pair, mirrored, norm(pair.relativePose->C));
		} else
			FlipBaseline(pair);
		ComputePairsWeights(scene);
	};
	const SequenceNeighbours neighbours(std::vector<bool>(15, false), 3);
	const unsigned numLinks = 15 + 14 + 13;
	Scene scene, unsupported;
	buildArc(scene, true);
	buildArc(unsupported, false);
	Scene plain(scene);
	const PairSet all = KeptPairs(scene), others = Without(all, {{7, 8}});
	const unsigned numJudged = (unsigned)all.size() - numLinks; // every pair that is no neighbour
	TEST_CHECK(RunCut(plain, kDefaultDirectionBar) == 1 && KeptPairs(plain) == others, "the flipped neighbour pair (7,8) without the neighbours: %zu of %zu pairs kept, expected it alone removed",
		plain.pairs.size(), all.size());
	LogCapture log;
	if (!ExpectCut(scope, "the flipped link (7,8)", scene, &neighbours, {1, numLinks, numJudged, 0, 1}, all) ||
		!ExpectCut(scope, "the neighbour pair (7,8) flipped against its matches", unsupported, &neighbours, {1, numLinks - 1, numJudged, 0, 0}, others))
		return false;
	TEST_CHECK(log.Count("Translation cut: kept 1 link(s) of the sequence chain it would have removed") == 1, "the translation stage did not report the one link it kept, once");
	return true;
}

// Pairs under the near-duplicate ray angle are judged like any other. The near-duplicate look-alikes (i, i + 10) of
// the way out, the second frame a quarter of a step past the period or before it (the same picture a little to the
// side), are vetoed with the look-alikes, and stay without the neighbours; the true pairs whose ray angle is set
// under the angle (those more than 3 apart of each third image, revisits among them; the geometry untouched) are
// judged and kept. With the second frame at the period exactly no match has a ray angle: the pair has no scale
// equation, is not judged and stays
static bool VetoJudgesNearDuplicates(const TestScope& scope)
{
	const SequenceNeighbours neighbours = CorridorNeighbours();
	Corridor corridor = MakeCorridor(), exact(corridor);
	Scene& scene = corridor.scene;
	PlantNearDuplicates(corridor, kNearOffset);
	unsigned numLow = 0, numLowRevisits = 0;
	float minAngle = FLT_MAX, maxAngle = 0;
	for (ImagePair& pair : scene.pairs) {
		const PairIds ids(pair.ID1, pair.ID2);
		if (corridor.nearDuplicates.count(ids)) {
			minAngle = MINF(minAngle, R2D(pair.meanRayAngle));
			maxAngle = MAXF(maxAngle, R2D(pair.meanRayAngle));
		} else if (!corridor.lookAlikes.count(ids) && pair.ID2 - pair.ID1 > 3 && pair.ID1 % 3 == 0) {
			pair.meanRayAngle = D2R(kNearDuplicateRayAngle * 0.75f);
			++numLow;
			numLowRevisits += pair.ID1 + pair.ID2 == kCorridorImages - 1;
		}
	}
	ComputePairsWeights(scene);
	TEST_CHECK(corridor.nearDuplicates.size() == 20 && minAngle > 0.5f && maxAngle < kNearDuplicateRayAngle && numLow == 65 && numLowRevisits == 10,
		"premise not met: %zu near-duplicate look-alikes with a ray angle of %g-%g degrees, expected 20 between 0.5 and %g; %u true pairs under the angle, %u of them revisits, "
		"expected 65 and 10", corridor.nearDuplicates.size(), minAngle, maxAngle, kNearDuplicateRayAngle, numLow, numLowRevisits);
	Scene plain(scene);
	RunCut(plain, kDefaultDirectionBar);
	const PairSet keptPlain = KeptPairs(plain);
	TEST_CHECK(std::includes(keptPlain.begin(), keptPlain.end(), corridor.nearDuplicates.begin(), corridor.nearDuplicates.end()),
		"premise not met: without the neighbours the cut removes a near-duplicate look-alike");
	{
		LogCapture log(4); // the lines of the vetoed pairs
		if (!ExpectCut(scope, "the near-duplicates", scene, &neighbours, {1, kCorridorLinks, 220 + 20, 28 + 20, 0}, corridor.TruePairs()) ||
			!ExpectVetoedLines(scope, "the near-duplicates", log.text, corridor))
			return false;
	}
	PlantNearDuplicates(exact, 0);
	return ExpectCut(scope, "the same picture exactly", exact.scene, &neighbours, {1, kCorridorLinks, 220, 28, 0}, Without(exact.pairs, exact.lookAlikes));
}

// The capture ends standing still: the last six cameras (54-59) at one place, the pairs among them without a baseline.
// 54, 55 and 56 are placed by their links to the frames before the place and 57, 58 and 59, whose neighbour pairs all
// stand still, where 56 stands. Their pairs are judged like any other: the look-alikes of a frame standing still are
// vetoed and no true pair is; the 15 pairs among the six triangulate nothing and are not judged
static bool VetoJudgesStillImages(const TestScope& scope)
{
	const IIndex stillFrom = 54, firstStill = stillFrom + 3;
	Corridor corridor = MakeCorridor(kCorridorImages, true, stillFrom);
	unsigned numStillPairs = 0, numStillNeighbours = 0, numStillLookAlikes = 0;
	for (const PairIds& ids : corridor.pairs) {
		numStillPairs += ids.first >= stillFrom;
		numStillNeighbours += ids.first >= stillFrom && ids.second - ids.first <= 3;
		numStillLookAlikes += ids.second >= firstStill && corridor.lookAlikes.count(ids);
	}
	TEST_CHECK(corridor.lookAlikes.size() == 2 * 28 && numStillLookAlikes == 6 && numStillPairs == 15 && numStillNeighbours == 12, "the corridor ending still holds %zu look-alikes, "
		"%u of them of the images 57-59, and %u pairs standing still, %u of them neighbours", corridor.lookAlikes.size(), numStillLookAlikes, numStillPairs, numStillNeighbours);
	const unsigned numLinks = kCorridorLinks - numStillNeighbours, numJudged = (unsigned)corridor.pairs.size() - numLinks - numStillPairs;
	const SequenceNeighbours neighbours = CorridorNeighbours();
	LogCapture log(4); // the lines of the vetoed pairs
	if (!ExpectCut(scope, "the corridor ending still", corridor.scene, &neighbours, {1, numLinks, numJudged, 2 * 28, 0}, corridor.TruePairs()) ||
		!ExpectVetoedLines(scope, "the corridor ending still", log.text, corridor))
		return false;
	const String solved = String::FormatString("Sequence chain solve: 1 segment(s) placing %u images by %u links of %u candidates", firstStill, numLinks, numLinks);
	TEST_CHECK(log.Contains(solved) && log.Contains("), 3 still image(s) ("), "the corridor ending still: the solve did not log \"%s\" and 3 still image(s)", solved.c_str());
	return true;
}

// The SEQUENTIAL mode is a sequence run that also matches the last images with the first, to close a loop: pairs
// that are no neighbours, so no links, and the veto judges them. On a loop of 80 cameras looking outwards, with the
// pairs of that mode at an overlap of 3, the 6 closing pairs are true: judged and kept. They are kept when the chain
// drifts, too: with every link's baseline tilted up by 0.15 of its length the chain is a helix that ends 12 steps
// above its start, a closing pair a step long that far off it. That is more than 8 local steps and less than 0.2 of
// the path between its two images, the term of the tolerance that is there for the drift. (The bar of the translation
// stage is then 180 degrees: its centres drift with the links.)
static bool VetoKeepsLoopClosures(const TestScope& scope)
{
	constexpr IIndex numImages = 80;
	const REAL radius = numImages / (2 * M_PI); // a step of 1
	std::vector<Pose3D> poses;
	for (IIndex i = 0; i < numImages; ++i) {
		const RMatrix R(Point3(0, 2 * M_PI * i / numImages, 0));
		poses.emplace_back(R, Point3(R.t() * Point3(0, 0, radius)));
	}
	// points all around, 15-30 units outside the loop: far enough to triangulate with the tilted baselines
	std::mt19937 rng(13);
	std::uniform_real_distribution<REAL> angleDist(0, 2 * M_PI), distanceDist(radius + 15, radius + 30), yDist(-3, 3);
	std::vector<Point3> points(4000);
	for (Point3& X : points) {
		const REAL angle = angleDist(rng), distance = distanceDist(rng);
		X = Point3(distance * SIN(angle), yDist(rng), distance * COS(angle));
	}
	Strip loop(poses, points);
	for (IIndex i = 0; i < numImages; ++i)
		for (IIndex k = 1; k <= 3; ++k)
			loop.AddPair(MINF(i, (i + k) % numImages), MAXF(i, (i + k) % numImages), 1);
	ComputePairsWeights(loop.scene);
	Scene drifting(loop.scene);
	for (ImagePair& pair : drifting.pairs)
		if (pair.ID2 - pair.ID1 <= 3) // the cameras turn about the vertical, Y in every frame
			pair.relativePose->C = CMatrix(pair.relativePose->C + Point3(0, REAL(0.15) * norm(pair.relativePose->C), 0));
	MatchConfig cfg;
	cfg.mode = MatchConfig::SEQUENTIAL;
	const std::optional<SequenceNeighbours> neighbours = SequenceNeighbours::FromConfig(loop.scene, cfg);
	TEST_CHECK(neighbours.has_value(), "the closed loop: the SEQUENTIAL mode has no neighbours");
	const SequenceFilterStats expected{1, 3 * numImages - 6, 6, 0, 0};
	return ExpectCut(scope, "the closed loop", loop.scene, &*neighbours, expected, KeptPairs(loop.scene)) &&
		ExpectCut(scope, "the closed loop, its chain drifting", drifting, &*neighbours, expected, KeptPairs(drifting), false, 180.f);
}

// The chain of an image sequence in the contradiction cut (FilterPairsByContradiction given the neighbours), on a
// periodic corridor with real features (MakeCorridor), the neighbours 3 apart:
//  - the rule by values (ChainVetoes), and the pair-scale equations the veto reads (ComputePairScales with anchors);
//  - the veto removes exactly the look-alikes and judges every other pair; without the neighbours the cut keeps them;
//  - it keeps to a segment: a name break at the turn, two components of the rotation averaging;
//  - it abstains where no chain is solved: too few cameras, no translation stage, no neighbours, no images;
//  - a vetoed pair is not offered to the repair and makes no spot there;
//  - Scene::Reconstruct reads the sequence from the option, or the SEQUENTIAL mode, and the names;
//  - a link the translation stage would remove is kept, a neighbour pair the chain did not solve is not;
//  - pairs under the near-duplicate ray angle, and the pairs of images placed standing still, are judged;
//  - the pairs closing a loop are judged, and kept while the chain's drift is within the path term of the tolerance.
bool SequenceVetoTest()
{
	TestScope scope("SequenceVetoTest");
	if (!VetoRuleByValues(scope) || !AnchoredPairScalesAreThePlainOnes(scope) || !VetoRemovesLookAlikes(scope) || !VetoKeepsToASegment(scope) ||
		!VetoAbstainsWithoutAChain(scope) || !VetoedPairsAreNotRepaired(scope) || !ReconstructReadsTheSequence(scope) || !FlippedLinkIsProtected(scope) ||
		!VetoJudgesNearDuplicates(scope) || !VetoJudgesStillImages(scope) || !VetoKeepsLoopClosures(scope))
		return false;
	return scope.Pass("the rule and its equations; on the corridor the 28 look-alikes vetoed of 220 judged pairs, by segment, never repaired; the abstentions; "
		"Scene::Reconstruct; the protected link; the near-duplicates, the still images and the loop closures judged");
}

// A matched scene goes through the view-graph filters (the triplet filter, the contradiction cut and its repair)
// once, and a scene saved after them is where the reconstruction starts. On the ring of ContradictionCutTest saved as
// matched, three pairs planted the repair brings back: (1) a match-images-only run that loads it filters it once,
// to the pairs, repair marks and seed views of the two filters applied once by hand, and leaves it PAIRS_FILTERED;
// (2) saved so and given back as source, it is not filtered again: the reloaded scene holds the very pairs, marks,
// weights and seed views it was saved with, a pair flipped after the filters (which they remove when they run)
// included.
bool FilteredSceneReloadTest()
{
	TestScope scope("FilteredSceneReloadTest");
	const ScopedTempDir tmpDir(_T("FilteredSceneReloadTest"));
	TEST_CHECK(tmpDir.IsValid(), "cannot create the temporary folder");
	// the pairs, their matches, poses, weights and repair marks, and the seed views, equal
	const auto sameGraph = [&scope](const Scene& a, const Scene& b, const char* what) -> bool {
		bool same = a.pairs.size() == b.pairs.size() && a.seedViews.size() == b.seedViews.size() &&
			std::equal(a.seedViews.begin(), a.seedViews.end(), b.seedViews.begin());
		for (IIndex i = 0; same && i < a.pairs.size(); ++i) {
			const ImagePair& p = a.pairs[i];
			const ImagePair& q = b.pairs[i];
			same = p.ID1 == q.ID1 && p.ID2 == q.ID2 && p.matches.size() == q.matches.size() &&
				p.numFilteredInliers == q.numFilteredInliers && p.numDenseInliers == q.numDenseInliers &&
				p.returnedByRepair == q.returnedByRepair && p.relativePose.has_value() == q.relativePose.has_value() &&
				(!p.relativePose.has_value() || (ComputeAngle(p.relativePose->R, q.relativePose->R) > 1 - 1e-12 &&
					norm(p.relativePose->C - q.relativePose->C) < 1e-12)) &&
				ISEQUAL(p.GetCompositeWeight(), q.GetCompositeWeight());
		}
		TEST_CHECK(same, "%s: %u pairs and %u seed views, expected %u pairs and %u seed views, each the same",
			what, (unsigned)b.pairs.size(), (unsigned)b.seedViews.size(), (unsigned)a.pairs.size(), (unsigned)a.seedViews.size());
		return true;
	};
	ReconstructionConfig cfg;
	cfg.matchImagesOnly = true;
	cfg.matchCfg.lensFitEnabled = false; // the ring's cameras are exact: no lens to fit
	using STATE = Scene::Status::STATE;

	// the ring saved as matched, three pairs planted that the cut removes and its repair brings back
	Scene matched;
	{
		Scene sceneGT;
		GenerateTestScene(sceneGT, RingConfig(300), &matched);
	}
	for (ImagePair& pair : matched.pairs)
		pair.meanRayAngle = D2R(10.f);
	ImagePair* pFlipped = matched.FindPair(3, 9);
	ImagePair* pRotated = matched.FindPair(6, 14);
	TEST_CHECK(pFlipped != NULL && pRotated != NULL && matched.FindPair(5, 13) != NULL, "the ring has no pair (3,9), (6,14) or (5,13)");
	FlipBaseline(*pFlipped);
	pRotated->relativePose->R = RMatrix(Point3(0, D2R(REAL(30)), 0)) * pRotated->relativePose->R;
	TEST_CHECK(matched.status.nState.isSet(STATE::MATCHED) && !matched.status.nState.isSet(STATE::PAIRS_FILTERED), "the generated ring is not a matched scene before the filters");
	const String matchedPath = tmpDir(_T("matched.sfm"));
	TEST_CHECK(matched.Save(matchedPath), "cannot save the matched scene '%s'", matchedPath.c_str());

	// (1) the filters run once on the loaded matched scene: the two filters applied once by hand to the same scene
	Scene filtered(2);
	TEST_CHECK(filtered.Reconstruct(matchedPath, cfg) && filtered.status.nState.isSet(STATE::PAIRS_FILTERED), "the match-images-only run of the matched scene failed or left it unfiltered");
	Scene once(2);
	TEST_CHECK(once.Load(matchedPath), "cannot load the matched scene");
	ComputePairsWeights(once, cfg.matchCfg.weightingCfg);
	std::vector<float> scores;
	FilterPairsByTriplets(once, cfg.tripletFilterCfg, cfg.matchCfg.weightingCfg, &once.seedViews, &scores);
	FilterPairsByContradiction(once, scores, cfg.minPairWeight, cfg.maxPairDirectionResidual, cfg.matchCfg.weightingCfg, cfg.repairContradictedPairs);
	if (!sameGraph(once, filtered, "the matched scene loaded and filtered"))
		return false;
	const unsigned numReturned = (unsigned)std::count_if(filtered.pairs.begin(), filtered.pairs.end(), [](const ImagePair& pair) { return pair.returnedByRepair; });
	TEST_CHECK(numReturned != 0 && !filtered.seedViews.empty(), "premise not met: the filters repaired %u pairs and named %u seed views, expected some of each",
		numReturned, (unsigned)filtered.seedViews.size());

	// (2) a pair flipped after the filters, which they remove when they run, then the scene saved and given back
	FlipBaseline(*filtered.FindPair(5, 13));
	{
		Scene again(filtered);
		const std::vector<float> unscored(again.pairs.size(), TripletScores::unscored);
		TEST_CHECK(FilterPairsByContradiction(again, unscored, cfg.minPairWeight, cfg.maxPairDirectionResidual, cfg.matchCfg.weightingCfg) != 0,
			"premise not met: the contradiction cut does not remove the pair (5,13) flipped");
	}
	const String filteredPath = tmpDir(_T("filtered.sfm"));
	TEST_CHECK(filtered.Save(filteredPath), "cannot save the filtered scene '%s'", filteredPath.c_str());
	Scene reloaded(2);
	TEST_CHECK(reloaded.Reconstruct(filteredPath, cfg) && reloaded.status.nState.isSet(STATE::PAIRS_FILTERED), "the match-images-only run of the filtered scene failed or left it unfiltered");
	ComputePairsWeights(filtered, cfg.matchCfg.weightingCfg); // the weights of the graph with (5,13) flipped, as the run weighs them
	if (!sameGraph(filtered, reloaded, "the filtered scene reloaded"))
		return false;
	return scope.Pass("%u pairs, %u repaired, %u seed views", (unsigned)reloaded.pairs.size(), numReturned, (unsigned)reloaded.seedViews.size());
}

// ComputePairScales (its equations built in flat per-view arrays, in parallel) gives the equations of the
// node-map build it replaced, which the test carries: the same count, in the same order, with the same
// weights to the last bit and the same scale ratios to the rounding of the trimmed sum (the shared
// cList::GetTrimmedMean adds the kept ratios in another order than the sorted one), at one thread and
// at four, on a scene whose pairs share features, one of them listing a feature twice, over all the
// pairs and over a subset of them.
static std::vector<ScalePair> ReferencePairScales(const Scene& scene, const IIndexArr& pairIndices)
{
	typedef std::pair<uint32_t, double> PairDist;
	std::unordered_map<uint32_t, std::unordered_map<uint32_t, std::vector<PairDist>>> viewFeatureDists;
	const REAL maxCosAngle = COS(D2R(0.5));
	const REAL reprojPixelThreshold = 6;
	for (const IIndex pairIdx : pairIndices) {
		const ImagePair& pair = scene.pairs[pairIdx];
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		const Camera& cam1 = *img1.pCamera;
		const Camera& cam2 = *img2.pCamera;
		const Pose3D& relPose = pair.relativePose.value();
		const REAL cosReprojAngle = COS(REAL(0.5) * (cam1.PixelErrorToAngular(reprojPixelThreshold) + cam2.PixelErrorToAngular(reprojPixelThreshold)));
		for (const DMatch& match : pair.matches) {
			const Point3 b1 = cam1.UnprojectNormalized(Cast<REAL>(img1.keypoints[match.queryIdx].pt));
			const Point3 b2 = cam2.UnprojectNormalized(Cast<REAL>(img2.keypoints[match.trainIdx].pt));
			Point3 Xlocal;
			if (!TriangulatePoint3D(relPose.R, relPose.C, b1, b2, Xlocal))
				continue;
			const REAL nX = norm(Xlocal);
			if (nX < ZEROTOLERANCE<REAL>() || b1.dot(Xlocal / nX) < cosReprojAngle)
				continue;
			const Point3 Xcam2 = relPose.TransformPointW2C(Xlocal);
			const REAL nXc2 = norm(Xcam2);
			if (nXc2 < ZEROTOLERANCE<REAL>() || b2.dot(Xcam2 / nXc2) < cosReprojAngle)
				continue;
			const Point3 V1 = Xlocal, V2 = Xlocal - relPose.C;
			if (ComputeAngle(V1.ptr(), V2.ptr()) > maxCosAngle)
				continue;
			std::vector<PairDist>& arr1 = viewFeatureDists[pair.ID1][match.queryIdx];
			if (std::none_of(arr1.begin(), arr1.end(), [pairIdx](const PairDist& e) { return e.first == pairIdx; }))
				arr1.emplace_back(pairIdx, cv::norm(Xlocal));
			std::vector<PairDist>& arr2 = viewFeatureDists[pair.ID2][match.trainIdx];
			if (std::none_of(arr2.begin(), arr2.end(), [pairIdx](const PairDist& e) { return e.first == pairIdx; }))
				arr2.emplace_back(pairIdx, cv::norm(Xlocal - relPose.C));
		}
	}
	std::map<uint32_t, std::map<uint32_t, DoubleArr>> pairRatios;
	for (const auto& viewIt : viewFeatureDists)
		for (const auto& featIt : viewIt.second) {
			const std::vector<PairDist>& obs = featIt.second;
			for (size_t i = 0; i + 1 < obs.size(); ++i)
				for (size_t j = i + 1; j < obs.size(); ++j) {
					uint32_t pairIdx1 = obs[i].first, pairIdx2 = obs[j].first;
					double dist1 = obs[i].second, dist2 = obs[j].second;
					if (pairIdx1 > pairIdx2) {
						std::swap(pairIdx1, pairIdx2);
						std::swap(dist1, dist2);
					}
					pairRatios[pairIdx1][pairIdx2].push_back(dist2 / dist1);
				}
		}
	std::vector<ScalePair> scalePairs;
	for (auto& it1 : pairRatios)
		for (auto& it2 : it1.second) {
			DoubleArr& ratios = it2.second;
			if (ratios.size() < 15)
				continue;
			ratios.Sort();
			const size_t start = ratios.size() / 10, end = ratios.size() - start;
			const double numValid = static_cast<double>(end - start);
			scalePairs.emplace_back(it2.first, it1.first, std::accumulate(ratios.begin() + start, ratios.begin() + end, 0.0) / numValid, (float)SQRT(numValid));
		}
	return scalePairs;
}

bool PairScalesTest()
{
	TestScope scope("PairScalesTest");
	Scene sceneGT, scene;
	SceneConfig cfg;
	cfg.numImages = 24;
	cfg.numPoints = 400;
	cfg.rotationAngleStep = 10.0;
	cfg.generatePairs = true;
	GenerateTestScene(sceneGT, cfg, &scene);
	// a feature listed twice by one pair: the first match's distance is the one kept
	for (ImagePair& pair : scene.pairs)
		if (pair.matches.size() > 20) {
			pair.matches.push_back(pair.matches[5]);
			break;
		}
	IIndexArr all, subset;
	FOREACH(i, scene.pairs) {
		if (!scene.pairs[i].relativePose.has_value())
			continue;
		all.push_back((IIndex)i);
		if (i % 3 != 1)
			subset.push_back((IIndex)i);
	}
	#ifdef _USE_OPENMP
	const int maxThreads = omp_get_max_threads();
	#endif
	double maxRelDiff = 0; // the largest relative difference of a scale ratio from the reference's
	for (const IIndexArr* pPairs : {&all, &subset}) {
		const std::vector<ScalePair> expected = ReferencePairScales(scene, *pPairs);
		TEST_CHECK(expected.size() >= 50, "the scene gives only %zu equations", expected.size());
		for (const int numThreads : {1, 4}) {
			#ifdef _USE_OPENMP
			omp_set_num_threads(numThreads);
			#endif
			const std::vector<ScalePair> equations = ComputePairScales(scene, *pPairs);
			bool same = equations.size() == expected.size();
			for (size_t i = 0; same && i < equations.size(); ++i) {
				const double relDiff = ABS(equations[i].scaleRatio / expected[i].scaleRatio - 1);
				maxRelDiff = MAXF(maxRelDiff, relDiff);
				same = equations[i].idxA == expected[i].idxA && equations[i].idxB == expected[i].idxB &&
					relDiff < 1e-12 && equations[i].weight == expected[i].weight;
			}
			TEST_CHECK(same, "%zu pairs, %d threads: %zu equations against the reference's %zu, or they differ", pPairs->size(), numThreads, equations.size(), expected.size());
		}
	}
	#ifdef _USE_OPENMP
	omp_set_num_threads(maxThreads);
	#endif
	return scope.Pass("scale ratios within %g of the reference's", maxRelDiff);
}

// The scale estimator solves every connected component of its equations on its own: the one holding the fixed index is
// pinned at the fixed index, any other has its log-scales at mean zero (the minimum-norm least squares of the whole system),
// and an index no equation names keeps scale 1. The reference is a dense JacobiSVD of the whole system with the fixed index
// eliminated, computed here; the answer must equal it to 1e-9 in log scale. Cases: two components with the fixed index in the
// first, then in the second; the same estimator solving the two components joined by a bridge and then without it (a robust
// round that drops the bridge leaves a component the fixed index no longer reaches, which must come out at mean zero whatever
// the solve before it left there); and no fixed index given, as the view-graph filter calls the estimator (the 3-argument
// overload among them), whose gauge is then the index of the largest summed weight, in the first component and in the second.
bool ScaleAveragingTest()
{
	TestScope scope("ScaleAveragingTest");
	std::mt19937 rng(11);
	std::uniform_real_distribution<double> uniform(0, 1);
	std::normal_distribution<double> noise(0, 0.05);
	// the first component holds indices 0..5, the second 7..11, and index 6 is named by no equation
	constexpr uint32_t numIndices = 12, unnamed = 6;
	const std::vector<uint32_t> first{0, 1, 2, 3, 4, 5}, second{7, 8, 9, 10, 11};
	std::vector<double> logScales(numIndices);
	for (double& logScale : logScales)
		logScale = 2 * uniform(rng) - 1;
	// noisy ratios (so the least squares is not the planted scales), a ring and chords in each component
	std::vector<ScalePair> separate;
	const auto addEquations = [&](const std::vector<uint32_t>& indices) {
		const auto add = [&](uint32_t a, uint32_t b) {
			separate.emplace_back(a, b, std::exp(logScales[b] - logScales[a] + noise(rng)), (float)(1 + 9 * uniform(rng)));
		};
		for (size_t i = 0; i < indices.size(); ++i)
			add(indices[i], indices[(i + 1) % indices.size()]);
		for (size_t i = 0; i + 2 < indices.size(); ++i)
			add(indices[i], indices[i + 2]);
	};
	addEquations(first);
	addEquations(second);
	std::vector<ScalePair> bridged = separate;
	bridged.emplace_back(3, 8, std::exp(logScales[8] - logScales[3] + noise(rng)), 5.f);
	// the dense reference: the whole system with the fixed index eliminated, its minimum-norm least squares
	const auto reference = [&](const std::vector<ScalePair>& system, uint32_t fixed) {
		std::vector<int> column(numIndices, -1);
		int numColumns = 0;
		for (const ScalePair& pair : system)
			for (const uint32_t idx : {pair.idxA, pair.idxB})
				if (idx != fixed && column[idx] < 0)
					column[idx] = numColumns++;
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero((Eigen::Index)system.size(), numColumns);
		Eigen::VectorXd b((Eigen::Index)system.size());
		FOREACH(i, system) {
			const ScalePair& pair = system[i];
			if (column[pair.idxA] >= 0)
				A(i, column[pair.idxA]) = -(double)pair.weight;
			if (column[pair.idxB] >= 0)
				A(i, column[pair.idxB]) = (double)pair.weight;
			b(i) = (double)pair.weight * std::log((double)pair.scaleRatio);
		}
		const Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
		std::vector<double> expected(numIndices, 0.0);
		for (uint32_t i = 0; i < numIndices; ++i)
			if (column[i] >= 0)
				expected[i] = x(column[i]);
		return expected;
	};
	// the gauge the estimator takes when none is given: the index of the largest summed weight
	const auto bestConnected = [&](const std::vector<ScalePair>& system) {
		std::vector<float> connection(numIndices, 0.f);
		for (const ScalePair& pair : system) {
			connection[pair.idxA] += pair.weight;
			connection[pair.idxB] += pair.weight;
		}
		uint32_t best = 0;
		for (uint32_t i = 1; i < numIndices; ++i)
			if (connection[i] > connection[best])
				best = i;
		return best;
	};
	// one solve of `system` by `estimator` against the reference, asking for the fixed index `requested` (NO_ID: none, through
	// the 3-argument overload when `shortCall`); `twoComponents`: the system has the two components, the one the fixed index
	// does not reach is at mean zero
	const auto check = [&](const char* what, GlobalScaleEstimator& estimator, const std::vector<ScalePair>& system, uint32_t requested, bool twoComponents, bool shortCall = false) -> bool {
		const uint32_t fixed = requested != NO_ID ? requested : bestConnected(system);
		std::vector<REAL> scales;
		const bool solved = shortCall ? estimator.EstimateScales(system, numIndices, scales) : estimator.EstimateScales(system, numIndices, requested, scales);
		TEST_CHECK(solved && scales.size() == numIndices, "%s: the solve failed", what);
		TEST_CHECK(std::abs(std::log((double)scales[fixed])) < 1e-12, "%s: the fixed index %u has scale %g, not 1", what, fixed, (double)scales[fixed]);
		TEST_CHECK(std::abs(std::log((double)scales[unnamed])) < 1e-12, "%s: the index no equation names has scale %g, not 1", what, (double)scales[unnamed]);
		const std::vector<double> expected = reference(system, fixed);
		double worst = 0;
		for (uint32_t i = 0; i < numIndices; ++i)
			worst = MAXF(worst, std::abs(std::log((double)scales[i]) - expected[i]));
		double meanUnreached = 0;
		if (twoComponents) {
			const std::vector<uint32_t>& unreached = std::find(first.begin(), first.end(), fixed) != first.end() ? second : first;
			for (const uint32_t idx : unreached)
				meanUnreached += std::log((double)scales[idx]);
			meanUnreached /= (double)unreached.size();
		}
		TEST_CHECK(worst < 1e-9 && std::abs(meanUnreached) < 1e-9, "%s: the log scales differ from the dense SVD's by up to %g; the mean log scale of the component "
			"the fixed index does not reach is %g", what, worst, meanUnreached);
		return true;
	};
	GlobalScaleEstimator inFirst, inSecond, bridge;
	if (!check("two components, the fixed index in the first", inFirst, separate, 2, true))
		return false;
	if (!check("two components, the fixed index in the second", inSecond, separate, 9, true))
		return false;
	if (!check("one component, joined by a bridge", bridge, bridged, 2, false))
		return false;
	if (!check("the bridge dropped after a solve with it", bridge, separate, 2, true))
		return false;
	// no fixed index given: the weights of one component raised tenfold put the best-connected index in it
	for (const bool inFirstComponent : {true, false}) {
		std::vector<ScalePair> weighted = separate;
		for (ScalePair& pair : weighted)
			pair.weight *= (pair.idxA < unnamed) == inFirstComponent ? 10.f : 1.f;
		const char* const what = inFirstComponent ? "no fixed index, the best-connected index in the first component" : "no fixed index, the best-connected index in the second component";
		TEST_CHECK((bestConnected(weighted) < unnamed) == inFirstComponent, "%s: the test's best-connected index %u is in the other component", what, bestConnected(weighted));
		GlobalScaleEstimator unfixed;
		if (!check(what, unfixed, weighted, NO_ID, true, inFirstComponent))
		return false;
	}
	return scope.Pass("the log scales equal the dense SVD's to 1e-9 on two components, a bridge, the bridge dropped, and no fixed index");
}

// The global translation estimator (GlobalTranslationEstimator) solves its weighted least squares by the normal equations: the
// matrix it factors is the weighted graph Laplacian of the pairs with the gauge node (and the lowest
// node of each component without it) pinned, exactly the free nodes plus twice the distinct free-free
// pairs of non-zeros, and its centres equal Eigen's SparseQR solution of the incidence matrix to 1e-9
// of the scene's extent on (1) a connected graph, (2) a graph of two dense groups joined by one weak
// pair, row weights over 1e-2..1e2, (3) a graph of two components, the second without the gauge (placed
// up to its own translation, as the QR solution is), (4) two floating rings of unit weights, whose full
// Laplacian is exactly singular; and (5) a graph of the size of a view graph's (1200 nodes, 25173 rows)
// is solved in seconds (a Release build checks the time), not in the half minute the QR takes; and (6) the 3-argument
// overload, which takes no gauge, gives the SparseQR centres up to one translation on a connected graph.
bool TranslationAveragingTest()
{
	TestScope scope("TranslationAveragingTest");
	std::mt19937 rng(11);
	std::uniform_real_distribution<double> uniform(0, 1);
	std::normal_distribution<double> noise(0, 0.5);
	constexpr double extent = 100;
	std::vector<Point3> positions(1200);
	for (Point3& position : positions)
		position = Point3(uniform(rng) * extent, uniform(rng) * extent, uniform(rng) * extent);
	// the rows between nodes of [first, last): a ring and `chords` random pairs per node, weights log-uniform in [1e-2, 1e2]
	const auto addRows = [&](std::vector<TranslationPair>& pairs, uint32_t first, uint32_t last, unsigned chords) {
		const auto add = [&](uint32_t a, uint32_t b) {
			if (a == b)
				return;
			const float weight = (float)std::pow(10.0, uniform(rng) * 4 - 2);
			pairs.emplace_back(a, b, positions[b] - positions[a] + Point3(noise(rng), noise(rng), noise(rng)), weight);
		};
		for (uint32_t i = first; i < last; ++i) {
			add(i, i + 1 < last ? i + 1 : first);
			for (unsigned c = 0; c < chords; ++c)
				add(i, first + (uint32_t)(uniform(rng) * (last - first)) % (last - first));
		}
	};
	// the least-squares reference: SparseQR of the weighted incidence matrix, the gauge node eliminated
	const auto reference = [&](const std::vector<TranslationPair>& pairs, uint32_t gauge, std::vector<Point3>& centres) {
		std::unordered_map<uint32_t, int> variable;
		for (const TranslationPair& pair : pairs)
			for (const uint32_t node : {pair.idxA, pair.idxB})
				if (node != gauge)
					variable.emplace(node, (int)variable.size());
		std::vector<Eigen::Triplet<double>> triplets;
		Eigen::MatrixXd b = Eigen::MatrixXd::Zero((Eigen::Index)pairs.size(), 3);
		FOREACH(i, pairs) {
			const TranslationPair& pair = pairs[i];
			if (pair.idxA != gauge)
				triplets.emplace_back((int)i, variable[pair.idxA], -(double)pair.weight);
			if (pair.idxB != gauge)
				triplets.emplace_back((int)i, variable[pair.idxB], (double)pair.weight);
			for (int d = 0; d < 3; ++d)
				b((Eigen::Index)i, d) = (double)pair.weight * (double)pair.relativeTranslation[d];
		}
		Eigen::SparseMatrix<double> A((int)pairs.size(), (int)variable.size());
		A.setFromTriplets(triplets.begin(), triplets.end());
		Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
		solver.compute(A);
		centres.assign(positions.size(), Point3::ZERO);
		for (int d = 0; d < 3; ++d) {
			const Eigen::VectorXd x = solver.solve(b.col(d));
			for (const auto& [node, index] : variable)
				centres[node][d] = (REAL)x(index);
		}
	};
	// the largest difference of the two solutions, the component of each node taken up to its own
	// translation unless it holds the gauge
	const auto difference = [&](const std::vector<TranslationPair>& pairs, uint32_t gauge, const std::vector<Point3>& a, const std::vector<Point3>& b) {
		DisjointSet<uint32_t> sets((uint32_t)positions.size());
		for (const TranslationPair& pair : pairs)
			sets.Union(pair.idxA, pair.idxB);
		std::unordered_map<uint32_t, uint32_t> origin; // the lowest node of each component
		for (const TranslationPair& pair : pairs)
			for (const uint32_t node : {pair.idxA, pair.idxB}) {
				uint32_t& low = origin.emplace(sets.Find(node), node).first->second;
				low = MINF(low, node);
			}
		REAL worst = 0;
		for (const auto& [root, low] : origin) {
			const uint32_t shiftNode = sets.Find(gauge) == root ? gauge : low;
			for (const TranslationPair& pair : pairs)
				for (const uint32_t node : {pair.idxA, pair.idxB})
					if (sets.Find(node) == root)
						worst = MAXF(worst, norm((a[node] - a[shiftNode]) - (b[node] - b[shiftNode])));
		}
		return worst / extent;
	};
	// the nodes the estimator pins at the origin: the gauge, and the lowest node of each component without it
	const auto pinsOf = [&](const std::vector<TranslationPair>& pairs, uint32_t gauge) {
		DisjointSet<uint32_t> components((uint32_t)positions.size());
		for (const TranslationPair& pair : pairs)
			components.Union(pair.idxA, pair.idxB);
		std::unordered_map<uint32_t, uint32_t> pinOfComponent;
		pinOfComponent.emplace(components.Find(gauge), gauge);
		for (const TranslationPair& pair : pairs)
			for (const uint32_t node : {pair.idxA, pair.idxB}) {
				uint32_t& low = pinOfComponent.emplace(components.Find(node), node).first->second;
				if (low != gauge)
					low = MINF(low, node);
			}
		std::unordered_set<uint32_t> pins;
		for (const auto& [component, pin] : pinOfComponent)
			pins.insert(pin);
		return pins;
	};
	const auto checkLaplacian = [&](const char* what, const GlobalTranslationEstimator& estimator, const std::vector<TranslationPair>& pairs, uint32_t gauge) -> bool {
		// the matrix handed to the factorization is the weighted graph Laplacian of the free nodes (the pinned
		// nodes eliminated), built here from the pairs alone: the
		// diagonal the sum of the squared weights of the node's pairs, an off-diagonal minus the squared
		// weights of the pairs between its two nodes (a duplicated or reversed pair summed), both triangles
		// held, so exactly the free nodes plus twice the distinct free-free pairs of non-zeros (a QR of the
		// incidence matrix has a row per pair)
		const std::unordered_set<uint32_t> pins = pinsOf(pairs, gauge);
		const auto isFree = [&](uint32_t node) { return pins.find(node) == pins.end(); };
		std::unordered_set<uint32_t> nodes;
		std::map<std::pair<uint32_t, uint32_t>, double> laplacian;
		for (const TranslationPair& pair : pairs) {
			nodes.insert(pair.idxA);
			nodes.insert(pair.idxB);
			const double weight2 = SQUARE((double)pair.weight);
			if (isFree(pair.idxA))
				laplacian[{pair.idxA, pair.idxA}] += weight2;
			if (isFree(pair.idxB))
				laplacian[{pair.idxB, pair.idxB}] += weight2;
			if (isFree(pair.idxA) && isFree(pair.idxB)) {
				laplacian[{pair.idxA, pair.idxB}] -= weight2;
				laplacian[{pair.idxB, pair.idxA}] -= weight2;
			}
		}
		const GlobalTranslationEstimator::NormalEquations& normal = estimator.GetNormalEquations();
		const Eigen::SparseMatrix<double>& matrix = normal.matrix;
		size_t count = 0;
		const size_t numFree = nodes.size() - pins.size();
		bool consistent = matrix.rows() == matrix.cols() && (size_t)matrix.rows() == numFree && normal.nodeOfRow.size() == numFree;
		for (int column = 0; consistent && column < matrix.outerSize(); ++column)
			for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, column); it; ++it, ++count) {
				const auto expected = laplacian.find({normal.nodeOfRow[it.row()], normal.nodeOfRow[it.col()]});
				consistent = expected != laplacian.end() && std::abs(it.value() - expected->second) <= 1e-12 * std::abs(expected->second);
				if (!consistent)
					break;
			}
		TEST_CHECK(consistent && count == laplacian.size(), "%s: the factored matrix (%d x %d, %zu non-zeros) is not the Laplacian of the %zu free nodes and %zu entries the pairs give",
			what, (int)matrix.rows(), (int)matrix.cols(), (size_t)matrix.nonZeros(), numFree, laplacian.size());
		return true;
	};
	// one solve against the SparseQR reference: with the gauge the first node of the pairs, or (`noGauge`) through the 3-argument
	// overload, whose centres equal the reference's up to one translation
	const auto check = [&](const char* what, const std::vector<TranslationPair>& pairs, bool noGauge = false) -> bool {
		const uint32_t gauge = pairs.front().idxA;
		GlobalTranslationEstimator estimator;
		std::vector<Point3> centres(positions.size(), Point3(1e6, 1e6, 1e6)), expected; // stale values the solve must overwrite
		const bool solved = noGauge ? estimator.EstimateTranslations(pairs, (uint32_t)positions.size(), centres)
			: estimator.EstimateTranslations(pairs, (uint32_t)positions.size(), gauge, centres);
		TEST_CHECK(solved, "%s: the solve failed", what);
		if (!noGauge) {
			if (!checkLaplacian(what, estimator, pairs, gauge))
				return false;
			// every pinned node (the gauge, and the lowest node of each component without it) is written at the origin
			for (const uint32_t pin : pinsOf(pairs, gauge))
				TEST_CHECK(norm(centres[pin]) == 0, "%s: pinned node %u is %g from the origin", what, pin, (double)norm(centres[pin]));
		}
		reference(pairs, gauge, expected);
		const REAL error = difference(pairs, gauge, centres, expected);
		TEST_CHECK(error < REAL(1e-9), "%s: the centres differ from the SparseQR ones by %g of the extent", what, (double)error);
		return true;
	};
	{
		std::vector<TranslationPair> pairs;
		addRows(pairs, 0, 80, 4);
		if (!check("connected graph", pairs))
			return false;
	}
	{
		std::vector<TranslationPair> pairs;
		addRows(pairs, 0, 40, 6);
		addRows(pairs, 40, 80, 6);
		pairs.emplace_back(39, 40, positions[40] - positions[39], 1e-2f);
		if (!check("weak bridge", pairs))
			return false;
	}
	{
		std::vector<TranslationPair> pairs;
		addRows(pairs, 0, 40, 6);
		addRows(pairs, 50, 90, 6);
		if (!check("two components", pairs))
			return false;
	}
	{
		// unit weights and whole offsets: the floating ring's Laplacian is exactly singular
		std::vector<TranslationPair> pairs;
		for (const uint32_t first : {0u, 10u})
			for (uint32_t i = 0; i < 6; ++i)
				pairs.emplace_back(first + i, first + (i + 1) % 6, Point3(REAL(i % 2), REAL(1), REAL(i % 3)), 1.f);
		if (!check("floating ring of unit weights", pairs))
			return false;
	}
	{
		std::vector<TranslationPair> pairs;
		addRows(pairs, 0, 80, 4);
		if (!check("connected graph, no gauge given", pairs, true))
			return false;
	}
	{
		std::vector<TranslationPair> pairs;
		addRows(pairs, 0, 1200, 20);
		GlobalTranslationEstimator estimator;
		std::vector<Point3> centres;
		#ifdef _RELEASE
		const auto start = std::chrono::steady_clock::now();
		#endif
		TEST_CHECK(estimator.EstimateTranslations(pairs, (uint32_t)positions.size(), 0, centres), "the view-graph sized solve failed");
		#ifdef _RELEASE // a wall-clock bound holds in a Release build only: a Debug or sanitizer build is slower by an order of magnitude
		const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
		#endif
		if (!checkLaplacian("1200 nodes", estimator, pairs, 0))
			return false;
		REAL sumSq = 0;
		for (uint32_t i = 1; i < (uint32_t)positions.size(); ++i)
			sumSq += SQUARE(norm((centres[i] - centres[0]) - (positions[i] - positions[0])));
		const REAL rms = SQRT(sumSq / (REAL)positions.size());
		TEST_CHECK(rms < REAL(1), "1200 nodes, %zu rows: %g rms from the truth", pairs.size(), (double)rms);
		#ifdef _RELEASE
		// a sanity bound, not the check (the structure above is): 6 s, 10x the 0.6 s the solve takes on an idle core (34 s by the QR it replaced)
		TEST_CHECK(seconds <= 6, "1200 nodes, %zu rows: %.1f s", pairs.size(), seconds);
		#endif
	}
	return scope.Pass("the factored matrix is the Laplacian and the centres equal the SparseQR ones to 1e-9 on five graphs and the no-gauge call; the 1200-node solve is fast");
}

// The scale solve of GlobalScaleEstimator (conjugate gradients on the normal equations, started from the
// previous solve of the same estimator) gives the answer of a sparse Cholesky of the gauge-pinned system
// (no ridge) to 1e-9, on a system shaped like a view graph's pair scales (400 unknowns, ~16 equations
// each, weights sqrt of 15-300 points) and on a badly conditioned one (weights over 1e-2..1e2, two
// groups joined by one weak equation), always converged at the tolerance and not at the iteration
// bound. The start is the estimator's previous scales, kept relative to the gauge: a system perturbed
// a little is solved in at most half the iterations of a solve from nothing, and one with another
// gauge, more unknowns or fewer, or a robust round's change of a tenth of the equations has the
// same answer.
bool ScaleConjugateGradientsTest()
{
	TestScope scope("ScaleConjugateGradientsTest");
	std::mt19937 rng(5);
	std::uniform_real_distribution<double> uniform(0, 1);
	std::normal_distribution<double> noise(0, 0.02);
	constexpr uint32_t numUnknowns = 400, numGrown = 500, numShrunk = 300;
	std::vector<double> logScales(600); // the grown system (500) and the badly conditioned one (600) index it
	for (double& logScale : logScales)
		logScale = (uniform(rng) - 0.5);
	// the equations among the unknowns of [first, last): a ring and 15 random chords each, weight(): the weight of one
	const auto addEquations = [&](std::vector<ScalePair>& equations, uint32_t first, uint32_t last, const std::function<float()>& weight) {
		const auto add = [&](uint32_t a, uint32_t b) {
			if (a != b)
				equations.emplace_back(a, b, std::exp(logScales[b] - logScales[a] + noise(rng)), weight());
		};
		for (uint32_t i = first; i < last; ++i) {
			add(i, i + 1 < last ? i + 1 : first);
			for (unsigned c = 0; c < 15; ++c)
				add(i, first + (uint32_t)(uniform(rng) * (last - first)) % (last - first));
		}
	};
	const auto viewGraphWeight = [&]() { return (float)std::sqrt(15 + uniform(rng) * 285); };
	std::vector<ScalePair> pairs;
	addEquations(pairs, 0, numUnknowns, viewGraphWeight);
	// the Cholesky reference: the normal equations of the gauge-eliminated system, which are positive definite (no ridge)
	const auto reference = [&](const std::vector<ScalePair>& equations, uint32_t numNodes, uint32_t gauge, std::vector<double>& x) {
		const auto variable = [&](uint32_t node) { return (int)(node < gauge ? node : node - 1); };
		std::vector<Eigen::Triplet<double>> triplets;
		Eigen::VectorXd b = Eigen::VectorXd::Zero((Eigen::Index)equations.size());
		FOREACH(i, equations) {
			const ScalePair& pair = equations[i];
			if (pair.idxA != gauge)
				triplets.emplace_back((int)i, variable(pair.idxA), -(double)pair.weight);
			if (pair.idxB != gauge)
				triplets.emplace_back((int)i, variable(pair.idxB), (double)pair.weight);
			b(i) = (double)pair.weight * std::log((double)pair.scaleRatio);
		}
		Eigen::SparseMatrix<double> A((int)equations.size(), (int)numNodes - 1);
		A.setFromTriplets(triplets.begin(), triplets.end());
		const Eigen::SparseMatrix<double> normal = A.transpose() * A;
		Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(normal);
		Eigen::VectorXd solution = solver.solve(A.transpose() * b);
		// one step of refinement against the incidence matrix: the normal equations alone are off by their condition number times 1e-16
		solution += solver.solve(A.transpose() * (b - A * solution));
		x.assign(numNodes, 0.0);
		for (uint32_t i = 0; i < numNodes; ++i)
			if (i != gauge)
				x[i] = solution(variable(i));
	};
	// a solve of `equations` by `estimator`, its iterations; with `verify` it is also the Cholesky answer to 1e-9
	const auto solve = [&](const char* what, GlobalScaleEstimator& estimator, const std::vector<ScalePair>& equations, uint32_t numNodes, uint32_t gauge, bool verify, unsigned& iterations) -> bool {
		std::vector<REAL> scales;
		#ifdef _RELEASE
		const auto start = std::chrono::steady_clock::now();
		#endif
		TEST_CHECK(estimator.EstimateScales(equations, numNodes, gauge, scales), "%s: the solve failed", what);
		#ifdef _RELEASE // a wall-clock bound holds in a Release build only: a Debug or sanitizer build is slower by an order of magnitude
		const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
		#endif
		const GlobalScaleEstimator::Solve& stats = estimator.GetSolve();
		iterations = stats.iterations;
		TEST_CHECK(stats.converged, "%s: %u conjugate gradients iterations, not converged at the tolerance", what, stats.iterations);
		#ifdef _RELEASE
		// a sanity bound, not the check (the iterations are): 5 s, far over what the largest solve (500 unknowns) takes on an idle machine
		TEST_CHECK(seconds <= 5, "%s: the solve took %.1f s", what, seconds);
		#endif
		if (!verify)
			return true;
		std::vector<double> expected;
		reference(equations, numNodes, gauge, expected);
		double worst = 0;
		for (uint32_t i = 0; i < numNodes; ++i)
			worst = MAXF(worst, std::abs(std::log((double)scales[i]) - expected[i]));
		TEST_CHECK(worst < 1e-9, "%s: the log scales differ from the Cholesky ones by %g", what, worst);
		return true;
	};
	// the iterations of the same system solved from nothing (a fresh estimator)
	const auto cold = [&](const char* what, const std::vector<ScalePair>& equations, uint32_t numNodes, uint32_t gauge, unsigned& iterations) {
		GlobalScaleEstimator fresh;
		return solve(what, fresh, equations, numNodes, gauge, false, iterations);
	};
	// a system of the view graph's kind: from nothing the iterations stay near a few dozen
	GlobalScaleEstimator estimator;
	unsigned iterations, coldIterations;
	if (!solve("first solve", estimator, pairs, numUnknowns, 0, true, iterations))
		return false;
	TEST_CHECK(iterations <= 100, "first solve: %u iterations from nothing", iterations);
	// a perturbation of every scale ratio by 1e-10, no equation dropped: the previous scales are nearly the answer, and
	// the iterations fall with the logarithm of the distance, so half of them are saved only by a nearly exact start
	for (ScalePair& pair : pairs)
		pair.scaleRatio *= 1 + 1e-10 * uniform(rng);
	if (!solve("perturbed ratios", estimator, pairs, numUnknowns, 0, true, iterations) ||
		!cold("perturbed ratios from nothing", pairs, numUnknowns, 0, coldIterations))
		return false;
	TEST_CHECK(2 * iterations <= coldIterations, "the system with ratios perturbed by 1e-10 took %u iterations from the previous solve, %u from nothing", iterations, coldIterations);
	// every weight perturbed by 1%: fewer iterations from the previous solve than from nothing
	for (ScalePair& pair : pairs)
		pair.weight *= (float)(1 + 0.01 * uniform(rng));
	if (!solve("perturbed weights", estimator, pairs, numUnknowns, 0, true, iterations) ||
		!cold("perturbed weights from nothing", pairs, numUnknowns, 0, coldIterations))
		return false;
	TEST_CHECK(iterations < coldIterations, "the system with weights perturbed by 1%% took %u iterations from the previous solve, %u from nothing", iterations, coldIterations);
	// the same system with another gauge: the scales of the previous solve are taken relative to it
	if (!solve("another gauge", estimator, pairs, numUnknowns, 7, true, iterations) ||
		!cold("another gauge from nothing", pairs, numUnknowns, 7, coldIterations))
		return false;
	TEST_CHECK(2 * iterations <= coldIterations, "the other gauge took %u iterations from the previous solve, %u from nothing", iterations, coldIterations);
	// the robust average's next round: a tenth of the equations out, the others reweighted: the answer only
	{
		std::vector<ScalePair> active;
		FOREACH(i, pairs)
			if (i % 10 != 3)
				active.emplace_back(pairs[i].idxA, pairs[i].idxB, pairs[i].scaleRatio, pairs[i].weight * (1.f + 0.5f * (float)uniform(rng)));
		pairs.swap(active);
		if (!solve("a tenth of the equations dropped", estimator, pairs, numUnknowns, 0, true, iterations))
			return false;
	}
	// more unknowns than the previous solve had, then fewer: nothing of the previous scales is read out of its range
	{
		std::vector<ScalePair> grown = pairs;
		addEquations(grown, numUnknowns, numGrown, viewGraphWeight);
		for (uint32_t i = 0; i < 4; ++i)
			grown.emplace_back(i, numUnknowns + i, std::exp(logScales[numUnknowns + i] - logScales[i]), 10.f);
		if (!solve("more unknowns", estimator, grown, numGrown, 0, true, iterations))
			return false;
		std::vector<ScalePair> shrunk;
		addEquations(shrunk, 0, numShrunk, viewGraphWeight);
		if (!solve("fewer unknowns", estimator, shrunk, numShrunk, 0, true, iterations))
			return false;
	}
	// badly conditioned: weights over 1e-2..1e2 and two groups held together by one weak equation
	{
		std::vector<ScalePair> bad;
		const auto badWeight = [&]() { return (float)std::pow(10.0, uniform(rng) * 4 - 2); };
		addEquations(bad, 0, 300, badWeight);
		addEquations(bad, 300, 600, badWeight);
		bad.emplace_back(299, 300, std::exp(logScales[300] - logScales[299]), 3e-2f);
		GlobalScaleEstimator fresh;
		if (!solve("badly conditioned", fresh, bad, 600, 0, true, iterations))
			return false;
	}
	return scope.Pass("the conjugate gradients solve equals the Cholesky one to 1e-9, converged, and a near start saves iterations");
}

// The robust averaging (RobustAverage) ignores a wrong edge: five scales, and five translations, on a ring with chords, one chord
// wrong (the scale by a factor 3, the translation by 0.8): the estimates match the truth and the wrong chord's residual stands out.
bool RobustAveragingTest()
{
	TestScope scope("RobustAveragingTest");

	// five scales on a ring with one wrong chord
	{
		const REAL truth[5] = {1.0, 2.0, 0.5, 4.0, 1.5};
		std::vector<ScalePair> pairs;
		for (uint32_t i = 0; i < 5; ++i) {
			const uint32_t j = (i + 1) % 5;
			const uint32_t a = MINF(i, j), b = MAXF(i, j);
			pairs.push_back(ScalePair{a, b, truth[b] / truth[a], 10.f});
		}
		pairs.push_back(ScalePair{0u, 2u, truth[2] / truth[0], 10.f});
		pairs.push_back(ScalePair{1u, 3u, 3.0 * truth[3] / truth[1], 10.f}); // wrong by a factor 3
		pairs.push_back(ScalePair{2u, 4u, truth[4] / truth[2], 10.f});
		GlobalScaleEstimator estimator;
		std::vector<REAL> scales, residuals;
		const bool solved = RobustAverage(pairs, std::log(REAL(1.05)), 10,
			[&](const std::vector<ScalePair>& active, std::vector<REAL>& s) { return estimator.EstimateScales(active, 5, 0, s); },
			[&](size_t i, const std::vector<REAL>& s) { const ScalePair& p = pairs[i]; return std::log(p.scaleRatio) - (std::log(s[p.idxB]) - std::log(s[p.idxA])); },
			scales, residuals);
		TEST_CHECK(solved, "the scale solver failed");
		for (uint32_t i = 0; i < 5; ++i) {
			const REAL r = scales[i] / truth[i];
			TEST_CHECK(MAXF(r, REAL(1) / r) <= REAL(1.02), "scale %u off by %.3f", i, r);
		}
		TEST_CHECK(residuals[6] >= std::log(REAL(2.5)) && residuals[0] <= std::log(REAL(1.02)), "scale log residuals %.3f / %.3f", residuals[6], residuals[0]);
	}

	// five translations on a ring with one wrong chord
	{
		const Point3 truth[5] = {Point3(0,0,0), Point3(1,0,0), Point3(1,1,0), Point3(0,1,0), Point3(0.5,0.5,1)};
		std::vector<TranslationPair> pairs;
		for (uint32_t i = 0; i < 5; ++i) {
			const uint32_t j = (i + 1) % 5;
			const uint32_t a = MINF(i, j), b = MAXF(i, j);
			pairs.push_back(TranslationPair{a, b, truth[b] - truth[a], 10.f});
		}
		pairs.push_back(TranslationPair{0u, 2u, truth[2] - truth[0], 10.f});
		pairs.push_back(TranslationPair{1u, 3u, truth[3] - truth[1] + Point3(0.8, 0, 0), 10.f}); // wrong by 0.8
		GlobalTranslationEstimator estimator;
		std::vector<Point3> t;
		std::vector<REAL> residuals;
		const bool solved = RobustAverage(pairs, REAL(0.05), 10,
			[&](const std::vector<TranslationPair>& active, std::vector<Point3>& s) { return estimator.EstimateTranslations(active, 5, 0, s); },
			[&](size_t i, const std::vector<Point3>& s) { const TranslationPair& p = pairs[i]; return norm(s[p.idxB] - s[p.idxA] - p.relativeTranslation); },
			t, residuals);
		TEST_CHECK(solved, "the translation solver failed");
		for (uint32_t i = 0; i < 5; ++i)
			TEST_CHECK(norm(t[i] - truth[i]) <= REAL(0.02), "translation node %u off by %.3f", i, norm(t[i] - truth[i]));
		TEST_CHECK(residuals[6] >= REAL(0.7) && residuals[0] <= REAL(0.02), "translation residuals %.3f / %.3f", residuals[6], residuals[0]);
	}
	return scope.Pass("the wrong chord ignored by the scale and the translation averaging, its residual standing out");
}

} // namespace SFM
