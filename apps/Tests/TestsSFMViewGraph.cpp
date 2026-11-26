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
	const auto KeptPairs = [](const Scene& scene) {
		std::set<std::pair<IIndex,IIndex>> kept;
		for (const ImagePair& pair : scene.pairs)
			kept.emplace(pair.ID1, pair.ID2);
		return kept;
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
	const auto keptPairs = [](const Scene& scene) {
		std::set<std::pair<IIndex,IIndex>> kept;
		for (const ImagePair& pair : scene.pairs)
			kept.emplace(pair.ID1, pair.ID2);
		return kept;
	};
	constexpr float minPairWeight = 3.f;
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
		TEST_CHECK(!(removed != lookAlikes.size() || keptPairs(scene).size() != 8), "the scored octagon removed %u pairs, expected %zu look-alikes", removed, lookAlikes.size());
		for (const auto& ids : lookAlikes)
			TEST_CHECK(!(keptPairs(scene).count(ids)), "look-alike pair (%u,%u) kept", ids.first, ids.second);
	}
	{
		// unscored, the same graph starts in the symmetric mode and cuts the true ring instead
		Scene scene; buildOctagon(scene);
		const std::vector<float> scores(scene.pairs.size(), TripletScores::unscored);
		FilterPairsByContradiction(scene, scores, minPairWeight, 0.f, weightingCfg);
		bool allLookAlikes = true;
		for (const auto& ids : lookAlikes)
			allLookAlikes = allLookAlikes && !keptPairs(scene).count(ids);
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
		TEST_CHECK(!(removed != 1 || keptPairs(scene).count({0, 4}) || keptPairs(scene).size() != 8), "the capped wrong pair: %u pairs removed, expected only the wrong one", removed);
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
		const auto kept = keptPairs(scene);
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
	const auto build = [&](Scene& scene) {
		Scene sceneGT;
		GenerateTestScene(sceneGT, RingConfig(300), &scene);
		ComputePairsWeights(scene);
		for (ImagePair& pair : scene.pairs)
			pair.meanRayAngle = D2R(10.f);
	};
	const auto findPair = [](Scene& scene, IIndex a, IIndex b) -> ImagePair* {
		for (ImagePair& pair : scene.pairs)
			if (pair.ID1 == a && pair.ID2 == b)
				return &pair;
		return NULL;
	};
	const auto flip = [](ImagePair& pair) { pair.relativePose->C = -pair.relativePose->C; };
	const auto turn = [](ImagePair& pair) {
		pair.relativePose->C = RMatrix(Point3(D2R(REAL(70)), D2R(REAL(70)), D2R(REAL(70)))) * pair.relativePose->C;
	};
	const auto run = [&](Scene& scene, float bar, bool repair = false) {
		const std::vector<float> scores(scene.pairs.size(), TripletScores::unscored);
		return FilterPairsByContradiction(scene, scores, minPairWeight, bar, weightingCfg, repair);
	};
	{
		Scene scene; build(scene);
		const size_t numPairs = scene.pairs.size();
		ImagePair* pFlipped = findPair(scene, 3, 9);
		ImagePair* pTurned = findPair(scene, 2, 11);
		TEST_CHECK(!(pFlipped == NULL || pTurned == NULL), "the ring has no pair (3,9) or (2,11)");
		flip(*pFlipped);
		turn(*pTurned);
		const unsigned removed = run(scene, maxDirectionResidual);
		const auto kept = keptPairs(scene);
		TEST_CHECK(!(removed != 2 || kept.size() != numPairs - 2 || kept.count({3, 9}) || kept.count({2, 11})), "the planted flipped and turned pairs: %u removed of %zu, expected exactly those two", removed, numPairs);
		Scene clean; build(clean);
		TEST_CHECK(!(run(clean, maxDirectionResidual) != 0), "the ring with exact poses lost pairs");
	}
	{
		// two groups (images 0-7, 8-15) joined by the one pair (7,8), its baseline flipped
		Scene scene; build(scene);
		const IIndex half = 8;
		for (size_t idxPair = scene.pairs.size(); idxPair-- > 0; ) {
			const ImagePair& pair = scene.pairs[idxPair];
			if ((pair.ID1 < half) != (pair.ID2 < half) && !(pair.ID1 == 7 && pair.ID2 == 8))
				scene.pairs.RemoveAt(idxPair);
		}
		ImagePair* pBridge = findPair(scene, 7, 8);
		TEST_CHECK(!(pBridge == NULL), "the ring has no pair (7,8)");
		flip(*pBridge);
		const unsigned removed = run(scene, maxDirectionResidual);
		TEST_CHECK(!(removed != 0 || !keptPairs(scene).count({7, 8})), "the bridge was removed (%u pairs)", removed);
	}
	for (const bool shortRay : {true, false}) {
		Scene scene; build(scene);
		ImagePair* pPair = findPair(scene, 4, 12);
		TEST_CHECK(!(pPair == NULL), "the ring has no pair (4,12)");
		flip(*pPair);
		pPair->meanRayAngle = D2R(shortRay ? 1.5f : 10.f);
		const unsigned removed = run(scene, maxDirectionResidual);
		TEST_CHECK(!(removed != (shortRay ? 0u : 1u)), "the flipped pair at %s ray angle: %u removed", shortRay ? "1.5 degrees of" : "10 degrees of", removed);
	}
	{
		// bar 0: the rotation stage alone. One more pair is 30 degrees off in rotation: it goes at either
		// bar; the flipped and the turned baselines go only when the translation stage runs
		const std::set<std::pair<IIndex,IIndex>> rotated{{6, 14}}, flippedAndTurned{{3, 9}, {2, 11}};
		for (const float bar : {maxDirectionResidual, 0.f}) {
			Scene scene; build(scene);
			const size_t numPairs = scene.pairs.size();
			ImagePair* pRotated = findPair(scene, 6, 14);
			ImagePair* pFlipped = findPair(scene, 3, 9);
			ImagePair* pTurned = findPair(scene, 2, 11);
			TEST_CHECK(!(pRotated == NULL || pFlipped == NULL || pTurned == NULL), "the ring has no pair (6,14), (3,9) or (2,11)");
			pRotated->relativePose->R = RMatrix(Point3(0, D2R(REAL(30)), 0)) * pRotated->relativePose->R;
			flip(*pFlipped);
			turn(*pTurned);
			const unsigned removed = run(scene, bar);
			const auto kept = keptPairs(scene);
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
			flip(*findPair(scene, 3, 9));
			turn(*findPair(scene, 2, 11));
			ImagePair* pRotated = findPair(scene, 6, 14);
			pRotated->relativePose->R = RMatrix(Point3(0, D2R(REAL(30)), 0)) * pRotated->relativePose->R;
			ImagePair* pShifted = findPair(scene, 4, 12);
			flip(*pShifted);
			for (size_t i = 0; i + 1 < pShifted->matches.size(); ++i)
				pShifted->matches[i].trainIdx = pShifted->matches[i + 1].trainIdx;
		};
		Scene scene; build(scene);
		const size_t numPairs = scene.pairs.size();
		const std::set<std::pair<IIndex,IIndex>> planted{{3, 9}, {2, 11}, {6, 14}};
		std::map<std::pair<IIndex,IIndex>, Pose3D> exact;
		for (const auto& [a, b] : planted)
			exact[{a, b}] = findPair(scene, a, b)->relativePose.value();
		plant(scene);
		const unsigned removed = run(scene, maxDirectionResidual, true);
		const auto kept = keptPairs(scene);
		TEST_CHECK(!(removed != 1 || kept.size() != numPairs - 1 || kept.count({4, 12})), "the repair: %u pairs removed of %zu, expected only the pair with the shifted matches", removed, numPairs);
		for (const auto& [ids, pose] : exact) {
			const ImagePair& pair = *findPair(scene, ids.first, ids.second);
			const REAL rotationError = R2D(ACOS(MINF(MAXF(ComputeAngle(pair.relativePose->R, pose.R), REAL(-1)), REAL(1))));
			const REAL directionError = R2D(ACOS(MINF(MAXF(pair.relativePose->C.dot(pose.C) / (norm(pair.relativePose->C) * norm(pose.C)), REAL(-1)), REAL(1))));
			TEST_CHECK(!(!(rotationError < 0.1 && directionError < 0.1) || !pair.E.has_value() || !pair.F.has_value()), "the repaired pair (%u,%u) is %g degrees off in rotation and %g in direction", ids.first, ids.second, rotationError, directionError);
		}
		// the weights are final before the repair: a repaired pair forms no triangle, so its triangle share is 0 and every
		// other pair weighs what it weighs in the graph without the repaired pairs
		Scene without; build(without);
		plant(without);
		run(without, maxDirectionResidual);
		unsigned numReturned = 0;
		for (const ImagePair& pair : scene.pairs) {
			if (pair.returnedByRepair) {
				++numReturned;
				TEST_CHECK(!(pair.weightTriplet != 0.f), "the repaired pair (%u,%u) has the triangle share %g, expected 0", pair.ID1, pair.ID2, pair.weightTriplet);
				continue;
			}
			ImagePair* pOther = findPair(without, pair.ID1, pair.ID2);
			TEST_CHECK(!(pOther == NULL || !ISEQUAL(pair.GetCompositeWeight(), pOther->GetCompositeWeight())), "the pair (%u,%u) weighs %g with the repair and %g without the repaired pairs", pair.ID1, pair.ID2,
					pair.GetCompositeWeight(), pOther ? pOther->GetCompositeWeight() : -1.f);
		}
		TEST_CHECK(!(numReturned != planted.size()), "%u repaired pairs flagged, expected %zu", numReturned, planted.size());
	}
	{
		// a connected graph of 5 pairs, a loop and a chord among them, one baseline flipped: under
		// minScaleComponent pairs nothing is solved, so nothing is judged
		Scene scene; build(scene);
		const std::set<std::pair<IIndex,IIndex>> graph{{0, 1}, {1, 2}, {2, 3}, {0, 3}, {0, 2}};
		for (size_t idxPair = scene.pairs.size(); idxPair-- > 0; )
			if (!graph.count({scene.pairs[idxPair].ID1, scene.pairs[idxPair].ID2}))
				scene.pairs.RemoveAt(idxPair);
		ImagePair* pFlipped = findPair(scene, 0, 2);
		TEST_CHECK(!(pFlipped == NULL || scene.pairs.size() != graph.size()), "the ring has not the 5 pairs of the small graph");
		flip(*pFlipped);
		const unsigned removed = run(scene, maxDirectionResidual);
		TEST_CHECK(!(removed != 0 || scene.pairs.size() != graph.size()), "the 5-pair graph lost %u pairs", removed);
	}
	{
		// two components: images 0-8 and 9-15 as two groups of cameras with every pair inside a group and none
		// between them, so the rotation stage averages two components (the second pass of its loop) and the
		// pair-scale graph falls into two components, each of more than minScaleComponent pairs; one baseline
		// of the smaller group is flipped. Only that pair goes, and the cut reports both solves and every pair
		// judged: the flipped pair is judged by the smaller group's own solve.
		Scene scene; build(scene);
		const IIndex firstOfSecond = 9;
		for (size_t idxPair = scene.pairs.size(); idxPair-- > 0; )
			if ((scene.pairs[idxPair].ID1 < firstOfSecond) != (scene.pairs[idxPair].ID2 < firstOfSecond))
				scene.pairs.RemoveAt(idxPair);
		const size_t numPairs = scene.pairs.size();
		ImagePair* pFlipped = findPair(scene, 10, 13);
		TEST_CHECK(!(pFlipped == NULL || numPairs != 9 * 8 / 2 + 7 * 6 / 2), "the two groups hold %zu pairs, or have no pair (10,13)", numPairs);
		flip(*pFlipped);
		LogCapture log;
		const unsigned removed = run(scene, maxDirectionResidual);
		const auto kept = keptPairs(scene);
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

// The chain of an image sequence (SolveSequenceChain), on a walk with real features: 40 cameras 1 unit apart along X,
// each turned a few degrees about the vertical, looking sideways at points 5-15 units away, every pair up to 6 apart
// with its exact relative pose (unit baseline, so the scales are the solve's) and 10 degrees of ray angle, the true
// rotations, neighbours 3 apart: (1) one segment of all 40 images, whose centres after a scale and translation fit to
// the truth (the segment's frame has the rotations' orientation) are within 2% of the walk's length, the local step and
// every step of the path within 2% of the fitted step, and a chain scale exactly on the 114 neighbour pairs, each
// giving its link the length of its true baseline; (2) a name break between 19 and 20 makes two segments of 20, no pair
// across it a link; (4) a frame that stands still (image 20 at image 19's centre, their
// pair under the near-duplicate ray angle with a meaningless baseline) is no link, and the pairs 2 and 3 apart keep the
// one segment; (5) every pair under the near-duplicate ray angle places nothing; (6) a removed pair, a zero baseline, an
// unrotated image, overlap 0 and an empty scene, a neighbour pair of weight 0 (no link); (7) two components sharing an image: it belongs to the larger, and
// the links of the smaller that touch it keep no scale; (8) frames standing still: the walk ends with its last four
// cameras at one place, so the last image has no link, only still pairs (neighbour pairs under the near-duplicate ray
// angle, or of a baseline without length), and is placed at exactly the centre and path length of image 38, the
// nearest placed image, the links and the local step those of the scene without its pairs; a removed still pair
// places nothing, the pairs of the scene in another order place it alike; seven cameras at
// one place need a second pass; an image with still pairs to both sides takes the nearest placed image, the lower one
// on a tie.
bool SequenceChainTest()
{
	TestScope scope("SequenceChainTest");
	constexpr IIndex numImages = 40;
	constexpr IIndex overlap = 3, maxPairSpan = 6;
	constexpr REAL tolerance = REAL(0.02);
	const REAL walkLength = REAL(numImages - 1);
	// the walk: image i at (i,0,0) (image standStill at the centre of the one before it, the images after stillFrom at
	// its centre), turned 4 sin(0.9 i) degrees about the vertical; the pairs up to maxPairSpan apart, each with the
	// matches of the points both images see
	const auto buildWalk = [&](Scene& scene, std::vector<Point3>& trueCentres, IIndex standStill = NO_ID, IIndex stillFrom = NO_ID) {
		PinholeCamera* pCamera = new PinholeCamera(cv::Size(640, 480), 400.0, 400.0, 320.0, 240.0);
		pCamera->trustIntrinsics = true;
		scene.cameras.emplace_back(pCamera);
		trueCentres.clear();
		for (IIndex i = 0; i < numImages; ++i) {
			const Pose3D pose(RMatrix(Point3(0, D2R(REAL(4) * SIN(REAL(0.9) * i)), 0)),
				Point3(REAL(i == standStill ? i - 1 : MINF(i, stillFrom)), 0, 0));
			scene.images.emplace_back(i, "", pose, 0, pCamera);
			trueCentres.push_back(pose.C);
		}
		std::mt19937 rng(7);
		std::uniform_real_distribution<REAL> xDist(-6, 45), yDist(-2, 2), zDist(5, 15);
		std::vector<std::vector<uint32_t>> featureOf(numImages); // per image: the feature of each point, NO_ID where unseen
		for (unsigned p = 0; p < 2000; ++p) {
			const Point3 X(xDist(rng), yDist(rng), zDist(rng));
			for (Image& image : scene.images) {
				const auto [proj, valid] = image.ProjectPoint(X);
				if (!valid || proj.x < 10 || proj.y < 10 || proj.x > 630 || proj.y > 470) {
					featureOf[image.ID].push_back(NO_ID);
					continue;
				}
				featureOf[image.ID].push_back((uint32_t)image.keypoints.size());
				image.keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
			}
		}
		for (IIndex i = 0; i + 1 < numImages; ++i) {
			for (IIndex j = i + 1; j < numImages && j - i <= maxPairSpan; ++j) {
				ImagePair& pair = scene.pairs.emplace_back(i, j);
				FOREACH(p, featureOf[i])
					if (featureOf[i][p] != NO_ID && featureOf[j][p] != NO_ID)
						pair.matches.emplace_back(featureOf[i][p], featureOf[j][p]);
				Pose3D relativePose = scene.images[j] / scene.images[i];
				const bool still = trueCentres[i] == trueCentres[j];
				// a pair that stands still has no baseline: its two-view pose gives a meaningless unit direction
				relativePose.C = still ? CMatrix(0, 1, 0) : CMatrix(relativePose.C * (REAL(1) / norm(relativePose.C)));
				pair.relativePose = relativePose;
				pair.E = ImagePair::ComposeEssentialMatrix(relativePose);
				pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), scene.images[i].GetK(), scene.images[j].GetK());
				pair.meanRayAngle = still ? 0.f : D2R(10.f);
			}
		}
		ComputePairsWeights(scene);
	};
	const auto rotationsOf = [](const Scene& scene) {
		std::vector<Point3> rotations;
		for (const Image& image : scene.images)
			rotations.emplace_back(image.R.GetRotationAxisAngle());
		return rotations;
	};
	const auto noBreaks = [&]() { return SequenceNeighbours(std::vector<bool>(numImages - 1, false), overlap); };
	const auto breakAt = [&](IIndex k) { std::vector<bool> breaks(numImages - 1, false); breaks[k] = true; return SequenceNeighbours(breaks, overlap); };
	const auto acrossMiddle = [](const ImagePair& pair) { return pair.ID1 < 20 && pair.ID2 >= 20; };
	const auto numPlaced = [](const SequenceChain& chain) {
		return (unsigned)std::count_if(chain.segmentOf.begin(), chain.segmentOf.end(), [](uint32_t s) { return s != NO_ID; });
	};
	// the scale of the least squares fit s * c + t of a segment's centres to the true ones, and the largest distance after it
	const auto fit = [](const SequenceChain& chain, const std::vector<Point3>& trueCentres, uint32_t segment, REAL& scale) {
		Point3 meanChain(0, 0, 0), meanTrue(0, 0, 0);
		unsigned n = 0;
		FOREACH(i, chain.segmentOf)
			if (chain.segmentOf[i] == segment) {
				meanChain += chain.centres[i];
				meanTrue += trueCentres[i];
				++n;
			}
		meanChain *= REAL(1) / n;
		meanTrue *= REAL(1) / n;
		REAL num = 0, den = 0;
		FOREACH(i, chain.segmentOf)
			if (chain.segmentOf[i] == segment) {
				num += (chain.centres[i] - meanChain).dot(trueCentres[i] - meanTrue);
				den += normSq(chain.centres[i] - meanChain);
			}
		scale = num / den;
		REAL maxDistance = 0;
		FOREACH(i, chain.segmentOf)
			if (chain.segmentOf[i] == segment)
				maxDistance = MAXF(maxDistance, norm(scale * (chain.centres[i] - meanChain) - (trueCentres[i] - meanTrue)));
		return maxDistance;
	};

	// (1) the walk
	REAL drift = 0, fittedStep = 0, maxStepError = 0, maxLinkError = 0;
	{
		Scene scene;
		std::vector<Point3> trueCentres;
		buildWalk(scene, trueCentres);
		const std::vector<Point3> rotations = rotationsOf(scene);
		const SequenceChain chain = SolveSequenceChain(scene, rotations, noBreaks());
		TEST_CHECK(chain.numSegments == 1 && numPlaced(chain) == numImages, "the walk: %u segment(s) placing %u images, expected one of %u", chain.numSegments, numPlaced(chain), numImages);
		TEST_CHECK(chain.segmentOf.size() == numImages && chain.centres.size() == numImages && chain.pathLength.size() == numImages &&
			chain.localStep.size() == 1 && chain.linkScale.size() == scene.pairs.size(), "the walk: the vectors are not sized by image, segment and pair");
		REAL scale;
		drift = fit(chain, trueCentres, 0, scale);
		TEST_CHECK(scale > 0 && drift <= tolerance * walkLength, "the walk: a centre is %g from the truth after the fit, over %g of the walk's length %g", drift, tolerance, walkLength);
		fittedStep = REAL(1) / scale;
		TEST_CHECK(ABS(chain.localStep[0] - fittedStep) <= tolerance * fittedStep, "the walk: the local step is %g, the fitted step %g", chain.localStep[0], fittedStep);
		TEST_CHECK(chain.pathLength[0] == 0, "the walk: the path starts at %g", chain.pathLength[0]);
		for (IIndex i = 1; i < numImages; ++i)
			maxStepError = MAXF(maxStepError, ABS(chain.pathLength[i] - chain.pathLength[i - 1] - fittedStep) / fittedStep);
		TEST_CHECK(maxStepError <= tolerance, "the walk: a step of the path is %g of the fitted step off", maxStepError);
		unsigned numNeighbourPairs = 0;
		FOREACH(idxPair, scene.pairs) {
			const ImagePair& pair = scene.pairs[idxPair];
			const bool neighbours = pair.ID2 - pair.ID1 <= overlap;
			numNeighbourPairs += neighbours;
			TEST_CHECK(neighbours == (chain.linkScale[idxPair] > 0), "the walk: the pair (%u,%u) has the chain scale %g", pair.ID1, pair.ID2, chain.linkScale[idxPair]);
			if (!neighbours)
				continue;
			// the link's length in the segment's frame is its scale times its baseline's: the true one in fitted steps
			const REAL length = chain.linkScale[idxPair] * norm(pair.relativePose->C) * scale;
			maxLinkError = MAXF(maxLinkError, ABS(length - REAL(pair.ID2 - pair.ID1)) / REAL(pair.ID2 - pair.ID1));
		}
		TEST_CHECK(chain.numLinks == numNeighbourPairs && numNeighbourPairs == 114, "the walk: %u links, expected the %u neighbour pairs", chain.numLinks, numNeighbourPairs);
		TEST_CHECK(maxLinkError <= tolerance, "the walk: a link's length is %g of its true baseline off", maxLinkError);
	}

	// (2) a name break between 19 and 20, and (6) the pairs across 19|20 removed
	{
		Scene scene;
		std::vector<Point3> trueCentres;
		buildWalk(scene, trueCentres);
		const std::vector<Point3> rotations = rotationsOf(scene);
		const auto twoSegments = [&](const SequenceChain& chain) {
			if (chain.numSegments != 2 || numPlaced(chain) != numImages)
				return false;
			for (IIndex i = 0; i < numImages; ++i)
				if (chain.segmentOf[i] != (i < 20 ? 0u : 1u))
					return false;
			FOREACH(idxPair, scene.pairs)
				if (acrossMiddle(scene.pairs[idxPair]) && chain.linkScale[idxPair] != 0)
					return false;
			return chain.localStep.size() == 2 && chain.pathLength[20] == 0;
		};
		const SequenceChain broken = SolveSequenceChain(scene, rotations, breakAt(19));
		TEST_CHECK(twoSegments(broken), "the break at 19|20: %u segment(s) placing %u images, expected 0-19 and 20-39 with no link across", broken.numSegments, numPlaced(broken));
		TEST_CHECK(broken.numLinks == 2 * (19 + 18 + 17), "the break at 19|20: %u links", broken.numLinks);
		// a neighbour pair whose baseline has no length is no link
		FOREACH(idxPair, scene.pairs) {
			ImagePair& pair = scene.pairs[idxPair];
			if (pair.ID1 != 10 || pair.ID2 != 11)
				continue;
			const CMatrix C = pair.relativePose->C;
			pair.relativePose->C = CMatrix::ZERO;
			const SequenceChain flat = SolveSequenceChain(scene, rotations, noBreaks());
			pair.relativePose->C = C;
			TEST_CHECK(flat.numSegments == 1 && numPlaced(flat) == numImages && flat.linkScale[idxPair] == 0 && flat.numLinks == 114 - 1,
				"a zero baseline on (10,11): %u segment(s) placing %u images with %u links, its scale %g", flat.numSegments, numPlaced(flat), flat.numLinks, flat.linkScale[idxPair]);
			// a neighbour pair of no composite weight is no link (the centres solve would skip it)
			const float weightSpatial = pair.weightSpatial;
			pair.InvalidateWeight();
			ASSERT(!pair.HasValidWeight());
			const SequenceChain weightless = SolveSequenceChain(scene, rotations, noBreaks());
			pair.weightSpatial = weightSpatial;
			TEST_CHECK(weightless.numSegments == 1 && numPlaced(weightless) == numImages && weightless.linkScale[idxPair] == 0 && weightless.numLinks == 114 - 1,
				"a weight of 0 on (10,11): %u segment(s) placing %u images with %u links, its scale %g", weightless.numSegments, numPlaced(weightless),
				weightless.numLinks, weightless.linkScale[idxPair]);
		}
		std::vector<bool> removed(scene.pairs.size(), false);
		FOREACH(idxPair, scene.pairs)
			removed[idxPair] = acrossMiddle(scene.pairs[idxPair]);
		const SequenceChain cut = SolveSequenceChain(scene, rotations, noBreaks(), &removed);
		TEST_CHECK(twoSegments(cut), "the pairs across 19|20 removed: %u segment(s) placing %u images, expected 0-19 and 20-39", cut.numSegments, numPlaced(cut));
		// an unrotated image: its pairs are no links, it is not placed, the rest is one segment
		std::vector<Point3> unrotated(rotations);
		unrotated[numImages - 1] = Point3::INF;
		const SequenceChain partial = SolveSequenceChain(scene, unrotated, noBreaks());
		TEST_CHECK(partial.numSegments == 1 && numPlaced(partial) == numImages - 1 && partial.segmentOf[numImages - 1] == NO_ID &&
			partial.numLinks == 114 - 3, "an unrotated last image: %u segment(s) placing %u images with %u links", partial.numSegments, numPlaced(partial), partial.numLinks);
		// overlap 0: no neighbours, no link
		const SequenceChain alone = SolveSequenceChain(scene, rotations, SequenceNeighbours(std::vector<bool>(numImages - 1, false), 0));
		TEST_CHECK(alone.numSegments == 0 && numPlaced(alone) == 0 && alone.numLinks == 0 && alone.segmentOf.size() == numImages &&
			alone.linkScale.size() == scene.pairs.size(), "overlap 0: %u segment(s)", alone.numSegments);
	}

	// (4) a frame that stands still
	REAL stillDrift = 0;
	{
		Scene scene;
		std::vector<Point3> trueCentres;
		const IIndex standStill = 20;
		buildWalk(scene, trueCentres, standStill);
		const SequenceChain chain = SolveSequenceChain(scene, rotationsOf(scene), noBreaks());
		TEST_CHECK(chain.numSegments == 1 && numPlaced(chain) == numImages && chain.numLinks == 114 - 1, "the frame standing still: %u segment(s) placing %u images with %u links, expected one of %u with 113",
			chain.numSegments, numPlaced(chain), chain.numLinks, numImages);
		// its pairs 2 and 3 apart are links, which place it: none is placed by standing still
		TEST_CHECK(chain.numStill == 0, "the frame standing still: %u image(s) placed by standing still, expected its links to place it", chain.numStill);
		FOREACH(idxPair, scene.pairs) {
			const ImagePair& pair = scene.pairs[idxPair];
			const bool still = pair.ID1 == standStill - 1 && pair.ID2 == standStill;
			TEST_CHECK(!still || chain.linkScale[idxPair] == 0, "the pair standing still is a link of scale %g", chain.linkScale[idxPair]);
		}
		REAL scale;
		stillDrift = fit(chain, trueCentres, 0, scale);
		TEST_CHECK(scale > 0 && stillDrift <= tolerance * walkLength, "the frame standing still: a centre is %g from the truth after the fit", stillDrift);
	}

	// (5) every pair under the near-duplicate ray angle
	{
		Scene scene;
		std::vector<Point3> trueCentres;
		buildWalk(scene, trueCentres);
		for (ImagePair& pair : scene.pairs)
			pair.meanRayAngle = D2R(kNearDuplicateRayAngle * 0.75f);
		const SequenceChain chain = SolveSequenceChain(scene, rotationsOf(scene), noBreaks());
		// every neighbour pair is a still pair, but no segment is solved: an image standing still starts none
		TEST_CHECK(chain.numSegments == 0 && numPlaced(chain) == 0 && chain.numLinks == 0 && chain.numStill == 0 && chain.localStep.empty() &&
			std::all_of(chain.linkScale.begin(), chain.linkScale.end(), [](REAL s) { return s == 0; }), "every pair under the near-duplicate angle: %u segment(s) placing %u images, %u by standing still",
			chain.numSegments, numPlaced(chain), chain.numStill);
	}

	// (7) two components sharing an image: the pairs across image 15 are removed, and at image 15 the pairs
	// ending at it keep only the matches on the left half of its image, the pairs starting at it the right half, so no
	// scale equation joins the two sides; the larger component (images 15-39) owns image 15, and the links of the smaller
	// one ending at 15 keep no scale (it is in the frame of neither segment)
	{
		Scene scene;
		std::vector<Point3> trueCentres;
		buildWalk(scene, trueCentres);
		const IIndex shared = 15;
		const Image& sharedImage = scene.images[shared];
		std::vector<bool> removed(scene.pairs.size(), false);
		FOREACH(idxPair, scene.pairs) {
			ImagePair& pair = scene.pairs[idxPair];
			removed[idxPair] = pair.ID1 < shared && pair.ID2 > shared;
			if (pair.ID1 != shared && pair.ID2 != shared)
				continue;
			const bool left = pair.ID2 == shared;
			pair.matches.erase(std::remove_if(pair.matches.begin(), pair.matches.end(), [&](const DMatch& match) {
				return (sharedImage.keypoints[left ? match.trainIdx : match.queryIdx].pt.x < 320.f) != left;
			}), pair.matches.end());
		}
		const SequenceChain chain = SolveSequenceChain(scene, rotationsOf(scene), noBreaks(), &removed);
		bool segmentsRight = chain.numSegments == 2 && numPlaced(chain) == numImages;
		for (IIndex i = 0; segmentsRight && i < numImages; ++i)
			segmentsRight = chain.segmentOf[i] == (i < shared ? 0u : 1u);
		TEST_CHECK(segmentsRight, "two components sharing image %u: %u segment(s) placing %u images, expected 0-%u and %u-39", shared, chain.numSegments, numPlaced(chain), shared - 1, shared);
		unsigned numPositive = 0;
		FOREACH(idxPair, scene.pairs) {
			const ImagePair& pair = scene.pairs[idxPair];
			if (chain.linkScale[idxPair] > 0) {
				++numPositive;
				TEST_CHECK(chain.segmentOf[pair.ID1] == chain.segmentOf[pair.ID2], "two components sharing an image: the link (%u,%u) of scale %g joins the segments %u and %u",
					pair.ID1, pair.ID2, chain.linkScale[idxPair], chain.segmentOf[pair.ID1], chain.segmentOf[pair.ID2]);
			}
			if (pair.ID2 == shared && pair.ID1 < shared)
				TEST_CHECK(chain.linkScale[idxPair] == 0, "two components sharing an image: the link (%u,%u) of the smaller component keeps the scale %g", pair.ID1, pair.ID2, chain.linkScale[idxPair]);
		}
		TEST_CHECK(chain.numLinks == numPositive && numPositive == (14 + 13 + 12) + (24 + 23 + 22), "two components sharing an image: %u links, %u of positive scale, expected 108", chain.numLinks, numPositive);
	}

	// (8) frames standing still. The walk ends with its last four cameras (36-39) at one place: the pairs among them
	// stand still (no baseline, a ray angle of 0), so 36, 37 and 38 are placed by their links to the images before the
	// place and 39, whose three neighbour pairs all stand still, by none
	unsigned numLinksStill = 0;
	{
		const IIndex stillFrom = 36, last = numImages - 1;
		Scene scene;
		std::vector<Point3> trueCentres;
		buildWalk(scene, trueCentres, NO_ID, stillFrom);
		const std::vector<Point3> rotations = rotationsOf(scene);
		const auto ofLast = [&](const ImagePair& pair) { return pair.ID2 == last && pair.ID1 >= stillFrom; };
		String solveLine;
		SequenceChain chain;
		{
			LogCapture log;
			chain = SolveSequenceChain(scene, rotations, noBreaks());
			solveLine = log.text;
		}
		TEST_CHECK(chain.numSegments == 1 && numPlaced(chain) == numImages && chain.segmentOf[last] == 0 && chain.numStill == 1,
			"the walk ending still: %u segment(s) placing %u images, %u by standing still, image %u in the segment %u; expected one segment of %u, image %u placed by standing still",
			chain.numSegments, numPlaced(chain), chain.numStill, last, chain.segmentOf[last], numImages, last);
		TEST_CHECK(chain.centres[last] == chain.centres[last - 1] && chain.pathLength[last] == chain.pathLength[last - 1],
			"the walk ending still: image %u is %g from the centre of image %u, its path %g against %g; expected exactly its centre and path",
			last, norm(chain.centres[last] - chain.centres[last - 1]), last - 1, chain.pathLength[last], chain.pathLength[last - 1]);
		TEST_CHECK(solveLine.find("Sequence chain solve: 1 segment(s) placing 39 images by 108 of 108 links") != String::npos && solveLine.find("), 1 still image(s)") != String::npos,
			"the walk ending still: the solve logged \"%s\", expected 39 images placed by 108 links and 1 still image(s)", solveLine.c_str());
		unsigned numStillPairs = 0;
		FOREACH(idxPair, scene.pairs) {
			const ImagePair& pair = scene.pairs[idxPair];
			if (pair.ID1 < stillFrom)
				continue;
			++numStillPairs;
			TEST_CHECK(chain.linkScale[idxPair] == 0, "the walk ending still: the still pair (%u,%u) is a link of scale %g", pair.ID1, pair.ID2, chain.linkScale[idxPair]);
		}
		TEST_CHECK(numStillPairs == 6 && chain.numLinks == 114 - numStillPairs, "the walk ending still: %u links and %u still pairs, expected 108 and 6", chain.numLinks, numStillPairs);
		numLinksStill = chain.numLinks;
		// the same scene before the rule could act, without the pairs of the last image: the same links and local step
		{
			Scene before;
			std::vector<Point3> centresBefore;
			buildWalk(before, centresBefore, NO_ID, stillFrom);
			RFOREACH(idxPair, before.pairs)
				if (before.pairs[idxPair].ID2 == last)
					before.pairs.RemoveAtMove(idxPair);
			const SequenceChain chainBefore = SolveSequenceChain(before, rotations, noBreaks());
			TEST_CHECK(chainBefore.numSegments == 1 && numPlaced(chainBefore) == numImages - 1 && chainBefore.segmentOf[last] == NO_ID && chainBefore.numStill == 0,
				"the walk ending still, without the pairs of image %u: %u segment(s) placing %u images, %u by standing still", last, chainBefore.numSegments, numPlaced(chainBefore), chainBefore.numStill);
			TEST_CHECK(chain.numLinks == chainBefore.numLinks && chain.localStep[0] == chainBefore.localStep[0],
				"the walk ending still: %u links and a local step of %.17g, without the pairs of image %u %u and %.17g", chain.numLinks, chain.localStep[0], last, chainBefore.numLinks, chainBefore.localStep[0]);
			for (IIndex i = 0; i < last; ++i)
				TEST_CHECK(chain.centres[i] == chainBefore.centres[i] && chain.pathLength[i] == chainBefore.pathLength[i], "the walk ending still: image %u moves by %g with the pairs of image %u", i, norm(chain.centres[i] - chainBefore.centres[i]), last);
		}
		// the pairs of the scene in the opposite order: the same image placed by the same one
		{
			Scene reversed;
			std::vector<Point3> centresReversed;
			buildWalk(reversed, centresReversed, NO_ID, stillFrom);
			std::reverse(reversed.pairs.begin(), reversed.pairs.end());
			const SequenceChain chainReversed = SolveSequenceChain(reversed, rotations, noBreaks());
			TEST_CHECK(chainReversed.numStill == 1 && chainReversed.segmentOf[last] == 0 && chainReversed.centres[last] == chainReversed.centres[last - 1] &&
				chainReversed.pathLength[last] == chainReversed.pathLength[last - 1], "the walk ending still, its pairs reversed: %u image(s) placed by standing still, image %u %g from the centre of image %u",
				chainReversed.numStill, last, norm(chainReversed.centres[last] - chainReversed.centres[last - 1]), last - 1);
		}
		// which placed image an image standing still takes: with the pair (38,39) removed the next nearest, 37; with
		// (37,39) removed too, 36; with its three still pairs removed, none
		const auto solveWithout = [&](unsigned numRemoved) {
			std::vector<bool> removed(scene.pairs.size(), false);
			FOREACH(idxPair, scene.pairs)
				removed[idxPair] = ofLast(scene.pairs[idxPair]) && last - scene.pairs[idxPair].ID1 <= numRemoved;
			return SolveSequenceChain(scene, rotations, noBreaks(), &removed);
		};
		const SequenceChain removedOne = solveWithout(1);
		TEST_CHECK(removedOne.numStill == 1 && removedOne.centres[last] == removedOne.centres[last - 2] && removedOne.pathLength[last] == removedOne.pathLength[last - 2],
			"the still pair (38,39) removed: %u image(s) placed by standing still, image %u %g from the centre of image %u", removedOne.numStill, last, norm(removedOne.centres[last] - removedOne.centres[last - 2]), last - 2);
		const SequenceChain removedTwo = solveWithout(2);
		TEST_CHECK(removedTwo.numStill == 1 && removedTwo.centres[last] == removedTwo.centres[last - 3] && removedTwo.pathLength[last] == removedTwo.pathLength[last - 3],
			"the still pairs (38,39) and (37,39) removed: %u image(s) placed by standing still, image %u %g from the centre of image %u", removedTwo.numStill, last, norm(removedTwo.centres[last] - removedTwo.centres[last - 3]), last - 3);
		const SequenceChain removedAll = solveWithout(3);
		TEST_CHECK(removedAll.numStill == 0 && removedAll.segmentOf[last] == NO_ID && numPlaced(removedAll) == numImages - 1 && removedAll.numLinks == chain.numLinks,
			"the still pairs of image %u removed: %u image(s) placed by standing still, %u images placed", last, removedAll.numStill, numPlaced(removedAll));
		// an unrotated image is not placed by standing still either
		std::vector<Point3> unrotated(rotations);
		unrotated[last] = Point3::INF;
		const SequenceChain partial = SolveSequenceChain(scene, unrotated, noBreaks());
		TEST_CHECK(partial.numStill == 0 && partial.segmentOf[last] == NO_ID, "the walk ending still, image %u unrotated: %u image(s) placed by standing still", last, partial.numStill);
		// still by its baseline, not by its angle: the pairs of the last image at 10 degrees of ray angle and a
		// baseline of no length are still pairs as well
		for (ImagePair& pair : scene.pairs)
			if (ofLast(pair)) {
				pair.meanRayAngle = D2R(10.f);
				pair.relativePose->C = CMatrix::ZERO;
			}
		const SequenceChain flat = SolveSequenceChain(scene, rotations, noBreaks());
		TEST_CHECK(flat.numStill == 1 && flat.segmentOf[last] == 0 && flat.centres[last] == flat.centres[last - 1] && flat.numLinks == chain.numLinks,
			"the pairs of image %u with a baseline of no length: %u image(s) placed by standing still, %u links", last, flat.numStill, flat.numLinks);
	}
	// seven cameras (33-39) at one place: 33, 34 and 35 are placed by their links, 36, 37 and 38 by standing still
	// beside 35, the nearest placed image of each, and 39, whose neighbours 36-38 no link places, by the second pass
	{
		const IIndex stillFrom = 33;
		Scene scene;
		std::vector<Point3> trueCentres;
		buildWalk(scene, trueCentres, NO_ID, stillFrom);
		const SequenceChain chain = SolveSequenceChain(scene, rotationsOf(scene), noBreaks());
		TEST_CHECK(chain.numSegments == 1 && numPlaced(chain) == numImages && chain.numStill == 4, "seven cameras at one place: %u segment(s) placing %u images, %u by standing still, expected one of %u and the images 36-39",
			chain.numSegments, numPlaced(chain), chain.numStill, numImages);
		for (IIndex i = stillFrom + 3; i < numImages; ++i)
			TEST_CHECK(chain.segmentOf[i] == 0 && chain.centres[i] == chain.centres[stillFrom + 2] && chain.pathLength[i] == chain.pathLength[stillFrom + 2],
				"seven cameras at one place: image %u is %g from the centre of image %u", i, norm(chain.centres[i] - chain.centres[stillFrom + 2]), stillFrom + 2);
	}
	// an image whose neighbour pairs are all still pairs, in the middle of the walk (their ray angle under the
	// near-duplicate angle, the geometry untouched): it takes the nearest placed image, 19 of 19 and 21 (the lower on a
	// tie), 21 once (19,20) is removed, 18 of 18 and 22 once (20,21) is removed too
	{
		const IIndex middle = 20;
		Scene scene;
		std::vector<Point3> trueCentres;
		buildWalk(scene, trueCentres);
		const std::vector<Point3> rotations = rotationsOf(scene);
		for (ImagePair& pair : scene.pairs)
			if ((pair.ID1 == middle || pair.ID2 == middle) && pair.ID2 - pair.ID1 <= overlap)
				pair.meanRayAngle = D2R(kNearDuplicateRayAngle * 0.75f);
		const auto takenBy = [&](std::initializer_list<IIndex> removedOthers, unsigned& numStill, unsigned& numLinks) {
			std::vector<bool> removed(scene.pairs.size(), false);
			FOREACH(idxPair, scene.pairs) {
				const ImagePair& pair = scene.pairs[idxPair];
				for (const IIndex other : removedOthers)
					if ((pair.ID1 == middle && pair.ID2 == other) || (pair.ID2 == middle && pair.ID1 == other))
						removed[idxPair] = true;
			}
			const SequenceChain chain = SolveSequenceChain(scene, rotations, noBreaks(), &removed);
			numStill = chain.numStill;
			numLinks = chain.numLinks;
			if (chain.numSegments != 1 || numPlaced(chain) != numImages)
				return NO_ID;
			IIndex taken = NO_ID;
			for (IIndex i = 0; i < numImages; ++i)
				if (i != middle && chain.centres[i] == chain.centres[middle] && chain.pathLength[i] == chain.pathLength[middle]) {
					if (taken != NO_ID)
						return NO_ID;
					taken = i;
				}
			return taken;
		};
		unsigned numStill, numLinks;
		IIndex taken = takenBy({}, numStill, numLinks);
		TEST_CHECK(taken == middle - 1 && numStill == 1 && numLinks == 114 - 6, "an image with still pairs to both sides: placed at image %u, %u by standing still, %u links; expected image %u, the lower of the two nearest, and 108 links",
			taken, numStill, numLinks, middle - 1);
		taken = takenBy({middle - 1}, numStill, numLinks);
		TEST_CHECK(taken == middle + 1 && numStill == 1, "an image with still pairs to both sides, (19,20) removed: placed at image %u, expected %u", taken, middle + 1);
		taken = takenBy({middle - 1, middle + 1}, numStill, numLinks);
		TEST_CHECK(taken == middle - 2 && numStill == 1, "an image with still pairs to both sides, (19,20) and (20,21) removed: placed at image %u, expected %u", taken, middle - 2);
	}

	// (6) an empty scene
	{
		Scene scene;
		const SequenceChain chain = SolveSequenceChain(scene, {}, SequenceNeighbours(std::vector<bool>(), overlap));
		TEST_CHECK(chain.numSegments == 0 && chain.numStill == 0 && chain.segmentOf.empty() && chain.linkScale.empty(), "the empty scene: %u segment(s)", chain.numSegments);
	}
	return scope.Pass("the walk of %u images: one segment, drift %.4g after the fit (%.3g%% of the walk), fitted step %.4g, step error %.3g%%, "
		"link length error %.3g%%; standing still: drift %.4g; the break, the near-duplicates, the edge cases, the shared image; "
		"the walk ending still: image 39 placed at image 38 over %u links, seven cameras at one place, the nearest placed image",
		numImages, drift, REAL(100) * drift / walkLength, fittedStep, REAL(100) * maxStepError, REAL(100) * maxLinkError, stillDrift, numLinksStill);
}

// The chain of an image sequence in the contradiction cut (FilterPairsByContradiction given the neighbours): the veto of
// the pairs that contradict the chain, and the protection of its links.
// (1) The rule by values (ChainVetoes): the tolerance is the larger of 8 local steps and 0.2 of the path, and only a
//     pair shorter than a third of the chain distance is vetoed.
// On a periodic corridor with real features: a pattern of points repeated every 10 units along X, 60 cameras looking
// sideways at it, 30 walking out along +X one unit apart and 30 walking back 2 units further from the points, so
// (i, 59 - i) are true revisits; every pair up to 6 apart and every revisit with its exact relative pose (the baseline at
// an arbitrary length per pair, so the lengths are the scales'), and the planted look-alikes (i, i + 11) and (i, i + 21)
// of the way out, whose relative pose is the true one of (i, i + 1) and whose matches join each point's feature in i
// to the feature of its copy one (two) periods on: the right rotation and a baseline of one step between frames 11
// and 21 apart. Neighbours 3 apart, the cut unscored:
// (2) every look-alike is vetoed and nothing else is removed, the chain one segment of 174 links, every other pair
//     judged; the lines of the judged pairs give each look-alike the length of one step against a chain distance of 11
//     or 21, and each true pair its true length; a neighbour pair that is no link (its weight 0) is judged like any
//     other pair;
// (3) without the neighbours the cut is the call as it was (the same kept pairs as the six-argument call, the stats
//     zero), and keeps the look-alikes: their direction agrees with the centres;
// (4) with the repair on, a vetoed look-alike stays removed (its matches would pass the repair: they agree on the
//     chain's direction) while a true pair with a flipped baseline comes back;
// (5) a break at the turn (29|30) gives two segments: the look-alikes of each (those of the way back planted too) are
//     still vetoed, and the pairs across the turn, the revisits among them, are neither judged nor removed;
// (6) the veto abstains: 5 cameras (9 links, under the pairs a scale component needs), a bar of 0 (no translation
//     stage), overlap 0, and scenes of no image and of one give the result of the cut without the neighbours;
// (7) a segment does not span two components of the rotation averaging (their rotations share no frame);
// (8) the view-graph filter of a matched scene reads the option: Scene::Reconstruct with imageSequence set vetoes the
//     look-alikes of the saved corridor, the names of its images making the break at the turn, and keeps them without.
// On the arc of ContradictionCutTest (6): (9) a neighbour pair with a flipped baseline that its matches support (new
// features of its second image, where a camera at the mirrored centre sees the points) is removed without the
// neighbours and kept with them, the one protected link; flipped with its true matches it triangulates nothing, has
// no scale, is no link and is removed with the neighbours as without.
// On the corridor again, the pairs and frames without a baseline:
// (10) pairs under the near-duplicate ray angle are judged: the near-duplicate look-alikes (i, i + 10) of the way out
//     (the second frame a quarter of a step past the period, or before it: the same picture a little to the side,
//     as new features of the second image) are all vetoed, a quarter of a step long on a chain distance of 10, and
//     stay without the neighbours; true pairs with the ray angle set under the angle are judged at their true length
//     and kept; with the second frame at the period exactly no match has a ray angle, the pair has no scale
//     equation and is not judged;
// (11) with the repair on, the vetoed near-duplicate look-alikes stay removed and make no spot: a look-alike two
//     periods on that the rotation stage removed, which they would vouch for as one spot, is not returned;
// (12) the capture ends standing still (the last six cameras at one place): the three that no link places stand
//     where the last placed one does, their look-alikes are vetoed like the others and no true pair is removed.
bool SequenceVetoTest()
{
	TestScope scope("SequenceVetoTest");
	// (1) the rule by values, the local step 1
	TEST_CHECK(!ChainVetoes(10, 1, 11, 1, 100), "a discrepancy of 10 on a path of 100 (tolerance 20) vetoes");
	TEST_CHECK(ChainVetoes(10, 1, 11, 1, 20), "a discrepancy of 10 on a path of 20 (tolerance 8) does not veto");
	TEST_CHECK(!ChainVetoes(7.9, 1, 9, 1, 9), "a discrepancy of 7.9 (tolerance 8) vetoes");
	TEST_CHECK(!ChainVetoes(30, 12, 30, 1, 30), "a pair of length 12 on a chain distance of 30 (not short: 12 >= 30 / 3) is vetoed");
	TEST_CHECK(ChainVetoes(30, 9, 30, 1, 30), "a pair of length 9 on a chain distance of 30, 30 off, is not vetoed");
	TEST_CHECK(!ChainVetoes(1, 0.5, 0, 1, 0), "a pair whose two images share one centre (a chain distance of 0) is vetoed");

	using PairIds = std::pair<IIndex,IIndex>;
	using PairSet = std::set<PairIds>;
	constexpr IIndex numOut = 30, numImages = 2 * numOut;
	constexpr IIndex overlap = 3, maxPairSpan = 6;
	constexpr int period = 10, firstCopy = -2, numCopies = 7;
	constexpr unsigned numPatternPoints = 400;
	constexpr float minPairWeight = 3.f;
	constexpr REAL tolerance = REAL(0.02);
	const PairsWeightingConfig weightingCfg;
	const float maxDirectionResidual = ReconstructionConfig().maxPairDirectionResidual;
	using STATE = Scene::Status::STATE;

	// the corridor: image i of the way out at (i,0,0), of the way back at (59-i,0,-2), turned 4 sin(2 pi x / 10) degrees
	// about the vertical (the period of the pattern, so the frames a period apart see the same picture); the images
	// after stillFrom stand at its place (the capture ends standing still)
	const auto placeOf = [&](IIndex i, IIndex stillFrom) {
		const IIndex k = MINF(i, stillFrom);
		return k < numOut ? Point3(REAL(k), 0, 0) : Point3(REAL(numImages - 1 - k), 0, -2);
	};
	const auto positionOf = [&](IIndex i) { return placeOf(i, NO_ID); };
	// the pairs of a corridor, its look-alikes and near-duplicate look-alikes among them, and what a pair added later
	// reads: the points, and per image the feature of each (NO_ID where unseen)
	struct Corridor { PairSet pairs, lookAlikes, nearDuplicates; std::vector<Point3> points; std::vector<std::vector<uint32_t>> featureOf; };
	const auto buildCorridor = [&](Scene& scene, IIndex numCameras, bool lookAlikesBack, IIndex stillFrom = NO_ID) {
		Corridor corridor;
		PinholeCamera* pCamera = new PinholeCamera(cv::Size(640, 480), 400.0, 400.0, 320.0, 240.0);
		pCamera->trustIntrinsics = true;
		scene.cameras.emplace_back(pCamera);
		for (IIndex i = 0; i < numCameras; ++i) {
			const Point3 C = placeOf(i, stillFrom);
			const Pose3D pose(RMatrix(Point3(0, D2R(REAL(4) * SIN(REAL(2) * M_PI * C.x / period)), 0)), C);
			scene.images.emplace_back(i, String::FormatString("%s_%04u.jpg", i < numOut ? "out" : "back", i), pose, 0, pCamera);
		}
		// the features: point k of the pattern in copy c is the point c * numPatternPoints + k
		std::mt19937 rng(11);
		std::uniform_real_distribution<REAL> xDist(0, period), yDist(-2, 2), zDist(5, 15);
		std::vector<Point3> pattern;
		for (unsigned k = 0; k < numPatternPoints; ++k)
			pattern.emplace_back(xDist(rng), yDist(rng), zDist(rng));
		std::vector<std::vector<uint32_t>>& featureOf = corridor.featureOf; // per image: the feature of each point, NO_ID where unseen
		featureOf.resize(numCameras);
		for (int copy = 0; copy < numCopies; ++copy) {
			for (const Point3& X0 : pattern) {
				const Point3 X(X0.x + REAL(period * (firstCopy + copy)), X0.y, X0.z);
				corridor.points.push_back(X);
				for (Image& image : scene.images) {
					const auto [proj, valid] = image.ProjectPoint(X);
					if (!valid || proj.x < 10 || proj.y < 10 || proj.x > 630 || proj.y > 470) {
						featureOf[image.ID].push_back(NO_ID);
						continue;
					}
					featureOf[image.ID].push_back((uint32_t)image.keypoints.size());
					image.keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
				}
			}
		}
		// a pair (a,b) with the true relative pose of (a,standIn), where b stands copyShift periods from standIn, its
		// matches joining each point in a to its copy copyShift periods on in b: a true pair is its own stand-in
		const auto addPair = [&](IIndex a, IIndex b, IIndex standIn) {
			const int copyShift = ROUND2INT((placeOf(b, stillFrom).x - placeOf(standIn, stillFrom).x) / period);
			ImagePair& pair = scene.pairs.emplace_back(a, b);
			for (int copy = MAXF(0, -copyShift); copy < MINF(numCopies, numCopies - copyShift); ++copy)
				for (unsigned k = 0; k < numPatternPoints; ++k) {
					const uint32_t feature1 = featureOf[a][copy * numPatternPoints + k];
					const uint32_t feature2 = featureOf[b][(copy + copyShift) * numPatternPoints + k];
					if (feature1 != NO_ID && feature2 != NO_ID)
						pair.matches.emplace_back(feature1, feature2);
				}
			Pose3D relativePose = scene.images[standIn] / scene.images[a];
			// a pair standing still has no baseline: its two-view pose gives a meaningless direction, and no ray angle
			const bool still = placeOf(standIn, stillFrom) == placeOf(a, stillFrom);
			const REAL length = REAL(0.5) + REAL(0.25) * ((3 * a + 5 * b) % 7);
			relativePose.C = still ? CMatrix(0, length, 0) : CMatrix(relativePose.C * (length / norm(relativePose.C)));
			pair.relativePose = relativePose;
			pair.E = ImagePair::ComposeEssentialMatrix(relativePose);
			pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), scene.images[a].GetK(), scene.images[b].GetK());
			pair.meanRayAngle = still ? 0.f : D2R(10.f);
			corridor.pairs.emplace(a, b);
		};
		for (IIndex i = 0; i + 1 < numCameras; ++i) {
			for (IIndex j = i + 1; j < numCameras; ++j) {
				const bool sameWay = j < numOut || (lookAlikesBack && i >= numOut);
				// a look-alike: j stands one or two periods from the image after i (frames 11 and 21 apart while the
				// cameras walk), so it sees what that image sees
				const REAL shift = ABS(placeOf(j, stillFrom).x - placeOf(i + 1, stillFrom).x);
				if (j - i <= maxPairSpan || i + j == numImages - 1) {
					addPair(i, j, j);
				} else if (sameWay && (shift == REAL(period) || shift == REAL(2 * period))) {
					addPair(i, j, i + 1);
					corridor.lookAlikes.emplace(i, j);
				}
			}
		}
		if (numCameras > 0) // the weights of a scene without images are not ComputePairsWeights' to compute
			ComputePairsWeights(scene);
		return corridor;
	};
	// The pair (a,b) as a camera of b's rotation standing `offset` along X from a matches a: every point a sees joined
	// to a new feature of b, where that camera sees it, with that camera's relative pose (the baseline at an arbitrary
	// length) and the ray angle its matches measure. With b a whole number of periods from a, where both have one
	// rotation and see the same picture, it is the pair of b standing `offset` past those periods: a look-alike with
	// a baseline of `offset`. At offset 0 the two see the same picture exactly: no parallax, a pose with a meaningless
	// direction and no ray angle. The weights are left to be computed.
	const auto addBeside = [&](Scene& scene, const Corridor& corridor, IIndex a, IIndex b, REAL offset) -> ImagePair& {
		const Image& image1 = scene.images[a];
		Image& image2 = scene.images[b];
		const Image beside(image2.ID, "", Pose3D(image2.R, Point3(image1.C) + Point3(offset, 0, 0)), image2.cameraID, image2.pCamera);
		ImagePair& pair = scene.pairs.emplace_back(a, b);
		FOREACH(p, corridor.points) {
			if (corridor.featureOf[a][p] == NO_ID)
				continue;
			const auto [proj, valid] = beside.ProjectPoint(corridor.points[p]);
			if (!valid || proj.x < 10 || proj.y < 10 || proj.x > 630 || proj.y > 470)
				continue;
			pair.matches.emplace_back(corridor.featureOf[a][p], (uint32_t)image2.keypoints.size());
			image2.keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
		}
		Pose3D relativePose = beside / image1;
		const REAL length = REAL(0.5) + REAL(0.25) * ((3 * a + 5 * b) % 7);
		relativePose.C = offset == 0 ? CMatrix(length, 0, 0) : CMatrix(relativePose.C * (length / norm(relativePose.C)));
		pair.relativePose = relativePose;
		pair.E = ImagePair::ComposeEssentialMatrix(relativePose);
		pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), image1.GetK(), image2.GetK());
		pair.meanRayAngle = offset == 0 ? 0.f : pair.ComputeMeanRayAngle(image1, image2);
		return pair;
	};
	// the near-duplicate look-alikes: every pair (i, i + 10) of the way out, the second image standing `offset` past the
	// period, before it for an odd i
	const auto plantNearDuplicates = [&](Scene& scene, Corridor& corridor, REAL offset) {
		for (IIndex i = 0; i + period < numOut; ++i) {
			addBeside(scene, corridor, i, i + period, i % 2 == 0 ? offset : -offset);
			corridor.pairs.emplace(i, i + period);
			corridor.nearDuplicates.emplace(i, i + period);
		}
		ComputePairsWeights(scene);
	};
	const auto keptPairs = [](const Scene& scene) {
		PairSet kept;
		for (const ImagePair& pair : scene.pairs)
			kept.emplace(pair.ID1, pair.ID2);
		return kept;
	};
	const auto without = [](const PairSet& pairs, const PairSet& removed) {
		PairSet left;
		std::set_difference(pairs.begin(), pairs.end(), removed.begin(), removed.end(), std::inserter(left, left.end()));
		return left;
	};
	const auto findPair = [](Scene& scene, IIndex a, IIndex b) -> ImagePair* {
		for (ImagePair& pair : scene.pairs)
			if (pair.ID1 == a && pair.ID2 == b)
				return &pair;
		return NULL;
	};
	const auto noBreaks = [](IIndex numCameras, IIndex numNeighbours) {
		return SequenceNeighbours(std::vector<bool>(numCameras > 0 ? numCameras - 1 : 0, false), numNeighbours);
	};
	const auto run = [&](Scene& scene, float bar, bool repair, const SequenceNeighbours* neighbours, SequenceFilterStats* stats) {
		const std::vector<float> scores(scene.pairs.size(), TripletScores::unscored);
		return FilterPairsByContradiction(scene, scores, minPairWeight, bar, weightingCfg, repair, neighbours, stats);
	};
	const auto isZero = [](const SequenceFilterStats& stats) {
		return stats.numSegments == 0 && stats.numLinks == 0 && stats.numJudged == 0 && stats.numVetoed == 0 && stats.numProtected == 0;
	};
	// the lines the cut logs per judged pair and per placed image, as the measurements read them; the images are named
	// by their index
	struct JudgedPair { IIndex a, b; double length, chain, path, discrepancy; int veto; };
	struct PlacedImage { unsigned segment; Point3 centre; };
	const auto indexOf = [](const char* stem) { const char* number = strrchr(stem, '_'); return number == NULL ? NO_ID : (IIndex)atoi(number + 1); };
	const auto parseLog = [&](const String& text, std::vector<JudgedPair>& judged, std::map<IIndex,PlacedImage>& placed) {
		char stem1[64], stem2[64];
		for (size_t pos = text.find("Chain pair: "); pos != String::npos; pos = text.find("Chain pair: ", pos + 1)) {
			JudgedPair pair;
			if (sscanf(text.c_str() + pos, "Chain pair: %63s %63s length %lf chain %lf path %lf discrepancy %lf veto %d",
				stem1, stem2, &pair.length, &pair.chain, &pair.path, &pair.discrepancy, &pair.veto) != 7)
				return false;
			pair.a = indexOf(stem1);
			pair.b = indexOf(stem2);
			judged.push_back(pair);
		}
		for (size_t pos = text.find("Chain centre: "); pos != String::npos; pos = text.find("Chain centre: ", pos + 1)) {
			PlacedImage image;
			double x, y, z;
			if (sscanf(text.c_str() + pos, "Chain centre: %63s %u %lf %lf %lf", stem1, &image.segment, &x, &y, &z) != 5)
				return false;
			image.centre = Point3(x, y, z);
			placed[indexOf(stem1)] = image;
		}
		return true;
	};

	// (2) the corridor with the neighbours: the look-alikes vetoed, nothing else removed
	const unsigned numChainLinks = (numImages - 1) + (numImages - 2) + (numImages - 3);
	struct Range { double lo{std::numeric_limits<double>::max()}, hi{std::numeric_limits<double>::lowest()}; void Add(double v) { lo = MINF(lo, v); hi = MAXF(hi, v); } };
	Range lengthOne, lengthTwo, discrepancyOne, discrepancyTwo, trueLengthError, trueDiscrepancy, revisitDiscrepancy;
	unsigned numLookAlikes = 0, numJudgedPairs = 0;
	{
		Scene scene;
		const Corridor corridor = buildCorridor(scene, numImages, false);
		numLookAlikes = (unsigned)corridor.lookAlikes.size();
		numJudgedPairs = (unsigned)corridor.pairs.size() - numChainLinks;
		TEST_CHECK(numLookAlikes == 19 + 9 && corridor.pairs.size() == numChainLinks + (56 + 55 + 54) + 27 + numLookAlikes, "the corridor holds %zu pairs, %u of them look-alikes", corridor.pairs.size(), numLookAlikes);
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		LogCapture log;
		const unsigned removed = run(scene, maxDirectionResidual, false, &neighbours, &stats);
		TEST_CHECK(stats.numSegments == 1 && stats.numLinks == numChainLinks, "the corridor: %u segment(s) over %u links, expected one over %u", stats.numSegments, stats.numLinks, numChainLinks);
		TEST_CHECK(stats.numVetoed == numLookAlikes && stats.numJudged == numJudgedPairs && stats.numProtected == 0,
			"the corridor: %u of %u judged pairs vetoed, %u links protected, expected the %u look-alikes of %u and none", stats.numVetoed, stats.numJudged, stats.numProtected, numLookAlikes, numJudgedPairs);
		TEST_CHECK(removed == numLookAlikes && keptPairs(scene) == without(corridor.pairs, corridor.lookAlikes), "the corridor: %u pairs removed, expected exactly the %u look-alikes", removed, numLookAlikes);
		const String report = String::FormatString("Sequence chain: 1 segment(s) over %u links; veto removed %u of %u judged pairs; 0 chain pairs protected", numChainLinks, numLookAlikes, numJudgedPairs);
		TEST_CHECK(log.Contains(report), "the corridor: the cut did not report \"%s\"", report.c_str());
		// the lines of the judged pairs and of the centres: in steps of the chain's frame, a look-alike is one step
		// long on a chain distance of 11 or 21, a true pair has its true length and no discrepancy
		std::vector<JudgedPair> judged;
		std::map<IIndex,PlacedImage> placed;
		TEST_CHECK(parseLog(log.text, judged, placed), "the corridor: a line of a judged pair or of a centre does not read");
		TEST_CHECK(judged.size() == numJudgedPairs && placed.size() == numImages, "the corridor: %zu lines of judged pairs and %zu of centres, expected %u and %u", judged.size(), placed.size(), numJudgedPairs, numImages);
		const REAL step = norm(placed[numOut - 1].centre - placed[0].centre) / REAL(numOut - 1);
		TEST_CHECK(step > 0, "the corridor: the centres of the way out do not move");
		for (const JudgedPair& pair : judged) {
			TEST_CHECK(pair.a < pair.b && corridor.pairs.count({pair.a, pair.b}), "the corridor: the line of a judged pair names (%u,%u)", pair.a, pair.b);
			const REAL trueDistance = norm(positionOf(pair.b) - positionOf(pair.a));
			const REAL chainDistance = norm(placed[pair.b].centre - placed[pair.a].centre);
			const REAL length = pair.length / step, discrepancy = pair.discrepancy / step;
			TEST_CHECK(ABS(pair.chain - chainDistance) <= tolerance * step && ABS(pair.chain / step - trueDistance) <= tolerance * trueDistance,
				"the corridor: the pair (%u,%u) has the chain distance %g steps, its centres %g, the truth %g", pair.a, pair.b, pair.chain / step, chainDistance / step, trueDistance);
			if (corridor.lookAlikes.count({pair.a, pair.b})) {
				const bool onePeriod = pair.b - pair.a == period + 1;
				(onePeriod ? lengthOne : lengthTwo).Add(length);
				(onePeriod ? discrepancyOne : discrepancyTwo).Add(discrepancy);
				TEST_CHECK(pair.veto == 1 && ABS(length - 1) <= tolerance && ABS(discrepancy - (trueDistance - 1)) <= tolerance * trueDistance &&
					ABS(pair.path / step - trueDistance) <= tolerance * trueDistance,
					"the corridor: the look-alike (%u,%u) has veto %d, length %g steps, discrepancy %g and path %g, expected 1, 1, %g and %g", pair.a, pair.b, pair.veto, length, discrepancy, pair.path / step, trueDistance - 1, trueDistance);
				continue;
			}
			const bool revisit = pair.a + pair.b == numImages - 1;
			trueLengthError.Add(ABS(length - trueDistance) / trueDistance);
			(revisit ? revisitDiscrepancy : trueDiscrepancy).Add(discrepancy);
			// the path runs through the turn, a step of 2
			const REAL truePath = REAL(pair.b - pair.a) + (pair.a < numOut && pair.b >= numOut ? 1 : 0);
			TEST_CHECK(pair.veto == 0 && ABS(length - trueDistance) <= tolerance * trueDistance && discrepancy <= tolerance * MAXF(trueDistance, REAL(1)) &&
				ABS(pair.path / step - truePath) <= tolerance * truePath,
				"the corridor: the true pair (%u,%u) has veto %d, length %g steps, discrepancy %g and path %g, expected 0, %g, 0 and %g", pair.a, pair.b, pair.veto, length, discrepancy, pair.path / step, trueDistance, truePath);
		}
	}
	{
		// a neighbour pair that is no link (its weight 0): it is judged like any other pair, and kept, being true
		Scene scene;
		const Corridor corridor = buildCorridor(scene, numImages, false);
		findPair(scene, 10, 12)->InvalidateWeight();
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, false, &neighbours, &stats);
		TEST_CHECK(stats.numSegments == 1 && stats.numLinks == numChainLinks - 1 && stats.numVetoed == numLookAlikes && stats.numJudged == numJudgedPairs + 1 &&
			removed == numLookAlikes && keptPairs(scene) == without(corridor.pairs, corridor.lookAlikes),
			"a neighbour pair of weight 0: %u segment(s) over %u links, %u of %u judged pairs vetoed, %u removed", stats.numSegments, stats.numLinks, stats.numVetoed, stats.numJudged, removed);
	}

	// (3) without the neighbours: the call as it was
	PairSet keptWithout;
	{
		Scene asBefore, scene;
		const Corridor corridor = buildCorridor(asBefore, numImages, false);
		buildCorridor(scene, numImages, false);
		const std::vector<float> scores(asBefore.pairs.size(), TripletScores::unscored);
		const unsigned removedBefore = FilterPairsByContradiction(asBefore, scores, minPairWeight, maxDirectionResidual, weightingCfg, false);
		SequenceFilterStats stats;
		stats.numSegments = stats.numLinks = stats.numJudged = stats.numVetoed = stats.numProtected = 7;
		const unsigned removed = run(scene, maxDirectionResidual, false, nullptr, &stats);
		keptWithout = keptPairs(scene);
		TEST_CHECK(removed == removedBefore && keptWithout == keptPairs(asBefore), "without the neighbours: %u pairs removed, the six-argument call removes %u", removed, removedBefore);
		TEST_CHECK(isZero(stats), "without the neighbours: the stats are not zero (%u segments, %u links, %u judged, %u vetoed, %u protected)", stats.numSegments, stats.numLinks, stats.numJudged, stats.numVetoed, stats.numProtected);
		for (const PairIds& ids : corridor.lookAlikes)
			TEST_CHECK(keptWithout.count(ids), "premise not met: without the neighbours the cut removes the look-alike (%u,%u)", ids.first, ids.second);
	}

	// (4) the repair: a vetoed pair is not offered to it, a pair the translation stage removed is
	{
		Scene scene;
		const Corridor corridor = buildCorridor(scene, numImages, false);
		ImagePair* pFlipped = findPair(scene, 12, 17);
		pFlipped->relativePose->C = -pFlipped->relativePose->C;
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, true, &neighbours, &stats);
		TEST_CHECK(stats.numVetoed == numLookAlikes && removed == numLookAlikes && keptPairs(scene) == without(corridor.pairs, corridor.lookAlikes),
			"the repair: %u pairs vetoed and %u removed, expected the %u look-alikes to stay removed", stats.numVetoed, removed, numLookAlikes);
		unsigned numReturned = 0;
		for (const ImagePair& pair : scene.pairs)
			numReturned += pair.returnedByRepair;
		pFlipped = findPair(scene, 12, 17);
		TEST_CHECK(numReturned == 1 && pFlipped != NULL && pFlipped->returnedByRepair, "the repair: %u pairs returned, expected the flipped (12,17) alone", numReturned);
	}

	// (5) a break at the turn
	{
		Scene scene;
		const Corridor corridor = buildCorridor(scene, numImages, true);
		std::vector<bool> breaks(numImages - 1, false);
		breaks[numOut - 1] = true;
		const SequenceNeighbours neighbours(breaks, overlap);
		unsigned numInside = 0; // the pairs inside a way that are not neighbours
		for (const PairIds& ids : corridor.pairs)
			numInside += (ids.first < numOut) == (ids.second < numOut) && ids.second - ids.first > overlap;
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, false, &neighbours, &stats);
		TEST_CHECK(corridor.lookAlikes.size() == 2 * numLookAlikes && numInside == 2 * (26 + 25 + 24 + numLookAlikes), "the corridor with the look-alikes of the way back holds %zu look-alikes and %u pairs inside a way", corridor.lookAlikes.size(), numInside);
		TEST_CHECK(stats.numSegments == 2 && stats.numLinks == 2 * ((numOut - 1) + (numOut - 2) + (numOut - 3)), "the break at the turn: %u segment(s) over %u links", stats.numSegments, stats.numLinks);
		TEST_CHECK(stats.numVetoed == 2 * numLookAlikes && stats.numJudged == numInside, "the break at the turn: %u of %u judged pairs vetoed, expected the %u look-alikes of the %u pairs inside a way",
			stats.numVetoed, stats.numJudged, 2 * numLookAlikes, numInside);
		TEST_CHECK(removed == 2 * numLookAlikes && keptPairs(scene) == without(corridor.pairs, corridor.lookAlikes), "the break at the turn: %u pairs removed, expected exactly the %u look-alikes", removed, 2 * numLookAlikes);
	}

	// (6) the veto abstains
	{
		// 5 cameras: 9 links, under the pairs a scale component needs
		Scene scene, plain;
		const Corridor corridor = buildCorridor(scene, 5, false);
		buildCorridor(plain, 5, false);
		const SequenceNeighbours neighbours = noBreaks(5, overlap);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, false, &neighbours, &stats);
		const unsigned removedPlain = run(plain, maxDirectionResidual, false, nullptr, nullptr);
		TEST_CHECK(corridor.pairs.size() == 10 && isZero(stats) && removed == removedPlain && keptPairs(scene) == keptPairs(plain),
			"5 cameras: %u segment(s), %u judged, %u vetoed, %u removed (%u without the neighbours)", stats.numSegments, stats.numJudged, stats.numVetoed, removed, removedPlain);
	}
	{
		// a bar of 0: no translation stage, so no chain and no veto
		Scene scene, plain;
		buildCorridor(scene, numImages, false);
		buildCorridor(plain, numImages, false);
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, 0.f, false, &neighbours, &stats);
		const unsigned removedPlain = run(plain, 0.f, false, nullptr, nullptr);
		TEST_CHECK(isZero(stats) && removed == removedPlain && keptPairs(scene) == keptPairs(plain), "the bar 0: %u segment(s), %u judged, %u vetoed, %u removed (%u without the neighbours)",
			stats.numSegments, stats.numJudged, stats.numVetoed, removed, removedPlain);
	}
	{
		// overlap 0: no neighbours, no chain
		Scene scene;
		buildCorridor(scene, numImages, false);
		const SequenceNeighbours neighbours = noBreaks(numImages, 0);
		SequenceFilterStats stats;
		run(scene, maxDirectionResidual, false, &neighbours, &stats);
		TEST_CHECK(isZero(stats) && keptPairs(scene) == keptWithout, "overlap 0: %u segment(s), %u judged, %u vetoed, or other pairs kept than without the neighbours", stats.numSegments, stats.numJudged, stats.numVetoed);
	}
	for (const IIndex numCameras : {IIndex(0), IIndex(1)}) {
		// no image, and one: nothing to judge
		Scene scene;
		buildCorridor(scene, numCameras, false);
		const SequenceNeighbours neighbours(scene, overlap, MatchConfig().sequenceMaxGap);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, false, &neighbours, &stats);
		TEST_CHECK(isZero(stats) && removed == 0 && scene.pairs.empty(), "%u image(s): %u segment(s), %u judged, %u removed", numCameras, stats.numSegments, stats.numJudged, removed);
	}

	// (7) two components of the rotation averaging: the pairs joining the images 0-14 to the rest score 0, so the
	// averaging solves the two sides apart, each in its own frame; the chain's segments stay inside a side, so the veto
	// judges only pairs whose rotations share a frame: the four look-alikes inside each side
	{
		Scene scene;
		const Corridor corridor = buildCorridor(scene, numImages, false);
		const IIndex firstOfSecond = 15;
		std::vector<float> scores(scene.pairs.size(), TripletScores::unscored);
		PairSet inside;
		FOREACH(idxPair, scene.pairs) {
			const ImagePair& pair = scene.pairs[idxPair];
			if ((pair.ID1 < firstOfSecond) != (pair.ID2 < firstOfSecond))
				scores[idxPair] = 0.f;
			else if (corridor.lookAlikes.count({pair.ID1, pair.ID2}))
				inside.emplace(pair.ID1, pair.ID2);
		}
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		const unsigned removed = FilterPairsByContradiction(scene, scores, minPairWeight, maxDirectionResidual, weightingCfg, false, &neighbours, &stats);
		TEST_CHECK(inside.size() == 8 && stats.numSegments == 2 && stats.numVetoed == inside.size() && removed == inside.size() && keptPairs(scene) == without(corridor.pairs, inside),
			"two rotation components: %u segment(s), %u vetoed and %u removed, expected two segments and the %zu look-alikes inside a side", stats.numSegments, stats.numVetoed, removed, inside.size());
	}

	// (8) the view-graph filter of a matched scene: the option off keeps the look-alikes, the option on vetoes them, the
	// neighbours read from the scene's names (out_0000..out_0029, back_0030..back_0059: a break at the turn)
	unsigned numKeptLookAlikes = 0, numLostTruePairs = 0;
	{
		const ScopedTempDir tmpDir(_T("SequenceVetoTest"));
		TEST_CHECK(tmpDir.IsValid(), "cannot create the temporary folder");
		Corridor corridor;
		const String matchedPath = tmpDir(_T("matched.sfm"));
		{
			Scene matched;
			corridor = buildCorridor(matched, numImages, true);
			matched.status.nState.set(STATE::FEATURES_EXTRACTED);
			matched.status.nState.set(STATE::MATCHED);
			TEST_CHECK(matched.Save(matchedPath), "cannot save the matched corridor '%s'", matchedPath.c_str());
		}
		ReconstructionConfig cfg;
		cfg.matchImagesOnly = true;
		cfg.matchCfg.lensFitEnabled = false; // the corridor's camera is exact: no lens to fit
		Scene plain(2);
		TEST_CHECK(plain.Reconstruct(matchedPath, cfg) && plain.status.nState.isSet(STATE::PAIRS_FILTERED), "the match-images-only run of the matched corridor failed");
		const PairSet keptPlain = keptPairs(plain);
		for (const PairIds& ids : corridor.lookAlikes)
			numKeptLookAlikes += (unsigned)keptPlain.count(ids);
		numLostTruePairs = (unsigned)(corridor.pairs.size() - corridor.lookAlikes.size()) - ((unsigned)keptPlain.size() - numKeptLookAlikes);
		TEST_CHECK(numKeptLookAlikes != 0, "premise not met: without the option the view-graph filter keeps no look-alike");
		cfg.matchCfg.imageSequence = true;
		LogCapture log;
		Scene sequence(2);
		TEST_CHECK(sequence.Reconstruct(matchedPath, cfg) && sequence.status.nState.isSet(STATE::PAIRS_FILTERED), "the match-images-only run of the matched corridor as a sequence failed");
		const String report = String::FormatString("Sequence chain: 2 segment(s) over %u links; veto removed %u of", 2 * ((numOut - 1) + (numOut - 2) + (numOut - 3)), 2 * numLookAlikes);
		TEST_CHECK(log.Contains(report), "the view-graph filter with the option did not report \"%s\"", report.c_str());
		TEST_CHECK(keptPairs(sequence) == without(corridor.pairs, corridor.lookAlikes), "the view-graph filter with the option keeps %u pairs, expected the %zu pairs that are no look-alikes",
			(unsigned)sequence.pairs.size(), corridor.pairs.size() - corridor.lookAlikes.size());
	}

	// (9) the protection, on the arc of ContradictionCutTest (6): the neighbour pair (7,8) with its baseline flipped
	const auto buildArc = [&](Scene& scene, bool supported) {
		Scene sceneGT;
		GenerateTestScene(sceneGT, RingConfig(300), &scene);
		for (ImagePair& pair : scene.pairs)
			pair.meanRayAngle = D2R(10.f);
		ImagePair& pair = *findPair(scene, 7, 8);
		pair.relativePose->C = -pair.relativePose->C;
		if (supported) {
			// matches that support the flipped pose: every point image 7 sees, joined to a new feature of image 8 where
			// a camera of image 8's rotation standing at its centre mirrored about image 7's sees the point
			const Image& image1 = scene.images[7];
			Image& image2 = scene.images[8];
			const Image mirrored(image2.ID, "", Pose3D(image2.R, Point3(image1.C) * REAL(2) - Point3(image2.C)), image2.cameraID, image2.pCamera);
			pair.matches.clear();
			for (const Track& track : sceneGT.tracks) {
				const auto [proj, valid] = mirrored.ProjectPoint(track.position);
				if (!valid || !Image8U::isInside(proj, mirrored.GetSize()))
					continue;
				for (const Observation& obs : track.observations) {
					if (obs.imageID != image1.ID)
						continue;
					pair.matches.emplace_back(obs.featureID, (uint32_t)image2.keypoints.size());
					image2.keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
				}
			}
			pair.E = ImagePair::ComposeEssentialMatrix(pair.relativePose.value());
			pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), image1.GetK(), image2.GetK());
		}
		ComputePairsWeights(scene);
		return (unsigned)pair.matches.size();
	};
	const SequenceNeighbours arcNeighbours = noBreaks(16, overlap);
	unsigned numSupporting = 0;
	{
		Scene scene, plain;
		numSupporting = buildArc(scene, true);
		buildArc(plain, true);
		const size_t numPairs = scene.pairs.size();
		const unsigned removedPlain = run(plain, maxDirectionResidual, false, nullptr, nullptr);
		TEST_CHECK(removedPlain == 1 && plain.pairs.size() == numPairs - 1 && !keptPairs(plain).count({7, 8}), "the flipped neighbour pair (7,8) without the neighbours: %u pairs removed, expected it alone", removedPlain);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, false, &arcNeighbours, &stats);
		TEST_CHECK(stats.numSegments == 1 && stats.numLinks == 15 + 14 + 13 && stats.numVetoed == 0, "the arc: %u segment(s) over %u links, %u pairs vetoed", stats.numSegments, stats.numLinks, stats.numVetoed);
		TEST_CHECK(removed == 0 && scene.pairs.size() == numPairs && stats.numProtected == 1, "the flipped neighbour pair (7,8) with the neighbours: %u pairs removed and %u links protected, expected it kept as the one protected link",
			removed, stats.numProtected);
	}
	{
		// flipped with its true matches, the pair triangulates nothing: no scale, so no link, and nothing protects it
		Scene scene;
		buildArc(scene, false);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, false, &arcNeighbours, &stats);
		TEST_CHECK(stats.numSegments == 1 && stats.numLinks == 15 + 14 + 13 - 1 && stats.numProtected == 0 && removed == 1 && !keptPairs(scene).count({7, 8}),
			"the neighbour pair (7,8) flipped against its matches: %u links, %u protected, %u removed, expected no link, none and it alone", stats.numLinks, stats.numProtected, removed);
	}

	// (10) pairs under the near-duplicate ray angle are judged like any other: the near-duplicate look-alikes (i, i + 10)
	// of the way out, a quarter of a step long, are vetoed with the look-alikes, and the true pairs whose ray angle is
	// set under the angle (every pair more than 3 apart of each third image, revisits among them; their geometry
	// untouched) are judged at their true length and kept
	constexpr REAL nearOffset = REAL(0.25);
	const auto lowAngle = [&](const ImagePair& pair) { return pair.ID2 - pair.ID1 > overlap && pair.ID1 % 3 == 0; };
	Range nearAngle, nearLength, nearDiscrepancy, lowLengthError;
	unsigned numNear = 0, numLow = 0;
	{
		Scene scene, plain;
		Corridor corridor = buildCorridor(scene, numImages, false);
		plantNearDuplicates(scene, corridor, nearOffset);
		numNear = (unsigned)corridor.nearDuplicates.size();
		PairSet low;
		for (ImagePair& pair : scene.pairs) {
			const PairIds ids(pair.ID1, pair.ID2);
			if (corridor.nearDuplicates.count(ids))
				nearAngle.Add(R2D(pair.meanRayAngle));
			else if (!corridor.lookAlikes.count(ids) && lowAngle(pair)) {
				pair.meanRayAngle = D2R(kNearDuplicateRayAngle * 0.75f);
				low.insert(ids);
			}
		}
		ComputePairsWeights(scene);
		numLow = (unsigned)low.size();
		unsigned numLowRevisits = 0;
		for (const PairIds& ids : low)
			numLowRevisits += ids.first + ids.second == numImages - 1;
		TEST_CHECK(numNear == numOut - period && nearAngle.lo > 0.5 && nearAngle.hi < kNearDuplicateRayAngle,
			"premise not met: %u near-duplicate look-alikes with a ray angle of %g-%g degrees, expected %u between 0.5 and %g", numNear, nearAngle.lo, nearAngle.hi, numOut - period, kNearDuplicateRayAngle);
		TEST_CHECK(numLow == 65 && numLowRevisits == 10, "premise not met: %u true pairs under the near-duplicate angle, %u of them revisits", numLow, numLowRevisits);
		// without the neighbours the near-duplicate look-alikes stay, as the look-alikes do
		{
			Corridor corridorPlain = buildCorridor(plain, numImages, false);
			plantNearDuplicates(plain, corridorPlain, nearOffset);
			run(plain, maxDirectionResidual, false, nullptr, nullptr);
			const PairSet keptPlain = keptPairs(plain);
			for (const PairIds& ids : corridor.nearDuplicates)
				TEST_CHECK(keptPlain.count(ids), "premise not met: without the neighbours the cut removes the near-duplicate look-alike (%u,%u)", ids.first, ids.second);
		}
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		LogCapture log;
		const unsigned removed = run(scene, maxDirectionResidual, false, &neighbours, &stats);
		TEST_CHECK(stats.numSegments == 1 && stats.numLinks == numChainLinks && stats.numProtected == 0, "the near-duplicates: %u segment(s) over %u links, %u protected, expected one over %u", stats.numSegments, stats.numLinks, stats.numProtected, numChainLinks);
		TEST_CHECK(stats.numVetoed == numLookAlikes + numNear && stats.numJudged == numJudgedPairs + numNear,
			"the near-duplicates: %u of %u judged pairs vetoed, expected the %u look-alikes and the %u near-duplicate look-alikes of %u", stats.numVetoed, stats.numJudged, numLookAlikes, numNear, numJudgedPairs + numNear);
		TEST_CHECK(removed == numLookAlikes + numNear && keptPairs(scene) == without(without(corridor.pairs, corridor.lookAlikes), corridor.nearDuplicates),
			"the near-duplicates: %u pairs removed, expected exactly the %u look-alikes and the %u near-duplicate look-alikes", removed, numLookAlikes, numNear);
		std::vector<JudgedPair> judged;
		std::map<IIndex,PlacedImage> placed;
		TEST_CHECK(parseLog(log.text, judged, placed) && judged.size() == numJudgedPairs + numNear && placed.size() == numImages, "the near-duplicates: %zu lines of judged pairs and %zu of centres", judged.size(), placed.size());
		const REAL step = norm(placed[numOut - 1].centre - placed[0].centre) / REAL(numOut - 1);
		unsigned numNearLines = 0, numLowLines = 0;
		for (const JudgedPair& pair : judged) {
			const PairIds ids(pair.a, pair.b);
			const REAL trueDistance = norm(positionOf(pair.b) - positionOf(pair.a));
			const REAL length = pair.length / step, discrepancy = pair.discrepancy / step;
			if (corridor.nearDuplicates.count(ids)) {
				// the second image stands a quarter of a step past the period, or before it: its offset is that far from the chain's
				const REAL off = trueDistance - (pair.a % 2 == 0 ? nearOffset : -nearOffset);
				++numNearLines;
				nearLength.Add(length);
				nearDiscrepancy.Add(discrepancy);
				TEST_CHECK(pair.veto == 1 && ABS(length - nearOffset) <= tolerance * nearOffset && ABS(pair.chain / step - trueDistance) <= tolerance * trueDistance && ABS(discrepancy - off) <= tolerance * trueDistance,
					"the near-duplicate look-alike (%u,%u) has veto %d, length %g steps, chain distance %g and discrepancy %g, expected 1, %g, %g and %g", pair.a, pair.b, pair.veto, length, pair.chain / step, discrepancy, nearOffset, trueDistance, off);
			} else if (low.count(ids)) {
				++numLowLines;
				lowLengthError.Add(ABS(length - trueDistance) / trueDistance);
				TEST_CHECK(pair.veto == 0 && ABS(length - trueDistance) <= tolerance * trueDistance && discrepancy <= tolerance * MAXF(trueDistance, REAL(1)),
					"the true pair (%u,%u) under the near-duplicate angle has veto %d, length %g steps and discrepancy %g, expected 0, %g and 0", pair.a, pair.b, pair.veto, length, discrepancy, trueDistance);
			} else
				TEST_CHECK(pair.veto == (corridor.lookAlikes.count(ids) ? 1 : 0), "the near-duplicates: the pair (%u,%u) has veto %d", pair.a, pair.b, pair.veto);
		}
		TEST_CHECK(numNearLines == numNear && numLowLines == numLow, "the near-duplicates: %u lines of near-duplicate look-alikes and %u of true pairs under the angle, expected %u and %u", numNearLines, numLowLines, numNear, numLow);
	}
	{
		// the same picture exactly (the second image at the period): no match has a ray angle, so the pair has no scale
		// equation, is not judged and stays
		Scene scene;
		Corridor corridor = buildCorridor(scene, numImages, false);
		plantNearDuplicates(scene, corridor, 0);
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, false, &neighbours, &stats);
		TEST_CHECK(stats.numVetoed == numLookAlikes && stats.numJudged == numJudgedPairs && removed == numLookAlikes && keptPairs(scene) == without(corridor.pairs, corridor.lookAlikes),
			"the same picture exactly: %u of %u judged pairs vetoed and %u removed, expected the %u look-alikes of %u, the %zu pairs without parallax not judged", stats.numVetoed, stats.numJudged, removed, numLookAlikes, numJudgedPairs, corridor.nearDuplicates.size());
	}

	// (11) the repair and the vetoed near-duplicate look-alikes: they stay removed, and they make no spot. The pair (5,25),
	// a look-alike two periods on whose rotation is 30 degrees off, is removed by the rotation stage, so the repair is
	// offered it: its matches agree on the direction of the centres, no image holds a kept pair with both its images,
	// and only the vetoed (5,15) and (15,25) would make its two images one spot
	{
		Scene scene;
		Corridor corridor = buildCorridor(scene, numImages, false);
		plantNearDuplicates(scene, corridor, nearOffset);
		const PairIds turnedIds(5, 5 + 2 * period);
		ImagePair& turned = addBeside(scene, corridor, turnedIds.first, turnedIds.second, 1);
		const Matrix3x3 turnedR = RMatrix(Point3(0, D2R(REAL(30)), 0)) * turned.relativePose->R;
		turned.relativePose->R = RMatrix(turnedR);
		const float turnedAngle = R2D(turned.meanRayAngle);
		corridor.pairs.insert(turnedIds);
		ImagePair* pFlipped = findPair(scene, 12, 17);
		pFlipped->relativePose->C = -pFlipped->relativePose->C;
		ComputePairsWeights(scene);
		TEST_CHECK(turnedAngle >= kNearDuplicateRayAngle, "premise not met: the turned pair (%u,%u) has a ray angle of %g degrees, itself a near-duplicate", turnedIds.first, turnedIds.second, turnedAngle);
		std::set<IIndex> partners[2]; // the images holding a true pair with the first and with the second image of the turned pair
		for (const PairIds& ids : corridor.pairs) {
			if (ids == turnedIds || corridor.lookAlikes.count(ids) || corridor.nearDuplicates.count(ids))
				continue;
			for (int k = 0; k < 2; ++k) {
				const IIndex image = k == 0 ? turnedIds.first : turnedIds.second;
				if (ids.first == image || ids.second == image)
					partners[k].insert(ids.first == image ? ids.second : ids.first);
			}
		}
		std::vector<IIndex> common;
		std::set_intersection(partners[0].begin(), partners[0].end(), partners[1].begin(), partners[1].end(), std::back_inserter(common));
		TEST_CHECK(common.empty() && !partners[0].empty() && !partners[1].empty(), "premise not met: %zu images hold a true pair with both images of the turned pair (%u,%u)", common.size(), turnedIds.first, turnedIds.second);
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		const unsigned removed = run(scene, maxDirectionResidual, true, &neighbours, &stats);
		PairSet gone(corridor.lookAlikes);
		gone.insert(corridor.nearDuplicates.begin(), corridor.nearDuplicates.end());
		gone.insert(turnedIds);
		unsigned numReturned = 0;
		for (const ImagePair& pair : scene.pairs)
			numReturned += pair.returnedByRepair;
		pFlipped = findPair(scene, 12, 17);
		TEST_CHECK(stats.numVetoed == numLookAlikes + numNear && findPair(scene, turnedIds.first, turnedIds.second) == NULL,
			"the repair and the near-duplicates: %u pairs vetoed, expected %u, and the turned pair (%u,%u) %s", stats.numVetoed, numLookAlikes + numNear, turnedIds.first, turnedIds.second,
			findPair(scene, turnedIds.first, turnedIds.second) == NULL ? "stays removed" : "came back: the vetoed near-duplicate look-alikes made its two images one spot");
		TEST_CHECK(numReturned == 1 && pFlipped != NULL && pFlipped->returnedByRepair && removed == gone.size() && keptPairs(scene) == without(corridor.pairs, gone),
			"the repair and the near-duplicates: %u pairs returned and %u removed, expected the flipped (12,17) alone returned and the %zu look-alikes, near-duplicate look-alikes and the turned pair removed", numReturned, removed, gone.size());
	}

	// (12) the capture ends standing still: the last six cameras (54-59) at one place, the pairs among them without
	// baseline. 54, 55 and 56 are placed by their links to the frames before the place, and 57, 58 and 59, whose
	// neighbour pairs all stand still, where 56 stands; their pairs are then judged like any other: the look-alikes
	// of a frame standing still are vetoed, and no true pair is, the pairs among the six neither
	unsigned numStillLookAlikes = 0, numStillJudged = 0;
	{
		const IIndex stillFrom = 54, firstStill = stillFrom + 3;
		Scene scene;
		const Corridor corridor = buildCorridor(scene, numImages, true, stillFrom);
		unsigned numStillPairs = 0, numStillNeighbours = 0;
		for (const PairIds& ids : corridor.pairs) {
			numStillPairs += ids.first >= stillFrom;
			numStillNeighbours += ids.first >= stillFrom && ids.second - ids.first <= overlap;
		}
		for (const PairIds& ids : corridor.lookAlikes)
			numStillLookAlikes += ids.second >= firstStill;
		TEST_CHECK(corridor.lookAlikes.size() == 2 * numLookAlikes && numStillLookAlikes == 6 && numStillPairs == 15 && numStillNeighbours == 12,
			"the corridor ending still holds %zu look-alikes, %u of them of the images %u-59, and %u pairs standing still, %u of them neighbours", corridor.lookAlikes.size(), numStillLookAlikes, firstStill, numStillPairs, numStillNeighbours);
		const SequenceNeighbours neighbours = noBreaks(numImages, overlap);
		SequenceFilterStats stats;
		LogCapture log;
		const unsigned removed = run(scene, maxDirectionResidual, false, &neighbours, &stats);
		TEST_CHECK(stats.numSegments == 1 && stats.numLinks == numChainLinks - numStillNeighbours && stats.numProtected == 0, "the corridor ending still: %u segment(s) over %u links, expected one over %u", stats.numSegments, stats.numLinks, numChainLinks - numStillNeighbours);
		TEST_CHECK(stats.numVetoed == corridor.lookAlikes.size() && removed == corridor.lookAlikes.size() && keptPairs(scene) == without(corridor.pairs, corridor.lookAlikes),
			"the corridor ending still: %u pairs vetoed and %u removed, expected exactly the %zu look-alikes, the %u of the images %u-59 among them", stats.numVetoed, removed, corridor.lookAlikes.size(), numStillLookAlikes, firstStill);
		const String solved = String::FormatString("Sequence chain solve: 1 segment(s) placing %u images by %u of %u links", firstStill, stats.numLinks, stats.numLinks);
		TEST_CHECK(log.Contains(solved) && log.Contains("), 3 still image(s)"), "the corridor ending still: the solve did not log \"%s\" and 3 still image(s)", solved.c_str());
		std::vector<JudgedPair> judged;
		std::map<IIndex,PlacedImage> placed;
		TEST_CHECK(parseLog(log.text, judged, placed) && judged.size() == stats.numJudged && placed.size() == numImages, "the corridor ending still: %zu lines of judged pairs (%u judged) and %zu of centres, expected every image placed", judged.size(), stats.numJudged, placed.size());
		for (IIndex i = firstStill; i < numImages; ++i)
			TEST_CHECK(placed[i].segment == placed[firstStill - 1].segment && placed[i].centre == placed[firstStill - 1].centre, "the corridor ending still: image %u is %g from the centre of image %u", i, norm(placed[i].centre - placed[firstStill - 1].centre), firstStill - 1);
		const REAL step = norm(placed[numOut - 1].centre - placed[0].centre) / REAL(numOut - 1);
		unsigned numVetoLines = 0, numStillVetoLines = 0, numStillTrueLines = 0, numStillPairLines = 0;
		for (const JudgedPair& pair : judged) {
			const PairIds ids(pair.a, pair.b);
			const REAL trueDistance = norm(placeOf(pair.b, stillFrom) - placeOf(pair.a, stillFrom));
			const REAL length = pair.length / step;
			if (corridor.lookAlikes.count(ids)) {
				++numVetoLines;
				numStillVetoLines += pair.b >= firstStill;
				TEST_CHECK(pair.veto == 1 && ABS(length - 1) <= tolerance && ABS(pair.chain / step - trueDistance) <= tolerance * trueDistance,
					"the corridor ending still: the look-alike (%u,%u) has veto %d, length %g steps and chain distance %g, expected 1, 1 and %g", pair.a, pair.b, pair.veto, length, pair.chain / step, trueDistance);
				continue;
			}
			if (pair.a >= stillFrom) {
				// a pair among the frames standing still, were it judged: nothing to veto
				++numStillPairLines;
				TEST_CHECK(pair.veto == 0, "the corridor ending still: the pair (%u,%u) standing still is vetoed", pair.a, pair.b);
				continue;
			}
			numStillTrueLines += pair.b >= firstStill;
			TEST_CHECK(pair.veto == 0 && ABS(length - trueDistance) <= tolerance * trueDistance && ABS(pair.chain / step - trueDistance) <= tolerance * trueDistance,
				"the corridor ending still: the true pair (%u,%u) has veto %d, length %g steps and chain distance %g, expected 0, %g and %g", pair.a, pair.b, pair.veto, length, pair.chain / step, trueDistance, trueDistance);
		}
		numStillJudged = stats.numJudged;
		TEST_CHECK(numVetoLines == corridor.lookAlikes.size() && numStillVetoLines == numStillLookAlikes,
			"the corridor ending still: %u look-alikes judged, %u of them of the images %u-59; expected %zu and %u", numVetoLines, numStillVetoLines, firstStill, corridor.lookAlikes.size(), numStillLookAlikes);
		// every pair that is no link is judged but the pairs among the six, which triangulate nothing; the images
		// 57-59 have 9 true pairs with the frames before the place (3, 2 and 1 up to 6 before them, and a revisit each)
		TEST_CHECK(stats.numJudged == corridor.pairs.size() - stats.numLinks - numStillPairs && numStillPairLines == 0 && numStillTrueLines == 9,
			"the corridor ending still: %u pairs judged, %u of them among the frames standing still and %u true pairs of the images %u-59; expected %zu, 0 and 9", stats.numJudged, numStillPairLines, numStillTrueLines, firstStill,
			corridor.pairs.size() - stats.numLinks - numStillPairs);
	}
	return scope.Pass("the corridor: %u look-alikes vetoed of %u judged pairs (length %.4g-%.4g and %.4g-%.4g steps on 11 and 21, discrepancy %.4g-%.4g and %.4g-%.4g), "
		"the true pairs' length within %.3g%%, their discrepancy under %.3g steps (the revisits' %.3g); the repair, the break, the abstentions, two rotation components; "
		"the view-graph filter (without the option %u of %u look-alikes kept and %u true pairs removed, with it none of either); the arc: the flipped link protected (%u supporting matches); "
		"under the near-duplicate angle: %u near-duplicate look-alikes (ray angle %.3g-%.3g degrees) vetoed at a length of %.4g-%.4g steps on 10, %.4g-%.4g off, %u true pairs judged and kept, their length within %.3g%%, "
		"the repair makes no spot of a vetoed pair; the corridor ending still: 3 images placed by standing still, their %u look-alikes vetoed, %u pairs judged",
		numLookAlikes, numJudgedPairs, lengthOne.lo, lengthOne.hi, lengthTwo.lo, lengthTwo.hi, discrepancyOne.lo, discrepancyOne.hi, discrepancyTwo.lo, discrepancyTwo.hi,
		REAL(100) * trueLengthError.hi, trueDiscrepancy.hi, revisitDiscrepancy.hi, numKeptLookAlikes, 2 * numLookAlikes, numLostTruePairs, numSupporting,
		numNear, nearAngle.lo, nearAngle.hi, nearLength.lo, nearLength.hi, nearDiscrepancy.lo, nearDiscrepancy.hi, numLow, REAL(100) * lowLengthError.hi, numStillLookAlikes, numStillJudged);
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
	const auto findPair = [](Scene& scene, IIndex a, IIndex b) -> ImagePair* {
		for (ImagePair& pair : scene.pairs)
			if (pair.ID1 == a && pair.ID2 == b)
				return &pair;
		return NULL;
	};
	const auto flip = [](ImagePair& pair) { pair.relativePose->C = -pair.relativePose->C; };
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
	ImagePair* pFlipped = findPair(matched, 3, 9);
	ImagePair* pRotated = findPair(matched, 6, 14);
	TEST_CHECK(pFlipped != NULL && pRotated != NULL && findPair(matched, 5, 13) != NULL, "the ring has no pair (3,9), (6,14) or (5,13)");
	flip(*pFlipped);
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
	flip(*findPair(filtered, 5, 13));
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
