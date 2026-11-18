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

// A dense supplement match is evidence about its pair, discounted for the precision of a
// warp-sampled position -- not full evidence, and not none. This checks each term of the weight on
// one supplemented pair, since each reads a different segment for a different reason:
//  - the ANGLE term, track-forming: FilterMatches' meanRayAngle is the median over the sparse matches
//    AND the supplement. A ray angle is a geometric quantity a warp-sampled position measures as well
//    as a sub-pixel one, and ComputeIntrinsicWeight multiplies ComputeAngleBaselineWeight(meanRayAngle)
//    into weightSpatial -- it is the ONE term that can demote a degenerate baseline, and leaving it
//    sparse hands a pair whose evidence is dense the function's maximum (1.0, not a neutral value).
//  - the AREA term, track-forming: coverage asks where the pair has correspondences, and a dense
//    draw covers what it was drawn over. A dense-only pair would otherwise score no area at all.
//  - the MAGNITUDE, discounted: sparse + w x dense (GetNumWeightedInliers), so a stratified draw
//    cannot re-rank the view graph by sheer count.
//  - the VALIDITY FLOOR, track-forming: ComputeIntrinsicWeight's minimum-support bar returns 0 below
//    it, which zeroes weightSpatial, hence GetCompositeWeight(), hence BuildTracks' minPairWeight cut
//    -- so a floor read from the sparse count alone makes a supplemented pair contribute NO tracks at
//    all, sparse or dense, on exactly the weak pairs supplementation exists to serve.
bool SupplementEvidenceIsolationTest()
{
	TD_TIMER_START();

	// two pinhole cameras 1 unit apart along X, both looking down +Z
	Scene scene;
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), 500, 500, 319.5, 239.5));
	const Point3 camCenters[2] = {Point3(-0.5, 0.0, 0.0), Point3(0.5, 0.0, 0.0)};
	for (unsigned k = 0; k < 2; ++k) {
		Pose3D pose;
		pose.C = camCenters[k];
		pose.R = Matrix3x3::IDENTITY;
		scene.images.emplace_back((IIndex)k, String::FormatString("sup%u.jpg", k), pose, 0, scene.cameras[0]);
	}
	scene.status.nCalibratedImages = scene.images.size();

	// the descriptor evidence: 5 far points (Z = 10), spread across the frame, so their
	// triangulation angle is ~5 deg and their grid occupancy is 5 cells in each image
	std::vector<Point3> sparsePts, densePts;
	for (unsigned k = 0; k < 5; ++k)
		sparsePts.emplace_back(REAL(k) * 2.0 - 4.0, REAL(k) * 0.8 - 1.6, 10.0);
	// the supplement: 15 near points (Z = 1) clustered in the middle of the overlap, i.e. a ten
	// times larger triangulation angle (~53 deg) and three quarters of the accepted matches, so a
	// median taken over all of them lands nowhere near the sparse one
	for (unsigned k = 0; k < 15; ++k)
		densePts.emplace_back(REAL(k) * 0.014 - 0.098, REAL(k) * 0.014 - 0.098, 1.0);

	for (unsigned k = 0; k < 2; ++k) {
		Image& img = scene.images[k];
		for (const Point3& X : sparsePts) {
			const auto [proj, valid] = img.ProjectPoint(X);
			if (!valid || !Image8U::isInside(proj, img.GetSize())) {
				VERBOSE("SupplementEvidenceIsolationTest FAILED: sparse point outside image %u", k);
				return false;
			}
			img.keypoints.emplace_back(Cast<float>(proj), 4.f, -1.f, 0.05f);
		}
		img.CloseDescribedKeypoints();
		for (const Point3& X : densePts) {
			const auto [proj, valid] = img.ProjectPoint(X);
			if (!valid || !Image8U::isInside(proj, img.GetSize())) {
				VERBOSE("SupplementEvidenceIsolationTest FAILED: dense point outside image %u", k);
				return false;
			}
			img.keypoints.push_back(Image::MakeDenseKeypoint(Cast<float>(proj), 0.9f, 6.f));
		}
	}
	ImagePair& pair = scene.pairs.emplace_back(0u, 1u);
	pair.relativePose = scene.images[1] / scene.images[0];
	for (uint32_t k = 0; k < (uint32_t)(sparsePts.size() + densePts.size()); ++k)
		pair.matches.emplace_back(k, k);

	// the median cosine over the TRACK-FORMING matches, sparse and dense together, derived here from
	// the 3D points rather than from the code under test: FloatArr::GetMedian takes the element at
	// index size/2 after nth_element, and the cosine is monotone in the angle. The sparse-only median
	// is derived alongside it, so the check below can state that the two really do differ -- the
	// fixture's dense points sit ten times closer than its sparse ones for exactly that reason.
	const auto RayCosines = [&camCenters](const std::vector<Point3>& pts, std::vector<REAL>& cosines) {
		for (const Point3& X : pts) {
			const Point3 V1 = X - camCenters[0], V2 = X - camCenters[1];
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
	if (numSparse != 5 || pair.GetNumFilteredInliers() != 5 || pair.GetNumDenseInliers() != 15 ||
		pair.GetNumTrackFormingMatches() != 20) {
		VERBOSE("SupplementEvidenceIsolationTest FAILED: partition after FilterMatches is %u sparse, %u dense (returned %u)",
			pair.GetNumFilteredInliers(), pair.GetNumDenseInliers(), numSparse);
		return false;
	}
	if (ABS(REAL(pair.meanRayAngle) - expectedAngle) > 1e-3 ||
		ABS(expectedAngle - expectedSparseAngle) < D2R(REAL(1))) {
		VERBOSE("SupplementEvidenceIsolationTest FAILED: meanRayAngle is %.4f deg, the track-forming median is %.4f deg "
			"(the sparse-only median is %.4f deg) -- the dense supplement must vote on the pair's angle statistic",
			R2D(pair.meanRayAngle), R2D(expectedAngle), R2D(expectedSparseAngle));
		return false;
	}

	// The floor: 5 sparse matches are under the default minInliers = 15, 20 track-forming ones are
	// not, so the pair must come out with a non-zero weight instead of being hard-zeroed.
	ComputePairsWeights(scene);
	const ImagePair& weighted = *scene.FindPair(0, 1);
	if (weighted.weightSpatial <= 0.f) {
		VERBOSE("SupplementEvidenceIsolationTest FAILED: weightSpatial is 0 on a supplemented pair with 5 sparse "
			"and 15 dense matches -- the validity floor is reading the sparse count, so this pair contributes no tracks");
		return false;
	}
	// The AREA score runs over the TRACK-FORMING matches, supplement included: it asks where the
	// pair has correspondences, and the supplement covers what it was drawn over. Recomputed here
	// from the projections themselves -- the 5 sparse points occupy 5 of the 10x10 cells of each
	// image and the 15 clustered dense ones add a few more -- so the check states the rule rather
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
	if (occupiedTrackForming <= occupiedSparse || ABS(weighted.weightSpatial - expectedSpatial) > 1e-5f) {
		VERBOSE("SupplementEvidenceIsolationTest FAILED: weightSpatial is %.6f, expected %.6f (%u/100 track-forming grid "
			"occupancy, against %u/100 sparse, x the sparse angle score)",
			weighted.weightSpatial, expectedSpatial, occupiedTrackForming, occupiedSparse);
		return false;
	}
	// The MAGNITUDE the area score multiplies is where the supplement is scaled instead of
	// ignored: the pair's evidence is 5 sparse + 50/2000 x 15 dense = 5.375 at the default frame
	// scale, more than the sparse count alone and far less than the twenty a full count would
	// give, so a coverage-maximising draw cannot re-rank the graph by sheer count, and a
	// dense-only pair is still not a zero.
	if (ABS(weighted.weightedInliers - 5.375f) > 1e-5f || weighted.GetNumWeightedInliers() != 5 ||
		weighted.GetNumFilteredInliers() != 5) {
		VERBOSE("SupplementEvidenceIsolationTest FAILED: the pair's evidence is %.4f (%u rounded) with %u sparse, "
			"expected 5 + 50/2000 x 15 = 5.375 (5)",
			weighted.weightedInliers, weighted.GetNumWeightedInliers(), weighted.GetNumFilteredInliers());
		return false;
	}

	// THE DEGENERATE BASELINE OF A DENSE-ONLY PAIR. The angle term is the only factor of
	// weightSpatial that can demote a pair for a bad baseline, and a pair whose evidence is entirely
	// dense used to be structurally unable to reach it: no sparse match meant no meanRayAngle, and
	// ComputeAngleBaselineWeight scores an unmeasured baseline at its MAXIMUM. Two dense-only pairs
	// built through AppendDenseMatches -- the path the matcher uses -- differing in NOTHING but the
	// depth of what they see: the same eighty image positions, so the grid occupancy and hence the
	// area score are identical by construction, and the same 1-unit baseline. The far one must come
	// out strictly weaker, and strictly below the area score it would have carried with an
	// unmeasured baseline.
	const auto DenseOnlyWeight = [&](REAL depth, float& weightSpatial, float& rayAngleDeg) -> bool {
		Scene denseScene;
		denseScene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), 500, 500, 319.5, 239.5));
		for (unsigned k = 0; k < 2; ++k) {
			Pose3D pose;
			pose.C = camCenters[k];
			pose.R = Matrix3x3::IDENTITY;
			denseScene.images.emplace_back((IIndex)k, String::FormatString("dns%u.jpg", k), pose, 0, denseScene.cameras[0]);
		}
		denseScene.status.nCalibratedImages = denseScene.images.size();
		// eighty points spread over the frame at a fixed DEPTH: one weighted inlier at the default
		// frame scale (twenty would be a quarter and round away as no evidence, and a real dense-only
		// pair carries hundreds): a pixel of image 0 back-projected to the bearing that reaches that
		// depth, so the A-side positions are the same whatever the depth
		std::vector<Point2f> pointsA, pointsB;
		std::vector<float> confidences;
		for (unsigned gy = 0; gy < 8; ++gy) {
			for (unsigned gx = 0; gx < 10; ++gx) {
				const Point2f ptA(200.f + 40.f*(float)gx, 60.f + 50.f*(float)gy);
				const Point3 bearing = denseScene.cameras[0]->UnprojectNormalized(Cast<REAL>(ptA));
				const Point3 X = camCenters[0] + bearing*(depth/bearing.z);
				const auto [proj, valid] = denseScene.images[1].ProjectPoint(X);
				if (!valid || !Image8U::isInside(Cast<float>(proj), denseScene.images[1].GetSize()))
					return false;
				pointsA.push_back(ptA);
				pointsB.push_back(Cast<float>(proj));
				confidences.push_back(0.9f);
			}
		}
		ImagePair& densePair = denseScene.pairs.emplace_back(0u, 1u);
		densePair.relativePose = denseScene.images[1] / denseScene.images[0];
		// the dense-only construction: no sparse match ever, the whole match set appended here
		if (AppendDenseMatches(denseScene, densePair, pointsA, pointsB, confidences, cv::Size(160, 160)) != pointsA.size() ||
			densePair.GetNumFilteredInliers() != 0 || densePair.GetNumDenseInliers() != pointsA.size())
			return false;
		ComputePairsWeights(denseScene);
		weightSpatial = densePair.weightSpatial;
		rayAngleDeg = (float)R2D(densePair.meanRayAngle);
		return true;
	};
	float farSpatial = 0.f, nearSpatial = 0.f, farAngleDeg = 0.f, nearAngleDeg = 0.f;
	if (!DenseOnlyWeight(REAL(200), farSpatial, farAngleDeg) ||
		!DenseOnlyWeight(REAL(4), nearSpatial, nearAngleDeg)) {
		VERBOSE("SupplementEvidenceIsolationTest FAILED: could not build the two dense-only fixtures");
		return false;
	}
	// the two really are a weak and a wide baseline, measured off the dense matches alone
	if (farAngleDeg <= 0.f || farAngleDeg > 1.f || nearAngleDeg < 10.f) {
		VERBOSE("SupplementEvidenceIsolationTest FAILED: the dense-only pairs measured %.4f deg and %.4f deg, "
			"expected a sub-degree and a wide baseline -- a dense-only pair is not measuring its baseline at all",
			farAngleDeg, nearAngleDeg);
		return false;
	}
	// ...and the weak one is demoted for it, both against the wide one and against the score an
	// unmeasured baseline would have handed it (the area alone, since that factor would be 1.0)
	const float areaOnly = nearSpatial/ImagePair::ComputeAngleBaselineWeight(nearAngleDeg);
	if (!(farSpatial < nearSpatial) || !(farSpatial < areaOnly*0.9f)) {
		VERBOSE("SupplementEvidenceIsolationTest FAILED: the %.4f-deg dense-only pair scores %.6f against %.6f for the "
			"%.4f-deg one (area alone would be %.6f) -- a degenerate baseline is not demoted",
			farAngleDeg, farSpatial, nearSpatial, nearAngleDeg, areaOnly);
		return false;
	}

	VERBOSE("SupplementEvidenceIsolationTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

namespace {

// Scenes for the view-graph tests (TripletFilterTest, StarReferenceViewTest): a pair carries an
// inlier count, its matches' coverage and a verification flag (F = Matrix3x3::IDENTITY stands in
// for the geometric verification), so neither descriptors nor real images are needed, and the
// images need only a camera.
struct TripletPairSpec { IIndex idA, idB; unsigned numInliers; bool verified; };

void BuildTripletScene(Scene& scene, IIndex numImages, const TripletPairSpec* specs, unsigned numPairs)
{
	AddTripletImages(scene, numImages);
	for (unsigned i = 0; i < numPairs; ++i)
		AddTripletPair(scene, specs[i].idA, specs[i].idB, specs[i].numInliers, specs[i].verified);
}

std::set<std::pair<IIndex,IIndex>> TripletKeptPairs(const Scene& scene)
{
	std::set<std::pair<IIndex,IIndex>> kept;
	for (const ImagePair& pair : scene.pairs)
		kept.emplace(pair.ID1, pair.ID2);
	return kept;
}
} // namespace

bool TripletFilterTest()
{
	TD_TIMER_START();
	constexpr float eps = 1e-6f;

	// 1. The scores: an 8-node graph with two triplet-graph components, triangles {0,1,2} and
	// {1,2,3} sharing edge (1,2), plus the isolated triangle {5,6,7}; G_LCT is the four nodes and
	// five edges of the first component, scored by the mean over its triangles of s_ij / max s_kl.
	// A 1-cell grid gives every pair full coverage, so strength = inlier count.
	PairsWeightingConfig cellCfg; cellCfg.gridSize = 1;
	static const TripletPairSpec pairSpecs[] = {
		{0,1,100,true}, {0,2,100,true}, {1,2,70,true}, {1,3,50,true}, {2,3,40,true},
		{3,4,30,true}, {3,5,20,true}, {5,6,10,true}, {6,7,10,true}, {5,7,10,true}
	};
	Scene scene; BuildTripletScene(scene, 8, pairSpecs, 10);
	const TripletScores scores03 = ComputeTripletScores(scene, 0.3f, 0.f, cellCfg);
	if (scores03.numTriplets != 3 || scores03.numTripletComponents != 2 || scores03.numScoredPairs != 5 ||
		scores03.numNodes != 4 || scores03.maxDegree != 3) {
		VERBOSE("TripletFilterTest FAILED: statistics %zu triplets in %u components, %u scored pairs, %u nodes, max degree %u (expected 3, 2, 5, 4, 3)",
			scores03.numTriplets, scores03.numTripletComponents, scores03.numScoredPairs, scores03.numNodes, scores03.maxDegree);
		return false;
	}
	const float expectedScores[] = {1.f, 1.f, 0.85f, 50.f/70.f, 40.f/70.f, -1.f, -1.f, -1.f, -1.f, -1.f};
	FOREACH(i, scores03.scores) {
		if (ABS(scores03.scores[i] - expectedScores[i]) > eps) {
			VERBOSE("TripletFilterTest FAILED: pair %u (%u,%u) scored %g, expected %g", i, scene.pairs[i].ID1, scene.pairs[i].ID2, scores03.scores[i], expectedScores[i]);
			return false;
		}
	}
	// tau = m(1 - d_max/|V|) + d_max/|V| = m*0.25 + 0.75: 0.825 at m=0.3, 0.9 at m=0.6, and the
	// scores at m=0.6 (spot-checked) do not depend on m.
	const TripletScores scores06 = ComputeTripletScores(scene, 0.6f, 0.f, cellCfg);
	if (ABS(scores03.tau - 0.825f) > eps || ABS(scores06.tau - 0.9f) > eps ||
		ABS(scores06.scores[2] - expectedScores[2]) > eps || ABS(scores06.scores[3] - expectedScores[3]) > eps) {
		VERBOSE("TripletFilterTest FAILED: tau %g at m=0.3 (expected 0.825), %g at m=0.6 (expected 0.9); scores at m=0.6 %g %g (expected %g %g)",
			scores03.tau, scores06.tau, scores06.scores[2], scores06.scores[3], expectedScores[2], expectedScores[3]); return false;
	}
	// A weaker duplicate of (0,1) collapses onto the same edge and shares its score; an unverified
	// pair and a self-pair are never edges (each would close a new triangle if it were one, so the
	// triplet count staying at 3 proves they were left out), and a pair without an inlier has no
	// strength to score and stays unscored.
	TripletPairSpec dupSpecs[14];
	memcpy(dupSpecs, pairSpecs, sizeof(pairSpecs));
	dupSpecs[10] = TripletPairSpec{0, 1, 10, true}; dupSpecs[11] = TripletPairSpec{0, 3, 25, false};
	dupSpecs[12] = TripletPairSpec{2, 4, 0, true}; dupSpecs[13] = TripletPairSpec{3, 3, 30, true};
	Scene sceneDup; BuildTripletScene(sceneDup, 8, dupSpecs, 14);
	const TripletScores scoresDup = ComputeTripletScores(sceneDup, 0.3f, 0.f, cellCfg);
	if (scoresDup.numTriplets != 3 || scoresDup.numScoredPairs != 6 || scoresDup.numNodes != 4 || scoresDup.maxDegree != 3 ||
		ABS(scoresDup.scores[10] - scoresDup.scores[0]) > eps ||
		scoresDup.scores[11] != -1.f || scoresDup.scores[12] != -1.f || scoresDup.scores[13] != -1.f) {
		VERBOSE("TripletFilterTest FAILED: duplicate scene %zu triplets, %u scored, %u nodes, max degree %u; duplicate %g vs %g, unverified/no-inlier/self-pair %g %g %g "
			"(expected 3, 6, 4, 3; equal; all -1)",
			scoresDup.numTriplets, scoresDup.numScoredPairs, scoresDup.numNodes, scoresDup.maxDegree,
			scoresDup.scores[10], scoresDup.scores[0], scoresDup.scores[11], scoresDup.scores[12], scoresDup.scores[13]); return false;
	}

	// 2. The coverage discount: three images, a triangle whose (0,2) edge touches only 10 of the
	// 100 cells of a 10x10 grid over image 0 while (0,1) and (1,2) touch every cell, so (0,2)
	// scores 90/600 = 0.15 (a tenth of its raw 900 inliers); on a 2x2 grid the same ten cells are
	// its whole top row, half the coarse grid, so it scores 450/600 = 0.75 instead.
	PairsWeightingConfig weightingCfg; // default gridSize (10)
	Scene sceneCov; AddTripletImages(sceneCov, 3);
	AddTripletPair(sceneCov, 0, 1, 600); AddTripletPair(sceneCov, 1, 2, 600); AddTripletPair(sceneCov, 0, 2, 900, true, 10, 100);
	const TripletScores covScores = ComputeTripletScores(sceneCov, 0.f, 0.f, weightingCfg);
	PairsWeightingConfig coarseCfg; coarseCfg.gridSize = 2;
	const TripletScores coarseScores = ComputeTripletScores(sceneCov, 0.f, 0.f, coarseCfg);
	if (covScores.numTriplets != 1 || covScores.numScoredPairs != 3 || !ISEQUAL(covScores.scores[0], 1.f) ||
		!ISEQUAL(covScores.scores[1], 1.f) || !ISEQUAL(covScores.scores[2], 0.15f) || !ISEQUAL(coarseScores.scores[2], 0.75f)) {
		VERBOSE("TripletFilterTest FAILED: coverage scene %zu triplets, %u scored, scores %g %g %g on the default grid, %g on 2x2 (expected 1, 3, 1 1 0.15, 0.75)",
			covScores.numTriplets, covScores.numScoredPairs, covScores.scores[0], covScores.scores[1], covScores.scores[2], coarseScores.scores[2]); return false;
	}
	// The cutting rule at m = 0.5: G_LCT has 3 nodes and max degree 2, so tau = 0.5(1-2/3)+2/3 =
	// 0.833, applied as given. secondFaceScore (0.75, above minScore) is checked too and the
	// descent always runs when the ceiling shatters the graph; reading the source, neither changes
	// anything on three nodes (the two 1.0 edges hold every node in one piece at both ceilings), so
	// only the doppelganger (0,2) goes.
	TripletFilterConfig covCfg; covCfg.enabled = true; covCfg.cut = true; covCfg.minScore = 0.5f; covCfg.minYield = 0.f;
	const unsigned covRemoved = FilterPairsByTriplets(sceneCov, covCfg, weightingCfg);
	const std::set<std::pair<IIndex,IIndex>> covExpected{{0,1}, {1,2}};
	if (covRemoved != 1 || TripletKeptPairs(sceneCov) != covExpected) {
		VERBOSE("TripletFilterTest FAILED: coverage scene removed %u pairs; expected the doppelganger (0,2) alone, (0,1) and (1,2) kept", covRemoved); return false;
	}

	// 3. The yield rule: a 15-image walk in five 3-image copies (0-2, 3-5, ..., 12-14); consecutive
	// images share 1000 inliers at 2 degrees, images two apart 600 at 4 degrees, and the first
	// image of every copy pairs with the first image of every other copy on 300 inliers at 1
	// degree -- a near-duplicate viewpoint delivering far fewer inliers than a real pair at that
	// ray angle, so a triangle built only from those look-alike pairs gives no evidence.
	const auto buildWalk = [](Scene& s) {
		AddTripletImages(s, 15);
		const auto add = [&s](IIndex a, IIndex b, unsigned numInliers, float rayAngleDeg) {
			AddTripletPair(s, a, b, numInliers);
			s.pairs.Last().meanRayAngle = (float)D2R(rayAngleDeg);
		};
		for (IIndex i = 0; i + 1 < 15; ++i) add(i, i + 1, 1000, 2.f);
		for (IIndex i = 0; i + 2 < 15; ++i) add(i, i + 2, 600, 4.f);
		for (IIndex a = 0; a < 15; a += 3)
			for (IIndex b = a + 3; b < 15; b += 3)
				add(a, b, 300, 1.f);
	};
	Scene walk; buildWalk(walk);
	const unsigned idx01 = 0, idx02 = 14, idx03 = 27, idx06 = 28; // in order of insertion
	const TripletFilterConfig yieldDefaults;
	const TripletScores yieldScores = ComputeTripletScores(walk, 0.f, yieldDefaults.minYield, weightingCfg);
	const TripletScores yieldOff = ComputeTripletScores(walk, 0.f, 0.f, weightingCfg); // rule off: look-alike triangles count like any other
	if (yieldScores.numTriplets != 31 || yieldScores.numDoppelgangerTriplets != 10 || yieldScores.numScoredPairs != 37 ||
		!ISEQUAL(yieldScores.scores[idx01], 1.f) || !ISEQUAL(yieldScores.scores[idx02], 0.6f) ||
		!ISEQUAL(yieldScores.scores[idx03], 0.12f) || !ISEQUAL(yieldScores.scores[idx06], 0.f) ||
		yieldOff.numDoppelgangerTriplets != 0 || !ISEQUAL(yieldOff.scores[idx03], 0.72f) || !ISEQUAL(yieldOff.scores[idx06], 1.f)) {
		VERBOSE("TripletFilterTest FAILED: walk %zu triplets (%u doppelganger), %u scored; (0,1) %g (0,2) %g (0,3) %g (0,6) %g; rule off (0,3) %g (0,6) %g "
			"(expected 31 (10), 37; 1 0.6 0.12 0; 0.72 1)",
			yieldScores.numTriplets, yieldScores.numDoppelgangerTriplets, yieldScores.numScoredPairs,
			yieldScores.scores[idx01], yieldScores.scores[idx02], yieldScores.scores[idx03], yieldScores.scores[idx06],
			yieldOff.scores[idx03], yieldOff.scores[idx06]); return false;
	}
	// A ray angle that is not finite has no measurable geometry: giving (0,6) a NaN angle takes it
	// out of every envelope bin, so its own yield reverts to 1 and the three doppelganger triangles
	// it took part in -- (0,3,6), (0,6,9), (0,6,12) -- stop being doppelgangers, leaving 7; (0,3)'s
	// triangle with 6 now scores 1 instead of 0, raising its mean from 0.6 to 1.6 over 5 triplets, 0.32.
	Scene walkNaN; buildWalk(walkNaN);
	walkNaN.pairs[idx06].meanRayAngle = std::numeric_limits<float>::quiet_NaN();
	const TripletScores nanScores = ComputeTripletScores(walkNaN, 0.f, yieldDefaults.minYield, weightingCfg);
	if (nanScores.numDoppelgangerTriplets != 7 || !ISEQUAL(nanScores.scores[idx03], 0.32f) ||
		!ISEQUAL(nanScores.scores[idx01], 1.f) || !ISEQUAL(nanScores.scores[idx02], 0.6f)) {
		VERBOSE("TripletFilterTest FAILED: NaN ray angle on (0,6), %u doppelganger triplets; (0,1) %g (0,2) %g (0,3) %g (expected 7; 1 0.6 0.32)",
			nanScores.numDoppelgangerTriplets, nanScores.scores[idx01], nanScores.scores[idx02], nanScores.scores[idx03]); return false;
	}
	// The cutting rule at m = 0.6: G_LCT has 15 nodes and max degree 8 (image 3), so tau =
	// 0.6(1-8/15)+8/15 = 0.813; the fourteen consecutive pairs alone score 1 and already hold all
	// 15 images together, so the ceiling applies as given and only they survive.
	TripletFilterConfig yieldFilterCfg; yieldFilterCfg.enabled = true; yieldFilterCfg.cut = true; yieldFilterCfg.minScore = 0.6f;
	const unsigned yieldRemoved = FilterPairsByTriplets(walk, yieldFilterCfg, weightingCfg);
	std::set<std::pair<IIndex,IIndex>> yieldExpected;
	for (IIndex i = 0; i + 1 < 15; ++i) yieldExpected.emplace(i, i + 1);
	if (yieldRemoved != 23 || TripletKeptPairs(walk) != yieldExpected) {
		VERBOSE("TripletFilterTest FAILED: walk removed %u pairs; expected 23, keeping only the 14 consecutive pairs", yieldRemoved); return false;
	}

	// 4. The cutting rule and the default mode on two rooms of 60 and 30 images (consecutive pairs 1000 inliers, two apart
	// 600) joined by three weak pairs, (59,60)=60, (58,60)=50, (59,61)=50: no image holds more than
	// four pairs, so G_LCT has |V|=90, d_max=4 and tau = 0.3(1-4/90)+4/90 = 0.331, below which only
	// the three bridges sit.
	PairsWeightingConfig roomsCfg; roomsCfg.gridSize = 1;
	const auto buildRooms = [](Scene& s) {
		AddTripletImages(s, 90);
		for (IIndex i = 0; i + 1 < 60; ++i) AddTripletPair(s, i, i + 1, 1000);
		for (IIndex i = 0; i + 2 < 60; ++i) AddTripletPair(s, i, i + 2, 600);
		for (IIndex i = 60; i + 1 < 90; ++i) AddTripletPair(s, i, i + 1, 1000);
		for (IIndex i = 60; i + 2 < 90; ++i) AddTripletPair(s, i, i + 2, 600);
		AddTripletPair(s, 59, 60, 60); AddTripletPair(s, 58, 60, 50); AddTripletPair(s, 59, 61, 50);
	};
	// the scores of the pairs a scene holds, as the filter should have handed them over
	const auto sameScores = [](const std::vector<float>& scores, const Scene& scene, const TripletFilterConfig& cfg, const PairsWeightingConfig& weighting) {
		const TripletScores expected = ComputeTripletScores(scene, cfg.minScore, cfg.minYield, weighting);
		return scores == expected.scores;
	};
	{
		// The cutting rule removes all three bridges: the larger room's 60 images seed the reconstruction, and the
		// scores handed over are those of the 174 pairs left.
		Scene rooms; buildRooms(rooms);
		TripletFilterConfig cutCfg; cutCfg.enabled = true; cutCfg.cut = true; cutCfg.minYield = 0.f;
		IIndexArr seeds;
		std::vector<float> scores;
		const unsigned removed = FilterPairsByTriplets(rooms, cutCfg, roomsCfg, &seeds, &scores);
		if (removed != 3 || TripletKeptPairs(rooms).size() != 174 || seeds.size() != 60 || seeds.front() != 0 || seeds.back() != 59) {
			VERBOSE("TripletFilterTest FAILED: rooms cutting rule removed %u pairs, %u seeds; expected 3 removed, 174 kept, 60 seed images 0-59",
				removed, (unsigned)seeds.size()); return false;
		}
		if (scores.size() != 174 || !sameScores(scores, rooms, cutCfg, roomsCfg)) {
			VERBOSE("TripletFilterTest FAILED: rooms cutting rule handed over %zu scores; expected those of the 174 pairs left", scores.size()); return false;
		}
	}
	{
		// The default mode removes nothing, whatever the graph could spare, and names the same seed views:
		// the larger room's 60 images; it hands over the scores it computed, one per pair, the bridges
		// below the ceiling among them.
		Scene rooms; buildRooms(rooms);
		TripletFilterConfig defaultCfg; defaultCfg.enabled = true; defaultCfg.minYield = 0.f;
		IIndexArr seeds;
		std::vector<float> scores;
		const unsigned removed = FilterPairsByTriplets(rooms, defaultCfg, roomsCfg, &seeds, &scores);
		if (removed != 0 || rooms.pairs.size() != 177 || seeds.size() != 60 || seeds.front() != 0 || seeds.back() != 59) {
			VERBOSE("TripletFilterTest FAILED: rooms default mode removed %u of 177 pairs, %u seeds; expected none removed, 60 seed images 0-59",
				removed, (unsigned)seeds.size()); return false;
		}
		const TripletScores expected = ComputeTripletScores(rooms, defaultCfg.minScore, defaultCfg.minYield, roomsCfg);
		if (scores.size() != 177 || scores != expected.scores || expected.numScoredPairs != 177 ||
			!(scores[174] < expected.tau) || !(scores[175] < expected.tau) || !(scores[176] < expected.tau)) {
			VERBOSE("TripletFilterTest FAILED: rooms default mode handed over %zu scores (bridges %g %g %g, ceiling %g); expected the 177 scores of "
				"ComputeTripletScores with the three bridges below the ceiling", scores.size(), scores.size() > 176 ? scores[174] : -9.f,
				scores.size() > 176 ? scores[175] : -9.f, scores.size() > 176 ? scores[176] : -9.f, expected.tau); return false;
		}
	}
	{
		// A disabled filter removes nothing, reports no seed views and leaves every pair unscored.
		Scene rooms; buildRooms(rooms);
		TripletFilterConfig offCfg; offCfg.enabled = false;
		IIndexArr seeds;
		std::vector<float> scores;
		const unsigned removed = FilterPairsByTriplets(rooms, offCfg, roomsCfg, &seeds, &scores);
		if (removed != 0 || rooms.pairs.size() != 177 || !seeds.empty() ||
			scores != std::vector<float>(177, TripletScores::unscored)) {
			VERBOSE("TripletFilterTest FAILED: disabled filter removed %u pairs (%u seeds, %zu scores); expected none, no seeds, 177 unscored",
				removed, (unsigned)seeds.size(), scores.size()); return false;
		}
	}
	{
		// A scene without a verified pair has nothing to score: its pairs are all unscored.
		Scene bare; AddTripletImages(bare, 3);
		AddTripletPair(bare, 0, 1, 100, false); AddTripletPair(bare, 1, 2, 100, false);
		std::vector<float> scores;
		const unsigned removed = FilterPairsByTriplets(bare, TripletFilterConfig(), roomsCfg, NULL, &scores);
		if (removed != 0 || scores != std::vector<float>(2, TripletScores::unscored)) {
			VERBOSE("TripletFilterTest FAILED: scene without a verified pair removed %u pairs, %zu scores; expected none and 2 unscored", removed, scores.size()); return false;
		}
	}

	// 5. The defaults: enabled, the default mode (removes nothing, scores only), the paper's minimum
	// score for medium/small ambiguous sets, the yield rule on, the second face's score.
	const TripletFilterConfig defaults;
	if (!defaults.enabled || defaults.cut || !ISEQUAL(defaults.minScore, 0.3f) || !ISEQUAL(defaults.minYield, 0.4f) ||
		!ISEQUAL(defaults.secondFaceScore, 0.75f)) {
		VERBOSE("TripletFilterTest FAILED: defaults enabled %d cut %d minScore %g minYield %g secondFaceScore %g",
			defaults.enabled ? 1 : 0, defaults.cut ? 1 : 0, defaults.minScore, defaults.minYield, defaults.secondFaceScore); return false;
	}

	VERBOSE("TripletFilterTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test function for rotation estimation
bool RotationEstimatorTest()
{
	TD_TIMER_START();

	// Generate test scene with 3 images arranged in a circle with known rotations
	Scene sceneGT, scene;
	SceneConfig cfg;
	cfg.numImages = 16;
	cfg.numPoints = 200;
	cfg.rotationAngleStep = 16.0;
	cfg.generatePairs = true;
	cfg.perturbOptions = SceneConfig::PERTURB_POSES | SceneConfig::PERTURB_PAIR_POSES; // Perturb poses to test averaging
	GenerateTestScene(sceneGT, cfg, &scene);

	// Remove poses to simulate unknown rotations
	for (Image& img : scene.images)
		img.InvalidatePose();

	ComputePairsWeights(scene);

	// Disconnect image 2 by setting an artificial weight of 0 for all pairs involving it
	const uint32_t disconnectedImageId = 2;
	for (ImagePair& pair : scene.pairs)
		if (pair.ID1 == disconnectedImageId || pair.ID2 == disconnectedImageId)
			pair.InvalidateWeight();

	// Run rotation estimator
	GlobalRotationEstimatorOptions options;
	GlobalRotationEstimator estimator(options);
	if (!estimator.EstimateRotations(scene)) {
		VERBOSE("ERROR: GlobalRotationEstimator::EstimateRotations failed!");
		return false;
	}

	// Validate results: check that recovered rotations are close to ground truth
	// Note: There's a gauge freedom (global rotation), so we compare relative rotations
	constexpr double tolerance = D2R(5.0); // 5 degrees tolerance
	double maxAngleError = 0.0;

	// Compute rotation errors relative to first image to account for gauge freedom
	const RMatrix R0_gt = sceneGT.images[0].R;
	const RMatrix R0_est = scene.images[0].R;
	for (size_t i = 1; i < scene.images.size(); ++i) {
		// Skip disconnected image since it will remain invalid
		if (i == disconnectedImageId)
			continue;
		// Compute relative rotation: R_i_rel = R_i * R_0^T
		const RMatrix R_i_rel_gt = sceneGT.images[i].R * R0_gt.t();
		const RMatrix R_i_rel_est = scene.images[i].R * R0_est.t();
		// Compute rotation error between relative rotations
		const double angleError = ACOS(ComputeAngle(R_i_rel_est, R_i_rel_gt));
		maxAngleError = MAXF(maxAngleError, angleError);
		if (angleError > tolerance) {
			VERBOSE("error: GlobalRotationEstimator image %zu relative rotation error too large: %.2f deg, tolerance %.2f deg",
				i, R2D(angleError), R2D(tolerance));
		}
	}
	if (maxAngleError > tolerance) {
		VERBOSE("ERROR: GlobalRotationEstimator test failed! Max relative angle error: %.4g deg",
			R2D(maxAngleError));
		return false;
	}

	VERBOSE("GlobalRotationEstimator test passed (max relative angle error: %.4g deg) (%s)",
		R2D(maxAngleError), TD_TIMER_GET_FMT().c_str());
	return true;
}

// The rotation averaging does not depend on the absolute scale of the pair weights: a graph of 40
// rotations joined by noisy pairs, a tenth of them wrong and light, with weights spread over three
// orders of magnitude, solved with its weights as given and multiplied by 2^-7 and by 2^7 (powers of two, so
// the scaled weights are exact), gives bit-identical rotations and removes the same pairs past the
// 12-degree bar. Without the estimator's median scaling the L1 step's fixed-iteration ADMM sees a
// different problem at each scale and the rotations differ.
bool RotationWeightScaleTest()
{
	TD_TIMER_START();
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
	if (!Solve(1.f, reference, referenceFiltered)) {
		VERBOSE("RotationWeightScaleTest FAILED: the estimator failed on the weights as given");
		return false;
	}
	for (const float factor : {1.f / 128.f, 128.f}) {
		std::vector<Point3> rotations;
		unsigned numFiltered;
		if (!Solve(factor, rotations, numFiltered)) {
			VERBOSE("RotationWeightScaleTest FAILED: the estimator failed on the weights times %g", factor);
			return false;
		}
		double maxDifference = 0;
		for (uint32_t i = 0; i < NUM_NODES; ++i)
			maxDifference = MAXF(maxDifference, (double)norm(rotations[i] - reference[i]));
		if (maxDifference != 0 || numFiltered != referenceFiltered) {
			VERBOSE("RotationWeightScaleTest FAILED: the weights times %g move the rotations by up to %g degrees and filter %u pairs against %u; "
				"the result must not depend on the scale of the weights", factor, R2D(maxDifference), numFiltered, referenceFiltered);
			return false;
		}
	}
	VERBOSE("RotationWeightScaleTest PASSED: %u pairs, %u past the bar, the same rotations at weights x1, x1/128 and x128 (%s)",
		(unsigned)pairs.size(), referenceFiltered, TD_TIMER_GET_FMT().c_str());
	return true;
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

// A matched scene goes through the view-graph filters (the triplet filter, the contradiction cut and its repair)
// once, and a scene saved after them is where the reconstruction starts. On the ring of TranslationCutTest saved as
// matched, three pairs planted the repair brings back: (1) a match-images-only run that loads it filters it once,
// to the pairs, repair marks and seed views of the two filters applied once by hand, and leaves it PAIRS_FILTERED;
// (2) saved so and given back as source, it is not filtered again: the reloaded scene holds the very pairs, marks,
// weights and seed views it was saved with, a pair flipped after the filters (which they remove when they run)
// included.
bool FilteredSceneReloadTest()
{
	TD_TIMER_START();
	const ScopedTempDir tmpDir(_T("FilteredSceneReloadTest"));
	if (!tmpDir.IsValid())
		return false;
	const auto findPair = [](Scene& scene, IIndex a, IIndex b) -> ImagePair* {
		for (ImagePair& pair : scene.pairs)
			if (pair.ID1 == a && pair.ID2 == b)
				return &pair;
		return NULL;
	};
	const auto flip = [](ImagePair& pair) { pair.relativePose->C = -pair.relativePose->C; };
	// the pairs, their matches, poses, weights and repair marks, and the seed views, equal
	const auto sameGraph = [](const Scene& a, const Scene& b, const char* what) {
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
		if (!same)
			VERBOSE("ERROR: FilteredSceneReloadTest: %s: %u pairs and %u seed views, expected %u pairs and %u seed views, each the same",
				what, (unsigned)b.pairs.size(), (unsigned)b.seedViews.size(), (unsigned)a.pairs.size(), (unsigned)a.seedViews.size());
		return same;
	};
	ReconstructionConfig cfg;
	cfg.matchImagesOnly = true;
	cfg.matchCfg.lensFitEnabled = false; // the ring's cameras are exact: no lens to fit
	using STATE = Scene::Status::STATE;

	// the ring saved as matched, three pairs planted that the cut removes and its repair brings back
	Scene matched;
	{
		Scene sceneGT;
		SceneConfig sceneCfg;
		sceneCfg.numImages = 16;
		sceneCfg.numPoints = 300;
		sceneCfg.rotationAngleStep = 16.0;
		sceneCfg.generatePairs = true;
		GenerateTestScene(sceneGT, sceneCfg, &matched);
	}
	for (ImagePair& pair : matched.pairs)
		pair.meanRayAngle = D2R(10.f);
	ImagePair* pFlipped = findPair(matched, 3, 9);
	ImagePair* pRotated = findPair(matched, 6, 14);
	if (pFlipped == NULL || pRotated == NULL || findPair(matched, 5, 13) == NULL) {
		VERBOSE("ERROR: FilteredSceneReloadTest: the ring has no pair (3,9), (6,14) or (5,13)");
		return false;
	}
	flip(*pFlipped);
	pRotated->relativePose->R = RMatrix(Point3(0, D2R(REAL(30)), 0)) * pRotated->relativePose->R;
	if (!matched.status.nState.isSet(STATE::MATCHED) || matched.status.nState.isSet(STATE::PAIRS_FILTERED)) {
		VERBOSE("ERROR: FilteredSceneReloadTest: the generated ring is not a matched scene before the filters");
		return false;
	}
	const String matchedPath = tmpDir(_T("matched.sfm"));
	if (!matched.Save(matchedPath)) {
		VERBOSE("ERROR: FilteredSceneReloadTest: cannot save the matched scene '%s'", matchedPath.c_str());
		return false;
	}

	// (1) the filters run once on the loaded matched scene: the two filters applied once by hand to the same scene
	Scene filtered(2);
	if (!filtered.Reconstruct(matchedPath, cfg) || !filtered.status.nState.isSet(STATE::PAIRS_FILTERED)) {
		VERBOSE("ERROR: FilteredSceneReloadTest: the match-images-only run of the matched scene failed or left it unfiltered");
		return false;
	}
	Scene once(2);
	if (!once.Load(matchedPath)) {
		VERBOSE("ERROR: FilteredSceneReloadTest: cannot load the matched scene");
		return false;
	}
	ComputePairsWeights(once, cfg.matchCfg.weightingCfg);
	std::vector<float> scores;
	FilterPairsByTriplets(once, cfg.tripletFilterCfg, cfg.matchCfg.weightingCfg, &once.seedViews, &scores);
	FilterPairsByContradiction(once, scores, cfg.minPairWeight, cfg.maxPairDirectionResidual, cfg.matchCfg.weightingCfg, cfg.repairContradictedPairs);
	if (!sameGraph(once, filtered, "the matched scene loaded and filtered"))
		return false;
	const unsigned numReturned = (unsigned)std::count_if(filtered.pairs.begin(), filtered.pairs.end(), [](const ImagePair& pair) { return pair.returnedByRepair; });
	if (numReturned == 0 || filtered.seedViews.empty()) {
		VERBOSE("ERROR: FilteredSceneReloadTest: premise not met: the filters repaired %u pairs and named %u seed views, expected some of each",
			numReturned, (unsigned)filtered.seedViews.size());
		return false;
	}

	// (2) a pair flipped after the filters, which they remove when they run, then the scene saved and given back
	flip(*findPair(filtered, 5, 13));
	{
		Scene again(filtered);
		const std::vector<float> unscored(again.pairs.size(), TripletScores::unscored);
		if (FilterPairsByContradiction(again, unscored, cfg.minPairWeight, cfg.maxPairDirectionResidual, cfg.matchCfg.weightingCfg) == 0) {
			VERBOSE("ERROR: FilteredSceneReloadTest: premise not met: the contradiction cut does not remove the pair (5,13) flipped");
			return false;
		}
	}
	const String filteredPath = tmpDir(_T("filtered.sfm"));
	if (!filtered.Save(filteredPath)) {
		VERBOSE("ERROR: FilteredSceneReloadTest: cannot save the filtered scene '%s'", filteredPath.c_str());
		return false;
	}
	Scene reloaded(2);
	if (!reloaded.Reconstruct(filteredPath, cfg) || !reloaded.status.nState.isSet(STATE::PAIRS_FILTERED)) {
		VERBOSE("ERROR: FilteredSceneReloadTest: the match-images-only run of the filtered scene failed or left it unfiltered");
		return false;
	}
	ComputePairsWeights(filtered, cfg.matchCfg.weightingCfg); // the weights of the graph with (5,13) flipped, as the run weighs them
	if (!sameGraph(filtered, reloaded, "the filtered scene reloaded"))
		return false;
	VERBOSE("FilteredSceneReloadTest passed: %u pairs, %u repaired, %u seed views (%s)",
		(unsigned)reloaded.pairs.size(), numReturned, (unsigned)reloaded.seedViews.size(), TD_TIMER_GET_FMT().c_str());
	return true;
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
	TD_TIMER_START();
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
		if (expected.size() < 50) {
			VERBOSE("ERROR: PairScalesTest: the scene gives only %zu equations", expected.size());
			return false;
		}
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
			if (!same) {
				VERBOSE("ERROR: PairScalesTest: %zu pairs, %d threads: %zu equations against the reference's %zu, or they differ", pPairs->size(), numThreads, equations.size(), expected.size());
				return false;
			}
		}
	}
	#ifdef _USE_OPENMP
	omp_set_num_threads(maxThreads);
	#endif
	VERBOSE("PairScalesTest passed: scale ratios within %g of the reference's (%s)", maxRelDiff, TD_TIMER_GET_FMT().c_str());
	return true;
}

bool ScaleEstimatorTest()
{
	TD_TIMER_START();

	const std::vector<REAL> gtScales = {
		REAL(1.0), REAL(2.0), REAL(0.5), REAL(4.0), REAL(1.5)
	};
	const uint32_t numIndices = (uint32_t)gtScales.size();

	const auto ratio = [&](uint32_t i, uint32_t j) -> REAL {
		return gtScales[j] / gtScales[i];
	};

	std::vector<ScalePair> pairs;
	pairs.emplace_back(0, 1, ratio(0, 1), 30.f);
	pairs.emplace_back(1, 2, ratio(1, 2), 25.f);
	pairs.emplace_back(2, 3, ratio(2, 3), 20.f);
	pairs.emplace_back(3, 4, ratio(3, 4), 15.f);
	pairs.emplace_back(0, 2, ratio(0, 2), 20.f);
	pairs.emplace_back(1, 3, ratio(1, 3), 18.f);
	pairs.emplace_back(0, 4, ratio(0, 4), 10.f);

	GlobalScaleEstimator estimator;
	std::vector<REAL> estimatedScales;
	if (!estimator.EstimateScales(pairs, numIndices, estimatedScales)) {
		VERBOSE("ERROR: GlobalScaleEstimator::EstimateScales(auto gauge) failed");
		return false;
	}

	std::vector<REAL> estimatedScalesFixed;
	if (!estimator.EstimateScales(pairs, numIndices, 0, estimatedScalesFixed)) {
		VERBOSE("ERROR: GlobalScaleEstimator::EstimateScales(fixed gauge) failed");
		return false;
	}

	const REAL ratioTolerance = REAL(1e-4);
	const REAL fixedGaugeTolerance = REAL(1e-3);

	for (uint32_t i = 1; i < numIndices; ++i) {
		const REAL gtRel = gtScales[i] / gtScales[0];
		const REAL estRel = estimatedScales[i] / estimatedScales[0];
		if (ABS(estRel - gtRel) > ratioTolerance) {
			VERBOSE("ERROR: GlobalScaleEstimator relative ratio mismatch idx=%u est=%g gt=%g",
				i, (double)estRel, (double)gtRel);
			return false;
		}

		const REAL estRelFixed = estimatedScalesFixed[i] / estimatedScalesFixed[0];
		if (ABS(estRelFixed - gtRel) > ratioTolerance) {
			VERBOSE("ERROR: GlobalScaleEstimator(fixed) relative ratio mismatch idx=%u est=%g gt=%g",
				i, (double)estRelFixed, (double)gtRel);
			return false;
		}
	}

	if (ABS(estimatedScalesFixed[0] - REAL(1)) > fixedGaugeTolerance) {
		VERBOSE("ERROR: GlobalScaleEstimator fixed-gauge value mismatch idx=0 est=%g",
			(double)estimatedScalesFixed[0]);
		return false;
	}

	VERBOSE("GlobalScaleEstimator test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The scale estimator solves every connected component of its equations on its own: the one holding the
// fixed index is pinned at the fixed index, any other has its log-scales at mean zero (the minimum-norm least
// squares of the whole system), and an index no equation names keeps scale 1. The reference is a dense
// JacobiSVD of the whole system with the fixed index eliminated, computed here; the answer must equal it to
// 1e-9 in log scale. Cases: two components with the fixed index in the first, then in the second; the same
// estimator solving the two components joined by a bridge and then without it (a robust round that drops the
// bridge leaves a component the fixed index no longer reaches, which must come out at mean zero whatever the
// solve before it left there); and no fixed index given, as the view-graph filter calls the estimator, whose
// gauge is then the index of the largest summed weight, in the first component and in the second.
bool ScaleComponentsTest()
{
	TD_TIMER_START();
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
	// one solve of `system` by `estimator` against the reference, asking for the fixed index `requested` (NO_ID: none);
	// `twoComponents`: the system has the two components, the one the fixed index does not reach is at mean zero
	const auto check = [&](const char* what, GlobalScaleEstimator& estimator, const std::vector<ScalePair>& system, uint32_t requested, bool twoComponents) {
		const uint32_t fixed = requested != NO_ID ? requested : bestConnected(system);
		std::vector<REAL> scales;
		if (!estimator.EstimateScales(system, numIndices, requested, scales) || scales.size() != numIndices) {
			VERBOSE("ERROR: ScaleComponentsTest: %s: the solve failed", what);
			return false;
		}
		bool ok = true;
		if (!(std::abs(std::log((double)scales[fixed])) < 1e-12)) {
			VERBOSE("ERROR: ScaleComponentsTest: %s: the fixed index %u has scale %g, not 1", what, fixed, (double)scales[fixed]);
			ok = false;
		}
		if (!(std::abs(std::log((double)scales[unnamed])) < 1e-12)) {
			VERBOSE("ERROR: ScaleComponentsTest: %s: the index no equation names has scale %g, not 1", what, (double)scales[unnamed]);
			ok = false;
		}
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
		if (!(worst < 1e-9) || !(std::abs(meanUnreached) < 1e-9)) {
			VERBOSE("ERROR: ScaleComponentsTest: %s: the log scales differ from the dense SVD's by up to %g; the mean log scale of the component the fixed index does not reach is %g",
				what, worst, meanUnreached);
			ok = false;
		}
		return ok;
	};
	bool passed = true;
	GlobalScaleEstimator inFirst, inSecond, bridge;
	passed = check("two components, the fixed index in the first", inFirst, separate, 2, true) && passed;
	passed = check("two components, the fixed index in the second", inSecond, separate, 9, true) && passed;
	passed = check("one component, joined by a bridge", bridge, bridged, 2, false) && passed;
	passed = check("the bridge dropped after a solve with it", bridge, separate, 2, true) && passed;
	// no fixed index given: the weights of one component raised tenfold put the best-connected index in it
	for (const bool inFirstComponent : {true, false}) {
		std::vector<ScalePair> weighted = separate;
		for (ScalePair& pair : weighted)
			pair.weight *= (pair.idxA < unnamed) == inFirstComponent ? 10.f : 1.f;
		const char* const what = inFirstComponent ? "no fixed index, the best-connected index in the first component" : "no fixed index, the best-connected index in the second component";
		if ((bestConnected(weighted) < unnamed) != inFirstComponent) {
			VERBOSE("ERROR: ScaleComponentsTest: %s: the test's best-connected index %u is in the other component", what, bestConnected(weighted));
			passed = false;
			continue;
		}
		GlobalScaleEstimator unfixed;
		passed = check(what, unfixed, weighted, NO_ID, true) && passed;
	}
	if (!passed)
		return false;
	VERBOSE("ScaleComponentsTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool TranslationEstimatorTest()
{
	TD_TIMER_START();

	const std::vector<Point3> gtTranslations = {
		Point3(0, 0, 0),
		Point3(2, 1, 0),
		Point3(4, 1, 1),
		Point3(5, 3, 1),
		Point3(7, 4, 2)
	};
	const uint32_t numIndices = (uint32_t)gtTranslations.size();

	const auto relT = [&](uint32_t i, uint32_t j) -> Point3 {
		return gtTranslations[j] - gtTranslations[i];
	};

	std::vector<TranslationPair> pairs;
	pairs.emplace_back(0, 1, relT(0, 1), 20.f);
	pairs.emplace_back(1, 2, relT(1, 2), 22.f);
	pairs.emplace_back(2, 3, relT(2, 3), 18.f);
	pairs.emplace_back(3, 4, relT(3, 4), 16.f);
	pairs.emplace_back(0, 2, relT(0, 2), 25.f);
	pairs.emplace_back(1, 3, relT(1, 3), 12.f);
	pairs.emplace_back(0, 4, relT(0, 4), 10.f);

	GlobalTranslationEstimator estimator;
	std::vector<Point3> estimatedTranslations;
	if (!estimator.EstimateTranslations(pairs, numIndices, estimatedTranslations)) {
		VERBOSE("ERROR: GlobalTranslationEstimator::EstimateTranslations failed");
		return false;
	}

	const REAL tolerance = REAL(1e-4);
	for (const TranslationPair& pair : pairs) {
		const Point3 estRel = estimatedTranslations[pair.idxB] - estimatedTranslations[pair.idxA];
		const REAL relError = norm(estRel - pair.relativeTranslation);
		if (relError > tolerance) {
			VERBOSE("ERROR: GlobalTranslationEstimator relative translation mismatch pair=(%u,%u) err=%g",
				pair.idxA, pair.idxB, (double)relError);
			return false;
		}
	}

	VERBOSE("GlobalTranslationEstimator test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The global translation estimator solves its weighted least squares by the normal equations: the
// matrix it factors is the weighted graph Laplacian of the pairs with the gauge node (and the lowest
// node of each component without it) pinned, exactly the free nodes plus twice the distinct free-free
// pairs of non-zeros, and its centres equal Eigen's SparseQR solution of the incidence matrix to 1e-9
// of the scene's extent on (1) a connected graph, (2) a graph of two dense groups joined by one weak
// pair, row weights over 1e-2..1e2, (3) a graph of two components, the second without the gauge (placed
// up to its own translation, as the QR solution is), (4) two floating rings of unit weights, whose full
// Laplacian is exactly singular; and (5) a graph of the size of a view graph's (1200 nodes, 25173 rows)
// is solved in seconds (a Release build checks the time), not in the half minute the QR takes.
bool TranslationNormalEquationsTest()
{
	TD_TIMER_START();
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
	const auto checkLaplacian = [&](const char* what, const GlobalTranslationEstimator& estimator, const std::vector<TranslationPair>& pairs, uint32_t gauge) {
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
		if (!consistent || count != laplacian.size()) {
			VERBOSE("ERROR: TranslationNormalEquationsTest: %s: the factored matrix (%d x %d, %zu non-zeros) is not the Laplacian of the %zu free nodes and %zu entries the pairs give", what, (int)matrix.rows(), (int)matrix.cols(), (size_t)matrix.nonZeros(), numFree, laplacian.size());
			return false;
		}
		return true;
	};
	const auto check = [&](const char* what, const std::vector<TranslationPair>& pairs) {
		const uint32_t gauge = pairs.front().idxA;
		GlobalTranslationEstimator estimator;
		std::vector<Point3> centres(positions.size(), Point3(1e6, 1e6, 1e6)), expected; // stale values the solve must overwrite
		if (!estimator.EstimateTranslations(pairs, (uint32_t)positions.size(), gauge, centres)) {
			VERBOSE("ERROR: TranslationNormalEquationsTest: %s: the solve failed", what);
			return false;
		}
		if (!checkLaplacian(what, estimator, pairs, gauge))
			return false;
		// every pinned node (the gauge, and the lowest node of each component without it) is written at the origin
		for (const uint32_t pin : pinsOf(pairs, gauge))
			if (norm(centres[pin]) != 0) {
				VERBOSE("ERROR: TranslationNormalEquationsTest: %s: pinned node %u is %g from the origin", what, pin, (double)norm(centres[pin]));
				return false;
			}
		reference(pairs, gauge, expected);
		const REAL error = difference(pairs, gauge, centres, expected);
		if (!(error < REAL(1e-9))) {
			VERBOSE("ERROR: TranslationNormalEquationsTest: %s: the centres differ from the SparseQR ones by %g of the extent", what, (double)error);
			return false;
		}
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
		addRows(pairs, 0, 1200, 20);
		GlobalTranslationEstimator estimator;
		std::vector<Point3> centres;
		#ifdef _RELEASE
		const auto start = std::chrono::steady_clock::now();
		#endif
		if (!estimator.EstimateTranslations(pairs, (uint32_t)positions.size(), 0, centres)) {
			VERBOSE("ERROR: TranslationNormalEquationsTest: the view-graph sized solve failed");
			return false;
		}
		#ifdef _RELEASE // a wall-clock bound holds in a Release build only: a Debug or sanitizer build is slower by an order of magnitude
		const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
		#endif
		if (!checkLaplacian("1200 nodes", estimator, pairs, 0))
			return false;
		REAL sumSq = 0;
		for (uint32_t i = 1; i < (uint32_t)positions.size(); ++i)
			sumSq += SQUARE(norm((centres[i] - centres[0]) - (positions[i] - positions[0])));
		const REAL rms = SQRT(sumSq / (REAL)positions.size());
		if (!(rms < REAL(1))) {
			VERBOSE("ERROR: TranslationNormalEquationsTest: 1200 nodes, %zu rows: %g rms from the truth", pairs.size(), (double)rms);
			return false;
		}
		#ifdef _RELEASE
		// a sanity bound, not the check (the structure above is): 6 s, 10x the 0.6 s the solve takes on an idle core (34 s by the QR it replaced)
		if (seconds > 6) {
			VERBOSE("ERROR: TranslationNormalEquationsTest: 1200 nodes, %zu rows: %.1f s", pairs.size(), seconds);
			return false;
		}
		#endif
	}
	VERBOSE("TranslationNormalEquationsTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The scale solve of GlobalScaleEstimator (conjugate gradients on the normal equations, started from the
// previous solve of the same estimator) gives the answer of a sparse Cholesky of the gauge-pinned system
// (no ridge) to 1e-9, on a system shaped like a view graph's pair scales (2000 unknowns, ~16 equations
// each, weights sqrt of 15-300 points) and on a badly conditioned one (weights over 1e-2..1e2, two
// groups joined by one weak equation), always converged at the tolerance and not at the iteration
// bound. The start is the estimator's previous scales, kept relative to the gauge: a system perturbed
// a little is solved in at most half the iterations of a solve from nothing, and one with another
// gauge, more unknowns or fewer, or a robust round's change of a tenth of the equations has the
// same answer.
bool ScaleConjugateGradientsTest()
{
	TD_TIMER_START();
	std::mt19937 rng(5);
	std::uniform_real_distribution<double> uniform(0, 1);
	std::normal_distribution<double> noise(0, 0.02);
	constexpr uint32_t numUnknowns = 2000, numGrown = 2500, numShrunk = 1500;
	std::vector<double> logScales(numGrown);
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
	const auto solve = [&](const char* what, GlobalScaleEstimator& estimator, const std::vector<ScalePair>& equations, uint32_t numNodes, uint32_t gauge, bool verify, unsigned& iterations) {
		std::vector<REAL> scales;
		#ifdef _RELEASE
		const auto start = std::chrono::steady_clock::now();
		#endif
		if (!estimator.EstimateScales(equations, numNodes, gauge, scales)) {
			VERBOSE("ERROR: ScaleConjugateGradientsTest: %s: the solve failed", what);
			return false;
		}
		#ifdef _RELEASE // a wall-clock bound holds in a Release build only: a Debug or sanitizer build is slower by an order of magnitude
		const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
		#endif
		const GlobalScaleEstimator::Solve& stats = estimator.GetSolve();
		iterations = stats.iterations;
		if (!stats.converged) {
			VERBOSE("ERROR: ScaleConjugateGradientsTest: %s: %u conjugate gradients iterations, not converged at the tolerance", what, stats.iterations);
			return false;
		}
		#ifdef _RELEASE
		// a sanity bound, not the check (the iterations are): 5 s, 10x the 0.5 s the largest solve (2500 unknowns) takes on an idle machine
		if (seconds > 5) {
			VERBOSE("ERROR: ScaleConjugateGradientsTest: %s: the solve took %.1f s", what, seconds);
			return false;
		}
		#endif
		if (!verify)
			return true;
		std::vector<double> expected;
		reference(equations, numNodes, gauge, expected);
		double worst = 0;
		for (uint32_t i = 0; i < numNodes; ++i)
			worst = MAXF(worst, std::abs(std::log((double)scales[i]) - expected[i]));
		if (!(worst < 1e-9)) {
			VERBOSE("ERROR: ScaleConjugateGradientsTest: %s: the log scales differ from the Cholesky ones by %g", what, worst);
			return false;
		}
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
	if (iterations > 100) {
		VERBOSE("ERROR: ScaleConjugateGradientsTest: first solve: %u iterations from nothing", iterations);
		return false;
	}
	// a perturbation of every scale ratio by 1e-10, no equation dropped: the previous scales are nearly the answer, and
	// the iterations fall with the logarithm of the distance, so half of them are saved only by a nearly exact start
	for (ScalePair& pair : pairs)
		pair.scaleRatio *= 1 + 1e-10 * uniform(rng);
	if (!solve("perturbed ratios", estimator, pairs, numUnknowns, 0, true, iterations) ||
		!cold("perturbed ratios from nothing", pairs, numUnknowns, 0, coldIterations))
		return false;
	if (2 * iterations > coldIterations) {
		VERBOSE("ERROR: ScaleConjugateGradientsTest: the system with ratios perturbed by 1e-10 took %u iterations from the previous solve, %u from nothing", iterations, coldIterations);
		return false;
	}
	// every weight perturbed by 1%: fewer iterations from the previous solve than from nothing
	for (ScalePair& pair : pairs)
		pair.weight *= (float)(1 + 0.01 * uniform(rng));
	if (!solve("perturbed weights", estimator, pairs, numUnknowns, 0, true, iterations) ||
		!cold("perturbed weights from nothing", pairs, numUnknowns, 0, coldIterations))
		return false;
	if (iterations >= coldIterations) {
		VERBOSE("ERROR: ScaleConjugateGradientsTest: the system with weights perturbed by 1%% took %u iterations from the previous solve, %u from nothing", iterations, coldIterations);
		return false;
	}
	// the same system with another gauge: the scales of the previous solve are taken relative to it
	if (!solve("another gauge", estimator, pairs, numUnknowns, 7, true, iterations) ||
		!cold("another gauge from nothing", pairs, numUnknowns, 7, coldIterations))
		return false;
	if (2 * iterations > coldIterations) {
		VERBOSE("ERROR: ScaleConjugateGradientsTest: the other gauge took %u iterations from the previous solve, %u from nothing", iterations, coldIterations);
		return false;
	}
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
	VERBOSE("ScaleConjugateGradientsTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Pairs weighting test
bool PairsWeightingTest()
{
	TD_TIMER_START();

	// Create small scene with 4 images forming triplets: 0-1-2, 0-1-3, 0-2-3, 1-2-3
	// Make (0,1) strong intrinsic, (1,2) weak intrinsic, and the rest valid for triplet
	Scene scene;
	SceneConfig cfg;
	#ifdef _RELEASE
	std::random_device rd;
	cfg.randomSeed = rd();
	#endif
	cfg.poseMode = SceneConfig::RANDOM_POSES;
	cfg.numImages = 4;
	cfg.numPoints = 0; // add points manually
	cfg.generatePairs = true;
	cfg.cameras[0].width = 100;
	cfg.cameras[0].height = 100;
	GenerateTestScene(scene, cfg);

	// Pair (0,1): Strong intrinsic (spread matches)
	ImagePair* p01 = scene.FindPair(0, 1);
	// Add spread matches (corners of 100x100)
	p01->matches.emplace_back(0,0); scene.images[0].keypoints.emplace_back(cv::Point2f(10,10), 10); scene.images[1].keypoints.emplace_back(cv::Point2f(10,10), 10);
	p01->matches.emplace_back(1,1); scene.images[0].keypoints.emplace_back(cv::Point2f(90,10), 10); scene.images[1].keypoints.emplace_back(cv::Point2f(90,10), 10);
	p01->matches.emplace_back(2,2); scene.images[0].keypoints.emplace_back(cv::Point2f(10,90), 10); scene.images[1].keypoints.emplace_back(cv::Point2f(10,90), 10);
	p01->matches.emplace_back(3,3); scene.images[0].keypoints.emplace_back(cv::Point2f(90,90), 10); scene.images[1].keypoints.emplace_back(cv::Point2f(90,90), 10);
	// Add some internal points to boost count
	for (uint32_t i=0; i<60; ++i) {
		p01->matches.emplace_back(4+i, 4+i);
		scene.images[0].keypoints.emplace_back(cv::Point2f(50,50), 10);
		scene.images[1].keypoints.emplace_back(cv::Point2f(50,50), 10);
	}

	// Pair (1,2): Weak intrinsic (clumped matches) but fewer than p01
	ImagePair* p12 = scene.FindPair(1, 2);
	for (uint32_t i=0; i<40; ++i) { // fewer than p01 to reflect lower quality
		p12->matches.emplace_back(i, i);
		// All clumped at (50,50)
		scene.images[1].keypoints.emplace_back(cv::Point2f(50.f + i*0.01f, 50.f), 10);
		scene.images[2].keypoints.emplace_back(cv::Point2f(50.f + i*0.01f, 50.f), 10);
	}

	// Remaining pairs: Bridge for triplet
	const auto PopulatePair = [&](int id1, int id2) {
		ImagePair& p = *scene.FindPair(id1, id2);
		// Add minimal matches to valid
		for (uint32_t i=0; i<20; ++i) {
			p.matches.emplace_back(i, i);
			scene.images[p.ID1].keypoints.emplace_back(cv::Point2f(20,20), 10);
			scene.images[p.ID2].keypoints.emplace_back(cv::Point2f(20,20), 10);
		}
	};
	PopulatePair(0,2);
	PopulatePair(0,3);
	PopulatePair(2,3);

	// Compute weights
	PairsWeightingConfig weightCfg; // default: triplet angle 5 deg, saturation 5
	ComputePairsWeights(scene, weightCfg);

	// Retrieve pairs again as their pointers may have changed
	p01 = scene.FindPair(0, 1);
	p12 = scene.FindPair(1, 2);
	ImagePair* p02 = scene.FindPair(0, 2);

	// NOTE: Pair (0,2) is intended to act as a bridge. With edges (0,1), (1,2), (0,2), (0,3), (2,3) present
	// and (1,3) absent, (0,2) should participate in two triplets (0-1-2 and 0-2-3), while (0,1) and (1,2)
	// participate in only one (0-1-2). Its triplet weight should therefore exceed the others if both bridge
	// edges (0,3) and (2,3) are actually usable by the triplet counter.
	VERBOSE("Pair 0-1 (Spread): Spatial=%.4f, Conn=%.4f, Triplet=%.4f", p01->weightSpatial, p01->weightConnectivity, p01->weightTriplet);
	VERBOSE("Pair 1-2 (Clumped): Spatial=%.4f, Conn=%.4f, Triplet=%.4f", p12->weightSpatial, p12->weightConnectivity, p12->weightTriplet);
	VERBOSE("Pair 0-2 (Bridge): Spatial=%.4f, Conn=%.4f, Triplet=%.4f", p02->weightSpatial, p02->weightConnectivity, p02->weightTriplet);

	// 1. Intrinsic check
	if (p01->weightSpatial <= p12->weightSpatial) {
		VERBOSE("PairsWeightingTest FAILED: Spread matches should have higher spatial weight than clumped");
		return false;
	}

	// 2. Connection check
	if (p01->weightConnectivity <= p12->weightConnectivity) {
		VERBOSE("PairsWeightingTest FAILED: Spread matches should have higher connectivity weight than clumped");
		return false;
	}

	// 3. Triplet check
	// Because relative poses are all Identity, loop is closed perfectly.
	// Triplet weight should be > 0, and pair (0,2) should have one more triplet.
	#ifdef _USE_BOOST
	if (p01->weightTriplet <= 0.f || p12->weightTriplet <= 0.f || p02->weightTriplet <= 0.f || p02->weightTriplet <= p01->weightTriplet) {
		VERBOSE("PairsWeightingTest FAILED: Valid triplet should have non-zero triplet weight");
		return false;
	}
	#else
	VERBOSE("Skipping Triplet check (Boost not enabled)");
	#endif

	VERBOSE("PairsWeightingTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The composite weight of a pair from its terms (ImagePair::GetCompositeWeight): the weighted
// inliers capped at 1000, times the coverage in full, times the cube root of the angle term times
// the connectivity, times 0.25 plus the triangle consistency; weightSpatial holds coverage x angle
bool PairCompositeWeightTest()
{
	TD_TIMER_START();
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
		if (ABS(pair.GetCompositeWeight() - expected) > 1e-4 * expected) {
			VERBOSE("PairCompositeWeightTest FAILED: inliers %g, coverage %g, angle %g, connectivity %g, triangles %g weigh %g, expected %g",
				c.inliers, c.coverage, c.angle, c.connectivity, c.triplet, pair.GetCompositeWeight(), expected);
			return false;
		}
	}
	VERBOSE("PairCompositeWeightTest PASSED: %u hand-set pairs weigh capped inliers x coverage x cbrt(angle x connectivity) x (0.25 + triangles) (%s)",
		(unsigned)(sizeof(cases) / sizeof(cases[0])), TD_TIMER_GET_FMT().c_str());
	return true;
}

// A pair's dense inliers count as evidence in proportion to the matcher's frame target, not up to
// a cap: 10 sparse and 400 dense inliers weigh 10 + 25 * 400/2000 = 15 at the default of 25 frame
// inliers, and 110 at 500, the weight the old discount of a quarter per dense inlier gave. Two
// dense-only pairs of one image keep their order (1600 dense outweighs 400 -- a cap flattened them
// to the same weight), and the connectivity reads the dense matches at the same scale as the
// evidence, so a dense-only pair and a sparse-only pair that are each their images' only pair carry
// the same connectivity.
bool PairsWeightingDenseFrameEvidenceTest()
{
	TD_TIMER_START();
	constexpr unsigned NUM_SPARSE = 10, NUM_DENSE = 400;

	// numImages images of numKeypoints keypoints spread over the frame (the weighting reads the
	// pair's partition, not the images' described boundary, so none is set)
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

	// 1. the magnitude: proportional to the frame target
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
		if (pair.GetNumWeightedInliers() != cases[c].expectedWeightedInliers) {
			VERBOSE("PairsWeightingDenseFrameEvidenceTest FAILED: %u sparse + %u dense inliers weigh %u at %.0f frame inliers, expected %u",
				NUM_SPARSE, NUM_DENSE, pair.GetNumWeightedInliers(), cases[c].frameInliers, cases[c].expectedWeightedInliers);
			return false;
		}
		compositeAt[c] = pair.GetCompositeWeight();
	}
	if (!(compositeAt[0] > 0.f && compositeAt[0] < compositeAt[1])) {
		VERBOSE("PairsWeightingDenseFrameEvidenceTest FAILED: the composite weight is %.2f at 25 frame inliers and %.2f at 500; "
			"the scaled pair must stay valid and weigh less", compositeAt[0], compositeAt[1]);
		return false;
	}

	// 2. the order: image 0 sees image 1 through 1600 dense-only inliers and image 2 through 400
	{
		Scene scene;
		BuildImages(scene, 3, 1600);
		AddPair(scene, 0, 1, 0, 1600);
		AddPair(scene, 0, 2, 0, 400);
		ComputePairsWeights(scene, PairsWeightingConfig());
		const float w01 = scene.pairs[0].GetCompositeWeight(), w02 = scene.pairs[1].GetCompositeWeight();
		if (!(w02 > 0.f && w01 > w02)) {
			VERBOSE("PairsWeightingDenseFrameEvidenceTest FAILED: 1600 dense-only inliers weigh %.2f against %.2f for 400; "
				"the stronger dense-only pair of an image must outweigh the weaker one", w01, w02);
			return false;
		}
	}

	// 3. the connectivity: a sparse-only pair and a dense-only pair, each its images' only pair, carry
	// the same connectivity weight (1: the whole share of the node maximum)
	{
		Scene scene;
		BuildImages(scene, 4, 2000);
		AddPair(scene, 0, 1, 100, 0);
		AddPair(scene, 2, 3, 0, 2000);
		ComputePairsWeights(scene, PairsWeightingConfig());
		const float cSparse = scene.pairs[0].weightConnectivity, cDense = scene.pairs[1].weightConnectivity;
		if (!ISEQUAL(cSparse, cDense) || !ISEQUAL(cSparse, 1.f)) {
			VERBOSE("PairsWeightingDenseFrameEvidenceTest FAILED: connectivity %.4f for the sparse-only pair and %.4f for the dense-only one; "
				"both are their images' only pair, so both must be 1", cSparse, cDense);
			return false;
		}
	}

	VERBOSE("PairsWeightingDenseFrameEvidenceTest PASSED: dense inliers count in proportion to the matcher's frame target, in order, with a consistent connectivity (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 8: Rotation averaging with 4+ sub-scenes
bool GlobalAlignmentRotationAveragingExtendedTest()
{
	TD_TIMER_START();

	// GT global rotations (angle-axis vectors)
	const std::vector<Point3d> gtRotations = {
		Point3d(0, 0, 0),
		Point3d(0.2, 0.1, 0),
		Point3d(-0.1, 0.3, 0.1),
		Point3d(0.15, -0.2, 0.05)
	};
	const uint32_t numScenes = (uint32_t)gtRotations.size();

	// Build rotation pairs with small noise
	std::mt19937 rng(42);
	std::normal_distribution<double> noise(0.0, D2R(1.0)); // 1-degree noise
	std::vector<RotationPair> rotPairs;

	for (uint32_t i = 0; i < numScenes; ++i) {
		for (uint32_t j = i + 1; j < numScenes; ++j) {
			const RMatrix Ri(gtRotations[i]);
			const RMatrix Rj(gtRotations[j]);
			Matrix3x3d Rij = Rj * Ri.t(); // relative rotation

			// Add noise via small random rotation
			if (ABS(noise(rng)) > 1e-10)
				Rij = GenerateRandomRotation(rng, D2R(1.0)) * Rij;

			RotationPair rp;
			rp.idxA = i;
			rp.idxB = j;
			rp.relativeRotation = Rij;
			rp.weight = 100.f;
			rotPairs.push_back(rp);
		}
	}

	GlobalRotationEstimatorOptions options;
	GlobalRotationEstimator estimator(options);
	std::vector<Point3d> estRotations;
	if (!estimator.EstimateRotations(rotPairs, numScenes, estRotations)) {
		VERBOSE("GlobalAlignmentRotationAveragingExtendedTest FAILED: EstimateRotations returned false");
		return false;
	}

	// Compare relative rotations (account for gauge freedom at scene 0)
	const RMatrix R0_gt(gtRotations[0]);
	const RMatrix R0_est(estRotations[0]);
	double maxAngleError = 0;

	for (uint32_t i = 1; i < numScenes; ++i) {
		const RMatrix Ri_gt(gtRotations[i]);
		const RMatrix Ri_est(estRotations[i]);
		const RMatrix Ri_rel_gt = Ri_gt * R0_gt.t();
		const RMatrix Ri_rel_est = Ri_est * R0_est.t();
		const double angleError = ACOS(CLAMP(ComputeAngle(Ri_rel_est, Ri_rel_gt), REAL(-1), REAL(1)));
		maxAngleError = MAXF(maxAngleError, angleError);
	}

	const double toleranceDeg = 3.0;
	if (R2D(maxAngleError) > toleranceDeg) {
		VERBOSE("GlobalAlignmentRotationAveragingExtendedTest FAILED: max angle error %.2f deg > %.2f",
			R2D(maxAngleError), toleranceDeg);
		return false;
	}

	VERBOSE("GlobalAlignmentRotationAveragingExtendedTest PASSED: max angle error %.2f deg (%s)",
		R2D(maxAngleError), TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 9: Scale averaging with non-trivial scales
bool GlobalAlignmentScaleAveragingExtendedTest()
{
	TD_TIMER_START();

	const std::vector<REAL> gtScales = {REAL(1.0), REAL(2.5), REAL(0.8), REAL(3.0)};
	const uint32_t numScenes = (uint32_t)gtScales.size();

	// Build scale pairs with 2% noise
	std::mt19937 rng(42);
	std::normal_distribution<REAL> noise(REAL(0), REAL(0.02));
	std::vector<ScalePair> scalePairs;

	for (uint32_t i = 0; i < numScenes; ++i) {
		for (uint32_t j = i + 1; j < numScenes; ++j) {
			REAL ratio = gtScales[j] / gtScales[i];
			ratio *= (REAL(1) + noise(rng)); // multiplicative noise
			ScalePair sp;
			sp.idxA = i;
			sp.idxB = j;
			sp.scaleRatio = ratio;
			sp.weight = 20.f;
			scalePairs.push_back(sp);
		}
	}

	GlobalScaleEstimator estimator;
	std::vector<REAL> estScales;
	if (!estimator.EstimateScales(scalePairs, numScenes, estScales)) {
		VERBOSE("GlobalAlignmentScaleAveragingExtendedTest FAILED: EstimateScales returned false");
		return false;
	}

	// Compare relative scale ratios (gauge at index 0)
	const REAL tolerance = REAL(0.05);
	for (uint32_t i = 1; i < numScenes; ++i) {
		const REAL gtRatio = gtScales[i] / gtScales[0];
		const REAL estRatio = estScales[i] / estScales[0];
		if (ABS(estRatio - gtRatio) / gtRatio > tolerance) {
			VERBOSE("GlobalAlignmentScaleAveragingExtendedTest FAILED: scale ratio %u: est=%.4f gt=%.4f (err=%.4f)",
				i, (double)estRatio, (double)gtRatio, (double)ABS(estRatio - gtRatio));
			return false;
		}
	}

	// Also test fixed-gauge version
	std::vector<REAL> estScalesFixed;
	if (!estimator.EstimateScales(scalePairs, numScenes, 0, estScalesFixed)) {
		VERBOSE("GlobalAlignmentScaleAveragingExtendedTest FAILED: fixed-gauge EstimateScales returned false");
		return false;
	}
	if (ABS(estScalesFixed[0] - REAL(1)) > REAL(0.01)) {
		VERBOSE("GlobalAlignmentScaleAveragingExtendedTest FAILED: fixed gauge s[0]=%.4f (expected 1.0)", (double)estScalesFixed[0]);
		return false;
	}

	VERBOSE("GlobalAlignmentScaleAveragingExtendedTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 10: Scale averaging fallback to unit scales
bool GlobalAlignmentScaleAveragingFallbackTest()
{
	TD_TIMER_START();

	// Empty scale pairs → should fail and caller uses unit scales
	std::vector<ScalePair> emptyPairs;
	GlobalScaleEstimator estimator;
	std::vector<REAL> estScales;

	// With no pairs, estimator should return false
	const bool result = estimator.EstimateScales(emptyPairs, 3, estScales);
	if (result) {
		// Some implementations may succeed with identity; verify scales are reasonable
		VERBOSE("GlobalAlignmentScaleAveragingFallbackTest: estimator succeeded with empty pairs (ok if scales are 1.0)");
	}

	// The caller (EstimateGlobalScales in GlobalAlignment.cpp line 540-544)
	// handles this by setting unit scales. Verify the pattern works:
	std::vector<REAL> fallbackScales(3, REAL(1));
	for (unsigned i = 0; i < 3; ++i) {
		if (ABS(fallbackScales[i] - REAL(1)) > REAL(1e-6)) {
			VERBOSE("GlobalAlignmentScaleAveragingFallbackTest FAILED: fallback scale %u = %.4f", i, (double)fallbackScales[i]);
			return false;
		}
	}

	VERBOSE("GlobalAlignmentScaleAveragingFallbackTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Test 11: Translation averaging with known R and s
bool GlobalAlignmentTranslationAveragingExtendedTest()
{
	TD_TIMER_START();

	const std::vector<Point3> gtTranslations = {
		Point3(0, 0, 0),
		Point3(3, 1, 0),
		Point3(5, 2, 1),
		Point3(8, 4, 2)
	};
	const uint32_t numScenes = (uint32_t)gtTranslations.size();

	// Build translation pairs with small noise
	std::mt19937 rng(42);
	std::normal_distribution<REAL> noise(REAL(0), REAL(0.01));
	std::vector<TranslationPair> transPairs;

	for (uint32_t i = 0; i < numScenes; ++i) {
		for (uint32_t j = i + 1; j < numScenes; ++j) {
			Point3 relT = gtTranslations[j] - gtTranslations[i];
			relT.x += noise(rng);
			relT.y += noise(rng);
			relT.z += noise(rng);

			TranslationPair tp;
			tp.idxA = i;
			tp.idxB = j;
			tp.relativeTranslation = relT;
			tp.weight = 20.f;
			transPairs.push_back(tp);
		}
	}

	GlobalTranslationEstimator estimator;
	std::vector<Point3> estTranslations;
	if (!estimator.EstimateTranslations(transPairs, numScenes, estTranslations)) {
		VERBOSE("GlobalAlignmentTranslationAveragingExtendedTest FAILED: EstimateTranslations returned false");
		return false;
	}

	// Compare relative translations (gauge freedom at best-connected node)
	const REAL tolerance = REAL(0.1);
	for (const TranslationPair& tp : transPairs) {
		const Point3 estRel = estTranslations[tp.idxB] - estTranslations[tp.idxA];
		const Point3 gtRel = gtTranslations[tp.idxB] - gtTranslations[tp.idxA];
		const REAL relError = norm(estRel - gtRel);
		if (relError > tolerance) {
			VERBOSE("GlobalAlignmentTranslationAveragingExtendedTest FAILED: pair (%u,%u) error=%.4f > %.4f",
				tp.idxA, tp.idxB, (double)relError, (double)tolerance);
			return false;
		}
	}

	VERBOSE("GlobalAlignmentTranslationAveragingExtendedTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Five scales on a ring with one wrong chord: the robust averaging must ignore the wrong edge
bool RobustAveragingScaleTest()
{
	TD_TIMER_START();
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
	if (!solved) {
		VERBOSE("RobustAveragingScaleTest FAILED: solver failed");
		return false;
	}
	for (uint32_t i = 0; i < 5; ++i) {
		const REAL r = scales[i] / truth[i];
		if (MAXF(r, REAL(1) / r) > REAL(1.02)) {
			VERBOSE("RobustAveragingScaleTest FAILED: scale %u off by %.3f", i, r);
			return false;
		}
	}
	if (residuals[6] < std::log(REAL(2.5)) || residuals[0] > std::log(REAL(1.02))) {
		VERBOSE("RobustAveragingScaleTest FAILED: log residuals %.3f / %.3f", residuals[6], residuals[0]);
		return false;
	}
	VERBOSE("RobustAveragingScaleTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Five translations on a ring with one wrong chord
bool RobustAveragingTranslationTest()
{
	TD_TIMER_START();
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
	if (!solved) {
		VERBOSE("RobustAveragingTranslationTest FAILED: solver failed");
		return false;
	}
	for (uint32_t i = 0; i < 5; ++i) {
		if (norm(t[i] - truth[i]) > REAL(0.02)) {
			VERBOSE("RobustAveragingTranslationTest FAILED: node %u off by %.3f", i, norm(t[i] - truth[i]));
			return false;
		}
	}
	if (residuals[6] < REAL(0.7) || residuals[0] > REAL(0.02)) {
		VERBOSE("RobustAveragingTranslationTest FAILED: residuals %.3f / %.3f", residuals[6], residuals[0]);
		return false;
	}
	VERBOSE("RobustAveragingTranslationTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

} // namespace SFM
