/*
 * TestsSFMBlock.cpp
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

// Triplet star-initialization test: 3-view scene with tracks + StarInitializer + BA
bool TripletStarInitTest()
{
	TD_TIMER_START();
	std::mt19937 rng(123);

	// Generate synthetic scene
	Scene sceneGT, scene;
	SceneConfig cfg;
	std::uniform_real_distribution<REAL> kDist(-0.1, 0.1);
	cfg.cameras.front().k1 = kDist(rng);
	cfg.cameras.front().k2 = kDist(rng);
	if (cfg.cameras.front().k1 * cfg.cameras.front().k2 > 0)
		cfg.cameras.front().k2 *= -1; // ensure k1 and k2 have different signs
	cfg.numImages = 3;
	cfg.numPoints = 300;
	cfg.poseMode = SceneConfig::RANDOM_POSES;
	cfg.generateDescriptors = true;
	cfg.generatePairs = true; // Automatically create image pairs with matches
	cfg.perturbOptions = SceneConfig::PERTURB_ALL;
	GenerateTestScene(sceneGT, cfg, &scene);

	// Allow BA to refine intrinsics
	const PinholeCamera& gt_camera = *static_cast<PinholeCamera*>(sceneGT.cameras[0]);
	PinholeCamera& cam = *static_cast<PinholeCamera*>(scene.cameras[0]);
	cam.trustIntrinsics = false;
	DEBUG("TripletStarInitTest: Ground-truth camera: f=%.2f, k1=%.6f, k2=%.6f",
		gt_camera.fx, gt_camera.k1, gt_camera.k2);

	// Test triangulation (using GT poses first to verify)
	const unsigned numInlierTracks = TriangulateTracks(scene, false, 8, 0.5f);
	if (numInlierTracks+25 < sceneGT.tracks.size() || norm(sceneGT.tracks[0].position - scene.tracks[0].position) > 1.0) {
		VERBOSE("TripletStarInitTest: triangulate points failed (num=%u vs %u, err=%.4f)",
			numInlierTracks, (unsigned)sceneGT.tracks.size(), norm(sceneGT.tracks[0].position - scene.tracks[0].position));
		return false;
	}
	scene.tracks.clear(); // Clear tracks to let StarInitializer rebuild them

	// Randomly scale the translation to simulate unknown baselines
	std::uniform_real_distribution<REAL> scaleDist(0.5, 2.0);
	for (ImagePair& pair : scene.pairs)
		if (pair.relativePose)
			pair.relativePose->C *= scaleDist(rng);

	// Invalidate view poses (StarInitializer will reconstruct them)
	scene.images[0].InvalidatePose();
	scene.images[1].InvalidatePose();
	scene.images[2].InvalidatePose();

	// Build tracks in sub-scene
	PairsWeightingConfig weightCfg; // defaults
	ComputePairsWeights(scene, weightCfg);
	BuildTracks(scene, -1.f);
	if (scene.tracks.empty()) {
		VERBOSE("TripletStarInitTest: BuildTracks produced zero tracks");
		return false;
	}

	// Star initialization (reference will be center with connectivity 2)
	StarInitConfig initCfg; // defaults (refineFocalLength = true)
	initCfg.minViews = 3;
	if (!StarInitializer::Initialize(scene, initCfg)) {
		VERBOSE("TripletStarInitTest: StarInitializer failed");
		return false;
	}
	DEBUG("TripletStarInitTest: Initialized triplet with %u triangulated tracks",
		(unsigned)scene.tracks.size())

	// Verify the star recovers the focal on its own: step 7 of Initialize() refines only the
	// focal length, so this is everything the star itself is responsible for
	const REAL focalErr = ABS(cam.fx - gt_camera.fx) / gt_camera.fx;
	DEBUG("TripletStarInitTest: Refined camera: f=%.2f (err=%.2f%%), k1=%.6f, k2=%.6f",
		cam.fx, focalErr * 100, cam.k1, cam.k2);
	if (focalErr > 0.05) { // Allow 5% focal error
		VERBOSE("TripletStarInitTest: focal length error too large (%.2f%%)", focalErr * 100);
		return false;
	}

	// The star leaves distortion to the global bundle adjustments the resection runs once the
	// model is large enough to constrain it; reproduce the one it runs right after
	// (ResectionConfig::DeriveBAConfigs) and check the distortion recovery that BA is actually
	// responsible for
	BAConfig fullBaConfig;
	fullBaConfig.RefineMainIntrinsics();
	if (!BundleAdjustment::Adjust(scene, fullBaConfig)) {
		VERBOSE("TripletStarInitTest: post-star bundle adjustment with intrinsics refinement failed");
		return false;
	}
	const REAL k1Err = ABS(cam.k1 - gt_camera.k1);
	const REAL k2Err = ABS(cam.k2 - gt_camera.k2);
	DEBUG("TripletStarInitTest: Post-star BA distortion: k1=%.6f (err=%.6f), k2=%.6f (err=%.6f)",
		cam.k1, k1Err, cam.k2, k2Err);
	if (k1Err > 0.01 || k2Err > 0.01) { // Allow 0.01 absolute error in distortion
		VERBOSE("TripletStarInitTest: distortion error too large (k1_err=%.6f, k2_err=%.6f)", k1Err, k2Err);
		return false;
	}

	// GenerateTestScene's perturbation mask has no entry for k3 (ground truth k3 is 0), so perturb
	// it by hand: a further main-set solve must leave it alone (k3 is not in the main set), and only
	// a following extended-set solve, which does refine it, must bring it back near the truth
	cam.k3 = 0.05;
	BAConfig mainBaConfig;
	mainBaConfig.RefineMainIntrinsics();
	if (!BundleAdjustment::Adjust(scene, mainBaConfig)) {
		VERBOSE("TripletStarInitTest: post-star bundle adjustment with a perturbed k3 failed");
		return false;
	}
	if (cam.k3 != 0.05) {
		VERBOSE("TripletStarInitTest: k3 changed by the main-set solve (0.05 -> %.9g)", cam.k3);
		return false;
	}
	BAConfig extendedBaConfig = mainBaConfig;
	extendedBaConfig.RefineExtendedIntrinsics();
	if (!BundleAdjustment::Adjust(scene, extendedBaConfig)) {
		VERBOSE("TripletStarInitTest: post-star extended bundle adjustment failed");
		return false;
	}
	const REAL k3Err = ABS(cam.k3 - gt_camera.k3);
	DEBUG("TripletStarInitTest: Post-star extended BA distortion: k3=%.6f (err=%.6f)", cam.k3, k3Err);
	if (k3Err > 0.01) { // Allow 0.01 absolute error in distortion
		VERBOSE("TripletStarInitTest: k3 error too large (err=%.6f)", k3Err);
		return false;
	}

	// A forced/known focal (StarInitConfig::refineFocalLength = false) stays fixed through the
	// star's own bundle adjustment: construct the camera already at the ground-truth focal, as a
	// forced value would be at import, and check the exact bit pattern survives rather than a
	// tolerance -- the star must never touch it. Distortion stays untouched too: nothing in step 7
	// refines it, fixed focal or not.
	Scene sceneGT2, scene2;
	SceneConfig cfg2 = cfg;
	cfg2.perturbOptions = SceneConfig::PERTURB_ALL & ~SceneConfig::PERTURB_INTRINSICS;
	GenerateTestScene(sceneGT2, cfg2, &scene2);
	PinholeCamera& cam2 = *static_cast<PinholeCamera*>(scene2.cameras[0]);
	cam2.trustIntrinsics = false;
	const REAL fixedFocal = cam2.fx;
	const REAL fixedK1 = cam2.k1;
	const REAL fixedK2 = cam2.k2;

	scene2.tracks.clear(); // let StarInitializer rebuild them, same as the first case above
	for (ImagePair& pair : scene2.pairs)
		if (pair.relativePose)
			pair.relativePose->C *= scaleDist(rng); // unknown baselines, same as the first case above
	scene2.images[0].InvalidatePose();
	scene2.images[1].InvalidatePose();
	scene2.images[2].InvalidatePose();
	ComputePairsWeights(scene2, weightCfg);
	BuildTracks(scene2, -1.f);
	if (scene2.tracks.empty()) {
		VERBOSE("TripletStarInitTest: BuildTracks produced zero tracks (fixed-focal case)");
		return false;
	}
	StarInitConfig initCfg2;
	initCfg2.minViews = 3;
	initCfg2.refineFocalLength = false;
	if (!StarInitializer::Initialize(scene2, initCfg2)) {
		VERBOSE("TripletStarInitTest: StarInitializer failed (fixed-focal case)");
		return false;
	}
	if (cam2.fx != fixedFocal) {
		VERBOSE("TripletStarInitTest: fixed focal changed (%.9g -> %.9g)", fixedFocal, cam2.fx);
		return false;
	}
	if (cam2.k1 != fixedK1 || cam2.k2 != fixedK2) {
		VERBOSE("TripletStarInitTest: fixed-focal case distortion changed (k1 %.9g -> %.9g, k2 %.9g -> %.9g)",
			fixedK1, cam2.k1, fixedK2, cam2.k2);
		return false;
	}

	VERBOSE("TripletStarInitTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The star initializer's reference view is chosen among the seed views the caller names -- the
// triplet filter's largest ceiling piece -- and only among every image when none is named, or
// none of the named has a valid pair. After the filter, the seed's side of a symmetric building
// becomes the model, and the heaviest image overall sits in the densest cluster of look-alike
// views, which is the worst place to start.
bool StarReferenceViewTest()
{
	TD_TIMER_START();
	// Ten images. A triangle (0-2) joined by 3000-inlier pairs: 6000 weighted inliers each, the
	// heaviest in the scene. A chain (3-8): 500-inlier consecutive pairs, 300-inlier pairs two
	// apart, and one 100-inlier pair (5,8), so that image 5 (1700) outweighs 6 (1600), 4 and 7
	// (1300), 8 (900) and 3 (800). Image 9 has no pair at all.
	// A candidate also needs enough valid pairs to centre a star, since the initializer refuses a
	// star smaller than its minimum.
	Scene scene;
	AddTripletImages(scene, 10);
	const auto addPair = [&scene](IIndex a, IIndex b, unsigned numInliers) {
		AddTripletPair(scene, a, b, numInliers);
		ImagePair& pair = scene.pairs.back();
		pair.relativePose = Pose3D::Identity();
		pair.weightSpatial = 1.f;
		pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
		pair.weightConnectivity = 1.f;
	};
	addPair(0, 1, 3000);
	addPair(1, 2, 3000);
	addPair(0, 2, 3000);
	for (IIndex i = 3; i + 1 < 9; ++i)
		addPair(i, i + 1, 500);
	for (IIndex i = 3; i + 2 < 9; ++i)
		addPair(i, i + 2, 300);
	addPair(5, 8, 100);
	const IIndex refAll = StarInitializer::SelectReferenceView(scene, IIndexArr(), 2, PairsWeightingConfig());
	if (refAll != 0) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u with no seed views; expected 0, the heaviest image", refAll);
		return false;
	}
	IIndexArr chain;
	for (IIndex i = 3; i < 9; ++i)
		chain.push_back(i);
	const IIndex refChain = StarInitializer::SelectReferenceView(scene, chain, 2, PairsWeightingConfig());
	if (refChain != 5) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u among the chain's images; expected 5, the heaviest "
			"of them, not an image of the heavier triangle", refChain);
		return false;
	}
	IIndexArr lonely;
	lonely.push_back(9);
	const IIndex refLonely = StarInitializer::SelectReferenceView(scene, lonely, 2, PairsWeightingConfig());
	if (refLonely != 0) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u with a seed view that has no valid pair; expected "
			"the fallback to every image, 0", refLonely);
		return false;
	}
	// A seed view with two pairs cannot centre a star of three arms: image 3 is skipped, and so are
	// the triangle's images (two pairs each) when the choice falls to every image; the reference is
	// the heaviest image with three or more pairs, 5: a seed piece whose heaviest image holds two
	// pairs must not stall the star initializer.
	IIndexArr three;
	three.push_back(3);
	const IIndex refThree = StarInitializer::SelectReferenceView(scene, three, 3, PairsWeightingConfig());
	if (refThree != 5) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u with a seed view of two pairs and a star of three arms "
			"required; expected 5, the heaviest image with three or more pairs", refThree);
		return false;
	}
	// No image has six pairs: the heaviest image is returned and the initializer reports the shortfall.
	const IIndex refSix = StarInitializer::SelectReferenceView(scene, IIndexArr(), 6, PairsWeightingConfig());
	if (refSix != 0) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u with six arms required and no image having them; "
			"expected 0, the heaviest image", refSix);
		return false;
	}
	VERBOSE("StarReferenceViewTest PASSED: the reference view is the heaviest seed view with enough pairs to centre a star, "
		"and the heaviest such image when none is named or usable (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The star initializer lists, for the reference view, only the neighbours whose matches fix their
// distance: the pair's inliers whose ray angle reaches the reconstruction's triangulation bar
// (StarInitConfig::evidenceMinAngle), not all of them. Image 0 is the reference. Five near views
// (1-5, 0.5-1.5 units to the side) see 200 points at depth 4-8, a ray angle of 3-20 deg, and 500 far
// points at depth 400-800 (under 0.2 deg); six far views (6-11, 3-6 units to the side, 0.4-0.9 deg
// of the far points) see the far points alone. Every pair has the unit baseline a relative pose has,
// so a distance is known only to a view whose pair triangulates the near points. By inlier count a
// far view (500) is within the star's ratio of a near one (700) and would enter it at the distance
// its unit baseline guesses; by evidence it is left out, and the near views' distances to the
// reference agree. Without the near points no pair has any evidence and there is no star, and the
// star says so ("insufficient initial views"). The keyframe extractor's configuration (the defaults
// and three views) builds the same star.
// The floor of the star (StarInitConfig::ratioInliersFilter, 0.2) is a share of the mean evidence of
// the three strongest candidates, compared with each candidate's evidence; a floor that keeps fewer
// neighbours than the star needed goes on with the neighbours it keeps. With the near/far scene above
// a ratio of 0.98 (a floor of 189 over the evidence 199, 192, 188, 184, 179) keeps the first two near
// views, fewer than the three the star needs, and the star is built from them.
// A burst of near-duplicate frames (1-3, 0.02-0.04 units to the side) shares 2500 far points and a
// dozen points under a unit from the reference with it, 3012 inliers a pair of which only the dozen
// measure a distance (evidence 11-12), and four wider views (4-7, 0.8-1.5 units) see 200 near points
// and 500 far ones, 679-692 inliers of which 179-192 are evidence. The floor, 0.2 of the mean evidence
// of the three strongest (38), keeps the four wider views and leaves the burst out, where a floor of
// 0.2 of the 3012 inliers (602) would keep none.
bool StarEvidenceTest()
{
	TD_TIMER_START();
	const std::vector<REAL> nearSides{0.5, 0.8, 1.0, 1.2, 1.5};
	const std::vector<REAL> farSides{3.0, 3.6, 4.2, 4.8, 5.4, 6.0};
	const unsigned numNear = (unsigned)nearSides.size(), numFar = (unsigned)farSides.size();
	// the points of one depth range and the views that see them (the reference sees every point)
	struct PointGroup {
		unsigned count;
		REAL minDepth, maxDepth;
		std::function<bool(IIndex)> seenBy;
	};
	REAL cameraScale = 1; // the camera's pixels per unit: 2 is the same field of view on twice the pixels
	REAL lateralShare = 0.3; // the views' offset along y, as a share of their offset along x, on odd views
	const auto buildScene = [&](Scene& scene, const std::vector<REAL>& sides, const std::vector<PointGroup>& groups) {
		PinholeCamera* pCamera = new PinholeCamera(cv::Size(ROUND2INT(640 * cameraScale), ROUND2INT(480 * cameraScale)), 400 * cameraScale, 400 * cameraScale, 320 * cameraScale, 240 * cameraScale);
		pCamera->trustIntrinsics = true;
		scene.cameras.emplace_back(pCamera);
		FOREACH(i, sides) {
			Pose3D pose;
			pose.R = Matrix3x3::IDENTITY;
			pose.C = Point3(sides[i], (i % 2) * sides[i] * lateralShare, 0);
			scene.images.emplace_back(i, "", pose, 0, scene.cameras[0]);
		}
		std::mt19937 rng(7);
		std::uniform_int_distribution<int> pixelX(ROUND2INT(40 * cameraScale), ROUND2INT(600 * cameraScale)), pixelY(ROUND2INT(40 * cameraScale), ROUND2INT(440 * cameraScale));
		for (const PointGroup& group : groups) {
			std::uniform_real_distribution<REAL> depth(group.minDepth, group.maxDepth);
			for (unsigned p = 0; p < group.count; ++p) {
				const Point3 X = scene.images[0].UnprojectPoint(Point2(pixelX(rng), pixelY(rng)), depth(rng));
				std::vector<std::pair<Image*, Point2>> seen;
				for (Image& img : scene.images) {
					if (img.ID != 0 && !group.seenBy(img.ID))
						continue;
					const auto [proj, valid] = img.ProjectPoint(X);
					if (!valid || !Image8U::isInside(proj, img.GetSize()))
						continue;
					seen.emplace_back(&img, proj);
				}
				// a track holds at least two observations: a point only the reference sees makes none
				if (seen.size() < 2)
					continue;
				Track track(X);
				for (const auto& [img, proj] : seen) {
					img->keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
					track.observations.emplace_back(img->ID, (uint32_t)img->keypoints.size() - 1);
				}
				track.numInliers = (uint8_t)track.observations.size();
				scene.tracks.emplace_back(std::move(track));
			}
		}
		for (IIndex j = 1; j < scene.images.size(); ++j) {
			ImagePair& pair = scene.pairs.emplace_back(0, j);
			for (const Track& track : scene.tracks) {
				uint32_t obs0 = NO_ID, obsJ = NO_ID;
				for (const auto& obs : track.observations) {
					if (obs.imageID == 0)
						obs0 = obs.featureID;
					else if (obs.imageID == j)
						obsJ = obs.featureID;
				}
				if (obs0 != NO_ID && obsJ != NO_ID)
					pair.matches.emplace_back(obs0, obsJ);
			}
			pair.numFilteredInliers = (int)pair.matches.size();
			pair.relativePose = scene.images[j] / scene.images[0];
			pair.relativePose->C /= norm(pair.relativePose->C); // a relative pose has no scale
			pair.E = ImagePair::ComposeEssentialMatrix(pair.relativePose.value());
			pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), scene.images[0].GetK(), scene.images[j].GetK());
			pair.weightSpatial = 1.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 1.f;
		}
		scene.status.nCalibratedImages = scene.images.size();
	};
	const auto runStar = [&](Scene& scene, const std::vector<REAL>& sides, const std::vector<PointGroup>& groups, const StarInitConfig& cfg) {
		buildScene(scene, sides, groups);
		for (Image& img : scene.images)
			img.InvalidatePose();
		return StarInitializer::Initialize(scene, cfg);
	};
	std::vector<REAL> sides{0};
	sides.insert(sides.end(), nearSides.begin(), nearSides.end());
	sides.insert(sides.end(), farSides.begin(), farSides.end());
	const PointGroup nearPoints{200, 4, 8, [numNear](IIndex v) { return v <= numNear; }};
	const PointGroup farPoints{500, 400, 800, [](IIndex) { return true; }};
	// the truth: each view's distance to the reference
	Scene truth;
	buildScene(truth, sides, {nearPoints, farPoints});
	std::vector<REAL> trueDist(truth.images.size());
	FOREACH(i, truth.images)
		trueDist[i] = norm(truth.images[i].C);
	// the premise: by inlier count every far pair would pass the star's ratio of the mean of the three
	// heaviest pairs
	{
		std::vector<int> inliers;
		for (const ImagePair& pair : truth.pairs)
			inliers.push_back(pair.numFilteredInliers);
		std::sort(inliers.begin(), inliers.end(), std::greater<int>());
		const int meanTop = (inliers[0] + inliers[1] + inliers[2]) / 3;
		const int floorInliers = ROUND2INT<int>(meanTop * StarInitConfig().ratioInliersFilter);
		for (IIndex j = numNear + 1; j < truth.images.size(); ++j) {
			const int numInliers = truth.pairs[j-1].numFilteredInliers;
			if (numInliers < floorInliers) {
				VERBOSE("StarEvidenceTest FAILED: far pair (0,%u) has %d inliers, under the star's ratio of the heaviest (%d); the scene does not set the by-count trap", j, numInliers, floorInliers);
				return false;
			}
		}
	}
	// by evidence only the near views enter the star, at consistent distances
	StarInitConfig cfg;
	cfg.refineFocalLength = false;
	Scene byEvidence;
	if (!runStar(byEvidence, sides, {nearPoints, farPoints}, cfg)) {
		VERBOSE("StarEvidenceTest FAILED: the star by evidence did not initialize");
		return false;
	}
	REAL minRatio = FLT_MAX, maxRatio = 0;
	for (IIndex v = 1; v < byEvidence.images.size(); ++v) {
		const bool bNear = v <= numNear;
		if (byEvidence.images[v].HasPose() != bNear) {
			VERBOSE("StarEvidenceTest FAILED: %s view %u %s the star by evidence", bNear ? "near" : "far", v, bNear ? "is not in" : "is in");
			return false;
		}
		if (bNear) {
			const REAL ratio = norm(byEvidence.images[v].C) / trueDist[v];
			minRatio = MINF(minRatio, ratio);
			maxRatio = MAXF(maxRatio, ratio);
		}
	}
	if ((maxRatio - minRatio) > 0.01 * minRatio) {
		VERBOSE("StarEvidenceTest FAILED: the near views' distances to the reference span %.2f%% of one scale; expected 1%%",
			(maxRatio / minRatio - 1) * 100);
		return false;
	}
	// no evidence: without the near points no pair measures a distance, so there is no star
	{
		Scene noEvidence;
		LogCapture log;
		if (runStar(noEvidence, sides, {farPoints}, cfg)) {
			VERBOSE("StarEvidenceTest FAILED: a star was built from pairs none of which measures a distance");
			return false;
		}
		if (!log.Contains("insufficient initial views")) {
			VERBOSE("StarEvidenceTest FAILED: the star without evidence did not fail for want of initial views:\n%s", log.text.c_str());
			return false;
		}
	}
	// the keyframe extractor's configuration: the defaults and three views
	StarInitConfig keyframeCfg;
	keyframeCfg.minViews = 3;
	Scene keyframes;
	if (!runStar(keyframes, sides, {nearPoints, farPoints}, keyframeCfg)) {
		VERBOSE("StarEvidenceTest FAILED: the star of the keyframe extractor's configuration did not initialize");
		return false;
	}
	for (IIndex v = 1; v <= numNear; ++v) {
		if (!keyframes.images[v].HasPose()) {
			VERBOSE("StarEvidenceTest FAILED: near view %u is not in the star of the keyframe extractor's configuration", v);
			return false;
		}
	}
	// a floor that keeps fewer neighbours than the star needs but some: the star goes on with them
	{
		StarInitConfig floorCfg(cfg);
		floorCfg.ratioInliersFilter = 0.98f;
		Scene kept;
		LogCapture log;
		if (!runStar(kept, sides, {nearPoints, farPoints}, floorCfg)) {
			VERBOSE("StarEvidenceTest FAILED: the star did not go on with the two neighbours the floor keeps:\n%s", log.text.c_str());
			return false;
		}
		for (IIndex v = 1; v <= numNear; ++v) {
			const bool bKept = v <= 2;
			if (kept.images[v].HasPose() != bKept) {
				VERBOSE("StarEvidenceTest FAILED: near view %u %s the star of the floor that keeps two", v, bKept ? "is not in" : "is in");
				return false;
			}
		}
	}
	// near-duplicate burst: the floor keeps the wider views and leaves the burst out
	const unsigned numBurst = 3;
	const std::vector<REAL> burstSides{0, 0.02, 0.03, 0.04, 0.8, 1.0, 1.2, 1.5};
	const PointGroup widePoints{200, 4, 8, [numBurst](IIndex v) { return v > numBurst; }};
	const PointGroup burstPoints{2500, 400, 800, [numBurst](IIndex v) { return v <= numBurst; }};
	const PointGroup closePoints{12, 0.3, 0.6, [numBurst](IIndex v) { return v <= numBurst; }};
	{
		Scene burst;
		LogCapture log;
		if (!runStar(burst, burstSides, {widePoints, farPoints, burstPoints, closePoints}, cfg)) {
			VERBOSE("StarEvidenceTest FAILED: the star beside a burst of near-duplicate frames was refused:\n%s", log.text.c_str());
			return false;
		}
		for (IIndex v = 1; v < burstSides.size(); ++v)
			if (burst.images[v].HasPose() != (v > numBurst)) {
				VERBOSE("StarEvidenceTest FAILED: %s view %u %s the star beside a burst of near-duplicate frames", v > numBurst ? "wider" : "burst", v, v > numBurst ? "is not in" : "is in");
				return false;
			}
	}
	// The arms of pairs whose baseline direction is less sure than the bar of the block start (kMaxArmDirectionSigma, 0.09 degrees:
	// the matching noise over the parallax energy of the pair's matches in their weakest direction) are not placed. Views 1 to
	// 4 sit at the same distance and see points of the same depths (25 to 35 units, ray angles of 1.6 to 2.3 degrees, all of them
	// evidence), so their pairs have the same median parallax; they differ in the number of matches, 12 and 30 against 400 and
	// 800: the direction of the first two is not fixed, that of the others is. A pure parallax bar could not tell them apart. The
	// same field of view on twice the pixels drops the same arms.
	const auto armScene = [&](Scene& scene, const std::vector<unsigned>& counts, REAL minDepth, REAL maxDepth, unsigned numShared = 0) {
		std::vector<REAL> armSides(counts.size() + 1, REAL(1));
		armSides[0] = 0;
		std::vector<PointGroup> armGroups;
		FOREACH(i, counts)
			armGroups.push_back({counts[i], minDepth, maxDepth, [i](IIndex v) { return v == i + 1; }});
		if (numShared > 0) // points every view sees: the scale of the arms is fixed on them
			armGroups.push_back({numShared, minDepth, maxDepth, [](IIndex) { return true; }});
		buildScene(scene, armSides, armGroups);
		for (Image& img : scene.images)
			img.InvalidatePose();
		for (ImagePair& pair : scene.pairs)
			pair.meanRayAngle = pair.ComputeMeanRayAngle(scene.images[pair.ID1], scene.images[pair.ID2]);
	};
	StarInitConfig armCfg(keyframeCfg);
	armCfg.ratioInliersFilter = 0; // no floor: the arms the bar removes are the only ones left out
	lateralShare = 0;
	for (const REAL scale : {REAL(1), REAL(2)}) {
		cameraScale = scale;
		Scene noisy;
		armScene(noisy, {12, 30, 800, 1600}, 15, 21, 20);
		const REAL medianAngle = R2D(noisy.pairs[0].meanRayAngle);
		for (const ImagePair& pair : noisy.pairs)
			if (ABS(R2D(pair.meanRayAngle) - medianAngle) > 0.1 * medianAngle) {
				VERBOSE("StarEvidenceTest FAILED: the pairs of the same distance have different median ray angles (%g and %g degrees, camera scale %g)", medianAngle, R2D(pair.meanRayAngle), scale);
				return false;
			}
		LogCapture log;
		if (!StarInitializer::Initialize(noisy, armCfg)) {
			VERBOSE("StarEvidenceTest FAILED: the star beside two pairs of 12 and 30 matches was refused (camera scale %g):\n%s", scale, log.text.c_str());
			return false;
		}
		for (IIndex v = 1; v < noisy.images.size(); ++v)
			if (noisy.images[v].HasPose() != (v > 2)) {
				VERBOSE("StarEvidenceTest FAILED: view %u %s the star beside two pairs of 12 and 30 matches (camera scale %g)", v, v > 2 ? "is not in" : "is in", scale);
				return false;
			}
	}
	// the direction sigma of a pair (ImagePair::ComputeBaselineDirectionSigma) is the noise over the square root of the energy of
	// its matches: it falls as 1 / sqrt(number of matches) and grows with the distance of the points, as 1 / parallax. The points
	// of the smaller count are the first of the larger (same generator) and the farther points are the near ones at twice the
	// depth, so the ratios are those of the formula up to the small-angle approximation (ray angles under 4 degrees, under 1%)
	// and the sampling of the 300 extra points, whose energy is not exactly that of the first 100 (5% observed): the tolerance
	// is 10%
	{
		cameraScale = 1;
		const auto sigmaOf = [&](unsigned count, REAL minDepth, REAL maxDepth) {
			Scene scene;
			armScene(scene, {count}, minDepth, maxDepth);
			return scene.pairs[0].ComputeBaselineDirectionSigma(scene.images[0], scene.images[1], REAL(0.0005));
		};
		const REAL sigma100 = sigmaOf(100, 25, 35), sigma400 = sigmaOf(400, 25, 35), sigmaFar = sigmaOf(100, 50, 70);
		if (ABS(sigma100 / sigma400 - 2) > 0.1) {
			VERBOSE("StarEvidenceTest FAILED: the direction sigma of 100 matches is %g times that of 400, expected 2", sigma100 / sigma400);
			return false;
		}
		if (ABS(sigmaFar / sigma100 - 2) > 0.1) {
			VERBOSE("StarEvidenceTest FAILED: the direction sigma at twice the distance is %g times that at the distance, expected 2", sigmaFar / sigma100);
			return false;
		}
		Scene empty;
		armScene(empty, {100}, 25, 35);
		empty.pairs[0].numFilteredInliers = 0; // no track-forming match: no direction is fixed
		if (!std::isinf(empty.pairs[0].ComputeBaselineDirectionSigma(empty.images[0], empty.images[1], REAL(0.0005)))) {
			VERBOSE("StarEvidenceTest FAILED: a pair without track-forming matches has a finite direction sigma");
			return false;
		}
	}
	// the arms the floor and the cap chose are the arms the star would have without the rule: with the cap at 3 the star holds
	// the three views of the most evidence; the first of them has 12 matches only (its weighted inliers are those of a dense
	// supplement, 2000), so its direction is not fixed, and it leaves a gap, the fourth view does not take its place
	{
		cameraScale = 1;
		StarInitConfig cappedCfg(armCfg);
		cappedCfg.maxViews = 3;
		Scene gap;
		armScene(gap, {12, 1200, 900, 800}, 15, 21, 20);
		gap.pairs[0].weightedInliers = 2000.f;
		if (!StarInitializer::Initialize(gap, cappedCfg)) {
			VERBOSE("StarEvidenceTest FAILED: the star of the cap of 3 with a pair of 12 matches was refused");
			return false;
		}
		for (IIndex v = 1; v < gap.images.size(); ++v)
			if (gap.images[v].HasPose() != (v == 2 || v == 3)) {
				VERBOSE("StarEvidenceTest FAILED: view %u %s the star of the cap of 3 with a pair of 12 matches", v, gap.images[v].HasPose() ? "is in" : "is not in");
				return false;
			}
	}
	cameraScale = 1;
	lateralShare = 0.3;
	VERBOSE("StarEvidenceTest PASSED: %u far views left out of the star by evidence, near distances within %.3f%%, no star without evidence, "
		"the keyframe configuration initializes, the star goes on with the neighbours the floor keeps, the star beside a burst of %u near-duplicate frames starts with the wider views, no arm is placed from a pair whose baseline direction is under the bar, at either resolution (%s)",
		numFar, (maxRatio / minRatio - 1) * 100, numBurst, TD_TIMER_GET_FMT().c_str());
	return true;
}

// A near-duplicate pair is never the yardstick of good connectivity (PairsWeightingConfig::maxDuplicateOverlap /
// maxDuplicateRayAngle). A reference view 0 with a burst of three near-duplicate frames (1-3, 0.02-0.04 apart, 2500
// points 2-4 units away over the whole frame, plus a few close ones) and four wide views (4-7, 0.8-1.5 apart) whose points fill only the left
// half of the frame, so the burst pairs cover the whole frame (overlap 1, ray angle well under 1.5 deg) and the wide pairs half
// of it. With the rule off the burst pair is the reference's yardstick: it shrinks every wide pair's connectivity factor. With
// the default bars it reads the first non-duplicate pair: the strongest wide pair's connectivity is 1. The star's floor is a
// share of the strongest candidates' evidence, which the burst has little of, so the star starts with the wide views either
// way, also beside a burst the bars do not flag (its points over 70% of the frame, under the 0.75 bar), whose inliers would
// lift a floor in inliers above every wide view's evidence. The burst pair (1,2), weaker than (0,1) and (0,2), joins two images
// whose pairs are all near-duplicates: they keep today's yardstick, so its factor is the same with the rule on and off. The
// stored overlap follows the pair's current matches (a pair cut to half the frame reads half at the next weighting pass).
// Last, the predicate itself: the overlap and the angle must both say so.
// The star fixes its scale on the pair of the reference view with the most weighted inliers, the count every other
// reader of a pair's evidence uses. Reference 0 at the origin, views 1, 2, 3 at 1, 2, 3 units along x, every pair
// (0, j) carrying a unit baseline: (0, 1) holds every match of its two views but only 20 track-forming ones (the rest
// past the strict filter, raw matches its weighted inliers do not count), (0, 2) every other match, (0, 3) every third.
// The scale fixed on (0, 2), the heaviest by weighted inliers, puts view 2 at distance 1 and view 1 at 0.5; fixed on
// (0, 1), the most matches, it would put view 1 at 1.
bool StarScaleGaugeTest()
{
	TD_TIMER_START();
	Scene scene;
	PinholeCamera* pCamera = new PinholeCamera(cv::Size(640, 480), REAL(400), REAL(400), REAL(320), REAL(240));
	pCamera->trustIntrinsics = true;
	scene.cameras.emplace_back(pCamera);
	for (IIndex i = 0; i < 4; ++i) {
		Pose3D pose;
		pose.R = Matrix3x3::IDENTITY;
		pose.C = Point3(REAL(i), REAL(0.1) * (i % 2), 0);
		scene.images.emplace_back(i, "", pose, 0, scene.cameras[0]);
	}
	std::mt19937 rng(20261113);
	std::uniform_int_distribution<int> pixelX(40, 600), pixelY(40, 440);
	std::uniform_real_distribution<REAL> depth(6, 12);
	for (unsigned p = 0; p < 600 && scene.tracks.size() < 300; ++p) {
		const Point3 X = scene.images[0].UnprojectPoint(Point2(pixelX(rng), pixelY(rng)), depth(rng));
		std::vector<Point2> projections;
		for (const Image& img : scene.images) {
			const auto [proj, valid] = img.ProjectPoint(X);
			if (valid && Image8U::isInside(proj, img.GetSize()))
				projections.push_back(proj);
		}
		if (projections.size() != scene.images.size())
			continue;
		Track track(X);
		FOREACH(i, scene.images) {
			scene.images[i].keypoints.emplace_back(projections[i], 0.f, 0.f, 10.f);
			track.observations.emplace_back(IIndex(i), (uint32_t)scene.images[i].keypoints.size() - 1);
		}
		scene.tracks.emplace_back(std::move(track));
	}
	const auto addPair = [&](IIndex b, unsigned stride, int numFiltered) {
		ImagePair& pair = scene.pairs.emplace_back(IIndex(0), b);
		FOREACH(t, scene.tracks)
			if (t % stride == 0)
				pair.matches.emplace_back(scene.tracks[t].observations[0].featureID, scene.tracks[t].observations[b].featureID);
		pair.numFilteredInliers = numFiltered < 0 ? (int)pair.matches.size() : numFiltered;
		pair.relativePose = scene.images[b] / scene.images[0];
		pair.relativePose->C /= norm(pair.relativePose->C);
		pair.E = ImagePair::ComposeEssentialMatrix(pair.relativePose.value());
		pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), scene.images[0].GetK(), scene.images[b].GetK());
		pair.meanRayAngle = pair.ComputeMeanRayAngle(scene.images[0], scene.images[b]);
	};
	addPair(1, 1, 20);
	addPair(2, 2, -1);
	addPair(3, 3, -1);
	ComputePairsWeights(scene);
	const ImagePair& p01 = *scene.FindPair(0, 1);
	const ImagePair& p02 = *scene.FindPair(0, 2);
	if (!(p01.GetNumInliers() > p02.GetNumInliers() && p01.GetNumWeightedInliers() < p02.GetNumWeightedInliers() &&
		IsPoseLinkPair(p01) && IsPoseLinkPair(p02) && IsPoseLinkPair(*scene.FindPair(0, 3)))) {
		VERBOSE("StarScaleGaugeTest FAILED: premise, pair (0,1) %u matches / %u weighted inliers, pair (0,2) %u / %u",
			p01.GetNumInliers(), p01.GetNumWeightedInliers(), p02.GetNumInliers(), p02.GetNumWeightedInliers());
		return false;
	}
	if (!StarInitializer::EstimateGlobalScale(scene, 0, IIndexArr{1, 2, 3}, PairsWeightingConfig())) {
		VERBOSE("StarScaleGaugeTest FAILED: the scale estimation failed");
		return false;
	}
	const REAL d1 = norm(scene.images[1].C), d2 = norm(scene.images[2].C);
	if (ABS(d2 - 1) > 0.01 || ABS(d1 - 0.5) > 0.02) {
		VERBOSE("StarScaleGaugeTest FAILED: view 1 at %.3f and view 2 at %.3f from the reference (0.5 and 1 expected: the scale "
			"fixed on (0,2), the pair with the most weighted inliers)", d1, d2);
		return false;
	}
	VERBOSE("StarScaleGaugeTest PASSED: the scale is fixed on the pair with the most weighted inliers, view 2 at %.3f and "
		"view 1 at %.3f, not on the one with the most matches (%s)", d2, d1, TD_TIMER_GET_FMT().c_str());
	return true;
}

bool NearDuplicateYardstickTest()
{
	TD_TIMER_START();
	struct PointGroup {
		unsigned count;
		REAL minDepth, maxDepth;
		int maxPixelX;
		std::function<bool(IIndex)> seenBy;
	};
	const std::vector<REAL> sides{0, 0.02, 0.03, 0.04, 0.8, 1.0, 1.2, 1.5};
	const unsigned numBurst = 3;
	const auto buildScene = [&](Scene& scene, const std::vector<PointGroup>& groups) {
		PinholeCamera* pCamera = new PinholeCamera(cv::Size(640, 480), REAL(400), REAL(400), REAL(320), REAL(240));
		pCamera->trustIntrinsics = true;
		scene.cameras.emplace_back(pCamera);
		FOREACH(i, sides) {
			Pose3D pose;
			pose.R = Matrix3x3::IDENTITY;
			pose.C = Point3(sides[i], (i % 2) * sides[i] * 0.3, 0);
			scene.images.emplace_back(i, "", pose, 0, scene.cameras[0]);
		}
		std::mt19937 rng(7);
		for (const PointGroup& group : groups) {
			std::uniform_int_distribution<int> pixelX(40, group.maxPixelX), pixelY(40, 440);
			std::uniform_real_distribution<REAL> depth(group.minDepth, group.maxDepth);
			for (unsigned p = 0; p < group.count; ++p) {
				const Point3 X = scene.images[0].UnprojectPoint(Point2(pixelX(rng), pixelY(rng)), depth(rng));
				std::vector<std::pair<Image*, Point2>> seen;
				for (Image& img : scene.images) {
					if (img.ID != 0 && !group.seenBy(img.ID))
						continue;
					const auto [proj, valid] = img.ProjectPoint(X);
					if (!valid || !Image8U::isInside(proj, img.GetSize()))
						continue;
					seen.emplace_back(&img, proj);
				}
				if (seen.size() < 2)
					continue;
				Track track(X); // no inlier yet, as BuildTracks leaves it: the star triangulates the tracks its views see
				for (const auto& [img, proj] : seen) {
					img->keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
					track.observations.emplace_back(img->ID, (uint32_t)img->keypoints.size() - 1);
				}
								scene.tracks.emplace_back(std::move(track));
			}
		}
		const auto addPair = [&](IIndex a, IIndex b, unsigned stride) {
			ImagePair& pair = scene.pairs.emplace_back(a, b);
			unsigned k = 0;
			for (const Track& track : scene.tracks) {
				uint32_t obsA = NO_ID, obsB = NO_ID;
				for (const auto& obs : track.observations) {
					if (obs.imageID == a)
						obsA = obs.featureID;
					else if (obs.imageID == b)
						obsB = obs.featureID;
				}
				if (obsA != NO_ID && obsB != NO_ID && (k++ % stride) == 0)
					pair.matches.emplace_back(obsA, obsB);
			}
			pair.numFilteredInliers = (int)pair.matches.size();
			pair.relativePose = scene.images[b] / scene.images[a];
			pair.relativePose->C /= norm(pair.relativePose->C);
			pair.E = ImagePair::ComposeEssentialMatrix(pair.relativePose.value());
			pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), scene.images[a].GetK(), scene.images[b].GetK());
			pair.meanRayAngle = pair.ComputeMeanRayAngle(scene.images[a], scene.images[b]);
		};
		for (IIndex j = 1; j < scene.images.size(); ++j)
			addPair(0, j, 1);
		addPair(1, 2, 2); // every other match: weaker than (0,1) and (0,2), still over the whole frame
		scene.status.nCalibratedImages = scene.images.size();
	};
	const PointGroup widePoints{200, 4, 8, 300, [numBurst](IIndex v) { return v > numBurst; }};
	const PointGroup farPoints{500, 400, 800, 300, [](IIndex) { return true; }};
	const PointGroup burstPoints{2500, 2, 4, 600, [numBurst](IIndex v) { return v <= numBurst; }}; // near enough for a measurable ray angle, well under 1.5 deg
	const PointGroup closePoints{12, 0.3, 0.6, 600, [numBurst](IIndex v) { return v <= numBurst; }};
	const std::vector<PointGroup> groups{widePoints, farPoints, burstPoints, closePoints};
	// per pair (0,j), j = 1..7: its connectivity factor, overlap, ray angle (deg), weighted inliers and star evidence
	struct Reading { std::vector<float> conn, overlap, rayAngle; std::vector<unsigned> inliers, evidence; float conn12, overlap12; bool star; std::vector<bool> posed; String log; };
	const auto run = [&](const PairsWeightingConfig& weighting, const std::vector<PointGroup>& groups) {
		Scene scene;
		buildScene(scene, groups);
		ComputePairsWeights(scene, weighting);
		Reading r;
		for (IIndex j = 1; j < scene.images.size(); ++j) {
			const ImagePair& pair = *scene.FindPair(0, j);
			r.conn.push_back(pair.weightConnectivity);
			r.overlap.push_back(pair.overlapArea);
			r.rayAngle.push_back(R2D(pair.meanRayAngle));
			r.inliers.push_back(pair.GetNumWeightedInliers());
			r.evidence.push_back(ROUND2INT<unsigned>(pair.GetNumWeightedInliers() * pair.ComputeShareAboveAngle(scene.images[0], scene.images[j], StarInitConfig().evidenceMinAngle)));
		}
		r.conn12 = scene.FindPair(1, 2)->weightConnectivity;
		r.overlap12 = scene.FindPair(1, 2)->overlapArea;
		for (Image& img : scene.images)
			img.InvalidatePose();
		// the median ray angle of a wide pair falls under the star's parallax bar (its matches are mostly the far points), where the star
		// places no arm; the wide pairs read what their near points give, so the star below is the yardstick's alone
		for (IIndex j = numBurst + 1; j < scene.images.size(); ++j)
			scene.FindPair(0, j)->meanRayAngle = D2R(10.f);
		StarInitConfig cfg;
		cfg.refineFocalLength = false;
		cfg.pairsWeighting = weighting;
		LogCapture log;
		r.star = StarInitializer::Initialize(scene, cfg);
		r.log = log.text;
		for (const Image& img : scene.images)
			r.posed.push_back(img.HasPose());
		return r;
	};
	// the premise of a burst: its pairs cover more than minBurstOverlap of the frame at a ray angle under 1 deg, the wide pairs
	// at most half of it
	const auto premise = [&](const Reading& r, float minBurstOverlap) {
		for (IIndex j = 1; j < sides.size(); ++j) {
			const float overlap = r.overlap[j - 1], rayAngle = r.rayAngle[j - 1];
			if ((j <= numBurst) != (overlap > minBurstOverlap) || (j > numBurst && overlap > 0.5f) || (j <= numBurst && !(rayAngle > 0.f && rayAngle < 1.f))) {
				VERBOSE("NearDuplicateYardstickTest FAILED: premise: pair (0,%u) overlap %.2f, ray angle %.3f deg", j, overlap, rayAngle);
				return false;
			}
		}
		if (r.overlap12 <= minBurstOverlap) {
			VERBOSE("NearDuplicateYardstickTest FAILED: premise: pair (1,2) overlap %.2f", r.overlap12);
			return false;
		}
		return true;
	};
	PairsWeightingConfig offCfg;
	offCfg.maxDuplicateOverlap = 0.f;
	const Reading off = run(offCfg, groups), on = run(PairsWeightingConfig(), groups);
	if (!premise(off, 0.9f) || !premise(on, 0.9f))
		return false;
	// rule off: the burst is the yardstick, every wide factor under 0.6; the star's floor, in the units of evidence, keeps
	// the wide views either way
	float maxWideOff = 0, maxWideOn = 0;
	for (IIndex j = numBurst + 1; j < sides.size(); ++j) {
		maxWideOff = MAXF(maxWideOff, off.conn[j - 1]);
		maxWideOn = MAXF(maxWideOn, on.conn[j - 1]);
	}
	if (maxWideOff >= 0.6f || !off.star) {
		VERBOSE("NearDuplicateYardstickTest FAILED: with the rule off the strongest wide pair's connectivity is %.3f (under 0.6 expected) and the star %s (started expected):\n%s",
			maxWideOff, off.star ? "started" : "was refused", off.log.c_str());
		return false;
	}
	// rule on: the strongest wide pair is the reference's yardstick
	if (maxWideOn < 0.999f) {
		VERBOSE("NearDuplicateYardstickTest FAILED: with the rule on the strongest wide pair's connectivity is %.3f (1 expected: the first non-duplicate pair is the yardstick)", maxWideOn);
		return false;
	}
	if (!on.star) {
		VERBOSE("NearDuplicateYardstickTest FAILED: the star was refused with the rule on (expected to start with the wide views):\n%s", on.log.c_str());
		return false;
	}
	for (IIndex v = numBurst + 1; v < sides.size(); ++v)
		if (!on.posed[v] || !off.posed[v]) {
			VERBOSE("NearDuplicateYardstickTest FAILED: wide view %u is not in the star with the rule %s", v, on.posed[v] ? "off" : "on");
			return false;
		}
	// a burst the bars do not flag: its points over 70% of the frame, under the 0.75 bar; by inlier count it would lift the
	// floor above every wide view's evidence (the three heaviest pairs are the burst's), by evidence the wide views pass it,
	// the star starts with them and the burst, a dozen inliers of evidence each, is left to the resection
	unsigned maxWideEvidence = 0, floorInliers = 0;
	float maxBurstOverlap = 0;
	{
		std::vector<PointGroup> partGroups(groups);
		partGroups[2].maxPixelX = partGroups[3].maxPixelX = 420; // the burst and its close points over 7 of the 10 grid columns
		const PairsWeightingConfig cfg;
		const Reading part = run(cfg, partGroups);
		if (!premise(part, 0.6f))
			return false;
		std::vector<unsigned> inliers(part.inliers);
		std::sort(inliers.begin(), inliers.end(), std::greater<unsigned>());
		floorInliers = ROUND2INT<unsigned>((inliers[0] + inliers[1] + inliers[2]) / 3 * StarInitConfig().ratioInliersFilter);
		for (IIndex j = 1; j < sides.size(); ++j) {
			const bool bBurst = j <= numBurst;
			if (bBurst)
				maxBurstOverlap = MAXF(maxBurstOverlap, part.overlap[j - 1]);
			else
				maxWideEvidence = MAXF(maxWideEvidence, part.evidence[j - 1]);
			if (bBurst ? part.overlap[j - 1] >= cfg.maxDuplicateOverlap : part.evidence[j - 1] >= floorInliers) {
				VERBOSE("NearDuplicateYardstickTest FAILED: premise: pair (0,%u) overlap %.2f (under %.2f expected for a burst pair), evidence %u (under the inlier floor %u expected for a wide pair)",
					j, part.overlap[j - 1], cfg.maxDuplicateOverlap, part.evidence[j - 1], floorInliers);
				return false;
			}
		}
		if (!part.star) {
			VERBOSE("NearDuplicateYardstickTest FAILED: the star beside a burst the bars do not flag was refused (expected to start with the wide views):\n%s", part.log.c_str());
			return false;
		}
		for (IIndex v = 1; v < sides.size(); ++v)
			if (part.posed[v] != (v > numBurst)) {
				VERBOSE("NearDuplicateYardstickTest FAILED: %s view %u %s the star beside a burst the bars do not flag", v > numBurst ? "wide" : "burst", v, part.posed[v] ? "is in" : "is not in");
				return false;
			}
	}
	// an image whose pairs are all near-duplicates keeps today's yardstick
	if (off.conn12 >= 0.9f || ABS(on.conn12 - off.conn12) > 1e-6f) {
		VERBOSE("NearDuplicateYardstickTest FAILED: the burst pair (1,2) has connectivity %.4f off and %.4f on (equal and under 0.9 expected: its images keep today's yardstick)",
			off.conn12, on.conn12);
		return false;
	}
	// the stored overlap follows the pair's matches: a burst pair cut to the left half of the frame after a first weighting
	// pass reads the half overlap at the next pass, and is no near-duplicate any more
	{
		Scene scene;
		buildScene(scene, groups);
		const PairsWeightingConfig cfg;
		ComputePairsWeights(scene, cfg);
		ImagePair& pair = *scene.FindPair(0, 1);
		const float before = pair.overlapArea;
		std::vector<DMatch> left;
		for (const DMatch& m : pair.matches)
			if (scene.images[0].keypoints[m.queryIdx].pt.x < 300.f)
				left.push_back(m);
		pair.matches = std::move(left);
		pair.numFilteredInliers = (int)pair.matches.size();
		pair.weightedInliers = -1.f;
		ComputePairsWeights(scene, cfg);
		const ImagePair& cut = *scene.FindPair(0, 1);
		const float after = cut.overlapArea, coverage = ComputePairCoverage(cut, scene.images[0], scene.images[1], cfg.gridSize);
		if (before <= 0.9f || after != coverage || after > 0.5f || cfg.IsNearDuplicate(cut)) {
			VERBOSE("NearDuplicateYardstickTest FAILED: the burst pair cut to the left half stores an overlap of %.2f after the second weighting pass (its matches cover %.2f; %.2f before the cut) and is %sa near-duplicate",
				after, coverage, before, cfg.IsNearDuplicate(cut) ? "" : "not ");
			return false;
		}
	}
	// the predicate: a near-duplicate needs the overlap above the bar AND a measured ray angle under it
	{
		const PairsWeightingConfig cfg;
		ImagePair pair(0, 1);
		const auto test = [&](float overlap, float rayAngleDeg) {
			pair.overlapArea = overlap;
			pair.meanRayAngle = D2R(rayAngleDeg);
			return cfg.IsNearDuplicate(pair);
		};
		if (!test(0.9f, 1.f) || test(0.7f, 1.f) || test(0.9f, 2.f) || test(0.9f, 0.f) || offCfg.IsNearDuplicate(pair)) {
			VERBOSE("NearDuplicateYardstickTest FAILED: the near-duplicate test: overlap 0.9 and 1 deg %d (1 expected), overlap 0.7 %d, angle 2 deg %d, angle not measured %d, rule off %d (0 expected)",
				(int)test(0.9f, 1.f), (int)test(0.7f, 1.f), (int)test(0.9f, 2.f), (int)test(0.9f, 0.f), (int)offCfg.IsNearDuplicate(pair));
			return false;
		}
	}
	VERBOSE("NearDuplicateYardstickTest PASSED: strongest wide connectivity %.3f -> %.3f, the star starts with the wide views both ways, burst pair (1,2) %.3f both ways, "
		"a burst the bars do not flag (overlap %.2f) lifts no floor above the wide views (evidence %u at most, a floor in inliers %u), the stored overlap follows a cut of the matches, the test needs both the overlap and the angle (%s)",
		maxWideOff, maxWideOn, off.conn12, maxBurstOverlap, maxWideEvidence, floorInliers, TD_TIMER_GET_FMT().c_str());
	return true;
}

namespace {
// The given image's correspondences with the first numTrue + numNoise tracks of the scene: the
// first numTrue at the projections the image's own pose gives and the rest at random pixels, so the
// pose an estimator recovers from them is the image's own on a numTrue / (numTrue + numNoise) share
void AddResectionCorrespondences(Scene& scene, IIndex imageID, unsigned numTrue, unsigned numNoise, std::mt19937& rng)
{
	Image& img = scene.images[imageID];
	std::uniform_real_distribution<float> pixelX(10.f, 630.f), pixelY(10.f, 470.f);
	for (unsigned i = 0; i < numTrue + numNoise; ++i) {
		Track& track = scene.tracks[i];
		cv::Point2f pt(pixelX(rng), pixelY(rng));
		if (i < numTrue) {
			const auto [proj, valid] = img.ProjectPoint(track.position);
			ASSERT(valid && Image8U::isInside(proj, img.GetSize()));
			pt = cv::Point2f((float)proj.x, (float)proj.y);
		}
		track.observations.emplace_back(imageID, (uint32_t)img.keypoints.size());
		img.keypoints.emplace_back(pt, 1.f);
	}
}

// Image 4's correspondences in the resection cases: 40 at the projections its own pose gives and 60 at random
// pixels, so the pose an estimator recovers is its true one on a 40% inlier share, low enough for the link checks to apply
constexpr unsigned RESECTION_TRUE_CORRESPONDENCES = 40;
constexpr unsigned RESECTION_NOISE_CORRESPONDENCES = 60;

// The arc scene of the weak-evidence case: eight cameras, images 0-3 registered with 150
// triangulated tracks, image 4 unregistered with the 40 true and 60 random correspondences of the
// direction case, joined to registered image 3 by a pair of 20 weighted inliers and to unregistered
// image 5 by one of 1000: its evidence lies almost entirely in an image the model does not hold
// yet, and image 5 holds no correspondence. Images 6 and 7 hold 30 true correspondences each and
// are joined to registered images by pairs of 300 and 200 weighted inliers.
void BuildResectionEvidenceScene(Scene& scene, std::vector<Pose3D>& truePoses, std::mt19937& rng)
{
	BuildResectionArc(scene, 8);
	AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);

	AddResectionCorrespondences(scene, 4, RESECTION_TRUE_CORRESPONDENCES, RESECTION_NOISE_CORRESPONDENCES, rng);
	AddResectionCorrespondences(scene, 6, 30, 0, rng);
	AddResectionCorrespondences(scene, 7, 30, 0, rng);
	AddResectionPair(scene, 3, 4, 20);
	AddResectionPair(scene, 4, 5, 1000);
	AddResectionPair(scene, 2, 6, 300);
	AddResectionPair(scene, 3, 7, 200);
	truePoses.clear();
	for (const Image& img : scene.images)
		truePoses.push_back(static_cast<const Pose3D&>(img));
	for (IIndex i = 4; i < scene.images.size(); ++i)
		scene.images[i].InvalidatePose();
	scene.status.nCalibratedImages = 4;
	TriangulateTracks(scene, false, 4.f, 1.f);
}

// Where the resection's attempts hold the first one that registered or failed the given image (String::npos
// when there was none): the order of two attempts is the order of these positions
size_t FirstAttempt(const Resection::Stats& stats, IIndex imageID, bool registered)
{
	const auto it = std::find_if(stats.attempts.begin(), stats.attempts.end(),
		[&](const Resection::Stats::Attempt& a) { return a.imageID == imageID && a.registered == registered; });
	return it == stats.attempts.end() ? String::npos : size_t(it - stats.attempts.begin());
}
} // namespace

// The checks a resected pose passes before it enters the model (Resection::RegisterImage), each case a row on the
// arc of cameras (ArcScene) whose image 4 is resected: the minimum inlier share and the rotation its strongest pair
// composes, the quorum of agreeing pairs that overrules a false witness, the baseline direction each link predicts
// (weighed in weighted inliers, the pose's own inliers voting for it, and forgiven by the larger of the pair's ray
// angle and the model's parallax), the rotation all links predict (weighted mean), and at the end the direction
// tolerance of one pair alone (MeasurePairDisagreement)
bool ResectionLinkChecksTest()
{
	TestScope scope("ResectionLinkChecksTest");
	constexpr REAL STEP = ArcScene::STEP;

	// Image 4's scene: an arc of numImages whose images 0-3 hold 150 tracks; image 4 sees the first 100 of them,
	// numTrue at the projections of `projecting` (its own pose when NULL) and the rest at random pixels (40 of 100
	// by default: a share low enough for every link check to apply), and is linked to the given images. closeStep > 0
	// places image 5 that many degrees along the arc from image 4, registered and seeing the tracks too; every other
	// image from 4 on is unregistered.
	struct Link { IIndex image; unsigned inliers; ArcLinkSpec spec; };
	const auto LinkScene = [](uint32_t seed, unsigned numImages, const std::vector<Link>& links, REAL closeStep = 0,
		const Pose3D* projecting = NULL, unsigned numTrue = 40)
	{
		ArcScene arc(numImages, seed);
		if (closeStep > 0)
			arc.Place(5, 4 * STEP + closeStep);
		arc.Tracks(closeStep > 0 ? IIndexArr{0, 1, 2, 3, 5} : IIndexArr{0, 1, 2, 3}, 150);
		arc.Correspondences(4, numTrue, 100 - numTrue, projecting);
		for (const Link& link : links)
			arc.Pair(link.image, 4, link.inliers, link.spec);
		IIndexArr unregistered;
		for (IIndex i = 4; i < numImages; ++i)
			if (closeStep == 0 || i != 5)
				unregistered.push_back(i);
		arc.Unregister(unregistered);
		return arc;
	};

	// the scenes. The decoy: 20 of image 4's 100 correspondences are the projections of a pose 40 degrees further
	// along the arc and 80 random, so the strongest consensus a robust estimator finds is that wrong pose, on 20%
	const Pose3D decoy(ArcScene::Pose(4 * STEP + 40));
	const auto Decoy = [&] { return LinkScene(20260908, 5, {{3, 400}}, 0, &decoy, 20); };
	// the quorum: the pose found is the right one, but the strongest pair is to image 3, whose stored pose is then
	// displaced 40 degrees along the arc, so it predicts a rotation that far off; the weaker pairs to images 2 and 1
	// agree with the pose. The adjustments that follow start from the displaced image 3 and, the scale of the model
	// being free (only image 1 is held), settle at another scale: every image ends up exactly on the truth after one
	// similarity of scale 0.43, image 4 3.49 units from its true centre before it, hence RIGHT_SHAPE
	const auto Quorum = [&](bool corroborating) {
		ArcScene arc(LinkScene(20260908, 5, corroborating ?
			std::vector<Link>{{3, 400}, {2, 300}, {1, 250}} : std::vector<Link>{{3, 400}}));
		static_cast<Pose3D&>(arc.scene.images[3]) = ArcScene::Pose(3 * STEP + 40);
		return arc;
	};
	// the direction cases: pairs to images 3, 2, 1 of the given weights, all reporting the given ray angle, the first
	// numTurned with their translation turned 40 degrees, the shape of a pair whose translation is wrong
	const auto Turned = [&](unsigned numTurned, REAL rayAngleDeg, std::array<unsigned, 3> weights = {400, 300, 250}) {
		std::vector<Link> links;
		for (unsigned k = 0; k < 3; ++k)
			links.push_back({IIndex(3 - k), weights[k], {rayAngleDeg, k < numTurned ? REAL(40) : REAL(0)}});
		return LinkScene(20260918, 5, links);
	};
	// the model-parallax cases: a wrong pose with image 4's rotation at a centre whose baseline to image 3 is the true
	// one lifted 25 degrees out of the arc's plane, 0.38 units from the true centre
	Pose3D lifted(ArcScene::Pose(4 * STEP));
	{
		const Point3 C3(ArcScene::Pose(3 * STEP).C), baseline(lifted.C - C3);
		lifted.C = C3 + baseline * COS(D2R(25.0)) + Point3(0, norm(baseline) * SIN(D2R(25.0)), 0);
	}
	const auto Parallax = [&](const std::vector<Link>& links, REAL closeStep = 1, const Pose3D* projecting = NULL) {
		return LinkScene(20261025, 6, links, closeStep, projecting);
	};
	// the link rotation cases: pairs of the given weights with their rotation turned about the vertical axis
	const auto Rotated = [&](const std::vector<Link>& links) { return LinkScene(20261105, 6, links); };

	ResectionConfig defaults, ratioOff, ratioAndDirectionOff, allOff, quorumOnly, direction5, direction3, direction0, linkRotationOff;
	ratioOff.minInlierRatio = 0.f;
	ratioAndDirectionOff = ratioOff;
	ratioAndDirectionOff.maxLinkDirectionError = 0.f;
	allOff = ratioAndDirectionOff;
	allOff.maxRelativeRotationError = 180.f;
	allOff.maxLinkRotationDisagreement = 0.f;
	quorumOnly.maxLinkRotationDisagreement = 0.f; // the link rotation check hears every link, the false witness too
	direction5.maxLinkDirectionError = 5.f;
	// a premise of the parallax cases, not the default: their numbers (31, 36.9, 16.7, 2.15 degrees) are read
	// against 3 degrees, and the default 5 would forgive the lifted pose and the turns of the first two
	direction3.maxLinkDirectionError = 3.f;
	direction0.maxLinkDirectionError = 0.f;
	linkRotationOff.maxLinkRotationDisagreement = 0.f;

	// REFUSED: image 4 stays unregistered; REGISTERED: it registers; RIGHT_ROTATION: it registers within 1 degree of its
	// true rotation; RIGHT_POSE: and within 1% of the arc's extent of its true centre; RIGHT_SHAPE: RIGHT_ROTATION, and every
	// registered image is within 1 degree and 1% of the extent of its true pose after the best similarity (PoseErrors);
	// WRONG: it registers more than 15 degrees off its true rotation
	enum Expect { REFUSED, REGISTERED, RIGHT_ROTATION, RIGHT_POSE, RIGHT_SHAPE, WRONG };
	struct Case {
		const char* name; std::function<ArcScene()> scene; const ResectionConfig& config; Expect expect; const char* why;
		bool single = false;         // RegisterImage(4) alone, not the resection's loop
		const char* logged = NULL;   // a line the log must hold
	};
	const Case cases[] = {
		{"decoy, defaults", Decoy, defaults, REFUSED, "20 of 100 inliers is under the 25% minimum share and far under the 100 that would waive it"},
		{"decoy, every rule off", Decoy, allOff, WRONG, "with the share, the composed rotation, the link direction and the link rotation off, the decoy pose is accepted, so the rules are what keeps it out"},
		{"decoy, share off", Decoy, ratioOff, REFUSED, "the pose is 40 degrees from the rotation its strongest pair composes, and its 20% share does not earn it the benefit of the doubt"},
		{"decoy, share and direction off", Decoy, ratioAndDirectionOff, REFUSED, "with every direction check off the composed-rotation check still refuses it"},
		{"quorum", [&] { return Quorum(true); }, quorumOnly, RIGHT_SHAPE, "two of three pairs agree with the pose and overrule the strongest pair, to the displaced image 3"},
		{"false witness alone", [&] { return Quorum(false); }, defaults, REFUSED, "the pair to the displaced image 3 is the only witness left"},
		{"1 of 3 turned", [&] { return Turned(1, 20); }, direction5, RIGHT_ROTATION, "the two agreeing pairs outweigh the turned one: a single misplaced neighbour cannot refuse a right pose"},
		{"2 of 3 turned", [&] { return Turned(2, 20); }, direction5, REFUSED, "the links that contradict the direction outweigh the one that agrees"},
		{"2 of 3 turned, direction off", [&] { return Turned(2, 20); }, direction0, RIGHT_ROTATION, "the direction check is what refused the case above"},
		{"2 of 3 turned, 1-degree rays", [&] { return Turned(2, 1); }, direction5, RIGHT_ROTATION, "near-duplicate pairs fix no direction: they are judged on the rotation alone"},
		{"2 of 3 turned, 4-degree rays", [&] { return Turned(2, 4); }, direction5, REFUSED, "4-degree rays would forgive 51 degrees, but the model places images 3 and 2 10 and 20 degrees of parallax from the pose and the larger angle sets the tolerance (27 and 14 degrees)"},
		{"turned 400 against 150+150", [&] { return Turned(1, 20, {400, 150, 150}); }, defaults, REFUSED, "weighed in weighted inliers, not links: the 400 that contradict outweigh the 300 that agree, which a count of links would have outvoted two to one"},
		{"turned 335 against 150+150", [&] { return Turned(1, 20, {335, 150, 150}); }, defaults, RIGHT_ROTATION, "the pose's own 40 inliers vote for it: 335 outweighs the 300 links that agree but not the 340 with the pose"},
		{"turned 345 against 150+150", [&] { return Turned(1, 20, {345, 150, 150}); }, defaults, REFUSED, "345 outweighs the 340 of the agreeing links and the pose together"},
		{"parallax 1: close in the model only", [&] { return Parallax({{5, 400, {5, 40}}, {3, 100, {20}}}); }, direction3, RIGHT_POSE, "the heaviest link, to image 5, is a pair of 5-degree rays turned 40 degrees (its own tolerance, 31, would refuse), but the model sees images 4 and 5 under 2 degrees of parallax: it abstains and the right link to image 3 vouches"},
		{"parallax 2: the model tightens", [&] { return Parallax({{3, 400, {4}}}, 1, &lifted); }, direction3, REFUSED, "the lifted pose is 25 degrees off its only link's direction; the pair's 4-degree rays forgive 36.9, but the model places the pose 10 degrees of parallax from image 3 and the larger angle holds it to 16.7"},
		{"parallax 3: close in both", [&] { return Parallax({{5, 400, {1}}, {3, 100, {20, 40}}}); }, direction3, RIGHT_POSE, "the pair to image 5 is a near-duplicate the model also sees under 2 degrees, so it agrees and its 400 outweigh the 100 of the turned pair to image 3"},
		{"parallax 4: direction off", [&] { return Parallax({{3, 400, {4}}}, 1, &lifted); }, direction0, REGISTERED, "maxLinkDirectionError 0 turns every direction check off and the lifted pose of case 2 is accepted"},
		{"parallax 5: a small angle forgives", [&] { return Parallax({{5, 400, {4, 20}}, {3, 100, {20}}}, 4); }, direction3, RIGHT_POSE, "image 5 4 degrees along: both the pair's rays and the model's parallax are about 4 degrees, which forgives 36.9, so the link turned 20 degrees agrees"},
		{"parallax 6: the pair tightens", [&] { return Parallax({{5, 400, {80, 2.6}}}, 3); }, direction3, REFUSED, "the pair's 80-degree rays hold the direction to 2.15 degrees, whatever the model's 3 degrees of parallax or the plain 3 would forgive"},
		{"rotation: heavy link off", [&] { return Rotated({{3, 800, {0, 0, 13}}, {2, 100}, {1, 100}}); }, defaults, REFUSED, "the link of 800 of the 1000 weighted inliers predicts a rotation 13 degrees off: weighted mean 10.4 over the 8-degree bar (an unweighted 4.3 would accept), while the quorum and the composed rotation pass it",
			false, "away from the rotations its 3 links predict"},
		{"rotation: check off", [&] { return Rotated({{3, 800, {0, 0, 13}}, {2, 100}, {1, 100}}); }, linkRotationOff, REGISTERED, "maxLinkRotationDisagreement 0 turns the check off: it is what refused the case above"},
		{"rotation: light links off", [&] { return Rotated({{3, 800}, {2, 100, {0, 0, 13}}, {1, 100, {0, 0, 13}}}); }, defaults, REGISTERED, "the weighted mean is 2.6 degrees (an unweighted 8.7 would refuse)"},
		{"rotation: no registered link", [&] { return Rotated({{5, 800, {0, 0, 30}}}); }, defaults, REGISTERED, "an image whose only pair is to the unregistered image 5 is not judged (the loop does not try it: RegisterImage alone)",
			true},
	};
	const auto Describe = [](Expect e) {
		return e == REFUSED ? "refused" : e == REGISTERED ? "registered" : e == RIGHT_ROTATION ? "registered at its true rotation" :
			e == RIGHT_POSE ? "registered at its true pose" : e == RIGHT_SHAPE ? "registered, the model its true shape" :
			"registered more than 15 degrees off";
	};
	for (const Case& c : cases) {
		ArcScene arc(c.scene());
		std::optional<LogCapture> log;
		if (c.logged != NULL)
			log.emplace();
		Resection resection(arc.scene, c.config);
		if (c.single)
			resection.RegisterImage(4);
		else
			resection.RegisterImages();
		const Image& img = arc.scene.images[4];
		const bool registered = img.HasPose();
		const double rotErr = registered ? RotationErrorDeg(img.R, arc.truth[4].R) : 0.0;
		const REAL centreErr = registered ? norm(img.C - arc.truth[4].C) : REAL(0);
		const PoseError shape = PoseErrors(arc.scene, arc.truth);
		const bool met = c.expect == REFUSED ? !registered : c.expect == REGISTERED ? registered :
			c.expect == RIGHT_ROTATION ? registered && rotErr <= 1.0 :
			c.expect == RIGHT_POSE ? registered && rotErr <= 1.0 && centreErr <= 0.01 * ArcScene::EXTENT :
			c.expect == RIGHT_SHAPE ? registered && rotErr <= 1.0 && shape.rotationMax <= 1.0 && shape.positionMax <= 0.01 : registered && rotErr > 15.0;
		TEST_CHECK(met, "%s: image 4 %s (%.2f deg and %.4f units off its true pose; the model %.2f deg and %.4f of its extent "
			"off the truth after the best similarity), expected %s: %s", c.name, registered ? "registered" : "was refused", rotErr,
			centreErr, shape.rotationMax, shape.positionMax, Describe(c.expect), c.why);
		TEST_CHECK(c.logged == NULL || log->Contains(c.logged), "%s: the log does not say \"%s\": %s", c.name, c.logged, c.why);
	}

	// One pair alone: a pose shifted sideways by 0.14 of a unit baseline is 7.97 degrees off the direction the pair
	// predicts; the tolerance follows the pair's ray angle, atan(tan(5 deg) / rayAngle) as seen from what it triangulates
	{
		Pose3D neighbour, image, placed;
		neighbour.R = image.R = placed.R = Matrix3x3::IDENTITY;
		neighbour.C = Point3(0, 0, 0);
		image.C = Point3(1, 0, 0);
		placed.C = Point3(1, 0.14, 0);
		ImagePair pair(0, 1);
		pair.relativePose = image / neighbour;
		pair.weightedInliers = 100.f;
		pair.weightSpatial = pair.overlapArea = pair.weightConnectivity = 1.f;
		struct Tolerance { float rayAngleDeg; bool within; const char* why; };
		for (const Tolerance& t : {
			Tolerance{4.f, true, "a pair whose rays are 4 degrees apart cannot place its image nearer than 50 degrees of direction"},
			Tolerance{60.f, false, "a pair whose rays are 60 degrees apart fixes the direction to about the rotation tolerance"},
			Tolerance{0.f, false, "a pair of unknown ray angle is held to the plain tolerance"}})
		{
			pair.meanRayAngle = (float)D2R(t.rayAngleDeg);
			const PairDisagreement d = MeasurePairDisagreement(pair, placed, neighbour, 0);
			TEST_CHECK(d.rotation <= 1e-3 && d.HasDirection() && ABS(d.direction - R2D(ATAN(0.14))) <= 0.1,
				"the pair measures %.2f degrees of rotation and %.2f of direction against a pose shifted by 0.14 of its baseline", d.rotation, d.direction);
			TEST_CHECK(d.Within(5.f) == t.within, "at a ray angle of %.0f degrees the %.2f-degree direction error was %s the 5-degree tolerance: %s",
				t.rayAngleDeg, d.direction, t.within ? "held against" : "forgiven by", t.why);
		}
	}
	return scope.Pass("%u resections on the arc met their expectation: the share, the composed rotation, the quorum, the weighted "
		"link direction with its parallax tolerance and the weighted link rotation; one pair's tolerance follows its ray angle",
		(unsigned)(sizeof(cases) / sizeof(cases[0])));
}

// An image whose evidence lies almost entirely in unregistered images is registered all the same,
// once the candidates with more of their evidence in the model have had their turn: the next image
// is the one whose verified pairs to registered images weigh most, so image 4, joined to the model
// by a pair of 20 weighted inliers (2% of its pair weight), is tried after images 6 and 7 (300 and
// 200), although it holds the most correspondences (100 against 30), and its pose, right, enters
// the model however little of its evidence the model holds.
bool ResectionWeakEvidenceTest()
{
	TD_TIMER_START();
	std::mt19937 rng(20261025);
	Scene scene;
	std::vector<Pose3D> truePoses;
	BuildResectionEvidenceScene(scene, truePoses, rng);
	Resection resection(scene, ResectionConfig());
	resection.RegisterImages();
	for (IIndex imageID : {IIndex(4), IIndex(6), IIndex(7)}) {
		if (!scene.images[imageID].HasPose()) {
			VERBOSE("ResectionWeakEvidenceTest FAILED: image %u was left out%s", imageID, imageID == 4 ?
				"; its pose is right, only 2% of its pair weight lies in registered images" : "");
			return false;
		}
		if (RotationErrorDeg(scene.images[imageID].R, truePoses[imageID].R) > 1.0) {
			VERBOSE("ResectionWeakEvidenceTest FAILED: image %u was registered %.2f deg off the truth", imageID,
				RotationErrorDeg(scene.images[imageID].R, truePoses[imageID].R));
			return false;
		}
	}
	const size_t at4 = FirstAttempt(resection.GetStats(), 4, true), at6 = FirstAttempt(resection.GetStats(), 6, true), at7 = FirstAttempt(resection.GetStats(), 7, true);
	if (!(at6 < at7 && at7 < at4) || FirstAttempt(resection.GetStats(), 4, false) != String::npos) {
		VERBOSE("ResectionWeakEvidenceTest FAILED: images 6, 7 and 4 were not registered in that order (registered pair "
			"weights 300, 200 and 20), or image 4 failed an attempt");
		return false;
	}

	VERBOSE("ResectionWeakEvidenceTest PASSED: the image with the least registered pair weight is tried last and its right "
		"pose is registered however little of its evidence the model holds (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The next image is the candidate whose verified pairs to registered images weigh most, not the one
// with the most 2D-3D correspondences, and every candidate holding minCorrespondences of them is in
// the list. Seven cameras of the arc, images 0-3 registered with 150 tracks. Image 4 holds 100
// correspondences at random pixels (its pose cannot be estimated) and its pairs are to registered
// image 3 (20 weighted inliers) and unregistered image 6 (1000); image 5 holds 25 true
// correspondences, under 0.3 of image 4's 100, and a pair of 300 to image 3. Image 5 is tried first
// and registers; image 4 is tried after it and fails. The list is read again after each
// registration: in a second scene images 4, 5 and 6 hold 30 true correspondences each, images 4
// and 5 pairs of 400 and 100 to image 3, image 6 a pair of 1000 to image 4 alone; image 4 registers
// first, and image 6, which its registration made the heaviest, comes before image 5.
bool ResectionNextImageOrderTest()
{
	TD_TIMER_START();
	std::mt19937 rng(20261025);
	Scene scene;
	BuildResectionArc(scene, 7);
	AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
	AddResectionCorrespondences(scene, 4, 0, 100, rng);
	AddResectionCorrespondences(scene, 5, 25, 0, rng);
	AddResectionPair(scene, 3, 4, 20);
	AddResectionPair(scene, 4, 6, 1000);
	AddResectionPair(scene, 3, 5, 300);
	const Pose3D truePose5 = static_cast<const Pose3D&>(scene.images[5]);
	for (IIndex imageID = 4; imageID < 7; ++imageID)
		scene.images[imageID].InvalidatePose();
	scene.status.nCalibratedImages = 4;
	TriangulateTracks(scene, false, 4.f, 1.f);

	Resection resection(scene, ResectionConfig());
	resection.RegisterImages();
	if (scene.images[4].HasPose()) {
		VERBOSE("ResectionNextImageOrderTest FAILED: image 4 was registered on random correspondences, so the scene does not "
			"pose the problem");
		return false;
	}
	if (!scene.images[5].HasPose() || RotationErrorDeg(scene.images[5].R, truePose5.R) > 1.0) {
		VERBOSE("ResectionNextImageOrderTest FAILED: image 5 was %s; its 25 true correspondences and its pair of 300 to a "
			"registered image make it the next image, although it holds under 0.3 of image 4's 100 correspondences",
			scene.images[5].HasPose() ? "registered off its true pose" : "left out");
		return false;
	}
	const size_t failed4 = FirstAttempt(resection.GetStats(), 4, false);
	if (failed4 == String::npos || FirstAttempt(resection.GetStats(), 5, true) > failed4) {
		VERBOSE("ResectionNextImageOrderTest FAILED: image 4 was %s; image 5, whose pairs to registered images weigh 300 "
			"against image 4's 20, is the one to try first", failed4 == String::npos ? "never tried" : "tried before image 5");
		return false;
	}
	{
		std::mt19937 rng(20261025);
		Scene scene;
		BuildResectionArc(scene, 7);
		AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
		for (IIndex imageID = 4; imageID < 7; ++imageID)
			AddResectionCorrespondences(scene, imageID, 30, 0, rng);
		AddResectionPair(scene, 3, 4, 400);
		AddResectionPair(scene, 3, 5, 100);
		AddResectionPair(scene, 4, 6, 1000);
		for (IIndex imageID = 4; imageID < 7; ++imageID)
			scene.images[imageID].InvalidatePose();
		scene.status.nCalibratedImages = 4;
		TriangulateTracks(scene, false, 4.f, 1.f);
		Resection resection(scene, ResectionConfig());
		resection.RegisterImages();
		const size_t at4 = FirstAttempt(resection.GetStats(), 4, true), at5 = FirstAttempt(resection.GetStats(), 5, true), at6 = FirstAttempt(resection.GetStats(), 6, true);
		if (at4 == String::npos || at5 == String::npos || at6 == String::npos || !(at4 < at6 && at6 < at5)) {
			VERBOSE("ResectionNextImageOrderTest FAILED: images 4, 6 and 5 were not registered in that order; once image 4 is "
				"registered, image 6's pair of 1000 to it outweighs image 5's 100, so the list read again puts image 6 next");
			return false;
		}
	}

	VERBOSE("ResectionNextImageOrderTest PASSED: the candidate whose pairs to registered images weigh most is tried first, "
		"whatever its share of the best correspondence count, and the list is read again after each registration (%s)",
		TD_TIMER_GET_FMT().c_str());
	return true;
}

// A round tries the candidates down the list until one registers, the last one too: seven cameras of
// the arc, images 0-3 registered with 150 tracks; images 4 and 5 hold 100 and 90 correspondences at
// random pixels (their poses cannot be estimated) and the heaviest pairs to registered images (1000
// and 500), image 6 holds 25 true correspondences, under 0.3 of image 4's 100, and a pair of 50.
// Images 4 and 5 are tried and fail, image 6 is tried after them and registers.
bool ResectionTriesEveryCandidateTest()
{
	TD_TIMER_START();
	std::mt19937 rng(20261025);
	Scene scene;
	BuildResectionArc(scene, 7);
	AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
	AddResectionCorrespondences(scene, 4, 0, 100, rng);
	AddResectionCorrespondences(scene, 5, 0, 90, rng);
	AddResectionCorrespondences(scene, 6, 25, 0, rng);
	AddResectionPair(scene, 3, 4, 1000);
	AddResectionPair(scene, 3, 5, 500);
	AddResectionPair(scene, 3, 6, 50);
	const Pose3D truePose6 = static_cast<const Pose3D&>(scene.images[6]);
	for (IIndex imageID = 4; imageID < 7; ++imageID)
		scene.images[imageID].InvalidatePose();
	scene.status.nCalibratedImages = 4;
	TriangulateTracks(scene, false, 4.f, 1.f);

	Resection resection(scene, ResectionConfig());
	resection.RegisterImages();
	if (scene.images[4].HasPose() || scene.images[5].HasPose()) {
		VERBOSE("ResectionTriesEveryCandidateTest FAILED: an image was registered on random correspondences, so the scene "
			"does not pose the problem");
		return false;
	}
	if (!scene.images[6].HasPose() || RotationErrorDeg(scene.images[6].R, truePose6.R) > 1.0) {
		VERBOSE("ResectionTriesEveryCandidateTest FAILED: image 6 was %s; the candidates ahead of it failed, so it is the one "
			"to try next, whatever its share of the best correspondence count",
			scene.images[6].HasPose() ? "registered off its true pose" : "left out");
		return false;
	}
	const size_t registered6 = FirstAttempt(resection.GetStats(), 6, true);
	if (FirstAttempt(resection.GetStats(), 4, false) > registered6 || FirstAttempt(resection.GetStats(), 5, false) > registered6) {
		VERBOSE("ResectionTriesEveryCandidateTest FAILED: image 6 was tried before images 4 and 5, whose pairs to registered "
			"images weigh more");
		return false;
	}

	VERBOSE("ResectionTriesEveryCandidateTest PASSED: the candidates are tried down the list until one registers, the last "
		"one too (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The average inlier ratio that forces a full bundle adjustment when it falls under
// ResectionConfig::avgInliersRatioForceBA reads the registered images alone: an image whose attempt
// fails or is refused is not in the model, so the zero it would add says nothing about how well the
// model fits. Nine cameras of the arc, images 0-3 registered with 150 tracks. Image 4 holds the most
// correspondences, 30 at the projections its pose gives and 120 at random pixels, a 20% share that
// the acceptance refuses (minInlierRatio 25%), and it is tried first in every round: each of images 4-8
// has one link, to image 3, all of one weight, so the count (150 against 100) decides; images 5-8 hold
// 70 true and 30 random each, a 70% share, above the 60% bar, and one of them registers in each round.
// Counting the refusals as zeros, the average falls to 0.35 at the third registration and forces a
// full adjustment there, a third one besides the two every such resection runs; counting the
// registrations alone it stays at 0.7, and the full adjustments are only those two: the one the
// stall retry runs once image 4 is the only candidate left, and the one that closes the resection.
bool ResectionRefusedAverageTest()
{
	TD_TIMER_START();
	constexpr IIndex REFUSED = 4;
	constexpr IIndex NUM_IMAGES = 9;
	std::mt19937 rng(20261020);
	Scene scene;
	BuildResectionArc(scene, NUM_IMAGES);
	AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
	AddResectionCorrespondences(scene, REFUSED, 30, 120, rng);
	for (IIndex imageID = REFUSED + 1; imageID < NUM_IMAGES; ++imageID)
		AddResectionCorrespondences(scene, imageID, 70, 30, rng);
	for (IIndex imageID = REFUSED; imageID < NUM_IMAGES; ++imageID)
		AddResectionPair(scene, 3, imageID, 100);
	std::vector<Pose3D> truePoses;
	for (const Image& img : scene.images)
		truePoses.push_back(static_cast<const Pose3D&>(img));
	for (IIndex imageID = REFUSED; imageID < NUM_IMAGES; ++imageID)
		scene.images[imageID].InvalidatePose();
	scene.status.nCalibratedImages = 4;
	TriangulateTracks(scene, false, 4.f, 1.f);

	Resection resection(scene, ResectionConfig());
	resection.RegisterImages();
	if (scene.images[REFUSED].HasPose()) {
		VERBOSE("ResectionRefusedAverageTest FAILED: image %u was registered at a 20%% inlier share, so the scene does not "
			"pose the problem", REFUSED);
		return false;
	}
	for (IIndex imageID = REFUSED + 1; imageID < NUM_IMAGES; ++imageID) {
		if (!scene.images[imageID].HasPose() || RotationErrorDeg(scene.images[imageID].R, truePoses[imageID].R) > 1.0) {
			VERBOSE("ResectionRefusedAverageTest FAILED: image %u was not registered at its true pose", imageID);
			return false;
		}
	}
	const unsigned numFullAdjustments = resection.GetStats().numFullAdjustments;
	if (numFullAdjustments != 2) {
		VERBOSE("ResectionRefusedAverageTest FAILED: %u full adjustments ran, expected only the stall retry's and the one "
			"that closes the resection: the refusals of image %u pulled the average inlier ratio under its bar",
			numFullAdjustments, REFUSED);
		return false;
	}

	VERBOSE("ResectionRefusedAverageTest PASSED: an image whose pose is refused adds nothing to the average inlier "
		"ratio, so it forces no full adjustment (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// A resection that stalls with images registered since its last full adjustment runs one and tries
// again: the adjustment re-triangulates the tracks the registrations made triangulable, which can
// give the remaining images the correspondences they lacked. Seven cameras of the arc, images 0-3
// registered with 150 tracks. Image 4 sees 60 of them, so it is the first image registered; the 40
// tracks B that images 5 and 6 see are observed by images 3-6, so they gain their second registered
// view with image 4, but nothing triangulates them before the next round (triangulateEvery is off,
// a local or a scheduled adjustment is not due, and more than one candidate is left untried). Images
// 5 and 6 then find either no 2D-3D correspondence (the resection finds no image to attempt) or 16
// each, 4 at their true projections and 12 at random pixels, on which they fail to register (every
// candidate fails): either way the resection stalls with one registration since its last
// adjustment. Stopping there leaves both out; the adjustment of the retry triangulates B and
// registers both at their true poses. In a third scene the tracks of images 5 and 6 are seen by no
// registered image: the adjustment unlocks nothing, the second stall comes with no registration
// since it and the resection stops, so the loop terminates. Every resection closes with its full
// adjustment, whatever ran before it, so each of the three runs two (the retry's and the closing
// one), and a resection that registers nothing runs the closing one alone.
bool ResectionStallRetryTest()
{
	TD_TIMER_START();
	constexpr IIndex NUM_IMAGES = 7;
	struct Case { bool unlockable; unsigned numTrueOnA; unsigned numNoiseOnA; const char* why; };
	const Case cases[] = {
		{ true,  0, 0,  "no image has a correspondence left (the resection finds no image to attempt)" },
		{ true,  4, 12, "the images with correspondences fail to register on them (every candidate fails)" },
		{ false, 0, 0,  "the adjustment unlocks nothing, so the retry has to stop" },
	};
	for (const Case& c : cases) {
		std::mt19937 rng(20261021);
		Scene scene;
		BuildResectionArc(scene, NUM_IMAGES);
		AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
		AddResectionCorrespondences(scene, 4, 60, 0, rng);
		if (c.numTrueOnA + c.numNoiseOnA > 0) {
			AddResectionCorrespondences(scene, 5, c.numTrueOnA, c.numNoiseOnA, rng);
			AddResectionCorrespondences(scene, 6, c.numTrueOnA, c.numNoiseOnA, rng);
		}
		const size_t numTracksA = scene.tracks.size();
		AddResectionTracks(scene, c.unlockable ? std::vector<IIndex>{3, 4, 5, 6} : std::vector<IIndex>{5, 6}, 40, rng);
		AddResectionPair(scene, 3, 4, 300);
		AddResectionPair(scene, 4, 5, 200);
		AddResectionPair(scene, 4, 6, 200);
		std::vector<Pose3D> truePoses;
		for (const Image& img : scene.images)
			truePoses.push_back(static_cast<const Pose3D&>(img));
		for (IIndex imageID = 4; imageID < NUM_IMAGES; ++imageID)
			scene.images[imageID].InvalidatePose();
		scene.status.nCalibratedImages = 4;
		TriangulateTracks(scene, false, 4.f, 1.f);
		for (size_t t = numTracksA; t < scene.tracks.size(); ++t) {
			if (scene.tracks[t].IsInlier()) {
				VERBOSE("ResectionStallRetryTest FAILED: a track one registered image sees was triangulated, so the scene does "
					"not pose the problem");
				return false;
			}
		}

		Resection resection(scene, ResectionConfig());
		resection.RegisterImages();
		if (!scene.images[4].HasPose() || RotationErrorDeg(scene.images[4].R, truePoses[4].R) > 1.0) {
			VERBOSE("ResectionStallRetryTest FAILED: image 4 was not registered at its true pose");
			return false;
		}
		for (IIndex imageID = 5; imageID < NUM_IMAGES; ++imageID) {
			const bool registered = scene.images[imageID].HasPose();
			if (registered != c.unlockable) {
				VERBOSE("ResectionStallRetryTest FAILED: image %u was %s where %s", imageID,
					registered ? "registered" : "left out", c.why);
				return false;
			}
			if (registered && RotationErrorDeg(scene.images[imageID].R, truePoses[imageID].R) > 1.0) {
				VERBOSE("ResectionStallRetryTest FAILED: image %u was not registered at its true pose", imageID);
				return false;
			}
		}
		const unsigned numFullAdjustments = resection.GetStats().numFullAdjustments;
		if (numFullAdjustments != 2) {
			VERBOSE("ResectionStallRetryTest FAILED: %u full adjustments ran where %s, expected the retry's and the one that "
				"closes the resection", numFullAdjustments, c.why);
			return false;
		}
	}

	// the resection registers nothing (image 4's correspondences are random pixels): the closing
	// adjustment runs all the same
	{
		std::mt19937 rng(20261021);
		Scene scene;
		BuildResectionArc(scene, 5);
		AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
		AddResectionCorrespondences(scene, 4, 0, 100, rng);
		scene.images[4].InvalidatePose();
		scene.status.nCalibratedImages = 4;
		TriangulateTracks(scene, false, 4.f, 1.f);
		Resection resection(scene, ResectionConfig());
		resection.RegisterImages();
		if (scene.images[4].HasPose()) {
			VERBOSE("ResectionStallRetryTest FAILED: image 4 was registered on random correspondences, so the scene does not "
				"pose the problem");
			return false;
		}
		const unsigned numFullAdjustments = resection.GetStats().numFullAdjustments;
		if (numFullAdjustments != 1) {
			VERBOSE("ResectionStallRetryTest FAILED: %u full adjustments ran in a resection that registered nothing, expected "
				"the one that closes it", numFullAdjustments);
			return false;
		}
	}

	VERBOSE("ResectionStallRetryTest PASSED: a resection that stalls after registrations runs its full adjustment and tries "
		"again, whether no image or no candidate registers, stops when the adjustment unlocks nothing, and always closes with "
		"its full adjustment (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The full adjustment that ends a resection runs whether or not the resection had an image to try:
// its closing form triangulates every track again at the resection's bar (maxReprojError, 4 px) and
// adjusts, which re-admits the observations that the final adjustment's tighter bar (2 px) dropped
// and the resection's own bar keeps. Four cameras of the arc, 150 tracks seen by all of them, and
// 60 tracks seen by images 0 and 1 alone, whose observation in image 0 is 6 px off vertically,
// across the epipolar lines of the arc: the two views share the error, about 3 px each, under the
// 4 px bar and over the 2 px one. The scene is triangulated at 4 px and then filtered at 2 px, as the
// final adjustment leaves it, and the 60 tracks are not valid. Case 1: every image holds a pose, as
// when the merge registered them all. Case 2: the only image without a pose is withheld by
// ImagesRemoved, as the weak-image filter withholds an image the model contradicts; it holds 100
// true correspondences and a link to image 3, so it registers when it is not withheld. In both the resection has no image
// to try, registers nothing, still logs that it had none, and runs its closing adjustment once,
// after which the 60 tracks are valid again.
bool ResectionClosingAlwaysTest()
{
	TD_TIMER_START();
	constexpr unsigned NUM_GOOD = 150, NUM_BAD = 60;
	constexpr float FINE_BAR = 2.f; // the bar of the final adjustment (ReconstructionConfig::maxFineReprojError)
	const ResectionConfig config;
	const auto countValid = [](const Scene& scene) {
		unsigned numValid = 0;
		for (const Track& track : scene.tracks)
			numValid += track.IsInlier();
		return numValid;
	};
	// the scene the resection receives: the images 0-3 posed, the tracks triangulated at the resection's
	// bar and then filtered at the fine one; image 4, when there is one, has no pose
	const auto buildScene = [&](Scene& scene, unsigned numImages) {
		std::mt19937 rng(20261026);
		BuildResectionArc(scene, numImages);
		AddResectionTracks(scene, {0, 1, 2, 3}, NUM_GOOD, rng);
		if (numImages > 4)
			AddResectionCorrespondences(scene, 4, 100, 0, rng);
		AddResectionTracks(scene, {0, 1}, NUM_BAD, rng);
		for (size_t t = NUM_GOOD; t < scene.tracks.size(); ++t)
			scene.images[0].keypoints[scene.tracks[t].observations[0].featureID].pt.y += 6.f;
		if (numImages > 4) {
			AddResectionPair(scene, 3, 4, 300);
			scene.images[4].InvalidatePose();
			scene.status.nCalibratedImages = 4;
		}
		TriangulateTracks(scene, false, config.maxReprojError, config.minAngleThreshold);
		const unsigned numLoose = countValid(scene);
		FilterTracks(scene, FINE_BAR, config.minAngleThreshold, ResectionConfig::multDepthNear, ResectionConfig::multDepthFar);
		return std::make_pair(numLoose, countValid(scene));
	};
	struct Case { unsigned numImages; bool withhold; const char* what; };
	const Case cases[] = {
		{ 4, false, "every image holds a pose" },
		{ 5, true,  "the only image without a pose is withheld" },
	};
	for (const Case& c : cases) {
		Scene scene;
		const auto [numLoose, numFine] = buildScene(scene, c.numImages);
		if (numLoose != NUM_GOOD + NUM_BAD || numFine != NUM_GOOD) {
			VERBOSE("ResectionClosingAlwaysTest FAILED: %u tracks are valid at %g px and %u at %g px, expected %u and %u, so "
				"the scene does not pose the problem", numLoose, config.maxReprojError, numFine, FINE_BAR, NUM_GOOD + NUM_BAD, NUM_GOOD);
			return false;
		}
		if (c.withhold) {
			// the image registers when it is not withheld
			Scene control;
			buildScene(control, c.numImages);
			Resection(control, config).RegisterImages();
			if (!control.images[4].HasPose()) {
				VERBOSE("ResectionClosingAlwaysTest FAILED: image 4 was not registered without ImagesRemoved, so the scene "
					"does not pose the problem");
				return false;
			}
		}

		LogCapture log;
		Resection resection(scene, config);
		if (c.withhold) {
			RemovedImages withheld;
			withheld.all.push_back(IIndex(4));
			withheld.contradicting.push_back(IIndex(4));
			resection.ImagesRemoved(withheld);
		}
		const bool registered = resection.RegisterImages();
		const unsigned numClosed = countValid(scene);
		if (numClosed != NUM_GOOD + NUM_BAD) {
			VERBOSE("ResectionClosingAlwaysTest FAILED: %s, and %u of the %u tracks are valid after the resection, the %u that "
				"the final adjustment's %g px bar dropped are not back: the closing adjustment did not triangulate them again "
				"at %g px", c.what, numClosed, NUM_GOOD + NUM_BAD, NUM_BAD, FINE_BAR, config.maxReprojError);
			return false;
		}
		const unsigned numFullAdjustments = resection.GetStats().numFullAdjustments;
		if (numFullAdjustments != 1) {
			VERBOSE("ResectionClosingAlwaysTest FAILED: %s, and %u full adjustments ran, expected the one that closes the "
				"resection", c.what, numFullAdjustments);
			return false;
		}
		if (log.Count("warning: no unregistered images") != 1) {
			VERBOSE("ResectionClosingAlwaysTest FAILED: %s, and the resection did not log that it had no unregistered image",
				c.what);
			return false;
		}
		if (registered || (c.withhold && scene.images[4].HasPose())) {
			VERBOSE("ResectionClosingAlwaysTest FAILED: %s, and the resection reports having registered an image, so "
				"a caller would filter the weakly connected images again", c.what);
			return false;
		}
	}

	VERBOSE("ResectionClosingAlwaysTest PASSED: a resection with no image to try, because every image holds a pose or the "
		"only ones left are withheld, still closes with its full adjustment, which brings the tracks the fine bar dropped "
		"back (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// A filter that removes images from the model right before a resection changes the model as a
// registration does: the tracks of the removed images lost views. Six cameras of the arc, as the
// whole-scene pass receives them after the final adjustment. Images 0-3 share 150 tracks. Image 4 is
// posed but observes only 40 two-view tracks with image 3, so the covisibility graph holds no edge for
// it and FilterWeaklyConnectedImages removes it (for want of evidence: no pair contradicts it). Image 5
// has no pose, a link to image 1, and observes 60 tracks with images 0 and 1 alone, whose observation
// in image 0 is 6 px off across the epipolar lines of the arc: triangulated at the resection's bar
// (4 px) they are valid, filtered at the final adjustment's (2 px) they are not. So image 5 holds no
// correspondence, the resection finds no image to attempt and stalls before registering anything.
// Told of the removal (ImagesRemoved), the stall runs the full adjustment, which triangulates the 60
// tracks again, and the next pass registers image 5 at its true pose. In the control image 4 also sees
// the 150 tracks, the filter removes nothing, the model is unchanged since its last adjustment, and
// the same stall ends the resection with image 5 left out and only the closing adjustment run.
bool ResectionRemovalStallTest()
{
	TD_TIMER_START();
	constexpr unsigned NUM_SHARED = 150, NUM_PAIRWISE = 40, NUM_BAD = 60;
	constexpr float FINE_BAR = 2.f; // the bar of the final adjustment (ReconstructionConfig::maxFineReprojError)
	const ResectionConfig config;
	struct Case { bool removed; const char* what; };
	const Case cases[] = {
		{ true,  "the filter removed image 4" },
		{ false, "the filter removed nothing" },
	};
	for (const Case& c : cases) {
		std::mt19937 rng(20261103);
		Scene scene;
		BuildResectionArc(scene, 6);
		AddResectionTracks(scene, c.removed ? std::vector<IIndex>{0, 1, 2, 3} : std::vector<IIndex>{0, 1, 2, 3, 4}, NUM_SHARED, rng);
		AddResectionTracks(scene, {3, 4}, NUM_PAIRWISE, rng);
		const size_t firstBad = scene.tracks.size();
		AddResectionTracks(scene, {0, 1, 5}, NUM_BAD, rng);
		for (size_t t = firstBad; t < scene.tracks.size(); ++t)
			scene.images[0].keypoints[scene.tracks[t].observations[0].featureID].pt.y += 6.f;
		const Pose3D truePose5 = static_cast<const Pose3D&>(scene.images[5]);
		AddResectionPair(scene, 1, 5, 200);
		scene.images[5].InvalidatePose();
		scene.status.nCalibratedImages = 5;
		TriangulateTracks(scene, false, config.maxReprojError, config.minAngleThreshold);
		FilterTracks(scene, FINE_BAR, config.minAngleThreshold, ResectionConfig::multDepthNear, ResectionConfig::multDepthFar);
		for (size_t t = firstBad; t < scene.tracks.size(); ++t) {
			if (scene.tracks[t].IsInlier()) {
				VERBOSE("ResectionRemovalStallTest FAILED: %s, and a track of image 5 is valid at %g px, so the scene does not "
					"pose the problem", c.what, FINE_BAR);
				return false;
			}
		}
		const RemovedImages removed = FilterWeaklyConnectedImages(scene, 5, 0.15f, 1.5f, 2, 0.f, 0.f);
		if ((removed.all.size() == 1 && removed.all[0] == 4) != c.removed || !removed.contradicting.empty()) {
			VERBOSE("ResectionRemovalStallTest FAILED: %s was expected, but the filter removed %u images (the first %d, %u "
				"contradicting), so the scene does not pose the problem", c.what, removed.all.size(),
				removed.all.empty() ? -1 : (int)removed.all[0], removed.contradicting.size());
			return false;
		}

		LogCapture log;
		Resection resection(scene, config);
		resection.ImagesRemoved(removed);
		resection.RegisterImages();
		const bool registered = scene.images[5].HasPose();
		if (registered != c.removed) {
			VERBOSE("ResectionRemovalStallTest FAILED: %s, and image 5 was %s: %s", c.what, registered ? "registered" : "left out",
				c.removed ? "the stall that followed the removal did not run the full adjustment that triangulates its tracks "
					"again" : "a stall with the model unchanged since its last adjustment did not end the resection");
			return false;
		}
		if (registered && RotationErrorDeg(scene.images[5].R, truePose5.R) > 1.0) {
			VERBOSE("ResectionRemovalStallTest FAILED: %s, and image 5 was not registered at its true pose", c.what);
			return false;
		}
		const unsigned numRemovalStalls = log.Count("registered since the last full adjustment, images removed)");
		const unsigned numFullAdjustments = resection.GetStats().numFullAdjustments;
		if (numRemovalStalls != (c.removed ? 1u : 0u) || numFullAdjustments != (c.removed ? 3u : 1u)) {
			VERBOSE("ResectionRemovalStallTest FAILED: %s, and %u stalls ran their adjustment for the removal and %u full "
				"adjustments ran, expected %s", c.what, numRemovalStalls, numFullAdjustments,
				c.removed ? "1, and 3 (the removal's, the one after image 5 registered, the closing one)" : "0, and 1 (the closing one)");
			return false;
		}
	}

	VERBOSE("ResectionRemovalStallTest PASSED: a stall after a filter removed images runs the full adjustment, which "
		"triangulates the tracks again and lets the next pass register what they hold, and a stall with nothing removed or "
		"registered since the last adjustment ends the resection (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool ResectionInlierBarTest()
{
	TD_TIMER_START();

	// Image 4 observes 100 tracks, each at its true projection displaced by up to 7 px in a random
	// direction: the precision of warp-sampled (dense) correspondences on a full-resolution image
	// (their median error over a reconstructed scene is 6 px), not that of detected keypoints. The
	// inlier share such an image reaches is what the bar decides, and it is what the resection reads
	// its model's health from (avgInliersRatioForceBA): under the default bar nearly every one of
	// these correspondences is an inlier, under a 4 px bar barely half of them are
	constexpr REAL MAX_WARP_OFFSET = 7;
	constexpr unsigned NUM_CORRESPONDENCES = 100;
	const auto registerImage = [](const ResectionConfig& config) {
		std::mt19937 rng(20260914);
		Scene scene;
		BuildResectionArc(scene, 5);
		AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
		Image& img = scene.images[4];
		std::uniform_real_distribution<REAL> direction(0, 2 * PI), magnitude(0, MAX_WARP_OFFSET);
		for (unsigned i = 0; i < NUM_CORRESPONDENCES; ++i) {
			Track& track = scene.tracks[i];
			const auto [proj, valid] = img.ProjectPoint(track.position);
			ASSERT(valid && Image8U::isInside(proj, img.GetSize()));
			const REAL angle = direction(rng), offset = magnitude(rng);
			track.observations.emplace_back(IIndex(4), (uint32_t)img.keypoints.size());
			img.keypoints.emplace_back(cv::Point2f((float)(proj.x + offset * COS(angle)),
				(float)(proj.y + offset * SIN(angle))), 1.f);
		}
		img.InvalidatePose();
		scene.status.nCalibratedImages = 4;
		TriangulateTracks(scene, false, 4.f, 1.f);
		Resection resection(scene, config);
		const auto [numInliers, numPoints, numDescribed] = resection.RegisterImage(IIndex(4));
		return std::make_pair(numInliers, numPoints);
	};

	const auto [inliersDefault, pointsDefault] = registerImage(ResectionConfig());
	DEBUG("ResectionInlierBarTest: %u/%u inliers under the default bar", inliersDefault, pointsDefault);
	if (pointsDefault != NUM_CORRESPONDENCES || inliersDefault < NUM_CORRESPONDENCES * 9 / 10) {
		VERBOSE("ResectionInlierBarTest FAILED: only %u of %u correspondences within %g px, the precision of a "
			"warp-sampled one, are inliers under the default bar", inliersDefault, pointsDefault, MAX_WARP_OFFSET);
		return false;
	}

	// The same image under a 4 px bar, the precision of a detected keypoint: what makes the default
	// bar the reason the inlier share above is what it is
	ResectionConfig config4px;
	config4px.ransac.threshold = 4.0;
	const auto [inliers4px, points4px] = registerImage(config4px);
	DEBUG("ResectionInlierBarTest: %u/%u inliers under a 4 px bar", inliers4px, points4px);
	if (inliers4px >= NUM_CORRESPONDENCES * 3 / 4) {
		VERBOSE("ResectionInlierBarTest FAILED: a 4 px bar counts %u of %u of those correspondences as inliers too, "
			"so the default bar is not what the inlier share rests on", inliers4px, points4px);
		return false;
	}

	VERBOSE("ResectionInlierBarTest PASSED: %u of %u correspondences within %g px are inliers under the default bar "
		"and %u under a 4 px one (%s)", inliersDefault, pointsDefault, MAX_WARP_OFFSET, inliers4px,
		TD_TIMER_GET_FMT().c_str());
	return true;
}

// Image 4 observes 300 tracks: 40 through described keypoints at their exact projections and 260
// through dense ones displaced 2.5 px the same way, the systematic error a coarse warp leaves on
// its samples. Every correspondence lies within the inlier bar, so the robust fit over all of them
// follows the far more numerous dense ones and turns the camera by most of the shift; the polish
// that closes the fit gives each dense correspondence ResectionConfig::denseFitWeight against a
// described one's 1 and brings the pose back to the described ones, where the same fit with the
// dense correspondences at full weight stays on the bias.
bool ResectionDenseFitWeightTest()
{
	TD_TIMER_START();

	constexpr unsigned NUM_TRACKS = 300, NUM_DESCRIBED = 40;
	const Point2 bias(2.5, 0);
	const auto registerImage = [&](const ResectionConfig& config, double& rotationError, double& centreError, unsigned& numInliers) {
		std::mt19937 rng(20261005);
		Scene scene;
		BuildResectionArc(scene, 5);
		AddResectionTracks(scene, {0, 1, 2, 3}, NUM_TRACKS, rng);
		Image& img = scene.images[4];
		const Pose3D truePose(img);
		for (unsigned i = 0; i < NUM_TRACKS; ++i) {
			if (i == NUM_DESCRIBED)
				img.CloseDescribedKeypoints();
			Track& track = scene.tracks[i];
			const auto [proj, valid] = img.ProjectPoint(track.position);
			ASSERT(valid && Image8U::isInside(proj, img.GetSize()));
			const Point2 pt(i < NUM_DESCRIBED ? proj : Point2(proj + bias));
			track.observations.emplace_back(IIndex(4), (uint32_t)img.keypoints.size());
			img.keypoints.emplace_back(cv::Point2f((float)pt.x, (float)pt.y), 1.f);
		}
		ASSERT(img.NumDescribedKeypoints() == NUM_DESCRIBED && img.IsDenseKeypoint(NUM_DESCRIBED));
		img.InvalidatePose();
		scene.status.nCalibratedImages = 4;
		TriangulateTracks(scene, false, 4.f, 1.f);
		Resection resection(scene, config);
		numInliers = std::get<0>(resection.RegisterImage(IIndex(4)));
		if (!img.HasPose())
			return false;
		rotationError = RotationErrorDeg(img.R, truePose.R);
		centreError = norm(Point3(img.C - truePose.C));
		return true;
	};

	constexpr float DENSE_FIT_WEIGHT = 0.003f; // the polish's weight of a dense correspondence, as measured
	ResectionConfig configFull, configWeighted;
	configFull.denseFitWeight = 1.f;
	configWeighted.denseFitWeight = DENSE_FIT_WEIGHT;
	double rotFull, cenFull, rotWeighted, cenWeighted;
	unsigned inliersFull, inliersWeighted;
	if (!registerImage(configFull, rotFull, cenFull, inliersFull) || !registerImage(configWeighted, rotWeighted, cenWeighted, inliersWeighted)) {
		VERBOSE("ResectionDenseFitWeightTest FAILED: image 4 was not registered");
		return false;
	}
	// the premise: every correspondence is an inlier, and at full weight the dense ones pull the pose
	if (inliersFull != NUM_TRACKS || inliersWeighted != NUM_TRACKS || rotFull < 0.1) {
		VERBOSE("ResectionDenseFitWeightTest FAILED: premise not met: %u and %u of %u inliers, %.4f deg off at full weight",
			inliersFull, inliersWeighted, NUM_TRACKS, rotFull);
		return false;
	}
	if (rotWeighted > 0.25*rotFull || cenWeighted > 0.25*cenFull) {
		VERBOSE("ResectionDenseFitWeightTest FAILED: the weighted polish leaves the pose %.4f deg / %.4g off, %.4f deg / %.4g "
			"at full weight", rotWeighted, cenWeighted, rotFull, cenFull);
		return false;
	}
	VERBOSE("ResectionDenseFitWeightTest PASSED: %.4f deg / %.4g off with the dense correspondences at full weight, "
		"%.4f deg / %.4g at %g (%u dense, %u described) (%s)", rotFull, cenFull, rotWeighted, cenWeighted,
		DENSE_FIT_WEIGHT, NUM_TRACKS - NUM_DESCRIBED, NUM_DESCRIBED, TD_TIMER_GET_FMT().c_str());
	return true;
}

// The resection's inlier bars read what a pose's inliers weigh, as a pair weighs its matches: a described inlier counts
// one, a dense one PairsWeightingConfig::DenseScale() (1/40). Image 4 observes tracks at their exact projections:
// - 20 described correspondences weigh 20 and pass the bar of 12;
// - 2 described and 300 dense weigh 9.5: 302 inliers, every correspondence of the image, and the pose is refused;
// - 20 described and 100 dense among 380 further correspondences that agree on nothing (displaced 60-120 px) weigh 22.5:
//   a share under the 25% minimum that 120 inliers would waive by their count, and their weight under 100 does not.
bool ResectionWeightedInlierBarTest()
{
	TD_TIMER_START();
	const auto registerImage = [](unsigned numDescribed, unsigned numDense, unsigned numStray, unsigned& numInliers) {
		std::mt19937 rng(20261113);
		Scene scene;
		BuildResectionArc(scene, 5);
		const unsigned numTracks = numDescribed + numDense + numStray;
		AddResectionTracks(scene, {0, 1, 2, 3}, numTracks, rng);
		Image& img = scene.images[4];
		std::uniform_real_distribution<REAL> direction(0, 2 * PI), magnitude(60, 120);
		for (unsigned i = 0; i < numTracks; ++i) {
			if (i == numDescribed)
				img.CloseDescribedKeypoints();
			Track& track = scene.tracks[i];
			const auto [proj, valid] = img.ProjectPoint(track.position);
			ASSERT(valid && Image8U::isInside(proj, img.GetSize()));
			Point2 pt(proj);
			if (i >= numDescribed + numDense) {
				const REAL angle = direction(rng), offset = magnitude(rng);
				pt = Point2(proj.x + offset * COS(angle), proj.y + offset * SIN(angle));
			}
			track.observations.emplace_back(IIndex(4), (uint32_t)img.keypoints.size());
			img.keypoints.emplace_back(cv::Point2f((float)pt.x, (float)pt.y), 1.f);
		}
		img.InvalidatePose();
		scene.status.nCalibratedImages = 4;
		TriangulateTracks(scene, false, 4.f, 1.f);
		Resection resection(scene, ResectionConfig());
		numInliers = std::get<0>(resection.RegisterImage(IIndex(4)));
		return img.HasPose();
	};
	unsigned inliersDescribed, inliersDense, inliersShare;
	const bool acceptedDescribed = registerImage(20, 0, 0, inliersDescribed);
	const bool acceptedDense = registerImage(2, 300, 0, inliersDense);
	const bool acceptedShare = registerImage(20, 100, 380, inliersShare);
	if (!acceptedDescribed || acceptedDense || acceptedShare) {
		VERBOSE("ResectionWeightedInlierBarTest FAILED: 20 described inliers %s (accepted expected), 2 described and 300 dense "
			"%s (refused expected: weight 9.5 under the bar of 12), 20 described and 100 dense at a share under 25%% %s (refused "
			"expected: weight 22.5 under the 100 that waives the share)", acceptedDescribed ? "accepted" : "refused",
			acceptedDense ? "accepted" : "refused", acceptedShare ? "accepted" : "refused");
		return false;
	}
	VERBOSE("ResectionWeightedInlierBarTest PASSED: 20 described inliers pass the bar, 2 described and 300 dense weigh 9.5 and "
		"do not, 120 inliers weighing 22.5 do not waive the share (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The shared walks of the 2D-3D linking (Link2D3D.h) and of an image's links (PoseLink.h), on the arc:
// images 0-2 registered with 20 triangulated tracks, image 3 unregistered and observing the first 10
// of them, the first of those twice (two keypoints of one image a track can hold), and an outlier
// track it observes too. Its pairs: to registered 2 (300 weighted inliers) and 1 (100) with a relative
// pose, to registered 0 with a valid weight but no relative pose, to unregistered 4 (500) with one.
bool LinkWalksTest()
{
	TD_TIMER_START();
	std::mt19937 rng(20261105);
	Scene scene;
	BuildResectionArc(scene, 5);
	AddResectionTracks(scene, {0, 1, 2}, 20, rng);
	AddResectionCorrespondences(scene, 3, 10, 0, rng);
	Image& img = scene.images[3];
	// a second keypoint of image 3 on the first track, one pixel off the first
	const Observation first = scene.tracks[0].observations.back();
	ASSERT(first.imageID == 3);
	scene.tracks[0].observations.emplace_back(IIndex(3), (uint32_t)img.keypoints.size());
	img.keypoints.emplace_back(img.keypoints[first.featureID].pt + cv::Point2f(1.f, 0.f), 1.f);
	if (!AddResectionTrack(scene, Point3(0, 0, 0), {0, 1, 3})) {
		VERBOSE("LinkWalksTest FAILED: the outlier track's point does not project in its images");
		return false;
	}
	AddResectionPair(scene, 2, 3, 300);
	AddResectionPair(scene, 1, 3, 100);
	AddResectionPair(scene, 0, 3, 50);
	scene.pairs.back().relativePose.reset();
	AddResectionPair(scene, 3, 4, 500);
	AddResectionPair(scene, 1, 2, 200);
	img.InvalidatePose();
	scene.images[4].InvalidatePose();
	scene.status.nCalibratedImages = 3;
	TriangulateTracks(scene, false, 4.f, 1.f);
	Track& outlier = scene.tracks.back();
	if (!outlier.IsInlier()) {
		VERBOSE("LinkWalksTest FAILED: the extra track did not triangulate");
		return false;
	}
	outlier.numInliers = 1;

	// the PnP input: image 3's first keypoint on each inlier track, in track order, nothing of the outlier track
	CameraCorrespondences correspondences;
	CollectTrackCorrespondences(scene, 3, correspondences);
	if (correspondences.size() != 10 || correspondences.numDescribed != 10 || correspondences.dense.size() != 10) {
		VERBOSE("LinkWalksTest FAILED: %u correspondences (%u described) collected for image 3, 10 expected",
			correspondences.size(), correspondences.numDescribed);
		return false;
	}
	for (unsigned t = 0; t < 10; ++t) {
		const Track& track = scene.tracks[t];
		uint32_t featureID = NO_ID;
		for (const Observation& obs : track.observations)
			if (obs.imageID == 3) {
				featureID = obs.featureID;
				break;
			}
		const Point3 bearing(img.pCamera->UnprojectNormalized(Cast<REAL>(img.keypoints[featureID].pt)));
		if (correspondences.points[t] != Eigen::Vector3d(track.position.x, track.position.y, track.position.z) ||
			correspondences.bearings[t] != Eigen::Vector3d(bearing.x, bearing.y, bearing.z)) {
			VERBOSE("LinkWalksTest FAILED: correspondence %u is not the first observation of image 3 on track %u", t, t);
			return false;
		}
	}

	// the counts: every observation of an inlier track, and the ones in its inlier prefix
	const std::vector<TrackObservationCount> counts(CountTrackObservations(scene));
	for (IIndex i = 0; i < 3; ++i)
		if (counts[i].observations != 20 || counts[i].inliers != 20) {
			VERBOSE("LinkWalksTest FAILED: registered image %u counts %u observations, %u inliers (20 and 20 expected)",
				i, counts[i].observations, counts[i].inliers);
			return false;
		}
	if (counts[3].observations != 11 || counts[3].inliers != 0 || counts[4].observations != 0) {
		VERBOSE("LinkWalksTest FAILED: image 3 counts %u observations, %u inliers (11 and 0 expected), image 4 %u",
			counts[3].observations, counts[3].inliers, counts[4].observations);
		return false;
	}

	// the links of the model, seen from their other side: the pairs to a registered image carrying a
	// relative pose, not the one to image 0 without it
	const auto isRegistered = [&scene](IIndex i) { return scene.images[i].HasPose(); };
	std::vector<std::pair<IIndex, IIndex>> linkVisits;
	ForEachLink(scene.pairs, isRegistered, [&linkVisits](IIndex image, IIndex registered, const ImagePair&) {
		linkVisits.emplace_back(image, registered);
	});
	const std::vector<std::pair<IIndex, IIndex>> expectedLinks{{3, 2}, {3, 1}, {1, 2}, {2, 1}};
	if (linkVisits != expectedLinks) {
		VERBOSE("LinkWalksTest FAILED: %u link visits (4 expected: 3 to 2 and 1, and 1 and 2 to each other)",
			(unsigned)linkVisits.size());
		return false;
	}
	std::vector<PoseLink> links;
	CollectPoseLinks(scene.pairs, 3, isRegistered, links);
	if (links.size() != 2 || links[0].neighborID != 2 || links[0].numInliers != 300 || links[1].neighborID != 1) {
		VERBOSE("LinkWalksTest FAILED: image 3 has %u links, the strongest to image %d; 2 expected, to images 2 then 1",
			(unsigned)links.size(), links.empty() ? -1 : (int)links[0].neighborID);
		return false;
	}

	VERBOSE("LinkWalksTest PASSED: image 3 has 10 PnP correspondences (first keypoint per inlier track, the outlier track "
		"out), 11 observations counted, links to 2 then 1 and none to the unposed pair or the unregistered image (%s)",
		TD_TIMER_GET_FMT().c_str());
	return true;
}

// An image is tried for resection only once it has a link (ForEachLink: a verified pair with a relative
// pose to a registered image), on six cameras of the arc, images 0-3 registered with 150 tracks:
// (1) image 4 holds 40 true correspondences, its only pair with a relative pose goes to unregistered
// image 5 (300), and a pair to registered image 0 of 1000 weighted inliers carries no relative pose,
// so it heads the order; image 5 holds 40 true correspondences and a link to image 3 (200). Image 4
// waits, image 5 registers, and image 4 registers after it, through its link to image 5.
// (2) the order reads the links alone: image 4 also has a link to image 2 (100), image 5 holds the same 40
// correspondences and its link to image 3 (200) weighs more than image 4's links, though less than image 4's
// pair without a relative pose; image 5 is tried first and registers, then image 4.
// (3) image 5 holds no correspondence: image 4, linked to nothing registered, is never tried; once
// image 5 holds its correspondences, a second resection registers image 5 and then image 4.
bool ResectionLinkGateTest()
{
	TD_TIMER_START();
	const auto build = [](Scene& scene, std::vector<Pose3D>& truePoses, bool image5Correspondences, bool control, std::mt19937& rng) {
		BuildResectionArc(scene, 6);
		AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
		AddResectionCorrespondences(scene, 4, 40, 0, rng);
		if (image5Correspondences)
			AddResectionCorrespondences(scene, 5, 40, 0, rng);
		AddResectionPair(scene, 0, 4, 1000);
		scene.pairs.back().relativePose.reset();
		AddResectionPair(scene, 4, 5, 300);
		AddResectionPair(scene, 3, 5, 200);
		if (control)
			AddResectionPair(scene, 2, 4, 100);
		truePoses.clear();
		for (const Image& img : scene.images)
			truePoses.push_back(static_cast<const Pose3D&>(img));
		scene.images[4].InvalidatePose();
		scene.images[5].InvalidatePose();
		scene.status.nCalibratedImages = 4;
		TriangulateTracks(scene, false, 4.f, 1.f);
	};
	const auto atTruePose = [](const Scene& scene, const std::vector<Pose3D>& truePoses, IIndex i) {
		return scene.images[i].HasPose() && RotationErrorDeg(scene.images[i].R, truePoses[i].R) < 1.0;
	};
	const auto firstAttempt = [](const Resection& resection, IIndex i) {
		return MINF(FirstAttempt(resection.GetStats(), i, true), FirstAttempt(resection.GetStats(), i, false));
	};
	std::vector<Pose3D> truePoses;

	// (1) the image without a link waits for its paired image, then registers through it
	{
		std::mt19937 rng(20261105);
		Scene scene;
		build(scene, truePoses, true, false, rng);
		Resection resection(scene, ResectionConfig());
		resection.RegisterImages();
		const size_t registered5 = FirstAttempt(resection.GetStats(), 5, true), tried4 = firstAttempt(resection, 4);
		if (!atTruePose(scene, truePoses, 5) || !atTruePose(scene, truePoses, 4) || registered5 == String::npos || tried4 < registered5) {
			VERBOSE("ResectionLinkGateTest FAILED: image 4, whose only pair with a relative pose goes to unregistered image 5, "
				"was %s; it must wait until image 5 is registered and then register at its true pose (image 5 %s)",
				tried4 < registered5 ? "tried before image 5 was registered" : (atTruePose(scene, truePoses, 4) ? "registered" : "left out or off its pose"),
				atTruePose(scene, truePoses, 5) ? "registered at its true pose" : "not registered at its true pose");
			return false;
		}
	}

	// (2) an image with a link is tried, in the order of its links' weight; its pair without a relative pose counts for nothing
	{
		std::mt19937 rng(20261105);
		Scene scene;
		build(scene, truePoses, true, true, rng);
		Resection resection(scene, ResectionConfig());
		resection.RegisterImages();
		const size_t registered5 = FirstAttempt(resection.GetStats(), 5, true), tried4 = firstAttempt(resection, 4);
		if (!atTruePose(scene, truePoses, 4) || !atTruePose(scene, truePoses, 5) || registered5 == String::npos || tried4 < registered5) {
			VERBOSE("ResectionLinkGateTest FAILED: image 5, whose link to image 3 (200) outweighs image 4's link to image 2 (100), was %s; "
				"image 4's pair of 1000 to image 0 has no relative pose and must not count in the order (image 4 %s)",
				tried4 < registered5 ? "tried after image 4" : "not registered at its true pose",
				atTruePose(scene, truePoses, 4) ? "registered at its true pose" : "not registered at its true pose");
			return false;
		}
	}

	// (3) an image linked to nothing registered is never tried; it registers once its paired image is
	{
		std::mt19937 rng(20261105);
		Scene scene;
		build(scene, truePoses, false, false, rng);
		{
			Resection resection(scene, ResectionConfig());
			resection.RegisterImages();
			if (scene.images[4].HasPose() || firstAttempt(resection, 4) != String::npos) {
				VERBOSE("ResectionLinkGateTest FAILED: image 4, which holds 40 correspondences but no link, was %s",
					scene.images[4].HasPose() ? "registered" : "tried");
				return false;
			}
		}
		// image 5 now observes 40 of the tracks, at the projections its true pose gives
		Image& img5 = scene.images[5];
		static_cast<Pose3D&>(img5) = truePoses[5];
		AddResectionCorrespondences(scene, 5, 40, 0, rng);
		img5.InvalidatePose();
		Resection resection(scene, ResectionConfig());
		resection.RegisterImages();
		const size_t registered5 = FirstAttempt(resection.GetStats(), 5, true), tried4 = firstAttempt(resection, 4);
		if (!atTruePose(scene, truePoses, 5) || !atTruePose(scene, truePoses, 4) || registered5 == String::npos || tried4 < registered5) {
			VERBOSE("ResectionLinkGateTest FAILED: once image 5 holds its correspondences, it was %s and image 4 %s",
				atTruePose(scene, truePoses, 5) ? "registered" : "not registered at its true pose",
				tried4 < registered5 ? "was tried before it" : (atTruePose(scene, truePoses, 4) ? "registered" : "was left out or off its pose"));
			return false;
		}
	}

	VERBOSE("ResectionLinkGateTest PASSED: an image whose pairs with a relative pose go to unregistered images waits and registers "
		"once one of them is registered, and is never tried while none is; the order follows the links' weight alone (%s)",
		TD_TIMER_GET_FMT().c_str());
	return true;
}

namespace {

// A verified pair as the image filter reads one: on top of what the resection needs (the ground
// truth relative pose and a valid weight) it carries filtered inliers, which is the count the filter
// requires before it takes a pair's word about a pose.
void AddFilterPair(Scene& scene, IIndex idA, IIndex idB, unsigned numInliers)
{
	AddResectionPair(scene, idA, idB, numInliers);
	ImagePair& pair = scene.pairs.back();
	pair.matches.resize(numInliers);
	pair.numFilteredInliers = (int)numInliers;
}

// Eight cameras of the resection arc, all at their true poses. Cameras 0-3 share 150 tracks, so they
// alone carry the covisibility graph; cameras 4-7 observe nothing but two-view tracks with their
// neighbours, which the covisibility count (three inlier views to an edge) never sees, so the graph
// holds no edge for any of them and the filter cuts every one. They are joined to the model by
// verified pairs alone. Cameras 4 and 5 each have two pairs to the kept cameras 2 and 3, plus the
// pair joining them to each other. Camera 6's two pairs run to 4 and 5, and camera 7's to 5 and 6:
// neither has a pair to a kept camera.
void BuildFilterScene(Scene& scene, std::mt19937& rng)
{
	BuildResectionArc(scene, 8);
	AddResectionTracks(scene, {0, 1, 2, 3}, 150, rng);
	AddResectionTracks(scene, {3, 4}, 40, rng);
	AddResectionTracks(scene, {4, 5}, 40, rng);
	AddResectionTracks(scene, {5, 6}, 40, rng);
	AddResectionTracks(scene, {6, 7}, 40, rng);
	AddFilterPair(scene, 3, 4, 100);
	AddFilterPair(scene, 2, 4, 100);
	AddFilterPair(scene, 4, 5, 100);
	AddFilterPair(scene, 3, 5, 100);
	AddFilterPair(scene, 2, 5, 100);
	AddFilterPair(scene, 5, 6, 100);
	AddFilterPair(scene, 4, 6, 100);
	AddFilterPair(scene, 6, 7, 100);
	AddFilterPair(scene, 5, 7, 100);
	TriangulateTracks(scene, false, 4.f, 1.f);
}

// The IDs of the images still valid, for the messages below
String ValidImageList(const Scene& scene)
{
	String list;
	FOREACH(imgIdx, scene.images)
		if (scene.images[imgIdx].IsValid())
			list += String::FormatString(list.empty() ? "%u" : ",%u", imgIdx);
	return list.empty() ? String("none") : list;
}
} // namespace

// Of the images the filter removes, only the ones whose verified pairs to the kept images
// contradict the pose the model gives them are named as contradicting; an image removed because
// nothing vouches for it is not
bool ContradictingImagesTest()
{
	TD_TIMER_START();
	constexpr REAL CONTRADICTION_DISPLACEMENT = 20; // degrees camera 5's stored pose is moved by

	// Cameras 4-7 hold no covisibility edge and are all removed. Camera 5's stored pose moved 20
	// degrees along the arc contradicts its pairs to the kept cameras 2 and 3, so it is removed as
	// contradicting; camera 4's pairs to the same cameras agree with its pose, and cameras 6 and 7
	// have no pair to a kept camera at all, so none of the three is contradicting
	{
		std::mt19937 rng(20260908);
		Scene scene;
		BuildFilterScene(scene, rng);
		static_cast<Pose3D&>(scene.images[5]) =
			ResectionArcPose(5 * RESECTION_ARC_STEP + CONTRADICTION_DISPLACEMENT);
		const RemovedImages removed = FilterWeaklyConnectedImages(scene, 5, 0.15f, 1.5f, 2, 0.f, 0.f);
		if (scene.images[4].IsValid() || scene.images[5].IsValid() || scene.images[6].IsValid() || scene.images[7].IsValid()) {
			VERBOSE("ContradictingImagesTest FAILED: cameras 4-7 were expected removed (valid: %s), so the "
				"scene does not pose the problem", ValidImageList(scene).c_str());
			return false;
		}
		if (removed.contradicting.size() != 1 || removed.contradicting[0] != 5) {
			String list;
			for (const IIndex id : removed.contradicting)
				list += String::FormatString(list.empty() ? "%u" : ",%u", id);
			VERBOSE("ContradictingImagesTest FAILED: camera 5 alone contradicts the model, but the filter named %s "
				"(removed %u)", list.empty() ? "none" : list.c_str(), removed.all.size());
			return false;
		}
	}

	VERBOSE("ContradictingImagesTest PASSED: the one image whose pairs disagree with its pose is named, the three "
		"removed without a pair that does are not (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// ===============================================================================
// Blocks and the pool (SceneBlocks)
// ===============================================================================

// A two-cluster scene (A = images 0-11, B = 12-23) split into two blocks by hand and each block
// "reconstructed" on the ground truth (its images posed, tracks built and triangulated), as the pool
// resection receives the blocks: the images of `wrong` sit in block 0 (A's) although they belong to B,
// unposed as a block leaves an image it could not register, and the images of `pool` are in no block.
// The global images hold no pose, as in a reconstruction.
static void BuildPoolBlocks(Scene& scene, SceneBlocks& sceneBlocks, std::vector<Pose3D>& gtPoses,
	const std::set<IIndex>& wrong, const std::set<IIndex>& pool, const ReconstructionConfig& config)
{
	GenerateTwoClusterScene(scene, 12, 12, 10, 60, 300);
	gtPoses.resize(scene.images.size());
	FOREACH(i, scene.images) {
		gtPoses[i].R = scene.images[i].R;
		gtPoses[i].C = scene.images[i].C;
		scene.images[i].InvalidatePose();
	}
	scene.status.nCalibratedImages = 0;
	ScenePartition partition;
	partition.clusters.resize(2);
	FOREACH(i, scene.images) {
		if (pool.count(i))
			partition.pool.push_back(i);
		else
			partition.clusters[i < 12 || wrong.count(i) ? 0 : 1].push_back(i);
	}
	sceneBlocks.Extract(partition);
	FOREACH(b, sceneBlocks.blocks) {
		Scene& block = sceneBlocks.blocks[b];
		FOREACH(l, block.images) {
			const IIndex g = sceneBlocks.localToGlobal[b][l];
			if (wrong.count(g))
				continue;
			block.images[l].R = gtPoses[g].R;
			block.images[l].C = gtPoses[g].C;
		}
		BuildTracks(block, config.minPairWeight, config.trackConflictCfg);
		TriangulateTracks(block, true, config.maxReprojError, config.minAngleThreshold);
		block.RecomputeCalibratedImages();
		block.status.nState.set(Scene::Status::STATE::CALIBRATED);
	}
}

// The two blocks hold the same images, pairs and tracks, byte for byte, in the same order
static bool SameBlock(const Scene& a, const Scene& b, String& diff)
{
	if (a.cameras.size() != b.cameras.size() || a.images.size() != b.images.size() ||
		a.pairs.size() != b.pairs.size() || a.tracks.size() != b.tracks.size()) {
		diff = String::FormatString("sizes: cameras %u/%u, images %u/%u, pairs %u/%u, tracks %u/%u",
			a.cameras.size(), b.cameras.size(), a.images.size(), b.images.size(),
			a.pairs.size(), b.pairs.size(), a.tracks.size(), b.tracks.size());
		return false;
	}
	FOREACH(c, a.cameras)
		if (a.cameras[c]->GetIntrinsicsString() != b.cameras[c]->GetIntrinsicsString()) {
			diff = String::FormatString("camera %u", c);
			return false;
		}
	FOREACH(i, a.images) {
		const Image &x = a.images[i], &y = b.images[i];
		if (x.ID != y.ID || x.cameraID != y.cameraID || x.fileName != y.fileName ||
			memcmp(&x.R, &y.R, sizeof(x.R)) != 0 || memcmp(&x.C, &y.C, sizeof(x.C)) != 0 ||
			x.numDescribedKeypoints != y.numDescribedKeypoints || x.keypoints.size() != y.keypoints.size() ||
			(!x.keypoints.empty() && memcmp(x.keypoints.data(), y.keypoints.data(), x.keypoints.size() * sizeof(cv::KeyPoint)) != 0) ||
			x.descriptors.rows != y.descriptors.rows || x.descriptors.cols != y.descriptors.cols ||
			(!x.descriptors.empty() && cv::norm(x.descriptors, y.descriptors, cv::NORM_INF) != 0)) {
			diff = String::FormatString("image %u", i);
			return false;
		}
	}
	FOREACH(p, a.pairs) {
		const ImagePair &x = a.pairs[p], &y = b.pairs[p];
		if (x.ID1 != y.ID1 || x.ID2 != y.ID2 || x.matches.size() != y.matches.size() ||
			(!x.matches.empty() && memcmp(x.matches.data(), y.matches.data(), x.matches.size() * sizeof(x.matches[0])) != 0) ||
			x.GetCompositeWeight() != y.GetCompositeWeight()) {
			diff = String::FormatString("pair %u (%u,%u) vs (%u,%u)", p, x.ID1, x.ID2, y.ID1, y.ID2);
			return false;
		}
	}
	FOREACH(t, a.tracks) {
		const Track &x = a.tracks[t], &y = b.tracks[t];
		if (x.numInliers != y.numInliers || memcmp(&x.position, &y.position, sizeof(x.position)) != 0 ||
			x.observations.size() != y.observations.size() ||
			memcmp(x.observations.data(), y.observations.data(), x.observations.size() * sizeof(Observation)) != 0) {
			diff = String::FormatString("track %u", t);
			return false;
		}
	}
	return true;
}

// Insert then refuse: two pool images go into block B (their features, their pairs to B and to each
// other, B's tracks rebuilt with them) and are handed back as the pool resection hands back what a block
// refused (Release): the block's cameras, images and pairs come back byte for byte, its tracks hold no
// observation of the refused images and stay valid (what the images joined stays), and the images are in
// the pool again with their features and pairs; on the way the block keeps its local IDs ordered as the
// global ones and every pair ID1 < ID2
bool SceneBlocksInsertReleaseTest()
{
	TD_TIMER_START();
	const ReconstructionConfig config;
	ResectionConfig resectionCfg = config.resectionCfg;
	resectionCfg.DeriveBAConfigs(config.baConfig);
	Scene scene;
	SceneBlocks sceneBlocks(scene);
	std::vector<Pose3D> gtPoses;
	BuildPoolBlocks(scene, sceneBlocks, gtPoses, {}, {17, 18}, config);
	const IIndex b = 1;
	const IIndexArr images{17, 18};
	const Scene before(sceneBlocks.blocks[b]);
	const IIndexArr localToGlobalBefore(sceneBlocks.localToGlobal[b]);
	std::set<std::pair<IIndex,IIndex>> globalPairsBefore;
	for (const ImagePair& pair : scene.pairs)
		globalPairsBefore.emplace(pair.ID1, pair.ID2);
	const size_t numKeypoints[2] = {scene.images[17].keypoints.size(), scene.images[18].keypoints.size()};
	size_t numObservationsBefore = 0;
	for (const Track& track : before.tracks)
		numObservationsBefore += track.observations.size();

	sceneBlocks.Insert(b, images, config, resectionCfg);
	const Scene& block = sceneBlocks.blocks[b];
	const IIndexArr& l2g = sceneBlocks.localToGlobal[b];
	if (l2g.size() != localToGlobalBefore.size() + 2 || !std::is_sorted(l2g.begin(), l2g.end()) ||
		std::adjacent_find(l2g.begin(), l2g.end()) != l2g.end() || block.images.size() != l2g.size()) {
		VERBOSE("SceneBlocksInsertReleaseTest FAILED: after Insert the block's local IDs are not the sorted global ones");
		return false;
	}
	for (IIndex g : images) {
		if (sceneBlocks.blockOf[g] != b || scene.images[g].HasFeatures() || !std::binary_search(l2g.begin(), l2g.end(), g)) {
			VERBOSE("SceneBlocksInsertReleaseTest FAILED: image %u not moved into the block with its features", g);
			return false;
		}
	}
	FOREACH(l, block.images)
		if (block.images[l].ID != l) {
			VERBOSE("SceneBlocksInsertReleaseTest FAILED: image at %u holds ID %u", l, block.images[l].ID);
			return false;
		}
	for (const ImagePair& pair : block.pairs)
		if (pair.ID1 >= pair.ID2 || pair.ID2 >= block.images.size()) {
			VERBOSE("SceneBlocksInsertReleaseTest FAILED: pair (%u,%u) after Insert", pair.ID1, pair.ID2);
			return false;
		}
	for (const ImagePair& pair : scene.pairs)
		if (sceneBlocks.blockOf[pair.ID1] == b && sceneBlocks.blockOf[pair.ID2] == b) {
			VERBOSE("SceneBlocksInsertReleaseTest FAILED: pair (%u,%u) of two images of the block left in the global scene", pair.ID1, pair.ID2);
			return false;
		}
	size_t numObservations = 0;
	for (const Track& track : block.tracks) {
		std::set<IIndex> seen;
		for (const Observation& obs : track.observations) {
			if (obs.imageID >= block.images.size() || obs.featureID >= block.images[obs.imageID].keypoints.size() ||
				!seen.insert(obs.imageID).second) {
				VERBOSE("SceneBlocksInsertReleaseTest FAILED: a track holds an invalid or repeated observation (%u,%u)", obs.imageID, obs.featureID);
				return false;
			}
		}
		numObservations += track.observations.size();
	}
	if (block.pairs.size() <= before.pairs.size() || numObservations <= numObservationsBefore) {
		VERBOSE("SceneBlocksInsertReleaseTest FAILED: Insert took no pair (%u -> %u) or attached no keypoint (%u -> %u observations); the fixture does not exercise it",
			before.pairs.size(), block.pairs.size(), (unsigned)numObservationsBefore, (unsigned)numObservations);
		return false;
	}
	const unsigned numPairsInserted = block.pairs.size() - before.pairs.size();
	const unsigned numObservationsInserted = (unsigned)(numObservations - numObservationsBefore);

	sceneBlocks.Release(b, images);
	// the cameras, images and pairs as they were (the tracks compared apart: what the refused images joined stays)
	Scene withoutTracks(sceneBlocks.blocks[b]);
	withoutTracks.tracks = before.tracks;
	String diff;
	if (!SameBlock(before, withoutTracks, diff)) {
		VERBOSE("SceneBlocksInsertReleaseTest FAILED: the refusal did not return the block's cameras, images and pairs: %s", diff.c_str());
		return false;
	}
	for (const Track& track : sceneBlocks.blocks[b].tracks) {
		std::set<IIndex> seen;
		bool valid = track.IsValid() && track.numInliers <= track.observations.size();
		FOREACH(k, track.observations) {
			const Observation& obs = track.observations[k];
			valid &= obs.imageID < sceneBlocks.blocks[b].images.size() && seen.insert(obs.imageID).second &&
				obs.featureID < sceneBlocks.blocks[b].images[obs.imageID].keypoints.size() &&
				(k >= track.numInliers || sceneBlocks.blocks[b].images[obs.imageID].HasPose());
		}
		if (!valid) {
			VERBOSE("SceneBlocksInsertReleaseTest FAILED: a track left after the refusal is not valid (%u observations, %u inliers)",
				track.observations.size(), track.numInliers);
			return false;
		}
	}
	if (sceneBlocks.localToGlobal[b] != localToGlobalBefore) {
		VERBOSE("SceneBlocksInsertReleaseTest FAILED: Release did not return the block's local IDs");
		return false;
	}
	std::set<std::pair<IIndex,IIndex>> globalPairs;
	for (const ImagePair& pair : scene.pairs)
		globalPairs.emplace(pair.ID1, pair.ID2);
	if (globalPairs != globalPairsBefore || scene.pairs.size() != globalPairsBefore.size()) {
		VERBOSE("SceneBlocksInsertReleaseTest FAILED: the global scene holds %u pairs after Release, %u before",
			scene.pairs.size(), (unsigned)globalPairsBefore.size());
		return false;
	}
	FOREACH(i, images)
		if (sceneBlocks.blockOf[images[i]] != NO_ID || scene.images[images[i]].keypoints.size() != numKeypoints[i]) {
			VERBOSE("SceneBlocksInsertReleaseTest FAILED: image %u is not back in the pool with its features", images[i]);
			return false;
		}

	VERBOSE("SceneBlocksInsertReleaseTest PASSED: Insert took %u pairs and %u observations, the refusal returned the block's cameras, images and pairs byte for byte and left valid tracks (%s)",
		numPairsInserted, numObservationsInserted, TD_TIMER_GET_FMT().c_str());
	return true;
}

// A block that takes images through the pool ends with the tracks its own track building would have
// given it with those images from the start: images 16-18 of B are inserted together into B's block, the
// pairs among them weigh the most (consecutive frames of a real capture) and the middle one has no pair to
// the block at all, so it reaches the block only through its two neighbours; the tracks after Insert, as
// sets of observations in global IDs, are the tracks of BuildTracks over the block holding them from the
// start, and every inserted image holds keypoints on triangulated tracks
bool SceneBlocksInsertRebuildTest()
{
	TD_TIMER_START();
	const ReconstructionConfig config;
	ResectionConfig resectionCfg = config.resectionCfg;
	resectionCfg.DeriveBAConfigs(config.baConfig);
	const IIndexArr chain{16, 17, 18};
	// the block's tracks as sets of observations in global IDs
	const auto TrackSets = [](const Scene& block, const IIndexArr& l2g) {
		std::set<std::vector<std::pair<IIndex, uint32_t>>> sets;
		for (const Track& track : block.tracks) {
			std::vector<std::pair<IIndex, uint32_t>> obs;
			for (const Observation& o : track.observations)
				obs.emplace_back(l2g[o.imageID], o.featureID);
			std::sort(obs.begin(), obs.end());
			sets.insert(std::move(obs));
		}
		return sets;
	};
	const auto EditPairs = [&chain](Scene& scene) {
		RFOREACH(p, scene.pairs) {
			ImagePair& pair = scene.pairs[p];
			const bool in1 = std::find(chain.begin(), chain.end(), pair.ID1) != chain.end();
			const bool in2 = std::find(chain.begin(), chain.end(), pair.ID2) != chain.end();
			if (in1 && in2) {
				pair.weightSpatial = 20.f; // the heaviest pairs of the scene
			} else if ((pair.ID1 == chain[1] || pair.ID2 == chain[1]) && (in1 != in2)) {
				scene.pairs.RemoveAtMove(p); // the middle image: no pair to the block
			}
		}
	};
	// the keypoints of each chain image on triangulated tracks of block 1
	const auto CountOnInlierTracks = [&chain](const Scene& block, const IIndexArr& l2g, std::vector<unsigned>& counts) {
		counts.assign(chain.size(), 0);
		for (const Track& track : block.tracks) {
			if (!track.IsInlier())
				continue;
			for (const Observation& obs : track.observations) {
				const auto it = std::find(chain.begin(), chain.end(), l2g[obs.imageID]);
				if (it != chain.end())
					++counts[it - chain.begin()];
			}
		}
	};
	const auto Partition = [](const Scene& scene, const IIndexArr& pool) {
		ScenePartition partition;
		partition.clusters.resize(2);
		FOREACH(i, scene.images) {
			if (std::find(pool.begin(), pool.end(), (IIndex)i) != pool.end())
				partition.pool.push_back(i);
			else
				partition.clusters[i < 12 ? 0 : 1].push_back(i);
		}
		return partition;
	};
	// pose every block image but the chain on the truth, then build and triangulate its tracks
	const auto Reconstruct = [&](Scene& block, const IIndexArr& l2g, const std::vector<Pose3D>& gtPoses) {
		FOREACH(l, block.images) {
			if (std::find(chain.begin(), chain.end(), l2g[l]) != chain.end())
				continue;
			block.images[l].R = gtPoses[l2g[l]].R;
			block.images[l].C = gtPoses[l2g[l]].C;
		}
		BuildTracks(block, config.minPairWeight, config.trackConflictCfg);
		TriangulateTracks(block, true, config.maxReprojError, config.minAngleThreshold);
		block.RecomputeCalibratedImages();
		block.status.nState.set(Scene::Status::STATE::CALIBRATED);
	};
	const auto Generate = [&](Scene& scene, std::vector<Pose3D>& gtPoses) {
		GenerateTwoClusterScene(scene, 12, 12, 10, 60, 300);
		EditPairs(scene);
		gtPoses.resize(scene.images.size());
		FOREACH(i, scene.images) {
			gtPoses[i].R = scene.images[i].R;
			gtPoses[i].C = scene.images[i].C;
			scene.images[i].InvalidatePose();
		}
		scene.status.nCalibratedImages = 0;
	};

	// the reference: the chain in block 1 from the start, unposed
	std::vector<unsigned> reference, inserted;
	std::set<std::vector<std::pair<IIndex, uint32_t>>> referenceTracks, insertedTracks;
	{
		Scene scene;
		std::vector<Pose3D> gtPoses;
		Generate(scene, gtPoses);
		SceneBlocks sceneBlocks(scene);
		sceneBlocks.Extract(Partition(scene, IIndexArr()));
		Reconstruct(sceneBlocks.blocks[1], sceneBlocks.localToGlobal[1], gtPoses);
		CountOnInlierTracks(sceneBlocks.blocks[1], sceneBlocks.localToGlobal[1], reference);
		referenceTracks = TrackSets(sceneBlocks.blocks[1], sceneBlocks.localToGlobal[1]);
	}
	// the pool: block 1 without the chain, then the chain inserted
	{
		Scene scene;
		std::vector<Pose3D> gtPoses;
		Generate(scene, gtPoses);
		SceneBlocks sceneBlocks(scene);
		sceneBlocks.Extract(Partition(scene, chain));
		Reconstruct(sceneBlocks.blocks[1], sceneBlocks.localToGlobal[1], gtPoses);
		sceneBlocks.Insert(1, chain, config, resectionCfg);
		CountOnInlierTracks(sceneBlocks.blocks[1], sceneBlocks.localToGlobal[1], inserted);
		insertedTracks = TrackSets(sceneBlocks.blocks[1], sceneBlocks.localToGlobal[1]);
	}
	if (insertedTracks != referenceTracks) {
		std::vector<std::vector<std::pair<IIndex, uint32_t>>> onlyInserted, onlyReference;
		std::set_difference(insertedTracks.begin(), insertedTracks.end(), referenceTracks.begin(), referenceTracks.end(), std::back_inserter(onlyInserted));
		std::set_difference(referenceTracks.begin(), referenceTracks.end(), insertedTracks.begin(), insertedTracks.end(), std::back_inserter(onlyReference));
		VERBOSE("SceneBlocksInsertRebuildTest FAILED: after Insert %u tracks, built with the block from the start %u: %u only after Insert, %u only in the reference",
			(unsigned)insertedTracks.size(), (unsigned)referenceTracks.size(), (unsigned)onlyInserted.size(), (unsigned)onlyReference.size());
		return false;
	}
	FOREACH(i, chain) {
		if (reference[i] == 0 || inserted[i] == 0) {
			VERBOSE("SceneBlocksInsertRebuildTest FAILED: image %u holds %u keypoints on triangulated tracks after Insert (%u in the reference)",
				chain[i], inserted[i], reference[i]);
			return false;
		}
	}

	VERBOSE("SceneBlocksInsertRebuildTest PASSED: %u tracks after Insert, the same observation sets as BuildTracks over the block holding the images from the start; keypoints on triangulated tracks %u/%u/%u (reference %u/%u/%u) (%s)",
		(unsigned)insertedTracks.size(), inserted[0], inserted[1], inserted[2], reference[0], reference[1], reference[2], TD_TIMER_GET_FMT().c_str());
	return true;
}

// An image the partition put in the wrong cluster (B's image 18 in A's block, where it has only a
// few weak pairs and stays unregistered) ends registered in B's block before the merge: the pool
// resection releases it from A and offers it to B, whose verified pairs to it weigh the most, and
// B's own resection registers it where the ground truth has it
bool PoolResectionWrongClusterTest()
{
	TD_TIMER_START();
	const ReconstructionConfig config;
	ResectionConfig resectionCfg = config.resectionCfg;
	resectionCfg.pairsWeighting = config.matchCfg.weightingCfg;
	resectionCfg.DeriveBAConfigs(config.baConfig);
	const IIndex wrong = 18;
	Scene scene;
	SceneBlocks sceneBlocks(scene);
	std::vector<Pose3D> gtPoses;
	BuildPoolBlocks(scene, sceneBlocks, gtPoses, {wrong}, {}, config);
	if (sceneBlocks.blockOf[wrong] != 0) {
		VERBOSE("PoolResectionWrongClusterTest FAILED: the fixture did not put image %u in block 0", wrong);
		return false;
	}

	sceneBlocks.ResectPool(resectionCfg, config);
	if (sceneBlocks.blocks.size() != 2 || sceneBlocks.blockOf[wrong] != 1) {
		VERBOSE("PoolResectionWrongClusterTest FAILED: image %u ended in block %d of %u, expected block 1",
			wrong, sceneBlocks.blockOf[wrong] == NO_ID ? -1 : (int)sceneBlocks.blockOf[wrong], (unsigned)sceneBlocks.blocks.size());
		return false;
	}
	const auto PoseOf = [&](IIndex g) -> const Image& {
		const IIndex b = sceneBlocks.blockOf[g];
		const IIndexArr& l2g = sceneBlocks.localToGlobal[b];
		return sceneBlocks.blocks[b].images[std::lower_bound(l2g.begin(), l2g.end(), g) - l2g.begin()];
	};
	for (IIndex g = 0; g < scene.images.size(); ++g) {
		if (sceneBlocks.blockOf[g] != (g < 12 ? 0u : 1u) || !PoseOf(g).HasPose()) {
			VERBOSE("PoolResectionWrongClusterTest FAILED: image %u not registered in its cluster's block", g);
			return false;
		}
	}
	// the pose against its neighbours in B, whatever gauge the block's adjustments left it in
	const Image &img = PoseOf(wrong), &prev = PoseOf(wrong - 1), &next = PoseOf(wrong + 1);
	const RMatrix relR(img.R * prev.R.t()), relRGT(gtPoses[wrong].R * gtPoses[wrong - 1].R.t());
	const REAL rotationError = R2D(ACOS(ComputeAngle(relR, relRGT)));
	const Point3 dir(img.C - prev.C), dirGT(gtPoses[wrong].C - gtPoses[wrong - 1].C);
	const REAL directionError = R2D(ACOS(CLAMP(dir.dot(dirGT) / (norm(dir) * norm(dirGT)), REAL(-1), REAL(1))));
	const REAL ratio = norm(dir) / norm(next.C - prev.C), ratioGT = norm(dirGT) / norm(gtPoses[wrong + 1].C - gtPoses[wrong - 1].C);
	if (rotationError > 0.5 || directionError > 2.0 || ABS(ratio / ratioGT - 1) > 0.02) {
		VERBOSE("PoolResectionWrongClusterTest FAILED: image %u registered off the truth: %.3f deg rotation, %.3f deg direction, baseline ratio %.4f vs %.4f",
			wrong, rotationError, directionError, ratio, ratioGT);
		return false;
	}

	VERBOSE("PoolResectionWrongClusterTest PASSED: image %u moved from block 0 to block 1, %.3f deg rotation, %.3f deg direction off the truth (%s)",
		wrong, rotationError, directionError, TD_TIMER_GET_FMT().c_str());
	return true;
}

// A pool image is offered only to the blocks it has links in (verified pairs with a relative pose to their
// registered images): pool image 18 (B's) has its links to B's registered images, and its pairs to the images
// of block A carry no relative pose (F-only pairs): the generated one loses its relative pose, and every other
// image of A gets a pair of 1000 weighted inliers, the heaviest pairs of the scene, together heavier than its
// links; it is sent to block B first, registers there, and is never tried in block A
bool PoolResectionLinkOnlyTest()
{
	TD_TIMER_START();
	const ReconstructionConfig config;
	ResectionConfig resectionCfg = config.resectionCfg;
	resectionCfg.pairsWeighting = config.matchCfg.weightingCfg;
	resectionCfg.DeriveBAConfigs(config.baConfig);
	const IIndex image = 18;
	Scene scene;
	SceneBlocks sceneBlocks(scene);
	std::vector<Pose3D> gtPoses;
	BuildPoolBlocks(scene, sceneBlocks, gtPoses, {}, {image}, config);
	std::set<IIndex> paired; // the images of block A the image already has a pair to
	for (ImagePair& pair : scene.pairs) {
		const IIndex other = pair.ID1 == image ? pair.ID2 : pair.ID1;
		if ((pair.ID1 == image || pair.ID2 == image) && sceneBlocks.blockOf[other] == 0) {
			pair.relativePose.reset();
			paired.insert(other);
		}
	}
	for (IIndex a = 0; a < 12; ++a) {
		if (paired.count(a))
			continue;
		ImagePair pair(a, image);
		pair.weightedInliers = 1000.f;
		pair.weightSpatial = 1000.f; // quality factors far past any real pair's: 10000 each, the heaviest pairs of the scene
		pair.overlapArea = 1.f;
		pair.weightConnectivity = 1.f;
		pair.weightTriplet = 0.75f;
		scene.pairs.emplace_back(std::move(pair));
	}
	float weightA = 0.f, weightB = 0.f; // the image's F-only pairs to block 0, its links to block 1
	unsigned numLinksA = 0;
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.HasValidWeight() || (pair.ID1 != image && pair.ID2 != image))
			continue;
		const IIndex b = sceneBlocks.blockOf[pair.ID1 == image ? pair.ID2 : pair.ID1];
		if (b == 0 && pair.relativePose.has_value())
			++numLinksA;
		else if (b == 0)
			weightA += pair.GetCompositeWeight();
		else if (b == 1 && pair.relativePose.has_value())
			weightB += pair.GetCompositeWeight();
	}
	if (sceneBlocks.blockOf[image] != NO_ID || numLinksA != 0 || weightB <= 0.f || weightA <= weightB) {
		VERBOSE("PoolResectionLinkOnlyTest FAILED: the fixture does not pose the problem: image %u in block %d, %u links to block 0, its F-only pairs to it weigh %g, its links to block 1 %g",
			image, sceneBlocks.blockOf[image] == NO_ID ? -1 : (int)sceneBlocks.blockOf[image], numLinksA, weightA, weightB);
		return false;
	}

	LogCapture log;
	sceneBlocks.ResectPool(resectionCfg, config);
	const String toA(String::FormatString("image %u (from block -1) -> block 0:", image));
	const String toB(String::FormatString("image %u (from block -1) -> block 1: registered", image));
	if (log.Contains(toA.c_str()) || !log.Contains(toB.c_str()) || sceneBlocks.blockOf[image] != 1) {
		VERBOSE("PoolResectionLinkOnlyTest FAILED: image %u, whose pairs to block 0 carry no relative pose, was %s; it ended in block %d",
			image, log.Contains(toA.c_str()) ? "tried in block 0" : "not registered in block 1 at its first attempt",
			sceneBlocks.blockOf[image] == NO_ID ? -1 : (int)sceneBlocks.blockOf[image]);
		return false;
	}

	VERBOSE("PoolResectionLinkOnlyTest PASSED: image %u, F-only pairs of %g to block 0 and links of %g to block 1, was sent to block 1 first and registered there (%s)",
		image, weightA, weightB, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

} // namespace SFM
