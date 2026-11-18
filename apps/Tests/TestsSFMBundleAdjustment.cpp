/*
 * TestsSFMBundleAdjustment.cpp
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

bool BAPinholeReprojectionJacobianTest()
{
	return PinholeReprojectionJacobianTest();
}

bool ObservationSigmasTest()
{
	TD_TIMER_START();

	// a posed synthetic scene whose observations are exact, so every reprojection error below is
	// one this test put there
	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 4;
	cfg.numPoints = 200;
	GenerateTestScene(scene, cfg);

	// displace the described observations by 0.5 px and give every track a second, DENSE observation
	// of the same point displaced by 2.0 px: k = 4 by construction.
	// Neither population is displaced UNIFORMLY, which is what makes this a test of the median: one
	// observation in ten is thrown 20 px out and one in ten left almost on the point, so of the four
	// statistics one could read off the population only the median still reports the bulk of it. On
	// the described side the mean answers ~2.4 px and the max 20 px against the 0.5 px the median
	// holds, and the min answers 0.05 px. That robustness is the whole reason these sigmas can be
	// read off RAW residuals, before any solve has pulled the outliers in.
	constexpr float describedError = 0.5f, denseError = 2.0f;
	constexpr float outlierError = 20.0f, innerError = 0.05f;
	const auto Displace = [](size_t idxObs, float nominal) {
		return idxObs%10 == 0 ? outlierError : (idxObs%10 == 1 ? innerError : nominal);
	};
	for (Image& img : scene.images)
		img.CloseDescribedKeypoints();
	size_t idxObs = 0;
	for (Track& track : scene.tracks) {
		const size_t numDescribedObs = track.observations.size();
		for (size_t i = 0; i < numDescribedObs; ++i, ++idxObs) {
			const Observation obs = track.observations[i];
			Image& img = scene.images[obs.imageID];
			const float x = img.keypoints[obs.featureID].pt.x;
			img.keypoints[obs.featureID].pt.x = x + Displace(idxObs, describedError);
			const cv::KeyPoint dense(x + Displace(idxObs, denseError),
				img.keypoints[obs.featureID].pt.y, 10.f, -1.f, 0.9f);
			const uint32_t featID = (uint32_t)img.keypoints.size();
			img.keypoints.push_back(dense);
			track.observations.emplace_back(obs.imageID, featID);
		}
		track.numInliers = (uint8_t)MINF((size_t)track.observations.size(), (size_t)255);
	}

	double sigmaDescribed = 0, sigmaDense = 0;
	size_t numDescribed = 0, numDense = 0;
	ComputeObservationSigmas(scene, sigmaDescribed, numDescribed, sigmaDense, numDense);
	if (numDescribed == 0 || numDense != numDescribed) {
		VERBOSE("ObservationSigmasTest FAILED: %u described and %u dense observations",
			(unsigned)numDescribed, (unsigned)numDense);
		return false;
	}
	// the medians are the displacement four fifths of each population carries, and their ratio is the
	// k the weight is computed from. To a hundredth of a pixel rather than exactly: the implementation
	// takes the UPPER median (errors[size/2]), which on an even-sized population is one particular
	// element and not the average of the middle two, so the value asserted here is the displacement
	// and the tolerance covers the reprojection of it
	if (ABS(sigmaDescribed - describedError) > 0.01 || ABS(sigmaDense - denseError) > 0.01) {
		VERBOSE("ObservationSigmasTest FAILED: sigmas %.4f described / %.4f dense against %.2f / %.2f -- "
			"a mean would answer near %.2f / %.2f and a max %.2f",
			sigmaDescribed, sigmaDense, describedError, denseError,
			0.8f*describedError + 0.1f*(outlierError + innerError),
			0.8f*denseError + 0.1f*(outlierError + innerError), outlierError);
		return false;
	}

	// a scene with no dense keypoints reports none, and reports the described population anyway
	Scene sparseOnly;
	GenerateTestScene(sparseOnly, cfg);
	ComputeObservationSigmas(sparseOnly, sigmaDescribed, numDescribed, sigmaDense, numDense);
	if (numDense != 0 || numDescribed == 0) {
		VERBOSE("ObservationSigmasTest FAILED: a scene with no dense keypoints reported %u of them",
			(unsigned)numDense);
		return false;
	}

	VERBOSE("Observation sigmas test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

bool DenseObservationWeightEstimateTest()
{
	TD_TIMER_START();

	// the scene of ObservationSigmasTest: described observations off by 0.5 px, dense ones by
	// 2.0 px, so k = 4 and the weight the estimator must return is 1/16
	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 4;
	cfg.numPoints = 200;
	GenerateTestScene(scene, cfg);
	constexpr float describedError = 0.5f, denseError = 2.0f;
	for (Image& img : scene.images)
		img.CloseDescribedKeypoints();
	for (Track& track : scene.tracks) {
		const size_t numDescribedObs = track.observations.size();
		for (size_t i = 0; i < numDescribedObs; ++i) {
			const Observation obs = track.observations[i];
			Image& img = scene.images[obs.imageID];
			img.keypoints[obs.featureID].pt.x += describedError;
			const cv::KeyPoint dense(img.keypoints[obs.featureID].pt.x - describedError + denseError,
				img.keypoints[obs.featureID].pt.y, 10.f, -1.f, 0.9f);
			const uint32_t featID = (uint32_t)img.keypoints.size();
			img.keypoints.push_back(dense);
			track.observations.emplace_back(obs.imageID, featID);
		}
		track.numInliers = (uint8_t)MINF((size_t)track.observations.size(), (size_t)255);
	}

	BAConfig config;
	config.denseObservationWeight = -1.0; // estimate
	const double weight = EstimateDenseObservationWeight(scene, config);
	if (ABS(weight - 1.0/16.0) > 0.005) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: weight %.4f against 1/k^2 = %.4f for k = 4",
			weight, 1.0/16.0);
		return false;
	}

	// a dense population strictly MORE precise than the described one is not worth MORE than it:
	// shift the dense keypoints from the 2.0 px displacement above down to 0.25 px (by subtracting
	// denseError - upperBoundDenseError = 1.75, not the 1.5 that would leave the two populations
	// merely equal), well inside the described population's 0.5 px, so k = 0.5 and the unclamped
	// value would be 4.0 -- it is that 4.0 the clamp below must reject, not an accidentally-exact
	// k = 1 that would pass even with no clamp at all
	constexpr float upperBoundDenseError = 0.25f;
	for (Image& img : scene.images)
		for (uint32_t k = img.NumDescribedKeypoints(); k < img.keypoints.size(); ++k)
			img.keypoints[k].pt.x -= denseError - upperBoundDenseError;
	const double clamped = EstimateDenseObservationWeight(scene, config);
	if (ABS(clamped - 1.0) > 0.005) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: a dense population twice as precise as the "
			"described one weighs %.4f instead of being clamped to 1", clamped);
		return false;
	}

	// and a dense population far coarser than the described one -- the case the whole weight exists
	// for, a textureless region where the warp is the only correspondence there is -- is clamped to
	// MIN_DENSE_OBSERVATION_WEIGHT (0.01, file-local to BundleAdjustment.cpp) rather than losing
	// almost all its influence: 10 px against the described population's 0.5 px gives k = 20 and an
	// unclamped 1/k^2 = 0.0025, four times below the floor
	Scene coarseDense;
	GenerateTestScene(coarseDense, cfg);
	constexpr float coarseDenseError = 10.0f;
	for (Image& img : coarseDense.images)
		img.CloseDescribedKeypoints();
	for (Track& track : coarseDense.tracks) {
		const size_t numDescribedObs = track.observations.size();
		for (size_t i = 0; i < numDescribedObs; ++i) {
			const Observation obs = track.observations[i];
			Image& img = coarseDense.images[obs.imageID];
			img.keypoints[obs.featureID].pt.x += describedError;
			const cv::KeyPoint dense(img.keypoints[obs.featureID].pt.x - describedError + coarseDenseError,
				img.keypoints[obs.featureID].pt.y, 10.f, -1.f, 0.9f);
			const uint32_t featID = (uint32_t)img.keypoints.size();
			img.keypoints.push_back(dense);
			track.observations.emplace_back(obs.imageID, featID);
		}
		track.numInliers = (uint8_t)MINF((size_t)track.observations.size(), (size_t)255);
	}
	const double floored = EstimateDenseObservationWeight(coarseDense, config);
	if (ABS(floored - 0.01) > 0.0005) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: a dense population 20x coarser than the "
			"described one weighs %.4f instead of being floored to 0.01", floored);
		return false;
	}

	// and a scene with no dense keypoints at all falls back to the configured constant rather than
	// dividing by a sigma it does not have
	Scene sparseOnly;
	GenerateTestScene(sparseOnly, cfg);
	const double fallback = EstimateDenseObservationWeight(sparseOnly, config);
	if (fallback != DENSE_OBSERVATION_WEIGHT) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: fallback %.4f against the constant %.4f",
			fallback, DENSE_OBSERVATION_WEIGHT);
		return false;
	}

	// the fallback's other form, and the one a real run actually reaches: a dense population that
	// EXISTS but is too small to give a sigma. A few images into an incremental reconstruction on a
	// textureless capture the dense observations are still a handful while the described ones are
	// already thousands, and a median over 40 points is not a sigma to divide by. Both populations
	// are displaced here, so it is the count that refuses the sample and not a zero sigma.
	Scene fewDense;
	SceneConfig fewDenseCfg;
	fewDenseCfg.numImages = 4;
	fewDenseCfg.numPoints = 300;
	GenerateTestScene(fewDense, fewDenseCfg);
	constexpr size_t numFewDense = 40;
	for (Image& img : fewDense.images)
		img.CloseDescribedKeypoints();
	size_t numAdded = 0;
	for (Track& track : fewDense.tracks) {
		const size_t numDescribedObs = track.observations.size();
		for (size_t i = 0; i < numDescribedObs; ++i) {
			const Observation obs = track.observations[i];
			Image& img = fewDense.images[obs.imageID];
			img.keypoints[obs.featureID].pt.x += describedError;
			if (numAdded >= numFewDense)
				continue;
			const cv::KeyPoint dense(img.keypoints[obs.featureID].pt.x - describedError + denseError,
				img.keypoints[obs.featureID].pt.y, 10.f, -1.f, 0.9f);
			const uint32_t featID = (uint32_t)img.keypoints.size();
			img.keypoints.push_back(dense);
			track.observations.emplace_back(obs.imageID, featID);
			++numAdded;
		}
		track.numInliers = (uint8_t)MINF((size_t)track.observations.size(), (size_t)255);
	}
	double sigmaFewDescribed = 0, sigmaFewDense = 0;
	size_t numFewDescribedObs = 0, numFewDenseObs = 0;
	ComputeObservationSigmas(fewDense, sigmaFewDescribed, numFewDescribedObs, sigmaFewDense, numFewDenseObs);
	if (numFewDenseObs != numFewDense || numFewDescribedObs < 100 ||
		sigmaFewDescribed <= 0.0 || sigmaFewDense <= 0.0) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: the small-sample fixture holds %u dense and %u described "
			"observations at sigmas %.4f/%.4f -- it must put the dense population alone under the threshold",
			(unsigned)numFewDenseObs, (unsigned)numFewDescribedObs, sigmaFewDescribed, sigmaFewDense);
		return false;
	}
	const double smallSample = EstimateDenseObservationWeight(fewDense, config);
	if (smallSample != DENSE_OBSERVATION_WEIGHT) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: %u dense observations against %u described ones weigh "
			"%.4f instead of falling back to the constant %.4f",
			(unsigned)numFewDenseObs, (unsigned)numFewDescribedObs, smallSample, DENSE_OBSERVATION_WEIGHT);
		return false;
	}

	VERBOSE("Dense observation weight estimate test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// Pins SetBAIntrinsicFlags (ReconstructionConfig::IntrinsicFlags -> BAConfig) and
// ResectionConfig::DeriveBAConfigs, the two places the reconstruction's intrinsic flags turn
// into the bundle-adjustment switches that are actually solved.
bool BAIntrinsicFlagsTest()
{
	TD_TIMER_START();

	// INTRINSIC_MAIN: focal length and k1, k2 only
	{
		BAConfig baCfg;
		SetBAIntrinsicFlags(baCfg, ReconstructionConfig::INTRINSIC_MAIN);
		if (!baCfg.refineFocalLength || !baCfg.refineRadialDistortion12 ||
			baCfg.refineFocalLengthAspectRatio || baCfg.refinePrincipalPoint ||
			baCfg.refineRadialDistortion3 || baCfg.refineTangentialDistortion ||
			baCfg.refineRadialDistortion456) {
			VERBOSE("BAIntrinsicFlagsTest FAILED: INTRINSIC_MAIN did not set exactly focal length and k1, k2");
			return false;
		}
	}

	// INTRINSIC_MAIN_EXTRA: adds k3, the principal point and the tangential terms
	{
		BAConfig baCfg;
		SetBAIntrinsicFlags(baCfg, ReconstructionConfig::INTRINSIC_MAIN_EXTRA);
		if (!baCfg.refineFocalLength || !baCfg.refineRadialDistortion12 ||
			!baCfg.refineRadialDistortion3 || !baCfg.refinePrincipalPoint ||
			!baCfg.refineTangentialDistortion ||
			baCfg.refineFocalLengthAspectRatio || baCfg.refineRadialDistortion456) {
			VERBOSE("BAIntrinsicFlagsTest FAILED: INTRINSIC_MAIN_EXTRA did not set focal length, k1, k2, "
				"k3, the principal point and the tangential terms");
			return false;
		}
	}

	// INTRINSIC_NONE: nothing refined
	{
		BAConfig baCfg;
		SetBAIntrinsicFlags(baCfg, ReconstructionConfig::INTRINSIC_NONE);
		if (baCfg.IsRefiningIntrinsics()) {
			VERBOSE("BAIntrinsicFlagsTest FAILED: INTRINSIC_NONE left an intrinsic refined");
			return false;
		}
	}

	// ResectionConfig::DeriveBAConfigs, given a reconstruction config that allows every intrinsic:
	// fullBAConfig is restricted to the main set, extendedBAConfig keeps all of it
	{
		BAConfig baseCfg;
		SetBAIntrinsicFlags(baseCfg, ReconstructionConfig::INTRINSIC_MAIN_EXTRA);
		ResectionConfig resectionCfg;
		resectionCfg.DeriveBAConfigs(baseCfg);
		const BAConfig& fullCfg = resectionCfg.fullBAConfig;
		const BAConfig& extCfg = resectionCfg.extendedBAConfig;
		if (!fullCfg.refineFocalLength || !fullCfg.refineRadialDistortion12 ||
			fullCfg.refineRadialDistortion3 || fullCfg.refinePrincipalPoint ||
			fullCfg.refineTangentialDistortion) {
			VERBOSE("BAIntrinsicFlagsTest FAILED: fullBAConfig kept an extended-set intrinsic");
			return false;
		}
		if (!extCfg.refineFocalLength || !extCfg.refineRadialDistortion12 ||
			!extCfg.refineRadialDistortion3 || !extCfg.refinePrincipalPoint ||
			!extCfg.refineTangentialDistortion) {
			VERBOSE("BAIntrinsicFlagsTest FAILED: extendedBAConfig dropped an intrinsic the reconstruction allowed");
			return false;
		}
	}

	VERBOSE("BAIntrinsicFlagsTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// ===============================================================================
// Spherical camera full-hemisphere reconstruction test
// Exercises the triangulation + BA pipeline on a spherical scene where 3D points
// are distributed in ALL directions around the cameras (front, back, sides), so
// that observations span the full equirectangular image including longitudes
// |theta| > pi/2. This is the regression harness for the S^2 -> R^2 singularity
// in SphericalCamera::Unproject and the pinhole DLT in TriangulateDLT.
//
// It also guards against the left-right (X) mirror reported on real 360 scenes:
// the closing block pins the absolute equirectangular convention of
// SphericalCamera::Project against hand-reasoned ground truth, and runs an O(3)
// (reflection-allowed) Kabsch handedness check on the reconstructed rig+points.
// ===============================================================================
bool ReconstructSphericalSyntheticTest()
{
	VERBOSE("\n=== ReconstructSphericalSyntheticTest: full-hemisphere spherical scene ===");

	// Build a spherical scene manually so we control 3D point placement directly.
	Scene sceneGT;
	const int width = 2048, height = 1024;
	sceneGT.cameras.emplace_back(new SphericalCamera(cv::Size(width, height)));

	// 6 cameras arranged in a small 3D cluster near origin, all sharing identity
	// rotation. With identity rotation + small translation, points on the far
	// side of origin land at camera-space Z < 0 (the equirectangular "back half").
	const unsigned numImages = 6;
	const Point3 camCenters[numImages] = {
		Point3(-0.6,  0.0, -0.3),
		Point3( 0.6,  0.0, -0.3),
		Point3(-0.6,  0.0,  0.3),
		Point3( 0.6,  0.0,  0.3),
		Point3( 0.0, -0.4,  0.0),
		Point3( 0.0,  0.4,  0.0),
	};
	for (unsigned i = 0; i < numImages; ++i) {
		Pose3D pose;
		pose.C = camCenters[i];
		pose.R = Matrix3x3::IDENTITY;
		sceneGT.images.emplace_back(static_cast<IIndex>(i), String(), pose, 0, sceneGT.cameras[0]);
	}
	sceneGT.status.nCalibratedImages = sceneGT.images.size();

	// Generate 3D points uniformly on a sphere of radius ~5 around origin. With
	// the camera cluster at origin and points at distance 5 in all directions,
	// every point is visible from every camera, and roughly half the observations
	// fall in the camera-space Z < 0 "back" hemisphere of the equirectangular image.
	const unsigned numPoints = 80;
	std::mt19937 rng(1337);
	std::uniform_real_distribution<REAL> cosThetaDist(REAL(-1), REAL(1));
	std::uniform_real_distribution<REAL> phiDist(REAL(-M_PI), REAL(M_PI));
	std::uniform_real_distribution<REAL> radiusDist(REAL(4.5), REAL(5.5));
	for (unsigned p = 0; p < numPoints; ++p) {
		const REAL r = radiusDist(rng);
		const REAL ct = cosThetaDist(rng);
		const REAL st = SQRT(REAL(1) - ct*ct);
		const REAL ph = phiDist(rng);
		const Point3 X(r * st * COS(ph), r * ct, r * st * SIN(ph));

		Track track(X);
		for (unsigned v = 0; v < numImages; ++v) {
			Image& img = sceneGT.images[v];
			const auto [proj, valid] = img.ProjectPoint(X);
			if (!valid || !Image8U::isInside(proj, img.GetSize()))
				continue;
			const uint32_t featID = static_cast<uint32_t>(img.keypoints.size());
			img.keypoints.emplace_back(Cast<float>(proj), 0.f, 0.f, 10.f);
			track.observations.emplace_back(img.ID, featID);
		}
		if (track.observations.size() < 2) {
			// Drop the keypoints we just added — track is unusable
			for (const auto& obs : track.observations)
				sceneGT.images[obs.imageID].keypoints.pop_back();
			continue;
		}
		track.numInliers = static_cast<uint8_t>(track.observations.size());
		sceneGT.tracks.emplace_back(std::move(track));
	}
	sceneGT.status.nTracks = sceneGT.tracks.size();
	sceneGT.status.nState.set(Scene::Status::STATE::FEATURES_EXTRACTED);

	// Count back-hemisphere observations: camera-space Z < 0.
	// This is the coverage check — the test only catches G1 if at least some
	// observations fall in the back hemisphere of the equirectangular image.
	unsigned totalObs = 0, backObs = 0;
	for (const Track& track : sceneGT.tracks) {
		for (const auto& obs : track.observations) {
			const Image& img = sceneGT.images[obs.imageID];
			const Point3 Xcam = img.TransformPointW2C(track.position);
			++totalObs;
			if (Xcam.z < 0)
				++backObs;
		}
	}
	VERBOSE("Scene: %u images, %u tracks, %u observations (%u back-hemisphere, %.1f%%)",
	        (unsigned)sceneGT.images.size(), (unsigned)sceneGT.tracks.size(),
	        totalObs, backObs, totalObs > 0 ? 100.0 * backObs / totalObs : 0.0);
	if (backObs < totalObs / 4) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: scene setup produced too few back-hemisphere observations (%u/%u); "
		        "test must exercise the full sphere to expose G1", backObs, totalObs);
		return false;
	}

	// Clone scene and clear track positions — force fresh triangulation
	// from the 2D observations + GT poses. This is the entry point that
	// exercises the pinhole-plane DLT formulation in TriangulateDLT.
	Scene scene = sceneGT;
	for (Track& track : scene.tracks)
		track.position = Point3(REAL(0), REAL(0), REAL(0));

	// Triangulate all tracks. Use a generous reprojection threshold (20 pixels
	// on a 2048-wide equirectangular image ≈ 3.5°) and a low minimum triangulation
	// angle (0.5°) so the test isolates G1/G2 failure modes rather than geometric
	// insufficiency.
	const unsigned inlierTracks = TriangulateTracks(scene, /*outliersOnly=*/false, /*reprojThreshold=*/20.f, /*minAngleThreshold=*/0.5f);
	VERBOSE("TriangulateTracks: %u inlier tracks of %u total",
	        inlierTracks, (unsigned)scene.tracks.size());

	// Measure 3D recovery error against ground truth
	REAL sum3D = 0, max3D = 0;
	unsigned recovered = 0;
	for (IIndex t = 0; t < scene.tracks.size(); ++t) {
		const Point3& rec = scene.tracks[t].position;
		const Point3& gt = sceneGT.tracks[t].position;
		const REAL err = norm(rec - gt);
		sum3D += err;
		max3D = MAX(max3D, err);
		if (err < REAL(0.1))
			++recovered;
	}
	const REAL mean3D = scene.tracks.size() > 0 ? sum3D / scene.tracks.size() : REAL(0);
	VERBOSE("Triangulation 3D recovery: %u/%u within 0.1m, mean %.4f m, max %.4f m",
	        recovered, (unsigned)scene.tracks.size(), mean3D, max3D);

	// Also measure reprojection error of the triangulated points.
	// meanAng is reported in degrees by ComputeTracksMeanReprojectionError.
	const auto [meanReprojErr, meanAng] = ComputeTracksMeanReprojectionError(scene);
	VERBOSE("Triangulation reprojection error: mean %.4f px (angular %.4f deg)",
	        meanReprojErr, meanAng);

	// Strict success criterion: at least 95% of tracks must recover to within
	// 10cm of ground truth with sub-pixel reprojection error AND near-zero
	// angular error. The angular metric is the critical one for spherical
	// cameras: ComputeTracksMeanReprojectionError currently uses the 2D
	// Camera::Unproject + .homogeneous() form to build the observed ray, which
	// aliases back-hemisphere observations onto the front hemisphere. For a
	// perfectly recovered scene with full-sphere point coverage, this produces
	// ~90 degrees of "angular error" instead of ~0 — the fingerprint of G1.
	const unsigned expectedRecovered = static_cast<unsigned>(scene.tracks.size() * 0.95);
	if (recovered < expectedRecovered) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: only %u/%u tracks recovered within 0.1m (expected >= %u)",
		        recovered, (unsigned)scene.tracks.size(), expectedRecovered);
		return false;
	}
	if (meanReprojErr > REAL(1.0)) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: mean reprojection error %.4f px exceeds 1.0 px threshold",
		        meanReprojErr);
		return false;
	}
	if (meanAng > REAL(1.0)) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: mean angular error %.4f deg exceeds 1.0 deg threshold. "
		        "This exposes G1: ComputeTracksMeanReprojectionError uses the 2D Camera::Unproject() form to "
		        "build the observed bearing ray; for back-hemisphere features on a spherical camera, the "
		        "aliasing produces ~180 deg error that averages to ~90 deg across a full-sphere scene. "
		        "The fix is to use Camera::UnprojectNormalized() which returns a 3D unit bearing vector "
		        "that is singularity-free and not front-hemisphere-biased.",
		        meanAng);
		return false;
	}

	// Run global BA on the triangulated scene and verify it converges and
	// improves (or at least preserves) the reconstruction.
	BAConfig baCfg;
	baCfg.maxIterations = 30;
	baCfg.robustThreshold = 2.f;
	if (!BundleAdjustment::Adjust(scene, baCfg)) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: BundleAdjustment::Adjust returned false");
		return false;
	}
	const auto [baMeanErr, baMeanAng] = ComputeTracksMeanReprojectionError(scene);
	VERBOSE("Post-BA reprojection error: mean %.4f px (angular %.4f deg)", baMeanErr, baMeanAng);
	if (baMeanErr > REAL(1.0)) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: post-BA reprojection error %.4f px exceeds 1.0 px threshold",
		        baMeanErr);
		return false;
	}
	if (baMeanAng > REAL(1.0)) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: post-BA mean angular error %.4f deg exceeds 1.0 deg threshold",
		        baMeanAng);
		return false;
	}

	// ---------------------------------------------------------------------------
	// Left-right mirror / handedness regression checks.
	//
	// The reconstruction above creates its 2D observations with
	// SphericalCamera::Project and then inverts the SAME Project during
	// triangulation, so any globally-consistent sign flip in the azimuth
	// convention (an X-mirror) cancels in the round trip and stays invisible to
	// the recovery-error metrics. The two checks below expose such a flip
	// WITHOUT round-tripping through Project.
	//
	// (A) Pin the absolute equirectangular convention: assert SphericalCamera::Project
	//     maps canonical camera-space directions to the geometrically-correct side
	//     of the image. The expected side is reasoned from first principles for the
	//     standard equirect layout (forward +Z at the centre column, camera-right
	//     +X on the right half, and the Y-DOWN convention shared with the pinhole
	//     camera: camera +Y toward the bottom row) — NOT from Project.
	const Camera& sphCam = *sceneGT.cameras[0];
	const REAL uCenter = REAL(width) / 2;
	const REAL vCenter = REAL(height) / 2;
	struct ConventionCase { Point3 dirCam; const char* name; int expectU; int expectV; };
	// expectU/expectV: -1 => strictly left/above centre, +1 => strictly right/below, 0 => ~centre
	const ConventionCase cases[] = {
		{ Point3( 0,  0, 1), "forward +Z",  0,  0 },  // centre column, equator
		{ Point3( 1,  0, 1), "right   +X", +1,  0 },  // right of centre
		{ Point3(-1,  0, 1), "left    -X", -1,  0 },  // left of centre
		{ Point3( 0,  1, 1), "+Y (down)",   0, +1 },  // below centre (v large), Y-down
		{ Point3( 0, -1, 1), "-Y (up)  ",   0, -1 },  // above centre (v small), Y-down
	};
	for (const ConventionCase& c : cases) {
		const auto [px, valid] = sphCam.Project(c.dirCam);
		if (!valid) {
			VERBOSE("ReconstructSphericalSyntheticTest FAILED: Project(%s) returned invalid", c.name);
			return false;
		}
		bool ok = true;
		if (c.expectU < 0)      ok = ok && (px.x < uCenter - REAL(1));
		else if (c.expectU > 0) ok = ok && (px.x > uCenter + REAL(1));
		else                    ok = ok && (ABS(px.x - uCenter) < REAL(1));
		if (c.expectV < 0)      ok = ok && (px.y < vCenter - REAL(1));
		else if (c.expectV > 0) ok = ok && (px.y > vCenter + REAL(1));
		else                    ok = ok && (ABS(px.y - vCenter) < REAL(1));
		if (!ok) {
			VERBOSE("ReconstructSphericalSyntheticTest FAILED: equirect convention mismatch for %s -> pixel (%.1f, %.1f); "
			        "this is the left-right (X) mirror fingerprint", c.name, px.x, px.y);
			return false;
		}
		DEBUG("Convention %s -> pixel (%.1f, %.1f) [centre (%.1f, %.1f)]", c.name, px.x, px.y, uCenter, vCenter);
	}
	// Exact numeric lock: pure camera-right (+X, z=0) must land at u = 3/4 width,
	// pure camera-left (-X) at u = 1/4 width (standard equirect azimuth mapping).
	{
		const auto [pxR, vR] = sphCam.Project(Point3( 1, 0, 0));
		const auto [pxL, vL] = sphCam.Project(Point3(-1, 0, 0));
		if (!vR || !vL ||
		    ABS(pxR.x - REAL(0.75) * width) > REAL(1) ||
		    ABS(pxL.x - REAL(0.25) * width) > REAL(1)) {
			VERBOSE("ReconstructSphericalSyntheticTest FAILED: azimuth mapping is mirrored "
			        "(+X u=%.1f expected %.1f, -X u=%.1f expected %.1f)",
			        pxR.x, REAL(0.75) * width, pxL.x, REAL(0.25) * width);
			return false;
		}
	}

	// (B) O(3) Kabsch reflection test on the reconstructed rig + points vs GT.
	//     The optimal reflection-allowed alignment R = V*U^T from SVD(H) = U*S*V^T
	//     satisfies sign(det R) == sign(det H) because the singular values are
	//     non-negative; so det(H) > 0 proves the reconstruction matches GT under a
	//     PROPER rotation (no reflection), i.e. it is not left-right mirrored.
	//     (No SVD needed — only the sign of the 3x3 cross-covariance determinant.)
	//     The rig (camera centres) and the points are stacked together so a
	//     "rig + points mirror together" failure is caught as a single det flip.
	Point3Arr src, dst;
	for (IIndex t = 0; t < scene.tracks.size(); ++t) {
		if (norm(scene.tracks[t].position - sceneGT.tracks[t].position) < REAL(0.5)) {
			src.emplace_back(scene.tracks[t].position);
			dst.emplace_back(sceneGT.tracks[t].position);
		}
	}
	FOREACH(i, scene.images) {
		src.emplace_back(scene.images[i].C);
		dst.emplace_back(sceneGT.images[i].C);
	}
	if (src.size() < 4) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: too few rig+point correspondences (%u) for handedness check",
		        (unsigned)src.size());
		return false;
	}
	Point3 sMean(REAL(0), REAL(0), REAL(0)), dMean(REAL(0), REAL(0), REAL(0));
	for (const Point3& p : src) sMean += p;
	for (const Point3& p : dst) dMean += p;
	sMean /= (REAL)src.size();
	dMean /= (REAL)dst.size();
	REAL Hxx = 0, Hxy = 0, Hxz = 0, Hyx = 0, Hyy = 0, Hyz = 0, Hzx = 0, Hzy = 0, Hzz = 0;
	FOREACH(i, src) {
		const Point3 s = src[i] - sMean, d = dst[i] - dMean;
		Hxx += s.x*d.x; Hxy += s.x*d.y; Hxz += s.x*d.z;
		Hyx += s.y*d.x; Hyy += s.y*d.y; Hyz += s.y*d.z;
		Hzx += s.z*d.x; Hzy += s.z*d.y; Hzz += s.z*d.z;
	}
	const REAL detH = Hxx*(Hyy*Hzz - Hyz*Hzy) - Hxy*(Hyx*Hzz - Hyz*Hzx) + Hxz*(Hyx*Hzy - Hyy*Hzx);
	VERBOSE("Handedness check: det(cross-covariance) = %.4g over %u rig+point correspondences",
	        detH, (unsigned)src.size());
	if (detH <= REAL(0)) {
		VERBOSE("ReconstructSphericalSyntheticTest FAILED: reconstruction aligns to GT only under a reflection "
		        "(det = %.4g <= 0) -> left-right mirror detected", detH);
		return false;
	}

	VERBOSE("ReconstructSphericalSyntheticTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

// Small SFM smoke test: build tiny scene and run BundleAdjustment::Adjust
bool PipelineTest()
{
	// Test 1: Basic BA with quaternion poses (baseline test)
	{
		VERBOSE("\n--- Test 1: Basic BA with quaternion poses ---");
		Scene sceneGT, scene;

		// Generate test scene with perturbations
		SceneConfig sceneCfg;
		sceneCfg.perturbOptions = SceneConfig::PERTURB_ALL;
		GenerateTestScene(sceneGT, sceneCfg, &scene);

		BAConfig cfg;
		cfg.maxIterations = 20;
		BundleAdjustment ba(scene, cfg);
		if (!ba.Adjust()) {
			VERBOSE("Test 1 FAILED: BundleAdjustment returned false");
			return false;
		}

		// Compute mean reprojection error after BA
		const auto [meanErr, meanAng] = ComputeTracksMeanReprojectionError(scene);
		if (meanErr > 1.0) {
			VERBOSE("Test 1 FAILED: reprojection error too large");
			return false;
		}

		// Pose uncertainty from the BA covariance: one entry per image, the gauge reference
		// exactly 0, every other registered image finite and strictly positive
		const PoseUncertaintyArr uncertainty = ba.ComputePoseUncertainty();
		if (uncertainty.size() != scene.images.size()) {
			VERBOSE("Test 1 FAILED: pose uncertainty not computed (%u/%u images)",
				(unsigned)uncertainty.size(), (unsigned)scene.images.size());
			return false;
		}
		unsigned numDatum = 0;
		FOREACH(i, uncertainty) {
			const PoseUncertainty& u = uncertainty[i];
			if (!u.IsValid()) {
				VERBOSE("Test 1 FAILED: pose uncertainty missing for image %u", i);
				return false;
			}
			const float rotVar = u.MaxRotationVariance();
			const float posVar = u.MaxPositionVariance();
			if (!ISFINITE(rotVar) || !ISFINITE(posVar) || rotVar < 0.f || posVar < 0.f) {
				VERBOSE("Test 1 FAILED: invalid pose uncertainty for image %u (rotVar %g, posVar %g)", i, rotVar, posVar);
				return false;
			}
			// full 3x3 position covariance: off-diagonals finite and Cauchy-Schwarz-consistent
			if (!ISFINITE(u.posCov.x) || !ISFINITE(u.posCov.y) || !ISFINITE(u.posCov.z)) {
				VERBOSE("Test 1 FAILED: invalid position covariance for image %u", i);
				return false;
			}
			constexpr float tol = 1.01f;
			if (ABS(u.posCov.x) > SQRT(u.posVar.x * u.posVar.y) * tol + FLT_EPSILON ||
			    ABS(u.posCov.y) > SQRT(u.posVar.x * u.posVar.z) * tol + FLT_EPSILON ||
			    ABS(u.posCov.z) > SQRT(u.posVar.y * u.posVar.z) * tol + FLT_EPSILON) {
				VERBOSE("Test 1 FAILED: position covariance not positive semi-definite for image %u", i);
				return false;
			}
			if (rotVar == 0.f && posVar == 0.f)
				++numDatum; // gauge reference
		}
		if (numDatum != 1) {
			VERBOSE("Test 1 FAILED: expected exactly 1 gauge-reference image, got %u", numDatum);
			return false;
		}
		VERBOSE("Test 1 PASSED");
	}

	// Test 2: Spherical camera with angular reprojection error
	{
		VERBOSE("\n--- Test 2: Spherical camera BA ---");
		Scene scene;

		// Generate test scene with spherical camera
		SceneConfig sceneCfg;
		sceneCfg.cameras[0].type = SceneConfig::SPHERICAL;
		sceneCfg.cameras[0].width = 1024;
		sceneCfg.cameras[0].height = 512;
		sceneCfg.perturbOptions = SceneConfig::PERTURB_ALL;
		Scene sceneGT;
		GenerateTestScene(sceneGT, sceneCfg, &scene);
		VERBOSE("Test 5: Spherical camera scene created with %u images, %u tracks",
		        (unsigned)scene.images.size(), (unsigned)scene.tracks.size());

		BAConfig cfg;
		cfg.maxIterations = 30;
		cfg.robustThreshold = 1.f; // 1 pixel threshold (auto-converted to angular)
		if (!BundleAdjustment::Adjust(scene, cfg)) {
			VERBOSE("Test 2 FAILED: BundleAdjustment returned false");
			return false;
		}

		// Compute mean reprojection error after BA
		const auto [meanErr, meanAng] = ComputeTracksMeanReprojectionError(scene);
		if (meanErr > 1.0) {
			VERBOSE("Test 2 FAILED: reprojection error too large");
			return false;
		}
		VERBOSE("Test 2 PASSED (spherical camera works correctly)");
	}

	// Test 3: Refine focal length
	{
		VERBOSE("\n--- Test 3: Refine focal length ---");
		Scene scene;

		// Generate test scene (GT) - no perturbations
		SceneConfig sceneCfg;
		sceneCfg.cameras[0].focal = 420.0; // GT focal length
		sceneCfg.numImages = 4;
		sceneCfg.numPoints = 50;
		GenerateTestScene(scene, sceneCfg);

		// Manually perturb focal length only
		// Keypoints remain at GT positions for fx=420
		PinholeCamera* cam = (PinholeCamera*)scene.cameras[0];
		cam->fx = 380.0;
		cam->fy = 380.0;
		cam->trustIntrinsics = false; // Allow BA to refine
		VERBOSE("Test 3: Initial fx = %.2f (gt = %.2f)", cam->fx, sceneCfg.cameras[0].focal);

		BAConfig cfg;
		cfg.maxIterations = 30;
		cfg.refineFocalLength = true;
		cfg.refinePosesRotation = cfg.refinePosesPosition = false;  // Fix poses to GT
		cfg.refinePoints = false; // Fix points to GT
		cfg.robustThreshold = 2.f;
		if (!BundleAdjustment::Adjust(scene, cfg)) {
			VERBOSE("Test 3 FAILED: BundleAdjustment returned false");
			return false;
		}

		VERBOSE("Test 3: Refined fx = %.2f (gt = %.2f)", cam->fx, sceneCfg.cameras[0].focal);
		const double fx_error = ABS(cam->fx - sceneCfg.cameras[0].focal);
		if (fx_error > 5.0) {
			VERBOSE("Test 3 FAILED: focal length error = %.2f > 5.0", fx_error);
			return false;
		}
		VERBOSE("Test 3 PASSED (fx error = %.2f pixels)", fx_error);
	}

	// Test 4: Refine radial distortion
	{
		VERBOSE("\n--- Test 4: Refine radial distortion ---");
		Scene scene;

		// Generate test scene (GT) with distortion - no perturbations
		SceneConfig sceneCfg;
		sceneCfg.cameras[0].k1 = 0.1;
		sceneCfg.cameras[0].k2 = -0.05;
		sceneCfg.numImages = 4;
		sceneCfg.numPoints = 80;
		sceneCfg.perturbOptions = SceneConfig::PERTURB_NONE;
		GenerateTestScene(scene, sceneCfg);

		// Manually reset distortion - keypoints remain at GT positions
		PinholeCamera* cam = (PinholeCamera*)scene.cameras[0];
		cam->k1 = 0.05; // Initial guess (GT is 0.1)
		cam->k2 = 0.0;
		cam->trustIntrinsics = false; // Allow BA to refine
		VERBOSE("Test 4: Initial k1=%.4f, k2=%.4f (gt: k1=%.4f, k2=%.4f)",
		        cam->k1, cam->k2, sceneCfg.cameras[0].k1, sceneCfg.cameras[0].k2);

		BAConfig cfg;
		cfg.maxIterations = 40;
		cfg.refineRadialDistortion12 = true;
		cfg.refinePosesRotation = cfg.refinePosesPosition = false;  // Fix poses to GT
		cfg.refinePoints = false; // Fix points to GT
		if (!BundleAdjustment::Adjust(scene, cfg)) {
			VERBOSE("Test 4 FAILED: BundleAdjustment returned false");
			return false;
		}

		VERBOSE("Test 4: Refined k1=%.4f, k2=%.4f (gt: k1=%.4f, k2=%.4f)",
		        cam->k1, cam->k2, sceneCfg.cameras[0].k1, sceneCfg.cameras[0].k2);
		const double k1_error = ABS(cam->k1 - sceneCfg.cameras[0].k1);
		const double k2_error = ABS(cam->k2 - sceneCfg.cameras[0].k2);
		if (k1_error > 0.02 || k2_error > 0.02) {
			VERBOSE("Test 4 FAILED: distortion error too large (k1=%.4f, k2=%.4f)", k1_error, k2_error);
			return false;
		}
		VERBOSE("Test 4 PASSED (k1 error=%.4f, k2 error=%.4f)", k1_error, k2_error);
	}

	// Test 5: GPS position constraints
	{
		VERBOSE("\n--- Test 5: GPS position constraints ---");
		Scene sceneGT, scene;

		SceneConfig sceneCfg;
		sceneCfg.numImages = 4;
		sceneCfg.numPoints = 80;
		sceneCfg.poseMode = SceneConfig::RANDOM_POSES;
		sceneCfg.generateGPS = true; // Generate GPS metadata automatically
		sceneCfg.perturbOptions = SceneConfig::PERTURB_POSES;
		GenerateTestScene(sceneGT, sceneCfg, &scene);
		VERBOSE("Test 5: Initial position error = %.3f m (view 0)",
		        norm(scene.images[0].C - sceneGT.images[0].C));

		BAConfig cfg;
		cfg.gpsPositionWeight = 1.0;     // Enable GPS constraints
		cfg.gpsPositionWeightZ = 1.0;
		cfg.gpsWeightScaleFactor = 0.1;  // Reduce influence for test
		if (!BundleAdjustment::Adjust(scene, cfg)) {
			VERBOSE("Test 5 FAILED: BundleAdjustment returned false");
			return false;
		}

		// Check if positions are closer to ground truth
		double total_pos_error = 0.0;
		FOREACH(i, scene.images) {
			double err = norm(scene.images[i].C - sceneGT.images[i].C);
			total_pos_error += err;
		}
		double mean_pos_error = total_pos_error / scene.images.size();
		if (mean_pos_error > 0.2) {
			VERBOSE("Test 5 FAILED: mean position error = %.3f m > 0.2 m", mean_pos_error);
			return false;
		}
		VERBOSE("Test 5 PASSED (mean position error = %.3f m)", mean_pos_error);
	}

	// Test 6: Scene::Transform - verify that transforming scene preserves projections
	{
		VERBOSE("\n--- Test 6: Scene::Transform with projection verification ---");
		Scene scene;
		std::mt19937 rng(456);

		// Generate test scene with random poses and tracks
		SceneConfig sceneCfg;
		#ifdef _RELEASE
		std::random_device rd;
		sceneCfg.randomSeed = rd();
		#endif
		sceneCfg.numImages = 5;
		sceneCfg.numPoints = 100;
		sceneCfg.poseMode = SceneConfig::RANDOM_POSES;
		GenerateTestScene(scene, sceneCfg);

		// Select a subset of points to track (e.g. every 8th track with enough observations)
		struct PointProjection {
			uint32_t trackIdx;
			uint32_t imageIdx;
			Point2 projection;
		};
		std::vector<PointProjection> originalProjections;
		// Compute original projections for each selected point
		unsigned numSelectedPoints = 0;
		FOREACH(trackIdx, scene.tracks) {
			const Track& track = scene.tracks[trackIdx];
			if (!track.IsInlier())
				continue; // at least 2 inlier observations
			for (const auto& obs : track) {
				const Image& img = scene.images[obs.imageID];
				const auto [proj, valid] = img.ProjectPoint(track.position);
				if (valid)
					originalProjections.push_back({trackIdx, obs.imageID, proj});
			}
			++numSelectedPoints;
			trackIdx += 7; // skip some tracks to reduce total number of projections
		}
		if (originalProjections.empty()) {
			VERBOSE("Test 6 FAILED: no projections could be computed");
			return false;
		}
		VERBOSE("Test 6: Generated %u original projections for %u selected points",
			(unsigned)originalProjections.size(), numSelectedPoints);

		// Generate random transformation
		Transform T = Transform::Random(rng);
		VERBOSE("Test 6: Applying random transform: scale=%.4f, translation=%.4f,%.4f,%.4f",
			T.scale, T.t.x, T.t.y, T.t.z);

		// Apply transformation to scene
		scene.Transform(T);

		// Recompute projections and compare with original
		int errorCount = 0;
		REAL maxProjectionError = 0.f, sumProjectionError = 0.f;
		for (const auto& origProj : originalProjections) {
			const Track& track = scene.tracks[origProj.trackIdx];
			const Image& img = scene.images[origProj.imageIdx];
			const auto [newProj, valid] = img.ProjectPoint(track.position);
			if (!valid) {
				VERBOSE("Test 6 FAILED: projection invalid after transform");
				return false;
			}

			const REAL pixelError = norm(newProj - origProj.projection);
			maxProjectionError = MAX(maxProjectionError, pixelError);
			sumProjectionError += pixelError;
			errorCount++;

			// Allow small numerical error (up to 0.01 pixels)
			if (pixelError > 0.01f) {
				VERBOSE("Test 6 WARNING: projection error = %.4f pixels (track %u, image %u)",
					pixelError, origProj.trackIdx, origProj.imageIdx);
			}
		}

		// With floating point arithmetic and transformation, we expect very small errors
		// (due to numerical precision, not algorithmic issues)
		const REAL meanProjectionError = errorCount > 0 ? sumProjectionError / errorCount : 0.f;
		if (meanProjectionError > 0.01f) {
			VERBOSE("Test 6 FAILED: mean projection error too large = %.6f pixels", meanProjectionError);
			return false;
		}
		VERBOSE("Test 6 PASSED (max projection error = %.6f pixels, mean = %.6f pixels)",
		        maxProjectionError, meanProjectionError);
	}
	return true;
}

// GPS-prior BA on a geo-aligned scene: the priors anchor the gauge, so BA fixes no
// pose and ComputePoseUncertainty must return absolute (datum-free) covariances;
// also exercises the missing-accuracy fallback (a view without EXIF accuracy tags
// must not produce non-finite residuals).
bool GPSPriorPoseUncertaintyTest()
{
	VERBOSE("\n=== GPSPriorPoseUncertaintyTest: absolute pose covariance under GPS priors ===");
	Scene sceneGT, scene;
	SceneConfig sceneCfg;
	sceneCfg.numImages = 6;
	sceneCfg.numPoints = 100;
	sceneCfg.poseMode = SceneConfig::RANDOM_POSES;
	sceneCfg.generateGPS = true; // synthesizes GPS metadata and aligns the scene to ENU
	sceneCfg.perturbOptions = SceneConfig::PERTURB_POSES;
	GenerateTestScene(sceneGT, sceneCfg, &scene);
	if (!scene.status.nState.isSet(Scene::Status::STATE::GEO_ALIGN)) {
		VERBOSE("GPSPriorPoseUncertaintyTest FAILED: generated scene not geo-aligned");
		return false;
	}
	// one view without accuracy tags: BA must fall back to default accuracies
	View::Metadata& meta = static_cast<View&>(scene.images[1]).metadata;
	meta.positionAccuracy = 0.f;
	meta.positionAccuracyZ = 0.f;

	BAConfig cfg;
	cfg.gpsPositionWeight = 1.0;
	cfg.gpsPositionWeightZ = 1.0;
	cfg.gpsWeightScaleFactor = 0.1;
	BundleAdjustment ba(scene, cfg);
	if (!ba.Adjust()) {
		VERBOSE("GPSPriorPoseUncertaintyTest FAILED: GPS-prior BundleAdjustment returned false");
		return false;
	}
	// poses must stay commensurate with the GPS accuracy
	double meanPosError = 0;
	FOREACH(i, scene.images)
		meanPosError += norm(scene.images[i].C - sceneGT.images[i].C);
	meanPosError /= scene.images.size();
	if (meanPosError > 0.2) {
		VERBOSE("GPSPriorPoseUncertaintyTest FAILED: mean position error = %.3f m > 0.2 m", meanPosError);
		return false;
	}

	const PoseUncertaintyArr uncertainty = ba.ComputePoseUncertainty();
	if (uncertainty.size() != scene.images.size()) {
		VERBOSE("GPSPriorPoseUncertaintyTest FAILED: pose uncertainty not computed (%u/%u images)",
			(unsigned)uncertainty.size(), (unsigned)scene.images.size());
		return false;
	}
	FOREACH(i, uncertainty) {
		const PoseUncertainty& u = uncertainty[i];
		if (!u.IsValid() ||
			!ISFINITE(u.MaxRotationVariance()) || !ISFINITE(u.MaxPositionVariance()) ||
			!ISFINITE(u.posCov.x) || !ISFINITE(u.posCov.y) || !ISFINITE(u.posCov.z)) {
			VERBOSE("GPSPriorPoseUncertaintyTest FAILED: invalid pose uncertainty for image %u", i);
			return false;
		}
		// absolute gauge: GPS priors anchor every pose, no datum must be designated
		if (u.MaxRotationVariance() == 0.f && u.MaxPositionVariance() == 0.f) {
			VERBOSE("GPSPriorPoseUncertaintyTest FAILED: unexpected gauge datum at image %u", i);
			return false;
		}
	}

	// Cross-check the fast Schur + selected-inverse covariance against Ceres' own (slow, dense)
	// covariance estimator on the same solved problem. GPS priors make the system full rank, so
	// both compute the same marginal pose covariance and must agree up to numerical error.
	const PoseUncertaintyArr reference = ba.ComputePoseUncertaintyCeres();
	if (reference.size() != uncertainty.size()) {
		VERBOSE("GPSPriorPoseUncertaintyTest FAILED: Ceres reference covariance not computed (%u/%u)",
			(unsigned)reference.size(), (unsigned)uncertainty.size());
		return false;
	}
	// Per-image relative error: position covariance via Frobenius norm, rotation variance per axis.
	const auto frob = [](const Matrix3x3f& m) {
		float s = 0.f; for (int k = 0; k < 9; ++k) s += m.val[k]*m.val[k]; return SQRT(s);
	};
	const auto frobDiff = [](const Matrix3x3f& A, const Matrix3x3f& B) {
		float s = 0.f; for (int k = 0; k < 9; ++k) { const float d = A.val[k]-B.val[k]; s += d*d; } return SQRT(s);
	};
	const auto relErr = [](float a, float b) { return ABS(a - b) / MAXF(ABS(b), 1e-12f); };
	float maxPosRelErr = 0.f, maxRotRelErr = 0.f;
	unsigned numChecked = 0;
	FOREACH(i, uncertainty) {
		const PoseUncertainty& a = uncertainty[i];
		const PoseUncertainty& b = reference[i];
		if (!a.IsValid() || !b.IsValid())
			continue;
		const Matrix3x3f Ca = a.GetPositionCovariance(), Cb = b.GetPositionCovariance();
		maxPosRelErr = MAXF(maxPosRelErr, frobDiff(Ca, Cb) / MAXF(frob(Cb), 1e-12f));
		maxRotRelErr = MAXF(maxRotRelErr, relErr(a.rotVar.x, b.rotVar.x));
		maxRotRelErr = MAXF(maxRotRelErr, relErr(a.rotVar.y, b.rotVar.y));
		maxRotRelErr = MAXF(maxRotRelErr, relErr(a.rotVar.z, b.rotVar.z));
		++numChecked;
	}
	if (numChecked == 0) {
		VERBOSE("GPSPriorPoseUncertaintyTest FAILED: no images to cross-check against Ceres");
		return false;
	}
	constexpr float crossCheckTol = 0.05f; // 5% — the two use different linear-algebra paths
	if (maxPosRelErr > crossCheckTol || maxRotRelErr > crossCheckTol) {
		VERBOSE("GPSPriorPoseUncertaintyTest FAILED: covariance disagrees with Ceres reference "
			"(max rel err: position %.3g, rotation %.3g > %.2g over %u images)",
			maxPosRelErr, maxRotRelErr, crossCheckTol, numChecked);
		return false;
	}
	VERBOSE("GPSPriorPoseUncertaintyTest PASSED (mean position error = %.3f m; "
		"Ceres cross-check max rel err: position %.3g, rotation %.3g over %u images)",
		meanPosError, maxPosRelErr, maxRotRelErr, numChecked);
	return true;
}

// Pose-quality report roundtrip: pose uncertainty recorded on the scene from the last
// BA + ExportPoseUncertaintyCSV (one row per image keyed by SFM image ID, exactly one
// all-zero gauge datum for a non-GPS BA), ExportMVS -> MVS::Scene::Load preserving the
// (non-contiguous) SFM image IDs the report is correlated by, Scene::Transform mapping
// the position covariance, and .sfm serialization preserving the record.
bool PoseUncertaintyExportTest()
{
	VERBOSE("\n=== PoseUncertaintyExportTest: quality report CSV + image-ID roundtrip ===");
	Scene sceneGT, scene;
	SceneConfig sceneCfg;
	sceneCfg.perturbOptions = SceneConfig::PERTURB_ALL;
	GenerateTestScene(sceneGT, sceneCfg, &scene);
	// non-contiguous IDs prove the CSV/interface correlation is ID-based, not index-based
	FOREACH(i, scene.images)
		scene.images[i].ID = 10 + i * 3;
	scene.status.nState.set(Scene::Status::STATE::CALIBRATED);

	const ScopedTempDir tmpDir(_T("PoseUncertaintyExportTest"));
	if (!tmpDir.IsValid())
		return false;

	const String mvsPath = tmpDir(_T("scene.mvs"));
	if (!ExportMVS(mvsPath, scene, {})) {
		VERBOSE("PoseUncertaintyExportTest FAILED: ExportMVS returned false");
		return false;
	}

	// record the pose uncertainty on the scene from the (last) bundle adjustment,
	// as Scene::Reconstruct does when ReconstructionConfig::estimatePoseUncertainty is set
	{
		BAConfig cfg;
		BundleAdjustment ba(scene, cfg);
		if (!ba.Adjust()) {
			VERBOSE("PoseUncertaintyExportTest FAILED: BundleAdjustment returned false");
			return false;
		}
		scene.poseUncertainty = ba.ComputePoseUncertainty();
	}
	if (scene.poseUncertainty.size() != scene.images.size()) {
		VERBOSE("PoseUncertaintyExportTest FAILED: pose uncertainty not computed (%u/%u images)",
			(unsigned)scene.poseUncertainty.size(), (unsigned)scene.images.size());
		return false;
	}
	const String csvPath = tmpDir(_T("quality.csv"));
	const unsigned numValid = ExportPoseUncertaintyCSV(csvPath, scene);
	if (numValid != scene.images.size()) {
		VERBOSE("PoseUncertaintyExportTest FAILED: exported %u valid rows, expected %u",
			numValid, scene.images.size());
		return false;
	}

	// re-read the CSV: one data row per image, IDs matching the assigned ones,
	// exactly one datum row with all-zero sigmas
	std::ifstream is(csvPath);
	if (!is.is_open()) {
		VERBOSE("PoseUncertaintyExportTest FAILED: cannot re-open '%s'", csvPath.c_str());
		return false;
	}
	unsigned numRows = 0, numDatum = 0;
	std::unordered_set<unsigned long> csvIDs;
	std::string line;
	while (std::getline(is, line)) {
		if (line.empty() || line[0] == '#')
			continue;
		std::vector<std::string> fields;
		size_t start = 0;
		for (size_t pos; (pos = line.find(',', start)) != std::string::npos; start = pos + 1)
			fields.push_back(line.substr(start, pos - start));
		fields.push_back(line.substr(start));
		char* end;
		const unsigned long id = std::strtoul(fields[0].c_str(), &end, 10);
		if (end == fields[0].c_str() || *end != '\0')
			continue; // header line
		if (fields.size() < 16) {
			VERBOSE("PoseUncertaintyExportTest FAILED: CSV row with %u fields, expected 16", (unsigned)fields.size());
			return false;
		}
		++numRows;
		csvIDs.insert(id);
		if (fields[3] != "0") {
			++numDatum;
			for (int f = 4; f <= 12; ++f) {
				if (std::atof(fields[f].c_str()) != 0.0) {
					VERBOSE("PoseUncertaintyExportTest FAILED: non-zero sigma on the datum row (field %d)", f);
					return false;
				}
			}
		}
	}
	if (numRows != scene.images.size() || numDatum != 1) {
		VERBOSE("PoseUncertaintyExportTest FAILED: %u rows (%u expected), %u datum rows (1 expected)",
			numRows, scene.images.size(), numDatum);
		return false;
	}
	FOREACH(i, scene.images) {
		if (csvIDs.count(scene.images[i].ID) == 0) {
			VERBOSE("PoseUncertaintyExportTest FAILED: image ID %u missing from the CSV", scene.images[i].ID);
			return false;
		}
	}

	// the exported .mvs must preserve the SFM image IDs (the report correlation key)
	MVS::Scene mvsScene(1);
	if (mvsScene.Load(mvsPath) == MVS::Scene::SCENE_NA) {
		VERBOSE("PoseUncertaintyExportTest FAILED: MVS::Scene::Load returned SCENE_NA");
		return false;
	}
	if (mvsScene.images.size() != scene.images.size()) {
		VERBOSE("PoseUncertaintyExportTest FAILED: %u MVS images, expected %u",
			(unsigned)mvsScene.images.size(), scene.images.size());
		return false;
	}
	FOREACH(i, mvsScene.images) {
		if (mvsScene.images[i].ID != scene.images[i].ID) {
			VERBOSE("PoseUncertaintyExportTest FAILED: MVS image %u has ID %u, expected %u",
				i, mvsScene.images[i].ID, scene.images[i].ID);
			return false;
		}
	}
	// vertex views must reference array positions, still in range
	for (const MVS::PointCloud::ViewArr& views : mvsScene.pointcloud.pointViews)
		for (const MVS::PointCloud::View view : views)
			if (view >= mvsScene.images.size()) {
				VERBOSE("PoseUncertaintyExportTest FAILED: vertex view %u out of range", view);
				return false;
			}

	// world-transform consistency: Scene::Transform must map the recorded position
	// covariance as scale^2 * R * Cov * R^T and leave the rotation variance untouched
	IIndex idxCheck = NO_ID;
	FOREACH(i, scene.poseUncertainty)
		if (scene.poseUncertainty[i].IsValid() && scene.poseUncertainty[i].MaxPositionVariance() > 0.f) {
			idxCheck = i;
			break;
		}
	if (idxCheck == NO_ID) {
		VERBOSE("PoseUncertaintyExportTest FAILED: no non-datum uncertainty entry to check");
		return false;
	}
	const PoseUncertainty before = scene.poseUncertainty[idxCheck];
	std::mt19937 rng(789);
	const Transform T = Transform::Random(rng);
	scene.Transform(T);
	const PoseUncertainty& after = scene.poseUncertainty[idxCheck];
	const Matrix3x3 cov(
		before.posVar.x, before.posCov.x, before.posCov.y,
		before.posCov.x, before.posVar.y, before.posCov.z,
		before.posCov.y, before.posCov.z, before.posVar.z);
	const Matrix3x3 covT(T.R * cov * T.R.t() * SQUARE(T.scale));
	const auto isNear = [](float a, float b) {
		return ABS(a - b) <= 1e-3f * MAXF(MAXF(ABS(a), ABS(b)), 1e-12f);
	};
	if (!isNear((float)covT(0,0), after.posVar.x) || !isNear((float)covT(1,1), after.posVar.y) || !isNear((float)covT(2,2), after.posVar.z) ||
	    !isNear((float)covT(0,1), after.posCov.x) || !isNear((float)covT(0,2), after.posCov.y) || !isNear((float)covT(1,2), after.posCov.z) ||
	    after.rotVar != before.rotVar) {
		VERBOSE("PoseUncertaintyExportTest FAILED: transformed covariance mismatch");
		return false;
	}

	// serialization roundtrip preserves the recorded uncertainty
	const String sfmPath = tmpDir(_T("scene.sfm"));
	if (!scene.Save(sfmPath, ARCHIVE_BINARY)) {
		VERBOSE("PoseUncertaintyExportTest FAILED: scene save failed");
		return false;
	}
	Scene scene2;
	if (!scene2.Load(sfmPath) || scene2.poseUncertainty.size() != scene.poseUncertainty.size()) {
		VERBOSE("PoseUncertaintyExportTest FAILED: scene load lost the pose uncertainty");
		return false;
	}
	FOREACH(i, scene.poseUncertainty) {
		const PoseUncertainty& a = scene.poseUncertainty[i];
		const PoseUncertainty& b = scene2.poseUncertainty[i];
		if (a.rotVar != b.rotVar || a.posVar != b.posVar || a.posCov != b.posCov) {
			VERBOSE("PoseUncertaintyExportTest FAILED: serialized uncertainty mismatch at image %u", i);
			return false;
		}
	}

	VERBOSE("PoseUncertaintyExportTest PASSED (%u images, %u valid rows)", scene.images.size(), numValid);
	return true;
}

namespace {
constexpr REAL RESECTION_ARC_EXTENT = 5; // 2*RESECTION_ARC_RADIUS*sin(3*RESECTION_ARC_STEP)
} // namespace

// Five arc cameras observe 300 tracks at their exact projections, and camera 2's stored pose is put off its true
// one the way a camera placed before the lens was known sits in the model, its observations several pixels off the
// points the four others hold exactly:
// - turned REPEATED_TEST_TURN_DEG about its vertical axis: every observation 8.7-9.4 px off, past the 4 px bar and
//   within 3 x 4 = 12 px. A single solve of the inlier prefix has nothing to move the camera with and leaves it off;
//   the repeated adjustment fits them, brings the camera back in one solve and stops there, the next solve's input
//   being the same as this one's;
// - rolled REPEATED_TEST_ROLL_DEG about its optical axis: its observations 0.8-20 px off, the ones near the image
//   centre within 12 px and the ones near the corners past it. The first solve brings the camera back from the inner
//   ones, the outer ones then enter the input, which changes far more than the share of the camera's input that calls
//   for another solve, and the second solve changes nothing, so the adjustment ends after two;
// - rolled REPEATED_TEST_SHORT_ROLL_DEG with solves of one iteration: the first solve stops short of the camera's
//   pose and the next ones finish the way; with ResectionConfig::fullBARepeats 0 the adjustment is that first solve
//   alone, the camera left off where it stopped, exactly as one solve of the 12 px input leaves it.
bool RepeatedFullAdjustmentTest()
{
	TD_TIMER_START();

	constexpr unsigned NUM_TRACKS = 300;
	constexpr IIndex OFF = 2;
	constexpr REAL REPEATED_TEST_TURN_DEG = 1; // 8.7 px at a 500 px focal, the middle of 4-12 px
	constexpr REAL REPEATED_TEST_ROLL_DEG = 7; // 0.8-20 px off: the points project within 163 px of the image centre
	constexpr REAL REPEATED_TEST_SHORT_ROLL_DEG = 10; // with one iteration a solve leaves it 0.05 deg off
	constexpr double MAX_RECOVERED_ROTATION_DEG = 0.02; // where a recovered camera has to end
	constexpr double MIN_SHORT_ROTATION_DEG = 0.04; // where one one-iteration solve leaves the short-rolled camera
	const ResectionConfig config;
	struct Outcome {
		double rotationError = 0;          // degrees off its true rotation after the adjustment
		unsigned prefixObservations = 0;   // of its observations, the ones in an inlier prefix after it
		unsigned solves = 0;               // solves the adjustment ran (0 = a single solve of the prefix)
		double minError = DBL_MAX, maxError = 0; // its observations' errors at the pose it started from
		unsigned withinInput = 0;          // of them, the ones within the 12 px input bar
		RMatrix R;                         // its rotation after the adjustment
	};
	enum Mode { PREFIX, INPUT_ONCE, REPEATED };
	// the arc scene with camera OFF moved by `offset`, the observations filtered at the bar, then a single solve of
	// the inlier prefix, a single solve of the 12 px input (the filter at the bar after either), or the repeated
	// adjustment with `repeats`
	const auto adjust = [&](const RMatrix& offset, Mode mode, unsigned repeats, unsigned maxIterations) {
		Outcome out;
		std::mt19937 rng(20261015);
		Scene scene;
		BuildResectionArc(scene, 5);
		AddResectionTracks(scene, {0, 1, 2, 3, 4}, NUM_TRACKS, rng);
		Image& img = scene.images[OFF];
		const RMatrix trueR(img.R);
		img.R = offset * trueR;
		for (const Track& track : scene.tracks) {
			for (const Observation& obs : track.observations) {
				if (obs.imageID != OFF)
					continue;
				const auto [proj, valid] = img.ProjectPoint(track.position);
				ASSERT(valid);
				const double error = norm(Point2(proj) - Cast<REAL>(img.keypoints[obs.featureID].pt));
				out.minError = MINF(out.minError, error);
				out.maxError = MAXF(out.maxError, error);
				out.withinInput += error < ResectionConfig::fullBAInputErrorFactor * config.maxReprojError;
			}
		}
		ResectionConfig cfg = config;
		cfg.fullBARepeats = repeats;
		BAConfig baConfig = cfg.fullBAConfig;
		baConfig.maxIterations = maxIterations;
		baConfig.numThreads = 1; // the solver's own threads sum in the order they finish, and two of these solves are compared to the last digit
		FilterTracks(scene, cfg.maxReprojError, cfg.minAngleThreshold, cfg.multDepthNear, cfg.multDepthFar);
		if (mode == REPEATED) {
			out.solves = AdjustRepeated(scene, baConfig, cfg);
		} else {
			if (mode == INPUT_ONCE)
				FilterTracks(scene, cfg.maxReprojError * ResectionConfig::fullBAInputErrorFactor, cfg.minAngleThreshold, cfg.multDepthNear, cfg.multDepthFar);
			BundleAdjustment::Adjust(scene, baConfig);
			FilterTracks(scene, cfg.maxReprojError, cfg.minAngleThreshold, cfg.multDepthNear, cfg.multDepthFar);
		}
		out.R = img.R;
		out.rotationError = RotationErrorDeg(img.R, trueR);
		for (const Track& track : scene.tracks)
			for (const Observation& obs : track)
				out.prefixObservations += obs.imageID == OFF;
		return out;
	};

	const unsigned iterations = config.fullBAConfig.maxIterations;
	const RMatrix turn(Point3(0, 1, 0), D2R(REPEATED_TEST_TURN_DEG));
	const RMatrix roll(Point3(0, 0, 1), D2R(REPEATED_TEST_ROLL_DEG));
	const RMatrix shortRoll(Point3(0, 0, 1), D2R(REPEATED_TEST_SHORT_ROLL_DEG));
	const Outcome turnedPrefix = adjust(turn, PREFIX, config.fullBARepeats, iterations);
	const Outcome turned = adjust(turn, REPEATED, config.fullBARepeats, iterations);
	const Outcome rolled = adjust(roll, REPEATED, config.fullBARepeats, iterations);
	const Outcome shortRepeated = adjust(shortRoll, REPEATED, config.fullBARepeats, 1);
	const Outcome shortOnce = adjust(shortRoll, REPEATED, 0, 1);
	const Outcome shortInput = adjust(shortRoll, INPUT_ONCE, 0, 1);
	// the premises: the turned camera's observations all past the bar and within the input bar; the rolled one's
	// partly within the input bar, enough of them to bring it back and few enough for the rest to change its input
	const double inputBar = ResectionConfig::fullBAInputErrorFactor * config.maxReprojError;
	if (config.fullBARepeats == 0 || turned.minError <= config.maxReprojError || turned.maxError >= inputBar ||
		rolled.withinInput < NUM_TRACKS / 4 || rolled.withinInput > NUM_TRACKS * 9 / 10) {
		VERBOSE("RepeatedFullAdjustmentTest FAILED: premise not met: %u repeats by default, the turned camera's observations "
			"are %.2f-%.2f px off (all between %g and %g expected), %u of the rolled one's (%.2f-%.2f px off) are within %g px "
			"(%u-%u expected)", config.fullBARepeats, turned.minError, turned.maxError, config.maxReprojError, inputBar,
			rolled.withinInput, rolled.minError, rolled.maxError, inputBar, NUM_TRACKS / 4, NUM_TRACKS * 9 / 10);
		return false;
	}
	// a single solve of the prefix does not move the turned camera, and none of its observations comes back
	if (turnedPrefix.rotationError < 0.9*REPEATED_TEST_TURN_DEG || turnedPrefix.prefixObservations != 0) {
		VERBOSE("RepeatedFullAdjustmentTest FAILED: a single solve of the inlier prefix leaves the turned camera %.4f deg off "
			"with %u observations in the prefix; expected it left %g deg off with none", turnedPrefix.rotationError,
			turnedPrefix.prefixObservations, REPEATED_TEST_TURN_DEG);
		return false;
	}
	for (const auto& [name, out, solves] : {std::make_tuple("turned", turned, 1u), std::make_tuple("rolled", rolled, 2u)}) {
		if (out.rotationError > MAX_RECOVERED_ROTATION_DEG || out.prefixObservations != NUM_TRACKS || out.solves != solves) {
			VERBOSE("RepeatedFullAdjustmentTest FAILED: the %s camera ends %.4f deg off with %u of its %u observations in "
				"the prefix after %u solves; expected it within %g deg with all of them after %u", name, out.rotationError,
				out.prefixObservations, NUM_TRACKS, out.solves, MAX_RECOVERED_ROTATION_DEG, solves);
			return false;
		}
	}
	// solves of one iteration: repeated, the camera comes back; with no repeats, one solve, the camera left where the
	// 12 px input's single solve leaves it
	if (shortRepeated.rotationError > MAX_RECOVERED_ROTATION_DEG || shortRepeated.solves < 2) {
		VERBOSE("RepeatedFullAdjustmentTest FAILED: with solves of one iteration the repeated adjustment leaves the camera "
			"%.4f deg off after %u solves; expected it within %g deg after two or more", shortRepeated.rotationError,
			shortRepeated.solves, MAX_RECOVERED_ROTATION_DEG);
		return false;
	}
	if (shortOnce.solves != 1 || shortOnce.rotationError < MIN_SHORT_ROTATION_DEG ||
		RotationErrorDeg(shortOnce.R, shortInput.R) > 1e-9) {
		VERBOSE("RepeatedFullAdjustmentTest FAILED: with no repeats and solves of one iteration the adjustment ran %u solves "
			"and left the camera %.4f deg off, %.2e deg from where one solve of the 12 px input leaves it; expected one solve, "
			"at least %g deg off, at that same pose", shortOnce.solves, shortOnce.rotationError,
			RotationErrorDeg(shortOnce.R, shortInput.R), MIN_SHORT_ROTATION_DEG);
		return false;
	}
	VERBOSE("RepeatedFullAdjustmentTest PASSED: a camera turned %g deg (observations %.2f-%.2f px off) stays %.4f deg off "
		"through a single solve of the inlier prefix and ends %.4f deg off after %u solve of the repeated adjustment; rolled "
		"%g deg (%u of %u observations within %g px) it ends %.4f deg off after %u solves; rolled %g deg with solves of one "
		"iteration it ends %.4f deg off after %u solves, and %.4f deg off after the single solve of no repeats (%s)",
		REPEATED_TEST_TURN_DEG, turned.minError, turned.maxError, turnedPrefix.rotationError, turned.rotationError,
		turned.solves, REPEATED_TEST_ROLL_DEG, rolled.withinInput, NUM_TRACKS, inputBar, rolled.rotationError, rolled.solves,
		REPEATED_TEST_SHORT_ROLL_DEG, shortRepeated.rotationError, shortRepeated.solves, shortOnce.rotationError,
		TD_TIMER_GET_FMT().c_str());
	return true;
}

// A full adjustment stopped by its solve cap ends as one stopped for want of change: the tracks without a point
// triangulated again at the bar. The arc scene with ResectionConfig::fullBARepeats 0 (one solve, the cap reached at
// once) and CAPPED_TRIANGULATION_TRACKS of its tracks left without a point (inlier prefix 0, position moved away),
// every one of them seen at its exact projection by the five cameras: after AdjustRepeated every one holds a point
// again, within the bar of its keypoints.
bool CappedAdjustmentTriangulationTest()
{
	TD_TIMER_START();
	constexpr unsigned NUM_TRACKS = 200;
	constexpr unsigned CAPPED_TRIANGULATION_TRACKS = 20;
	std::mt19937 rng(20261111);
	Scene scene;
	BuildResectionArc(scene, 5);
	AddResectionTracks(scene, {0, 1, 2, 3, 4}, NUM_TRACKS, rng);
	ResectionConfig cfg;
	cfg.fullBARepeats = 0;
	for (unsigned t = 0; t < CAPPED_TRIANGULATION_TRACKS; ++t) {
		Track& track = scene.tracks[t];
		track.position += Point3(10, 10, 10);
		track.numInliers = 0;
	}
	const unsigned solves = AdjustRepeated(scene, cfg.fullBAConfig, cfg);
	unsigned numBack = 0;
	for (unsigned t = 0; t < CAPPED_TRIANGULATION_TRACKS; ++t)
		numBack += scene.tracks[t].IsInlier() ? 1u : 0u;
	if (solves != 1 || numBack != CAPPED_TRIANGULATION_TRACKS) {
		VERBOSE("CappedAdjustmentTriangulationTest FAILED: %u solves (1 expected, the cap), %u of the %u tracks left without "
			"a point hold one after the adjustment (all expected)", solves, numBack, CAPPED_TRIANGULATION_TRACKS);
		return false;
	}
	VERBOSE("CappedAdjustmentTriangulationTest PASSED: the adjustment stopped by its cap after %u solve triangulated the %u "
		"pointless tracks again (%s)", solves, numBack, TD_TIMER_GET_FMT().c_str());
	return true;
}

// Twelve arc cameras in a chain, every track seen by three consecutive cameras at its exact projection: per triple a
// few described tracks, which the per-image observation cap keeps whole, and many dense ones, of which it keeps a few
// observations an image and leaves most tracks out of the solve. The cameras and the dense points sit at the truth;
// the described points start CAPPED_SCALE_PUSH further from the cameras (scaled about their centroid), the depth error
// a solve of a new input corrects. A global solve with the cap applied, one camera held as the reference and no GPS:
// with nothing else holding the scale, the iterations that bring the described points back also move the cameras along it
// (+0.83% here), and the dense tracks the cap left out keep their points, so 12% of their observations land past the
// 4 px bar (mean 2.3 px) and the filter after the solve would drop them. Held, the solve ends at the truth: every
// camera where it was and every left-out observation on its keypoint.
bool CappedSolveScaleTest()
{
	TD_TIMER_START();

	constexpr unsigned NUM_CAMERAS = 12;
	constexpr unsigned NUM_DESCRIBED = 30;          // per triple: at most 90 an image, all kept by the cap
	constexpr unsigned NUM_DENSE = 400;             // per triple: the cap keeps what is left of an image's budget
	constexpr REAL CAPPED_SCALE_PUSH = 0.02;        // the described points' start, this share further from the cameras
	constexpr unsigned CAPPED_SCALE_CAP = 100;      // observations an image keeps, whatever the scene's size
	constexpr double MAX_SCALE_CHANGE = 1e-4;       // the cameras' scale after the solve against the true one
	constexpr double MAX_LEFT_OUT_PAST_BAR = 0.01;  // share of the left-out observations past the 4 px bar
	const ResectionConfig config;
	std::mt19937 rng(20261016);
	Scene scene;
	BuildResectionArc(scene, NUM_CAMERAS);
	for (unsigned k = 0; k + 2 < NUM_CAMERAS; ++k)
		AddResectionTracks(scene, {k, k + 1, k + 2}, NUM_DESCRIBED, rng);
	const size_t numDescribedTracks = scene.tracks.size();
	for (Image& img : scene.images)
		img.CloseDescribedKeypoints();
	for (unsigned k = 0; k + 2 < NUM_CAMERAS; ++k)
		AddResectionTracks(scene, {k, k + 1, k + 2}, NUM_DENSE, rng);
	std::vector<Point3> trueCenters;
	for (const Image& img : scene.images)
		trueCenters.push_back(img.C);
	Point3 cameraCentroid(0, 0, 0);
	for (const Point3& c : trueCenters)
		cameraCentroid += c;
	cameraCentroid *= REAL(1) / trueCenters.size();
	for (size_t ti = 0; ti < numDescribedTracks; ++ti)
		scene.tracks[ti].position = cameraCentroid + (scene.tracks[ti].position - cameraCentroid) * (REAL(1) + CAPPED_SCALE_PUSH);
	std::vector<Point3> startPoints;
	for (const Track& track : scene.tracks)
		startPoints.push_back(track.position);
	FilterTracks(scene, config.maxReprojError * ResectionConfig::fullBAInputErrorFactor, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	BAConfig baConfig = config.fullBAConfig;
	baConfig.maxObservationsPerImage = CAPPED_SCALE_CAP;
	baConfig.minObservationsForCap = 0;
	BundleAdjustment::Adjust(scene, baConfig);

	// the scale: the spread of the camera centres about their centroid, against the true one
	const auto spread = [](const std::vector<Point3>& centers) {
		Point3 mean(0, 0, 0);
		for (const Point3& c : centers)
			mean += c;
		mean *= REAL(1) / centers.size();
		double sum = 0;
		for (const Point3& c : centers)
			sum += normSq(c - mean);
		return SQRT(sum / centers.size());
	};
	std::vector<Point3> centers;
	for (const Image& img : scene.images)
		centers.push_back(img.C);
	const double scaleChange = spread(centers) / spread(trueCenters) - 1;
	// the observations of the inlier tracks the solve left out (their points untouched)
	unsigned leftOut = 0, leftOutTracks = 0, pastBar = 0, solvedDescribed = 0;
	double sumError = 0;
	FOREACH(ti, scene.tracks) {
		const Track& track = scene.tracks[ti];
		if (!track.IsInlier())
			continue;
		if (track.position != startPoints[ti]) {
			solvedDescribed += ti < numDescribedTracks;
			continue;
		}
		++leftOutTracks;
		for (const Observation& obs : track) {
			const Image& img = scene.images[obs.imageID];
			const auto [proj, valid] = img.ProjectPoint(track.position);
			const double error = valid ? norm(Point2(proj) - Cast<REAL>(img.keypoints[obs.featureID].pt)) : DBL_MAX;
			++leftOut;
			pastBar += error > config.maxReprojError;
			sumError += MINF(error, 1e3);
		}
	}
	if (leftOut == 0 || solvedDescribed != numDescribedTracks) {
		VERBOSE("CappedSolveScaleTest FAILED: premise not met: the cap left %u tracks out of the solve and solved %u of the %u "
			"described ones", leftOutTracks, solvedDescribed, (unsigned)numDescribedTracks);
		return false;
	}
	const double pastShare = (double)pastBar / leftOut;
	if (ABS(scaleChange) > MAX_SCALE_CHANGE || pastShare > MAX_LEFT_OUT_PAST_BAR) {
		VERBOSE("CappedSolveScaleTest FAILED: the capped solve changed the cameras' scale by %+.4f%% and left %u of the %u "
			"observations of the %u tracks it left out past %g px (mean %.2f px); expected within %g%% and %g%%",
			scaleChange * 100, pastBar, leftOut, leftOutTracks, config.maxReprojError, sumError / leftOut, MAX_SCALE_CHANGE * 100,
			MAX_LEFT_OUT_PAST_BAR * 100);
		return false;
	}
	VERBOSE("CappedSolveScaleTest PASSED: a capped solve without GPS kept the cameras' scale within %+.4f%% and %u of the %u "
		"observations of the %u tracks it left out within %g px (mean %.3f px) (%s)", scaleChange * 100, leftOut - pastBar,
		leftOut, leftOutTracks, config.maxReprojError, sumError / leftOut, TD_TIMER_GET_FMT().c_str());
	return true;
}

bool DenseReprojectionBarTest()
{
	TD_TIMER_START();
	std::mt19937 rng(20260914);

	// Seven posed cameras of the arc share 30 exact tracks, plus two tracks whose observation in the
	// last image is 7 pixels off: a described keypoint carries one, a dense keypoint the other. The
	// displacement is vertical, across the plane of the arc, the direction the other six views
	// constrain tightly: a least-squares triangulation then leaves most of it in the displaced view
	// (about six sevenths, between the 4-pixel bar and twice it), where a displacement along the arc
	// would be traded for depth and spread over every view.
	constexpr unsigned numViews = 7;
	constexpr float displacement = 7.f;
	Scene scene;
	BuildResectionArc(scene, numViews);
	const std::vector<IIndex> allViews{0, 1, 2, 3, 4, 5, 6};
	AddResectionTracks(scene, allViews, 30, rng);
	if (!AddResectionTrack(scene, Point3(0.2, 0.1, -0.3), allViews)) {
		VERBOSE("DenseReprojectionBarTest FAILED: the described track does not project inside every image");
		return false;
	}
	const size_t idxDescribed = scene.tracks.size() - 1;
	Image& lastImage = scene.images[numViews - 1];
	lastImage.CloseDescribedKeypoints(); // every keypoint the last image gains from here on is dense
	if (!AddResectionTrack(scene, Point3(-0.3, 0.2, 0.1), allViews)) {
		VERBOSE("DenseReprojectionBarTest FAILED: the dense track does not project inside every image");
		return false;
	}
	const size_t idxDense = scene.tracks.size() - 1;
	for (const size_t idx : {idxDescribed, idxDense}) {
		const Observation& obs = scene.tracks[idx].observations.back();
		ASSERT(obs.imageID == numViews - 1);
		lastImage.keypoints[obs.featureID].pt.y += displacement;
	}
	if (lastImage.IsDenseKeypoint(scene.tracks[idxDescribed].observations.back().featureID) ||
		!lastImage.IsDenseKeypoint(scene.tracks[idxDense].observations.back().featureID)) {
		VERBOSE("DenseReprojectionBarTest FAILED: the two displaced keypoints are not one described and one dense");
		return false;
	}
	const auto Check = [&scene, idxDescribed, idxDense](const char* stage, unsigned expectedDescribed, unsigned expectedDense) {
		if (scene.tracks[idxDescribed].numInliers != expectedDescribed || scene.tracks[idxDense].numInliers != expectedDense) {
			VERBOSE("DenseReprojectionBarTest FAILED: %s kept %u/%u observations of the described track and %u/%u of "
				"the dense one, expected %u and %u", stage, scene.tracks[idxDescribed].numInliers, numViews,
				scene.tracks[idxDense].numInliers, numViews, expectedDescribed, expectedDense);
			return false;
		}
		return true;
	};

	// At a 4-pixel bar with the dense observations held to twice that, the filter keeps the dense
	// observation and drops the described one; held to the same bar, it drops both
	FilterTracks(scene, 4.f, 1.f, 0.f, 0.f, 2.f);
	if (!Check("the filter, with dense observations held to twice the 4-pixel bar,", numViews - 1, numViews))
		return false;
	FilterTracks(scene, 4.f, 1.f, 0.f, 0.f, 1.f);
	if (!Check("the filter, at a 4-pixel bar for both kinds,", numViews - 1, numViews - 1))
		return false;

	// The triangulation reads the two bars the same way
	for (Track& track : scene.tracks)
		track.position = Point3(0, 0, 0);
	TriangulateTracks(scene, false, 4.f, 1.f, 2.f);
	if (!Check("the triangulation, with dense observations held to twice the 4-pixel bar,", numViews - 1, numViews))
		return false;
	for (Track& track : scene.tracks)
		track.position = Point3(0, 0, 0);
	TriangulateTracks(scene, false, 4.f, 1.f, 1.f);
	if (!Check("the triangulation, at a 4-pixel bar for both kinds,", numViews - 1, numViews - 1))
		return false;

	VERBOSE("DenseReprojectionBarTest PASSED: the track filter and the triangulation hold a dense observation to its "
		"own multiple of the reprojection bar (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool FilterTracksUnposedTrackTest()
{
	TD_TIMER_START();
	std::mt19937 rng(20261027);

	// The star puts its reference view exactly at the world origin, and a track whose only posed view
	// is that reference was never triangulated (TriangulateTracks leaves it out), so it still holds the
	// default position, which a scene saved before the default became an improbable point stores as (0,0,0):
	// the point the reference camera sits on (the track below is given that position). The filter must not judge such
	// a track, because it has no position to judge: from the reference the point is at the camera
	// centre, the angle to the observed ray is 0/0 (NaN), which the gate's comparison lets through.
	// Image 0 is the reference at the origin, image 3 has no pose, and the positionless track sees
	// exactly those two; 30 exact tracks over the posed images 1 and 2 show the filter still judges
	// the tracks that can have a position.
	Scene scene;
	BuildResectionArc(scene, 4);
	static_cast<Pose3D&>(scene.images[0]) = Pose3D::Identity();
	scene.images[3].InvalidatePose();
	AddResectionTracks(scene, {1, 2}, 30, rng);
	const size_t numExact = scene.tracks.size();
	Track positionless;
	positionless.position = Point3(0, 0, 0); // what a scene saved before the default was an improbable point stores
	for (const IIndex imageID : {IIndex(0), IIndex(3)}) {
		Image& img = scene.images[imageID];
		positionless.observations.emplace_back(imageID, (uint32_t)img.keypoints.size());
		img.keypoints.emplace_back(cv::Point2f(100.f, 150.f), 1.f);
	}
	scene.tracks.emplace_back(std::move(positionless));
	if (!scene.images[0].IsValid() || scene.images[3].IsValid() || scene.images[0].C != CMatrix::ZERO ||
		scene.tracks.back().position != Point3(0, 0, 0)) {
		VERBOSE("FilterTracksUnposedTrackTest FAILED: premise not met: the reference at the origin, the unposed image and the track at the default position");
		return false;
	}

	FilterTracks(scene, 4.f, 1.f, 0.f, 0.f, 1.f);
	const Track& track = scene.tracks.back();
	if (track.numInliers != 0) {
		VERBOSE("FilterTracksUnposedTrackTest FAILED: the filter judged the track with one posed view at its default position "
			"and kept %u of its %u observations, expected none", track.numInliers, track.GetNumObservations());
		return false;
	}
	for (size_t i = 0; i < numExact; ++i) {
		if (scene.tracks[i].numInliers != 2) {
			VERBOSE("FilterTracksUnposedTrackTest FAILED: the exact track %zu of the two posed views kept %u observations, expected 2",
				i, scene.tracks[i].numInliers);
			return false;
		}
	}
	if (scene.status.nTracks != numExact) {
		VERBOSE("FilterTracksUnposedTrackTest FAILED: the filter counted %u inlier tracks, expected the %zu exact ones", scene.status.nTracks, numExact);
		return false;
	}

	VERBOSE("FilterTracksUnposedTrackTest PASSED: the track filter leaves a track with one posed view, at the origin where the reference camera "
		"sits, unjudged and not an inlier, and keeps judging the %zu tracks of two posed views (%s)", numExact, TD_TIMER_GET_FMT().c_str());
	return true;
}

bool FilterTracksUntriangulatedTrackTest()
{
	TD_TIMER_START();

	// A track of two posed views that was never triangulated is judged by the callers that filter before
	// they triangulate (the resection's full adjustment, the merged model's adjustment) at the position it
	// was made with. That position must not be a point a posed camera can sit on: the star puts its
	// reference camera exactly at the world origin, where a position of (0,0,0) makes the camera-frame point
	// zero, its angle to the observed ray 0/0 (NaN), which the gate's comparison lets through (a kept
	// observation of a track that has no position, and ASSERT(projValid) in a Debug build). A track
	// made without a position holds an improbable one instead, far from every keypoint and with rays that
	// meet at no angle, so the filter's own tests reject it.
	Scene scene;
	BuildResectionArc(scene, 2);
	static_cast<Pose3D&>(scene.images[0]) = Pose3D::Identity();
	Track& untriangulated = scene.tracks.emplace_back(); // as BuildTracks makes its tracks
	for (const IIndex imageID : {IIndex(0), IIndex(1)}) {
		Image& img = scene.images[imageID];
		untriangulated.observations.emplace_back(imageID, (uint32_t)img.keypoints.size());
		img.keypoints.emplace_back(cv::Point2f(100.f, 150.f), 1.f);
	}
	if (!scene.images[0].IsValid() || !scene.images[1].IsValid() || scene.images[0].C != CMatrix::ZERO ||
		scene.tracks.back().GetNumPosedObservations(scene.images) != 2) {
		VERBOSE("FilterTracksUntriangulatedTrackTest FAILED: premise not met: the reference at the origin, a second posed image and a track of both");
		return false;
	}

	FilterTracks(scene, 4.f, 1.f, 0.f, 0.f, 1.f);
	const Track& track = scene.tracks.back();
	if (track.numInliers != 0 || scene.status.nTracks != 0) {
		VERBOSE("FilterTracksUntriangulatedTrackTest FAILED: the filter kept %u of the %u observations of a track that was never triangulated "
			"(%u inlier tracks), expected none", track.numInliers, track.GetNumObservations(), scene.status.nTracks);
		return false;
	}
	const Point3 untriangulatedPosition(Track::UNTRIANGULATED_COORD);
	if (Track().position != untriangulatedPosition || track.position != untriangulatedPosition ||
		Track::UNTRIANGULATED_COORD < 1e12 || !ISFINITE(Track::UNTRIANGULATED_COORD * Track::UNTRIANGULATED_COORD)) {
		VERBOSE("FilterTracksUntriangulatedTrackTest FAILED: a track made without a position holds (%g,%g,%g) and a default Track (%g,%g,%g), "
			"expected the same far finite coordinate %g in all three", track.position.x, track.position.y, track.position.z,
			Track().position.x, Track().position.y, Track().position.z, Track::UNTRIANGULATED_COORD);
		return false;
	}

	VERBOSE("FilterTracksUntriangulatedTrackTest PASSED: a track of two posed views, one of them the camera at the origin, that was never triangulated "
		"holds the far default position and the filter rejects it (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool TriangulationFailureTest()
{
	TD_TIMER_START();

	// Two posed views of the arc see one exact track, at a point 100 units beyond the middle of their
	// two centres: about half a degree of parallax. The keypoints are exact, so the reprojection test
	// passes and both views are inliers, and the rays meet below the 1.5-degree minimum angle. A
	// triangulation that fails on the angle returns 0, and the track it leaves must hold no inliers:
	// IsInlier() is what the adjustments that follow read, and the caller counts the track as failed
	constexpr float minAngle = 1.5f, looseAngle = 0.1f;
	Scene scene;
	BuildResectionArc(scene, 2);
	const Point3 mid = (scene.images[0].C + scene.images[1].C) * 0.5;
	if (!AddResectionTrack(scene, mid * (1 - 100 / norm(mid)), {0, 1})) {
		VERBOSE("TriangulationFailureTest FAILED: the track does not project inside both images");
		return false;
	}
	const auto Fresh = [&scene]() {
		Track track(scene.tracks[0]);
		track.position = Point3(0, 0, 0);
		track.numInliers = 0;
		return track;
	};
	bool passed = true;
	const auto Check = [&passed](const char* what, bool ok, unsigned returned, const Track& track) {
		if (ok)
			return;
		VERBOSE("TriangulationFailureTest FAILED: %s returned %u, leaving the track with %u inliers (%s), expected 0 and no inliers",
			what, returned, track.numInliers, track.IsInlier() ? "an inlier" : "not an inlier");
		passed = false;
	};

	// Control: held to a looser angle, the same track triangulates, with the parallax the test means
	{
		Track track = Fresh();
		const unsigned returned = TriangulateSkewLLS(track, scene.images, 4.f, looseAngle);
		const double angle = R2D(track.ComputeMaxAngleBetweenRays(scene.images));
		if (returned != 2 || !track.IsInlier() || angle < 0.25 || angle > 1.0) {
			VERBOSE("TriangulationFailureTest FAILED: premise not met: at %g degrees the track triangulates with %u inliers (%s) and %.3f degrees of parallax, expected 2 and a parallax between 0.25 and 1",
				looseAngle, returned, track.IsInlier() ? "an inlier" : "not an inlier", angle);
			return false;
		}
	}
	{
		Track track = Fresh();
		const unsigned returned = TriangulateSkewLLS(track, scene.images, 4.f, minAngle);
		Check("TriangulateSkewLLS below the minimum angle", returned == 0 && !track.IsInlier(), returned, track);
	}
	{
		Track track = Fresh();
		const unsigned returned = TriangulateDLT(track, scene.images, 4.f, minAngle);
		Check("TriangulateDLT below the minimum angle", returned == 0 && !track.IsInlier(), returned, track);
	}
	{
		Track& track = scene.tracks[0];
		track.position = Point3(0, 0, 0);
		track.numInliers = 0;
		const unsigned returned = TriangulateTracks(scene, false, 4.f, minAngle);
		Check("TriangulateTracks below the minimum angle", returned == 0 && !track.IsInlier() && scene.status.nTracks == 0, returned, track);
		// the track did not triangulate, so the next call that only triangulates the tracks without inliers
		// takes it up again, and at a looser angle it succeeds
		const unsigned retried = TriangulateTracks(scene, true, 4.f, looseAngle);
		if (retried != 1 || !track.IsInlier()) {
			VERBOSE("TriangulationFailureTest FAILED: the next TriangulateTracks of the tracks without inliers, at %g degrees, triangulated %u tracks (the track is %s), expected 1 and an inlier",
				looseAngle, retried, track.IsInlier() ? "an inlier" : "not an inlier");
			passed = false;
		}
	}

	// A triangulation that fails for want of inliers leaves none either: three views of one point whose
	// second and third keypoints are 10 pixels off in opposite directions across the plane of the arc
	// (the direction every view constrains alike) leave the first view the only one within the 4-pixel
	// bar of the solution, so one inlier is counted against the minimum of two. Measured on the solution
	// the two functions count against, the first view is 1.14 pixels off in TriangulateSkewLLS and 2.12 in
	// TriangulateDLT, the other two 10.04 and 10.03 in the first and 10.10 and 10.19 in the second. The
	// angle bar is 0 in the two checks, so that only the count of inliers can fail them
	{
		constexpr float displacement = 10.f;
		Scene fit;
		BuildResectionArc(fit, 3);
		if (!AddResectionTrack(fit, Point3(0.2, 0.1, -0.3), {0, 1, 2})) {
			VERBOSE("TriangulationFailureTest FAILED: the track of three views does not project inside every image");
			return false;
		}
		const Track& exact = fit.tracks[0];
		fit.images[1].keypoints[exact.observations[1].featureID].pt.y += displacement;
		fit.images[2].keypoints[exact.observations[2].featureID].pt.y -= displacement;
		const auto FreshFit = [&fit]() {
			Track track(fit.tracks[0]);
			track.position = Point3(0, 0, 0);
			track.numInliers = 0;
			return track;
		};
		// Control: with no angle bar and one inlier enough, both functions count the one inlier the test means
		{
			Track skew = FreshFit(), dlt = FreshFit();
			const unsigned returnedSkew = TriangulateSkewLLS(skew, fit.images, 4.f, 0.f, 1);
			const unsigned returnedDLT = TriangulateDLT(dlt, fit.images, 4.f, 0.f, 1);
			if (returnedSkew != 1 || returnedDLT != 1) {
				VERBOSE("TriangulationFailureTest FAILED: premise not met: with one inlier enough and no angle bar TriangulateSkewLLS counts %u inliers and TriangulateDLT %u, expected 1 and 1",
					returnedSkew, returnedDLT);
				return false;
			}
		}
		{
			Track track = FreshFit();
			const unsigned returned = TriangulateSkewLLS(track, fit.images, 4.f, 0.f);
			Check("TriangulateSkewLLS with one inlier", returned == 0 && track.GetNumInliers() == 0, returned, track);
		}
		{
			Track track = FreshFit();
			const unsigned returned = TriangulateDLT(track, fit.images, 4.f, 0.f);
			Check("TriangulateDLT with one inlier", returned == 0 && track.GetNumInliers() == 0, returned, track);
		}
		// and so does one that comes in with inliers counted earlier and has one posed view left, which
		// the function turns down before it counts anything
		{
			fit.images[1].InvalidatePose();
			fit.images[2].InvalidatePose();
			Track track = FreshFit();
			track.numInliers = 2;
			const unsigned returned = TriangulateSkewLLS(track, fit.images, 4.f, minAngle);
			Check("TriangulateSkewLLS with one posed view", returned == 0 && track.GetNumInliers() == 0, returned, track);
		}
	}
	if (!passed)
		return false;

	VERBOSE("TriangulationFailureTest PASSED: a triangulation that fails, for the minimum angle (TriangulateSkewLLS, TriangulateDLT, TriangulateTracks) or for want of "
		"a second inlier (TriangulateSkewLLS, TriangulateDLT), leaves the track with no inliers, and the next TriangulateTracks of the tracks without inliers "
		"takes it up again (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// TriangulateSkewLLS on spherical cameras whose rays lie across the optical axis. A point seen by three
// 4096x2048 panoramas side by side (one metre apart, identity rotation) 10 m away in the plane of their
// centres has a bearing with dz = 0 in every one of them: the first two rows of [d]x R, the two the
// system once kept, are then parallel and give a single equation per view that fixes only the depth
// along the optical axis, so the point is not determined across it. The three rows determine it.
// Exact keypoints at dz = 0 must triangulate to the point (the two rows return the minimum-norm
// solution, metres away, which the reprojection test then rejects); with 1 pixel of noise the median
// position error at dz = 0 and 0.05 must stay under 0.2 m (the two rows: about 10 m and 0.4 m, the
// three: about 0.09 m)
bool SphericalTriangulationTest()
{
	TD_TIMER_START();
	constexpr int width = 4096, height = 2048, numViews = 3, numPoints = 200;
	Scene scene;
	scene.cameras.emplace_back(new SphericalCamera(cv::Size(width, height)));
	for (unsigned i = 0; i < numViews; ++i) {
		Pose3D pose;
		pose.C = Point3(i, 0, 0);
		pose.R = Matrix3x3::IDENTITY;
		scene.images.emplace_back(static_cast<IIndex>(i), String(), pose, 0, scene.cameras[0]);
	}
	std::mt19937 rng(4096);
	std::uniform_real_distribution<REAL> azimuthDist(REAL(0.5), REAL(M_PI) - REAL(0.5));
	std::normal_distribution<float> noiseDist(0.f, 1.f);
	bool passed = true;
	for (const REAL dz : {REAL(0), REAL(0.05)}) {
		for (const bool noisy : {false, true}) {
			if (!noisy && dz != REAL(0))
				continue;
			CLISTDEF0(double) errors;
			for (int p = 0; p < numPoints; ++p) {
				// a bearing from the first view with the given dz, on either side of the baseline
				const REAL azimuth = azimuthDist(rng) * ((p & 1) ? REAL(1) : REAL(-1));
				const REAL radial = SQRT(REAL(1) - dz*dz);
				const Point3 X = Point3(radial * COS(azimuth), radial * SIN(azimuth), dz) * REAL(10);
				for (Image& img : scene.images)
					img.keypoints.clear();
				Track track(Point3(0, 0, 0));
				for (Image& img : scene.images) {
					const auto [proj, valid] = img.ProjectPoint(X);
					if (!valid || !Image8U::isInside(proj, img.GetSize())) {
						VERBOSE("SphericalTriangulationTest FAILED: the point does not project inside view %u", img.ID);
						return false;
					}
					cv::Point2f pt = Cast<float>(proj);
					if (noisy) {
						pt.x += noiseDist(rng);
						pt.y += noiseDist(rng);
					}
					img.keypoints.emplace_back(pt, 0.f, 0.f, 10.f);
					track.observations.emplace_back(img.ID, 0);
				}
				const unsigned returned = TriangulateSkewLLS(track, scene.images, 8.f, 0.f);
				if (!noisy && (returned != numViews || norm(track.position - X) > 5e-3)) {
					VERBOSE("SphericalTriangulationTest FAILED: exact keypoints at dz = %g: %u inliers, position %g m off, expected %d and under 0.005",
						(double)dz, returned, norm(track.position - X), numViews);
					passed = false;
					break;
				}
				errors.push_back(norm(track.position - X));
			}
			if (!noisy || errors.empty())
				continue;
			const double median = errors.GetMedian();
			if (!(median < 0.2)) {
				VERBOSE("SphericalTriangulationTest FAILED: with 1 pixel of noise at dz = %g the median position error is %.3f m, expected under 0.2",
					(double)dz, median);
				passed = false;
			}
			VERBOSE("SphericalTriangulationTest: dz = %g, 1 px noise: median position error %.3f m", (double)dz, median);
		}
	}
	if (!passed)
		return false;
	VERBOSE("SphericalTriangulationTest PASSED: a point across the optical axis of three panoramas triangulates (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

namespace {

// How thick, in Y, the slab the cross-joint points are drawn from is: they fill the usual 2-unit box
// in X and Z but only this much of it in Y, so at 0 they lie exactly in the plane the arc lies in.
// That is the lever on how much of the bend the two-view geometry across the joint can see: raise it
// and the points off the plane start to resist, so the reprojections alone recover part of the bend.
constexpr REAL JOINT_TRACK_SPREAD_Y = 0;

// numTracks tracks the given images share, at points drawn from the 2-unit box around the origin
// flattened to the given half-height in Y
void AddFlatResectionTracks(Scene& scene, const std::vector<IIndex>& imageIDs, unsigned numTracks,
	REAL spreadY, std::mt19937& rng)
{
	std::uniform_real_distribution<REAL> coord(-1, 1);
	for (unsigned i = 0; i < numTracks; ) {
		if (AddResectionTrack(scene, Point3(coord(rng), spreadY*coord(rng), coord(rng)), imageIDs))
			++i;
	}
}

// Eight cameras of the arc, with a joint between cameras 3 and 4. Cameras 0-3 and cameras 4-7 each
// carry three-view tracks, so either half of the chain holds its own shape; the only structure
// across the joint is 40 tracks cameras 3 and 4 share with nobody else, so no track spans 2-3-4 or
// 3-4-5 and the two halves are joined by two-view geometry alone. Those cross-joint points lie in
// the plane of the arc -- the plane the camera centres and the axis the joint bends about span --
// and a two-view pair reads a rotation about that axis only off the points that lie outside it, so
// with none there the reprojections cannot tell how far the joint is bent. That is the joint a
// doorway or a textureless wall leaves in a real capture, stated exactly. Verified pairs carrying
// the true relative poses and 300 inliers join every camera to the next two along the chain, the
// joint included, which is the evidence the reprojections do not have.
void BuildPairConstraintScene(Scene& scene, std::vector<Pose3D>& gtPoses, std::mt19937& rng)
{
	BuildResectionArc(scene, 8);
	gtPoses.clear();
	for (const Image& img : scene.images)
		gtPoses.push_back(static_cast<const Pose3D&>(img));

	AddResectionTracks(scene, {0, 1, 2}, 80, rng);
	AddResectionTracks(scene, {1, 2, 3}, 80, rng);
	AddFlatResectionTracks(scene, {3, 4}, 40, JOINT_TRACK_SPREAD_Y, rng);
	AddResectionTracks(scene, {4, 5, 6}, 80, rng);
	AddResectionTracks(scene, {5, 6, 7}, 80, rng);

	FOREACH(id, scene.images) {
		if (id + 1 < scene.images.size())
			AddResectionPair(scene, id, id + 1, 300);
		if (id + 2 < scene.images.size())
			AddResectionPair(scene, id, id + 2, 300);
	}
	TriangulateTracks(scene, false, 4.f, 1.f);
}

// Bend the model at the joint: cameras 4-7 and the points only they observe turn as one rigid block
// about the arc's axis through camera 4's centre, the hinge. The block keeps its own shape, so every
// track inside it still projects exactly where it is observed and the reprojection residuals of both
// halves stay at zero; what the turn contradicts is every pair across the joint, by the whole angle.
void BendPairConstraintScene(Scene& scene, REAL angleDegrees)
{
	const Point3 pivot = scene.images[4].C;
	const RMatrix rot(REAL(0), D2R(angleDegrees), REAL(0)); // about the world Y axis, the arc's axis
	for (IIndex id = 4; id < scene.images.size(); ++id) {
		Image& img = scene.images[id];
		img.C = pivot + rot*(img.C - pivot);
		img.R = img.R * rot.t();
	}
	for (Track& track : scene.tracks) {
		bool blockOnly = true;
		for (const Observation& obs : track.observations)
			if (obs.imageID < 4) { blockOnly = false; break; }
		if (blockOnly)
			track.position = pivot + rot*(track.position - pivot);
	}
}

// The largest rotation error (degrees) and camera-centre error (world units) of the model against
// the truth, over every image.
//
// A bundle adjustment with no metric prior holds one camera fixed, which pins where the model sits
// and how it is turned but not how large it is: scaling every camera and every point about that
// camera changes no reprojection, and no relative-pose residual either, since a direction says
// nothing about length. That one free gauge is taken out before the centres are compared -- the
// model is resized about its own centroid until it spreads as far as the truth does -- so what the
// centre error measures is the model's shape. A solve that fixes a whole block of cameras leaves no
// such freedom, and the resizing is then a no-op.
void WorstPoseError(const Scene& scene, const std::vector<Pose3D>& gtPoses,
	double& maxRotationDeg, double& maxCenter)
{
	Point3 centroid(0, 0, 0), gtCentroid(0, 0, 0);
	FOREACH(id, scene.images) {
		centroid += scene.images[id].C;
		gtCentroid += gtPoses[id].C;
	}
	centroid /= (REAL)scene.images.size();
	gtCentroid /= (REAL)scene.images.size();
	REAL spread = 0, gtSpread = 0;
	FOREACH(id, scene.images) {
		spread += (REAL)norm(scene.images[id].C - centroid);
		gtSpread += (REAL)norm(gtPoses[id].C - gtCentroid);
	}
	const REAL scale = spread > ZEROTOLERANCE<REAL>() ? gtSpread/spread : REAL(1);
	maxRotationDeg = maxCenter = 0;
	FOREACH(id, scene.images) {
		maxRotationDeg = MAXF(maxRotationDeg, RotationErrorDeg(scene.images[id].R, gtPoses[id].R));
		const Point3 center(gtCentroid + (scene.images[id].C - centroid)*scale);
		maxCenter = MAXF(maxCenter, (double)norm(center - gtPoses[id].C));
	}
}
} // namespace

// The verified pairs hold the model where the tracks cannot: a chain bent at a joint whose only
// structure is two-view and degenerate stays bent under the reprojection residuals alone, and the
// relative-pose residuals straighten it -- in the global solve and in the local one alike
bool BundleAdjustmentPairConstraintTest()
{
	TD_TIMER_START();
	constexpr REAL PAIR_CONSTRAINT_BEND = 10;                        // degrees the block is turned by
	constexpr double MAX_ROTATION_ERROR = 0.5;                       // degrees
	constexpr double MAX_CENTER_ERROR = 0.01 * RESECTION_ARC_EXTENT; // 1% of the arc's extent
	const IIndex farthestFromHinge = 7; // the camera at the far end of the turned block

	// Without them the bend survives the solve: no reprojection residual in either half of the chain
	// is violated by it, and the cross-joint tracks are blind to it
	{
		std::mt19937 rng(20260909);
		Scene scene;
		std::vector<Pose3D> gtPoses;
		BuildPairConstraintScene(scene, gtPoses, rng);
		BendPairConstraintScene(scene, PAIR_CONSTRAINT_BEND);
		BAConfig config;
		config.relativeRotationSigma = config.relativeTranslationSigma = 0.f;
		if (!BundleAdjustment::Adjust(scene, config)) {
			VERBOSE("BundleAdjustmentPairConstraintTest FAILED: the bundle adjustment did not solve the bent scene "
				"with the pair residuals off");
			return false;
		}
		const double rotationError = RotationErrorDeg(scene.images[farthestFromHinge].R, gtPoses[farthestFromHinge].R);
		if (rotationError <= 5.0) {
			VERBOSE("BundleAdjustmentPairConstraintTest FAILED: camera %u is %.2f degrees from the truth after the "
				"solve with the pair residuals off, so the reprojections already straighten the %.0f-degree bend and "
				"the scene does not pose the problem", farthestFromHinge, rotationError, (double)PAIR_CONSTRAINT_BEND);
			return false;
		}
	}

	// With them every pair across the joint is heard, and the block returns to where it belongs
	{
		std::mt19937 rng(20260909);
		Scene scene;
		std::vector<Pose3D> gtPoses;
		BuildPairConstraintScene(scene, gtPoses, rng);
		BendPairConstraintScene(scene, PAIR_CONSTRAINT_BEND);
		if (!BundleAdjustment::Adjust(scene, BAConfig())) {
			VERBOSE("BundleAdjustmentPairConstraintTest FAILED: the bundle adjustment did not solve the bent scene "
				"with the pair residuals on");
			return false;
		}
		double maxRotationDeg, maxCenter;
		WorstPoseError(scene, gtPoses, maxRotationDeg, maxCenter);
		if (maxRotationDeg > MAX_ROTATION_ERROR || maxCenter > MAX_CENTER_ERROR) {
			VERBOSE("BundleAdjustmentPairConstraintTest FAILED: the global solve left the model %.3f degrees and "
				"%.4f units from the truth, past the %.1f degrees and %.3f units the pairs should recover",
				maxRotationDeg, maxCenter, MAX_ROTATION_ERROR, MAX_CENTER_ERROR);
			return false;
		}
	}

	// And the local solve is held the same way: cameras 0-3 are constants, so the pairs across the
	// joint constrain the free block alone
	{
		std::mt19937 rng(20260909);
		Scene scene;
		std::vector<Pose3D> gtPoses;
		BuildPairConstraintScene(scene, gtPoses, rng);
		BendPairConstraintScene(scene, PAIR_CONSTRAINT_BEND);
		IIndexArr localViews, fixedViews;
		for (IIndex id = 0; id < 4; ++id)
			fixedViews.push_back(id);
		for (IIndex id = 4; id < 8; ++id)
			localViews.push_back(id);
		if (!BundleAdjustment::AdjustLocal(scene, localViews, fixedViews, BAConfig())) {
			VERBOSE("BundleAdjustmentPairConstraintTest FAILED: the local bundle adjustment did not solve the bent scene");
			return false;
		}
		double maxRotationDeg, maxCenter;
		WorstPoseError(scene, gtPoses, maxRotationDeg, maxCenter);
		if (maxRotationDeg > MAX_ROTATION_ERROR || maxCenter > MAX_CENTER_ERROR) {
			VERBOSE("BundleAdjustmentPairConstraintTest FAILED: the local solve left the model %.3f degrees and "
				"%.4f units from the truth, past the %.1f degrees and %.3f units the pairs should recover",
				maxRotationDeg, maxCenter, MAX_ROTATION_ERROR, MAX_CENTER_ERROR);
			return false;
		}
	}

	VERBOSE("BundleAdjustmentPairConstraintTest PASSED: a %.0f-degree bend at a joint the tracks leave free survives "
		"the reprojection residuals and is undone by the relative-pose residuals of the verified pairs, globally and "
		"in a local window alike (%s)", (double)PAIR_CONSTRAINT_BEND, TD_TIMER_GET_FMT().c_str());
	return true;
}

// Give every track of a generated scene a dense twin of each of its observations, and add a
// dense-only copy of every track: the first kind of track ranks above the second inside the
// budget, so a solve over this scene meets both halves of the cap's rule that a track takes two
// observations into it or none -- the tracks it has to top back up, and the ones it may let go.
void AddDenseObservations(Scene& scene)
{
	for (Image& img : scene.images)
		img.CloseDescribedKeypoints();
	TrackArr denseTracks;
	for (Track& track : scene.tracks) {
		Track denseTrack(track.position);
		const unsigned numObservations = track.GetNumInliers();
		for (unsigned i = 0; i < numObservations; ++i) {
			const Observation obs = track.observations[i];
			Image& img = scene.images[obs.imageID];
			const cv::KeyPoint kp(img.keypoints[obs.featureID]);
			// the twin the described track takes, then the one its dense-only copy takes
			for (Observation* denseObs : { &track.observations.AddEmpty(), &denseTrack.observations.AddEmpty() }) {
				*denseObs = Observation(obs.imageID, (uint32_t)img.keypoints.size());
				img.keypoints.push_back(kp);
			}
		}
		track.numInliers = (uint8_t)MINF((unsigned)track.observations.size(), 255u);
		denseTrack.numInliers = (uint8_t)MINF((unsigned)denseTrack.observations.size(), 255u);
		denseTracks.emplace_back(std::move(denseTrack));
	}
	scene.tracks.Join(denseTracks);
}

// Observations the scene holds, the count the per-image budget's threshold is measured against:
// the registered images' observations of its inlier tracks, over the whole of scene.tracks
unsigned CountSceneObservations(const Scene& scene)
{
	unsigned numObservations = 0;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track)
			if (scene.images[obs.imageID].IsValid())
				++numObservations;
	}
	return numObservations;
}

bool ReprojectionReportOverUsedTest()
{
	TD_TIMER_START();
	// a posed synthetic scene whose observations are exact: every error below is one this test put there. The
	// observations of image 0 stand for those a per-image budget leaves out of a solve
	constexpr IIndex LEFT_OUT = 0;
	constexpr float USED_ERROR = 0.5f, LEFT_OUT_ERROR = 1.f, HUGE_ERROR = 1e6f;
	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 4;
	cfg.numPoints = 200;
	GenerateTestScene(scene, cfg);
	unsigned numLeftOut = 0, numUsed = 0;
	for (const Track& track : scene.tracks) {
		for (const Observation& obs : track) {
			cv::KeyPoint& kp = scene.images[obs.imageID].keypoints[obs.featureID];
			if (obs.imageID != LEFT_OUT) {
				kp.pt.x += USED_ERROR;
				++numUsed;
			} else {
				kp.pt.x += numLeftOut++ == 0 ? HUGE_ERROR : LEFT_OUT_ERROR;
			}
		}
	}
	if (numLeftOut == 0 || numUsed == 0) {
		VERBOSE("ReprojectionReportOverUsedTest FAILED: the scene holds %u observations of image %u and %u of the others", numLeftOut, (unsigned)LEFT_OUT, numUsed);
		return false;
	}
	const auto [meanUsed, angUsed] = ComputeTracksMeanReprojectionError(scene, [](uint32_t imageID, uint32_t) { return imageID != LEFT_OUT; });
	if (ABS(meanUsed - USED_ERROR) > 0.01f || !ISFINITE(angUsed)) {
		VERBOSE("ReprojectionReportOverUsedTest FAILED: the mean over the used observations is %.4f px, expected the %.2f px they carry", (double)meanUsed, (double)USED_ERROR);
		return false;
	}
	// the mean over every observation, as a solve without a budget reports it, is the one the left-out outlier dominates
	const float meanAll = ComputeTracksMeanReprojectionError(scene).first;
	if (meanAll < 10.f * USED_ERROR) {
		VERBOSE("ReprojectionReportOverUsedTest FAILED: the mean over every observation is %.4f px, expected the left-out outlier to dominate it", (double)meanAll);
		return false;
	}
	VERBOSE("ReprojectionReportOverUsedTest PASSED: %.3f px over the %u used observations (%u left out) against %.1f px over all (%s)",
		(double)meanUsed, numUsed, numLeftOut, (double)meanAll, TD_TIMER_GET_FMT().c_str());
	return true;
}

bool BAObservationCapTest()
{
	TD_TIMER_START();
	// The cap decides which observations an image contributes, so what it must not change is the
	// solution: on a scene whose images carry far more of them than it allows, the capped solve has
	// to recover the poses the uncapped one recovers -- on a dense-matched scene and on a
	// described-only one alike, since the budget covers every keypoint kind. What it keeps is drawn
	// again for every solve, so the comparison is on the solution and never on the kept set.
	constexpr double MAX_ROTATION_ERROR = 0.05;   // degrees the cap may add to the uncapped solve's
	constexpr double MAX_CENTER_ERROR = 0.005;    // units it may add (the arrangement spans ~6)
	constexpr unsigned CAP = 100;                 // well under what an image of either scene holds
	SceneConfig cfg;
	cfg.numImages = 8;
	cfg.numPoints = 300;
	cfg.perturbOptions = SceneConfig::PERTURB_POSES | SceneConfig::PERTURB_POINTS;
	double maxRotationDeg[2], maxCenter[2], maxRotationDegUncapped[2], maxCenterUncapped[2];
	unsigned iScene = 0;
	for (const bool dense : {true, false}) {
		const char* const kind = dense ? "dense-matched" : "described-only";
		Scene truth, capped, uncapped;
		GenerateTestScene(truth, cfg, &capped);
		{
			Scene truthAgain;
			GenerateTestScene(truthAgain, cfg, &uncapped);
		}
		if (dense) {
			AddDenseObservations(capped);
			AddDenseObservations(uncapped);
		}
		std::vector<Pose3D> gtPoses;
		for (const Image& img : truth.images)
			gtPoses.push_back(img);

		BAConfig config;
		config.minObservationsForCap = 0; // whatever this scene's size, the budget applies
		config.maxObservationsPerImage = 0; // every observation
		config.minTriangulationAngle = 1.f; // the bar the budget keeps every track at
		if (!BundleAdjustment::Adjust(uncapped, config)) {
			VERBOSE("BAObservationCapTest FAILED: the uncapped bundle adjustment did not solve the %s scene", kind);
			return false;
		}
		config.maxObservationsPerImage = CAP;
		if (!BundleAdjustment::Adjust(capped, config)) {
			VERBOSE("BAObservationCapTest FAILED: the capped bundle adjustment did not solve the %s scene", kind);
			return false;
		}
		WorstPoseError(uncapped, gtPoses, maxRotationDegUncapped[iScene], maxCenterUncapped[iScene]);
		WorstPoseError(capped, gtPoses, maxRotationDeg[iScene], maxCenter[iScene]);
		if (maxRotationDeg[iScene] > maxRotationDegUncapped[iScene] + MAX_ROTATION_ERROR ||
			maxCenter[iScene] > maxCenterUncapped[iScene] + MAX_CENTER_ERROR) {
			VERBOSE("BAObservationCapTest FAILED: the capped solve of the %s scene left the model %.4f degrees and "
				"%.5f units from the truth, past the %.4f and %.5f the uncapped one reaches by more than the %.2f "
				"degrees and %.3f units the cap may cost", kind, maxRotationDeg[iScene], maxCenter[iScene],
				maxRotationDegUncapped[iScene], maxCenterUncapped[iScene], MAX_ROTATION_ERROR, MAX_CENTER_ERROR);
			return false;
		}

		// and a solve the threshold leaves alone fits every observation it was given
		const unsigned numObservations = CountSceneObservations(capped);
		config.minObservationsForCap = numObservations + 1;
		BundleAdjustment ba(capped, config);
		if (!ba.Adjust()) {
			VERBOSE("BAObservationCapTest FAILED: the uncapped bundle adjustment did not re-solve the %s scene", kind);
			return false;
		}
		unsigned numResiduals = 0;
		for (const unsigned numImageResiduals : ba.GetNumReprojResidualsPerImage())
			numResiduals += numImageResiduals;
		if (numResiduals != numObservations) {
			VERBOSE("BAObservationCapTest FAILED: the %s scene is under the threshold the cap applies from, yet its "
				"solve fitted %u of its %u observations", kind, numResiduals, numObservations);
			return false;
		}
		++iScene;
	}
	VERBOSE("BAObservationCapTest PASSED: capping an image at %u observations leaves the solve %.4f degrees and %.5f "
		"units from the truth against %.4f and %.5f uncapped on a dense-matched scene, %.4f and %.5f against %.4f and "
		"%.5f on a described-only one, and a scene under the threshold keeps every observation (%s)",
		CAP, maxRotationDeg[0], maxCenter[0], maxRotationDegUncapped[0], maxCenterUncapped[0],
		maxRotationDeg[1], maxCenter[1], maxRotationDegUncapped[1], maxCenterUncapped[1], TD_TIMER_GET_FMT().c_str());
	return true;
}

// A track the per-image budget leaves with only views that lie close together gets back the dropped view
// that spans the widest angle with one of them. Image 2 holds 100 observations against a budget of 95, all
// in one cell of its frame, the 60 shortest tracks (seen by the images 0, 1 and 2, the first two 10 degrees
// apart) being the only ones the budget can take from it: the 40 longer ones (images 2-5) come first in the cell.
// Two views 10 degrees apart are under the 15-degree bar, and image 2 sees the point 20 degrees from image 0,
// so a track that lost its view in image 2 is given it back and the image fits all of its 12 observations.
bool BAObservationCapWideViewTest()
{
	TD_TIMER_START();
	constexpr unsigned CAP = 95, NUM_SHORT = 60, NUM_LONG = 40;
	constexpr float BAR = 15.f;
	constexpr IIndex WIDE = 2;
	Scene scene;
	BuildResectionArc(scene, 6);
	// the points are drawn from the pixels of the wide image's frame, a few pixels around the middle of one
	// cell of the budget's 10 x 10 grid (the cell [320,384) x [240,288)), at a depth near the arc's centre
	std::mt19937 rng(20261114);
	std::uniform_real_distribution<REAL> jitter(-5, 5), depth(4.8, 5.2);
	const Image& wide = scene.images[WIDE];
	for (unsigned i = 0; i < NUM_SHORT + NUM_LONG; ) {
		const Point2 pixel(352 + jitter(rng), 264 + jitter(rng));
		const Point3 X = wide.C + wide.R.t() * (wide.pCamera->UnprojectNormalized(pixel) * depth(rng));
		const bool added = i < NUM_SHORT ? AddResectionTrack(scene, X, {0, 1, WIDE}) : AddResectionTrack(scene, X, {WIDE, 3, 4, 5});
		if (added)
			++i;
	}
	FilterTracks(scene, 4.f, 1.f);
	BAConfig config;
	config.maxObservationsPerImage = CAP;
	config.minObservationsForCap = 0;
	config.minTriangulationAngle = BAR;
	BundleAdjustment ba(scene, config);
	if (!ba.Adjust()) {
		VERBOSE("BAObservationCapWideViewTest FAILED: the capped bundle adjustment did not solve the scene");
		return false;
	}
	const UnsignedArr& fitted = ba.GetNumReprojResidualsPerImage();
	const unsigned expected[6] = {NUM_SHORT, NUM_SHORT, NUM_SHORT + NUM_LONG, NUM_LONG, NUM_LONG, NUM_LONG};
	for (IIndex id = 0; id < 6; ++id) {
		if (fitted[id] != expected[id]) {
			VERBOSE("BAObservationCapWideViewTest FAILED: image %u fits %u observations, expected %u (image %u holds %u against a budget of %u: the tracks that lose "
				"a view there have their other two views about %.0f degrees apart, under the %.0f-degree bar, and take it back)",
				id, fitted[id], expected[id], WIDE, NUM_SHORT + NUM_LONG, CAP, 10.0, (double)BAR);
			return false;
		}
	}
	VERBOSE("BAObservationCapWideViewTest PASSED: image %u fits all of its %u observations against a budget of %u, the %u it would lose each taken back for the "
		"track whose other two views lie 10 degrees apart (%s)", WIDE, NUM_SHORT + NUM_LONG, CAP, NUM_SHORT + NUM_LONG - CAP, TD_TIMER_GET_FMT().c_str());
	return true;
}

// The same bundle adjustment solved twice, the second time with the small blocks it allocates laid out
// in the reverse order, ends at the same bits. From BAConfig::minResidualsForSparseSolver reprojection
// residuals on a solve also refines its blocks by inner iterations, one round of mutually independent
// blocks after another, and which blocks share a round is a choice among blocks with as many neighbours
// as one another: made by where the solver's records of the blocks lie in memory, a reconstruction would
// follow from what the heap held before it as much as from its input. Ten cameras along the arc, each
// joined by a verified pair to its two nearest neighbours on either side -- a pose graph whose cameras
// tie in their number of links: the two ends, the two next to them and the six between -- and 1200
// three-view tracks, moved off the truth with the poses so the solve has work to do. The switch is set
// under the scene's residuals so that it takes the inner iterations; a third solve with the switch
// left at its default takes the dense solver, ends close to the first and not at its bits, which shows
// the switch is what the setting moved.
bool BAInnerIterationOrderTest()
{
	TestScope scope("BAInnerIterationOrderTest");
	constexpr unsigned NUM_IMAGES = 10;
	constexpr unsigned NUM_TRACKS_PER_TRIPLE = 150; // 8 triples of consecutive cameras: 3600 residuals
	constexpr unsigned MIN_RESIDUALS = 3000;        // the switch to inner iterations, under the scene's residuals
	constexpr unsigned PAIR_INLIERS = 300;
	constexpr REAL ROTATION_NOISE_DEG = 0.2;        // each pose turned this far about each axis, at most
	constexpr REAL CENTER_NOISE = 0.02;             // each centre and each point moved this far per axis, at most
	constexpr double MAX_SOLVER_GAP = 1e-3;         // the dense and the inner-iteration solve end this close, in units

	const auto buildScene = [&](Scene& scene) {
		std::mt19937 rng(20261015);
		BuildResectionArc(scene, NUM_IMAGES);
		for (IIndex i = 0; i + 2 < NUM_IMAGES; ++i)
			AddResectionTracks(scene, {i, i + 1, i + 2}, NUM_TRACKS_PER_TRIPLE, rng);
		for (IIndex i = 0; i < NUM_IMAGES; ++i) {
			if (i + 1 < NUM_IMAGES)
				AddResectionPair(scene, i, i + 1, PAIR_INLIERS);
			if (i + 2 < NUM_IMAGES)
				AddResectionPair(scene, i, i + 2, PAIR_INLIERS);
		}
		TriangulateTracks(scene, false, 4.f, 1.f);
		std::uniform_real_distribution<REAL> unit(-1, 1);
		for (Image& img : scene.images) {
			const RMatrix turn(Point3(unit(rng), unit(rng), unit(rng)) * D2R(ROTATION_NOISE_DEG));
			img.R = turn * img.R;
			img.C += Point3(unit(rng), unit(rng), unit(rng)) * CENTER_NOISE;
		}
		for (Track& track : scene.tracks)
			track.position += Point3(unit(rng), unit(rng), unit(rng)) * CENTER_NOISE;
	};
	BAConfig config;
	config.numThreads = 1; // the solver's own threads sum in the order they finish
	config.minResidualsForSparseSolver = MIN_RESIDUALS;

	Scene first;
	buildScene(first);
	{
		BundleAdjustment ba(first, config);
		TEST_CHECK(ba.Adjust(), "the first bundle adjustment did not solve the scene");
		unsigned numResiduals = 0;
		for (const unsigned numImageResiduals : ba.GetNumReprojResidualsPerImage())
			numResiduals += numImageResiduals;
		TEST_CHECK(numResiduals >= MIN_RESIDUALS, "the scene gave %u reprojection residuals, under the %u from which the solve "
			"refines by inner iterations", numResiduals, MIN_RESIDUALS);
	}

	// the second solve with every small block it allocates placed below the one before, so the
	// solver's records of the blocks lie in the reverse of the order they had in the first
	Scene second;
	buildScene(second);
	size_t pooledBytes;
	{
		const ScopedDescendingHeap descendingHeap;
		TEST_CHECK(BundleAdjustment::Adjust(second, config), "the second bundle adjustment did not solve the scene");
		pooledBytes = descendingHeap.BytesHandedOut();
	}

	unsigned numPosesApart = 0, numPointsApart = 0;
	double maxCenterApart = 0, maxPointApart = 0;
	FOREACH(i, first.images) {
		const Image& a = first.images[i];
		const Image& b = second.images[i];
		if (memcmp(a.R.val, b.R.val, sizeof(a.R.val)) != 0 || memcmp(&a.C, &b.C, sizeof(a.C)) != 0) {
			++numPosesApart;
			maxCenterApart = MAXF(maxCenterApart, (double)norm(a.C - b.C));
		}
	}
	FOREACH(i, first.tracks) {
		const Point3& a = first.tracks[i].position;
		const Point3& b = second.tracks[i].position;
		if (memcmp(&a, &b, sizeof(a)) != 0) {
			++numPointsApart;
			maxPointApart = MAXF(maxPointApart, (double)norm(a - b));
		}
	}
	TEST_CHECK(numPosesApart == 0 && numPointsApart == 0, "solved twice from the same start, the scene ended with %u of %u "
		"poses (centres up to %g apart) and %u of %u points (up to %g apart) not the same",
		numPosesApart, first.images.size(), maxCenterApart, numPointsApart, first.tracks.size(), maxPointApart);

	// the switch left at its default: the dense solver, a different path to the same optimum
	Scene dense;
	buildScene(dense);
	config.minResidualsForSparseSolver = BAConfig().minResidualsForSparseSolver;
	TEST_CHECK(BundleAdjustment::Adjust(dense, config), "the dense bundle adjustment did not solve the scene");
	bool sameBits = true;
	double maxGap = 0;
	FOREACH(i, first.images) {
		sameBits = sameBits && memcmp(&first.images[i].C, &dense.images[i].C, sizeof(first.images[i].C)) == 0;
		maxGap = MAXF(maxGap, (double)norm(first.images[i].C - dense.images[i].C));
	}
	TEST_CHECK(!sameBits, "the solve below the switch ended at the very bits of the one past it: the setting did not move the switch");
	TEST_CHECK(maxGap < MAX_SOLVER_GAP, "the dense solve and the inner-iteration solve end %g apart in a camera centre, over the %g bar",
		maxGap, MAX_SOLVER_GAP);
	return scope.Pass("a bundle adjustment refined by inner iterations ends at the same bits with the first %s of the small blocks it "
		"allocates placed in the reverse order, and within %.2g of the dense solve of the same scene",
		Util::formatBytes(pooledBytes).c_str(), maxGap);
}

// An image whose described observations fix its pose on their own hears its dense ones less: each
// dense residual of an image weighs BAConfig::denseHalfWeightObservations / (that + the image's
// described observations) of the dense weight, so warp samples that share a bias pull an image rich
// in detections far less than a uniform dense weight lets them
bool BADenseWeightPerImageTest()
{
	TD_TIMER_START();
	constexpr IIndex BIASED_IMAGE = 3;       // the image whose dense keypoints are all shifted
	constexpr float DENSE_BIAS = 2.f;        // pixels every dense keypoint of that image is shifted by
	constexpr double MIN_UNIFORM_ERROR = 0.02; // degrees the bias must cost under a uniform dense weight
	constexpr double MAX_ERROR_RATIO = 0.5;  // the per-image weight must at least halve that error
	SceneConfig cfg;
	cfg.numImages = 8;
	cfg.numPoints = 1000;
	double rotationError[2];
	for (const bool perImage : {false, true}) {
		Scene truth, scene;
		GenerateTestScene(truth, cfg, &scene);
		AddDenseObservations(scene);
		Image& img = scene.images[BIASED_IMAGE];
		for (size_t k = img.NumDescribedKeypoints(); k < img.keypoints.size(); ++k)
			img.keypoints[k].pt.x += DENSE_BIAS;
		BAConfig config;
		config.denseObservationWeight = 0.25; // pinned, so both solves weigh a dense residual alike
		config.relativeRotationSigma = config.relativeTranslationSigma = 0.f; // the reprojections alone
		config.denseHalfWeightObservations = perImage ? 300u : 0u;
		config.maxDenseObservationWeight = 0.0; // the scaling alone, held to no ceiling
		if (!BundleAdjustment::Adjust(scene, config)) {
			VERBOSE("BADenseWeightPerImageTest FAILED: the bundle adjustment did not solve the scene (%s dense weight)",
				perImage ? "per-image" : "uniform");
			return false;
		}
		std::vector<Pose3D> gtPoses;
		for (const Image& gt : truth.images)
			gtPoses.push_back(gt);
		double maxCenter;
		WorstPoseError(scene, gtPoses, rotationError[perImage], maxCenter);
	}
	if (rotationError[0] < MIN_UNIFORM_ERROR) {
		VERBOSE("BADenseWeightPerImageTest FAILED: a %.0f-pixel bias on the dense keypoints of image %u costs only "
			"%.4f degrees under a uniform dense weight, so the scene does not pose the problem",
			(double)DENSE_BIAS, BIASED_IMAGE, rotationError[0]);
		return false;
	}
	if (rotationError[1] > MAX_ERROR_RATIO * rotationError[0]) {
		VERBOSE("BADenseWeightPerImageTest FAILED: the per-image dense weight leaves the model %.4f degrees from the "
			"truth against %.4f under a uniform one, more than %.0f%% of it", rotationError[1], rotationError[0],
			100.0 * MAX_ERROR_RATIO);
		return false;
	}
	VERBOSE("BADenseWeightPerImageTest PASSED: a %.0f-pixel bias on the dense keypoints of a detection-rich image "
		"leaves the model %.4f degrees from the truth under a per-image dense weight against %.4f under a uniform one "
		"(%s)", (double)DENSE_BIAS, rotationError[1], rotationError[0], TD_TIMER_GET_FMT().c_str());
	return true;
}

// No image's dense residuals weigh more than BAConfig::maxDenseObservationWeight: warp samples that
// share a bias pull the model at the dense weight the scene measured, and held to the ceiling they
// pull it far less
bool BADenseWeightCeilingTest()
{
	TD_TIMER_START();
	constexpr IIndex BIASED_IMAGE = 3;       // the image whose dense keypoints are all shifted
	constexpr float DENSE_BIAS = 2.f;        // pixels every dense keypoint of that image is shifted by
	constexpr double DENSE_WEIGHT = 0.25;    // the dense weight both solves start from, over the ceiling
	constexpr double CEILING = 0.01;         // the ceiling the second solve holds every image to
	constexpr double MIN_UNCAPPED_ERROR = 0.02; // degrees the bias must cost without a ceiling
	constexpr double MAX_ERROR_RATIO = 0.5;  // the ceiling must at least halve that error
	SceneConfig cfg;
	cfg.numImages = 8;
	cfg.numPoints = 1000;
	double rotationError[2];
	for (const bool capped : {false, true}) {
		Scene truth, scene;
		GenerateTestScene(truth, cfg, &scene);
		AddDenseObservations(scene);
		Image& img = scene.images[BIASED_IMAGE];
		for (size_t k = img.NumDescribedKeypoints(); k < img.keypoints.size(); ++k)
			img.keypoints[k].pt.x += DENSE_BIAS;
		BAConfig config;
		config.denseObservationWeight = DENSE_WEIGHT; // pinned, so both solves start from the same weight
		config.relativeRotationSigma = config.relativeTranslationSigma = 0.f; // the reprojections alone
		config.denseHalfWeightObservations = 0u; // the same weight for every image: the ceiling alone
		config.maxDenseObservationWeight = capped ? CEILING : 0.0;
		if (!BundleAdjustment::Adjust(scene, config)) {
			VERBOSE("BADenseWeightCeilingTest FAILED: the bundle adjustment did not solve the scene (%s)",
				capped ? "with the ceiling" : "without a ceiling");
			return false;
		}
		std::vector<Pose3D> gtPoses;
		for (const Image& gt : truth.images)
			gtPoses.push_back(gt);
		double maxCenter;
		WorstPoseError(scene, gtPoses, rotationError[capped], maxCenter);
	}
	if (rotationError[0] < MIN_UNCAPPED_ERROR) {
		VERBOSE("BADenseWeightCeilingTest FAILED: a %.0f-pixel bias on the dense keypoints of image %u costs only "
			"%.4f degrees at dense weight %g, so the scene does not pose the problem",
			(double)DENSE_BIAS, BIASED_IMAGE, rotationError[0], DENSE_WEIGHT);
		return false;
	}
	if (rotationError[1] > MAX_ERROR_RATIO * rotationError[0]) {
		VERBOSE("BADenseWeightCeilingTest FAILED: held to %g the dense weight leaves the model %.4f degrees from the "
			"truth against %.4f at %g, more than %.0f%% of it", CEILING, rotationError[1], rotationError[0],
			DENSE_WEIGHT, 100.0 * MAX_ERROR_RATIO);
		return false;
	}
	VERBOSE("BADenseWeightCeilingTest PASSED: a %.0f-pixel bias on the dense keypoints of one image leaves the model "
		"%.4f degrees from the truth with the dense weight held to %g against %.4f at %g (%s)", (double)DENSE_BIAS,
		rotationError[1], CEILING, rotationError[0], DENSE_WEIGHT, TD_TIMER_GET_FMT().c_str());
	return true;
}

} // namespace SFM
