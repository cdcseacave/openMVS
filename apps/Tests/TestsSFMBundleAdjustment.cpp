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

namespace {

// Give every track of a generated scene a DENSE twin of each of its observations (at most maxDense twins in all), after
// shifting the described observation along x by describedShift(k) and the twin by denseShift(k) from where the described one
// was, k counting the observations in track order
void DisplaceWithDenseTwins(Scene& scene, const std::function<float(size_t)>& describedShift, const std::function<float(size_t)>& denseShift,
	size_t maxDense = SIZE_MAX)
{
	for (Image& img : scene.images)
		img.CloseDescribedKeypoints();
	size_t k = 0, numAdded = 0;
	for (Track& track : scene.tracks) {
		const size_t numDescribedObs = track.observations.size();
		for (size_t i = 0; i < numDescribedObs; ++i, ++k) {
			const Observation obs = track.observations[i];
			Image& img = scene.images[obs.imageID];
			const float x = img.keypoints[obs.featureID].pt.x;
			img.keypoints[obs.featureID].pt.x = x + describedShift(k);
			if (numAdded >= maxDense)
				continue;
			const cv::KeyPoint dense(x + denseShift(k), img.keypoints[obs.featureID].pt.y, 10.f, -1.f, 0.9f);
			track.observations.emplace_back(obs.imageID, (uint32_t)img.keypoints.size());
			img.keypoints.push_back(dense);
			++numAdded;
		}
		track.numInliers = (uint8_t)MINF((size_t)track.observations.size(), (size_t)255);
	}
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
} // namespace

// The weights a bundle adjustment gives dense observations, against the described ones of the same scene (4 images, 200
// tracks, exact observations the test displaces):
// 1. the sigmas read off the raw residuals are medians: described observations 0.5 px off and dense ones 2.0 px off, one in
//    ten thrown 20 px out and one in ten left almost on the point, where a mean would answer about 2.4 px and a max 20;
// 2. the estimated dense weight is 1/k^2 of the sigma ratio k (1/16 for k = 4), clamped to 1 when the dense population is the
//    more precise, floored to 0.01 when it is 20x coarser, and the configured constant when the scene has no dense keypoint
//    or a dense sample (40) too small to give a sigma;
// 3. the weight an image's dense residuals carry: with every dense keypoint of one image shifted 2 px, the per-image scaling
//    (BAConfig::denseHalfWeightObservations) and the ceiling (maxDenseObservationWeight) each at least halve the rotation
//    error that a uniform dense weight leaves
bool BADenseWeightTest()
{
	TestScope scope("BADenseWeightTest");
	constexpr float describedError = 0.5f, denseError = 2.0f;
	SceneConfig cfg;
	cfg.numImages = 4;
	cfg.numPoints = 200;
	const auto Shift = [](float value) { return [value](size_t) { return value; }; };

	// 1. the sigmas
	{
		constexpr float outlierError = 20.0f, innerError = 0.05f;
		const auto Displace = [](size_t k, float nominal) {
			return k % 10 == 0 ? outlierError : (k % 10 == 1 ? innerError : nominal);
		};
		Scene scene;
		GenerateTestScene(scene, cfg);
		DisplaceWithDenseTwins(scene, [&](size_t k) { return Displace(k, describedError); }, [&](size_t k) { return Displace(k, denseError); });
		double sigmaDescribed = 0, sigmaDense = 0;
		size_t numDescribed = 0, numDense = 0;
		ComputeObservationSigmas(scene, sigmaDescribed, numDescribed, sigmaDense, numDense);
		TEST_CHECK(numDescribed != 0 && numDense == numDescribed, "%u described and %u dense observations", (unsigned)numDescribed, (unsigned)numDense);
		// to a hundredth of a pixel rather than exactly: the implementation takes the UPPER median (errors[size/2]), which on an
		// even-sized population is one particular element, and the tolerance covers the reprojection of the displacement
		TEST_CHECK(ABS(sigmaDescribed - describedError) <= 0.01 && ABS(sigmaDense - denseError) <= 0.01, "sigmas %.4f described / %.4f dense "
			"against %.2f / %.2f -- a mean would answer near %.2f / %.2f and a max %.2f", sigmaDescribed, sigmaDense, describedError, denseError,
			0.8f*describedError + 0.1f*(outlierError + innerError), 0.8f*denseError + 0.1f*(outlierError + innerError), outlierError);
		Scene sparseOnly;
		GenerateTestScene(sparseOnly, cfg);
		ComputeObservationSigmas(sparseOnly, sigmaDescribed, numDescribed, sigmaDense, numDense);
		TEST_CHECK(numDense == 0 && numDescribed != 0, "a scene with no dense keypoints reported %u of them", (unsigned)numDense);
	}

	// 2. the estimated weight
	{
		BAConfig config;
		config.denseObservationWeight = -1.0; // estimate
		Scene scene;
		GenerateTestScene(scene, cfg);
		DisplaceWithDenseTwins(scene, Shift(describedError), Shift(denseError));
		const double weight = EstimateDenseObservationWeight(scene, config);
		TEST_CHECK(ABS(weight - 1.0/16.0) <= 0.005, "weight %.4f against 1/k^2 = %.4f for k = 4", weight, 1.0/16.0);
		// a dense population strictly MORE precise than the described one is not worth MORE than it: shifted down to 0.25 px
		// (by 1.75, not the 1.5 that would leave the two merely equal) k = 0.5 and the unclamped value would be 4.0
		constexpr float upperBoundDenseError = 0.25f;
		for (Image& img : scene.images)
			for (uint32_t k = img.NumDescribedKeypoints(); k < img.keypoints.size(); ++k)
				img.keypoints[k].pt.x -= denseError - upperBoundDenseError;
		const double clamped = EstimateDenseObservationWeight(scene, config);
		TEST_CHECK(ABS(clamped - 1.0) <= 0.005, "a dense population twice as precise as the described one weighs %.4f instead of being "
			"clamped to 1", clamped);
		// a dense population far coarser -- the case the weight exists for, a textureless region where the warp is the only
		// correspondence there is -- is floored to MIN_DENSE_OBSERVATION_WEIGHT (0.01, file-local to BundleAdjustment.cpp) rather
		// than losing almost all its influence: 10 px against 0.5 px gives k = 20 and an unclamped 0.0025
		Scene coarseDense;
		GenerateTestScene(coarseDense, cfg);
		DisplaceWithDenseTwins(coarseDense, Shift(describedError), Shift(10.f));
		const double floored = EstimateDenseObservationWeight(coarseDense, config);
		TEST_CHECK(ABS(floored - 0.01) <= 0.0005, "a dense population 20x coarser than the described one weighs %.4f instead of being "
			"floored to 0.01", floored);
		// a scene with no dense keypoints falls back to the configured constant rather than dividing by a sigma it does not have
		Scene sparseOnly;
		GenerateTestScene(sparseOnly, cfg);
		const double fallback = EstimateDenseObservationWeight(sparseOnly, config);
		TEST_CHECK(fallback == DENSE_OBSERVATION_WEIGHT, "fallback %.4f against the constant %.4f", fallback, DENSE_OBSERVATION_WEIGHT);
		// the fallback's other form, the one a real run reaches: a dense population that EXISTS but is too small to give a
		// sigma (a few images into an incremental reconstruction of a textureless capture), with both populations displaced so
		// that it is the count that refuses the sample and not a zero sigma
		constexpr size_t numFewDense = 40;
		SceneConfig fewDenseCfg;
		fewDenseCfg.numImages = 4;
		fewDenseCfg.numPoints = 300;
		Scene fewDense;
		GenerateTestScene(fewDense, fewDenseCfg);
		DisplaceWithDenseTwins(fewDense, Shift(describedError), Shift(denseError), numFewDense);
		double sigmaFewDescribed = 0, sigmaFewDense = 0;
		size_t numFewDescribedObs = 0, numFewDenseObs = 0;
		ComputeObservationSigmas(fewDense, sigmaFewDescribed, numFewDescribedObs, sigmaFewDense, numFewDenseObs);
		TEST_CHECK(numFewDenseObs == numFewDense && numFewDescribedObs >= 100 && sigmaFewDescribed > 0.0 && sigmaFewDense > 0.0,
			"the small-sample fixture holds %u dense and %u described observations at sigmas %.4f/%.4f -- it must put the dense "
			"population alone under the threshold", (unsigned)numFewDenseObs, (unsigned)numFewDescribedObs, sigmaFewDescribed, sigmaFewDense);
		const double smallSample = EstimateDenseObservationWeight(fewDense, config);
		TEST_CHECK(smallSample == DENSE_OBSERVATION_WEIGHT, "%u dense observations against %u described ones weigh %.4f instead of falling "
			"back to the constant %.4f", (unsigned)numFewDenseObs, (unsigned)numFewDescribedObs, smallSample, DENSE_OBSERVATION_WEIGHT);
	}

	// 3. what an image's dense residuals weigh: the model's worst rotation error (degrees, after the best similarity, PoseErrors) after a solve of 8 images and 1000
	// tracks whose image 3 has every dense keypoint shifted 2 px, the dense weight pinned at 0.25, the reprojections alone
	constexpr IIndex BIASED_IMAGE = 3;
	constexpr float DENSE_BIAS = 2.f;
	constexpr double CEILING = 0.01, MIN_UNIFORM_ERROR = 0.02, MAX_ERROR_RATIO = 0.5;
	const auto BiasedRotationError = [&](unsigned halfWeightObservations, double ceiling, double& rotationError) {
		SceneConfig sceneCfg;
		sceneCfg.numImages = 8;
		sceneCfg.numPoints = 1000;
		Scene truth, scene;
		GenerateTestScene(truth, sceneCfg, &scene);
		AddDenseObservations(scene);
		Image& img = scene.images[BIASED_IMAGE];
		for (size_t k = img.NumDescribedKeypoints(); k < img.keypoints.size(); ++k)
			img.keypoints[k].pt.x += DENSE_BIAS;
		BAConfig config;
		config.denseObservationWeight = 0.25;
		config.relativeRotationSigma = config.relativeTranslationSigma = 0.f;
		config.denseHalfWeightObservations = halfWeightObservations;
		config.maxDenseObservationWeight = ceiling;
		TEST_CHECK(BundleAdjustment::Adjust(scene, config), "the bundle adjustment did not solve the scene (denseHalfWeightObservations %u, "
			"ceiling %g)", halfWeightObservations, ceiling);
		std::vector<Pose3D> gtPoses;
		for (const Image& gt : truth.images)
			gtPoses.push_back(gt);
		rotationError = PoseErrors(scene, gtPoses).rotationMax;
		return true;
	};
	double uniformError, perImageError, ceilingError;
	if (!BiasedRotationError(0u, 0.0, uniformError) || !BiasedRotationError(300u, 0.0, perImageError) || !BiasedRotationError(0u, CEILING, ceilingError))
		return false;
	TEST_CHECK(uniformError >= MIN_UNIFORM_ERROR, "a %.0f-pixel bias on the dense keypoints of image %u costs only %.4f degrees under a "
		"uniform dense weight, so the scene does not pose the problem", (double)DENSE_BIAS, BIASED_IMAGE, uniformError);
	TEST_CHECK(perImageError <= MAX_ERROR_RATIO * uniformError, "the per-image dense weight leaves the model %.4f degrees from the truth "
		"against %.4f under a uniform one, more than %.0f%% of it", perImageError, uniformError, 100.0 * MAX_ERROR_RATIO);
	TEST_CHECK(ceilingError <= MAX_ERROR_RATIO * uniformError, "held to %g the dense weight leaves the model %.4f degrees from the truth "
		"against %.4f at 0.25, more than %.0f%% of it", CEILING, ceilingError, uniformError, 100.0 * MAX_ERROR_RATIO);
	return scope.Pass("the sigmas are medians, the estimated dense weight is clamped and floored and falls back to its constant; a %.0f-pixel "
		"bias on one image's dense keypoints leaves %.4f degrees (uniform weight), %.4f (per-image weight), %.4f (ceiling %g)",
		(double)DENSE_BIAS, uniformError, perImageError, ceilingError, CEILING);
}
/*----------------------------------------------------------------*/

// Pins SetBAIntrinsicFlags (ReconstructionConfig::IntrinsicFlags -> BAConfig) and ResectionConfig::DeriveBAConfigs, the two
// places the reconstruction's intrinsic flags turn into the bundle-adjustment switches that are actually solved
bool BAIntrinsicFlagsTest()
{
	TestScope scope("BAIntrinsicFlagsTest");
	const char* const names[] = {"focal length", "k1 k2", "focal aspect ratio", "principal point", "k3", "tangential", "k4 k5 k6"};
	const auto Read = [](const BAConfig& c) {
		return std::array<bool, 7>{c.refineFocalLength, c.refineRadialDistortion12, c.refineFocalLengthAspectRatio, c.refinePrincipalPoint,
			c.refineRadialDistortion3, c.refineTangentialDistortion, c.refineRadialDistortion456};
	};
	const auto Same = [&](const char* what, const std::array<bool, 7>& got, const std::array<bool, 7>& expected) {
		for (unsigned i = 0; i < got.size(); ++i)
			TEST_CHECK(got[i] == expected[i], "%s: %s is %srefined, expected %srefined", what, names[i], got[i] ? "" : "not ", expected[i] ? "" : "not ");
		return true;
	};
	// INTRINSIC_MAIN: focal length and k1, k2 only; INTRINSIC_MAIN_EXTRA adds k3, the principal point and the tangential terms
	BAConfig main, extra, none;
	SetBAIntrinsicFlags(main, ReconstructionConfig::INTRINSIC_MAIN);
	SetBAIntrinsicFlags(extra, ReconstructionConfig::INTRINSIC_MAIN_EXTRA);
	SetBAIntrinsicFlags(none, ReconstructionConfig::INTRINSIC_NONE);
	if (!Same("INTRINSIC_MAIN", Read(main), {true, true, false, false, false, false, false}) ||
		!Same("INTRINSIC_MAIN_EXTRA", Read(extra), {true, true, false, true, true, true, false}))
		return false;
	TEST_CHECK(!none.IsRefiningIntrinsics(), "INTRINSIC_NONE left an intrinsic refined");
	// ResectionConfig::DeriveBAConfigs, given a reconstruction config that allows every intrinsic: fullBAConfig is restricted
	// to the main set, extendedBAConfig keeps all of it
	ResectionConfig resectionCfg;
	resectionCfg.DeriveBAConfigs(extra);
	if (!Same("fullBAConfig", Read(resectionCfg.fullBAConfig), {true, true, false, false, false, false, false}))
		return false;
	TEST_CHECK(Read(resectionCfg.extendedBAConfig) == Read(extra), "extendedBAConfig dropped an intrinsic the reconstruction allowed");
	return scope.Pass("the intrinsic flags set the switches of the main set, the extended set and none, and the resection's full adjustment keeps the main set");
}
/*----------------------------------------------------------------*/

// Spherical camera full-hemisphere reconstruction: the triangulation + BA pipeline on a spherical scene whose 3D points are
// distributed in ALL directions around the cameras (front, back, sides), so that observations span the full equirectangular
// image including longitudes |theta| > pi/2. This is the regression harness for the S^2 -> R^2 singularity in
// SphericalCamera::Unproject and the pinhole DLT in TriangulateDLT. It also guards against the left-right (X) mirror reported
// on real 360 scenes: the closing blocks pin the absolute equirectangular convention of SphericalCamera::Project against
// hand-reasoned ground truth, and run an O(3) (reflection-allowed) Kabsch handedness check on the reconstructed rig + points.
bool ReconstructSphericalSyntheticTest()
{
	TestScope scope("ReconstructSphericalSyntheticTest");
	// 6 cameras in a small 3D cluster near the origin, all with identity rotation: points on the far side of the origin land
	// at camera-space Z < 0 (the equirectangular "back half"); 80 points 4.5-5.5 away in every direction
	const Scene sceneGT(MakeSphereScene({Point3(-0.6, 0, -0.3), Point3(0.6, 0, -0.3), Point3(-0.6, 0, 0.3), Point3(0.6, 0, 0.3),
		Point3(0, -0.4, 0), Point3(0, 0.4, 0)}, 80, 1337));
	const cv::Size size(sceneGT.images[0].GetSize());

	// the coverage check: the test only catches G1 if some observations fall in the back hemisphere (camera-space Z < 0)
	unsigned totalObs = 0, backObs = 0;
	for (const Track& track : sceneGT.tracks)
		for (const Observation& obs : track.observations) {
			++totalObs;
			backObs += sceneGT.images[obs.imageID].TransformPointW2C(track.position).z < 0;
		}
	TEST_CHECK(backObs >= totalObs / 4, "scene setup produced too few back-hemisphere observations (%u/%u); the test must exercise the "
		"full sphere to expose G1", backObs, totalObs);

	// clear the track positions: a fresh triangulation from the 2D observations + the true poses is the entry point that exercises
	// the pinhole-plane DLT formulation in TriangulateDLT, with a generous bar (20 px on a 2048-wide image is 3.5 degrees) and
	// a low minimum angle (0.5 degrees) so the test isolates G1/G2 failure modes rather than geometric insufficiency
	Scene scene(sceneGT);
	for (Track& track : scene.tracks)
		track.position = Point3(0, 0, 0);
	TriangulateTracks(scene, false, 20.f, 0.5f);
	unsigned recovered = 0;
	for (IIndex t = 0; t < scene.tracks.size(); ++t)
		recovered += norm(scene.tracks[t].position - sceneGT.tracks[t].position) < REAL(0.1);
	const auto [meanReprojErr, meanAng] = ComputeTracksMeanReprojectionError(scene);
	// at least 95% of the tracks within 10 cm of the truth with sub-pixel reprojection error AND near-zero angular error: the
	// angular metric is the critical one -- an observed ray built from the 2D Camera::Unproject form aliases back-hemisphere
	// observations onto the front hemisphere and reads ~90 degrees of error on a perfectly recovered full-sphere scene (G1;
	// Camera::UnprojectNormalized, the singularity-free 3D bearing, is the fix)
	TEST_CHECK(recovered >= (unsigned)(scene.tracks.size() * 0.95), "only %u/%u tracks recovered within 0.1 m (95%% expected)",
		recovered, (unsigned)scene.tracks.size());
	TEST_CHECK(meanReprojErr <= 1.0, "mean reprojection error %.4f px exceeds 1.0 px", meanReprojErr);
	TEST_CHECK(meanAng <= 1.0, "mean angular error %.4f deg exceeds 1.0 deg: the G1 fingerprint (the observed ray of a back-hemisphere feature aliased "
		"onto the front hemisphere)", meanAng);

	// global BA on the triangulated scene converges and preserves the reconstruction
	BAConfig baCfg;
	baCfg.maxIterations = 30;
	baCfg.robustThreshold = 2.f;
	TEST_CHECK(BundleAdjustment::Adjust(scene, baCfg), "BundleAdjustment::Adjust returned false");
	const auto [baMeanErr, baMeanAng] = ComputeTracksMeanReprojectionError(scene);
	TEST_CHECK(baMeanErr <= 1.0 && baMeanAng <= 1.0, "post-BA reprojection error %.4f px and mean angular error %.4f deg, 1.0 expected at most both",
		baMeanErr, baMeanAng);

	// Left-right mirror: the observations above come from SphericalCamera::Project and triangulation inverts the SAME Project, so
	// a globally consistent sign flip of the azimuth convention cancels in the round trip. (A) pins the absolute equirectangular
	// convention, reasoned from first principles (forward +Z at the centre column, camera-right +X on the right half, Y down
	// as for the pinhole camera) and not from Project; expectU/expectV: -1 strictly left/above the centre, +1 right/below, 0 at it
	const Camera& sphCam = *sceneGT.cameras[0];
	const REAL uCenter = REAL(size.width) / 2, vCenter = REAL(size.height) / 2;
	struct ConventionCase { Point3 dirCam; const char* name; int expectU; int expectV; };
	for (const ConventionCase& c : {
		ConventionCase{Point3( 0,  0, 1), "forward +Z", 0, 0}, ConventionCase{Point3( 1,  0, 1), "right +X", +1, 0},
		ConventionCase{Point3(-1,  0, 1), "left -X", -1, 0}, ConventionCase{Point3( 0,  1, 1), "+Y (down)", 0, +1},
		ConventionCase{Point3( 0, -1, 1), "-Y (up)", 0, -1}}) {
		const auto [px, valid] = sphCam.Project(c.dirCam);
		TEST_CHECK(valid, "Project(%s) returned invalid", c.name);
		const auto Side = [](REAL v, REAL centre, int expect) {
			return expect < 0 ? v < centre - 1 : expect > 0 ? v > centre + 1 : ABS(v - centre) < 1;
		};
		TEST_CHECK(Side(px.x, uCenter, c.expectU) && Side(px.y, vCenter, c.expectV), "equirect convention mismatch for %s -> pixel (%.1f, %.1f): the "
			"left-right (X) mirror fingerprint", c.name, px.x, px.y);
	}
	// the exact numeric lock: pure camera-right (+X, z = 0) at u = 3/4 of the width, camera-left (-X) at 1/4
	const auto [pxR, validR] = sphCam.Project(Point3(1, 0, 0));
	const auto [pxL, validL] = sphCam.Project(Point3(-1, 0, 0));
	TEST_CHECK(validR && validL && ABS(pxR.x - REAL(0.75) * size.width) <= 1 && ABS(pxL.x - REAL(0.25) * size.width) <= 1,
		"azimuth mapping is mirrored (+X u=%.1f expected %.1f, -X u=%.1f expected %.1f)", pxR.x, REAL(0.75) * size.width, pxL.x, REAL(0.25) * size.width);

	// (B) the O(3) Kabsch reflection test on the reconstructed rig + points against the truth: the optimal reflection-allowed
	// alignment from SVD(H) has sign(det R) == sign(det H), so det(H) > 0 proves the reconstruction matches under a PROPER
	// rotation (not mirrored); the rig and the points are stacked so that "rig + points mirror together" is one determinant flip
	Point3Arr src, dst;
	for (IIndex t = 0; t < scene.tracks.size(); ++t)
		if (norm(scene.tracks[t].position - sceneGT.tracks[t].position) < REAL(0.5)) {
			src.emplace_back(scene.tracks[t].position);
			dst.emplace_back(sceneGT.tracks[t].position);
		}
	FOREACH(i, scene.images) {
		src.emplace_back(scene.images[i].C);
		dst.emplace_back(sceneGT.images[i].C);
	}
	TEST_CHECK(src.size() >= 4, "too few rig+point correspondences (%u) for the handedness check", (unsigned)src.size());
	Point3 sMean(0, 0, 0), dMean(0, 0, 0);
	for (const Point3& p : src) sMean += p;
	for (const Point3& p : dst) dMean += p;
	sMean /= (REAL)src.size();
	dMean /= (REAL)dst.size();
	REAL H[3][3] = {};
	FOREACH(i, src) {
		const Point3 s = src[i] - sMean, d = dst[i] - dMean;
		const REAL sv[3] = {s.x, s.y, s.z}, dv[3] = {d.x, d.y, d.z};
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				H[r][c] += sv[r] * dv[c];
	}
	const REAL detH = H[0][0]*(H[1][1]*H[2][2] - H[1][2]*H[2][1]) - H[0][1]*(H[1][0]*H[2][2] - H[1][2]*H[2][0]) + H[0][2]*(H[1][0]*H[2][1] - H[1][1]*H[2][0]);
	TEST_CHECK(detH > 0, "the reconstruction aligns to the truth only under a reflection (det %.4g <= 0): left-right mirror", detH);
	return scope.Pass("%u/%u tracks recovered within 0.1 m, %.4f px and %.4f deg before the BA, %.4f px and %.4f deg after, the equirect convention is not "
		"mirrored and the handedness is proper (det %.4g)", recovered, (unsigned)scene.tracks.size(), meanReprojErr, meanAng, baMeanErr, baMeanAng, detH);
}
/*----------------------------------------------------------------*/

// The smoke tests of the bundle adjustment on synthetic scenes (the library's PinholeReprojectionJacobianTest, which checks
// the Jacobian itself, runs beside it): the plain solve, a spherical camera, the refined focal length and radial distortion,
// and Scene::Transform preserving the projections
bool BundleAdjustmentBasicsTest()
{
	TestScope scope("BundleAdjustmentBasicsTest");

	// 1. the basic solve, from perturbed poses, points, intrinsics and keypoints
	{
		Scene sceneGT, scene;
		SceneConfig sceneCfg;
		sceneCfg.perturbOptions = SceneConfig::PERTURB_ALL;
		GenerateTestScene(sceneGT, sceneCfg, &scene);
		BAConfig cfg;
		cfg.maxIterations = 20;
		TEST_CHECK(BundleAdjustment::Adjust(scene, cfg), "the basic bundle adjustment returned false");
		const auto [meanErr, meanAng] = ComputeTracksMeanReprojectionError(scene);
		TEST_CHECK(meanErr <= 1.0, "basic solve: mean reprojection error %.4f px, 1 expected at most", meanErr);
	}

	// 2. a spherical camera, with the angular reprojection error
	{
		Scene sceneGT, scene;
		SceneConfig sceneCfg;
		sceneCfg.cameras[0].type = SceneConfig::SPHERICAL;
		sceneCfg.cameras[0].width = 1024;
		sceneCfg.cameras[0].height = 512;
		sceneCfg.perturbOptions = SceneConfig::PERTURB_ALL;
		GenerateTestScene(sceneGT, sceneCfg, &scene);
		BAConfig cfg;
		cfg.maxIterations = 30;
		cfg.robustThreshold = 1.f; // 1 pixel threshold (auto-converted to angular)
		TEST_CHECK(BundleAdjustment::Adjust(scene, cfg), "the spherical bundle adjustment returned false");
		const auto [meanErr, meanAng] = ComputeTracksMeanReprojectionError(scene);
		TEST_CHECK(meanErr <= 1.0, "spherical camera: mean reprojection error %.4f px, 1 expected at most", meanErr);
	}

	// 3. the focal length refined from 380 to the true 420, poses and points held at the truth
	{
		Scene scene;
		SceneConfig sceneCfg;
		sceneCfg.cameras[0].focal = 420.0;
		sceneCfg.numImages = 4;
		sceneCfg.numPoints = 50;
		GenerateTestScene(scene, sceneCfg);
		PinholeCamera& cam = *static_cast<PinholeCamera*>(scene.cameras[0]); // the keypoints stay at the truth for fx = 420
		cam.fx = cam.fy = 380.0;
		cam.trustIntrinsics = false; // allow BA to refine
		BAConfig cfg;
		cfg.maxIterations = 30;
		cfg.refineFocalLength = true;
		cfg.refinePosesRotation = cfg.refinePosesPosition = false;
		cfg.refinePoints = false;
		cfg.robustThreshold = 2.f;
		TEST_CHECK(BundleAdjustment::Adjust(scene, cfg), "the focal length bundle adjustment returned false");
		TEST_CHECK(ABS(cam.fx - sceneCfg.cameras[0].focal) <= 5.0, "focal length refined to %.2f, the truth %.2f (5 allowed)", cam.fx, sceneCfg.cameras[0].focal);
	}

	// 4. the radial distortion refined from (0.05, 0) to the true (0.1, -0.05), poses and points held at the truth
	{
		Scene scene;
		SceneConfig sceneCfg;
		sceneCfg.cameras[0].k1 = 0.1;
		sceneCfg.cameras[0].k2 = -0.05;
		sceneCfg.numImages = 4;
		sceneCfg.numPoints = 80;
		GenerateTestScene(scene, sceneCfg);
		PinholeCamera& cam = *static_cast<PinholeCamera*>(scene.cameras[0]);
		cam.k1 = 0.05;
		cam.k2 = 0.0;
		cam.trustIntrinsics = false;
		BAConfig cfg;
		cfg.maxIterations = 40;
		cfg.refineRadialDistortion12 = true;
		cfg.refinePosesRotation = cfg.refinePosesPosition = false;
		cfg.refinePoints = false;
		TEST_CHECK(BundleAdjustment::Adjust(scene, cfg), "the radial distortion bundle adjustment returned false");
		TEST_CHECK(ABS(cam.k1 - sceneCfg.cameras[0].k1) <= 0.02 && ABS(cam.k2 - sceneCfg.cameras[0].k2) <= 0.02,
			"distortion refined to k1 %.4f k2 %.4f, the truth %.4f %.4f (0.02 allowed)", cam.k1, cam.k2, sceneCfg.cameras[0].k1, sceneCfg.cameras[0].k2);
	}

	// 5. Scene::Transform preserves the projections: every 8th inlier track's observations project the same after a random similarity
	double meanProjectionError;
	{
		Scene scene;
		std::mt19937 rng(456);
		SceneConfig sceneCfg;
		sceneCfg.numImages = 5;
		sceneCfg.numPoints = 100;
		sceneCfg.poseMode = SceneConfig::RANDOM_POSES;
		GenerateTestScene(scene, sceneCfg);
		struct PointProjection { uint32_t trackIdx, imageIdx; Point2 projection; };
		std::vector<PointProjection> originals;
		FOREACH(trackIdx, scene.tracks) {
			const Track& track = scene.tracks[trackIdx];
			if (!track.IsInlier())
				continue;
			for (const auto& obs : track) {
				const auto [proj, valid] = scene.images[obs.imageID].ProjectPoint(track.position);
				if (valid)
					originals.push_back({trackIdx, obs.imageID, proj});
			}
			trackIdx += 7; // skip some tracks to reduce the number of projections
		}
		TEST_CHECK(!originals.empty(), "no projections could be computed");
		scene.Transform(Transform::Random(rng));
		REAL sumProjectionError = 0;
		for (const PointProjection& o : originals) {
			const auto [newProj, valid] = scene.images[o.imageIdx].ProjectPoint(scene.tracks[o.trackIdx].position);
			TEST_CHECK(valid, "the projection of track %u in image %u is invalid after the transform", o.trackIdx, o.imageIdx);
			sumProjectionError += norm(newProj - o.projection);
		}
		meanProjectionError = sumProjectionError / originals.size();
		TEST_CHECK(meanProjectionError <= 0.01, "Scene::Transform moved the projections by %.6f px on average, 0.01 expected at most", meanProjectionError);
	}
	return scope.Pass("the basic and the spherical solves fit, the focal length and the radial distortion are refined to the truth, Scene::Transform "
		"moves the projections by %.2g px", meanProjectionError);
}
/*----------------------------------------------------------------*/

namespace {
// The pose uncertainty a bundle adjustment records, one entry per image: every entry valid with finite variances and a finite,
// Cauchy-Schwarz-consistent position covariance, and the number of images that hold no variance at all (the gauge datum a
// solve with no absolute prior fixes) the expected one
bool CheckPoseUncertainty(const TestScope& scope, const PoseUncertaintyArr& uncertainty, size_t numImages, unsigned expectedDatum)
{
	TEST_CHECK(uncertainty.size() == numImages, "pose uncertainty not computed (%u/%u images)", (unsigned)uncertainty.size(), (unsigned)numImages);
	unsigned numDatum = 0;
	FOREACH(i, uncertainty) {
		const PoseUncertainty& u = uncertainty[i];
		TEST_CHECK(u.IsValid(), "pose uncertainty missing for image %u", i);
		const float rotVar = u.MaxRotationVariance(), posVar = u.MaxPositionVariance();
		TEST_CHECK(ISFINITE(rotVar) && ISFINITE(posVar) && rotVar >= 0.f && posVar >= 0.f, "invalid pose uncertainty for image %u (rotVar %g, posVar %g)", i, rotVar, posVar);
		TEST_CHECK(ISFINITE(u.posCov.x) && ISFINITE(u.posCov.y) && ISFINITE(u.posCov.z), "invalid position covariance for image %u", i);
		constexpr float tol = 1.01f;
		TEST_CHECK(ABS(u.posCov.x) <= SQRT(u.posVar.x * u.posVar.y) * tol + FLT_EPSILON && ABS(u.posCov.y) <= SQRT(u.posVar.x * u.posVar.z) * tol + FLT_EPSILON &&
			ABS(u.posCov.z) <= SQRT(u.posVar.y * u.posVar.z) * tol + FLT_EPSILON, "position covariance not positive semi-definite for image %u", i);
		numDatum += rotVar == 0.f && posVar == 0.f;
	}
	TEST_CHECK(numDatum == expectedDatum, "%u gauge-reference images, %u expected", numDatum, expectedDatum);
	return true;
}
} // namespace

// The pose uncertainty a bundle adjustment records:
// 1. GPS priors on a geo-aligned scene anchor the gauge, so BA fixes no pose and ComputePoseUncertainty returns absolute
//    (datum-free) covariances, one view without EXIF accuracy tags falls back to default accuracies (no non-finite residual),
//    and the fast Schur + selected-inverse covariance agrees with Ceres' own (slow, dense) estimator to 5%;
// 2. without a prior the gauge reference is the one image of zero variance, and the quality report round-trips: the
//    uncertainty recorded on the scene, ExportPoseUncertaintyCSV (one row per image keyed by the SFM image ID, one all-zero
//    datum row), ExportMVS -> MVS::Scene::Load keeping the non-contiguous image IDs the report is correlated by,
//    Scene::Transform mapping the position covariance, and the .sfm serialization keeping the record
bool PoseUncertaintyTest()
{
	TestScope scope("PoseUncertaintyTest");

	// 1. GPS priors
	double meanPosError, maxPosRelErr = 0, maxRotRelErr = 0;
	unsigned numChecked = 0;
	{
		Scene sceneGT, scene;
		SceneConfig sceneCfg;
		sceneCfg.numImages = 6;
		sceneCfg.numPoints = 100;
		sceneCfg.poseMode = SceneConfig::RANDOM_POSES;
		sceneCfg.generateGPS = true; // synthesizes GPS metadata and aligns the scene to ENU
		sceneCfg.perturbOptions = SceneConfig::PERTURB_POSES;
		GenerateTestScene(sceneGT, sceneCfg, &scene);
		TEST_CHECK(scene.status.nState.isSet(Scene::Status::STATE::GEO_ALIGN), "the generated scene is not geo-aligned");
		View::Metadata& meta = static_cast<View&>(scene.images[1]).metadata; // one view without accuracy tags
		meta.positionAccuracy = 0.f;
		meta.positionAccuracyZ = 0.f;
		BAConfig cfg;
		cfg.gpsPositionWeight = 1.0;
		cfg.gpsPositionWeightZ = 1.0;
		cfg.gpsWeightScaleFactor = 0.1;
		BundleAdjustment ba(scene, cfg);
		TEST_CHECK(ba.Adjust(), "the GPS-prior bundle adjustment returned false");
		// the poses stay commensurate with the GPS accuracy
		meanPosError = 0;
		FOREACH(i, scene.images)
			meanPosError += norm(scene.images[i].C - sceneGT.images[i].C);
		meanPosError /= scene.images.size();
		TEST_CHECK(meanPosError <= 0.2, "mean position error %.3f m > 0.2 m", meanPosError);
		// the GPS priors anchor every pose: no datum
		const PoseUncertaintyArr uncertainty = ba.ComputePoseUncertainty();
		if (!CheckPoseUncertainty(scope, uncertainty, scene.images.size(), 0))
			return false;
		// the cross-check against Ceres' covariance on the same solved problem: the priors make the system full rank, so both
		// compute the same marginal pose covariance up to numerical error (5%, the two use different linear-algebra paths)
		const PoseUncertaintyArr reference = ba.ComputePoseUncertaintyCeres();
		TEST_CHECK(reference.size() == uncertainty.size(), "Ceres reference covariance not computed (%u/%u)", (unsigned)reference.size(), (unsigned)uncertainty.size());
		const auto frob = [](const Matrix3x3f& m) {
			float s = 0.f; for (int k = 0; k < 9; ++k) s += m.val[k]*m.val[k]; return SQRT(s);
		};
		const auto frobDiff = [](const Matrix3x3f& A, const Matrix3x3f& B) {
			float s = 0.f; for (int k = 0; k < 9; ++k) { const float d = A.val[k]-B.val[k]; s += d*d; } return SQRT(s);
		};
		const auto relErr = [](float a, float b) { return ABS(a - b) / MAXF(ABS(b), 1e-12f); };
		FOREACH(i, uncertainty) {
			const PoseUncertainty &a = uncertainty[i], &b = reference[i];
			if (!a.IsValid() || !b.IsValid())
				continue;
			const Matrix3x3f Ca = a.GetPositionCovariance(), Cb = b.GetPositionCovariance();
			maxPosRelErr = MAXF(maxPosRelErr, (double)(frobDiff(Ca, Cb) / MAXF(frob(Cb), 1e-12f)));
			maxRotRelErr = MAXF(MAXF(maxRotRelErr, (double)relErr(a.rotVar.x, b.rotVar.x)), MAXF((double)relErr(a.rotVar.y, b.rotVar.y), (double)relErr(a.rotVar.z, b.rotVar.z)));
			++numChecked;
		}
		TEST_CHECK(numChecked > 0, "no images to cross-check against Ceres");
		TEST_CHECK(maxPosRelErr <= 0.05 && maxRotRelErr <= 0.05, "the covariance disagrees with the Ceres reference (max relative error: "
			"position %.3g, rotation %.3g, 0.05 allowed, over %u images)", maxPosRelErr, maxRotRelErr, numChecked);
	}

	// 2. no prior, the report round trip
	Scene sceneGT, scene;
	SceneConfig sceneCfg;
	sceneCfg.perturbOptions = SceneConfig::PERTURB_ALL;
	GenerateTestScene(sceneGT, sceneCfg, &scene);
	// non-contiguous IDs prove the CSV/interface correlation is ID-based, not index-based
	FOREACH(i, scene.images)
		scene.images[i].ID = 10 + i * 3;
	scene.status.nState.set(Scene::Status::STATE::CALIBRATED);
	const ScopedTempDir tmpDir(_T("PoseUncertaintyTest"));
	TEST_CHECK(tmpDir.IsValid(), "no temporary directory");
	const String mvsPath = tmpDir(_T("scene.mvs"));
	TEST_CHECK(ExportMVS(mvsPath, scene, {}), "ExportMVS returned false");
	// record the pose uncertainty on the scene from the (last) bundle adjustment, as Scene::Reconstruct does when
	// ReconstructionConfig::estimatePoseUncertainty is set
	{
		BAConfig cfg;
		BundleAdjustment ba(scene, cfg);
		TEST_CHECK(ba.Adjust(), "the bundle adjustment returned false");
		scene.poseUncertainty = ba.ComputePoseUncertainty();
	}
	if (!CheckPoseUncertainty(scope, scene.poseUncertainty, scene.images.size(), 1))
		return false;
	const String csvPath = tmpDir(_T("quality.csv"));
	const unsigned numValid = ExportPoseUncertaintyCSV(csvPath, scene);
	TEST_CHECK(numValid == scene.images.size(), "exported %u valid rows, expected %u", numValid, (unsigned)scene.images.size());
	// re-read the CSV: one data row per image, IDs matching the assigned ones, exactly one datum row with all-zero sigmas
	std::ifstream is(csvPath);
	TEST_CHECK(is.is_open(), "cannot re-open '%s'", csvPath.c_str());
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
		TEST_CHECK(fields.size() >= 16, "CSV row with %u fields, expected 16", (unsigned)fields.size());
		++numRows;
		csvIDs.insert(id);
		if (fields[3] != "0") {
			++numDatum;
			for (int f = 4; f <= 12; ++f)
				TEST_CHECK(std::atof(fields[f].c_str()) == 0.0, "non-zero sigma on the datum row (field %d)", f);
		}
	}
	TEST_CHECK(numRows == scene.images.size() && numDatum == 1, "%u rows (%u expected), %u datum rows (1 expected)", numRows, (unsigned)scene.images.size(), numDatum);
	FOREACH(i, scene.images)
		TEST_CHECK(csvIDs.count(scene.images[i].ID) != 0, "image ID %u missing from the CSV", scene.images[i].ID);
	// the exported .mvs preserves the SFM image IDs (the report correlation key), and its vertex views stay in range
	MVS::Scene mvsScene(1);
	TEST_CHECK(mvsScene.Load(mvsPath) != MVS::Scene::SCENE_NA, "MVS::Scene::Load returned SCENE_NA");
	TEST_CHECK(mvsScene.images.size() == scene.images.size(), "%u MVS images, expected %u", (unsigned)mvsScene.images.size(), (unsigned)scene.images.size());
	FOREACH(i, mvsScene.images)
		TEST_CHECK(mvsScene.images[i].ID == scene.images[i].ID, "MVS image %u has ID %u, expected %u", i, mvsScene.images[i].ID, scene.images[i].ID);
	for (const MVS::PointCloud::ViewArr& views : mvsScene.pointcloud.pointViews)
		for (const MVS::PointCloud::View view : views)
			TEST_CHECK(view < mvsScene.images.size(), "vertex view %u out of range", view);
	// Scene::Transform maps the recorded position covariance as scale^2 * R * Cov * R^T and leaves the rotation variance untouched
	IIndex idxCheck = NO_ID;
	FOREACH(i, scene.poseUncertainty)
		if (scene.poseUncertainty[i].IsValid() && scene.poseUncertainty[i].MaxPositionVariance() > 0.f) {
			idxCheck = i;
			break;
		}
	TEST_CHECK(idxCheck != NO_ID, "no non-datum uncertainty entry to check");
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
	const auto isNear = [](float a, float b) { return ABS(a - b) <= 1e-3f * MAXF(MAXF(ABS(a), ABS(b)), 1e-12f); };
	TEST_CHECK(isNear((float)covT(0,0), after.posVar.x) && isNear((float)covT(1,1), after.posVar.y) && isNear((float)covT(2,2), after.posVar.z) &&
		isNear((float)covT(0,1), after.posCov.x) && isNear((float)covT(0,2), after.posCov.y) && isNear((float)covT(1,2), after.posCov.z) &&
		after.rotVar == before.rotVar, "the transformed covariance does not match scale^2 R Cov R^T");
	// the serialization round trip preserves the recorded uncertainty
	const String sfmPath = tmpDir(_T("scene.sfm"));
	TEST_CHECK(scene.Save(sfmPath, ARCHIVE_BINARY), "the scene save failed");
	Scene scene2;
	TEST_CHECK(scene2.Load(sfmPath) && scene2.poseUncertainty.size() == scene.poseUncertainty.size(), "the scene load lost the pose uncertainty");
	FOREACH(i, scene.poseUncertainty) {
		const PoseUncertainty &a = scene.poseUncertainty[i], &b = scene2.poseUncertainty[i];
		TEST_CHECK(a.rotVar == b.rotVar && a.posVar == b.posVar && a.posCov == b.posCov, "serialized uncertainty mismatch at image %u", i);
	}
	return scope.Pass("GPS priors give datum-free covariances within %.3g (position) and %.3g (rotation) of Ceres' over %u images (mean position error %.3f m); "
		"the quality report of %u images round-trips through the CSV, the .mvs, Scene::Transform and the .sfm", maxPosRelErr, maxRotRelErr, numChecked,
		meanPosError, (unsigned)scene.images.size());
}
/*----------------------------------------------------------------*/

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
// And a full adjustment stopped by its solve cap ends as one stopped for want of change: with fullBARepeats 0 (one solve, the
// cap reached at once) and 20 of 200 tracks left without a point (inlier prefix 0, position moved away), every one seen at its
// exact projection by the five cameras, every one holds a point again after AdjustRepeated, triangulated again at the bar.
bool RepeatedFullAdjustmentTest()
{
	TestScope scope("RepeatedFullAdjustmentTest");

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
		ArcScene arc(5, 20261015);
		arc.Tracks({0, 1, 2, 3, 4}, NUM_TRACKS);
		Scene& scene = arc.scene;
		Image& img = scene.images[OFF];
		img.R = offset * arc.truth[OFF].R;
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
		out.rotationError = RotationErrorDeg(img.R, arc.truth[OFF].R);
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
	TEST_CHECK(config.fullBARepeats != 0 && turned.minError > config.maxReprojError && turned.maxError < inputBar &&
		rolled.withinInput >= NUM_TRACKS / 4 && rolled.withinInput <= NUM_TRACKS * 9 / 10, "premise not met: %u repeats by default, the turned "
		"camera's observations are %.2f-%.2f px off (all between %g and %g expected), %u of the rolled one's (%.2f-%.2f px off) are within %g px (%u-%u expected)",
		config.fullBARepeats, turned.minError, turned.maxError, config.maxReprojError, inputBar, rolled.withinInput, rolled.minError, rolled.maxError,
		inputBar, NUM_TRACKS / 4, NUM_TRACKS * 9 / 10);
	// a single solve of the prefix does not move the turned camera, and none of its observations comes back
	TEST_CHECK(turnedPrefix.rotationError >= 0.9*REPEATED_TEST_TURN_DEG && turnedPrefix.prefixObservations == 0, "a single solve of the inlier prefix "
		"leaves the turned camera %.4f deg off with %u observations in the prefix; expected it left %g deg off with none", turnedPrefix.rotationError,
		turnedPrefix.prefixObservations, REPEATED_TEST_TURN_DEG);
	for (const auto& [name, out, solves] : {std::make_tuple("turned", turned, 1u), std::make_tuple("rolled", rolled, 2u)})
		TEST_CHECK(out.rotationError <= MAX_RECOVERED_ROTATION_DEG && out.prefixObservations == NUM_TRACKS && out.solves == solves, "the %s camera ends "
			"%.4f deg off with %u of its %u observations in the prefix after %u solves; expected it within %g deg with all of them after %u", name,
			out.rotationError, out.prefixObservations, NUM_TRACKS, out.solves, MAX_RECOVERED_ROTATION_DEG, solves);
	// solves of one iteration: repeated, the camera comes back; with no repeats, one solve, the camera left where the
	// 12 px input's single solve leaves it
	TEST_CHECK(shortRepeated.rotationError <= MAX_RECOVERED_ROTATION_DEG && shortRepeated.solves >= 2, "with solves of one iteration the repeated "
		"adjustment leaves the camera %.4f deg off after %u solves; expected it within %g deg after two or more", shortRepeated.rotationError,
		shortRepeated.solves, MAX_RECOVERED_ROTATION_DEG);
	TEST_CHECK(shortOnce.solves == 1 && shortOnce.rotationError >= MIN_SHORT_ROTATION_DEG && RotationErrorDeg(shortOnce.R, shortInput.R) <= 1e-9,
		"with no repeats and solves of one iteration the adjustment ran %u solves and left the camera %.4f deg off, %.2e deg from where one solve of the "
		"12 px input leaves it; expected one solve, at least %g deg off, at that same pose", shortOnce.solves, shortOnce.rotationError,
		RotationErrorDeg(shortOnce.R, shortInput.R), MIN_SHORT_ROTATION_DEG);

	// the cap: one solve, and the tracks left without a point triangulated again
	constexpr unsigned CAPPED_TRIANGULATION_TRACKS = 20;
	ArcScene capped(5, 20261111);
	capped.Tracks({0, 1, 2, 3, 4}, 200);
	ResectionConfig noRepeats;
	noRepeats.fullBARepeats = 0;
	for (unsigned t = 0; t < CAPPED_TRIANGULATION_TRACKS; ++t) {
		capped.scene.tracks[t].position += Point3(10, 10, 10);
		capped.scene.tracks[t].numInliers = 0;
	}
	const unsigned cappedSolves = AdjustRepeated(capped.scene, noRepeats.fullBAConfig, noRepeats);
	unsigned numBack = 0;
	for (unsigned t = 0; t < CAPPED_TRIANGULATION_TRACKS; ++t)
		numBack += capped.scene.tracks[t].IsInlier();
	TEST_CHECK(cappedSolves == 1 && numBack == CAPPED_TRIANGULATION_TRACKS, "%u solves (1 expected, the cap), %u of the %u tracks left without a point "
		"hold one after the adjustment (all expected)", cappedSolves, numBack, CAPPED_TRIANGULATION_TRACKS);
	return scope.Pass("a camera turned %g deg (observations %.2f-%.2f px off) stays %.4f deg off through a single solve of the inlier prefix and ends %.4f deg off "
		"after %u solve of the repeated adjustment; rolled %g deg (%u of %u observations within %g px) it ends %.4f deg off after %u solves; rolled %g deg with "
		"solves of one iteration it ends %.4f deg off after %u solves, and %.4f deg off after the single solve of no repeats; a solve stopped by its cap "
		"triangulates the %u pointless tracks again", REPEATED_TEST_TURN_DEG, turned.minError, turned.maxError, turnedPrefix.rotationError, turned.rotationError,
		turned.solves, REPEATED_TEST_ROLL_DEG, rolled.withinInput, NUM_TRACKS, inputBar, rolled.rotationError, rolled.solves, REPEATED_TEST_SHORT_ROLL_DEG,
		shortRepeated.rotationError, shortRepeated.solves, shortOnce.rotationError, numBack);
}
/*----------------------------------------------------------------*/

// The track filter and the triangulation on the arc:
// 1. a dense observation is held to its own multiple of the reprojection bar. Seven posed cameras share 30 exact tracks, plus two
//    tracks whose observation in the last image is 7 pixels off: a described keypoint carries one, a dense keypoint the other.
//    The displacement is vertical, across the plane of the arc, the direction the other six views constrain tightly: a
//    least-squares triangulation then leaves most of it in the displaced view (about six sevenths, between the 4-pixel bar
//    and twice it), where a displacement along the arc would be traded for depth and spread over every view;
// 2. a track with one posed view, the star's reference at the world origin, is unjudged. The star puts its reference view exactly
//    at the origin, and a track whose only posed view is that reference was never triangulated, so it still holds the position a
//    scene saved before the default became an improbable point stores, (0,0,0): the point the reference camera sits on, where
//    the angle to the observed ray is 0/0 (NaN), which the gate's comparison lets through. 30 exact tracks over two other posed
//    images show the filter still judges the tracks that can have a position;
// 3. a track of two posed views never triangulated holds the improbable default position (far, finite, rays that meet at no
//    angle), not a point a posed camera can sit on, so that the filters that run before a triangulation reject it.
bool FilterTracksTest()
{
	TestScope scope("FilterTracksTest");

	// 1. the dense bar
	constexpr unsigned numViews = 7;
	constexpr float displacement = 7.f;
	{
		ArcScene arc(numViews, 20260914);
		Scene& scene = arc.scene;
		const IIndexArr allViews{0, 1, 2, 3, 4, 5, 6};
		arc.Tracks(allViews, 30);
		TEST_CHECK(arc.TrackAt(Point3(0.2, 0.1, -0.3), allViews), "the described track does not project inside every image");
		const size_t idxDescribed = scene.tracks.size() - 1;
		Image& lastImage = scene.images[numViews - 1];
		lastImage.CloseDescribedKeypoints(); // every keypoint the last image gains from here on is dense
		TEST_CHECK(arc.TrackAt(Point3(-0.3, 0.2, 0.1), allViews), "the dense track does not project inside every image");
		const size_t idxDense = scene.tracks.size() - 1;
		arc.Displace(numViews - 1, (unsigned)idxDescribed, 1, Point2f(0.f, displacement));
		arc.Displace(numViews - 1, (unsigned)idxDense, 1, Point2f(0.f, displacement));
		TEST_CHECK(!lastImage.IsDenseKeypoint(scene.tracks[idxDescribed].observations.back().featureID) &&
			lastImage.IsDenseKeypoint(scene.tracks[idxDense].observations.back().featureID), "the two displaced keypoints are not one described and one dense");
		const auto Check = [&](const char* stage, unsigned expectedDescribed, unsigned expectedDense) {
			TEST_CHECK(scene.tracks[idxDescribed].numInliers == expectedDescribed && scene.tracks[idxDense].numInliers == expectedDense, "%s kept %u/%u "
				"observations of the described track and %u/%u of the dense one, expected %u and %u", stage, scene.tracks[idxDescribed].numInliers, numViews,
				scene.tracks[idxDense].numInliers, numViews, expectedDescribed, expectedDense);
			return true;
		};
		// at a 4-pixel bar with the dense observations held to twice that, the filter keeps the dense observation and drops the
		// described one; held to the same bar, it drops both; the triangulation reads the two bars the same way
		FilterTracks(scene, 4.f, 1.f, 0.f, 0.f, 2.f);
		if (!Check("the filter, with dense observations held to twice the 4-pixel bar,", numViews - 1, numViews))
			return false;
		FilterTracks(scene, 4.f, 1.f, 0.f, 0.f, 1.f);
		if (!Check("the filter, at a 4-pixel bar for both kinds,", numViews - 1, numViews - 1))
			return false;
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
	}

	// 2. the track with one posed view, at the origin
	size_t numExact;
	{
		ArcScene arc(4, 20261027);
		Scene& scene = arc.scene;
		// image 0 is the reference at the origin, image 3 has no pose, and the positionless track sees exactly those two
		static_cast<Pose3D&>(scene.images[0]) = Pose3D::Identity();
		scene.images[3].InvalidatePose();
		arc.Tracks({1, 2}, 30);
		numExact = scene.tracks.size();
		Track positionless;
		positionless.position = Point3(0, 0, 0); // what a scene saved before the default was an improbable point stores
		for (const IIndex imageID : {IIndex(0), IIndex(3)}) {
			Image& img = scene.images[imageID];
			positionless.observations.emplace_back(imageID, (uint32_t)img.keypoints.size());
			img.keypoints.emplace_back(cv::Point2f(100.f, 150.f), 1.f);
		}
		scene.tracks.emplace_back(std::move(positionless));
		TEST_CHECK(scene.images[0].IsValid() && !scene.images[3].IsValid() && scene.images[0].C == CMatrix::ZERO && scene.tracks.back().position == Point3(0, 0, 0),
			"premise not met: the reference at the origin, the unposed image and the track at the default position");
		FilterTracks(scene, 4.f, 1.f, 0.f, 0.f, 1.f);
		TEST_CHECK(scene.tracks.back().numInliers == 0, "the filter judged the track with one posed view at its default position and kept %u of its %u "
			"observations, expected none", scene.tracks.back().numInliers, scene.tracks.back().GetNumObservations());
		for (size_t i = 0; i < numExact; ++i)
			TEST_CHECK(scene.tracks[i].numInliers == 2, "the exact track %zu of the two posed views kept %u observations, expected 2", i, scene.tracks[i].numInliers);
		TEST_CHECK(scene.status.nTracks == numExact, "the filter counted %u inlier tracks, expected the %zu exact ones", scene.status.nTracks, numExact);
	}

	// 3. the never-triangulated track
	{
		ArcScene arc(2, 20261027);
		Scene& scene = arc.scene;
		static_cast<Pose3D&>(scene.images[0]) = Pose3D::Identity();
		Track& untriangulated = scene.tracks.emplace_back(); // as BuildTracks makes its tracks
		for (const IIndex imageID : {IIndex(0), IIndex(1)}) {
			Image& img = scene.images[imageID];
			untriangulated.observations.emplace_back(imageID, (uint32_t)img.keypoints.size());
			img.keypoints.emplace_back(cv::Point2f(100.f, 150.f), 1.f);
		}
		TEST_CHECK(scene.images[0].IsValid() && scene.images[1].IsValid() && scene.images[0].C == CMatrix::ZERO && scene.tracks.back().GetNumPosedObservations(scene.images) == 2,
			"premise not met: the reference at the origin, a second posed image and a track of both");
		FilterTracks(scene, 4.f, 1.f, 0.f, 0.f, 1.f);
		const Track& track = scene.tracks.back();
		TEST_CHECK(track.numInliers == 0 && scene.status.nTracks == 0, "the filter kept %u of the %u observations of a track that was never triangulated "
			"(%u inlier tracks), expected none", track.numInliers, track.GetNumObservations(), scene.status.nTracks);
		const Point3 untriangulatedPosition(Track::UNTRIANGULATED_COORD);
		TEST_CHECK(Track().position == untriangulatedPosition && track.position == untriangulatedPosition && Track::UNTRIANGULATED_COORD >= 1e12 &&
			ISFINITE(Track::UNTRIANGULATED_COORD * Track::UNTRIANGULATED_COORD), "a track made without a position holds (%g,%g,%g) and a default Track (%g,%g,%g), "
			"expected the same far finite coordinate %g in all three", track.position.x, track.position.y, track.position.z, Track().position.x,
			Track().position.y, Track().position.z, Track::UNTRIANGULATED_COORD);
	}
	return scope.Pass("the filter and the triangulation hold a dense observation to its own multiple of the reprojection bar; a track with one posed view "
		"at the origin and a track never triangulated are left unjudged and not inliers, while the %zu tracks of two posed views are still judged", numExact);
}
/*----------------------------------------------------------------*/
// Triangulation, its failures and a geometry that once defeated it.
// 1. A failure leaves the track with no inliers (IsInlier() is what the adjustments read, and the caller counts the track as
//    failed). Two posed views of the arc (ArcScene) see one exact track 100 units beyond the middle of their centres, about
//    half a degree of parallax, under the 1.5-degree minimum: at 0.1 degrees it triangulates with both views (the premise),
//    at 1.5 TriangulateSkewLLS, TriangulateDLT and TriangulateTracks return 0 and leave no inlier, and the next TriangulateTracks
//    of the tracks without inliers, at 0.1 degrees, takes it up again. Three views of one point whose second and third keypoints
//    are 10 px off in opposite directions across the plane of the arc (the direction every view constrains alike) leave the
//    first view the only one within the 4-px bar (measured: 1.14 px in TriangulateSkewLLS and 2.12 in TriangulateDLT, the other
//    two over 10): with one inlier enough both count it (the premise), with the default two both fail and leave none, and so
//    does a track coming in with inliers counted earlier and one posed view left, turned down before anything is counted.
// 2. TriangulateSkewLLS on panoramas whose rays lie across the optical axis: three 4096x2048 panoramas one metre apart
//    (MakeSphereScene, no points), a point 10 m away in the plane of their centres has dz = 0 in every bearing, where the first
//    two rows of [d]x R, the two the system once kept, are parallel and fix only the depth along the optical axis; the three
//    rows determine it. Exact keypoints at dz = 0 triangulate to the point within 5 mm with every view, and with 1 px of noise
//    the median error at dz = 0 and 0.05 stays under 0.2 m (the two rows: about 10 m and 0.4 m, the three: about 0.09 m).
bool TriangulationTest()
{
	TestScope scope("TriangulationTest");
	constexpr float minAngle = 1.5f, looseAngle = 0.1f;
	// a copy of the scene's first track, its triangulation undone
	const auto Fresh = [](const Scene& scene) {
		Track track(scene.tracks[0]);
		track.position = Point3(0, 0, 0);
		track.numInliers = 0;
		return track;
	};

	// 1. the angle bar
	{
		ArcScene arc(2, 1);
		Scene& scene = arc.scene;
		const Point3 mid = (scene.images[0].C + scene.images[1].C) * 0.5;
		TEST_CHECK(arc.TrackAt(mid * (1 - 100 / norm(mid)), {0, 1}), "the track does not project inside both images");
		{
			Track track = Fresh(scene);
			const unsigned returned = TriangulateSkewLLS(track, scene.images, 4.f, looseAngle);
			const double angle = R2D(track.ComputeMaxAngleBetweenRays(scene.images));
			TEST_CHECK(returned == 2 && track.IsInlier() && angle >= 0.25 && angle <= 1.0, "premise not met: at %g degrees the track "
				"triangulates with %u inliers (%s) and %.3f degrees of parallax, expected 2 and a parallax between 0.25 and 1", looseAngle,
				returned, track.IsInlier() ? "an inlier" : "not an inlier", angle);
		}
		for (const bool dlt : {false, true}) {
			Track track = Fresh(scene);
			const unsigned returned = dlt ? TriangulateDLT(track, scene.images, 4.f, minAngle) : TriangulateSkewLLS(track, scene.images, 4.f, minAngle);
			TEST_CHECK(returned == 0 && !track.IsInlier(), "%s below the minimum angle returned %u, leaving the track %s, expected 0 and "
				"no inliers", dlt ? "TriangulateDLT" : "TriangulateSkewLLS", returned, track.IsInlier() ? "an inlier" : "not an inlier");
		}
		Track& track = scene.tracks[0];
		track.position = Point3(0, 0, 0);
		track.numInliers = 0;
		const unsigned returned = TriangulateTracks(scene, false, 4.f, minAngle);
		TEST_CHECK(returned == 0 && !track.IsInlier() && scene.status.nTracks == 0, "TriangulateTracks below the minimum angle returned "
			"%u, leaving the track %s, expected 0 and no inliers", returned, track.IsInlier() ? "an inlier" : "not an inlier");
		const unsigned retried = TriangulateTracks(scene, true, 4.f, looseAngle);
		TEST_CHECK(retried == 1 && track.IsInlier(), "the next TriangulateTracks of the tracks without inliers, at %g degrees, "
			"triangulated %u tracks (the track is %s), expected 1 and an inlier", looseAngle, retried, track.IsInlier() ? "an inlier" : "not an inlier");
	}

	// 1. the inlier count
	{
		ArcScene arc(3, 1);
		Scene& fit = arc.scene;
		TEST_CHECK(arc.TrackAt(Point3(0.2, 0.1, -0.3), {0, 1, 2}), "the track of three views does not project inside every image");
		arc.Displace(1, 0, 1, Point2f(0.f, 10.f));
		arc.Displace(2, 0, 1, Point2f(0.f, -10.f));
		{
			Track skew = Fresh(fit), dlt = Fresh(fit);
			const unsigned returnedSkew = TriangulateSkewLLS(skew, fit.images, 4.f, 0.f, 1);
			const unsigned returnedDLT = TriangulateDLT(dlt, fit.images, 4.f, 0.f, 1);
			TEST_CHECK(returnedSkew == 1 && returnedDLT == 1, "premise not met: with one inlier enough and no angle bar TriangulateSkewLLS "
				"counts %u inliers and TriangulateDLT %u, expected 1 and 1", returnedSkew, returnedDLT);
		}
		for (const bool dlt : {false, true}) {
			Track track = Fresh(fit);
			const unsigned returned = dlt ? TriangulateDLT(track, fit.images, 4.f, 0.f) : TriangulateSkewLLS(track, fit.images, 4.f, 0.f);
			TEST_CHECK(returned == 0 && track.GetNumInliers() == 0, "%s with one inlier returned %u, leaving %u inliers, expected 0 and none",
				dlt ? "TriangulateDLT" : "TriangulateSkewLLS", returned, track.GetNumInliers());
		}
		fit.images[1].InvalidatePose();
		fit.images[2].InvalidatePose();
		Track track = Fresh(fit);
		track.numInliers = 2;
		const unsigned returned = TriangulateSkewLLS(track, fit.images, 4.f, minAngle);
		TEST_CHECK(returned == 0 && track.GetNumInliers() == 0, "TriangulateSkewLLS with one posed view returned %u, leaving %u inliers, "
			"expected 0 and none", returned, track.GetNumInliers());
	}

	// 2. panoramas, rays across the optical axis
	constexpr unsigned numViews = 3, numPoints = 200;
	Scene scene(MakeSphereScene({Point3(0, 0, 0), Point3(1, 0, 0), Point3(2, 0, 0)}, 0, 0, cv::Size(4096, 2048)));
	std::mt19937 rng(4096);
	std::uniform_real_distribution<REAL> azimuthDist(REAL(0.5), REAL(M_PI) - REAL(0.5));
	std::normal_distribution<float> noiseDist(0.f, 1.f);
	String medians;
	for (const auto& [dz, noisy] : {std::make_pair(REAL(0), false), std::make_pair(REAL(0), true), std::make_pair(REAL(0.05), true)}) {
		CLISTDEF0(double) errors;
		for (unsigned p = 0; p < numPoints; ++p) {
			// a bearing from the first view with the given dz, on either side of the baseline
			const REAL azimuth = azimuthDist(rng) * ((p & 1) ? REAL(1) : REAL(-1));
			const REAL radial = SQRT(REAL(1) - dz*dz);
			const Point3 X = Point3(radial * COS(azimuth), radial * SIN(azimuth), dz) * REAL(10);
			Track track(Point3(0, 0, 0));
			for (Image& img : scene.images) {
				img.keypoints.clear();
				const auto [proj, valid] = img.ProjectPoint(X);
				TEST_CHECK(valid && Image8U::isInside(proj, img.GetSize()), "the point does not project inside view %u", img.ID);
				cv::Point2f pt = Cast<float>(proj);
				if (noisy) {
					pt.x += noiseDist(rng);
					pt.y += noiseDist(rng);
				}
				img.keypoints.emplace_back(pt, 0.f, 0.f, 10.f);
				track.observations.emplace_back(img.ID, 0);
			}
			const unsigned returned = TriangulateSkewLLS(track, scene.images, 8.f, 0.f);
			TEST_CHECK(noisy || (returned == numViews && norm(track.position - X) <= 5e-3), "exact keypoints at dz = %g: %u inliers, "
				"position %g m off, expected %u and under 0.005", (double)dz, returned, norm(track.position - X), numViews);
			errors.push_back(norm(track.position - X));
		}
		if (!noisy)
			continue;
		const double median = errors.GetMedian();
		TEST_CHECK(median < 0.2, "with 1 pixel of noise at dz = %g the median position error is %.3f m, expected under 0.2", (double)dz, median);
		medians += String::FormatString("%s%.3f m at dz = %g", medians.empty() ? "" : ", ", median, (double)dz);
	}
	return scope.Pass("a triangulation failing for the angle or for a second inlier leaves no inliers and is taken up again; a point "
		"across the optical axis of three panoramas triangulates (1 px noise: %s)", medians.c_str());
}


namespace {

// How thick, in Y, the slab the cross-joint points are drawn from is: they fill the usual 2-unit box
// in X and Z but only this much of it in Y, so at 0 they lie exactly in the plane the arc lies in.
// That is the lever on how much of the bend the two-view geometry across the joint can see: raise it
// and the points off the plane start to resist, so the reprojections alone recover part of the bend.
constexpr REAL JOINT_TRACK_SPREAD_Y = 0;

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
ArcScene BuildPairConstraintScene()
{
	ArcScene arc(8, 20260909);
	arc.Tracks({0, 1, 2}, 80);
	arc.Tracks({1, 2, 3}, 80);
	arc.Tracks({3, 4}, 40, JOINT_TRACK_SPREAD_Y);
	arc.Tracks({4, 5, 6}, 80);
	arc.Tracks({5, 6, 7}, 80);
	for (IIndex id = 0; id < 8; ++id) {
		if (id + 1 < 8)
			arc.Pair(id, id + 1, 300);
		if (id + 2 < 8)
			arc.Pair(id, id + 2, 300);
	}
	arc.Triangulate();
	return arc;
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
} // namespace

// The verified pairs hold the model where the tracks cannot: a chain bent at a joint whose only
// structure is two-view and degenerate stays bent under the reprojection residuals alone, and the
// relative-pose residuals straighten it -- in the global solve and in the local one alike
bool BundleAdjustmentPairConstraintTest()
{
	TestScope scope("BundleAdjustmentPairConstraintTest");
	constexpr REAL PAIR_CONSTRAINT_BEND = 10;                  // degrees the block is turned by
	constexpr double MAX_ROTATION_ERROR = 0.5;                 // degrees
	constexpr double MAX_CENTER_ERROR = 1e-3;                  // of the extent of the arc (PoseErrors); the solves measure under 1e-7 (the old bar was 0.05 units, 0.0087 of this extent)
	const IIndex farthestFromHinge = 7; // the camera at the far end of the turned block

	// without them the bend survives the solve: no reprojection residual in either half of the chain is violated by it, and
	// the cross-joint tracks are blind to it
	{
		ArcScene arc(BuildPairConstraintScene());
		BendPairConstraintScene(arc.scene, PAIR_CONSTRAINT_BEND);
		BAConfig config;
		config.relativeRotationSigma = config.relativeTranslationSigma = 0.f;
		TEST_CHECK(BundleAdjustment::Adjust(arc.scene, config), "the bundle adjustment did not solve the bent scene with the pair residuals off");
		const double rotationError = RotationErrorDeg(arc.scene.images[farthestFromHinge].R, arc.truth[farthestFromHinge].R);
		TEST_CHECK(rotationError > 5.0, "camera %u is %.2f degrees from the truth after the solve with the pair residuals off, so the reprojections already "
			"straighten the %.0f-degree bend and the scene does not pose the problem", farthestFromHinge, rotationError, (double)PAIR_CONSTRAINT_BEND);
	}
	// with them every pair across the joint is heard and the block returns to where it belongs, in the global solve, and in
	// the local one, where cameras 0-3 are constants so that the pairs across the joint constrain the free block alone
	for (const bool local : {false, true}) {
		ArcScene arc(BuildPairConstraintScene());
		BendPairConstraintScene(arc.scene, PAIR_CONSTRAINT_BEND);
		if (local) {
			TEST_CHECK(BundleAdjustment::AdjustLocal(arc.scene, {4, 5, 6, 7}, {0, 1, 2, 3}, BAConfig()), "the local bundle adjustment did not solve the bent scene");
		} else {
			TEST_CHECK(BundleAdjustment::Adjust(arc.scene, BAConfig()), "the bundle adjustment did not solve the bent scene with the pair residuals on");
		}
		const PoseError error = PoseErrors(arc.scene, arc.truth);
		TEST_CHECK(error.rotationMax <= MAX_ROTATION_ERROR && error.positionMax <= MAX_CENTER_ERROR, "the %s solve left the model %.3f degrees and %.4f of its extent "
			"from the truth, past the %.1f degrees and %.3f the pairs should recover", local ? "local" : "global", error.rotationMax, error.positionMax,
			MAX_ROTATION_ERROR, MAX_CENTER_ERROR);
	}
	return scope.Pass("a %.0f-degree bend at a joint the tracks leave free survives the reprojection residuals and is undone by the relative-pose "
		"residuals of the verified pairs, globally and in a local window alike", (double)PAIR_CONSTRAINT_BEND);
}
/*----------------------------------------------------------------*/

// The mean reprojection error a solve reports is taken over the observations the solve used, not the ones its per-image budget
// left out: an observation of image 0 stands for those the budget leaves out of a solve, and one of them is a million pixels off
bool ReprojectionReportOverUsedTest()
{
	TestScope scope("ReprojectionReportOverUsedTest");
	// a posed synthetic scene whose observations are exact: every error below is one this test put there
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
	TEST_CHECK(numLeftOut != 0 && numUsed != 0, "the scene holds %u observations of image %u and %u of the others", numLeftOut, (unsigned)LEFT_OUT, numUsed);
	const auto [meanUsed, angUsed] = ComputeTracksMeanReprojectionError(scene, [](uint32_t imageID, uint32_t) { return imageID != LEFT_OUT; });
	TEST_CHECK(ABS(meanUsed - USED_ERROR) <= 0.01f && ISFINITE(angUsed), "the mean over the used observations is %.4f px, expected the %.2f px they carry",
		(double)meanUsed, (double)USED_ERROR);
	// the mean over every observation, as a solve without a budget reports it, is the one the left-out outlier dominates
	const float meanAll = ComputeTracksMeanReprojectionError(scene).first;
	TEST_CHECK(meanAll >= 10.f * USED_ERROR, "the mean over every observation is %.4f px, expected the left-out outlier to dominate it", (double)meanAll);
	return scope.Pass("%.3f px over the %u used observations (%u left out) against %.1f px over all", (double)meanUsed, numUsed, numLeftOut, (double)meanAll);
}
/*----------------------------------------------------------------*/

// The per-image observation budget (BAConfig::maxObservationsPerImage):
// 1. it decides which observations an image contributes, so what it must not change is the solution: on a scene whose images
//    carry far more of them than it allows, the capped solve recovers the poses the uncapped one recovers -- on a dense-matched
//    and on a described-only scene alike, since the budget covers every keypoint kind; what it keeps is drawn again for every
//    solve, so the comparison is on the solution and never on the kept set; and a solve the threshold leaves alone fits every
//    observation it was given;
// 2. a track it leaves with only views that lie close together gets back the dropped view that spans the widest angle with one
//    of them. Image 2 holds 100 observations against a budget of 95, all in one cell of its frame, the 60 shortest tracks (seen
//    by the images 0, 1 and 2, the first two 10 degrees apart) being the only ones the budget can take from it: the 40 longer
//    ones (images 2-5) come first in the cell. Two views 10 degrees apart are under the 15-degree bar, and image 2 sees the point
//    20 degrees from image 0, so a track that lost its view in image 2 is given it back and the image fits all of its 100
//    observations;
// 3. the solve of a capped input holds the scale: twelve arc cameras in a chain, every track seen by three consecutive cameras at
//    its exact projection, per triple a few described tracks, which the cap keeps whole, and many dense ones, of which it keeps
//    a few observations an image and leaves most tracks out of the solve. The cameras and the dense points sit at the truth;
//    the described points start 2% further from the cameras (scaled about their centroid), the depth error a solve of a new
//    input corrects. With one camera held as the reference and no GPS, nothing else holds the scale, the iterations that bring
//    the described points back also move the cameras along it (+0.83% here), and the dense tracks the cap left out keep their
//    points, so 12% of their observations land past the 4 px bar and the filter after the solve would drop them. Held, the
//    solve ends at the truth: every camera where it was and every left-out observation on its keypoint.
bool BAObservationCapTest()
{
	TestScope scope("BAObservationCapTest");

	// 1. the solution
	constexpr double MAX_ROTATION_ERROR = 0.05;   // degrees the cap may add to the uncapped solve's
	constexpr double MAX_CENTER_ERROR = 0.001;    // share of the extent it may add (0.005 units, on an arrangement that spans ~6)
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
		TEST_CHECK(BundleAdjustment::Adjust(uncapped, config), "the uncapped bundle adjustment did not solve the %s scene", kind);
		config.maxObservationsPerImage = CAP;
		TEST_CHECK(BundleAdjustment::Adjust(capped, config), "the capped bundle adjustment did not solve the %s scene", kind);
		const PoseError errorUncapped = PoseErrors(uncapped, gtPoses), errorCapped = PoseErrors(capped, gtPoses);
		maxRotationDegUncapped[iScene] = errorUncapped.rotationMax;
		maxCenterUncapped[iScene] = errorUncapped.positionMax;
		maxRotationDeg[iScene] = errorCapped.rotationMax;
		maxCenter[iScene] = errorCapped.positionMax;
		TEST_CHECK(maxRotationDeg[iScene] <= maxRotationDegUncapped[iScene] + MAX_ROTATION_ERROR && maxCenter[iScene] <= maxCenterUncapped[iScene] + MAX_CENTER_ERROR,
			"the capped solve of the %s scene left the model %.4f degrees and %.5f of its extent from the truth, past the %.4f and %.5f the uncapped one "
			"reaches by more than the %.2f degrees and %.3f the cap may cost", kind, maxRotationDeg[iScene], maxCenter[iScene], maxRotationDegUncapped[iScene],
			maxCenterUncapped[iScene], MAX_ROTATION_ERROR, MAX_CENTER_ERROR);
		// and a solve the threshold leaves alone fits every observation it was given
		const unsigned numObservations = CountSceneObservations(capped);
		config.minObservationsForCap = numObservations + 1;
		BundleAdjustment ba(capped, config);
		TEST_CHECK(ba.Adjust(), "the uncapped bundle adjustment did not re-solve the %s scene", kind);
		unsigned numResiduals = 0;
		for (const unsigned numImageResiduals : ba.GetNumReprojResidualsPerImage())
			numResiduals += numImageResiduals;
		TEST_CHECK(numResiduals == numObservations, "the %s scene is under the threshold the cap applies from, yet its solve fitted %u of its %u observations",
			kind, numResiduals, numObservations);
		++iScene;
	}

	// 2. the wide view taken back
	{
		constexpr unsigned WIDE_CAP = 95, NUM_SHORT = 60, NUM_LONG = 40;
		constexpr float BAR = 15.f;
		constexpr IIndex WIDE = 2;
		ArcScene arc(6, 20261114);
		Scene& scene = arc.scene;
		// the points are drawn from the pixels of the wide image's frame, a few pixels around the middle of one cell of the
		// budget's 10 x 10 grid (the cell [320,384) x [240,288)), at a depth near the arc's centre
		std::mt19937 rng(20261114);
		std::uniform_real_distribution<REAL> jitter(-5, 5), depth(4.8, 5.2);
		const Image& wide = scene.images[WIDE];
		for (unsigned i = 0; i < NUM_SHORT + NUM_LONG; ) {
			const Point2 pixel(352 + jitter(rng), 264 + jitter(rng));
			const Point3 X = wide.C + wide.R.t() * (wide.pCamera->UnprojectNormalized(pixel) * depth(rng));
			if (arc.TrackAt(X, i < NUM_SHORT ? IIndexArr{0, 1, WIDE} : IIndexArr{WIDE, 3, 4, 5}))
				++i;
		}
		FilterTracks(scene, 4.f, 1.f);
		BAConfig config;
		config.maxObservationsPerImage = WIDE_CAP;
		config.minObservationsForCap = 0;
		config.minTriangulationAngle = BAR;
		BundleAdjustment ba(scene, config);
		TEST_CHECK(ba.Adjust(), "the capped bundle adjustment did not solve the wide-view scene");
		const UnsignedArr& fitted = ba.GetNumReprojResidualsPerImage();
		const unsigned expected[6] = {NUM_SHORT, NUM_SHORT, NUM_SHORT + NUM_LONG, NUM_LONG, NUM_LONG, NUM_LONG};
		for (IIndex id = 0; id < 6; ++id)
			TEST_CHECK(fitted[id] == expected[id], "image %u fits %u observations, expected %u (image %u holds %u against a budget of %u: the tracks that lose "
				"a view there have their other two views about 10 degrees apart, under the %.0f-degree bar, and take it back)", id, fitted[id], expected[id],
				WIDE, NUM_SHORT + NUM_LONG, WIDE_CAP, (double)BAR);
	}

	// 3. the scale
	constexpr unsigned NUM_CAMERAS = 12;
	constexpr unsigned NUM_DESCRIBED = 30;          // per triple: at most 90 an image, all kept by the cap
	constexpr unsigned NUM_DENSE = 400;             // per triple: the cap keeps what is left of an image's budget
	constexpr REAL CAPPED_SCALE_PUSH = 0.02;        // the described points' start, this share further from the cameras
	constexpr unsigned CAPPED_SCALE_CAP = 100;      // observations an image keeps, whatever the scene's size
	constexpr double MAX_SCALE_CHANGE = 1e-4;       // the cameras' scale after the solve against the true one
	constexpr double MAX_LEFT_OUT_PAST_BAR = 0.01;  // share of the left-out observations past the 4 px bar
	const ResectionConfig resectionCfg;
	ArcScene arc(NUM_CAMERAS, 20261016);
	Scene& scene = arc.scene;
	for (unsigned k = 0; k + 2 < NUM_CAMERAS; ++k)
		arc.Tracks({k, k + 1, k + 2}, NUM_DESCRIBED);
	const size_t numDescribedTracks = scene.tracks.size();
	for (Image& img : scene.images)
		img.CloseDescribedKeypoints();
	for (unsigned k = 0; k + 2 < NUM_CAMERAS; ++k)
		arc.Tracks({k, k + 1, k + 2}, NUM_DENSE);
	Point3 cameraCentroid(0, 0, 0);
	for (const Pose3D& pose : arc.truth)
		cameraCentroid += pose.C;
	cameraCentroid *= REAL(1) / arc.truth.size();
	for (size_t ti = 0; ti < numDescribedTracks; ++ti)
		scene.tracks[ti].position = cameraCentroid + (scene.tracks[ti].position - cameraCentroid) * (REAL(1) + CAPPED_SCALE_PUSH);
	std::vector<Point3> startPoints;
	for (const Track& track : scene.tracks)
		startPoints.push_back(track.position);
	FilterTracks(scene, resectionCfg.maxReprojError * ResectionConfig::fullBAInputErrorFactor, resectionCfg.minAngleThreshold, resectionCfg.multDepthNear, resectionCfg.multDepthFar);
	BAConfig baConfig = resectionCfg.fullBAConfig;
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
	std::vector<Point3> centers, trueCenters;
	for (const Image& img : scene.images)
		centers.push_back(img.C);
	for (const Pose3D& pose : arc.truth)
		trueCenters.push_back(pose.C);
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
			pastBar += error > resectionCfg.maxReprojError;
			sumError += MINF(error, 1e3);
		}
	}
	TEST_CHECK(leftOut != 0 && solvedDescribed == numDescribedTracks, "premise not met: the cap left %u tracks out of the solve and solved %u of the %u "
		"described ones", leftOutTracks, solvedDescribed, (unsigned)numDescribedTracks);
	const double pastShare = (double)pastBar / leftOut;
	TEST_CHECK(ABS(scaleChange) <= MAX_SCALE_CHANGE && pastShare <= MAX_LEFT_OUT_PAST_BAR, "the capped solve changed the cameras' scale by %+.4f%% and left "
		"%u of the %u observations of the %u tracks it left out past %g px (mean %.2f px); expected within %g%% and %g%%", scaleChange * 100, pastBar, leftOut,
		leftOutTracks, resectionCfg.maxReprojError, sumError / leftOut, MAX_SCALE_CHANGE * 100, MAX_LEFT_OUT_PAST_BAR * 100);
	return scope.Pass("capping an image at %u observations leaves the solve %.4f degrees and %.5f of the extent from the truth against %.4f and %.5f uncapped on a "
		"dense-matched scene, %.4f and %.5f against %.4f and %.5f on a described-only one, a scene under the threshold keeps every observation, an image takes "
		"back the wide view it would lose, and a capped solve without GPS kept the cameras' scale within %+.4f%% and %u of the %u observations it left out within "
		"%g px", CAP, maxRotationDeg[0], maxCenter[0], maxRotationDegUncapped[0], maxCenterUncapped[0], maxRotationDeg[1], maxCenter[1], maxRotationDegUncapped[1],
		maxCenterUncapped[1], scaleChange * 100, leftOut - pastBar, leftOut, resectionCfg.maxReprojError);
}
/*----------------------------------------------------------------*/

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
		ArcScene arc(NUM_IMAGES, 20261015);
		for (IIndex i = 0; i + 2 < NUM_IMAGES; ++i)
			arc.Tracks({i, i + 1, i + 2}, NUM_TRACKS_PER_TRIPLE);
		for (IIndex i = 0; i < NUM_IMAGES; ++i) {
			if (i + 1 < NUM_IMAGES)
				arc.Pair(i, i + 1, PAIR_INLIERS);
			if (i + 2 < NUM_IMAGES)
				arc.Pair(i, i + 2, PAIR_INLIERS);
		}
		arc.Triangulate();
		std::uniform_real_distribution<REAL> unit(-1, 1);
		for (Image& img : arc.scene.images) {
			const RMatrix turn(Point3(unit(arc.rng), unit(arc.rng), unit(arc.rng)) * D2R(ROTATION_NOISE_DEG));
			img.R = turn * img.R;
			img.C += Point3(unit(arc.rng), unit(arc.rng), unit(arc.rng)) * CENTER_NOISE;
		}
		for (Track& track : arc.scene.tracks)
			track.position += Point3(unit(arc.rng), unit(arc.rng), unit(arc.rng)) * CENTER_NOISE;
		scene = std::move(arc.scene);
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
/*----------------------------------------------------------------*/

} // namespace SFM
