/*
 * TestsSFMCalibration.cpp
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
#include <future>
#include <thread>
#include <functional>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestSFM "));

namespace SFM {

// The two-view fixture of the inlier tests through a lens: a 640x480 camera at f 400 with a barrel
// lens it knows (LENS_INLIER_K1: 44 px at the corner), and a second view turned
// LENS_INLIER_YAW_DEG about the vertical and moved 0.3 sideways, so that a point near the edge of
// one image lands near the centre of the other and the lens displaces it differently in each --
// what bends the epipolar line. The points sit on a 16x12 grid of the first image at depths 4-7;
// each one whose exact projection lands at least LENS_INLIER_MARGIN px inside the second image is
// kept, as one keypoint in each image under the same index.
constexpr REAL LENS_INLIER_K1 = REAL(-0.08);
constexpr REAL LENS_INLIER_YAW_DEG = REAL(15);
constexpr int LENS_INLIER_MARGIN = 8;
// a keypoint of the first image this far out of its corner radius counts as a corner match
constexpr REAL LENS_INLIER_CORNER_SHARE = REAL(0.75);
// the epipolar bar the keyframe matcher is called with in the test below, pixels
constexpr float LENS_INLIER_GEOMETRIC_BAR = 2.f;

static void MakeLensInlierScene(Scene& scene)
{
	const int width = 640, height = 480;
	PinholeCamera* const camera = new PinholeCamera(cv::Size(width, height),
		REAL(400), REAL(400), REAL(width - 1)/2, REAL(height - 1)/2);
	camera->trustIntrinsics = true;
	camera->SetDistortion(LENS_INLIER_K1, 0, 0, 0);
	scene.cameras.emplace_back(camera);
	const REAL yaw = D2R(LENS_INLIER_YAW_DEG);
	Pose3D poseA, poseB;
	poseA.R = Matrix3x3::IDENTITY;
	poseA.C = Point3(0, 0, 0);
	poseB.R = Matrix3x3(COS(yaw), 0, -SIN(yaw), 0, 1, 0, SIN(yaw), 0, COS(yaw));
	poseB.C = Point3(REAL(0.3), 0, 0);
	scene.images.emplace_back((IIndex)0, String("0.jpg"), poseA, (IIndex)0, scene.cameras[0]);
	scene.images.emplace_back((IIndex)1, String("1.jpg"), poseB, (IIndex)0, scene.cameras[0]);
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];
	constexpr int gridX = 16, gridY = 12;
	for (int gy = 0; gy < gridY; ++gy) {
		for (int gx = 0; gx < gridX; ++gx) {
			const Point2 ptA(
				LENS_INLIER_MARGIN + gx*REAL(width - 1 - 2*LENS_INLIER_MARGIN)/(gridX - 1),
				LENS_INLIER_MARGIN + gy*REAL(height - 1 - 2*LENS_INLIER_MARGIN)/(gridY - 1));
			// camera A sits at the origin unrotated: its ray, which Unproject leaves at z = 1, scaled to the depth
			const REAL depth = REAL(4) + REAL((gx*5 + gy*3) % 7)*REAL(0.5);
			const Point3 X(imgA.Ray(ptA)*depth);
			const auto [ptB, valid] = imgB.ProjectPoint(X);
			if (!valid ||
				ptB.x < LENS_INLIER_MARGIN || ptB.x > width - 1 - LENS_INLIER_MARGIN ||
				ptB.y < LENS_INLIER_MARGIN || ptB.y > height - 1 - LENS_INLIER_MARGIN)
				continue;
			imgA.keypoints.emplace_back((float)ptA.x, (float)ptA.y, 10.f);
			imgB.keypoints.emplace_back((float)ptB.x, (float)ptB.y, 10.f);
		}
	}
}

// the keypoints of the first image of MakeLensInlierScene that lie in its corners
static std::vector<uint32_t> LensInlierCorners(const Image& img)
{
	const PinholeCamera& camera = *static_cast<const PinholeCamera*>(img.pCamera);
	const REAL cornerRadius = norm(Point2(camera.cx, camera.cy));
	std::vector<uint32_t> corners;
	FOREACH(i, img.keypoints)
		if (norm(Point2(img.keypoints[i].pt.x - camera.cx, img.keypoints[i].pt.y - camera.cy)) >= LENS_INLIER_CORNER_SHARE*cornerRadius)
			corners.push_back((uint32_t)i);
	return corners;
}

// The pixel Sampson distance of a correspondence to a fundamental matrix, x2^T F x1 over its gradient
static double PixelSampsonDistance(const Matrix3x3& F, const Point2f& ptA, const Point2f& ptB)
{
	const Eigen::Matrix3d M = F; // implicit TMatrix<double,3,3> -> Eigen::Matrix3d, as MatchGeometric.cpp reads E
	const Eigen::Vector3d a(ptA.x, ptA.y, 1.0), b(ptB.x, ptB.y, 1.0);
	const Eigen::Vector3d Ma(M*a), Mtb(M.transpose()*b);
	return ABS(b.dot(Ma))/SQRT(Ma.x()*Ma.x() + Ma.y()*Ma.y() + Mtb.x()*Mtb.x() + Mtb.y()*Mtb.y());
}

constexpr double LENS_REVERIFY_WRONG_SHIFT = 8.0;
// a dense sample within this share of the corner radius of the centre is a central one: the lens
// moves it about 2 px at most there (LENS_INLIER_K1), so the pinhole and the true geometry agree on it
constexpr REAL LENS_REVERIFY_CENTRAL_SHARE = REAL(0.4);
// where a dense sample is drawn from its keypoint of A, in pixels along each axis: a central one
// between the grid points, a corner one toward the image corner, where the lens bends the pinhole
// epipolar line the most and inside the LENS_INLIER_MARGIN the keypoints keep from the edges
constexpr REAL LENS_REVERIFY_CENTRAL_OFFSET = REAL(12);
constexpr REAL LENS_REVERIFY_CORNER_OFFSET = REAL(6);

// The lens fixture with the guided matches (every keypoint of A matched to its own index in B) and the keypoints of A's corners, which
// the matching tests below share
struct LensInlierFixture {
	Scene scene;
	std::vector<uint32_t> corners;
	std::vector<DMatch> guided;

	LensInlierFixture()
	{
		MakeLensInlierScene(scene);
		corners = LensInlierCorners(scene.images[0]);
		FOREACH(i, scene.images[0].keypoints)
			guided.emplace_back((uint32_t)i, (uint32_t)i);
	}
};

// The inlier tests of a RoMa2 pair through a lens (SampsonTest): on two views through a barrel lens the camera knows, every guided
// match on the true geometry is a sparse inlier, the corner ones included, while the pixel fundamental matrix of the same geometry,
// composed without the lens, puts the corner ones past the matcher's epipolar bar
static bool CheckDenseLensInliers(const TestScope& scope)
{
	LensInlierFixture fx;
	Image& imgA = fx.scene.images[0];
	Image& imgB = fx.scene.images[1];
	MatchConfig matchCfg;
	PairsMatcher matcher(fx.scene, matchCfg);
	DenseMatchConfig config;
	// an admitted pair with no inlier cells: no dense fill, the guided matches alone are fitted and classified
	PairVerdict verdict;
	verdict.admitted = true;
	ImagePair pair(0, 1);
	ArmPair(pair, fx.scene);
	DenseMatches dense;
	constexpr int warpSize = 64;
	TEST_CHECK(AssemblePairDense(matcher, imgA, imgB, verdict, fx.guided, config, warpSize, pair, dense), "the pair was not assembled");
	TEST_CHECK(pair.outlierMatches.empty() && pair.GetNumFilteredInliers() == fx.guided.size(), "%u of the %u matches on the true geometry are inliers, %u rejected",
		pair.GetNumFilteredInliers(), (unsigned)fx.guided.size(), (unsigned)pair.outlierMatches.size());
	double worst = 0;
	for (uint32_t i : fx.corners)
		worst = MAXF(worst, PixelSampsonDistance(pair.F.value(), imgA.keypoints[i].pt, imgB.keypoints[i].pt));
	TEST_CHECK(worst > (double)matchCfg.maxEpipolarError, "the corner matches lie within %.2f px of the pixel F, the fixture does not bend the epipolar lines past the %.1f px bar",
		worst, matchCfg.maxEpipolarError);
	VERBOSE("LensMatchingTest: %u corner matches kept through the lens, up to %.1f px off the pixel F", (unsigned)fx.corners.size(), worst);
	return true;
}

// The keyframe extractor's geometric match through a lens (MatchFeaturesGeometric): on the same two views, with exact tracked guesses
// and a unique descriptor per correspondence, every corner correspondence is matched, while its pixel Sampson distance to the pixel F
// the match estimates exceeds the bar the match is held to
static bool CheckKeyframeMatchThroughLens(const TestScope& scope)
{
	LensInlierFixture fx;
	Image& imgA = fx.scene.images[0];
	Image& imgB = fx.scene.images[1];
	const int numPoints = (int)imgA.keypoints.size();
	constexpr int descBytes = 32;
	imgA.descriptors.create(numPoints, descBytes, CV_8U);
	imgB.descriptors.create(numPoints, descBytes, CV_8U);
	for (int i = 0; i < numPoints; ++i) {
		std::mt19937 descRng(0x5EEDu ^ (unsigned)i);
		for (int b = 0; b < descBytes; ++b) {
			const uint8_t byte = (uint8_t)(descRng() & 0xFF);
			imgA.descriptors.at<uint8_t>(i, b) = byte;
			imgB.descriptors.at<uint8_t>(i, b) = byte;
		}
	}
	std::vector<Point2f> tracked1(numPoints), tracked2(numPoints);
	std::vector<uchar> trackStatus(numPoints, 1);
	for (int i = 0; i < numPoints; ++i) {
		tracked1[i] = imgA.keypoints[i].pt;
		tracked2[i] = imgB.keypoints[i].pt;
	}
	MatchConfig matchCfg;
	matchCfg.minMatches = 30;
	matchCfg.maxEpipolarError = 4.f;
	matchCfg.matchRatio = 0.9f;
	matchCfg.descriptorsAreBinary = true;
	matchCfg.minTriangulationAngle = 0.f;
	matchCfg.reprojThreshold = 0.f;
	matchCfg.epipoleFilterThreshold = 0.f;
	PairsMatcher matcher(fx.scene, matchCfg);
	ImagePair pair(0, 1);
	TEST_CHECK(MatchFeaturesGeometric(matcher, imgA, imgB, tracked1, tracked2, trackStatus, pair, LENS_INLIER_GEOMETRIC_BAR), "no geometry was estimated");
	std::unordered_set<uint32_t> matched;
	for (const DMatch& m : pair.matches)
		if (m.queryIdx == m.trainIdx)
			matched.insert(m.queryIdx);
	unsigned numCornersMatched = 0;
	double worst = 0;
	for (uint32_t i : fx.corners) {
		numCornersMatched += (unsigned)matched.count(i);
		worst = MAXF(worst, PixelSampsonDistance(pair.F.value(), imgA.keypoints[i].pt, imgB.keypoints[i].pt));
	}
	TEST_CHECK(numCornersMatched == fx.corners.size(), "%u of the %u corner correspondences matched", numCornersMatched, (unsigned)fx.corners.size());
	TEST_CHECK(worst > (double)LENS_INLIER_GEOMETRIC_BAR, "the corner correspondences lie within %.2f px of the pixel epipolar geometry, the fixture does not exercise the lens", worst);
	VERBOSE("LensMatchingTest: %u corner correspondences matched through the lens, up to %.1f px off the pixel F", numCornersMatched, worst);
	return true;
}

// A RoMa2 pair matched through the pinhole and re-verified once its camera knows its lens: on the lens fixture, classified at matching
// through the pinhole geometry of the true pose (the verdict's, which stands when the union fit is refused), the guided matches at the
// corners the pinhole rejected are on the true geometry and come back as sparse inliers, and the dense samples the pinhole bar let
// through off the true geometry (placed on the pinhole epipolar line, LENS_REVERIFY_WRONG_SHIFT px or more from the true position) are
// dropped while the true dense samples stay
static bool CheckReverifyThroughLens(const TestScope& scope)
{
	LensInlierFixture fx;
	Scene& scene = fx.scene;
	PinholeCamera& camera = *static_cast<PinholeCamera*>(scene.cameras[0]);
	const PinholeCamera truth(camera); // the true lens, which projects the dense samples below
	camera.SetDistortion(0, 0, 0, 0);   // the matching sees a pinhole
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];
	MatchConfig matchCfg;
	PairsMatcher matcher(scene, matchCfg);
	DenseMatchConfig config;
	matcher.SetDense(NULL, config);
	// the matching keeps the pinhole geometry of the true pose: its own union fit, refused here by an estimator asking for more inliers
	// than the pair has, would absorb the lens into a pose degrees off whose epipolar lines pass within a pixel of every corner match
	// (3.8 deg and 0.96 px at LENS_INLIER_K1), leaving nothing for the lens to regain
	MatchConfig matchCfgPinhole(matchCfg);
	matchCfgPinhole.minMatches = (unsigned)fx.guided.size() + 1;
	PairsMatcher matcherPinhole(scene, matchCfgPinhole);
	PairVerdict verdict;
	verdict.admitted = true;
	ImagePair pair(0, 1);
	ArmPair(pair, scene);
	DenseMatches unused;
	constexpr int warpSize = 64;
	TEST_CHECK(AssemblePairDense(matcherPinhole, imgA, imgB, verdict, fx.guided, config, warpSize, pair, unused), "the pair was not assembled through the pinhole");
	const size_t numRejected = pair.outlierMatches.size();
	TEST_CHECK(numRejected >= 4, "the pinhole rejected only %u true matches, the fixture does not exercise the lens", (unsigned)numRejected);
	// the dense segment as a pinhole bar leaves it: true samples between the central grid points, and at the corners samples drawn toward
	// the image corner and moved onto the pinhole epipolar line, off the true position
	DenseMatches dense;
	unsigned numTrueDense = 0;
	FOREACH(i, imgA.keypoints) {
		const Point2f& kA = imgA.keypoints[i].pt;
		const bool corner = std::find(fx.corners.begin(), fx.corners.end(), (uint32_t)i) != fx.corners.end();
		const Point2 ptA(corner ?
			Point2(kA.x + (kA.x < camera.cx ? -LENS_REVERIFY_CORNER_OFFSET : LENS_REVERIFY_CORNER_OFFSET),
			       kA.y + (kA.y < camera.cy ? -LENS_REVERIFY_CORNER_OFFSET : LENS_REVERIFY_CORNER_OFFSET)) :
			Point2(kA.x + LENS_REVERIFY_CENTRAL_OFFSET, kA.y + LENS_REVERIFY_CENTRAL_OFFSET));
		if (!Image8U::isInside(Point2f((float)ptA.x, (float)ptA.y), imgA.GetSize()))
			continue; // off the edge of A
		const Point3 X(truth.Unproject(ptA)*REAL(5)); // camera A at the origin unrotated
		const auto [ptBtrue, valid] = truth.Project(imgB.R*(X - imgB.C));
		if (!valid || !Image8U::isInside(Point2f((float)ptBtrue.x, (float)ptBtrue.y), imgB.GetSize()))
			continue;
		Point2f ptB((float)ptBtrue.x, (float)ptBtrue.y);
		if (corner) {
			// onto the epipolar line of the pinhole F the pair holds
			const Eigen::Matrix3d F = pair.F.value();
			const Eigen::Vector3d l(F*Eigen::Vector3d(ptA.x, ptA.y, 1.0));
			const double d = (l.x()*ptB.x + l.y()*ptB.y + l.z())/(l.x()*l.x() + l.y()*l.y());
			const Point2f moved((float)(ptB.x - d*l.x()), (float)(ptB.y - d*l.y()));
			if (norm(moved - ptB) < LENS_REVERIFY_WRONG_SHIFT)
				continue;
			ptB = moved;
		} else if (norm(Point2(ptA.x - camera.cx, ptA.y - camera.cy)) < LENS_REVERIFY_CENTRAL_SHARE*norm(Point2(camera.cx, camera.cy))) {
			++numTrueDense;
		} else {
			continue; // neither central nor a corner: left out
		}
		dense.pointsA.emplace_back((float)ptA.x, (float)ptA.y);
		dense.pointsB.push_back(ptB);
		dense.confidences.push_back(0.5f);
	}
	const unsigned numWrongDense = (unsigned)dense.pointsA.size() - numTrueDense;
	TEST_CHECK(numTrueDense >= 4 && numWrongDense >= 2, "%u true and %u wrong dense samples, too few to judge", numTrueDense, numWrongDense);
	std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap;
	StorePairDense(scene, pairIndexMap, std::move(pair), dense, warpSize);
	// now the camera knows its lens, and the pair is verified again through it
	camera.SetDistortion(LENS_INLIER_K1, 0, 0, 0);
	matcher.ComputeRelativePoses(true, false, {scene.cameras[0]});
	const ImagePair& stored = scene.pairs[0];
	TEST_CHECK(stored.GetNumFilteredInliers() == fx.guided.size(), "%u of the %u true matches are sparse inliers through the lens", stored.GetNumFilteredInliers(), (unsigned)fx.guided.size());
	TEST_CHECK(stored.outlierMatches.empty(), "%u matches are still outliers through the lens, where every true match is an inlier and every wrong dense sample dropped", (unsigned)stored.outlierMatches.size());
	TEST_CHECK(stored.matches.size() == stored.GetNumTrackFormingMatches(), "%u matches past the track-forming ones", (unsigned)stored.matches.size() - stored.GetNumTrackFormingMatches());
	TEST_CHECK(stored.GetNumDenseInliers() == numTrueDense && HasDensePartition(scene, stored), "%u dense samples kept, expected the %u true ones of the %u (partition %s)",
		stored.GetNumDenseInliers(), numTrueDense, numTrueDense + numWrongDense, HasDensePartition(scene, stored) ? "kept" : "broken");
	for (unsigned k = 0; k < stored.GetNumDenseInliers(); ++k) {
		const Point2f& ptA = imgA.keypoints[stored.matches[stored.GetNumFilteredInliers() + k].queryIdx].pt;
		TEST_CHECK(norm(Point2(ptA.x - camera.cx, ptA.y - camera.cy)) < LENS_REVERIFY_CENTRAL_SHARE*norm(Point2(camera.cx, camera.cy)),
			"a dense sample off the true geometry was kept at (%.1f, %.1f)", ptA.x, ptA.y);
	}
	VERBOSE("LensMatchingTest: %u true matches regained, %u wrong dense samples dropped", (unsigned)numRejected, numWrongDense);
	return true;
}

// The matchers and the re-verification through a lens the camera knows, on one fixture of two views through a barrel lens (the second
// turned 15 degrees and moved 0.3 sideways, so that a point near the edge of one image lands near the centre of the other):
// - the inlier tests of a RoMa2 pair: every guided match on the true geometry is a sparse inlier, the corner ones included, while
//   the pixel fundamental matrix of the same geometry, composed without the lens, puts the corner ones past the matcher's epipolar bar;
// - the keyframe extractor's geometric match: every corner correspondence is matched though it lies past the bar of the pixel geometry;
// - the re-verification once the camera knows its lens: the guided matches at the corners the pinhole rejected come back as sparse
//   inliers, and the dense samples the pinhole bar let through off the true geometry are dropped while the true ones stay.
bool LensMatchingTest()
{
	TestScope scope("LensMatchingTest");
	{
		LensInlierFixture fx;
		TEST_CHECK(fx.corners.size() >= 4, "only %u correspondences in A's corners, the fixture does not reach the lens", (unsigned)fx.corners.size());
	}
	if (!CheckDenseLensInliers(scope) || !CheckKeyframeMatchThroughLens(scope) || !CheckReverifyThroughLens(scope))
		return false;
	return scope.Pass("the inlier tests, the keyframe match and the re-verification through the lens");
}

namespace {

// One synthetic camera of the lens fit's scenes: its frame, focal and TRUE lens
struct LensTestCamera {
	int width, height;
	REAL focal, k1, k2;
};
// A: a strong barrel lens, 164 px at the corner (Playground's is 72 px); B: another device, a mild
// pincushion of 22 px; P: A's frame without a lens -- the cameras the prototype's synthetic scenes
// were drawn with (lens_fit_synthetic.py), whose measurements set the bars below
const LensTestCamera LENS_TEST_A{1920, 1080, REAL(1400), REAL(-0.20), REAL(0.05)};
const LensTestCamera LENS_TEST_B{1280, 960, REAL(900), REAL(0.06), REAL(-0.03)};
const LensTestCamera LENS_TEST_P{1920, 1080, REAL(1400), REAL(0), REAL(0)};
// A's frame through a pincushion of +160 px at the corner (k1 alone), which a fit from no lens falls short
// of in 42 of 100 scenes of 40 pairs
const LensTestCamera LENS_TEST_PINCUSHION{1920, 1080, REAL(1400), REAL(0.37664), REAL(0)};
// A's frame through a lens too faint to matter, 1e-3 px at the corner: exact positions through it
// leave the poses alone an error at the level of float rounding, which a fitted lens can still lower
// by more than MatchConfig::lensFitMinErrorDrop, and for which no lens may be kept
const LensTestCamera LENS_TEST_FAINT{1920, 1080, REAL(1400), REAL(-1.5e-6), REAL(0)};
// A given k1 (k2 0) that folds camera B's lens at 0.41 of its focal from the centre, 365 px, well inside
// its frame (its corner lies at 0.89): past the fold its undistortion has no answer
constexpr REAL LENS_TEST_FOLDING_K1 = REAL(-0.9);

// The focal a camera of a lens scene is given, and whether it is trusted (a calibration, a forced focal) or not (an EXIF
// 35 mm-equivalent focal, the import's default)
struct LensTestGiven {
	REAL focalScale; // the given focal, a scale of the true one
	bool trusted;
};

enum class LensTestMotion {
	TURN,   // the second view turns 10-30 deg about a near-vertical axis and moves 0.2-0.8 sideways: the lens shows
	FORWARD // it moves 0.5-1.5 along the optical axis rolling 6-10 deg about it: the lens moves points along the epipolar lines, and hides
};
struct LensTestPairs {
	unsigned cameraA, cameraB; // indices into the cameras given to BuildLensTestScene
	LensTestMotion motion;
	unsigned count;
};
constexpr unsigned LENS_TEST_POINTS = 400;                   // correspondences a pair is drawn with, before its verification
constexpr unsigned LENS_TEST_ATTEMPTS = 40*LENS_TEST_POINTS; // points tried for them
constexpr REAL LENS_TEST_NOISE_PX = REAL(0.5);               // Gaussian noise of every measured position, pixels
constexpr REAL LENS_TEST_OUTLIER_SHARE = REAL(0.05);         // share of a pair's second-view positions replaced by random ones
constexpr unsigned LENS_TEST_RADII = 64;                     // radii the displacement curves are compared at
constexpr REAL LENS_TEST_REACH_PERCENTILE = REAL(0.99);      // a camera's reach: this quantile of its matched keypoints' radii
// the bars: within the reach of its matches a lens is recovered to 2 px (over 30 seeds of these scenes
// the 90th percentile is 1.25 px for one camera, 1.69 px for one beside a held lens; 2 px is 1.2% of the
// 164 px lens and far under a fit that fails, 3.4 px and more), except the strong lens fitted from cross
// pairs alone, whose second unknown lens leaves 3.35 median, 3.80 p90 in the prototype: 5 px for it
constexpr REAL LENS_TEST_MAX_ERROR_PX = REAL(2);
// the focal check's bar: a focal the check moved lands within 1% of the true one
constexpr REAL LENS_TEST_FOCAL_MAX_ERROR = REAL(0.01);
// The import's default focal of a camera with no EXIF focal, a ratio of its long side (ImportConfig::defaultFocalRatio)
constexpr REAL LENS_TEST_DEFAULT_FOCAL_RATIO = REAL(1.2);
// The share of its sparse inliers a pair keeps when verified again while one of its cameras still has an unknown focal
constexpr double LENS_TEST_MIN_INLIER_SHARE = 0.9;
// The focal camera B is given in LensFitFocalTest's estimated case, a scale of the true one: 4% short, where the free
// fit lowers the median error 8% (0.371 -> 0.340 px), under the bar a trusted focal is held to
constexpr REAL LENS_TEST_ESTIMATED_FOCAL_SCALE = REAL(0.96);
constexpr REAL LENS_TEST_CROSS_MAX_ERROR_PX = REAL(5);
// The scene of LensFitRecoveryTest's slow case, and the rounds of pose re-estimation its fit takes more
// than: its first joint fit reaches 63 px of the 164 at the corner, 18 of its 40 pairs keeping their
// pinhole baselines reversed, and the rounds then replace 2, 4, 6 and 3 poses before one replaces none
// (5 rounds; 3 leave the lens 6.7 px off within reach). Over 100 seeds of this scene no fit that ends
// takes more than 5 rounds
constexpr unsigned LENS_TEST_SLOW_SEED = 32;
constexpr unsigned LENS_TEST_SLOW_MIN_ROUNDS = 3;
// The scenes of LensFitRecoveryTest's fits held short from no lens: camera A's, whose first joint fit stops
// at 52 px of the 164 at the corner, 19 of its 40 pairs keeping their pinhole baselines more than 90 deg
// off and each then explained best by its pose at that lens, so that no round of pose re-estimation
// replaces one (86 px off within reach); and the pincushion's, whose first joint fit stops at 70 px of the
// 160 (71 px off). Each fails the check, and the fit started again from its lens doubled reaches the
// lens
constexpr unsigned LENS_TEST_HELD_SEED = 14;
constexpr unsigned LENS_TEST_PINCUSHION_SEED = 7;

// A rotation of `deg` degrees about `axis` (normalized here), Rodrigues
Matrix3x3 LensTestRotation(Point3 axis, REAL deg)
{
	axis = normalized(axis);
	const REAL a = D2R(deg), c = COS(a), s = SIN(a), t = REAL(1) - c;
	return Matrix3x3(
		t*axis.x*axis.x + c,        t*axis.x*axis.y - s*axis.z, t*axis.x*axis.z + s*axis.y,
		t*axis.x*axis.y + s*axis.z, t*axis.y*axis.y + c,        t*axis.y*axis.z - s*axis.x,
		t*axis.x*axis.z - s*axis.y, t*axis.y*axis.z + s*axis.x, t*axis.z*axis.z + c);
}

// A synthetic capture whose lenses only its pairs can tell: for every entry of `pairs`, `count`
// two-view setups of their own (two images each), LENS_TEST_POINTS points 3-15 m in front of the
// first view projected into both through the TRUE lenses, with `noisePx` of Gaussian noise and
// `outlierShare` of the second view's positions replaced by random ones. The scene's cameras carry
// the true focal times `givenFocalScale` (1: the true focal) and the true centre, trust them and have
// NO lens, and every pair is verified through that pinhole exactly as the matching verifies it
// (PairsMatcher::GeometricFilter at the default MatchConfig); a pair that fails is not stored. A
// stored pair weighs its inlier count (unit quality factors, triplet 0.75). truePoses receives each
// stored pair's true relative pose (x2 = R x1 + t), keyed by the pair's image IDs: neither the order
// the pairs are built in (a pair that fails leaves a gap) nor their position in scene.pairs
// (ComputePairsWeights sorts them) names a pair. `given`, where not empty, gives each camera its own focal scale and kind
// instead: an untrusted camera's pairs are verified through F, as the matching verifies them.
// Every draw is made from the raw output of the std::mt19937, whose sequence the standard fixes, and
// not through std::uniform_real_distribution or std::normal_distribution, whose algorithms are each
// standard library's own: so that every platform the suite runs on draws the same scene, to the
// rounding of its own log and cos. Each draw is a statement of its own, since C++ leaves the order in
// which a call's arguments are evaluated unspecified.
void BuildLensTestScene(Scene& scene, const std::vector<LensTestCamera>& cameras,
	const std::vector<LensTestPairs>& pairs, unsigned seed, std::unordered_map<PairIdx, Pose3D>& truePoses,
	REAL noisePx = LENS_TEST_NOISE_PX, REAL outlierShare = LENS_TEST_OUTLIER_SHARE, REAL givenFocalScale = REAL(1),
	const std::vector<LensTestGiven>& given = {})
{
	ASSERT(givenFocalScale > 0);
	ASSERT(given.empty() || given.size() == cameras.size()); // one entry per camera
	std::mt19937 rng(seed);
	// a uniform in [0, 1) at a double's precision: the top 27 bits of one output and the top 26 of the
	// next, a 53-bit integer scaled by 2^-53
	const auto Unit = [&]() {
		const uint64_t high = rng() >> 5;
		const uint64_t low = rng() >> 6;
		return std::ldexp((double)((high << 26) | low), -53);
	};
	const auto Uniform = [&](double a, double b) { return a + (b - a)*Unit(); };
	const auto Sign = [&]() { return Unit() < 0.5 ? -1.0 : 1.0; };
	// a standard normal by Box-Muller, its first uniform taken in (0, 1] so that its log is finite
	const auto Gauss = [&]() {
		const double u1 = 1.0 - Unit();
		const double u2 = Unit();
		return SQRT(-2.0*std::log(u1))*COS(2.0*M_PI*u2);
	};
	std::vector<PinholeCamera> truth;
	FOREACH(i, cameras) {
		const LensTestCamera& c = cameras[i];
		const REAL scale = given.empty() ? givenFocalScale : given[i].focalScale;
		PinholeCamera* const camera = new PinholeCamera(cv::Size(c.width, c.height),
			c.focal*scale, c.focal*scale, REAL(c.width - 1)/2, REAL(c.height - 1)/2);
		camera->trustIntrinsics = given.empty() || given[i].trusted;
		scene.cameras.emplace_back(camera);
		truth.push_back(PinholeCamera(cv::Size(c.width, c.height),
			c.focal, c.focal, REAL(c.width - 1)/2, REAL(c.height - 1)/2));
		truth.back().trustIntrinsics = true;
		truth.back().SetDistortion(c.k1, c.k2, 0, 0);
	}
	const auto Inside = [](const PinholeCamera& camera, const Point2& x) {
		return x.x >= 0 && x.y >= 0 && x.x <= camera.size.width - 1 && x.y <= camera.size.height - 1;
	};
	MatchConfig matchCfg;
	PairsMatcher matcher(scene, matchCfg);
	for (const LensTestPairs& spec : pairs) {
		const PinholeCamera& trueA = truth[spec.cameraA];
		const PinholeCamera& trueB = truth[spec.cameraB];
		for (unsigned n = 0; n < spec.count; ++n) {
			// the first view turned a little about a random axis at the origin, the second placed by the motion
			const double axisX = Gauss();
			const double axisY = Gauss();
			const double axisZ = Gauss();
			const double angle = Uniform(0, 5);
			const Matrix3x3 R1(LensTestRotation(Point3(axisX, axisY, axisZ), angle));
			Matrix3x3 Rrel;
			Point3 Crel;
			if (spec.motion == LensTestMotion::TURN) {
				const double tiltX = 0.2*Gauss();
				const double tiltZ = 0.2*Gauss();
				const double turnSign = Sign();
				const double turn = Uniform(10, 30);
				const double sideSign = Sign();
				const double side = Uniform(0.2, 0.8);
				const double rise = 0.05*Gauss();
				const double advance = 0.1*Gauss();
				Rrel = LensTestRotation(Point3(tiltX, 1, tiltZ), turnSign*turn);
				Crel = Point3(sideSign*side, rise, advance);
			} else {
				const double rollSign = Sign();
				const double roll = Uniform(6, 10);
				const double advance = Uniform(0.5, 1.5);
				Rrel = LensTestRotation(Point3(0, 0, 1), rollSign*roll);
				Crel = Point3(0, 0, advance);
			}
			Pose3D pose1, pose2;
			pose1.R = R1;
			pose1.C = Point3(0, 0, 0);
			pose2.R = Rrel*R1;
			pose2.C = R1.t()*Crel;
			// the points, through the first view's frustum, and their projections through the true lenses
			const double spread = (double)trueA.size.width/trueA.fx/1.3;
			std::vector<Point2f> xA, xB;
			for (unsigned attempt = 0; attempt < LENS_TEST_ATTEMPTS && xA.size() < LENS_TEST_POINTS; ++attempt) {
				const double z = Uniform(3, 15);
				const double x = Uniform(-0.75, 0.75)*spread*z;
				const double y = Uniform(-0.45, 0.45)*spread*z;
				const Point3 Xc(x, y, z);
				const Point3 X(pose1.R.t()*Xc + pose1.C);
				const auto [pA, okA] = trueA.Project(pose1.R*(X - pose1.C));
				const auto [pB, okB] = trueB.Project(pose2.R*(X - pose2.C));
				if (!okA || !okB || !Inside(trueA, pA) || !Inside(trueB, pB))
					continue;
				const double noiseAx = noisePx*Gauss();
				const double noiseAy = noisePx*Gauss();
				const double noiseBx = noisePx*Gauss();
				const double noiseBy = noisePx*Gauss();
				xA.emplace_back((float)(pA.x + noiseAx), (float)(pA.y + noiseAy));
				xB.emplace_back((float)(pB.x + noiseBx), (float)(pB.y + noiseBy));
			}
			const size_t numOutliers = (size_t)(outlierShare*(REAL)xA.size());
			for (size_t k = 0; k < numOutliers; ++k) {
				const double x = Uniform(0, trueB.size.width - 1);
				const double y = Uniform(0, trueB.size.height - 1);
				xB[k] = Point2f((float)x, (float)y);
			}
			// two images of their own, with no pose: the fit reads the pairs alone
			const IIndex id1 = (IIndex)scene.images.size(), id2 = id1 + 1;
			scene.images.emplace_back(id1, String::FormatString("%u.jpg", id1), Pose3D(), (IIndex)spec.cameraA, scene.cameras[spec.cameraA]);
			scene.images.emplace_back(id2, String::FormatString("%u.jpg", id2), Pose3D(), (IIndex)spec.cameraB, scene.cameras[spec.cameraB]);
			Image& img1 = scene.images[id1];
			Image& img2 = scene.images[id2];
			img1.InvalidatePose();
			img2.InvalidatePose();
			ImagePair pair(id1, id2);
			FOREACH(k, xA) {
				img1.keypoints.emplace_back(xA[k].x, xA[k].y, 1.f);
				img2.keypoints.emplace_back(xB[k].x, xB[k].y, 1.f);
				pair.matches.emplace_back((uint32_t)k, (uint32_t)k);
			}
			if (!matcher.GeometricFilter(img1, img2, pair))
				continue;
			pair.weightSpatial = pair.weightConnectivity = 1.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightTriplet = 0.75f;
			truePoses.emplace(PairIdx(pair.ID1, pair.ID2), pose2 / pose1);
			scene.pairs.emplace_back(std::move(pair));
		}
	}
}

// The worst difference, pixels, between a fitted lens's undistortion displacement (at the camera's own
// focal) and the true one's (at the true focal) over the radii a camera's matched keypoints reach (their
// LENS_TEST_REACH_PERCENTILE quantile), at LENS_TEST_RADII radii from the centre
REAL LensCurveError(const Scene& scene, unsigned idxCamera, const LensTestCamera& lens)
{
	const PinholeCamera& camera = *static_cast<const PinholeCamera*>(scene.cameras[idxCamera]);
	std::vector<REAL> radii;
	for (const ImagePair& pair : scene.pairs) {
		for (unsigned k = 0; k < pair.GetNumFilteredInliers(); ++k) {
			const DMatch& m = pair.matches[k];
			for (const auto& [imageID, featureID] : {std::make_pair(pair.ID1, m.queryIdx), std::make_pair(pair.ID2, m.trainIdx)})
				if (scene.images[imageID].pCamera == scene.cameras[idxCamera])
					radii.push_back(norm(Point2(scene.images[imageID].keypoints[featureID].pt.x - camera.cx,
						scene.images[imageID].keypoints[featureID].pt.y - camera.cy)));
		}
	}
	ASSERT(!radii.empty());
	std::sort(radii.begin(), radii.end());
	const REAL reach = radii[(size_t)(LENS_TEST_REACH_PERCENTILE*(radii.size() - 1))];
	REAL worst = 0;
	for (unsigned i = 0; i <= LENS_TEST_RADII; ++i) {
		const REAL r = reach*i/LENS_TEST_RADII;
		worst = MAXF(worst, ABS(LensDisplacement(camera.fx, camera.k1, camera.k2, r) - LensDisplacement(lens.focal, lens.k1, lens.k2, r)));
	}
	return worst;
}
} // namespace

// A scene of a lens test and what the library made of it. The cases of a lens test are independent scenes and fits, so each runs on a
// thread of its own (RunConcurrently) and the checks, which end the test at the first failure, are made afterwards in the order of
// the cases, each with its own message.
struct LensTestFit {
	Scene scene;
	LensFitReport report;
	bool solved = false;          // CalibrateFocalsAndFitLenses' result
	std::vector<double> measured; // what the case took before or after the fit, for its checks

	explicit LensTestFit(unsigned numThreads) : scene(numThreads) {}
};
// A job builds its scene (with the given thread count, which the lens fit and the matcher read from the scene) and fits it. Contract: a
// job touches only the objects it creates itself and the library calls the lens tests make (BuildLensTestScene, FitLensesFromPairs,
// FitLensesAndReverifyPairs, CalibrateFocalsAndFitLenses), which keep no mutable static state, draw from no global random generator
// (every draw is from a std::mt19937 seeded in the job or from PoseLib's per-call seed) and write no file; a job makes no TEST_CHECK
// and captures no object another job writes.
typedef std::function<LensTestFit(unsigned numThreads)> LensTestJob;
static std::vector<LensTestFit> RunConcurrently(const std::vector<LensTestJob>& jobs)
{
	ASSERT(!jobs.empty());
	// the machine's cores are shared by the jobs, so that N cases do not each start as many threads as there are cores
	const unsigned numThreads = MAXF(1u, std::thread::hardware_concurrency()/(unsigned)jobs.size());
	const int numOpenCVThreads = cv::getNumThreads(); // ComputeRelativePoses sets it process-wide, from each scene's thread count
	#ifdef _USE_OPENMP
	const int numOpenMPThreads = omp_get_max_threads(); // a Scene sets it (here: when a fit is moved out of its job) to its own thread count
	#endif
	std::vector<std::future<LensTestFit>> futures;
	for (const LensTestJob& job : jobs)
		futures.push_back(std::async(std::launch::async, job, numThreads));
	std::vector<LensTestFit> fits;
	for (std::future<LensTestFit>& future : futures)
		fits.push_back(future.get());
	cv::setNumThreads(numOpenCVThreads);
	#ifdef _USE_OPENMP
	omp_set_num_threads(numOpenMPThreads);
	#endif
	return fits;
}
// the job of a scene of `cameras` built by BuildLensTestScene and fitted by FitLensesFromPairs (the cameras in `calibrated` held)
static LensTestJob FitJob(const std::vector<LensTestCamera>& cameras, const std::vector<LensTestPairs>& pairs, unsigned seed,
	REAL givenFocalScale = REAL(1), const std::unordered_set<size_t>& calibrated = {})
{
	return [=](unsigned numThreads) {
		LensTestFit fit(numThreads);
		std::unordered_map<PairIdx, Pose3D> truePoses;
		BuildLensTestScene(fit.scene, cameras, pairs, seed, truePoses, LENS_TEST_NOISE_PX, LENS_TEST_OUTLIER_SHARE, givenFocalScale);
		std::unordered_set<CameraPtr> calibratedCameras;
		for (const size_t idxCamera : calibrated)
			calibratedCameras.insert(fit.scene.cameras[idxCamera]);
		fit.report = FitLensesFromPairs(fit.scene, MatchConfig(), calibratedCameras);
		return fit;
	};
}

// The lens fit from pairs recovers the lenses that rendered them: one camera from 40 pairs that turn
// and move, to LENS_TEST_MAX_ERROR_PX over the radii its matches reach, the lens set on the camera;
// the same from a scene whose first joint fit falls far short of the lens, which the rounds of pose
// re-estimation reach after more than LENS_TEST_SLOW_MIN_ROUNDS of them, and from scenes whose
// first joint fit from no lens is held short of it, a barrel lens and a pincushion, which the check
// tells and the fit started again from the lens reached so far scaled reaches; a first fit that
// reaches its lens passes the check and runs no other start;
// two cameras with different lenses joined only by 40 cross pairs, each fitted, the weak lens to
// LENS_TEST_MAX_ERROR_PX and the strong one to LENS_TEST_CROSS_MAX_ERROR_PX
bool LensFitRecoveryTest()
{
	TestScope scope("LensFitRecoveryTest");
	const std::vector<LensTestFit> fits(RunConcurrently({
		FitJob({LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 40}}, 7),
		// one camera whose first joint fit falls far short of its lens: each round of pose re-estimation through the lens so far turns a
		// few more pairs right and takes the lens further, until a round replaces no pose
		FitJob({LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 40}}, LENS_TEST_SLOW_SEED),
		// one camera whose first joint fit from no lens is held short of its lens by the pairs whose pinhole poses it cannot turn: a
		// barrel lens, and a pincushion
		FitJob({LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 40}}, LENS_TEST_HELD_SEED),
		FitJob({LENS_TEST_PINCUSHION}, {{0, 0, LensTestMotion::TURN, 40}}, LENS_TEST_PINCUSHION_SEED),
		FitJob({LENS_TEST_A, LENS_TEST_B}, {{0, 1, LensTestMotion::TURN, 40}}, 11)}));
	{
		const LensTestFit& fit = fits[0];
		const LensFitOutcome& outcome = fit.report.cameras[0];
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		TEST_CHECK(outcome.status == LensFitOutcome::FITTED && camera.k1 == outcome.k1 && camera.k2 == outcome.k2 && camera.k3 == 0 && camera.p1 == 0 && camera.p2 == 0,
			"one camera, status %u, lens %g %g on the camera, median %.3f -> %.3f px over %u pairs",
			(unsigned)outcome.status, camera.k1, camera.k2, outcome.medianBefore, outcome.medianAfter, outcome.numPairs);
		const REAL error = LensCurveError(fit.scene, 0, LENS_TEST_A);
		TEST_CHECK(error <= LENS_TEST_MAX_ERROR_PX, "one camera, the fitted lens (%g, %g) is %.2f px off the true one within its matches' reach", camera.k1, camera.k2, error);
		TEST_CHECK(fit.report.numStarts == 1, "one camera, its fit ran %u starts where its first passes the check", fit.report.numStarts);
		VERBOSE("LensFitRecoveryTest: one camera, k1 %.4f k2 %.4f (true -0.20 0.05), %.2f px off within reach, median %.3f -> %.3f px, %.1f s",
			camera.k1, camera.k2, error, outcome.medianBefore, outcome.medianAfter, fit.report.seconds);
	}
	{
		const LensTestFit& fit = fits[1];
		const LensFitOutcome& outcome = fit.report.cameras[0];
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		TEST_CHECK(outcome.status == LensFitOutcome::FITTED, "one camera recovered slowly, status %u, median %.3f -> %.3f px over %u pairs",
			(unsigned)outcome.status, outcome.medianBefore, outcome.medianAfter, outcome.numPairs);
		const REAL error = LensCurveError(fit.scene, 0, LENS_TEST_A);
		TEST_CHECK(error <= LENS_TEST_MAX_ERROR_PX, "one camera recovered slowly, the fitted lens (%g, %g) is %.2f px off the true one within its matches' reach after %u rounds of pose re-estimation",
			camera.k1, camera.k2, error, (unsigned)fit.report.reestimatedPoses.size());
		TEST_CHECK(fit.report.reestimatedPoses.size() > LENS_TEST_SLOW_MIN_ROUNDS, "one camera recovered slowly, its fit took only %u rounds of pose re-estimation, the fixture does not exercise them",
			(unsigned)fit.report.reestimatedPoses.size());
		VERBOSE("LensFitRecoveryTest: one camera recovered slowly, %.2f px off within reach after %u rounds of pose re-estimation, %.1f s",
			error, (unsigned)fit.report.reestimatedPoses.size(), fit.report.seconds);
	}
	for (const bool pincushion : {false, true}) {
		const LensTestFit& fit = fits[2 + pincushion];
		const char* const name = pincushion ? "a pincushion held short from no lens" : "one camera held short from no lens";
		const LensTestCamera& lens = pincushion ? LENS_TEST_PINCUSHION : LENS_TEST_A;
		const LensFitOutcome& outcome = fit.report.cameras[0];
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		TEST_CHECK(outcome.status == LensFitOutcome::FITTED, "%s, status %u, median %.3f -> %.3f px over %u pairs", name, (unsigned)outcome.status,
			outcome.medianBefore, outcome.medianAfter, outcome.numPairs);
		const REAL error = LensCurveError(fit.scene, 0, lens);
		TEST_CHECK(error <= LENS_TEST_MAX_ERROR_PX, "%s, the fitted lens (%g, %g) is %.2f px off the true one within its matches' reach", name, camera.k1, camera.k2, error);
		TEST_CHECK(fit.report.numStarts >= 2, "%s, its fit ran %u start, the check let the first through", name, fit.report.numStarts);
		VERBOSE("LensFitRecoveryTest: %s, k1 %.4f k2 %.4f (true %.4f %.4f), %.2f px off within reach, %.1f s", name, camera.k1, camera.k2, lens.k1, lens.k2, error, fit.report.seconds);
	}
	{
		const LensTestFit& fit = fits[4];
		TEST_CHECK(fit.report.cameras[0].status == LensFitOutcome::FITTED && fit.report.cameras[1].status == LensFitOutcome::FITTED, "two cameras, statuses %u %u",
			(unsigned)fit.report.cameras[0].status, (unsigned)fit.report.cameras[1].status);
		const REAL errorA = LensCurveError(fit.scene, 0, LENS_TEST_A), errorB = LensCurveError(fit.scene, 1, LENS_TEST_B);
		TEST_CHECK(errorA <= LENS_TEST_CROSS_MAX_ERROR_PX && errorB <= LENS_TEST_MAX_ERROR_PX, "two cameras joined by cross pairs only, %.2f and %.2f px off the true lenses within reach", errorA, errorB);
		VERBOSE("LensFitRecoveryTest: two cameras from cross pairs only, %.2f and %.2f px off within reach", errorA, errorB);
	}
	return scope.Pass("one camera, the slow and the held-short scenes, two cameras from cross pairs");
}

// The focal check of the lens fit corrects a trusted focal its pairs clearly reject and leaves a right
// one exactly as it is: the strong barrel lens of camera A given a focal 20% short and 20% long of the
// one its pairs were rendered with, and a camera without a lens given one 15% short, each take a focal
// within LENS_TEST_FOCAL_MAX_ERROR of the true one, the lens fitted at it within LENS_TEST_MAX_ERROR_PX
// of the true one over its matches' reach; so do the scenes whose fit from no lens is held short of the
// lens (LENS_TEST_PINCUSHION_SEED's pincushion given a focal 15% short and 15% long, LENS_TEST_HELD_SEED's
// barrel given one 15% short), whose held fit fails the check of a fit until it restarts, and that barrel
// given a focal 20% short, whose free fit from no lens stalls 16% short while its held fit passes the
// check; camera B's mild lens given a focal 4% short keeps it while it is trusted (the free fit lowers the
// median error under lensFitFocalMinErrorDrop) and takes the free fit's within the same bars where it is
// an untrusted start (an estimate); given its true focal, the camera keeps it bit for bit and the lens
// comes out as true within the same bar
bool LensFitFocalTest()
{
	TestScope scope("LensFitFocalTest");
	struct Variant {
		const char* name;
		LensTestCamera camera;
		REAL givenFocalScale;
		unsigned seed;
	};
	const Variant moved[] = {
		{"a strong lens, the focal 20% short", LENS_TEST_A, REAL(0.8), 7},
		{"a strong lens, the focal 20% long", LENS_TEST_A, REAL(1.2), 7},
		{"no lens, the focal 15% short", LENS_TEST_P, REAL(0.85), 7},
		{"a pincushion held short from no lens, the focal 15% short", LENS_TEST_PINCUSHION, REAL(0.85), LENS_TEST_PINCUSHION_SEED},
		{"a pincushion held short from no lens, the focal 15% long", LENS_TEST_PINCUSHION, REAL(1.15), LENS_TEST_PINCUSHION_SEED},
		{"a strong lens held short from no lens, the focal 15% short", LENS_TEST_A, REAL(0.85), LENS_TEST_HELD_SEED},
		{"a strong lens whose free fit from no lens stalls, the focal 20% short", LENS_TEST_A, REAL(0.8), LENS_TEST_HELD_SEED},
	};
	constexpr size_t numMoved = sizeof(moved)/sizeof(moved[0]);
	std::vector<LensTestJob> jobs;
	for (const Variant& variant : moved)
		jobs.push_back(FitJob({variant.camera}, {{0, 0, LensTestMotion::TURN, 40}}, variant.seed, variant.givenFocalScale));
	// camera B's mild lens given a focal 4% short: trusted, the focal is a prior the bar protects, and the free fit's median error falls
	// 8% (under lensFitFocalMinErrorDrop), so it is kept; an untrusted start, an unknown, the check takes the free fit's focal and the
	// lens is fitted at it
	jobs.push_back(FitJob({LENS_TEST_B}, {{0, 0, LensTestMotion::TURN, 40}}, 7, LENS_TEST_ESTIMATED_FOCAL_SCALE));
	jobs.push_back(FitJob({LENS_TEST_B}, {{0, 0, LensTestMotion::TURN, 40}}, 7, LENS_TEST_ESTIMATED_FOCAL_SCALE, {0}));
	// the true focal given
	jobs.push_back(FitJob({LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 40}}, 7));
	const std::vector<LensTestFit> fits(RunConcurrently(jobs));
	for (size_t i = 0; i < numMoved; ++i) {
		const Variant& variant = moved[i];
		const LensTestFit& fit = fits[i];
		const LensFitOutcome& outcome = fit.report.cameras[0];
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		const REAL focalError = ABS(camera.fx/variant.camera.focal - REAL(1));
		TEST_CHECK(outcome.focalMoved && camera.fx == camera.fy && focalError <= LENS_TEST_FOCAL_MAX_ERROR,
			"%s, focal %s %g -> %g (true %g, %.2f%% off), median %.3f held, %.3f free", variant.name, outcome.focalMoved ? "moved" : "kept",
			outcome.focalBefore, camera.fx, variant.camera.focal, 100*focalError, outcome.medianFocalHeld, outcome.medianFocalFree);
		REAL lensError = 0;
		if (variant.camera.k1 != 0 || variant.camera.k2 != 0) {
			TEST_CHECK(outcome.status == LensFitOutcome::FITTED, "%s, lens status %u at the moved focal", variant.name, (unsigned)outcome.status);
			lensError = LensCurveError(fit.scene, 0, variant.camera);
			TEST_CHECK(lensError <= LENS_TEST_MAX_ERROR_PX, "%s, the lens (%g, %g) at the moved focal %g is %.2f px off the true one",
				variant.name, camera.k1, camera.k2, camera.fx, lensError);
		}
		VERBOSE("LensFitFocalTest: %s, focal %g -> %g (true %g, %.2f%% off), lens %.2f px off, median %.3f held, %.3f free, %.1f s",
			variant.name, outcome.focalBefore, camera.fx, variant.camera.focal, 100*focalError, lensError,
			outcome.medianFocalHeld, outcome.medianFocalFree, fit.report.seconds);
	}
	for (const bool estimated : {false, true}) {
		const LensTestFit& fit = fits[numMoved + estimated];
		const LensFitOutcome& outcome = fit.report.cameras[0];
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		const REAL focalError = ABS(camera.fx/LENS_TEST_B.focal - REAL(1));
		const REAL lensError = outcome.status == LensFitOutcome::FITTED ? LensCurveError(fit.scene, 0, LENS_TEST_B) : REAL(0);
		const bool right = estimated ?
			outcome.focalMoved && camera.fx == camera.fy && focalError <= LENS_TEST_FOCAL_MAX_ERROR &&
				outcome.status == LensFitOutcome::FITTED && lensError <= LENS_TEST_MAX_ERROR_PX :
			!outcome.focalMoved && camera.fx == LENS_TEST_ESTIMATED_FOCAL_SCALE*LENS_TEST_B.focal;
		TEST_CHECK(right, "a mild lens given a focal 4%% short, %s: focal %s %g -> %g (true %g, %.2f%% off), lens status %u, %.2f px off, median %.3f held, %.3f free",
			estimated ? "estimated" : "trusted", outcome.focalMoved ? "moved" : "kept", outcome.focalBefore, camera.fx,
			LENS_TEST_B.focal, 100*focalError, (unsigned)outcome.status, lensError, outcome.medianFocalHeld, outcome.medianFocalFree);
		VERBOSE("LensFitFocalTest: a mild lens given a focal 4%% short, %s: focal %s %g -> %g (true %g, %.2f%% off), lens %.2f px off, median %.3f held, %.3f free",
			estimated ? "estimated" : "trusted", outcome.focalMoved ? "moved" : "kept", outcome.focalBefore, camera.fx,
			LENS_TEST_B.focal, 100*focalError, lensError, outcome.medianFocalHeld, outcome.medianFocalFree);
	}
	{
		// the true focal given: kept bit for bit, and the lens the true one
		const LensTestFit& fit = fits[numMoved + 2];
		const LensFitOutcome& outcome = fit.report.cameras[0];
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		const REAL lensError = outcome.status == LensFitOutcome::FITTED ? LensCurveError(fit.scene, 0, LENS_TEST_A) : REAL(0);
		TEST_CHECK(!outcome.focalMoved && camera.fx == LENS_TEST_A.focal && camera.fy == LENS_TEST_A.focal && outcome.focalBefore == LENS_TEST_A.focal &&
			outcome.status == LensFitOutcome::FITTED && lensError <= LENS_TEST_MAX_ERROR_PX,
			"the true focal, %s, focal %g, lens status %u, %.2f px off the true one, median %.3f held, %.3f free",
			outcome.focalMoved ? "moved" : "kept", camera.fx, (unsigned)outcome.status, lensError, outcome.medianFocalHeld, outcome.medianFocalFree);
		VERBOSE("LensFitFocalTest: the true focal kept (the free fit reached %g), lens %.2f px off, median %.3f held, %.3f free",
			outcome.focalFree, lensError, outcome.medianFocalHeld, outcome.medianFocalFree);
	}
	return scope.Pass("the seven moved-focal scenes, a mild lens given a focal 4%% short trusted and estimated, the true focal kept");
}

// The lens fit leaves a camera it cannot take as it is, and refuses what no lens explains:
// - refusals: pairs of a camera without a lens (its median error does not fall), forward pairs that hide a strong lens (the
//   acceptance refuses rather than keep a lens the pairs cannot tell, folding or not), and exact positions (the error is float
//   rounding), without a lens or through one too faint to matter, which the error drop alone would keep; every refused camera is left
//   without a lens;
// - a camera the fit cannot hold leaves it with every pair it has: one under the minimum of eligible pairs, one whose given lens has
//   terms past k1 and k2, one whose focal is not trusted and one whose given lens folds within its frame are each left as they are, and
//   the camera they pair with is fitted from its own pairs alone (those of the 30 it is built with that pass the fit's per-pair bars),
//   to LENS_TEST_MAX_ERROR_PX; a camera given its true lens as k1, k2 alone is held, its lens untouched, and its cross pairs, either
//   camera first, join the fitted camera's own
bool LensFitLimitsTest()
{
	TestScope scope("LensFitLimitsTest");
	struct Refusal {
		const char* name;
		LensTestCamera camera;
		LensTestMotion motion;
		REAL noisePx, outlierShare;
		bool mayFold; // refused as FOLDS is right too, not only as NO_ERROR_DROP
	};
	const Refusal refusals[] = {
		{"no lens", LENS_TEST_P, LensTestMotion::TURN, LENS_TEST_NOISE_PX, LENS_TEST_OUTLIER_SHARE, false},
		{"forward pairs", LENS_TEST_A, LensTestMotion::FORWARD, LENS_TEST_NOISE_PX, LENS_TEST_OUTLIER_SHARE, true},
		{"no noise", LENS_TEST_P, LensTestMotion::TURN, REAL(0), REAL(0), false},
		{"a faint lens", LENS_TEST_FAINT, LensTestMotion::TURN, REAL(0), REAL(0), false},
	};
	constexpr size_t numRefusals = sizeof(refusals)/sizeof(refusals[0]);
	struct Held {
		enum class Kind {
			CROSS_UNDER_MINIMUM, // camera B as built, with too few cross pairs to be fitted
			FULLER_LENS,         // camera B given a lens with a tangential term besides k1, k2
			FOLDING_LENS,        // camera B given a k1, k2 lens that folds within its frame
			UNTRUSTED,           // camera B's focal not trusted
			HELD_LENS            // camera B given its true lens, k1, k2 alone
		};
		const char* name;
		Kind kind;
		unsigned numCross, numCrossReversed; // cross pairs built with camera A's image first, and with camera B's
		LensFitOutcome::Status expected;     // camera B's
	};
	const Held helds[] = {
		{"under the minimum", Held::Kind::CROSS_UNDER_MINIMUM, 10, 0, LensFitOutcome::TOO_FEW_PAIRS},
		{"a fuller given lens", Held::Kind::FULLER_LENS, 30, 0, LensFitOutcome::EXCLUDED},
		{"a given lens that folds", Held::Kind::FOLDING_LENS, 30, 0, LensFitOutcome::EXCLUDED},
		{"an untrusted focal", Held::Kind::UNTRUSTED, 30, 0, LensFitOutcome::EXCLUDED},
		{"a held lens", Held::Kind::HELD_LENS, 15, 15, LensFitOutcome::HELD},
	};
	const MatchConfig config;
	std::vector<LensTestJob> jobs;
	for (const Refusal& refusal : refusals)
		jobs.push_back([=](unsigned numThreads) {
			LensTestFit fit(numThreads);
			std::unordered_map<PairIdx, Pose3D> truePoses;
			BuildLensTestScene(fit.scene, {refusal.camera}, {{0, 0, refusal.motion, 30}}, 13, truePoses, refusal.noisePx, refusal.outlierShare);
			fit.report = FitLensesFromPairs(fit.scene, MatchConfig());
			return fit;
		});
	for (const Held& held : helds)
		jobs.push_back([=](unsigned numThreads) {
			LensTestFit fit(numThreads);
			Scene& scene = fit.scene;
			std::unordered_map<PairIdx, Pose3D> truePoses;
			BuildLensTestScene(scene, {LENS_TEST_A, LENS_TEST_B}, {{0, 0, LensTestMotion::TURN, 30},
				{0, 1, LensTestMotion::TURN, held.numCross}, {1, 0, LensTestMotion::TURN, held.numCrossReversed}}, 17, truePoses);
			PinholeCamera& cameraB = *static_cast<PinholeCamera*>(scene.cameras[1]);
			switch (held.kind) {
			case Held::Kind::FULLER_LENS:
				cameraB.SetDistortion(LENS_TEST_B.k1, LENS_TEST_B.k2, REAL(1e-4), 0);
				break;
			case Held::Kind::FOLDING_LENS:
				cameraB.SetDistortion(LENS_TEST_FOLDING_K1, 0, 0, 0);
				break;
			case Held::Kind::UNTRUSTED:
				cameraB.trustIntrinsics = false;
				break;
			case Held::Kind::HELD_LENS:
				cameraB.SetDistortion(LENS_TEST_B.k1, LENS_TEST_B.k2, 0, 0);
				break;
			default:
				break;
			}
			// camera A's pairs the fit can read, its own and those across to camera B in either order: the pinhole verification leaves
			// some under MatchConfig::lensFitMatchesPerPair sparse inliers through the strong lens
			unsigned numOwnPairs = 0, numCrossAFirst = 0, numCrossBFirst = 0;
			for (const ImagePair& pair : scene.pairs) {
				if (!pair.relativePose.has_value() || !pair.HasValidWeight() || pair.GetNumFilteredInliers() < MatchConfig().lensFitMatchesPerPair)
					continue;
				const REAL angle = RotationErrorDeg(pair.relativePose->R, RMatrix::IDENTITY);
				if (angle < MatchConfig().lensFitMinRotation || angle > MatchConfig().lensFitMaxRotation)
					continue;
				const bool onA1 = scene.images[pair.ID1].pCamera == scene.cameras[0];
				const bool onA2 = scene.images[pair.ID2].pCamera == scene.cameras[0];
				if (onA1 && onA2)
					++numOwnPairs;
				else if (onA1)
					++numCrossAFirst;
				else if (onA2)
					++numCrossBFirst;
			}
			fit.measured = {(double)numOwnPairs, (double)numCrossAFirst, (double)numCrossBFirst, cameraB.k1, cameraB.k2, cameraB.p1};
			fit.report = FitLensesFromPairs(scene, MatchConfig());
			return fit;
		});
	const std::vector<LensTestFit> fits(RunConcurrently(jobs));
	for (size_t i = 0; i < numRefusals; ++i) {
		const Refusal& variant = refusals[i];
		const LensTestFit& fit = fits[i];
		const LensFitOutcome& outcome = fit.report.cameras[0];
		TEST_CHECK(outcome.numEligiblePairs >= config.lensFitMinPairs, "%s, only %u eligible pairs, the fixture does not reach the fit", variant.name, outcome.numEligiblePairs);
		TEST_CHECK((outcome.status == LensFitOutcome::NO_ERROR_DROP || (variant.mayFold && outcome.status == LensFitOutcome::FOLDS)) && !fit.scene.cameras[0]->HasDistortion(),
			"%s, status %u, lens %g %g kept, median %.4g -> %.4g px", variant.name, (unsigned)outcome.status, outcome.k1, outcome.k2, outcome.medianBefore, outcome.medianAfter);
		VERBOSE("LensFitLimitsTest: %s refused (status %u), median %.4g -> %.4g px", variant.name, (unsigned)outcome.status, outcome.medianBefore, outcome.medianAfter);
	}
	for (size_t i = 0; i < sizeof(helds)/sizeof(helds[0]); ++i) {
		const Held& variant = helds[i];
		const LensTestFit& fit = fits[numRefusals + i];
		const unsigned numOwnPairs = (unsigned)fit.measured[0], numCrossAFirst = (unsigned)fit.measured[1], numCrossBFirst = (unsigned)fit.measured[2];
		const PinholeCamera& cameraB = *static_cast<const PinholeCamera*>(fit.scene.cameras[1]);
		const LensFitOutcome& a = fit.report.cameras[0];
		const LensFitOutcome& b = fit.report.cameras[1];
		TEST_CHECK(numOwnPairs >= config.lensFitMinPairs, "%s, camera A has only %u eligible pairs of its own, the fixture does not reach the fit", variant.name, numOwnPairs);
		const bool held = variant.kind == Held::Kind::HELD_LENS;
		TEST_CHECK(!held || (numCrossAFirst != 0 && numCrossBFirst != 0), "%s, %u and %u eligible cross pairs with camera A's and camera B's image first, the fixture does not reach both orders",
			variant.name, numCrossAFirst, numCrossBFirst);
		const unsigned numExpectedPairs = numOwnPairs + (held ? numCrossAFirst + numCrossBFirst : 0u);
		TEST_CHECK(b.status == variant.expected && cameraB.k1 == fit.measured[3] && cameraB.k2 == fit.measured[4] && cameraB.p1 == fit.measured[5],
			"%s, camera B status %u (expected %u), lens %g %g %g; camera A status %u over %u pairs",
			variant.name, (unsigned)b.status, (unsigned)variant.expected, cameraB.k1, cameraB.k2, cameraB.p1, (unsigned)a.status, a.numPairs);
		TEST_CHECK(a.status == LensFitOutcome::FITTED && a.numPairs == numExpectedPairs, "%s, camera A status %u over %u pairs, expected fitted over %u (%u of its own)",
			variant.name, (unsigned)a.status, a.numPairs, numExpectedPairs, numOwnPairs);
		const REAL error = LensCurveError(fit.scene, 0, LENS_TEST_A);
		TEST_CHECK(error <= LENS_TEST_MAX_ERROR_PX, "%s, camera A %.2f px off its true lens within reach", variant.name, error);
		VERBOSE("LensFitLimitsTest: %s, camera B status %u, camera A fitted over %u pairs (%u of its own), k1 %.4f k2 %.4f, %.2f px off within reach",
			variant.name, (unsigned)b.status, a.numPairs, numOwnPairs, a.k1, a.k2, error);
	}
	return scope.Pass("four refusals and five cameras the fit cannot hold left as they are, the camera beside them fitted");
}

// The lens stage's bars: every pair verified again lands within LENS_STAGE_MAX_ROTATION_DEG of its true rotation and, in the median,
// LENS_STAGE_MAX_DIRECTION_DEG of its true baseline direction
constexpr REAL LENS_STAGE_MAX_ROTATION_DEG = REAL(0.5);
constexpr REAL LENS_STAGE_MAX_DIRECTION_DEG = REAL(0.5);
constexpr REAL LENS_STAGE_MIN_PINHOLE_ERROR_DEG = REAL(1); // the fixture: the pinhole's median rotation error must reach this
// The rotation error and the baseline direction error, degrees, of every pair of the scene against its true pose (named by its images:
// ComputeRelativePoses recomputes the pair weights, which sorts scene.pairs, and BuildLensTestScene stores only the pairs its verification
// passes): the median rotation error, the worst rotation error and the median direction error; false where a pair has no pose
static bool LensStageErrors(const Scene& scene, const std::unordered_map<PairIdx, Pose3D>& truePoses, REAL& medianRotation, REAL& worstRotation, REAL& medianDirection)
{
	ASSERT(!scene.pairs.empty());
	std::vector<REAL> rotation, direction;
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value())
			return false;
		const Pose3D& truePose = truePoses.at(PairIdx(pair.ID1, pair.ID2));
		rotation.push_back(RotationErrorDeg(pair.relativePose->R, truePose.R));
		direction.push_back(R2D(ACOS(CLAMP(normalized(pair.relativePose->GetT()).dot(normalized(truePose.GetT())), REAL(-1), REAL(1)))));
	}
	const auto Median = [](std::vector<REAL>& values) {
		std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
		return values[values.size()/2];
	};
	worstRotation = *std::max_element(rotation.begin(), rotation.end());
	medianRotation = Median(rotation);
	medianDirection = Median(direction);
	return true;
}

// The lens stage of the matching (FitLensesAndReverifyPairs) verifies the pairs of a camera again at what it fitted:
// - the lens: on 40 pairs of camera A verified through the pinhole, whose rotations come out degrees off, the lens is fitted and set
//   and every pair verified again through it lands within LENS_STAGE_MAX_ROTATION_DEG of its true rotation and, in the median,
//   LENS_STAGE_MAX_DIRECTION_DEG of its true baseline direction; a second run, the camera now carrying its lens, holds it and fits
//   nothing;
// - the focal, even where no lens is kept at it: a camera without a lens, given a focal 15% short of the one its 40 pairs were
//   rendered with, has them verified through the short focal (rotations degrees off); the focal check moves its focal to the true one
//   and the fit keeps no lens (there is none), and the pairs then come out within the same bars.
bool LensFitStageTest()
{
	TestScope scope("LensFitStageTest");
	REAL pinholeRotation, worstRotation, pinholeDirection, medianDirection, unused;
	{
		Scene scene;
		std::unordered_map<PairIdx, Pose3D> truePoses;
		BuildLensTestScene(scene, {LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 40}}, 23, truePoses);
		TEST_CHECK(LensStageErrors(scene, truePoses, pinholeRotation, unused, pinholeDirection) && pinholeRotation >= LENS_STAGE_MIN_PINHOLE_ERROR_DEG,
			"through the pinhole the pairs are only %.2f deg off (median), the fixture does not exercise the lens", pinholeRotation);
		MatchConfig config;
		PairsMatcher matcher(scene, config);
		const LensFitReport report = FitLensesAndReverifyPairs(matcher);
		TEST_CHECK(report.cameras[0].status == LensFitOutcome::FITTED && scene.cameras[0]->HasDistortion(), "the lens was not fitted (status %u)", (unsigned)report.cameras[0].status);
		TEST_CHECK(LensStageErrors(scene, truePoses, unused, worstRotation, medianDirection), "a pair lost its pose when verified again through the lens");
		TEST_CHECK(worstRotation <= LENS_STAGE_MAX_ROTATION_DEG && medianDirection <= LENS_STAGE_MAX_DIRECTION_DEG,
			"through the fitted lens the pairs are up to %.2f deg off in rotation, %.2f deg in baseline direction (median)", worstRotation, medianDirection);
		const LensFitReport again = FitLensesAndReverifyPairs(matcher);
		TEST_CHECK(again.cameras[0].status == LensFitOutcome::HELD && again.FittedCameras().empty(), "a second run fitted the camera again (status %u)", (unsigned)again.cameras[0].status);
		VERBOSE("LensFitStageTest: rotation median %.2f -> worst %.2f deg, baseline direction median %.2f -> %.2f deg", pinholeRotation, worstRotation, pinholeDirection, medianDirection);
	}
	{
		Scene scene;
		std::unordered_map<PairIdx, Pose3D> truePoses;
		BuildLensTestScene(scene, {LENS_TEST_P}, {{0, 0, LensTestMotion::TURN, 40}}, 7, truePoses,
			LENS_TEST_NOISE_PX, LENS_TEST_OUTLIER_SHARE, REAL(0.85));
		TEST_CHECK(LensStageErrors(scene, truePoses, pinholeRotation, unused, unused) && pinholeRotation >= LENS_STAGE_MIN_PINHOLE_ERROR_DEG,
			"through the short focal the pairs are only %.2f deg off (median), the fixture does not exercise the focal", pinholeRotation);
		size_t numInliersBefore = 0;
		for (const ImagePair& pair : scene.pairs)
			numInliersBefore += pair.GetNumFilteredInliers();
		MatchConfig config;
		PairsMatcher matcher(scene, config);
		const LensFitReport report = FitLensesAndReverifyPairs(matcher);
		const LensFitOutcome& outcome = report.cameras[0];
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(scene.cameras[0]);
		const REAL focalError = ABS(camera.fx/LENS_TEST_P.focal - REAL(1));
		TEST_CHECK(outcome.focalMoved && outcome.status != LensFitOutcome::FITTED && !camera.HasDistortion() && focalError <= LENS_TEST_FOCAL_MAX_ERROR,
			"focal %s to %g (true %g, %.2f%% off), lens status %u: the fixture needs the focal moved and no lens kept",
			outcome.focalMoved ? "moved" : "kept", camera.fx, LENS_TEST_P.focal, 100*focalError, (unsigned)outcome.status);
		TEST_CHECK(LensStageErrors(scene, truePoses, unused, worstRotation, medianDirection), "a pair lost its pose when verified again at the moved focal");
		size_t numInliersAfter = 0;
		for (const ImagePair& pair : scene.pairs)
			numInliersAfter += pair.GetNumFilteredInliers();
		TEST_CHECK(worstRotation <= LENS_STAGE_MAX_ROTATION_DEG && medianDirection <= LENS_STAGE_MAX_DIRECTION_DEG,
			"at the moved focal %g the pairs are up to %.2f deg off in rotation (%.2f deg median through the short focal %g), %.2f deg in baseline direction (median); sparse inliers %zu -> %zu",
			camera.fx, worstRotation, pinholeRotation, outcome.focalBefore, medianDirection, numInliersBefore, numInliersAfter);
		VERBOSE("LensFitStageTest: focal %g -> %g, rotation median %.2f -> worst %.2f deg, baseline direction median %.2f deg, sparse inliers %zu -> %zu",
			outcome.focalBefore, camera.fx, pinholeRotation, worstRotation, medianDirection, numInliersBefore, numInliersAfter);
	}
	return scope.Pass("the lens fitted and held on a second run, the moved focal verifying the pairs again within %.1f deg", LENS_STAGE_MAX_ROTATION_DEG);
}

// A camera that does not trust its focal is fitted with its lens straight from the focal it was imported with by the focal
// and lens stage (CalibrateFocalsAndFitLenses): trusted at it, its pairs posed from their F there, its focal free in the lens
// fit with no bar, then its pairs verified through E at the focal and lens fitted. Camera A's strong barrel given a focal 20%
// short (an EXIF 35 mm-equivalent focal of the full sensor on a cropped frame) and given the import's default 1.2 times its
// long side (65% long) is fitted from that start to LENS_TEST_FOCAL_MAX_ERROR, the lens to LENS_TEST_MAX_ERROR_PX; beside a
// trusted camera A, an untrusted camera B given 10% short is fitted from its start while A keeps its focal bit for bit; a
// camera the lens fit cannot take (fewer pairs than lensFitMinPairs), and every untrusted camera when the lens fit is off, goes
// through the view-graph calibration and then through E at its focal, so that none is left at an unverified start
bool LensFitUntrustedFocalTest()
{
	TestScope scope("LensFitUntrustedFocalTest");
	// every pair that holds matches has a relative pose and an essential matrix: verified through E
	const auto VerifiedThroughE = [](const Scene& scene) {
		for (const ImagePair& pair : scene.pairs)
			if (pair.HasMatches() && (!pair.relativePose.has_value() || !pair.E.has_value()))
				return false;
		return true;
	};
	// the case jobs: the scene of cameras given their focal scale and kind, its calibration and, in the third, the share of the
	// inliers of each pair of A and B the calibration kept
	const auto Calibrate = [](const std::vector<LensTestCamera>& cameras, const std::vector<LensTestPairs>& pairs, unsigned seed,
		const std::vector<LensTestGiven>& given, bool lensFitEnabled) -> LensTestJob {
		return [=](unsigned numThreads) {
			LensTestFit fit(numThreads);
			std::unordered_map<PairIdx, Pose3D> truePoses;
			BuildLensTestScene(fit.scene, cameras, pairs, seed, truePoses, LENS_TEST_NOISE_PX, LENS_TEST_OUTLIER_SHARE, REAL(1), given);
			std::unordered_map<PairIdx, unsigned> crossInliers; // the sparse inliers of each pair of A and B, as the matching verified them
			for (const ImagePair& pair : fit.scene.pairs)
				if (fit.scene.images[pair.ID1].pCamera != fit.scene.images[pair.ID2].pCamera)
					crossInliers.emplace(PairIdx(pair.ID1, pair.ID2), pair.GetNumFilteredInliers());
			MatchConfig config;
			config.lensFitEnabled = lensFitEnabled;
			PairsMatcher matcher(fit.scene, config);
			fit.solved = CalibrateFocalsAndFitLenses(matcher, ViewGraphCalibratorConfig(), &fit.report);
			double worstShare = 1;
			for (const auto& [idPair, numBefore] : crossInliers) {
				unsigned numAfter = 0;
				for (const ImagePair& pair : fit.scene.pairs)
					if (PairIdx(pair.ID1, pair.ID2) == idPair)
						numAfter = pair.GetNumFilteredInliers();
				worstShare = MINF(worstShare, (double)numAfter/numBefore);
			}
			fit.measured = {worstShare};
			return fit;
		};
	};
	const REAL scales[] = {REAL(0.8), LENS_TEST_DEFAULT_FOCAL_RATIO*(REAL)LENS_TEST_A.width/LENS_TEST_A.focal};
	const REAL scaleB = LENS_TEST_DEFAULT_FOCAL_RATIO*(REAL)LENS_TEST_B.width/LENS_TEST_B.focal;
	ASSERT(10 < MatchConfig().lensFitMinPairs); // the fixture of the lens-fit-off case holds fewer pairs than the lens fit needs
	const std::vector<LensTestFit> fits(RunConcurrently({
		// camera A's strong barrel from 40 turning pairs, given a focal 20% short, then the import's default
		Calibrate({LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 40}}, 7, {{scales[0], false}}, true),
		Calibrate({LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 40}}, 7, {{scales[1], false}}, true),
		// the trusted camera A beside the untrusted camera B given 10% short, each with its own turning pairs and 40 cross pairs
		Calibrate({LENS_TEST_A, LENS_TEST_B}, {{0, 0, LensTestMotion::TURN, 40}, {1, 1, LensTestMotion::TURN, 40}, {0, 1, LensTestMotion::TURN, 40}}, 11,
			{{REAL(1), true}, {REAL(0.9), false}}, true),
		// an untrusted camera B the lens fit cannot take (6 pairs of its own and 6 with A, under lensFitMinPairs), given the import's default
		// focal (71% long), beside the trusted camera A whose lens it fits: the pairs with A, which the lens stage verifies again through
		// A's lens while B's focal is still unknown, keep their inliers until the calibration estimates B and its pairs are verified
		// through E there
		Calibrate({LENS_TEST_A, LENS_TEST_B}, {{0, 0, LensTestMotion::TURN, 40}, {1, 1, LensTestMotion::TURN, 6}, {0, 1, LensTestMotion::TURN, 6}}, 11,
			{{REAL(1), true}, {scaleB, false}}, true),
		// an untrusted camera the lens fit cannot take: too few pairs for it (10, under lensFitMinPairs), or the lens fit off
		Calibrate({LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 10}}, 7, {{REAL(0.85), false}}, true),
		Calibrate({LENS_TEST_A}, {{0, 0, LensTestMotion::TURN, 40}}, 7, {{REAL(0.85), false}}, false)}));
	for (size_t i = 0; i < 2; ++i) {
		const LensTestFit& fit = fits[i];
		const REAL start = scales[i]*LENS_TEST_A.focal;
		const LensFitOutcome& outcome = fit.report.cameras[0];
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		const REAL focalError = ABS(camera.fx/LENS_TEST_A.focal - REAL(1));
		const REAL lensError = outcome.status == LensFitOutcome::FITTED ? LensCurveError(fit.scene, 0, LENS_TEST_A) : REAL(-1);
		TEST_CHECK(fit.solved && outcome.focalBefore == start && outcome.focalMoved && camera.TrustIntrinsics() && camera.fx == camera.fy &&
			focalError <= LENS_TEST_FOCAL_MAX_ERROR && outcome.status == LensFitOutcome::FITTED && lensError <= LENS_TEST_MAX_ERROR_PX && VerifiedThroughE(fit.scene),
			"a strong lens given %g: the lens fit started from %g, focal %s to %g (true %g, %.2f%% off), lens status %u, %.2f px off, %s",
			start, outcome.focalBefore, outcome.focalMoved ? "moved" : "kept", camera.fx, LENS_TEST_A.focal, 100*focalError,
			(unsigned)outcome.status, lensError, VerifiedThroughE(fit.scene) ? "its pairs verified through E" : "some pair not verified through E");
		VERBOSE("LensFitUntrustedFocalTest: a strong lens given %g: fitted from it to %g (true %g, %.2f%% off), lens %.2f px off",
			start, camera.fx, LENS_TEST_A.focal, 100*focalError, lensError);
	}
	{
		const LensTestFit& fit = fits[2];
		const REAL start = REAL(0.9)*LENS_TEST_B.focal;
		const PinholeCamera& cameraA = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		const PinholeCamera& cameraB = *static_cast<const PinholeCamera*>(fit.scene.cameras[1]);
		const REAL focalErrorB = ABS(cameraB.fx/LENS_TEST_B.focal - REAL(1));
		TEST_CHECK(fit.solved && cameraA.fx == LENS_TEST_A.focal && cameraA.fy == LENS_TEST_A.focal && fit.report.cameras[1].focalBefore == start &&
			cameraB.TrustIntrinsics() && focalErrorB <= LENS_TEST_FOCAL_MAX_ERROR && VerifiedThroughE(fit.scene),
			"a trusted camera beside an untrusted one given 10%% short: A's focal %g %g (true %g), B's lens fit started from %g (its start %g), focal %g (true %g, %.2f%% off), %s",
			cameraA.fx, cameraA.fy, LENS_TEST_A.focal, fit.report.cameras[1].focalBefore, start, cameraB.fx, LENS_TEST_B.focal,
			100*focalErrorB, VerifiedThroughE(fit.scene) ? "the pairs verified through E" : "some pair not verified through E");
		VERBOSE("LensFitUntrustedFocalTest: a trusted camera beside an untrusted one given 10%% short: A's focal kept %g, B's %g -> %g (true %g, %.2f%% off)",
			cameraA.fx, start, cameraB.fx, LENS_TEST_B.focal, 100*focalErrorB);
	}
	{
		const LensTestFit& fit = fits[3];
		const REAL start = scaleB*LENS_TEST_B.focal;
		const PinholeCamera& cameraB = *static_cast<const PinholeCamera*>(fit.scene.cameras[1]);
		const double worstShare = fit.measured[0];
		TEST_CHECK(fit.solved && fit.report.cameras[0].status == LensFitOutcome::FITTED && fit.report.cameras[1].status == LensFitOutcome::TOO_FEW_PAIRS &&
			cameraB.TrustIntrinsics() && cameraB.fx != start && VerifiedThroughE(fit.scene) && worstShare >= LENS_TEST_MIN_INLIER_SHARE,
			"an untrusted camera with too few pairs beside a trusted one fitted: A's lens status %u, B's %u, B %s at %g (its start %g, true %g), %s, the worst pair of A and B keeping %.2f of its inliers",
			(unsigned)fit.report.cameras[0].status, (unsigned)fit.report.cameras[1].status, cameraB.TrustIntrinsics() ? "trusted" : "untrusted",
			cameraB.fx, start, LENS_TEST_B.focal, VerifiedThroughE(fit.scene) ? "the pairs verified through E" : "some pair not verified through E", worstShare);
		VERBOSE("LensFitUntrustedFocalTest: an untrusted camera with too few pairs beside a trusted one fitted: B calibrated %g -> %g (true %g), the worst pair of A and B keeping %.2f of its inliers",
			start, cameraB.fx, LENS_TEST_B.focal, worstShare);
	}
	for (const bool lensFit : {true, false}) {
		const LensTestFit& fit = fits[4 + !lensFit];
		const char* const name = lensFit ? "too few pairs" : "the lens fit off";
		const REAL start = REAL(0.85)*LENS_TEST_A.focal;
		const PinholeCamera& camera = *static_cast<const PinholeCamera*>(fit.scene.cameras[0]);
		TEST_CHECK(fit.solved && camera.TrustIntrinsics() && camera.fx != start && VerifiedThroughE(fit.scene), "%s: %s, focal %g (its start %g), %s", name,
			camera.TrustIntrinsics() ? "trusted" : "untrusted", camera.fx, start, VerifiedThroughE(fit.scene) ? "its pairs verified through E" : "some pair not verified through E");
		VERBOSE("LensFitUntrustedFocalTest: %s: calibrated %g -> %g, its pairs verified through E", name, start, camera.fx);
	}
	return scope.Pass("a strong lens fitted from a focal 20%% short and from the import's default, a trusted camera beside an untrusted one, an untrusted camera the lens fit cannot take calibrated and verified through E");
}

// View graph calibrator test: Refine focal length using view graph optimization
bool ViewGraphCalibratorTest()
{
	TestScope scope("ViewGraphCalibratorTest");

	// Generate synthetic scene with known ground truth focal length
	Scene sceneGT, scene;
	SceneConfig cfg;
	cfg.poseMode = SceneConfig::RANDOM_POSES;
	cfg.numImages = 8;             // Use multiple views for stronger constraints
	cfg.numPoints = 500;           // Sufficient 3D points for robust estimation
	cfg.generatePairs = true;      // Generate pairs with matches
	cfg.generateDescriptors = true;
	cfg.perturbOptions = SceneConfig::PERTURB_KEYPOINTS;  // Only perturb keypoints, keep poses exact
	GenerateTestScene(sceneGT, cfg, &scene);

	// Get initial camera state
	const PinholeCamera& gt_camera = *static_cast<PinholeCamera*>(sceneGT.cameras[0]);
	PinholeCamera& cam = *static_cast<PinholeCamera*>(scene.cameras[0]);
	const double gt_focal = gt_camera.fx;

	// Re-estimate F from noisy keypoints using RANSAC and validate them first;
	// with the GT focal length still in place. The calibrated branch of
	// PairsMatcher::GeometricFilter composes F = K2^-T * E * K1^-1 using
	// the camera's current K, so any perturbation applied to cam.fx before
	// this loop would be baked into F itself
	unsigned numFailedPairs = 0;
	MatchConfig matchCfg;
	matchCfg.maxEpipolarError = 3.f; // pixels
	matchCfg.descriptorsAreBinary = cfg.binaryDescriptors;
	PairsMatcher matcher(scene, matchCfg);
	for (ImagePair& pair : scene.pairs) {
		ASSERT(pair.GetNumMatches() >= 15);
		// Re-estimate F from noisy keypoints using RANSAC.
		// This is how F would be computed in a real SfM pipeline
		// (computing F analytically from accurate E creates degenerate σ₁=σ₂ case)
		TEST_CHECK(matcher.GeometricFilter(scene.images[pair.ID1], scene.images[pair.ID2], pair), "failed to estimate F for pair %u-%u", pair.ID1, pair.ID2);
		if (pair.GetNumFilteredInliers() < pair.GetNumMatches())
			++numFailedPairs;
	}
	if (numFailedPairs > 0)
		VERBOSE("warning: ViewGraphCalibratorTest: %u / %u pairs had epipolar constraint violations", numFailedPairs, (unsigned)scene.pairs.size());

	// Now that F encodes the GT focal length, perturb the camera's stored
	// focal so the calibrator has actual work to do.
	const double initial_focal = cam.fy = cam.fx *= 1.3; // perturb initial focal length by +30%

	// Apply view graph calibrator
	cam.trustIntrinsics = false; // the synthetic scene generator marks intrinsics trusted; the test exercises focal refinement
	ViewGraphCalibratorConfig vgConfig;
	vgConfig.minPairWeight = -1.f; // use all pairs, the synthetic ones carry no weight
	ViewGraphCalibrator calibrator(vgConfig);
	TEST_CHECK(calibrator.Solve(scene), "Calibrator.Solve() returned false");

	// Check refined focal length
	const double refined_focal = cam.fx;
	const double focal_error = ABS(refined_focal - gt_focal) / gt_focal;
	const double initial_error = ABS(initial_focal - gt_focal) / gt_focal;
	// refined estimate should be closer to GT than initial estimate
	TEST_CHECK(focal_error < initial_error, "refinement did not improve the focal estimate: initial error %.2f%%, refined error %.2f%%", initial_error*100, focal_error*100);
	// refined estimate should be within 15% of ground truth (reasonable tolerance given keypoint perturbation and RANSAC F estimation)
	const double tolerance = 0.15;
	TEST_CHECK(focal_error <= tolerance, "refined focal length error %.2f%% exceeds the tolerance %.2f%%", focal_error*100, tolerance*100);
	// the camera intrinsics stay reasonable (square pixels)
	const double aspect_ratio = cam.fy / cam.fx;
	if (ABS(aspect_ratio - 1.0) > 0.05)
		VERBOSE("warning: ViewGraphCalibratorTest - Camera aspect ratio differs from 1.0 (%.4f)", aspect_ratio);
	return scope.Pass("focal error %.2f%%, improvement %.2f%%", focal_error * 100, (initial_error - focal_error) / initial_error * 100);
}

} // namespace SFM
