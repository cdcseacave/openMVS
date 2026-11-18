/*
 * TestsSFMPipeline.cpp
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

// EstimateSimilarityTransform recovers a known scale, rotation and translation from exact correspondences
bool TestSimilarityTransform()
{
	#ifndef _RELEASE
	std::mt19937 rng(123);
	#else
	std::random_device rd;
	std::mt19937 rng(rd());
	#endif

	// Define a known similarity: scale * R * x + t
	const double scale = 1.5;
	Eigen::AngleAxisd aa(M_PI / 13.0, Eigen::Vector3d(0.3, 0.5, 0.2).normalized());
	Matrix3x3 R = aa.toRotationMatrix();
	Point3 t(0.7, -0.3, 0.2);

	// Create a small set of 3D points
	Point3Arr src, dst;
	std::uniform_real_distribution<REAL> transDist(-100, 100);
	for (int i = 0; i < 6; ++i) {
		src.emplace_back(transDist(rng), transDist(rng), transDist(rng));
		dst.emplace_back(scale * R * src[i] + t);
	}

	Transform tr;
	if (EstimateSimilarityTransform(src, dst, tr) == 0) {
		VERBOSE("SimilarityTransformUnitTest: EstimateSimilarityTransform failed");
		return false;
	}

	// Check scale within tolerance
	const double sErr = ABS(tr.scale - scale);
	const double tol = 1e-2;
	VERBOSE("SimilarityTransformUnitTest: scale=%f (expected %f)", tr.scale, scale);
	if (sErr >= tol)
		return false;

	// Validate transform by applying it to source points and comparing to destination
	double maxErr = 0.0;
	for (size_t i = 0; i < src.size(); ++i) {
		const Point3 mapped = tr.scale * tr.R * src[i] + tr.t;
		double err = norm(mapped - dst[i]);
		maxErr = MAXF(maxErr, err);
	}
	VERBOSE("SimilarityTransformUnitTest: All tests passed (max residual = %.6g)", maxErr);
	return maxErr < 1e-6;
}
/*----------------------------------------------------------------*/

// Re-align a scene transformed away from its imported camera frame.
bool AlignToPriorPosesTest()
{
	Scene scene;
	SceneConfig sceneCfg;
	sceneCfg.numImages = 8;
	sceneCfg.numPoints = 80;
	sceneCfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	sceneCfg.rotationAngleStep = 45;
	GenerateTestScene(scene, sceneCfg);
	for (const Image& image : scene.images)
		scene.priorPoses.emplace(image.ID, Pose3D(image.R, image.C));

	std::mt19937 rng(321);
	scene.Transform(Transform::Random(rng));
	if (!scene.AlignToPriorPoses(0.f)) {
		VERBOSE("AlignToPriorPosesTest FAILED: alignment returned false");
		return false;
	}
	for (const Image& image : scene.images) {
		const Pose3D& prior = scene.priorPoses.at(image.ID);
		const REAL centerError = norm(image.C - prior.C);
		const REAL rotationError = ACOS(ComputeAngle(image.R, prior.R));
		if (centerError > REAL(1e-4) || rotationError > REAL(1e-4)) {
			VERBOSE("AlignToPriorPosesTest FAILED: image %u error is %g position, %g degrees rotation",
				image.ID, centerError, R2D(rotationError));
			return false;
		}
	}

	VERBOSE("AlignToPriorPosesTest PASSED");
	return true;
}

// Re-align a straight-line (collinear) capture: the camera centers alone leave the roll
// about the trajectory unconstrained, so the alignment must recover the rotation from the
// camera rotations (the collinear fallback of EstimateSimilarityTransformWithRotations).
bool AlignToPriorPosesCollinearTest()
{
	Scene scene;
	SceneConfig sceneCfg;
	sceneCfg.numImages = 8;
	sceneCfg.numPoints = 80;
	sceneCfg.poseMode = SceneConfig::SIMPLE_TRANSLATION;
	sceneCfg.addPoseRotations = true;
	sceneCfg.rotationAngleStep = 5;
	GenerateTestScene(scene, sceneCfg);
	for (const Image& image : scene.images)
		scene.priorPoses.emplace(image.ID, Pose3D(image.R, image.C));

	std::mt19937 rng(654);
	scene.Transform(Transform::Random(rng));
	// the default threshold ratio engages the collinearity detection on this trajectory
	if (!scene.AlignToPriorPoses()) {
		VERBOSE("AlignToPriorPosesCollinearTest FAILED: alignment returned false");
		return false;
	}
	for (const Image& image : scene.images) {
		const Pose3D& prior = scene.priorPoses.at(image.ID);
		const REAL centerError = norm(image.C - prior.C);
		const REAL rotationError = ACOS(ComputeAngle(image.R, prior.R));
		if (centerError > REAL(1e-4) || rotationError > REAL(1e-4)) {
			VERBOSE("AlignToPriorPosesCollinearTest FAILED: image %u error is %g position, %g degrees rotation",
				image.ID, centerError, R2D(rotationError));
			return false;
		}
	}

	VERBOSE("AlignToPriorPosesCollinearTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

namespace {
// ===============================================================================
// Phase 5: Cube-map bridge tests
// ===============================================================================

// Helper: synthesize a simple equirectangular test image with 6 distinct
// color patches, one facing each cube face. Each patch is a small square
// centered on the equirectangular pixel corresponding to the cube-face
// look direction, so a correctly-rendered face has that color at its
// center pixel.

struct FaceColorSample {
	Point3 bodyDir;   // unit direction in sphere body frame (Y-up)
	Pixel8U color;    // BGR color assigned to this patch
};

// Pixel8U(r, g, b) — TPixel takes R first (named args by channel).
static const std::array<Pixel8U, 6> kFaceColors = {{
	Pixel8U(  0,   0, 255), // +Z blue
	Pixel8U(255,   0,   0), // -Z red
	Pixel8U(  0, 255,   0), // +X green
	Pixel8U(255, 255,   0), // -X yellow
	Pixel8U(255, 255, 255), // +Y up white
	Pixel8U(128, 128, 128), // -Y down gray
}};

static std::array<FaceColorSample, 6> BuildFaceCenterSamples(const SphereCubeMap::TangentFacesGeometry& geom)
{
	ASSERT(geom.numFaces == 6);
	std::array<FaceColorSample, 6> samples;
	const REAL f = geom.K(0,0);
	const REAL cx = geom.K(0,2);
	const REAL cy = geom.K(1,2);
	const int u = geom.faceSize / 2;
	const int v = geom.faceSize / 2;
	const Point3 centerRayFace((REAL(u) - cx) / f, (REAL(v) - cy) / f, REAL(1));
	for (int k = 0; k < 6; ++k) {
		samples[k].bodyDir = normalized(geom.rotations[k].t() * centerRayFace);
		samples[k].color = kFaceColors[k];
	}
	return samples;
}

static void BuildCheckerboardEquirect(
	Image8U3& src,
	int width,
	int height,
	const std::array<FaceColorSample, 6>& samples)
{
	src.create(height, width);
	// Paint a default dark gray background.
	src.setTo(cv::Scalar(40, 40, 40));
	// Stamp each face patch: a 5% x 5% rectangle centered on the
	// equirectangular pixel at the body direction.
	SphericalCamera sphCam(cv::Size(width, height));
	const int patchHalfW = std::max(2, width / 20);
	const int patchHalfH = std::max(2, height / 20);
	for (const FaceColorSample& sample : samples) {
		const auto [p, ok] = sphCam.Project(sample.bodyDir);
		if (!ok)
			continue;
		const int cx = ROUND2INT(p.x);
		const int cy = ROUND2INT(p.y);
		for (int dy = -patchHalfH; dy <= patchHalfH; ++dy) {
			const int y = cy + dy;
			if (y < 0 || y >= height) continue;
			for (int dx = -patchHalfW; dx <= patchHalfW; ++dx) {
				const int x = ((cx + dx) % width + width) % width;
				src(y, x) = sample.color;
			}
		}
	}
}

static void BuildCheckerboardEquirect(Image8U3& src, int width, int height)
{
	static const std::array<FaceColorSample, 6> kAxisSamples = {{
		{ Point3( 0,  0,  1), kFaceColors[0] },
		{ Point3( 0,  0, -1), kFaceColors[1] },
		{ Point3( 1,  0,  0), kFaceColors[2] },
		{ Point3(-1,  0,  0), kFaceColors[3] },
		{ Point3( 0,  1,  0), kFaceColors[4] },
		{ Point3( 0, -1,  0), kFaceColors[5] },
	}};
	BuildCheckerboardEquirect(src, width, height, kAxisSamples);
}
} // namespace

bool CubeMapFaceRenderTest()
{
	VERBOSE("\n=== CubeMapFaceRenderTest: equirectangular -> 6 pinhole faces ===");

	const int faceSize = 128;
	const auto geom = SphereCubeMap::MakeTangentFacesGeometry(6, faceSize);
	const auto samples = BuildFaceCenterSamples(geom);

	// Synthesize a 512x256 equirectangular source with one colored patch per face.
	Image8U3 src;
	BuildCheckerboardEquirect(src, 512, 256, samples);
	const std::vector<Image8U3> facesVec =
		SphereCubeMap::SphericalToTangentialFaces<Pixel8U>(src, geom);
	for (unsigned k = 0; k < 6; ++k) {
		const Image8U3& face = facesVec[k];
		if (face.cols != faceSize || face.rows != faceSize) {
			VERBOSE("CubeMapFaceRenderTest FAILED: face %u size mismatch (%dx%d expected %dx%d)",
			        k, face.cols, face.rows, faceSize, faceSize);
			return false;
		}
		// Read the central pixel of the face; it should be dominated by
		// the color assigned to face k's body direction.
		const Pixel8U& center = face(faceSize/2, faceSize/2);
		const Pixel8U& expected = kFaceColors[k];
		const int db = std::abs((int)center.b - (int)expected.b);
		const int dg = std::abs((int)center.g - (int)expected.g);
		const int dr = std::abs((int)center.r - (int)expected.r);
		if (db > 16 || dg > 16 || dr > 16) {
			VERBOSE("CubeMapFaceRenderTest FAILED: face %u center (%u,%u,%u) differs from expected (%u,%u,%u)",
			        k, center.b, center.g, center.r, expected.b, expected.g, expected.r);
			return false;
		}
	}

	VERBOSE("CubeMapFaceRenderTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

bool CubeMapBridgeGeometryTest()
{
	VERBOSE("\n=== CubeMapBridgeGeometryTest: rig platform + face images + observations via ExportMVS ===");

	Scene scene;
	Pose3D poseRelGT;
	BuildSphericalTwoViewScene(scene, poseRelGT);

	// Export to a temp .mvs so we can inspect the serialised interface. The
	// internal platform / image / vertex emission is exercised end-to-end
	// (the 4 old bridge helpers are now file-local to InterfaceMVS.cpp).
	const ScopedTempDir tmpDir(_T("CubeMapBridgeGeometryTest"));
	if (!tmpDir.IsValid())
		return false;
	const String mvsPath = tmpDir(_T("scene.mvs"));

	ExportMVSConfig cfg;
	cfg.undistortAlpha    = 0.f;
	cfg.onlyInlierTracks  = true;
	cfg.includeColors     = false;
	if (!SFM::ExportMVS(mvsPath, scene, cfg)) {
		VERBOSE("CubeMapBridgeGeometryTest FAILED: ExportMVS returned false");
		return false;
	}

	// Read the serialised interface back for inspection.
	MVS::Interface iface;
	if (!MVS::ARCHIVE::SerializeLoad(iface, mvsPath.c_str())) {
		VERBOSE("CubeMapBridgeGeometryTest FAILED: SerializeLoad returned false");
		return false;
	}

	// Expect exactly one rig platform (one shared spherical camera).
	if (iface.platforms.size() != 1) {
		VERBOSE("CubeMapBridgeGeometryTest FAILED: expected 1 platform, got %zu", iface.platforms.size());
		return false;
	}
	const auto& platform = iface.platforms[0];
	if (platform.cameras.size() != 6) {
		VERBOSE("CubeMapBridgeGeometryTest FAILED: expected 6 face cameras, got %zu", platform.cameras.size());
		return false;
	}

	// Verify each face camera intrinsics and rotation.
	const auto rotations = SphereCubeMap::FaceRotations(6);
	const Matrix3x3 expectedK = SphereCubeMap::FaceIntrinsics(1024, 6);
	for (unsigned k = 0; k < 6; ++k) {
		const auto& cam = platform.cameras[k];
		if (cam.width != 1024 || cam.height != 1024) {
			VERBOSE("CubeMapBridgeGeometryTest FAILED: face %u size %ux%u != 1024x1024", k, cam.width, cam.height);
			return false;
		}
		if (std::abs(cam.K(0,0) - expectedK(0,0)) > 1e-9 || std::abs(cam.K(1,1) - expectedK(1,1)) > 1e-9 ||
		    std::abs(cam.K(0,2) - expectedK(0,2)) > 1e-9 || std::abs(cam.K(1,2) - expectedK(1,2)) > 1e-9) {
			VERBOSE("CubeMapBridgeGeometryTest FAILED: face %u K mismatch", k);
			return false;
		}
		const Matrix3x3& expectedR = rotations[k];
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				if (std::abs(cam.R(i,j) - expectedR(i,j)) > 1e-9) {
					VERBOSE("CubeMapBridgeGeometryTest FAILED: face %u R(%d,%d) mismatch (%.4f vs %.4f)",
					        k, i, j, cam.R(i,j), expectedR(i,j));
					return false;
				}
			}
		}
		if (std::abs(cam.C.x) > 1e-9 || std::abs(cam.C.y) > 1e-9 || std::abs(cam.C.z) > 1e-9) {
			VERBOSE("CubeMapBridgeGeometryTest FAILED: face %u C != 0", k);
			return false;
		}
	}

	// Two source images → two rig poses.
	if (platform.poses.size() != 2) {
		VERBOSE("CubeMapBridgeGeometryTest FAILED: expected 2 platform poses, got %zu", platform.poses.size());
		return false;
	}
	// 2 source images × 6 faces = 12 MVS images.
	if (iface.images.size() != 12) {
		VERBOSE("CubeMapBridgeGeometryTest FAILED: expected 12 MVS images, got %zu", iface.images.size());
		return false;
	}

	// Verify per-image platformID/cameraID/poseID wiring.
	// Face images are emitted contiguously per source image in face order,
	// so imgs[6*srcIdx + k] is the face k of source srcIdx.
	for (unsigned srcIdx = 0; srcIdx < 2; ++srcIdx) {
		for (unsigned k = 0; k < 6; ++k) {
			const auto& img = iface.images[6 * srcIdx + k];
			if (img.platformID != 0) {
				VERBOSE("CubeMapBridgeGeometryTest FAILED: image (src=%u face=%u) platformID=%u", srcIdx, k, img.platformID);
				return false;
			}
			if (img.cameraID != k) {
				VERBOSE("CubeMapBridgeGeometryTest FAILED: image (src=%u face=%u) cameraID=%u", srcIdx, k, img.cameraID);
				return false;
			}
			if (img.poseID != srcIdx) {
				VERBOSE("CubeMapBridgeGeometryTest FAILED: image (src=%u face=%u) poseID=%u", srcIdx, k, img.poseID);
				return false;
			}
		}
	}

	// Every vertex should have at least 2 face-view entries (one per source image
	// that sees the track); tracks visible in both sources cover ≥ 2 faces total.
	unsigned totalObservations = 0;
	if (iface.vertices.empty()) {
		VERBOSE("CubeMapBridgeGeometryTest FAILED: expected non-empty vertices");
		return false;
	}
	for (const auto& v : iface.vertices) {
		if (v.views.size() < 2) {
			VERBOSE("CubeMapBridgeGeometryTest FAILED: vertex has only %zu views (expected >=2)", v.views.size());
			return false;
		}
		totalObservations += (unsigned)v.views.size();
	}
	VERBOSE("CubeMapBridgeGeometryTest: %u vertices, %u total face observations",
		(unsigned)iface.vertices.size(), totalObservations);
	VERBOSE("CubeMapBridgeGeometryTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

bool CubeMapBridgeEndToEndTest()
{
	VERBOSE("\n=== CubeMapBridgeEndToEndTest: on-disk cube-map pixel roundtrip ===");

	const ScopedTempDir tmpDir(_T("CubeMapBridgeEndToEndTest"));
	if (!tmpDir.IsValid())
		return false;

	// Synthesize a 256x128 equirectangular source image with one colored
	// patch per face (same helper the render test uses) and save it as .jxl.
	const auto geom = SphereCubeMap::MakeTangentFacesGeometry(6, 128);
	const auto samples = BuildFaceCenterSamples(geom);
	Image8U3 src;
	BuildCheckerboardEquirect(src, 256, 128, samples);
	const String srcFileName(tmpDir(_T("source.jxl")));
	if (!src.Save(srcFileName)) {
		VERBOSE("CubeMapBridgeEndToEndTest FAILED: cannot save source image '%s'", srcFileName.c_str());
		return false;
	}

	// Minimal scene: one spherical camera, one image referencing the file.
	Scene scene;
	scene.cameras.emplace_back(new SphericalCamera(cv::Size(256, 128)));
	Pose3D pose;
	pose.C = Point3(0, 0, 0);
	pose.R = Matrix3x3::IDENTITY;
	scene.images.emplace_back(0, srcFileName, pose, 0, scene.cameras[0]);

	// Export via the full ExportMVS pipeline: it renders + saves the N faces
	// under `undistortImageDir`, writes the .mvs, and emits the rig platform,
	// face images and track vertices the same way production MVS export does.
	String outputDir = tmpDir.Path() + _T("/faces/");
	Util::ensureFolder(outputDir);
	const String mvsPath = tmpDir(_T("scene.mvs"));

	ExportMVSConfig cfg;
	cfg.undistortImageDir = outputDir;
	cfg.undistortAlpha    = 0.f;
	cfg.onlyInlierTracks  = true;
	cfg.includeColors     = false;
	cfg.sphericalFaceSize  = 128;
	if (!SFM::ExportMVS(mvsPath, scene, cfg)) {
		VERBOSE("CubeMapBridgeEndToEndTest FAILED: ExportMVS returned false");
		return false;
	}

	// Verify each face file exists and its central pixel matches the patch color.
	const String stem = Util::getFileName(srcFileName);
	for (unsigned k = 0; k < 6; ++k) {
		const String faceFileName = outputDir + stem + String::FormatString(_T("_face%u"), k) + _T(".jxl");
		if (!std::filesystem::exists(faceFileName.c_str())) {
			VERBOSE("CubeMapBridgeEndToEndTest FAILED: face file '%s' not written", faceFileName.c_str());
			return false;
		}
		Image8U3 face;
		if (!face.Load(faceFileName)) {
			VERBOSE("CubeMapBridgeEndToEndTest FAILED: cannot load face file '%s'", faceFileName.c_str());
			return false;
		}
		if (face.cols != cfg.sphericalFaceSize || face.rows != cfg.sphericalFaceSize) {
			VERBOSE("CubeMapBridgeEndToEndTest FAILED: face %u size %dx%d != %dx%d",
			        k, face.cols, face.rows, cfg.sphericalFaceSize, cfg.sphericalFaceSize);
			return false;
		}
		const Pixel8U& center = face(cfg.sphericalFaceSize/2, cfg.sphericalFaceSize/2);
		const Pixel8U& expected = kFaceColors[k];
		// JXL is lossless for default settings but allow generous tolerance
		// because the equirectangular source is only 256x128 so the 5% patch
		// is only ~12 pixels wide — bilinear sampling at the face center may
		// already smear slightly.
		const int db = std::abs((int)center.b - (int)expected.b);
		const int dg = std::abs((int)center.g - (int)expected.g);
		const int dr = std::abs((int)center.r - (int)expected.r);
		if (db > 32 || dg > 32 || dr > 32) {
			VERBOSE("CubeMapBridgeEndToEndTest FAILED: face %u center (%u,%u,%u) differs from expected (%u,%u,%u)",
			        k, center.b, center.g, center.r, expected.b, expected.g, expected.r);
			return false;
		}
	}

	VERBOSE("CubeMapBridgeEndToEndTest PASSED (6 faces written + verified under %s)", tmpDir.Path().c_str());
	return true;
}
/*----------------------------------------------------------------*/

bool CubeMapBridgeMVSLoadTest()
{
	VERBOSE("\n=== CubeMapBridgeMVSLoadTest: ExportMVS -> MVS::Scene::Load roundtrip ===");

	const ScopedTempDir tmpDir(_T("CubeMapBridgeMVSLoadTest"));
	if (!tmpDir.IsValid())
		return false;

	// Build the same 2-view spherical scene but also materialize a source
	// .jxl file on disk for each image so RenderAndWriteFaces has something
	// to read. BuildSphericalTwoViewScene doesn't touch pixels, so we patch
	// the fileName + write a synthetic equirectangular after-the-fact.
	Scene scene;
	Pose3D poseRelGT;
	BuildSphericalTwoViewScene(scene, poseRelGT);

	Image8U3 src;
	BuildCheckerboardEquirect(src, 2048, 1024);
	FOREACH(i, scene.images) {
		Image& img = scene.images[i];
		const String path = tmpDir(String::FormatString(_T("sphere_%u.jxl"), (unsigned)i));
		if (!src.Save(path)) {
			VERBOSE("CubeMapBridgeMVSLoadTest FAILED: cannot save source image '%s'", path.c_str());
			return false;
		}
		img.fileName = path;
	}

	// Export the scene with the cube-map bridge. The face files land
	// alongside the .mvs output (no undistort dir provided).
	const String mvsPath = tmpDir(_T("scene.mvs"));
	ExportMVSConfig cfg;
	cfg.includeColors      = false;
	cfg.sphericalFaceSize  = 256;  // small for speed
	if (!ExportMVS(mvsPath, scene, cfg)) {
		VERBOSE("CubeMapBridgeMVSLoadTest FAILED: ExportMVS returned false");
		return false;
	}

	// Load the resulting .mvs via the MVS library and check structure.
	MVS::Scene mvsScene(1);
	const auto loaded = mvsScene.Load(mvsPath);
	if (loaded == MVS::Scene::SCENE_NA) {
		VERBOSE("CubeMapBridgeMVSLoadTest FAILED: MVS::Scene::Load returned SCENE_NA");
		return false;
	}
	if (mvsScene.platforms.size() != 1) {
		VERBOSE("CubeMapBridgeMVSLoadTest FAILED: expected 1 platform, got %u",
		        (unsigned)mvsScene.platforms.size());
		return false;
	}
	const auto& platform = mvsScene.platforms[0];
	if (platform.cameras.size() != 6) {
		VERBOSE("CubeMapBridgeMVSLoadTest FAILED: expected 6 mounted cameras, got %u",
		        (unsigned)platform.cameras.size());
		return false;
	}
	if (platform.poses.size() != 2) {
		VERBOSE("CubeMapBridgeMVSLoadTest FAILED: expected 2 poses, got %u",
		        (unsigned)platform.poses.size());
		return false;
	}
	if (mvsScene.images.size() != 12) {
		VERBOSE("CubeMapBridgeMVSLoadTest FAILED: expected 12 images, got %u",
		        (unsigned)mvsScene.images.size());
		return false;
	}
	if (mvsScene.pointcloud.points.empty()) {
		VERBOSE("CubeMapBridgeMVSLoadTest FAILED: empty point cloud after load");
		return false;
	}
	VERBOSE("CubeMapBridgeMVSLoadTest: loaded %u platforms, %u images, %u points",
	        (unsigned)mvsScene.platforms.size(),
	        (unsigned)mvsScene.images.size(),
	        (unsigned)mvsScene.pointcloud.points.size());

	VERBOSE("CubeMapBridgeMVSLoadTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

bool CubeMapBridgeMixedSceneTest()
{
	VERBOSE("\n=== CubeMapBridgeMixedSceneTest: pinhole + spherical pair in one export ===");

	// Build a minimal scene with two cameras: one pinhole (640x480) and one
	// spherical (2048x1024). Each camera contributes one image. We populate
	// tracks by hand so each track has at least one observation from each
	// image — that's enough to exercise both branches of ExportMVS's Phase 4
	// track expansion.
	Scene scene;
	// Pinhole camera + image
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), 500.0, 500.0, 320.0, 240.0));
	Pose3D ppose;
	ppose.C = Point3(0, 0, 0);
	ppose.R = Matrix3x3::IDENTITY;
	scene.images.emplace_back(0, String(_T("pinhole.jxl")), ppose, 0, scene.cameras[0]);
	// Spherical camera + image
	scene.cameras.emplace_back(new SphericalCamera(cv::Size(2048, 1024)));
	Pose3D spose;
	spose.C = Point3(0.5, 0, 0);
	spose.R = Matrix3x3::IDENTITY;
	scene.images.emplace_back(1, String(_T("sphere.jxl")), spose, 1, scene.cameras[1]);
	scene.status.nCalibratedImages = 2;

	// Generate a handful of 3D points in front of both cameras (+Z direction)
	// so each point is visible from pinhole (z>0 in pinhole frame) AND from
	// the spherical camera's forward (+Z) face.
	for (int i = 0; i < 10; ++i) {
		Point3 X(0.1 * (i - 5), 0.2 * (i % 3), 3.0 + 0.3 * i);
		Track track(X);
		// Each image gets one synthetic keypoint per track.
		// For pinhole: project through the camera.
		const auto [p0, ok0] = scene.images[0].ProjectPoint(X);
		const auto [p1, ok1] = scene.images[1].ProjectPoint(X);
		if (!ok0 || !ok1)
			continue;
		const uint32_t f0 = (uint32_t)scene.images[0].keypoints.size();
		const uint32_t f1 = (uint32_t)scene.images[1].keypoints.size();
		scene.images[0].keypoints.emplace_back(Cast<float>(p0), 0.f, 0.f, 10.f);
		scene.images[1].keypoints.emplace_back(Cast<float>(p1), 0.f, 0.f, 10.f);
		track.observations.emplace_back(0u, f0);
		track.observations.emplace_back(1u, f1);
		track.numInliers = (uint8_t)track.observations.size();
		scene.tracks.emplace_back(std::move(track));
	}
	VERBOSE("MixedSceneTest: built %u tracks", (unsigned)scene.tracks.size());

	// Materialize a fake pinhole jxl file so SavePixels / SceneLoad won't bark
	// (we only care about the spherical image being readable by the bridge).
	// The pinhole image is never read because its pixels aren't needed by
	// ExportMVS itself — it only serializes the path. However the spherical
	// side does need a readable file.
	const ScopedTempDir tmpDir(_T("CubeMapBridgeMixedSceneTest"));
	if (!tmpDir.IsValid())
		return false;

	Image8U3 srcSphere;
	BuildCheckerboardEquirect(srcSphere, 2048, 1024);
	const String spherePath = tmpDir(_T("sphere.jxl"));
	if (!srcSphere.Save(spherePath)) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: cannot save sphere source");
		return false;
	}
	scene.images[1].fileName = spherePath;
	// Give the pinhole image a plausible path too (existence not required by ExportMVS).
	scene.images[0].fileName = tmpDir(_T("pinhole.jxl"));

	// Export
	const String mvsPath = tmpDir(_T("scene.mvs"));
	ExportMVSConfig cfg;
	cfg.includeColors      = false;
	cfg.sphericalFaceSize  = 256;
	if (!ExportMVS(mvsPath, scene, cfg)) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: ExportMVS returned false");
		return false;
	}

	// Load and inspect
	MVS::Scene mvsScene(1);
	const auto loaded = mvsScene.Load(mvsPath);
	if (loaded == MVS::Scene::SCENE_NA) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: MVS::Scene::Load returned SCENE_NA");
		return false;
	}
	if (mvsScene.platforms.size() != 2) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: expected 2 platforms, got %u",
		        (unsigned)mvsScene.platforms.size());
		return false;
	}
	// Find the pinhole platform (1 camera) and the spherical rig platform (6 cameras).
	int pinholeIdx = -1, rigIdx = -1;
	for (unsigned p = 0; p < mvsScene.platforms.size(); ++p) {
		if (mvsScene.platforms[p].cameras.size() == 1)
			pinholeIdx = (int)p;
		else if (mvsScene.platforms[p].cameras.size() == 6)
			rigIdx = (int)p;
	}
	if (pinholeIdx < 0 || rigIdx < 0) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: couldn't find pinhole(1-cam) and rig(6-cam) platforms");
		return false;
	}
	if (mvsScene.platforms[pinholeIdx].poses.size() != 1) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: pinhole platform should have 1 pose");
		return false;
	}
	if (mvsScene.platforms[rigIdx].poses.size() != 1) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: rig platform should have 1 pose");
		return false;
	}
	// 1 pinhole + 6 face images = 7 MVS images.
	if (mvsScene.images.size() != 7) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: expected 7 images (1 pinhole + 6 faces), got %u",
		        (unsigned)mvsScene.images.size());
		return false;
	}
	if (mvsScene.pointcloud.points.empty()) {
		VERBOSE("CubeMapBridgeMixedSceneTest FAILED: empty point cloud after load");
		return false;
	}
	VERBOSE("CubeMapBridgeMixedSceneTest: %u platforms, %u images, %u points",
	        (unsigned)mvsScene.platforms.size(),
	        (unsigned)mvsScene.images.size(),
	        (unsigned)mvsScene.pointcloud.points.size());

	VERBOSE("CubeMapBridgeMixedSceneTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

bool CubeMapBridgeDropTopBottomTest()
{
	VERBOSE("\n=== CubeMapBridgeDropTopBottomTest: 4-face rig drops zenith/nadir ===");

	// Place points at the 6 cardinal directions (radius 5). The 6-face
	// version should see ALL points (each axis-aligned point maps to
	// exactly one face); the 4-face version should drop the +Y and -Y
	// points because those faces are removed.
	struct AxisPoint { Point3 X; int expectedFace; };
	const AxisPoint pts[6] = {
		{ Point3(0, 0,  5), 0 }, // +Z
		{ Point3(0, 0, -5), 1 }, // -Z
		{ Point3(5, 0,  0), 2 }, // +X
		{ Point3(-5, 0, 0), 3 }, // -X
		{ Point3(0,  5, 0), 4 }, // +Y (zenith)
		{ Point3(0, -5, 0), 5 }, // -Y (nadir)
	};

	// Pure-geometry projection (identical math to the MVS-export helper
	// ProjectTrackOntoSphericalFaces in InterfaceMVS.cpp, minus the
	// MVS::Interface plumbing). Returns the face indices (0..numFaces-1)
	// into which X projects with positive depth.
	auto ProjectFaces = [](const Point3& X,
	                       const SphereCubeMap::TangentFacesGeometry& geom) {
		std::vector<int> hit;
		const REAL f  = geom.K(0,0);
		const REAL cx = geom.K(0,2);
		const REAL cy = geom.K(1,2);
		const REAL zEps = REAL(1e-9);
		for (int k = 0; k < geom.numFaces; ++k) {
			const Point3 Xf = geom.rotations[k] * X; // pose is identity
			if (Xf.z < zEps) continue;
			const REAL u = f * Xf.x / Xf.z + cx;
			const REAL v = f * Xf.y / Xf.z + cy;
			if (u < REAL(0) || u >= REAL(geom.faceSize)) continue;
			if (v < REAL(0) || v >= REAL(geom.faceSize)) continue;
			hit.push_back(k);
		}
		return hit;
	};

	// 6-face case: every axis point should project into its assigned face.
	{
		const auto geom = SphereCubeMap::MakeTangentFacesGeometry(6, 512);
		for (unsigned i = 0; i < 6; ++i) {
			const auto hit = ProjectFaces(pts[i].X, geom);
			if (hit.empty()) {
				VERBOSE("CubeMapBridgeDropTopBottomTest FAILED: 6-face point %u has 0 views", i);
				return false;
			}
			bool foundExpected = false;
			for (int k : hit)
				if (k == pts[i].expectedFace) { foundExpected = true; break; }
			if (!foundExpected) {
				VERBOSE("CubeMapBridgeDropTopBottomTest FAILED: 6-face point %u missing expected face %u",
				        i, pts[i].expectedFace);
				return false;
			}
		}
	}

	// 4-face case (numFaces = 4, equivalent to the legacy dropTopBottomFaces
	// flag): axis points +Y and -Y now have no face that sees them.
	{
		const auto geom = SphereCubeMap::MakeTangentFacesGeometry(4, 512);
		if (geom.numFaces != 4) {
			VERBOSE("CubeMapBridgeDropTopBottomTest FAILED: MakeTangentFacesGeometry(4) returned numFaces=%d",
			        geom.numFaces);
			return false;
		}
		// +Z, -Z, +X, -X should all still land.
		for (unsigned i = 0; i < 4; ++i) {
			const auto hit = ProjectFaces(pts[i].X, geom);
			if (hit.empty()) {
				VERBOSE("CubeMapBridgeDropTopBottomTest FAILED: 4-face horizontal point %u has 0 views", i);
				return false;
			}
		}
		// +Y and -Y: zero hits (top/bottom faces are gone).
		for (unsigned i = 4; i < 6; ++i) {
			const auto hit = ProjectFaces(pts[i].X, geom);
			if (!hit.empty()) {
				VERBOSE("CubeMapBridgeDropTopBottomTest FAILED: 4-face zenith/nadir point %u has %u views (expected 0)",
				        i, (unsigned)hit.size());
				return false;
			}
		}
	}

	VERBOSE("CubeMapBridgeDropTopBottomTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

// GPS alignment degeneracy test: coincident or collinear GPS positions
// (e.g. a phone tagging many consecutive images with the same stale fix)
// must make AlignToGPS fail gracefully, leaving the scene untouched,
// instead of collapsing it with a scale-0 similarity transform
bool AlignToGPSDegenerateTest()
{
	const auto setGPS = [](Scene& scene, const std::function<double(const Image&)>& latitude) {
		for (Image& img : scene.images) {
			View::Metadata& meta = static_cast<View&>(img).metadata;
			meta.latitude = latitude(img);
			meta.longitude = 2.1649;
			meta.altitude = 80.4;
			meta.positionAccuracy = 5.0;
			meta.positionAccuracyZ = 5.0;
		}
	};
	const auto checkUnchanged = [](const Scene& scene, const Point3& C0, const char* test) {
		if (scene.status.nState.isSet(Scene::Status::STATE::GEO_ALIGN) ||
			!ISZERO(norm(scene.images[0].C - C0))) {
			VERBOSE("AlignToGPSDegenerateTest FAILED: scene modified by rejected alignment (%s)", test);
			return false;
		}
		return true;
	};

	SceneConfig cfg;
	cfg.numImages = 12;
	cfg.numPoints = 50;
	cfg.rotationAngleStep = 30.0;
	Scene scene;
	GenerateTestScene(scene, cfg);
	const Point3 C0 = scene.images[0].C;

	// Test 1: all images share the same GPS fix
	setGPS(scene, [](const Image&) { return 41.3918; });
	if (scene.AlignToGPS(5.0)) {
		VERBOSE("AlignToGPSDegenerateTest FAILED: coincident GPS positions accepted");
		return false;
	}
	if (!checkUnchanged(scene, C0, "coincident"))
		return false;

	// Test 2: only two distinct GPS fixes (collinear)
	setGPS(scene, [](const Image& img) { return img.ID % 2 ? 41.3918 : 41.39191; });
	if (scene.AlignToGPS(5.0)) {
		VERBOSE("AlignToGPSDegenerateTest FAILED: collinear GPS positions accepted");
		return false;
	}
	if (!checkUnchanged(scene, C0, "collinear"))
		return false;

	// Test 3: the closed-form estimation must reject coincident destination points
	{
		std::mt19937 rng(42);
		std::uniform_real_distribution<REAL> dist(-10, 10);
		Point3Arr src, dst;
		for (int i = 0; i < 12; ++i) {
			src.emplace_back(dist(rng), dist(rng), dist(rng));
			dst.emplace_back(1.0, 2.0, 3.0);
		}
		SEACAVE::Transform t;
		if (EstimateSimilarityTransform(src, dst, t, 0.0) != 0) {
			VERBOSE("AlignToGPSDegenerateTest FAILED: scale-0 similarity transform accepted");
			return false;
		}
	}

	// Test 4: well-spread GPS positions must still align (positive control)
	{
		SceneConfig cfgGPS;
		cfgGPS.numImages = 12;
		cfgGPS.numPoints = 50;
		cfgGPS.rotationAngleStep = 30.0;
		cfgGPS.circularRadius = 30.0;
		cfgGPS.generateGPS = true; // aligns to GPS with threshold 0 during generation
		Scene sceneGPS;
		GenerateTestScene(sceneGPS, cfgGPS);
		const REAL baseline = norm(sceneGPS.images[0].C - sceneGPS.images[6].C);
		if (!sceneGPS.AlignToGPS(5.0)) {
			VERBOSE("AlignToGPSDegenerateTest FAILED: well-spread GPS positions rejected");
			return false;
		}
		const REAL baselineAligned = norm(sceneGPS.images[0].C - sceneGPS.images[6].C);
		if (ABS(baselineAligned / baseline - REAL(1)) > REAL(1e-6)) {
			VERBOSE("AlignToGPSDegenerateTest FAILED: scale not preserved (%g -> %g)", baseline, baselineAligned);
			return false;
		}
	}

	VERBOSE("AlignToGPSDegenerateTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

// The pipeline on the four bundled photographs (BundledImagesConfig: AKAZE, every pair matched), each stage from the
// state the one before left:
// 1. refused before any work (no feature extracted): a SKIP run (--match-mode -1) on a scene without matches, and a
//    GPS prior weight, horizontal or vertical, with a spherical camera;
// 2. a match-images-only Scene::Reconstruct from the images: pairs matched, the pairs CSV (--export-pairs-csv) written
//    right after the matching, before any later step could drop a pair; the scene saved;
// 3. that .sfm as the source: neither matched nor filtered again (PAIRS_FILTERED), its pairs only weighed and exported;
// 4. SKIP on that .sfm: reconstructed from its own matches, the pair list untouched; SKIP on a saved scene holding
//    features but no matches: refused;
// 5. the global solver (--use-global-solver: Scene::ReconstructGlobal) on the matched scene registers every image;
// 6. the incremental reconstruction from deliberately wrong intrinsics (900 px, k1 0.6, k2 -0.09, untrusted):
//    tracks, the star, the adjustment refining them (ReconstructMatchedScene) recover the focal and the distortion.
bool RealImagesPipelineTest(bool verbose)
{
	TestScope scope("RealImagesPipelineTest");
	const ScopedTempDir tmpDir(_T("RealImagesPipelineTest"));
	TEST_CHECK(tmpDir.IsValid(), "no temporary folder");
	const auto NumLines = [](const String& path) {
		std::ifstream ifs(path);
		if (!ifs.is_open())
			return -1;
		int numLines = 0;
		for (std::string line; std::getline(ifs, line); )
			++numLines;
		return numLines;
	};

	// the deliberately wrong intrinsics the reconstruction of case 6 recovers from (900 px, k1 0.6, k2 -0.09)
	ImportConfig forced;
	forced.focalLength = 900.f;
	forced.k1 = 0.60f;
	forced.k2 = -0.09f;

	// 1. refused before any work
	const auto Refused = [](Camera* camera, const ReconstructionConfig& cfg) {
		Scene scene;
		scene.cameras.emplace_back(camera);
		for (IIndex i = 0; i < 2; ++i)
			scene.images.emplace_back(i, String(), Pose3D(), 0, scene.cameras[0]);
		return !scene.Reconstruct(String(), cfg) && !scene.status.nState.isSet(Scene::Status::STATE::FEATURES_EXTRACTED);
	};
	ReconstructionConfig skip, gpsHorizontal, gpsVertical;
	skip.matchCfg.mode = MatchConfig::SKIP;
	gpsHorizontal.baConfig.gpsPositionWeight = 1.0;
	gpsVertical.baConfig.gpsPositionWeightZ = 1.0;
	TEST_CHECK(Refused(new PinholeCamera(cv::Size(64, 48), REAL(60), REAL(60), REAL(32), REAL(24)), skip),
		"a SKIP run on a scene without matches was not refused at the start");
	TEST_CHECK(Refused(new SphericalCamera(cv::Size(64, 32)), gpsHorizontal), "the horizontal GPS weight was not refused at the start with panoramas");
	TEST_CHECK(Refused(new SphericalCamera(cv::Size(64, 32)), gpsVertical), "the vertical GPS weight was not refused at the start with panoramas");

	// 2. the images matched, the pairs CSV written right after the matching
	ReconstructionConfig matchOnly = BundledImagesConfig();
	matchOnly.matchImagesOnly = true;
	ReconstructionConfig cfg = matchOnly;
	cfg.exportPairsCSV = tmpDir(_T("pairs_a.csv"));
	Scene matched(2);
	TEST_CHECK(matched.Reconstruct(MAKE_PATH("images"), cfg) && !matched.pairs.empty() &&
		!matched.status.nState.isSet(Scene::Status::STATE::CALIBRATED), "the match-images-only run matched %u pairs", (unsigned)matched.pairs.size());
	TEST_CHECK(NumLines(cfg.exportPairsCSV) == (int)matched.pairs.size() + 1, "the pairs CSV has %d lines, expected %u matched pairs + 1 header",
		NumLines(cfg.exportPairsCSV), (unsigned)matched.pairs.size());
	const String matchedPath(tmpDir(_T("matched.sfm")));
	TEST_CHECK(matched.Save(matchedPath), "cannot save the matched scene");

	// 3. the saved scene as the source: weighed and exported, not matched or filtered again
	{
		Scene reloaded(2);
		cfg.exportPairsCSV = tmpDir(_T("pairs_b.csv"));
		TEST_CHECK(reloaded.Reconstruct(matchedPath, cfg) && reloaded.status.nState.isSet(Scene::Status::STATE::MATCHED),
			"the saved matched scene was not taken as the source");
		TEST_CHECK(NumLines(cfg.exportPairsCSV) == (int)reloaded.pairs.size() + 1, "the pairs CSV of the saved scene has %d lines, expected %u "
			"matched pairs + 1 header", NumLines(cfg.exportPairsCSV), (unsigned)reloaded.pairs.size());
	}

	// 4. SKIP: the saved scene reconstructs from its own matches; a saved scene with features and no matches is refused
	{
		ReconstructionConfig skipCfg = BundledImagesConfig();
		skipCfg.matchCfg.mode = MatchConfig::SKIP;
		Scene scene(2);
		TEST_CHECK(scene.Reconstruct(matchedPath, skipCfg), "the saved scene did not reconstruct with SKIP");
		TEST_CHECK(scene.pairs.size() == matched.pairs.size() && scene.status.nCalibratedImages >= 2,
			"SKIP left %u pairs (the saved %u expected) and registered %u images", (unsigned)scene.pairs.size(),
			(unsigned)matched.pairs.size(), scene.status.nCalibratedImages);
		const String featuredPath(tmpDir(_T("featured.sfm")));
		{
			Scene featured(2);
			TEST_CHECK(BundledImagesFeatures(featured, forced) && featured.Save(featuredPath), "cannot save the scene with features");
		}
		Scene unmatched(2);
		TEST_CHECK(!unmatched.Reconstruct(featuredPath, skipCfg), "a saved scene without matches was not refused by SKIP");
	}

	// 5. the global solver on the matched scene
	TEST_CHECK(matched.ReconstructGlobal(matchOnly), "the global solver failed");
	TEST_CHECK(matched.status.nState.isSet(Scene::Status::STATE::CALIBRATED) && matched.status.nCalibratedImages == matched.images.size() &&
		matched.status.nTracks >= 100, "the global solver registered %u of %u images, %u points",
		matched.status.nCalibratedImages, (unsigned)matched.images.size(), matched.status.nTracks);

	// 6. the incremental reconstruction recovering wrong intrinsics
	Scene scene(2);
	TEST_CHECK(BundledImagesFeatures(scene, forced), "the bundled images were not imported");
	static_cast<PinholeCamera*>(scene.cameras[0])->trustIntrinsics = false;
	TEST_CHECK(scene.MatchPairs(BundledImagesConfig().matchCfg), "the features of the bundled images were not matched");
	// tracks, the star and the adjustment; the focal within 100 px of 700, at most 10 px of distortion
	TEST_CHECK(ReconstructMatchedScene(scene, "RealImagesPipelineTest", 1500, 3000, 10), "the reconstruction from the wrong intrinsics");
	if (verbose) {
		// the reconstructed scene in both native SfM and MVS formats, into the data folder
		TEST_CHECK(scene.Save(MAKE_PATH("reconstruct_test.sfm")) && SFM::ExportMVS(MAKE_PATH("reconstruct_test.mvs"), scene),
			"cannot save the reconstructed scene");
	}
	return scope.Pass("refusals before any work, matching with the pairs CSV from the images and from the saved scene, SKIP from "
		"saved matches, the global solver, and the incremental reconstruction recovering wrong intrinsics (%u tracks)", scene.status.nTracks);
}

} // namespace SFM
