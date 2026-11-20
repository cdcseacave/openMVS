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

// The estimators that bring a scene into the frame of external positions. EstimateSimilarityTransform recovers a known
// scale, rotation and translation from six exact correspondences (the scale to 1e-2 and the mapped source points to
// 1e-6) and refuses coincident destination points (a scale-0 transform would collapse a scene), with and without the refinement behind it.
// An exactly collinear capture takes its rotation from the paired rotations (EstimateSimilarityTransformWithRotations). Scene::AlignToPriorPoses
// re-aligns a scene transformed away from its imported camera frame: on a circle of cameras, and on a straight-line
// (collinear) capture, where the camera centers alone leave the roll about the trajectory unconstrained so that it must
// recover the rotation from the camera rotations (the collinear fallback of EstimateSimilarityTransformWithRotations).
// Scene::AlignToGPS must fail gracefully on coincident or collinear GPS positions (a phone tagging many consecutive
// images with the same stale fix), leaving the scene untouched, and still align well-spread positions.
bool SceneAlignmentTest()
{
	TestScope scope("SceneAlignmentTest");

	// 1. a known similarity: scale * R * x + t
	{
		std::mt19937 rng(123);
		const double scale = 1.5;
		Eigen::AngleAxisd aa(M_PI / 13.0, Eigen::Vector3d(0.3, 0.5, 0.2).normalized());
		Matrix3x3 R = aa.toRotationMatrix();
		Point3 t(0.7, -0.3, 0.2);
		Point3Arr src, dst;
		std::uniform_real_distribution<REAL> transDist(-100, 100);
		for (int i = 0; i < 6; ++i) {
			src.emplace_back(transDist(rng), transDist(rng), transDist(rng));
			dst.emplace_back(scale * R * src[i] + t);
		}
		Transform tr;
		TEST_CHECK(EstimateSimilarityTransform(src, dst, tr) != 0, "EstimateSimilarityTransform failed");
		TEST_CHECK(ABS(tr.scale - scale) < 1e-2, "scale %f, expected %f", tr.scale, scale);
		double maxErr = 0.0;
		for (size_t i = 0; i < src.size(); ++i)
			maxErr = MAXF(maxErr, norm(tr.scale * tr.R * src[i] + tr.t - dst[i]));
		TEST_CHECK(maxErr < 1e-6, "the transform maps the source points %.6g from the destination (1e-6 at most)", maxErr);
	}
	// the closed-form estimation must reject coincident destination points: on 12 points the refinement behind it would
	// refuse the scale-0 start as well, on 6 (under the 10 the refinement needs) nothing but the guard on the estimated scale does
	for (const int numPoints : {12, 6}) {
		std::mt19937 rng(42);
		std::uniform_real_distribution<REAL> dist(-10, 10);
		Point3Arr src, dst;
		for (int i = 0; i < numPoints; ++i) {
			src.emplace_back(dist(rng), dist(rng), dist(rng));
			dst.emplace_back(1.0, 2.0, 3.0);
		}
		SEACAVE::Transform t;
		TEST_CHECK(EstimateSimilarityTransform(src, dst, t, 0.0) == 0, "%d points: a scale-0 similarity transform was accepted", numPoints);
	}
	// an exactly collinear capture: the centres leave the roll about the line free (the plain similarity maps them right with
	// any roll), so the alignment must take the rotation from the paired rotations; the true similarity comes back
	{
		std::mt19937 rng(77);
		const Transform truth(Transform::Random(rng));
		Matrix3x3 truthR(truth.R);
		Point3Arr srcCenters, dstCenters;
		Matrix3x3Arr srcRots, dstRots;
		for (int k = 0; k < 8; ++k) {
			srcCenters.emplace_back(0.7 * k, 0.35 * k, 0.14 * k);
			dstCenters.emplace_back(truth * srcCenters.back());
			srcRots.emplace_back(Eigen::AngleAxisd(0.09 * k, Eigen::Vector3d(0.2, 1.0, 0.1).normalized()).toRotationMatrix());
			dstRots.emplace_back(srcRots.back() * truthR.t());
		}
		Transform tr;
		TEST_CHECK(EstimateSimilarityTransformWithRotations(srcCenters, dstCenters, srcRots, dstRots, tr, 0.05) != 0,
			"the alignment of an exactly collinear capture failed");
		const REAL cosError = ComputeAngle(tr.R, truth.R);
		TEST_CHECK(cosError >= COS(D2R(0.01)) && ABS(tr.scale / truth.scale - 1) < 1e-6,
			"collinear capture: the rotation is %.4f degrees from the true one and the scale %g of %g (the roll about the line is not constrained by the centres)",
			R2D(ACOS(MINF(cosError, REAL(1)))), tr.scale, truth.scale);
	}

	// 2. the prior poses: the scene is moved by a random similarity and brought back to its imported frame
	const auto AlignPrior = [&](const char* name, SceneConfig::PoseMode poseMode, REAL angleStep, bool rotations, uint32_t seed, bool defaultRatio) {
		Scene scene;
		SceneConfig sceneCfg;
		sceneCfg.numImages = 8;
		sceneCfg.numPoints = 80;
		sceneCfg.poseMode = poseMode;
		sceneCfg.addPoseRotations = rotations;
		sceneCfg.rotationAngleStep = angleStep;
		GenerateTestScene(scene, sceneCfg);
		for (const Image& image : scene.images)
			scene.priorPoses.emplace(image.ID, Pose3D(image.R, image.C));
		std::mt19937 rng(seed);
		scene.Transform(Transform::Random(rng));
		// on the line the default threshold ratio is what engages the collinearity detection
		TEST_CHECK(defaultRatio ? scene.AlignToPriorPoses() : scene.AlignToPriorPoses(0.f), "%s: the alignment returned false", name);
		for (const Image& image : scene.images) {
			const Pose3D& prior = scene.priorPoses.at(image.ID);
			const REAL centerError = norm(image.C - prior.C);
			const REAL rotationError = ACOS(ComputeAngle(image.R, prior.R));
			TEST_CHECK(centerError <= REAL(1e-4) && rotationError <= REAL(1e-4), "%s: image %u error is %g position, %g degrees rotation",
				name, image.ID, centerError, R2D(rotationError));
		}
		return true;
	};
	if (!AlignPrior("circle", SceneConfig::CIRCULAR_ARRANGEMENT, 45, false, 321, false) ||
		!AlignPrior("line", SceneConfig::SIMPLE_TRANSLATION, 5, true, 654, true))
		return false;

	// 3. GPS: the fixes the scene's images report
	const auto SetGPS = [](Scene& scene, const std::function<double(const Image&)>& latitude) {
		for (Image& img : scene.images) {
			View::Metadata& meta = static_cast<View&>(img).metadata;
			meta.latitude = latitude(img);
			meta.longitude = 2.1649;
			meta.altitude = 80.4;
			meta.positionAccuracy = 5.0;
			meta.positionAccuracyZ = 5.0;
		}
	};
	SceneConfig cfg;
	cfg.numImages = 12;
	cfg.numPoints = 50;
	cfg.rotationAngleStep = 30.0;
	Scene scene;
	GenerateTestScene(scene, cfg);
	const Point3 C0 = scene.images[0].C;
	SetGPS(scene, [](const Image&) { return 41.3918; }); // all images share the same fix
	TEST_CHECK(!scene.AlignToGPS(5.0), "coincident GPS positions accepted");
	TEST_CHECK(!scene.status.nState.isSet(Scene::Status::STATE::GEO_ALIGN) && ISZERO(norm(scene.images[0].C - C0)),
		"scene modified by the rejected alignment (coincident)");
	SetGPS(scene, [](const Image& img) { return img.ID % 2 ? 41.3918 : 41.39191; }); // only two distinct fixes (collinear)
	TEST_CHECK(!scene.AlignToGPS(5.0), "collinear GPS positions accepted");
	TEST_CHECK(!scene.status.nState.isSet(Scene::Status::STATE::GEO_ALIGN) && ISZERO(norm(scene.images[0].C - C0)),
		"scene modified by the rejected alignment (collinear)");
	// well-spread GPS positions must still align (positive control)
	SceneConfig cfgGPS = cfg;
	cfgGPS.circularRadius = 30.0;
	cfgGPS.generateGPS = true; // aligns to GPS with threshold 0 during generation
	Scene sceneGPS;
	GenerateTestScene(sceneGPS, cfgGPS);
	const REAL baseline = norm(sceneGPS.images[0].C - sceneGPS.images[6].C);
	TEST_CHECK(sceneGPS.AlignToGPS(5.0), "well-spread GPS positions rejected");
	const REAL baselineAligned = norm(sceneGPS.images[0].C - sceneGPS.images[6].C);
	TEST_CHECK(ABS(baselineAligned / baseline - REAL(1)) <= REAL(1e-6), "scale not preserved (%g -> %g)", baseline, baselineAligned);
	return scope.Pass("the similarity is recovered and a scale-0 one refused, the prior poses realign a circle and a line, degenerate GPS fixes are "
		"rejected leaving the scene untouched and spread ones align");
}
/*----------------------------------------------------------------*/

namespace {
// A synthetic equirectangular test image with 6 distinct color patches, one facing each cube face. Each patch is a small
// square centered on the equirectangular pixel of the cube-face look direction, so a correctly rendered face has that
// color at its center pixel.
struct FaceColorSample {
	Point3 bodyDir;   // unit direction in sphere body frame (Y-up)
	Pixel8U color;    // BGR color assigned to this patch
};

// Pixel8U(r, g, b) -- TPixel takes R first (named args by channel).
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

static void BuildCheckerboardEquirect(Image8U3& src, int width, int height, const std::array<FaceColorSample, 6>& samples)
{
	src.create(height, width);
	src.setTo(cv::Scalar(40, 40, 40)); // a default dark gray background
	// stamp each face patch: a 5% x 5% rectangle centered on the equirectangular pixel at the body direction
	SphericalCamera sphCam(cv::Size(width, height));
	const int patchHalfW = std::max(2, width / 20);
	const int patchHalfH = std::max(2, height / 20);
	for (const FaceColorSample& sample : samples) {
		const auto [p, ok] = sphCam.Project(sample.bodyDir);
		ASSERT(ok);
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

// the center pixel of a face image must be dominated by the color assigned to its face, within tol per channel
static bool FaceCenterHasColor(const Image8U3& face, unsigned k, int tol)
{
	const Pixel8U& center = face(face.rows/2, face.cols/2);
	const Pixel8U& expected = kFaceColors[k];
	return std::abs((int)center.b - (int)expected.b) <= tol && std::abs((int)center.g - (int)expected.g) <= tol &&
		std::abs((int)center.r - (int)expected.r) <= tol;
}

// one more track at X seen by every image of a panorama scene, at its exact projection
static void AddSphereTrack(Scene& scene, const Point3& X)
{
	Track track(X);
	for (Image& img : scene.images) {
		const auto [proj, valid] = img.ProjectPoint(X);
		ASSERT(valid && Image8U::isInside(proj, img.GetSize()));
		track.observations.emplace_back(img.ID, (uint32_t)img.keypoints.size());
		img.keypoints.emplace_back(Cast<float>(proj), 0.f, 0.f, 10.f);
	}
	track.numInliers = (uint8_t)track.observations.size();
	scene.tracks.emplace_back(std::move(track));
	scene.status.nTracks = scene.tracks.size();
}
} // namespace

// The cube-map bridge of SFM::ExportMVS: every spherical source image is expanded into 6 (or 4) pinhole cube-map faces,
// emitted as a rig platform in MVS::Interface format, with a file tree MVS::Scene::Load reads back without any pinhole
// regression. Sections:
// 1. the rendering contract: an equirectangular image gives 6 pinhole faces whose center pixel holds each face's color;
// 2. one export of two panoramas (MakeSphereScene, 256x128, plus a track at the zenith and one at the nadir) read back
//    three ways: the interface (one rig platform of 6 face cameras at the face intrinsics and rotations, one pose per
//    source image, 12 images wired to their platform, camera and pose, every vertex seen by at least 2 faces), the face
//    files (written, sized, their center pixel the face's color), and MVS::Scene::Load (the same counts, a point cloud);
// 3. a pinhole and a spherical image in one export: two platforms, the rig's 6 faces plus the pinhole image;
// 4. the same two panoramas exported with 4 faces (sphericalNumFaces, the legacy drop of the top and bottom faces): 4
//    face cameras, 8 images, and the zenith and nadir tracks, which no face sees, are not exported while the 6-face
//    export holds them.
bool CubeMapExportTest()
{
	TestScope scope("CubeMapExportTest");
	constexpr int faceSize = 128;
	const auto geom = SphereCubeMap::MakeTangentFacesGeometry(6, faceSize);
	const auto samples = BuildFaceCenterSamples(geom);

	// 1. equirectangular -> 6 pinhole faces
	Image8U3 equirect;
	BuildCheckerboardEquirect(equirect, 256, 128, samples);
	{
		const std::vector<Image8U3> faces = SphereCubeMap::SphericalToTangentialFaces<Pixel8U>(equirect, geom);
		TEST_CHECK(faces.size() == 6, "%u faces rendered, expected 6", (unsigned)faces.size());
		for (unsigned k = 0; k < 6; ++k) {
			TEST_CHECK(faces[k].cols == faceSize && faces[k].rows == faceSize, "face %u size mismatch (%dx%d expected %dx%d)",
				k, faces[k].cols, faces[k].rows, faceSize, faceSize);
			const Pixel8U& center = faces[k](faceSize/2, faceSize/2);
			TEST_CHECK(FaceCenterHasColor(faces[k], k, 16), "face %u center (%u,%u,%u) differs from the expected (%u,%u,%u)",
				k, center.b, center.g, center.r, kFaceColors[k].b, kFaceColors[k].g, kFaceColors[k].r);
		}
	}

	// 2. one export, read back three ways
	const ScopedTempDir tmpDir(_T("CubeMapExportTest"));
	TEST_CHECK(tmpDir.IsValid(), "no temporary folder");
	Scene scene(MakeSphereScene({Point3(-0.5, 0, 0), Point3(0.5, 0, 0.2)}, 120, 2027, cv::Size(256, 128)));
	AddSphereTrack(scene, Point3(0, 5, 0));  // zenith
	AddSphereTrack(scene, Point3(0, -5, 0)); // nadir
	FOREACH(i, scene.images) {
		scene.images[i].fileName = tmpDir(String::FormatString(_T("sphere_%u.jxl"), (unsigned)i));
		TEST_CHECK(equirect.Save(scene.images[i].fileName), "cannot save source image '%s'", scene.images[i].fileName.c_str());
	}
	const String facesDir = tmpDir.Path() + _T("/faces/");
	Util::ensureFolder(facesDir);
	const String mvsPath = tmpDir(_T("scene.mvs"));
	ExportMVSConfig cfg;
	cfg.undistortImageDir = facesDir;
	cfg.undistortAlpha = 0.f;
	cfg.onlyInlierTracks = true;
	cfg.includeColors = false;
	cfg.sphericalFaceSize = faceSize;
	TEST_CHECK(SFM::ExportMVS(mvsPath, scene, cfg), "ExportMVS returned false");

	// the interface: exactly one rig platform (one shared spherical camera) of 6 face cameras
	MVS::Interface iface;
	TEST_CHECK(MVS::ARCHIVE::SerializeLoad(iface, mvsPath.c_str()), "SerializeLoad returned false");
	TEST_CHECK(iface.platforms.size() == 1 && iface.platforms[0].cameras.size() == 6, "expected 1 platform of 6 face cameras, got %zu platforms, %zu cameras",
		iface.platforms.size(), iface.platforms.empty() ? size_t(0) : iface.platforms[0].cameras.size());
	const auto& platform = iface.platforms[0];
	const auto rotations = SphereCubeMap::FaceRotations(6);
	const Matrix3x3 expectedK = SphereCubeMap::FaceIntrinsics(faceSize, 6);
	for (unsigned k = 0; k < 6; ++k) {
		const auto& cam = platform.cameras[k];
		TEST_CHECK(cam.width == faceSize && cam.height == faceSize, "face %u size %ux%u != %dx%d", k, cam.width, cam.height, faceSize, faceSize);
		TEST_CHECK(std::abs(cam.K(0,0) - expectedK(0,0)) <= 1e-9 && std::abs(cam.K(1,1) - expectedK(1,1)) <= 1e-9 &&
			std::abs(cam.K(0,2) - expectedK(0,2)) <= 1e-9 && std::abs(cam.K(1,2) - expectedK(1,2)) <= 1e-9, "face %u K mismatch", k);
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				TEST_CHECK(std::abs(cam.R(i,j) - rotations[k](i,j)) <= 1e-9, "face %u R(%d,%d) mismatch (%.4f vs %.4f)", k, i, j, cam.R(i,j), rotations[k](i,j));
		TEST_CHECK(std::abs(cam.C.x) <= 1e-9 && std::abs(cam.C.y) <= 1e-9 && std::abs(cam.C.z) <= 1e-9, "face %u C != 0", k);
	}
	// two source images: two rig poses and 2 source images x 6 faces = 12 MVS images, emitted contiguously per source image
	// in face order, so images[6*srcIdx + k] is the face k of source srcIdx
	TEST_CHECK(platform.poses.size() == 2 && iface.images.size() == 12, "expected 2 platform poses and 12 images, got %zu and %zu",
		platform.poses.size(), iface.images.size());
	for (unsigned srcIdx = 0; srcIdx < 2; ++srcIdx) {
		for (unsigned k = 0; k < 6; ++k) {
			const auto& img = iface.images[6 * srcIdx + k];
			TEST_CHECK(img.platformID == 0 && img.cameraID == k && img.poseID == srcIdx, "image (src=%u face=%u) wired to platform %u camera %u pose %u",
				srcIdx, k, img.platformID, img.cameraID, img.poseID);
		}
	}
	// every exported vertex is seen by at least 2 face images, and the zenith and nadir tracks are exported
	TEST_CHECK(!iface.vertices.empty(), "expected non-empty vertices");
	unsigned numPoles = 0;
	for (const auto& v : iface.vertices) {
		TEST_CHECK(v.views.size() >= 2, "vertex has only %zu views (expected >=2)", v.views.size());
		numPoles += (ABS(v.X.x) < 1e-3f && ABS(ABS(v.X.y) - 5.f) < 1e-3f && ABS(v.X.z) < 1e-3f);
	}
	TEST_CHECK(numPoles == 2, "%u of the zenith and nadir vertices exported with 6 faces, expected 2", numPoles);

	// the face files: every face of every source written next to the faces folder, at the face size, its center the face's color
	// (the source is only 256x128, so its patches are ~12 pixels wide: a generous tolerance)
	for (unsigned srcIdx = 0; srcIdx < 2; ++srcIdx) {
		const String stem = Util::getFileName(scene.images[srcIdx].fileName);
		for (unsigned k = 0; k < 6; ++k) {
			const String faceFileName = facesDir + stem + String::FormatString(_T("_face%u"), k) + _T(".jxl");
			TEST_CHECK(std::filesystem::exists(faceFileName.c_str()), "face file '%s' not written", faceFileName.c_str());
			Image8U3 face;
			TEST_CHECK(face.Load(faceFileName), "cannot load face file '%s'", faceFileName.c_str());
			TEST_CHECK(face.cols == faceSize && face.rows == faceSize, "face file '%s' size %dx%d != %dx%d", faceFileName.c_str(),
				face.cols, face.rows, faceSize, faceSize);
			TEST_CHECK(FaceCenterHasColor(face, k, 32), "face file '%s' center (%u,%u,%u) differs from the expected (%u,%u,%u)", faceFileName.c_str(),
				face(faceSize/2, faceSize/2).b, face(faceSize/2, faceSize/2).g, face(faceSize/2, faceSize/2).r,
				kFaceColors[k].b, kFaceColors[k].g, kFaceColors[k].r);
		}
	}

	// MVS::Scene::Load reads the export back
	{
		MVS::Scene mvsScene(1);
		TEST_CHECK(mvsScene.Load(mvsPath) != MVS::Scene::SCENE_NA, "MVS::Scene::Load returned SCENE_NA");
		TEST_CHECK(mvsScene.platforms.size() == 1, "loaded %u platforms, expected 1", (unsigned)mvsScene.platforms.size());
		TEST_CHECK(mvsScene.platforms[0].cameras.size() == 6 && mvsScene.platforms[0].poses.size() == 2 && mvsScene.images.size() == 12,
			"loaded %u mounted cameras, %u poses, %u images; expected 6, 2, 12", (unsigned)mvsScene.platforms[0].cameras.size(),
			(unsigned)mvsScene.platforms[0].poses.size(), (unsigned)mvsScene.images.size());
		TEST_CHECK(!mvsScene.pointcloud.points.empty(), "empty point cloud after load");
	}

	// 3. a pinhole and a spherical image in one export: both branches of the track expansion
	{
		Scene mixed;
		mixed.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), 500.0, 500.0, 320.0, 240.0));
		Pose3D ppose;
		ppose.C = Point3(0, 0, 0);
		ppose.R = Matrix3x3::IDENTITY;
		mixed.images.emplace_back(0, tmpDir(_T("pinhole.jxl")), ppose, 0, mixed.cameras[0]); // never read: ExportMVS only serializes its path
		mixed.cameras.emplace_back(new SphericalCamera(cv::Size(256, 128)));
		Pose3D spose;
		spose.C = Point3(0.5, 0, 0);
		spose.R = Matrix3x3::IDENTITY;
		mixed.images.emplace_back(1, scene.images[0].fileName, spose, 1, mixed.cameras[1]);
		mixed.status.nCalibratedImages = 2;
		// points in front of both cameras (+Z), so each is seen by the pinhole and by the spherical camera's forward face
		for (int i = 0; i < 10; ++i) {
			const Point3 X(0.1 * (i - 5), 0.2 * (i % 3), 3.0 + 0.3 * i);
			if (mixed.images[0].ProjectPoint(X).second && mixed.images[1].ProjectPoint(X).second)
				AddSphereTrack(mixed, X);
		}
		TEST_CHECK(!mixed.tracks.empty(), "no point projects into both images of the mixed scene");
		const String mixedPath = tmpDir(_T("mixed.mvs"));
		ExportMVSConfig mixedCfg;
		mixedCfg.includeColors = false;
		mixedCfg.sphericalFaceSize = faceSize;
		TEST_CHECK(ExportMVS(mixedPath, mixed, mixedCfg), "ExportMVS of the mixed scene returned false");
		MVS::Scene mvsScene(1);
		TEST_CHECK(mvsScene.Load(mixedPath) != MVS::Scene::SCENE_NA, "MVS::Scene::Load of the mixed export returned SCENE_NA");
		TEST_CHECK(mvsScene.platforms.size() == 2, "expected 2 platforms, got %u", (unsigned)mvsScene.platforms.size());
		// the pinhole platform has 1 camera, the spherical rig platform 6, each holds 1 pose
		int pinholeIdx = -1, rigIdx = -1;
		for (unsigned p = 0; p < mvsScene.platforms.size(); ++p) {
			if (mvsScene.platforms[p].cameras.size() == 1)
				pinholeIdx = (int)p;
			else if (mvsScene.platforms[p].cameras.size() == 6)
				rigIdx = (int)p;
		}
		TEST_CHECK(pinholeIdx >= 0 && rigIdx >= 0, "couldn't find pinhole(1-cam) and rig(6-cam) platforms");
		TEST_CHECK(mvsScene.platforms[pinholeIdx].poses.size() == 1 && mvsScene.platforms[rigIdx].poses.size() == 1, "each platform should have 1 pose");
		TEST_CHECK(mvsScene.images.size() == 7 && !mvsScene.pointcloud.points.empty(), "expected 7 images (1 pinhole + 6 faces) and a point cloud, got %u images, %u points",
			(unsigned)mvsScene.images.size(), (unsigned)mvsScene.pointcloud.points.size());
	}

	// 4. 4 faces: the zenith and nadir faces are gone, so the tracks at the poles have no view and are not exported
	{
		const String facesDir4 = tmpDir.Path() + _T("/faces4/");
		Util::ensureFolder(facesDir4);
		const String mvsPath4 = tmpDir(_T("scene4.mvs"));
		ExportMVSConfig cfg4 = cfg;
		cfg4.undistortImageDir = facesDir4;
		cfg4.sphericalNumFaces = 4;
		TEST_CHECK(SFM::ExportMVS(mvsPath4, scene, cfg4), "ExportMVS with 4 faces returned false");
		MVS::Interface iface4;
		TEST_CHECK(MVS::ARCHIVE::SerializeLoad(iface4, mvsPath4.c_str()), "SerializeLoad of the 4-face export returned false");
		TEST_CHECK(iface4.platforms.size() == 1 && iface4.platforms[0].cameras.size() == 4 && iface4.images.size() == 8,
			"the 4-face export holds %zu platforms, %zu cameras, %zu images; expected 1, 4, 8", iface4.platforms.size(),
			iface4.platforms.empty() ? size_t(0) : iface4.platforms[0].cameras.size(), iface4.images.size());
		unsigned numPoles4 = 0;
		for (const auto& v : iface4.vertices) {
			TEST_CHECK(v.views.size() >= 2, "4-face vertex has only %zu views (expected >=2)", v.views.size());
			numPoles4 += (ABS(v.X.x) < 1e-3f && ABS(ABS(v.X.y) - 5.f) < 1e-3f && ABS(v.X.z) < 1e-3f);
		}
		TEST_CHECK(numPoles4 == 0 && iface4.vertices.size() < iface.vertices.size(), "the 4-face export holds %u zenith/nadir vertices (0 expected) and %zu vertices (fewer than the 6-face %zu expected)",
			numPoles4, iface4.vertices.size(), iface.vertices.size());
	}
	return scope.Pass("6 faces rendered at their colors, one export read back as interface, face files and MVS scene, a mixed pinhole and spherical export, "
		"and the 4-face export without the zenith and nadir tracks (%zu vertices of %zu)", iface.vertices.size(), scene.tracks.size());
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
