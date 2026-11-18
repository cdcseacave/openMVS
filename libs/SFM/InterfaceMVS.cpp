////////////////////////////////////////////////////////////////////
// InterfaceMVS.cpp
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "InterfaceMVS.h"
#include "Scene.h"
#include "../Math/Camera/CameraUndistort.h"

// Import/Export scene and depth-maps to MVS and DMAP Interface format respectively
#ifndef _USE_OPENCV
#define _USE_OPENCV
#endif
#include "../IO/Interface.h"
#include <filesystem>
#include <map>

#ifdef _USE_OPENMP
#define INTERFACEMVS_USE_OPENMP
#endif

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

namespace {

struct DistortedDepthMap {
	MVS::DepthDataRaw data;
	Image32F depths;
	cv::Mat normals, confidence, views;

	bool Load(const String& path) {
		ASSERT(!path.empty());
		if (!MVS::ImportDepthDataRaw(path, data, depths, normals, confidence, views) ||
			(data.header.type & MVS::HeaderDepthDataRaw::SPHERICAL)) {
			VERBOSE("error: cannot read pinhole depth-map '%s'", path.c_str());
			return false;
		}
		return true;
	}

	bool Undistort(const String& path, const UndistortMaps& maps, const KMatrix& imageK) {
		ASSERT(!depths.empty() && !maps.map1.empty());
		if (depths.size() != maps.map1.size()) {
			VERBOSE("error: undistortion map size differs from depth-map '%s'", path.c_str());
			return false;
		}
		Image32F output;
		maps.RemapDepth(depths, output);
		for (cv::Mat* map: {&normals, &confidence, &views}) {
			if (!map->empty()) {
				cv::Mat remapped;
				maps.Remap(*map, remapped, cv::INTER_NEAREST);
				*map = std::move(remapped);
			}
		}
		// The temporary file shares the original directory, so relative image paths stay valid.
		const String tmp(path + ".tmp");
		data.K = imageK;
		std::error_code ec;
		if (MVS::ExportDepthDataRaw(tmp, data, output, normals, confidence, views)) {
			std::filesystem::rename(tmp.c_str(), path.c_str(), ec);
			if (!ec)
				return true;
		}
		File::deleteFile(tmp);
		VERBOSE("error: cannot replace depth-map '%s'", path.c_str());
		return false;
	}
};

} // namespace

bool SFM::UndistortDMAP(const String& path, const UndistortMaps& maps, const KMatrix& imageK)
{
	ASSERT(!path.empty() && !maps.map1.empty());
	DistortedDepthMap depthMap;
	return depthMap.Load(path) && depthMap.Undistort(path, maps, imageK);
}

// the camera matrix and the Brown distortion of the camera of an image, in the orientation of its file
// (View::RevertRotation): the original normalized coordinates of an image rotated on load are (y,-x)
// of the working ones, so the radial terms are invariant and the tangential ones become p1'=-p2,
// p2'=p1; returns the size of the file
static cv::Size RevertRotation(const Image& img, KMatrix& K, BrownDistortion<REAL>& distortion)
{
	ASSERT(img.pCamera != NULL && img.pCamera->GetType() == CameraType::PINHOLE);
	ASSERT(ISZERO(K(0,1)) && K(2,0) == 0 && K(2,1) == 0 && K(2,2) == 1);
	if (img.IsRotated()) {
		const REAL p1(distortion.p1);
		distortion.p1 = -distortion.p2;
		distortion.p2 = p1;
	}
	return img.RevertRotation(&K);
}

// the orientation of the file of an image, indexing the per-orientation exports of its camera: 1 for
// a portrait file the SfM rotated on load, 0 otherwise; the export of an equirectangular camera is
// never rotated
static int FileOrientation(const Image& img)
{
	ASSERT(img.pCamera != NULL);
	return img.pCamera->GetType() == CameraType::PINHOLE && img.IsRotated() ? 1 : 0;
}

bool SFM::UndistortDepthMaps(const Scene& scene,
	const CLISTDEF2(String)& depthMapFiles,
	std::optional<double> alpha,
	std::unordered_map<const Camera*, KMatrix>* undistortedIntrinsics)
{
	ASSERT(!depthMapFiles.empty() && depthMapFiles.size() == scene.images.size());
	ASSERT(!alpha || (*alpha >= 0 && *alpha <= 1));
	ASSERT(alpha || !undistortedIntrinsics);
	TD_TIMER_STARTD();
	// A single camera can have both portrait and landscape files and several depth-map resolutions.
	using MapKey = std::tuple<IIndex, bool, int, int>;
	std::map<MapKey, std::pair<UndistortMaps,KMatrix>> cache;
	int failures = 0;
	#ifdef INTERFACEMVS_USE_OPENMP
	#pragma omp parallel for schedule(dynamic) reduction(+:failures)
	#endif
	for (int_t i = 0; i < (int_t)scene.images.size(); ++i) {
		const Image& img = scene.images[i];
		const String& path = depthMapFiles[i];
		if (path.empty() || !img.IsValid() || img.pCamera->GetType() != CameraType::PINHOLE || !img.pCamera->HasDistortion())
			continue;
		DistortedDepthMap depthMap;
		if (!depthMap.Load(path)) {
			++failures;
			continue;
		}
		const cv::Size fileSize(img.GetOriginalSize()), depthSize(depthMap.depths.size());
		if (depthMap.data.header.imageWidth != (uint32_t)fileSize.width || depthMap.data.header.imageHeight != (uint32_t)fileSize.height) {
			VERBOSE("error: image dimensions differ from the header of '%s'", path.c_str());
			++failures;
			continue;
		}
		const std::pair<UndistortMaps,KMatrix>* entry;
		#ifdef INTERFACEMVS_USE_OPENMP
		#pragma omp critical(SFM_UndistortDepthMaps)
		#endif
		{
			const auto [it, inserted] = cache.try_emplace(MapKey(img.cameraID, img.IsRotated(), depthSize.width, depthSize.height));
			auto& [maps, Ku] = it->second;
			if (inserted) {
				const PinholeCamera& cam = *static_cast<const PinholeCamera*>(img.pCamera);
				KMatrix K(cam.GetK());
				BrownDistortion<REAL> distortion(cam.GetDistortion());
				if (alpha) {
					// Exported pixels are undistorted in the SfM working orientation, then rotated back.
					Ku = UndistortedK(K, distortion, cam.GetSize(), *alpha);
					if (undistortedIntrinsics)
						undistortedIntrinsics->emplace(&cam, Ku);
					img.RevertRotation(&Ku);
				}
				RevertRotation(img, K, distortion);
				if (!alpha)
					Ku = UndistortedK(K, distortion, fileSize); // native Brown loader uses the file orientation
				maps.Init(ScaleK(K, fileSize, depthSize), distortion, depthSize, ScaleK(Ku, fileSize, depthSize));
			}
			entry = &it->second;
		}
		if (!depthMap.Undistort(path, entry->first, entry->second))
			++failures;
	}
	DEBUG("Depth-map undistortion finished with %d failures (%s)", failures, TD_TIMER_GET_FMT().c_str());
	return failures == 0;
}
/*----------------------------------------------------------------*/


// The DMAP file format and its codec live in IO/Interface.h, which depends on nothing
// but the standard library and OpenCV and so is shared by the SFM and MVS libraries;
// the two functions below only adapt this library's types to it. Do not reimplement
// the packing here.
bool SFM::ImportDepthDataRaw(const String& fileName, String& imageFileName,
	IIndexArr& IDs, cv::Size& imageSize, cv::Size& depthSize,
	KMatrix& K, RMatrix& R, CMatrix& C,
	float& dMin, float& dMax,
	Image32F& depthMap, Image32F3& normalMap, Image32F& confMap, Image8U4& viewsMap, unsigned flags,
	bool* pbConfAdjusted)
{
	static_assert(sizeof(double) == sizeof(REAL));
	static_assert(sizeof(uint32_t) == sizeof(IIndex));

	MVS::DepthDataRaw data;
	if (!MVS::ImportDepthDataRaw(static_cast<const std::string&>(fileName), data,
		depthMap, normalMap, confMap, viewsMap, flags))
	{
		DEBUG("error: reading depth-data from file '%s'", fileName.c_str());
		return false;
	}
	if (pbConfAdjusted)
		*pbConfAdjusted = (data.header.type & MVS::HeaderDepthDataRaw::CONF_ADJUSTED) != 0;
	imageFileName = data.imageFileName;
	IDs.CopyOf(data.IDs.data(), (IIndex)data.IDs.size());
	K = data.K;
	R = data.R;
	C = data.C;
	dMin = data.header.dMin;
	dMax = data.header.dMax;
	imageSize.width = (int)data.header.imageWidth;
	imageSize.height = (int)data.header.imageHeight;
	depthSize.width = (int)data.header.depthWidth;
	depthSize.height = (int)data.header.depthHeight;
	return true;
} // ImportDepthDataRaw

bool SFM::ExportDepthDataRaw(const String& fileName, const String& imageFileName,
	const IIndexArr& IDs, const cv::Size& imageSize,
	const KMatrix& K, const RMatrix& R, const Point3& C,
	float dMin, float dMax,
	const Image32F& depthMap, const Image32F& confMap, const Image8U4& viewsMap,
	bool bConfAdjusted)
{
	ASSERT(!IDs.empty() && IDs.size() < 256);
	ASSERT(!depthMap.empty());
	ASSERT(confMap.empty() || depthMap.size() == confMap.size());
	ASSERT(viewsMap.empty() || depthMap.size() == viewsMap.size());
	ASSERT(depthMap.width() <= (int)imageSize.width && depthMap.height() <= (int)imageSize.height);
	static_assert(sizeof(double) == sizeof(REAL));
	static_assert(sizeof(uint32_t) == sizeof(IIndex));

	MVS::DepthDataRaw data;
	data.header.imageWidth = (uint32_t)imageSize.width;
	data.header.imageHeight = (uint32_t)imageSize.height;
	data.header.dMin = dMin;
	data.header.dMax = dMax;
	if (bConfAdjusted)
		data.header.type |= MVS::HeaderDepthDataRaw::CONF_ADJUSTED; // carried through by the codec
	// store the image path relative to the depth-map, so that the two travel together
	data.imageFileName = MAKE_PATH_REL(Util::getFullPath(Util::getFilePath(fileName)), Util::getFullPath(imageFileName));
	data.IDs.assign(IDs.begin(), IDs.end());
	data.K = K;
	data.R = R;
	data.C = C;
	if (!MVS::ExportDepthDataRaw(static_cast<const std::string&>(fileName), data,
		depthMap, cv::Mat()/*this library does not estimate normals*/, confMap, viewsMap))
	{
		DEBUG("error: writing depth-data to file '%s'", fileName.c_str());
		return false;
	}
	return true;
} // ExportDepthDataRaw
/*----------------------------------------------------------------*/


bool SFM::ImportMVS(const String& fileName, Scene& scene, bool loadColors)
{
	using MVSInterface = MVS::Interface;
	using MVSPlatform = MVS::Interface::Platform;

	TD_TIMER_STARTD();
	scene.Release();

	MVSInterface iface;
	if (!MVS::ARCHIVE::SerializeLoad(iface, fileName.c_str())) {
		VERBOSE("error: failed to load '%s'", fileName.c_str());
		return false;
	}

	const String basePath = MAKE_PATH_FULL(WORKING_FOLDER_FULL, Util::getFilePath(fileName));
	std::unordered_map<PairIdx, IIndex> camToID;
	camToID.reserve(iface.platforms.size());
	const auto DeduceSize = [&](uint32_t pid, uint32_t cid)->cv::Size {
		for (const auto& img : iface.images) {
			if (img.platformID != pid || img.cameraID != cid)
				continue;
			IMAGEPTR pImage(CImage::Create(MAKE_PATH_FULL(basePath, img.name), CImage::READ));
			if (pImage != NULL && pImage->ReadHeader())
				return cv::Size(pImage->GetWidth(), pImage->GetHeight());
		}
		return cv::Size();
	};

	for (uint32_t pid = 0; pid < iface.platforms.size(); ++pid) {
		const MVSPlatform& platform = iface.platforms[pid];
		for (uint32_t cid = 0; cid < platform.cameras.size(); ++cid) {
			const auto& cam = platform.cameras[cid];
			cv::Size size((int)cam.width, (int)cam.height);
			if (size.width == 0 || size.height == 0)
				size = DeduceSize(pid, cid);
			if (size.width <= 0 || size.height <= 0) {
				VERBOSE("error: missing resolution for platform %u camera %u", pid, cid);
				continue;
			}
			Camera* pCamera;
			if (cam.model == MVSPlatform::Camera::SPHERICAL) {
				pCamera = new SphericalCamera(size);
			} else {
				const auto fullK = platform.GetFullK(cid, (uint32_t)size.width, (uint32_t)size.height);
				PinholeCamera* pc = new PinholeCamera(size);
				pc->SetK(fullK);
				pc->trustIntrinsics = true;
				if (cam.model == MVSPlatform::Camera::BROWN) {
					if (cam.params.size() != 5 && cam.params.size() != 8) {
						VERBOSE("error: Brown camera %u of platform %u needs 5 or 8 distortion coefficients", cid, pid);
						delete pc;
						continue;
					}
					pc->SetDistortion(cam.params[0], cam.params[1], cam.params[2], cam.params[3], cam.params[4]);
					if (cam.params.size() == 8) {
						pc->useAdditionalDistortion = true;
						pc->k4 = cam.params[5]; pc->k5 = cam.params[6]; pc->k6 = cam.params[7];
					}
				} else if (cam.model != MVSPlatform::Camera::PINHOLE) {
					VERBOSE("error: unknown model %u of camera %u of platform %u", (unsigned)cam.model, cid, pid);
					delete pc;
					continue;
				}
				pCamera = pc;
			}
			pCamera->SetName(cam.name.empty() ? platform.name : cam.name);
			pCamera->metadata.model = cam.bandName;
			const IIndex camID = scene.cameras.size();
			scene.cameras.emplace_back(pCamera);
			camToID.emplace(PairIdx(pid, cid), camID);
		}
	}

	scene.images.reserve(iface.images.size());
	std::vector<IIndex> imageIndices(iface.images.size(), NO_ID);
	uint32_t posed = 0;
	for (size_t i = 0; i < iface.images.size(); ++i) {
		const auto& inImg = iface.images[i];
		const auto key = PairIdx(inImg.platformID, inImg.cameraID);
		auto itCam = camToID.find(key);
		if (itCam == camToID.end()) {
			VERBOSE("error: skipping image %zu (platform %u camera %u not found)", i, inImg.platformID, inImg.cameraID);
			continue;
		}
		const IIndex camID = itCam->second;
		const String imgPath = MAKE_PATH_FULL(basePath, inImg.name);
		imageIndices[i] = scene.images.size();
		Image& img = scene.images.emplace_back(
		    (inImg.ID != MVS::NO_ID) ? inImg.ID : scene.images.size(),
			imgPath);
		img.cameraID = camID;
		img.pCamera = scene.cameras[camID];
		if (inImg.IsValid()) {
			if (inImg.poseID >= iface.platforms[inImg.platformID].poses.size()) {
				VERBOSE("error: invalid pose %u for image %zu", inImg.poseID, i);
				return false;
			}
			const auto pose = iface.platforms[inImg.platformID].GetPose(inImg.cameraID, inImg.poseID);
			img.R = pose.R;
			img.C = pose.C;
			posed++;
		}
	}

	scene.tracks.reserve(iface.vertices.size());
	if (loadColors)
		scene.colors.reserve(iface.vertices.size());
	for (size_t v = 0; v < iface.vertices.size(); ++v) {
		const auto& vx = iface.vertices[v];
		Track track(vx.X);
		track.observations.reserve(vx.views.size());
		for (const auto& vw : vx.views) {
			if (vw.imageID >= imageIndices.size() || imageIndices[vw.imageID] == NO_ID)
				continue;
			const IIndex imageID = imageIndices[vw.imageID];
			IIndex featureID = NO_ID;
			if (iface.HasObservations()) {
				auto& keypoints = scene.images[imageID].keypoints;
				featureID = (IIndex)keypoints.size();
				keypoints.emplace_back(cv::Point2f(vw.x.x, vw.x.y), 1.f);
			}
			track.observations.emplace_back(imageID, featureID);
		}
		track.numInliers = (uint8_t)std::min<size_t>(track.observations.size(), (size_t)std::numeric_limits<uint8_t>::max());
		if (!track.IsValid())
			continue;
		scene.tracks.emplace_back(std::move(track));
		if (loadColors) {
			Pixel8U col = Pixel8U::BLACK;
			if (v < iface.verticesColor.size()) {
				const TPoint3<uint8_t> c = iface.verticesColor[v].c;
				col = Pixel8U(c);
			}
			scene.colors.emplace_back(col);
		}
	}

	scene.transform = iface.transform;
	if (iface.obb.IsValid()) {
		OBB3::MATRIX rot = Matrix3x3(iface.obb.rot);
		OBB3::POINT ptMin = Point3(iface.obb.ptMin);
		OBB3::POINT ptMax = Point3(iface.obb.ptMax);
		scene.obb.Set(rot, ptMin, ptMax);
	}

	scene.status.nCalibratedImages = posed;
	scene.status.nTracks = (uint32_t)std::count_if(scene.tracks.begin(), scene.tracks.end(),
		[](const Track& t) { return t.IsInlier(); });
	scene.status.nState = Scene::Status::STATE::EMPTY;
	if (scene.status.nTracks > 0)
		scene.status.nState.set(Scene::Status::STATE::MATCHED);
	if (posed > 0)
		scene.status.nState.set(Scene::Status::STATE::CALIBRATED);

	DEBUG("Scene imported from MVS: %u platforms, %u images (%u posed), %u tracks%s from '%s' (%s)",
		(unsigned)iface.platforms.size(), scene.images.size(), posed, (unsigned)scene.tracks.size(),
		(loadColors && !scene.colors.empty()) ? " with colors" : "",
		fileName.c_str(), TD_TIMER_GET_FMT().c_str());
	return true;
} // ImportMVS


// ------------------------------------------------------------------
// MVS-export helpers converting a pre-built
// SphereCubeMap::TangentFacesGeometry into MVS::Interface records
// ------------------------------------------------------------------
namespace {

using SphereCubeMap::TangentFacesGeometry;

// One rig Platform per spherical SFM camera, N face Cameras each.
void EmitSphericalPlatforms(
	const Scene& scene,
	const TangentFacesGeometry& geometry,
	MVS::Interface& iface,
	std::unordered_map<const Camera*, uint32_t>& camToPlatform)
{
	ASSERT(geometry.numFaces > 0);
	for (CameraPtr const cam : scene.cameras) {
		if (cam == NULL || cam->GetType() != CameraType::SPHERICAL)
			continue;
		if (camToPlatform.count(cam))
			continue;
		const uint32_t platformID = (uint32_t)iface.platforms.size();
		camToPlatform[cam] = platformID;

		MVS::Interface::Platform& platform = iface.platforms.emplace_back();
		platform.name = cam->metadata.name;
		platform.cameras.reserve(geometry.numFaces);
		for (int k = 0; k < geometry.numFaces; ++k) {
			MVS::Interface::Platform::Camera& outCam = platform.cameras.emplace_back();
			outCam.name   = String::FormatString("face%d", k);
			outCam.width  = (uint32_t)geometry.faceSize;
			outCam.height = (uint32_t)geometry.faceSize;
			outCam.K = geometry.K;
			outCam.R = geometry.rotations[k];
			outCam.C = MVS::Interface::Pos3d(0, 0, 0);
		}
	}
}

// Write the faces and register the same paths in the interface scene. Global face IDs provide
// unique names even when panoramas in different directories have the same basename.
bool ExportSphericalFaces(const Image& img, const TangentFacesGeometry& geometry, const String& outputDir,
	const String& sceneDir, const String& extension, uint32_t platformID, MVS::Interface& iface,
	std::vector<uint32_t>& imageIndices, uint32_t& nextID)
{
	ASSERT(img.pCamera != NULL && img.pCamera->GetType() == CameraType::SPHERICAL);
	ASSERT(platformID < iface.platforms.size() && geometry.numFaces > 0 && imageIndices.empty());
	Image8U3 pixels;
	if (img.HasPixels())
		pixels = img.GetImage8U3();
	else if (!LoadImage(img.fileName, pixels, PF_R8G8B8)) {
		VERBOSE("error: cannot read panorama '%s'", img.fileName.c_str());
		return false;
	}
	const auto faces = SphereCubeMap::SphericalToTangentialFaces<Pixel8U>(pixels, geometry);
	MVS::Interface::Platform& platform = iface.platforms[platformID];
	uint32_t poseID = MVS::NO_ID;
	if (img.IsValid()) {
		poseID = (uint32_t)platform.poses.size();
		platform.poses.emplace_back(img.R, img.C);
	}
	const String stem(Util::getFileName(img.fileName));
	imageIndices.reserve(faces.size());
	for (int k = 0; k < geometry.numFaces; ++k) {
		const String path(outputDir + String::FormatString("cube%u_", nextID) + stem + String::FormatString("_face%d", k) + extension);
		if (!faces[k].Save(path)) {
			VERBOSE("error: cannot write cube-map face '%s'", path.c_str());
			return false;
		}
		imageIndices.push_back((uint32_t)iface.images.size());
		auto& outImg = iface.images.emplace_back();
		outImg.name = MAKE_PATH_REL(sceneDir, path);
		outImg.platformID = platformID;
		outImg.cameraID = (uint32_t)k;
		outImg.poseID = poseID;
		outImg.ID = nextID++;
	}
	return true;
}

// Add a Vertex::View entry for every face of one spherical image's
// rig pose that sees the observation: the faces are decided by the direction
// (panorama frame) of pFeatureRay, the ray of the feature, if given, mapped to
// each face as the view observation (so it lies inside the face image),
// otherwise by the direction of the 3D world point X.
void ProjectTrackOntoSphericalFaces(
	const Point3& X,
	const Pose3D& sphericalPose,
	const TangentFacesGeometry& geometry,
	const std::vector<uint32_t>& faceImageIDs,
	const Point3* pFeatureRay,
	MVS::Interface::Vertex& vertex)
{
	ASSERT((int)faceImageIDs.size() == geometry.numFaces);
	const REAL f  = geometry.K(0,0);
	const REAL cx = geometry.K(0,2);
	const REAL cy = geometry.K(1,2);
	const REAL zEps = REAL(1e-9);
	const Point3 X_body = sphericalPose.R * (X - sphericalPose.C);
	const Point3& dirBody = pFeatureRay ? *pFeatureRay : X_body;

	for (int k = 0; k < geometry.numFaces; ++k) {
		const Point3 dirFace = geometry.rotations[k] * dirBody;
		if (dirFace.z < zEps)
			continue;
		const REAL u = f * dirFace.x / dirFace.z + cx;
		const REAL v = f * dirFace.y / dirFace.z + cy;
		if (u < REAL(0) || u >= REAL(geometry.faceSize)) continue;
		if (v < REAL(0) || v >= REAL(geometry.faceSize)) continue;
		MVS::Interface::Vertex::View view;
		view.imageID    = faceImageIDs[k];
		view.confidence = 0.f;
		if (pFeatureRay)
			view.x = MVS::Interface::Pos2f(float(u), float(v));
		vertex.views.push_back(view);
	}
}

} // namespace


bool SFM::ExportMVS(const String& fileName, const Scene& scene, ExportMVSConfig config)
{
	using MVSInterface = MVS::Interface;
	using MVSPlatform = MVS::Interface::Platform;
	using MVSCamera = MVS::Interface::Platform::Camera;
	using MVSVertex = MVS::Interface::Vertex;
	using Mat33d = MVS::Interface::Mat33d;
	using Pos3d = MVS::Interface::Pos3d;

	if (scene.images.empty()) {
		VERBOSE("error: scene to be exported is empty");
		return false;
	}
	TD_TIMER_STARTD();
	MVSInterface iface;

	// Detect whether this scene contains any spherical cameras, and whether
	// they are expanded into N-face rigs using the SphereCubeMap geometry.
	bool hasSpherical = false;
	for (CameraPtr const cam : scene.cameras) {
		if (cam != NULL && cam->GetType() == CameraType::SPHERICAL) {
			hasSpherical = true;
			break;
		}
	}
	const bool cubeMap = hasSpherical && config.sphericalExport == ExportMVSConfig::Spherical::CUBEMAP;

	// Build the sphere→tangent-faces geometry ONCE and reuse it across
	// every cube-map phase (platform emission, face-image emission,
	// rendering, track projection).
	SphereCubeMap::TangentFacesGeometry sphericalGeom;
	if (cubeMap) {
		if (!SphereCubeMap::IsSupportedNumFaces(config.sphericalNumFaces) || config.sphericalFaceSize <= 0) {
			VERBOSE("error: unsupported sphericalNumFaces=%d (expected 4,6,8,12,20) or sphericalFaceSize=%d",
				config.sphericalNumFaces, config.sphericalFaceSize);
			return false;
		}
		sphericalGeom = SphereCubeMap::MakeTangentFacesGeometry(
			config.sphericalNumFaces, config.sphericalFaceSize);
	}

	const String mvsFileDir = MAKE_PATH_FULL(WORKING_FOLDER_FULL, Util::getFilePath(fileName));
	String cubeMapOutputDir;
	if (cubeMap) {
		cubeMapOutputDir = config.undistortImageDir.empty() ? mvsFileDir : MAKE_PATH_FULL(WORKING_FOLDER_FULL, config.undistortImageDir);
		Util::ensureValidFolderPath(cubeMapOutputDir);
		Util::ensureFolder(cubeMapOutputDir);
	}

	// Map each SFM camera pointer to its (possibly multi-camera) platform, per orientation of the image
	// files: the images rotated on load (View::IsRotated) are portrait files sharing the landscape
	// camera, so they get a platform of their own, the camera in the orientation of their files.
	std::unordered_map<const Camera*, uint32_t> camToPlatform[2];
	camToPlatform[0].reserve(scene.cameras.size());
	// per camera, an image of each orientation of its files, the camera of the platform rotated as it
	std::unordered_map<const Camera*, std::array<const Image*,2>> camImages;
	for (const Image& img : scene.images) {
		std::array<const Image*,2>& images = camImages.emplace(img.pCamera, std::array<const Image*,2>{NULL, NULL}).first->second;
		const Image*& image = images[FileOrientation(img)];
		if (image == NULL)
			image = &img;
	}

	const bool undistort = !config.undistortImageDir.empty();
	std::unordered_map<const Camera*, KMatrix> undistortK;
	CLISTDEF2(String) undistortedPaths;
	if (undistort && !scene.UndistortImages(config.undistortImageDir, config.extension,
			config.undistortAlpha, &undistortedPaths, &undistortK)) {
		VERBOSE("error: failed to undistort the images into '%s'", config.undistortImageDir.c_str());
		return false;
	}

	// ----- Platforms and cameras -----
	// Pinhole and native spherical: one platform with a single camera per SFM camera and orientation
	// of its image files (FileOrientation).
	// Cube-map spherical: one platform with N face cameras via EmitSphericalPlatforms.
	unsigned numBrownCams = 0; // SFM cameras exported as BROWN
	for (CameraPtr const cam : scene.cameras) {
		if (cam->GetType() == CameraType::SPHERICAL && cubeMap)
			continue; // deferred to EmitSphericalPlatforms below
		const auto itImages = camImages.find(cam);
		for (int rotated = 0; rotated < 2; ++rotated) {
			// a camera without images keeps the orientation of its images as loaded
			const Image* const pImage(itImages == camImages.end() ? NULL : itImages->second[rotated]);
			if (pImage == NULL && (rotated || itImages != camImages.end()))
				continue;
			ASSERT(camToPlatform[rotated].count(cam) == 0);
			const uint32_t platformID = (uint32_t)iface.platforms.size();
			camToPlatform[rotated][cam] = platformID;
			MVSPlatform& platform = iface.platforms.emplace_back();
			platform.name = cam->metadata.name;
			MVSCamera& outCam = platform.cameras.emplace_back();
			outCam.width = (uint32_t)cam->GetWidth();
			outCam.height = (uint32_t)cam->GetHeight();
			// Camera extrinsics relative to platform: identity (platform pose holds absolute)
			outCam.R = Mat33d::eye();
			outCam.C = Pos3d(0, 0, 0);
			if (cam->GetType() == CameraType::SPHERICAL) {
				// the angular camera matrix of the equirectangular images, never rotated (FileOrientation)
				ASSERT(!rotated);
				const Spherical<REAL> sphere(Spherical<REAL>::FromSize(cam->GetWidth(), cam->GetHeight()));
				outCam.model = MVSCamera::SPHERICAL;
				outCam.K = Mat33d::eye();
				outCam.K(0,0) = sphere.fx; outCam.K(1,1) = sphere.fy;
				outCam.K(0,2) = sphere.cx; outCam.K(1,2) = sphere.cy;
				continue;
			}
			ASSERT(cam->GetType() == CameraType::PINHOLE);
			// Build intrinsic matrix
			KMatrix K(cam->GetK());
			BrownDistortion<REAL> distortion{};
			if (cam->HasDistortion()) {
				// the cameras undistorted on export have their undistorted images (UndistortImages
				// skips the invalid cameras); the others are exported with their distortion
				const auto itK = undistortK.find(cam);
				if (itK != undistortK.end()) {
					K = itK->second;
				} else {
					distortion = static_cast<const PinholeCamera*>(cam)->GetDistortion();
					outCam.model = MVSCamera::BROWN;
					if (camToPlatform[1-rotated].count(cam) == 0)
						++numBrownCams; // counted once per SFM camera
				}
			}
			if (rotated) {
				// the portrait files of the images rotated on load
				const cv::Size size(RevertRotation(*pImage, K, distortion));
				outCam.width = (uint32_t)size.width;
				outCam.height = (uint32_t)size.height;
			}
			outCam.K = K;
			if (outCam.model == MVSCamera::BROWN)
				outCam.params = BrownDistortionToParams(distortion);
		}
	}
	if (cubeMap)
		EmitSphericalPlatforms(scene, sphericalGeom, iface, camToPlatform[0]);

	// ----- Images and poses -----
	// sfmToMvsImage[i] holds the images-array positions produced for SFM image i
	// (the values Vertex::View::imageID references, distinct from the global Image::ID).
	// Pinhole images get 1 entry; spherical images get N (== numFaces).
	const String basePath = mvsFileDir;
	std::vector<std::vector<uint32_t>> sfmToMvsImage(scene.images.size());
	iface.images.reserve(scene.images.size());
	// Global image IDs: pinhole images keep their SFM image ID so external per-image data
	// (e.g. the pose-quality CSV) can be correlated after import; spherical cube-map faces
	// draw fresh IDs past the largest SFM ID (depth-map file naming requires uniqueness).
	// Vertex::View::imageID must stay the images-array position, tracked separately.
	uint32_t nextSyntheticID = 0;
	for (const Image& img : scene.images)
		if (img.ID != NO_ID && img.ID >= nextSyntheticID)
			nextSyntheticID = img.ID + 1;
	FOREACH(i, scene.images) {
		const Image& img = scene.images[i];
		const Camera* cam = img.pCamera;
		ASSERT(cam != NULL);

		if (cam->GetType() == CameraType::SPHERICAL && cubeMap) {
			if (!ExportSphericalFaces(img, sphericalGeom, cubeMapOutputDir, basePath, config.extension,
				camToPlatform[0].at(cam), iface, sfmToMvsImage[i], nextSyntheticID))
				return false;
			continue;
		}

		const uint32_t platformID = camToPlatform[FileOrientation(img)].at(cam);
		MVSPlatform& platform = iface.platforms[platformID];
		uint32_t poseID = MVS::NO_ID;
		if (img.IsValid()) {
			// Pose3D stores R (world->camera) and C (camera center world coordinates)
			poseID = (uint32_t)platform.poses.size();
			auto& pose = platform.poses.emplace_back(img.R, img.C);
			img.RevertRotation(NULL, &pose.R); // revert any image rotation
		}
		// Build Interface::Image record
		// the images of an undistorted camera reference their undistorted files (UndistortImages),
		// the others their original file
		MVS::Interface::Image outImg;
		outImg.name = img.fileName;
		if (undistortK.count(cam) != 0) {
			// UndistortImages fails on a posed image it can not undistort; an unposed image whose
			// file can not be read is unused and keeps its path
			ASSERT(!undistortedPaths[i].empty() || !img.IsValid());
			if (!undistortedPaths[i].empty())
				outImg.name = undistortedPaths[i];
		}
		outImg.name = MAKE_PATH_REL(basePath, outImg.name);
		outImg.platformID = platformID;
		outImg.cameraID = 0; // single camera per platform
		outImg.poseID = poseID; // may be NO_ID
		outImg.ID = (img.ID != NO_ID ? img.ID : nextSyntheticID++);
		sfmToMvsImage[i].push_back((uint32_t)iface.images.size()); // vertex views reference the array position
		iface.images.emplace_back(std::move(outImg));
	}

	// ----- Vertices -----
	// For each inlier track, expand every observation through the
	// sfmToMvsImage map: pinhole observations become a single view entry;
	// spherical observations become one entry per face that sees the 3D
	// point (0..6 per observation).
	iface.vertices.reserve(scene.tracks.size());
	bool emitColors = config.includeColors;
	if (emitColors)
		emitColors = !scene.colors.empty();
	if (emitColors)
		iface.verticesColor.reserve(scene.tracks.size());
	if (config.includeObservations)
		iface.flags |= MVSInterface::HAS_OBSERVATIONS;
	// the keypoint of an observation in the exported image file of a single-view (not cube-map) image
	const auto FeatureInFile = [&](const Image& img, const Point2f& kp) -> MVS::Interface::Pos2f {
		Point2f x(kp);
		const auto itK = undistortK.find(img.pCamera);
		if (itK != undistortK.end()) {
			// the undistorted image file: the undistorted ray of the keypoint through the undistorted camera
			const Point3 ray(img.pCamera->Unproject(Point2(kp)));
			const KMatrix& Ku = itK->second;
			const Eigen::Vector2d u(Pinhole<REAL>{Ku(0,0), Ku(1,1), Ku(0,2), Ku(1,2)}.NormalizedToPixel(Eigen::Vector2d(ray.x, ray.y)));
			x = Point2f(Point2(u));
		}
		x = img.ToOriginalOrientation(x);
		return MVS::Interface::Pos2f(x.x, x.y);
	};
	FOREACH(i, scene.tracks) {
		const Track& track = scene.tracks[i];
		if (!track.IsValid())
			continue;
		if (config.onlyInlierTracks && !track.IsInlier())
			continue;
		MVSVertex v;
		v.X = Cast<float>(track.position);
		v.views.reserve(track.GetNumInliers());
		for (const Observation& obs : track) {
			const uint32_t sfmImageID = obs.imageID;
			if (sfmImageID >= sfmToMvsImage.size())
				continue;
			const std::vector<uint32_t>& mvsIDs = sfmToMvsImage[sfmImageID];
			if (mvsIDs.empty())
				continue;
			const Image& srcImg = scene.images[sfmImageID];
			if (config.includeObservations && obs.featureID >= srcImg.keypoints.size()) {
				VERBOSE("error: observations requested, but image %u has no keypoint %u", sfmImageID, obs.featureID);
				return false;
			}
			if (srcImg.pCamera->GetType() == CameraType::SPHERICAL && cubeMap) {
				Point3 featureRay;
				if (config.includeObservations)
					featureRay = srcImg.pCamera->UnprojectNormalized(Point2(srcImg.keypoints[obs.featureID].pt));
				ProjectTrackOntoSphericalFaces(
					track.position, srcImg, sphericalGeom, mvsIDs, config.includeObservations ? &featureRay : NULL, v);
			} else {
				MVSVertex::View view;
				view.imageID = mvsIDs.front();
				view.confidence = 0.f;
				if (config.includeObservations)
					view.x = FeatureInFile(srcImg, Point2f(srcImg.keypoints[obs.featureID].pt));
				v.views.push_back(view);
			}
		}
		// A vertex with <2 views is unusable for dense reconstruction; skip it.
		if (v.views.size() < 2)
			continue;
		iface.vertices.emplace_back(std::move(v));
		if (emitColors) {
			MVS::Interface::Color col; // BGR order
			col.c = scene.colors[i];
			iface.verticesColor.push_back(col);
		}
	}

	// ----- Serialize -----
	if (!MVS::ARCHIVE::SerializeSave(iface, fileName.c_str())) {
		VERBOSE("error: failed serialization for '%s'", fileName.c_str());
		return false;
	}
	DEBUG("Scene exported as MVS: %u platforms, %u images (%u valid), %u points%s%s to '%s' (%s)",
		(unsigned)iface.platforms.size(), (unsigned)iface.images.size(), scene.status.nCalibratedImages, (unsigned)iface.vertices.size(),
		undistort ? " (undistorted)" : numBrownCams ? " (Brown cameras)" : "", cubeMap ? " (spherical cube-map expanded)" : "",
		fileName.c_str(), TD_TIMER_GET_FMT().c_str());
	return true;
} // ExportMVS
/*----------------------------------------------------------------*/
