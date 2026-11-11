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

#ifdef _USE_OPENMP
#define INTERFACEMVS_USE_OPENMP
#endif

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

bool SFM::UndistortDMAP(const String& depthMapFile,
	const UndistortMaps& maps, const KMatrix& imageUndistortedK)
{
	// Import depth-map
	String imageFileName;
	IIndexArr IDs;
	cv::Size imageSize, depthSize;
	KMatrix K;
	RMatrix R;
	CMatrix C;
	float dMin, dMax;
	Image32F depthMap, confMap;
	Image32F3 normalMap;
	Image8U4 viewsMap;
	bool bConfAdjusted(false);
	if (!ImportDepthDataRaw(depthMapFile, imageFileName, IDs, imageSize, depthSize, K, R, C, dMin, dMax,
			depthMap, normalMap, confMap, viewsMap, 15/*all maps*/, &bConfAdjusted)) {
		DEBUG("warning: failed to import depth-map from '%s'", depthMapFile.c_str());
		return false;
	}
	if (!normalMap.empty()) {
		DEBUG("warning: undistortion of normal-maps not implemented yet, skipping");
	}
	// Undistort depth-map using depth-aware interpolation at its native resolution
	Image32F undistortedDepthMap;
	maps.RemapDepth(depthMap, undistortedDepthMap);
	// Undistort confidence and views-map if available (using nearest neighbor interpolation)
	Image32F undistortedConfMap;
	if (!confMap.empty())
		maps.Remap(confMap, undistortedConfMap, cv::INTER_NEAREST);
	Image8U4 undistortedViewsMap;
	if (!viewsMap.empty())
		maps.Remap(viewsMap, undistortedViewsMap, cv::INTER_NEAREST);
	// Export undistorted depth-map to a temporary file first; undistortion resamples the
	// confidence but does not change what it measures, so a recalibrated map stays
	// recalibrated and keeps its CONF_ADJUSTED flag
	const String tempDepthMapFile = depthMapFile + ".tmp";
	if (!ExportDepthDataRaw(tempDepthMapFile, imageFileName, IDs, imageSize,
			imageUndistortedK, R, C, dMin, dMax,
			undistortedDepthMap, undistortedConfMap, undistortedViewsMap, bConfAdjusted)) {
		DEBUG("warning: failed to export undistorted depth-map to '%s'", tempDepthMapFile.c_str());
		return false;
	}
	// Rename temporary file to the original file
	if (!File::renameFile(tempDepthMapFile, depthMapFile)) {
		DEBUG("warning: failed to rename '%s' to '%s'", tempDepthMapFile.c_str(), depthMapFile.c_str());
		// Try to remove the temporary file
		File::deleteFile(tempDepthMapFile);
	}
	return true;
}

bool SFM::UndistortDepthMaps(const Scene& scene,
	const CLISTDEF2(String)& depthMapFiles,
	double alpha,
	std::unordered_map<const Camera*, KMatrix>* undistortedIntrinsics)
{
	ASSERT(!depthMapFiles.empty());
	TD_TIMER_STARTD();
	// per camera: the maps at the depth-map resolution and the undistorted K at the image resolution
	std::unordered_map<const Camera*, std::pair<UndistortMaps,KMatrix>> undistortMaps;
	undistortMaps.reserve(scene.cameras.size());

	// For each image/depth-map, compute maps at the depth resolution using K scaled to that resolution
	#ifdef INTERFACEMVS_USE_OPENMP
	#pragma omp parallel for schedule(dynamic)
	#endif
	for (int_t _i = 0; _i < (int_t)scene.images.size(); ++_i) {
		const IIndex i = static_cast<IIndex>(_i);
		const String& dfile = depthMapFiles[i];
		if (dfile.empty())
			continue;
		const Image& img = scene.images[i];
		if (!img.IsValid())
			continue;
		const Camera* cam = img.pCamera;
		if (cam->GetType() != CameraType::PINHOLE)
			continue; // only pinhole supported
		const PinholeCamera* pc = static_cast<const PinholeCamera*>(cam);
		if (!pc->HasDistortion())
			continue; // nothing to undistort for depth-map
		// Check if undistortion maps already computed for this camera
		const std::pair<UndistortMaps,KMatrix>& data = [&]() -> const std::pair<UndistortMaps,KMatrix>& {
			std::pair<UndistortMaps,KMatrix>* pData = nullptr;
			#ifdef INTERFACEMVS_USE_OPENMP
			#pragma omp critical
			#endif
			{
				pData = &undistortMaps[pc];
				if (pData->first.map1.empty()) {
					// read only the header to obtain the image size and depth-map resolution;
					// flags=0 requests the meta-data alone, leaving the map arguments untouched
					MVS::DepthDataRaw meta;
					cv::Mat unusedMap;
					if (!MVS::ImportDepthDataRaw(static_cast<const std::string&>(dfile), meta,
							unusedMap, unusedMap, unusedMap, unusedMap, 0)) {
						DEBUG("warning: failed to read depth header '%s'", dfile.c_str());
						exit(EXIT_FAILURE);
					}
					const cv::Size imageSize((int)meta.header.imageWidth, (int)meta.header.imageHeight);
					const cv::Size depthSize((int)meta.header.depthWidth, (int)meta.header.depthHeight);
					// the undistorted camera of the images (the one the MVS scene uses), scaled
					// to the depth-map resolution, so the depth-map lands on its pixel grid
					ASSERT(imageSize == pc->GetSize());
					pData->second = UndistortedK(pc->GetK(), pc->GetDistortion(), imageSize, alpha);
					const KMatrix Kdepth = ScaleK(pc->GetK(), imageSize, depthSize);
					pData->first.Init(Kdepth, pc->GetDistortion(), depthSize, ScaleK(pData->second, imageSize, depthSize));
					if (undistortedIntrinsics)
						undistortedIntrinsics->emplace(pc, pData->second);
				}
			}
			return *pData;
		}();
		// Undistort DMAP with depth-aware interpolation and write back
		if (!UndistortDMAP(dfile, data.first, data.second)) {
			DEBUG("warning: undistort depth failed for '%s'", dfile.c_str());
			continue;
		}
		DEBUG_EXTRA("Depth-map undistorted for image %d", i);
	}
	DEBUG("Depth-maps undistorted successfully in %s", TD_TIMER_GET_FMT().c_str());
	return true;
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
	std::unordered_map<PairIdx::PairIndex, IIndex> camToID;
	camToID.reserve(iface.platforms.size());
	const auto DeduceSize = [&](uint32_t pid, uint32_t cid)->cv::Size {
		for (const auto& img : iface.images) {
			if (img.platformID != pid || img.cameraID != cid)
				continue;
			IMAGEPTR pImage(CImage::Create(basePath + img.name, CImage::READ));
			const bool valid = (pImage != NULL && pImage->ReadHeader());
			pImage.Release();
			if (valid)
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
			camToID.emplace(PairIdx(pid, cid).idx, camID);
		}
	}

	scene.images.reserve(iface.images.size());
	uint32_t posed = 0;
	for (size_t i = 0; i < iface.images.size(); ++i) {
		const auto& inImg = iface.images[i];
		const auto key = PairIdx(inImg.platformID, inImg.cameraID).idx;
		auto itCam = camToID.find(key);
		if (itCam == camToID.end()) {
			VERBOSE("error: skipping image %zu (platform %u camera %u not found)", i, inImg.platformID, inImg.cameraID);
			continue;
		}
		const IIndex camID = itCam->second;
		const String imgPath = basePath + inImg.name;
		Image& img = scene.images.emplace_back(
		    (inImg.ID != MVS::NO_ID) ? inImg.ID : scene.images.size(),
			imgPath);
		img.cameraID = camID;
		img.pCamera = scene.cameras[camID];
		if (inImg.IsValid()) {
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
			if (vw.imageID >= scene.images.size())
				continue;
			track.observations.emplace_back(vw.imageID, NO_ID);
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

// Phase 1: one rig Platform per spherical SFM camera, N face Cameras each.
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

// Phase 2: append pose + N face Image records for one spherical SFM image.
// mvsImagePositions receives the array positions of the created records (used as
// Vertex::View::imageID); the global IDs are drawn from nextID (unique per image,
// required by depth-map file naming).
void AppendSphericalFaceImages(
	const Image& img,
	const TangentFacesGeometry& geometry,
	const String& basePath,
	const String& extension,
	uint32_t platformID,
	MVS::Interface& iface,
	std::vector<uint32_t>& mvsImagePositions,
	uint32_t& nextID)
{
	ASSERT(img.pCamera != NULL);
	ASSERT(img.pCamera->GetType() == CameraType::SPHERICAL);
	ASSERT(platformID < iface.platforms.size());
	ASSERT(geometry.numFaces > 0);

	MVS::Interface::Platform& platform = iface.platforms[platformID];
	const uint32_t poseID = (uint32_t)platform.poses.size();
	MVS::Interface::Platform::Pose& pose = platform.poses.emplace_back();
	pose.R = img.R;
	pose.C = img.C;

	const String stem = Util::getFileName(img.fileName);
	mvsImagePositions.clear();
	mvsImagePositions.reserve(geometry.numFaces);
	for (int k = 0; k < geometry.numFaces; ++k) {
		MVS::Interface::Image& outImg = iface.images.emplace_back();
		outImg.name = basePath + stem + String::FormatString(_T("_face%d"), k) + extension;
		outImg.platformID = platformID;
		outImg.cameraID   = (uint32_t)k;
		outImg.poseID     = poseID;
		outImg.ID         = nextID++;
		mvsImagePositions.push_back((uint32_t)(iface.images.size() - 1));
	}
}

// Phase 3: render + save N face images for every spherical source image.
// Geometry is shared across all calls; each image pays only the per-pixel
// render cost (not the rotation-table build). Fails on the first panorama
// that can not be read or face that can not be written, since the exported
// scene already references every face file.
bool RenderAndSaveSphericalFaces(
	const Scene& scene,
	const TangentFacesGeometry& geometry,
	const String& extension,
	const String& outputDir)
{
	ASSERT(!outputDir.empty() && geometry.numFaces > 0);
	String outDir = outputDir;
	Util::ensureValidFolderPath(outDir);
	Util::ensureFolder(outDir);
	unsigned written = 0;
	for (const Image& img : scene.images) {
		if (img.pCamera == NULL || img.pCamera->GetType() != CameraType::SPHERICAL)
			continue;

		bool loadedHere = false;
		if (!img.HasPixels()) {
			const_cast<Image&>(img).LoadPixels();
			loadedHere = true;
		}
		if (!img.HasPixels()) {
			VERBOSE("error: can not read the panorama '%s' to render its cube-map faces", img.fileName.c_str());
			return false;
		}

		// Normalize pixel format to BGR/8U if needed and
		// render all N faces in one call, consuming the shared geometry
		const std::vector<Image8U3> faces =
			SphereCubeMap::SphericalToTangentialFaces<Pixel8U>(img.GetImage8U3(), geometry);
		if (loadedHere)
			const_cast<Image&>(img).ReleasePixels();

		const String stem = Util::getFileName(img.fileName);
		for (int k = 0; k < (int)faces.size(); ++k) {
			const String facePath = outDir + stem +
				String::FormatString(_T("_face%d"), k) + extension;
			if (!faces[k].Save(facePath)) {
				VERBOSE("error: can not write the cube-map face '%s'", facePath.c_str());
				return false;
			}
			++written;
		}
	}
	DEBUG("SphereCubeMap: wrote %u face images to '%s'", written, outDir.c_str());
	return true;
}

// Phase 4: project 3D world point X through each face of one spherical
// image's rig pose; push a Vertex::View entry for every face that sees it;
// pFeatureRay, if given, is the ray of the feature (panorama frame), mapped
// to each face as the view observation.
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

	for (int k = 0; k < geometry.numFaces; ++k) {
		const Point3 X_face = geometry.rotations[k] * X_body;
		if (X_face.z < zEps)
			continue;
		const REAL u = f * X_face.x / X_face.z + cx;
		const REAL v = f * X_face.y / X_face.z + cy;
		if (u < REAL(0) || u >= REAL(geometry.faceSize)) continue;
		if (v < REAL(0) || v >= REAL(geometry.faceSize)) continue;
		MVS::Interface::Vertex::View view;
		view.imageID    = faceImageIDs[k];
		view.confidence = 0.f;
		if (pFeatureRay) {
			const Point3 ray = geometry.rotations[k] * (*pFeatureRay);
			ASSERT(ray.z > 0);
			view.x = MVS::Interface::Pos2f(float(f * ray.x / ray.z + cx), float(f * ray.y / ray.z + cy));
		}
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

	// Resolve the face output directory for cube-map images. Prefer the
	// caller-provided undistort dir (mirrors the existing "outputs next to
	// the .mvs file" contract); if empty default to the .mvs file's own
	// directory so everything stays self-contained.
	const String mvsFileDir = MAKE_PATH_FULL(WORKING_FOLDER_FULL, Util::getFilePath(fileName));
	String cubeMapOutputDir;
	if (cubeMap)
		cubeMapOutputDir = config.undistortImageDir.empty() ? mvsFileDir : config.undistortImageDir;

	// Map each SFM camera pointer to its (possibly multi-camera) platform.
	std::unordered_map<const Camera*, uint32_t> camToPlatform;
	camToPlatform.reserve(scene.cameras.size());

	const bool undistort = !config.undistortImageDir.empty();
	std::unordered_map<const Camera*, KMatrix> undistortK;
	CLISTDEF2(String) undistortedPaths;
	if (undistort)
		scene.UndistortImages(config.undistortImageDir, config.extension,
		config.undistortAlpha, &undistortedPaths, &undistortK);

	// ----- Phase 1: platforms + cameras -----
	// Pinhole and native spherical: one platform with a single camera per SFM camera.
	// Cube-map spherical: one platform with N face cameras via EmitSphericalPlatforms.
	unsigned numBrownCams = 0;
	for (CameraPtr const cam : scene.cameras) {
		ASSERT(camToPlatform.count(cam) == 0);
		if (cam->GetType() == CameraType::SPHERICAL && cubeMap)
			continue; // deferred to EmitSphericalPlatforms below
		const uint32_t platformID = (uint32_t)iface.platforms.size();
		camToPlatform[cam] = platformID;
		MVSPlatform& platform = iface.platforms.emplace_back();
		platform.name = cam->metadata.name;
		MVSCamera& outCam = platform.cameras.emplace_back();
		outCam.width = (uint32_t)cam->GetWidth();
		outCam.height = (uint32_t)cam->GetHeight();
		// Camera extrinsics relative to platform: identity (platform pose holds absolute)
		outCam.R = Mat33d::eye();
		outCam.C = Pos3d(0, 0, 0);
		if (cam->GetType() == CameraType::SPHERICAL) {
			// the angular camera matrix of the equirectangular images
			const Spherical<REAL> sphere(Spherical<REAL>::FromSize(cam->GetWidth(), cam->GetHeight()));
			outCam.model = MVSCamera::SPHERICAL;
			outCam.K = Mat33d::eye();
			outCam.K(0,0) = sphere.fx; outCam.K(1,1) = sphere.fy;
			outCam.K(0,2) = sphere.cx; outCam.K(1,2) = sphere.cy;
			continue;
		}
		ASSERT(cam->GetType() == CameraType::PINHOLE);

		// Build intrinsic matrix
		outCam.K = cam->GetK();
		BrownDistortion<REAL> distortion{};
		if (cam->HasDistortion()) {
			// the cameras undistorted on export have their undistorted images (UndistortImages
			// skips the invalid cameras); the others are exported with their distortion
			const auto itK = undistortK.find(cam);
			if (itK != undistortK.end()) {
				outCam.K = itK->second;
			} else {
				distortion = static_cast<const PinholeCamera*>(cam)->GetDistortion();
				outCam.model = MVSCamera::BROWN;
				++numBrownCams;
			}
		}
		// Find first image using this camera to deduce image rotation
		for (const Image& img : scene.images) {
			if (img.pCamera != cam)
				continue;
			if (!img.IsRotated())
				break;
			// Image was rotated at load; adjust intrinsic to match the original orientation
			const cv::Size size = img.RevertRotation(&outCam.K);
			outCam.width = (uint32_t)size.width;
			outCam.height = (uint32_t)size.height;
			// the original normalized coordinates are (y,-x) of the working ones: the radial terms
			// are invariant, the tangential ones become p1'=-p2, p2'=p1
			const REAL p1 = distortion.p1;
			distortion.p1 = -distortion.p2;
			distortion.p2 = p1;
			break;
		}
		if (outCam.model == MVSCamera::BROWN) {
			outCam.params = {distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3};
			if (distortion.k4 != 0 || distortion.k5 != 0 || distortion.k6 != 0)
				outCam.params.insert(outCam.params.end(), {distortion.k4, distortion.k5, distortion.k6});
		}
	}
	if (cubeMap)
		EmitSphericalPlatforms(scene, sphericalGeom, iface, camToPlatform);

	// ----- Phase 2: images + poses -----
	// sfmToMvsImage[i] holds the images-array positions produced for SFM image i
	// (the values Vertex::View::imageID references, distinct from the global Image::ID).
	// Pinhole images get 1 entry; spherical images get N (== numFaces).
	const String basePath = mvsFileDir;
	String faceBaseSlash;
	if (cubeMap) {
		faceBaseSlash = MAKE_PATH_REL(basePath, cubeMapOutputDir);
		if (!faceBaseSlash.empty() && faceBaseSlash.back() != PATH_SEPARATOR)
			faceBaseSlash += PATH_SEPARATOR;
	}
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
			AppendSphericalFaceImages(
				img, sphericalGeom, faceBaseSlash, config.extension,
				camToPlatform[cam], iface, sfmToMvsImage[i], nextSyntheticID);
			continue;
		}

		const uint32_t platformID = camToPlatform[cam];
		MVSPlatform& platform = iface.platforms[platformID];
		uint32_t poseID = MVS::NO_ID;
		if (img.IsValid()) {
			// Pose3D stores R (world->camera) and C (camera center world coordinates)
			poseID = (uint32_t)platform.poses.size();
			auto& pose = platform.poses.emplace_back(img.R, img.C);
			img.RevertRotation(NULL, &pose.R); // revert any image rotation
		}
		// Build Interface::Image record
		// Use the undistorted image path for the undistorted cameras, otherwise the original
		MVS::Interface::Image outImg;
		const bool undistorted(undistortK.count(cam) != 0);
		outImg.name = undistorted ? undistortedPaths[i] : img.fileName;
		if (undistorted && outImg.name.empty() && img.IsValid()) {
			// the image file could not be read for undistortion
			VERBOSE("error: no undistorted image of '%s'", img.fileName.c_str());
			return false;
		}
		outImg.name = MAKE_PATH_REL(basePath, outImg.name);
		outImg.platformID = platformID;
		outImg.cameraID = 0; // single camera per platform
		outImg.poseID = poseID; // may be NO_ID
		outImg.ID = (img.ID != NO_ID ? img.ID : nextSyntheticID++);
		sfmToMvsImage[i].push_back((uint32_t)iface.images.size()); // vertex views reference the array position
		iface.images.emplace_back(std::move(outImg));
	}

	// ----- Phase 3: render + write cube-map face pixels -----
	if (cubeMap) {
		if (!RenderAndSaveSphericalFaces(scene, sphericalGeom, config.extension, cubeMapOutputDir))
			return false;
	}

	// ----- Phase 4: vertices (tracks) -----
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
			x = Point2f((float)u.x(), (float)u.y());
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

	// ----- Phase 5: serialize -----
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
