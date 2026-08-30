////////////////////////////////////////////////////////////////////
// Scene.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Scene.h"
#include "Image.h"
#include "../Math/GeodeticTransforms.h"
#include "Track.h"
#include "Triangulation.h"
#include "SceneCluster.h"
#include "StarInitializer.h"
#include "Resection.h"
#include "BundleAdjustment.h"
#include "GlobalAlignment.h"
#include "GlobalDescriptors.h"
#include "GlobalRotationAveraging.h"
#include "GlobalPositioning.h"
#include "SimilarityTransform.h"
#include "InterfaceMVS.h"
#include "ImportCOLMAP.h"
#include "RoMa2Matcher.h"

#include <TinyEXIF.h>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define SCENE_USE_OPENMP
#endif

#define SFM_PROJECT_ID "SFM\0" // identifies the SFM project stream
#define SFM_PROJECT_VERSION 1  // SFM project stream layout version (bump on any breaking header/serialization change)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Scene   "));

// Translate the reconstruction intrinsic flags into the matching bundle-adjustment switches
static void SetBAIntrinsicFlags(BAConfig& baCfg, unsigned baIntrinsicFlags)
{
	baCfg.refineFocalLength = (baIntrinsicFlags & ReconstructionConfig::INTRINSIC_FOCAL_LENGTH) != 0;
	baCfg.refineFocalLengthAspectRatio = (baIntrinsicFlags & ReconstructionConfig::INTRINSIC_FOCAL_LENGTH_ASPECT_RATIO) != 0;
	baCfg.refinePrincipalPoint = (baIntrinsicFlags & ReconstructionConfig::INTRINSIC_PRINCIPAL_POINT) != 0;
	baCfg.refineRadialDistortion123 = (baIntrinsicFlags & ReconstructionConfig::INTRINSIC_RADIAL_DIST_123) != 0;
	baCfg.refineTangentialDistortion = (baIntrinsicFlags & ReconstructionConfig::INTRINSIC_TANGENTIAL_DIST) != 0;
	baCfg.refineRadialDistortion456 = (baIntrinsicFlags & ReconstructionConfig::INTRINSIC_RADIAL_DIST_456) != 0;
}


Scene::Scene(unsigned _nMaxThreads)
	: transform(Matrix4x4::IDENTITY), obb(true), nMaxThreads(Thread::getMaxThreads(_nMaxThreads)),
	  threadPool(nMaxThreads)
{
	#ifdef _USE_OPENMP
	if (nMaxThreads != 0)
		omp_set_num_threads(nMaxThreads);
	#endif
}
Scene::Scene(const Scene& scene)
	: Scene(scene.nMaxThreads)
{
	*this = scene;
}

Scene::Scene(Scene&& scene) noexcept
	: Scene(scene.nMaxThreads)
{
	*this = std::move(scene);
}

Scene& Scene::operator=(const Scene& scene) {
	if (this == &scene)
		return *this;
	Release();
	// Copy cameras using Clone()
	for (const Camera* cam : scene.cameras) {
		Camera* camCopy = cam->Clone();
		cameras.emplace_back(camCopy);
	}
	// Copy images
	for (const Image& img : scene.images) {
		Image& imgCopy = images.emplace_back(img);
		imgCopy.pCamera = cameras[imgCopy.cameraID];
	}
	// Copy pairs
	pairs = scene.pairs;
	// Copy tracks
	for (const Track& track : scene.tracks)
		tracks.emplace_back(track);
	// Copy status
	colors = scene.colors;
	poseUncertainty = scene.poseUncertainty;
	priorPoses = scene.priorPoses;
	transform = scene.transform;
	obb = scene.obb;
	status = scene.status;
	return *this;
}

Scene& Scene::operator=(Scene&& scene) noexcept {
	if (this == &scene)
		return *this;
	Release();
	cameras = std::move(scene.cameras);
	images = std::move(scene.images);
	pairs = std::move(scene.pairs);
	tracks = std::move(scene.tracks);
	colors = std::move(scene.colors);
	poseUncertainty = std::move(scene.poseUncertainty);
	priorPoses = std::move(scene.priorPoses);
	transform = scene.transform;
	obb = scene.obb;
	status = scene.status;
	return *this;
}

void Scene::Release() {
	// Delete all cameras
	cameras.ReleaseDelete();
	images.Release();
	pairs.Release();
	tracks.Release();
	colors.clear();
	poseUncertainty.Release();
	priorPoses.clear();
	transform = Matrix4x4::IDENTITY;
	obb = OBB3(true);
	status = Status();
}


bool Scene::HasImagesWithGPS(bool validOnly) const {
	for (const Image& img : images) {
		if (validOnly && !img.IsValid())
			continue;
		if (img.View::metadata.HasGPS())
			return true;
	}
	return false;
}


bool Scene::InvalidateImage(IIndex imgID)
{
	ASSERT(imgID < images.size());
	Image& image = images[imgID];
	if (!image.IsValid())
		return false;
	image.InvalidatePose();
	// Set this image as outlier from any tracks it is inlier in
	for (Track& track : tracks) {
		if (!track.IsInlier())
			continue;
		RFOREACHRAW(i, track.numInliers) {
			if (track.observations[i].imageID == imgID) {
				if (--track.numInliers != i)
					std::swap(track.observations[track.numInliers], track.observations[i]);
				break;
			}
		}
	}
	--status.nCalibratedImages;
	return true;
}
unsigned Scene::InvalidateImages(const IIndexArr& imgIDs)
{
	// Mark every requested valid image as dropped, then demote all of them from the
	// tracks in a single sweep (the per-image InvalidateImage would sweep all tracks once
	// per image; here N drops cost one sweep). A track may lose several observations at
	// once, so iterate its inlier prefix in reverse: each swap-with-last stays valid under
	// the shrinking count, unlike the single-image version that can break after one hit.
	std::vector<uint8_t> drop(images.size(), 0);
	unsigned n = 0;
	for (const IIndex id : imgIDs) {
		ASSERT(id < images.size());
		Image& image = images[id];
		if (image.IsValid()) {
			image.InvalidatePose();
			drop[id] = 1;
			++n;
		}
	}
	if (n == 0)
		return 0;
	for (Track& track : tracks) {
		if (!track.IsInlier())
			continue;
		RFOREACHRAW(i, track.numInliers)
			if (drop[track.observations[i].imageID])
				if (--track.numInliers != i)
					std::swap(track.observations[track.numInliers], track.observations[i]);
	}
	status.nCalibratedImages -= n;
	return n;
}
// Rescan all images and refresh status.nCalibratedImages; the counter is normally
// delta-maintained by the incremental solvers (StarInitializer/Resection/InvalidateImage),
// so this is only needed by paths that set poses in bulk
uint32_t Scene::RecomputeCalibratedImages()
{
	status.nCalibratedImages = 0;
	for (const Image& image : images)
		if (image.IsValid())
			++status.nCalibratedImages;
	return status.nCalibratedImages;
}
bool Scene::Save(const String& fileName, ARCHIVE_TYPE nArchiveType) const
{
	#ifdef _USE_BOOST
	TD_TIMER_STARTD();
	// open the output stream
	std::ofstream fs(fileName, std::ios::out | std::ios::binary);
	if (!fs.is_open()) {
		VERBOSE("error: unable to open file '%s'", fileName.c_str());
		return false;
	}
	// save project ID
	fs.write(SFM_PROJECT_ID, 4);
	// save stream type (compression layer used by boost serialization)
	const uint32_t nType = nArchiveType;
	fs.write((const char*)&nType, sizeof(uint32_t));
	// save the stream layout version so future changes can be detected/rejected
	const uint32_t nVersion = SFM_PROJECT_VERSION;
	fs.write((const char*)&nVersion, sizeof(uint32_t));
	// reserve some bytes
	const uint32_t nReserved = 0;
	fs.write((const char*)&nReserved, sizeof(uint32_t));
	// serialize out the current state
	if (!SerializeSave(*this, fs, nArchiveType)) {
		VERBOSE("error: serialization failed for file '%s' (archive type %d)", fileName.c_str(), (int)nArchiveType);
		return false;
	}
	DEBUG_EXTRA("Scene saved (%s): %u cameras, %u images (%u calibrated), %u pairs, %u tracks",
				TD_TIMER_GET_FMT().c_str(),
				cameras.size(), images.size(), status.nCalibratedImages, pairs.size(), tracks.size());
	return true;
	#else
	VERBOSE("error: boost serialization not available");
	return false;
	#endif
}

bool Scene::Load(const String& fileName)
{
	#ifdef _USE_BOOST
	TD_TIMER_STARTD();
	// open the input stream
	std::ifstream fs(fileName, std::ios::in | std::ios::binary);
	if (!fs.is_open()) {
		VERBOSE("error: unable to open file '%s'", fileName.c_str());
		return false;
	}
	// load and validate project header ID
	char szHeader[4];
	fs.read(szHeader, 4);
	if (!fs || strncmp(szHeader, SFM_PROJECT_ID, 4) != 0) {
		VERBOSE("error: invalid SFM project '%s'", fileName.c_str());
		return false;
	}
	// load stream type (compression layer used by boost serialization)
	uint32_t nType;
	fs.read((char*)&nType, sizeof(uint32_t));
	// load the stream layout version and reject any file not written by this exact layout:
	// the stream carries no per-class versions, so an older layout would silently desynchronize
	uint32_t nVersion;
	fs.read((char*)&nVersion, sizeof(uint32_t));
	// skip reserved bytes
	uint32_t nReserved;
	fs.read((char*)&nReserved, sizeof(uint32_t));
	if (!fs) {
		VERBOSE("error: invalid SFM project header '%s'", fileName.c_str());
		return false;
	}
	if (nVersion != SFM_PROJECT_VERSION) {
		VERBOSE("error: unsupported SFM project version %u (this build reads only version %u) in '%s'",
			nVersion, (unsigned)SFM_PROJECT_VERSION, fileName.c_str());
		return false;
	}
	// serialize in the current state
	if (!SerializeLoad(*this, fs, (ARCHIVE_TYPE)nType)) {
		VERBOSE("error: deserialization failed for file '%s' (archive type %d)", fileName.c_str(), (int)nType);
		return false;
	}
	DEBUG_EXTRA("Scene loaded (%s): %u cameras, %u images (%u calibrated), %u pairs, %u tracks",
				TD_TIMER_GET_FMT().c_str(),
				cameras.size(), images.size(), status.nCalibratedImages, pairs.size(), tracks.size());
	return true;
	#else
	VERBOSE("error: boost serialization not available");
	return false;
	#endif
}

bool Scene::Import(const String& source, const ImportConfig& config)
{
	// 1) Collect image file list (either semicolon-separated or directory)
	CLISTDEF2(String) imageFiles;
	// If source is a directory list files using std::filesystem (cross-platform)
	if (std::filesystem::is_directory(std::string(MAKE_PATH_SAFE(source)))) {
		// List image files in directory
		for (const auto& entry : std::filesystem::directory_iterator(std::string(MAKE_PATH_SAFE(source)))) {
			if (!entry.is_regular_file())
				continue;
			const String ext = String(entry.path().extension().string()).ToLower();
			if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".tif" || ext == ".tiff" || ext == ".jxl" || ext == ".exr" || ext == ".webp" || ext == ".heic" || ext == ".heif")
				imageFiles.emplace_back(entry.path().string());
		}
		if (!imageFiles.empty()) {
			// Sort files by path or numeric stem if possible
			Unsigned64Arr fileNumericStems(imageFiles.size());
			fileNumericStems.MemsetValue((uint64_t)-1);
			FOREACH(i, imageFiles) {
				const String stem = Util::getFileName(imageFiles[i]);
				if (stem.empty())
					continue;
				char* endPtr = nullptr;
				const char* cStr = stem.c_str();
				errno = 0;
				const long long parsed = std::strtoll(cStr, &endPtr, 10);
				if (errno != 0 || endPtr == cStr || *endPtr != '\0' || parsed < 0)
					continue;
				uint64_t value = static_cast<uint64_t>(parsed);
				fileNumericStems[i] = value;
			}
			IIndexArr sortedIndices(imageFiles.size());
			std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
			std::sort(sortedIndices.begin(), sortedIndices.end(), [&](IIndex a, IIndex b) {
				const uint64_t numA = fileNumericStems[a];
				const uint64_t numB = fileNumericStems[b];
				if (numA != (uint64_t)-1 && numB != (uint64_t)-1)
					return numA < numB;
				return imageFiles[a] < imageFiles[b];
			});
			CLISTDEF2(String) sortedFiles;
			sortedFiles.reserve(imageFiles.size());
			for (IIndex idx : sortedIndices)
				sortedFiles.push_back(std::move(imageFiles[idx]));
			imageFiles = std::move(sortedFiles);
		}
	} else {
		// If source contains semicolon, treat as list
		Util::strSplit(source, ';', imageFiles);
	}
	if (imageFiles.size() == 1 && Util::getFileExt(imageFiles.front()) == ".sfm" &&
		File::isFile(MAKE_PATH_SAFE(imageFiles.front())))
	{
		if (!Load(MAKE_PATH_SAFE(imageFiles.front())))
			return false;
	} else if (imageFiles.size() < 2) {
		VERBOSE("error: no input images found for '%s'", source.c_str());
		return false;
	}

	if (IsEmpty()) {
		// 2a) Load images and EXIF metadata; create per-image cameras
		images.resize(imageFiles.size());
		#ifdef SCENE_USE_OPENMP
		cv::setNumThreads(1); // temporary turn of multi-threading for OpenCV functions
		#pragma omp parallel for schedule(dynamic)
		#endif
		for (int_t _i = 0; _i < (int_t)images.size(); ++_i) {
			const IIndex i = (IIndex)_i;
			const String& imgPath = imageFiles[i];
			Image& img = images[i];
			img.ID = i;
			img.fileName = imgPath;
			Util::ensureValidPath(img.fileName);
			img.fileName = MAKE_PATH_FULL(WORKING_FOLDER_FULL, img.fileName);
			if (!img.LoadMetadata(config.defaultFocalRatio)) {
				VERBOSE("error: failed to load metadata for '%s'", imgPath.c_str());
				continue;
			}
		}
		#ifdef SCENE_USE_OPENMP
		cv::setNumThreads(nMaxThreads); // restore OpenCV threading
		#endif

		// 2b) Import camera poses from file (if configured); this runs before the camera
		// de-duplication below, so that identical per-frame imported intrinsics collapse
		// into a single shared camera
		if (!config.importPosesFile.empty() && config.importPosesMode != PoseImportMode::NONE &&
			!ImportPoses(*this, config.importPosesFile, config.importPosesMode, config.framesConvention))
			return false;

		// 2c) Cluster identical cameras (exact match) and assign shared cameras
		std::unordered_map<String, IIndex> camKeyToID;
		auto cameraKey = [](const Camera* cam)->String {
			const String type = CameraTypeToString(cam->GetType());
			String key = type + "|" + String::FormatString("%dx%d", cam->GetWidth(), cam->GetHeight());
			if (type == "Pinhole") {
				const PinholeCamera* pc = static_cast<const PinholeCamera*>(cam);
				key += String::FormatString("|%.9f|%.9f|%.9f|%.9f", pc->fx, pc->fy, pc->cx, pc->cy);
				key += String::FormatString("|%.9f|%.9f|%.9f|%.9f|%.9f|%.9f", pc->k1, pc->k2, pc->k3, pc->p1, pc->p2, pc->k4);
				key += String::FormatString("|%.9f|%.9f", pc->k5, pc->k6);
				key += pc->trustIntrinsics ? "|trusted" : "|untrusted";
			}
			// include metadata for strict grouping
			key += "|" + cam->metadata.name + "|" + cam->metadata.model;
			key += String::FormatString("|%.6f|%.6f", cam->metadata.sensorWidth, cam->metadata.sensorHeight);
			return key;
		};
		unsigned numTrustedCameras = 0;
		for (Image& img : images) {
			if (!img.HasCamera())
				continue;
			const String key = cameraKey(img.pCamera);
			auto it = camKeyToID.emplace(key, cameras.size());
			const IIndex camID = it.first->second;
			if (it.second) {
				// new camera
				cameras.emplace_back(img.pCamera);
				if (img.pCamera->TrustIntrinsics())
					++numTrustedCameras;
			} else {
				// reuse existing, delete duplicate
				SAFE_DELETE(img.pCamera);
				img.pCamera = cameras[camID];
			}
			img.cameraID = camID;
		}
		VERBOSE("Imported %u images, %u unique cameras (%u trusted intrinsics)",
		        images.size(), cameras.size(), numTrustedCameras);
	} else if (!config.importPosesFile.empty() && config.importPosesMode != PoseImportMode::NONE) {
		// The scene was resumed from a saved .sfm: the images already share de-duplicated
		// cameras, so per-frame intrinsics cannot be applied (they would overwrite a camera
		// shared by other images); import the poses only
		PoseImportMode mode = config.importPosesMode;
		if (mode == PoseImportMode::POSES_INTRINSICS) {
			VERBOSE("warning: intrinsics from '%s' are ignored when resuming from a saved scene; importing the poses only",
				config.importPosesFile.c_str());
			mode = PoseImportMode::POSES;
		}
		if (!ImportPoses(*this, config.importPosesFile, mode, config.framesConvention))
			return false;
	}

	// 2d) Apply forced focal length and distortion parameters to specified images (if configured)
	if (config.focalLength > 0.f || config.k1 != 0.f || config.k2 != 0.f) {
		IDXArr imageIndices;
		if (config.imageIndicesStr.empty()) {
			// Apply to all images
			imageIndices.resize(images.size());
			std::iota(imageIndices.begin(), imageIndices.end(), 0);
		} else {
			// Parse image indices
			const String errorMsg = Util::parseIndexRanges(config.imageIndicesStr, images.size(), imageIndices, "image");
			if (!errorMsg.empty()) {
				VERBOSE("error: parsing image indices (%s)", errorMsg.c_str());
				return false;
			}
		}
		// Convert to set for fast lookup
		std::unordered_set<IIndex> uniqueIndices;
		uniqueIndices.reserve(imageIndices.size());
		for (IDX idx : imageIndices)
			uniqueIndices.insert((IIndex)idx);
		// Count camera usage across all images
		std::unordered_map<CameraPtr, IIndexArr> cameraUsage;
		for (IIndex i = 0; i < images.size(); ++i) {
			if (images[i].HasCamera())
				cameraUsage[images[i].pCamera].push_back(i);
		}
		unsigned nModified = 0, nDuplicated = 0;
		std::unordered_set<CameraPtr> processedCameras;
		for (IIndex idx : imageIndices) {
			Image& img = images[idx];
			if (!img.HasCamera())
				continue;
			// Skip if we already processed this camera (multiple selected images share same camera)
			if (processedCameras.count(img.pCamera))
				continue;
			// Check if camera is PinholeCamera
			PinholeCamera* pinholeCamera = dynamic_cast<PinholeCamera*>(img.pCamera);
			if (!pinholeCamera) {
				VERBOSE("error: image %u has non-pinhole camera (spherical cameras not supported for forced parameters)", idx);
				return false;
			}
			// Check if camera is shared with images NOT in the selection
			const auto& usageList = cameraUsage[img.pCamera];
			bool sharedWithNonSelected = false;
			for (IIndex usedBy : usageList) {
				if (uniqueIndices.find(usedBy) == uniqueIndices.end()) {
					sharedWithNonSelected = true;
					break;
				}
			}
			if (sharedWithNonSelected) {
				// Duplicate camera for selected images only
				PinholeCamera* newCamera = static_cast<PinholeCamera*>(pinholeCamera->Clone());
				if (config.focalLength > 0.f)
					newCamera->fx = newCamera->fy = config.focalLength;
				if (config.k1 != 0.f)
					newCamera->k1 = config.k1;
				if (config.k2 != 0.f)
					newCamera->k2 = config.k2;
				newCamera->trustIntrinsics = true;
				// Add to cameras array and assign to all selected images using this camera
				const IIndex newCamID = cameras.size();
				cameras.emplace_back(newCamera);
				for (IIndex usedBy : usageList) {
					if (uniqueIndices.count(usedBy)) {
						images[usedBy].pCamera = newCamera;
						images[usedBy].cameraID = newCamID;
					}
				}
				processedCameras.insert(newCamera);
				++nDuplicated;
				++nModified;
			} else {
				// Camera only used by selected images - modify directly
				if (config.focalLength > 0.f)
					pinholeCamera->fx = pinholeCamera->fy = config.focalLength;
				if (config.k1 != 0.f)
					pinholeCamera->k1 = config.k1;
				if (config.k2 != 0.f)
					pinholeCamera->k2 = config.k2;
				pinholeCamera->trustIntrinsics = true;
				processedCameras.insert(pinholeCamera);
				++nModified;
			}
		}
		if (config.focalLength > 0.f)
			VERBOSE("Forced focal length %.2f pixels for %u cameras (%u duplicated)",
			        config.focalLength, nModified, nDuplicated);
		if (config.k1 != 0.f || config.k2 != 0.f)
			VERBOSE("Forced distortion k1=%.6f, k2=%.6f for %u cameras (%u duplicated)",
			        config.k1, config.k2, nModified, nDuplicated);
	}

	return true;
}

bool Scene::ExtractFeatures(const FeatureExtractionConfig& config)
{
	if (status.nState.isSet(Status::STATE::FEATURES_EXTRACTED)) {
		VERBOSE("warning: features already extracted");
		return true;
	}
	TD_TIMER_START();

	// Use FeaturesExtractor to extract features from all images
	FeaturesExtractor extractor(*this, config);
	const size_t numFeatures = extractor.Extract();

	status.nState.set(Status::STATE::FEATURES_EXTRACTED);
	status.nFeaturesType = config.detectorType;

	VERBOSE("Features extracted (%s): %u features (%.2f per image) for %u images (%s)",
			FeatureTypeToString(config.detectorType).c_str(),
			numFeatures,(double)numFeatures / images.size(),
			images.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

bool Scene::MatchPairs(const MatchConfig& config, const ROMA2Config& roma2Cfg, const ViewGraphCalibratorConfig& vgConfig)
{
	// In-process ROMA2: one loaded model serves the retrieval descriptors and the dense
	// matching; it lives for this call only (design decision 7). Declared before
	// pairsMatcher: anything holding a RoMa2Onnx* or a MakeLayers() tensor (pairsMatcher.SetROMA2
	// below) must be destroyed before the model, so roma2 must outlive it and therefore be
	// declared first (destructors run in reverse declaration order).
	RoMa2Onnx roma2;
	PairsMatcher pairsMatcher(*this, config);

	const String modelPath(roma2Cfg.ResolveModelPath());
	if (roma2Cfg.enabled && (roma2Cfg.useRetrieval || roma2Cfg.useMatching) && modelPath.empty()) {
		// design decision 10: a requested-but-unavailable model is an error, never a silent
		// fallback to the vocabulary tree (which is what IsInProcessEnabled() would otherwise
		// quietly do, since an empty model path makes it return false)
		VERBOSE("error: ROMA2 requested but no model path given (set --roma2-model or OPENMVS_ROMA2_MODEL_PATH)");
		return false;
	}
	const bool useROMA2 = roma2Cfg.IsInProcessEnabled() && !status.nState.isSet(Status::STATE::MATCHED);
	if (useROMA2) {
		if (!RoMa2Onnx::IsAvailable()) {
			VERBOSE("error: ROMA2 model '%s' requested, but this build has no ONNX Runtime support", modelPath.c_str());
			return false;
		}
		// the ONNX sessions are only loaded when they still have something to produce: the dense
		// warps, or global descriptors this scene does not carry yet. Retrieval alone over
		// descriptors an earlier run already stored ranks the pairs straight from
		// Image::globalDescriptor (PairsMatcher::QueryRetrieval) and never enters a session, so
		// loading 1.2 GB of graph weights onto the device for it would buy nothing
		if (roma2Cfg.useMatching || !status.nState.isSet(Status::STATE::GLOBAL_DESCRIPTORS)) {
			if (!roma2.Load(modelPath, roma2Cfg.setting, roma2Cfg.useGPU ? roma2Cfg.provider : String("cpu"))) {
				VERBOSE("error: failed to load ROMA2 model '%s' (%s)", modelPath.c_str(), roma2Cfg.setting.c_str());
				return false;
			}
			if (roma2Cfg.useRetrieval && !ComputeGlobalDescriptors(roma2, roma2Cfg))
				return false;
		} else {
			DEBUG("ROMA2 retrieval reuses the %u global descriptors stored in the scene; no model loaded", images.size());
		}
	}
	pairsMatcher.SetROMA2(useROMA2 && roma2Cfg.useMatching ? &roma2 : NULL, roma2Cfg);

	if (status.nState.isSet(Status::STATE::MATCHED)) {
		VERBOSE("warning: pairs already matched, skipping");
		pairsMatcher.ComputeRelativePoses();
	} else {
		// Convert lightweight config to typed MatchConfig
		pairsMatcher.Match();
		status.nState.set(Status::STATE::MATCHED);
	}

	if (config.viewGraphCalibrationEnabled) {
		// Run ViewGraph calibration in order to improve focal-length and relative poses
		ViewGraphCalibrator calibrator(vgConfig);
		if (!calibrator.Solve(*this)) {
			DEBUG("warning: ViewGraph calibration failed");
			return false;
		}
		// Recompute relative-pose for all pairs with updated image cameras
		if (!calibrator.GetUpdatedCameras().empty())
			pairsMatcher.ComputeRelativePoses(true, false, calibrator.GetUpdatedCameras());
	}
	return true;
}

bool Scene::ComputeGlobalDescriptors(RoMa2Onnx& roma2, const ROMA2Config& config)
{
	if (status.nState.isSet(Status::STATE::GLOBAL_DESCRIPTORS))
		return true;
	if (images.empty()) {
		// an empty scene would otherwise make "0 described == 0 images" vacuously true below
		VERBOSE("error: no images to describe");
		return false;
	}
	// the retrieval index needs one row per image; Scene::Import already rejected unreadable
	// images, so a miss here (a per-image load/describe failure) is a real failure
	if (ComputeGlobalDescriptorsROMA2(*this, roma2, config) != images.size()) {
		VERBOSE("error: failed to describe all %u images with the ROMA2 model", (unsigned)images.size());
		return false;
	}
	status.nState.set(Status::STATE::GLOBAL_DESCRIPTORS);
	return true;
}

namespace {
// Export the pairs/retrieval-rankings CSV diagnostics of a matched scene
// (ReconstructionConfig::exportPairsCSV / exportRetrievalCSV, when non-empty) right after
// Scene::Reconstruct() finishes pair matching, before any later reconstruction step
// (largest-connected-component clustering, weak-image filtering, resection) can drop pairs or
// leave images unregistered. A failed export only logs a warning: both files are diagnostics and
// must never cost the caller the reconstructed scene itself (ruling R-F1) -- not a member of
// Scene since it has an implicit precondition (must run right after matching) that makes it
// unsuitable as public API, and it has no callers outside Reconstruct().
void ExportMatchingCSVs(const Scene& scene, const ReconstructionConfig& config)
{
	if (!config.exportPairsCSV.empty() &&
		!PairsMatcher::ExportPairsCSV(scene, config.exportPairsCSV, config.minPairWeight))
		VERBOSE("warning: failed to export image pairs to CSV file '%s'", config.exportPairsCSV.c_str());
	if (!config.exportRetrievalCSV.empty() &&
		!ExportRetrievalRankingsCSV(scene, config.exportRetrievalCSV, 50))
		VERBOSE("warning: failed to export retrieval rankings to CSV file '%s'", config.exportRetrievalCSV.c_str());
}

// Export the matching diagnostics of a freshly matched scene, then disambiguate its view graph
// with the camera-triplet filter (ViewGraphTriplets.h, off unless the caller enables it). The
// order is deliberate: the CSVs describe the whole matched graph and carry the triplet score of
// every pair, including the pairs the filter is about to remove, so a run can be re-scored and
// re-thresholded offline from its own export alone.
void ExportMatchingCSVsAndFilterPairs(Scene& scene, const ReconstructionConfig& config)
{
	ExportMatchingCSVs(scene, config);
	FilterPairsByTriplets(scene, config.tripletFilterCfg, config.matchCfg.weightingCfg);
}
} // namespace

bool Scene::Reconstruct(const String& source, const ReconstructionConfig& config)
{
	TD_TIMER_START();
	VERBOSE("Starting reconstruction from '%s'", source.c_str());

	#if 1
	if (!source.empty()) {
		// Start a new reconstruction from the source list or folder of images
		// or load existing scene if source is pointing to a SFM file
		Release(); // clear existing scene if any
		if (!Import(source, config.importCfg))
			return false;
		if (status.nState.isSet(Status::STATE::CALIBRATED)) {
			VERBOSE("warning: scene already calibrated after import");
			return false;
		}
		if (config.matchImagesOnly && status.nState.isSet(Status::STATE::MATCHED)) {
			// the requested work is already done; still resolve an AUTO frames.json
			// convention before the caller re-persists the scene (the loaded pairs are
			// already matched, so the detection can run)
			VERBOSE("warning: scene already matched after import");
			ExportMatchingCSVsAndFilterPairs(*this, config);
			if (config.HasKnownPoses() && !ResolveFramesConvention(*this,
					config.importCfg.framesConvention, config.importCfg.importPosesFile))
				return false;
			return true;
		}
	}

	// ImportCOLMAP(MAKE_PATH("colmap/scene_init.glmp"), *this);
	// ImportCOLMAP(MAKE_PATH("colmap/scene_init.glmp"), *this, false, false);
	// PairsMatcher pairsMatcher(*this, config.matchCfg);
	// pairsMatcher.ComputeRelativePoses(false, false);

	// Extract image features
	if (!ExtractFeatures(config.featuresCfg))
		return false;

	// Match image pairs
	if (!MatchPairs(config.matchCfg, config.roma2Cfg, config.viewgraphCfg))
		return false;

	// export the pairs/retrieval-rankings CSV diagnostics right after matching, before any
	// reconstruction step (clustering, weak-image filtering, resection) can drop pairs or
	// leave images unregistered; covers both the match-images-only run and a full reconstruction,
	// and is immediately followed by the (opt-in) triplet disambiguation of the view graph
	ExportMatchingCSVsAndFilterPairs(*this, config);

	if (config.matchImagesOnly) {
		// a frames.json imported with an AUTO convention must be resolved before the scene
		// is persisted, otherwise possibly-flipped poses are saved with no record of the
		// ambiguity and every later consumer inherits reversed optical axes
		if (config.HasKnownPoses() && !ResolveFramesConvention(*this,
				config.importCfg.framesConvention, config.importCfg.importPosesFile))
			return false;
		VERBOSE("Image pairs matched only as per configuration, reconstruction skipped");
		return true;
	}

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2) {
		// Save intermediate scene after matching for debugging
		Save(MAKE_PATH("scene_pre_reconstruction.sfm"), config.importCfg.archiveType);
	}
	#endif
	#else
	// Shortcut features and matching by directly loading a pre-reconstruction scene (for debugging)
	Load(MAKE_PATH("scene_pre_reconstruction.sfm"));
	#endif

	// Run reconstruction method
	if (config.HasKnownPoses() ? !ReconstructKnownPoses(config)
	    : config.useGlobalSolver ? !ReconstructGlobal(config)
	                             : !ReconstructHierarchical(config))
		return false;

	// Pre-final global bundle adjustment
	BAConfig finalBaCfg = config.baConfig;
	finalBaCfg.maxIterations = 25;
	finalBaCfg.refineFocalLength = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_FOCAL_LENGTH) != 0;
	finalBaCfg.refineRadialDistortion123 = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_RADIAL_DIST_123) != 0;
	BundleAdjustment::Adjust(*this, finalBaCfg);
	FilterTracks(*this, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	TriangulateTracks(*this, true, config.maxReprojError, config.minAngleThreshold);
	FilterTracks(*this, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	status.nState.set(Status::STATE::CALIBRATED);

	// Final global bundle adjustment
	finalBaCfg.maxIterations = config.baConfig.maxIterations;
	SetBAIntrinsicFlags(finalBaCfg, config.baIntrinsicFlags);
	BundleAdjustment::Adjust(*this, finalBaCfg);
	FilterTracks(*this, config.maxFineReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);

	// Filter weakly connected images and resection remaining images into the reconstruction
	FilterWeaklyConnectedImages(*this);
	if (status.nCalibratedImages < images.size()) {
		Resection resection(*this, config.resectionCfg);
		resection.RegisterImages();
		FilterWeaklyConnectedImages(*this);
	}

	// Align the scene back to the imported prior poses in known-poses mode (preserving the
	// input frame is the point of that path, so it takes precedence over GPS, which would
	// yank the scene out of that very frame), else to GPS if available. A failed prior-pose
	// alignment leaves the scene in the refined (arbitrary-gauge) frame: it is reported, but
	// must not discard the finished reconstruction
	if (config.HasKnownPoses() && !priorPoses.empty()) {
		if (!AlignToPriorPoses())
			VERBOSE("warning: could not align the reconstruction back to the imported pose frame; "
				"the result is left in the refined (arbitrary) frame");
	} else if (config.thAlignGPS > 0 && HasImagesWithGPS())
		AlignToGPS(config.thAlignGPS);

	// Refine the geo-aligned reconstruction with GPS position priors (if enabled): the GPS
	// residuals are gated on GEO_ALIGN and their meters-vs-pixels weighting assumes the metric
	// ENU frame, so this is the earliest point in the pipeline where they can take effect
	// (validated for pinhole cameras; spherical scenes use angular residuals the weighting
	// does not account for).
	// Disable intrinsics which already converged in the final bundle adjustment.
	BAConfig uncBaCfg = finalBaCfg;
	uncBaCfg.refineFocalLength = uncBaCfg.refineFocalLengthAspectRatio = uncBaCfg.refinePrincipalPoint =
	uncBaCfg.refineRadialDistortion123 = uncBaCfg.refineTangentialDistortion = uncBaCfg.refineRadialDistortion456 = false;
	if (config.baConfig.IsRefiningGPS() && status.nState.isSet(Status::STATE::GEO_ALIGN)) {
		BundleAdjustment ba(*this, uncBaCfg);
		if (ba.Adjust()) {
			if (config.estimatePoseUncertainty) {
				// the GPS priors anchor the gauge, so this supersedes the earlier record
				// with absolute ENU covariances (and covers the images resected since)
				poseUncertainty = ba.ComputePoseUncertainty();
			}
			FilterTracks(*this, config.maxFineReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
		}
	} else if (config.estimatePoseUncertainty) {
		BundleAdjustment ba(*this, uncBaCfg);
		if (ba.Adjust()) {
			poseUncertainty = ba.ComputePoseUncertainty();
			FilterTracks(*this, config.maxFineReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
		}
	}

	// Estimate color for points
	if (config.extractColors)
		SampleColors();

	VERBOSE("Reconstruction complete: %u images (%u total), %u points (%u total) in %s",
		status.nCalibratedImages, images.size(), status.nTracks, tracks.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

bool Scene::ReconstructHierarchical(const ReconstructionConfig& config)
{
	if (status.nState.isSet(Status::STATE::CALIBRATED)) {
		VERBOSE("warning: scene already calibrated");
		return true;
	}

	TD_TIMER_STARTD();

	// 1. Cluster scene if necessary
	std::vector<Scene> subScenes;
	std::vector<IIndexArr> localToGlobals;
	#if 1
	if (config.clusterCfg.maxViewsPerCluster > 0 && images.size() > config.clusterCfg.maxViewsPerCluster) {
		SceneCluster clusterer(*this, config.clusterCfg);
		subScenes = clusterer.SplitScene(&localToGlobals);
	} else {
		subScenes.emplace_back(std::move(*this));
	}

	// 2. Reconstruct each sub-scene in parallel
	threadPool.detach_loop(IIndex(0), (IIndex)subScenes.size(), [&](IIndex i) {
		Scene& subScene = subScenes[i];
		DEBUG("Reconstructing sub-scene %u with %u images...", i, subScene.images.size());

		// Build tracks
		BuildTracks(subScene, config.minPairWeight);

		// Initialize with star initializer
		if (!StarInitializer::Initialize(subScene, config.initCfg)) {
			VERBOSE("error: star initialization failed for sub-scene %u (skipping)", i);
			return; // skip this sub-scene
		}

		// Incrementally resect images into the reconstruction; every Ceres solve
		// clamps itself to the sub-scene's thread budget (see BundleAdjustment)
		Resection resection(subScene, config.resectionCfg);
		resection.RegisterImages();

		// Local / global bundle adjustment for this sub-scene
		BAConfig baCfg = config.baConfig;
		SetBAIntrinsicFlags(baCfg, config.baIntrinsicFlags);
		BundleAdjustment::Adjust(subScene, baCfg);
		FilterTracks(subScene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	});
	threadPool.wait();
	#if 0
	SerializeSave(*this, MAKE_PATH("scene_pre_hierarchical_scene.sfm"), config.importCfg.archiveType);
	SerializeSave(subScenes, MAKE_PATH("scene_pre_hierarchical_reconstruction.sfm"), config.importCfg.archiveType);
	SerializeSave(localToGlobals, MAKE_PATH("scene_pre_hierarchical_local_to_globals.sfm"), config.importCfg.archiveType);
	#endif
	#else
	SerializeLoad(*this, MAKE_PATH("scene_pre_hierarchical_scene.sfm"), config.importCfg.archiveType);
	SerializeLoad(subScenes, MAKE_PATH("scene_pre_hierarchical_reconstruction.sfm"), config.importCfg.archiveType);
	SerializeLoad(localToGlobals, MAKE_PATH("scene_pre_hierarchical_local_to_globals.sfm"), config.importCfg.archiveType);
	#endif

	// 3. Merge/align sub-scenes (simple merge + final BA)
	if (subScenes.size() == 1) {
		*this = std::move(subScenes[0]);
	} else {
		// merge sub-scenes, or, if not possible, keep only the largest sub-scene
		GlobalAlignment globalAlign(*this, config.globalAlignmentCfg);
		globalAlign.MergeScenes(subScenes, localToGlobals);
	}
	DEBUG("Hierarchical reconstruction complete: %u/%u images, %u/%u points (%s)",
		status.nCalibratedImages, images.size(), status.nTracks, tracks.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

bool Scene::ReconstructGlobal(const ReconstructionConfig& config)
{
	if (status.nState.isSet(Status::STATE::CALIBRATED)) {
		VERBOSE("warning: scene already calibrated");
		return true;
	}

	TD_TIMER_STARTD();

	// 1. Global Rotation Averaging
	GlobalRotationEstimatorOptions rotOptions;
	GlobalRotationEstimator rotEstimator(rotOptions);
	// Run rotation averaging twice for better convergence:
	// first pass to get initial rotations and remove problematic pair, second pass to refine
	unsigned numFilteredPairs;
	if (!rotEstimator.EstimateRotations(*this, &numFilteredPairs)) {
		VERBOSE("error: global rotation averaging 1 failed");
		return false;
	}
	if (numFilteredPairs != 0 && !rotEstimator.EstimateRotations(*this)) {
		VERBOSE("error: global rotation averaging 2 failed");
		return false;
	}

	// CompareScenes(*this, MAKE_PATH("rscene_init.mvs"));
	// ImportCOLMAP(MAKE_PATH("colmap/scene_rotations.glmp"), *this);
	// CompareScenes(*this, MAKE_PATH("rscene_init.mvs"));

	// Build tracks
	BuildTracks(*this, config.minPairWeight);

	// 2. Global Positioning
	GlobalPositionerOptions posOptions;
	GlobalPositioner posEstimator(posOptions);
	if (!posEstimator.Solve(*this)) {
		VERBOSE("error: global positioning failed");
		return false;
	}

	// 3. Update Status
	FilterTracks(*this, 6.f, 1.f);
	RecomputeCalibratedImages();
	status.nState.set(Status::STATE::CALIBRATED);

	// 4. Bundle Adjustment for position and structure refinement only
	BAConfig baCfg;
	baCfg.refinePosesRotation = false;
	baCfg.maxIterations = 12;
	BundleAdjustment::Adjust(*this, baCfg);
	FilterTracks(*this, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);

	// Bundle Adjustment with full pose and structure refinement
	baCfg.refinePosesRotation = true;
	baCfg.maxIterations = 25;
	BundleAdjustment::Adjust(*this, baCfg);
	FilterTracks(*this, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	TriangulateTracks(*this, true, config.maxReprojError, config.minAngleThreshold);

	DEBUG("Global reconstruction complete: %u/%u images, %u/%u points (%s)",
		status.nCalibratedImages, images.size(), status.nTracks, tracks.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

bool Scene::ReconstructKnownPoses(const ReconstructionConfig& config)
{
	if (status.nState.isSet(Status::STATE::CALIBRATED)) {
		VERBOSE("warning: scene already calibrated");
		return true;
	}

	TD_TIMER_STARTD();

	// 1. Validate the pose import actually covered the dataset: a file-name mismatch in the
	// user's poses file would otherwise silently degrade into a from-scratch reconstruction
	// of the few matched images, so fail loudly instead of falling back to standard SfM.
	// This is a sanity gate against such a mismatch (which poses ~0 images), not a coverage
	// requirement: partially covered captures are legitimate (the pose-guided pair selection
	// adds visual pairs for the unposed images and the reconstruction tail resects them)
	constexpr float minPosedImagesRatio = 0.2f; // sanity fraction of images that must be posed
	constexpr IIndex maxUnposedNamesLogged = 10; // cap the unmatched list, then summarize
	IIndexArr unposedImages;
	unsigned numPosedImages = 0;
	FOREACH(i, images) {
		if (images[i].HasPose())
			++numPosedImages;
		else
			unposedImages.push_back(i);
	}
	const unsigned minPosedImages = MAXF(2u, (unsigned)CEIL2INT(minPosedImagesRatio*images.size()));
	if (numPosedImages < minPosedImages) {
		String unmatched;
		FOREACH(k, unposedImages) {
			if (k >= maxUnposedNamesLogged) {
				unmatched += String::FormatString(", ... (%u more)", unposedImages.size()-k);
				break;
			}
			unmatched += String::FormatString("%s%s", k == 0 ? " " : ", ",
				Util::getFileNameExt(images[unposedImages[k]].fileName).c_str());
		}
		VERBOSE("error: known-poses reconstruction needs a pose for at least %u of the %u images, "
			"but only %u were matched by name in '%s'; unmatched:%s",
			minPosedImages, images.size(), numPosedImages,
			config.importCfg.importPosesFile.c_str(), unmatched.c_str());
		return false;
	}

	// 2. Resolve the camera-axes convention of the imported transforms: a frames.json does not
	// declare it and the import optimistically applied the ARKit one; the two hypotheses
	// differ by a pi rotation about the camera X axis, so the wrong choice reverses every
	// viewing direction and triangulation collapses
	if (!ResolveFramesConvention(*this, config.importCfg.framesConvention, config.importCfg.importPosesFile))
		return false;

	// 3. Remember the imported poses: the bundle adjustment below refines them freely, and
	// AlignToPriorPoses() uses this snapshot to bring the result back to the input frame
	priorPoses.clear();
	priorPoses.reserve(images.size());
	for (const Image& img: images)
		if (img.HasPose())
			priorPoses.emplace(img.ID, Pose3D(img.R, img.C));

	// 4. Build tracks and triangulate them with the imported poses; the poses are only
	// approximate (and the intrinsics may still come from EXIF), so triangulate with a
	// permissive reprojection threshold - the accurate-pose threshold would reject most of
	// the correct tracks before the bundle adjustment ever gets a chance to fix the geometry
	BuildTracks(*this, config.minPairWeight);
	if (tracks.empty()) {
		VERBOSE("error: no tracks could be built from the matched pairs");
		return false;
	}
	const float initReprojError = config.maxReprojError*4.f;
	TriangulateTracks(*this, false, initReprojError, config.minAngleThreshold);
	const std::pair<float, float> initError = FilterTracks(*this, initReprojError,
		config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	if (status.nTracks == 0) {
		VERBOSE("error: no track survived triangulation with the imported poses "
			"(wrong camera-axes convention or wrong image-to-pose association?)");
		return false;
	}
	DEBUG("Triangulated %u/%u tracks with the imported poses (%.2f pixels, %.3f degrees)",
		status.nTracks, tracks.size(), initError.first, initError.second);

	// 5. The imported poses are the reconstruction: mark the images calibrated so that the
	// bundle adjustment and the tail's resection treat them as posed (the counter is otherwise
	// delta-maintained by the incremental solvers, which never ran here)
	RecomputeCalibratedImages();
	status.nState.set(Status::STATE::CALIBRATED);

	// 6. Finetune bundle adjustment: the poses are already close, so this mostly absorbs the
	// intrinsics error; re-triangulate the outlier tracks with the tightened geometry and run a
	// second pass - the same mini-BA -> re-triangulate -> BA convergence pattern the star
	// initializer uses
	BAConfig baCfg = config.baConfig;
	SetBAIntrinsicFlags(baCfg, config.baIntrinsicFlags);
	bool trustedIntrinsics = true;
	for (const Camera* cam: cameras) {
		if (!cam->TrustIntrinsics()) {
			trustedIntrinsics = false;
			break;
		}
	}
	if (!trustedIntrinsics) {
		// a focal length guessed from the default focal ratio (no usable metadata) is by far
		// the weakest prior of a poses-only import, and the known poses make the bundle
		// adjustment over it well conditioned, so refine it even when the caller asked for
		// no intrinsic refinement
		DEBUG("Cameras with untrusted intrinsics present: enabling main-intrinsics refinement");
		baCfg.RefineMainIntrinsics();
	}
	baCfg.maxIterations = 25;
	if (!BundleAdjustment::Adjust(*this, baCfg)) {
		VERBOSE("error: known-poses bundle adjustment failed");
		return false;
	}
	TriangulateTracks(*this, true, config.maxReprojError, config.minAngleThreshold);
	FilterTracks(*this, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	baCfg.maxIterations = config.baConfig.maxIterations;
	if (!BundleAdjustment::Adjust(*this, baCfg)) {
		VERBOSE("error: known-poses bundle adjustment failed");
		return false;
	}
	const std::pair<float, float> finalError = FilterTracks(*this, config.maxReprojError,
		config.minAngleThreshold, config.multDepthNear, config.multDepthFar);

	RecomputeCalibratedImages();
	DEBUG("Known-poses reconstruction complete: %u/%u images (%u posed by import), %u/%u points, "
		"%.2f pixels (%s)",
		status.nCalibratedImages, images.size(), numPosedImages,
		status.nTracks, tracks.size(), finalError.first, TD_TIMER_GET_FMT().c_str());
	return true;
}

bool Scene::SampleColors()
{
	TD_TIMER_STARTD();

	// Select for each track the observation with the smallest reprojection error and
	// group the selection by image; this needs the geometry only, so that the pixels
	// can be sampled below one image at a time
	typedef std::pair<uint32_t,uint32_t> TrackFeature; // track and the feature seeing it
	std::vector<std::vector<TrackFeature>> imageSamples(images.size());
	colors.resize(tracks.size());
	FOREACH(trackID, tracks) {
		// outliers and tracks not projecting in any of their views remain black
		colors[trackID] = Pixel8U::BLACK;
		const Track& track = tracks[trackID];
		if (!track.IsInlier())
			continue;
		float minError = FLT_MAX;
		uint32_t bestObsIdx;
		for (uint32_t obsIdx = 0; obsIdx < track.GetNumInliers(); ++obsIdx) {
			const Observation& obs = track.observations[obsIdx];
			const Image& img = images[obs.imageID];
			ASSERT(img.IsValid());
			ASSERT(obs.featureID < img.keypoints.size());
			// Compute pixel reprojection error
			const cv::KeyPoint& kp = img.keypoints[obs.featureID];
			const auto [projected, valid] = img.ProjectPoint(track.position);
			if (!valid)
				continue;
			const float pixelError = norm(Cast<float>(projected) - kp.pt);
			if (pixelError < minError) {
				minError = pixelError;
				bestObsIdx = obsIdx;
			}
		}
		if (minError >= FLT_MAX)
			continue;
		const Observation& bestObs = track.observations[bestObsIdx];
		imageSamples[bestObs.imageID].emplace_back(trackID, bestObs.featureID);
	}

	// Sample the colors one image at a time, releasing right away the pixels loaded
	// here: the images of a large scene do not fit together in memory
	Sampler::Linear<float> sampler;
	FOREACH(imageID, images) {
		const std::vector<TrackFeature>& samples = imageSamples[imageID];
		if (samples.empty())
			continue;
		Image& img = images[imageID];
		const bool wasLoaded = img.HasPixels();
		if (!wasLoaded)
			img.LoadPixels();
		const int numChannels(img.HasPixels() ? img.pixels.channels() : 0);
		if (numChannels != 1 && numChannels != 3) {
			if (!wasLoaded)
				img.ReleasePixels();
			colors.Release();
			return false;
		}
		// Sample color from image at keypoint location using bilinear interpolation
		for (const TrackFeature& sample: samples) {
			const cv::KeyPoint& kp = img.keypoints[sample.second];
			if (numChannels == 1)
				colors[sample.first].set((uint8_t)CLAMP(ROUND2INT(Sampler::Sample<uint8_t,float>(img.pixels, sampler, kp.pt)), 0, 255));
			else
				colors[sample.first] = Sampler::Sample<Pixel8U,Pixel32F>(img.pixels, sampler, kp.pt).cast<uint8_t>();
		}
		if (!wasLoaded)
			img.ReleasePixels();
	}
	DEBUG_EXTRA("Colors sampled for %u tracks (%s)",
		tracks.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

bool Scene::AlignToGPS(double threshold)
{
	// 1. Collect valid GPS positions and convert to ECEF
	Point3Arr camCenters;
	Point3dArr ecefPositions;
	Point3d centerECEF(0, 0, 0);
	for (const Image& img : images) {
		if (!img.IsValid())
			continue;
		// Check if GPS data is valid (simple check: not all zero)
		const View::Metadata& viewMeta = img.View::metadata;
		if (!viewMeta.HasGPS())
			continue;
		camCenters.push_back(img.C);
		Point3d ecef;
		WGS84ToECEF(viewMeta.latitude, viewMeta.longitude, viewMeta.altitude, ecef.x, ecef.y, ecef.z);
		ecefPositions.push_back(ecef);
		centerECEF += ecef;
	}
	if (camCenters.size() < 3) {
		VERBOSE("error: insufficient GPS data (found %u, need 3+)", (unsigned)camCenters.size());
		return false;
	}

	// 2. Compute centroid
	centerECEF /= (double)ecefPositions.size();

	// 3. Convert ECEF to ENU (centered at centroid)
	// We need the LLA of the centroid for the ENU frame
	double lat0, lon0, alt0;
	ECEFToWGS84(centerECEF.x, centerECEF.y, centerECEF.z, lat0, lon0, alt0);
	Point3Arr enuPositions;
	enuPositions.reserve(ecefPositions.size());
	for (const auto& ecef : ecefPositions) {
		double e, n, u;
		ECEFToENU(ecef.x, ecef.y, ecef.z, centerECEF.x, centerECEF.y, centerECEF.z, lat0, lon0, e, n, u);
		enuPositions.emplace_back((REAL)e, (REAL)n, (REAL)u);
	}

	// 4. Verify the GPS positions are well spread: the similarity transform needs
	// at least 3 distinct, non-collinear positions spread wider than the GPS noise
	// (consumer devices often tag consecutive images with the same stale GPS fix)
	if (threshold > 0) {
		Point3 mean(Point3::ZERO);
		for (const Point3& enu : enuPositions)
			mean += enu;
		mean /= (double)enuPositions.size();
		Eigen::Matrix3d cov(Eigen::Matrix3d::Zero());
		for (const Point3& enu : enuPositions) {
			const Eigen::Vector3d d(Point3d(enu - mean));
			cov += d * d.transpose();
		}
		cov /= (double)enuPositions.size();
		// standard deviation along each principal axis, in increasing order
		const Eigen::Vector3d spread(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(cov, Eigen::EigenvaluesOnly).eigenvalues().cwiseMax(0.).cwiseSqrt());
		if (spread(1) < threshold) {
			VERBOSE("error: GPS positions nearly coincident or collinear (spread %.1fx%.1fx%.1fm, need %.1fm+), skipping GPS alignment",
				spread(2), spread(1), spread(0), threshold);
			return false;
		}
	}

	// 5. Estimate similarity transform: Camera -> ENU
	SEACAVE::Transform T_cam_to_enu;
	if (EstimateSimilarityTransform(camCenters, enuPositions, T_cam_to_enu, threshold) == 0) {
		VERBOSE("error: failed to estimate transform");
		return false;
	}

	// 6. Transform the scene to ENU
	Transform(T_cam_to_enu);

	// 7. Set Scene::transform to the transform that brings ENU (centered) back to Absolute (ECEF)
	transform = Matrix4x4::IDENTITY;
	transform(0, 3) = centerECEF.x;
	transform(1, 3) = centerECEF.y;
	transform(2, 3) = centerECEF.z;

	status.nState.set(Status::STATE::GEO_ALIGN);
	VERBOSE("Scene aligned to GPS: aligned %u images (scale=%.4f)",
		(unsigned)camCenters.size(), T_cam_to_enu.scale);
	return true;
}

REAL SFM::MedianNearestCameraDistance(BS::light_thread_pool& threadPool, const Point3Arr& centers)
{
	if (centers.size() < 2)
		return REAL(0);
	REALArr nearestDistances(centers.size());
	threadPool.detach_loop(IIndex(0), (IIndex)centers.size(), [&](IIndex i) {
		REAL minDistSq = std::numeric_limits<REAL>::max();
		FOREACH(j, centers)
			if (i != j)
				minDistSq = MINF(minDistSq, normSq(centers[i]-centers[j]));
		nearestDistances[i] = SQRT(minDistSq);
	});
	threadPool.wait();
	return nearestDistances.GetMedian();
}

bool Scene::AlignToPriorPoses(float thresholdRatio)
{
	// 1. Collect the refined/prior camera-center correspondences; images resected along the
	// way have no prior and simply ride along with the transform applied below
	Point3Arr refinedCenters, priorCenters;
	IIndexArr alignedImages;
	refinedCenters.reserve((IIndex)priorPoses.size());
	priorCenters.reserve((IIndex)priorPoses.size());
	alignedImages.reserve((IIndex)priorPoses.size());
	FOREACH(i, images) {
		const Image& img = images[i];
		if (!img.IsValid())
			continue;
		const auto it = priorPoses.find(img.ID);
		if (it == priorPoses.end())
			continue;
		alignedImages.push_back(i);
		refinedCenters.push_back(img.C);
		priorCenters.push_back(it->second.C);
	}
	if (refinedCenters.size() < 3) {
		VERBOSE("error: insufficient prior poses left after filtering (found %u, need 3+), skipping prior-pose alignment",
			(unsigned)refinedCenters.size());
		return false;
	}

	// 2. Derive the RANSAC threshold from the capture itself: the prior frame is metric for an
	// AR capture but arbitrary for a normalized one, so the only meaningful scale reference is
	// the spacing of the prior cameras; half of it keeps a slowly drifting trajectory (exactly
	// what the finetune corrects) fully inlier while rejecting a camera that ended up a whole
	// frame-spacing away from its prior
	const double medianDist = (double)MedianNearestCameraDistance(threadPool, priorCenters);
	if (medianDist <= 0) {
		VERBOSE("error: prior camera centers are coincident, skipping prior-pose alignment");
		return false;
	}
	const double threshold = (double)thresholdRatio*medianDist;

	// 3. Estimate the transform bringing the refined scene back to the prior frame; a
	// (nearly) collinear capture (corridor walk, straight flight line) leaves the roll about
	// the trajectory unconstrained by the centers alone, so the estimation falls back to the
	// camera rotations there (see EstimateSimilarityTransformWithRotations)
	Matrix3x3Arr refinedRots(alignedImages.size()), priorRots(alignedImages.size());
	FOREACH(k, alignedImages) {
		const Image& img = images[alignedImages[k]];
		refinedRots[k] = img.R;
		priorRots[k] = priorPoses.at(img.ID).R;
	}
	SEACAVE::Transform T_refined_to_prior;
	if (EstimateSimilarityTransformWithRotations(refinedCenters, priorCenters,
			refinedRots, priorRots, T_refined_to_prior, threshold) == 0) {
		VERBOSE("error: failed to estimate the transform to the prior pose frame");
		return false;
	}
	// apply; unlike AlignToGPS this leaves Scene::transform and GEO_ALIGN alone, as the prior
	// frame is the dataset's own frame, not a geo-referenced one
	Transform(T_refined_to_prior);

	// 5. Report how far the finetune moved the poses, now that both are in the same frame
	DoubleArr posErr, rotErrDeg;
	posErr.reserve(alignedImages.size());
	rotErrDeg.reserve(alignedImages.size());
	double maxPosErr = 0, maxRotErrDeg = 0;
	for (const IIndex i: alignedImages) {
		const Image& img = images[i];
		const Pose3D& prior = priorPoses.at(img.ID);
		const double posDelta = norm(img.C-prior.C);
		const double rotDelta = R2D(ACOS(ComputeAngle(img.R, prior.R)));
		maxPosErr = MAXF(maxPosErr, posDelta);
		maxRotErrDeg = MAXF(maxRotErrDeg, rotDelta);
		posErr.push_back(posDelta);
		rotErrDeg.push_back(rotDelta);
	}
	VERBOSE("Scene aligned to the imported prior poses: %u images (scale=%.4f) | "
		"center delta median %g max %g (prior units) | rotation delta median %.3f max %.3f degrees",
		alignedImages.size(), T_refined_to_prior.scale,
		posErr.GetMedian(), maxPosErr, rotErrDeg.GetMedian(), maxRotErrDeg);
	return true;
}

void Scene::Transform(const struct Transform& T)
{
	// Apply to all cameras
	// When transforming world by T, poses transform as:
	// - New camera center: C_new = T * C = scale * (R * C) + t
	// - New rotation: R_new = R * T.R^T (applied on the right)
	// This ensures: X_cam = R_new * (X_new - C_new) = R * T.R^T * T * (X - C) = T.scale * R * (X - C)
	// (scale cancels out in the camera projection math)
	for (Image& img : images) {
		if (img.IsValid()) {
			img.R = img.R * T.R.t();
			img.C = T * img.C;
		}
	}

	// Apply to all points
	for (Track& track : tracks)
		track.position = T * track.position;

	// Keep the recorded pose uncertainty consistent with the new world frame: the position
	// covariance maps as scale^2 * R * Cov * R^T (rotation uncertainty is about the camera
	// axes and is unaffected by a world transform)
	if (!poseUncertainty.empty()) {
		const REAL s2(SQUARE(T.scale));
		for (PoseUncertainty& u : poseUncertainty) {
			if (!u.IsValid())
				continue;
			const Matrix3x3 cov(
				u.posVar.x, u.posCov.x, u.posCov.y,
				u.posCov.x, u.posVar.y, u.posCov.z,
				u.posCov.y, u.posCov.z, u.posVar.z);
			const Matrix3x3 covT(T.R * cov * T.R.t() * s2);
			u.posVar = Point3f((float)covT(0,0), (float)covT(1,1), (float)covT(2,2));
			u.posCov = Point3f((float)covT(0,1), (float)covT(0,2), (float)covT(1,2));
		}
	}
}

bool Scene::UndistortImages(String outputDir, String extension, float alpha,
	CLISTDEF2(String)* outImagePaths,
	std::unordered_map<const Camera*, KMatrix>* undistortedIntrinsics) const
{
	if (outputDir.empty())
		return true;
	if (extension.empty())
		extension = ".jxl";

	struct UndistortData {
		cv::Mat map1;
		cv::Mat map2;
		KMatrix newK;
	};
	std::unordered_map<const Camera*, UndistortData> undistortMaps;
	undistortMaps.reserve(cameras.size());
	for (CameraPtr const camPtr : cameras) {
		if (!camPtr->IsValid() || !camPtr->HasDistortion())
			continue;
		const cv::Size imgSize(camPtr->GetSize());
		switch (camPtr->GetType()) {
		case CameraType::PINHOLE: {
			const PinholeCamera* pc = static_cast<const PinholeCamera*>(camPtr);
			const cv::Mat distCoeffs = pc->GetDistortionCoeffs();
			UndistortData data;
			data.newK = cv::getOptimalNewCameraMatrix(pc->GetK(), distCoeffs, imgSize, alpha);
			cv::initUndistortRectifyMap(pc->GetK(), distCoeffs, cv::noArray(), data.newK, imgSize, CV_16SC2, data.map1, data.map2);
			if (undistortedIntrinsics)
				undistortedIntrinsics->emplace(pc, data.newK);
			undistortMaps.emplace(pc, std::move(data));
			break;
		}
		default:
			// unsupported camera type
			ASSERT("unsupported camera type for undistortion" == NULL);
		}
	}
	if (undistortMaps.empty())
		return true; // no camera has distortion: skip undistortion and do not create the output folder

	// There is at least one distorted camera to correct, so create the output folder
	Util::ensureValidFolderPath(outputDir);
	Util::ensureFolder(outputDir);

	if (outImagePaths)
		outImagePaths->assign(images.size(), String());
	#ifdef SCENE_USE_OPENMP
	#pragma omp parallel for schedule(dynamic)
	#endif
	for (int_t _i = 0; _i < (int_t)images.size(); ++_i) {
		const IIndex i = static_cast<IIndex>(_i);
		const Image& img = images[i];
		if (!img.IsValid())
			continue;
		bool loadedHere = false;
		if (!img.HasPixels()) {
			const_cast<Image&>(img).LoadPixels();
			loadedHere = true;
		}
		if (!img.HasPixels())
			continue;
		cv::Mat undistorted;
		const UndistortData& data = undistortMaps.at(img.pCamera);
		// Undistort image
		cv::remap(img.pixels, undistorted, data.map1, data.map2, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar::all(0));
		if (loadedHere)
			const_cast<Image&>(img).ReleasePixels();
		// Restore original orientation if the working copy was rotated to landscape
		undistorted = img.ToOriginalOrientation(undistorted);
		const String stem = Util::getFileName(img.fileName);
		const String outPath = outputDir + stem + extension;
		if (!SaveImage(undistorted, outPath)) {
			VERBOSE("error: saving undistorted image '%s' to '%s' failed", img.fileName.c_str(), outPath.c_str());
			continue;
		}
		if (outImagePaths)
			(*outImagePaths)[i] = outPath;
	}
	return true;
}

// Precompute neighbor views based on shared track observations
void Scene::PrecomputeTrackBasedNeighbors(std::vector<ViewScoreArr>& neighbors) const
{
	neighbors.clear();
	neighbors.resize(images.size());
	if (images.empty() || tracks.empty())
		return;

	TD_TIMER_STARTD();

	// Helper struct to accumulate neighbor statistics
	struct TrackNeighborStats {
		float angleSum = 0.f;       // sum of angles between viewing rays
		uint32_t angleCount = 0;    // count of valid angle computations
		CLISTDEF0(uint32_t) sharedTrackIDs; // IDs of shared tracks
	};

	// Process each reference image
	FOREACH(refID, images) {
		const Image& refImage = images[refID];
		if (!refImage.IsValid())
			continue;

		// Statistics per potential neighbor image
		std::vector<TrackNeighborStats> stats(images.size());

		// Iterate over all tracks and find shared observations
		FOREACH(trackID, tracks) {
			const Track& track = tracks[trackID];
			if (!track.IsInlier())
				continue;

			// Check if reference image observes this track
			bool refObserves = false;
			for (const Observation& obs : track) {
				if (obs.imageID == refID) {
					refObserves = true;
					break;
				}
			}
			if (!refObserves)
				continue;

			// Compute depth and viewing direction for reference image
			const auto [trackInRefImg, validRef] = refImage.ProjectPoint(track.position);
			if (!validRef)
				continue;
			Point3 V1 = refImage.C - track.position;

			// Count shared observations and compute angles
			for (const Observation& obs : track) {
				if (obs.imageID == refID)
					continue;
				const Image& otherImage = images[obs.imageID];
				ASSERT(otherImage.IsValid());
				const auto [trackInOtherImg, validOther] = otherImage.ProjectPoint(track.position);
				if (!validOther)
					continue;
				TrackNeighborStats& stat = stats[obs.imageID];
				stat.sharedTrackIDs.emplace_back(trackID);
				const Point3 V2 = otherImage.C - track.position;
				const float cosAngle = static_cast<float>(CLAMP(V1.dot(V2) / (norm(V1) * norm(V2)), -1.0, 1.0));
				stat.angleSum += ACOS(cosAngle);
				++stat.angleCount;
			}
		}

		// Build ViewScoreArr for this reference image
		ViewScoreArr& refNeighbors = neighbors[refID];
		CLISTDEF0(Point2f) projs(0, 256);

		// Get reference image size for area computation
		const Point2f boundsA(refImage.GetSize());
		FOREACH(viewID, images) {
			const TrackNeighborStats& stat = stats[viewID];
			if (stat.sharedTrackIDs.empty())
				continue;

			const Image& otherImage = images[viewID];
			ASSERT(otherImage.IsValid());

			// Compute overlap area by projecting shared tracks
			const Point2f boundsB(otherImage.GetSize());
			projs.Empty();
			for (const uint32_t trackID : stat.sharedTrackIDs) {
				const Track& track = tracks[trackID];
				const auto [ptB, validB] = otherImage.ProjectPoint(track.position);
				if (!validB || ptB.x < 0 || ptB.x >= boundsB.x || ptB.y < 0 || ptB.y >= boundsB.y)
					continue;
				const auto [ptA, validA] = refImage.ProjectPoint(track.position);
				if (!validA || ptA.x < 0 || ptA.x >= boundsA.x || ptA.y < 0 || ptA.y >= boundsA.y)
					continue;
				projs.emplace_back(Cast<float>(ptA));
			}

			// Add neighbor entry
			ViewScore& neighbor = refNeighbors.AddEmpty();
			neighbor.ID = static_cast<uint32_t>(viewID);
			neighbor.points = stat.sharedTrackIDs.size();
			neighbor.angle = stat.angleCount > 0 ? stat.angleSum / stat.angleCount : 0.f;
			neighbor.area = projs.empty() ? 0.f : ComputeCoveredArea<float,2,16,false>(reinterpret_cast<const float*>(projs.data()), projs.size(), &boundsA.x);
		}

		// Sort neighbors by number of shared tracks (descending)
		refNeighbors.Sort([](const ViewScore& a, const ViewScore& b) {
			return a.points > b.points;
		});
	}

	DEBUG_EXTRA("Track-based neighbors precomputed: %u images (%s)", images.size(), TD_TIMER_GET_FMT().c_str());
}
/*----------------------------------------------------------------*/


// Export tracks and optionally image positions to PLY format
bool Scene::ExportPLY(const String& fileName, bool bExportImages, bool bInliersOnly, bool bBinary) const
{
	// Count tracks to export
	uint32_t numTracks = 0;
	if (bInliersOnly) {
		for (const Track& track : tracks)
			if (track.IsInlier())
				numTracks++;
	} else {
		numTracks = (uint32_t)tracks.size();
	}
	if (numTracks == 0) {
		DEBUG("warning: no tracks to export");
		return false;
	}

	// Count calibrated images to export
	uint32_t numImages = 0;
	if (bExportImages) {
		for (const Image& image : images)
			if (image.HasPose())
				numImages++;
	}

	const uint32_t numVertices = numTracks + numImages;

	// Define vertex structure for PLY export
	struct Vertex {
		Point3f p; // 3D position
		Pixel8U c; // color
	};

	// Define PLY properties
	static const PLY::PlyProperty props[] = {
		{"x",     PLY::Float32, PLY::Float32, offsetof(Vertex, p.x), 0, 0, 0, 0},
		{"y",     PLY::Float32, PLY::Float32, offsetof(Vertex, p.y), 0, 0, 0, 0},
		{"z",     PLY::Float32, PLY::Float32, offsetof(Vertex, p.z), 0, 0, 0, 0},
		{"red",   PLY::Uint8,   PLY::Uint8,   offsetof(Vertex, c.r), 0, 0, 0, 0},
		{"green", PLY::Uint8,   PLY::Uint8,   offsetof(Vertex, c.g), 0, 0, 0, 0},
		{"blue",  PLY::Uint8,   PLY::Uint8,   offsetof(Vertex, c.b), 0, 0, 0, 0}
	};

	// Element names
	static const char* elem_names[] = {
		"vertex"
	};

	// Create PLY file
	PLY ply;
	if (!ply.write(fileName, 1, elem_names, bBinary ? PLY::BINARY_LE : PLY::ASCII))
		return false;

	// Describe properties
	ply.describe_property("vertex", 6, props);
	ply.element_count("vertex", numVertices);

	// Write header
	if (!ply.header_complete())
		return false;

	// Export tracks
	Vertex vertex;
	FOREACH(trackID, tracks) {
		const Track& track = tracks[trackID];

		// Skip outliers if requested
		if (bInliersOnly && !track.IsInlier())
			continue;

		// Set position
		vertex.p = Cast<float>(track.position);

		// Set color (use color from colors array if available, otherwise white)
		vertex.c = !colors.empty() ? colors[trackID] : Pixel8U::WHITE;

		ply.put_element(&vertex);
	}

	// Export image positions
	if (bExportImages) {
		for (const Image& image : images) {
			if (!image.HasPose())
				continue;

			// Set position to camera center
			vertex.p = Cast<float>(image.C);

			// Use a distinct color for cameras (yellow)
			vertex.c = Pixel8U(255, 255, 0);

			ply.put_element(&vertex);
		}
	}

	VERBOSE("Exported %u tracks%s%s to '%s'",
		numTracks,
		bExportImages ? String::FormatString(" and %u image positions", numImages).c_str() : "",
		bInliersOnly ? " (inliers only)" : "",
		fileName.c_str());
	return true;
}
/*----------------------------------------------------------------*/


bool SFM::CompareScenes(const Scene& scene, const String& gtFile, bool matchByName)
{
	Scene gtScene;
	if (!ImportMVS(gtFile, gtScene)) {
		VERBOSE("error: failed to load GT scene '%s'", gtFile.c_str());
		return false;
	}

	// Check if scene is calibrated (full pose estimation complete)
	const bool isCalibrated = scene.status.nState.isSet(Scene::Status::STATE::CALIBRATED);

	// Build lookup of GT images either by ID or by filename (stem)
	std::unordered_map<IIndex, IIndex> gtById;
	std::unordered_map<String, IIndex> gtByName;
	gtById.reserve(gtScene.images.size());
	gtByName.reserve(gtScene.images.size());
	FOREACH(i, gtScene.images) {
		const Image& img = gtScene.images[i];
		if (!img.HasPose())
			continue;
		if (matchByName) {
			const std::string key(Util::getFileName(img.fileName).c_str());
			gtByName.emplace(key, i);
		} else {
			gtById.emplace(img.ID, i);
		}
	}

	Point3Arr srcCenters, dstCenters;
	Matrix3x3Arr srcRots, dstRots;
	IIndexArr sceneIdx, gtIdx;
	srcCenters.reserve(scene.images.size());
	dstCenters.reserve(scene.images.size());
	srcRots.reserve(scene.images.size());
	dstRots.reserve(scene.images.size());
	sceneIdx.reserve(scene.images.size());
	gtIdx.reserve(scene.images.size());
	FOREACH(i, scene.images) {
		const Image& img = scene.images[i];
		if (!isCalibrated && !img.HasCamera())
			continue;
		if (isCalibrated && !img.HasPose())
			continue;
		if (matchByName) {
			const String key(Util::getFileName(img.fileName));
			auto it = gtByName.find(key);
			if (it == gtByName.end())
				continue;
			sceneIdx.push_back(i);
			gtIdx.push_back(it->second);
			srcCenters.push_back(img.C);
			dstCenters.push_back(gtScene.images[it->second].C);
			srcRots.push_back(img.R);
			dstRots.push_back(gtScene.images[it->second].R);
		} else {
			auto it = gtById.find(img.ID);
			if (it == gtById.end())
				continue;
			sceneIdx.push_back(i);
			gtIdx.push_back(it->second);
			srcCenters.push_back(img.C);
			dstCenters.push_back(gtScene.images[it->second].C);
			srcRots.push_back(img.R);
			dstRots.push_back(gtScene.images[it->second].R);
		}
	}
	if (sceneIdx.size() < 3) {
		VERBOSE("error: insufficient common posed images (%zu) using %s matching to compare scenes",
			sceneIdx.size(), matchByName ? "name" : "ID");
		return false;
	}

	// Compare rotations and positions (if available)
	constexpr double rotErrorThresholdDeg = 10.0;
	constexpr double rotErrorLargeThresholdDeg = 30.0;
	unsigned numLargeRotErrors = 0, numVeryLargeRotErrors = 0;
	DoubleArr rotErrDeg, posErr;
	rotErrDeg.reserve(sceneIdx.size());
	if (isCalibrated)
		posErr.reserve(sceneIdx.size());
	if (isCalibrated) {
		// Full calibrated scene: estimate similarity transform and compare both rotation and
		// position; the rotation-aware estimation keeps a (nearly) collinear capture (corridor
		// walk) comparable, where a center-only fit would report an arbitrary roll about the
		// trajectory as rotation error
		const double threshold = 0.5 * (double)MedianNearestCameraDistance(gtScene.threadPool, dstCenters);
		Transform align;
		if (EstimateSimilarityTransformWithRotations(srcCenters, dstCenters, srcRots, dstRots, align, threshold) == 0) {
			VERBOSE("error: compare scenes similarity estimation failed (%zu matches)", srcCenters.size());
			return false;
		}

		FOREACH(k, sceneIdx) {
			const Image& img = scene.images[sceneIdx[k]];
			const Image& gtImg = gtScene.images[gtIdx[k]];

			const Point3 C_aligned = align * img.C;
			posErr.push_back(norm(C_aligned - gtImg.C));

			const Matrix3x3 R_aligned(img.R * align.R.t());
			const double ang = R2D(ACOS(ComputeAngle(R_aligned, gtImg.R)));
			if (ang > rotErrorLargeThresholdDeg)
				++numVeryLargeRotErrors;
			else if (ang > rotErrorThresholdDeg)
				++numLargeRotErrors;
			rotErrDeg.push_back(ang);
		}
	} else {
		// Uncalibrated scene (rotation-only): robustly estimate common alignment then compare
		Matrix3x3 alignR;
		if (!EstimateRotationAlignment(srcRots, dstRots, alignR)) {
			VERBOSE("error: rotation alignment estimation failed (%zu matches)", srcRots.size());
			return false;
		}
		FOREACH(k, sceneIdx) {
			const Matrix3x3 R_rel_scene(scene.images[sceneIdx[k]].R * alignR);
			const Matrix3x3& R_gt = gtScene.images[gtIdx[k]].R;
			const double ang = R2D(ACOS(ComputeAngle(R_rel_scene, R_gt)));
			if (ang > rotErrorLargeThresholdDeg)
				++numVeryLargeRotErrors;
			else if (ang > rotErrorThresholdDeg)
				++numLargeRotErrors;
			rotErrDeg.push_back(ang);
		}
	}

	const MeanStdMinMax<double> rotStats(rotErrDeg.data(), rotErrDeg.size());
	if (isCalibrated) {
		const MeanStdMinMax<double> posStats(posErr.data(), posErr.size());
		VERBOSE("Compare scenes (calibrated): matched %zu images (by %s) | rotErr[deg] mean %.3f med %.3f std %.3f max %.3f large %u very-large %u | posErr mean %.4f med %.4f std %.4f max %.4f",
			sceneIdx.size(), matchByName ? "name" : "ID", rotStats.GetMean(), rotErrDeg.GetMedian(), rotStats.GetStdDev(), rotStats.GetMax(), numLargeRotErrors, numVeryLargeRotErrors,
			posStats.GetMean(), posErr.GetMedian(), posStats.GetStdDev(), posStats.GetMax());
	} else {
		VERBOSE("Compare scenes (rotation-only): matched %zu images (by %s) | rotErr[deg] mean %.3f med %.3f std %.3f max %.3f large %u very-large %u",
			sceneIdx.size(), matchByName ? "name" : "ID", rotStats.GetMean(), rotErrDeg.GetMedian(), rotStats.GetStdDev(), rotStats.GetMax(), numLargeRotErrors, numVeryLargeRotErrors);
	}
	return true;
}
/*----------------------------------------------------------------*/
