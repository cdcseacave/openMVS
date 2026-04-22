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
#include "GlobalRotationAveraging.h"
#include "GlobalPositioning.h"
#include "SimilarityTransform.h"
#include "InterfaceMVS.h"
#include "ImportCOLMAP.h"

#include <TinyEXIF.h>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define SCENE_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Scene   "));


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
bool Scene::Save(const String& fileName, ARCHIVE_TYPE nArchiveType) const
{
	#ifdef _USE_BOOST
	TD_TIMER_STARTD();
	if (!SerializeSave(*this, fileName, nArchiveType)) {
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

bool Scene::Load(const String& fileName, ARCHIVE_TYPE nArchiveType)
{
	#ifdef _USE_BOOST
	TD_TIMER_STARTD();
	if (!SerializeLoad(*this, fileName, nArchiveType)) {
		VERBOSE("error: deserialization failed for file '%s' (archive type %d)", fileName.c_str(), (int)nArchiveType);
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
			if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".tif" || ext == ".tiff" || ext == ".jxl" || ext == ".exr" || ext == ".webp")
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
		if (!Load(MAKE_PATH_SAFE(imageFiles.front()), config.archiveType))
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

		// 2b) Import camera poses from CSV file (if configured)
		if (!config.importPosesCSV.empty()) {
			unsigned numPosesImported = ImportPosesCSV(config.importPosesCSV, images, config.importPosesMode);
			if (numPosesImported == 0) {
				VERBOSE("error: failed to import poses from CSV file '%s'", config.importPosesCSV.c_str());
				return false;
			}
			DEBUG("Imported poses for %u images from CSV file '%s'", numPosesImported, config.importPosesCSV.c_str());
		}

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
	PairsMatcher pairsMatcher(*this, config);

	// Import matches from ROMA2 NPZ file
	if (!roma2Cfg.importROMA2Path.empty() && ImportROMA2Matches(pairsMatcher, roma2Cfg) == 0) {
		VERBOSE("error: failed to import from '%s'", roma2Cfg.importROMA2Path.c_str());
		return false;
	}

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
			VERBOSE("warning: scene already matched after import");
			return false;
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

	if (config.matchImagesOnly) {
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
	Load(MAKE_PATH("scene_pre_reconstruction.sfm"), config.importCfg.archiveType);
	#endif

	// Run reconstruction method
	if (config.useGlobalSolver ? !ReconstructGlobal(config) : !ReconstructHierarchical(config))
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
	finalBaCfg.refineFocalLengthAspectRatio = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_FOCAL_LENGTH_ASPECT_RATIO) != 0;
	finalBaCfg.refineTangentialDistortion = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_TANGENTIAL_DIST) != 0;
	finalBaCfg.refineRadialDistortion456 = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_RADIAL_DIST_456) != 0;
	finalBaCfg.refinePrincipalPoint = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_PRINCIPAL_POINT) != 0;
	BundleAdjustment::Adjust(*this, finalBaCfg);
	FilterTracks(*this, config.maxFineReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);

	// Filter weakly connected images and resection remaining images into the reconstruction
	FilterWeaklyConnectedImages(*this);
	if (status.nCalibratedImages < images.size()) {
		Resection resection(*this, config.resectionCfg);
		resection.RegisterImages();
		FilterWeaklyConnectedImages(*this);
	}

	// Align scene to GPS if available
	if (config.thAlignGPS > 0 && HasImagesWithGPS())
		AlignToGPS(config.thAlignGPS);

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

		// Incrementally resect images into the reconstruction
		Resection resection(subScene, config.resectionCfg);
		resection.RegisterImages();

		// Local / global bundle adjustment for this sub-scene
		BAConfig baCfg = config.baConfig;
		baCfg.refineFocalLength = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_FOCAL_LENGTH) != 0;
		baCfg.refineFocalLengthAspectRatio = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_FOCAL_LENGTH_ASPECT_RATIO) != 0;
		baCfg.refinePrincipalPoint = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_PRINCIPAL_POINT) != 0;
		baCfg.refineRadialDistortion123 = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_RADIAL_DIST_123) != 0;
		baCfg.refineTangentialDistortion = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_TANGENTIAL_DIST) != 0;
		baCfg.refineRadialDistortion456 = (config.baIntrinsicFlags & ReconstructionConfig::INTRINSIC_RADIAL_DIST_456) != 0;
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

	#if 0
	// Import initial poses from CSV file (if configured)
	unsigned numPosesImported = ImportPosesCSV(config.importCfg.importPosesCSV, images, 1);
	BuildTracks(*this, config.minPairWeight);
	TriangulateTracks(*this, false, 16.f);
	FOREACH(i, images) {
		Image& img = images[i];
		img.LoadPixels(true);
		float a = EstimateImageSharpness(img.pixels); // ensure sharpness is estimated
		VERBOSE("Image %u blur estimate: %.4f, '%s'", img.ID, a, img.fileName.c_str());
	}
	#endif

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
	status.nCalibratedImages = 0;
	for (const Image& img : images)
		if (img.IsValid())
			status.nCalibratedImages++;
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
/*----------------------------------------------------------------*/


bool Scene::SampleColors()
{
	TD_TIMER_STARTD();
	bool wereImagesLoaded = false;

	// Resize colors array to match tracks
	Sampler::Linear<float> sampler;
	colors.resize(tracks.size());
	FOREACH(trackID, tracks) {
		const Track& track = tracks[trackID];
		if (!track.IsInlier()) {
			// Outlier: set black color
			colors[trackID] = Pixel8U::BLACK;
			continue;
		}

		// Inlier: find observation with smallest reprojection error
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
		if (minError >= FLT_MAX) {
			colors[trackID] = Pixel8U::BLACK;
			continue;
		}

		// Sample color from the best observation
		const Observation& bestObs = track.observations[bestObsIdx];
		Image& img = images[bestObs.imageID];
		const cv::KeyPoint& kp = img.keypoints[bestObs.featureID];

		// Load image pixels if not already loaded
		const bool wasLoaded = img.HasPixels();
		if (!wasLoaded) {
			img.LoadPixels();
			wereImagesLoaded = true;
		}
		if (!img.HasPixels()) {
			colors.Release();
			return false;
		}
		// Sample color from image at keypoint location using bilinear interpolation
		Pixel8U sampledColor;
		switch (img.pixels.channels()) {
		case 1:
			sampledColor.set((uint8_t)CLAMP(ROUND2INT(Sampler::Sample<uint8_t,float>(img.pixels, sampler, kp.pt)), 0, 255));
			break;
		case 3:
			sampledColor = Sampler::Sample<Pixel8U,Pixel32F>(img.pixels, sampler, kp.pt).cast<uint8_t>();
			break;
		default:
			colors.Release();
			return false;
		}
		colors[trackID] = sampledColor;
	}

	// Release image pixels if they were not loaded before
	if (!wereImagesLoaded)
		for (Image& img : images)
			img.ReleasePixels();
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

	// 4. Estimate similarity transform: Camera -> ENU
	SEACAVE::Transform T_cam_to_enu;
	if (EstimateSimilarityTransform(camCenters, enuPositions, T_cam_to_enu, threshold) == 0) {
		VERBOSE("error: failed to estimate transform");
		return false;
	}

	// 5. Transform the scene to ENU
	Transform(T_cam_to_enu);

	// 6. Set Scene::transform to the transform that brings ENU (centered) back to Absolute (ECEF)
	transform = Matrix4x4::IDENTITY;
	transform(0, 3) = centerECEF.x;
	transform(1, 3) = centerECEF.y;
	transform(2, 3) = centerECEF.z;

	status.nState.set(Status::STATE::GEO_ALIGN);
	VERBOSE("Scene aligned to GPS: aligned %u images (scale=%.4f)",
		(unsigned)camCenters.size(), T_cam_to_enu.scale);
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
}

bool Scene::UndistortImages(String outputDir, String extension, float alpha,
	CLISTDEF2(String)* outImagePaths,
	std::unordered_map<const Camera*, KMatrix>* undistortedIntrinsics) const
{
	if (outputDir.empty())
		return true;
	Util::ensureValidFolderPath(outputDir);
	Util::ensureFolder(outputDir);
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
		return true;

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
		if (SaveImage(undistorted, outPath) && outImagePaths)
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
		// Full calibrated scene: estimate similarity transform and compare both rotation and position
		Transform align;
		if (EstimateSimilarityTransform(srcCenters, dstCenters, align) == 0) {
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
