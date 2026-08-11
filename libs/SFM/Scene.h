////////////////////////////////////////////////////////////////////
// Scene.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_SCENE_H_
#define _SFM_SCENE_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"
#include "Image.h"
#include "ImagePair.h"
#include "Track.h"
#include "PairsMatcher.h"
#include "FeaturesExtractor.h"
#include "ViewGraphCalibrator.h"
#include "StarInitializer.h"
#include "Resection.h"
#include "SceneCluster.h"
#include "BundleAdjustment.h"
#include "GlobalAlignment.h"
#include "ImportROMA2.h"
#include "PoseIO.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

/**
 * @brief Configuration for importing images and initializing cameras
 */
struct SFM_API ImportConfig {
	bool useExif = true;             // attempt to parse EXIF (tinyexif) if available
	float defaultFocalRatio = 1.2f;  // fallback focal = ratio * max(width,height)
	float focalLength = 0.f;         // force focal length (in pixels) for specified images (0 = disabled)
	float k1 = 0.f;                  // force k1 distortion coefficient (0 = not used)
	float k2 = 0.f;                  // force k2 distortion coefficient (0 = not used)
	String imageIndicesStr;          // image indices to apply forced parameters (empty = all images)
	String importPosesFile;          // import camera poses from file (.csv or .json); empty = disabled
	PoseImportMode importPosesMode = PoseImportMode::NONE; // what to import from the poses file (see PoseImportMode)
	// camera-axes convention of a frames.json poses file (ignored for .csv);
	// AUTO imports the poses with the ARKit convention and defers the decision to
	// DetectFramesConvention(), which can only resolve it once the pairs are matched
	FramesConvention framesConvention = FramesConvention::AUTO;
	ARCHIVE_TYPE archiveType = ARCHIVE_DEFAULT; // archive type for loading/saving scenes
};

/**
 * @brief Lightweight reconstruction configuration
 *
 * This struct avoids depending on other SFM headers to prevent
 * circular includes. The values are later converted to the
 * internal typed configurations inside Scene::Reconstruct().
 */
struct SFM_API ReconstructionConfig {
	// Import configuration
	ImportConfig importCfg;

	// Feature extraction configuration
	FeatureExtractionConfig featuresCfg;

	// ROMA2 configuration
	ROMA2Config roma2Cfg;

	// Matching parameters (will be translated to MatchConfig)
	MatchConfig matchCfg;
	bool matchImagesOnly{false}; // only match image pairs and save scene without reconstruction

	// View graph calibration parameters
	ViewGraphCalibratorConfig viewgraphCfg;

	// Tracks parameters
	float minPairWeight{3.f}; // minimum weight for a pair to be used in creating tracks (0 = disabled)
	float maxReprojError{4.f}; // Reprojection error for coarse triangulation and filtering
	float maxFineReprojError{2.f}; // Reprojection error for fine triangulation and filtering
	float minAngleThreshold{1.5f}; // Minimum triangulation angle (degrees)
	float multDepthNear{0.05f}; // Near depth threshold multiplier
	float multDepthFar{20.f}; // Far depth threshold multiplier

	// Clustering parameters
	ClusterConfig clusterCfg;

	// Initialization parameters
	StarInitConfig initCfg;

	// Resection parameters
	ResectionConfig resectionCfg;
	bool useGlobalSolver{false}; // use global solver instead of hierarchical solver

	// Global alignment parameters
	GlobalAlignmentConfig globalAlignmentCfg;

	// Bundle adjustment parameters
	enum IntrinsicFlags : unsigned {
		INTRINSIC_NONE = 0,
		INTRINSIC_FOCAL_LENGTH = 1 << 0,              // Refine fx, fy
		INTRINSIC_FOCAL_LENGTH_ASPECT_RATIO = 1 << 1, // Refine fx, fy while keeping aspect ratio constant
		INTRINSIC_PRINCIPAL_POINT = 1 << 2,           // Refine cx, cy
		INTRINSIC_RADIAL_DIST_123 = 1 << 3,           // Refine k1, k2, k3
		INTRINSIC_TANGENTIAL_DIST = 1 << 4,           // Refine p1, p2
		INTRINSIC_RADIAL_DIST_456 = 1 << 5,           // Refine k4, k5, k6
		INTRINSIC_MAIN = INTRINSIC_FOCAL_LENGTH | INTRINSIC_RADIAL_DIST_123,
		INTRINSIC_MAIN_EXTRA = INTRINSIC_FOCAL_LENGTH | INTRINSIC_RADIAL_DIST_123 |
						INTRINSIC_PRINCIPAL_POINT | INTRINSIC_TANGENTIAL_DIST,
		INTRINSIC_ALL = INTRINSIC_FOCAL_LENGTH | INTRINSIC_FOCAL_LENGTH_ASPECT_RATIO |
						INTRINSIC_PRINCIPAL_POINT | INTRINSIC_RADIAL_DIST_123 |
						INTRINSIC_TANGENTIAL_DIST | INTRINSIC_RADIAL_DIST_456
	};
	unsigned baIntrinsicFlags{INTRINSIC_MAIN_EXTRA}; // which intrinsics to refine
	BAConfig baConfig;  // detailed BA configuration

	float thAlignGPS{5.f}; // threshold for aligning to GPS (meters)
	bool extractColors{false}; // extract colors for reconstructed points
	bool estimatePoseUncertainty{false}; // record per-image pose uncertainty from the last global bundle adjustment

	// true when the pose import was configured with a mode that brings in extrinsics,
	// which selects the known-poses ("finetune") reconstruction path
	bool HasKnownPoses() const {
		return !importCfg.importPosesFile.empty() &&
			(importCfg.importPosesMode == PoseImportMode::POSES_INTRINSICS ||
			 importCfg.importPosesMode == PoseImportMode::POSES);
	}
};


// Scene contains all data for a Structure-from-Motion reconstruction:
// cameras, images, image pairs, and optionally a 3D point cloud
class SFM_API Scene
{
public:
	// Camera array (can be shared between images)
	CameraPtrArr cameras;

	// Image array
	ImageArr images;

	// Image pair array (relationships between images)
	ImagePairArr pairs;

	// 3D point tracks (observations + triangulated positions)
	TrackArr tracks;

	// Optional per-track colors (aligned with tracks array)
	Pixel8UArr colors;

	// Optional per-image pose uncertainty estimated from the last global bundle adjustment
	// run during reconstruction (empty unless ReconstructionConfig::estimatePoseUncertainty);
	// kept consistent with the current world frame by Scene::Transform
	PoseUncertaintyArr poseUncertainty;

	// Camera poses as imported before refinement, keyed by image ID; used to re-align the
	// refined reconstruction back to the input frame (Scene::AlignToPriorPoses).
	// Transient: deliberately not serialized, but preserved by regular Scene copies and moves.
	std::unordered_map<IIndex, Pose3D> priorPoses;

	// Optional transformation used to convert from absolute to relative coordinate system
	Matrix4x4 transform;

	// Optional minimum oriented bounding box containing the scene Region of Interest (ROI)
	OBB3 obb;

	// Structure storing status related data
	struct Status {
		enum class STATE : uint8_t {
			EMPTY = 0,
			FEATURES_EXTRACTED = 1,
			MATCHED = 2,
			CALIBRATED = 4,
			GEO_ALIGN = 8
		};
		Flags nState{STATE::EMPTY}; // current state (now type-safe with STATE enum)
		FeatureType nFeaturesType{FeatureType::NONE}; // type of features extracted (0=none,1=AKAZE,2=ORB,3=SIFT)
		uint32_t nCalibratedImages{0}; // number of calibrated images
		uint32_t nTracks{0}; // number of inlier tracks

		#ifdef _USE_BOOST
		// implement BOOST serialization
		template<class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & nState;
			ar & nFeaturesType;
			ar & nCalibratedImages;
			ar & nTracks;
		}
		#endif
	} status;

	unsigned nMaxThreads; // maximum number of threads used to distribute the work load (always >0, default = hardware concurrency)
	BS::light_thread_pool threadPool; // thread pool for parallel processing

public:
	Scene(unsigned _nMaxThreads=0);
	Scene(const Scene& scene);
	Scene(Scene&& scene) noexcept;
	~Scene() { Release(); }

	Scene& operator=(const Scene& scene);
	Scene& operator=(Scene&& scene) noexcept;

	// Release all resources
	void Release();

	// Check if scene is empty
	inline bool IsEmpty() const {
		return images.empty();
	}

	// Check if any images have GPS metadata
	bool HasImagesWithGPS(bool validOnly = true) const;

	// Find pair by image IDs
	ImagePair* FindPair(IIndex ID1, IIndex ID2) {
		ASSERT(ID1 < ID2);
		for (ImagePair& pair: pairs)
			if (pair.ID1 == ID1 && pair.ID2 == ID2)
				return &pair;
		return NULL;
	}
	const ImagePair* FindPair(IIndex ID1, IIndex ID2) const {
		ASSERT(ID1 < ID2);
		for (const ImagePair& pair: pairs)
			if (pair.ID1 == ID1 && pair.ID2 == ID2)
				return &pair;
		return NULL;
	}

	// Invalidate image pose and remove it from any tracks it is part of
	bool InvalidateImage(IIndex imgID);
	// Batch variant: invalidate several images with a single sweep over all tracks
	// (O(tracks) total instead of O(tracks) per image). Returns the number invalidated.
	unsigned InvalidateImages(const IIndexArr& imgIDs);

	// Rescan all images and refresh status.nCalibratedImages (which is otherwise delta-maintained);
	// returns the recomputed count
	uint32_t RecomputeCalibratedImages();

	// Save/Load scene to file
	// (the save file embeds a small header recording the archive/compression type,
	//  so Load is self-describing and does not need to be told the archive type)
	bool Save(const String& fileName, ARCHIVE_TYPE nArchiveType = ARCHIVE_DEFAULT) const;
	bool Load(const String& fileName);

	/**
	 * @brief Import images and initialize cameras from source
	 * @param source Either a folder path (will scan images) or a list of image paths separated by ';'
	 * @param config Import configuration
	 * @return true if images were successfully imported
	 */
	bool Import(const String& source, const ImportConfig& config);

	/**
	 * @brief Extract features from all images that don't have features yet
	 * @param config Feature extraction configuration
	 * @return true if feature extraction completed successfully
	 */
	bool ExtractFeatures(const FeatureExtractionConfig& config);

	/**
	 * @brief Match image pairs to find correspondences
	 * @param config Matching configuration
	 * @return true if matching completed successfully
	 */
	bool MatchPairs(const MatchConfig& config, const ROMA2Config& roma2Cfg = ROMA2Config(), const ViewGraphCalibratorConfig& vgConfig = ViewGraphCalibratorConfig());

	/**
	 * @brief Run a full reconstruction from a folder or semicolon-separated list
	 * @param source Either a folder path (will scan images) or a list of image paths separated by ';'
	 * @param config Reconstruction configuration
	 * @return true if reconstruction completed (partial recon still returns true but may be degraded)
	 */
	bool Reconstruct(const String& source, const ReconstructionConfig& config);

	/**
	 * @brief Run hierarchical reconstruction
	 * @param config Reconstruction configuration
	 * @return true if reconstruction completed
	 *
	 * Pipeline:
	 * 1. Cluster scene if necessary
	 * 2. Reconstruct each cluster
	 * 3. Merge/align sub-scenes
	 */
	bool ReconstructHierarchical(const ReconstructionConfig& config);

	/**
	 * @brief Run global reconstruction
	 * @param config Reconstruction configuration
	 * @return true if reconstruction completed
	 *
	 * Pipeline:
	 * 1. Global Rotation Averaging
	 * 2. Global Positioning (Rotation fixed, random translation init)
	 */
	bool ReconstructGlobal(const ReconstructionConfig& config);

	/**
	 * @brief Run a finetune reconstruction starting from the imported camera poses
	 * @param config Reconstruction configuration (must satisfy config.HasKnownPoses())
	 * @return true if reconstruction completed
	 *
	 * Pipeline:
	 * 1. Validate the pose import covered the dataset and remember the imported poses
	 * 2. Resolve the camera-axes convention of a frames.json import (see DetectFramesConvention)
	 * 3. Build tracks and triangulate them with the imported poses
	 * 4. Bundle-adjust, re-triangulate the outliers, bundle-adjust again
	 *
	 * The imported poses are used as initialization only; the result is brought back to the
	 * input coordinate frame at the end of Reconstruct() by AlignToPriorPoses().
	 */
	bool ReconstructKnownPoses(const ReconstructionConfig& config);

	/**
	 * @brief Sample colors for each track from observations
	 *
	 * For inlier tracks, selects the observation with the smallest reprojection error
	 * and samples the color from that image at the observation location.
	 * For outlier tracks, sets the color to black.
	 * The colors array is resized to match the tracks array.
	 * @return true on success
	 */
	bool SampleColors();

	/**
	 * @brief Align the scene to GPS positions (if available)
	 *
	 * Estimates a similarity transform between the positions of the calibrated images
	 * and the corresponding GPS positions converted to ENU and centered to 0.
	 * The transform is stored in Scene::transform.
	 * @param threshold RANSAC threshold (0 to disable RANSAC)
	 * @return true if alignment was successful (requires at least 3 GPS positions)
	 */
	bool AlignToGPS(double threshold = 0.0);

	/**
	 * @brief Align the refined reconstruction back to the coordinate frame of the imported
	 *        prior poses (also anchors the otherwise-free scale)
	 *
	 * Estimates a similarity transform between the centers of the images still valid after
	 * filtering and their Scene::priorPoses counterparts, and applies it to the whole scene
	 * (including the images resected along the way, which have no prior). Unlike AlignToGPS
	 * this does not touch Scene::transform nor set GEO_ALIGN: the prior frame is not a
	 * geo-referenced one, it is simply the frame the poses were imported in.
	 * @param thresholdRatio RANSAC inlier threshold, expressed as a fraction of the median
	 *        distance between neighboring prior camera centers; the prior frame may be in
	 *        arbitrary units, so no absolute threshold can be chosen here (0 disables RANSAC)
	 * @return false if the priors are too few or too degenerate to estimate a similarity transform
	 */
	bool AlignToPriorPoses(float thresholdRatio = 0.5f);

	/**
	 * @brief Get ECEF centroid stored in trasform if the scene is aligned to GPS
	 * @return ECEF centroid
	 */
	const Point3 GetCenterECEF() const {
		ASSERT(status.nState.isSet(Status::STATE::GEO_ALIGN));
		return Point3(transform(0, 3), transform(1, 3), transform(2, 3));
	}

	/**
	 * @brief Apply a similarity transform to the scene
	 *
	 * Transforms all cameras, images, and 3D points.
	 * @param transform The similarity transform to apply
	 */
	void Transform(const struct Transform& transform);

	/**
	 * @brief Undistort all pinhole images with distortion using cached OpenCV remap maps.
	 * Writes undistorted images to the given directory and optionally returns the
	 * generated file paths and undistorted intrinsics (newK) per camera.
	 * @param outputDir destination directory (created if missing); no-op when empty
	 * @param extension output image extension (default: .jxl)
	 * @param alpha Free scaling parameter between 0 (when all the pixels in the undistorted image
	 *        are valid) and 1 (when all the source image pixels are retained in the undistorted image)
	 * @param outImagePaths optional output vector (size == images.size()) filled with
	 *        generated paths for images that were undistorted, empty otherwise
	 * @param undistortedIntrinsics optional output map camera* -> newK used for undistort
	 * @return true on success (including no-op when outputDir is empty)
	 */
	bool UndistortImages(String outputDir, String extension, float alpha = 0.6f,
		CLISTDEF2(String)* outImagePaths = NULL,
		std::unordered_map<const Camera*, KMatrix>* undistortedIntrinsics = NULL) const;

	/**
	 * @brief Precompute neighbor views based on shared tracks
	 *
	 * For each image, identifies all other images that share track observations
	 * and computes connectivity metrics:
	 * - Number of shared tracks (points visible in both images)
	 * - Average angle between viewing rays for shared tracks
	 * - Overlap area (fraction of reference image covered by shared points)
	 *
	 * Results are sorted by number of shared tracks (descending).
	 *
	 * @param neighbors Output array of neighbor scores per image (indexed by image ID)
	 *                  Must be pre-allocated with size == images.size()
	 */
	void PrecomputeTrackBasedNeighbors(std::vector<ViewScoreArr>& neighbors) const;

	/**
	 * @brief Export tracks and optionally image positions to PLY format
	 *
	 * Exports the reconstructed 3D positions of tracks to a PLY file.
	 * Track colors are exported if available (from colors array).
	 * Optionally, calibrated image positions can also be exported as vertices.
	 *
	 * @param fileName Output PLY file path
	 * @param bExportImages If true, also export calibrated image positions as vertices
	 * @param bInliersOnly If true, only export inlier tracks (numInliers >= 2)
	 * @param bBinary If true, write binary PLY; otherwise ASCII
	 * @return true on success
	 */
	bool ExportPLY(const String& fileName, bool bExportImages = false,
		bool bInliersOnly = true, bool bBinary = true) const;

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & cameras;
		ar & images;
		ar & pairs;
		ar & tracks;
		ar & colors;
		ar & poseUncertainty;
		ar & transform;
		ar & obb;
		ar & status;
	}
	#endif
};
/*----------------------------------------------------------------*/


/**
 * @brief Compare reconstructed scene against ground-truth poses from an MVS file
 * @param scene The reconstructed scene to evaluate
 * @param gtFile Path to MVS file containing ground-truth poses
 * @param matchByName If true, match images by filename stem; otherwise by image ID
 * @return true if comparison succeeded (does not indicate quality)
 */
SFM_API bool CompareScenes(const Scene& scene, const String& gtFile, bool matchByName = true);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_SCENE_H_
