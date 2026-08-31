/*
 * Resection.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_RESECTION_H_
#define _SFM_RESECTION_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "BundleAdjustment.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;
class SFM_API Image;

/**
 * @brief RANSAC options for robust estimation
 */
struct SFM_API RansacOptions
{
	double threshold = 1.0;        ///< Reprojection error threshold (pixels)
	double confidence = 0.9999;    ///< Desired confidence level
	size_t max_iterations = 100000;
	size_t min_iterations = 1000;
};

/**
 * @brief Configuration for incremental resection (image registration)
 */
struct SFM_API ResectionConfig
{
	unsigned minCorrespondences{15};    // Minimum 2D-3D correspondences to attempt resection
	unsigned minInliers{12};            // Minimum inliers to accept a pose
	unsigned maxLocalWindow{25};        // Max images in local BA window (0 = all neighbors)
	unsigned triangulateEvery{0};       // Run triangulation every N registered images (0 = disabled)
	unsigned localBAEvery{10};          // Run local BA every N registered images (0 = disabled)
	std::array<unsigned, 3> fullBAEvery{25, 50, 100}; // Run full BA every N registered images (0 = disabled)
	unsigned minRefineExtIntrs{100};    // Min number of registered images to refine extended intrinsics in full BA (0 = disabled)

	float ratioCorrespondences{0.3f};   // Min ratio of 2D-3D correspondences to best next image to accept for bundle resection (0 = disabled)
	float avgInliersRatioForceBA{0.6f}; // Minimum resection average inliers ratio to force full BA (0 = disabled)
	unsigned minImagesForceBA{3};       // Minimum images registered since the last full BA before a low average inliers
	                                    // ratio may force one; without it a persistently low ratio forces a full BA after
	                                    // every single registration (0 = no minimum; the fullBAEvery cadence is unaffected)
	float maxReprojError{4.f};          // Reprojection error for triangulation and filtering
	float minAngleThreshold{1.f};       // Minimum triangulation angle (degrees)
	float multDepthNear{0.05f};         // Near depth threshold multiplier
	float multDepthFar{20.f};           // Far depth threshold multiplier
	RansacOptions ransac;               // RANSAC options for absolute pose estimation
	BAConfig localBAConfig;             // Local BA settings (incremental)
	BAConfig fullBAConfig;              // Full BA settings (global)

	ResectionConfig() {
		// Robust absolute pose estimation
		ransac.threshold = 4.0;
		ransac.confidence = 0.999;
		ransac.max_iterations = 100000;
		ransac.min_iterations = 1000;

		// Local BA defaults (fast)
		localBAConfig.maxIterations = 20;
		localBAConfig.robustThreshold = 2.f;

		// Full BA defaults (stronger)
		fullBAConfig.maxIterations = 40;
		fullBAConfig.robustThreshold = 2.f;
		fullBAConfig.RefineMainIntrinsics();
	}
};

/**
 * @brief Incremental camera resection using PnP
 *
 * Registers new cameras by estimating absolute pose from 2D-3D correspondences.
 */
class SFM_API Resection
{
public:
	/**
	 * @brief Construct resection handler for a scene
	 * @param scene Scene to be incrementally registered
	 * @param config Resection configuration
	 */
	Resection(Scene& scene, const ResectionConfig& config);

	// Access scene
	const Scene& GetScene() const { return scene; }
	Scene& GetScene() { return scene; }

	// Access configuration
	const ResectionConfig& GetConfig() const { return config; }
	ResectionConfig& GetConfig() { return config; }

	/**
	 * @brief Register all remaining images connected to current reconstruction
	 * @return true if at least one image was registered
	 */
	bool RegisterImages();

private:
	Scene& scene;
	ResectionConfig config;

	using IIndexScores = std::unordered_map<IIndex,unsigned>;

	IIndexArr SelectNextImages(IIndexScores& unregistered) const;
	std::pair<unsigned, unsigned> RegisterImage(IIndex imageID);

	IIndexArr BuildLocalWindow(const IIndexArr& imageIDs) const;
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_RESECTION_H_
