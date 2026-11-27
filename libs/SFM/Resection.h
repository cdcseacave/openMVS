/*
 * Resection.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_RESECTION_H_
#define _SFM_RESECTION_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "BundleAdjustment.h"
#include "PairsWeighting.h"
#include "Track.h"


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
	unsigned minCorrespondences{15};      // Minimum 2D-3D correspondences (with a pose link) to attempt resection
	unsigned minInliers{12};              // Minimum weighted inliers (described + dense * DenseScale) to accept a pose
	float minInlierRatio{0.25f};          // Minimum inlier share of correspondences (0 = disabled)
	unsigned minInliersAbsolute{100};     // Weighted inliers from which minInlierRatio is waived
	float maxRelativeRotationError{15.f}; // Max angle (deg) from quorum-composed rotation (0 = disabled)
	float maxLinkRotationDisagreement{8.f}; // Max weighted-mean rotation angle (deg) across all links (0 = disabled)
	float maxLinkDirectionError{5.f};     // Max baseline-direction error (deg, scaled by parallax) on quorum links (<= 90; 0 = disabled)
	unsigned maxLocalWindow{25};          // Max images in local BA window (0 = all neighbors)
	unsigned localBAEvery{10};            // Run local BA every N registered images (0 = disabled)
	std::array<unsigned, 3> fullBAEvery{25, 50, 100}; // Run full BA every N registered images (0 = disabled)
	unsigned fullBARepeats{2};            // Max additional solves in AdjustRepeated while input keeps changing
	static constexpr float fullBAInputErrorFactor = 3.f; // Multiplies maxReprojError to gather BA input observations
	static constexpr float fullBARepeatMinChange = 0.05f; // Min per-camera input change fraction to trigger another solve
	static constexpr unsigned fullBARepeatMinObservations = 100; // Min observations for a camera to vote on repeat
	unsigned minRefineExtIntrs{100};      // Min registered images to refine extended intrinsics in full BA (0 = disabled)

	float avgInliersRatioForceBA{0.6f};   // Min normalized running-average inlier ratio before forcing full BA (0 = disabled)
	unsigned minImagesForceBA{3};         // Min registrations since last full BA before avgInliersRatioForceBA may trigger
	float denseInlierRatioFactor{0.58f};  // Expected healthy inlier share of dense correspondences relative to described ones
	float denseFitWeight{1.f};            // Weight of dense correspondences in post-RANSAC pose polish (1 = no polish)
	float maxReprojError{4.f};            // Reprojection error for triangulation and filtering (pixels)
	float minAngleThreshold{1.f};         // Min triangulation angle (deg) while growing a block
	static constexpr float multDepthNear = 0.05f;
	static constexpr float multDepthFar = 20.f;
	RansacOptions ransac;                 // RANSAC options for absolute pose estimation
	BAConfig localBAConfig;               // Local BA settings (never refines intrinsics)
	BAConfig fullBAConfig;                // Full BA settings (main-set intrinsics only)
	BAConfig extendedBAConfig;            // Extended BA settings (all intrinsics allowed by base config)

	PairsWeightingConfig pairsWeighting;  // Pair weighting config (used to weight dense pose inliers)

	ResectionConfig() {
		// 8 px accommodates coarse dense warp correspondences during PnP; BA and FilterTracks tighten to maxReprojError
		ransac.threshold = 8.0;
		ransac.confidence = 0.999;
		ransac.max_iterations = 100000;
		ransac.min_iterations = 1000;

		DeriveBAConfigs(BAConfig());
	}

	// Derive local/full/extended BA configs from baseCfg while preserving caller's global settings
	void DeriveBAConfigs(const BAConfig& baseCfg) {
		localBAConfig = baseCfg;
		localBAConfig.maxIterations = 20;
		localBAConfig.robustThreshold = 2.f;
		localBAConfig.ClearIntrinsicRefinement();

		fullBAConfig = baseCfg;
		fullBAConfig.maxIterations = 40;
		fullBAConfig.robustThreshold = 2.f;
		fullBAConfig.refineFocalLengthAspectRatio = false;
		fullBAConfig.refinePrincipalPoint = false;
		fullBAConfig.refineRadialDistortion3 = false;
		fullBAConfig.refineTangentialDistortion = false;
		fullBAConfig.refineRadialDistortion456 = false;

		extendedBAConfig = baseCfg;
		extendedBAConfig.maxIterations = 40;
		extendedBAConfig.robustThreshold = 2.f;

		localBAConfig.minTriangulationAngle = fullBAConfig.minTriangulationAngle = extendedBAConfig.minTriangulationAngle = minAngleThreshold;
	}
};

/**
 * @brief Full bundle adjustment of a model held to config.maxReprojError, repeated while what it fits keeps changing
 *
 * Each solve fits the observations within ResectionConfig::fullBAInputErrorFactor x maxReprojError of the model (the
 * track filter at that bar just before it); after it the filter holds the model to the bar again and the tracks left
 * without a point are triangulated again, which is what the next solve would fit. A solve that reached its minimum
 * differs from the next one only through that input, so another one runs, up to config.fullBARepeats of them, only
 * while some camera holding fullBARepeatMinObservations observations in the input has at least fullBARepeatMinChange
 * of them added or removed; the last solve allowed is followed by the same triangulation, so the model ends with the
 * tracks a stopped solve dropped triangulated again, as when it stops for want of change. The model is left filtered
 * at maxReprojError. Callable by any adjustment that holds its
 * model to the resection's bars, not only the resection's own.
 * @return the number of solves
 */
SFM_API unsigned AdjustRepeated(Scene& scene, const BAConfig& baConfig, const ResectionConfig& config);

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
	 *
	 * Each round tries the candidates of SelectNextImages down the list until one registers; the
	 * next round reads the list again. A stall (no candidate, or every candidate failed) with the model
	 * changed since the last full adjustment -- images registered, or images removed (ImagesRemoved) --
	 * runs that adjustment and tries again; any other stall ends the loop. Every resection closes with
	 * a full adjustment, whatever it registered, also when it has no image to try (every image holds a
	 * pose, or the ones left are withheld by ImagesRemoved): the loop is then skipped and the closing runs.
	 * @param closeWhenNothingRegistered false skips the closing adjustment when no image registered,
	 * so a model that took nothing is left exactly as it was (the pool resection's passes)
	 * @return true if at least one image was registered
	 */
	bool RegisterImages(bool closeWhenNothingRegistered = true);

	/**
	 * @brief The images a filter just removed from the model (FilterWeaklyConnectedImages): the ones
	 * it found contradicting the model are withheld from registration, as offering them back would
	 * only reproduce the same removal, and the removal counts as a change of the model since its last
	 * full adjustment, as a registration does: their tracks lost views, so a stall runs the full
	 * adjustment (which triangulates the tracks left without a position again) and another pass
	 * before the resection can stop
	 */
	void ImagesRemoved(const RemovedImages& removed);

	/**
	 * @brief Estimate the pose of one unregistered image from its 2D-3D correspondences to the
	 * model's inlier tracks, and set it on the image when it is accepted
	 * @return numInliers (0 when no pose was accepted), numPoints (2D-3D correspondences tried) and
	 * numDescribed (of those, the share carrying a descriptor rather than being dense/warp-sampled)
	 */
	std::tuple<unsigned, unsigned, unsigned> RegisterImage(IIndex imageID);

	// What the last RegisterImages did
	struct Stats {
		struct Attempt {
			IIndex imageID;
			bool registered; // false: the pose was refused or not estimated
		};
		std::vector<Attempt> attempts; // every image tried, in the order tried
		unsigned numFullAdjustments = 0; // full adjustments run, the stall retries' and the closing one included
	};
	const Stats& GetStats() const { return stats; }

private:
	Stats stats;
	Scene& scene;
	ResectionConfig config;
	std::unordered_set<IIndex> excludedImages;
	bool imagesRemoved = false; // images left the model since its last full adjustment (ImagesRemoved)

	using IIndexScores = std::unordered_map<IIndex,unsigned>;

	// Count each unregistered image's 2D-3D correspondences into unregistered and list the images
	// holding minCorrespondences of them and at least one link (ForEachLink), the heaviest link weight
	// first (the composite weight of the image's links; ties by the count)
	IIndexArr SelectNextImages(IIndexScores& unregistered) const;

	IIndexArr BuildLocalWindow(const IIndexArr& imageIDs) const;
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_RESECTION_H_
