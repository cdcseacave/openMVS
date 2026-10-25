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
	unsigned minCorrespondences{15};    // Minimum 2D-3D correspondences to attempt resection; every image holding them
	                                    // is a candidate, tried in the order of the composite weight of its verified
	                                    // pairs to registered images
	unsigned minInliers{12};            // Minimum inliers to accept a pose
	float minInlierRatio{0.25f};        // Minimum share of the 2D-3D correspondences that must be inliers for a pose to
	                                    // be accepted; a smaller consensus can agree on a pose that is nowhere near the
	                                    // image's true one (0 = disabled)
	unsigned minInliersAbsolute{100};   // Inlier count from which the ratio above is waived: that many correspondences
	                                    // agreeing on a pose vouch for it whatever share of the total they are
	float maxRelativeRotationError{15.f}; // Maximum angle (degrees) between the estimated rotation and the one the
	                                    // quorum of the verified pairs to already registered images composes -- the
	                                    // largest group of those pairs agreeing on a rotation within this same angle,
	                                    // so that one pair among several cannot speak for the image;
	                                    // checked for every pose, however large its inlier share (0 = disabled)
	float maxLinkDirectionError{3.f};   // Maximum angle (degrees) between the baseline direction a verified pair to a
	                                    // registered image predicts and the one the estimated centre makes with that
	                                    // neighbour, as seen from the structure: held to the larger of the pair's ray
	                                    // angle and the model's parallax of the two cameras (the baseline over the
	                                    // median distance of the pose's inlier points), a small angle forgiving a wide
	                                    // direction error; a link that the pair and the model both see under 2 degrees
	                                    // agrees, one that only one of them does abstains. A pose is refused when the
	                                    // quorum links that contradict the direction outweigh the ones that agree
	                                    // (0 = no direction check at all). Against the pair's ray angle alone, on the
	                                    // 14 test captures: right poses refused 517 times (21 images, 10 never
	                                    // registered) -> 22 (7 images, 4), wrong poses refused 97 -> 178, accepted
	                                    // 45 -> 41
	unsigned maxLocalWindow{25};        // Max images in local BA window (0 = all neighbors)
	unsigned triangulateEvery{0};       // Run triangulation every N registered images (0 = disabled)
	unsigned localBAEvery{10};          // Run local BA every N registered images (0 = disabled)
	std::array<unsigned, 3> fullBAEvery{25, 50, 100}; // Run full BA every N registered images (0 = disabled)
	unsigned fullBARepeats{4};          // Solves a full BA may run after its first (AdjustRepeated), each one only while the
	                                    // observations the next solve would fit differ enough from the ones the last one
	                                    // fitted; 0 = one solve of the observations within fullBAInputErrorFactor x
	                                    // maxReprojError. Playground's pinhole-verified matching needs them (F1 +0.017
	                                    // over one solve, which leaves two images off in every run), and its first full
	                                    // adjustments and 2678a364's relocating cameras take all five solves
	// constant: the observations a full BA fits are the ones within this multiple of maxReprojError of the model,
	// filtered just before each solve, the filter after it holding the model to maxReprojError again: a camera placed
	// a few degrees off has its ties to the rest of the model a few pixels past the bar at its current pose, outside
	// the inlier prefix, so no solve of the prefix alone brings it back. Playground's pinhole-verified matching left
	// its turning stretch 6.6 deg off with the lens right, its ties 4-14 px off the bent poses, and 3x brought it back;
	// on the eleven verified captures with the lens fitted, a tie or better on every capture
	static constexpr float fullBAInputErrorFactor = 3.f;
	// constant: another solve runs while some camera's input changes by at least this share of the observations it
	// gave the last solve (the observations the next solve's input adds or removes); every share above it skips solves
	// after which a camera still turns 1-4 deg, and 0.08 saves 6% of the time on the eleven verified captures but leaves
	// 7 of 5936fac3's images unregistered in two runs of three
	static constexpr float fullBARepeatMinChange = 0.05f;
	// constant: a camera holding fewer observations in the input is left out of that share; at 300 the share already
	// misses a camera of 2678a364 relocating 13 deg while it holds few observations
	static constexpr unsigned fullBARepeatMinObservations = 100;
	unsigned minRefineExtIntrs{100};    // Min number of registered images to refine extended intrinsics in full BA (0 = disabled)

	float avgInliersRatioForceBA{0.6f}; // Minimum resection average inliers ratio to force full BA, averaged over the
	                                    // registered images alone (an attempt that failed or was refused adds nothing;
	                                    // 0 = disabled)
	unsigned minImagesForceBA{3};       // Minimum images registered since the last full BA before a low average inliers
	                                    // ratio may force one; without it a persistently low ratio forces a full BA after
	                                    // every single registration (0 = no minimum; the fullBAEvery cadence is unaffected)
	float denseInlierRatioFactor{0.58f}; // Inlier share a registration made of dense correspondences alone reaches,
	                                      // relative to one made of described ones, when the model is healthy: the
	                                      // dense measurement is the less precise one, so its healthy inlier share is
	                                      // lower. Normalizes each registration's ratio before it feeds the average
	                                      // above (a fraction of avgInliersRatioForceBA, not an absolute ratio;
	                                      // 1 = no distinction)
	float denseFitWeight{1.f};          // Weight of a dense (warp-sampled) correspondence against a described one's 1 in
	                                    // the polish that closes an image's absolute pose: the robust fit runs on every
	                                    // correspondence, and the polish over its inliers lets the described ones,
	                                    // sub-pixel where a dense one is a coarse warp sample whose error its neighbours
	                                    // share, set the pose wherever they are many -- what ROMA2Config::denseFitWeight
	                                    // does for a pair's relative pose (1 = no polish, 0 = the described ones alone).
	                                    // Off: at 0.003 it tied the head on the eleven verified captures (F1 +0.0015
	                                    // [-0.006, +0.010], better on two, worse on none) before the dense weight ceiling
	                                    // of the bundle adjustment shipped, and was not measured with it
	float maxReprojError{4.f};          // Reprojection error for triangulation and filtering
	float minAngleThreshold{1.f};       // Minimum triangulation angle (degrees)
	// constant: near and far depth threshold multipliers of the track filter: a track nearer than
	// multDepthNear times the median distance of the tracks to their cameras, or farther than
	// multDepthFar times it, is removed (ReconstructionConfig has the same two values)
	static constexpr float multDepthNear = 0.05f;
	static constexpr float multDepthFar = 20.f;
	RansacOptions ransac;               // RANSAC options for absolute pose estimation
	BAConfig localBAConfig;             // Local BA settings (incremental); never refines intrinsics
	BAConfig fullBAConfig;              // Full BA settings (global), restricted to the main-set intrinsics
	                                     // (focal length, k1, k2) the reconstruction's flags allow
	BAConfig extendedBAConfig;          // Same as fullBAConfig, but with every intrinsic the reconstruction's
	                                     // flags allow, main and extended alike; used once minRefineExtIntrs
	                                     // images are registered, and for the final solve

	ResectionConfig() {
		// Robust absolute pose estimation. The inlier bar is set for the dense (warp-sampled)
		// correspondences, not only for the detected keypoints: the warp is computed at a fixed
		// resolution, so on a full-resolution image a dense correspondence lands several pixels off
		// (a median of 6 px over a reconstructed scene, against 1 px for a detected one), and the
		// correspondences of an image seen through dense matches are mostly dense. At a 4 px bar the
		// resection saw a third of such an image's correspondences as inliers on a healthy model,
		// which kept the average inlier ratio under avgInliersRatioForceBA and forced a full bundle
		// adjustment after nearly every registration (154 of them over 514 images, against 10 at 8 px,
		// the poses coming out the same); it also let a minority of the correspondences vote a pose
		// in that was degrees off. The bundle adjustments that follow are robust to the outliers the
		// wider bar admits and the track filter removes them afterwards, so maxReprojError stays at 4.
		// Once the dense matching gets more precise, 6 px is the bar to try.
		ransac.threshold = 8.0;
		ransac.confidence = 0.999;
		ransac.max_iterations = 100000;
		ransac.min_iterations = 1000;

		DeriveBAConfigs(BAConfig());
	}

	// Derive localBAConfig/fullBAConfig/extendedBAConfig from the scene's configured BAConfig
	// (dense-observation weight, GPS weights, keypoint-confidence gating, ...): copy it, then reapply
	// resection's own local overrides (iteration budget, robust threshold) on top, so every field the
	// caller set on the base config reaches every bundle adjustment unchanged. The resection never
	// turns on an intrinsic the reconstruction's flags did not already ask for: fullBAConfig keeps only
	// whichever of the main-set intrinsics (focal length, k1, k2) baseCfg has, while extendedBAConfig
	// keeps baseCfg's intrinsic flags as given, main and extended alike.
	void DeriveBAConfigs(const BAConfig& baseCfg) {
		// Local BA defaults (fast); never refines intrinsics
		localBAConfig = baseCfg;
		localBAConfig.maxIterations = 20;
		localBAConfig.robustThreshold = 2.f;
		localBAConfig.refineFocalLength = false;
		localBAConfig.refineFocalLengthAspectRatio = false;
		localBAConfig.refinePrincipalPoint = false;
		localBAConfig.refineRadialDistortion12 = false;
		localBAConfig.refineTangentialDistortion = false;
		localBAConfig.refineRadialDistortion456 = false;
		localBAConfig.refineRadialDistortion3 = false;

		// Full BA defaults (stronger), main-set intrinsics only
		fullBAConfig = baseCfg;
		fullBAConfig.maxIterations = 40;
		fullBAConfig.robustThreshold = 2.f;
		fullBAConfig.refineFocalLengthAspectRatio = false;
		fullBAConfig.refinePrincipalPoint = false;
		fullBAConfig.refineRadialDistortion3 = false;
		fullBAConfig.refineTangentialDistortion = false;
		fullBAConfig.refineRadialDistortion456 = false;

		// Extended BA: same solver settings, every intrinsic flag baseCfg allows
		extendedBAConfig = baseCfg;
		extendedBAConfig.maxIterations = 40;
		extendedBAConfig.robustThreshold = 2.f;
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
 * of them added or removed. The model is left filtered at maxReprojError. Callable by any adjustment that holds its
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
	 * next round reads the list again. A stall (no candidate, or every candidate failed) with images
	 * registered since the last full adjustment runs that adjustment and tries again; any other stall
	 * ends the loop. Every resection that enters its loop -- at least one unregistered image not
	 * withheld by ExcludeImages -- closes with a full adjustment, whatever it registered; with no
	 * such image it returns at once and adjusts nothing.
	 * @return true if at least one image was registered
	 */
	bool RegisterImages();

	/**
	 * @brief Withhold images from registration, e.g. ones a caller just removed for being
	 * weakly connected: offering them back would only reproduce the same removal
	 * @param imageIDs Images to leave unregistered
	 */
	void ExcludeImages(const IIndexArr& imageIDs);

	/**
	 * @brief Estimate the pose of one unregistered image from its 2D-3D correspondences to the
	 * model's inlier tracks, and set it on the image when it is accepted
	 * @return numInliers (0 when no pose was accepted), numPoints (2D-3D correspondences tried) and
	 * numDescribed (of those, the share carrying a descriptor rather than being dense/warp-sampled)
	 */
	std::tuple<unsigned, unsigned, unsigned> RegisterImage(IIndex imageID);

private:
	Scene& scene;
	ResectionConfig config;
	std::unordered_set<IIndex> excludedImages;

	using IIndexScores = std::unordered_map<IIndex,unsigned>;

	// Count each unregistered image's 2D-3D correspondences into unregistered and list the images
	// holding minCorrespondences of them, the heaviest registered pair weight first (the composite
	// weight of the image's valid pairs to registered images; ties by the count)
	IIndexArr SelectNextImages(IIndexScores& unregistered) const;

	IIndexArr BuildLocalWindow(const IIndexArr& imageIDs) const;
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_RESECTION_H_
