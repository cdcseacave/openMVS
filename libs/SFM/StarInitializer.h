/*
 * StarInitializer.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_STARINITIALIZER_H_
#define _SFM_STARINITIALIZER_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"
#include "BundleAdjustment.h"
#include "GlobalScaleAveraging.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// forward declarations to avoid circular includes
class SFM_API Scene;

// The pair-scale equations of the given scene pairs (indices into scene.pairs, each with a relative
// pose): for every (view, feature) two pairs both triangulate in their own unit frame, the ratio of
// the distances they give; per pair of pairs the 10%-trimmed mean of those ratios over at least 15
// common points, as ScalePair(pairIdx2, pairIdx1, mean ratio, sqrt(points kept)) with
// pairIdx1 < pairIdx2 (the indices are scene pair indices): log s(pairIdx1) - log s(pairIdx2)
// = log of the ratio, s the scale that takes a pair's unit frame to the common one. The equations
// come sorted by (pairIdx1, pairIdx2).
std::vector<ScalePair> SFM_API ComputePairScales(const Scene& scene, const IIndexArr& pairIndices);

/**
 * @brief Configuration for star initialization
 */
struct SFM_API StarInitConfig
{
	unsigned minViews{4};           // Minimum connected views
	unsigned maxViews{36};          // Maximum connected views
	float ratioInliersFilter{0.2f}; // Ratio threshold for inlier views filter (optional, 0 to disable)
	float maxReprojError{6.f};      // Maximum reprojection error (pixels)
	float minAngleThreshold{1.f};   // Minimum angle between cameras (degrees)
	float evidenceMinAngle{1.5f};   // Ray angle a pair's inlier must reach to count as star evidence (degrees): the reconstruction's minAngleThreshold, which Scene sets it from
	bool globalRotations{false};    // Use global rotation averaging to initialize rotations (optional)

	// Refine the focal length in the star's own bundle adjustment (step 7 of Initialize()); the
	// distortion is left to the resection's global bundle adjustments once the model is large
	// enough to constrain it (a star of a handful of views is not). False for a forced or
	// otherwise known focal, which must come out of the star exactly as it went in.
	bool refineFocalLength{true};

	// The images the reference view is chosen among; empty, every image. The triplet filter fills
	// it with the largest piece its ceiling leaves (ViewGraphTriplets.h).
	IIndexArr seedViews;

	// Base BA settings the mini bundle adjustments derive from (dense-observation weight, GPS
	// weights, keypoint-confidence gating, ...); the star initializer applies its own local
	// overrides (iteration budget, intrinsics refinement) on top, same as ResectionConfig.
	BAConfig baConfig;
};

/**
 * @brief Star-configuration initialization for SfM
 *
 * Initializes reconstruction from one reference view + multiple connected views.
 * More stable than two-view initialization.
 */
class SFM_API StarInitializer
{
public:
	/**
	 * @brief Initialize scene with star configuration
	 * @param scene Scene with relative poses between images
	 * @param config Initialization configuration
	 * @return true if initialization successful
	 */
	static bool Initialize(Scene& scene, const StarInitConfig& config);

	/**
	 * @brief Select the reference view: the image whose valid pairs carry the most weighted
	 * inliers, among the seed views when any of them qualifies, else among every image. A
	 * candidate qualifies with at least minPairs valid pairs, the smallest star the caller
	 * accepts less its centre; when no image qualifies the heaviest image is returned, so that
	 * the caller reports the shortfall.
	 * @param scene Scene with image pairs
	 * @param seedViews Candidate images (empty: every image)
	 * @param minPairs Valid pairs a candidate needs (StarInitConfig::minViews - 1)
	 * @return Image ID of reference view
	 */
	static IIndex SelectReferenceView(const Scene& scene, const IIndexArr& seedViews, unsigned minPairs);

	/**
	 * @brief Estimate global scale from multiple baselines
	 * @param scene Scene with initialized poses
	 * @param refViewID Reference view ID
	 * @param connectedViews IDs of connected views
	 * @return true if scale estimation successful
	 */
	static bool EstimateGlobalScale(
		Scene& scene,
		IIndex refViewID,
		const IIndexArr& connectedViews);
};

} // namespace SFM

#endif // _SFM_STARINITIALIZER_H_
