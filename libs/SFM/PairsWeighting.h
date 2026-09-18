////////////////////////////////////////////////////////////////////
// PairsWeighting.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_PAIRS_WEIGHTING_H_
#define _SFM_PAIRS_WEIGHTING_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "ImagePair.h" // DENSE_OBSERVATION_WEIGHT, the view graph's own (fixed) dense discount


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

class SFM_API Scene;

// Compute composite weights for all image pairs in the scene.
// This function analyzes the quality of matches and geometric consistency to populate:
// - weightSpatial: Intrinsic quality of the pair
// - weightConnectivity: Relative importance in the local graph
// - weightTriplet: Global reliability check
// - pComponents: Optional output of connected components of the image graph
//
// These weights are critical for robust Structure-from-Motion (SfM):
//
// 1. weightSpatial (Intrinsic):
//    Measures the spatial distribution (grid coverage) of feature matches across the image.
//    - Importance: Matches that are well-distributed across the full field of view constrain the
//      relative pose geometry much better than matches clumped in a single area. Good spatial
//      coverage reduces uncertainty and prevents degenerate pose solutions (e.g. uncertain depth).
//
// 2. weightConnectivity (Extrinsic):
//    Measures the strength of this pair relative to the strongest connections of the involved cameras.
//    - Importance: This normalizes the score to identify edges that are "locally important".
//      A weaker edge might still be critical if it is the only connection a camera has to the rest
//      of the graph (a bridge). Conversely, weak edges between already well-connected hubs can be pruned.
//
// 3. weightTriplet (Extrinsic):
//    Measures the number of consistent triangular loops (triplets) this pair participates in.
//    - Importance: This is the strongest verification of geometric validity. While false matches can
//      sometimes satisfy 2-view epipolar geometry, they almost never satisfy consistency checks across
//      3 views (R_jk * R_ij * R_ki ~= I). Pairs with high triplet support are highly reliable and
//      should be prioritized during rotation averaging and reconstruction.
//
// The combination of these weights allows SfM algorithms to robustly select and prioritize image pairs
// that provide the most reliable and informative geometric constraints.
// The pairs are sorted by their composite weights in decreasing order.
struct SFM_API PairsWeightingConfig
{
    int gridSize = 10; // grid size for intrinsic weight computation
    unsigned minInliers = 15; // minimum inliers to consider pair for weighting
    float sigmaInlierPerMatches = 0.6f; // expected inlier vs. number of matches ratio (0.6 - AKAZE/ORB, 0.77 - SIFT)
    float tripletSaturation = 5.f; // saturation point for triplet weighting
    float maxAngleTripletDegrees = 5.f; // maximum allowed rotation error (degrees) for triplet consistency
    // What one DENSE (ROMAv2 warp sampled) match is worth as pair evidence, relative to the 1.0 a
    // descriptor match carries: this pass is the one that holds it, and it writes the discounted
    // count every view-graph consumer then reads off the pair (ImagePair::GetNumWeightedInliers).
    // Not the same quantity as BAConfig::denseObservationWeight, which is measured per solve off
    // the scene's own residuals (EstimateDenseObservationWeight) -- this one has no CLI flag
    // because no measurement has ever asked for it, and is deliberately held fixed at
    // DENSE_OBSERVATION_WEIGHT while the bundle adjustment weight moves.
    float denseObservationWeight = (float)DENSE_OBSERVATION_WEIGHT;
    // How many DENSE inliers of a pair count as evidence at all: the dense fill samples a warp, so
    // a wide, imprecise overlap can yield hundreds of dense matches over a handful of descriptor
    // ones, and uncapped they would let such a pair outweigh, in the clustering and in every view
    // graph decision, the pairs around it that descriptors verified. Capped, a dense-only pair
    // keeps denseObservationWeight * denseInlierCap weighted inliers -- enough to weigh in the
    // tens among dense-only neighbours at a normal ray angle, so a textureless interior that only
    // dense matches link stays linked -- while a pair with descriptor evidence is ranked by it.
    // The value comes from the 2026-09-18 study of an outdoor (alameda) and an indoor
    // (OfficeBadLoop) matched scene (docs/design/ExperimentRecord.md).
    unsigned denseInlierCap = 100;
};

// The fraction of a gridSize x gridSize grid over the image that the pair's track-forming matches
// occupy, in [0,1], taken as the smaller of the two images' fractions: how much of the frame the
// pair's evidence covers, whatever its count. Pinhole images bin on a uniform pixel grid,
// spherical ones on equal-solid-angle cells. 0 for a pair with no stored matches.
float SFM_API ComputePairCoverage(const ImagePair& pair, const Image& img1, const Image& img2, int gridSize);

void SFM_API ComputePairsWeights(Scene& scene, const PairsWeightingConfig& config = PairsWeightingConfig(), IIndexArr* pComponents = NULL);

} // namespace SFM

#endif // _SFM_PAIRS_WEIGHTING_H_
