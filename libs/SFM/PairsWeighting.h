////////////////////////////////////////////////////////////////////
// PairsWeighting.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_PAIRS_WEIGHTING_H_
#define _SFM_PAIRS_WEIGHTING_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


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
};
void SFM_API ComputePairsWeights(Scene& scene, const PairsWeightingConfig& config = PairsWeightingConfig(), IIndexArr* pComponents = NULL);

} // namespace SFM

#endif // _SFM_PAIRS_WEIGHTING_H_
