////////////////////////////////////////////////////////////////////
// PairsWeighting.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_PAIRS_WEIGHTING_H_
#define _SFM_PAIRS_WEIGHTING_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "ImagePair.h"


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
    // What a pair's DENSE (ROMAv2 warp sampled, descriptor-less) inliers are worth as EVIDENCE that
    // two images see the same thing, anchored to the matcher's own frame: a pair whose dense inliers
    // fill one whole frame's draw (denseMatchesPerFrame of them) counts them as this many descriptor
    // inliers, and a pair with fewer counts proportionally fewer, so the evidence of two dense-only
    // pairs of one image stays in the ratio of their dense counts. Proportional rather than capped: a
    // static cap (300 dense inliers at a quarter each) flattened every link of an image whose pairs
    // all exceeded it, and through the connectivity term's per-image maximum a textureless interior
    // with 1000-1700 dense matches per adjacent pair had a near and a far link weigh the same and a
    // 13-degree-wrong far link outrank the right adjacent one (indoor capture chris-house, 55 images
    // placed 12-15 units off). Anchored, a whole dense frame is a modest descriptor pair (measured at 25, and 50
    // keeps it so): on the same capture's matching the clustering graph keeps every adjacent pair and every image, a wrong
    // 50-image revisit community falls from 0.056 to 0.022 of coupling, and on alameda the three
    // wide-baseline dense-heavy pairs that once glued a seven-image community to the wrong block
    // fall from 5.4/3.8/1.3 to 0.4/0.2/0.1, well under the clustering bar of 3. This is the view
    // graph's own number and nothing else's: bundle adjustment weighs a dense OBSERVATION by the
    // precision it measures per solve (BAConfig::denseObservationWeight, DENSE_OBSERVATION_WEIGHT as
    // its fallback), a different question whose answer once happened to share the value 0.25.
    //
    // 50 (1/40 of the draw) since 2026-10-29, together with the direction check of a new pose, which
    // weighs the pose's own inliers as a pair weighs its matches. 25 (1/80) left that vote too little
    // to outweigh a few weak contradicting links. Measured on 33 captures against the code before
    // both: the summed F1 change is +0.374 by pose_score and +0.213 by a rotation-fitted re-scoring
    // that cannot flip between alignments, mostly 5ada248e 0.719 -> 0.887/0.890 (3 of 4 runs register
    // about 200 more images); 1/40 alone gives +0.198 / +0.053. 1/20 (100) was measured and refused:
    // 77-80 wrong images against 50-61, and 6DCA1E8E runs with 103 and 82 wrong under both scores;
    // 100 and 150 were already refuted at 25's expense on 2026-09-22 (the same 41 images refused at
    // every value, the indoor body wrong at 150).
    float denseFrameInliers = 50.f;
    // The matcher's dense draw per frame the number above is anchored to. The application hands it
    // the value it gives the matcher (ROMA2Config::denseMatchesPerFrame); the default is that config's.
    unsigned denseMatchesPerFrame = 2000;
    // What one dense inlier counts as in sparse inliers: its share of the frame draw times what a whole
    // frame is worth. The one formula of the pair evidence (ComputePairsWeights) and of every other
    // reader that weighs dense inliers beside described ones (the resection's direction check).
    float DenseScale() const {
        ASSERT(denseMatchesPerFrame > 0);
        return denseFrameInliers / (float)denseMatchesPerFrame;
    }
};

// The fraction of a gridSize x gridSize grid over the image that the pair's track-forming matches
// occupy, in [0,1], taken as the smaller of the two images' fractions: how much of the frame the
// pair's evidence covers, whatever its count. Pinhole images bin on a uniform pixel grid,
// spherical ones on equal-solid-angle cells. 0 for a pair with no stored matches.
float SFM_API ComputePairCoverage(const ImagePair& pair, const Image& img1, const Image& img2, int gridSize);

void SFM_API ComputePairsWeights(Scene& scene, const PairsWeightingConfig& config = PairsWeightingConfig(), IIndexArr* pComponents = NULL);

} // namespace SFM

#endif // _SFM_PAIRS_WEIGHTING_H_
