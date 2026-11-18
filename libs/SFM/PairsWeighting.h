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
	float tripletSaturation = 5.f; // saturation point for triplet weighting
	float maxAngleTripletDegrees = 5.f; // maximum allowed rotation error (degrees) for triplet consistency
	// What a pair's DENSE (ROMAv2 warp sampled, descriptor-less) inliers are worth as EVIDENCE that
	// two images see the same thing, anchored to the matcher's own frame: a pair whose dense inliers
	// fill one whole frame's draw (denseMatchesPerFrame of them) counts them as this many descriptor
	// inliers, and a pair with fewer counts proportionally fewer, so the evidence of two dense-only
	// pairs of one image stays in the ratio of their dense counts. Proportional rather than capped: a
	// static cap flattens every link of an image whose pairs all exceed it, and through the
	// connectivity term's per-image maximum a textureless interior with a thousand and more dense
	// matches per adjacent pair then weighs a near and a far link the same, so that a wrong far link
	// can outrank the right adjacent one. Anchored, a whole dense frame is a modest descriptor pair:
	// a few dense-heavy wide-baseline pairs cannot glue a small community to the wrong block. This is
	// the view graph's own number and nothing else's: bundle adjustment weighs a dense OBSERVATION by
	// the precision it measures per solve (BAConfig::denseObservationWeight, DENSE_OBSERVATION_WEIGHT
	// as its fallback), a different question.
	//
	// 50 (1/40 of the draw) gives the direction check of a new pose, which weighs the pose's own
	// inliers as a pair weighs its matches, enough vote to outweigh a few weak contradicting links; a
	// smaller value leaves that vote too little, and a larger one (1/20 and up) admits more wrong
	// images.
	float denseFrameInliers = 50.f;
	// The matcher's dense draw per frame the number above is anchored to. The application hands it
	// the value it gives the matcher (DenseMatchConfig::denseMatchesPerFrame); the default is that config's.
	unsigned denseMatchesPerFrame = 2000;
	// What one dense inlier counts as in sparse inliers: its share of the frame draw times what a whole
	// frame is worth. The one formula of the pair evidence (ComputePairsWeights) and of every other
	// reader that weighs dense inliers beside described ones (the resection's direction check).
	float DenseScale() const {
		ASSERT(denseMatchesPerFrame > 0);
		return denseFrameInliers / (float)denseMatchesPerFrame;
	}
	// NEAR-DUPLICATE pairs (ImagePair::IsNearDuplicate): two frames from almost the same spot share nearly the whole
	// frame and get the most matches of any pair, yet fix no distance, so such a pair is never the yardstick of an
	// image's connectivity (Yardstick): the connectivity factor of the pair weights, the star's reference view, floor
	// and scale gauge, and the triplet yield's capacity. A pair is one when its overlap (ImagePair::overlapArea, the
	// 10 x 10 coverage of its track-forming matches) is above maxDuplicateOverlap AND its measured ray angle is under
	// maxDuplicateRayAngle degrees; the overlap keeps out the pairs turned on the spot by a large rotation, whose ray
	// angle is small too. The defaults flag pairs whose true parallax is mostly under 1.5 deg, which fix no distance.
	// 0 turns the rule off; an angle of 0 leaves the overlap alone to decide.
	float maxDuplicateOverlap = 0.75f;
	float maxDuplicateRayAngle = 1.5f;
	bool IsNearDuplicate(const ImagePair& pair) const {
		return pair.IsNearDuplicate(maxDuplicateOverlap, D2R(maxDuplicateRayAngle));
	}
};

// The yardstick of an image's connectivity: the count of its strongest pair, a near-duplicate pair left out unless the
// image has no other pair (then it keeps the strongest of its near-duplicates). Add() every pair of the image; Count() is
// the yardstick and Pair() the pair holding it.
struct Yardstick {
	unsigned all = 0, distinct = 0;
	uint32_t pairAll = NO_ID, pairDistinct = NO_ID;
	void Add(unsigned count, bool nearDuplicate, uint32_t pair = NO_ID) {
		if (count > all) {
			all = count;
			pairAll = pair;
		}
		if (!nearDuplicate && count > distinct) {
			distinct = count;
			pairDistinct = pair;
		}
	}
	unsigned Count() const { return distinct > 0 ? distinct : all; }
	uint32_t Pair() const { return distinct > 0 ? pairDistinct : pairAll; }
};

// The fraction of a gridSize x gridSize grid over the image that the pair's track-forming matches
// occupy, in [0,1], taken as the smaller of the two images' fractions: how much of the frame the
// pair's evidence covers, whatever its count. Pinhole images bin on a uniform pixel grid,
// spherical ones on equal-solid-angle cells. 0 for a pair with no stored matches.
float SFM_API ComputePairCoverage(const ImagePair& pair, const Image& img1, const Image& img2, int gridSize);

void SFM_API ComputePairsWeights(Scene& scene, const PairsWeightingConfig& config = PairsWeightingConfig(), IIndexArr* pComponents = NULL);

} // namespace SFM

#endif // _SFM_PAIRS_WEIGHTING_H_
