////////////////////////////////////////////////////////////////////
// ViewGraphTriplets.h
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_VIEW_GRAPH_TRIPLETS_H_
#define _SFM_VIEW_GRAPH_TRIPLETS_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "PairsWeighting.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

class SFM_API Scene;

// Triplet-based view-graph disambiguation, reimplemented from
//   S. M. Manam and V. M. Govindu, "Leveraging Camera Triplets for Efficient and Accurate
//   Structure-from-Motion", CVPR 2024 (Algorithm 1, Eqn. 3).
//
// The view graph G = (V,E) has the images as nodes and the geometrically verified pairs as
// edges, each carrying a single integer: its epipolar inlier count n_ij. Wrong edges — the
// repeated-structure ("doppelganger") pairs a retrieval step happily proposes and two-view
// geometry happily verifies — are found purely from how that integer is distributed over the
// triangles of the graph: a true edge is, in every triangle it belongs to, comparable in
// strength to the strongest edge of that triangle, while a false edge is systematically the
// weak side of triangles built around true edges.
//
// This is orthogonal to the rotation-cycle-consistency weight ImagePair::weightTriplet
// (PairsWeighting.cpp): a doppelganger's false edges are mutually *consistent* — the rotations
// around such a cycle close — so a cycle-error test cannot see them, while their inlier counts
// still fall short of the true structure around them. Both scores are kept.
struct SFM_API TripletFilterConfig
{
	bool enabled = false;   // remove the pairs the triplet score rejects (opt-in, see docs/design/TripletDisambiguation.md)
	float minScore = 0.6f;  // the paper's minimum edge score m: 0.6 generic/large-scale, 0.9 highly ambiguous, 0.3 medium/small ambiguous
};

// Per-pair triplet scores of a view graph, plus the statistics of the graph they were read from.
struct SFM_API TripletScores
{
	std::vector<float> scores;      // one entry per scene.pairs index; -1 = unscored (not an edge of G_LCT)
	float tau;                      // the threshold of Eqn. 3 for the requested minimum score m
	unsigned numTriplets;           // triplets (3-cycles) of the whole view graph G
	unsigned numTripletComponents;  // connected components of the triplet graph G_T
	unsigned numScoredPairs;        // scene.pairs entries that got a score (the edges of G_LCT)
	unsigned numNodes;              // |V| of G_LCT (the graph tau is derived from)
	unsigned maxDegree;             // d_max of G_LCT
};

// Score every pair of the scene by the paper's Algorithm 1, steps 1-9: build the triplet graph
// G_T of the view graph, keep the edges participating in its largest connected component
// (G_LCT), score each such edge by the mean over its triplets of n_ij / max_{(k,l) in t} n_kl,
// and derive the threshold tau from the minimum score m and the connectivity of G_LCT.
// Only pairs with HasGeometricVerification() and at least one inlier are edges of G; every
// other pair — and every edge outside G_LCT, including the edges in no triplet at all — stays
// unscored (-1). The scores themselves do not depend on m; only `tau` does, so a caller that
// wants the scores alone can pass 0.
TripletScores SFM_API ComputeTripletScores(const Scene& scene, float minScore);

// Apply Algorithm 1 step 10 to the scene: remove the pairs scoring below tau *and* the unscored
// pairs, then recompute the pair weights so the connectivity/cycle-consistency weights describe
// the filtered graph. Step 11 of the paper (extract the largest connected component of the
// filtered graph) is deliberately not applied: openMVS selects components itself (SceneCluster).
// A disabled config is a no-op. Returns the number of removed pairs.
// The weighting config is not defaulted on purpose: re-weighting with anything other than the
// config the run itself matched with would silently change gridSize/minInliers under the caller.
unsigned SFM_API FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config,
	const PairsWeightingConfig& weightingCfg);

} // namespace SFM

#endif // _SFM_VIEW_GRAPH_TRIPLETS_H_
