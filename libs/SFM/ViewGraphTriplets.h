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
// edges, each carrying one strength s_ij = n_ij * c_ij: its epipolar inlier count n_ij discounted
// by c_ij, the fraction of the frame those inliers cover (ComputePairCoverage, the grid the pair
// weighting measures on). Wrong edges -- the repeated-structure ("doppelganger") pairs a
// retrieval step happily proposes and two-view geometry happily verifies -- are found purely from
// how that strength is distributed over the triangles of the graph: a true edge is, in every
// triangle it belongs to, comparable to the strongest edge of that triangle, while a false edge is
// systematically the weak side of triangles built around true edges. The paper weighs edges by
// n_ij alone; the coverage is what tells a doppelganger with more inliers than the true junction
// beside it (matches on the duplicated object and nowhere else) from that junction (matches over
// the whole overlap), which the counts and the triangles cannot.
//
// This is orthogonal to the rotation-cycle-consistency weight ImagePair::weightTriplet
// (PairsWeighting.cpp): a doppelganger's false edges are mutually *consistent* — the rotations
// around such a cycle close — so a cycle-error test cannot see them, while their inlier counts
// still fall short of the true structure around them. Both scores are kept.
struct SFM_API TripletFilterConfig
{
	bool enabled = false;   // remove the pairs the triplet score rejects (opt-in, see docs/design/TripletDisambiguation.md)
	// The paper's tau(m) is a ceiling: below it, the threshold is the strictest one whose survivor
	// graph keeps 99% of the unfiltered largest component together. Off, tau(m) is applied as given.
	bool autoTau = true;
	// The paper's minimum edge score m, in [0,1] (the domain this implementation enforces): 0.6
	// generic/large-scale, 0.9 highly ambiguous, 0.3 medium/small ambiguous. With autoTau this is
	// the ceiling the threshold is derived from and never exceeds.
	float minScore = 0.6f;
};

// The view graph the filter would leave behind at a given threshold: what the search judges.
struct SFM_API SurvivorGraph
{
	unsigned numNodes;          // images incident to at least one edge of the UNFILTERED graph
	unsigned largestComponent;  // images in the largest connected component of the kept edges
	unsigned numLowDegree;      // of those nodes, how many have degree < 2 in the kept graph
	unsigned numKept;           // kept edges: a scene pair duplicating an already-counted image
	                            // pair counts once, matching ComputeTripletScores' own collapse
};

// Evaluate the graph left by keeping every unscored pair and every pair scoring at or above `tau`.
// Pass tau = 0 for the unfiltered graph: scores lie in [0,1] and unscored pairs are always kept.
// Nodes are counted on the unfiltered graph, so an image that loses all its edges still counts as
// a node -- with degree 0, which is exactly what the low-degree test is there to catch.
SurvivorGraph SFM_API EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau);

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
// (G_LCT), score each such edge by the mean over its triplets of s_ij / max_{(k,l) in t} s_kl,
// and derive the threshold tau from the minimum score m and the connectivity of G_LCT. The
// strength s_ij = n_ij * c_ij is the epipolar inlier count discounted by c_ij, the fraction of the
// frame the inliers cover; gridSize is the coverage grid (PairsWeightingConfig::gridSize, so the
// filter measures coverage on the grid the pair weighting does).
// Only pairs with HasGeometricVerification() and at least one inlier are edges of G; every
// other pair — and every edge outside G_LCT, including the edges in no triplet at all — stays
// unscored (-1). A verified pair whose matches are not stored has a coverage of 0 and is not an
// edge either. The scores themselves do not depend on m; only `tau` does, so a caller that
// wants the scores alone can pass 0.
TripletScores SFM_API ComputeTripletScores(const Scene& scene, float minScore, int gridSize);

// Apply Algorithm 1 step 10 to the scene: remove only the pairs scoring below tau, then recompute
// the pair weights so the connectivity/cycle-consistency weights describe the filtered graph. This
// is a deliberate departure from the paper's step 1, which discards every edge outside G_LCT --
// including every edge in no triangle at all -- as if absence of triplet evidence meant a false
// pair; on the two labelled references those unscored pairs are overwhelmingly true (426 of 490 on
// one, 415 of 441 on the other), so an unscored pair carries no evidence either way and is kept.
// Step 11 of the paper (extract the largest connected component of the filtered graph) is
// deliberately not applied: openMVS selects components itself (SceneCluster).
// A disabled config is a no-op. Returns the number of removed pairs.
// The weighting config is not defaulted on purpose: re-weighting with anything other than the
// config the run itself matched with would silently change gridSize/minInliers under the caller.
unsigned SFM_API FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config,
	const PairsWeightingConfig& weightingCfg);

} // namespace SFM

#endif // _SFM_VIEW_GRAPH_TRIPLETS_H_
