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
//
// Coverage cannot see a look-alike that fills the frame -- a round, symmetric building seen from
// a third of a turn away -- and neither can the triangles, since such pairs form triangles among
// themselves that score every edge at 1. What can is the yield: two-view geometry reads such a
// pair as a near-duplicate viewpoint, yet it delivers a fraction of the inliers a near-duplicate
// pair of these images delivers, because only the repeated structure matches. A triangle whose
// three pairs all yield poorly is no evidence for any of them.
struct SFM_API TripletFilterConfig
{
	bool enabled = false;   // remove the pairs the triplet score rejects (opt-in, see docs/design/TripletDisambiguation.md)
	// The paper's tau(m) is a ceiling: below it, the threshold is the strictest one whose survivor
	// graph joins every piece the ceiling leaves. Off, tau(m) is applied as given.
	bool autoTau = true;
	// The paper's minimum edge score m, in [0,1] (the domain this implementation enforces): 0.6
	// generic/large-scale, 0.9 highly ambiguous, 0.3 medium/small ambiguous. With autoTau this is
	// the ceiling the threshold is derived from and never exceeds -- unless the graph shows a
	// second face, see secondFaceScore.
	float minScore = 0.6f;
	// A second, stricter ceiling, tau(secondFaceScore), tried first: it is the ceiling used when
	// the graph it leaves has two faces -- its largest piece holds a strict majority of the images
	// in pieces and its second-largest piece at least a third of the largest. A two-faced building
	// matched exhaustively (the church: seven matchings) splits into its faces at this ceiling and
	// merges them at tau(minScore) in half the matchings, while a building whose graph is one face
	// (Big Ben) is cut so thin at this ceiling that the reconstruction keeps a third of it. Values
	// at or below minScore switch the second ceiling off. Part of autoTau: with autoTau off it is
	// not tried.
	float secondFaceScore = 0.75f;
	// A triangle whose three pairs all yield less than this fraction of the inliers pairs at their
	// ray angle deliver in this graph (ComputeTripletScores) is a doppelganger triangle -- look-alike
	// copies vouching for one another -- and gives its edges no evidence. 0 switches the rule off.
	float minYield = 0.4f;
};

// The view graph the filter would leave behind at a given threshold: what the search judges.
struct SFM_API SurvivorGraph
{
	unsigned numNodes;          // images incident to at least one edge of the UNFILTERED graph
	unsigned largestComponent;  // images in the largest connected component of the kept edges
	unsigned numLowDegree;      // of those nodes, how many have degree < 2 in the kept graph
	unsigned numKept;           // kept edges: a scene pair duplicating an already-counted image
	                            // pair counts once, matching ComputeTripletScores' own collapse
	unsigned numPieces;         // components of the kept graph, inside the unfiltered graph's
	                            // largest component, holding at least minPiece nodes
	unsigned numInPieces;       // nodes in those components; the rest are stragglers
	unsigned largestPiece;      // images in the largest piece (0 when there is none)
	unsigned secondPiece;       // images in the second-largest piece (0 when there is none)
	IIndexArr largestPieceViews; // the images of the largest piece, ascending (empty when there is none);
	                            // between equally large pieces, the one holding the lowest image index
	IIndexArr secondPieceViews;  // the images of the second-largest piece, ascending (empty when there is none)
	IIndexArr pieceRoots;       // one image per piece, its smallest index, ascending
	bool viewsJoined;           // the images passed as `views` all lie in one component (true when none were passed)
};

// Evaluate the graph left by keeping every unscored pair and every pair scoring at or above `tau`.
// Pass tau = 0 for the unfiltered graph: scores lie in [0,1] and unscored pairs are always kept.
// Nodes are counted on the unfiltered graph, so an image that loses all its edges still counts as
// a node -- with degree 0, which is exactly what the low-degree test is there to catch.
// A component of at least minPiece nodes, inside the unfiltered graph's largest component, is a
// piece; a component of a different, always-disconnected island of the unfiltered graph never is,
// since no tau ever gives it an edge to the rest. The filter's descent joins pieces and lets
// stragglers be (see FilterPairsByTriplets).
// `views`, when given, are images whose joining the caller asks about -- the descent passes the
// pieces' roots of the ceiling's graph and reads `viewsJoined` at each candidate threshold:
// joining every piece means the pieces share one component, not the largest component reaching a
// count of nodes, which stragglers accreting onto one piece can satisfy with another piece still
// apart.
SurvivorGraph SFM_API EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau,
	unsigned minPiece = 1, const IIndexArr* views = NULL);

// Per-pair triplet scores of a view graph, plus the statistics of the graph they were read from.
struct SFM_API TripletScores
{
	std::vector<float> scores;      // one entry per scene.pairs index; `unscored` = not an edge of G_LCT
	static constexpr float unscored = -1.f; // kept at every threshold: the method has no evidence about the pair
	static constexpr float cut = -2.f;      // removed at every threshold: FilterPairsByTriplets marks, on its own copy, the pairs the face rule cuts
	float tau;                      // the threshold of Eqn. 3 for the requested minimum score m
	size_t numTriplets;             // triplets (3-cycles) of the whole view graph G
	unsigned numTripletComponents;  // connected components of the triplet graph G_T
	unsigned numDoppelgangerTriplets; // triplets of G_LCT whose three edges all yield below minYield: counted, no evidence
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
// The yield of an edge is u_ij / H(theta_ij), capped at 1: u_ij = n_ij / min(K_i, K_j) with K_i
// the inlier count of image i's strongest pair, and H the graph's own envelope -- the 90th
// percentile of u over the edges in each 1-degree bin of median ray angle (bins holding at least
// five edges), made non-increasing in the angle. A triplet whose three edges all yield less than
// minYield is a doppelganger triplet -- three look-alike copies vouching for one another, each
// pair reading as a near-duplicate viewpoint while delivering a fraction of the inliers such a
// pair delivers -- and adds nothing to its edges' score sums while still counting in their
// divisor; minYield 0 is the paper's scoring.
TripletScores SFM_API ComputeTripletScores(const Scene& scene, float minScore, float minYield, int gridSize);

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
// pSeedViews, when given, receives the images of the largest piece the ceiling leaves (the piece
// rule: at the ceiling, before any descent; ties to the piece holding the lowest image index),
// ascending, and is emptied when the filter is off or the ceiling leaves no piece: the
// reconstruction chooses its reference view among them (StarInitConfig::seedViews), since after
// the filter the seed's side of a symmetric building is the model and the heaviest image overall
// sits in the densest cluster of look-alike views.
unsigned SFM_API FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config,
	const PairsWeightingConfig& weightingCfg, IIndexArr* pSeedViews = NULL);

} // namespace SFM

#endif // _SFM_VIEW_GRAPH_TRIPLETS_H_
