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
	// Remove the pairs the triplet score rejects. On by default: without `cut` the filter
	// removes only what the graph can spare (see keepPairs), and a graph with no repeated
	// structure loses nothing it needs (docs/design/TripletDisambiguation.md).
	bool enabled = true;
	// The cutting rule: the ceiling applied as given when its largest piece holds a majority,
	// the pieces below it left apart, the second ceiling naming the faces and the other face cut
	// off, the descent below a shattered ceiling, no floor. The rule for a scene with repeated
	// structure -- a symmetric building is unfolded by cutting its graph -- and the rule that
	// halves the registrations of an interior, whose ceiling cuts off rooms. Off, the keep
	// mode: the candidates are the scored pairs below the threshold that join two pieces the
	// ceiling keeps apart (a weak pair inside one piece is kept), every image keeps at least
	// keepPairs of its pairs and enough of its best-scoring ones to hold keepMatches inliers, and
	// every component of the matched graph stays one component (its strongest candidates are
	// kept until it does). autoTau and secondFaceScore apply only with cut.
	bool cut = false;
	// The floor of the keep mode: the pairs and the weighted inliers (summed over its kept
	// pairs) every image keeps, counting only the pairs whose ray angle (meanRayAngle, the
	// median angle between the viewing rays of the track-forming matches) reaches keepMinAngle
	// degrees -- a near-duplicate pair yields no 3D point, and on a video every pair but the
	// consecutive ones is the weak side of a triangle a consecutive pair tops, so a burst of
	// near-duplicate frames would otherwise keep only its own pairs and lose every link to the
	// rest of the capture, then be invalidated for a median triangulation angle below the
	// reconstruction's 1.5 degrees; 3 is twice that bar. A pair whose angle was never measured
	// (0) counts. The image's counting pairs that are not candidates at that threshold count
	// first; below the floor, its best-scoring counting candidates are retained, ties to the
	// stronger. keepPairs and keepMatches at 0 keep nothing for the floor's sake, keepMinAngle
	// at 0 counts every pair; the components are kept whole regardless.
	unsigned keepPairs = 3;
	unsigned keepMatches = 2000;
	float keepMinAngle = 3.f;
	// The keep mode's threshold is the strictest one the graph fits: the ceiling when at most
	// this share of the images (nodes of the graph) already fall short of the floor from their
	// counting pairs that are not candidates at that threshold, else the largest score below the
	// ceiling at which that holds. The paper's tau(m) presumes an internet collection where an
	// image keeps hundreds of pairs above it and the ceiling stands; a small set matched
	// exhaustively puts the ceiling near 1 and fits a lower one, still above its doppelganger
	// pairs; an interior fits none -- most of its images hold fewer matches than the floor asks
	// for in the whole graph -- and nothing is removed, since on such a graph the reconstruction
	// flips under any change of its pairs. 1 fits every threshold.
	float keepMaxShort = 0.5f;
	// With cut: the paper's tau(m) is a ceiling: below it, the threshold is the strictest one
	// whose survivor graph joins every piece the ceiling leaves. Off, tau(m) is applied as given.
	// Without cut the ceiling only names the candidates, and this flag plays no part.
	bool autoTau = true;
	// The paper's minimum edge score m, in [0,1] (the domain this implementation enforces): 0.3
	// medium/small ambiguous (the default), 0.6 generic/large-scale, 0.9 highly ambiguous. With
	// autoTau this is the ceiling the threshold is derived from and never exceeds, a second face
	// included: see secondFaceScore.
	float minScore = 0.3f;
	// A second, stricter ceiling, tau(secondFaceScore), tried first to NAME the faces: when the
	// graph it leaves has two -- its largest piece holds a strict majority of the images in pieces
	// and its second-largest piece at least a third of the largest -- tau(minScore) applies inside
	// the larger face, every pair joining the other face to an image outside it is cut, and so is
	// every pair of an image outside both faces whose kept pairs reach both. A two-faced building
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

// Evaluate the graph left by keeping every unscored pair and every pair scoring at or above `tau`;
// a pair marked TripletScores::cut is removed at every tau. Pass tau = 0 for the unfiltered graph
// of a score array carrying no cut marks: scores lie in [0,1] and unscored pairs are always kept.
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
// Two modes. With config.cut the filter applies the cutting rule above: the ceiling as given
// when its largest piece holds a majority (the smaller pieces left apart), the second ceiling
// naming the faces, the descent below a shattered ceiling. Without it (the default) the
// candidates are the scored pairs below the threshold that join two pieces the ceiling keeps
// apart -- a graph the ceiling leaves in one piece loses nothing -- and of those only what the
// graph can spare goes: every image keeps at least config.keepPairs pairs and enough of its
// best-scoring pairs to hold config.keepMatches weighted inliers, counting only pairs whose
// ray angle reaches config.keepMinAngle degrees, at the strictest threshold at or below the
// ceiling where at most config.keepMaxShort of the images fall short of that floor from their
// counting pairs that are not candidates at that threshold; a graph that fits no threshold loses
// nothing, and every connected component of the matched graph stays one component, joined by its
// best-scoring candidates.
// A distinct image pair decides once, through its highest-scoring scene pair; duplicates follow it.
unsigned SFM_API FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config,
	const PairsWeightingConfig& weightingCfg, IIndexArr* pSeedViews = NULL);

} // namespace SFM

#endif // _SFM_VIEW_GRAPH_TRIPLETS_H_
