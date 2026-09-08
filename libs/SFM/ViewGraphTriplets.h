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


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

class SFM_API Scene;

// Camera-triplet view-graph disambiguation, after
//   S. M. Manam and V. M. Govindu, "Leveraging Camera Triplets for Efficient and Accurate
//   Structure-from-Motion", CVPR 2024 (Algorithm 1, Eqn. 3).
//
// The view graph has the images as nodes and the geometrically verified pairs as edges, each
// carrying a strength: its weighted inlier count discounted by the fraction of the frame the
// inliers cover. A wrong pair -- two views of look-alike, distinct parts of a scene, which
// retrieval proposes and two-view geometry verifies -- is found from the triangles of the graph
// alone: a true edge is comparable to the strongest edge of every triangle it belongs to, a false
// edge is the weak side of the triangles built around true edges. The edges of the largest
// connected component of the triplet graph (the triangles, adjacent when they share an edge) are
// scored by the mean over their triangles of s_ij / max s_kl, and the paper's threshold
// tau(m) = m (1 - r) + r, with r = d_max / |V| of that component, separates them. The coverage
// discount tells a look-alike pair with many inliers on the repeated object and none elsewhere
// from the true junction beside it, and a triangle whose three pairs all deliver far fewer inliers
// than pairs at their ray angle do in this graph is three look-alikes vouching for one another
// and gives no evidence.
//
// Two modes. In the keep mode, the default, tau(m) is a ceiling that only names the candidates:
// the scored pairs below it joining two pieces (connected components) the ceiling keeps apart.
// Of those, only what the graph can spare goes: every image keeps a floor of pairs and inliers,
// the threshold descends to the strictest one at which at most keepMaxShort of the images fall
// short of that floor, and every component of the matched graph stays one component. A graph in
// one piece at the ceiling, or one fitting no threshold, loses nothing, which is what lets the
// filter run on every scene. The cutting rule (cut) is the paper's: every scored pair below the
// threshold goes. A second, stricter ceiling names the two faces of a symmetric building and cuts
// the smaller face off; otherwise, below a ceiling that shatters the graph, the threshold descends
// to the strictest one joining the pieces again. It unfolds a collection of repeated structure, and it
// halves the registrations of an interior whose rooms it cuts off, so it is not the default.
struct SFM_API TripletFilterConfig
{
	bool enabled = true;          // filter the matched view graph
	bool cut = false;             // the cutting rule instead of the keep mode
	float minScore = 0.3f;        // the paper's minimum edge score m, in [0,1]: 0.3 for medium and small ambiguous sets, 0.6 generic, 0.9 highly ambiguous
	float minYield = 0.4f;        // a triangle whose three pairs all yield below this fraction of the inliers pairs at their ray angle deliver gives no evidence (0: off)
	// keep mode: the floor every image keeps where the graph can spare it, keepPairs pairs holding
	// keepMatches weighted inliers, counting only pairs whose ray angle reaches keepMinAngle degrees
	// (an unmeasured angle counts; a near-duplicate pair yields no 3D point), and the largest share
	// of the images that may fall short of it at the threshold; the threshold descends until that
	// holds, and a graph that fits no threshold loses nothing
	unsigned keepPairs = 3;
	unsigned keepMatches = 2000;
	float keepMinAngle = 3.f;
	float keepMaxShort = 0.5f;
	// cutting rule: the minimum score whose stricter ceiling names the faces, when the graph it
	// leaves has two (a majority piece and a second one of at least a third of it); at or below
	// minScore the faces are not looked for
	float secondFaceScore = 0.75f;
};

// The triplet scores of a matched scene, one per scene pair, and the statistics of the scored graph.
struct SFM_API TripletScores
{
	static constexpr float unscored = -1.f; // no evidence about the pair: kept at every threshold
	std::vector<float> scores;        // per scene.pairs index, in [0,1] or unscored
	float tau;                        // the threshold of Eqn. 3 for the given minimum score
	size_t numTriplets;               // triangles of the view graph
	unsigned numTripletComponents;    // connected components of the triplet graph
	unsigned numDoppelgangerTriplets; // triangles of the scored component that gave no evidence (the yield rule)
	unsigned numScoredPairs;          // scene pairs carrying a score
	unsigned numNodes;                // |V| of the scored graph
	unsigned maxDegree;               // d_max of the scored graph
};

// Score every verified pair of the scene by the triangles of its view graph (Algorithm 1, steps
// 1-9). A pair's strength is its weighted inlier count discounted by the fraction of the frame its
// inliers cover on a gridSize x gridSize grid (PairsWeightingConfig::gridSize, so the discount is
// measured as the pair weighting measures coverage). Only the edges of the largest connected
// component of the triplet graph are scored; every other pair -- unverified, without inliers or
// stored matches, in no triangle, or in a smaller component -- stays unscored. The scores do not
// depend on the minimum score, only tau does.
TripletScores SFM_API ComputeTripletScores(const Scene& scene, float minScore, float minYield, int gridSize);

// Remove the pairs the filter rejects (TripletFilterConfig) and recompute the pair weights when
// anything was removed; a disabled filter is a no-op. pSeedViews, when given, receives the images
// of the largest piece the ceiling leaves, ascending (empty when the filter is off or the graph
// has no edge): the reconstruction chooses its reference view among them, since after the filter
// the seed's side of a symmetric building is the model. Returns the number of removed pairs.
unsigned SFM_API FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config,
	const PairsWeightingConfig& weightingCfg, IIndexArr* pSeedViews = NULL);

} // namespace SFM

#endif // _SFM_VIEW_GRAPH_TRIPLETS_H_
