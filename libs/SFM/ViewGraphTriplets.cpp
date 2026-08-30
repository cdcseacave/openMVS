////////////////////////////////////////////////////////////////////
// ViewGraphTriplets.cpp
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)
//
// Triplet-based view-graph disambiguation, reimplemented from the paper
//   S. M. Manam and V. M. Govindu,
//   "Leveraging Camera Triplets for Efficient and Accurate Structure-from-Motion",
//   CVPR 2024, pp. 4959-4968 (Algorithm 1 and Eqn. 3).
// Written from the paper alone: no code was taken from the authors' MATLAB release or from any
// third-party reimplementation.

#include "Common.h"
#include "ViewGraphTriplets.h"
#include "Scene.h"

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

namespace {

// Union-find over the nodes of the triplet graph G_T (its nodes are the triplets of the view
// graph). Two triplets are adjacent iff they share an edge of the view graph, so the components
// of G_T are obtained without ever materialising its edges: every triplet is merged with the
// first triplet seen on each of its three edges, which links all triplets over that edge.
class TripletUnionFind
{
public:
	explicit TripletUnionFind(size_t numTriplets) : parents(numTriplets) {
		FOREACH(i, parents)
			parents[i] = (uint32_t)i;
	}

	uint32_t Find(uint32_t x) {
		while (parents[x] != x)
			x = parents[x] = parents[parents[x]]; // path halving
		return x;
	}
	void Union(uint32_t a, uint32_t b) {
		const uint32_t ra = Find(a), rb = Find(b);
		if (ra != rb)
			parents[MAXF(ra, rb)] = MINF(ra, rb); // keep the smallest index as root: deterministic components
	}

private:
	std::vector<uint32_t> parents;
};

// One 3-cycle of the view graph, as the indices of its three edges.
struct Triplet {
	uint32_t edges[3];
};

// sentinel for "no such edge / no such triplet / no such component" in the index arrays below
constexpr uint32_t NO_INDEX = (uint32_t)-1;

} // namespace


// F U N C T I O N S ///////////////////////////////////////////////

TripletScores SFM::ComputeTripletScores(const Scene& scene, float minScore)
{
	TripletScores result;
	result.scores.assign(scene.pairs.size(), -1.f);
	result.tau = minScore; // no G_LCT: d_max/|V| is taken as 0, so Eqn. 3 degenerates to tau = m
	result.numTriplets = result.numTripletComponents = result.numScoredPairs = 0;
	result.numNodes = result.maxDegree = 0;
	if (scene.pairs.empty() || scene.images.empty())
		return result;

	// 1. The edges of the view graph G: the geometrically verified pairs carrying at least one
	// inlier. Two scene pairs listing the same image pair collapse onto one edge weighted by the
	// strongest of them, and every such duplicate ends up with that edge's score, so a duplicate
	// can never be kept while its twin is removed.
	const IIndex numImages = scene.images.size();
	std::unordered_map<PairIdx::PairIndex, uint32_t> edgeOfImagePair;
	std::vector<uint32_t> edgeOfScenePair(scene.pairs.size(), NO_INDEX);
	std::vector<PairIdx> edgeImages;   // the two image indices of each edge (i < j)
	std::vector<unsigned> edgeInliers; // n_ij
	edgeOfImagePair.reserve(scene.pairs.size());
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		ASSERT(pair.ID1 < numImages && pair.ID2 < numImages, "ComputeTripletScores: pair references an unknown image");
		const unsigned numInliers = pair.GetNumFilteredInliers();
		if (!pair.HasGeometricVerification() || numInliers == 0)
			continue;
		const PairIdx imagePair(MakePairIdx(pair.ID1, pair.ID2));
		const auto inserted = edgeOfImagePair.emplace(imagePair.idx, (uint32_t)edgeImages.size());
		if (inserted.second) {
			edgeImages.emplace_back(imagePair);
			edgeInliers.emplace_back(numInliers);
		} else if (numInliers > edgeInliers[inserted.first->second]) {
			edgeInliers[inserted.first->second] = numInliers;
		}
		edgeOfScenePair[idxPair] = inserted.first->second;
	}
	const uint32_t numEdges = (uint32_t)edgeImages.size();
	if (numEdges == 0)
		return result;

	// 2. Sorted adjacency of G, each neighbour carrying the index of the edge reaching it, so
	// intersecting two adjacency lists yields the triangle *and* its three edges in one walk.
	struct Neighbor {
		IIndex image;
		uint32_t edge;
		inline bool operator<(const Neighbor& r) const { return image < r.image; }
	};
	std::vector<std::vector<Neighbor>> adjacency(numImages);
	for (uint32_t e = 0; e < numEdges; ++e) {
		adjacency[edgeImages[e].i].emplace_back(Neighbor{edgeImages[e].j, e});
		adjacency[edgeImages[e].j].emplace_back(Neighbor{edgeImages[e].i, e});
	}
	for (std::vector<Neighbor>& neighbors : adjacency)
		std::sort(neighbors.begin(), neighbors.end());

	// 3. Enumerate the triplets of G: for every edge (i,j) with i < j, the common neighbours
	// k > j of i and j. The i < j < k ordering visits each triangle exactly once, from its
	// lexicographically smallest edge. This pass is serial: on the densest capture measured here
	// (377 images, 6241 pairs) it costs ~2 ms, far below the cost of the surrounding matching.
	std::vector<Triplet> triplets;
	for (uint32_t e = 0; e < numEdges; ++e) {
		const IIndex i = edgeImages[e].i, j = edgeImages[e].j;
		ASSERT(i < j, "ComputeTripletScores: edge images are not ordered");
		const std::vector<Neighbor>& adjI = adjacency[i];
		const std::vector<Neighbor>& adjJ = adjacency[j];
		size_t a = 0, b = 0;
		while (a < adjI.size() && b < adjJ.size()) {
			if (adjI[a].image < adjJ[b].image) {
				++a;
			} else if (adjJ[b].image < adjI[a].image) {
				++b;
			} else {
				if (adjI[a].image > j)
					triplets.emplace_back(Triplet{{e, adjI[a].edge, adjJ[b].edge}});
				++a; ++b;
			}
		}
	}
	result.numTriplets = (unsigned)triplets.size();
	if (triplets.empty())
		return result;

	// 4. Components of the triplet graph G_T, and the largest of them (Algorithm 1 step 2).
	TripletUnionFind components(triplets.size());
	std::vector<uint32_t> firstTripletOfEdge(numEdges, NO_INDEX);
	FOREACH(t, triplets) {
		for (uint32_t e : triplets[t].edges) {
			if (firstTripletOfEdge[e] == NO_INDEX)
				firstTripletOfEdge[e] = (uint32_t)t;
			else
				components.Union((uint32_t)t, firstTripletOfEdge[e]);
		}
	}
	std::unordered_map<uint32_t, unsigned> sizeOfComponent;
	FOREACH(t, triplets)
		++sizeOfComponent[components.Find((uint32_t)t)];
	result.numTripletComponents = (unsigned)sizeOfComponent.size();
	uint32_t largestComponent = NO_INDEX;
	unsigned largestComponentSize = 0;
	for (const auto& component : sizeOfComponent) {
		// ties break on the smaller root, which is the triplet found first: deterministic
		if (component.second > largestComponentSize ||
			(component.second == largestComponentSize && component.first < largestComponent)) {
			largestComponent = component.first;
			largestComponentSize = component.second;
		}
	}

	// 5. Score the edges of G_LCT (Algorithm 1 steps 4-8): one pass over the triplets of the
	// largest component accumulates both the per-triplet maximum n_kl and the per-edge running
	// mean of q^t_ij = n_ij / max_{(k,l) in t} n_kl.
	std::vector<double> scoreSumOfEdge(numEdges, 0.0);
	std::vector<unsigned> numTripletsOfEdge(numEdges, 0);
	FOREACH(t, triplets) {
		if (components.Find((uint32_t)t) != largestComponent)
			continue;
		const Triplet& triplet = triplets[t];
		const unsigned maxInliers = MAXF3(edgeInliers[triplet.edges[0]],
			edgeInliers[triplet.edges[1]], edgeInliers[triplet.edges[2]]);
		ASSERT(maxInliers > 0, "ComputeTripletScores: triplet with no inliers");
		for (uint32_t e : triplet.edges) {
			scoreSumOfEdge[e] += (double)edgeInliers[e] / (double)maxInliers;
			++numTripletsOfEdge[e];
		}
	}

	// 6. |V| and d_max of G_LCT, and the adaptive threshold of Eqn. 3 (ruling R-F2: both are
	// taken on G_LCT, the graph whose edges carry a score).
	std::unordered_map<IIndex, unsigned> degreeOfNode;
	for (uint32_t e = 0; e < numEdges; ++e) {
		if (numTripletsOfEdge[e] == 0)
			continue;
		result.maxDegree = MAXF(result.maxDegree, ++degreeOfNode[edgeImages[e].i]);
		result.maxDegree = MAXF(result.maxDegree, ++degreeOfNode[edgeImages[e].j]);
	}
	result.numNodes = (unsigned)degreeOfNode.size();
	if (result.numNodes > 0) {
		const float degreeRatio = (float)result.maxDegree / (float)result.numNodes;
		result.tau = minScore * (1.f - degreeRatio) + degreeRatio;
	}

	// 7. Spread the edge scores back onto the scene pairs
	FOREACH(idxPair, scene.pairs) {
		const uint32_t e = edgeOfScenePair[idxPair];
		if (e == NO_INDEX || numTripletsOfEdge[e] == 0)
			continue;
		result.scores[idxPair] = (float)(scoreSumOfEdge[e] / numTripletsOfEdge[e]);
		++result.numScoredPairs;
	}
	return result;
}
/*----------------------------------------------------------------*/

unsigned SFM::FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config, const PairsWeightingConfig& weightingCfg)
{
	if (!config.enabled)
		return 0;
	TD_TIMER_STARTD();
	const TripletScores tripletScores = ComputeTripletScores(scene, config.minScore);
	const unsigned numPairs = scene.pairs.size();
	unsigned numUnscored = 0, numBelowTau = 0;
	for (unsigned idxPair = 0, idxKept = 0; idxPair < numPairs; ++idxPair) {
		const float score = tripletScores.scores[idxPair];
		if (score >= tripletScores.tau) {
			++idxKept;
			continue;
		}
		if (score < 0.f)
			++numUnscored;
		else
			++numBelowTau;
		scene.pairs.RemoveAtMove(idxKept);
	}
	const unsigned numRemoved = numUnscored + numBelowTau;
	VERBOSE("Triplet filter: kept %u/%u pairs (tau %.3f from m %.2f; %u nodes, max degree %u; "
		"%u triplets in %u components; %u pairs unscored, %u below tau)",
		numPairs - numRemoved, numPairs, tripletScores.tau, config.minScore,
		tripletScores.numNodes, tripletScores.maxDegree,
		tripletScores.numTriplets, tripletScores.numTripletComponents, numUnscored, numBelowTau);
	// the connectivity and cycle-consistency weights were computed on the unfiltered graph and
	// the composite-weight order the reconstruction consumes is stale after the removals
	ComputePairsWeights(scene, weightingCfg);
	DEBUG("Filtered the view graph by camera triplets: %u pairs removed (%s)", numRemoved, TD_TIMER_GET_FMT().c_str());
	return numRemoved;
}
/*----------------------------------------------------------------*/
