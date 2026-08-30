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

// Union-find over the *edges* of the view graph. The nodes of the triplet graph G_T are the
// triplets and two of them are adjacent iff they share an edge, so unioning the three edges of
// every triplet reproduces exactly the components of G_T -- carried on the edges, and therefore in
// O(|E|) memory instead of the O(#triplets) a materialised triplet list would need. That matters:
// ExportPairsCSV scores the graph on every --export-pairs-csv run, and an exhaustively matched
// 1000-image scene has C(1000,3) ~ 1.7e8 triangles, some 2 GB of triplet records.
class EdgeUnionFind
{
public:
	explicit EdgeUnionFind(size_t numEdges) : parents(numEdges) {
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

// sentinel for "no such edge / no such component" in the index arrays below
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
	// inlier and joining two *different* images (a self-pair would otherwise fabricate triangles
	// out of a single node). Two scene pairs listing the same image pair collapse onto one edge
	// weighted by the strongest of them, and every such duplicate ends up with that edge's score,
	// so a duplicate can never be kept while its twin is removed.
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
		if (!pair.HasGeometricVerification() || numInliers == 0 || pair.ID1 == pair.ID2)
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
	// lexicographically smallest edge. The triplets are *streamed* to the visitor and never
	// stored: the list alone would be Theta(#triplets), which a near-complete graph makes
	// unaffordable, while everything below needs only O(|E|) state. The walk is serial: on the
	// densest capture measured here (377 images, 6241 pairs, 44889 triplets) one pass costs a
	// couple of milliseconds, far below the cost of the surrounding matching.
	const auto ForEachTriplet = [&](const auto& visit) {
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
						visit(e, adjI[a].edge, adjJ[b].edge);
					++a; ++b;
				}
			}
		}
	};

	// 4. Components of the triplet graph G_T, and the largest of them (Algorithm 1 step 2). The
	// first streaming pass unions the three edges of every triplet -- two triplets sharing an edge
	// therefore land in the same edge-component, which is exactly the adjacency G_T is built on --
	// and counts the triplets each edge takes part in.
	EdgeUnionFind components(numEdges);
	std::vector<unsigned> numTripletsOfEdge(numEdges, 0);
	ForEachTriplet([&](uint32_t e0, uint32_t e1, uint32_t e2) {
		components.Union(e0, e1);
		components.Union(e0, e2);
		++numTripletsOfEdge[e0];
		++numTripletsOfEdge[e1];
		++numTripletsOfEdge[e2];
		++result.numTriplets;
	});
	if (result.numTriplets == 0)
		return result;
	// every triplet contributes one to each of its three edges and all three share its component,
	// so a component's triplet count is a third of what its edges carry; the factor is common to
	// every component, so the maximum below is the same either way
	std::unordered_map<uint32_t, unsigned> tripletsOfComponent;
	for (uint32_t e = 0; e < numEdges; ++e)
		if (numTripletsOfEdge[e] > 0)
			tripletsOfComponent[components.Find(e)] += numTripletsOfEdge[e];
	result.numTripletComponents = (unsigned)tripletsOfComponent.size();
	uint32_t largestComponent = NO_INDEX;
	unsigned largestComponentSize = 0;
	for (const auto& component : tripletsOfComponent) {
		// ties break on the smaller root, the first edge of the component: deterministic
		if (component.second > largestComponentSize ||
			(component.second == largestComponentSize && component.first < largestComponent)) {
			largestComponent = component.first;
			largestComponentSize = component.second;
		}
	}
	// an edge of the largest component takes part in no triplet outside it (all three edges of a
	// triplet share one component), so numTripletsOfEdge is already |trp(i,j)|, the mean's divisor
	const auto IsScoredEdge = [&](uint32_t e) {
		return numTripletsOfEdge[e] > 0 && components.Find(e) == largestComponent;
	};

	// 5. Score the edges of G_LCT (Algorithm 1 steps 4-8): a second streaming pass over the
	// triplets of the largest component accumulates the per-triplet maximum n_kl and the per-edge
	// running sum of q^t_ij = n_ij / max_{(k,l) in t} n_kl.
	std::vector<double> scoreSumOfEdge(numEdges, 0.0);
	ForEachTriplet([&](uint32_t e0, uint32_t e1, uint32_t e2) {
		if (components.Find(e0) != largestComponent)
			return;
		const unsigned maxInliers = MAXF3(edgeInliers[e0], edgeInliers[e1], edgeInliers[e2]);
		ASSERT(maxInliers > 0, "ComputeTripletScores: triplet with no inliers");
		scoreSumOfEdge[e0] += (double)edgeInliers[e0] / (double)maxInliers;
		scoreSumOfEdge[e1] += (double)edgeInliers[e1] / (double)maxInliers;
		scoreSumOfEdge[e2] += (double)edgeInliers[e2] / (double)maxInliers;
	});

	// 6. |V| and d_max of G_LCT, and the adaptive threshold of Eqn. 3 (ruling R-F2: both are
	// taken on G_LCT, the graph whose edges carry a score).
	std::unordered_map<IIndex, unsigned> degreeOfNode;
	for (uint32_t e = 0; e < numEdges; ++e) {
		if (!IsScoredEdge(e))
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
		if (e == NO_INDEX || !IsScoredEdge(e))
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
	// compact in one forward pass -- moving every kept pair down and truncating once -- rather
	// than erasing pair by pair: each erase shifts the whole tail, so on a graph where the filter
	// removes most of the edges that would cost O(removed x kept) moves of a match-carrying pair
	const unsigned numPairs = scene.pairs.size();
	unsigned numUnscored = 0, numBelowTau = 0, numKept = 0;
	for (unsigned idxPair = 0; idxPair < numPairs; ++idxPair) {
		const float score = tripletScores.scores[idxPair];
		if (score < tripletScores.tau) {
			if (score < 0.f)
				++numUnscored;
			else
				++numBelowTau;
			continue;
		}
		if (numKept != idxPair)
			scene.pairs[numKept] = std::move(scene.pairs[idxPair]);
		++numKept;
	}
	const unsigned numRemoved = numPairs - numKept;
	ASSERT(numRemoved == numUnscored + numBelowTau, "FilterPairsByTriplets: removal count mismatch");
	if (numRemoved > 0)
		scene.pairs.RemoveLast(numRemoved);
	VERBOSE("Triplet filter: kept %u/%u pairs (tau %.3f from m %.2f; %u nodes, max degree %u; "
		"%u triplets in %u components; %u pairs unscored, %u below tau)",
		numPairs - numRemoved, numPairs, tripletScores.tau, config.minScore,
		tripletScores.numNodes, tripletScores.maxDegree,
		tripletScores.numTriplets, tripletScores.numTripletComponents, numUnscored, numBelowTau);
	// the connectivity and cycle-consistency weights were computed on the unfiltered graph and
	// the composite-weight order the reconstruction consumes is stale after the removals; a filter
	// that removed nothing left both intact, so re-running would only cost time
	if (numRemoved > 0)
		ComputePairsWeights(scene, weightingCfg);
	DEBUG("Filtered the view graph by camera triplets: %u pairs removed (%s)", numRemoved, TD_TIMER_GET_FMT().c_str());
	return numRemoved;
}
/*----------------------------------------------------------------*/
