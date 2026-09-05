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

#include <cmath>
#include <unordered_map>
#include <unordered_set>
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
		const unsigned numInliers = pair.GetNumWeightedInliers(); // n_ij: the pair's evidence, dense included
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

	// 6. |V| and d_max of G_LCT, and the adaptive threshold of Eqn. 3. Both quantities are taken
	// on G_LCT -- the largest connected component of the triplet graph, the same graph whose
	// edges carry a score -- rather than on the whole view graph.
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

SurvivorGraph SFM::EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau)
{
	SurvivorGraph result{0, 0, 0, 0};
	ASSERT(scores.size() == scene.pairs.size(), "EvaluateSurvivorGraph: one score per scene pair");
	const IIndex numImages = scene.images.size();
	if (numImages == 0)
		return result;
	// degree in the kept graph, and membership of the unfiltered one, in a single pass
	std::vector<unsigned> degree(numImages, 0);
	std::vector<bool> isNode(numImages, false);
	std::vector<uint32_t> parent(numImages);
	FOREACH(i, parent)
		parent[i] = (uint32_t)i;
	const auto Find = [&](uint32_t x) {
		while (parent[x] != x)
			x = parent[x] = parent[parent[x]];
		return x;
	};
	// Two scene pairs can describe the same image pair (ComputeTripletScores collapses those onto
	// one scored edge); this walk must agree with that or the two functions describing the same
	// graph would disagree about its degrees and edge count.
	std::unordered_set<PairIdx::PairIndex> seenEdges;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!pair.HasGeometricVerification() || pair.GetNumWeightedInliers() == 0 || pair.ID1 == pair.ID2)
			continue;
		isNode[pair.ID1] = isNode[pair.ID2] = true;
		if (!seenEdges.insert(MakePairIdx(pair.ID1, pair.ID2).idx).second)
			continue; // a duplicate of an edge already counted
		const float score = scores[idxPair];
		if (score >= 0.f && score < tau)
			continue; // removed: scored, and below the threshold
		++degree[pair.ID1];
		++degree[pair.ID2];
		++result.numKept;
		const uint32_t a = Find(pair.ID1), b = Find(pair.ID2);
		if (a != b)
			parent[MAXF(a, b)] = MINF(a, b);
	}
	std::unordered_map<uint32_t, unsigned> componentSize;
	for (IIndex i = 0; i < numImages; ++i) {
		if (!isNode[i])
			continue;
		++result.numNodes;
		if (degree[i] < 2)
			++result.numLowDegree;
		result.largestComponent = MAXF(result.largestComponent, ++componentSize[Find((uint32_t)i)]);
	}
	return result;
}
/*----------------------------------------------------------------*/

unsigned SFM::FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config, const PairsWeightingConfig& weightingCfg)
{
	if (!config.enabled)
		return 0;
	TD_TIMER_STARTD();
	// config.minScore is the paper's m and only means anything in [0,1]; CLAMP alone passes a NaN
	// through unclamped, and an out-of-range value would otherwise reach the ladder below as a
	// loop trip count (a stray 1e6 running 2e7 union-find passes over the whole view graph) or an
	// undefined float-to-int cast, so both are guarded here, once, before either happens.
	const float minScore = std::isnan(config.minScore) ? 0.f : CLAMP(config.minScore, 0.f, 1.f);
	const TripletScores tripletScores = ComputeTripletScores(scene, minScore);
	float tau = tripletScores.tau;
	if (config.autoTau) {
		// Eqn. 3 on G_LCT: tau(m) = m (1 - d_max/|V|) + d_max/|V|, monotone in m. The sweep starts
		// at the m that was asked for and only ever relaxes: strictness is a cost, not a virtue.
		// Sweeping UP instead -- taking the strictest threshold that survives the tests below --
		// removes 92% of a well-connected outdoor orbit while passing every one of them.
		//
		// tau can never drop below d_max/|V| however far the sweep relaxes: on a near-complete
		// graph that ratio approaches 1, so relaxation has nothing left to give and the likely
		// outcome is standing down rather than a lower tau -- expected, since a graph with barely
		// any missing edges has no doppelganger structure to find in the first place. On the real
		// graphs this branch has recorded, d_max/|V| sits between 0.21 and 0.33, so relaxation is
		// live there; it is only near-complete graphs where the floor bites.
		const float degreeRatio = tripletScores.numNodes > 0
			? (float)tripletScores.maxDegree / (float)tripletScores.numNodes : 0.f;
		const SurvivorGraph unfiltered = EvaluateSurvivorGraph(scene, tripletScores.scores, 0.f);
		// A candidate is accepted when it fragments nothing, strands nobody and rewrites little.
		//
		// The first two bars are RELATIVE to the unfiltered graph, and the second one has to be: a
		// real capture already has images below degree 2 before any filtering -- 5 of 209 on the
		// measured lidar graph -- so an absolute "under 1% of nodes" bar refuses every threshold,
		// including the one that removes nothing.
		//
		// The third bar is the one connectivity cannot supply. At the paper's own m = 0.6 the
		// filter removes 65-67% of a healthy, fully connected orbit and the first two bars still
		// pass, because the score is relative: on a dense graph the mean of n_ij / max(n_kl) over
		// many triangles sits well below 1 for almost every edge, doppelganger or not. A filter
		// removing two thirds of a graph is not finding outliers, and doppelgangers are a minority
		// by construction. MAX_REMOVED_FRACTION says what the filter is for; it is a policy choice
		// rather than a measured constant, and the log line below reports what was actually
		// removed so a wrong value shows up in the first run rather than being inferred.
		constexpr double MAX_REMOVED_FRACTION = 0.20;
		const unsigned minComponent = (unsigned)std::ceil(0.99 * (double)unfiltered.largestComponent);
		const unsigned maxLowDegree = unfiltered.numLowDegree +
			(unsigned)std::floor(0.01 * (double)unfiltered.numNodes);
		const unsigned minKept = unfiltered.numKept -
			(unsigned)std::floor(MAX_REMOVED_FRACTION * (double)unfiltered.numKept);
		tau = -1.f;
		for (int step = (int)std::floor(minScore / 0.05f); step >= 0; --step) {
			// clamped against minScore on every step, not just conceptually at the first one: a
			// step near the top can reconstruct a value one ULP above minScore by float rounding,
			// and the sweep must never end up stricter than what was asked for
			const float m = MINF((float)step * 0.05f, minScore);
			const float candidate = m * (1.f - degreeRatio) + degreeRatio;
			const SurvivorGraph survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, candidate);
			if (survivor.largestComponent >= minComponent && survivor.numLowDegree <= maxLowDegree &&
				survivor.numKept >= minKept) {
				tau = candidate;
				VERBOSE("Triplet filter: auto-tau relaxed m %.2f to %.2f (tau %.3f); survivor graph "
					"keeps %u/%u images in its largest component, %u below degree 2 (%u before), "
					"and %u/%u distinct image pairs",
					minScore, m, candidate, survivor.largestComponent,
					unfiltered.largestComponent, survivor.numLowDegree, unfiltered.numLowDegree,
					survivor.numKept, unfiltered.numKept);
				break;
			}
		}
		if (tau < 0.f) {
			// A graph where no threshold is safe is a graph this filter has no business touching.
			VERBOSE("Triplet filter: auto-tau found no threshold at or below m %.2f that keeps "
				"%u/%u images connected, at most %u below degree 2 (%u before) and at least %u/%u "
				"distinct image pairs; leaving the view graph alone",
				minScore, minComponent, unfiltered.largestComponent, maxLowDegree,
				unfiltered.numLowDegree, minKept, unfiltered.numKept);
			return 0;
		}
	}
	// compact in one forward pass -- moving every kept pair down and truncating once -- rather
	// than erasing pair by pair: each erase shifts the whole tail, so on a graph where the filter
	// removes most of the edges that would cost O(removed x kept) moves of a match-carrying pair
	const unsigned numPairs = scene.pairs.size();
	unsigned numUnscored = 0, numBelowTau = 0, numKept = 0;
	for (unsigned idxPair = 0; idxPair < numPairs; ++idxPair) {
		const float score = tripletScores.scores[idxPair];
		// An unscored pair is one the method has no evidence about: it takes part in no triangle,
		// or in none inside the largest triplet-graph component. Those are overwhelmingly TRUE
		// pairs -- 426 of 490 and 415 of 441 on the two labelled references -- so absence of
		// evidence keeps the pair. Only a scored pair below tau is removed.
		if (score < 0.f) {
			++numUnscored;
		} else if (score < tau) {
			++numBelowTau;
			continue;
		}
		if (numKept != idxPair)
			scene.pairs[numKept] = std::move(scene.pairs[idxPair]);
		++numKept;
	}
	const unsigned numRemoved = numPairs - numKept;
	ASSERT(numRemoved == numBelowTau, "FilterPairsByTriplets: removal count mismatch");
	if (numRemoved > 0)
		scene.pairs.RemoveLast(numRemoved);
	VERBOSE("Triplet filter: kept %u/%u scene pairs (tau %.3f; %u nodes, max degree %u; "
		"%u triplets in %u components; %u below tau removed, %u unscored kept)",
		numKept, numPairs, tau,
		tripletScores.numNodes, tripletScores.maxDegree,
		tripletScores.numTriplets, tripletScores.numTripletComponents, numBelowTau, numUnscored);
	// the connectivity and cycle-consistency weights were computed on the unfiltered graph and
	// the composite-weight order the reconstruction consumes is stale after the removals; a filter
	// that removed nothing left both intact, so re-running would only cost time
	if (numRemoved > 0)
		ComputePairsWeights(scene, weightingCfg);
	DEBUG("Filtered the view graph by camera triplets: %u pairs removed (%s)", numRemoved, TD_TIMER_GET_FMT().c_str());
	return numRemoved;
}
/*----------------------------------------------------------------*/
