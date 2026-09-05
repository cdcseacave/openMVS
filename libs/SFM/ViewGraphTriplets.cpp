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
#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <unordered_set>

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

// The yield of every edge: how much of what its two images can deliver the pair delivered,
// against what pairs at its ray angle deliver in this graph. u_e = n_e / min(K_i, K_j), K_i the
// inlier count of image i's strongest edge; the envelope H is the 90th percentile of u over the
// edges of each 1-degree bin of ray angle among bins holding at least five edges, made
// non-increasing in the angle by a suffix maximum (a near-duplicate viewpoint never promises
// less than a wider one), so a bin without an envelope of its own takes the nearest populated
// bin above it and the bins above the highest populated one keep its value; the yield is
// min(1, u_e / H). No populated bin at all -- fewer than five edges everywhere -- means no
// envelope and every yield 1. The percentile, the bin width and the bin floor are properties of
// the estimate, not of the scene: 75, 90 and 95 replay identically on every reference set.
// An edge whose ray angle is not finite or is negative has no measurable geometry: it takes no
// part in any bin's envelope and its own yield is left at the initial 1 -- absence of evidence is
// not evidence of a deficit, and the yield never removes a pair on its own.
static std::vector<float> ComputeEdgeYields(const std::vector<PairIdx>& edgeImages,
	const std::vector<unsigned>& edgeInliers, const std::vector<float>& edgeRayAngle, IIndex numImages)
{
	constexpr unsigned numBins = 90;        // 1-degree bins; 90 degrees and beyond share the last
	constexpr size_t minEdgesPerBin = 5;
	constexpr float percentile = 0.9f;
	const uint32_t numEdges = (uint32_t)edgeImages.size();
	std::vector<float> yields(numEdges, 1.f);
	std::vector<unsigned> capacity(numImages, 0);
	for (uint32_t e = 0; e < numEdges; ++e) {
		capacity[edgeImages[e].i] = MAXF(capacity[edgeImages[e].i], edgeInliers[e]);
		capacity[edgeImages[e].j] = MAXF(capacity[edgeImages[e].j], edgeInliers[e]);
	}
	std::vector<float> delivered(numEdges);
	std::vector<unsigned> binOfEdge(numEdges);
	std::vector<std::vector<float>> bins(numBins);
	for (uint32_t e = 0; e < numEdges; ++e) {
		if (!ISFINITE(edgeRayAngle[e]) || edgeRayAngle[e] < 0.f) {
			binOfEdge[e] = numBins; // sentinel: no measurable geometry, outside every real bin
			continue;
		}
		delivered[e] = (float)edgeInliers[e] / (float)MINF(capacity[edgeImages[e].i], capacity[edgeImages[e].j]);
		binOfEdge[e] = (unsigned)MINF((int)std::floor(R2D(edgeRayAngle[e])), (int)numBins - 1);
		bins[binOfEdge[e]].push_back(delivered[e]);
	}
	std::vector<float> envelope(numBins, -1.f);
	for (unsigned b = 0; b < numBins; ++b) {
		std::vector<float>& values = bins[b];
		if (values.size() < minEdgesPerBin)
			continue;
		const size_t rank = MINF(values.size() - 1, (size_t)((float)values.size() * percentile));
		std::nth_element(values.begin(), values.begin() + rank, values.end());
		envelope[b] = values[rank];
	}
	float best = -1.f;
	for (unsigned b = numBins; b-- > 0; ) {
		best = MAXF(best, envelope[b]);
		envelope[b] = best;
	}
	if (best < 0.f)
		return yields;
	for (unsigned b = 1; b < numBins; ++b)
		if (envelope[b] < 0.f)
			envelope[b] = envelope[b - 1];
	for (uint32_t e = 0; e < numEdges; ++e) {
		if (binOfEdge[e] == numBins)
			continue; // no finite, non-negative ray angle: yield stays at the initial 1
		yields[e] = MINF(1.f, delivered[e] / envelope[binOfEdge[e]]);
	}
	return yields;
}
// (an edge with no finite, non-negative ray angle is guarded into the numBins sentinel above and
// never reaches the std::floor/(int) cast, so every remaining binOfEdge is a finite, non-negative
// angle's floor and the cast to int is safe; envelope values are percentiles of delivered, which
// is in (0,1], so the division is safe once best is non-negative.)

TripletScores SFM::ComputeTripletScores(const Scene& scene, float minScore, float minYield, int gridSize)
{
	TripletScores result;
	result.scores.assign(scene.pairs.size(), -1.f);
	result.tau = minScore; // no G_LCT: d_max/|V| is taken as 0, so Eqn. 3 degenerates to tau = m
	result.numTriplets = result.numTripletComponents = result.numDoppelgangerTriplets = result.numScoredPairs = 0;
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
	std::vector<PairIdx> edgeImages;    // the two image indices of each edge (i < j)
	std::vector<float> edgeStrength;    // s_ij = n_ij * c_ij
	std::vector<unsigned> edgeInliers;  // n_ij of the scene pair that supplied the strength
	std::vector<float> edgeRayAngle;    // its median ray angle, radians
	edgeOfImagePair.reserve(scene.pairs.size());
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		ASSERT(pair.ID1 < numImages && pair.ID2 < numImages, "ComputeTripletScores: pair references an unknown image");
		const unsigned numInliers = pair.GetNumWeightedInliers(); // n_ij: the pair's evidence, dense included
		if (!pair.HasGeometricVerification() || numInliers == 0 || pair.ID1 == pair.ID2)
			continue;
		// s_ij: the count discounted by the fraction of the frame the inliers cover. A verified
		// pair whose matches are not stored has nothing to measure coverage on and is no edge --
		// EvaluateSurvivorGraph still counts it, so it ends up unscored and kept, which is what
		// "no evidence" means here.
		const float strength = (float)numInliers *
			ComputePairCoverage(pair, scene.images[pair.ID1], scene.images[pair.ID2], gridSize);
		if (strength <= 0.f)
			continue;
		const PairIdx imagePair(MakePairIdx(pair.ID1, pair.ID2));
		const auto inserted = edgeOfImagePair.emplace(imagePair.idx, (uint32_t)edgeImages.size());
		if (inserted.second) {
			edgeImages.emplace_back(imagePair);
			edgeStrength.emplace_back(strength);
			edgeInliers.emplace_back(numInliers);
			edgeRayAngle.emplace_back(pair.meanRayAngle);
		} else if (strength > edgeStrength[inserted.first->second]) {
			edgeStrength[inserted.first->second] = strength;
			edgeInliers[inserted.first->second] = numInliers;
			edgeRayAngle[inserted.first->second] = pair.meanRayAngle;
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
	// triplets of the largest component accumulates the per-triplet maximum s_kl and the per-edge
	// running sum of q^t_ij = s_ij / max_{(k,l) in t} s_kl. A triplet whose three edges all yield
	// below minYield is look-alike copies vouching for one another: it stays in the divisor
	// (numTripletsOfEdge) and adds nothing to the sum.
	const std::vector<float> yields = minYield > 0.f
		? ComputeEdgeYields(edgeImages, edgeInliers, edgeRayAngle, numImages) : std::vector<float>();
	std::vector<double> scoreSumOfEdge(numEdges, 0.0);
	ForEachTriplet([&](uint32_t e0, uint32_t e1, uint32_t e2) {
		if (components.Find(e0) != largestComponent)
			return;
		if (minYield > 0.f && MAXF3(yields[e0], yields[e1], yields[e2]) < minYield) {
			++result.numDoppelgangerTriplets;
			return;
		}
		const float maxStrength = MAXF3(edgeStrength[e0], edgeStrength[e1], edgeStrength[e2]);
		ASSERT(maxStrength > 0.f, "ComputeTripletScores: triplet with no strength");
		scoreSumOfEdge[e0] += (double)edgeStrength[e0] / (double)maxStrength;
		scoreSumOfEdge[e1] += (double)edgeStrength[e1] / (double)maxStrength;
		scoreSumOfEdge[e2] += (double)edgeStrength[e2] / (double)maxStrength;
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

SurvivorGraph SFM::EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau, unsigned minPiece)
{
	SurvivorGraph result{0, 0, 0, 0, 0, 0, 0};
	ASSERT(scores.size() == scene.pairs.size(), "EvaluateSurvivorGraph: one score per scene pair");
	const IIndex numImages = scene.images.size();
	if (numImages == 0)
		return result;
	// degree in the kept graph, and membership of the unfiltered one, in a single pass
	std::vector<unsigned> degree(numImages, 0);
	std::vector<bool> isNode(numImages, false);
	std::vector<uint32_t> parent(numImages), parentUnfiltered(numImages);
	FOREACH(i, parent) {
		parent[i] = (uint32_t)i;
		parentUnfiltered[i] = (uint32_t)i;
	}
	const auto Find = [](std::vector<uint32_t>& p, uint32_t x) {
		while (p[x] != x)
			x = p[x] = p[p[x]];
		return x;
	};
	// Two scene pairs can describe the same image pair (ComputeTripletScores collapses those onto
	// one scored edge); this walk must agree with that or the two functions describing the same
	// graph would disagree about its degrees and edge count.
	std::unordered_set<PairIdx::PairIndex> seenEdges;
	seenEdges.reserve(scene.pairs.size());
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!pair.HasGeometricVerification() || pair.GetNumWeightedInliers() == 0 || pair.ID1 == pair.ID2)
			continue;
		isNode[pair.ID1] = isNode[pair.ID2] = true;
		if (!seenEdges.insert(MakePairIdx(pair.ID1, pair.ID2).idx).second)
			continue; // a duplicate of an edge already counted
		// The unfiltered graph's own connectivity, independent of tau: an island the ceiling never
		// touches -- a verified pair sharing no edge path with the rest, at any score -- is fixed
		// here, before any threshold is applied, and stays an island at every tau below.
		const uint32_t ua = Find(parentUnfiltered, pair.ID1), ub = Find(parentUnfiltered, pair.ID2);
		if (ua != ub)
			parentUnfiltered[MAXF(ua, ub)] = MINF(ua, ub);
		const float score = scores[idxPair];
		if (score >= 0.f && score < tau)
			continue; // removed: scored, and below the threshold
		++degree[pair.ID1];
		++degree[pair.ID2];
		++result.numKept;
		const uint32_t a = Find(parent, pair.ID1), b = Find(parent, pair.ID2);
		if (a != b)
			parent[MAXF(a, b)] = MINF(a, b);
	}
	// The unfiltered graph's own largest component: a piece can only ever be joined to another
	// piece it already shares this component with -- an island of the unfiltered graph never
	// gains an edge to anything as tau falls, so it can never be joined to anything either.
	std::unordered_map<uint32_t, unsigned> unfilteredComponentSize;
	for (IIndex i = 0; i < numImages; ++i) {
		if (!isNode[i])
			continue;
		++unfilteredComponentSize[Find(parentUnfiltered, (uint32_t)i)];
	}
	uint32_t largestUnfilteredComponent = NO_INDEX;
	unsigned largestUnfilteredComponentSize = 0;
	for (const auto& component : unfilteredComponentSize) {
		// ties break on the smaller root, the first edge of the component: deterministic, the same
		// tie-break ComputeTripletScores uses for its own largest-component choice
		if (component.second > largestUnfilteredComponentSize ||
			(component.second == largestUnfilteredComponentSize && component.first < largestUnfilteredComponent)) {
			largestUnfilteredComponent = component.first;
			largestUnfilteredComponentSize = component.second;
		}
	}
	std::unordered_map<uint32_t, unsigned> componentSize;
	for (IIndex i = 0; i < numImages; ++i) {
		if (!isNode[i])
			continue;
		++result.numNodes;
		if (degree[i] < 2)
			++result.numLowDegree;
		result.largestComponent = MAXF(result.largestComponent, ++componentSize[Find(parent, (uint32_t)i)]);
	}
	for (const auto& component : componentSize) {
		if (component.second < minPiece)
			continue;
		// A kept component is entirely inside one unfiltered component (kept edges are a subset of
		// unfiltered ones), so its root -- itself a node -- tells which one; a component of a
		// different unfiltered component is an island's, never joinable to the rest, and not a piece.
		if (Find(parentUnfiltered, component.first) != largestUnfilteredComponent)
			continue;
		++result.numPieces;
		result.numInPieces += component.second;
		result.largestPiece = MAXF(result.largestPiece, component.second);
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
	// through unclamped, and Eqn. 3 would otherwise turn an out-of-range m into a ceiling outside
	// [0,1] too, so both are guarded here, once, before ComputeTripletScores uses it.
	const float minScore = std::isnan(config.minScore) ? 0.f : CLAMP(config.minScore, 0.f, 1.f);
	if (minScore != config.minScore)
		VERBOSE("warning: triplet filter: minimum edge score %g is outside [0,1], using %g", config.minScore, minScore);
	const float minYield = std::isnan(config.minYield) ? 0.f : CLAMP(config.minYield, 0.f, 1.f);
	if (minYield != config.minYield)
		VERBOSE("warning: triplet filter: minimum yield %g is outside [0,1], using %g", config.minYield, minYield);
	const TripletScores tripletScores = ComputeTripletScores(scene, minScore, minYield, weightingCfg.gridSize);
	const float degreeRatio = tripletScores.numNodes > 0
		? (float)tripletScores.maxDegree / (float)tripletScores.numNodes : 0.f;
	// Eqn. 3 on G_LCT, tau(m) = m (1 - d_max/|V|) + d_max/|V|, is the CEILING: the filter is never
	// stricter than the m it was given. On a complete view graph -- every pair verified, which is
	// what identical facades produce under exhaustive matching -- d_max/|V| is (|V|-1)/|V| and the
	// ceiling sits at 0.95-0.99 whatever m is; on the sparse graphs of video captures it is 0.7 or
	// below.
	const float ceiling = tripletScores.tau;
	float tau = ceiling;
	if (config.autoTau) {
		// Below the ceiling, the threshold is the STRICTEST one that joins every piece: the largest
		// value whose survivor graph holds, in one component, every image that the ceiling's
		// pieces hold together. A piece is a component of the survivor graph at the ceiling with
		// at least 1% of the unfiltered largest component; anything smaller is a straggler -- an
		// image the graph vouches for through a single weak pair -- and fetching it would admit
		// every edge between the ceiling and that pair's score to gain one image (church: from
		// 0.72 to 0.43 for twenty such images). Stragglers are neither chased nor removed: an
		// unscored pair still carries them, and so does a bridge above the chosen threshold.
		// Everything scored below the threshold is either a weak true pair the pieces do not
		// need or a doppelganger, and nothing in the inlier counts tells the two apart -- on the
		// ambiguous-scene datasets the doppelganger pairs OUTSCORE the true low-overlap pairs --
		// so the only defensible cut keeps the strong edges and exactly enough of them. On the
		// small sets, where every component is a piece, that removes 66-96% of the pairs and
		// leaves a chain's two endpoints at degree 1, which is why there is no bar on how much is
		// removed and none on low-degree images.
		const SurvivorGraph unfiltered = EvaluateSurvivorGraph(scene, tripletScores.scores, 0.f);
		const unsigned minPiece = (unsigned)std::ceil(0.01 * (double)unfiltered.largestComponent);
		SurvivorGraph survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, ceiling, minPiece);
		const unsigned minComponent = survivor.numInPieces;
		const unsigned numPieces = survivor.numPieces;
		const unsigned numStragglers = survivor.numNodes - survivor.numInPieces;
		// The descent repairs a ceiling that shattered the graph -- the small sets and every
		// exhaustively matched collection, where d_max/|V| is near 1 and the ceiling leaves
		// fragments of a few images each. A ceiling whose largest piece already holds a strict
		// majority of the images in pieces has done the paper's job: what hangs below it is a
		// straggler or the other face of a symmetric building (the church matched exhaustively
		// splits into its two facades at the ceiling, 143 and 85 images, and one 253-inlier pair
		// at 0.931 would join them), and nothing in the scores tells the two apart, so the
		// ceiling is applied as given and the smaller pieces stay apart. A strict majority, so a
		// graph cut into two equal halves is still repaired.
		const bool shattered = 2 * survivor.largestPiece <= survivor.numInPieces;
		if (shattered && survivor.largestComponent < minComponent) {
			// The largest component only grows as tau falls, so among the distinct scores below
			// the ceiling, strictest first, the first that passes is a binary search away. The
			// loosest candidate keeps every scored pair -- the unfiltered graph itself, whose
			// largest component holds every piece -- so it always passes, and the ceiling
			// leaving a piece apart means at least one scored pair sits below it.
			std::vector<float> candidates;
			candidates.reserve(tripletScores.numScoredPairs);
			for (float score : tripletScores.scores)
				if (score >= 0.f && score < ceiling)
					candidates.push_back(score);
			std::sort(candidates.begin(), candidates.end(), std::greater<float>());
			candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
			ASSERT(!candidates.empty(), "FilterPairsByTriplets: the ceiling leaves a piece apart with no score below it");
			size_t lo = 0, hi = candidates.size() - 1;
			while (lo < hi) {
				const size_t mid = (lo + hi) / 2;
				if (EvaluateSurvivorGraph(scene, tripletScores.scores, candidates[mid]).largestComponent >= minComponent)
					hi = mid;
				else
					lo = mid + 1;
			}
			tau = candidates[lo];
			survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, tau);
		}
		VERBOSE("Triplet filter: tau %.3f, %s (ceiling %.3f at m %.2f, d_max/|V| %.3f); the ceiling leaves "
			"%u pieces (components of at least %u images) holding %u images between them, and %u stragglers; "
			"survivor graph keeps %u/%u images in its largest component, %u below degree 2 (%u before), "
			"and %u/%u distinct image pairs",
			tau, tau < ceiling ? "the strictest threshold that joins every piece" :
				numPieces > 1 && !shattered ? "the ceiling applied as given, its largest piece holding a majority" :
				"the ceiling applied as given",
			ceiling, minScore, degreeRatio, numPieces, minPiece, minComponent, numStragglers,
			survivor.largestComponent, unfiltered.largestComponent,
			survivor.numLowDegree, unfiltered.numLowDegree, survivor.numKept, unfiltered.numKept);
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
		"%u triplets in %u components, %u doppelganger triplets gave no evidence; %u below tau removed, %u unscored kept)",
		numKept, numPairs, tau,
		tripletScores.numNodes, tripletScores.maxDegree,
		tripletScores.numTriplets, tripletScores.numTripletComponents, tripletScores.numDoppelgangerTriplets,
		numBelowTau, numUnscored);
	// the connectivity and cycle-consistency weights were computed on the unfiltered graph and
	// the composite-weight order the reconstruction consumes is stale after the removals; a filter
	// that removed nothing left both intact, so re-running would only cost time
	if (numRemoved > 0)
		ComputePairsWeights(scene, weightingCfg);
	DEBUG("Filtered the view graph by camera triplets: %u pairs removed (%s)", numRemoved, TD_TIMER_GET_FMT().c_str());
	return numRemoved;
}
/*----------------------------------------------------------------*/
