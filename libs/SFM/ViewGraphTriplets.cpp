////////////////////////////////////////////////////////////////////
// ViewGraphTriplets.cpp
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)
//
// Camera-triplet view-graph disambiguation after S. M. Manam and V. M. Govindu, "Leveraging
// Camera Triplets for Efficient and Accurate Structure-from-Motion", CVPR 2024 (Algorithm 1 and
// Eqn. 3), written from the paper alone.

#include "Common.h"
#include "ViewGraphTriplets.h"
#include "Scene.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

namespace {

constexpr uint32_t NO_INDEX = (uint32_t)-1;
constexpr float CUT = -2.f; // the score of a pair the face rule removes at every threshold

// Union-find with path halving; the root of a set is its smallest member, so the components are
// deterministic and a component's root belongs to it.
class UnionFind
{
public:
	explicit UnionFind(size_t size) : parent(size) {
		FOREACH(i, parent)
			parent[i] = (uint32_t)i;
	}
	uint32_t Find(uint32_t x) {
		while (parent[x] != x)
			x = parent[x] = parent[parent[x]];
		return x;
	}
	// join the sets of a and b; false when they were one already
	bool Join(uint32_t a, uint32_t b) {
		a = Find(a);
		b = Find(b);
		if (a == b)
			return false;
		parent[MAXF(a, b)] = MINF(a, b);
		return true;
	}
private:
	std::vector<uint32_t> parent;
};

// The view graph: the images as nodes, the geometrically verified pairs holding at least one
// inlier as edges, one edge per distinct image pair. Two scene pairs of the same images (the
// matcher never produces them, a scene file may hold them) share one edge, described by the
// stronger of them, and share its fate.
struct ViewGraph
{
	struct Edge {
		IIndex i, j;         // i < j
		float strength;      // weighted inliers x coverage; 0 when no matches are stored (never scored, always kept)
		unsigned numInliers; // weighted inliers
		float rayAngle;      // median ray angle of the track-forming matches, radians (0: not measured)
	};
	IIndex numImages;
	std::vector<Edge> edges;
	std::vector<uint32_t> edgeOfPair; // per scene pair: its edge, NO_INDEX when the pair is no edge
	std::vector<bool> isNode;         // per image: incident to an edge
	std::vector<bool> inMain;         // per image: in the largest connected component
	unsigned numNodes, mainSize;
};

ViewGraph BuildViewGraph(const Scene& scene, int gridSize)
{
	ViewGraph g;
	g.numImages = scene.images.size();
	g.edgeOfPair.assign(scene.pairs.size(), NO_INDEX);
	g.isNode.assign(g.numImages, false);
	std::unordered_map<PairIdx::PairIndex, uint32_t> edgeOfImages;
	edgeOfImages.reserve(scene.pairs.size());
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		ASSERT(pair.ID1 < g.numImages && pair.ID2 < g.numImages, "BuildViewGraph: pair references an unknown image");
		const unsigned numInliers = pair.GetNumWeightedInliers();
		if (!pair.HasGeometricVerification() || numInliers == 0 || pair.ID1 == pair.ID2)
			continue;
		const PairIdx images(MakePairIdx(pair.ID1, pair.ID2));
		const float strength = (float)numInliers *
			ComputePairCoverage(pair, scene.images[pair.ID1], scene.images[pair.ID2], gridSize);
		const auto inserted = edgeOfImages.emplace(images.idx, (uint32_t)g.edges.size());
		if (inserted.second) {
			g.edges.push_back({images.i, images.j, strength, numInliers, pair.meanRayAngle});
			g.isNode[images.i] = g.isNode[images.j] = true;
		} else {
			ViewGraph::Edge& edge = g.edges[inserted.first->second];
			if (strength > edge.strength || (strength == edge.strength && numInliers > edge.numInliers))
				edge = {images.i, images.j, strength, numInliers, pair.meanRayAngle};
		}
		g.edgeOfPair[idxPair] = inserted.first->second;
	}
	// the largest connected component, ties to the one holding the lowest image
	UnionFind uf(g.numImages);
	for (const ViewGraph::Edge& edge : g.edges)
		uf.Join(edge.i, edge.j);
	std::unordered_map<uint32_t, unsigned> sizes;
	g.numNodes = 0;
	for (IIndex i = 0; i < g.numImages; ++i) {
		if (!g.isNode[i])
			continue;
		++g.numNodes;
		++sizes[uf.Find(i)];
	}
	uint32_t mainRoot = NO_INDEX;
	g.mainSize = 0;
	for (const auto& [root, size] : sizes) {
		if (size > g.mainSize || (size == g.mainSize && root < mainRoot)) {
			mainRoot = root;
			g.mainSize = size;
		}
	}
	g.inMain.assign(g.numImages, false);
	for (IIndex i = 0; i < g.numImages; ++i)
		g.inMain[i] = g.isNode[i] && uf.Find(i) == mainRoot;
	return g;
}

// The yield of every edge: the inliers it delivered against what pairs at its ray angle deliver
// in this graph. u_e = n_e / min(K_i, K_j), with K_i the inlier count of image i's strongest
// edge; the envelope H is the 90th percentile of u over the edges of each 1-degree bin of ray
// angle (bins holding at least five edges), made non-increasing in the angle by a suffix maximum
// (a narrower viewpoint never promises less than a wider one), so an empty bin takes the nearest
// populated bin above it and the bins above the highest populated one keep its value; the yield
// is min(1, u_e / H). An edge without a measured ray angle takes no part in the envelope and
// yields 1: absence of evidence is not evidence of a deficit. With no populated bin every edge
// yields 1.
std::vector<float> ComputeYields(const ViewGraph& g)
{
	constexpr unsigned numBins = 90; // 1-degree bins; 90 degrees and beyond share the last
	constexpr size_t minEdgesPerBin = 5;
	constexpr float percentile = 0.9f;
	const uint32_t numEdges = (uint32_t)g.edges.size();
	std::vector<float> yields(numEdges, 1.f);
	std::vector<unsigned> capacity(g.numImages, 0);
	for (const ViewGraph::Edge& edge : g.edges) {
		if (edge.strength <= 0.f)
			continue;
		capacity[edge.i] = MAXF(capacity[edge.i], edge.numInliers);
		capacity[edge.j] = MAXF(capacity[edge.j], edge.numInliers);
	}
	std::vector<float> delivered(numEdges, 0.f);
	std::vector<unsigned> binOfEdge(numEdges, numBins); // numBins: no measured geometry
	std::vector<std::vector<float>> bins(numBins);
	for (uint32_t e = 0; e < numEdges; ++e) {
		const ViewGraph::Edge& edge = g.edges[e];
		if (edge.strength <= 0.f || !ISFINITE(edge.rayAngle) || edge.rayAngle <= 0.f)
			continue;
		delivered[e] = (float)edge.numInliers / (float)MINF(capacity[edge.i], capacity[edge.j]);
		binOfEdge[e] = MINF((unsigned)std::floor(R2D(edge.rayAngle)), numBins - 1);
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
	for (unsigned b = numBins; b-- > 0; )
		envelope[b] = best = MAXF(best, envelope[b]);
	if (best < 0.f)
		return yields;
	for (unsigned b = 1; b < numBins; ++b)
		if (envelope[b] < 0.f)
			envelope[b] = envelope[b - 1];
	for (uint32_t e = 0; e < numEdges; ++e)
		if (binOfEdge[e] < numBins)
			yields[e] = MINF(1.f, delivered[e] / envelope[binOfEdge[e]]);
	return yields;
}

// The triplet scores of the edges and the statistics of the scored graph.
struct EdgeScores
{
	std::vector<float> scores; // per edge, in [0,1] or TripletScores::unscored
	float tau;
	size_t numTriplets;
	unsigned numComponents, numDoppelganger, numNodes, maxDegree;
};

// Algorithm 1, steps 1-9: the triangles of the graph form the triplet graph, whose largest
// connected component is the scored graph; each of its edges scores the mean over its triangles
// of s_ij / max_{(k,l) in t} s_kl, a triangle whose three edges all yield below minYield giving
// nothing while still counting in the mean; tau follows from |V| and d_max of the scored graph.
// The triangles are streamed and never stored: everything here is O(|E|) memory, where a dense
// 1000-image graph has some 1.7e8 triangles.
EdgeScores ScoreEdges(const ViewGraph& g, float minScore, float minYield)
{
	const uint32_t numEdges = (uint32_t)g.edges.size();
	EdgeScores result;
	result.scores.assign(numEdges, TripletScores::unscored);
	result.tau = minScore; // no scored graph: d_max/|V| is 0
	result.numTriplets = 0;
	result.numComponents = result.numDoppelganger = result.numNodes = result.maxDegree = 0;

	// sorted adjacency of the edges with a strength, each neighbour carrying its edge, so
	// intersecting two lists yields a triangle together with its three edges
	struct Neighbor {
		IIndex image;
		uint32_t edge;
		inline bool operator<(const Neighbor& r) const { return image < r.image; }
	};
	std::vector<std::vector<Neighbor>> adjacency(g.numImages);
	for (uint32_t e = 0; e < numEdges; ++e) {
		const ViewGraph::Edge& edge = g.edges[e];
		if (edge.strength <= 0.f)
			continue;
		adjacency[edge.i].push_back({edge.j, e});
		adjacency[edge.j].push_back({edge.i, e});
	}
	for (std::vector<Neighbor>& neighbors : adjacency)
		std::sort(neighbors.begin(), neighbors.end());
	// every triangle once, from its edge (i,j) with i < j through the common neighbours k > j
	const auto ForEachTriplet = [&](const auto& visit) {
		for (uint32_t e = 0; e < numEdges; ++e) {
			const ViewGraph::Edge& edge = g.edges[e];
			if (edge.strength <= 0.f)
				continue;
			const std::vector<Neighbor>& adjI = adjacency[edge.i];
			const std::vector<Neighbor>& adjJ = adjacency[edge.j];
			for (size_t a = 0, b = 0; a < adjI.size() && b < adjJ.size(); ) {
				if (adjI[a].image < adjJ[b].image) {
					++a;
				} else if (adjJ[b].image < adjI[a].image) {
					++b;
				} else {
					if (adjI[a].image > edge.j)
						visit(e, adjI[a].edge, adjJ[b].edge);
					++a;
					++b;
				}
			}
		}
	};

	// the components of the triplet graph, carried on the edges (the three edges of a triangle
	// share one component), and the triangles each edge takes part in
	UnionFind components(numEdges);
	std::vector<unsigned> numTripletsOfEdge(numEdges, 0);
	ForEachTriplet([&](uint32_t e0, uint32_t e1, uint32_t e2) {
		components.Join(e0, e1);
		components.Join(e0, e2);
		++numTripletsOfEdge[e0];
		++numTripletsOfEdge[e1];
		++numTripletsOfEdge[e2];
		++result.numTriplets;
	});
	if (result.numTriplets == 0)
		return result;
	// the largest component by triangles (each counted on its three edges), ties to the smallest root
	std::unordered_map<uint32_t, size_t> tripletsOfComponent;
	for (uint32_t e = 0; e < numEdges; ++e)
		if (numTripletsOfEdge[e] > 0)
			tripletsOfComponent[components.Find(e)] += numTripletsOfEdge[e];
	result.numComponents = (unsigned)tripletsOfComponent.size();
	uint32_t largest = NO_INDEX;
	size_t largestSize = 0;
	for (const auto& [root, size] : tripletsOfComponent) {
		if (size > largestSize || (size == largestSize && root < largest)) {
			largest = root;
			largestSize = size;
		}
	}

	// the scores
	const std::vector<float> yields = minYield > 0.f ? ComputeYields(g) : std::vector<float>();
	std::vector<double> sums(numEdges, 0.0);
	ForEachTriplet([&](uint32_t e0, uint32_t e1, uint32_t e2) {
		if (components.Find(e0) != largest)
			return;
		if (minYield > 0.f && MAXF3(yields[e0], yields[e1], yields[e2]) < minYield) {
			++result.numDoppelganger;
			return;
		}
		const double maxStrength = MAXF3(g.edges[e0].strength, g.edges[e1].strength, g.edges[e2].strength);
		sums[e0] += g.edges[e0].strength / maxStrength;
		sums[e1] += g.edges[e1].strength / maxStrength;
		sums[e2] += g.edges[e2].strength / maxStrength;
	});
	std::unordered_map<IIndex, unsigned> degree;
	for (uint32_t e = 0; e < numEdges; ++e) {
		if (numTripletsOfEdge[e] == 0 || components.Find(e) != largest)
			continue;
		result.scores[e] = (float)(sums[e] / numTripletsOfEdge[e]);
		result.maxDegree = MAXF(result.maxDegree, ++degree[g.edges[e].i]);
		result.maxDegree = MAXF(result.maxDegree, ++degree[g.edges[e].j]);
	}
	result.numNodes = (unsigned)degree.size();
	if (result.numNodes > 0) {
		const float degreeRatio = (float)result.maxDegree / (float)result.numNodes;
		result.tau = minScore * (1.f - degreeRatio) + degreeRatio;
	}
	return result;
}

// an edge kept at a threshold: unscored, or scoring at or above it; a cut edge never
inline bool IsKept(float score, float tau)
{
	return score == TripletScores::unscored || (score >= 0.f && score >= tau);
}

// The root of every node's connected component in the graph kept at tau (NO_INDEX for an image
// that is no node).
std::vector<uint32_t> ComponentsAt(const ViewGraph& g, const std::vector<float>& scores, float tau)
{
	UnionFind uf(g.numImages);
	FOREACH(e, g.edges)
		if (IsKept(scores[e], tau))
			uf.Join(g.edges[e].i, g.edges[e].j);
	std::vector<uint32_t> root(g.numImages, NO_INDEX);
	for (IIndex i = 0; i < g.numImages; ++i)
		if (g.isNode[i])
			root[i] = uf.Find(i);
	return root;
}

// the given images all lie in one component
bool Joined(const std::vector<uint32_t>& root, const IIndexArr& views)
{
	FOREACH(i, views)
		if (root[views[i]] != root[views[0]])
			return false;
	return true;
}

// The pieces of the graph kept at a threshold: its connected components of at least minPiece
// nodes inside the largest component of the unfiltered graph (an island of the unfiltered graph
// never gains an edge to anything at any threshold, so it is never a piece to join).
struct Pieces
{
	std::vector<uint32_t> root; // per image: its component (ComponentsAt)
	IIndexArr roots;            // the pieces' roots, ascending
	unsigned numNodes;          // nodes in pieces
	IIndexArr largest, second;  // the images of the largest and the second-largest piece, ascending
};

Pieces PiecesAt(const ViewGraph& g, const std::vector<float>& scores, float tau, unsigned minPiece)
{
	Pieces p;
	p.root = ComponentsAt(g, scores, tau);
	p.numNodes = 0;
	std::unordered_map<uint32_t, unsigned> sizes;
	for (IIndex i = 0; i < g.numImages; ++i)
		if (g.inMain[i])
			++sizes[p.root[i]];
	// ties to the piece holding the lowest image, its root
	uint32_t root1 = NO_INDEX, root2 = NO_INDEX;
	unsigned size1 = 0, size2 = 0;
	for (const auto& [root, size] : sizes) {
		if (size < minPiece)
			continue;
		p.roots.push_back((IIndex)root);
		p.numNodes += size;
		if (size > size1 || (size == size1 && root < root1)) {
			root2 = root1;
			size2 = size1;
			root1 = root;
			size1 = size;
		} else if (size > size2 || (size == size2 && root < root2)) {
			root2 = root;
			size2 = size;
		}
	}
	std::sort(p.roots.begin(), p.roots.end());
	for (IIndex i = 0; i < g.numImages; ++i) {
		if (p.root[i] == NO_INDEX)
			continue;
		if (p.root[i] == root1)
			p.largest.push_back(i);
		else if (p.root[i] == root2)
			p.second.push_back(i);
	}
	return p;
}

// the distinct scores below the ceiling, strictest first: the thresholds a descent can stop at
std::vector<float> ThresholdsBelow(const std::vector<float>& scores, float ceiling)
{
	std::vector<float> thresholds;
	for (float score : scores)
		if (score >= 0.f && score < ceiling)
			thresholds.push_back(score);
	std::sort(thresholds.begin(), thresholds.end(), std::greater<float>());
	thresholds.erase(std::unique(thresholds.begin(), thresholds.end()), thresholds.end());
	return thresholds;
}

// the strictest of the thresholds (ordered strictest first) that passes a monotone test: one that
// passes at a threshold passes at every looser one; the loosest is known to pass
template <typename Test>
float StrictestPassing(const std::vector<float>& thresholds, const Test& passes)
{
	size_t lo = 0, hi = thresholds.size() - 1;
	while (lo < hi) {
		const size_t mid = (lo + hi) / 2;
		if (passes(thresholds[mid]))
			hi = mid;
		else
			lo = mid + 1;
	}
	return thresholds[lo];
}

// The score of every scene pair: its edge's, TripletScores::unscored for a pair that is no edge or
// whose edge is unscored
std::vector<float> ScoresOfPairs(const ViewGraph& g, const EdgeScores& edgeScores)
{
	std::vector<float> scores(g.edgeOfPair.size(), TripletScores::unscored);
	FOREACH(idxPair, scores) {
		const uint32_t e = g.edgeOfPair[idxPair];
		if (e != NO_INDEX && edgeScores.scores[e] >= 0.f)
			scores[idxPair] = edgeScores.scores[e];
	}
	return scores;
}

// a value that only means anything in [0,1], guarded once (CLAMP passes a NaN through)
float Unit(float value, const char* name)
{
	const float unit = std::isnan(value) ? 0.f : CLAMP(value, 0.f, 1.f);
	if (unit != value)
		VERBOSE("warning: triplet filter: %s %g is outside [0,1], using %g", name, value, unit);
	return unit;
}

} // namespace


// F U N C T I O N S ///////////////////////////////////////////////

TripletScores SFM::ComputeTripletScores(const Scene& scene, float minScore, float minYield, int gridSize)
{
	const ViewGraph graph = BuildViewGraph(scene, gridSize);
	const EdgeScores edgeScores = ScoreEdges(graph, minScore, minYield);
	TripletScores result;
	result.scores = ScoresOfPairs(graph, edgeScores);
	result.numScoredPairs = 0;
	for (const float score : result.scores)
		if (score != TripletScores::unscored)
			++result.numScoredPairs;
	result.tau = edgeScores.tau;
	result.numTriplets = edgeScores.numTriplets;
	result.numTripletComponents = edgeScores.numComponents;
	result.numDoppelgangerTriplets = edgeScores.numDoppelganger;
	result.numNodes = edgeScores.numNodes;
	result.maxDegree = edgeScores.maxDegree;
	return result;
}
/*----------------------------------------------------------------*/

unsigned SFM::FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config, const PairsWeightingConfig& weightingCfg,
	IIndexArr* pSeedViews, std::vector<float>* pScores)
{
	if (pSeedViews)
		pSeedViews->Empty();
	if (pScores)
		pScores->assign(scene.pairs.size(), TripletScores::unscored);
	if (!config.enabled)
		return 0;
	TD_TIMER_STARTD();
	const float minScore = Unit(config.minScore, "minimum edge score");
	const float minYield = Unit(config.minYield, "minimum yield");
	const float secondFace = Unit(config.secondFaceScore, "second-face score");

	const ViewGraph graph = BuildViewGraph(scene, weightingCfg.gridSize);
	const EdgeScores edgeScores = ScoreEdges(graph, minScore, minYield);
	const uint32_t numEdges = (uint32_t)graph.edges.size();
	if (numEdges == 0) {
		VERBOSE("Triplet filter: no verified pairs, nothing to do");
		return 0;
	}
	std::vector<float> scores = edgeScores.scores; // the face rule marks the edges it cuts
	const float degreeRatio = edgeScores.numNodes > 0 ? (float)edgeScores.maxDegree / (float)edgeScores.numNodes : 0.f;
	const float ceiling = edgeScores.tau; // tau(m): the filter is never stricter than the paper's threshold
	float tau = ceiling;
	// a piece is a component of at least 1% of the largest component; a ceiling leaving none at
	// all -- a large collection shattered into pairs -- is the graph most in need of repair, and
	// every component is then a piece, as on a small set
	unsigned minPiece = (unsigned)std::ceil(0.01 * (double)graph.mainSize);
	std::vector<bool> removed(numEdges, false);
	unsigned numCut = 0;

	// The cutting rule, part one: a stricter ceiling names the faces. When the graph it leaves
	// has two -- a majority piece and a second piece holding at least a third of it -- the
	// paper's ceiling applies inside the larger face and the other face is cut off: every pair
	// joining it to an image outside it goes, and so does every pair of an ambiguous image, one
	// outside both faces whose kept pairs at the paper's ceiling reach both.
	bool twoFaced = false;
	if (config.cut && secondFace > minScore) {
		const float secondCeiling = secondFace * (1.f - degreeRatio) + degreeRatio;
		const Pieces faces = PiecesAt(graph, scores, secondCeiling, minPiece);
		twoFaced = 2 * faces.largest.size() > faces.numNodes && 3 * faces.second.size() >= faces.largest.size();
		if (twoFaced) {
			enum : uint8_t { NO_FACE = 0, FACE_A = 1, FACE_B = 2 };
			std::vector<uint8_t> face(graph.numImages, NO_FACE), touches(graph.numImages, NO_FACE);
			for (IIndex i : faces.largest)
				face[i] = FACE_A;
			for (IIndex i : faces.second)
				face[i] = FACE_B;
			for (uint32_t e = 0; e < numEdges; ++e) {
				const ViewGraph::Edge& edge = graph.edges[e];
				if (!IsKept(scores[e], ceiling))
					continue;
				if (face[edge.i] == NO_FACE)
					touches[edge.i] |= face[edge.j];
				if (face[edge.j] == NO_FACE)
					touches[edge.j] |= face[edge.i];
			}
			std::vector<bool> ambiguous(graph.numImages);
			unsigned numAmbiguous = 0;
			for (IIndex i = 0; i < graph.numImages; ++i)
				if ((ambiguous[i] = face[i] == NO_FACE && touches[i] == (FACE_A | FACE_B)))
					++numAmbiguous;
			for (uint32_t e = 0; e < numEdges; ++e) {
				const ViewGraph::Edge& edge = graph.edges[e];
				if ((face[edge.i] == FACE_B) != (face[edge.j] == FACE_B) || ambiguous[edge.i] || ambiguous[edge.j]) {
					scores[e] = CUT;
					++numCut;
				}
			}
			VERBOSE("Triplet filter: the ceiling at the second-face score %.2f (%.3f) leaves two faces of %u and %u images: "
				"%u distinct pairs joining the smaller face or one of %u ambiguous images cut off",
				secondFace, secondCeiling, (unsigned)faces.largest.size(), (unsigned)faces.second.size(), numCut, numAmbiguous);
		} else {
			VERBOSE("Triplet filter: the ceiling at the second-face score %.2f (%.3f) leaves no second face (largest piece %u, second %u)",
				secondFace, secondCeiling, (unsigned)faces.largest.size(), (unsigned)faces.second.size());
		}
	}

	// The pieces the ceiling leaves; the reconstruction seeds in the largest of them, whatever is
	// joined to it below: the resection refuses the look-alike bridges the descent lets through
	// but cannot choose the side it starts on, and the heaviest image overall sits in the densest
	// cluster of look-alike views.
	Pieces atCeiling = PiecesAt(graph, scores, ceiling, minPiece);
	if (atCeiling.roots.empty() && minPiece > 1) {
		minPiece = 1;
		atCeiling = PiecesAt(graph, scores, ceiling, minPiece);
	}
	if (pSeedViews)
		*pSeedViews = atCeiling.largest;

	if (config.cut) {
		// The cutting rule, part two: a ceiling whose largest piece holds a strict majority of the
		// pieced images has done the paper's job, and what hangs below it -- a straggler, or the
		// other face of a symmetric building -- stays apart. A ceiling that shattered the graph
		// (small sets, and every exhaustively matched collection, where d_max/|V| is near 1)
		// descends to the strictest threshold that joins every piece again; stragglers, the
		// components too small to be pieces, are neither chased nor removed. With the faces
		// named there is nothing to join, and the descent does not run.
		const bool shattered = 2 * atCeiling.largest.size() <= atCeiling.numNodes;
		if (shattered && atCeiling.roots.size() > 1 && !twoFaced) {
			const std::vector<float> thresholds = ThresholdsBelow(scores, ceiling);
			if (!thresholds.empty())
				tau = StrictestPassing(thresholds, [&](float threshold) {
					return Joined(ComponentsAt(graph, scores, threshold), atCeiling.roots);
				});
		}
		for (uint32_t e = 0; e < numEdges; ++e)
			removed[e] = scores[e] == CUT || (scores[e] >= 0.f && scores[e] < tau);
		VERBOSE("Triplet filter: cutting rule, tau %.3f (%s); the ceiling %.3f (m %.2f, d_max/|V| %.3f) leaves %u pieces of at least %u images "
			"holding %u of %u nodes",
			tau, tau < ceiling ? "the strictest threshold joining every piece" :
				twoFaced ? "the ceiling inside the larger face" : "the ceiling",
			ceiling, minScore, degreeRatio, (unsigned)atCeiling.roots.size(), minPiece, atCeiling.numNodes, graph.numNodes);
	}

	// remove the scene pairs of the removed edges, compacting in one pass
	const unsigned numPairs = scene.pairs.size();
	unsigned numKept = 0;
	for (unsigned idxPair = 0; idxPair < numPairs; ++idxPair) {
		const uint32_t e = graph.edgeOfPair[idxPair];
		if (e != NO_INDEX && removed[e])
			continue;
		if (numKept != idxPair)
			scene.pairs[numKept] = std::move(scene.pairs[idxPair]);
		++numKept;
	}
	const unsigned numRemoved = numPairs - numKept;
	if (numRemoved > 0)
		scene.pairs.RemoveLast(numRemoved);
	VERBOSE("Triplet filter: kept %u/%u pairs (%zu triangles in %u components, %u without evidence; scored graph of %u nodes, "
		"max degree %u); the reconstruction seeds in the largest piece the ceiling leaves (%u images)",
		numKept, numPairs, edgeScores.numTriplets, edgeScores.numComponents, edgeScores.numDoppelganger,
		edgeScores.numNodes, edgeScores.maxDegree, (unsigned)atCeiling.largest.size());
	// the connectivity and cycle-consistency weights describe the unfiltered graph
	if (numRemoved > 0)
		ComputePairsWeights(scene, weightingCfg);
	// the scores of the pairs left: those computed above when nothing was removed, else the scores of the
	// graph the removal left (a triangle lost changes the score of the pairs beside it)
	if (pScores)
		*pScores = numRemoved == 0 ? ScoresOfPairs(graph, edgeScores) :
			ComputeTripletScores(scene, minScore, minYield, weightingCfg.gridSize).scores;
	DEBUG("Filtered the view graph by camera triplets: %u pairs removed (%s)", numRemoved, TD_TIMER_GET_FMT().c_str());
	return numRemoved;
}
/*----------------------------------------------------------------*/
