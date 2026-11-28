/*
 * SceneCluster.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "SceneCluster.h"
#include "Scene.h"
#include "Image.h"
#include "ImagePair.h"
#include "../Math/GeodeticTransforms.h"
#include <Eigen/Eigenvalues>

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

namespace {

// Analyze a cluster's internal connectivity and weakest prefix cut along its Fiedler order.
struct ClusterCoupling {
	unsigned larger = 0, smaller = 0; // sizes of the two sides of the weakest cut
	double coupling = 1.0;            // its interface weight / the weaker side's internal weight
	double cutWeight = 0.0;           // its interface weight
	double smallerInternal = 0.0;     // the internal weight of its smaller side
	bool connected = true;            // internal graph in one piece
	bool weak = false;                // the weakest cut is under the coupling floor
	std::vector<uint8_t> side;        // 1 marks the smaller side of the weakest cut, 0 the rest (filled when connected and a cut qualifies)
};

ClusterCoupling AnalyzeClusterCoupling(
	unsigned numImages,
	const std::vector<std::pair<uint32_t, uint32_t>>& edges,
	const std::vector<double>& weights,
	unsigned minSide, float minCoupling)
{
	ClusterCoupling r;
	if (numImages < 2 || edges.empty())
		return r;

	// connectivity — a disconnected final cluster is itself a clustering fault
	DisjointSet<uint32_t> ds(numImages);
	for (const auto& e : edges)
		ds.Union(e.first, e.second);
	const std::unordered_map<uint32_t, unsigned> compSizes = ds.CompressAllPaths().GetComponentSizes();
	if (compSizes.size() > 1) {
		unsigned s1 = 0, s2 = 0;
		for (const auto& [root, size] : compSizes) {
			if (size > s1) { s2 = s1; s1 = size; }
			else if (size > s2) s2 = size;
		}
		r.larger = s1; r.smaller = s2;
		r.coupling = 0.0;
		r.connected = false;
		r.weak = true;
		return r;
	}

	// Fiedler (second-smallest eigenvector) of the weighted Laplacian
	Eigen::MatrixXd L = Eigen::MatrixXd::Zero(numImages, numImages);
	for (size_t k = 0; k < edges.size(); ++k) {
		const uint32_t u = edges[k].first, v = edges[k].second;
		const double w = weights[k];
		L(u, u) += w; L(v, v) += w;
		L(u, v) -= w; L(v, u) -= w;
	}
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(L);
	const Eigen::VectorXd fiedler = es.eigenvectors().col(1);

	// the images in Fiedler order (ties by index), and every prefix cut weighed as it grows: an image
	// moving into the prefix turns its edges to the prefix internal and its edges to the rest into cut
	std::vector<uint32_t> order(numImages);
	std::iota(order.begin(), order.end(), 0u);
	std::stable_sort(order.begin(), order.end(), [&fiedler](uint32_t a, uint32_t b) {
		return fiedler[(Eigen::Index)a] < fiedler[(Eigen::Index)b];
	});
	std::vector<std::vector<std::pair<uint32_t, double>>> adjacency(numImages);
	double internalRest = 0.0;
	for (size_t k = 0; k < edges.size(); ++k) {
		adjacency[edges[k].first].emplace_back(edges[k].second, weights[k]);
		adjacency[edges[k].second].emplace_back(edges[k].first, weights[k]);
		internalRest += weights[k];
	}
	std::vector<uint8_t> inPrefix(numImages, 0);
	double internalPrefix = 0.0, cut = 0.0;
	unsigned bestK = 0;
	double best = std::numeric_limits<double>::max(), bestCut = 0.0;
	for (unsigned k = 1; k < numImages; ++k) {
		const uint32_t u = order[k - 1];
		for (const auto& [v, w] : adjacency[u]) {
			if (inPrefix[v]) {
				internalPrefix += w;
				cut -= w;
			} else {
				cut += w;
				internalRest -= w;
			}
		}
		inPrefix[u] = 1;
		if (MINF(k, numImages - k) < minSide)
			continue;
		const double weaker = MINF(internalPrefix, internalRest);
		if (weaker <= 0.0)
			continue; // a side without internal weight has no cohesion to measure the cut against
		if (cut/weaker < best) {
			best = cut/weaker;
			bestK = k;
			bestCut = cut;
		}
	}
	if (bestK == 0)
		return r; // no cut leaves minSide images with internal weight on both sides
	const bool prefixSmaller = bestK <= numImages - bestK;
	r.side.assign(numImages, 0);
	for (unsigned k = 0; k < numImages; ++k)
		r.side[order[k]] = (k < bestK) == prefixSmaller ? 1 : 0;
	for (size_t k = 0; k < edges.size(); ++k)
		if (r.side[edges[k].first] && r.side[edges[k].second])
			r.smallerInternal += weights[k];
	r.smaller = MINF(bestK, numImages - bestK);
	r.larger = numImages - r.smaller;
	r.coupling = best;
	r.cutWeight = bestCut;
	r.weak = best < (double)minCoupling;
	return r;
}

// Bucket the internal covisibility edges of every cluster (endpoints remapped to the
// cluster-local indices) in one sweep over the global pair graph, for the coupling
// analysis; clusterOf maps an image to its cluster (-1 for none) and localIndex to its
// position in it. Pairs below minPairWeight are skipped, matching the graph every
// consumer of the coupling invariant sees: BuildConnectivityGraph drops them before
// clustering and BuildTracks forms no tracks from them, so they constrain nothing.
// scene.pairs still holds every pair here — intra-cluster pairs are moved into the
// blocks only later, in SceneBlocks::Extract.
void BucketClusterEdges(const Scene& scene, float minPairWeight,
	const std::vector<int>& clusterOf, const std::vector<uint32_t>& localIndex,
	std::vector<std::vector<std::pair<uint32_t, uint32_t>>>& edges, std::vector<std::vector<double>>& weights)
{
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.ExceedsWeightBar(minPairWeight))
			continue;
		const float weight(pair.GetCompositeWeight());
		const int c = clusterOf[pair.ID1];
		if (c < 0 || c != clusterOf[pair.ID2])
			continue;
		edges[c].emplace_back(localIndex[pair.ID1], localIndex[pair.ID2]);
		weights[c].push_back((double)weight);
	}
}

// Run AnalyzeClusterCoupling on every cluster that will become a sub-scene, numbered exactly
// as SceneBlocks::Extract numbers them (isSubScene decides which ones, a standalone
// community among them), so a flag here lines up with the merge-time telemetry of the same
// sub-scene. This is the health check for the two-scale defect: the merge stage aligns the
// sub-scenes to each other but cannot tell whether one of them reconstructed at two scales, so
// the invariant that no sub-scene holds two blocks joined by a seam too sparse to observe their
// relative scale is verified here, structurally, on the graph that clustering produced.
void ReportClusterCoupling(const Scene& scene, const std::vector<IIndexArr>& clusters, const ClusterConfig& config,
	const std::vector<bool>& isSubScene)
{
	std::vector<int> clusterOf(scene.images.size(), -1);
	std::vector<uint32_t> localIndex(scene.images.size(), 0);
	std::vector<unsigned> clusterSize;
	int subSceneIdx = 0;
	for (size_t c = 0; c < clusters.size(); ++c) {
		if (!isSubScene[c])
			continue;
		const IIndexArr& cluster = clusters[c];
		uint32_t li = 0;
		for (IIndex g : cluster) {
			clusterOf[g] = subSceneIdx;
			localIndex[g] = li++;
		}
		clusterSize.push_back((unsigned)cluster.size());
		++subSceneIdx;
	}
	if (subSceneIdx == 0)
		return;
	const unsigned numKept = (unsigned)subSceneIdx;
	std::vector<std::vector<std::pair<uint32_t, uint32_t>>> edges(numKept);
	std::vector<std::vector<double>> weights(numKept);
	BucketClusterEdges(scene, config.minPairWeight, clusterOf, localIndex, edges, weights);
	unsigned numWeak = 0;
	for (unsigned s = 0; s < numKept; ++s) {
		const unsigned V = clusterSize[s];
		const ClusterCoupling cc = AnalyzeClusterCoupling(V, edges[s], weights[s], config.minWaistSideViews, config.minClusterCoupling);
		if (!cc.connected) {
			VERBOSE("warning: Sub-scene %u internally disconnected at creation: components %u+%u images; clustering produced a split sub-scene", s, cc.larger, cc.smaller);
			++numWeak;
		} else if (cc.weak) {
			VERBOSE("warning: Sub-scene %u under-coupled at creation: blocks %u+%u images joined at coupling %.3f < %.3f; it may reconstruct at two scales, which the merge cannot repair", s, cc.larger, cc.smaller, cc.coupling, config.minClusterCoupling);
			++numWeak;
		} else {
			VERBOSE("Sub-scene %u coupling ok: %u images, weakest prefix cut coupling %.3f (%u+%u)", s, V, cc.coupling, cc.larger, cc.smaller);
		}
	}
	VERBOSE("Cluster coupling check: %u/%u sub-scenes flagged weak at creation", numWeak, numKept);
}

// Map every image to the cluster holding it (-1 for an image no cluster holds)
std::vector<int> MapNodesToClusters(const std::vector<IIndexArr>& clusters, IIndex nViews)
{
	std::vector<int> nodeToCluster(nViews, -1);
	for (size_t c = 0; c < clusters.size(); ++c)
		for (IIndex u : clusters[c])
			nodeToCluster[u] = (int)c;
	return nodeToCluster;
}

// Append every image of one cluster to another and leave the source empty: the single step both
// merge passes take, followed by EraseEmptyClusters once the pass is done. A caller that keeps an
// image-to-cluster map passes it in to have it follow the move.
void AbsorbCluster(std::vector<IIndexArr>& clusters, size_t from, size_t into, std::vector<int>* nodeToCluster = NULL)
{
	ASSERT(from != into);
	for (IIndex u : clusters[from]) {
		if (nodeToCluster)
			(*nodeToCluster)[u] = (int)into;
		clusters[into].push_back(u);
	}
	clusters[from].clear();
}

// Drop the clusters an absorb emptied
void EraseEmptyClusters(std::vector<IIndexArr>& clusters)
{
	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [](const IIndexArr& c) {
		return c.empty();
	}), clusters.end());
}

} // namespace

bool SFM::IsStrongSeam(const std::pair<SeamTrackStats, SeamTrackStats>& seam, const ClusterConfig& config)
{
	return seam.first.usable >= config.minSeamTracks &&
		seam.first.CamerasWithAtLeast(config.minSeamCameraTracks) >= config.minSeamCameras &&
		seam.second.CamerasWithAtLeast(config.minSeamCameraTracks) >= config.minSeamCameras;
}

unsigned SeamTrackStats::CamerasWithAtLeast(unsigned n) const
{
	unsigned numCameras = 0;
	for (const auto& [image, numTracks] : perCamera)
		if (numTracks >= n)
			++numCameras;
	return numCameras;
}

SceneCluster::SceneCluster(Scene& scene, const ClusterConfig& config)
	: scene(scene), config(config)
{
}

ScenePartition SceneCluster::Partition()
{
	const IIndex nViews = scene.images.size();
	if (nViews == 0) {
		return {};
	}
	if (config.maxViewsPerCluster == 0 || nViews <= config.maxViewsPerCluster) {
		// No need to split - a single cluster of every image
		DEBUG("Scene has %u images, no clustering needed", nViews);
		ScenePartition partition;
		partition.clusters.emplace_back(nViews);
		FOREACH(i, partition.clusters[0])
			partition.clusters[0][i] = i;
		return partition;
	}

	// the seams are measured on the tracks, so a scene that arrives with only matches gets them
	// here; every sub-scene rebuilds its own tracks after the split
	if (scene.tracks.empty())
		BuildTracks(scene, config.minPairWeight, config.trackConflictCfg);

	BuildConnectivityGraph();

	return PartitionAggregativeClustering();
}

void SceneCluster::BuildConnectivityGraph()
{
	const IIndex nViews = scene.images.size();
	xadj.assign(nViews + 1, 0);

	std::vector<int> degrees(nViews, 0);
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.ExceedsWeightBar(config.minPairWeight))
			continue;
		degrees[pair.ID1]++;
		degrees[pair.ID2]++;
	}
	for (IIndex i = 0; i < nViews; ++i) {
		xadj[i+1] = xadj[i] + degrees[i];
	}

	adjncy.assign(xadj.back(), 0);
	adjwgt.assign(xadj.back(), 0.f);
	std::vector<int> offsets = xadj;

	for (const ImagePair& pair : scene.pairs) {
		if (!pair.ExceedsWeightBar(config.minPairWeight))
			continue;
		const float w = pair.GetCompositeWeight();

		int idx1 = offsets[pair.ID1]++;
		adjncy[idx1] = pair.ID2;
		adjwgt[idx1] = w;

		int idx2 = offsets[pair.ID2]++;
		adjncy[idx2] = pair.ID1;
		adjwgt[idx2] = w;
	}
	VERBOSE("Built connectivity graph: %u nodes, %u edges", (unsigned)nViews, (unsigned)(adjncy.size() / 2));
}

ScenePartition SceneCluster::PartitionAggregativeClustering()
{
	const IIndex nViews = scene.images.size();
	std::vector<IIndexArr> clusters(nViews);
	for (IIndex i = 0; i < nViews; ++i)
		clusters[i].push_back(i);

	GreedyMergeClusters(clusters);

	RefineClustersLocalSearch(clusters);
	MergeSmallClusters(clusters);
	RefineClustersBalance(clusters);
	RefineClustersSplitDisconnected(clusters);
	RefineClustersSplitThinWaist(clusters);
	RefineClustersForSeams(clusters);

	return FinishPartition(clusters);
}

// Bottom-up greedy max-weight clustering under capacity and minimum-coupling constraints.
void SceneCluster::GreedyMergeClusters(std::vector<IIndexArr>& clusters)
{
	const IIndex nViews = scene.images.size();
	std::vector<int> nodeToCluster = MapNodesToClusters(clusters, nViews);

	struct Edge {
		int u, v;
		double weight;
		bool operator<(const Edge& other) const {
			if (weight != other.weight) return weight < other.weight;
			return u != other.u ? u > other.u : v > other.v;
		}
	};
	std::priority_queue<Edge> pq;
	std::vector<std::unordered_map<int, double>> adj(clusters.size());
	std::vector<double> wint(clusters.size(), 0.0);

	auto rebuildPQ = [&]() {
		pq = std::priority_queue<Edge>();
		adj.assign(clusters.size(), {});
		wint.assign(clusters.size(), 0.0);
		for (IIndex u = 0; u < nViews; ++u) {
			const int cu = nodeToCluster[u];
			if (cu < 0) continue;
			for (int i = xadj[u]; i < xadj[u+1]; ++i) {
				const int cv = nodeToCluster[adjncy[i]];
				if (cv < 0) continue;
				if (cu < cv)
					adj[cu][cv] += adjwgt[i];
				else if (cu == cv)
					wint[cu] += adjwgt[i];
			}
		}
		for (size_t i = 0; i < clusters.size(); ++i) {
			wint[i] *= 0.5;
			if (clusters[i].empty()) continue;
			for (const auto& p : adj[i]) {
				adj[p.first][(int)i] = p.second;
				pq.push({(int)i, p.first, p.second});
			}
		}
	};

	rebuildPQ();

	unsigned numMergesSinceRefine = 0;
	while (!pq.empty()) {
		const Edge e = pq.top();
		pq.pop();

		const int u = e.u;
		const int v = e.v;
		if (clusters[u].empty() || clusters[v].empty()) continue;
		const auto it = adj[u].find(v);
		if (it == adj[u].end() || it->second != e.weight) continue;

		if (clusters[u].size() + clusters[v].size() > config.maxViewsPerCluster) continue;
		if (clusters[u].size() + clusters[v].size() > config.targetViewsPerCluster &&
			(MINF(clusters[u].size(), clusters[v].size()) >= config.minViewsPerCluster ||
			 MAXF(clusters[u].size(), clusters[v].size()) > config.targetViewsPerCluster)) continue;
		if (config.minClusterCoupling > 0 &&
			e.weight < (double)config.minClusterCoupling * MINF(wint[u], wint[v]))
			continue;

		AbsorbCluster(clusters, (size_t)v, (size_t)u, &nodeToCluster);
		wint[u] += wint[v] + e.weight;
		wint[v] = 0.0;
		numMergesSinceRefine++;

		adj[u].erase(v);
		for (const auto& p : adj[v]) {
			const int nxt = p.first;
			if (nxt == u) continue;
			adj[nxt].erase(v);
			adj[u][nxt] += p.second;
			adj[nxt][u] = adj[u][nxt];
			pq.push({u, nxt, adj[u][nxt]});
		}
		adj[v].clear();

		const unsigned mergesPerRefine = MAXF(10u, config.maxViewsPerCluster / 10);
		if (numMergesSinceRefine >= mergesPerRefine) {
			RefineClustersLocalSearch(clusters);
			nodeToCluster = MapNodesToClusters(clusters, nViews);
			adj.resize(clusters.size());
			rebuildPQ();
			numMergesSinceRefine = 0;
		}
	}
}

// True if cluster c is under the floor, weakly coupled to all neighbours, yet has enough strong
// merge seams to stand alone as its own sub-scene.
bool SceneCluster::IsStandaloneCommunity(const std::vector<IIndexArr>& clusters, size_t c,
	const std::vector<int>& nodeToCluster, const SeamTrackStatsMap& seamStats) const
{
	ASSERT(c < clusters.size() && !clusters[c].empty());
	if (config.minClusterCoupling <= 0.f || clusters[c].size() >= config.minViewsPerCluster)
		return false;
	double internal = 0.0;
	std::unordered_map<int, double> borderWeight;
	for (IIndex u : clusters[c]) {
		for (int i = xadj[u]; i < xadj[u+1]; ++i) {
			const int cv = nodeToCluster[adjncy[i]];
			if (cv == (int)c)
				internal += adjwgt[i];
			else if (cv >= 0)
				borderWeight[cv] += adjwgt[i];
		}
	}
	internal *= 0.5;
	double heaviest = 0.0;
	for (const auto& p : borderWeight)
		heaviest = MAXF(heaviest, p.second);
	if (heaviest >= (double)config.minClusterCoupling * internal)
		return false;
	unsigned numStrong = 0, numAdjacent = 0;
	for (const auto& [pair, seam] : seamStats) {
		if (pair.first != (uint32_t)c && pair.second != (uint32_t)c)
			continue;
		++numAdjacent;
		if (IsStrongSeam(seam, config))
			++numStrong;
	}
	return numAdjacent > 0 && numStrong >= MINF(config.minClusterDegree, numAdjacent);
}

void SceneCluster::MergeSmallClusters(std::vector<IIndexArr>& clusters)
{
	std::vector<int> nodeToCluster = MapNodesToClusters(clusters, scene.images.size());

	bool changed = true;
	while (changed) {
		changed = false;
		const SeamTrackStatsMap seamStats = std::any_of(clusters.begin(), clusters.end(),
			[&](const IIndexArr& c) { return !c.empty() && c.size() < config.minViewsPerCluster; }) ?
			ComputeSeamTrackStats(clusters) : SeamTrackStatsMap();
		for (size_t c = 0; c < clusters.size(); ++c) {
			if (clusters[c].empty() || clusters[c].size() >= config.minViewsPerCluster) continue;
			if (IsStandaloneCommunity(clusters, c, nodeToCluster, seamStats)) continue;

			std::unordered_map<int, double> cluster_weights;
			for (IIndex u : clusters[c]) {
				for (int i = xadj[u]; i < xadj[u+1]; ++i) {
					const int target_c = nodeToCluster[adjncy[i]];
					if (target_c != (int)c && target_c != -1)
						cluster_weights[target_c] += adjwgt[i];
				}
			}

			int best_target = -1;
			double max_weight = -1.0;
			for (const auto& p : cluster_weights) {
				if (clusters[p.first].size() + clusters[c].size() > config.maxViewsPerCluster + config.maxOverCapacity)
					continue;
				if (p.second > max_weight || (p.second == max_weight && p.first < best_target)) {
					max_weight = p.second;
					best_target = p.first;
				}
			}

			if (best_target != -1) {
				AbsorbCluster(clusters, c, (size_t)best_target, &nodeToCluster);
				changed = true;
			}
		}
	}

	EraseEmptyClusters(clusters);
}

SeamTrackStatsMap SceneCluster::ComputeSeamTrackStats(const std::vector<IIndexArr>& clusters) const
{
	const std::vector<int> clusterOf = MapNodesToClusters(clusters, scene.images.size());

	SeamTrackStatsMap stats;
	std::map<uint32_t, unsigned> numObservations;
	for (const Track& track : scene.tracks) {
		if (!track.IsValid())
			continue;
		numObservations.clear();
		for (const Observation& obs : track.observations) {
			ASSERT(obs.imageID < clusterOf.size());
			if (clusterOf[obs.imageID] >= 0)
				++numObservations[(uint32_t)clusterOf[obs.imageID]];
		}
		if (numObservations.size() < 2)
			continue;
		for (auto x = numObservations.cbegin(); x != numObservations.cend(); ++x) {
			for (auto y = std::next(x); y != numObservations.cend(); ++y) {
				if (x->second < 2 && y->second < 2)
					continue;
				std::pair<SeamTrackStats, SeamTrackStats>& seam = stats[std::make_pair(x->first, y->first)];
				++seam.first.usable;
				++seam.second.usable;
				for (const Observation& obs : track.observations) {
					const int c = clusterOf[obs.imageID];
					if (c == (int)x->first)
						++seam.first.perCamera[obs.imageID];
					else if (c == (int)y->first)
						++seam.second.perCamera[obs.imageID];
				}
			}
		}
	}
	return stats;
}

// A cluster the merge could never attach: the seams it has carry too few usable tracks, or have
// them in too few cameras, to register it against its neighbours. Reconstructed on its own it would
// come out as a block the merge has to leave unplaced, so it is joined to the neighbour it shares
// the most usable tracks with while that still fits inside the capacity slack.
// A cluster is held to as many strong seams as its position can give it: a cluster with a single
// neighbour can never have a second strong one, and absorbing it would only leave the absorbing
// cluster in the same one-sided position — the leaf would move, the chain would not close — so the
// bar is minClusterDegree capped by the number of neighbours the cluster actually has.
void SceneCluster::MergeLeafClusters(std::vector<IIndexArr>& clusters)
{
	// every round merges each cluster at most once, so the cluster count bounds the rounds
	for (unsigned round = (unsigned)clusters.size(); round > 0; --round) {
		const SeamTrackStatsMap stats = ComputeSeamTrackStats(clusters);
		std::vector<unsigned> numStrong(clusters.size(), 0), numAdjacent(clusters.size(), 0),
			bestUsable(clusters.size(), 0);
		std::vector<int> bestNeighbour(clusters.size(), -1);
		for (const auto& [pair, seam] : stats) {
			const uint32_t sides[2] = {pair.first, pair.second};
			// every pair the statistics hold shares at least one seam-usable track, which is what
			// makes the two clusters neighbours at all
			++numAdjacent[sides[0]];
			++numAdjacent[sides[1]];
			if (IsStrongSeam(seam, config)) {
				++numStrong[sides[0]];
				++numStrong[sides[1]];
			}
			for (unsigned s = 0; s < 2; ++s) {
				if (seam.first.usable > bestUsable[sides[s]]) {
					bestUsable[sides[s]] = seam.first.usable;
					bestNeighbour[sides[s]] = (int)sides[1-s];
				}
			}
		}

		bool changed = false;
		std::vector<bool> merged(clusters.size(), false);
		std::vector<size_t> refused; // the leaves their neighbour has no room for, reported once
		for (size_t c = 0; c < clusters.size(); ++c) {
			if (clusters[c].size() < config.minViewsPerCluster || merged[c])
				continue; // under the floor: the floor rule decides where it goes
			if (numStrong[c] >= MINF(config.minClusterDegree, numAdjacent[c]))
				continue; // as many strong seams as this cluster could have
			const int into = bestNeighbour[c];
			if (into < 0 || merged[into])
				continue; // nothing to join, or the neighbour already took one in this round
			if (clusters[c].size() + clusters[into].size() > config.maxViewsPerCluster + config.maxOverCapacity) {
				refused.push_back(c);
				continue;
			}
			VERBOSE("Cluster %u (%u images) has %u strong neighbours; merged into cluster %u",
				(unsigned)c, (unsigned)clusters[c].size(), numStrong[c], (unsigned)into);
			AbsorbCluster(clusters, c, (size_t)into);
			merged[c] = merged[into] = true;
			changed = true;
		}
		if (!changed) {
			// nothing moved, so these clusters are the ones the split ends with and their numbers
			// are final: a leaf reported here is one that stays unattached
			for (size_t c : refused)
				VERBOSE("Cluster %u (%u images) has %u strong neighbours and no room to merge",
					(unsigned)c, (unsigned)clusters[c].size(), numStrong[c]);
			break;
		}
		EraseEmptyClusters(clusters);
	}
}

void SceneCluster::RefineClustersForSeams(std::vector<IIndexArr>& clusters)
{
	MergeLeafClusters(clusters);
	// the pass above moves whole clusters around and can leave a part under the floor; the floor rule
	// takes them the same way it takes any other small cluster
	MergeSmallClusters(clusters);

	// the seam of every adjacent pair as the merge will read it: the raw material for setting the
	// track and camera floors on a real capture
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 0) {
		for (const auto& [pair, seam] : ComputeSeamTrackStats(clusters))
			DEBUG("Cluster pair (%u, %u): %u seam-usable tracks, cameras %u/%u with >= %u",
				pair.first, pair.second, seam.first.usable,
				seam.first.CamerasWithAtLeast(config.minSeamCameraTracks),
				seam.second.CamerasWithAtLeast(config.minSeamCameraTracks),
				config.minSeamCameraTracks);
	}
	#endif
}

void SceneCluster::RefineClustersLocalSearch(std::vector<IIndexArr>& clusters)
{
	const unsigned sizeLimit = MINF(config.targetViewsPerCluster, config.maxViewsPerCluster);
	const IIndex nViews = scene.images.size();
	std::vector<int> nodeToCluster = MapNodesToClusters(clusters, nViews);

	bool changed = true;
	int iters = 0;
	while (changed && iters < 20) {
		changed = false;
		iters++;
		for (IIndex u = 0; u < nViews; ++u) {
			int current_c = nodeToCluster[u];
			if (current_c == -1) continue;

			int best_target = current_c;
			double max_gain = 0.0;

			std::unordered_map<int, double> cluster_weights;
			for (int i = xadj[u]; i < xadj[u+1]; ++i) {
				const int target_c = nodeToCluster[adjncy[i]];
				if (target_c != -1)
					cluster_weights[target_c] += adjwgt[i];
			}

			const double current_internal_weight = cluster_weights[current_c];
			for (const auto& p : cluster_weights) {
				const int target_c = p.first;
				if (target_c == current_c || clusters[target_c].size() >= sizeLimit) continue;
				const double gain = p.second - current_internal_weight;
				if (gain > max_gain || (gain == max_gain && gain > 0.0 && target_c < best_target)) {
					max_gain = gain;
					best_target = target_c;
				}
			}

			if (best_target != current_c) {
				nodeToCluster[u] = best_target;
				clusters[best_target].push_back(u);
				auto it = std::find(clusters[current_c].begin(), clusters[current_c].end(), u);
				if (it != clusters[current_c].end()) {
					*it = clusters[current_c].back();
					clusters[current_c].pop_back();
				}
				changed = true;
			}
		}
	}

	EraseEmptyClusters(clusters);
}

// Whether cluster c without image u keeps every prefix cut along its Fiedler order at or above
// minClusterCoupling, and stays connected
bool SceneCluster::KeepsCoupling(const std::vector<IIndexArr>& clusters, int c, IIndex u) const
{
	std::vector<int> localIndex(scene.images.size(), -1);
	uint32_t li = 0;
	for (IIndex g : clusters[c])
		if (g != u)
			localIndex[g] = (int)li++;
	std::vector<std::pair<uint32_t, uint32_t>> edges;
	std::vector<double> weights;
	for (IIndex g : clusters[c]) {
		if (g == u)
			continue;
		const int lg = localIndex[g];
		for (int i = xadj[g]; i < xadj[g + 1]; ++i) {
			const int lv = localIndex[adjncy[i]];
			if (lv > lg) {
				edges.emplace_back((uint32_t)lg, (uint32_t)lv);
				weights.push_back((double)adjwgt[i]);
			}
		}
	}
	const ClusterCoupling cc = AnalyzeClusterCoupling(li, edges, weights, config.minWaistSideViews, config.minClusterCoupling);
	return cc.connected && !cc.weak;
}

// Move well-connected boundary images out of the largest cluster into smaller neighbours while
// keeping the source cluster connected and above minClusterCoupling.
void SceneCluster::RefineClustersBalance(std::vector<IIndexArr>& clusters)
{
	if (xadj.empty())
		BuildConnectivityGraph();

	constexpr float kBalanceAffinity = 0.7f;
	constexpr float kBalanceTolerance = 1.25f;

	const IIndex nViews = scene.images.size();
	std::vector<int> nodeToCluster = MapNodesToClusters(clusters, nViews);

	unsigned sizeBefore = 0;
	for (const IIndexArr& c : clusters)
		sizeBefore = MAXF(sizeBefore, (unsigned)c.size());

	const unsigned maxMoves = 2 * config.maxViewsPerCluster;
	unsigned numMoves = 0;
	while (numMoves < maxMoves) {
		int src = -1;
		unsigned srcSize = 0;
		unsigned numActive = 0;
		uint64_t totalSize = 0;
		for (size_t c = 0; c < clusters.size(); ++c) {
			const unsigned size = (unsigned)clusters[c].size();
			if (size == 0) continue;
			++numActive;
			totalSize += size;
			if (size > srcSize) {
				srcSize = size;
				src = (int)c;
			}
		}
		if (src < 0)
			break;
		const float mean = (float)totalSize / (float)numActive;
		if (srcSize <= CEIL2INT<unsigned>(kBalanceTolerance * mean) || srcSize <= config.minViewsPerCluster)
			break;

		struct Move { double weight; IIndex u; int target; };
		std::vector<Move> moves;
		for (IIndex u = 0; u < nViews; ++u) {
			if (nodeToCluster[u] != src)
				continue;

			std::unordered_map<int, double> cluster_weights;
			double wSrc = 0.0;
			for (int i = xadj[u]; i < xadj[u+1]; ++i) {
				const int target_c = nodeToCluster[adjncy[i]];
				if (target_c == src)
					wSrc += adjwgt[i];
				else if (target_c != -1)
					cluster_weights[target_c] += adjwgt[i];
			}
			if (wSrc == 0.0)
				continue;

			int targetC = -1;
			double targetW = 0.0;
			for (const auto& p : cluster_weights) {
				const unsigned tSize = (unsigned)clusters[p.first].size();
				if (tSize >= srcSize - 1 || tSize >= config.maxViewsPerCluster || tSize < config.minViewsPerCluster) continue;
				if (p.second > targetW || (p.second == targetW && p.first < targetC)) {
					targetW = p.second;
					targetC = p.first;
				}
			}
			if (targetC < 0 || targetW < (double)kBalanceAffinity * wSrc)
				continue;

			moves.push_back({targetW, u, targetC});
		}
		std::sort(moves.begin(), moves.end(), [](const Move& a, const Move& b) {
			return a.weight > b.weight || (a.weight == b.weight && a.u < b.u);
		});
		const Move* chosen = NULL;
		for (const Move& move : moves) {
			if (KeepsCoupling(clusters, src, move.u)) {
				chosen = &move;
				break;
			}
		}
		if (!chosen)
			break;

		auto it = std::find(clusters[src].begin(), clusters[src].end(), chosen->u);
		ASSERT(it != clusters[src].end());
		*it = clusters[src].back();
		clusters[src].pop_back();
		clusters[chosen->target].push_back(chosen->u);
		nodeToCluster[chosen->u] = chosen->target;
		++numMoves;
	}

	if (numMoves > 0) {
		unsigned sizeAfter = 0;
		for (const IIndexArr& c : clusters)
			sizeAfter = MAXF(sizeAfter, (unsigned)c.size());
		VERBOSE("Clustering balance: moved %u images (largest cluster %u -> %u images)", numMoves, sizeBefore, sizeAfter);
	}
}

void SceneCluster::RefineClustersSplitDisconnected(std::vector<IIndexArr>& clusters)
{
	const IIndex nViews = scene.images.size();
	const std::vector<int> nodeToCluster = MapNodesToClusters(clusters, nViews);
	std::vector<uint8_t> visited(nViews, 0);

	std::vector<IIndexArr> new_clusters;
	for (size_t c = 0; c < clusters.size(); ++c) {
		if (clusters[c].empty()) continue;
		const IIndexArr members(clusters[c]);
		bool first = true;
		for (IIndex startNode : members) {
			if (visited[startNode]) continue;
			IIndexArr component;
			std::queue<IIndex> q;
			q.push(startNode);
			visited[startNode] = 1;
			while (!q.empty()) {
				const IIndex u = q.front();
				q.pop();
				component.push_back(u);
				for (int i = xadj[u]; i < xadj[u+1]; ++i) {
					const IIndex v = adjncy[i];
					if (nodeToCluster[v] == (int)c && !visited[v]) {
						visited[v] = 1;
						q.push(v);
					}
				}
			}
			if (first) {
				clusters[c] = std::move(component);
				first = false;
			} else {
				new_clusters.push_back(std::move(component));
			}
		}
	}
	if (!new_clusters.empty())
		clusters.insert(clusters.end(), new_clusters.begin(), new_clusters.end());
}

void SceneCluster::RefineClustersSplitThinWaist(std::vector<IIndexArr>& clusters)
{
	if (xadj.empty())
		BuildConnectivityGraph();
	if (config.minClusterCoupling <= 0)
		return;

	constexpr unsigned kWaistRoundsPerCluster = 4;
	unsigned budget = kWaistRoundsPerCluster * (unsigned)clusters.size();
	std::vector<int> clusterOf;
	std::vector<uint32_t> localIndex;
	std::vector<std::vector<std::pair<uint32_t, uint32_t>>> edges;
	std::vector<std::vector<double>> weights;
	bool dirty = true;

	for (size_t c = 0; c < clusters.size() && budget > 0; ) {
		if (dirty) {
			clusterOf.assign(scene.images.size(), -1);
			localIndex.assign(scene.images.size(), 0);
			for (size_t ci = 0; ci < clusters.size(); ++ci) {
				uint32_t li = 0;
				for (IIndex g : clusters[ci]) {
					clusterOf[g] = (int)ci;
					localIndex[g] = li++;
				}
			}
			edges.assign(clusters.size(), {});
			weights.assign(clusters.size(), {});
			BucketClusterEdges(scene, config.minPairWeight, clusterOf, localIndex, edges, weights);
			dirty = false;
		}

		const unsigned V = (unsigned)clusters[c].size();
		if (V < 2 * config.minWaistSideViews) {
			++c;
			continue;
		}
		const ClusterCoupling cc = AnalyzeClusterCoupling(V, edges[c], weights[c], config.minWaistSideViews, config.minClusterCoupling);
		if (!cc.connected || !cc.weak || cc.side.size() != V) {
			++c;
			continue;
		}
		IIndexArr stretch, rest;
		FOREACH(i, clusters[c])
			(cc.side[i] ? stretch : rest).push_back(clusters[c][i]);
		if (cc.smaller >= config.minViewsPerCluster) {
			VERBOSE("Clustering: split thin-waist cluster (%u images, internal coupling %.3f < %.3f) into %u+%u images",
				V, cc.coupling, config.minClusterCoupling, (unsigned)rest.size(), (unsigned)stretch.size());
			clusters[c] = std::move(rest);
			clusters.push_back(std::move(stretch));
			dirty = true;
			--budget;
			continue;
		}

		std::unordered_map<int, double> interfaces;
		for (IIndex u : stretch) {
			for (int i = xadj[u]; i < xadj[u + 1]; ++i) {
				const int cv = clusterOf[adjncy[i]];
				if (cv >= 0 && cv != (int)c)
					interfaces[cv] += adjwgt[i];
			}
		}
		int target = -1;
		double targetWeight = 0.0;
		for (const auto& [t, w] : interfaces)
			if (w > targetWeight || (w == targetWeight && t < target)) { target = t; targetWeight = w; }
		const double internalTarget = target >= 0 ? std::accumulate(weights[target].begin(), weights[target].end(), 0.0) : 0.0;
		if (target < 0 || targetWeight <= cc.cutWeight ||
			targetWeight < (double)config.minClusterCoupling * MINF(cc.smallerInternal, internalTarget) ||
			clusters[target].size() + stretch.size() > config.maxViewsPerCluster + config.maxOverCapacity) {
			DEBUG("Clustering: a stretch of %u images coupled %.3f to its cluster of %u has no neighbour holding it better; left in place",
				(unsigned)stretch.size(), cc.coupling, V);
			++c;
			continue;
		}
		VERBOSE("Clustering: moved a stretch of %u images, coupled %.3f to its cluster of %u images, to the cluster of %u images it is coupled %.3f to",
			(unsigned)stretch.size(), cc.coupling, V, (unsigned)clusters[target].size(),
			targetWeight / MAXF(MINF(cc.smallerInternal, internalTarget), 1e-9));
		for (IIndex u : stretch)
			clusters[target].push_back(u);
		clusters[c] = std::move(rest);
		dirty = true;
		--budget;
	}
}

ScenePartition SceneCluster::FinishPartition(std::vector<IIndexArr>& clusters)
{
	const std::vector<int> nodeToCluster = MapNodesToClusters(clusters, scene.images.size());
	const SeamTrackStatsMap seamStats = std::any_of(clusters.begin(), clusters.end(),
		[&](const IIndexArr& c) { return !c.empty() && c.size() < config.minViewsPerCluster; }) ?
		ComputeSeamTrackStats(clusters) : SeamTrackStatsMap();
	std::vector<bool> isSubScene(clusters.size());
	for (size_t c = 0; c < clusters.size(); ++c)
		isSubScene[c] = !clusters[c].empty() &&
			(clusters[c].size() >= config.minViewsPerCluster || IsStandaloneCommunity(clusters, c, nodeToCluster, seamStats));

	// verify the clusters are well connected before reconstruction; the internal
	// covisibility graph still holds every intra-cluster pair (the blocks move them out
	// when they are extracted), so this must run before
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		ReportClusterCoupling(scene, clusters, config, isSubScene);
	#endif

	ScenePartition partition;
	partition.clusters.reserve(clusters.size());
	for (size_t c = 0; c < clusters.size(); ++c) {
		if (clusters[c].empty()) continue;
		IIndexArr& cluster = clusters[c];
		// Sort by global ID so that local IDs preserve global ordering:
		// localID1 < localID2 means also globalID1 < globalID2
		// so pair ID ordering (ID1 < ID2) is maintained through local-global remapping
		cluster.Sort();
		if (!isSubScene[c]) {
			DEBUG("Small cluster of %u views goes to the pool", (unsigned)cluster.size());
			partition.pool.Join(cluster);
			continue;
		}
		partition.clusters.emplace_back(std::move(cluster));
	}
	partition.pool.Sort();
	return partition;
}

bool SceneCluster::ExportClusterPositions(
	const std::vector<Scene>& subScenes,
	const String& fileName)
{
	// Compute the ECEF centroid for normalizing the positions (optional, can help with visualization if large numbers)
	Point3dArr ecefPositions;
	Point3d centerECEF(0, 0, 0);
	FOREACH(clusterID, subScenes) {
		const Scene& scene = subScenes[clusterID];
		for (const Image& img : scene.images) {
			// Check if GPS data is valid (simple check: not all zero)
			const View::Metadata& viewMeta = img.View::metadata;
			if (!viewMeta.HasGPS())
				continue;
			Point3d ecef;
			WGS84ToECEF(viewMeta.latitude, viewMeta.longitude, viewMeta.altitude, ecef.x, ecef.y, ecef.z);
			ecefPositions.push_back(ecef);
			centerECEF += ecef;
		}
	}
	if (ecefPositions.empty()) {
		DEBUG("warning: no images with GPS positions found");
		return false;
	}
	centerECEF /= (double)ecefPositions.size();
	double lat0, lon0, alt0;
	ECEFToWGS84(centerECEF.x, centerECEF.y, centerECEF.z, lat0, lon0, alt0);

	// Generate unique color per cluster
	auto GenerateClusterColor = [](size_t clusterID, size_t numClusters) -> Pixel8U {
		if (numClusters == 1)
			return Pixel8U::RED; // red for single cluster
		// Generate distinct colors using HSV color space
		Pixel32F hsv{
			(float)clusterID / (float)numClusters * 360.f,
			0.9f,
			0.9f
		};
		Pixel32F rgb = CONVERT::HSV2RGB(hsv) * 255.f; // scale to [0, 255]
		return rgb.cast<uint8_t>();
	};

	// the ENU position (east, north, up, about the centroid) of each image, in the color of its cluster
	std::vector<Point3f> positions;
	std::vector<Pixel8U> colors;
	unsigned vertexCount = 0;
	positions.reserve(ecefPositions.size());
	colors.reserve(ecefPositions.size());
	FOREACH(clusterID, subScenes) {
		const Scene& scene = subScenes[clusterID];
		const Pixel8U clusterColor = GenerateClusterColor(clusterID, subScenes.size());
		for (const Image& img : scene.images) {
			if (!img.View::metadata.HasGPS())
				continue;
			const Point3d& ecef = ecefPositions[vertexCount++];
			double e, n, u;
			ECEFToENU(ecef.x, ecef.y, ecef.z, centerECEF.x, centerECEF.y, centerECEF.z, lat0, lon0, e, n, u);
			positions.emplace_back(static_cast<float>(e), static_cast<float>(n), static_cast<float>(u));
			colors.emplace_back(clusterColor);
		}
	}
	PLY ply;
	ply.AddElement("vertex", positions.size(), {
		PLY::Scalar("x", &positions[0].x, sizeof(Point3f)),
		PLY::Scalar("y", &positions[0].y, sizeof(Point3f)),
		PLY::Scalar("z", &positions[0].z, sizeof(Point3f)),
		PLY::Scalar("red", &colors[0].r, sizeof(Pixel8U)),
		PLY::Scalar("green", &colors[0].g, sizeof(Pixel8U)),
		PLY::Scalar("blue", &colors[0].b, sizeof(Pixel8U))});
	if (!ply.Save(fileName))
		return false;

	VERBOSE("Exported %u GPS positions corresponding to %u clusters to '%s'",
		(unsigned)ecefPositions.size(), (unsigned)subScenes.size(), fileName.c_str());
	return true;
}
/*----------------------------------------------------------------*/
