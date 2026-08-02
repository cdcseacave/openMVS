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

constexpr float WEIGHT_MULTIPLIER = 10.f; // Multiplier to convert float weights to integers for METIS

namespace {

// Creation-time connectivity self-check for a single finished cluster.
//
// The greedy merge refuses to join two mature clusters across an interface
// carrying less than minClusterCoupling of the weaker side's internal weight, so
// that no sub-scene contains two blocks joined only by a sparse seam (such a seam
// lets scale drift accumulate unobserved and reconstructs as two independently
// scaled blocks — the two-scale bug the merge-time split then has to repair).
// This routine verifies that invariant on the FINAL clusters: it finds each
// cluster's best balanced bipartition (the spectral / Fiedler cut of its internal
// covisibility graph) and expresses the interface as the same coupling ratio the
// merge test uses. A cluster whose two substantial halves fall below the floor was
// mis-assembled by clustering; flagging it here points debugging at the clustering
// decision instead of the post-reconstruction symptom. Purely structural, log only.
struct ClusterCoupling {
	unsigned larger = 0, smaller = 0; // bipartition block sizes (order-independent)
	double coupling = 1.0;            // interface weight / weaker block internal weight
	bool connected = true;            // internal graph in one piece
	bool weak = false;                // substantial balanced blocks below the coupling floor
	std::vector<uint8_t> side;        // Fiedler bipartition (0/1 per local index; filled when connected)
};

ClusterCoupling AnalyzeClusterCoupling(
	unsigned numImages,
	const std::vector<std::pair<uint32_t, uint32_t>>& edges,
	const std::vector<double>& weights,
	unsigned minBlock, float minCoupling)
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

	// Fiedler (second-smallest eigenvector) bipartition of the weighted Laplacian
	Eigen::MatrixXd L = Eigen::MatrixXd::Zero(numImages, numImages);
	for (size_t k = 0; k < edges.size(); ++k) {
		const uint32_t u = edges[k].first, v = edges[k].second;
		const double w = weights[k];
		L(u, u) += w; L(v, v) += w;
		L(u, v) -= w; L(v, u) -= w;
	}
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(L);
	const Eigen::VectorXd fiedler = es.eigenvectors().col(1);
	std::vector<uint8_t> side(numImages);
	unsigned n0 = 0;
	for (uint32_t i = 0; i < numImages; ++i) {
		side[i] = fiedler[(Eigen::Index)i] >= 0.0 ? 0 : 1;
		if (side[i] == 0)
			++n0;
	}
	const unsigned n1 = numImages - n0;
	double cut = 0.0, wint0 = 0.0, wint1 = 0.0;
	for (size_t k = 0; k < edges.size(); ++k) {
		const uint32_t u = edges[k].first, v = edges[k].second;
		const double w = weights[k];
		if (side[u] != side[v]) cut += w;
		else if (side[u] == 0) wint0 += w; else wint1 += w;
	}
	const double minInt = MINF(wint0, wint1);
	r.coupling = minInt > 1e-9 ? cut / minInt : 0.0;
	r.larger = MAXF(n0, n1);
	r.smaller = MINF(n0, n1);
	r.weak = r.smaller >= minBlock && r.coupling < (double)minCoupling;
	r.side = std::move(side);
	return r;
}

// Bucket the internal covisibility edges of every cluster (endpoints remapped to the
// cluster-local indices) in one sweep over the global pair graph, for the coupling
// analysis; clusterOf maps an image to its cluster (-1 for none) and localIndex to its
// position in it. Pairs below minPairWeight are skipped, matching the graph every
// consumer of the coupling invariant sees: BuildConnectivityGraph drops them before
// clustering and BuildTracks forms no tracks from them, so they constrain nothing.
// scene.pairs still holds every pair here — intra-cluster pairs are moved into the
// sub-scenes only later, in BuildSubScenesFromClusters.
void BucketClusterEdges(const Scene& scene, float minPairWeight,
	const std::vector<int>& clusterOf, const std::vector<uint32_t>& localIndex,
	std::vector<std::vector<std::pair<uint32_t, uint32_t>>>& edges, std::vector<std::vector<double>>& weights)
{
	for (const ImagePair& pair : scene.pairs) {
		const float weight(pair.GetCompositeWeight());
		if (weight < minPairWeight)
			continue;
		const int c = clusterOf[pair.ID1];
		if (c < 0 || c != clusterOf[pair.ID2])
			continue;
		edges[c].emplace_back(localIndex[pair.ID1], localIndex[pair.ID2]);
		weights[c].push_back((double)weight);
	}
}

// Run AnalyzeClusterCoupling on every cluster that will become a sub-scene, numbered exactly
// as BuildSubScenesFromClusters numbers them (small clusters skipped), so a flag here lines
// up with the merge-time telemetry of the same sub-scene. This is the health check for the
// two-scale defect: the merge stage aligns the sub-scenes to each other but cannot tell
// whether one of them reconstructed at two scales, so the invariant that no sub-scene holds
// two blocks joined by a seam too sparse to observe their relative scale is verified here,
// structurally, on the graph that clustering produced.
void ReportClusterCoupling(const Scene& scene, const std::vector<IIndexArr>& clusters, const ClusterConfig& config)
{
	std::vector<int> clusterOf(scene.images.size(), -1);
	std::vector<uint32_t> localIndex(scene.images.size(), 0);
	std::vector<unsigned> clusterSize;
	int subSceneIdx = 0;
	for (const IIndexArr& cluster : clusters) {
		if (cluster.size() < config.minViewsPerCluster)
			continue;
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
		const ClusterCoupling cc = AnalyzeClusterCoupling(V, edges[s], weights[s], MAXF(config.minViewsPerCluster, V / 5), config.minClusterCoupling);
		if (!cc.connected) {
			VERBOSE("warning: Sub-scene %u internally disconnected at creation: components %u+%u images; clustering produced a split sub-scene", s, cc.larger, cc.smaller);
			++numWeak;
		} else if (cc.weak) {
			VERBOSE("warning: Sub-scene %u under-coupled at creation: blocks %u+%u images joined at coupling %.3f < %.3f; it may reconstruct at two scales, which the merge cannot repair", s, cc.larger, cc.smaller, cc.coupling, config.minClusterCoupling);
			++numWeak;
		} else {
			VERBOSE("Sub-scene %u coupling ok: %u images, spectral cut coupling %.3f", s, V, cc.coupling);
		}
	}
	VERBOSE("Cluster coupling check: %u/%u sub-scenes flagged weak at creation", numWeak, numKept);
}

} // namespace

SceneCluster::SceneCluster(Scene& scene, const ClusterConfig& config)
	: scene(scene), config(config)
{
}

Scene SceneCluster::ExtractSubScene(
	const IIndexArr& viewIndices,
	const IIndexArr& globalToLocal,
	unsigned nThreadsPerCluster)
{
	Scene subScene(nThreadsPerCluster);

	// Map: global camera index -> local camera index
	std::unordered_map<IIndex, IIndex> cameraMap;

	// Copy images and cameras; move expensive data (keypoints, descriptors)
	// from the global scene to sub-scenes to save memory during reconstruction.
	// These are moved back during GlobalAlignment::MergeSingleScene.
	for (IIndex globalID : viewIndices) {
		Image& img = scene.images[globalID];
		// Ensure camera exists in sub-scene
		const IIndex globalCamID = img.cameraID;
		auto ret = cameraMap.emplace(globalCamID, subScene.cameras.size());
		if (ret.second) {
			// Clone camera for independent bundle adjustment
			subScene.cameras.emplace_back(scene.cameras[globalCamID]->Clone());
		}
		IIndex localCamID = ret.first->second;
		// Add image with remapped IDs
		Image subImg = img;
		subImg.ID = subScene.images.size();
		subImg.cameraID = localCamID;
		subImg.pCamera = subScene.cameras[localCamID];
		// Move expensive data from global to sub-scene to save memory
		subImg.keypoints = std::move(img.keypoints);
		subImg.descriptors = std::move(img.descriptors);
		subScene.images.emplace_back(std::move(subImg));
	}

	// Move image pairs (only with image IDs within cluster)
	for (uint32_t i = 0; i < scene.pairs.size(); ++i) {
		ImagePair& pair = scene.pairs[i];
		const IIndex localID1 = globalToLocal[pair.ID1];
		const IIndex localID2 = globalToLocal[pair.ID2];
		if (localID1 == NO_ID || localID2 == NO_ID)
			continue; // pair crosses cluster boundary
		// Move pair to avoid copying large match vectors
		ASSERT(localID1 < localID2);
		pair.ID1 = localID1;
		pair.ID2 = localID2;
		subScene.pairs.emplace_back(std::move(pair));
		scene.pairs.RemoveAtMove(i--);
	}

	// Copy tracks (include tracks with ≥2 observations in this cluster)
	for (const Track& srcTrack : scene.tracks) {
		Track dstTrack;
		dstTrack.position = srcTrack.position;
		FOREACH(k, srcTrack.observations) {
			const Observation& obs = srcTrack.observations[k];
			const uint32_t localID = globalToLocal[obs.imageID];
			if (localID == NO_ID)
				continue; // observation outside this cluster
			dstTrack.observations.emplace_back(localID, obs.featureID);
			if (k < srcTrack.numInliers)
				++dstTrack.numInliers;
		}
		// Include track if it has at least 2 observations in this cluster
		if (dstTrack.GetNumObservations() >= 2)
			subScene.tracks.emplace_back(dstTrack);
	}

    VERBOSE("Sub-scene: %u images, %u pairs, %u tracks",
	    subScene.images.size(), subScene.pairs.size(),
	    subScene.tracks.size());
	return subScene;
}

std::vector<Scene> SceneCluster::SplitScene(std::vector<IIndexArr>* outLocalToGlobal)
{
	IIndex nViews = scene.images.size();
	if (nViews == 0) {
		return {};
	}
	if (config.maxViewsPerCluster == 0 || nViews <= config.maxViewsPerCluster) {
		// No need to split - create single cluster with identity mapping
		DEBUG("Scene has %u images, no clustering needed", nViews);
		return {std::move(scene)};
	}

	BuildConnectivityGraph();

	return config.useCommunityDetection ?
		SplitSceneCommunityDetection(outLocalToGlobal) :
		SplitSceneAggregativeClustering(outLocalToGlobal);
}

void SceneCluster::BuildConnectivityGraph()
{
	const IIndex nViews = scene.images.size();
	xadj.assign(nViews + 1, 0);

	std::vector<int> degrees(nViews, 0);
	for (const ImagePair& pair : scene.pairs) {
		if (pair.GetCompositeWeight() < config.minPairWeight)
			continue;
		degrees[pair.ID1]++;
		degrees[pair.ID2]++;
	}
	for (IIndex i = 0; i < nViews; ++i) {
		xadj[i+1] = xadj[i] + degrees[i];
	}

	adjncy.assign(xadj.back(), 0);
	adjwgt.assign(xadj.back(), 0);
	std::vector<int> offsets = xadj;

	for (const ImagePair& pair : scene.pairs) {
		const float weight = pair.GetCompositeWeight();
		if (weight < config.minPairWeight)
			continue;
		const int w = cvRound(weight * WEIGHT_MULTIPLIER);

		int idx1 = offsets[pair.ID1]++;
		adjncy[idx1] = pair.ID2;
		adjwgt[idx1] = w;

		int idx2 = offsets[pair.ID2]++;
		adjncy[idx2] = pair.ID1;
		adjwgt[idx2] = w;
	}
	VERBOSE("Built connectivity graph: %u nodes, %u edges", (unsigned)nViews, (unsigned)(adjncy.size() / 2));
}

std::vector<Scene> SceneCluster::SplitSceneAggregativeClustering(std::vector<IIndexArr>* outLocalToGlobal)
{
	const IIndex nViews = scene.images.size();
	std::vector<IIndexArr> clusters(nViews);
	for (IIndex i = 0; i < nViews; ++i)
		clusters[i].push_back(i);

	GreedyMergeClusters(clusters, true);

	RefineClustersLocalSearch(clusters);
	MergeSmallClusters(clusters);
	RefineClustersBalance(clusters);
	RefineClustersSplitDisconnected(clusters);
	RefineClustersSplitThinWaist(clusters);
	RefineClustersRescueOrphans(clusters);

	return BuildSubScenesFromClusters(clusters, outLocalToGlobal);
}

// Merge clusters bottom-up: repeatedly join the two clusters connected by the
// highest aggregate edge weight, subject to the capacity limit and the coupling
// acceptance test. The aggregate weight between two communities grows with their
// sizes, so many individually weak pairs eventually top the queue even when they
// represent only a few percent of either side's internal cohesion; reconstructing
// across such a sparse interface lets scale drift accumulate unobserved, hence
// mature clusters are only merged when the interface carries at least
// minClusterCoupling of the weaker side's internal weight. Any thin seam that still
// slips through — including one that only emerges as a cluster accretes from both
// sides — is caught after the fact by RefineClustersSplitThinWaist. A refusal is
// not permanent: the edge is re-pushed whenever either side changes.
void SceneCluster::GreedyMergeClusters(std::vector<IIndexArr>& clusters, bool periodicRefine)
{
	const IIndex nViews = scene.images.size();
	std::vector<int> nodeToCluster(nViews, -1);
	for (size_t c = 0; c < clusters.size(); ++c)
		for (IIndex n : clusters[c])
			nodeToCluster[n] = (int)c;

	struct Edge {
		int u, v;
		int64_t weight;
		bool operator<(const Edge& other) const { return weight < other.weight; }
	};
	std::priority_queue<Edge> pq;
	std::vector<std::unordered_map<int, int64_t>> adj(clusters.size());
	std::vector<int64_t> wint(clusters.size(), 0);

	auto rebuildPQ = [&]() {
		pq = std::priority_queue<Edge>();
		adj.assign(clusters.size(), {});
		wint.assign(clusters.size(), 0);
		for (IIndex u = 0; u < nViews; ++u) {
			const int cu = nodeToCluster[u];
			if (cu < 0) continue;
			for (int i = xadj[u]; i < xadj[u+1]; ++i) {
				const int cv = nodeToCluster[adjncy[i]];
				if (cv < 0) continue;
				if (cu < cv)
					adj[cu][cv] += adjwgt[i];
				else if (cu == cv)
					wint[cu] += adjwgt[i]; // each internal edge seen from both endpoints
			}
		}
		for (size_t i = 0; i < clusters.size(); ++i) {
			wint[i] /= 2;
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
		if (config.minClusterCoupling > 0 &&
			MINF(clusters[u].size(), clusters[v].size()) >= config.minViewsPerCluster &&
			(float)e.weight < config.minClusterCoupling * (float)MINF(wint[u], wint[v]))
			continue; // two established communities joined only by a sparse interface

		for (IIndex node : clusters[v]) {
			nodeToCluster[node] = u;
			clusters[u].push_back(node);
		}
		clusters[v].clear();
		wint[u] += wint[v] + e.weight;
		wint[v] = 0;
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

		if (periodicRefine) {
			const unsigned mergesPerRefine = MAXF(10u, config.maxViewsPerCluster / 10);
			if (numMergesSinceRefine >= mergesPerRefine) {
				RefineClustersLocalSearch(clusters);
				nodeToCluster.assign(nViews, -1);
				for (size_t c = 0; c < clusters.size(); ++c)
					for (IIndex n : clusters[c])
						nodeToCluster[n] = (int)c;
				adj.resize(clusters.size());
				rebuildPQ();
				numMergesSinceRefine = 0;
			}
		}
	}
}

std::vector<Scene> SceneCluster::SplitSceneCommunityDetection(std::vector<IIndexArr>* outLocalToGlobal)
{
	const IIndex nViews = scene.images.size();
	IIndexArr nodes(0u, nViews);
	for (IIndex i = 0; i < nViews; ++i)
		nodes.push_back(i);

	// Detect natural communities, then bound each by the cluster capacity
	const std::vector<IIndexArr> communities = DetectCommunities(nodes, 1.f);
	std::vector<IIndexArr> clusters;
	clusters.reserve(communities.size());
	for (const IIndexArr& community : communities)
		SplitOversizedCommunity(community, 2.f, clusters);
	VERBOSE("Community detection: %u communities -> %u capacity-bounded atoms",
		(unsigned)communities.size(), (unsigned)clusters.size());

	// Pack communities into clusters up to capacity; the coupling acceptance test
	// keeps weakly-coupled communities as separate sub-scenes
	GreedyMergeClusters(clusters, false);

	RefineClustersLocalSearch(clusters);
	MergeSmallClusters(clusters);
	RefineClustersBalance(clusters);
	RefineClustersSplitDisconnected(clusters);
	RefineClustersSplitThinWaist(clusters);
	RefineClustersRescueOrphans(clusters);

	return BuildSubScenesFromClusters(clusters, outLocalToGlobal);
}

std::vector<IIndexArr> SceneCluster::DetectCommunities(const IIndexArr& nodes, float gamma) const
{
	// Atom-level graph, one atom per node; selfw tracks 2x the internal weight of
	// each aggregated atom so modularity stays exact across aggregation rounds
	std::vector<IIndexArr> atoms;
	atoms.reserve(nodes.size());
	std::unordered_map<IIndex, int> nodeToAtom;
	nodeToAtom.reserve(nodes.size());
	for (IIndex u : nodes) {
		nodeToAtom.emplace(u, (int)atoms.size());
		IIndexArr atom;
		atom.push_back(u);
		atoms.emplace_back(std::move(atom));
	}
	std::vector<std::map<int, double>> adjw(atoms.size());
	std::vector<double> selfw(atoms.size(), 0);
	for (IIndex u : nodes) {
		const int au = nodeToAtom[u];
		for (int i = xadj[u]; i < xadj[u+1]; ++i) {
			const auto it = nodeToAtom.find(adjncy[i]);
			if (it != nodeToAtom.end() && it->second != au)
				adjw[au][it->second] += adjwgt[i];
		}
	}

	for (;;) {
		const size_t nAtoms = atoms.size();
		// weighted degree per atom (internal weight counts fully)
		std::vector<double> k(nAtoms);
		double twoM = 0;
		for (size_t a = 0; a < nAtoms; ++a) {
			double s = selfw[a];
			for (const auto& p : adjw[a])
				s += p.second;
			k[a] = s;
			twoM += s;
		}
		if (twoM <= 0)
			break;

		// local moving phase (deterministic: ascending atom order, ordered maps)
		std::vector<int> comm(nAtoms);
		for (size_t a = 0; a < nAtoms; ++a)
			comm[a] = (int)a;
		std::vector<double> sigma = k;
		bool movedAny = false;
		for (unsigned iter = 0; iter < 30; ++iter) {
			bool moved = false;
			for (size_t a = 0; a < nAtoms; ++a) {
				std::map<int, double> wc;
				for (const auto& p : adjw[a])
					wc[comm[p.first]] += p.second;
				const int ca = comm[a];
				sigma[ca] -= k[a];
				int bestC = ca;
				const auto itSelf = wc.find(ca);
				double bestGain = (itSelf != wc.end() ? itSelf->second : 0.0) - gamma * k[a] * sigma[ca] / twoM;
				for (const auto& p : wc) {
					if (p.first == ca) continue;
					const double gain = p.second - gamma * k[a] * sigma[p.first] / twoM;
					if (gain > bestGain + 1e-9) {
						bestGain = gain;
						bestC = p.first;
					}
				}
				sigma[bestC] += k[a];
				if (bestC != ca) {
					comm[a] = bestC;
					moved = movedAny = true;
				}
			}
			if (!moved)
				break;
		}

		std::map<int, std::vector<int>> groups;
		for (size_t a = 0; a < nAtoms; ++a)
			groups[comm[a]].push_back((int)a);
		if (!movedAny || groups.size() == nAtoms)
			break;

		// aggregation phase
		std::vector<int> remap(nAtoms);
		{
			int i = 0;
			for (const auto& g : groups) {
				for (int a : g.second)
					remap[a] = i;
				++i;
			}
		}
		std::vector<IIndexArr> newAtoms(groups.size());
		std::vector<std::map<int, double>> newAdjw(groups.size());
		std::vector<double> newSelfw(groups.size(), 0);
		{
			int i = 0;
			for (const auto& g : groups) {
				for (int a : g.second) {
					for (IIndex u : atoms[a])
						newAtoms[i].push_back(u);
					newSelfw[i] += selfw[a];
					for (const auto& p : adjw[a]) {
						const int j = remap[p.first];
						if (j == i)
							newSelfw[i] += p.second; // internal edge, seen from both sides
						else
							newAdjw[i][j] += p.second;
					}
				}
				++i;
			}
		}
		atoms = std::move(newAtoms);
		adjw = std::move(newAdjw);
		selfw = std::move(newSelfw);
	}

	for (IIndexArr& atom : atoms)
		atom.Sort();
	return atoms;
}

void SceneCluster::SplitOversizedCommunity(const IIndexArr& community, float gamma, std::vector<IIndexArr>& out) const
{
	if (community.size() <= config.maxViewsPerCluster) {
		out.push_back(community);
		return;
	}
	if (gamma <= 64.f) {
		const std::vector<IIndexArr> parts = DetectCommunities(community, gamma);
		if (parts.size() > 1) {
			for (const IIndexArr& part : parts)
				SplitOversizedCommunity(part, gamma * 2, out);
			return;
		}
		SplitOversizedCommunity(community, gamma * 2, out);
		return;
	}
	// fully dense community that resists splitting: any cut is acceptable, halve it
	const IIndex half = community.size() / 2;
	IIndexArr a, b;
	FOREACH(i, community)
		(i < half ? a : b).push_back(community[i]);
	SplitOversizedCommunity(a, gamma, out);
	SplitOversizedCommunity(b, gamma, out);
}

void SceneCluster::MergeSmallClusters(std::vector<IIndexArr>& clusters)
{
	const IIndex nViews = scene.images.size();
	std::vector<int> nodeToCluster(nViews, -1);
	for (size_t c = 0; c < clusters.size(); ++c) {
		for (IIndex u : clusters[c]) {
			nodeToCluster[u] = (int)c;
		}
	}

	bool changed = true;
	while (changed) {
		changed = false;
		for (size_t c = 0; c < clusters.size(); ++c) {
			if (clusters[c].size() == 0 || clusters[c].size() >= config.minViewsPerCluster) continue;

			std::unordered_map<int, int> cluster_weights;
			for (IIndex u : clusters[c]) {
				for (int i = xadj[u]; i < xadj[u+1]; ++i) {
					int v = adjncy[i];
					int target_c = nodeToCluster[v];
					if (target_c != (int)c && target_c != -1) {
						cluster_weights[target_c] += adjwgt[i];
					}
				}
			}

			int best_target = -1;
			int max_weight = -1;
			for (const auto& p : cluster_weights) {
				if (p.second > max_weight) {
					if (clusters[p.first].size() + clusters[c].size() <= config.maxViewsPerCluster + config.maxOverCapacity) {
						max_weight = p.second;
						best_target = p.first;
					}
				}
			}

			if (best_target != -1) {
				for (IIndex u : clusters[c]) {
					nodeToCluster[u] = best_target;
				}
				for (IIndex val : clusters[c]) {
					clusters[best_target].push_back(val);
				}
				clusters[c].clear();
				changed = true;
			}
		}
	}

	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [](const IIndexArr& c) {
		return c.empty();
	}), clusters.end());
}

void SceneCluster::RefineClustersLocalSearch(std::vector<IIndexArr>& clusters)
{
	const IIndex nViews = scene.images.size();
	std::vector<int> nodeToCluster(nViews, -1);
	for (size_t c = 0; c < clusters.size(); ++c) {
		for (IIndex u : clusters[c]) {
			nodeToCluster[u] = (int)c;
		}
	}

	bool changed = true;
	int iters = 0;
	while (changed && iters < 20) {
		changed = false;
		iters++;
		for (IIndex u = 0; u < nViews; ++u) {
			int current_c = nodeToCluster[u];
			if (current_c == -1) continue;

			int best_target = current_c;
			int max_gain = 0;

			std::unordered_map<int, int> cluster_weights;
			for (int i = xadj[u]; i < xadj[u+1]; ++i) {
				int v = adjncy[i];
				int target_c = nodeToCluster[v];
				if (target_c != -1) {
					cluster_weights[target_c] += adjwgt[i];
				}
			}

			int current_internal_weight = cluster_weights[current_c];

			for (const auto& p : cluster_weights) {
				int target_c = p.first;
				int weight_to_target = p.second;
				if (target_c == current_c) continue;
				if (clusters[target_c].size() < config.maxViewsPerCluster) {
					int gain = weight_to_target - current_internal_weight;
					if (gain > max_gain) {
						max_gain = gain;
						best_target = target_c;
					}
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

	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [](const IIndexArr& c) {
		return c.empty();
	}), clusters.end());
}

// Move well-connected boundary images out of the largest cluster into smaller
// neighbors. Sub-scenes reconstruct concurrently, so wall-clock time is
// dominated by the largest cluster; moving a modest number of strongly-shared
// images shortens that critical path. Conservative by construction: a move is
// only made when the candidate's connectivity to the target cluster is a
// large fraction of its connectivity to its own cluster, so weakly-coupled
// images (e.g. an isolated strip of views) never move.
void SceneCluster::RefineClustersBalance(std::vector<IIndexArr>& clusters)
{
	constexpr float kBalanceAffinity = 0.7f;   // min ratio of target-weight to current-internal-weight for a move
	constexpr float kBalanceTolerance = 1.25f; // stop when largest cluster <= tolerance * mean cluster size

	const IIndex nViews = scene.images.size();
	std::vector<int> nodeToCluster(nViews, -1);
	for (size_t c = 0; c < clusters.size(); ++c) {
		for (IIndex u : clusters[c]) {
			nodeToCluster[u] = (int)c;
		}
	}

	unsigned sizeBefore = 0;
	for (const IIndexArr& c : clusters)
		sizeBefore = MAXF(sizeBefore, (unsigned)c.size());

	const unsigned maxMoves = 2 * config.maxViewsPerCluster;
	unsigned numMoves = 0;
	while (numMoves < maxMoves) {
		// pick the largest active cluster (ties -> lowest index) and the mean size over non-empty clusters
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
		if (srcSize <= CEIL2INT<unsigned>(kBalanceTolerance * mean))
			break;
		if (srcSize <= config.minViewsPerCluster)
			break; // moving further would shrink the largest cluster below the minimum

		// scan every image in src (ascending order) for the best eligible move this sweep
		IIndex bestU = NO_ID;
		int bestC = -1;
		int bestWeight = 0;
		for (IIndex u = 0; u < nViews; ++u) {
			if (nodeToCluster[u] != src)
				continue;

			std::unordered_map<int, int> cluster_weights;
			int wSrc = 0;
			for (int i = xadj[u]; i < xadj[u+1]; ++i) {
				const int v = adjncy[i];
				const int target_c = nodeToCluster[v];
				if (target_c == src)
					wSrc += adjwgt[i];
				else if (target_c != -1)
					cluster_weights[target_c] += adjwgt[i];
			}
			if (wSrc == 0)
				continue;

			int targetC = -1;
			int targetW = 0;
			for (const auto& p : cluster_weights) {
				if ((unsigned)clusters[p.first].size() >= srcSize - 1) continue; // would not strictly reduce imbalance
				if ((unsigned)clusters[p.first].size() >= config.maxViewsPerCluster) continue;
				if ((unsigned)clusters[p.first].size() < config.minViewsPerCluster) continue; // not a viable sub-scene, moving into it cannot shorten the critical path
				if (p.second > targetW || (p.second == targetW && p.first < targetC)) {
					targetW = p.second;
					targetC = p.first;
				}
			}
			if (targetC < 0 || (float)targetW < kBalanceAffinity * (float)wSrc)
				continue;

			if (targetW > bestWeight) {
				bestWeight = targetW;
				bestU = u;
				bestC = targetC;
			}
		}

		if (bestU == NO_ID)
			break; // no eligible move this sweep

		auto it = std::find(clusters[src].begin(), clusters[src].end(), bestU);
		ASSERT(it != clusters[src].end());
		*it = clusters[src].back();
		clusters[src].pop_back();
		clusters[bestC].push_back(bestU);
		nodeToCluster[bestU] = bestC;
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
	std::vector<int> nodeToCluster(nViews, -1);
	for (size_t c = 0; c < clusters.size(); ++c) {
		for (IIndex u : clusters[c]) {
			nodeToCluster[u] = (int)c;
		}
	}

	std::vector<IIndexArr> new_clusters;
	for (size_t c = 0; c < clusters.size(); ++c) {
		if (clusters[c].empty()) continue;

		std::unordered_set<IIndex> remaining(clusters[c].begin(), clusters[c].end());
		bool first = true;
		while (!remaining.empty()) {
			IIndex start_node = *remaining.begin();
			IIndexArr component;
			std::queue<IIndex> q;
			q.push(start_node);
			remaining.erase(start_node);
			while (!q.empty()) {
				IIndex u = q.front();
				q.pop();
				component.push_back(u);
				for (int i = xadj[u]; i < xadj[u+1]; ++i) {
					IIndex v = adjncy[i];
					if (nodeToCluster[v] == (int)c && remaining.count(v)) {
						q.push(v);
						remaining.erase(v);
					}
				}
			}
			if (first) {
				clusters[c] = component;
				first = false;
			} else {
				new_clusters.push_back(component);
			}
		}
	}
	if (!new_clusters.empty()) {
		clusters.insert(clusters.end(), new_clusters.begin(), new_clusters.end());
	}
}

void SceneCluster::RefineClustersSplitThinWaist(std::vector<IIndexArr>& clusters)
{
	if (config.minClusterCoupling <= 0)
		return;
	// bucket every cluster's internal edges in a single sweep over the pair graph;
	// after a split each half's edges are a filter of the parent's bucket (a parent
	// edge is either internal to a half or part of the cut), so the pair graph is
	// never scanned again
	std::vector<std::vector<std::pair<uint32_t, uint32_t>>> edges(clusters.size());
	std::vector<std::vector<double>> weights(clusters.size());
	{
		std::vector<int> clusterOf(scene.images.size(), -1);
		std::vector<uint32_t> localIndex(scene.images.size(), 0);
		for (size_t c = 0; c < clusters.size(); ++c) {
			uint32_t li = 0;
			for (IIndex g : clusters[c]) {
				clusterOf[g] = (int)c;
				localIndex[g] = li++;
			}
		}
		BucketClusterEdges(scene, config.minPairWeight, clusterOf, localIndex, edges, weights);
	}
	// a split appends the second half for later re-analysis and leaves the first half
	// in place to be re-analysed immediately; each split strictly shrinks a cluster and
	// both halves are >= minViewsPerCluster, so the process is naturally bounded
	unsigned budget = (unsigned)clusters.size();
	for (size_t c = 0; c < clusters.size() && budget > 0; ) {
		const IIndexArr& cluster = clusters[c];
		const unsigned V = (unsigned)cluster.size();
		if (V < 2 * config.minViewsPerCluster) { // cannot yield two keepable halves
			++c;
			continue;
		}
		const ClusterCoupling cc = AnalyzeClusterCoupling(V, edges[c], weights[c],
			MAXF(config.minViewsPerCluster, V / 5), config.minClusterCoupling);
		if (!cc.connected || !cc.weak || cc.side.size() != V ||
			cc.smaller < config.minViewsPerCluster) {
			++c;
			continue;
		}
		IIndexArr halves[2];
		std::vector<uint32_t> newLocal(V);
		FOREACH(i, cluster) {
			newLocal[i] = (uint32_t)halves[cc.side[i]].size();
			halves[cc.side[i]].push_back(cluster[i]);
		}
		std::vector<std::pair<uint32_t, uint32_t>> halfEdges[2];
		std::vector<double> halfWeights[2];
		for (size_t e = 0; e < edges[c].size(); ++e) {
			const auto [u, v] = edges[c][e];
			if (cc.side[u] != cc.side[v])
				continue; // a cut edge belongs to neither half
			const int s = cc.side[u];
			halfEdges[s].emplace_back(newLocal[u], newLocal[v]);
			halfWeights[s].push_back(weights[c][e]);
		}
		VERBOSE("Clustering: split thin-waist cluster (%u images, internal coupling %.3f < %.3f) into %u+%u images",
			V, cc.coupling, config.minClusterCoupling, (unsigned)halves[0].size(), (unsigned)halves[1].size());
		clusters[c] = std::move(halves[0]);
		edges[c] = std::move(halfEdges[0]);
		weights[c] = std::move(halfWeights[0]);
		clusters.push_back(std::move(halves[1]));
		edges.push_back(std::move(halfEdges[1]));
		weights.push_back(std::move(halfWeights[1]));
		--budget;
		// leave c in place: re-analyse the replacing half; the appended half is reached later
	}
}

void SceneCluster::RefineClustersRescueOrphans(std::vector<IIndexArr>& clusters)
{
	const IIndex nViews = scene.images.size();
	std::vector<int> nodeToCluster(nViews, -1);
	for (size_t c = 0; c < clusters.size(); ++c) {
		for (IIndex u : clusters[c]) {
			nodeToCluster[u] = (int)c;
		}
	}

	for (size_t c = 0; c < clusters.size(); ++c) {
		if (clusters[c].empty() || clusters[c].size() >= config.minViewsPerCluster) continue;

		// This cluster is still too small, try to reassign its nodes individually
		IIndexArr nodes = clusters[c];
		clusters[c].clear();
		for (IIndex u : nodes) {
			std::unordered_map<int, int> cluster_weights;
			for (int i = xadj[u]; i < xadj[u+1]; ++i) {
				int v = adjncy[i];
				int target_c = nodeToCluster[v];
				if (target_c != -1 && target_c != (int)c) {
					cluster_weights[target_c] += adjwgt[i];
				}
			}

			int best_target = -1;
			int max_weight = -1;
			for (const auto& p : cluster_weights) {
				if (p.second > max_weight) {
					if (clusters[p.first].size() < config.maxViewsPerCluster + config.maxOverCapacity) {
						max_weight = p.second;
						best_target = p.first;
					}
				}
			}

			if (best_target != -1) {
				nodeToCluster[u] = best_target;
				clusters[best_target].push_back(u);
			} else {
				// No cluster is connected to this image: keep it out of the sub-scenes
				// (its cluster stays small and is skipped); the image remains in the
				// global scene and is registered by the post-merge resection instead
				clusters[c].push_back(u);
			}
		}
	}

	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [](const IIndexArr& c) {
		return c.empty();
	}), clusters.end());
}

std::vector<Scene> SceneCluster::BuildSubScenesFromClusters(
	std::vector<IIndexArr>& clusters,
	std::vector<IIndexArr>* outLocalToGlobal)
{
	IIndex nSkippedViews = 0;
	std::vector<Scene> subScenes;
	subScenes.reserve(clusters.size());
	if (outLocalToGlobal)
		outLocalToGlobal->reserve(clusters.size());

	const unsigned nClusters = (unsigned)clusters.size();
	const unsigned nThreadsPerCluster = MAXF(1u, scene.nMaxThreads / MAXF(nClusters, 1u));
	DEBUG_EXTRA("Allocating %u threads per sub-scene (%u clusters, %u parent threads)",
		nThreadsPerCluster, nClusters, scene.nMaxThreads);

	// verify the clusters are well connected before reconstruction; the internal
	// covisibility graph still holds every intra-cluster pair (ExtractSubScene moves
	// them out below), so this must run before the extraction loop
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		ReportClusterCoupling(scene, clusters, config);
	#endif

	for (IIndexArr& cluster : clusters) {
		// Sort by global ID so that local IDs preserve global ordering:
		// localID1 < localID2 means also globalID1 < globalID2
		// so pair ID ordering (ID1 < ID2) is maintained through local-global remapping
		cluster.Sort();
		if (cluster.size() < config.minViewsPerCluster) {
			DEBUG("warning: skipping small cluster with %u views", (unsigned)cluster.size());
			nSkippedViews += cluster.size();
			continue;
		}
		IIndexArr globalToLocal(scene.images.size());
		globalToLocal.MemsetValue(NO_ID);
		FOREACH(localID, cluster)
			globalToLocal[cluster[localID]] = localID;
		subScenes.emplace_back(ExtractSubScene(cluster, globalToLocal, nThreadsPerCluster));
		if (outLocalToGlobal)
			outLocalToGlobal->emplace_back(std::move(cluster));
	}
	DEBUG("Clustering: split into %u sub-scenes and %u skipped views, %u cross-sub-scene pairs remain",
		(unsigned)subScenes.size(), nSkippedViews, scene.pairs.size());
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2 && !subScenes.empty() && !subScenes[0].images.empty() && subScenes[0].images[0].View::metadata.HasGPS())
		ExportClusterPositions(subScenes, MAKE_PATH(String("clusters_gps.ply")));
	#endif
	return subScenes;
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

	// Define vertex structure for PLY export
	struct Vertex {
		Point3f p; // GPS position (longitude, latitude, altitude)
		Pixel8U c; // color (cluster ID)
	};
	// Define PLY properties
	static const PLY::PlyProperty props[] = {
		{"x",     PLY::Float32, PLY::Float32, offsetof(Vertex, p.x), 0, 0, 0, 0},
		{"y",     PLY::Float32, PLY::Float32, offsetof(Vertex, p.y), 0, 0, 0, 0},
		{"z",     PLY::Float32, PLY::Float32, offsetof(Vertex, p.z), 0, 0, 0, 0},
		{"red",   PLY::Uint8,   PLY::Uint8,   offsetof(Vertex, c.r), 0, 0, 0, 0},
		{"green", PLY::Uint8,   PLY::Uint8,   offsetof(Vertex, c.g), 0, 0, 0, 0},
		{"blue",  PLY::Uint8,   PLY::Uint8,   offsetof(Vertex, c.b), 0, 0, 0, 0}
	};
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex"
	};

	// Create PLY file
	PLY ply;
	if (!ply.write(fileName, 1, elem_names, PLY::BINARY_LE))
		return false;
	ply.describe_property("vertex", 6, props);
	ply.element_count("vertex", ecefPositions.size());
	if (!ply.header_complete())
		return false;

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

	// Write vertices
	unsigned vertexCount = 0;
	Vertex vertex;
	FOREACH(clusterID, subScenes) {
		const Scene& scene = subScenes[clusterID];
		const Pixel8U clusterColor = GenerateClusterColor(clusterID, subScenes.size());
		for (const Image& img : scene.images) {
			if (!img.View::metadata.HasGPS())
				continue;
			const Point3d& ecef = ecefPositions[vertexCount++];
			// Convert ECEF to ENU (centered at centroid)
			double e, n, u;
			ECEFToENU(ecef.x, ecef.y, ecef.z, centerECEF.x, centerECEF.y, centerECEF.z, lat0, lon0, e, n, u);
			// Store ENU position (east, north, up)
			vertex.p.x = static_cast<float>(e);
			vertex.p.y = static_cast<float>(n);
			vertex.p.z = static_cast<float>(u);
			vertex.c = clusterColor;
			ply.put_element(&vertex);
		}
	}

	VERBOSE("Exported %u GPS positions corresponding to %u clusters to '%s'",
		(unsigned)ecefPositions.size(), (unsigned)subScenes.size(), fileName.c_str());
	return true;
}
/*----------------------------------------------------------------*/
