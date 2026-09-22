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
// The greedy merge holds every merge to the same test: it refuses to join two
// clusters across an interface carrying less than minClusterCoupling of the weaker
// side's internal weight, so that no sub-scene contains two blocks joined only by a
// sparse seam (such a seam lets scale drift accumulate unobserved and reconstructs
// as two independently scaled blocks — the two-scale bug the merge-time split then
// has to repair).
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
// as BuildSubScenesFromClusters numbers them (small clusters skipped unless the caller keeps
// them), so a flag here lines up with the merge-time telemetry of the same sub-scene. This is
// the health check for the two-scale defect: the merge stage aligns the sub-scenes to each
// other but cannot tell whether one of them reconstructed at two scales, so the invariant that
// no sub-scene holds two blocks joined by a seam too sparse to observe their relative scale is
// verified here, structurally, on the graph that clustering produced.
void ReportClusterCoupling(const Scene& scene, const std::vector<IIndexArr>& clusters, const ClusterConfig& config,
	bool skipSmallClusters)
{
	std::vector<int> clusterOf(scene.images.size(), -1);
	std::vector<uint32_t> localIndex(scene.images.size(), 0);
	std::vector<unsigned> clusterSize;
	int subSceneIdx = 0;
	for (const IIndexArr& cluster : clusters) {
		if (skipSmallClusters && cluster.size() < config.minViewsPerCluster)
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
		// Move expensive data from global to sub-scene to save memory, described-keypoint boundary
		// included: it indexes the keypoint array, so it has to follow it rather than stay behind
		// on an image that no longer has one
		subImg.MoveFeaturesFrom(img);
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

	// the seams are measured on the tracks, so a scene that arrives with only matches gets them
	// here; every sub-scene rebuilds its own tracks after the split
	if (scene.tracks.empty())
		BuildTracks(scene, config.minPairWeight, config.trackConflictCfg);

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
	RefineClustersForSeams(clusters);
	RefineClustersRescueOrphans(clusters);

	return BuildSubScenesFromClusters(clusters, outLocalToGlobal);
}

// Merge clusters bottom-up: repeatedly join the two clusters connected by the
// highest aggregate edge weight, subject to the capacity limit and the coupling
// acceptance test. The aggregate weight between two communities grows with their
// sizes, so many individually weak pairs eventually top the queue even when they
// represent only a few percent of either side's internal cohesion; reconstructing
// across such a sparse interface lets scale drift accumulate unobserved, hence
// every merge is held to at least minClusterCoupling of the weaker side's internal
// weight, whatever either side's size. The target-size rule below has one
// exception: a cluster already at the target still takes in a cluster under the
// floor, because that cluster has to go somewhere; without the same coupling bar
// on that merge, the community would be handed to whichever cluster with room
// happened to top the queue, over an interface far thinner than its own cohesion —
// the coupling test has to hold for exactly the merge the size rule exempts. A
// singleton has no internal weight, so it always passes. Any thin seam that still
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
		// a cluster grows to the target and stops there; the one merge that takes it past the target
		// is the one absorbing a cluster under the floor, which has to go somewhere
		if (clusters[u].size() + clusters[v].size() > config.targetViewsPerCluster &&
			(MINF(clusters[u].size(), clusters[v].size()) >= config.minViewsPerCluster ||
			 MAXF(clusters[u].size(), clusters[v].size()) > config.targetViewsPerCluster)) continue;
		// the coupling test holds for every merge, a community under the floor included: it is
		// held to its own cohesion, so a singleton (no internal weight) joins freely while a tight
		// small community is not absorbed over a thin interface by whichever cluster happens to
		// have room, and MergeSmallClusters later places it with the neighbour it shares the most
		// weight with
		if (config.minClusterCoupling > 0 &&
			(float)e.weight < config.minClusterCoupling * (float)MINF(wint[u], wint[v]))
			continue; // two communities joined only by a sparse interface

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
	RefineClustersForSeams(clusters);
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

// A cluster under the floor that no neighbour is coupled to -- its heaviest interface carries less
// than minClusterCoupling of its own internal weight, the test every greedy merge refused it by --
// yet that the merge could place, having the strong seams IsStrongSeam asks, as many as its track
// neighbours allow (the bar MergeLeafClusters holds every cluster above the floor to). Absorbed into
// the neighbour it shares the most weight with, as the small-cluster pass does with every cluster
// under the floor, such a community lands inside a block whose reconstruction starts elsewhere and
// reaches it only through single resections across that thin interface, which the evidence share
// the resection asks of every pose refuses one by one: on alameda a 40-image loop-closure community,
// sparse-rich inside and joined to the rest by 1.9% of its internal weight, stayed unregistered to
// the end that way. Kept as its own sub-scene it reconstructs on its own cohesion and enters through
// the merge's placement of the whole block over every seam track at once. A community the merge
// could not place either is absorbed as before: a block left unplaced would only hand the same
// images to the whole-scene resection.
bool SceneCluster::IsStandaloneCommunity(const std::vector<IIndexArr>& clusters, size_t c, const SeamTrackStatsMap& seamStats) const
{
	ASSERT(c < clusters.size() && !clusters[c].empty());
	if (config.minClusterCoupling <= 0.f || clusters[c].size() >= config.minViewsPerCluster)
		return false;
	const std::vector<int> nodeToCluster = MapNodesToClusters(clusters, scene.images.size());
	int64_t internal = 0;
	std::unordered_map<int, int64_t> interface;
	for (IIndex u : clusters[c]) {
		for (int i = xadj[u]; i < xadj[u+1]; ++i) {
			const int cv = nodeToCluster[adjncy[i]];
			if (cv == (int)c)
				internal += adjwgt[i]; // each internal edge seen from both endpoints
			else if (cv >= 0)
				interface[cv] += adjwgt[i];
		}
	}
	internal /= 2;
	int64_t heaviest = 0;
	for (const auto& p : interface)
		heaviest = MAXF(heaviest, p.second);
	if ((float)heaviest >= config.minClusterCoupling * (float)internal)
		return false; // coupled to a neighbour: the small-cluster pass places it there
	unsigned numStrong = 0, numAdjacent = 0;
	for (const auto& [pair, seam] : seamStats) {
		if (pair.first != (uint32_t)c && pair.second != (uint32_t)c)
			continue;
		++numAdjacent; // shares at least one seam-usable track with that cluster
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
		const SeamTrackStatsMap seamStats = ComputeSeamTrackStats(clusters);
		for (size_t c = 0; c < clusters.size(); ++c) {
			if (clusters[c].size() == 0 || clusters[c].size() >= config.minViewsPerCluster) continue;
			if (IsStandaloneCommunity(clusters, c, seamStats)) continue; // its own sub-scene, placed by the merge

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
				AbsorbCluster(clusters, c, (size_t)best_target, &nodeToCluster);
				changed = true;
			}
		}
	}

	EraseEmptyClusters(clusters);
}

SeamTrackStatsMap SceneCluster::ComputeSeamTrackStats(const std::vector<IIndexArr>& clusters) const
{
	std::vector<uint32_t> clusterOf(scene.images.size(), NO_ID);
	for (size_t c = 0; c < clusters.size(); ++c)
		for (IIndex u : clusters[c])
			clusterOf[u] = (uint32_t)c;

	SeamTrackStatsMap stats;
	std::map<uint32_t, unsigned> numObservations; // per cluster, for the track at hand
	for (const Track& track : scene.tracks) {
		if (!track.IsValid())
			continue;
		numObservations.clear();
		for (const Observation& obs : track.observations) {
			ASSERT(obs.imageID < clusterOf.size());
			if (clusterOf[obs.imageID] != NO_ID)
				++numObservations[clusterOf[obs.imageID]];
		}
		if (numObservations.size() < 2)
			continue; // the track stays inside one cluster
		// most tracks touch two clusters, so this is a pair or two per track
		for (auto x = numObservations.cbegin(); x != numObservations.cend(); ++x) {
			for (auto y = std::next(x); y != numObservations.cend(); ++y) {
				if (x->second < 2 && y->second < 2)
					continue; // neither side can triangulate it, so neither can pose the other
				std::pair<SeamTrackStats, SeamTrackStats>& seam = stats[std::make_pair(x->first, y->first)];
				++seam.first.usable;
				++seam.second.usable;
				for (const Observation& obs : track.observations) {
					const uint32_t c = clusterOf[obs.imageID];
					if (c == x->first)
						++seam.first.perCamera[obs.imageID];
					else if (c == y->first)
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

void SceneCluster::RepairClusterSeams(std::vector<IIndexArr>& clusters)
{
	// the seams to repair are collected first: repairing one moves images between its two clusters,
	// which the statistics of the seams sharing a cluster with it would not survive
	std::vector<std::pair<uint32_t, uint32_t>> thinSeams;
	for (const auto& [pair, seam] : ComputeSeamTrackStats(clusters)) {
		if (seam.first.usable < config.minSeamTracks)
			continue; // too little material to spread over more cameras
		if (seam.first.CamerasWithAtLeast(config.minSeamCameraTracks) >= config.minSeamCameras &&
			seam.second.CamerasWithAtLeast(config.minSeamCameraTracks) >= config.minSeamCameras)
			continue;
		thinSeams.push_back(pair);
	}
	for (const auto& [clusterA, clusterB] : thinSeams)
		RepairClusterSeam(clusters, clusterA, clusterB);
}

// One seam that carries usable tracks enough but holds them in too few cameras: the merge poses a
// camera only on a vote of its own, so such a seam registers on one or two cameras and is thrown
// out. Moving a boundary image across the cut hands the tracks it sees to the other side, where
// they now cross the cut, which spreads the seam over more cameras on both sides; the move that
// raises the weaker side the most is taken, while the two clusters stay within their size bounds.
void SceneCluster::RepairClusterSeam(std::vector<IIndexArr>& clusters, uint32_t clusterA, uint32_t clusterB)
{
	const IIndex nViews = scene.images.size();
	std::vector<uint8_t> side(nViews, 2); // 0 = first cluster, 1 = second, 2 = neither
	for (IIndex u : clusters[clusterA])
		side[u] = 0;
	for (IIndex u : clusters[clusterB])
		side[u] = 1;

	// the boundary images: the ones with a covisibility pair crossing the seam
	IIndexArr candidates;
	std::vector<bool> isCandidate(nViews, false);
	for (const ImagePair& pair : scene.pairs) {
		if (pair.GetCompositeWeight() < config.minPairWeight ||
			side[pair.ID1] > 1 || side[pair.ID2] > 1 || side[pair.ID1] == side[pair.ID2])
			continue;
		for (IIndex u : {pair.ID1, pair.ID2}) {
			if (!isCandidate[u]) {
				isCandidate[u] = true;
				candidates.push_back(u);
			}
		}
	}
	if (candidates.empty())
		return;
	candidates.Sort(); // the same seam is repaired the same way whatever order the pairs came in

	// the tracks each candidate takes part in: moving it changes the seam through those alone
	std::unordered_map<IIndex, std::vector<uint32_t>> tracksOf;
	for (IIndex u : candidates)
		tracksOf.emplace(u, std::vector<uint32_t>());
	FOREACH(t, scene.tracks) {
		for (const Observation& obs : scene.tracks[t].observations) {
			const auto it = tracksOf.find(obs.imageID);
			if (it != tracksOf.end())
				it->second.push_back(t);
		}
	}

	// What one track contributes to the seam under the current sides, added or taken back. Both the
	// count below and the per-move recount go through here, so they read the same tracks and a take
	// back can only ever undo an add of its own.
	const auto ApplyTrack = [&side](std::pair<SeamTrackStats, SeamTrackStats>& seam, const Track& track, bool add) {
		if (!track.IsValid())
			return;
		const auto Bump = [add](unsigned& value) { if (add) ++value; else --value; };
		unsigned numObservations[2] = {0, 0};
		for (const Observation& obs : track.observations)
			if (side[obs.imageID] < 2)
				++numObservations[side[obs.imageID]];
		if (numObservations[0] == 0 || numObservations[1] == 0 ||
			MAXF(numObservations[0], numObservations[1]) < 2)
			return;
		Bump(seam.first.usable);
		Bump(seam.second.usable);
		for (const Observation& obs : track.observations) {
			if (side[obs.imageID] == 0)
				Bump(seam.first.perCamera[obs.imageID]);
			else if (side[obs.imageID] == 1)
				Bump(seam.second.perCamera[obs.imageID]);
		}
	};
	std::pair<SeamTrackStats, SeamTrackStats> seam;
	for (const Track& track : scene.tracks)
		ApplyTrack(seam, track, true);

	// the seam after moving one image to the other side, its own tracks the only ones re-read
	const auto StatsAfterMove = [&](IIndex u) {
		std::pair<SeamTrackStats, SeamTrackStats> moved = seam;
		for (uint32_t t : tracksOf[u])
			ApplyTrack(moved, scene.tracks[t], false);
		side[u] ^= 1;
		for (uint32_t t : tracksOf[u])
			ApplyTrack(moved, scene.tracks[t], true);
		side[u] ^= 1;
		return moved;
	};
	const auto SeamCameras = [this](const SeamTrackStats& stats) {
		return stats.CamerasWithAtLeast(config.minSeamCameraTracks);
	};

	const unsigned camerasBefore[2] = {SeamCameras(seam.first), SeamCameras(seam.second)};
	const unsigned maxMoves = config.maxViewsPerCluster / 10;
	unsigned numMoves = 0;
	while (numMoves < maxMoves) {
		const unsigned cameras[2] = {SeamCameras(seam.first), SeamCameras(seam.second)};
		if (cameras[0] >= config.minSeamCameras && cameras[1] >= config.minSeamCameras)
			break; // the seam carries the merge now
		IIndex bestImage = NO_ID;
		unsigned bestObjective = MINF(cameras[0], cameras[1]);
		std::pair<SeamTrackStats, SeamTrackStats> bestSeam;
		for (IIndex u : candidates) {
			const IIndexArr& from = clusters[side[u] == 0 ? clusterA : clusterB];
			const IIndexArr& into = clusters[side[u] == 0 ? clusterB : clusterA];
			if (from.size() <= config.minViewsPerCluster || into.size() >= config.maxViewsPerCluster)
				continue; // the move would push one of the two out of its size bounds
			std::pair<SeamTrackStats, SeamTrackStats> moved = StatsAfterMove(u);
			const unsigned objective = MINF(SeamCameras(moved.first), SeamCameras(moved.second));
			if (objective > bestObjective) {
				bestObjective = objective;
				bestImage = u;
				bestSeam = std::move(moved);
			}
		}
		if (bestImage == NO_ID)
			break; // no move spreads the seam wider
		IIndexArr& from = clusters[side[bestImage] == 0 ? clusterA : clusterB];
		IIndexArr& into = clusters[side[bestImage] == 0 ? clusterB : clusterA];
		const auto it = std::find(from.begin(), from.end(), bestImage);
		ASSERT(it != from.end());
		*it = from.back();
		from.pop_back();
		into.push_back(bestImage);
		side[bestImage] ^= 1;
		seam = std::move(bestSeam);
		++numMoves;
	}
	if (numMoves == 0)
		return;
	VERBOSE("Cluster pair (%u, %u): %u boundary images moved; seam cameras %u/%u -> %u/%u",
		clusterA, clusterB, numMoves, camerasBefore[0], camerasBefore[1],
		SeamCameras(seam.first), SeamCameras(seam.second));
}

void SceneCluster::RefineClustersForSeams(std::vector<IIndexArr>& clusters)
{
	MergeLeafClusters(clusters);
	RepairClusterSeams(clusters);
	// both passes above move whole clusters and single images around, and either can leave a part
	// under the floor; the floor rule takes them the same way it takes any other small cluster
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
	// a cluster takes in a moved image only while it is under the size every cluster aims at; the
	// ceiling still bounds it when a configuration puts the aim above the ceiling
	const unsigned sizeLimit = MINF(config.targetViewsPerCluster, config.maxViewsPerCluster);
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
				if (clusters[target_c].size() < sizeLimit) {
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
	const SeamTrackStatsMap seamStats = ComputeSeamTrackStats(clusters);

	for (size_t c = 0; c < clusters.size(); ++c) {
		if (clusters[c].empty() || clusters[c].size() >= config.minViewsPerCluster) continue;
		if (IsStandaloneCommunity(clusters, c, seamStats)) continue; // stands alone: not dismantled

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

std::vector<Scene> SceneCluster::SplitSceneByClusters(
	const std::vector<IIndexArr>& clusters,
	std::vector<IIndexArr>* outLocalToGlobal)
{
	// the clusters are given, so there is nothing to partition and no connectivity graph to build
	std::vector<IIndexArr> given(clusters);
	return BuildSubScenesFromClusters(given, outLocalToGlobal, false);
}

std::vector<Scene> SceneCluster::BuildSubScenesFromClusters(
	std::vector<IIndexArr>& clusters,
	std::vector<IIndexArr>* outLocalToGlobal,
	const bool skipSmallClusters)
{
	IIndex nSkippedViews = 0;
	std::vector<Scene> subScenes;
	subScenes.reserve(clusters.size());
	if (outLocalToGlobal)
		outLocalToGlobal->reserve(clusters.size());

	// a cluster under the floor is skipped, except a community that stands alone (see IsStandaloneCommunity)
	const SeamTrackStatsMap seamStats = skipSmallClusters ? ComputeSeamTrackStats(clusters) : SeamTrackStatsMap();
	const auto IsSubScene = [&](size_t c) {
		return !skipSmallClusters || clusters[c].size() >= config.minViewsPerCluster || IsStandaloneCommunity(clusters, c, seamStats);
	};

	// the budget is shared by the clusters that will actually become sub-scenes, so counting them
	// first: a clustering yielding one real cluster and a crowd of skipped singletons must not
	// leave the one reconstruction that runs with a single thread
	unsigned nClusters = 0;
	for (size_t c = 0; c < clusters.size(); ++c) {
		if (clusters[c].empty()) continue;
		if (IsSubScene(c)) ++nClusters;
	}
	const unsigned nThreadsPerCluster = MAXF(1u, scene.nMaxThreads / MAXF(nClusters, 1u));
	DEBUG_EXTRA("Allocating %u threads per sub-scene (%u clusters, %u parent threads)",
		nThreadsPerCluster, nClusters, scene.nMaxThreads);

	// verify the clusters are well connected before reconstruction; the internal
	// covisibility graph still holds every intra-cluster pair (ExtractSubScene moves
	// them out below), so this must run before the extraction loop
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		ReportClusterCoupling(scene, clusters, config, skipSmallClusters);
	#endif

	// which global images each sub-scene took, as ranges: the membership every later stage is
	// reported against, so a block that comes out wrong can be traced back to the split
	std::vector<String> memberships;
	for (size_t c = 0; c < clusters.size(); ++c) {
		if (clusters[c].empty()) continue;
		IIndexArr& cluster = clusters[c];
		// Sort by global ID so that local IDs preserve global ordering:
		// localID1 < localID2 means also globalID1 < globalID2
		// so pair ID ordering (ID1 < ID2) is maintained through local-global remapping
		cluster.Sort();
		if (!IsSubScene(c)) {
			DEBUG("warning: skipping small cluster with %u views", (unsigned)cluster.size());
			nSkippedViews += cluster.size();
			continue;
		}
		String membership;
		FOREACH(i, cluster) {
			if (i > 0 && cluster[i] == cluster[i-1] + 1)
				continue; // still inside the current range
			IIndex last = cluster[i];
			for (IIndex j = i + 1; j < cluster.size() && cluster[j] == last + 1; ++j)
				last = cluster[j];
			if (!membership.empty())
				membership += ", ";
			membership += last == cluster[i] ?
				String::FormatString("%u", cluster[i]) :
				String::FormatString("%u-%u", cluster[i], last);
		}
		memberships.emplace_back(String::FormatString("sub-scene %u: %s (%u images)",
			(unsigned)subScenes.size(), membership.c_str(), (unsigned)cluster.size()));
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
	for (const String& membership : memberships)
		DEBUG_ULTIMATE("Clustering: %s", membership.c_str());
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
