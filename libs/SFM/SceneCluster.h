/*
 * SceneCluster.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_SCENECLUSTER_H_
#define _SFM_SCENECLUSTER_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// forward declarations to avoid circular includes
class SFM_API Scene;

/*
 * Hierarchical SfM — Scene Partitioning (Split Phase)
 * ====================================================
 *
 * When the number of images in a scene exceeds a manageable threshold, the
 * reconstruction problem becomes both computationally expensive and numerically
 * fragile: large bundle adjustment problems converge slowly and are more prone
 * to local minima. Hierarchical SfM addresses this by splitting the scene into
 * smaller, overlapping sub-scenes that can be reconstructed independently and
 * later merged into a single global scene.
 *
 * This file implements the SPLIT phase. The MERGE phase is in GlobalAlignment.h.
 *
 * ── Pipeline overview (split) ──────────────────────────────────────────────
 *
 * 1. BUILD COVISIBILITY GRAPH
 *    Construct a weighted undirected graph where each node is an image and each
 *    edge weight encodes the number of geometrically verified feature matches
 *    between two images (from image pairs). This graph captures the visual
 *    overlap structure of the dataset.
 *
 * 2. AGGREGATIVE CLUSTERING
 *    Partition the graph using bottom-up (agglomerative) clustering:
 *    - Start with each image as its own cluster.
 *    - Repeatedly merge the two clusters connected by the highest-weight edge,
 *      updating edge weights between the merged cluster and its neighbors.
 *    - Stop when every cluster has ≤ maxViewsPerCluster images.
 *    This greedy approach produces clusters that respect the covisibility
 *    structure: images that see many of the same features end up together,
 *    ensuring each sub-scene has strong internal connectivity.
 *
 * 3. CLUSTER REFINEMENT
 *    Post-process the clusters to improve quality:
 *    a) Merge small clusters: clusters below minViewsPerCluster are absorbed
 *       into their most-connected neighbor (up to maxOverCapacity slack).
 *    b) Local search: iteratively move or swap boundary images between clusters
 *       to improve a modularity + balance objective.
 *    c) Split disconnected: if a cluster has disconnected components in the
 *       covisibility graph, split it into separate clusters.
 *    d) Rescue orphans: small clusters that remain after splitting are absorbed
 *       into neighbors.
 *
 * 4. EXTRACT SUB-SCENES
 *    For each cluster, create an independent Scene object:
 *    - Copy camera definitions (with local camera IDs).
 *    - Copy images (with local image IDs), MOVING keypoints and descriptors
 *      from the global scene to the sub-scene to save memory.
 *    - MOVE image pairs whose both images belong to this cluster into the
 *      sub-scene (remapping image IDs to local indices).
 *    - Image pairs that cross cluster boundaries (one image in this cluster,
 *      the other in a different cluster) are LEFT in the global scene. These
 *      cross-sub-scene pairs are used later by GlobalAlignment to establish
 *      connections between independently reconstructed sub-scenes.
 *
 *    The output is:
 *    - A vector of sub-scenes with local IDs [0, N), each self-contained with
 *      its own cameras, images (with keypoints), and intra-cluster pairs.
 *    - A parallel vector of localToGlobal mappings: localToGlobal[sceneIdx][localImgID] = globalImgID.
 *    - The global scene retains its image array (now with empty keypoints for
 *      assigned images) and only the cross-sub-scene pairs.
 *
 * ── Memory protocol ────────────────────────────────────────────────────────
 *
 * The split/merge protocol is designed to minimize peak memory:
 *
 *   Global scene (before split):
 *     images[]     → keypoints, descriptors populated
 *     pairs[]      → all image pairs with matches
 *
 *   Global scene (after split):
 *     images[]     → keypoints/descriptors MOVED OUT (empty for clustered images)
 *     pairs[]      → only cross-sub-scene pairs remain
 *
 *   Sub-scenes (after split):
 *     images[]     → keypoints/descriptors MOVED IN from global
 *     pairs[]      → only intra-cluster pairs (moved from global)
 *
 * During independent reconstruction of each sub-scene (BuildTracks →
 * StarInitializer → Resection → BundleAdjustment), only that sub-scene's
 * data is in memory. After reconstruction, GlobalAlignment::MergeSingleScene
 * moves everything back to the global scene.
 */

/**
 * @brief Configuration for scene clustering
 */
struct SFM_API ClusterConfig
{
	unsigned maxViewsPerCluster{200};  // maximum images per cluster (0 = disable clustering)
	unsigned minViewsPerCluster{10};   // minimum images per cluster to keep (smaller clusters merged/reassigned)
	unsigned maxOverCapacity{20};      // maximum extra images a cluster can take over maxViewsPerCluster when absorbing orphans
	float minPairWeight{3.f};          // minimum composite weight for pair edge
	float minClusterCoupling{0.05f};   // refuse a merge whose interface weight falls below this fraction of the weaker side's internal weight, and split any final cluster with such an internal seam (0 = disabled)
	bool useCommunityDetection{false}; // partition by community detection + capacity packing instead of pure aggregative clustering
};

/**
 * @brief Scene partitioning using aggregative graph clustering
 *
 * See the top-level comment in this file for the full hierarchical SfM
 * split-phase architecture and memory protocol.
 */
class SFM_API SceneCluster
{
public:
	/**
	 * @brief Constructor - initializes clustering with scene and config
	 * @param scene Input scene with all images
	 * @param config Clustering configuration
	 */
	SceneCluster(Scene& scene, const ClusterConfig& config);

	/**
	 * @brief Split scene into sub-scenes using graph partitioning
	 * @param outLocalToGlobal Optional output vector of ID mappings (parallel to returned scenes)
	 * @return Vector of sub-scenes with local IDs [0, N)
	 */
	std::vector<Scene> SplitScene(std::vector<IIndexArr>* outLocalToGlobal = NULL);

	/**
	 * @brief Export cluster GPS positions to PLY file with unique colors per cluster
	 * @param subScenes Vector of scene clusters
	 * @param fileName Output PLY file path
	 * @return True if successful, false otherwise
	 */
	static bool ExportClusterPositions(
		const std::vector<Scene>& subScenes,
		const String& fileName);

private:
	// Build METIS connectivity graph in CSR format
	void BuildConnectivityGraph();

	// Extract sub-scene from cluster assignment
	Scene ExtractSubScene(
		const IIndexArr& viewIndices,
		const IIndexArr& globalToLocal,
		unsigned nThreadsPerCluster);

	// Aggregative clustering (greedy max-weight)
	std::vector<Scene> SplitSceneAggregativeClustering(std::vector<IIndexArr>* outLocalToGlobal);

	// Community detection (Louvain) followed by capacity packing; same interface
	// and refinement passes as the aggregative method, but clusters are built from
	// detected communities instead of individual images
	std::vector<Scene> SplitSceneCommunityDetection(std::vector<IIndexArr>* outLocalToGlobal);

	// Greedy max-weight merging of the given initial clusters under the capacity
	// limit and the minimum-coupling acceptance test (shared by both methods)
	void GreedyMergeClusters(std::vector<IIndexArr>& clusters, bool periodicRefine);

	// Deterministic Louvain community detection on a subset of the covisibility
	// graph (standard modularity with resolution gamma)
	std::vector<IIndexArr> DetectCommunities(const IIndexArr& nodes, float gamma) const;

	// Recursively split a community larger than maxViewsPerCluster by escalating
	// the detection resolution; falls back to halving for fully dense communities
	void SplitOversizedCommunity(const IIndexArr& community, float gamma, std::vector<IIndexArr>& out) const;

	// Helper: Merge small clusters with neighbors
	void MergeSmallClusters(std::vector<IIndexArr>& clusters);

	// Helper: Refine clusters using local search (move/swap nodes for modularity + balance)
	void RefineClustersLocalSearch(std::vector<IIndexArr>& clusters);

	// Helper: conservatively move well-connected boundary images out of the
	// largest cluster into smaller neighbors, to shorten the critical path of
	// concurrent sub-scene reconstruction (moves are gated by a minimum
	// affinity ratio to the target cluster, so weakly-coupled images never move)
	void RefineClustersBalance(std::vector<IIndexArr>& clusters);

	// Helper: Split disconnected components within clusters
	void RefineClustersSplitDisconnected(std::vector<IIndexArr>& clusters);

	// Helper: split any cluster whose best balanced bipartition (spectral cut of its
	// internal covisibility graph) is joined below the minClusterCoupling seam — the
	// thin-waist clusters that would otherwise reconstruct as two independently scaled
	// blocks; each half becomes its own sub-scene, realigned by the global Sim(3) merge
	void RefineClustersSplitThinWaist(std::vector<IIndexArr>& clusters);

	// Helper: Rescue small orphaned clusters
	void RefineClustersRescueOrphans(std::vector<IIndexArr>& clusters);

	// Helper: Create sub-scenes and logging/export from clusters
	std::vector<Scene> BuildSubScenesFromClusters(
		std::vector<IIndexArr>& clusters,
		std::vector<IIndexArr>* outLocalToGlobal);

private:
	Scene& scene;                  // Reference to input scene
	const ClusterConfig& config;   // Clustering configuration
	std::vector<int> xadj;         // CSR graph: adjacency start indices
	std::vector<int> adjncy;       // CSR graph: adjacency list
	std::vector<int> adjwgt;       // CSR graph: edge weights
};

} // namespace SFM

#endif // _SFM_SCENECLUSTER_H_
