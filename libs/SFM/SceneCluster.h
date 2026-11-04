/*
 * SceneCluster.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_SCENECLUSTER_H_
#define _SFM_SCENECLUSTER_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"
#include "Track.h"


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
 * smaller sub-scenes that share no cameras -- every image belongs to at most
 * one; an image no cluster holds well is left in the pool -- and overlap only in structure: the points the cameras on both sides
 * of a cut see, which the merge registers the sub-scenes on. Each sub-scene
 * can be reconstructed independently and later merged into a single global
 * scene.
 *
 * This file implements the PARTITION. The blocks extracted from it and the pool
 * resection between the blocks' reconstruction and the merge are in SceneBlocks.h;
 * the MERGE phase is in GlobalAlignment.h.
 *
 * ── Pipeline overview (split) ──────────────────────────────────────────────
 *
 * 1. BUILD COVISIBILITY GRAPH
 *    Construct a weighted undirected graph where each node is an image and each
 *    edge weight encodes the number of geometrically verified feature matches
 *    between two images (from image pairs). This graph captures the visual
 *    overlap structure of the dataset. If the scene arrives with matches but no
 *    tracks yet, they are built once here, so the seam statistics the refinement
 *    passes below read have something to read; each sub-scene rebuilds its own
 *    tracks again once it is split off.
 *
 * 2. AGGREGATIVE CLUSTERING
 *    Partition the graph using bottom-up (agglomerative) clustering:
 *    - Start with each image as its own cluster.
 *    - Repeatedly merge the two clusters connected by the highest-weight edge,
 *      updating edge weights between the merged cluster and its neighbors.
 *    - A cluster stops growing at targetViewsPerCluster, and takes in more only
 *      to absorb a cluster under minViewsPerCluster, never past maxViewsPerCluster.
 *    - Two clusters do not merge over an interface thinner than minClusterCoupling
 *      of the weaker side's own internal weight, whatever their sizes; a singleton
 *      has none and joins freely.
 *    This greedy approach produces clusters that respect the covisibility
 *    structure: images that see many of the same features end up together,
 *    ensuring each sub-scene has strong internal connectivity.
 *
 * 3. CLUSTER REFINEMENT
 *    Six passes tidy the greedy result and make every remaining boundary usable
 *    by the merge:
 *    a) Local search: iteratively move boundary images between clusters to
 *       improve a modularity + balance objective.
 *    b) Merge small clusters: clusters below minViewsPerCluster are absorbed
 *       into their most-connected neighbor (up to maxOverCapacity slack), except
 *       a community no neighbour is coupled to (its heaviest interface under the
 *       minClusterCoupling seam) that has the strong seams the merge places a
 *       block by: it stays its own sub-scene, reconstructs on its own cohesion
 *       and enters through the merge.
 *    c) Balance: move well-connected boundary images out of the largest cluster
 *       into smaller neighbors, gated by a minimum affinity ratio, to shorten the
 *       critical path of concurrent sub-scene reconstruction; a move that would
 *       leave part of the source cluster coupled under minClusterCoupling is
 *       refused.
 *    d) Split disconnected: if a cluster has disconnected components in the
 *       covisibility graph, split it into separate clusters.
 *    e) Split thin waists: a cluster whose weakest prefix cut along its Fiedler
 *       order is joined below the minClusterCoupling seam is split in two when
 *       both sides can stand (minViewsPerCluster), and otherwise its smaller
 *       side moves whole to the cluster it is most coupled to, rather than left
 *       to reconstruct as an appendage that bends or takes its own scale.
 *    f) Make the cuts usable by the merge: a cluster whose seams carry too few
 *       usable tracks, or hold them in too few cameras, to register it against
 *       its neighbors (IsStrongSeam, below minClusterDegree strong neighbors) is
 *       merged into the neighbor it shares the most usable tracks with; a final
 *       small-cluster pass absorbs whatever that left under the floor.
 *
 * 4. FINISH THE PARTITION (FinishPartition)
 *    A cluster still under minViewsPerCluster that is not a standalone community
 *    goes to the pool: its images are in no block, reconstructed by no block, and
 *    the pool resection (SceneBlocks::ResectPool) registers them in the block they
 *    connect to best. The partition is the clusters that become blocks plus the pool.
 *
 * 5. EXTRACT THE BLOCKS (SceneBlocks::Extract)
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
 *    The output (SceneBlocks) is:
 *    - A vector of sub-scenes with local IDs [0, N), each self-contained with
 *      its own cameras, images (with keypoints), and intra-cluster pairs.
 *    - A parallel vector of localToGlobal mappings: localToGlobal[sceneIdx][localImgID] = globalImgID.
 *    - blockOf[globalImgID]: the block holding the image, NO_ID for the pool.
 *    - The global scene retains its image array (now with empty keypoints for
 *      assigned images, whole for the pool's) and only the pairs no block holds:
 *      the cross-sub-scene pairs and the pairs with a pool image.
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
 *     images[]     → keypoints/descriptors MOVED OUT (empty for clustered images, kept for the pool)
 *     pairs[]      → only cross-sub-scene pairs and the pool's pairs remain
 *
 *   Sub-scenes (after split):
 *     images[]     → keypoints/descriptors MOVED IN from global
 *     pairs[]      → only intra-cluster pairs (moved from global)
 *
 * During independent reconstruction of each sub-scene (BuildTracks →
 * StarInitializer → Resection → BundleAdjustment), only that sub-scene's
 * data is in memory. The pool resection moves an image's features and pairs
 * between its block and the global scene (SceneBlocks::Release / Insert), so
 * they still exist in one place only. After reconstruction,
 * GlobalAlignment::MergeSingleScene moves everything back to the global scene.
 */

/**
 * @brief Configuration for scene clustering
 */
struct SFM_API ClusterConfig
{
	unsigned maxViewsPerCluster{200};    // ceiling; 0 = disable clustering
	unsigned targetViewsPerCluster{133}; // a cluster at or past this absorbs nothing more; the one merge that takes it past is the one absorbing a cluster under the floor
	unsigned minViewsPerCluster{53};     // floor: smaller clusters are merged into their strongest neighbour
	unsigned maxOverCapacity{20};        // extra images allowed over the ceiling when absorbing a small cluster
	unsigned minClusterDegree{2};        // strong neighbours a cluster is held to, capped by the number it has; below that it is merged into its heaviest one when the result fits (0 = keep every cluster)
	unsigned minSeamTracks{75};          // seam-usable tracks a cluster pair needs for the neighbour to count as strong
	unsigned minSeamCameras{3};          // cameras per side with enough seam-usable tracks for the neighbour to count as strong
	unsigned minSeamCameraTracks{30};    // seam-usable tracks one camera needs to count
	float minPairWeight{3.f};            // minimum composite weight for pair edge
	TrackConflictConfig trackConflictCfg; // how a component holding one image twice is settled when the tracks are built here
	float minClusterCoupling{0.05f};     // refuse a merge whose interface weight falls below this fraction of the weaker side's internal weight, whatever either side's size, and split any final cluster with such an internal seam (0 = disabled)
	unsigned minWaistSideViews{10};      // the fewest images either side of a prefix cut must hold for the coupling checks to weigh it: a smaller side's internal weight is a pair or two, too little for the ratio to judge (Playground's under-coupled stretches held 23 and 31)
	bool useCommunityDetection{false};   // partition by community detection + capacity packing instead of pure aggregative clustering

	// The ceiling and the two sizes that follow from it: a cluster aims at two thirds of the
	// ceiling and is not kept below four fifteenths of it. The single place they are derived, so
	// the defaults above and a user-given ceiling follow the same rule.
	void SetMaxViews(unsigned maxViews) {
		maxViewsPerCluster = maxViews;
		targetViewsPerCluster = maxViews * 2 / 3;
		minViewsPerCluster = maxViews * 4 / 15;
	}
};

/**
 * @brief The seam one cluster offers a neighbour, as the merge reads it
 *
 * Seam-usable tracks are the tracks with at least two observations in one cluster (it can
 * triangulate them) and at least one in the other (its camera sees them): the correspondences the
 * generalized pose estimation of one cluster's cameras against the other's points consumes. A
 * camera supports the merge's readings from either side, so every observation of a usable track
 * counts for the camera that made it.
 */
struct SFM_API SeamTrackStats
{
	unsigned usable{0};                             // tracks usable in either direction
	std::unordered_map<IIndex, unsigned> perCamera; // global image -> its seam-usable tracks toward the other cluster

	// how many cameras of this side carry at least n seam-usable tracks
	unsigned CamerasWithAtLeast(unsigned n) const;
};

// The seam of every adjacent cluster pair, read off the global tracks; the key (a, b) with a < b
// holds the statistics of side a toward b in .first and of side b toward a in .second
typedef std::map<std::pair<uint32_t, uint32_t>, std::pair<SeamTrackStats, SeamTrackStats>> SeamTrackStatsMap;

// A neighbour is strong when the seam between the two clusters carries enough usable tracks AND
// enough cameras on BOTH sides carry minSeamCameraTracks of them: the merge weighs a reading by the
// supporting cameras of its weaker side, and anything less leaves it no registration between the
// two it can stand on, whatever the covisibility graph says about them. The one place this rule is
// written.
bool SFM_API IsStrongSeam(const std::pair<SeamTrackStats, SeamTrackStats>& seam, const ClusterConfig& config);

/**
 * @brief The partition clustering ends with: the clusters that become blocks, each sorted by global
 * image ID, and the pool, the images no cluster holds well (sorted); together they hold every image once
 */
struct SFM_API ScenePartition
{
	std::vector<IIndexArr> clusters;
	IIndexArr pool;
};

/**
 * @brief Scene partitioning using aggregative graph clustering
 *
 * See the top-level comment in this file for the full hierarchical SfM
 * partition architecture and memory protocol.
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
	 * @brief Partition the scene by graph clustering into the clusters that become blocks and the pool
	 * (a scene within maxViewsPerCluster, or with clustering disabled, is one cluster); SceneBlocks
	 * extracts the blocks from it
	 */
	ScenePartition Partition();

	/**
	 * @brief The seam statistics of every adjacent cluster pair, read off the scene tracks
	 * @param clusters Clusters of global image IDs; an image in no cluster is ignored
	 */
	SeamTrackStatsMap ComputeSeamTrackStats(const std::vector<IIndexArr>& clusters) const;

	/**
	 * @brief Merge away every cluster the merge could not attach: one whose seams carry too few
	 * usable tracks, or too few cameras carrying minSeamCameraTracks of them, to register it against its
	 * neighbours. It joins the neighbour it shares the most usable tracks with, if the result fits.
	 */
	void MergeLeafClusters(std::vector<IIndexArr>& clusters);

	/**
	 * @brief Move well-connected boundary images out of the largest cluster into smaller neighbours, to
	 * shorten the critical path of concurrent sub-scene reconstruction: a move needs the image's weight to
	 * the target to reach a minimum affinity ratio of its weight to its own cluster, and is refused when
	 * the cluster it leaves would then hold a prefix cut under minClusterCoupling (AnalyzeClusterCoupling)
	 */
	void RefineClustersBalance(std::vector<IIndexArr>& clusters);

	/**
	 * @brief Split or reshape every cluster whose weakest prefix cut along its Fiedler order is under the
	 * minClusterCoupling seam: a side of minViewsPerCluster images or more becomes its own cluster; a
	 * smaller stretch moves whole to the neighbouring cluster it is most coupled to, when that cluster
	 * holds it at the floor, more than its own does, and has room; else it stays
	 */
	void RefineClustersSplitThinWaist(std::vector<IIndexArr>& clusters);

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

	// Aggregative clustering (greedy max-weight)
	ScenePartition PartitionAggregativeClustering();

	// Community detection (Louvain) followed by capacity packing; same interface
	// and refinement passes as the aggregative method, but clusters are built from
	// detected communities instead of individual images
	ScenePartition PartitionCommunityDetection();

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

	// a cluster under the floor that no neighbour is coupled to but that the merge could place: kept
	// as its own sub-scene by the small-cluster pass and the partition's finish
	bool IsStandaloneCommunity(const std::vector<IIndexArr>& clusters, size_t c, const SeamTrackStatsMap& seamStats) const;

	// Whether cluster c without image u keeps every prefix cut along its Fiedler order at or above
	// minClusterCoupling, and stays connected
	bool KeepsCoupling(const std::vector<IIndexArr>& clusters, int c, IIndex u) const;

	// Helper: Refine clusters using local search (move/swap nodes for modularity + balance)
	void RefineClustersLocalSearch(std::vector<IIndexArr>& clusters);

	// Helper: Split disconnected components within clusters
	void RefineClustersSplitDisconnected(std::vector<IIndexArr>& clusters);

	// Helper: the cuts as the merge will see them — merge away the clusters no seam can register,
	// and hand whatever that leaves under the floor back to the floor rule; the tail both
	// clustering methods end through
	void RefineClustersForSeams(std::vector<IIndexArr>& clusters);

	// Helper: the partition the refined clusters make: a cluster smaller than minViewsPerCluster
	// goes to the pool, except a community that stands alone (IsStandaloneCommunity), which becomes
	// a block at any size
	ScenePartition FinishPartition(std::vector<IIndexArr>& clusters);

private:
	Scene& scene;                  // Reference to input scene
	const ClusterConfig& config;   // Clustering configuration
	std::vector<int> xadj;         // CSR graph: adjacency start indices
	std::vector<int> adjncy;       // CSR graph: adjacency list
	std::vector<int> adjwgt;       // CSR graph: edge weights
};

} // namespace SFM

#endif // _SFM_SCENECLUSTER_H_
