# Hierarchical SFM Pipeline — Core Design

## Overview

The pipeline solves a fundamental scaling problem: running bundle adjustment on thousands of images at once is slow and numerically fragile. The solution is **divide → reconstruct → reunite** — split the scene into manageable clusters, reconstruct each independently, then align and merge everything back into one coordinate system.

Three phases, orchestrated by `Scene::ReconstructHierarchical()`:

```
Phase 1: SceneCluster::SplitScene()         → partition into sub-scenes
Phase 2: threadPool.detach_loop(subScenes)  → parallel incremental SFM
Phase 3: GlobalAlignment::MergeScenes()     → 5-stage alignment + merge
```

---

## Phase 1 — Scene Clustering

**Goal**: partition images into sub-scenes of bounded size (≤ `maxViewsPerCluster`, default 200).

### Covisibility Graph

A weighted undirected graph is built where nodes are images and edge weights are composite pair weights. Edges below `minPairWeight` (3.0) are discarded. The graph is stored in CSR format for compatibility with graph partitioning libraries.

### Aggregative Clustering

Bottom-up greedy merging that respects covisibility structure:

1. Initialize each image as a singleton cluster
2. Build a priority queue of edges sorted by weight (descending)
3. Pop the highest-weight edge; merge the two clusters if the result stays within the size limit
4. Periodically rebuild the PQ (every `max(10, maxViewsPerCluster / 10)` merges) to keep edge weights consistent

### Cluster Refinement (4 passes)

1. **MergeSmallClusters** — absorb clusters below `minViewsPerCluster` (10) into the most-connected neighbor, with `maxOverCapacity` (20) slack
2. **RefineClustersLocalSearch** — up to 20 iterations: move boundary images to whichever cluster maximizes internal connectivity (modularity + balance)
3. **RefineClustersSplitDisconnected** — split clusters whose images form disconnected components in the covisibility graph
4. **RefineClustersRescueOrphans** — absorb remaining small orphans into neighbors

### Sub-Scene Extraction

`ExtractSubScene()` creates an independent `Scene` per cluster. The memory protocol is the key design element:

| Data | Action |
|------|--------|
| Camera models | **Cloned** (independent copies for per-sub-scene BA) |
| Image keypoints & descriptors | **Moved** from global to sub-scene |
| Intra-cluster pairs | **Moved** from global to sub-scene |
| Cross-cluster pairs | **Left** in global scene (used in Phase 3) |
| Tracks | Filtered to observations with ≥2 views in cluster |

All IDs are remapped to a local `[0, N)` range. A `localToGlobal[localImgID] = globalImgID` mapping is stored for the merge phase.

After split the global scene is a shell: keypoints empty, only cross-cluster pairs remain.

---

## Phase 2 — Parallel Reconstruction

Each sub-scene runs the standard incremental SFM pipeline independently via a thread pool:

```
BuildTracks → StarInitializer → Resection → BundleAdjustment → FilterTracks
```

**BuildTracks**: union-find over feature matches within intra-cluster pairs; produces 3D track candidates from multi-view observations.

**StarInitializer**: selects the reference view (highest connectivity) and builds a star configuration (`minViews=4`, `maxViews=36`, `minTracksPerView=50`).

**Resection**: incrementally registers remaining images via PnP + RANSAC, with periodic local BA.

**BundleAdjustment**: Ceres Solver non-linear optimization refining poses, points, and intrinsics.

**FilterTracks**: removes tracks with high reprojection error, low triangulation angle, or depth outside bounds.

If initialization fails for a sub-scene it is skipped; those images remain uncalibrated.

---

## Phase 3 — Global Alignment (5-Stage Merge)

Each sub-scene lives in its own arbitrary coordinate system. The merge estimates **similarity transforms** (rotation + scale + translation) to bring all sub-scenes into a single frame, using a decoupled approach where each subproblem is (nearly) convex.

### Stage 1 — Relative Poses

For every pair of sub-scenes connected by cross-cluster pairs, estimate the rigid relative pose.

Cross-cluster image pairs are grouped by sub-scene pair, sorted by inlier count, and limited to the top 25. Matches are subsampled to 1000 per pair (evenly spaced to preserve spatial distribution). PoseLib's **generalized relative pose** solver runs with RANSAC (`max_epipolar_error ≈ 2px` in normalized coords, 100–10000 iterations).

Rejection criteria: fewer than 2 camera pairs, fewer than 25 inliers, or inlier ratio below 15%.

Output: `vector<ScenePair>`, each containing the relative pose and inlier count.

### Stage 2 — Rotation Averaging

Extract relative rotations `R_ij` from the scene pairs and solve for global rotations using an L1-ADMM + IRLS pipeline (adapted from GLOMAP):

1. **MST initialization**: Kruskal's maximum spanning tree (weights = inlier counts), BFS propagation from highest-degree root. Root fixed to identity (gauge freedom).
2. **L1-ADMM** (5 iterations): tangent-space linearization `δR_ij ≈ δR_j − δR_i`, sparse linear system, L1 robust loss.
3. **IRLS refinement** (up to 100 iterations): Geman-McClure weights `w = σ² / (σ² + ε²)²` with `σ = 5°`.
4. **Filter and re-solve**: remove pairs with angular residual > 12° and re-run.

Output: one angle-axis vector per sub-scene.

### Stage 3 — Scale Averaging

For each scene pair, match features across sub-scenes via cross-cluster pairs, look up 3D positions in both, compute camera-to-point depths, and take the **median depth ratio** as the pairwise scale (minimum 10 valid depth pairs required).

Solve the global system in log-space via weighted least-squares (SVD):

```
log(s_j) − log(s_i) = log(ratio_ij)
```

Gauge: first sub-scene pinned to `s = 1.0`.

### Stage 4 — Translation Averaging

Transform relative translations into the global frame using the now-known rotations and scales:

```
t_j − t_i = s_i · R_i^T · C_ij
```

where `C_ij` is the position of scene j's origin in scene i's local frame. Solve independently for X, Y, Z via sparse QR (COLAMDOrdering). Gauge: best-connected node pinned at origin.

### Stage 5 — Merge & Track Assembly

Apply the composed similarity transform to every sub-scene:

```
p_global = s_i · R_i^T · p_local + t_i
```

**Intrinsics averaging**: cameras shared across sub-scenes have their intrinsics averaged via the polymorphic `AccumulateIntrinsics / ScaleIntrinsics` interface.

**Data reunion**: keypoints, descriptors, and pairs are moved back to the global scene (reversing the split), with all IDs remapped from local to global.

**Track merging** (union-find over global feature IDs):

1. Seed the union-find with existing sub-scene tracks
2. Process **only** cross-sub-scene pairs — intra-sub-scene pairs are deliberately skipped to avoid over-merging tracks that BA had correctly separated
3. Two guards protect each union operation:
   - **Duplicate-image guard** — a single track cannot observe the same image twice
   - **3D proximity guard** — merged tracks must have positions within 2% of the scene bounding box diagonal
4. New cross-pair-only tracks are triangulated via `TriangulateSkewLLS()`
5. Final `FilterTracks` at 16px reprojection / 0.5° angle threshold

---

## Memory Protocol

The split/merge cycle minimizes peak memory by **moving** (not copying) expensive data:

```
Split:
  Global → Sub-scenes:  keypoints, descriptors, intra-cluster pairs   (MOVED)
  Global → Sub-scenes:  cameras                                       (CLONED)
  Global retains:       cross-cluster pairs only

Merge:
  Sub-scenes → Global:  keypoints, descriptors, pairs                 (MOVED BACK)
  Sub-scenes → Global:  poses, tracks                                 (COPIED/APPENDED)
  Sub-scenes → Global:  camera intrinsics                             (AVERAGED)
```

Key invariants:

- Keypoints and descriptors exist in exactly one place at any time
- Intra-cluster pairs move to sub-scenes during split, move back during merge; cross-cluster pairs never leave the global scene
- Colors are released during track reassembly (indices change) and must be rebuilt downstream

---

## Design Rationale

### Decoupled R → s → t Estimation

Each subproblem is convex (or nearly so) when solved independently: rotation averaging on SO(3) has well-studied convex relaxations, scale averaging in log-space is linear least-squares, and translation averaging given known rotations and scales is a linear system. Joint Sim(3) would require solving a 7-DOF non-convex optimization per pair.

### Cross-Sub-Scene Pairs Only in Track Merging

Intra-sub-scene pairs already had their tracks correctly formed during reconstruction. Re-processing them would over-merge tracks: outlier observations removed during BA may have been the reason two features stayed in separate tracks. Cross-sub-scene pairs are the only source of new inter-sub-scene connectivity.

### Union-Find with 3D Guards

The union-find pattern from `BuildTracks` is reused for efficiency. The 3D proximity guard (2% of bounding box diagonal) addresses a gap specific to the merge scenario: sub-scene tracks have disjoint image sets by construction, so the duplicate-image guard alone cannot catch false matches between sub-scenes.

### Median Depth Ratios for Scale

Using the median (rather than mean) of depth ratios provides robustness against outlier matches. Computing depth along the viewing direction (rather than raw 3D distance) gives a scale-invariant measurement that properly captures the relative scale between two coordinate systems.
