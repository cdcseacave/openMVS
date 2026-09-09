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

### Stage 1 — Relative Similarities

For every pair of sub-scenes the cross-cluster pairs connect, estimate the 7-DOF similarity `p_B = s·R·p_A + t` that maps one into the other. Two modes measure it; `GlobalAlignmentConfig::alignment` (`--cluster-alignment`) selects. Both start from the same per-sub-scene cache mapping `(localImage, feature)` to the 3D position of the inlier track holding that observation, and both walk the same cross-cluster matches (the track-forming prefix of each pair's `matches`); they differ in what they ask of a match.

**`ALIGN_POINTS` — similarity from 3D-3D correspondences.** A match contributes when **both** its endpoints hit the cache, giving one 3D point per sub-scene in its own local frame. `EstimateSimilarityTransform` fits the Sim(3) by RANSAC with the inlier distance set to a fraction (`simInlierThresholdFactor`, 1%) of the destination cloud's bounding-box diagonal, so the criterion is invariant to each sub-scene's arbitrary units.

This mode needs a match whose two endpoints both lie on a track. Warp-sampled (dense) keypoints are laid out on the source image's grid, so they coincide across that image's pairs and form tracks there, while the target side gets positions warped per pair which never join one. A seam bridged by dense matches alone can therefore end up with no correspondence at all.

**`ALIGN_CAMERAS` — generalized-camera PnP with scale.** One sub-scene's cameras form a rig whose internal poses are known in that sub-scene's frame and at its scale; the other sub-scene's inlier tracks are the 3D points; the cross-cluster matches are the rig's observations of them. A match contributes when **one** endpoint hits the cache — the other only has to be a keypoint — so a seam stays measurable from either side alone.

Correspondences are grouped per rig image (images that received none stay out of the rig), the 2D side entering as unit bearings from `Camera::UnprojectNormalized` so any central camera model works. PoseLib's `estimate_generalized_absolute_pose_scale_bearings` (LO-RANSAC over gp4ps followed by a scale-aware refinement) solves the rig pose and the rig-to-points scale together. It models `Z_k = R_k·(R·X + t) + scale·t_k`, i.e. it scales the rig's centers into the frame the points live in, so the similarity mapping the point sub-scene into the rig sub-scene is `p_rig = (1/scale)·R·p_point + (1/scale)·t`. The threshold is angular: the pixel threshold (`maxReprojError`, 4px, the resection's own) read through each camera's `PixelErrorToAngular`, the rig judged at the widest of them since the estimator scores it against one value. The scale is observable only across distinct rig centers, so a rig of fewer than two cameras is not estimated.

Both directions are estimated. A direction counts only if its inliers reach `minCommonTracks` and its inlier ratio reaches `minSimInlierRatio`. If both count they must agree — rotation within `maxSimRotationError`, scale ratio within `maxSimScaleRatio` — or the seam is **rejected**, never resolved in favour of the better supported estimate: a seam accepted wrong merges a whole block into the wrong place. Agreeing directions are then refined jointly, with Ceres, into one Sim(3) over the union of both inlier sets: seven parameters (unit quaternion on its manifold, translation, log scale), residuals the reprojection of A's points into B's cameras through `T` and of B's points into A's cameras through `T⁻¹`, under a Huber loss at the pixel threshold. The pair's weight is the sum of both inlier counts. If only one direction counts, it stands alone on its own inliers.

Output, in both modes: `vector<ScenePair>`, each carrying the full Sim(3) and the inlier count the later averaging weights by.

### Stage 2 — Rotation Averaging

Extract relative rotations `R_ij` from the scene pairs and solve for global rotations using an L1-ADMM + IRLS pipeline (adapted from GLOMAP):

1. **MST initialization**: Kruskal's maximum spanning tree (weights = inlier counts), BFS propagation from highest-degree root. Root fixed to identity (gauge freedom).
2. **L1-ADMM** (5 iterations): tangent-space linearization `δR_ij ≈ δR_j − δR_i`, sparse linear system, L1 robust loss.
3. **IRLS refinement** (up to 100 iterations): Geman-McClure weights `w = σ² / (σ² + ε²)²` with `σ = 5°`.
4. **Filter and re-solve**: remove pairs with angular residual > 12° and re-run.

Output: one angle-axis vector per sub-scene.

### Stage 3 — Scale Averaging

The pairwise scale comes straight out of Stage 1: each `relativeTransform` satisfies `p_B = (s_A/s_B)·R·p_A + t`, so its scale field is `s_A/s_B` and its reciprocal is the ratio `s_B/s_A` the estimator wants. Nothing is re-measured here, and a pair whose scale is not positive is dropped.

Solve the overdetermined system in log-space by weighted least-squares (SVD), each equation weighted by the pair's inlier count:

```
log(s_j) − log(s_i) = log(ratio_ij)
```

Gauge: the fixed node is the sub-scene carrying the most incident pair weight, eliminated from the system rather than penalized, so its scale is exactly 1. With no pairs at all every scale falls back to 1.

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

### One Seam, Two Directions

The camera alignment is asymmetric: it registers one sub-scene's cameras against the other's points, and the two sub-scenes are not interchangeable in it — each side's tracks and each side's camera spread carry different evidence. Running it both ways costs one more estimation per seam and buys two things a single direction cannot: a seam survives when only one side's keypoints ever formed tracks, and when both sides speak they can be made to agree before anything downstream trusts them. Rotation averaging is robust to a wrong edge, but scale and translation averaging are not, and a seam is the joint of a whole block — so disagreement rejects rather than arbitrates.
