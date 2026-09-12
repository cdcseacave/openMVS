# Hierarchical SFM Pipeline

## 1. Purpose and scope

Running bundle adjustment on thousands of images at once is slow and numerically fragile.
`Scene::ReconstructHierarchical()` addresses this with a **divide → reconstruct → reunite**
strategy: split the scene into covisibility-coherent clusters bounded in size, reconstruct each
cluster independently with the standard incremental pipeline, then estimate a similarity
transform per cluster and merge everything back into one coordinate system. When the whole scene
fits in a single cluster, the split/merge machinery is a no-op and the pipeline degrades to plain
incremental reconstruction.

Three phases:

```
Phase 1 (split):     SceneCluster::SplitScene()          — libs/SFM/SceneCluster.h/.cpp
Phase 2 (reconstruct): per-cluster incremental SFM         — thread pool over sub-scenes
Phase 3 (merge):     GlobalAlignment::MergeScenes()       — libs/SFM/GlobalAlignment.h/.cpp
```

## 2. Algorithm as implemented

### Phase 1 — `SceneCluster::SplitScene` (`libs/SFM/SceneCluster.cpp`)

1. **Covisibility graph** — `SceneCluster::BuildConnectivityGraph`: a weighted undirected graph
   (CSR format) where nodes are images and edge weights are composite pair weights; edges below
   `ClusterConfig::minPairWeight` are discarded.
2. **Partitioning** — one of two methods, selected by `ClusterConfig::useCommunityDetection`:
   - `SceneCluster::SplitSceneAggregativeClustering` — bottom-up greedy merging via
     `GreedyMergeClusters`: a priority queue of edges by weight; the top edge merges its two
     clusters when the result stays within `maxViewsPerCluster` and the edge weight is not below
     `minClusterCoupling` times the weaker cluster's internal weight (a merge refused this way is
     re-queued whenever either side later changes). The queue is rebuilt every
     `max(10, maxViewsPerCluster / 10)` merges, at which point `RefineClustersLocalSearch` also
     runs, keeping edge weights consistent with the evolving partition.
   - `SceneCluster::SplitSceneCommunityDetection` — deterministic Louvain community detection
     (`DetectCommunities`) on the covisibility graph, oversized communities recursively split by
     `SplitOversizedCommunity`, then the same `GreedyMergeClusters` capacity-packing pass.
3. **Refinement** — both partitioning methods run the same six passes, in this order:
   1. `RefineClustersLocalSearch` — up to 20 iterations moving boundary images to whichever
      neighboring cluster maximizes internal connectivity, capacity-limited.
   2. `MergeSmallClusters` — absorbs clusters below `minViewsPerCluster` into their
      most-connected neighbor, allowing up to `maxOverCapacity` slack over `maxViewsPerCluster`.
   3. `RefineClustersBalance` — conservatively moves well-connected boundary images out of the
      largest cluster into smaller neighbors (gated by a minimum affinity ratio) to shorten the
      critical path of concurrent Phase 2 reconstruction.
   4. `RefineClustersSplitDisconnected` — splits a cluster whose images form disconnected
      components in the covisibility graph.
   5. `RefineClustersSplitThinWaist` — splits any cluster whose best balanced bipartition
      (spectral cut of its internal covisibility graph) is joined below the `minClusterCoupling`
      seam, so a cluster spanning two blocks connected only by a sparse interface never
      reconstructs as two independently-scaled halves; each half becomes its own sub-scene,
      realigned by the Phase 3 Sim(3) merge.
   6. `RefineClustersRescueOrphans` — absorbs remaining small orphan clusters into neighbors.
4. **Sub-scene extraction** — `SceneCluster::ExtractSubScene` builds an independent `Scene` per
   cluster with all IDs remapped to a local `[0, N)` range; `localToGlobal[localImgID] =
   globalImgID` is retained for Phase 3. See the memory protocol in §4.

### Phase 2 — per-cluster reconstruction

Each sub-scene runs the standard incremental pipeline independently on a thread pool:

```
BuildTracks → StarInitializer → Resection → BundleAdjustment → FilterTracks
```

`StarInitializer` selects the reference view (highest connectivity) and builds a star
configuration (`minViews=4`, `maxViews=36`, `minTracksPerView=50`). `Resection` incrementally
registers remaining images via PnP + RANSAC with periodic local BA. If initialization fails for a
sub-scene it is skipped; those images remain uncalibrated and are re-registered in Phase 3 (see
demotion, below).

### Phase 3 — `GlobalAlignment::MergeScenes` (`libs/SFM/GlobalAlignment.cpp`)

1. **`EstimateRelativePoses`** — for every pair of sub-scenes connected by cross-cluster image
   pairs (`ImagePair::GetNumFilteredInliers() >= minCommonTracks`), collect 3D-3D correspondences:
   for each cross-sub-scene inlier match whose two endpoints each land on an existing inlier track
   in their own sub-scene, pair the two (already-triangulated) 3D positions. Estimate a 7-DOF
   Sim(3) transform via `EstimateSimilarityTransform` (`libs/SFM/SimilarityTransform.cpp`) with a
   RANSAC threshold set to `simInlierThresholdFactor` times the destination point cloud's
   bounding-box diagonal (so the criterion is scale-invariant across sub-scenes with arbitrary
   local units). Pairs are rejected for too few correspondences (`< minCommonTracks`), RANSAC
   failure, too few inliers, or inlier ratio below `minSimInlierRatio`. Output: `vector<ScenePair>`
   each carrying the full `Transform` (rotation, scale, translation) and inlier count. This
   replaces an earlier design that ran PoseLib's generalized relative-pose solver on 2D-2D
   correspondences (rejected — see §6).
2. **`EstimateGlobalRotations`** — extracts relative rotations from the scene pairs and solves
   for global rotations via `GlobalRotationEstimator` (`libs/SFM/GlobalRotationAveraging.h`,
   adapted from GLOMAP): maximum-spanning-tree initialization (Kruskal, weighted by inlier count),
   `maxNumL1Iterations` (5) of L1-ADMM tangent-space linearization, then
   `maxNumIrlsIterations` (100) of IRLS with Geman-McClure weights (`irlsLossParameterSigma` =
   5°); pairs whose angular residual exceeds `maxRelativeRotationAngle` (12°) are filtered and the
   solve re-run. Output: one angle-axis vector per sub-scene; sub-scenes the estimator cannot
   place are left with an `INF` rotation.
3. **`EstimateGlobalScales`** — pairwise scale ratios come directly from each `ScenePair`'s
   `relativeTransform.scale` (no separate depth-matching pass). Solves
   `log(s_j) - log(s_i) = log(s_ij)` via weighted least-squares in log-space
   (`GlobalScaleAveraging.h`). Gauge: first sub-scene pinned to `s = 1.0`.
4. **`EstimateGlobalTranslations`** — rotates and scales each pairwise translation into the
   global frame using the now-known rotations and scales, then solves the resulting linear system
   via `GlobalTranslationAveraging.h`. Gauge: best-connected sub-scene pinned at the origin.
5. **`ValidateAlignment` / `RefineDemotedAlignment`** — each surviving `ScenePair`'s measured
   Sim(3) is composed with the averaged global transforms of its two end-points; the residual is
   identity when the edge agrees with the consensus. Edges whose scale, rotation, or translation
   residual exceeds `maxSimScaleRatio` / `maxSimRotationError` / `maxSimTranslationError` are
   conflicting; the sub-scene most dominated by conflicting incident edge weight is demoted
   (a node with a single incident edge can never be flagged, since averaging satisfies it
   exactly). Sub-scenes left unplaced by rotation averaging are demoted as well. Demoting removes
   edges, so scale and translation are re-averaged over the survivors and re-validated until the
   verdict is stable, demoting at least one more sub-scene per iteration; demoting can disconnect
   the pair graph, in which case every sub-scene outside the largest surviving component is
   demoted too. A demoted sub-scene is merged **without** its poses and 3D positions — its images
   stay unregistered so the post-merge incremental resection re-registers them against the
   trusted consensus, rather than compensating for the conflict inside the merge stage itself (see
   §6 for the alternative that was tried and rejected here).
6. **`MergeTransformedScenes`** — applies `s_i · R_i, t_i` to each sub-scene
   (`Scene::Transform`), averages shared camera intrinsics via the polymorphic
   `Camera::AccumulateIntrinsics` / `ScaleIntrinsics` interface, then:
   - `MergeSingleScene` moves keypoints, descriptors, and image pairs back from each sub-scene to
     the global scene (reversing `ExtractSubScene`); a demoted sub-scene's poses and track
     inlier flags are dropped instead of copied.
   - `MergeTracksWithCrossSubScenePairs` — union-find over global feature IDs, seeded with each
     sub-scene's existing inlier tracks (`mergeTrackInliersOnly`); processes **only**
     cross-sub-scene pairs (intra-sub-scene pairs already had correct tracks from Phase 2 —
     re-processing them would over-merge). Each union is guarded by (a) no duplicate image in a
     track and (b) 3D proximity within a fraction of the scene bounding-box diagonal. New
     cross-pair-only tracks are triangulated via `TriangulateSkewLLS`.
   - Final `FilterTracks(scene, 16.f, 0.5f)` — 16px reprojection / 0.5° angle threshold.

`MergeScenes` returns `false` only if alignment cannot complete at all, in which case the global
scene is populated with the largest intact sub-scene so a partial reconstruction is never
discarded.

## 3. Parameters and defaults

| Parameter | Struct / CLI | Default | Meaning |
|---|---|---|---|
| `maxViewsPerCluster` | `ClusterConfig` / `--max-views-per-cluster` | 200 | Maximum images per cluster (0 disables clustering) |
| `minViewsPerCluster` | `ClusterConfig` | 10 | Clusters below this are merged/reassigned |
| `maxOverCapacity` | `ClusterConfig` | 20 | Slack allowed over `maxViewsPerCluster` when absorbing orphans |
| `minPairWeight` | `ClusterConfig` | 3.0 | Minimum composite covisibility edge weight kept in the graph |
| `minClusterCoupling` | `ClusterConfig` | 0.05 | Minimum interface/internal-weight ratio to accept a merge or keep a cluster unsplit (0 disables) |
| `useCommunityDetection` | `ClusterConfig` / `--cluster-communities` | false | Louvain community detection + capacity packing instead of pure aggregative clustering |
| `minViews` / `maxViews` / `minTracksPerView` | `StarInitializer` config | 4 / 36 / 50 | Star-configuration bounds for reference-view initialization |
| `minCommonTracks` | `GlobalAlignmentConfig` | 25 | Minimum 3D-3D correspondences to attempt a sub-scene pair's Sim(3) |
| `mergeTrackInliersOnly` | `GlobalAlignmentConfig` | true | Seed track union-find with inlier observations only |
| `simInlierThresholdFactor` | `GlobalAlignmentConfig` | 0.01 | Sim(3) RANSAC inlier threshold, as a fraction of the destination bbox diagonal |
| `minSimInlierRatio` | `GlobalAlignmentConfig` | 0.3 | Minimum Sim(3) RANSAC inlier ratio to accept a sub-scene pair |
| `simRansacMaxIters` | `GlobalAlignmentConfig` | 10000 | Sim(3) RANSAC iteration budget |
| `maxSimScaleRatio` | `GlobalAlignmentConfig` | 1.1 | Max per-edge scale residual vs. averaged global transforms before demotion |
| `maxSimRotationError` | `GlobalAlignmentConfig` | 3° | Max per-edge rotation residual before demotion |
| `maxSimTranslationError` | `GlobalAlignmentConfig` | 0.05 | Max per-edge translation residual (fraction of local camera-bbox diagonal) before demotion |
| `maxNumL1Iterations` | `GlobalRotationEstimatorOptions` | 5 | L1-ADMM iterations for rotation averaging |
| `maxNumIrlsIterations` | `GlobalRotationEstimatorOptions` | 100 | IRLS iterations for rotation averaging |
| `irlsLossParameterSigma` | `GlobalRotationEstimatorOptions` | 5° | Geman-McClure loss transition point |
| `maxRelativeRotationAngle` | `GlobalRotationEstimatorOptions` | 12° | Relative-rotation filter threshold before re-solving |
| final track filter | `GlobalAlignment::MergeTransformedScenes` | 16 px / 0.5° | Reprojection / angle threshold applied after track merging |

## 4. Invariants and constraints

- Keypoints and descriptors exist in exactly one place at any time: moved (not copied) from
  global to sub-scene on split, moved back on merge.
- Cross-cluster image pairs never leave the global scene; intra-cluster pairs move to the
  sub-scene on split and move back on merge.
- Camera models are cloned (not moved) into each sub-scene, since Phase 2 bundle-adjusts each
  sub-scene's copy independently; intrinsics are re-merged by averaging in Phase 3.
- Track merging in Phase 3 processes cross-sub-scene pairs only — intra-sub-scene pairs already
  have correctly formed tracks from Phase 2, and re-processing them risks over-merging tracks that
  BA had correctly kept separate.
- The 3D-proximity guard exists because sub-scene tracks have disjoint image sets by construction,
  so the duplicate-image guard alone cannot catch a false cross-sub-scene match.
- Colors are released during Phase 3 track reassembly (indices change) and must be rebuilt
  downstream.
- A sub-scene demoted by `ValidateAlignment` contributes its features, image pairs, and track
  observations to the merge, but never its poses or 3D positions — those come only from images
  the post-merge resection re-registers against the trusted consensus.
- `SceneCluster::RefineClustersSplitThinWaist` and the `minClusterCoupling` merge gate are the
  only places responsible for keeping a cluster's internal covisibility strong enough to recover a
  single consistent scale; `GlobalAlignment` does not re-diagnose a sub-scene's internal
  soundness, only cross-checks sub-scenes against each other (see §6).

## 5. Validation of the shipped defaults

none recorded

## 6. Rejected alternatives

- **PoseLib generalized relative-pose solver on 2D-2D correspondences for Stage 1** — assumes both
  sub-scene "rigs" share the same metric scale, which does not hold after independent
  hierarchical reconstruction; replaced by Sim(3) estimation directly from each sub-scene's own
  triangulated 3D points.
- **Joint Sim(3) (7-DOF) global averaging** — a non-convex joint optimization per pair; the
  decoupled rotation → scale → translation approach keeps each subproblem convex or nearly so.
- **Comparing each sub-scene's own two-view geometry against its own poses**, as an alternative
  internal-soundness check inside `GlobalAlignment` — tried and abandoned: on a 7-scene benchmark
  it flagged every scene (11-72% violated pair weight) because two-view relative poses are
  unreliable on the low-parallax and homography-degenerate pairs such captures are full of.

## 7. Open items

- No automated regression numbers for the hierarchical split/merge pipeline are currently
  tracked in-repo (see §5).
