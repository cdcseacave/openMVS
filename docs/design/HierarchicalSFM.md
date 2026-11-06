# Hierarchical SFM Pipeline

## 1. Purpose and scope

The pipeline solves a fundamental scaling problem: running bundle adjustment on thousands of images at once is slow and numerically fragile. The solution is **divide → reconstruct → reunite** — split the scene into manageable clusters, reconstruct each independently, then align and merge everything back into one coordinate system.

Four phases, orchestrated by `Scene::ReconstructHierarchical()`:

```
Phase 1: SceneCluster::Partition() + SceneBlocks::Extract()  → clusters become blocks, the rest is the pool
Phase 2: threadPool.detach_loop(blocks)                       → parallel incremental SFM
Phase 3: SceneBlocks::ResectPool()                            → unregistered images to the pool, registered into the block they connect to best
Phase 4: GlobalAlignment::MergeScenes()                       → read every block pair, average the block graph, merge
```

In plain words: the images are split into groups (blocks), each block is reconstructed on its own, every image a block could not register (and every image no block took) is put in one common pool and tried by the block it is best connected to, and then the blocks are placed against each other and merged.

## 2. Algorithm as implemented

### Phase 1 — Scene Clustering

**Goal**: partition images into sub-scenes of bounded size, each image belonging to at most one — an image no cluster holds well is left in the pool (Phase 3) — the sub-scenes share no cameras and overlap only in structure: the points the cameras on both sides of a cut see, which the merge registers the sub-scenes on. `--max-views-per-cluster` sets the ceiling (`ClusterConfig::maxViewsPerCluster`, default 200; 0 disables clustering); `ClusterConfig::SetMaxViews` derives the target a cluster aims at (`targetViewsPerCluster`, two thirds of the ceiling — default 133) and the floor under which a cluster is merged away (`minViewsPerCluster`, four fifteenths of the ceiling — default 53) from that one ceiling, so a user-given `--max-views-per-cluster` and the compiled-in defaults follow the same rule.

#### Covisibility Graph

A weighted undirected graph is built where nodes are images and edge weights are composite pair weights. Edges below `minPairWeight` (3.0) are discarded. The evidence behind that weight counts a pair's dense inliers in proportion to the matcher's frame target (`denseFrameInliers` 50 over `denseMatchesPerFrame` 2000, `PairsWeightingConfig`): a whole frame's worth of dense inliers is a modest descriptor pair's evidence, 50 weighted inliers, so a wide, imprecise dense-only overlap cannot outweigh the descriptor-verified pairs around it, while two dense-only pairs of one image keep the ratio of their counts and a textureless interior keeps its links in order. The graph is stored in CSR format for compatibility with graph partitioning libraries. If the scene arrives with matches but no tracks, they are built once here so the seam statistics the refinement passes below read have something to read; each sub-scene rebuilds its own tracks again once it is split off.

#### Aggregative Clustering

Bottom-up greedy merging that respects covisibility structure:

1. Initialize each image as a singleton cluster
2. Build a priority queue of edges sorted by weight (descending)
3. Pop the highest-weight edge; merge the two clusters unless the merge would cross `maxViewsPerCluster`, or would push the combined size past `targetViewsPerCluster` — except when the smaller side is still under `minViewsPerCluster` and has to go somewhere, and even then only while the larger side has not itself already exceeded `targetViewsPerCluster` — or would cross an interface thinner than `minClusterCoupling` (0.05, 0 = disabled) of the weaker side's own internal weight. The coupling test holds for that target-size exception too, not only once both sides are already past the floor: a community under the floor is exactly the case a thin interface can hand to whichever oversized cluster happens to have room, over neighbours it is far more attached to. A singleton has no internal weight to be thin against, so it always passes
4. Periodically rebuild the PQ and re-run the local-search pass below (every `max(10, maxViewsPerCluster / 10)` merges) to keep edge weights consistent

#### Cluster Refinement

Six passes tidy the greedy result and make every remaining cluster boundary usable by the merge; whatever is still under the floor after them, save a community that stands alone, goes to the pool (`SceneCluster::FinishPartition`):

1. **RefineClustersLocalSearch** — up to 20 iterations: move boundary images to whichever cluster maximizes internal connectivity (modularity + balance)
2. **MergeSmallClusters** — absorb clusters below `minViewsPerCluster` into the most-connected neighbor, with `maxOverCapacity` (20) slack; a community the coupling test refused during the greedy merge lands here too, placed by connection strength rather than by whichever cluster still had room — unless it is a community no neighbour is coupled to (its heaviest interface under `minClusterCoupling` of its own internal weight) that carries the strong seams the merge places a block by (`IsStrongSeam`, as many as its track neighbours allow): that one is left alone, reconstructs as its own sub-scene and enters through the merge (`SceneCluster::IsStandaloneCommunity`).
3. **RefineClustersBalance** — conservatively move well-connected boundary images out of the largest cluster into smaller neighbors, gated by a minimum affinity ratio, to shorten the critical path of concurrent sub-scene reconstruction; a move that would leave part of the source cluster coupled under `minClusterCoupling` is refused
4. **RefineClustersSplitDisconnected** — split clusters whose images form disconnected components in the covisibility graph
5. **RefineClustersSplitThinWaist** — split thin waists: a cluster whose weakest prefix cut along its Fiedler order is joined below the `minClusterCoupling` seam is split in two when both sides can stand (`minViewsPerCluster`), and otherwise its smaller side moves whole to the cluster it is most coupled to, rather than left to reconstruct as an appendage that bends or takes its own scale
6. **RefineClustersForSeams** — the cuts as the merge will see them. `MergeLeafClusters` merges away a cluster whose seams the merge could never attach — fewer than `minClusterDegree` (2, capped by the neighbours it actually has) strong neighbours, where a strong neighbour needs a seam of at least `minSeamTracks` (75) seam-usable tracks with, on both sides, at least `minSeamCameras` (3) cameras each carrying `minSeamCameraTracks` (30) of them (`IsStrongSeam`) — into the neighbour it shares the most usable tracks with, as long as the result still fits `maxViewsPerCluster + maxOverCapacity`. A final `MergeSmallClusters` pass absorbs whatever that left under the floor, save the communities that stand alone.

Removed on 2026-10-31 (#59), with the pool: the seam repair (`RepairClusterSeams`, which moved boundary images across a seam holding its tracks in too few cameras: 6 of the 9 seams it repaired over the 33 captures became unreadable, the moved image having no partners on its new side) and the orphan rescue (`RefineClustersRescueOrphans`, which dismantled a cluster still under the floor image by image into its neighbours). An image such a pass would have moved is now left where it is or put in the pool, and the pool resection registers it in the block that can.

#### Block Extraction (SceneBlocks)

`SceneBlocks` (`libs/SFM/SceneBlocks.h`) owns the state between the partition and the merge: the blocks (one `Scene` each), `localToGlobal` per block, `blockOf` per global image (`NO_ID` = the pool), and the global scene. `SceneBlocks::Extract` creates an independent `Scene` per cluster (local IDs sorted by global ID, so every pair keeps `ID1 < ID2` through each remap); the scene's threads are split over the blocks. The memory protocol is the key design element:

| Data | Action |
|------|--------|
| Camera models | **Cloned** (independent copies for per-sub-scene BA) |
| Image keypoints & descriptors | **Moved** from global to sub-scene |
| Intra-cluster pairs | **Moved** from global to sub-scene |
| Cross-cluster pairs, pairs with a pool image | **Left** in global scene (the pool's and the merge's) |
| Pool images' keypoints & descriptors | **Left** in global scene |
| Tracks | Filtered to observations with ≥2 views in cluster |

All IDs are remapped to a local `[0, N)` range. A `localToGlobal[localImgID] = globalImgID` mapping is stored for the merge phase.

After split the global scene is a shell for the blocks' images (keypoints empty) and holds the pool images whole; only the pairs no block holds remain in it.

### Phase 2 — Parallel Reconstruction

Each sub-scene runs the standard incremental SFM pipeline independently via a thread pool:

```
BuildTracks → StarInitializer → Resection → BundleAdjustment → FilterTracks
```

**BuildTracks**: union-find over feature matches within intra-cluster pairs; produces 3D track candidates from multi-view observations. A component holding one image twice is settled on its own links (`TrackConflictConfig`): the least-supported link on the path between the two keypoints is cut, keeping the side the triangles corroborate, and two keypoints within the merge distance count as one feature; the union veto in pair order remains as the fallback for components above the size bar, or when the cut is off.

**StarInitializer**: selects the reference view (highest connectivity) and builds a star configuration (`minViews=4`, `maxViews=36`).

**Resection**: incrementally registers remaining images via PnP + RANSAC, with periodic local BA.

**BundleAdjustment**: Ceres Solver non-linear optimization refining poses, points, and intrinsics.

**FilterTracks**: removes tracks with high reprojection error, low triangulation angle, or depth outside bounds (a track seen by fewer than two posed images has no position that two posed views determine and is not judged).

If initialization fails for a sub-scene it is skipped; the pool resection hands all its images to the pool and drops the block.

### Phase 3 — The pool resection (SceneBlocks::ResectPool)

In plain words: every image a block could not register leaves its block for a common pool, joining the images the partition left in no block; each pool image is then tried, one pass after another, by the block it is best connected to, until no more can be registered.

1. **Release**: every block hands its unregistered images to the pool (blocks in parallel): their features return to the global scene, their pairs too (in global IDs), their track observations are removed (an unposed image is never in an inlier prefix; a track left with fewer than two observations is dropped). A block whose star initialization failed hands every image and is dropped.
2. **Score**: for each pool image and block, the composite weight of the image's links (verified pairs with a relative pose) to that block's registered images — `ForEachLink` (`PoseLink.h`), the same sum `Resection::SelectNextImages` orders its candidates by.
3. **Pass**: each pool image goes to its best block not yet tried in this round (`SceneBlocks::Insert`: features, its pairs to the block and to the other inserted images, then `SceneBlocks::RebuildTracks`: the block's tracks built again by `BuildTracks` over its pairs, read in decreasing composite weight with the block's weight bar and conflict settling, so the block holds exactly the tracks it would hold had the images been in it from the start; a track with the same observations as before keeps its position, inlier state and observation order, every new or changed one is triangulated and filtered as the resection does after a registration — so an inserted image matched to two posed block images already holds a correspondence on the point their two keypoints now share). Every block with a batch runs `Resection::RegisterImages(false)` in parallel, the scene's threads split over them; the batch is the block's only unposed images, and every check of the ordinary resection applies (2D-3D minimum, PnP bars, rotation check, direction check, the schedule of adjustments). `false`: a block that registered nothing skips the closing adjustment and keeps its model exactly; one that registered is filtered as Phase 2 leaves a block. The images a block refused are released back (`SceneBlocks::Release`): their observations leave the block's tracks, and what they joined of those tracks stays (a point two of the block's views now share through a refused image keeps its position where it passes the filter: real evidence the merge would add anyway). The next `Insert` into the block rebuilds its tracks from its pairs again.
4. **Rounds**: a round ends when no pool image has an untried block with evidence; rounds repeat until one registers nothing. Blocks left empty are removed and the others renumbered (logged).

Logs: one VERBOSE line per round (passes, attempts, registered, left in the pool), one DEBUG line per attempt (image, its origin block, target, registered or refused with its 2D-3D correspondences when inserted and at the end, and its inliers), and a summary (registered per origin -> target block, -1 for the partition's pool; left in the pool; blocks dropped). The images left in the pool are what a skipped cluster was before: in no block, features in the global scene, left to the whole-scene resection after the merge.

### Phase 4 — Global Alignment (the block graph)

Each sub-scene — a **block** from here on, `GlobalAlignment`'s own word for it — lives in its own arbitrary coordinate system. `GlobalAlignment::MergeScenes` treats the blocks as the nodes of a small graph and every block pair the cross pairs join as an edge holding the readings of its Sim(3); it places the blocks as the view-graph filter places images — one robust averaging of the kept readings, every reading the averaged poses contradict cut, repeated until the kept set settles — and merges the model holding the most images back into the scene; every block outside it is merged without a pose, for the post-merge resection to recover.

#### Readings

The cross pairs of at least `minCommonTracks` (25) weighted inliers are grouped by block pair (`MapBlockPairLinks`), and every correspondence of a pair is collected in both directions (`CollectPairObservations`): a match counts in the direction "B's cameras on A's points" when its A endpoint lies on an inlier track of A, and in the other when its B endpoint lies on one of B. How a pair is read depends on `GlobalAlignmentConfig::alignment` (`--cluster-alignment`):

- **`ALIGN_CAMERAS`** (1, the default) — **each direction is read once**: the rig of one block's cameras against the other block's points by PoseLib's generalized absolute pose with scale (RANSAC on the resection's PnP options), refined over its inliers with Ceres against every bearing it explains at `maxReprojError` (4 px, the resection's own), the residual being the chord between observed and predicted bearings in each camera's own pixels under a Huber loss. A direction whose rig is one camera is a **one-camera door**: that camera's own PnP against the other block's points gives the rotation and the centre, and the median ratio of the depths its points have in the two blocks gives the scale (with fewer than 10 such ratios it observes none). A direction needs `minCommonTracks` correspondences and as many inliers. A reading observes a scale when the cameras holding `minCameraInliers` (10) of its inliers spread over at least `minRigSpreadRatio` (0.03) of their points' median depth. **The two directions stay two readings** of their pair: they are not grouped, and each is a measurement of its edge; where they agree they are each other's test.
- **`ALIGN_POINTS`** (0) — one 3D-3D similarity by RANSAC over the matches whose two endpoints both lie on an inlier track of their block (`simInlierThresholdFactor` 0.01 of the destination cloud's diagonal, `minSimInlierRatio` 0.3, `simRansacMaxIters` 10000).

Every reading is scored on every correspondence of its pair (`ScoreSeam`): a camera of either block that holds `minCameraInliers` inlier correspondences supports it, and the reading's camera count is the supporting cameras of its **weaker side** — a side held by one camera weighs one, however many correspondences that camera carries (the frame misplaced inside its own block that carries a whole seam on its side). It is also read against the **verified image pairs** across its pair (`MeasureSeamPairs`): a pair of at least `minPairInliers` (30) weighted inliers agrees when the relative rotation and the baseline direction the reading gives its two images are within `maxPairRotationResidual` (5°; the baseline at `atan(tan 5° / ray angle)`) of what it measured; the pairs' **share** is the inlier weight that agrees over all read (1 when none is read), and they **hold** against the reading when more weight disagrees than agrees.

A reading of the camera alignment always enters the block graph, weighing `max(cameras, 1) × share`. The 3D-3D reading enters only with a supporting camera on each side and its verified pairs not against it, weighing its cameras.

#### Selection: one averaging, one cut, repeated

Every connected component of the kept readings (all of them at first) is averaged by `AverageBlockPoses`, gauged at the block its readings weigh most on:

- **rotations** by `GlobalRotationEstimator` (its maximum spanning tree, L1, then its own Geman-McClure reweighting), run again over whatever a run leaves in another frame;
- **log-scales**, of the readings within the rotation bar that observe a scale, and **translations** — between the blocks' camera centroids (`BlockFootprint`), so they do not depend on where a block's arbitrary origin lies — of the readings within the scale bar, every reading reweighted by the inverse of its error (floored at 1e-4 in log-scale and 1e-3 of the footprint) as the estimators' row scale, 40 rounds at most: their normal equations weigh its square, which settles on one reading rather than on the L1 median. A reading of weight 0 (every verified pair across it against it) takes no part in these two solves and is still judged against their poses. A block the metric solves do not reach from the gauge gets no pose.

Every reading is then judged against the poses of its component: kept when its rotation is within `maxGraphRotationResidual` (5°) and, if it observes a scale, its scale ratio within `maxGraphScaleResidual` (1.20) and the translation it implies for A's camera centroid within `maxGraphTranslationResidual` (0.20) of the footprint the two blocks share (the smaller block's camera-box diagonal, never under `kMinExtentShare` 0.2 of the larger's). A reading whose two blocks the averaging did not both place cannot be judged and is off. The averaging and the cut repeat — a reading cut in one round is judged again in the next — until the kept set does not change, `kRobustRounds` (10) rounds at most.

**Bridges.** A block pair that no cycle of the other block pairs tests keeps a reading only where two witnesses besides the reading's own consistency hold: its weaker side holds `minSeamWeight` (2) supporting cameras, and its verified pairs do not hold against it; repeated until nothing changes. Its own two directions, within the bars of each other after the cut, are its own consistency test.

**The model.** Each connected component of the readings left is a model, its block poses the last averaging's; the one holding the most images is the scene. A block with no pose is a model of its own, with its reason — no seam, every reading off, or no pose from the averaging — in the log and the merge report.

#### Merge and the merged model's adjustment

The blocks of the merged model are transformed by their poses, a camera shared across blocks gets the average of its intrinsics, and keypoints, descriptors and pairs move back from every block (reversing `SceneBlocks::Extract`). **Closure**: every correspondence of every pair of placed blocks — whether or not a reading of it was kept — that the final block poses explain within the loose bar (`looseSeamFactor` 3 × `maxReprojError` = 12 px) joins its two tracks by construction, on the duplicate-image and length guards alone; the other connecting pairs (two blocks, or a pool image) join under the duplicate-image, length and reprojection guards; intra-block pairs are skipped. Every block outside the merged model is merged without its poses and its tracks without positions, so the resection can register its images one by one. A single block (a split the pool resection leaves with one block) is merged in its own frame through the same path, so its intrinsics and its tracks reach the scene as a placed block's do.

`AdjustMergedScene` (`Scene.h`) then filters the merged scene at the loose bar, triangulates again, filters again and runs one bundle adjustment: a model never closes its cycles exactly, and a block bent within the graph bars is off by about the loose bar at its far end, so its far matches stay in their tracks until the adjustment has straightened it. `Scene::ReconstructHierarchical` returns, and the reconstruction's shared tail — track filtering, re-triangulation, the final bundle adjustment at the bar, and the resection of every unregistered image (an unplaced block's images and the pool's among them) against the adjusted model (which always runs and closes with its full adjustment) — runs as for any other reconstruction.

## 3. Parameters and defaults

Every parameter is named with its default at the step of section 2 that reads it: the partition's in `ClusterConfig` (`SceneCluster.h`; `--max-views-per-cluster`, `--cluster-communities`), the block reconstruction's in `ReconstructionConfig` and `ResectionConfig` (`Scene.h`, `Resection.h`), the merge's in `GlobalAlignmentConfig` (`GlobalAlignment.h`; `--cluster-alignment`).

## 4. Invariants and constraints

The split/merge cycle minimizes peak memory by **moving** (not copying) expensive data:

```
Split:
  Global → Sub-scenes:  keypoints, descriptors, intra-cluster pairs   (MOVED)
  Global → Sub-scenes:  cameras                                       (CLONED)
  Global retains:       cross-cluster pairs, the pool images and their pairs

Pool resection:
  Block → Global:       a released image's keypoints, descriptors, pairs (MOVED, pairs to global IDs)
  Global → Block:       an inserted image's keypoints, descriptors, pairs to the block (MOVED, pairs to local IDs)
  Global → Block:       a camera the block does not have yet            (CLONED; one no image uses is deleted)

Merge:
  Sub-scenes → Global:  keypoints, descriptors, pairs                 (MOVED BACK)
  Sub-scenes → Global:  poses, tracks                                 (COPIED/APPENDED)
  Sub-scenes → Global:  camera intrinsics                             (AVERAGED)
```

Key invariants:

- Keypoints and descriptors exist in exactly one place at any time
- A pair lives in the block holding both its images, else in the global scene with global IDs; it moves with the images (split, pool resection, merge)
- `localToGlobal` stays strictly increasing, so every pair keeps `ID1 < ID2` in local and global IDs
- Colors are released during track reassembly (indices change) and must be rebuilt downstream

## 5. Validation of the shipped defaults

### Measured (2026-10-25, 8 threads, one run per arm, the 17 of the 18 test captures that split)

On identical saved blocks (each arm reading the control's blocks and reloading the same matched scene; `merge-retest-20261025.md` of the release tools):

| | the camera-vote merge it replaces | this merge | the final design (not shipped) | this merge, `--cluster-alignment 0` |
|---|---|---|---|---|
| blocks placed, of 103 | 90 | 102 | 102 | 99 |
| wrong blocks | 0 | 0 | 0 | 1 (alameda block 5, F1 0.9876 against 0.9972) |
| images registered | 9340 | 9348 | the same as this merge | |
| merge + its adjustment, wall s summed | 2786 | 1374 (alameda 1284 -> 417) | about this merge's | |

F1 against the replaced merge: 2 captures better, 2 worse (both thread-noisy: Palace, Outdoor_004), 8 ties on the approved and the new references; the resection now registers almost every image of the blocks the replaced merge left out, so its placement gain shows in blocks and time more than in F1. This merge and the final design place and register the same on 17 of 17 captures, F1 within the noise.

## 6. Rejected alternatives

The final design — extra modes on camera evidence, free space, the evidence product, a consensus over spanning forests and an exclusive-evidence bridge rule — places the same blocks on every capture measured and is recorded as an alternative in `docs/design/ExperimentRecord.md`, section "The block merge as one decision over the block graph (the final design, not shipped)".

### Decoupled R → s → t Estimation

The block graph is averaged by one routine, `AverageBlockPoses`, which solves rotation, then scale, then translation, never all seven parameters of a Sim(3) at once. Each subproblem is convex (or nearly so) on its own: rotation averaging on SO(3) has well-studied convex relaxations, scale averaging in log-space is linear least-squares, and translation averaging given known rotations and scales is a linear system. The scales and translations are reweighted toward the readings the solution satisfies (the inverse of each one's error, which their normal equations square), so a solve settles on one of a pair's readings rather than between them: the one nearest its least-squares start, which is why a reading cut in one round is judged again in the next, where the start has moved.

### Why a reading weighs the cameras of its weaker side

A transform can rack up inlier correspondences from a single repeated texture patch seen by one camera, or from one image whose keypoints happen to fall on a shared object — evidence that says nothing about whether the *block* agrees, only that one small part of it does. Counting the supporting cameras of the weaker side turns "many correspondences agree" into "many independent witnesses on both sides agree", and the share of the verified pairs' weight a reading reproduces is the one statistic that separates a gross reading from a right one (AUC 0.99 against 0.6-0.9 for the counts on the measured readings). As a factor of the weight and a witness of bridges the pairs lose nothing; as a gate on every reading they refused right readings of repeated facades (alameda), so they are not one.

### Why the cut needs no other selection

On the measured blocks the first reading of each direction is the right reading on every block pair where any reading is right; the two directions of a pair read alike on almost every pair, and where one is wrong the other and the cycles indict it. A mode search, a free-space test, an evidence product and a consensus over spanning forests (the final design) change no placement there, and each copy of a repeated structure they find is what makes the others necessary. What the averaging cannot judge is a bridge, a pair no cycle tests: it keeps a reading only with two cameras on its weaker side and the verified pairs not against it, which is what refuses a seam one camera holds and a symmetric bridge both directions read as the same copy.

### Union-Find with Guards

Track merging across blocks reuses the union-find pattern from `BuildTracks`, under three guards. Duplicate image: a joined track never holds one image twice. Length: a joined track never holds more than 255 observations (the inlier count is a byte). Reprojection: because sub-scene tracks have disjoint image sets by construction, the duplicate-image guard alone cannot catch a false match between two blocks, so the position of each root must reproject within the loose bar (12 px, judged as `FilterTracks` judges, behind the camera fails) into every posed observation of the other root (a root without a position is checked from the other side alone; one without posed observations offers nothing to check). It replaced a 3D distance of 2% of the bounding-box diagonal, which is one length for every depth: generous for a near point, strict for a far one whose depth two blocks measure poorly (#62). Every pair with a pool image is a connecting pair: a pool image's observations left its block's tracks, so its pairs are the only source of the tracks the post-merge resection reads for it. The one exception is the closure: a correspondence of a pair of placed blocks that the final block poses explain within the loose bar has already agreed with the placement, so it joins on the duplicate-image and length guards alone.

### Why a block left out is not resected as a block

A block the merged model does not hold is not a block about which nothing is known — it is one whose readings the averaging, the cut and the bridge witnesses could not tie to the model. Forcing it into the merged model anyway, as one rigid unit, would risk everything the model already holds on exactly the evidence that failed. What is not in question is the block's own images: many of them may still carry strong, correct pairs to the consensus. Merging the block's observations in without a pose and leaving its images to the ordinary post-merge resection lets each of them stand or fall on its own connections to the finished model, instead of the model standing or falling on all of them at once.

### Generalized relative pose on the 2D-2D correspondences of two blocks

Assumes both block "rigs" share the same metric scale, which does not hold after independent reconstruction; replaced by the readings above, which observe the scale (the generalized absolute pose with scale of one block's cameras against the other block's points, or a 3D-3D similarity).

### A block's own two-view geometry against its own poses

Tried as an internal-soundness check inside `GlobalAlignment` and abandoned: on a 7-scene benchmark it flagged every scene (11-72% violated pair weight) because two-view relative poses are unreliable on the low-parallax and homography-degenerate pairs such captures are full of.

## 7. Open items

- **A non-rigid block** — several bodies at different scales inside one block — cannot be placed by one similarity, and its neighbours' readings through it disagree; the averaging may then place it, or a rigid block next to it, wrong. Measured on older blocks (Outdoor_004 on the blocks of 2026-10-17: 3 of 7 blocks wrong, F1 0.50 against the final design's 0.73, which places such a block by one of its bodies through its extra modes). The current upstream (star evidence, no relative-pose fallback) makes rigid blocks on the new captures; 26ccbd51's sub-scene 1 and Outdoor_004's sub-scene 4 are non-rigid on the current blocks, and both are placed right.
- **Three planted scenes it places wrong**, none of which any capture of the 18 produces: a 180° chord carried by every camera of a block with no verified pairs to refuse it (only a free-space test sees it); a bridge whose two readings weigh alike a period of a repeated structure apart (only a search for a second mode sees the rival); a copy heavier than the truth on the pair a one-camera door closes.
- **A component the averaging places only in part** — blocks in another rotation frame, or tied to the gauge's side by readings without scale alone — loses the rest: their readings cannot be judged and go off, and their images go to the resection. Not seen on the 18 captures.
- **The 3D-3D alternative** (`--cluster-alignment 0`) is weaker: alameda places block 5 wrong (F1 0.9876 against 0.9972), 1c65aab4 places 2 of 4 blocks (the resection recovers the images, at 2.6 times the run's CPU), b68e320e one block fewer.
