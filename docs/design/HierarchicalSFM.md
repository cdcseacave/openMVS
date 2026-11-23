# Hierarchical SFM Pipeline

## 1. Purpose and scope

The pipeline solves a fundamental scaling problem: running bundle adjustment on thousands of images at once is slow and numerically fragile. The solution is **divide → reconstruct → reunite** — split the scene into manageable clusters, reconstruct each independently, then align and merge everything back into one coordinate system.

Four phases, orchestrated by `Scene::ReconstructHierarchical()`:

```
Phase 1: SceneCluster::Partition() + SceneBlocks::Extract()  → clusters become blocks, the rest is the pool
Phase 2: threadPool.detach_loop(blocks)                       → parallel incremental SFM
Phase 3: SceneBlocks::ResectPool()                            → unregistered images to the pool, registered into the block they connect to best
Phase 4: GlobalAlignment::MergeScenes(), in rounds            → the weak-image filter on every block, the merged model among them; read every
                                                                block pair, average the block graph, merge; the blocks left
                                                                out kept whole and read again against the merged model after
                                                                a pool step, released at the fixed point
```

In plain words: the images are split into groups (blocks), each block is reconstructed on its own, every image a block could not register (and every image no block took) is put in one common pool and tried by the block it is best connected to, and then the blocks are placed against each other and merged, each block (the model merged so far among them) first losing the images it holds on weak evidence; a block that could not be placed is kept whole and tried again against the merged model, until nothing changes, and only then are its images handed to the resection one by one.

## 2. Algorithm as implemented

### Phase 1 — Scene Clustering

**Goal**: partition images into sub-scenes of bounded size, each image belonging to at most one — an image no cluster holds well is left in the pool (Phase 3) — the sub-scenes share no cameras and overlap only in structure: the points the cameras on both sides of a cut see, which the merge registers the sub-scenes on. `--max-views-per-cluster` sets the ceiling (`ClusterConfig::maxViewsPerCluster`, default 200; 0 disables clustering); `ClusterConfig::SetMaxViews` derives the target a cluster aims at (`targetViewsPerCluster`, two thirds of the ceiling — default 133) and the floor under which a cluster is merged away (`minViewsPerCluster`, four fifteenths of the ceiling — default 53) from that one ceiling, so a user-given `--max-views-per-cluster` and the compiled-in defaults follow the same rule.

#### Covisibility Graph

A weighted undirected graph is built where nodes are images and edge weights are composite pair weights. Edges below `minPairWeight` (0.9, `ClusterConfig`) are discarded. The evidence behind that weight counts a pair's dense inliers in proportion to the matcher's frame target (`denseFrameInliers` 50 over `denseMatchesPerFrame` 2000, `PairsWeightingConfig`): a whole frame's worth of dense inliers is a modest descriptor pair's evidence, 50 weighted inliers, so a wide, imprecise dense-only overlap cannot outweigh the descriptor-verified pairs around it, while two dense-only pairs of one image keep the ratio of their counts and a textureless interior keeps its links in order. The graph is stored in CSR format for compatibility with graph partitioning libraries. If the scene arrives with matches but no tracks, they are built once here so the seam statistics the refinement passes below read have something to read; each sub-scene rebuilds its own tracks again once it is split off.

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

Each sub-scene — a **block** from here on, `GlobalAlignment`'s own word for it — lives in its own arbitrary coordinate system. `GlobalAlignment::MergeScenes` treats the blocks as the nodes of a small graph and every block pair the cross pairs join as an edge holding the readings of its Sim(3); it places the blocks as the view-graph filter places images — one robust averaging of the kept readings, every reading the averaged poses contradict cut, repeated until the kept set settles — and merges the model holding the most images back into the scene; every block outside it is either kept whole for the next round or, in the round that releases it, merged without a pose for the post-merge resection to recover (see *The merge in rounds* below).

#### Readings

Every cross pair, however light, is grouped by its block pair (`MapBlockPairLinks`): a block pair is read from all its cross pairs together, so two light pairs of one camera each add up to what one heavy pair carries (no per-pair bar). Every correspondence of a pair is collected in both directions (`CollectPairObservations`): a match counts in the direction "B's cameras on A's points" when its A endpoint lies on an inlier track of A, and in the other when its B endpoint lies on one of B. How a pair is read depends on `GlobalAlignmentConfig::alignment` (`--cluster-alignment`):

- **`ALIGN_CAMERAS`** (1, the default) — **each direction is read once**: the rig of one block's cameras against the other block's points by PoseLib's generalized absolute pose with scale (RANSAC on the resection's PnP options), refined over its inliers with Ceres against every bearing it explains at `maxReprojError` (4 px, the resection's own), the residual being the chord between observed and predicted bearings in each camera's own pixels under a Huber loss. A direction whose correspondences come from one camera gives **no reading**: one centre observes no scale, and a block is joined by all its cameras together or not at all; its pair is read by the other direction if that one is a rig, and a block no rig reading places is released at the fixed point (*The merge in rounds*), its images registered one by one. A direction needs `minCommonTracks` correspondences and as many inliers. A reading observes a scale when at least two cameras each hold `minCameraInliers` (10) of its inliers and those cameras spread over at least `minRigSpreadRatio` (0.03) of their points' median depth; a rig with one such camera observes none, however many cameras hold a few inliers beside it. **The two directions stay two readings** of their pair: they are not grouped, and each is a measurement of its edge; where they agree they are each other's test.
- **`ALIGN_POINTS`** (0) — one 3D-3D similarity by RANSAC over the matches of all the pair's cross pairs (pooled, as the camera alignment's) whose two endpoints both lie on an inlier track of their block (`simInlierThresholdFactor` 0.01 of the destination cloud's diagonal, `minSimInlierRatio` 0.3, `simRansacMaxIters` 10000).

Every reading is scored on every correspondence of its pair (`ScoreSeam`): a camera of either block that holds `minCameraInliers` inlier correspondences supports it, unless it is an image a pool step registered on the merged model (*The merge in rounds*), and the reading's camera count is the supporting cameras of its **weaker side** — a side held by one camera weighs one, however many correspondences that camera carries (the frame misplaced inside its own block that carries a whole seam on its side). It is also read against the **verified image pairs** across its pair (`MeasureSeamPairs`), every pair with a relative pose read, the pairs together: under `minPairInliers` (30) weighted inliers in total they say nothing; a pair agrees when the relative rotation and the baseline direction the reading gives its two images are within `maxPairRotationResidual` (5°; the baseline at `atan(tan 5° / ray angle)`) of what it measured; the pairs' **share** is the inlier weight that agrees over all read (1 when none is read), and they **hold** against the reading when more weight disagrees than agrees.

**The entry test** (`GlobalAlignment::RefusedReading`), one for both alignments: a reading enters the block graph only with `minSeamWeight` (2) supporting cameras on its weaker side, an observed scale, and its verified pairs not holding against it; it weighs its weaker side's supporting cameras times its pairs' share (`GlobalAlignment::ReadingWeight`): the camera count is how much evidence the reading carries, the share how far that evidence agrees. An image the pool step of a round registered on the merged model supports no reading of any later round (`GlobalAlignment::poolImages`, filled by `MergeInRounds`): its pose was fitted to the model, so a block that reaches the model only through such images — a chain of single images, not a block — has no support that tests the scale of its reading, and its reading is refused by the cameras. Images the pool resection registered inside a block, before the merge (Phase 3), are the block's own cameras and count. A reading refused leaves its block pair to its other readings; a block no reading places is kept whole and read again against the merged model in the next round of the merge, and released to the post-merge resection at the fixed point (*The merge in rounds*).

#### Selection: one averaging, one cut, repeated

Every connected component of the kept readings (all of them at first) is averaged by `AverageBlockPoses`, gauged at the block its readings weigh most on:

- **rotations** by `GlobalRotationEstimator` (its maximum spanning tree, L1, then its own Geman-McClure reweighting; the reading weights scaled to a median of `GlobalRotationEstimatorOptions::weightMedian` inside it, so only their ratios count), run again over whatever a run leaves in another frame;
- **log-scales**, of the readings within the rotation bar, and **translations** — each reading's tied at the centroid of its own seam cameras (`SeamCandidate::centre`: the cameras supporting the reading on either side, block B's where the reading puts them in A's frame), so it depends neither on where a block's arbitrary origin lies nor on any camera of a block away from the seam — of the readings within the scale bar, every reading reweighted by the inverse of its error (floored at 1e-4 in log-scale and 1e-3 of the seam cameras' spread) as the estimators' row scale, 40 rounds at most: their normal equations weigh its square, which settles on one reading rather than on the L1 median. A block the metric solves do not reach from the gauge gets no pose.

Every reading is then judged against the poses of its component: kept when its rotation is within `maxGraphRotationResidual` (5°), its scale ratio within `maxGraphScaleResidual` (1.20) and its seam within `maxGraphTranslationResidual` (0.20) of its own size: the centroid of the reading's seam cameras, carried into B by the reading and by the averaged poses, lands at two points whose distance is read as a fraction of the seam cameras' spread (`SeamCandidate::extent`, the distance between the two farthest apart, `CentresSpread`). Only the reading and its supporting cameras enter, so a camera a block holds far from its place, away from the seam, changes neither the point nor the unit. A reading whose two blocks the averaging did not both place cannot be judged and is off. The averaging and the cut repeat — a reading cut in one round is judged again in the next — until the kept set does not change, `kRobustRounds` (10) rounds at most.

**A block pair no cycle tests.** A block pair that no cycle of the other block pairs tests is judged as every other: its readings pass the entry test and stand on the cut against their component's poses, and a block no reading places stays out of the model for the next round.

**The model.** Each connected component of the readings left is a model, its block poses the last averaging's; the one holding the most images is the scene. A block with no pose is a model of its own, with its reason — no reading entered (it has no cross pairs, or the entry test refused every reading of it), every reading off, or no pose from the averaging — in the log and the merge report.

#### Merge and the merged model's adjustment

The blocks of the merged model are transformed by their poses, a camera shared across blocks gets the average of its intrinsics, and keypoints, descriptors and pairs move back from every block (reversing `SceneBlocks::Extract`). Tracks join under three guards (one image once per track, at most 255 observations, and for a match of a connecting pair the point of each side, where it has one, reprojecting within `maxJoinReprojError`, 48 px, into every posed observation of the other), and the triangulation and the track filter of the merged model's adjustment decide each observation. **Closure, a join priority**: the correspondences of the cross pairs of at least `minCommonTracks` (25) weighted inliers of every pair of placed blocks — whether or not a reading of it was kept — that the final block poses explain within the loose bar (`looseSeamFactor` 3 × `maxReprojError` = 12 px) join first; then every track-forming match of every connecting pair (two blocks, heavy and light cross pairs alike, or a pool image) is joined, so a seam-consistent match takes a feature before a competing match to the same image; intra-block pairs are skipped. A block outside the merged model is kept whole in its own scene (`releaseUnplaced` false: its features, pairs and tracks stay there, the connecting pairs to its images are skipped), or, in the round that releases it, merged without its poses and its tracks without positions, so the resection can register its images one by one. A single block (a split the pool resection leaves with one block) is merged in its own frame through the same path, so its intrinsics and its tracks reach the scene as a placed block's do.

`AdjustMergedScene` (`Scene.h`) then filters the merged scene at the loose bar, triangulates again, filters again and runs one bundle adjustment: a model never closes its cycles exactly, and a block bent within the graph bars is off by about the loose bar at its far end, so its far matches stay in their tracks until the adjustment has straightened it.

#### The merge in rounds

`SceneBlocks::MergeInRounds`, called by `Scene::ReconstructHierarchical`, runs the merge as one iterative stage. Each round calls `MergeScenes` and `AdjustMergedScene` and recounts the calibrated images. The model merged is the one carrying the most images in round 1, and from round 2 on the one holding block 0, the merged model read again (`MergeScenes`' `anchorBlock`): the merged model is the body that grows, and a heavier model of blocks left out never takes its place.

**The weak-image filter before a round** (`SceneBlocks::FilterWeakImages`): every round but the releasing one first runs `FilterWeaklyConnectedImages` (at its defaults, `IncrementalReconstruction.md`) on every block, in parallel: the clusters' blocks in round 1, the merged model (block 0) and the blocks left out after it (a block kept whole is checked again in the next round). It is a safety check for images a block registered on weak evidence — points on too little of the frame, seen from almost one place, or tied to the block by fewer than two neighbours — and not a test of the pose: on six captures it removed 46 block images before the first merge, 10 of them wrong, 17 right and 19 without a reference pose, while the blocks held 120 wrong images. An image it removes loses its pose and stays in its block, so the round reads the block's poses and tracks without it; once the block is merged in, the image is in the global scene with its features and no pose, a candidate of the pool step and of the final pass's resection like an image of the pool. Its tracks come back from its pairs: the ones to images outside its block at that merge, every one at the next round's merge, where it is an image in no block. A block left without a posed image hands every image to the pool and is removed (`RemoveEmptyBlocks`); with no block left nothing is merged and every image is left to the final pass's resection. An image taken from the merged model loses its pose in the global scene too (the merge copies a block's poses, it never clears one); when a pool step registers it again it does not count as an image that entered, so the images the filter and the pool step trade do not hold the rounds open. On nine captures it took images from the merged model on two (one on 6DCA1E8E; five on CAB_0120, which the previous pool step had entered and the closing filter removes too); on the merged model it costs 0.6 to 0.9 s a round, 12 s on alameda's 1693 images.

While blocks are left out:

1. **Pool step**: the whole-scene resection (`Resection::RegisterImages(false)`, the final pass's own function, filtering the tracks only when it registered an image) on the merged model; its candidates are the images in no block (the kept blocks' images hold no features in the global scene). The images it registers (posed after the step, not before it) are collected over the rounds into one set, which the next rounds' merges read: they are no supporting cameras of a reading.
2. **Regroup** (`SceneBlocks::RegroupAfterMerge`): block 0 becomes the merged model — every posed image of the global scene, extracted as a cluster is, its features, pairs and the scene's tracks moved in — followed by the blocks the round left out, as they are.
3. **Fixed point**: when the pool step entered no image and the last round placed no block (the second round always reads the blocks left once), the next round is the last: it reads them once more and releases whatever it does not place, merged without poses, to the final pass's resection.

`Scene::ReconstructHierarchical` returns, and the reconstruction's shared tail — track filtering, re-triangulation, the final bundle adjustment at the bar, and the resection of every unregistered image (an unplaced block's images, the pool's and the ones the filter removed before a round among them) against the adjusted model (which always runs and closes with its full adjustment) — runs as for any other reconstruction.

## 3. Parameters and defaults

Every parameter is named with its default at the step of section 2 that reads it: the partition's in `ClusterConfig` (`SceneCluster.h`; `--max-views-per-cluster`), the block reconstruction's in `ReconstructionConfig` and `ResectionConfig` (`Scene.h`, `Resection.h`), the merge's in `GlobalAlignmentConfig` (`GlobalAlignment.h`; `--cluster-alignment`).

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
- A cross-block track join the reprojection guard refuses leaves both tracks as they were; the guard reads only observations in posed images (asserted), and `maxJoinReprojError` is never negative (asserted)
- From round 2 of the merge, block 0 is the merged model (`SceneBlocks::RegroupAfterMerge`: every posed image of the global scene extracted as a cluster, the scene's tracks moved into it) and is placed in every round (asserted); a kept block's images hold no features and no pose in the global scene until the round that releases them

```
Merge round r >= 2 (RegroupAfterMerge):
  Global → Block 0:     the posed images' keypoints, descriptors, pairs (MOVED), tracks (COPIED, then released from the global scene)
  Kept blocks:          left as they are (their images keep their features in their own scene)
```

## 5. Validation of the shipped defaults

The merge with its entry test against the same merge without it (every reading entering, the bridge rule judging the block pairs no cycle tests), from the same saved matched scenes:

| capture | merge | right / wrong | pose score | registered | blocks placed per round | refusals, all rounds (cameras / scale / pairs) |
|---|---|---|---|---|---|---|
| CAB_0120 | without the entry test | 20 / 38 | .1253 | 58 / 346 | 2/6, 1/5, 1/5 | – |
| CAB_0120 | entry test | 20 / 38 | .1253 | 58 / 346 | 2/6, 1/5, 1/5 | 0 / 1 / 0 |
| 5ada248e | without | 517 / 3 | .9537 | 520 / 540 | 4/10, 7/7 | – |
| 5ada248e | entry test | 517 / 3 | .9406 | 520 / 540 | 4/10, 7/7 | 4 / 0 / 1 |
| 6DCA1E8E | without | 467 / 8 | .7466 | 599 / 872 | 11/15, 1/5, 1/5 | – |
| 6DCA1E8E | entry test | 468 / 7 | .7534 | 602 / 872 | 11/15, 1/5, 1/5 | 3 / 3 / 5 |
| alameda | without | 1731 / 0 | .9970 | 1734 / 1734 | 15/15 | – |
| alameda | entry test | 1731 / 0 | .9970 | 1734 / 1734 | 14/15, 2/2 | 4 / 2 / 3 |
| Palace | without | 493 / 1 | .8213 | 494 / 509 | 5/5 | – |
| Palace | entry test | 494 / 1 | .8203 | 495 / 509 | 5/5 | 0 / 0 / 0 |
| 4e5c6f6d | without | 249 / 33 | .9149 | 291 / 302 | 4/4 | – |
| 4e5c6f6d | entry test | 249 / 33 | .9140 | 288 / 302 | 4/4 | 0 / 0 / 0 |
| Outdoor_004 | without | 767 / 7 | .7329 | 774 / 824 | 8/8 | – |
| Outdoor_004 | entry test | 767 / 7 | .7326 | 774 / 824 | 8/8 | 2 / 0 / 0 |

Each row is one 8-thread reload of the capture's saved matched scene with the join guard at 48 px. The entry test places the same blocks in every round on six captures; on alameda one right block pair whose verified pairs are split (666 for, 697 against) is refused, and its block is placed in the next round against the merged model. The score differences are within each capture's swing between reloads (.014 on 6DCA1E8E, .02 on 5ada248e, where the same blocks are placed after the same refusals); 6DCA1E8E differs by one image whose score is 0.05, Palace by one image the merge without the test leaves unregistered. The refusals count every refusal of every round, a reading refused again in a later round each time: of the 28, 21 fall on block pairs whose two blocks the merge places right, and every such block is placed, at worst one round later; the other 7 fall on two blocks whose start was wrong (CAB_0120, 1), on a block no run places (6DCA1E8E, 1) and on a block the reference does not cover (6DCA1E8E, 5).


The pool-registered images as supporting cameras (`GlobalAlignment::poolImages`), against the entry test counting them as every other camera; each row is a reconstruction of the capture:

| capture | rule | runs | result |
|---|---|---|---|
| 540-image capture, observation budget 2000 per image | without | 8 | the late-joined part of 206 images ends at 0.79 of its size in 4 runs (434 to 440 right images, 78 to 84 wrong), right in the others |
| 540-image capture, budgets 1500 and 2000 | with | every run measured | the part ends within 0.6% of its size, 518 right and 2 wrong; run time +40% (the 206 images are registered one at a time by the final resection) |
| 872-image capture, growth angle 1.5 degrees | without / with | 1 each | 424 right / 51 wrong without, 468 / 7 with |
| 225 images; 1087 panoramas | without / with | 1 each | unchanged, time included |

A reading's translation read at its own seam cameras, against the rule it replaced (tied at the two blocks' camera centroids, read in units of the smaller block's camera box), both with the weak-image filter on every block and the merged model; registered / right / wrong and pose score:

| capture | blocks' centroids and box | the seam's cameras |
|---|---|---|
| Outdoor_004 | 774 / 767 / 7, .7329 | 774 / 767 / 7, .7328 |
| 6DCA1E8E | 600 / 468 / 7, .7514 | 600 / 468 / 7, .7248 |
| 4e5c6f6d | 290 / 249 / 33, .9147 | 289 / 249 / 33, .9145 |
| 5ada248e | 520 / 517 / 3, .9366 | 520 / 517 / 3, .9497 |
| CAB_0120 | 59 / 21 / 38, .1326 | 59 / 21 / 38, .1326 |
| Palace | 498 / 497 / 1, .8243 | 498 / 497 / 1, .8222 |
| alameda | 1734 / 1731 / 0, .9970 | 1734 / 1731 / 0, .9969 |
| Outdoor_004 with one camera of a block a billion units from its place (a development build, no filter before the merge) | 414 / 414 / 0, .5578 | 767 / 767 / 0, .7331 |

Each row is one 8-thread reload of the capture's saved matched scene; the score differences of the first seven rows are within each capture's swing between reloads, and the same blocks are placed in the same rounds. Read as shadow selections on the same 265 readings (264 right against the references, the wrong one on a block pair no cycle tests, kept by either rule), the two rules place the same blocks in every merge; at 0.20 the seam rule cuts 6 right readings and the old one 4. The two more are the two directions of one seam of six cameras whose spread is a tenth of the smaller block's extent, 0.7 of their own spread from the solution. A seam's spread is 0.49 / 1.00 / 1.60 of the old unit at the 10th percentile / median / 90th; the right readings sit within 0.08 of their seam's spread at the 95th percentile and 0.24 at the 99th. No wrong reading came within reach of the translation bar in these runs, so they do not say how far it could be raised.

The join guard at 48 px against the guard off, 12 and 24 px (eight captures, one 8-thread reload each): it refuses 2-7% of the attempted joins, the final tracks are as long as with the guard off, and the pose score does not separate the four values.

## 6. Rejected alternatives

The final design — extra modes on camera evidence, free space, the evidence product, a consensus over spanning forests and an exclusive-evidence bridge rule — places the same blocks on every capture measured; it is not shipped (first item below).

- **The block-graph merge of that final design**: the verified pairs as an entry gate refused right readings (5992d620 block 1 wrong, F1 -0.155) where as a factor they remove alameda's wrong block 3 (+0.0076); a free-space conflict factor separates copies (62-66% against 15-17%); a camera trigger instead of the unexplained-share trigger (alameda merge 906 s -> 270 s, 5936fac3 -0.0051) needs an exclusive-evidence bridge rule (else b68e320e's 55-image block is lost, F1 -0.119); a consensus over spanning forests moved only Outdoor_004 (+0.1761); closing at the plain bar costs 0.0126 accuracy. Not shipped: the simple merge gives the same blocks on 17 of 17 captures with about 750 lines fewer (planted failure tests HeavyFalseChordTest, CopyOnDoorTriangleTest, BridgeEqualReadingsTest).
- **The interleaving gate** (a placement vetoed when under 0.8 of the moved block's cameras keep an own nearest neighbour): it never refused a wrong placement on eight captures and refused right blocks (alameda, 4 of 14).
- **Skipping the coupling test for a cluster at the target size**: such a cluster absorbed a community under the floor over a thin interface (the seven-image appendage of alameda), so the partition tests the coupling of every merge.

- **The per-pair bars on the readings and the witness** (a cross pair of at least 25 weighted inliers to link two blocks, of 30 to witness a reading): two light pairs of one camera each say nothing alone where their sum does; with them same-image accuracy is .9114 on 5ada248e (.9424 pooled), within the noise elsewhere, and .619 on 6DCA1E8E (.604 pooled, not explained).
- **No guard on the cross-block join**: the joins it lets through leave tracks at points the merged model's filter and triangulation must repair (right after the join the mean reprojection error is 2.2 million px on 26ccbd51 and 44 thousand px on 5ada248e, against 1.7-2.0 px with the guard at 48 px), for no better final scene.
- **The join guard at 12 or 24 px**: it refuses 7-25% of the attempted joins at 12 px against 2-7% at 48 px, and at 12 px the final tracks are shorter (more of them with two views only); the pose score does not separate the values.
- **A reading's translation tied at the two blocks' camera centroids and read in units of the smaller block's camera box**: one camera a block's adjustment leaves far from its place moves the centroid and inflates the box, and every seam of the block is misread (with one a billion units away five of eight blocks land 5 to 15 km off; last row of the table in section 5).
- **A bridge rule** (a block pair no cycle tests keeps a reading only with 2 supporting cameras on its weaker side and its verified pairs not against it): on eight captures it turned off one reading (6DCA1E8E), which the averaging's cut also removes (3.6°, 6%, 9% of the smaller block's camera extent from the solution); a block it would keep out gets another chance in the next round.
- **The two directions of a join required to agree** (scale within 3%, translation within 0.03 of the smaller block's camera extent): right joins differ by up to 21% in scale and 0.14 of that extent in translation (5ada248e, Palace), and the one questionable join sits inside that range.
- **A block pair refused whole when one of its directions fails the entry test**: it releases blocks the other direction places right (5ada248e: six blocks released and registered image by image).
- **Both directions required, and their agreement within 3% / 0.03** (the strict form of the entry test): 6DCA1E8E breaks into 12 models and registers 122 of 872 images.
- **The merged model chosen by the images its blocks registered before the pool step**: it changes no outcome where blocks join, and on 6DCA1E8E it picks a part the reference does not cover.
- **Dense matches counted at 1/40 in the merge's bars** (a direction's 25 correspondences and 25 inliers, a supporting camera's 10 inliers): on 6DCA1E8E 11 readings fall under the direction bars (for one, 8957 correspondences and 479 inliers weigh 237 and 12), 5 more lose their cameras or their scale, round 1 places 3 of 15 blocks, and 202 of 872 images are registered (602 with the raw counts).
- **The pairs' share alone as the weight**: Palace .806-.807 against .820, a 53-inlier two-camera reading weighing as much as a 38k-inlier one.
- **Blocks left out never released** (the rounds end with their images unregistered): 6DCA1E8E registers 614 of 872 images against 846, at the same accuracy.

### Decoupled R → s → t Estimation

The block graph is averaged by one routine, `AverageBlockPoses`, which solves rotation, then scale, then translation, never all seven parameters of a Sim(3) at once. Each subproblem is convex (or nearly so) on its own: rotation averaging on SO(3) has well-studied convex relaxations, scale averaging in log-space is linear least-squares, and translation averaging given known rotations and scales is a linear system. The scales and translations are reweighted toward the readings the solution satisfies (the inverse of each one's error, which their normal equations square), so a solve settles on one of a pair's readings rather than between them: the one nearest its least-squares start, which is why a reading cut in one round is judged again in the next, where the start has moved.

### Why a reading needs two cameras on its weaker side, a scale, and its pairs

A transform can rack up inlier correspondences from a single repeated texture patch seen by one camera, or from one image whose keypoints happen to fall on a shared object — evidence that says nothing about whether the *block* agrees, only that one small part of it does. Requiring two supporting cameras on the weaker side turns "many correspondences agree" into "two independent witnesses on each side agree"; a reading without a scale cannot place its block, only turn it. The share of the verified pairs' weight a reading reproduces is the one statistic that separates a gross reading from a right one (AUC 0.99 against 0.6-0.9 for the counts on the measured readings), so it is both a factor of the weight and, held against the reading, a refusal; the other factor, the supporting cameras, keeps a reading of two cameras from weighing as much as one of forty. A right reading the test refuses waits: its block is read again against the merged model in the next round.

### Why the cut needs no other selection

On the measured blocks the first reading of each direction is the right reading on every block pair where any reading is right; the two directions of a pair read alike on almost every pair, and where one is wrong the other and the cycles indict it. A mode search, a free-space test, an evidence product and a consensus over spanning forests (the final design) change no placement there, and each copy of a repeated structure they find is what makes the others necessary.

### Union-Find with Guards

Track merging across blocks reuses the union-find pattern from `BuildTracks`, under three guards. Duplicate image: a joined track never holds one image twice. Length: a joined track never holds more than 255 observations (the inlier count is a byte). Reprojection, for the matches of the connecting pairs: the point of each side, where it has one, reprojects within `maxJoinReprojError` (48 px; 0 = off) into every posed observation of the other, judged as `FilterTracks` judges; the closure's correspondences skip it, since the final block poses already explain them within the loose bar. Every pair with a pool image is a connecting pair: a pool image's observations left its block's tracks, so its pairs are the only source of the tracks the post-merge resection reads for it.

### Why a block left out is not resected as a block

A block the merged model does not hold is not a block about which nothing is known — it is one whose readings the averaging and the cut could not tie to the model. Forcing it into the merged model anyway, as one rigid unit, would risk everything the model already holds on exactly the evidence that failed. What is not in question is the block's own images: many of them may still carry strong, correct pairs to the consensus. Merging the block's observations in without a pose and leaving its images to the ordinary post-merge resection lets each of them stand or fall on its own connections to the finished model, instead of the model standing or falling on all of them at once.

### Generalized relative pose on the 2D-2D correspondences of two blocks

Assumes both block "rigs" share the same metric scale, which does not hold after independent reconstruction; replaced by the readings above, which observe the scale (the generalized absolute pose with scale of one block's cameras against the other block's points, or a 3D-3D similarity).

### A block's own two-view geometry against its own poses

Tried as an internal-soundness check inside `GlobalAlignment` and abandoned: on a 7-scene benchmark it flagged every scene (11-72% violated pair weight) because two-view relative poses are unreliable on the low-parallax and homography-degenerate pairs such captures are full of.

## 7. Open items

- **A non-rigid block** — several bodies at different scales inside one block — cannot be placed by one similarity, and its neighbours' readings through it disagree; the averaging may then place it, or a rigid block next to it, wrong. Measured on older blocks of Outdoor_004 (3 of 7 blocks wrong, F1 0.50 against the final design's 0.73, which places such a block by one of its bodies through its extra modes). The current upstream (star evidence, no relative-pose fallback) makes rigid blocks on the new captures; 26ccbd51's sub-scene 1 and Outdoor_004's sub-scene 4 are non-rigid on the current blocks, and both are placed right.
- **A light false pair lending a camera**: pooled readings count every camera a cross pair adds, so a reading carried by a few false or copied pairs can outweigh the truth on a cycle (4e5c6f6d with three false pairs across one block pair places blocks 0 and 1 wrong). Nothing in the merge separates it.
- **Wrong images inside a block** are placed with their block: the merge assumes every image of a block is right in it.
- **Two planted scenes it places wrong**, neither of which any capture of the 18 produces: a 180° chord carried by every camera of a block with no verified pairs to refuse it (only a free-space test sees it); a block pair no cycle tests whose two readings weigh alike a period of a repeated structure apart (only a search for a second mode sees the rival).
- **A component the averaging places only in part** — blocks in another rotation frame — loses the rest: their readings cannot be judged and go off, and their images go to the resection. Not seen on the 18 captures.
- **A right block pair whose verified pairs are evenly split** is refused by the entry test and placed one round later (alameda: 666 for, 697 against; the run's wall time stays within a minute of the merge without the test).
- **The closure is a join priority that skips the join guard**: dropping it is a possible simplification, not measured (it would not be byte-identical).
- **The standalone community's seam bar** (`IsStrongSeam`, over the whole scene's track graph) counts something else than the merge's bar; a seam bar of the partition that counts what the merge counts is not built.
- **The 3D-3D alternative** (`--cluster-alignment 0`) is weaker: alameda places block 5 wrong (F1 0.9876 against 0.9972), 1c65aab4 places 2 of 4 blocks (the resection recovers the images, at 2.6 times the run's CPU), b68e320e one block fewer.
