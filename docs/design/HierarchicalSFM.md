# Hierarchical SFM Pipeline — Core Design

## Overview

The pipeline solves a fundamental scaling problem: running bundle adjustment on thousands of images at once is slow and numerically fragile. The solution is **divide → reconstruct → reunite** — split the scene into manageable clusters, reconstruct each independently, then align and merge everything back into one coordinate system.

Three phases, orchestrated by `Scene::ReconstructHierarchical()`:

```
Phase 1: SceneCluster::SplitScene()         → partition into sub-scenes
Phase 2: threadPool.detach_loop(subScenes)  → parallel incremental SFM
Phase 3: GlobalAlignment::MergeScenes()     → measure seams, place blocks, merge
```

---

## Phase 1 — Scene Clustering

**Goal**: partition images into sub-scenes of bounded size. `--max-views-per-cluster` sets the ceiling (`ClusterConfig::maxViewsPerCluster`, default 150; 0 disables clustering); `ClusterConfig::SetMaxViews` derives the target a cluster aims at (`targetViewsPerCluster`, two thirds of the ceiling — default 100) and the floor under which a cluster is merged away (`minViewsPerCluster`, four fifteenths of the ceiling — default 40) from that one ceiling, so a user-given `--max-views-per-cluster` and the compiled-in defaults follow the same rule.

### Covisibility Graph

A weighted undirected graph is built where nodes are images and edge weights are composite pair weights. Edges below `minPairWeight` (3.0) are discarded. The graph is stored in CSR format for compatibility with graph partitioning libraries. If the scene arrives with matches but no tracks, they are built once here so the seam statistics the refinement passes below read have something to read; each sub-scene rebuilds its own tracks again once it is split off.

### Aggregative Clustering

Bottom-up greedy merging that respects covisibility structure:

1. Initialize each image as a singleton cluster
2. Build a priority queue of edges sorted by weight (descending)
3. Pop the highest-weight edge; merge the two clusters unless the merge would cross `maxViewsPerCluster`, or would push the combined size past `targetViewsPerCluster` — except when the smaller side is still under `minViewsPerCluster` and has to go somewhere, and even then only while the larger side has not itself already exceeded `targetViewsPerCluster` — or would join two clusters both already at or past the floor over an interface thinner than `minClusterCoupling` (0.05, 0 = disabled) of the weaker side's own internal weight
4. Periodically rebuild the PQ and re-run the local-search pass below (every `max(10, maxViewsPerCluster / 10)` merges) to keep edge weights consistent

### Cluster Refinement

Seven passes tidy the greedy result and make every remaining cluster boundary usable by the merge:

1. **RefineClustersLocalSearch** — up to 20 iterations: move boundary images to whichever cluster maximizes internal connectivity (modularity + balance)
2. **MergeSmallClusters** — absorb clusters below `minViewsPerCluster` into the most-connected neighbor, with `maxOverCapacity` (20) slack
3. **RefineClustersBalance** — conservatively move well-connected boundary images out of the largest cluster into smaller neighbors, gated by a minimum affinity ratio, to shorten the critical path of concurrent sub-scene reconstruction
4. **RefineClustersSplitDisconnected** — split clusters whose images form disconnected components in the covisibility graph
5. **RefineClustersSplitThinWaist** — split any cluster whose best balanced bipartition is joined below the `minClusterCoupling` seam: the thin-waist clusters that would otherwise reconstruct as two independently scaled blocks
6. **RefineClustersForSeams** — the cuts as the merge will see them. `MergeLeafClusters` merges away a cluster whose seams the merge could never attach — fewer than `minClusterDegree` (2, capped by the neighbours it actually has) strong neighbours, where a strong neighbour needs a seam of at least `minSeamTracks` (75) seam-usable tracks with, on both sides, at least `minSeamCameras` (3) cameras each carrying `minSeamCameraTracks` (30) of them (`IsStrongSeam`) — into the neighbour it shares the most usable tracks with, as long as the result still fits `maxViewsPerCluster + maxOverCapacity`. `RepairClusterSeams` then widens a seam that carries tracks enough but holds them in too few cameras, moving boundary images across it to raise `min(camerasA, camerasB)` toward `minSeamCameras`. A final `MergeSmallClusters` pass mops up whatever either step left under the floor
7. **RefineClustersRescueOrphans** — absorb remaining small orphans into neighbors

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

## Phase 3 — Global Alignment (seams, consensus, placement, loops)

Each sub-scene — a **block** from here on, `GlobalAlignment`'s own word for it — lives in its own arbitrary coordinate system. `GlobalAlignment::MergeScenes` measures how adjacent blocks relate, decides which of those measurements can be trusted, places blocks one at a time into one or more models, and merges the model holding the most images back into the scene; every block outside it is merged without a pose, for the post-merge resection to recover.

### Measuring a seam

Every pair of blocks the cross-cluster pairs connect is measured in both directions, in whichever mode `GlobalAlignmentConfig::alignment` (`--cluster-alignment`, default `ALIGN_CAMERAS`) selects:

- **`ALIGN_POINTS`** — a 3D-3D similarity by RANSAC over matches whose two endpoints both hit an inlier-track cache, one per block (`simInlierThresholdFactor` 0.01, a fraction of the destination cloud's bounding-box diagonal; `minSimInlierRatio` 0.3; `simRansacMaxIters` 10000).
- **`ALIGN_CAMERAS`** (default) — one block's cameras as a generalized rig against the other block's inlier tracks (PoseLib's generalized absolute pose with scale), so a match needs only one endpoint on a track; both directions — B's rig on A's points, A's rig on B's points — are estimated.

Both modes are judged against the same pixel bar, `maxReprojError` (4px, the resection's own). A direction counts only once its inliers reach `minCommonTracks` (25) and, once the other direction has already counted, explains at least `minCrossSupportRatio` (0.5) of what that other direction explains — the "union support" gate.

Every image with at least `minVoteCorrespondences` (10) correspondences to the seam casts a vote: it supports when its inliers reach `minVoteInliers` (30), spread across at least `minVoteCoverage` (0.25) of a `kVoteGridCells`×`kVoteGridCells` (4×4) image grid, and its own inlier share reaches `kVoteSupportFraction` (0.3); it contradicts when it has at least `minVoteInliers` correspondences and its inlier share falls under `kVoteContraFraction` (0.1). A direction's "camera votes" gate then asks, on each side, for support at or above `minCameraVoteRatio` (2.0, or `minCameraVoteRatioVerified` 3.0 once the model it is being placed against rests on verified seams only) times contra, and at least `minSupportingCentres` (3) distinct supporting camera centres. A third gate, "interleaving", vetoes a transform that leaves the two blocks' cameras mixed together: after the transform, at least `minOwnNeighbourFraction` (0.8) of the moving cameras must have one of their own as their nearest neighbour rather than one of the other block's.

A direction can measure a scale only when its supporting centres spread, relative to the inliers' median depth, by at least `minRigSpreadRatio` (0.03); one that cannot borrows the scale the other direction measured, rescaling about its own rig's centre so the rig keeps the place it already had.

When both directions pass their gates they must agree — rotation within `maxSimRotationError` (3°) and scale ratio within `maxSimScaleRatio` (1.1) — and are then refined together, with Ceres, into one seam over the union of both inlier sets. Disagreement is settled by the camera vote totals when one direction beats the other by `voteMargin` (1.5) or more; otherwise both directions are kept, unresolved, for the seam graph to judge. A pair with only one measurable direction stands alone, to be confirmed later by the graph or by the placement. A pair every direction's gates refuse leaves no seam at all — unless its own cameras are themselves split over it, in which case it is left for the fold test described under "What does not fit" below.

Every seam carries a `weight`: the smaller of its total camera support and `maxVoteWeight` (30), the one number every later averaging and ranking reads.

### The seam graph

A seam is measured between two reconstructions that know nothing of each other, so a wrong one cannot be told from a right one by its own evidence — only the cycles it sits in can. Candidates are grouped into components by the blocks they share; a component of three or more blocks is averaged robustly (rotation, then scale, then translation, reweighting up to `kRobustRounds` (10) times, gauged at its best-connected block), and every candidate keeps the residual of that consensus against it. A seam that cannot observe a scale is judged on rotation alone; the rotation, scale and translation residuals must stay within `maxGraphRotationResidual` (5°), `maxGraphScaleResidual` (1.05) and the graph's own translation bar, `maxSimTranslationError` (0.05) — read as a fraction of the smaller of the two blocks' own extents, floored at `kMinExtentShare` (0.2) of the larger's so a pair with almost no footprint of its own is still judged by something. A component of only two blocks holds no cycle at all — the averaging reproduces its seam's own claim exactly, at zero residual — so such a seam is judged by the rules below rather than by a residual that could never fail.

Every candidate ends up in one of four classes:

- **ROBUST** — consistent with the consensus, and corroborated by another, independent path between the same two blocks that is also consistent.
- **VERIFIED** — consistent but uncorroborated, and either both directions of it agreed, it came from the 3D-3D estimator, or it stands alone: a one-direction seam with at least `minSupportingCentres` distinct centres whose own inliers reach `kStrongAloneFactor` (4) times `minCommonTracks` — 100 by default.
- **UNDECIDED** — the averaging could not place both its blocks in one frame; or it is consistent but uncorroborated and not strong enough to stand alone; or it is inconsistent and no alternative path is strong enough to reject it; or it is one of two disagreeing opinions on an otherwise isolated pair of blocks, left for the placement to weigh directly.
- **REJECTED** — inconsistent with the consensus, and a consistent alternative path between the same two blocks carries at least its weight divided by `rejectWeightMargin` (1.5) — the indirect-path margin that lets a corroborated alternative overrule a wrong seam without letting a merely-present one reject on a technicality.

### Initial poses per component

The trusted seams alone — ROBUST or VERIFIED — carry the blocks into a first guess. Each connected component of trusted seams is averaged about its own best-connected block and becomes one model, numbered by how much trusted weight it holds; a block no trusted seam reaches keeps no model at all. These poses are only where the placement starts from — nothing here is admitted yet.

### Placement

Models grow one at a time, the one holding the most trusted weight first. A model with nothing admitted yet is seeded at the block with the largest ROBUST weight of its own (ties broken by total trusted weight, then by calibrated-image count); a model already holding blocks — grown again after a re-validation — keeps them and the seams it rests on.

The next block tried is always the one the admitted blocks pool the most support for: a candidate's own weight counts in full, an UNDECIDED one at `kUndecidedSupportWeight` (0.25) of it. A block is a group of one, and the same routine places a whole model as a group too: every correspondence between the group and the model's admitted blocks is pooled (a pair with no candidate, not already refused by the pairwise gates, still contributes its raw correspondences), up to three hypotheses are formed — **H1** the group's cameras posed as a rig against the model's pooled points, **H2** the model's cameras posed against the group's pooled points, **H3** a lone block's own initial pose, when it has one — and each is scored over the whole pool and held to four gates:

- **G1 — union support**: the same gate a single seam answers to, now over the pooled observations.
- **G2 — camera votes**: the same camera-vote gate, at `minCameraVoteRatio` normally or `minCameraVoteRatioVerified` once the model rests on verified seams only.
- **G3 — neighbours**: the admitted neighbours with something to say must be behind the placement — at least one supports it, and no more than a `kNeighbourContraShare` (3) share (one in three) of those with an opinion contradicts it. A trusted neighbour seam that the placement's own residual would fail counts as a loop vote, not a contradiction — that residual is exactly what a closing cycle looks like.
- **G4 — interleaving**: the same gate a single seam answers to.

Among the hypotheses that pass, the camera votes decide. A block no hypothesis carries is deferred and retried once the model changes — an admission may confirm what it could not before; if the model has gone a full round with no change, the deferred blocks are handed back for another model to try, carrying the reason last recorded against them.

Every admission that does not close a cycle re-fits every admitted block jointly to the observations behind the seams the model rests on: seven free parameters per block (a unit quaternion, a translation, a log scale) and one chord residual per inlier observation, under a Huber loss at `maxReprojError`. An admission that closes a cycle instead averages the whole model over its own seams first, spreading the cycle's discrepancy over it, and only then refines — see "Loop closure" below.

### Loop closure

**The block pose graph.** A block whose own cameras the model leaves unmixed and whose admitted neighbours hold nothing against it — refused only by what it explains or by the camera votes, not by interleaving or an outright neighbour contradiction — and joined to two or more admitted blocks by trusted seams is the one a cycle runs through: no single pose of it can answer to every end of the cycle at once. Such a block is taken in on trust at its best hypothesis, the model is averaged over the seams it already rests on plus this block's own trusted ones, which spreads the cycle's discrepancy over the whole loop and moves the block to where its own seams — not its rejected hypothesis — put it, and it is scored again there against the same four gates, the neighbour gate included. It is admitted only if it passes there; the model is restored exactly as it was if it does not.

**A weak closing seam.** A pair of admitted blocks that carries correspondences but has no seam of its own — too thin for its own cameras to vote on — can still be read once the model predicts where it should lie: the pair's correspondences are collected under that prediction, refined over whatever they explain at `kLooseSeamFactor` (3) times `maxReprojError`, and scored like any other candidate, answering to union support and interleaving but not to its own cameras' votes — what stands behind such a seam is the model's own consistency, not the pair's own ability to measure itself. It enters the model at its scored weight, floored at 1 so that a seam no camera could vote on still holds its two blocks together.

**Camera relaxation.** Once the merged model is otherwise finished, if its largest model-seam error still exceeds `relaxSeamResidualFactor` (2.0) times `maxReprojError`, every camera of the placed blocks is given a similarity of its own, over a pose graph whose edges are charged under the seam bars themselves (rotation, translation and scale in one robust term): each camera to its `kIntraBlockEdgesPerCamera` (3) strongest covisible cameras within its own block, at `intraBlockEdgeWeight` (10.0), and to the cameras it faces across a seam the model rests on, at the weight of one seam. A block bent inside its own reconstruction has no single rigid pose that can meet every seam at once; this relaxation is what lets it meet them anyway. The relaxed cameras are written back into their own blocks' frames — the block poses themselves are untouched, so what bends is the reconstruction inside each block, never the placement between blocks.

### What does not fit

**Second models.** Blocks no model admitted start over: the trusted seams among just them form their own models, a block no trusted seam reaches becomes a model of one, and each of those is placed exactly like the first. Every resulting model is then placed as a single group against the model already carrying the most images, largest first — the same pool, hypotheses, gates and admission a single block answers to. A model that goes in hands over the seams it rested on, one similarity having carried all of its blocks at once. A model with no pair linking it to the merged model at all is left standing apart.

**Folds.** A block whose own cameras split over its best placement — roughly as many voting for it as against — is not a block that failed to place but two blocks a cluster boundary never let see each other. Each camera's vote assigns it a side; a camera that could not vote joins the side its own covisibility connects it to. The cut is accepted only when both sides keep at least `minFoldPartViews` (10) calibrated views, each side is internally connected, and the weight cut between them is no more than `maxFoldCutRatio` (0.2) of the weaker side's own internal weight. The two parts are re-extracted as their own blocks, their seams measured like any other block's, and everything the original, single block had measured is discarded.

**Re-validation.** Once the model that ends up merged has taken in everything it can, every block it admitted is judged once more, at the pose it ended up with, against the model as it has since grown: one whose cameras now contradict the grown model is let go, along with the seams it carried; one whose cameras now split is a fold an earlier admitted neighbour had hidden. A block let go keeps the images it holds out of the merged model, for the resection to recover one by one; the two parts of anything cut are offered back to the model to place again. It reads the same pool a placement reads, so evidence the seam graph rejected takes no part in it either: a block admitted on a seam the graph trusts, whose only contradiction lies in a pair that graph rejected, is not let go here.

**Unplaced blocks, reported.** The model holding the most images is the one the scene is merged at. Every block outside it — another model's, or one no model could place — is merged without a pose, so the post-merge resection can recover its images one by one against the consensus, the same recovery any other unregistered image gets: a block a cluster boundary happened to isolate is not different in kind from an image the incremental reconstruction itself failed to register, and forcing it into the model as a whole block, on evidence the gates have already weighed and refused, would risk the model for exactly the images the resection can still try individually. The merge report and the log name every one of them and the reason it stayed out.

### Finish

**Seam inliers by construction.** The kept correspondences of every seam the merged model rests on are collected before the merge and, when the union-find track merge runs, unioned on the duplicate-image guard alone — no 3D-proximity re-test, because a model seam has already agreed with the placement.

**Final bundle adjustment.** `Scene::ReconstructHierarchical` hands the merged scene — placed blocks and all — back to the shared tail of reconstruction: track filtering, re-triangulation, another filter pass, then one global bundle adjustment over the whole scene.

**Resection scope.** After that bundle adjustment, images too weakly connected are dropped and every remaining unregistered image — the ones from blocks the merge left unplaced, alongside any image a sub-scene's own incremental reconstruction never managed to register — is resected against the now-adjusted consensus in the same pass. Nothing about having belonged to an unplaced block routes an image to a different recovery path than any other unregistered image.

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

Every averaging in the merge — the seam graph's consensus, the initial poses, the block pose graph a placement or a loop closure reads — goes through one routine, and that routine solves rotation, then scale, then translation, never all seven parameters of a Sim(3) at once. Each subproblem is convex (or nearly so) on its own: rotation averaging on SO(3) has well-studied convex relaxations, scale averaging in log-space is linear least-squares, and translation averaging given known rotations and scales is a linear system. A joint solve would trade that for a single non-convex 7-DOF optimization per edge.

### Why the vote counts cameras, not just inliers

A transform can rack up inlier correspondences from a single repeated texture patch seen by one camera, or from one image whose keypoints happen to fall on a shared object — evidence that says nothing about whether the *block* agrees, only that one small part of it does. Counting distinct supporting camera centres, and requiring each supporting camera's own inliers to spread across its image rather than cluster in one cell, is what turns "many correspondences agree" into "many independent witnesses agree." The interleaving check adds the complementary case: a transform can explain every correspondence and still be wrong, if it drops one block's cameras inside the other's footprint instead of leaving each block's cameras among their own.

### Why averaging is trusted only over gated seams

A seam's own evidence cannot tell a wrong transform from a right one — both can fit their own correspondences well. What the seam graph adds is a second, independent opinion: a cycle of seams that closes consistently is evidence no single seam carries by itself. But that only works because every edge entering the graph has already passed the pairwise gates in "Measuring a seam" above; the graph is never asked to referee between raw, ungated correspondences, only between measurements that already cleared union support, camera votes and interleaving on their own. A two-block component — no cycle at all — is a case the averaging is structurally unable to judge, and it is left alone rather than given a residual that could never disagree with the seam that produced it.

### Union-Find with 3D Guards

Track merging across blocks reuses the union-find pattern from `BuildTracks`, guarded by 3D proximity: because sub-scene tracks have disjoint image sets by construction, the duplicate-image guard alone cannot catch a false match between two blocks, so merged tracks must also land within a small fraction of the scene's bounding-box diagonal of each other. The one exception is the correspondences behind a seam the merged model actually rests on: those have already been scored, gated and agreed with the placement, so they join on the duplicate-image guard alone — re-running the proximity test on evidence the placement has already vouched for would only risk losing it to noise in the very positions the seam was measured to correct.

### Why a block the gates refuse is not resected as a block

A block that no model could place is not a block about which nothing is known — it is one whose evidence the union-support, camera-vote, neighbour and interleaving gates weighed and could not accept as a whole. Forcing it into the merged model anyway, as one rigid unit, would risk everything the model already holds on exactly the evidence that failed to clear the bar. What is not in question is the block's own images: many of them may still carry strong, correct pairs to the consensus that never happened to close a cycle, or were outvoted only because the block as a whole disagreed elsewhere. Merging the block's observations in without a pose and leaving its images to the ordinary post-merge resection lets each of them stand or fall on its own connections to the finished model, instead of the model standing or falling on all of them at once.
