# Depth-Map Fusion

## 1. Purpose and scope

Depth-map fusion is the step between depth-map estimation and mesh reconstruction: it turns N
per-view depth-maps (each with its normal-map and per-pixel confidence, after PatchMatch and the
geometric-consistency iterations) into one point cloud, where every point carries the list of views
that saw it, a per-view weight, a normal and a colour. It is where per-view estimates stop being
images and become geometry — a pixel no other view confirms is dropped here, and a surface seen by
many views becomes one point rather than N.

`--fusion-filter` (`OPTDENSE::nFuseFilter`) selects the implementation: `0` merge
(`DepthMapsData::MergeDepthMaps`) projects every valid depth into world space as its own point with
no clustering; `1` fuse (`DepthMapsData::FuseDepthMaps`) joins depths that agree and drops points
that block another view's line of sight; `2` dense-fuse (`DepthMapsData::DenseFuseDepthMaps`, the
default) is described below. All three live in `libs/MVS/SceneDensify.cpp`; the call site is
`Scene::DenseReconstruction`. The mesh stage that consumes fusion's output keeps its own design
record in `DelaunayMeshReconstruction.md`.

## 2. Algorithm as implemented

`DepthMapsData::DenseFuseDepthMaps` fuses depth-maps one at a time, each
chosen with `FetchBestNextDMapIndex` as the not-yet-fused map with the
most neighbours already resident in the depth-map cache (ties broken toward fewer total neighbours),
so the cache is reused rather than thrashed. The estimation's GPU pools are released before fusion
starts (their pinned host buffers are most of the estimation's resident memory), and the cache
(`DMapCache`) is re-budgeted after every fused map (`FusionCacheSetup::Budget`): the memory free at
that moment, minus the safety margin and what the next map may add to the point-cloud
(`FusionMemoryReserve`: twice the last map's points, plus the per-point arrays' next growth), capped
at four working sets (`FusionWorkingSet`: the reference and `nMaxViewsFuse` neighbours) and never
below one. The cache thus shrinks as the cloud grows instead of taking the memory the cloud and other
processes need; the colour images stay resident when they fit in the free memory. A depth-map file that fails
to load (corrupt or truncated) is logged and left out: as a reference it is skipped, as a neighbour it
is never walked into. For the chosen reference map:

1. **Seeding.** Every pixel is visited once, in raster order, via the `FusePoint` lambda.
   A pixel with no depth, already consumed by an earlier cluster (`useMask`), or below the confidence
   floor is skipped; otherwise it seeds a new cluster and becomes its reference point and normal. The
   floor follows the scale of the map's confidence: `ConfRefine::FUSE_MIN_CONF` (0.07) on a
   recalibrated map (`DepthData::bConfAdjusted`), the estimation's own `1 − fNCCThresholdKeep` on a
   raw photometric one.
2. **Growing.** `FusePoint` walks the view-neighbourhood graph depth-first: each member projects
   into its neighbour views, and the pixel it lands on joins the cluster if it agrees with the
   cluster's *reference* point (not the neighbour the walk arrived from, so a cluster cannot drift
   away from its seed one join at a time) on all gates: depth similarity
   (`fDepthDiffThreshold`), lateral reprojection error (`fDepthReprojectionErrorThreshold`, tested
   as `normSq(diff) > maxReprojErrorSq`), and normal agreement (`fNormalDiffThreshold`). Each joined
   pixel is marked consumed in `useMask`, so a pixel belongs to at most one cluster, and the walk
   continues from it into *its* neighbours, bounded by `nMaxFuseDepth` and `nMaxPointsFuse`. A
   neighbour whose own measured depth lies more than `VIOLATION_MARGIN·fDepthDiffThreshold` behind
   the cluster's reprojected point (`ConfRefine::VIOLATION_MARGIN`, shared with the confidence
   recalibration) fails the join and is recorded as a free-space violation for that distinct view
   (`fusedViolViews`, deduplicated) instead of merely being skipped; a neighbour that passes the depth
   and reprojection gates but fails the normal gate is likewise recorded as a normal contradiction
   (`fusedNormViews`, deduplicated): it sees the same place and disputes the surface there.
3. **Keeping.** The cluster becomes a point when it has at least `nMinPixelsFuse` pixels *and*
   `nMinViewsFuse` distinct views (both clamped by the map count). Both minimums accept fractional
   "virtual" support, `fFusePriorWeight` times the seed's intra-map prior
   (`DepthMapsData::GetIntraMapPrior`/`ComputeIntraMapPrior`, the same prior the confidence
   recalibration uses — see `DepthMapConfidence.md`), which keeps an inlier lying on a coherent
   surface that too few views happened to confirm. A point kept *only* thanks to that support is
   "rescued". The **contradiction guard** (`nFuseViolationMax`, `< 0` disables it) then separates a
   lack of evidence from evidence against the point: a rescued point may be contradicted by at most
   `nFuseViolationMax` distinct views, free-space violations and normal contradictions together, and
   a point kept on real support alone is dropped when its normal contradictions outnumber its
   supporting views.
4. **Emitting.** The point's position is the component-wise median of its members' 3D locations —
   robust to a single bad join, and the reason weights never enter the position. Its views are the
   distinct views of its members, each weighted by that view's confidence, combined with `max` (not
   sum) across pixels from the same view since those are correlated observations of one surface; its
   normal is the normalised sum of member normals and its colour their mean.

Clusters that fail the keep-rule are discarded and their pixels stay consumed, unless
`bFuseRecycleDropped` is set: a dropped cluster then hands its pixels back to the pool
(`clusterMembers`, one `unset` per member) so a later seed or probe can still use them — a recycled
pixel can therefore be consumed more than once across the run.

## 3. Parameters and defaults

| CLI option | struct field | default | meaning |
|---|---|---|---|
| `--fusion-filter` | `OPTDENSE::nFuseFilter` | `2` | `0` merge, `1` fuse, `2` dense-fuse |
| `--fusion-mode` | `OPTDENSE::nFusionMode` (app-local) | `0` | `-2` SGM depth-maps & fusion, `-1` export SGM pair disparity-maps only, `0` depth-maps & fusion, `1` export depth-maps only |
| `--number-views-fuse` | `OPTDENSE::nMinViewsFuse` | `2` | minimum distinct views for a cluster to be kept (`<2` degrades keeping to merge-like behaviour) |
| *(config file only)* | `OPTDENSE::nMaxViewsFuse` | `32` | maximum neighbour depth-maps cached and walked per reference |
| *(config file only)* | `OPTDENSE::nMinPixelsFuse` | `5` | minimum joined pixels for a cluster to be kept |
| *(config file only)* | `OPTDENSE::nMaxPointsFuse` | `1000` | maximum pixels fused into a single point |
| *(config file only)* | `OPTDENSE::nMaxFuseDepth` | `100` | maximum recursion depth of the graph walk |
| `--fusion-depth-diff-threshold,t` | `OPTDENSE::fDepthDiffThreshold` | `0.01` | max relative depth difference to join |
| `--fusion-reprojection-threshold,d` | `OPTDENSE::fDepthReprojectionErrorThreshold` | `0.6` | max lateral reprojection error, in pixels, to join |
| *(config file only)* | `OPTDENSE::fNormalDiffThreshold` | `25` (degrees) | max normal disagreement to join |
| *(config file only)* | `OPTDENSE::fNCCThresholdKeep` | `0.9` | `1 − this` is the minimum confidence to seed/join a pixel of a raw (not recalibrated) map |
| — (compile-time) | `ConfRefine::FUSE_MIN_CONF` | `0.07` | minimum confidence to seed/join a pixel of a recalibrated map |
| `--fusion-prior-weight` | `OPTDENSE::fFusePriorWeight` | `4.0` | intra-map-prior virtual support weight (`0` disables the rescue) |
| *(config file only)* | `OPTDENSE::nFuseViolationMax` | `0` | contradiction guard: max distinct contradicting views on a rescued point; also enables dropping supported points outvoted by normal contradictions (`<0` disables the guard) |
| `--fusion-recycle-dropped` | `OPTDENSE::bFuseRecycleDropped` | `false` | hand a dropped cluster's pixels back to the pool instead of losing them for good |

The generous prior weight is safe only together with the contradiction guard: the rescue then
admits exactly the weakly supported points that nothing disputes, which are as accurate as fully
supported ones, while the guard removes the disputed ones. `--fusion-recycle-dropped` trades
precision for completeness and is off by default.

## 4. Invariants and constraints

- **Every join gate is judged against the cluster's seed, never against the pixel the walk arrived
  from** — this is what keeps a cluster from drifting away from its seed one join at a time.
- **A pixel belongs to at most one cluster at a time** (`useMask`); with `bFuseRecycleDropped` off, a
  consumed pixel is locked away for good even if its cluster is later dropped.
- **Only evidence against a point can remove it; missing evidence never does** — the contradiction
  guard counts views that saw the place and disagreed (free-space violations, normal contradictions),
  never views that were out of frame, empty, low-confidence or already consumed. A cluster meeting
  `nMinPixelsFuse`/`nMinViewsFuse` on real support is never rejected for free-space violations, only
  when normal contradictions outnumber its supporting views.
- **The virtual-support rescue can never fabricate a point from nothing**: it only ever tops up a
  cluster that has at least one real seed pixel (`!fusedViews.empty()` guard); with
  `fFusePriorWeight == 0` the prior is not even computed for fusion (`bUsePrior` false) and output is
  byte-identical to the rescue being absent.
- **Point position is always the plain median of member 3D locations**; weights are stored per view
  for downstream consumers (mesh visibility weighting, the Interface's `Vertex::View::confidence`)
  and never used to average the position itself.
- **The output depends on the cache size**: the reference order follows the cache contents
  (`FetchBestNextDMapIndex`), so a different cache consumes pixels in a different order and fuses a
  slightly different cloud (within the 0.003 F1 repeat spread). With free memory above the cap the
  cache size is fixed by the scene; below it, never compare fusion outputs produced beside another
  memory-heavy job.
  Every neighbour of the current reference stays resident while it is fused: eviction is
  least-recently-used and stops at the reference, which is touched before its neighbours.
- **The join thresholds are fusion's alone.** The confidence recalibration measures whether a depth
  is correct, with tolerances calibrated to the estimation noise (`ConfRefine::CONFIRM_DEPTH`,
  `DepthMapConfidence.md`), which is also the unit of the intra-map prior's plane-fit band; fusion's
  floor on the recalibrated confidence is its own `FUSE_MIN_CONF`. Changing the join thresholds
  therefore refines the clustering without moving the confidence scale that floor is applied to.

## 5. Validation of the shipped defaults

### Join thresholds

Full `DensifyPointCloud --number-views 24` at resolution levels 1 and 0 with the current code, one
depth-map set per scene and level, re-fused at each threshold pair (`--geometric-iters 0`; the
confidence recalibration does not read these thresholds, so one estimation serves the whole grid),
official Tanks-and-Temples evaluator re-aligning every cloud:

| scene | R1: 1.0 px / 1% | R1: 0.6 px / 1% | R0: 1.0 px / 1% | R0: 0.6 px / 1% |
|---|---|---|---|---|
| Meetingroom | 0.4515 | 0.4763 | 0.5399 | 0.5515 |
| Caterpillar | 0.6159 | 0.6366 | 0.6725 | 0.6736 |
| Truck | 0.7283 | 0.7444 | 0.7587 | 0.7624 |
| Church | 0.6166 | 0.6290 | 0.6615 | 0.6678 |
| Ignatius | 0.8047 | 0.8197 | 0.8941 | 0.8970 |
| Barn | 0.6526 | 0.6829 | 0.7264 | 0.7366 |
| mean | 0.6449 | **0.6648** | 0.7089 | **0.7148** |

`fDepthReprojectionErrorThreshold = 0.6` wins on every scene at both levels. Fusion's reprojection
gate compares the seed's projection with the rounded pixel the walk landed on, so it sees pixel
rounding plus the drift of multi-hop joins; 0.6 px keeps the joins that land near the seed's own
projection. A relative depth difference of 0.5% instead of 1% is neutral at R1 (mean 0.6632 with
0.6 px, 0.6450 with 1.0 px) and slightly worse at R0 (0.7118, 0.7063), so `fDepthDiffThreshold` stays
0.01: correct depths disagree by 0.3–1% at the 90th percentile, and a tighter join splits their
clusters without improving placement.

### Contradiction guard and `fFusePriorWeight = 4`

Fusion-only on frozen `--resolution-level 0 --number-views 24` depth-maps, official Tanks-and-Temples
evaluator re-aligning every cloud; baseline = the previous defaults (prior weight 3, guard on
free-space violations of rescued points only). Ignatius and Barn were held out of the analysis that
chose the rule:

| scene | cloud F1 before | cloud F1 with the guard | ΔF1 |
|---|---|---|---|
| Meetingroom | 0.5306 | 0.5419 | +0.011 |
| Truck | 0.7434 | 0.7591 | +0.016 |
| Church | 0.6583 | 0.6617 | +0.003 |
| Caterpillar | 0.6344 | 0.6679 | +0.034 |
| Ignatius | 0.8867 | 0.8943 | +0.008 |
| Barn | 0.7055 | 0.7250 | +0.020 |

Mean ΔF1 +0.015, every scene up. The rule was found by dumping every fusion cluster with the
outcome of each probed view and replaying keep-rules against the ground truth: weakly supported
clusters that no view contradicts are as accurate as fully supported ones on every scene, those
with normal contradictions are not.

End-to-end check of the shipped defaults (`--resolution-level 0 --number-views 24`, full densify
then `ReconstructMesh` at defaults, every product realigned), against the same flags at the previous
defaults:

| scene | cloud F1 | mesh F1 | previous cloud / mesh | densify | fusion | peak private |
|---|---|---|---|---|---|---|
| Truck | 0.7627 | 0.6870 | 0.7420 / 0.6816 | 12m14s | 4m40s | 15.1 GB |
| Meetingroom | 0.5513 | 0.5070 | 0.5318 / 0.5043 | 16m52s | 5m51s | 15.4 GB |

Both clouds reproduce the re-fused grid values above within 0.0003. Against a cache budgeted once
from all free memory with the GPU pools kept alive, the per-map budget cuts fusion from 6m10s / 7m06s
(Truck / Meetingroom), starts it at 4–5 GB private instead of ~9 GB and ends it at 15 GB instead
of 18–20 GB, for cloud F1 within 0.0003 and mesh F1 within 0.003 (the cache size changes the
fusion order, §4).

## 6. Rejected alternatives

- **A global `fFusePriorWeight` without the contradiction guard** — 0 wins on accuracy-bound scenes,
  4 on completeness-bound ones; any single value loses up to 0.035 F1 somewhere.
- **Adapting the prior weight locally** (image texture, ground sample distance, 3D density of
  supported points, supported share of the tile/image) — none transfers across scenes
  (leave-one-scene-out ≤ +0.0045 mean F1 against +0.0088 for the normal-contradiction count).
- **A stricter normal-contradiction ratio (0.75)** — same mean, moves gain between scenes and puts
  Church below the unguarded baseline.
- **Join depth tolerance 0.5%** — neutral at R1, −0.003 at R0 (§5): it splits clusters of correct
  depths without improving placement.
- **`nMinPixelsFuse` 3–4, `fNCCThresholdKeep` 0.85/0.95, `nFuseViolationMax` 1–2, the free-space
  guard on supported points** — dominated by the prior weight or inert.
- **Recycled pixels denied the rescue** — removes `--fusion-recycle-dropped`'s precision loss and its
  recall gain alike.
- **Seeding in descending-confidence order** — lower F1 on every scene tried.
- **Corroboration by already-fused agreeing pixels** — passes the cloud gate but inflates the point
  count past the mesh memory/wall bounds.
- **Re-probing the 4-neighbours of a failed join** — too little depth recoverable to pay for it.
- **A fusion cache taking all free memory** — no faster than four working sets (5m27s vs 5m26s,
  Meetingroom) while leaving the machine 2.7 GB instead of 4.1 GB; two working sets route the colour
  images through the cache and slow fusion by 8%.

## 7. Open items

- The fused cloud depends on the cache size through the reference order (§4); an order independent
  of the cache contents would make runs reproducible under any memory pressure.
- The point-cloud itself is the largest fusion allocation (about 250 B per point, most of it the two
  per-point view and weight lists); a flat views/weights layout would cut it roughly in half.
- `nMaxViewsFuse` (32) exceeds the estimation neighbourhood (`nMaxViews`, 12): the flood-fill can
  reach views that never contributed to estimation, and whether that interacts with the prior rescue
  is unexamined.
- Mesh wall time grows superlinearly with the fused point count on at least one scene; profile it
  before pushing fusion completeness further.
