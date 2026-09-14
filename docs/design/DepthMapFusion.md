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

`DepthMapsData::DenseFuseDepthMaps` (`SceneDensify.cpp:2440`) fuses depth-maps one at a time, each
chosen with `FetchBestNextDMapIndex` (`SceneDensify.cpp:2147`) as the not-yet-fused map with the
most neighbours already resident in the depth-map cache (ties broken toward fewer total neighbours),
so the cache is reused rather than thrashed; the cache (`DMapCache`) is sized from free RAM via
`GetAvailableMemory` and resized every `numDMapsReserveFusion` (10) maps. For the chosen reference
map:

1. **Seeding.** Every pixel is visited once, in raster order, via the `FusePoint` lambda
   (`SceneDensify.cpp:2501`). A pixel with no depth, with confidence below `1 − fNCCThresholdKeep`,
   or already consumed by an earlier cluster (`useMask`) is skipped; otherwise it seeds a new
   cluster and becomes its reference point and normal.
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
   (`fusedViolViews`, deduplicated) instead of merely being skipped.
3. **Keeping.** The cluster becomes a point when it has at least `nMinPixelsFuse` pixels *and*
   `nMinViewsFuse` distinct views (both clamped by the map count). Both minimums accept fractional
   "virtual" support, `fFusePriorWeight` times the seed's intra-map prior
   (`DepthMapsData::GetIntraMapPrior`/`ComputeIntraMapPrior`, the same prior the confidence
   recalibration uses — see `DepthMapConfidence.md`), which keeps an inlier lying on a coherent
   surface that too few views happened to confirm. A point kept *only* thanks to that support is
   "rescued", and must additionally have at most `nFuseViolationMax` distinct free-space-violating
   views (`nFuseViolationMax < 0` disables the guard; non-rescued points are never subject to it).
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
| `--fusion-depth-diff-threshold,t` | `OPTDENSE::fDepthDiffThreshold` | `0.01` | max relative depth difference to join (also the confidence recalibration's `thDepth`) |
| `--fusion-reprojection-threshold,d` | `OPTDENSE::fDepthReprojectionErrorThreshold` | `1.0` | max lateral reprojection error, in pixels, to join (also the confidence recalibration's `thReproj`) |
| *(config file only)* | `OPTDENSE::fNormalDiffThreshold` | `25` (degrees) | max normal disagreement to join |
| *(config file only)* | `OPTDENSE::fNCCThresholdKeep` | `0.9` | `1 − this` is the minimum confidence to seed/join a pixel |
| `--fusion-prior-weight` | `OPTDENSE::fFusePriorWeight` | `3.0` | intra-map-prior virtual support weight (`0` disables the rescue) |
| *(config file only)* | `OPTDENSE::nFuseViolationMax` | `0` | max distinct free-space-violating views tolerated on a rescued point (`<0` disables the guard) |
| `--fusion-recycle-dropped` | `OPTDENSE::bFuseRecycleDropped` | `false` | hand a dropped cluster's pixels back to the pool instead of losing them for good |

`--fusion-prior-weight 3` favours completeness and suits the default pipeline, where mesh
reconstruction follows and cleans the few extra outliers; `--fusion-prior-weight 2` trades some
completeness for fewer outliers when the dense point cloud itself is the final output.
`--fusion-recycle-dropped` trades precision for completeness the same way and is off by default for
the same reason.

## 4. Invariants and constraints

- **Every join gate is judged against the cluster's seed, never against the pixel the walk arrived
  from** — this is what keeps a cluster from drifting away from its seed one join at a time.
- **A pixel belongs to at most one cluster at a time** (`useMask`); with `bFuseRecycleDropped` off, a
  consumed pixel is locked away for good even if its cluster is later dropped.
- **The rescue guard (`nFuseViolationMax`) only ever applies to rescued points** — a cluster that
  already meets `nMinPixelsFuse`/`nMinViewsFuse` on real support alone is never rejected for
  free-space violations.
- **The virtual-support rescue can never fabricate a point from nothing**: it only ever tops up a
  cluster that has at least one real seed pixel (`!fusedViews.empty()` guard); with
  `fFusePriorWeight == 0` the prior is not even computed for fusion (`bUsePrior` false) and output is
  byte-identical to the rescue being absent.
- **Point position is always the plain median of member 3D locations**; weights are stored per view
  for downstream consumers (mesh visibility weighting, the Interface's `Vertex::View::confidence`)
  and never used to average the position itself.
- **Fusion degrades silently under memory pressure**: `DenseFuseDepthMaps` budgets its neighbour
  cache from free RAM and skips caching any neighbour that does not fit, logging `warning: not
  enough memory to cache depth-maps` — a starved run fuses a measurably different (smaller) cloud, so
  fusion must never run concurrently with another memory-heavy job when its output is being compared.
- `fDepthReprojectionErrorThreshold` and `fDepthDiffThreshold` are shared verbatim with the confidence
  recalibration's gate thresholds (`ConfRefine::Params::thReproj`/`thDepth`) by design, so the
  recalibrated confidence predicts what fusion will accept.

## 5. Validation of the shipped defaults

`fDepthReprojectionErrorThreshold = 1.0` (current default) vs. the previous default `1.2`, fusion-only
on frozen depth-maps (`--geometric-iters 0`, so only the constant under test changes), Tanks-and-Temples
training scenes, everything else at today's defaults:

| scene | P / R / F1 at 1.2 | P / R / F1 at 1.0 (shipped) | ΔF1 | points |
|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5770 / 0.7394 / 0.6482 | +0.0066 | +9.3% |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.7087 / 0.8566 / 0.7757 | +0.0043 | +8.5% |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.5130 / 0.3909 / 0.4437 | +0.0067 | +5.9% |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.6790 / 0.7713 / 0.7222 | +0.0046 | +10.9% |

Mean ΔF1 +0.0055, precision and recall both up on every scene; the raw-mesh F1 after
`ReconstructMesh` moves within ±0.0013, inside the ~0.0006 mesh noise floor — the tighter tolerance
does not starve clusters, it splits over-merged ones into distinct, better-placed points, and the
mesh step absorbs the extra points without cost. Benchmark protocol: `DensifyPointCloud
--resolution-level 1 --number-views 12 --estimate-roi 0 --crop-to-roi 0 --tower-mode 0` on Barn /
Ignatius / Meetingroom / Truck, scored against the laser ground truth with a frozen per-scene
alignment transform.

## 6. Rejected alternatives

- **`fFusePriorWeight` at 0 / 2 / 4 / 6** (default 3) — 0/2 give up completeness the rescue is meant
  to buy; 4 passes the cloud F1 gate but the mesh step rejects it (raw-mesh F1 down, mesh memory
  +12…40%).
- **`nMinPixelsFuse` 3 or 4** (default 5) — dominated by tuning `fFusePriorWeight` instead at every
  dose tried.
- **`fNCCThresholdKeep` 0.85 or 0.95** (default 0.9) — inert on cloud F1.
- **`nFuseViolationMax` −1 / 1 / 2** (default 0) — inert; the guard touches only 0.1–0.25% of valid
  depths at any of these settings.
- **The same free-space guard applied to non-rescued clusters** — inert.
- **Denying the prior rescue to any cluster containing a `--fusion-recycle-dropped` pixel** — removes
  that option's precision loss but cuts its recall gain by the same factor; recycled pixels pay their
  way only through the rescue, not independently of it.
- **Seeding clusters in descending-confidence order instead of raster order** — regressed cloud F1 on
  every scene tried, and worsened the recycle-dropped option when combined with it.
- **Corroboration** (probes landing on an already-fused pixel that agrees with the cluster count
  toward the keep-rule) — a moderate weight passes the cloud F1 gate but adds enough points that the
  mesh cost bounds (memory/wall) are exceeded.
- **Re-probing the 4-neighbours of a failed join** — instrumentation showed too little of the valid
  depth was recoverable this way to justify building it.
- **Reducing `ReconstructMesh --min-point-distance`** — a mesh-memory/wall trade, not a fusion change;
  left at its own default.

## 7. Open items

- Fusion degrades silently under memory pressure (§4): a neighbour that does not fit the cache is
  skipped with a warning rather than the run blocking or failing. Making the output independent of
  free RAM at run time (block until loadable, or fail loudly) is unimplemented.
- `nMaxViewsFuse` (32) is well above the typical estimation neighbourhood (`nMaxViews`, default 12):
  the flood-fill can reach views that never contributed to estimation, and whether that interacts
  with the prior rescue is unexamined.
- Mesh wall time has been observed to grow superlinearly with fused point count on at least one
  scene; a profile is needed before pushing fusion completeness further without a matching mesh-side
  check.
