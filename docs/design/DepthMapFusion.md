# Depth-Map Fusion

What the dense fusion stage does and how it is implemented (§ 1), followed by the record of the
August 2026 campaign that tuned it: one default changed and one opt-in option added (§ 2),
everything else that was tried listed in § 4 with its verdict, so that it is not re-proposed. The
mesh stage that consumes fusion's output keeps its own record in `DelaunayMeshReconstruction.md`.

## 1. The stage

Depth-map fusion is the step between depth-map estimation and mesh reconstruction: it turns N
per-view depth-maps (each with its normal-map and its per-pixel confidence, after PatchMatch and the
geometric-consistency iterations) into one point cloud, where every point carries the list of views
that saw it, a per-view weight, a normal and a colour. It is the point at which per-view estimates
stop being images and become geometry — a pixel that no other view confirms is dropped here, and a
surface seen by many views becomes one point rather than N.

`--fusion-filter` selects between three implementations; this document is about the default,
`2` (`FUSE_DENSEFILTER`, `MVS::DepthMapsData::DenseFuseDepthMaps` in `libs/MVS/SceneDensify.cpp`).
The other two do not cluster observations the way this one does: `0` merge (`MergeDepthMaps`)
projects every valid depth into world space as its own point, and `1` fuse (`FuseDepthMaps`) joins
depths that agree and drops points that block another view's line of sight.

**The algorithm.** Depth-maps are fused one at a time, each chosen as the one with the most
neighbours already resident in the depth-map cache (`FetchBestNextDMapIndex`), so the cache is
reused rather than thrashed; the cache itself is sized from free RAM and grows and shrinks as maps
are consumed. For the chosen reference map:

1. **Seeding.** Every pixel is visited once, in raster order. A pixel with no depth, with a
   confidence below `1 - fNCCThresholdKeep`, or already consumed by an earlier cluster is skipped;
   otherwise it seeds a new cluster and becomes its reference point and normal.
2. **Growing.** `FusePoint` walks the view-neighbourhood graph depth-first: each member projects
   into its neighbour views, and the pixel it lands on joins the cluster if it agrees with the
   cluster's *reference* point on all four gates — depth (`fDepthDiffThreshold`), lateral
   reprojection error (`fDepthReprojectionErrorThreshold`), normal (`fNormalDiffThreshold`) and
   confidence. Every gate is therefore judged against the seed, not against the neighbour the walk
   arrived from, so a cluster cannot drift away from its seed one join at a time. Each joined
   pixel is marked consumed (`useMask`), so a pixel belongs to at most one cluster, and the walk
   continues from it into *its* neighbours, bounded by `nMaxFuseDepth` and `nMaxPointsFuse`. A
   neighbour that rejects the join because its own measured depth lies well *behind* the cluster's
   point is recorded as a free-space violation — that view sees through where the point claims to be.
3. **Keeping.** The cluster becomes a point when it has at least `nMinPixelsFuse` pixels *and*
   `nMinViewsFuse` distinct views. Both minimums accept fractional "virtual" support,
   `fFusePriorWeight` times the seed's intra-map prior (its local depth/normal coherence, the same
   prior the confidence recalibration uses), which keeps an inlier lying on a coherent surface that
   too few views happened to confirm. A point kept *only* thanks to that support is *rescued*, and
   must additionally survive the free-space violations counted in step 2 (`nFuseViolationMax`).
4. **Emitting.** The point's position is the component-wise median of its members — robust to a
   single bad join, and the reason weights never enter the position. Its views are the distinct
   views of its members, each weighted by that view's confidence (max over pixels of the same view,
   since those are correlated observations of one surface); its normal is the normalised sum of
   member normals and its colour their mean.

Clusters that fail the keep-rule are discarded, and their pixels stay consumed — unless
`--fusion-recycle-dropped` (§ 2) hands them back.

Everything below is the record of the August 2026 campaign that tuned this stage: what changed
(§ 2), how it was measured (§ 3), and what was tried and rejected (§ 4).

## 2. What changed

**`fDepthReprojectionErrorThreshold` 1.2 → 1.0** — the lateral tolerance, in pixels, a probed pixel
must satisfy to join a cluster (`normSq(diff) > maxReprojErrorSq` in `FusePoint`). Set in three
places: the `OPTDENSE` default (`libs/MVS/DepthMap.cpp`), the `--fusion-reprojection-threshold` CLI
default (`apps/DensifyPointCloud/DensifyPointCloud.cpp`) and the Viewer's densify options
(`apps/Viewer/Scene.h`). The same value is the reprojection soft-gate width of the confidence
recalibration (`AdjustConfidence`, `ConfRefine::Params::thReproj`); it is shared with fusion by
design, so that the recalibrated confidence predicts what fusion will accept.

Fusion-only measurement on the frozen dmaps (§ 3), everything else at today's defaults:

| scene | base P / R / F1 | 1.0 P / R / F1 | ΔF1 | ΔP | ΔR | points |
|---|---|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5770 / 0.7394 / **0.6482** | **+0.0066** | +0.0024 | +0.0131 | +9.3 % |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.7087 / 0.8566 / **0.7757** | **+0.0043** | +0.0006 | +0.0094 | +8.5 % |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.5130 / 0.3909 / **0.4437** | **+0.0067** | +0.0058 | +0.0071 | +5.9 % |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.6790 / 0.7713 / **0.7222** | **+0.0046** | +0.0029 | +0.0069 | +10.9 % |

Mean **+0.0055**, every scene positive, precision *and* recall up everywhere. The `min-pixels` drop
share barely moves (+0.5…0.7 pp) while the reprojection rejections grow: the tighter tolerance does
not starve clusters, it splits over-merged ones into distinct, better-placed points. The same change
had measured +0.0028…+0.0052 in June, before the confidence recalibration and the prior rescue
existed (commit `dc32ab8`, never merged) — this is its second validation, on the other confidence
lineage.

Cost: fusion wall 0…−3 % (not slower), fusion peak memory +1…3 %, +6…11 % points. Downstream the
mesh absorbs the gain: raw-mesh F1 −0.0004 / 0.0000 / +0.0001 / +0.0013 (within the 0.0006 mesh
noise floor), Delaunay vertices ×1.05…1.07, mesh peak memory ×1.02…1.05.

End-to-end check (a full densification at the new default, so the recalibration runs at 1.0 as
well; single run, estimation noise ≲ 0.001 F1 on these scenes): Ignatius **0.7760** (P 0.7094 /
R 0.8563, 10.18 M points) and Truck **0.7226** (P 0.6800 / R 0.7710, 10.38 M points) — +0.0046 /
+0.0050 over the baseline and within 0.0004 of the fusion-only rows above, both with an unstarved
dmap cache. The recalibration at 1.0 neither adds to nor subtracts from the fusion gain.

**Why 1.0 and not lower.** The dose curve on the same dmaps is monotone down to 0.6 and bends only
at 0.5, where Meetingroom turns down:

| threshold | Barn | Ignatius | Meetingroom | Truck | mean ΔF1 | points |
|---|---|---|---|---|---|---|
| 1.1 | +0.0027 | +0.0015 | +0.0027 | +0.0019 | +0.0022 | +2…4 % |
| **1.0** | +0.0066 | +0.0043 | +0.0067 | +0.0046 | **+0.0055** | +6…11 % |
| 0.9 | +0.0149 | +0.0103 | +0.0141 | +0.0103 | +0.0124 | +12…28 % |
| 0.8 | +0.0175 | +0.0116 | +0.0174 | +0.0117 | +0.0146 | +14…34 % |
| 0.7 | +0.0205 | +0.0135 | +0.0196 | +0.0133 | +0.0167 | +17…41 % |
| 0.6 | +0.0243 | +0.0153 | +0.0212 | +0.0157 | +0.0191 | +17…50 % |
| 0.5 | +0.0272 | +0.0163 | +0.0186 | +0.0184 | +0.0201 | +10…54 % |

None of it survives the mesh. 0.9: raw-mesh ΔF1 −0.0012 / −0.0008 / +0.0008 / +0.0036 (mean
+0.0006) for +9…15 % mesh memory and +10…21 % mesh wall — at the cost bounds of § 3. 0.6: +0.0018
mean for +11…24 % memory and +11…36 % wall — over them. 1.0 is the clean pass, the whole cloud gain
at no mesh cost; 0.9 is the deepest dose still inside the bounds, worth considering only where the
point cloud is the product.

**New option `--fusion-recycle-dropped` (default off).** `useMask` is permanent: a pixel a cluster
consumed is never offered again, so a cluster the keep-rule then discards locks its pixels away from
every later cluster that needed them. With the switch on, a dropped cluster hands its members back
and a later seed or probe can still use them (`OPTDENSE::bFuseRecycleDropped`, one member list per
cluster and one `unset` per member on a drop; nothing is recorded when it is off). It buys
completeness with precision, measured on the same frozen dmaps at the new default:

| scene | ΔF1 | ΔP (pp) | ΔR (pp) | points |
|---|---|---|---|---|
| Barn | +0.0017 | −1.64 | +3.32 | +22.2 % |
| Ignatius | +0.0010 | −1.32 | +2.26 | +24.7 % |
| Meetingroom | +0.0160 | −3.34 | +4.98 | +41.9 % |
| Truck | −0.0040 | −1.86 | +1.56 | +16.6 % |

Mean +0.0037, but Truck fails the no-scene-below−0.003 clause of § 3, which is why it is an opt-in
and not a default. 12.7–27.4 M pixels come back per scene and fusion costs +30…80 % wall. The same
shape was measured at the old 1.2 threshold (mean +0.0043, Truck −0.0039), so the trade does not
move with the threshold. It addresses the case `--fusion-prior-weight` already speaks to: the dense
point-cloud is the final output. When a mesh reconstruction follows, leave it off — the graph-cut
interpolates what the extra points add, and every arm of this campaign that bought cloud F1 with
point count was absorbed by the mesh (§ 4).

## 3. How it was measured

- **Benchmark.** Tanks-and-Temples training scenes Barn / Ignatius / Meetingroom / Truck (410 /
  263 / 371 / 251 images), `DensifyPointCloud --resolution-level 1 --number-views 12
  --estimate-roi 0 --crop-to-roi 0 --tower-mode 0`; 10 M samples, seed 42; alignment to the laser
  ground truth frozen per scene (`<Scene>_final_transform.npy`). Baseline cloud F1 Barn 0.6416 /
  Ignatius 0.7714 / Meetingroom 0.4370 / Truck 0.7176; baseline raw-mesh F1 0.6225 / 0.7607 /
  0.4376 / 0.6544 at 16.0 / 8.7 / 11.3 / 9.8 GB mesh peak and 242 / 156 / 166 / 149 s mesh wall.
  (The bench's older reference clouds were June artifacts predating the confidence recalibration
  and the prior rescue of #1292, not a configuration; they were re-frozen, nothing was adopted.)
- **Frozen dmaps.** PatchMatch is unseeded: identical build and flags reproduce point counts with
  0.3–14 % run-to-run spread, so a fusion A/B across two densifications is invalid. Every fusion
  arm re-fuses the same `.dmap` set with one constant changed, via `--geometric-iters 0` (which
  loads the cached, geometric-consistent maps and goes straight to fusion); the fuse itself is
  serial and deterministic, so one run per arm suffices.
- **Memory.** `DenseFuseDepthMaps` budgets its neighbor-dmap cache from free RAM and silently
  skips every neighbor it could not cache (`warning: not enough memory to cache depth-maps`), so a
  starved run fuses a different cloud (Barn: +0.9 % points, +0.0009 F1). A fusion row counts only
  with no such warning in its log — never run fusion beside a mesh reconstruction or an evaluation.
- **Mesh visibility.** Mesh F1 is scored on samples cleaned by mesh visibility: the mesh is
  rendered into every scene camera and samples no camera sees are dropped, so hole-filled surface
  absent from the laser ground truth does not count against the mesh. On T&T this removes
  ≤ 0.03 % of the samples (raw and cleaned mesh F1 agree within 0.0001), and the mesh pipeline
  reproduces run-to-run exactly.
- **Gate.** Cloud: mean ΔF1 ≥ +0.003, no scene below −0.003. Downstream: raw-mesh ΔF1 not below
  −0.003 on any scene, mesh wall ≤ +25 %, mesh peak memory ≤ +15 %; a candidate adding > 20 %
  points is paired with a `--min-point-distance 2.0` mesh row. Noise floor: 0.0006 on the mesh,
  none on the cloud.

## 4. Tried and rejected

All on the frozen dmaps, cloud ΔF1 vs the baseline unless stated; every arm below was built and
measured, then removed again — none of it is in the tree.

| candidate | cloud ΔF1 (mean; worst scene) | verdict |
|---|---|---|
| threshold 0.9 … 0.5 | +0.012 … +0.020 | § 2: mesh-neutral, mesh cost over the bounds below 0.9 |
| `fFusePriorWeight` 0 / 2 / 4 / 6 (default 3) | −0.030 / −0.014 / +0.011 (Truck −0.001) / −0.006 | 4 passes the cloud gate and the mesh rejects it: raw-mesh −0.0018 mean, Truck −0.0089, mesh memory +12…40 %. The 0 and 2 rows show the rescue is the recall engine of today's fusion, worth +0.030 on raw and recalibrated confidence alike; its precision cost is the P/R trade of denser sampling, not outliers |
| `nMinPixelsFuse` 4 / 3 (default 5) | +0.008 (Truck −0.005) / −0.008 | dominated by the prior weight at every dose |
| `fNCCThresholdKeep` 0.95 / 0.85 (default 0.9) | −0.0005 / +0.0001 | inert |
| `nFuseViolationMax` −1 / 1 / 2 (default 0) | within ±0.001 | inert — the guard touches 0.12–0.25 % of the valid depths |
| the same free-space guard on non-rescued clusters, 0 / 1 / 2 allowed violations | within ±0.002 | inert |
| confidence recalibration off (`--postprocess-dmaps 0`) | −0.0017 | the recalibration is worth 0.001–0.003, all precision — stays on |
| `--postprocess-dmaps 8`, the standalone CPU recalibration, vs the integrated GPU pass | −0.0008 … +0.0002 | parity for CPU users (4–5 s, dmap-sized memory peak) — nothing to change |
| deny the prior rescue to any cluster that consumed a pixel recycled by `--fusion-recycle-dropped` (§ 2) | +0.0018 (Truck −0.0007) | the prior rescue is indeed where the recycle option’s precision loss sits — denying it cuts ΔP from −1.3…−3.3 to −0.3…−1.3 pp and removes the Truck regression — but it cuts the recall gain by the same factor (+0.4…+1.7 pp, +3…12 % points), so the mean drops to +0.0018. Recycled pixels pay their way only through the rescue: safe or strong, not both |
| seed clusters in descending confidence order instead of raster order | −0.003, negative on every scene | also worsens the recycle arm when combined with it |
| corroboration: probes landing on already-fused pixels that agree with the cluster count toward the keep-rule, weight 0.01 / 0.1 / 0.25 / 0.5 / 1 | 0 / +0.012 / **+0.018** / +0.015 / +0.007 | 0.25 passes the cloud gate (all scenes positive) at +63…97 % points; the mesh absorbs it: +0.0002 mean, Truck −0.0055, for +34…51 % mesh memory and +35…60 % mesh wall |
| re-probe the 4-neighbours of a failed join | — | instrumentation showed 0.2–0.4 % of the valid depths recoverable; not built |
| `ReconstructMesh --min-point-distance 2.0` (default 1.5) | raw-mesh −0.0035 (Truck −0.0074) | −23 % mesh memory — a trade, unchanged |

The structural arms were chosen by a pixel/probe accounting of what fusion admits and drops (25 %
of the valid depths of Barn/Ignatius, 35 % of Meetingroom, 20 % of Truck end in clusters the
keep-rule discards, and corroboration could have kept 12–16 % of them); that instrumentation was
removed with the arms.

## 5. Open

- Fusion degrades silently under memory pressure (§ 3 *Memory*): a neighbor that does not fit the
  cache is skipped with a warning. Blocking until it can be loaded, or failing, would make the
  output independent of the free RAM at run time.
- `nMaxViewsFuse` is 32 while estimation used 12 neighbors: the flood-fill reaches past the
  estimation neighborhood, and whether that interacts with the prior rescue is unexamined.
- Meetingroom's mesh wall grows superlinearly with its point count (×3.2 points cost ×6.1 wall in
  an early pairing, in every sub-stage); it needs a profile before any completeness push.
