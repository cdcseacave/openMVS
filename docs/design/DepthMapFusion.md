# Depth-Map Fusion

Record of the depth-map-fusion default-improvement campaign: where today's fusion behavior came
from (provenance), what it mechanically does differently from the June baseline (mechanism), the
slice-by-slice measurements as they land, and the decisions reserved to the maintainer. It covers
`MVS::DepthMapsData::DenseFuseDepthMaps`, the `--fusion-filter 2` default path, and the `OPTDENSE`
fusion defaults in `libs/MVS/DepthMap.cpp` / `apps/DensifyPointCloud/DensifyPointCloud.cpp`. The
mesh stage that consumes fusion's output keeps its own record,
`docs/design/DelaunayMeshReconstruction.md`, cross-linked from that document's §8; this document is
the fusion-side complement, not a replacement. Protocol in brief (§3): Tanks-and-Temples (T&T)
training scenes Barn / Ignatius / Meetingroom / Truck, fusion A/Bs run on frozen `.dmap` files,
gated on cloud F1, with mesh F1 reported (raw+gated surface, cleaned mesh alongside, both cleaned
by mesh visibility against the scene cameras since 2026-08-27 — § 3 *Mesh visibility*) as
mandatory downstream evidence, never as a substitute gate.

---

## 1. Provenance of the two cloud families

### The measured delta (cloud stage)

The mesh benchmark (`bench/out_mesh/results.csv`) carries two families of frozen dense clouds.
Family **A** is the frozen `<Scene>/runMetashape/scene_dense.{mvs,ply}` clouds the bench has used
since June. Family **B**, as originally measured, was the `<Scene>/runFusionStats/` clouds
re-densified 2026-08-19 for Phase 5.0/5.1 of the Delaunay work (three scenes; Barn had none):

| scene | A points | A cloud P / R / F1 | B points | B cloud P / R / F1 | Δcloud F1 |
|---|---|---|---|---|---|
| Barn | 7,703,886 | 0.5983 / 0.5994 / **0.5988** | *(no B cloud)* | — | — |
| Ignatius | 5,211,727 | 0.7271 / 0.7494 / **0.7381** | 9,844,135 | 0.7182 / 0.8370 / **0.7731** | **+0.0350** |
| Meetingroom | 3,106,975 | 0.5608 / 0.2264 / **0.3225** | 10,000,054 | 0.4853 / 0.3968 / **0.4366** | **+0.1141** |
| Truck | 5,990,832 | 0.6916 / 0.7210 / **0.7060** | 9,715,279 | 0.6785 / 0.7571 / **0.7156** | **+0.0096** |

Shape of the win: recall up a lot, precision down a little, points ×1.6–3.2. Recall
+0.0876 / +0.1704 / +0.0361; precision −0.0089 / −0.0755 / −0.0131.

**Correction (2026-08-25 review).** `runFusionStats/` (Aug 19, three scenes, second pass carrying
the since-removed `--export-unfused-file`) is superseded. The canonical family-B set is
`<Scene>/runConfAdj/`: all four scenes, Barn included, densified 2026-08-23 by the same build
(`e2f38b4f*`, Aug 18) with the identical flag set, dmaps kept (410/263/371/251). The bench's
current rows (`<Scene>@runConfAdj`, tag `halfmesh-review`) are already scored against them: cloud
F1 Barn **0.6416**, Ignatius **0.7713**, Meetingroom **0.4370**. Truck has not been re-measured
under `runConfAdj` as of this writing; the table above's `runFusionStats` value (**0.7156**) is
carried forward as the working number until it is. `--tag fusion-base` is the promoted baseline
(S0(b)); `S0(a)`, which would have back-filled a Barn `runFusionStats` folder, is void.

### Downstream (mesh) cost — wall time, memory, vertex count

The plan's original § 0 paired each cloud row with a downstream mesh measurement. Every mesh-F1
figure in that pairing — including the originally reported "Truck regresses −0.0058" and
"Meetingroom +0.0194" facts — was scored through the pre-2026-08-21 `Mesh::Clean` smoother bug
that collapsed the mesh F1 on fine-resolution scenes (Ignatius raw 0.6986 → 0.3295 after two smoothing iterations; `DelaunayMeshReconstruction.md` §3:
*"Do not cite a pre-2026-08-21 mesh-F1 number as evidence for anything."*). Those numbers are not
admissible evidence and are omitted here. Only the wall-time, peak-memory and
Delaunay-vertex-count columns, which do not depend on the smoother, survive:

| scene | A `recon_wall_s` | B `recon_wall_s` | A `peak_ws_mb` | B `peak_ws_mb` | A→B `delaunay_verts` |
|---|---|---|---|---|---|
| Ignatius | 139.2 | 243.5 (×1.75) | 5,353 | 9,262 (×1.73) | 2.74 M → 4.75 M |
| Meetingroom | 97.6 | 597.8 (×6.13) | 4,814 | 12,768 (×2.65) | 2.48 M → 6.50 M |
| Truck | 160.5 | 237.8 (×1.48) | 7,108 | 10,256 (×1.44) | 3.63 M → 5.22 M |

Meetingroom's wall time already grew far out of proportion to its point count under this stale
A/B pairing (×3.2 points → ×6.1 wall) — a cost-scaling signal independent of the invalidated F1s,
and the reason § 3's cost-pairing rule exists.

### Established cause

The two families differ only in the binary. Verified, not inferred:

| Evidence | Where |
|---|---|
| Family A command line, all four scenes: `DensifyPointCloud scene.mvs --resolution-level 1 --number-views 12 --estimate-roi 0 --crop-to-roi 0 --tower-mode 0` | `<Scene>/runMetashape/DensifyPointCloud-2606*.log`, Truck's re-freeze `-260729125834007651.log` |
| Family B command line: identical, plus `-v 2` (and `--export-unfused-file unfused.bin` on the second Ignatius/Meetingroom/Truck pass) | `<Scene>/runFusionStats/DensifyPointCloud-2608*.log` |
| Family A build: `OpenMVS x64 v2.4.0 (9a007b5*)`, Jun 6 2026 (Truck's re-freeze: `e884035`, Jul 29 2026 — still pre-#1292) | log header lines |
| Family B build: `OpenMVS x64 v2.4.0 (e2f38b4f*)`, Aug 18 2026 = develop head plus the local Phase-5.0 accounting patch | log header lines |
| Family A logs contain no `Adaptive confidence enabled` / `Integrated confidence recalibration` lines; family B logs contain both | log greps |

Note: the accounting lines above showed up at `-v 2` at the time; that no longer holds.
`FusionStats::Log()` is gated `DEBUG_ULTIMATE` (prints only when `2 < VERBOSITY_LEVEL`), so
reproducing the `Fusion pixel accounting` / `Fusion probe accounting` / confidence-histogram
lines today requires **`-v 3`** — the `-v 2` family-B logs cited above predate the committed
accounting form.

Diffing the option tables of the two revisions (`git show 9a007b5:libs/MVS/DepthMap.cpp` vs
`git show HEAD:libs/MVS/DepthMap.cpp`, `DEFVAR_OPTDENSE` lines only) yields exactly one commit's
worth of change to the densify defaults — `f61bffef` *"dense: depth-map confidence recalibration
(#1292)"*, 2026-08-13:

| Option | June (family A) | today (family B) | Symbol |
|---|---|---|---|
| `nOptimize` / `--postprocess-dmaps` | `0` | **`4`** = `ADJUST_CONFIDENCE_AUTO` | `OPTDENSE::nOptimize`, `libs/MVS/DepthMap.cpp` / `.h`, `DensifyPointCloud.cpp` |
| `fFusePriorWeight` / `--fusion-prior-weight` | *(did not exist)* | **`3.0`** | `OPTDENSE::fFusePriorWeight`, `libs/MVS/DepthMap.cpp`, `DensifyPointCloud.cpp` |
| `nFuseViolationMax` | *(did not exist)* | **`0`** (strict) | `OPTDENSE::nFuseViolationMax`, `libs/MVS/DepthMap.cpp` |
| `bEstimateConfidenceCUDA` | *(did not exist)* | `1` | `OPTDENSE::bEstimateConfidenceCUDA` (new), `libs/MVS/DepthMap.h` |
| `nMinViewsFilter`, `nMinViewsFilterAdjust` | present | retired | — |
| `nPatchMatchCUDAInstances` | *(did not exist)* | `4` | `OPTDENSE::nPatchMatchCUDAInstances`, `libs/MVS/DepthMap.cpp`, `DensifyPointCloud.cpp` |

Everything else a fusion result could depend on is byte-identical between the two revisions:
`fDepthReprojectionErrorThreshold 1.2`, `fDepthDiffThreshold 0.01`, `fNormalDiffThreshold 25`,
`nMinPixelsFuse 5`, `nMinViewsFuse 2`, `nMaxViewsFuse 32`, `nMaxPointsFuse 1000`,
`fNCCThresholdKeep 0.9`, `fEstimationGeometricWeight 0.1`, `nEstimationGeometricIters 2`,
`nFuseFilter 2`.

**Conclusion: the family-B behaviour IS today's default on develop.** The bench's family-A clouds
are simply stale June artifacts. There is no "family-B setting" to adopt — there is a stale
baseline to re-freeze (S0) and a set of further fusion levers to test (§ 3, § 4).

### Hypotheses checked and refuted

The Phase 5.0 T&T readout (in the pre-consolidation `DelaunayMeshReconstruction.md`, now git
history) attributed the A/B gap to a `Densify.ini` in the working folder overriding defaults
(reprojection threshold 1.2 vs today's 1.0, geometric weight 0.1 vs 0.3, pre-recalibration
confidence lineage). Two of those three claims are wrong:

- **`Densify.ini` was never read.** `DensifyPointCloud` calls `OPTDENSE::oConfig.Load` with a
  name that stays empty unless `--dense-config-file` is passed; `CConfigTable::Load` forwards
  straight to `SML::Load(f)`, which opens the literal path and fails. No family-A command line
  passed the flag. Decisive counterexample: `Meetingroom/runMetashape/` has no `Densify.ini` at
  all yet shows the most extreme family-A profile (P 0.5608 / R 0.2264).
- **The reprojection default is still 1.2, in both families.** Commit `dc32ab8` *"dense: lower
  default fusion reprojection threshold 1.2 -> 1.0"* (2026-06-10) is not on develop:
  `git merge-base --is-ancestor dc32ab8 HEAD` → NO; `git branch --contains dc32ab8` → only
  `dense-cell-neighbors`; `docs/fusion_reprojection_threshold.md` is absent from the worktree; the
  live code still reads `1.2`.
- **The geometric weight is still 0.1, in both families.** `833e93c` *"dense: raise PatchMatch
  geometric-consistency weight 0.1 -> 0.3"* (2026-06-09) is likewise only on
  `dense-cell-neighbors`; the live code reads `0.1`, and there is no `--geometric-weight` CLI
  option at all.
- **Not the neighbor selector.** `9a007b5` is titled "per-cell neighbor-view selection (WIP
  snapshot)", but grepping that revision for `CellViews|nCellViews|PerCell|nGrid` returns nothing
  — no per-cell selector was active. Both families log `Reference image N paired with 12 views`.
- **Not resolution, view count, ROI or tower mode** — identical flags, identical `960x540`
  depth-map sizes, identical image counts (263 / 371 / 251 / 410).

### Stranded commits

Commits `dc32ab8` (fusion reprojection threshold 1.2 → 1.0, with its
`docs/fusion_reprojection_threshold.md`) and `833e93c` (PatchMatch geometric weight 0.1 → 0.3) are
validated-in-June work that exists only on branch `dense-cell-neighbors` and is absent from
develop — flagged to the maintainer (§ 5). The first is re-validated by this campaign (slice S3);
the second is estimation-side, changes the depth maps themselves, and is out of scope here (it is
handed to a separate estimation plan).

---

## 2. Mechanism — what #1292 changed in what fusion admits

`DenseFuseDepthMaps` grows one cluster per unclaimed seed pixel by flood-filling neighbor views
through the `FusePoint` lambda, then applies a keep-rule. The gates:

| Gate | Location | Constant |
|---|---|---|
| seed/probe confidence | `FusePoint`, confidence check | `conf < minConfidence`, `minConfidence = 1 − fNCCThresholdKeep = 0.1` |
| depth agreement | `FusePoint`, depth-diff check (with an FSV sub-classification alongside it) | `fDepthDiffThreshold = 0.01` |
| lateral reprojection | `FusePoint`, reprojection check | `normSq(diff) > maxReprojErrorSq`, `maxReprojErrorSq = fDepthReprojectionErrorThreshold² = 1.44` |
| normal agreement | `FusePoint`, normal check | `fNormalDiffThreshold = 25°` |
| keep-rule | `DenseFuseDepthMaps` seed loop, after `FusePoint` returns | `pixels + virtualSupport ≥ nMinPixelsFuse(5)` **and** `views + virtualSupport ≥ nMinViewsFuse(2)` |
| FSV guard on rescued points | `DenseFuseDepthMaps` seed loop, after the keep-rule | `fusedViolViews.size() ≤ nFuseViolationMax(0)` |

(Line numbers drift with every commit — locate these by symbol, not by number; see § 3.)

#1292 moves two of these:

**(a) The confidence gate now reads a recalibrated posterior.** `AdjustConfidence`
(`libs/MVS/SceneDensify.cpp`) replaces photometric 1-NCC with a value that predicts "will this
depth survive fusion": `gate = 1 − exp(−(K + kPrior·pGeo)/tau)` over cross-view confirmations `K`
and the intra-map geometric prior `pGeo`. The output is bimodal, so the fixed 0.1 gate lands in
the valley rather than mid-distribution. Family B's own histograms confirm it: admitted pixels
pile into bins 8–9, dropped pixels into bin 0 (`Fusion confidence histogram` lines,
`Ignatius/runFusionStats/DensifyPointCloud-26081915165600B59D.log`).

**(b) The keep-rule gained a rescue term.** `virtualSupport = fFusePriorWeight · priorMap(i,j)`
(evaluated at the keep-rule above), `fFusePriorWeight = 3`, `priorMap ∈ [0,1]` from
`ComputeIntraMapPrior`. At prior 1.0 a cluster of 2 pixels seen by 1 view now passes a rule that
used to demand 5 pixels and 2 views. This attacks the drop channel Phase 5.0 measured as
dominant — 20–35 % of all valid depths lost to `min-pixels`. `nFuseViolationMax = 0` is the
counterweight: a rescued point contradicted by even one free-space-violating view is dropped
again through the FSV guard (non-rescued points are never subject to it).

That is the recall mechanism, and it is exactly the observed signature: many more, smaller
clusters (family A `nDepths`/point ≈ 18.7, family B ≈ 8.0 — `nDepths` sums `fusedViews.size()`
over all clusters, the older summary-line counter distinct from the `Fusion pixel accounting`
lines), higher recall, slightly lower precision.

**Post-#1292 residual head-room** (Phase 5.0 T&T accounting, preserved in git history): even with
the rescue, Ignatius still drops 52.0 % of valid depths (26.4 % low-conf, 25.4 % min-pixels),
Meetingroom 69.9 % (34.6 / 35.0), Truck 44.0 % (23.6 / 20.3); each scene throws away 8.7–10.0 M
pixels at confidence ≥ 0.7. The channel is not exhausted.

---

## 3. Protocol

### Harness

`bench/run_mesh.py --score-raw --work-subdir runConfAdj` for paired cloud+mesh rows; a new
`bench/run_fusion.py` for fusion-only arms (re-fuse frozen dmaps, cloud F1 only, no mesh stage).
Scenes Barn / Ignatius / Meetingroom / Truck; python sampler; `--sample-points 10000000`,
`--sample-seed 42`.

### Frozen dmaps — mandatory

PatchMatch is unseeded: identical build and flags reproduce point counts with 0.3–14 % run-to-run
spread (Barn 3.6 %, Meetingroom 14 %, Ignatius 0.3 % across three June runs; family-B Ignatius
4.5 % across two runs). Comparing two fusion settings across two separate densifications is
therefore invalid — every fusion A/B in this campaign runs on frozen `.dmap` files, re-fusing the
same bytes with one constant changed.

Re-running the densifier's command line in a folder that already holds its `.dmap` set is **not**
by itself a fusion-only run: the first pass does load the cached maps, but the geometric-consistency
iterations (`--geometric-iters`, default 2) then re-estimate every map (`depthNNNN.geo.dmap`).
A fusion-only re-run needs the frozen flags **plus `--geometric-iters 0`**, which loads the cached
(final, geometric-consistent) maps and goes straight to fusion; `--fusion-mode` is not a
fusion-only switch (`-1` exports disparity maps). `bench/run_fusion.py` bakes this in, snapshots
every `.dmap`'s size+mtime before the run and aborts if any changed.

### Memory — mandatory

`DenseFuseDepthMaps` (`libs/MVS/SceneDensify.cpp`) keeps neighbor depth maps in a memory-bounded
`DMapCache` whose budget is taken from free RAM at run time (`GetAvailableMemory` →
`cacheDMaps.SetMaxMemory`, re-evaluated per chunk). When the budget is too small the binary logs
`warning: not enough memory to cache depth-maps (2648MB needed, 2144MB available)` and the neighbor
loop silently skips every neighbor whose map is not resident (`if (depthDataB.IsEmpty()) continue;`)
— those neighbors are never probed for that reference image, and the fused cloud then depends on
how much RAM happened to be free while it ran. Evidence: the same Barn `--fusion-prior-weight 4`
re-fuse, same binary flags, same 410 frozen maps (identical valid-depth count 201,097,868 and
identical low-confidence drop 50,013,586), gave 19,764,512 points at a 92 % cache hit rate (clean
run, 14:19) vs. 19,937,081 points (+0.9 %, cloud F1 0.6502 → 0.6511) at a 90 % hit rate with two
starvation warnings (15:58, run beside a mesh reconstruction and a T&T evaluation). Clean runs on
this 32 GB machine all show 89–92 % hit rates; starved ones 53–90 %. A scan of all 95 arm logs
found 9 sweep rows that ran starved (hit rate in parentheses): Barn reproj09 (79 %); Ignatius
conf005 (58 %), conf015 (67 %), minpix3 (62 %), reproj11 (70 %); Meetingroom conf005 (53 %),
conf015 (86 %); Truck minpix4 (88 %), violm1 (90 %). The base rows and every other sweep row are
clean (the base arm reproduced its point counts exactly three times). **Rule**: a fusion arm must
never run beside a mesh reconstruction or a T&T evaluation. `bench/run_fusion.py` now records
`cache_warnings` / `cache_hit_pct` per row, waits for ≥ 10 GB free RAM before launching
(`--min-free-gb`), marks starved rows with `*` in `--summary`, and `--rerun-starved` repeats them.
**Replayed clean 2026-08-27 (01:30–01:50).** All 9 starved rows were re-run on a quiet machine and
came back with `cache_warnings = 0`; every § 4 verdict that rested on a starved row now rests on
the clean replay, and the provisional flags below are resolved.

### Mesh visibility — mandatory (2026-08-27)

When scoring a mesh, the sampled cloud must first be cleaned by mesh visibility: the mesh is
rendered into every scene camera (a z-buffer built by ray casting), and every sample not seen by
at least one camera is dropped. A mesh fills holes with surface no camera ever observed; that
surface can be geometrically correct and is still absent from the laser ground truth, so until it
is removed it counts as a precision miss that has nothing to do with the fusion or mesh-stage
lever under test. Implementation: `bench/mesh_visibility.py` (runs under the T&T evaluator venv;
cameras loaded from the scene `.mvs` via `scripts/python/MvsUtils.py`, pose composition follows
`Interface::Platform::GetPose`, the pixel-center convention follows `Platform::ScaleK`; renders at
the stored 960×540, a 3×3 max-filtered z-buffer, 1 % relative depth tolerance), wired into
`bench/run_mesh.py` as new `vis_*` (cleaned mesh) and `rawvis_*` (cleaned raw graph-cut surface)
columns alongside the unchanged `mesh_*` / `raw_*` columns. **`rawvis_f1` is the downstream metric
from now on; `raw_f1` is kept only for continuity.**

**Consequence.** Every mesh F1 in this document measured before 2026-08-27 is superseded by this
change — the S0 mesh baseline (0.6225 / 0.7607 / 0.4376 / 0.6544), the S4 prior-4 mesh pairing,
the S7 `--min-point-distance 2.0` row, and the S3 reprojection mesh pairings all scored `raw_f1`
without the visibility clean. They are being re-measured (base, reproj 0.9, reproj 0.6, prior 4 ×
baseline/mpd2 × 4 scenes, tag `fusion-vis`) and the affected tables will be replaced when the
re-runs land; the old numbers stay in place until then, each flagged inline.

### Logging and residuals

The `Fusion pixel accounting` / `Fusion probe accounting` / confidence-histogram lines are
reported at `-v 2`, the level of the fusion summary line (they were `DEBUG_ULTIMATE` = `-v 3`
when first committed, but `-v 3` also dumps every depth map as `depthNNNN.ply/.png/.conf.png`
into the working folder — ~13 MB per map, ~5 GB per scene — so an accounting readable only there
was unusable on a real dataset; lowered in the S9 commit). Both accounting residuals must read
`0` on every arm — they are self-checking partitions.

### Noise floors

- **Mesh stage**: 0.0006 (paired identical baseline runs, 4 scenes —
  `DelaunayMeshReconstruction.md` § 2). This supersedes an earlier 0.0013 estimate that predates
  the smoother fix.
- **Cloud stage** on frozen dmaps is deterministic — the fuse is serial and the only parallel
  region caches dmaps, never calls `FusePoint`. The residual source is the evaluator's ICP when
  `<Scene>_final_transform.npy` is absent; `run_mesh.py` logs its presence per scene. No such file
  exists yet for any scene — S0 captures the baseline cloud's refined transform into the real
  cache file for all four scenes before any gated run.
- `--noise-floor` re-measures both any time the machine or build changes.

### Acceptance gate

Adopted from the mesh effort's acceptance gates (`DelaunayMeshReconstruction.md` § 8), extended
for the downstream cost this plan can move:

- **Quality**: mean paired cloud-F1 ≥ **+0.003** beyond noise, and no scene regressing > 0.003. P
  and R each within 1 pp unless the complementary gain nets a clear F win.
- **Downstream, mandatory reporting, not merely advisory**: the same run reports mesh F1 (the
  `raw_f1` column — the raw+gated surface, cleaned `mesh_f1` reported alongside but not gating),
  `recon_wall_s`, `peak_ws_mb`, `pts_inserted`, `delaunay_verts`. Any of a `raw_f1` regression
  > 0.003 on any scene, a `recon_wall_s` increase > 25 % (median of ≥ 3 runs), or a `peak_ws_mb`
  increase > 15 % escalates the candidate to § 5 (maintainer decision) instead of being proposed
  as a default, even if the cloud gate passes.
- **Repeats**: 1 run suffices for a fusion-only quality row (byte-identical input); ≥ 3 runs and
  medians for anything quoting wall time or memory — and only for rows with `cache_warnings = 0`
  (see *Memory — mandatory*).
- **Instrumentation**: both fusion accounting residuals `0`; report the per-channel drop table
  (low-conf / min-pixels / min-views / violation) for every arm, so a quality change is always
  explained by a channel change.

### Cost pairing

Any candidate that raises the fused point count by > 20 % must be reported with a paired
`--min-point-distance 2.0` mesh row alongside the default 1.5, so the maintainer sees
quality-at-equal-cost, not just quality-at-any-cost.

---

## 4. Slice results

### S0 — Re-baseline the bench

*Done 2026-08-25.* `<Scene>/runConfAdj/` (all four scenes, one build, one flag set, dmaps kept) is
the canonical fusion baseline, tag `fusion-base`; the June `runMetashape` clouds stay frozen as the
historical row. Three checks passed before any arm ran:

1. **The HEAD binary reproduces the frozen clouds exactly.** A fusion-only re-fuse
   (`--geometric-iters 0`, § 3) of each scene's dmaps returns the frozen point count to the point:
   Barn 13,966,904 / Ignatius 9,424,360 / Meetingroom 8,801,700 / Truck 9,392,543. Fusion alone
   takes 76 / 54 / 64 / 53 s.
2. **Alignment frozen.** `<Scene>_final_transform.npy` written for all four scenes from that
   re-fuse's ICP, so every later arm is scored in the same GT frame (the evaluator loads the file
   and skips its ICP).
3. **Cloud stage** (frozen alignment): Barn P 0.5746 / R 0.7263 / **F1 0.6416**; Ignatius
   0.7081 / 0.8472 / **0.7714**; Meetingroom 0.5072 / 0.3838 / **0.4370**; Truck
   0.6761 / 0.7644 / **0.7176** — Barn/Ignatius/Meetingroom within 0.0001 of the pre-freeze
   bench rows, Truck's first `runConfAdj` number (+0.0020 over the superseded `runFusionStats`
   cloud, a different densification).

*Superseded 2026-08-27*: scored without the mesh-visibility clean now mandatory (§ 3 *Mesh
visibility*); re-measurement under tag `fusion-vis` pending, see that subsection.

Mesh baseline (`run_mesh.py --work-subdir runConfAdj --variants baseline --score-raw`, python
sampler, 10 M samples, seed 42), the reference for every paired mesh row that follows:

| scene | cloud F1 | raw+gated P / R / **F1** | cleaned P / R / F1 | Delaunay verts | cells | `recon_wall_s` | `peak_ws_mb` |
|---|---|---|---|---|---|---|---|
| Barn | 0.6416 | 0.5994 / 0.6473 / **0.6225** | 0.6225 / 0.6288 / 0.6257 | 8.42 M | 52.8 M | 310 | 16,370 |
| Ignatius | 0.7714 | 0.7511 / 0.7705 / **0.7607** | 0.7698 / 0.7365 / 0.7528 | 4.57 M | 28.6 M | 187 | 8,883 |
| Meetingroom | 0.4370 | 0.5078 / 0.3845 / **0.4376** | 0.5280 / 0.3769 / 0.4398 | 5.94 M | 38.0 M | 213 | 11,602 |
| Truck | 0.7176 | 0.6177 / 0.6958 / **0.6544** | 0.6321 / 0.6766 / 0.6536 | 5.11 M | 32.2 M | 208 | 10,011 |

Meetingroom's mesh (raw and cleaned) sits above its own cloud; the other three meshes absorb
0.011–0.063 of their cloud F1, Truck by far the most. The `recon_wall_s` column of this table was
measured while a library build ran on the same machine; it is a reference for memory and vertex
counts, and the timing rows are re-measured solo (≥ 3 runs, medians) wherever a candidate's cost
is judged (S7). The plan's § 0 "Truck mesh regresses" and "Meetingroom ×6.1 wall" facts are not
reproduced here — they compared two different clouds under the broken smoother; the paired mesh
rows of the later slices replace them.

### S1 — Record the provenance finding (doc-only)

*Done — this document.*

### S2 — Channel ablation on frozen dmaps

**(ii) prior rescue off on the recalibrated dmaps** (`--fusion-prior-weight 0`)

*Measured 2026-08-25* on the frozen `runConfAdj` dmaps, everything else at today's defaults:

| scene | base P / R / F1 | 0 P / R / F1 | ΔF1 | ΔP | ΔR | points |
|---|---|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5934 / 0.6353 / **0.6136** | **−0.0280** | +0.0188 | −0.0910 | −39.4 % |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.7198 / 0.7640 / **0.7412** | **−0.0302** | +0.0117 | −0.0832 | −43.9 % |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.5488 / 0.2944 / **0.3832** | **−0.0538** | +0.0416 | −0.0894 | −48.3 % |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.7003 / 0.7174 / **0.7087** | **−0.0089** | +0.0242 | −0.0470 | −37.6 % |

Mean **−0.0302**. The prior-rescue term is worth **+0.0302** mean cloud F1 on the recalibrated
dmaps (Truck +0.0089 … Meetingroom +0.0538); removing it trades P +1.2…+4.2 pp for R
−4.7…−9.1 pp and −38…−48 % points — the rescue is the recall engine of family B, and the second
half of the S2 split is answered: #1292's gain is not the confidence recalibration alone.

**(iii) raw confidence + prior rescue** (`rawbase`, `--postprocess-dmaps 0` — raw photometric
confidence, no confidence-adjust pass — prior rescue at its default weight 3)

*Measured 2026-08-26* on `runConfAdjRaw`, an independent full re-densify of each scene (tag
`fusion-s10`, `bench/campaign_raw.sh`), fused and scored against `base10`:

| scene | F1 | ΔF1 vs base | ΔP (pp) | ΔR (pp) | points | admitted % |
|---|---|---|---|---|---|---|
| Barn | 0.6387 | **−0.0029** | −0.54 | +0.13 | +1.5 % | 50.32 % |
| Ignatius | 0.7703 | **−0.0011** | −0.15 | −0.07 | +1.1 % | 48.61 % |
| Meetingroom | 0.4358 | **−0.0012** | −1.28 | +0.58 | +3.5 % | 31.39 % |
| Truck | 0.7161 | **−0.0015** | −0.28 | +0.04 | +1.0 % | 56.55 % |

Mean **−0.0017**. `rawrefuse`, a fusion-only re-fuse of these same dmaps, reproduces `rawbase`
exactly on all four scenes (identical F1/P/R/points) — the fusion-only re-fuse protocol holds on
this dmap set too. The confidence-adjust pass is worth 0.001–0.003 cloud F1, almost entirely
precision (R moves only −0.07…+0.58 pp, essentially flat), growing with scene difficulty: Ignatius
0.0011 < Truck 0.0015 < Barn 0.0029 (Meetingroom's F1 delta, 0.0012, is out of that order, but it
pays the largest precision cost, −1.28 pp). It is not the family-A→B jump.

**(iv) raw confidence, no rescue** (`rawprior0`, `--fusion-prior-weight 0` on the same
`runConfAdjRaw` dmaps) — family A's fusion rule (no confidence recalibration, no prior rescue) run
against family B's estimation:

| scene | F1 | ΔF1 vs base | ΔF1 vs `rawbase` | ΔF1 S2 (ii) (adjusted dmaps) | ΔP vs base (pp) | ΔR vs base (pp) | points vs base |
|---|---|---|---|---|---|---|---|
| Barn | 0.6110 | **−0.0306** | −0.0277 | −0.0280 | +1.10 | −8.75 | −37.9 % |
| Ignatius | 0.7402 | **−0.0312** | −0.0301 | −0.0302 | +0.81 | −8.14 | −42.3 % |
| Meetingroom | 0.3854 | **−0.0516** | −0.0504 | −0.0538 | +2.09 | −8.04 | −44.4 % |
| Truck | 0.7062 | **−0.0114** | −0.0099 | −0.0089 | +1.76 | −4.52 | −36.2 % |

Mean ΔF1 vs base **−0.0312**, vs `rawbase` **−0.0295** — close to the (ii) mean measured on the
adjusted dmaps (**−0.0302**), and within 0.0034 of it scene by scene (Meetingroom the widest gap,
Ignatius the closest at 0.0001). The prior-rescue term is worth almost the same, regardless of
which confidence feeds it. This answers § 6 open question 3: the family-B precision drop is the
rescue's own P/R trade, not outliers admitted by the recalibrated confidence — switching the rescue
off restores precision by +0.8…+2.1 pp at a cost of −4.5…−8.8 pp recall — and it is a trade nobody
should unwind, since F1 loses 0.010–0.052 depending on scene.

CUDA PatchMatch is unseeded, so the `runConfAdjRaw` dmaps behind (iii) and (iv) are an independent
estimation run, not byte-identical to the frozen `runConfAdj` set — part of the gap against
`base10` in both tables above is estimation noise on top of the ablated channel (§ 3). Fusion wall
time in these rows (`fus`) is not a solo measurement — other tooling ran concurrently — so no wall
conclusion is drawn from it here. S6, the CPU-parity arm, shares this same raw-dmap chain (§ 4 S6);
arm (i) — adjusted confidence with rescue — is `base10` itself and needs no separate row.

### S3 — C1: restore `fDepthReprojectionErrorThreshold` 1.2 → 1.0

*Measured 2026-08-25* on the frozen `runConfAdj` dmaps (`--fusion-reprojection-threshold 1.0`,
everything else at today's defaults, i.e. with #1292's recalibrated confidence and prior rescue
active — the re-validation the June evidence needed):

| scene | base P / R / F1 | 1.0 P / R / F1 | ΔF1 | ΔP | ΔR | points |
|---|---|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5770 / 0.7394 / **0.6482** | **+0.0066** | +0.0024 | +0.0131 | +9.3 % |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.7087 / 0.8566 / **0.7757** | **+0.0043** | +0.0006 | +0.0094 | +8.5 % |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.5130 / 0.3909 / **0.4437** | **+0.0067** | +0.0058 | +0.0071 | +5.9 % |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.6790 / 0.7713 / **0.7222** | **+0.0046** | +0.0029 | +0.0069 | +10.9 % |

Mean **+0.0055**, every scene positive, precision *and* recall up on every scene — the same
Pareto signature the June measurement found (+0.0028…+0.0052 pre-#1292), slightly larger now.
The channel table explains it: the `min-pixels` drop share barely moves (+0.5…+0.7 pp), so the
tighter lateral tolerance is not starving clusters — it is splitting over-merged ones into
distinct, better-placed points. **Cloud gate: passed.**

*Superseded 2026-08-27*: scored without the mesh-visibility clean now mandatory (§ 3 *Mesh
visibility*); re-measurement under tag `fusion-vis` pending, see that subsection.

Paired mesh rows (`run_mesh.py --cloud-name reproj10 --score-raw`, same protocol as S0):

| scene | raw+gated F1 base → 1.0 | cleaned F1 base → 1.0 | Delaunay verts | `peak_ws_mb` |
|---|---|---|---|---|
| Barn | 0.6225 → 0.6221 (−0.0004) | 0.6257 → 0.6241 (−0.0016) | ×1.07 | ×1.02 |
| Ignatius | 0.7607 → 0.7607 (0.0000) | 0.7528 → 0.7529 (+0.0001) | ×1.05 | ×1.03 |
| Meetingroom | 0.4376 → 0.4377 (+0.0001) | 0.4398 → 0.4399 (+0.0001) | ×1.05 | ×1.05 |
| Truck | 0.6544 → 0.6557 (+0.0013) | 0.6536 → 0.6549 (+0.0013) | ×1.07 | ×1.03 |

**The mesh stage absorbs the whole cloud gain**: three scenes within the 0.0006 mesh noise floor,
Truck +0.0013, nothing near the 0.003 escalation clause, memory +2–5 %. (The `recon_wall_s`
column was taken while the fusion sweep ran on the same machine; it is re-measured solo in S7.)
This is § 6 question 1 answered for this lever: a +0.005 cloud-F1 lever is invisible after
Delaunay + graph-cut at this point density, so the case for the flip rests on the cloud — where
the point cloud *is* the product — plus the fact that it costs nothing downstream.

**Recommendation** (decision reserved, § 5 item 1): adopt **at least** 1.0 as the default (the
dose curve below says lower may be better; that part waits on S7) — validated twice, on two
confidence lineages, Pareto on every scene, neutral for the mesh. Implementation is the two
literals of `dc32ab8` (`libs/MVS/DepthMap.cpp`, `DensifyPointCloud.cpp`) plus its
`docs/fusion_reprojection_threshold.md`.

**Dose curve** (same frozen dmaps)

| threshold | Barn ΔF1 | Ignatius ΔF1 | Meetingroom ΔF1 | Truck ΔF1 | mean ΔF1 | points range |
|---|---|---|---|---|---|---|
| 1.1 | +0.0027 | +0.0015 | +0.0027 | +0.0019 | **+0.0022** | +2.2…+4.4 % |
| 1.0 | +0.0066 | +0.0043 | +0.0067 | +0.0046 | **+0.0055** | +5.9…+10.9 % |
| 0.9 | +0.0149 | +0.0103 | +0.0141 | +0.0103 | **+0.0124** | +11.7…+27.7 % |
| 0.8 | +0.0175 | +0.0116 | +0.0174 | +0.0117 | **+0.0146** | +14.4…+33.9 % |
| 0.7 | +0.0205 | +0.0135 | +0.0196 | +0.0133 | **+0.0167** | +16.6…+41.3 % |
| 0.6 | +0.0243 | +0.0153 | +0.0212 | +0.0157 | **+0.0191** | +16.7…+49.5 % |

Full P/R/F1 for 0.9 (`--fusion-reprojection-threshold 0.9`):

| scene | base P / R / F1 | 0.9 P / R / F1 | ΔF1 | ΔP | ΔR | points |
|---|---|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5814 / 0.7540 / **0.6565** | **+0.0149** | +0.0068 | +0.0277 | +23.0 % |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.7128 / 0.8654 / **0.7817** | **+0.0103** | +0.0047 | +0.0182 | +21.4 % |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.5250 / 0.3955 / **0.4511** | **+0.0141** | +0.0178 | +0.0117 | +11.7 % |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.6840 / 0.7777 / **0.7279** | **+0.0103** | +0.0079 | +0.0133 | +27.7 % |

*Replayed clean 2026-08-27*: the Barn 0.9 row and the Ignatius 1.1 row had run dmap-cache starved
(79 % / 70 % hit rate, § 3 *Memory*); re-run on a quiet machine they came back clean
(`cache_warnings = 0`), and the corrected values are folded into the dose-curve table and the
Barn 0.9 row above. The 0.8 and 0.7 rows were clean throughout.

The curve is monotone from 1.1 down through 0.6, clean on every scene at every dose — 0.9 beats
1.0 on every scene with P *and* R up on every scene (P +0.5…+1.8 pp, R +1.2…+2.8 pp) at +12…28 %
points; 1.1, once replayed clean, is a small positive on every scene too (mean +0.0022, min
+0.0015 on Ignatius), not the wash the starved Ignatius reading (−0.0088) had suggested. The
reprojection-rejection counts grow with the tightening while low-conf drop is untouched, i.e. the
whole effect is cluster splitting: probes rejected by the reprojection test go base → 0.9 Barn
61.2 M → 90.4 M, Ignatius 32.5 M → 46.5 M, Meetingroom 29.0 M → 43.6 M, Truck 41.7 M → 58.5 M.

Full P/R/F1 for 0.7 (`--fusion-reprojection-threshold 0.7`):

| scene | base P / R / F1 | 0.7 P / R / F1 | ΔF1 | ΔP | ΔR | points |
|---|---|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5843 / 0.7638 / **0.6621** | **+0.0205** | +0.0097 | +0.0375 | +33.9 % |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.7133 / 0.8724 / **0.7849** | **+0.0135** | +0.0052 | +0.0252 | +31.7 % |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.5348 / 0.3984 / **0.4566** | **+0.0196** | +0.0276 | +0.0146 | +16.6 % |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.6856 / 0.7827 / **0.7309** | **+0.0133** | +0.0095 | +0.0183 | +41.3 % |

Consequence: **1.0 is where the gain first clears the +0.003 gate on every scene, not a floor** —
the curve is monotone all the way from 1.1 (a small positive, mean +0.0022) down through 0.6 (mean
+0.0191, table above), with no scene ever regressing; 0.5 is still queued (its Meetingroom run has
not landed), and the mesh pairing (S7) covers 0.9 plus the best deeper dose.

### S4 — C3 + C4: rescue-strength sweep

**Prior-weight dose** (`fFusePriorWeight`, same frozen `runConfAdj` dmaps)

| fFusePriorWeight | Barn ΔF1 | Ignatius ΔF1 | Meetingroom ΔF1 | Truck ΔF1 | mean ΔF1 | ΔP range | points range |
|---|---|---|---|---|---|---|---|
| 0 | −0.0280 | −0.0302 | −0.0538 | −0.0089 | **−0.0302** | +1.2…+4.2 pp | −38…−48 % |
| 2 | −0.0121 | −0.0147 | −0.0271 | −0.0036 | **−0.0144** | +0.6…+2.2 pp | −23…−30 % |
| 3 (default) | 0 | 0 | 0 | 0 | 0 (base) | — | — |
| 4 | +0.0086 | +0.0082 | +0.0272 | −0.0012 | **+0.0107** | −1.3…−2.7 pp | +37…60 % |
| 6 | −0.0130 | −0.0223 | +0.0399 | −0.0268 | **−0.0055** | −7.7…−10.5 pp | +144…342 % |

Full P/R/F1 for weight 4:

| scene | base P / R / F1 | 4 P / R / F1 | ΔF1 | ΔP | ΔR | points |
|---|---|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5592 / 0.7765 / **0.6502** | **+0.0086** | −0.0154 | +0.0502 | +41.5 % |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.6955 / 0.8870 / **0.7796** | **+0.0082** | −0.0126 | +0.0398 | +50.7 % |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.4800 / 0.4494 / **0.4642** | **+0.0272** | −0.0272 | +0.0656 | +60.1 % |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.6554 / 0.7898 / **0.7164** | **−0.0012** | −0.0207 | +0.0254 | +36.9 % |

**`nMinPixelsFuse` dose** (same dmaps)

| nMinPixelsFuse | Barn ΔF1 | Ignatius ΔF1 | Meetingroom ΔF1 | Truck ΔF1 | mean ΔF1 |
|---|---|---|---|---|---|
| 5 (default) | 0 | 0 | 0 | 0 | 0 (base) |
| 4 | +0.0050 | +0.0057 | +0.0271 | −0.0052 | **+0.0082** |
| 3 | −0.0155 | −0.0234 | +0.0380 | −0.0294 | **−0.0076** |

*Replayed clean 2026-08-27, now final*: Truck minpix4 (was 88 % hit rate) and Ignatius minpix3
(was 62 %) had run starved; clean, Truck minpix4 is −0.0052 (still fails the no-scene-below
−0.003 clause on its own) and Ignatius minpix3 is −0.0234 (a smaller regression than the starved
−0.0298 reading). Verdicts unchanged: minpix3 rejected, minpix4 dominated by prior weight 4 —
final.

**C3 vs C4 dominance.** Prior4 beats minpix4 on every scene in F1 *and* P while adding fewer
points (+37…60 % vs +40…79 %), and prior6 beats minpix3 the same way at ×2.4…4.4 vs ×2.5…4.6
points — a lower `nMinPixelsFuse` is a strictly worse way to buy the same recall, so **C4 is
closed**.

**Meetingroom** is the only scene that keeps gaining at every dose (+0.027 at prior 4, +0.040 at
prior 6, +0.038 at minpix 3) because it is recall-starved (R 0.38 at base); every other scene turns
negative past weight 4.

**Gate reading (`fFusePriorWeight` 4).** Mean passes (+0.0107), no-regression passes (Truck
−0.0012 is inside the 0.003 clause), the 1 pp precision clause fails on every scene (P
−1.3…−2.7 pp) and is only excused by the net F win, and points +37…60 % make the
`--min-point-distance 2.0` mesh pairing mandatory — **conditional pass, decided in S7**.

**C6 — `minConfidence`** (`fNCCThresholdKeep`, same dmaps)

`fNCCThresholdKeep 0.95` (minConfidence 0.05):

| scene | base P / R / F1 | 0.05 P / R / F1 | ΔF1 | ΔP | ΔR | points |
|---|---|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5718 / 0.7278 / **0.6404** | **−0.0012** | −0.0028 | +0.0015 | +1.1 % |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.7069 / 0.8476 / **0.7709** | **−0.0005** | −0.0012 | +0.0004 | +1.1 % |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.5003 / 0.3887 / **0.4375** | **+0.0005** | −0.0069 | +0.0049 | +2.8 % |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.6736 / 0.7653 / **0.7166** | **−0.0010** | −0.0025 | +0.0009 | +0.9 % |

Mean **−0.0005**.

`fNCCThresholdKeep 0.85` (minConfidence 0.15):

| scene | base P / R / F1 | 0.15 P / R / F1 | ΔF1 | ΔP | ΔR | points |
|---|---|---|---|---|---|---|
| Barn | 0.5746 / 0.7263 / 0.6416 | 0.5772 / 0.7241 / **0.6423** | **+0.0007** | +0.0026 | −0.0022 | −1.5 % |
| Ignatius | 0.7081 / 0.8472 / 0.7714 | 0.7097 / 0.8462 / **0.7719** | **+0.0005** | +0.0016 | −0.0010 | −1.4 % |
| Meetingroom | 0.5072 / 0.3838 / 0.4370 | 0.5145 / 0.3773 / **0.4353** | **−0.0017** | +0.0073 | −0.0065 | −3.5 % |
| Truck | 0.6761 / 0.7644 / 0.7176 | 0.6788 / 0.7633 / **0.7186** | **+0.0010** | +0.0027 | −0.0011 | −1.2 % |

Mean **+0.0001**.

Both sit far inside the ±0.003 gate margin on every scene (max |ΔF1| 0.0017, Meetingroom at 0.15): **C6 is
closed as inert, not as negative**. *Replayed clean 2026-08-27*: Ignatius and Meetingroom conf005
(58 % / 53 % hit rate) and conf015 (67 % / 86 %) had run starved on the original sweep — exactly
the cells the old channel explanation below was built on — and the starved cells exaggerated the
effect: Ignatius conf005 moved from a starved −0.0143 (−6.7 % points) to a clean −0.0005
(+1.1 % points), and conf015 collapses the same way (−0.0063 starved → +0.0005 clean). The old
low-confidence-seed narrative built from that starved Ignatius reading does not hold on the clean
data and is retracted. Barn and Truck, whose conf005/conf015 rows were clean throughout, never
supported it either (|ΔF1| ≤ 0.0012 on every dose). `fNCCThresholdKeep` at its default 0.9
(minConfidence 0.1) is indistinguishable from ±0.05 either side on every scene.

### S5 — C5: free-space-violation arms

**(i) `nFuseViolationMax` dose**

| nFuseViolationMax | Barn ΔF1 | Ignatius ΔF1 | Meetingroom ΔF1 | Truck ΔF1 | mean ΔF1 |
|---|---|---|---|---|---|
| −1 | −0.0002 | −0.0011 | +0.0003 | −0.0010 | **−0.0005** |
| 0 (default) | 0 | 0 | 0 | 0 | 0 (base) |
| 1 | −0.0001 | −0.0002 | +0.0003 | +0.0004 | **+0.0001** |
| 2 | −0.0005 | −0.0003 | +0.0003 | −0.0009 | **−0.0003** |

(Points move ≤ 1.5 % on every scene at −1 — none move enough to matter.)

The guard's dose is inert on every scene (mean −0.0005, worst Truck −0.0010, all inside the
±0.003 gate margin) — it
only ever touches 0.12–0.25 % of valid depths, and there is no evidence to move it off the
default, so **keep 0**.

*Replayed clean 2026-08-27*: Truck violm1 (90 % hit rate, 1 warning on the original sweep) is now
clean at −0.0010, not the −0.0035 the starved run had suggested — the guard is worth ≤ 0.001 F1,
i.e. inert by the gate's standard.

**(ii)** applying the guard to all clusters, not just rescued ones (`Fuse Violation Max All`, S10
binary, tag `fusion-s10`):

| Fuse Violation Max All | Barn ΔF1 | Ignatius ΔF1 | Meetingroom ΔF1 | Truck ΔF1 | mean ΔF1 | points |
|---|---|---|---|---|---|---|
| 0 | −0.0004 | +0.0004 | −0.0021 | +0.0016 | **−0.0001** | −1.0…−2.1 % |
| 1 | +0.0001 | +0.0002 | −0.0006 | +0.0006 | **+0.0001** | −0.3…−0.6 % |
| 2 | +0.0001 | +0.0001 | −0.0002 | +0.0002 | **+0.0001** | −0.1…−0.2 % |

Applying the free-space guard to non-rescued points as well is inert: every scene sits within the
0.0006 floor except Meetingroom at 0 (−0.0021), and points move only −0.1…−2.1 % across
the three settings. The clusters this extension would remove barely exist — keep
`Fuse Violation Max All` at −1 (unguarded). Resolves decision 4 (ii); full arm detail at § 4 S10.

### S6 — C2: CPU parity measurement

*Run 2026-08-26*, tag `fusion-s10`, arm `cpuadj`: `--gpu-device cpu --postprocess-dmaps 8
--geometric-iters 0` on the same `runConfAdjRaw` dmaps used by S2 (iii)/(iv) — estimation is
skipped (the dmaps already exist), so only the standalone confidence-recalibration sweep runs, on
CPU, rewriting the dmaps in place, then fusion.

| scene | F1 | ΔF1 vs base | ΔF1 vs `rawbase` | ΔP vs `rawbase` (pp) | points | admitted % |
|---|---|---|---|---|---|---|
| Barn | 0.6408 | −0.0008 | **+0.0021** | +0.39 | −1.1 % | 49.76 % |
| Ignatius | 0.7716 | +0.0002 | **+0.0013** | +0.21 | −0.8 % | 48.03 % |
| Meetingroom | 0.4366 | −0.0004 | **+0.0008** | +0.87 | −2.4 % | 30.42 % |
| Truck | 0.7174 | −0.0002 | **+0.0013** | +0.26 | −0.7 % | 56.07 % |

Mean ΔF1 vs base **−0.0003**; vs `rawbase` **+0.0014**. Against `base10` the standalone CPU sweep
lands within −0.0008…+0.0002 on every scene (points +0.3…1.1 %); against its own raw dmaps
(confound-free) it is a clean, all-precision win of +0.0008…+0.0021 (points −0.7…−2.4 %).

**Cost.** The standalone sweep itself takes 4–5 s of wall per scene on this machine (summed thread
time: Ignatius 263 maps / 87 s, Truck 251 / 95 s, Barn 410 / 118 s, Meetingroom 371 / 105 s — about
280–290 ms per map, each logged "adjusted using 8 other images"). It caches every depth map in
memory for the duration ("Adjust-confidence phase: caching all N depth-maps in memory"), estimated
peak 4.0–6.5 GB; whole-run `PeakWorkingSetSize` is 4.9–7.8 GB.

**Mechanism.** `--postprocess-dmaps` default `4` runs the adjust as the *integrated* epilogue of
the last geometric iteration when the dmaps were estimated on CUDA (`AdjustConfidenceCUDA`,
resident neighbor buffers reused from estimation, "costs almost nothing" per its own comment); `8`
forces the *standalone* sweep (`DepthMapsData::AdjustConfidence(DepthData&, idxNeighbors)`), which
is CPU-only — there is no standalone GPU variant, so a same-input GPU control does not exist. A
`CONF_ADJUSTED` flag on each dmap guards against a double adjust. On Ignatius the standalone sweep
reclassifies which pixels are admitted without changing how many: the raw dmaps drop 0.00 % of
valid depths to low-confidence and 50.57 % to min-pixels; after the CPU adjust, 26.44 % low-
confidence and 25.34 % min-pixels — a quarter of the valid depths change bucket — yet the admitted
share barely moves (48.61 % → 48.03 %).

**Confound.** S6 compares the standalone CPU sweep on an independently estimated (unseeded)
raw-dmap set against the integrated GPU epilogue baked into `base10`'s own estimation run — the
−0.0008…+0.0002-vs-`base10` gap therefore contains an estimation-noise term of unknown size. The
confound-free statement is `cpuadj` vs `rawbase` (+0.0008…+0.0021, all P) against the GPU
epilogue's own apparent value, `base10` − `rawbase` (+0.0011…+0.0029, all P, from S2 (iii)) — two
independent estimates of the same effect, within 0.0008 of each other on every scene.

**Verdict (decision 2).** `--postprocess-dmaps 8` gets a CPU user the GPU pipeline's confidence
quality within −0.0008…+0.0002 cloud F1 on every scene for 4–5 s of wall and a dmap-sized memory
peak — not proven exact parity (the confound above), but the two independent estimates of the
adjust's value agree within 0.0008.

### S7 — Cost pairing

*Superseded 2026-08-27*: scored without the mesh-visibility clean now mandatory (§ 3 *Mesh
visibility*); re-measurement under tag `fusion-vis` pending, see that subsection.

**C3 pairing (`fFusePriorWeight` 4)** — measured 2026-08-25, `run_mesh.py --cloud-name prior4
--variants baseline,mpd2 --score-raw`, plus the base cloud at `--min-point-distance 2.0`; wall
times not quotable (the rows ran beside the fusion sweep and pagefile growth), memory is
`peak_ws_mb`. Columns pair the cloud-stage lever (`fFusePriorWeight`) with its mesh-stage outcome
(`--min-point-distance`): each cell chains base 1.5 → prior4 1.5 → prior4 2.0.

| scene | raw+gated F1 | cleaned F1 | Delaunay verts | `peak_ws_mb` |
|---|---|---|---|---|
| Barn | 0.6225 → 0.6201 → 0.6180 | 0.6257 → 0.6213 → 0.6213 | 8,415,872 → 11,116,603 (×1.32) → 8,612,869 (×1.02) | 16,370 → 18,315 (×1.12) → 15,500 (×0.95) |
| Ignatius | 0.7607 → 0.7655 → 0.7610 | 0.7528 → 0.7571 → 0.7548 | 4,574,777 → 6,260,488 (×1.37) → 4,614,061 (×1.01) | 8,883 → 11,702 (×1.32) → 9,026 (×1.02) |
| Meetingroom | 0.4376 → 0.4371 → 0.4369 | 0.4398 → 0.4388 → 0.4394 | 5,942,233 → 8,612,660 (×1.45) → 6,823,526 (×1.15) | 11,602 → 16,290 (×1.40) → 13,230 (×1.14) |
| Truck | 0.6544 → 0.6455 → 0.6381 | 0.6536 → 0.6439 → 0.6378 | 5,112,786 → 6,536,027 (×1.28) → 4,916,732 (×0.96) | 10,011 → 12,485 (×1.25) → 9,346 (×0.93) |

*Superseded 2026-08-27*: scored without the mesh-visibility clean now mandatory (§ 3 *Mesh
visibility*); re-measurement under tag `fusion-vis` pending, see that subsection.

**The base cloud itself at 2.0** (the cost of decision 5):

| scene | raw F1 base 1.5 → base 2.0 | Δ | verts × | peak × |
|---|---|---|---|---|
| Barn | 0.6225 → 0.6206 | −0.0019 | ×0.80 | ×0.74 |
| Ignatius | 0.7607 → 0.7560 | −0.0047 | ×0.75 | ×0.76 |
| Meetingroom | 0.4376 → 0.4378 | +0.0002 | ×0.82 | ×0.80 |
| Truck | 0.6544 → 0.6470 | −0.0074 | ×0.77 | ×0.77 |

Mean **−0.0035**. `--min-point-distance 2.0` buys −23 % memory for −0.0035 mean raw F1, Truck
−0.0074.

**Verdict.** Prior weight 4's +0.0107 cloud gain becomes **−0.0018 mean raw-mesh F1** at 1.5
(−0.0024 / +0.0048 / −0.0005 / −0.0089) with peak memory +12…40 % (three scenes over the +15 %
clause) and verts +28…45 %; at equal cost (2.0) it is **−0.0053 mean** (−0.0045 / +0.0003 /
−0.0007 / −0.0163). Truck fails the raw-F1 clause at both densities. **C3 rejected downstream;
`fFusePriorWeight` stays 3.** Only Ignatius profits (+0.0048). Note that the Barn prior4 cloud in
these rows is the mildly starved re-fuse (19,937,081 points, § 3 *Memory*); the verdict does not
rest on Barn.

### S8 — Consolidation

*Not yet run.*

### S9 — Structural-loss instrumentation

*Done 2026-08-25* (commit `0d441378`; measurement behind the config-only switch
`Fuse Recoverable Stats=1`, off by default). The instrumented binary is **byte-identical** to the
baseline on Ignatius and Truck with the switch on and off (`cmp` of the fused `.mvs`); with it on,
fusion costs +30 % wall (Ignatius 66 s vs 51 s) — never use it for a timing row. Both accounting
residuals read 0 on every scene.

The observation is exact for every cluster the keep-rule drops: such a cluster never exceeded the
minimums, so all of its probes were issued from below them, and the counters are only evaluated
there (keepable clusters keep their hot early-outs). The hypothetical applies the keep-rule in
full — real-support thresholds, or virtual-support thresholds plus the free-space-violation guard
— so a violation-dropped cluster counts as recoverable only when the extra *real* support lifts it
out of the guard's reach. Percentages below are of the scene's valid depths.

| scene | dropped clusters (pixels) | corroboration | half-weight corroboration | released-pixel reuse (C8) | 4-neighbor re-probe (C12) | corroboration + re-probe |
|---|---|---|---|---|---|---|
| Barn | 40.3 M (51.3 M, 25.5 %) | 20.3 M (29.5 M, **14.7 %**) | 16.0 M (23.8 M, 11.9 %) | 3.35 M (5.9 M, 3.0 %) | 0.29 M (0.61 M, 0.3 %) | 20.4 M (29.6 M) |
| Ignatius | 26.2 M (33.9 M, 25.6 %) | 14.4 M (21.1 M, **16.0 %**) | 12.1 M (18.0 M, 13.6 %) | 2.83 M (4.9 M, 3.7 %) | 0.25 M (0.54 M, 0.4 %) | 14.5 M (21.3 M) |
| Meetingroom | 49.5 M (61.7 M, 35.2 %) | 19.0 M (28.6 M, **16.3 %**) | 12.8 M (20.4 M, 11.6 %) | 4.11 M (7.1 M, 4.0 %) | 0.29 M (0.65 M, 0.4 %) | 19.2 M (28.9 M) |
| Truck | 20.0 M (25.8 M, 20.4 %) | 10.0 M (15.0 M, **11.8 %**) | 8.0 M (12.3 M, 9.8 %) | 1.57 M (2.9 M, 2.3 %) | 0.13 M (0.28 M, 0.2 %) | 10.0 M (15.1 M) |

"X (Y, Z %)" = clusters the keep-rule would keep under that hypothesis (their real pixels, share of
valid depths). Supporting rates: of the already-fused probes examined, 55 / 66 / 40 / 63 %
(Barn / Ignatius / Meetingroom / Truck) pass the three join predicates against the current
reference; of those, 18 / 27 / 16 / 22 % land on pixels of dropped clusters. Only 2.6–4.2 % of the
no-depth probes and 5.6–11.7 % of the depth-disagreeing probes have an agreeing free 4-neighbor.
`min-views` never binds alone on any scene (0 drops); the violation guard removes 0.12–0.25 %.

**Verdict for S10.** Corroboration (C9) is the only structural channel above the plan's "a few
percent of valid depths" bar — 12–16 % on every scene, with the precision risk the plan names
(a corroboration-rescued cluster is often a near-duplicate of its neighbor). The 4-neighbor
re-probe (C12) is closed by this measurement: 0.2–0.4 %, an order of magnitude below the bar, not
built. Release-on-drop (C8) alone is 2–4 % — borderline, but it is cheap and already built as a
config arm, so it is measured rather than argued. C10 is deferred to C9's result.

### S10 — Build the structural winners

*Run 2026-08-25/26*, tag `fusion-s10`, binary `bench/bin_fusion_s10` (the config-gated arms of
`bench/campaign_s10.sh`, run on the frozen `runConfAdj` dmaps). The `base10` anchor arm (no config)
reproduces the frozen base exactly on all four scenes (Barn 0.6416 / Ignatius 0.7714 / Meetingroom
0.4370 / Truck 0.7176), so this binary is behaviourally identical to the campaign's other binaries
with every switch off. All eleven arms (44 rows, `corrob025` added 2026-08-27)
are clean: `cache_warnings = 0`, 91–92 % dmap-cache hit rate on every arm (§ 3 *Memory*). Fusion
wall time in these rows is **not** a solo measurement — the machine
ran other tooling concurrently, as elsewhere in § 4 — so no wall conclusion is drawn from any of
them here; that caveat is stated once for the whole slice rather than per arm.

**C8 — release dropped pixels back to the pool** (`release`, `Fuse Release Dropped=1`)

| scene | ΔF1 | ΔP (pp) | ΔR (pp) | points | admitted % |
|---|---|---|---|---|---|
| Barn | +0.0020 | −1.6 | +3.3 | +21.4 % | 57.9 % |
| Ignatius | +0.0020 | −1.3 | +2.5 | +25.7 % | 57.4 % |
| Meetingroom | +0.0169 | −3.3 | +5.1 | +43.3 % | 40.8 % |
| Truck | −0.0039 | −1.9 | +1.6 | +17.4 % | 63.2 % |

Mean **+0.0043** but Truck −0.0039 fails the no-regression clause. Precision falls 1.3–3.3 pp on
every scene, recall rises 1.6–5.1 pp, points grow +17…43 %, and 12.6–27.6 M pixels are reclaimed per
scene by returning a dropped cluster's pixels to the pool for a later seed to reclaim. A pure recall
lever — not recommended as a default; the `--min-point-distance 2.0` mesh pairing (§ 3 *Cost
pairing*) is not run because the cloud gate already fails.

**C11 — confidence-ordered seeding** (`seedconf`, `Fuse Seed By Confidence=1`)

| scene | ΔF1 | ΔP (pp) | ΔR (pp) | points | admitted % |
|---|---|---|---|---|---|
| Barn | −0.0038 | −0.2 | −0.7 | −4.4 % | 49.5 % |
| Ignatius | −0.0015 | −0.0 | −0.4 | −4.1 % | 47.7 % |
| Meetingroom | −0.0066 | −0.6 | −0.7 | −3.9 % | 30.0 % |
| Truck | −0.0018 | −0.1 | −0.2 | −3.8 % | 55.8 % |

Mean **−0.0034**, Meetingroom worst at −0.0066. Points fall ~4 % on every scene and precision and
recall move down together — ordering seed pixels by descending confidence instead of raster order
does not buy precision at all (ΔP only −0.0001…−0.0056, essentially flat). Rejected.

**C8 + C11 combined** (`relseed`, both configs)

| scene | ΔF1 | ΔP (pp) | ΔR (pp) | points | admitted % |
|---|---|---|---|---|---|
| Barn | −0.0007 | −1.8 | +3.0 | +19.1 % | 57.9 % |
| Ignatius | +0.0010 | −1.3 | +2.2 | +21.8 % | 57.2 % |
| Meetingroom | +0.0127 | −3.6 | +4.7 | +39.6 % | 40.7 % |
| Truck | −0.0054 | −2.0 | +1.5 | +14.0 % | 63.1 % |

Mean **+0.0019**, Truck −0.0054 — worse than release alone on every scene (Barn +0.0020 → −0.0007,
Ignatius +0.0020 → +0.0010, Meetingroom +0.0169 → +0.0127, Truck −0.0039 → −0.0054): seeding
subtracts from release's gain rather than adding to it. Rejected.

**S5 (ii) cross-ref — free-space-violation guard applied to all clusters** (`violall0/1/2`)

Same S10 binary and dmaps, `Fuse Violation Max All=0/1/2` — full per-scene table and verdict at
§ 4 S5 (ii): inert, mean −0.0001 / +0.0001 / +0.0001, points −0.1…−2.1 % across the three settings.
One note specific to these three arms: their fusion wall reads −25…−35 % against `base10`, but
`base10` itself ran on 2026-08-25 under the same shared-machine load as the rest of this table, so
this is not a solo timing and no wall conclusion is drawn from it; deferred to the S7 solo
re-timing.

**C9 — corroboration support, dose curve** (`corrob1`, `corrob05`, `corrob05all`, `corrob025`; the
last run clean 2026-08-27)

| corroboration weight | Barn ΔF1 | Ignatius ΔF1 | Meetingroom ΔF1 | Truck ΔF1 | mean ΔF1 | ΔP range (pp) | points range | admitted % range |
|---|---|---|---|---|---|---|---|---|
| 1.0 | +0.0053 | −0.0173 | +0.0550 | −0.0145 | **+0.0071** | −4.3…−7.2 | +104…212 % | 46…68 % |
| 0.5 | +0.0168 | −0.0089 | +0.0561 | −0.0056 | **+0.0146** | −1.0…−5.6 | +85…143 % | 41…66 % |
| 0.5 + release | +0.0127 | −0.0101 | +0.0543 | −0.0085 | **+0.0121** | −2.7…−5.9 | +88…161 % | 46…68 % |
| 0.25 | +0.0240 | +0.0038 | +0.0429 | +0.0020 | **+0.0182** | −0.7…−3.2 (MR +0.5) | +63…97 % | 37…63 % |

The dose curve is **reversed** from what a naive prior would expect: the smaller the corroboration
weight, the better the result — weight 1.0 (+0.0071 mean) → 0.5 (+0.0146) → **0.25 (+0.0182)** —
and 0.25 (`corrob025`, tag `fusion-s10`, run clean 2026-08-27) is the first C9 arm positive on
*all four* scenes (Barn +0.0240, Ignatius +0.0038, Meetingroom +0.0429, Truck +0.0020), i.e. the
first dose that clears the F1 half of the acceptance gate (§ 3: mean ≥ +0.003 beyond noise, no
scene < −0.003). Adding release-on-drop to weight 0.5 still does not help — `0.5 + release` trails
`0.5` alone on all four scenes (+0.0168→+0.0127, −0.0089→−0.0101, +0.0561→+0.0543,
−0.0056→−0.0085), the same release-subtracts pattern seen in C8+C11 above.

The plan's named precision risk holds at 1.0 and 0.5 — the F1 gain there is entirely recall
(Barn/Meetingroom R +10…14 pp across both doses), and Ignatius/Truck regress at every dose (their
precision −6.3…−7.2 pp at weight 1.0, −4.2…−5.6 pp at 0.5), so C9 fails the gate at 1 and 0.5.
Meetingroom's +0.055 at weight 1.0 is the density outlier (base R 0.38, § 4 S0) and, per this
document's standing rule, must not be judged alone. At 0.25 the picture changes: Ignatius and
Truck still lose precision (−3.17 pp / −2.25 pp) and Barn's loss shrinks to −0.70 pp, but
Meetingroom's own precision turns *positive* (+0.47 pp) — the precision clause is covered by its
own net-F-win exception (§ 3 *Acceptance gate*: "unless the complementary gain nets a clear F
win"). Points still grow +63…97 % at 0.25, which by § 3 *Cost pairing* makes the paired
`--min-point-distance 2.0` mesh row mandatory before any default decision — not yet run, and now
scheduled under the mesh-visibility protocol (§ 3 *Mesh visibility*, 2026-08-27) rather
than the pre-visibility mesh scoring used elsewhere in this document.

**Mechanism caveat — why the dose curve cannot converge back to base.** The corroboration weight
scales only the pixel channel of the keep-rule (`corroborationSupport = weight × clusterCorroboration`,
spent against `nMinPixelsFuse`, `libs/MVS/SceneDensify.cpp` ~3196–3225); the view channel is an
*unweighted* set union of the observing views with the corroborating views, spent against
`nMinViewsFuse`. Any weight > 0 therefore enables the full view-side relaxation, and the 1 / 0.5 /
0.25 dose only ever varies the pixel side — there is no weight at which the view channel returns to
its un-corroborated behaviour. A cluster kept only because of corroboration counts as rescued and is
still subject to the strict `nFuseViolationMax` (0) guard; the fused point itself is built from real
members only. A variant that spends only one of the two channels — pixel-only or view-only
corroboration — is an obvious follow-up this campaign left unbuilt, for the maintainer. The
reversed dose curve above is exactly what this predicts: shrinking the weight toward 0 leaves the
unweighted view-channel relaxation untouched while starving the pixel channel, i.e. it approaches
"view-channel-only corroboration" — and F1 keeps climbing as the weight shrinks, so the view
channel, not the pixel channel, is carrying C9's gain. Queued next: doses 0.1 and 0.01 (to
approximate the view-only limit), and the mandatory mesh pairing (baseline +
`--min-point-distance 2.0`) for 0.25, to be run under the mesh-visibility protocol (§ 3 *Mesh
visibility*, 2026-08-27).

**Verdict for § 5.** C8 (release-on-drop) and its C11 combination fail the no-regression clause on
Truck; C11 alone is negative on all four scenes; the S5 (ii) violation-guard extension is inert —
none of those three clears the cloud gate. C9 (corroboration) is the only mechanism with a real
signal: at weight 1.0 and 0.5 it trades Ignatius/Truck precision for Barn/Meetingroom recall and
fails the gate, but at **weight 0.25 it is the first S10 arm to clear the cloud-F1 half of the
acceptance gate** (mean +0.0182, no scene below +0.0020). It remains a completeness lever, not a
default, because its point-count growth (now +63…212 % across the dose range) has not been paired
with the mandatory mesh row (§ 3 *Cost pairing*) — that pairing is scheduled under the
mesh-visibility protocol above. Nothing here changes an `OPTDENSE` default yet; see § 5
item 7.

---

## 5. Decisions reserved to the maintainer

None of these are taken by this campaign; each needs an explicit decision on the evidence the
corresponding slice produces.

1. **`fDepthReprojectionErrorThreshold` 1.2 → 1.0** (S3) — a validated win that was written,
   documented and then never merged. Also: decide whether `dc32ab8` and `833e93c` were
   deliberately dropped or lost, and whether `dense-cell-neighbors` holds anything else in the
   same state — *pending*. S3 dose curve (fully clean 2026-08-27) is monotone from 1.1
   (+0.0022 mean, a small positive) down through 0.6 (+0.0191 mean) with no scene ever
   regressing; 1.0 is where the gain first clears the +0.003 gate, 0.9 gains +0.0124 mean at
   +12…28 % points; 0.5 is still queued (Meetingroom pending) and the mesh pairing for the
   deeper doses is pending.
2. **`nOptimize` 4 → 8** (S6) — the standalone CPU sweep matches the integrated GPU epilogue within
   −0.0008…+0.0002 cloud F1 on every scene for 4–5 s of wall and a 4.9–7.8 GB peak working set; two
   independent estimates of the adjust's own value (`base10 − rawbase` and `cpuadj − rawbase`)
   agree within 0.0008 on every scene, but the headline number is still confounded by an
   independent raw-dmap estimation run (§ 4 S6) — evidence for parity, not proof of it.
3. **`fFusePriorWeight`** away from `3.0`, and/or **`nMinPixelsFuse`** away from `5` (S4) —
   S4 + S7: `fFusePriorWeight` 4 passes the cloud gate (+0.0107) but the mesh rejects it (Truck raw
   F1 −0.0089, peak memory +12…40 %); `nMinPixelsFuse` is dominated by the prior weight at every
   dose — **no change recommended** (pending only the starved-row re-runs).
4. **`nFuseViolationMax`** away from `0`, and whether the FSV guard should apply to non-rescued
   points (S5) — S5 (i): inert on every scene (≤ 0.001 either direction, replayed
   clean 2026-08-27), no evidence to move off the default; keep 0; (ii): inert — applying the
   guard to non-rescued clusters too buys nothing beyond the noise floor; keep
   `Fuse Violation Max All` at −1 (unguarded).
5. **`ReconstructMesh --min-point-distance` 1.5 → 2.0** (S7) — mesh-side; also resolves the
   CLI-vs-library (1.5 vs 2) mismatch recorded during the mesh effort's audit (git history) —
   *pending*. S7 measured the base cloud at 2.0: −0.0035 mean raw F1 (Truck −0.0074) for −23 %
   memory — a memory/quality trade, not a free win.
6. **Re-freezing the canonical bench clouds** (S0b) — this rewrites the meaning of every
   historical `bench/out_mesh/results.csv` row and every `docs/design/DelaunayMeshReconstruction.md`
   table measured against them — *pending*.
7. **Any structural change to the fusion algorithm** (S10: pixel release on drop,
   confidence-ordered seeding, corroboration support, neighborhood re-probe) — unlike C1–C6 these
   are not one-line reverts of a constant; each changes which clusters form, so each lands only on
   its own S9-instrumented evidence plus a clean acceptance-gate pass, one commit per mechanism —
   S10 measured all four built mechanisms (release-on-drop, confidence-ordered seeding, both
   combined, corroboration at weight 1 / 0.5 / 0.25). Only corroboration at weight 0.25 passes the
   cloud-F1 gate; its mandatory mesh pairing is pending, scheduled under the mesh-visibility
   protocol (§ 3). Details at § 4 S10.

---

## 6. Open questions

1. **How much of the cloud-F1 gain the mesh stage absorbs is unknown until re-measured.** The
   original motivation for this question was a set of mesh-F1 deltas scored through the
   pre-2026-08-21 `Mesh::Clean` smoother bug (`DelaunayMeshReconstruction.md` § 3); those numbers
   are not admissible evidence, so whether fusion-side work has a low ceiling on end-to-end
   quality is open again, not settled. Should fusion candidates be gated on cloud F1 at all, or on
   mesh F1 directly (6–10× slower per arm)? This plan gates on cloud and reports mesh; the choice
   is revisitable.
2. **Meetingroom's ×6.1 mesh wall for ×3.2 points is unexplained** and superlinear in every
   sub-stage (`tri` ×5.4, `weight` ×8.2, `cut` ×5.9). Needs a profile before any completeness push
   is defaulted.
3. **Precision falls on every scene in family B** (−0.0089 / −0.0755 / −0.0131). Is that the
   prior-rescue admitting outliers — in which case `fFusePriorWeight 2` is the cloud-final answer
   its own doc string already suggests — or the unavoidable P/R trade of denser sampling? S2 + S4
   answer; until then it is the main reason not to push completeness harder.
   **Answered (S2 (iv)).** It is the rescue's own P/R trade, not the recalibrated confidence
   admitting outliers: turning the rescue off restores precision by +0.8…+2.1 pp at a cost of
   −4.5…−8.8 pp recall on both the raw and the adjusted confidence lineage (mean ΔF1 −0.0295 raw
   vs. −0.0302 adjusted, within 0.0034 scene by scene) — and it is a trade nobody should unwind,
   since F1 loses 0.010–0.052 (§ 4 S2).
4. **Estimation-side residual.** `ec13cab7` (parallel CUDA PatchMatch) and `69b25a37` (dmap codec
   / image cache / view-locality order) landed between the two builds. Presented as performance
   work; not verified numerically neutral. S2 arm (iv) is the falsification test.
   **Tested, falsification fails.** Arm (iv) (`rawprior0`) reverts the two gates § 2 says #1292
   moved — no confidence recalibration, no prior rescue — at the unchanged 1.2 reprojection
   threshold and the same bench flags family A was densified with (`--resolution-level 1
   --number-views 12 --estimate-roi 0 --crop-to-roi 0 --tower-mode 0`, verified in the Ignatius
   and Truck `runMetashape` logs; builds `9a007b5`, June, and `e884035`, July), run against
   today's estimation code. Compared against § 1's family-A cloud F1 per scene:

   | scene | family-A F1 (§ 1) | arm (iv) F1 | residual |
   |---|---|---|---|
   | Barn | 0.5988 | 0.6110 | **+0.0122** |
   | Ignatius | 0.7381 | 0.7402 | +0.0021 |
   | Meetingroom | 0.3225 | 0.3854 | **+0.0629** |
   | Truck | 0.7060 | 0.7062 | +0.0002 |

   Ignatius and Truck fall inside the 0.003 tolerance; Barn (+0.0122) and Meetingroom (+0.0629, an
   order of magnitude over) do not. The plan's own check — arm (iv) "should land near the family-A
   point counts" — splits the same way: Ignatius 5.44 M vs 5.21 M and Truck 5.99 M vs 5.99 M
   match, Barn 8.67 M vs 7.70 M (+13 %) and Meetingroom 4.89 M vs 3.11 M (+57 %) do not. The
   falsification fails on two of four scenes, so the
   estimation side is **not** shown to be numerically neutral. This measurement alone cannot
   attribute the residual to a specific commit — any fusion-side change between the family-A
   builds and today other than the two reverted gates is in it too — and it is entangled with
   PatchMatch's own
   run-to-run stochasticity (§ 3: unseeded, up to 14 % point-count spread on Meetingroom
   specifically, across June runs on the pre-#1292 estimator) — a repeated raw-chain run would be
   needed to separate the two. **Question stays open.**
5. **`nMaxViewsFuse = 32` vs `--number-views 12`.** Fusion caps its neighbor set at
   `nMaxViewsFuse` while estimation used 12; family-A logs imply ~18.7 views per cluster, i.e.
   fusion reaches past the estimation neighborhood through the flood-fill. Whether that is
   intended, and whether it interacts with the rescue rule, is unexamined.
6. **Is `nDepths` (the "%u depths" in the summary line) the number anyone wants?** It sums
   `fusedViews.size()` over all clusters including dropped ones, so the `points (%d%%)` figure it
   feeds is not a fusion yield. The `Fusion pixel accounting` line supersedes it; consider
   retiring or relabelling the older summary.
7. **Fusion silently degrades under memory pressure**: `DenseFuseDepthMaps` skips neighbors it
   could not cache and only warns. Should cache starvation be an error, should the loop block
   until the map can be loaded, or should the cache stream maps instead of skipping? Until
   decided, any fusion benchmark row is only valid with `cache_warnings = 0`.
