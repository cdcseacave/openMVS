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
gated on cloud F1, with mesh F1 reported (raw+gated surface, cleaned mesh alongside) as mandatory
downstream evidence, never as a substitute gate.

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
  medians for anything quoting wall time or memory.
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

*Not yet run.*

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

**Recommendation** (decision reserved, § 5 item 1): adopt 1.0 as the default — validated twice,
on two confidence lineages, Pareto on every scene, neutral for the mesh. Implementation is the
two literals of `dc32ab8` (`libs/MVS/DepthMap.cpp`, `DensifyPointCloud.cpp`) plus its
`docs/fusion_reprojection_threshold.md`.

### S4 — C3 + C4: rescue-strength sweep

*Not yet run.*

### S5 — C5: free-space-violation arms

*Not yet run.*

### S6 — C2: CPU parity measurement

*Not yet run.*

### S7 — Cost pairing

*Not yet run.*

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

*Not yet run.*

---

## 5. Decisions reserved to the maintainer

None of these are taken by this campaign; each needs an explicit decision on the evidence the
corresponding slice produces.

1. **`fDepthReprojectionErrorThreshold` 1.2 → 1.0** (S3) — a validated win that was written,
   documented and then never merged. Also: decide whether `dc32ab8` and `833e93c` were
   deliberately dropped or lost, and whether `dense-cell-neighbors` holds anything else in the
   same state — *pending*.
2. **`nOptimize` 4 → 8** (S6) — buys CPU users the whole #1292 quality jump at a CPU-pass cost —
   *pending*.
3. **`fFusePriorWeight`** away from `3.0`, and/or **`nMinPixelsFuse`** away from `5` (S4) —
   *pending*.
4. **`nFuseViolationMax`** away from `0`, and whether the FSV guard should apply to non-rescued
   points (S5) — *pending*.
5. **`ReconstructMesh --min-point-distance` 1.5 → 2.0** (S7) — mesh-side; also resolves the
   CLI-vs-library (1.5 vs 2) mismatch recorded during the mesh effort's audit (git history) —
   *pending*.
6. **Re-freezing the canonical bench clouds** (S0b) — this rewrites the meaning of every
   historical `bench/out_mesh/results.csv` row and every `docs/design/DelaunayMeshReconstruction.md`
   table measured against them — *pending*.
7. **Any structural change to the fusion algorithm** (S10: pixel release on drop,
   confidence-ordered seeding, corroboration support, neighborhood re-probe) — unlike C1–C6 these
   are not one-line reverts of a constant; each changes which clusters form, so each lands only on
   its own S9-instrumented evidence plus a clean acceptance-gate pass, one commit per mechanism —
   *pending*.

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
4. **Estimation-side residual.** `ec13cab7` (parallel CUDA PatchMatch) and `69b25a37` (dmap codec
   / image cache / view-locality order) landed between the two builds. Presented as performance
   work; not verified numerically neutral. S2 arm (iv) is the falsification test.
5. **`nMaxViewsFuse = 32` vs `--number-views 12`.** Fusion caps its neighbor set at
   `nMaxViewsFuse` while estimation used 12; family-A logs imply ~18.7 views per cluster, i.e.
   fusion reaches past the estimation neighborhood through the flood-fill. Whether that is
   intended, and whether it interacts with the rescue rule, is unexamined.
6. **Is `nDepths` (the "%u depths" in the summary line) the number anyone wants?** It sums
   `fusedViews.size()` over all clusters including dropped ones, so the `points (%d%%)` figure it
   feeds is not a fusion yield. The `Fusion pixel accounting` line supersedes it; consider
   retiring or relabelling the older summary.
