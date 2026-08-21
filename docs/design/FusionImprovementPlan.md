# Depth-Map Fusion — Default-Quality Improvement Plan

Target: `MVS::DepthMapsData::DenseFuseDepthMaps` (`libs/MVS/SceneDensify.cpp:2634`), the
`--fusion-filter 2` default path, and the `OPTDENSE` fusion defaults in
`libs/MVS/DepthMap.cpp` / `apps/DensifyPointCloud/DensifyPointCloud.cpp`.

Trigger: the mesh benchmark (`bench/out_mesh/results.csv`) contains two families of frozen dense
clouds whose cloud-F1 differs by +0.010 … +0.114, and the maintainer asked for the default fusion
to capture the better family. This plan first establishes **why** the two families differ (§ 0–2),
then ranks what is actually left to win (§ 4), with protocol (§ 5), slices (§ 6) and reserved
decisions (§ 7).

Conventions follow `docs/design/DelaunayMeshImprovementPlan.md`: § 4 candidates carry
mechanism / expected effect / risk / cost; § 5 reuses the § 8 acceptance gates of that plan; § 6 is
one reversible commit per slice, each with its own validation; results land in
`docs/design/DelaunayMeshReconstruction.md` alongside the mesh-side record.

---

## 0. The measured A-vs-B delta

Family **A** = the frozen `<Scene>/runMetashape/scene_dense.{mvs,ply}` clouds the bench has used
since June. Family **B** = the `<Scene>/runFusionStats/` clouds re-densified 2026-08-19 for
Phase 5.0/5.1 of the Delaunay work. Both under
`C:\Pro\TanksAndTemples\data\training\` (read-only).

| scene | A points | A cloud P / R / F1 | B points | B cloud P / R / F1 | Δcloud F1 |
|---|---|---|---|---|---|
| Barn | 7,703,886 | 0.5983 / 0.5994 / **0.5988** | *(no B cloud)* | — | — |
| Ignatius | 5,211,727 | 0.7271 / 0.7494 / **0.7381** | 9,844,135 | 0.7182 / 0.8370 / **0.7731** | **+0.0350** |
| Meetingroom | 3,106,975 | 0.5608 / 0.2264 / **0.3225** | 10,000,054 | 0.4853 / 0.3968 / **0.4366** | **+0.1141** |
| Truck | 5,990,832 | 0.6916 / 0.7210 / **0.7060** | 9,715,279 | 0.6785 / 0.7571 / **0.7156** | **+0.0096** |

Shape of the win: **recall up a lot, precision down a little, points ×1.6–3.2.**
Recall +0.0876 / +0.1704 / +0.0361; precision −0.0089 / −0.0755 / −0.0131.

The downstream (mesh) half of the same rows — `variant=baseline`, python sampler, seed 42:

| scene | A mesh F1 | B mesh F1 | Δmesh | A `recon_wall_s` | B `recon_wall_s` | A `peak_ws_mb` | B `peak_ws_mb` | A→B `delaunay_verts` |
|---|---|---|---|---|---|---|---|---|
| Ignatius | 0.3295 | 0.3348 | **+0.0053** | 139.2 | 243.5 (×1.75) | 5,353 | 9,262 (×1.73) | 2.74 M → 4.75 M |
| Meetingroom | 0.3372 | 0.3566 | **+0.0194** | 97.6 | 597.8 (**×6.13**) | 4,814 | 12,768 (×2.65) | 2.48 M → 6.50 M |
| Truck | 0.3569 | 0.3511 | **−0.0058** | 160.5 | 237.8 (×1.48) | 7,108 | 10,256 (×1.44) | 3.63 M → 5.22 M |

Two facts the headline hides and every candidate below must respect:

1. **Truck's mesh F1 regresses by 0.0058** — more than the § 8 "no scene regresses > 0.003"
   clause — even though its cloud F1 rose. A cloud win is not automatically a pipeline win.
2. **Meetingroom's mesh cost is superlinear in points**: ×3.2 points → ×6.1 wall
   (`tri_s` 14.0→76.1, `weight_total_s` 25.7→210.4, `graphcut_s` 24.2→143.3) and 12.8 GB peak on a
   31.8 GB machine. Completeness pushes must be cost-paired (§ 4 C7).

---

## 1. Established cause

**The two families differ only in the binary.** Verified, not inferred:

| Evidence | Where |
|---|---|
| Family A command line, all four scenes: `DensifyPointCloud scene.mvs --resolution-level 1 --number-views 12 --estimate-roi 0 --crop-to-roi 0 --tower-mode 0` | `<Scene>/runMetashape/DensifyPointCloud-2606*.log`, Truck `-260729125834007651.log` |
| Family B command line: **identical**, plus `-v 2` (and `--export-unfused-file unfused.bin` on the second Ignatius/Meetingroom/Truck pass) | `<Scene>/runFusionStats/DensifyPointCloud-2608*.log` |
| Family A build: `OpenMVS x64 v2.4.0 (9a007b5*)`, build date **Jun 6 2026** (Truck's re-freeze: `e884035`, Jul 29 2026 — still pre-#1292) | same logs, header lines |
| Family B build: `OpenMVS x64 v2.4.0 (e2f38b4f*)`, build date **Aug 18 2026** = develop head plus the local Phase-5.0 accounting patch | same logs |
| Family A logs contain **no** `Adaptive confidence enabled` / `Integrated confidence recalibration` lines; family B logs contain both | log greps |

Diffing the option tables of the two revisions (`git show 9a007b5:libs/MVS/DepthMap.cpp` vs
`git show HEAD:libs/MVS/DepthMap.cpp`, `DEFVAR_OPTDENSE` lines only) yields **exactly one commit's
worth of change** to the densify defaults — `f61bffef` *"dense: depth-map confidence recalibration
(#1292)"*, 2026-08-13:

| Option | June (family A) | today (family B) | Where |
|---|---|---|---|
| `nOptimize` / `--postprocess-dmaps` | `0` | **`4`** = `ADJUST_CONFIDENCE_AUTO` | `DepthMap.cpp:113`; `DepthMap.h:87-101`; `DensifyPointCloud.cpp:182` |
| `fFusePriorWeight` / `--fusion-prior-weight` | *(did not exist)* | **`3.0`** | `DepthMap.cpp:118`, `DensifyPointCloud.cpp:181` |
| `nFuseViolationMax` | *(did not exist)* | **`0`** (strict) | `DepthMap.cpp:119` |
| `bEstimateConfidenceCUDA` | *(did not exist)* | `1` | `DepthMap.cpp` (new), `DepthMap.h:161` |
| `nMinViewsFilter`, `nMinViewsFilterAdjust` | present | retired | — |
| `nPatchMatchCUDAInstances` | *(did not exist)* | `4` | `DepthMap.cpp`, `DensifyPointCloud.cpp:169` |

Everything else a fusion result could depend on is **byte-identical between the two revisions**:
`fDepthReprojectionErrorThreshold 1.2`, `fDepthDiffThreshold 0.01`, `fNormalDiffThreshold 25`,
`nMinPixelsFuse 5`, `nMinViewsFuse 2`, `nMaxViewsFuse 32`, `nMaxPointsFuse 1000`,
`fNCCThresholdKeep 0.9`, `fEstimationGeometricWeight 0.1`, `nEstimationGeometricIters 2`,
`nFuseFilter 2`.

**Conclusion: the family-B behaviour IS today's default on develop.** The bench's family-A clouds
are simply stale June artifacts. There is no "family-B setting" to adopt — there is a stale
baseline to re-freeze (slice S0) and a set of *further* fusion levers to test (§ 4).

### 1.1 Hypotheses checked and refuted

`docs/design/DelaunayMeshReconstruction.md` (Phase 5.0 T&T readout, ~line 1000) attributes the
A/B gap to *"a `Densify.ini` in the working folder overriding defaults (reprojection threshold 1.2
vs today's 1.0, geometric weight 0.1 vs 0.3, pre-recalibration confidence lineage)"*. Two of those
three claims are wrong and must be corrected (slice S1):

- **`Densify.ini` was never read.** `DensifyPointCloud` calls `OPTDENSE::oConfig.Load(OPT::strDenseConfigFileName)`
  (`DensifyPointCloud.cpp:272`) with a name that stays **empty** unless `--dense-config-file` is
  passed (`:207`, `:269`); `CConfigTable::Load` forwards straight to `SML::Load(f)`
  (`libs/Common/ConfigTable.cpp:100`), which opens the literal path and fails. No family-A command
  line passed the flag. Decisive counterexample: `Meetingroom/runMetashape/` has **no `Densify.ini`
  at all** yet shows the most extreme family-A profile (P 0.5608 / R 0.2264).
- **The reprojection default is still 1.2, in both families.** Commit `dc32ab8` *"dense: lower
  default fusion reprojection threshold 1.2 -> 1.0"* (2026-06-10 09:27) **is not on develop**:
  `git merge-base --is-ancestor dc32ab8 HEAD` → NO; `git branch --contains dc32ab8` → only
  `dense-cell-neighbors`; `docs/fusion_reprojection_threshold.md` is absent from the worktree;
  `DensifyPointCloud.cpp:180` and `DepthMap.cpp:104` both still read `1.2`. (See § 4 C1 — this is a
  live, validated, *unshipped* win.)
- **The geometric weight is still 0.1, in both families.** `833e93c` *"dense: raise PatchMatch
  geometric-consistency weight 0.1 -> 0.3"* (2026-06-09) is likewise only on `dense-cell-neighbors`;
  `DepthMap.cpp:124` reads `0.1`, and there is **no `--geometric-weight` CLI option** at all.
- **Not the neighbor selector.** `9a007b5` is titled "per-cell neighbor-view selection (WIP
  snapshot)", but grepping that revision for `CellViews|nCellViews|PerCell|nGrid` in
  `DepthMap.cpp`/`DensifyPointCloud.cpp` returns nothing — no per-cell selector was active. Both
  families log `Reference image N paired with 12 views`.
- **Not resolution, view count, ROI or tower mode** — identical flags, identical
  `960x540` depth-map sizes, identical image counts (263 / 371 / 251 / 410).

### 1.2 What remains hypothesis (not yet measured)

- **The split between the two #1292 sub-mechanisms** (confidence recalibration vs prior-rescue) is
  unmeasured. Slice S2 measures it; § 2 gives the two mechanisms.
- **The depth maps themselves also changed** between the two builds: `ec13cab7` (CUDA PatchMatch
  parallelized across workers) and `69b25a37` (shared `.dmap` codec, image cache, view-locality
  order) landed in the same window. These are presented as performance work, but PatchMatch is
  **unseeded**, so a residual estimation-side contribution cannot be excluded by inspection alone.
- **Size of that residual, bounded empirically**: identical build + identical flags gives
  Barn 7.75 M / 7.48 M / 7.70 M points across three June runs (spread 3.6 %), Meetingroom
  3.53 M / 3.56 M / 3.11 M (spread **14 %**), Ignatius 5.19 M / 5.21 M (0.3 %); family-B Ignatius
  9.42 M vs 9.84 M (4.5 %). Point count is noisy at the several-percent level; the A/B cloud-F1
  deltas (+0.010 … +0.114) are far above it, so attribution to *the #1292 lineage* is safe, but
  **every fusion A/B from here on must run on frozen `.dmap` files** (§ 5.2), never on separate
  densifications.

---

## 2. Mechanism — how #1292 changes what fusion admits

`DenseFuseDepthMaps` grows one cluster per unclaimed seed pixel by flood-filling neighbor views
through `FusePoint` (`SceneDensify.cpp:2688`), then applies a keep-rule. The gates:

| Gate | Line | Constant |
|---|---|---|
| seed/probe confidence | `:2719-2727` | `conf < minConfidence`, `minConfidence = 1 − fNCCThresholdKeep = 0.1` |
| depth agreement | `:2737` | `fDepthDiffThreshold = 0.01` (with the FSV sub-classification at `:2747`) |
| lateral reprojection | `:2755` | `normSq(diff) > maxReprojErrorSq`, `maxReprojErrorSq = fDepthReprojectionErrorThreshold² = 1.44` |
| normal agreement | `:2760` | `fNormalDiffThreshold = 25°` |
| keep-rule | `:2905-2907` | `pixels + virtualSupport ≥ nMinPixelsFuse(5)` **and** `views + virtualSupport ≥ nMinViewsFuse(2)` |
| FSV guard on rescued points | `:2916` | `fusedViolViews.size() ≤ nFuseViolationMax(0)` |

#1292 moves two of these:

**(a) The confidence gate now reads a recalibrated posterior.** `AdjustConfidence`
(`SceneDensify.cpp:1261-1330`) replaces photometric 1-NCC with a value that predicts *"will this
depth survive fusion"*: `gate = 1 − exp(−(K + kPrior·pGeo)/tau)` over cross-view confirmations `K`
and the intra-map geometric prior `pGeo`. The output is bimodal, so the fixed 0.1 gate lands in the
valley rather than mid-distribution. Family B's own histograms confirm it: admitted pixels pile
into bins 8–9, dropped pixels into bin 0 (`Fusion confidence histogram` lines,
`Ignatius/runFusionStats/DensifyPointCloud-26081915165600B59D.log`).

**(b) The keep-rule gained a rescue term.** `virtualSupport = fFusePriorWeight · priorMap(i,j)`
(`:2904`), `fFusePriorWeight = 3`, `priorMap ∈ [0,1]` from `ComputeIntraMapPrior` (`:1183`). At
prior 1.0 a cluster of **2 pixels seen by 1 view** now passes a rule that used to demand 5 pixels
and 2 views. This attacks the drop channel Phase 5.0 measured as dominant — 20–35 % of all valid
depths lost to `min-pixels`. `nFuseViolationMax = 0` is the counterweight: a *rescued* point
contradicted by even one free-space-violating view is dropped again (non-rescued points are never
subject to it).

That is the recall mechanism, and it is exactly the observed signature: many more, smaller clusters
(family A `nDepths/point` ≈ 18.7, family B ≈ 8.0 — `nDepths` sums `fusedViews.size()` over all
clusters, `:2947`), higher recall, slightly lower precision.

**Post-#1292 residual head-room** (Phase 5.0 T&T accounting, `docs/design/DelaunayMeshReconstruction.md`):
even *with* the rescue, Ignatius still drops 52.0 % of valid depths (26.4 % low-conf, 25.4 %
min-pixels), Meetingroom 69.9 % (34.6 / 35.0), Truck 44.0 % (23.6 / 20.3); each scene throws away
8.7–10.0 M pixels at confidence ≥ 0.7. The channel is not exhausted.

---

## 3. Candidate changes, ranked

Rank = (evidence strength) × (expected F1) ÷ (risk + cost). Every candidate is a single literal in
`libs/MVS/DepthMap.cpp` plus the matching `default_value(...)` in
`apps/DensifyPointCloud/DensifyPointCloud.cpp`, so each is a one-line revert.

### C1 — Restore `fDepthReprojectionErrorThreshold` 1.2 → 1.0 *(highest confidence)*

- **Mechanism**: `maxReprojErrorSq` at `SceneDensify.cpp:2755` is the *lateral* pixel tolerance for
  treating two estimates as the same surface point. At 1.2 it over-merges distinct surfaces into
  averaged blobs that are both wrong (precision) and off-GT (recall).
- **Evidence (do not re-derive, but do re-measure)**: `dc32ab8`'s own T&T table, R1/V12 baseline
  selector — Truck 0.7080→0.7126 (+0.0046), Barn 0.5875→0.5927 (+0.0052), Ignatius
  0.7337→0.7385 (+0.0048), Meetingroom 0.3440→0.3468 (+0.0028); **both** P and R rise on the three
  recall-healthy scenes; independent of the neighbor selector; +0.0013 at full resolution, never
  negative. 0.9 was rejected: it over-tightens Meetingroom below the old default.
- **Expected**: mean cloud F1 ≈ +0.004, all four scenes positive. Clears the § 5 gate on the June
  measurement.
- **Risk**: that measurement predates #1292. Tighter joining could now push more clusters under
  `nMinPixelsFuse`. **Counter-argument**: `fFusePriorWeight`'s rescue did not exist in June and
  absorbs exactly that failure mode, so the recall risk is lower today, not higher. Must be
  re-measured on frozen dmaps regardless.
- **Cost**: none (same work, different constant). Point count moves a few percent either way.
- **Files**: `libs/MVS/DepthMap.cpp:104`, `apps/DensifyPointCloud/DensifyPointCloud.cpp:180`,
  restore `docs/fusion_reprojection_threshold.md` from `dc32ab8`.

### C2 — CPU / non-CUDA parity for the confidence recalibration *(largest unrealized win)*

- **Mechanism**: `nOptimize` defaults to `4` = `ADJUST_CONFIDENCE_AUTO` (`DepthMap.h:99`), which
  `Scene::ComputeDepthMaps` resolves at `SceneDensify.cpp:3450-3458` to `ADJUST_CONFIDENCE` **only
  when the estimation backend is CUDA** (where it fuses into the last geometric iteration and costs
  ~18.5 ms/map — 4.9 s over 263 maps on Ignatius). On a CPU-only build, or with
  `--gpu-device cpu`, the bit is cleared and fusion sees raw photometric confidence: **every
  CPU user is still getting family-A fusion.**
- **Expected**: on CPU, the same class of jump family B shows (+0.035 … +0.114 cloud F1), minus
  whatever share belongs to the prior-rescue (which *is* already active on CPU, since `priorMap` is
  purely geometric).
- **Risk / cost**: the standalone CPU `AdjustConfidence` phase is a separate full-resolution pass
  over every depth map with its neighbors loaded (`SceneDensify.cpp:1345-1445`) — potentially a
  large fraction of CPU densify wall time. **Unmeasured. Measure before proposing any flip.**
- **Options**: (a) document and leave; (b) `nOptimize` default `4` → `8` (force everywhere);
  (c) keep `4` but log a loud one-liner telling CPU users what they are missing.
- **Files**: `libs/MVS/DepthMap.cpp:113`, `apps/DensifyPointCloud/DensifyPointCloud.cpp:182`.

### C3 — `fFusePriorWeight` sweep (0 / 2 / 3 / 4 / 6)

- **Mechanism**: the rescue term at `SceneDensify.cpp:2904`. Its own option doc already states the
  trade: *"default 3 favors completeness and suits the usual pipeline where mesh reconstruction
  follows and cleans the few extra outliers, use 2 when the dense point-cloud is the final output"*.
- **Expected**: monotone recall↑ / precision↓ in the weight. The interesting question is whether 3
  is the F1 optimum or merely the completeness-leaning choice — family B's precision drop
  (−0.0089 / −0.0755 / −0.0131) suggests 2 may be the F1 optimum on Meetingroom while 4–6 may still
  pay on Truck/Barn.
- **Risk**: scene-dependent optimum ⇒ likely no defaultable single value; a per-scene split is
  *not* acceptable as a default. Precision loss compounds downstream (see § 3.C7).
- **Cost**: fusion-only, ~40–50 s per arm per scene on frozen dmaps.

### C4 — `nMinPixelsFuse` 5 → 4 / 3

- **Mechanism**: the raw keep-rule threshold the rescue term softens. Attacks the same
  20–35 % channel bluntly and unconditionally.
- **Expected**: recall↑, precision↓, more than C3 for the same nominal loosening, because it
  rescues clusters *regardless* of whether they sit on a locally coherent surface.
- **Risk**: high. C3 is the principled version of the same lever; expect C4 to be strictly
  dominated. Test it to *prove* the prior-gated form is better, not because it is expected to win.
- **Rank below C3.** Files: `libs/MVS/DepthMap.cpp:90`.

### C5 — `nFuseViolationMax` arms

- **Mechanism**: `SceneDensify.cpp:2916`. Today `0` = drop any rescued point contradicted by ≥1
  free-space-violating view; `-1` disables the guard (byte-identical to pre-guard fusion).
- **Two arms**: (i) recall-side, `0 → 1 / 2 / -1`; (ii) **precision-side, the more interesting
  one** — extend the guard to *non-rescued* points as well. Phase 5.0 measured FSV at only ~0.1 %
  of probes but nonzero on every scene; family B's precision drop is the symptom this would target.
- **Cost**: fusion-only. Arm (ii) is a small, contained code change, not a constant flip.

### C6 — Confidence gate `minConfidence = 1 − fNCCThresholdKeep` *(low expectation, cheap falsification)*

- **Mechanism**: `SceneDensify.cpp:2644`, `:2720`. Default 0.1.
- **Evidence against**: Phase 5.0 proved the low-confidence drop channel is *exactly* the `[0,0.1)`
  histogram bin in every scene — the recalibrated posterior separates cleanly and the gate sits in
  the valley. Moving it should mostly trade one kind of nothing for another.
- **Do it anyway** as a 3-point sweep (0.05 / 0.10 / 0.15) because it is one fusion-only rerun and
  it closes the question. `fNCCThresholdKeep` is shared with estimation-side code
  (`DepthMap.cpp:460-463`, `:767`, `:854`), so **do not flip `fNCCThresholdKeep`** — if this ever
  looks promising, introduce a dedicated `fFuseMinConfidence` instead.

### C7 — Downstream cost containment (pairing requirement, not a fusion change)

Not optional: § 0 shows Meetingroom at ×6.1 mesh wall and 12.8 GB peak. The lever already exists —
`ReconstructMesh --min-point-distance` (`OPT::fDistInsert`, `ReconstructMesh.cpp:153`, CLI default
`1.5` px vs library default `2` in `Scene.h:149` — a mismatch the Delaunay plan § 0 already records)
decimates at Delaunay insertion:

| scene | family A points → `delaunay_verts` | family B points → `delaunay_verts` |
|---|---|---|
| Ignatius | 5.21 M → 2.74 M (53 %) | 9.84 M → 4.75 M (48 %) |
| Meetingroom | 3.11 M → 2.48 M (80 %) | 10.00 M → 6.50 M (65 %) |
| Truck | 5.99 M → 3.63 M (61 %) | 9.72 M → 5.22 M (54 %) |

So insertion already absorbs roughly half the growth, and Meetingroom — the scene that blew up — is
the one whose filter absorbs the *least*. **Rule for this plan**: any candidate that raises the
point count by > 20 % must be reported with a paired `--min-point-distance 2.0` mesh row, so the
maintainer sees quality-at-equal-cost, not just quality-at-any-cost.
Do **not** add a new cloud-side decimation option: `DensifyPointCloud --filter-point-cloud`
(`thFilterPointCloud`, default 0) is a visibility filter, not a spacing filter, and inventing a
second spacing knob duplicates `fDistInsert`.

### Explicitly out of scope

- `833e93c` (`fEstimationGeometricWeight` 0.1 → 0.3): estimation-side, not fusion; it changes the
  depth maps and therefore cannot be A/B'd on frozen dmaps. Note its stranded status in S1 and hand
  it to a separate estimation plan.
- Anything that changes `Conf2Weight` / `pointWeights` semantics — bound by the Delaunay plan's
  binding directive § 2.1 ("the weight is the plain [0,1] confidence, or nothing").
- `--carve-rays-file` (Phase 5.1): already benched, verdict opt-in, re-test deferred to Phase 4.1.

---

## 4. Benchmark protocol

### 4.1 Harness

`bench/run_mesh.py` (two-stage: cloud F1 + mesh F1 in one row) → `bench/out_mesh/results.csv`.
Scenes **Barn / Ignatius / Meetingroom / Truck**; python sampler; `--sample-points 10000000`;
`--sample-seed 42`. Columns already present and mandatory in every report:
`cloud_p, cloud_r, cloud_f1, mesh_p, mesh_r, mesh_f1, pts_inserted, delaunay_verts, cells,
tri_s, weight_total_s, graphcut_s, recon_wall_s, peak_ws_mb`.

`--work-subdir <name>` selects the frozen-cloud folder and suffixes the scene name
(`Ignatius@runFusionStats`), so fusion-variant rows never collide with baseline rows and the
`cloud_f1.json` cache (keyed on the `.mvs` size+mtime) invalidates automatically on a re-fuse.

Cheap cloud-only arms can use `bench/run_cell.py` (`--R 1 --V 12 --scene <S> --reproj <x>
--keep-dmaps`), but it needs two fixes first (slice S0): line 160 passes `--cuda-device`, renamed
to `--gpu-device` (CPU sentinel is now `-2` / `cpu` / empty), and line 152 passes
`--geometric-weight`, an option `DensifyPointCloud` does not have.

### 4.2 Fusion A/Bs run on frozen depth maps — mandatory

PatchMatch is unseeded; § 1.2 measures 0.3–14 % run-to-run point-count spread at identical
settings. Comparing two fusion settings across two densifications is therefore **invalid**.

The `runFusionStats/` folders keep every `depth0000..N.dmap`. Re-running the same
`DensifyPointCloud` command line in such a folder skips estimation entirely
(`SceneDensify.cpp:3694-3695`: `if (!File::access(path))` → `depthmapComputed`) and re-fuses in
~40–50 s instead of ~90–600 s. Every § 3 candidate is therefore evaluated as: *same bytes in,
one constant changed, fuse again*. Keep `--remove-dmaps 0` (the default) and never mix resolution
levels in one folder (a stale-resolution `.dmap` is a known crash, not a silent error).

Barn has **no** `runFusionStats/` folder — slice S0 creates it.

Run at `-v 2` so the `Fusion pixel accounting` / `Fusion probe accounting` / confidence-histogram
lines land in the log; both accounting residuals must read `0` on every arm (they are
self-checking partitions, `SceneDensify.cpp:2441-2540`).

### 4.3 Noise floor

- **Mesh stage**, measured on the frozen pipeline, per-scene repeat spread: Meetingroom
  0.3372 / 0.3379 / 0.3385 (0.0013), Barn 0.5575 / 0.5577 (0.0002), Ignatius 0.3295 / 0.3294,
  Truck 0.3569 / 0.3569. **Working floor: 0.0013 per scene.**
- **Cloud stage** on frozen dmaps is deterministic (the fuse is serial; the only parallel region
  caches dmaps and never calls `FusePoint`). The residual source is the evaluator's ICP when
  `<Scene>_final_transform.npy` is absent — `run_mesh.py` logs its presence per scene; require it
  present for all four scenes before any gated run.
- `--noise-floor` re-measures both any time the machine or build changes.

### 4.4 Acceptance gate

Adopted from `DelaunayMeshImprovementPlan.md` § 8 (energy/filtering changes), extended for the
downstream cost this plan can move:

- **Quality**: mean paired **cloud**-F1 ≥ **+0.003** beyond noise, **and no scene regressing >
  0.003**. P and R each within 1 pp unless the complementary gain nets a clear F win.
- **Downstream, mandatory reporting, not merely advisory**: the same run reports mesh F1,
  `recon_wall_s`, `peak_ws_mb`, `pts_inserted`, `delaunay_verts`. Any of
  — a mesh-F1 regression > 0.003 on any scene,
  — a mesh `recon_wall_s` increase > 25 % (median of ≥ 3 runs),
  — a `peak_ws_mb` increase > 15 %
  escalates the candidate to § 6 (maintainer decision) instead of being proposed as a default,
  **even if the cloud gate passes**. Family B itself would trip this on Truck (mesh −0.0058) and on
  Meetingroom (wall ×6.1) — which is the point of the clause.
- **Repeats**: 1 run suffices for a fusion-only quality row (byte-identical input); ≥ 3 runs and
  medians for anything quoting wall time or memory.
- **Instrumentation**: both fusion accounting residuals `0`; report the per-channel drop table
  (low-conf / min-pixels / min-views / violation) for every arm, so a quality change is always
  explained by a channel change.

---

## 5. Execution slices

One reversible commit per slice; each slice appends its before/after table and an
accepted/rejected verdict to `docs/design/DelaunayMeshReconstruction.md`. Builds and tests follow
`.github/instructions/`. `Tests.exe` prints nothing — assert the exit code (`Tests.exe 1 1` for the
selector+verbosity form).

**S0 — Re-baseline the bench (prerequisite, no library change).**
(a) Create `Barn/runFusionStats/` with one full densify at today's defaults and keep the dmaps.
(b) Re-freeze, or formally retire, the four `runMetashape` clouds — every current bench row compares
today's mesh code against a June cloud. Recommended: keep `runMetashape` frozen as the historical
row and promote `runFusionStats` (4 scenes) to the canonical fusion baseline, `--tag fusion-base`.
(c) Fix `bench/run_cell.py:152,160` (`--geometric-weight` does not exist; `--cuda-device` →
`--gpu-device`).
*Validation*: four `cloud_f1.json` caches regenerate; four baseline rows land in
`bench/out_mesh/results.csv`; `--noise-floor` re-run reproduces ≤ 0.0013.

**S1 — Record the provenance finding (doc-only).**
Correct the Phase-5.0 caveat paragraph in `docs/design/DelaunayMeshReconstruction.md` (the
`Densify.ini` / "1.0" / "0.3" claims are wrong — § 1.1), insert the § 1 provenance table, and record
that `dc32ab8` and `833e93c` are stranded on `dense-cell-neighbors` and absent from develop.
*Validation*: none needed; but flag both stranded commits to the maintainer in the same breath.

**S2 — Channel ablation on frozen dmaps (measurement, no default change).**
Four arms per scene, all fusion-only where possible:
(i) today's default; (ii) `--fusion-prior-weight 0`; (iii) dmaps re-estimated with
`--postprocess-dmaps 0`, then default fusion; (iv) both off.
Arms (iii)/(iv) need a re-densification per scene into a sibling folder (`runFusionStatsRaw/`)
because the confidence lives in the dmap.
*Deliverable*: the split of the +0.035 … +0.114 into recalibration vs prior-rescue, plus the
per-channel drop table per arm.
*Validation*: residuals `0` on all arms; arm (iv) should land near the family-A point counts
(5.2 M / 3.1 M / 6.0 M) — if it does not, § 1.2's residual (estimation-side drift) is larger than
assumed and § 1's attribution needs weakening.

**S3 — C1: restore `fDepthReprojectionErrorThreshold` 1.2 → 1.0.**
Cherry-pick `dc32ab8` (two literals + `docs/fusion_reprojection_threshold.md`).
*Validation*: fusion-only A/B on all four frozen dmap sets; § 4.4 gate; paired mesh rows.
Note in the commit body that the original evidence is pre-#1292 and this run is the re-validation.
*Revert*: one line.

**S4 — C3 + C4: rescue-strength sweep (measurement).**
`--fusion-prior-weight` ∈ {0, 2, 3, 4, 6} and `nMinPixelsFuse` ∈ {3, 4, 5} (config-file or a
temporary CLI passthrough), fusion-only, four scenes. No default change in this slice; the slice
produces the table and a single recommendation.
*Validation*: § 4.4, plus the explicit C4-vs-C3 dominance check (does prior-gated loosening beat
unconditional loosening at equal point count?).

**S5 — C5: free-space-violation arms.**
(i) `nFuseViolationMax` ∈ {-1, 0, 1, 2}; (ii) a contained code arm extending the guard to
non-rescued points.
*Validation*: § 4.4; arm (ii) must be provably off-state-identical when the guard is disabled
(byte-identical fused cloud vs today at `nFuseViolationMax = -1`).

**S6 — C2: CPU parity measurement.**
One CPU-only densify per scene (`--gpu-device cpu`) at `--postprocess-dmaps 0` vs `8`; report the
wall-time share of the standalone `AdjustConfidence` phase and the cloud-F1 delta.
*Validation*: § 4.4 on quality; the cost number decides which of the three C2 options is proposed.
*Note*: this is the only slice that cannot use frozen dmaps (the confidence is baked into them).

**S7 — Cost pairing.**
For every candidate that cleared § 4.4 in S3–S6, re-run the mesh stage at
`--min-point-distance` 1.5 (default) and 2.0 and report both (mesh F1, `recon_wall_s`,
`peak_ws_mb`) pairs.
*Validation*: identifies whether the cloud win survives at equal mesh cost.

**S8 — Consolidation.**
One commit per accepted default flip (each: one literal in `libs/MVS/DepthMap.cpp` + the matching
`default_value` in `apps/DensifyPointCloud/DensifyPointCloud.cpp` + option-doc text), plus the
results record. No flip lands without an explicit maintainer decision from § 6.

Order: **S0 → S1 → S2 → S3 → S4 → S5 → S6 → S7 → S8.** S1 can run in parallel with S0. S6 is
independent of S2–S5 and can be pipelined.

---

## 6. Reserved to the maintainer (default flips)

None of these are taken by this plan; each needs an explicit decision on the evidence the
corresponding slice produces.

1. **`fDepthReprojectionErrorThreshold` 1.2 → 1.0** (S3) — a validated win that was written,
   documented and then never merged. Also: decide whether `dc32ab8` and `833e93c` were deliberately
   dropped or lost, and whether `dense-cell-neighbors` holds anything else in the same state.
2. **`nOptimize` 4 → 8** (S6) — buys CPU users the whole #1292 quality jump at a CPU-pass cost.
3. **`fFusePriorWeight`** away from `3.0`, and/or **`nMinPixelsFuse`** away from `5` (S4).
4. **`nFuseViolationMax`** away from `0`, and whether the FSV guard should apply to non-rescued
   points (S5).
5. **`ReconstructMesh --min-point-distance` 1.5 → 2.0** (S7) — mesh-side; also resolves the
   CLI-vs-library (1.5 vs 2) mismatch already recorded in `DelaunayMeshImprovementPlan.md` § 0.
6. **Re-freezing the canonical bench clouds** (S0b) — this rewrites the meaning of every historical
   `bench/out_mesh/results.csv` row and every `docs/design/DelaunayMeshReconstruction.md` table
   measured against them.

---

## 7. Open questions

1. **The mesh stage eats most of the cloud gain.** Ignatius cloud +0.0350 → mesh +0.0053; Truck
   cloud +0.0096 → mesh **−0.0058**. The absolute mesh−cloud gap is worse still (Ignatius 0.773 →
   0.335). Tracked by the Delaunay plan, but it means *fusion-side* work has a low ceiling on
   end-to-end quality until it is understood. Should fusion candidates be gated on cloud F1 at all,
   or on mesh F1 directly (6–10× slower per arm)? This plan gates on cloud and *reports* mesh; the
   choice is revisitable.
2. **Meetingroom's ×6.1 mesh wall for ×3.2 points** is unexplained and superlinear in every
   sub-stage (`tri` ×5.4, `weight` ×8.2, `cut` ×5.9). Needs a profile before any completeness push
   is defaulted.
3. **Precision falls on every scene in family B** (−0.0089 / −0.0755 / −0.0131). Is that the
   prior-rescue admitting outliers — in which case `fFusePriorWeight 2` is the cloud-final answer
   its own doc string already suggests — or the unavoidable P/R trade of denser sampling? S2 + S4
   answer; until then it is the main reason not to push completeness harder.
4. **Estimation-side residual.** `ec13cab7` (parallel CUDA PatchMatch) and `69b25a37` (dmap codec /
   image cache / view-locality order) landed between the two builds. Presented as performance work;
   not verified numerically neutral. S2 arm (iv) is the falsification test.
5. **`nMaxViewsFuse = 32` vs `--number-views 12`.** Fusion caps its neighbor set at
   `nMaxViewsFuse` (`SceneDensify.cpp:2875`) while estimation used 12; family-A logs imply ~18.7
   views per cluster, i.e. fusion reaches past the estimation neighborhood through the flood-fill.
   Whether that is intended, and whether it interacts with the rescue rule, is unexamined.
6. **Is `nDepths` (`SceneDensify.cpp:2947`, the `%u depths` in the summary line) the number anyone
   wants?** It sums `fusedViews.size()` over *all* clusters including dropped ones, so the
   `points (%d%%)` figure it feeds is not a fusion yield. The `Fusion pixel accounting` line
   supersedes it; consider retiring or relabelling the older summary.
