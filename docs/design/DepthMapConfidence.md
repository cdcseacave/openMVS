# Depth-Map Confidence Recalibration — Design, Evidence and Research Record

## Overview

Every depth estimate OpenMVS produces carries a confidence in `[0,1]`, stored in the `.dmap` next to
the depth and used to gate fusion, to order points, and to weight visibility in the mesh step. By
default that number starts as a photometric score (`1 − NCC`), which answers *"how well did this
patch match?"* — a question only loosely related to the one every consumer actually asks, *"is this
depth correct?"*. A repetitive facade or a textureless wall can match beautifully and still be wrong.

The recalibration replaces the photometric score with a posterior that predicts **whether a depth
will survive fusion as an inlier**, built from three sources of evidence per pixel (an intra-map
plane-fit prior, continuous multi-view confirmation, free-space violations). Measured against
ground-truth depth on 28 scene-levels of BlendedMVS and ETH3D it lifts the pooled inlier/outlier
ROC-AUC from **0.844 to 0.926**, and roughly **doubles the depth retained at a fixed 1 %
contamination budget** (31.5 % → 57.9 %).

```
PatchMatch (CUDA)                                       DensifyPointCloud --postprocess-dmaps 8
  last geometric-consistency iteration                    standalone phase, CPU, all cores
  ├─ depth/normal/cost resident on device                  ├─ DMapCache-batched neighbor loads
  └─ ConfidenceCUDA.cu  (fused, ~3 ms/map)  ──┐            └─ AdjustConfidenceSweep (SSE)  ──┐
                                              ├──> confMap ──> .dmap (+ CONF_ADJUSTED flag) ──┤
  CPU PatchMatch: off by default ─────────────┘                                              │
                                                     fusion gate · point order · mesh weights ┘
```

This document is both the design record and the **research record**: what was tried, what was
measured, what was rejected, and what someone continuing the work needs to know. The experiment
harness itself (the `gt_bench/` tree and its Python tooling) was deliberately removed from the
shipping tree — § 11 says exactly how to get it back out of git history.

**Code.** `libs/MVS/ConfidenceRefine.h` (shared host/device math), `libs/MVS/ConfidenceCUDA.{h,cu}`
(kernels + launcher), `libs/MVS/SceneDensify.cpp` (`ComputeIntraMapPrior`, `AdjustConfidence`,
`AdjustConfidenceCUDA`, `AdjustConfidenceSweep`), `libs/MVS/PatchMatchCUDA.{cpp,inl}` (fused launch),
`libs/MVS/DMapCache.{h,cpp}` (phase-lifetime depth-map cache).

---

## 1. The model

Per reference pixel, with `K`/`Pconf` accumulated over the neighbour views:

```
gate      = 1 − exp(−(Kf + PRIOR_GATE·pGeo) / CONFIRM_TAU)      // is there any confirmation at all?
posterior = (PRIOR_STRENGTH·pGeo + Pconf) / (PRIOR_STRENGTH + Pconf + VIOLATION_W·V)
photo     = PHOTO_FLOOR + (1 − PHOTO_FLOOR)·confPhoto           // never fully discard photometry
conf      = clamp01(posterior · gate · photo)
if Kf ≥ 1: conf = max(conf, CONF_FLOOR · confPhoto)             // anti-cascade floor
```

- **`pGeo` — intra-map geometric prior** (`ComputeIntraMapPrior`): a local plane is fitted to the
  depth-map around the pixel; the pixel is scored by how well its neighbourhood agrees with that
  plane *and* by whether the plane's implied normal agrees with the estimated normal. A correct
  surface is locally coherent in both; a photometric mismatch usually is not.
- **`Kf`, `Pconf` — multi-view confirmation**: the pixel is projected into each neighbour view and
  compared against that view's own depth estimate through four **continuous** weights — G1 relative
  depth agreement, G2 forward-backward reprojection residual, G3 surface-normal agreement (a plain
  dot product, no threshold), G4 a smoothstep on the neighbour's own confidence. Each neighbour
  contributes a fractional vote (`Kf += w`, `Pconf += w·cN`), so agreement degrades smoothly instead
  of falling off a cliff.
- **`V` — free-space violations**: when a neighbour's own measured depth lies well *behind* our point
  along the same ray, that neighbour's line of sight passes *through* where we claim a surface is.
  That is direct negative evidence and dilutes the posterior. Counted separately from mere occlusion
  (a neighbour seeing something *closer* says nothing about our point) via `VIOLATION_MARGIN`.

### Shape constants (`ConfidenceRefine.h`)

| constant | value | role |
|---|---|---|
| `PRIOR_STRENGTH` | 2.0 | prior weight, as Beta pseudo-counts |
| `CONFIRM_TAU` | 1.5 | softness of the confirmation gate |
| `PRIOR_GATE` | 0.3 | prior's share of the gate when no neighbour confirms |
| `PHOTO_FLOOR` | 0.7 | minimum multiplicative photometric weight |
| `CONF_FLOOR` | 0.03 | anti-cascade floor (× photometric conf) once `Kf ≥ 1` |
| `VIOLATION_W` | 2.0 | denominator weight of the violation count |
| `VIOLATION_MARGIN` | 2.0 | how far behind (in units of `thDepth`) counts as a violation |

These are **compile-time constants, deliberately not user knobs**. They are one jointly-calibrated
operating point (§ 6.2): a full-grid sweep against ground truth picked them together, one global
setting won on every scene-level, and moving one without re-deriving the others degrades the result.
The gate *thresholds* (`minConfidence`, `thReproj`, `thDepth`) stay runtime because they are shared
with fusion.

They were DEFVARs during the research phase; the mapping, for reading the old documents, is
`fConfPriorStrength`→`PRIOR_STRENGTH`, `fConfConfirmTau`→`CONFIRM_TAU`,
`fConfPriorGate`→`PRIOR_GATE`, `fConfPhotoFloor`→`PHOTO_FLOOR`, `fConfFloor`→`CONF_FLOOR`,
`fConfViolationWeight`→`VIOLATION_W`, `fConfViolationMargin`→`VIOLATION_MARGIN`, and
`bConfSoftGates` is gone — the soft path is the only path.

### ⚠ The correctness invariant: raw neighbour confidence only

`cN` in G4 and `Pconf` **must be the neighbour's raw photometric confidence, never its adjusted
one.** Using adjusted confidence would (a) double-count geometric agreement — the neighbour's
posterior already folded in *its* neighbours, including us — and (b) make the result depend on worker
processing order, i.e. a nondeterministic cascade. The design is a **single Jacobi pass**: every
pixel's adjusted confidence is a function of raw neighbour confidences only.

How each path guarantees it:

- **fused / epilogue**: neighbours are read from the host `depthDataRef.images[]` snapshots that
  `InitViews` loaded from each neighbour's `.dmap`, i.e. the *previous* iteration's output, which was
  never adjusted (`geo.dmap → dmap` renaming happens after all workers join). Conveniently, the only
  copy that exists is also the only copy that is raw — there is no resident adjusted neighbour
  confidence on the device to reuse by accident.
- **standalone**: the `confMapAdjusted` deferred-swap barrier serves the same purpose.
- `InitViews` explicitly **skips moving a neighbour's confMap when it is flagged as adjusted**, so a
  re-run over already-adjusted dmaps cannot smuggle adjusted values in as evidence.

Never cache adjusted confidence on the device across views.

---

## 2. Where it runs

| path | when | cost |
|---|---|---|
| **fused in-estimation** (default on GPU) | CUDA estimates the depth-maps: the kernels run inside `PatchMatch::EstimateDepthMap` right after the last geometric-consistency kernels, reading the resident depth/normal (`cudaDepthNormalEstimates`) and raw cost (`cudaDepthNormalCosts`) | ~3.3 ms/map at 0.29 MP, 60 ms/map at 6.1 MP |
| **epilogue GPU re-upload** | fused launch failed, or the speckle/gap filters are on (they change the depth after estimation) | ~5 ms/map at 0.29 MP, 75 ms/map at 6.1 MP |
| **standalone CPU sweep** | `--postprocess-dmaps 8`, CPU estimation, `--geometric-iters 0`, or re-adjusting existing dmaps | 93 ms/map at 0.29 MP, ~2.4 s/map at 6.1 MP |

`--postprocess-dmaps` bits: `1` remove-speckles, `2` fill-gaps, `4` **ADJUST_CONFIDENCE_AUTO**
(default), `8` ADJUST_CONFIDENCE (force on). AUTO resolves in `ComputeDepthMaps` to on for CUDA
estimation and off for CPU — on the GPU the recalibration rides along on buffers that are already
there; on the CPU it costs a separate full-resolution sweep comparable to a fusion pass, which is not
a cost to impose by default. `Estimate Confidence CUDA = 0` (dense config file) forces the CPU
version even when CUDA estimates.

> **Reading the historical documents:** they say `--postprocess-dmaps 4` for "force adjust". That bit
> was renumbered when AUTO was introduced — **today that command is `--postprocess-dmaps 8`**. The
> old integrated-CPU opt-in (`Estimate Confidence = 1`) and the offline feature export
> (`--export-conf-features`) no longer exist (§ 10).

**Double-adjust guards.** In-process: the standalone phase is skipped for any view the integrated
path already adjusted. Cross-process: every recalibrated dmap carries a `CONF_ADJUSTED` flag bit in
its header (`Interface.h`), and the standalone phase warns and skips flagged views instead of
compounding the posterior on its own output. The flag survives the SFM undistortion rewrite and is
exposed to Python as `conf_adjusted` (`scripts/python/MvsUtils.py`).

---

## 3. Implementation history

| # | commits | what happened |
|---|---|---|
| 1 | `4888fc6` | first fusion-faithful confidence + an inlier/outlier eval harness; defaults tuned on Tanks&Temples with a mono-model pseudo-GT |
| 2 | `058d2b6`, `ac405bb` | intra-map prior redesigned from a depth-variance heuristic to a slope-aware **plane + normal** fit; variant A (agreement-only) selected |
| 3 | `16f4163`, `573ae83` | the same prior reused in fusion as *virtual support*, letting few-view inliers survive (`fFusePriorWeight`) |
| 4 | `ff92771`, `426d1f7`, `f1b72c1` | free-space-violation evidence; **soft (continuous) gates**; GT-recalibrated defaults |
| 5 | `309d376`, `ee00b08` | free-space guard on fusion-rescued points (`nFuseViolationMax`, counting *distinct* violating views) |
| 6 | `8b460a5`, `792b16c`, `5f1428c`, `eb471ae`, `e8de9bd` | speed: fused single-precision projection, neighbour-outer SSE sweep, phase-lifetime `DMapCache` (+ a mid-sweep eviction use-after-free fix), one shared prior with the nested-OpenMP trap closed |
| 7 | `6fa582f`, `18194c2` | integrated (in-estimation) CPU mode + the A/B that kept standalone as the CPU default |
| 8 | `8fd0516` | **`ConfidenceRefine.h`** — the per-pixel math extracted into one `__host__ __device__` header; the CPU refactor onto it verified byte-identical (0 of 5.6 M pixels differ) |
| 9 | `fe34ae6`, `55f7705`, `248fd0e`, `32d04b4` | `PriorKernel` + `SweepKernel` + host launcher; wiring, default-on, timing line, error-path fix |
| 10 | `a7c2a04`, `d3ce222` | **fused kernel**: resident-buffer reuse (reference maps no longer re-uploaded, −12.5 % transfer) and neighbour-depth `tex2D` reuse (−8.6 % more); per-worker device footprint `28+20n → 8+16n` B/px (≈ −27 %); cross-process `CONF_ADJUSTED` guard |
| 11 | `bb2bf4c`, `d6e7d56` | the mesh-weight lesson (§ 8.6): stop persisting the fusion-internal weight, persist the plain `[0,1]` confidence |
| 12 | `147f450` | latent upstream `Mesh::Clean` spike-removal infinite loop, exposed by this branch's denser clouds |
| 13 | `b70d386`, `ca3fc92` | research scaffolding pruned; AUTO default; wiki documentation |
| 14 | `3b1568d`, `b8f4f52` | release-readiness passes (review findings, DMAP format integration, warnings, includes) |

---

## 4. How it was measured

### 4.1 Ground truth

| | BlendedMVS (10 scene-levels) | ETH3D high-res (18 scene-levels) |
|---|---|---|
| scenes | 5 validation scenes × L0,L1 | courtyard, delivery_area, facade, meadow, office, pipes × L1,L2,L3 |
| GT geometry | textured mesh (`textured_mesh/tile_*.obj`) | registered laser scans (`dslr_scan_eval/`) |
| GT per-view depth | rendered depth (`rendered_depth_maps/*.pfm`) | laser-scan depth, on the **distorted** grid |
| poses | dataset-provided | COLMAP `dslr_calibration_undistorted` |

Per-view GT depth is **consumed, not re-derived** — both datasets ship it. The one transform applied
is for ETH3D, whose GT depth is on the distorted 6048×4032 sensor grid while OpenMVS reconstructs on
the undistorted grid: for each undistorted pixel the pinhole ray is forward-distorted through the
camera's `THIN_PRISM_FISHEYE` model and the distorted GT depth nearest-sampled there (verified to
~0 % median error against the raw scan). Depth convention is camera-frame **Z**, not ray distance.
This trap is worth remembering: *ETH3D's GT depth maps do not pair with its undistorted images.*

### 4.2 The paired protocol

Per scene-level: estimate depth-maps with raw confidence → **snapshot to `raw_dmaps/`** → adjust
confidence **in place** (depth untouched) → fuse at several `fFusePriorWeight` values → evaluate.
Because the adjust rewrites only the confidence channel, raw and adjusted sit on the *identical* set
of GT-labelled pixels, so raw-vs-adjusted is a clean comparison with no shared threshold to pick. A
pixel is a GT inlier iff `|d_est − d_gt| ≤ 1 %·d_gt`.

### 4.3 The metrics — and why ROC alone misleads

Treat the confidence as a retention score (keep iff `conf ≥ t`). Among kept pixels:
**contamination** = kept_outliers / kept_total (= 1 − precision); **completeness** = kept_inliers /
all_inliers (= recall). Sweeping `t` traces a frontier, evaluated only at distinct-confidence group
boundaries (a threshold *between* tied confidences is not a realizable operating point), and two
duals are read off it: *completeness kept at a contamination budget* and *contamination admitted at a
completeness target*. Both are invariant to monotone rescaling — which recalibration inevitably
applies — so they compare fairly where a fixed-threshold P/R cannot.

This mattered: the original headline metric (P/R at the fusion gate `t = 0.1`) was **uninformative**
— that gate keeps essentially everything, so recall ≈ 1.0 on every view. And ROC-AUC, being a pure
ranking score, improved on *every* scene while hiding a real regression at the high-completeness tail
(§ 6.1). Pick metrics in the units the consumer operates in.

Fusion is graded on the 3D cloud instead: completeness at tolerance (fraction of GT surface samples
with a reconstructed point within tol) and gross-outlier fraction (reconstructed points with no GT
surface within a much looser tol), via the official `ETH3DMultiViewEvaluation` for ETH3D and
area-uniform mesh sampling + exact grid nearest-neighbour for BlendedMVS. Tolerances are **not
cross-dataset comparable** (BlendedMVS scales with the bbox diagonal, ETH3D is fixed metres).

### 4.4 The offline sweep

Sweeping seven parameters through the C++ pipeline would have been impossibly slow, so the per-pixel
*features* (`K`, `V`, `Pconf`, `pGeo`, `photo`) were exported once per (scene, gate-mode, margin) and
the posterior recomputed offline in NumPy for any parameter set. The offline replica was verified
**byte-exact**: with the winning parameters compiled in, the C++ pipeline reproduced the
offline-predicted per-scene ROC to **0.0000** on all 6 deterministic ETH3D scene-levels. That
exactness is what made a 2430-combination full-grid sweep (not coordinate descent — no local-optimum
risk) affordable.

---

## 5. Verification of the GPU port

CUDA PatchMatch is **nondeterministic** (curand: ~71 % of depth pixels differ between two runs of the
same scene), so a naive separate-run confmap diff measures *geometry* noise, not the confidence code.
Parity was therefore established by isolating the backend:

1. **Identical-input harness** — run the GPU kernels on the exact projections and maps the CPU sweep
   just used, in one process: mean |Δconf| ≈ 1e-6, GT ROC-AUC **identical to 4 decimals**. The
   residual max |Δconf| ≈ 0.1–0.27 on a minority of gate-boundary pixels is float-vs-double `exp`
   tipping a discrete decision — unbiased, zero aggregate effect.
2. **Real pipeline, isolate by mechanism** — GPU-fused vs CPU-*integrated* (never vs
   CPU-*standalone*, which uses a different neighbour set and would show a ~0.03 gap that is not a
   GPU effect): |ΔROC| = **0.000026** for the fused kernel (gate was ≤ 0.005; GPU run-to-run noise
   alone is 0.0005).
3. **CPU refactor onto the shared header**: byte-identical, 0 of 5.6 M pixels differ.
4. `-DOpenMVS_USE_CUDA=OFF` builds and links clean, and the `.cu` is excluded by the CMake glob.

The neighbour-depth texture reuse needed its own argument: the texture is linear-filtered, but every
confidence read is an exact texel-centre fetch (`x+0.5`) whose interpolation weight is exactly 0, so
it is bit-identical to a linear-buffer read — *and* it is used per-neighbour only when the texture
holds the unresized map, otherwise the code falls back to uploading.

---

## 6. Results

### 6.1 Confidence quality — 28 scene-levels, current code (D2 codec, 2026-08-09)

| pool | ROC-AUC | compl @≤0.5 % contam | @≤1 % | @≤2 % | @≤5 % | contam @≥90 % compl | @≥95 % |
|---|---|---|---|---|---|---|---|
| **ALL (28)** | 0.844 → **0.926** | 18.7 → **44.9** % | 31.5 → **57.9** % | 48.3 → **70.9** % | 66.3 → **86.3** % | 10.4 → **7.1** % | 12.8 → 12.2 % |
| ETH3D (18) | 0.816 → 0.910 | 5.5 → 32.9 % | 17.9 → 49.0 % | 33.1 → 66.0 % | 59.0 → 84.7 % | 11.3 → 8.7 % | 14.1 → 13.1 % |
| BlendedMVS (10) | 0.895 → 0.956 | 45.3 → 66.4 % | 58.6 → 73.9 % | 78.6 → 79.7 % | 79.5 → 89.2 % | 8.7 → 4.1 % | 10.5 → 10.5 % |

- ROC-AUC improves on **28/28** scene-levels (mean +0.082; largest +0.20 on facade L3).
- Completeness at a ≤1 % contamination budget improves on **27/27** comparable scene-levels, mean
  **+28.5 pp** — at a fixed quality budget the adjusted confidence keeps roughly twice the surface.
- At raw confidence, **11 of 28 scene-levels have no usable ≤1 % operating point at all** (0.0–0.2 %
  completeness: the raw map's very highest confidences are already >1 % contaminated, so no clean
  subset exists at any threshold). Recalibration gives 8 of those a real one — facade L2 0 → 87.5 %,
  courtyard L3 0.1 → 63.2 %, pipes L1 0 → 53.3 %. Turning a confidence map that *cannot gate at all*
  into one that can is the single strongest argument for the feature.
- **Honest limits.** At extreme completeness targets (≥99 %) the frontier is essentially unchanged —
  recalibration reorders the middle of the ranking, not the deepest tail. And a few hard indoor /
  low-texture levels regress at ≥95 % completeness (office L2 −11 pp, office L1 −7.6 pp, meadow L2
  −3.7 pp, pipes L1 −3.5 pp) while still improving ROC and every ≤1–2 % budget point. That is
  outside the regime the feature serves, but it is real.

Also verified in the same run: the D2 quantized codec (confidence unorm8, depth float16) causes **no
measurable regression** — mean ROC raw ±0.0000, adjusted −0.0015, i.e. inside estimation noise.

### 6.2 The calibration sweep (10 scene-levels, 3.49 M labelled pixels, 2430 combinations)

| knob | pre-GT value | GT-calibrated |
|---|---|---|
| soft gates | off | **on** |
| violation weight | 0 (off) | **2.0** |
| violation margin | 3 | **2** |
| prior strength | 1.0 | **2.0** |
| confirm tau | 2.0 | **1.5** |
| photo floor | 0.5 | **0.7** |
| conf floor | 0.5 | **0.03** |

Pooled real-GT ROC-AUC **0.9463 → 0.9598** (+0.0135); every one of the 10 swept scene-levels
improved (min +0.0046, against a −0.005 guard). *(This pooling is pixel-pooled over 10 scene-levels
and is not the same statistic as the 0.844 → 0.926 macro-average over 28 in § 6.1 — do not mix
them.)* The **soft-gate flip is the dominant lever**: the best hard-gate combination reaches only
+0.0048, so the continuous weights contribute the majority of the gain. Selection maximized pooled
ROC subject to no scene-level dropping more than 0.005 and pooled precision at the fusion gate not
worsening. A per-resolution check found the global winner within 0.0005 of each regime's own best, so
one global setting ships.

### 6.3 Fusion few-view rescue (`fFusePriorWeight`)

| pool | w2 completeness gain | w3 gain | w2 gross added | w3 gross added | over +0.05 pp budget |
|---|---|---|---|---|---|
| ALL (28) | +2.6 / +2.4 / +6.4 pp | +6.5 / +6.9 / +13.8 pp | +0.07 pp | +0.17 pp | 2/28 (w2), 11/28 (w3) |
| ETH3D (18) | +3.3 / +3.0 / +6.4 pp | +8.0 / +7.4 / +13.8 pp | +0.01 pp | +0.04 pp | 0/18, 6/18 |
| BlendedMVS (10) | +1.3 / +0.8 / +4.0 pp | +3.8 / +2.6 / +10.0 pp | +0.16 pp | +0.40 pp | 2/10, 5/10 |

(mean / median / max). `w = 1` is a structural **no-op**: the binding gate is `nMinPixelsFuse = 5`
and a typical few-view cluster has ~2 fused pixels, so the virtual support has to reach ~3 before it
bridges the gate — the response is strongly nonlinear with a useful range of w ≈ 2–5.

**Decision: default stays w3.** The standard pipeline meshes after densifying, and the mesh step
cleans the few extra gross outliers while benefiting from the extra true points. Use
`--fusion-prior-weight 2` when the dense point cloud itself is the final output. (An earlier
recommendation was w2, on a strict per-scene outlier budget; the pipeline-level argument superseded
it. Both are defensible — the trade is documented in the CLI help.)

### 6.4 Speed

| stage | 0.29 MP (meadow L3) | 6.1 MP (courtyard L1) |
|---|---|---|
| fused GPU (+ tex2D reuse) | **3.28 ms/map** | **60.0 ms/map** |
| fused GPU (upload neighbour depth) | 3.59 ms/map | 65.6 ms/map |
| epilogue GPU re-upload | ~5.0 ms/map | 75.0 ms/map |
| CPU integrated sweep | 93.3 ms/map | ~2364 ms/map |

≈ **39× faster than the CPU sweep at 6 MP.** The original ≤50 ms/view target is met at reduced
resolutions and missed by 20 % at full 6 MP. The CPU side got ~1.3–2.5× faster over the arc (fused
single-precision projection, SSE neighbour-outer sweep, cached shared prior) but never approached the
aspirational 10× — an ablation showed the ceiling is ~2.6× (≈4.5× even with an infinitely fast sweep)
because the confirmation sweep is 75–80 % of the cost and is **memory-bandwidth bound**.

---

## 7. What we tried that did not ship

**7.1 MapAnything pseudo-GT — retired for tuning.** Before real GT was integrated, all confidence and
fusion-rescue tuning was guided by pseudo-GT clouds built from a monocular/multi-view foundation
model (MapAnything), voxel-fused with a multi-view gate. A dedicated study measured its error floor
against real GT and it was **larger than the effects being tuned**:

| | courtyard L2 | office L2 | bmvs_5a640093 L1 |
|---|---|---|---|
| median relative depth error (after per-view scale alignment) | 3.3 % | 6.3 % | 3.8 % |
| fraction > 10 % | 11.0 % | 37.3 % | 19.0 % |
| pseudo-GT cloud accuracy (loosest tier) | 23.5 % | 50.8 % | 13.9 % |
| pseudo-GT cloud gross-outlier rate | 34.8 % | 17.4 % | 61.7 % |
| **the real reconstruction it was grading**, same tolerance | 99.4 % / 0.26 % | 98.6 % / 0.70 % | 99.4 % / 0.14 % |

The grader was **one to two orders of magnitude less reliable than the thing it graded**, while the
decisions at stake were single-digit-percentage-point completeness deltas and sub-percentage-point
outlier deltas. Note the asymmetry: *completeness* against the pseudo-GT was reasonable (51–92 % — the
witness does find roughly the right surface) while *accuracy* was uniformly poor — exactly what noisy
per-pixel depth fused without correction produces. Also worth knowing: sparse-depth conditioning
helped but did not save it (office, *with* real sparse points, was the worst of the three), and
per-view scale alignment cannot fix cross-view *shape* disagreement, which is what a multi-view gate
depends on.

What survived the audit: the **directional** claims. "w3 ≥ w0 on every scene and every tolerance"
replicated exactly under real GT (28/28), and the confidence-recalibration conclusion was never
MapAnything-dependent (it was always graded against fusion-geometry labels, then real GT). What did
not: the magnitudes — the pseudo-GT's "adds ~no outliers" understated the real cost.

**7.2 MoGe / MoGe-2 completeness judges** — earlier mono-model "is this completeness real?" judges
(orientation-aware variants, cross-view consensus, capped normal gates). Same fate for the same
reason; superseded by real GT.

**7.3 Hard gates.** The original pass/fail gates lose the majority of the achievable gain (§ 6.2).
Continuous weights are the single most important modelling choice here.

**7.4 Integrated (in-estimation) CPU mode as the default.** It saves no I/O (measured: identical dmap
opens) and runs the sweep on only `nDenseWorkers` (= `nPatchMatchCUDAInstances`, default 4) threads
versus up to `nMaxThreads` for the standalone phase. Total wall time: wins on small/few-image scenes
(−3 s, −9 s), loses on large ones (+8 s, +42 s, +64 s). Even a perfectly decoupled thread pool only
*ties* standalone, because the work is identical. It became the right default only on the **GPU**,
where the kernels are nearly free.

**7.5 Raising `nPatchMatchCUDAInstances` to feed the inline sweep.** 4 → 16 workers cut total sweep
CPU by only 12 % (memory-bound), while oversubscribing the single GPU made raw estimation **26 %
slower**. The knob that looks like the lever is the wrong lever: it controls GPU dispatch concurrency
too.

**7.6 Second-chance fusion pass** (`bFuseSecondChance`) — a second pass to re-admit prior-supported
discarded seeds. Proven redundant given the `fFusePriorWeight` rescue; shipped off, then removed.

**7.7 `CONF_FLOOR = 0.5`.** The sweep cut it 16× purely on ROC-flatness (ROC is identical across
0.03/0.05/0.1), which does **not** test the floor's actual job — keeping genuinely-confirmed but
few-view inliers above the fusion gate. Recall at the gate rose (0.9217 → 0.9607), which is
reassuring but not a completeness proof. `0.05`/`0.1` are ROC-identical fallbacks if a downstream
completeness regression ever appears.

**7.8 A test-only determinism hash** in `Scene::EstimateNeighborViewsPointCloud` (BlendedMVS-style
scenes with no sparse cloud pick neighbours through a time-seeded RNG, which made fusion output
vary). It was reverted before release: the randomness is statistically fine, determinism is a *bench*
concern, and shipped code should not carry a test crutch. Consequence: recorded BlendedMVS fusion
numbers were taken with the hash in place, so re-runs carry ~5 % run-to-run spread in point counts.
Seed locally in a throwaway patch when re-benchmarking. A related misdiagnosis is recorded in
`92ca745`: the nondeterminism was first blamed on an OpenMP race, and it was the RNG.

**7.9 Exposing the shape constants as knobs** — they are one operating point, not seven independent
dials (§ 1).

**7.10 Exporting the estimator's geometric-consistency score as a fourth confidence feature** — it is
a transient local in `ScorePixelImage`, folded into the NCC score and never persisted; plumbing a new
per-pixel buffer through `DepthEstimator` *and* PatchMatchCUDA was judged not worth it.

---

## 8. Learnings worth carrying forward

**8.1 Grade a signal in the units its consumer operates in.** ROC-AUC improved on 28/28 scene-levels
while a real regression hid at the high-completeness tail. The contamination/completeness frontier —
which is what a thresholding consumer actually experiences — exposed it. A single aggregate number
almost always hides the operating point you care about.

**8.2 Pair your comparisons.** Because the adjust rewrites only the confidence channel, snapshotting
the raw dmaps first makes raw-vs-adjusted a same-pixels comparison. Without that, differences in
depth estimation swamp the effect.

**8.3 Never let a grader be less reliable than what it grades** (§ 7.1). Quantify the grader's own
error floor *first*, and compare it to the effect size you intend to resolve. Directional claims can
survive a noisy witness; magnitudes cannot.

**8.4 Make the reference implementation the only implementation.** Extracting the per-pixel math into
one `__host__ __device__` header before writing the kernel is what made "the GPU computes the same
thing" checkable at all (byte-identical CPU refactor, then |ΔROC| = 2.6e-5 for the kernel). Two copies
of a formula drift — as this codebase already learned with the dmap codec.

**8.5 Isolate nondeterminism before claiming parity.** With ~71 % of depth pixels differing run to
run, any two-run comparison measures the RNG. Compare backends on identical inputs in one process, or
compare mechanisms that share a neighbour set.

**8.6 A recalibration changes the *scale* of a number, and every downstream consumer that hardcoded
the old scale breaks.** The mesh step persisted `pointWeights = 1/(max(1−conf,0.03)·depth²)` — a
fusion-internal averaging kernel whose magnitude depends on the scene's length unit *and* on the
confidence calibration. Honest (≈2× lower) confidences pushed the weighted graph-cut past absolute
free-space constants tuned for weight ≈ 1 and collapsed mesh coverage (108 k points → 8 337 faces).
The fix was not to renormalize but to persist the plain dimensionless `[0,1]` confidence, matching the
interface's own semantics. When you recalibrate, audit every consumer of the old numeric range.

**8.7 An offline replica of the inner formula turns an intractable sweep into an afternoon** — and it
is only trustworthy if you prove it byte-exact against the shipping code (we did: 0.0000 drift).

**8.8 Optimize what dominates.** The confirmation sweep is 75–80 % of the cost and memory-bandwidth
bound; caching the prior (~20–25 %) could not move the needle, and more threads barely helped. Measure
the share before optimizing the part that is pleasant to optimize.

**8.9 Improving a feature reveals latent bugs downstream.** Denser, better-weighted clouds exposed an
infinite loop in upstream `Mesh::Clean` spike removal and a `DMapCache` accounting underflow. Budget
for that.

**8.10 Cross-process state needs on-disk state.** An in-process "already adjusted" flag is not enough
the moment a user splits estimate / adjust / fuse into separate invocations; the `CONF_ADJUSTED`
header bit is what makes the guard real.

---

## 9. If a better foundation model than MapAnything appears

The pseudo-GT idea is not dead — it was *retired at a measured error floor*. A newer model earns its
way back in by passing the same audit, in this order:

1. **Reproduce the audit.** For at least three deliberately diverse scenes with real GT (outdoor
   wide-baseline, textureless indoor, small-object / no-sparse) measure: per-view median relative
   depth error after per-view scale alignment against MVS-confident pixels, the >1/3/10 % tails, and
   then the fused pseudo-GT cloud's accuracy and gross-outlier rate against real GT — alongside the
   real reconstruction's numbers on the *same* GT, as the reference. The old study's numbers (§ 7.1)
   are the baseline to beat.
2. **Apply the admission rule.** A witness may drive a quantitative decision only if its error floor
   is at least an order of magnitude below the effect size under test. For the decisions on this
   branch (single-digit-pp completeness, sub-pp outlier deltas) that means roughly **≤0.3 % median
   relative depth error and ≤2 % cloud gross-outlier rate** — a demanding bar, and the honest one.
   Anything short of it stays a *directional* tool: "did completeness go up or down" on scenes with
   no GT, plus visual floater inspection.
3. **Don't re-litigate what real GT already answers.** ETH3D + BlendedMVS now cover the scene types
   the pseudo-GT work targeted. A new model's value is on **scene types real GT does not cover** —
   Tanks&Temples-style captures, client scenes, aerial, indoor scanning rigs — and as a *cheap
   pre-screen* before spending a 7-hour GT sweep.
4. **Reuse the scaffolding.** `MapAnyInfer*.py` (inference incl. a no-sparse variant),
   `MvsSparseDepth.py` (SfM sparse depth for conditioning), `MapAnyVoxelFuse.py` (per-view scale
   anchoring + multi-view-gated voxel fusion), `MapAnyVsGT.py` (the audit itself) and
   `CompletenessGT.py` are all recoverable per § 11 — the pipeline is model-agnostic apart from the
   inference call.
5. **Where a good model could genuinely change the design:** as a *fourth evidence source* in the
   posterior (a learned prior alongside `pGeo`), or as a completeness witness that lets the
   fusion-rescue weight be tuned per scene instead of globally. Both need the § 4 harness and the
   § 9.2 error bars before they mean anything.

---

## 10. Open threads

1. **The ≥95 %-completeness tail regression** on textureless indoor scenes (§ 6.1). Not the operating
   point the feature serves, but unexplained. Suspicion: the confirmation term over-rewards
   agreement among neighbours that are *all* wrong in the same way on repetitive/low-texture surfaces.
2. **`CONF_FLOOR = 0.03` has no completeness proof** (§ 7.7) — only ROC-flatness plus a recall
   improvement at the gate. A direct few-view completeness A/B against 0.5 would close it.
3. **CPU default is off** (AUTO). It costs a full sweep; the standalone path could be made much
   cheaper (the sweep is memory-bound, so the win is in data layout/blocking, not threads).
4. **Full-resolution GPU cost** is 60 ms/map, 20 % over the original ≤50 ms target. The remaining
   upload is neighbour conf + normal (~16n B/px) — inherent unless PatchMatch itself starts keeping
   them resident.
5. **Per-worker GPU allocation × workers can still OOM on very large images**; mitigated by the
   resident-buffer reuse, with the CPU fallback as the net.
6. **Integrated vs standalone diverge at low view counts** (|ΔROC| −0.013 on a 15-image scene) because
   the estimation-time neighbour set differs from the fusion-time one. Harmless today (the GPU path is
   the integrated one and is what ships) but it means the two paths are not interchangeable evidence.
7. **The offline sweep harness no longer exists in the tree** — re-running the calibration needs the
   `--export-conf-features` export path re-added (see `SweepConfParams.py` for the exact feature
   contract) *and* the DEFVARs temporarily restored, since the constants are now compile-time.
8. **A learned posterior** (features → probability, trained on GT labels) was never tried. The feature
   export plus 3.5 M labelled pixels per sweep is most of the dataset work already done.

---

## 11. Recovering the research artifacts

Everything below was removed from the shipping tree in **`b70d386`** ("dense: prune confidence
research scaffolding") and is intact in its parent commit. Nothing is lost:

```bash
git show b70d386^:gt_bench/RESULTS_AND_METHODOLOGY.md   # methodology + all July results in one file
git show b70d386^:gt_bench/GT_ASSESSMENT_D2.md          # the authoritative 28-scene-level assessment
git show b70d386^:gt_bench/SWEEP_GT.md                  # the calibration sweep (§ 6.2) in full
git show b70d386^:gt_bench/CONFIDENCE_GT_OPERATING.md   # the frontier metrics, per scene
git show b70d386^:gt_bench/VERDICT_MAPANYTHING.md       # the pseudo-GT audit (§ 7.1, § 9)
git show b70d386^:gt_bench/CUDA_CONFIDENCE.md           # GPU port: control model, parity, timing
git show b70d386^:gt_bench/AB_INTEGRATED.md             # integrated-vs-standalone A/B (§ 7.4-7.5)
git show b70d386^:gt_bench/W2_W3_RECHECK.md             # fusion weight decision
git show b70d386^:gt_bench/WEIGHT_SWEEP_GT.md           # w ∈ {0..5} sweep with the knee
git show b70d386^:gt_bench/TIMING_AFTER_WS2A.md         # CPU optimization gates, honestly failed
git show b70d386^:gt_bench/README.md                    # bench harness: datasets, layout, traps
git show b70d386^:gt_bench/BASELINE_2026-07.md          # pre-optimization reference snapshot
git show b70d386^:gt_bench/FINAL_2026-07.md             # ETH3D holdout run
git show 858942f:gt_bench/T14_HANDOFF.md                # the GPU-port handoff, with §4 invariant

# harness + tooling (same commit): run_scene.sh, run_confop.sh, eth3d_eval.sh, import_scenes.sh,
# aggregate_{gt,fuse,confop}.py, compare_features.py, replicate_equivalence.py, scenes_*.txt
git show b70d386^:gt_bench/run_scene.sh
# python: EvalConfidence.py, EvalFusionGT.py, GtUtils.py, CompletenessGT.py, SweepConfParams.py,
#         MapAny*.py, Moge*.py, MvsSparseDepth.py, RenderMonoPreview.py + tests/test_*.py
git show b70d386^:scripts/python/GtUtils.py
git show b70d386^ --stat                                # the full inventory in one listing
```

The whole research history is also kept on the local branch **`archive/confidence-research-history`**
(tip `858942f`), which has all of the above checked out at their last live state. It is not pushed to
any remote; push it if this record should outlive the local clone.

**Environment notes that will otherwise cost a day.** GT data belongs on a large mount, never in the
repo or on the root filesystem (the benchmark is ~40 GB extracted, and one full sweep is ~7 h on a
single A100). `ETH3DMultiViewEvaluation` must be built with `-std=c++17` (its CMakeLists hardcodes
C++11, which fails against PCL ≥ 1.14) and it links PCL dynamically — a system update that removes
`libpcl-*1.14` makes every ETH3D fusion eval fail with exit 127. The tool applies scan alignment to
the *ground truth* only, not to `--reconstruction_ply_path`; that is correct for real use and a trap
for self-tests. A new `.cu` file needs a CMake **reconfigure** (`FILE(GLOB)` runs at configure time).

---

## 12. Reproduce

```bash
# GPU, default: recalibration fused into the last geometric-consistency iteration
DensifyPointCloud scene.mvs -w WD --geometric-iters 2 -v 2
#   -> "Integrated confidence recalibration (GPU): … ms/map avg"

# force it on (CPU estimation, or re-adjust existing dmaps): today's bit is 8, not 4
DensifyPointCloud scene.mvs -w WD --postprocess-dmaps 8 --geometric-iters 0

# force the CPU implementation while CUDA estimates (parity work):
#   dense config file containing:  Estimate Confidence CUDA = 0

# turn it off entirely
DensifyPointCloud scene.mvs -w WD --postprocess-dmaps 0

# dense cloud is the final output (fewer outliers, slightly less completeness)
DensifyPointCloud scene.mvs -w WD --fusion-prior-weight 2

# inspect a dmap, including whether its confidence is already recalibrated
python3 -c "from MvsUtils import loadDMAP; d=loadDMAP('depth0000.dmap'); print(d['conf_adjusted'])"
```

For the full GT benchmark, recover `gt_bench/run_scene.sh` (§ 11) and follow its README: per
scene-level it estimates, snapshots the raw dmaps, adjusts, fuses at several weights and evaluates,
idempotently per stage.
