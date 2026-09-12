# Depth-Map Confidence Recalibration

## 1. Purpose and scope

Every depth estimate carries a confidence in `[0,1]`, stored in the `.dmap` next to the depth and
used to gate fusion (`libs/MVS/SceneDensify.cpp:DenseFuseDepthMaps`), to weight a point's per-view
observation, and to weight visibility in the mesh step. By default that number starts as a
photometric score (`1 − NCC`), which answers "how well did this patch match?" — a question only
loosely related to "is this depth correct?": a repetitive facade or a textureless wall can match
well and still be wrong.

The recalibration (`OPTDENSE::ADJUST_CONFIDENCE`) replaces the photometric score with a posterior
that predicts whether a depth will survive fusion as an inlier, combining an intra-map plane-fit
prior with continuous multi-view confirmation and a free-space-violation penalty.

**Code.** `libs/MVS/ConfidenceRefine.h` (shared host/device per-pixel math), `libs/MVS/ConfidenceCUDA.{h,cu}`
(GPU kernels + launchers), `libs/MVS/SceneDensify.cpp` (`DepthMapsData::ComputeIntraMapPrior`,
`GetIntraMapPrior`, `AdjustConfidence` (two overloads), `AdjustConfidenceCUDA`,
`AdjustConfidenceSweep`), `libs/MVS/PatchMatchCUDA.cpp` (fused launch inside
`PatchMatch::EstimateDepthMap`), `libs/MVS/DMapCache.{h,cpp}` (phase-lifetime depth-map cache used
by the standalone sweep), `libs/MVS/DepthMap.h` (`OPTDENSE::DepthFlags`, `DepthData::bConfAdjusted`),
`libs/MVS/Interface.h` (`HeaderDepthDataRaw::CONF_ADJUSTED`).

Out of scope: fusion's own join/keep gates and `fFusePriorWeight` (see `DepthMapFusion.md`, which
reuses the same intra-map prior as virtual support); the mesh step's consumption of the stored
per-view weight.

## 2. Algorithm as implemented

Per reference pixel, with `Kf`/`Pconf` accumulated over confirming neighbour views
(`ConfRefine::Posterior`, `libs/MVS/ConfidenceRefine.h`):

```
gate      = 1 − exp(−(Kf + PRIOR_GATE·pGeo) / CONFIRM_TAU)
posterior = (PRIOR_STRENGTH·pGeo + Pconf) / (PRIOR_STRENGTH + Pconf + VIOLATION_W·V)
photo     = PHOTO_FLOOR + (1 − PHOTO_FLOOR)·confPhoto
conf      = clamp01(posterior · gate · photo)
if Kf ≥ 1: conf = max(conf, CONF_FLOOR · confPhoto)        // anti-cascade floor
```

- **`pGeo` — intra-map geometric prior** (`DepthMapsData::ComputeIntraMapPrior`,
  `SceneDensify.cpp:1183`): fits a local depth plane to the 3x3 neighbourhood of each pixel
  (depth-similar neighbours only), scores the pixel by the plane-fit residual (`Pplane`), an
  inlier-count soft quorum (`gate`, ~4 inliers), and — when a normal map is available — the
  agreement between the plane-implied normal and the estimated normal (`Pnorm`). A correct surface
  is locally coherent in both; a photometric mismatch usually is not.
- **`Kf`, `Pconf` — multi-view confirmation**: the pixel is projected into each selected neighbour
  view and scored against that view's own depth/normal/confidence through four continuous weights —
  `SoftDepthW` (Gaussian relative-depth agreement, gate 1), `SoftReprojW` (forward-backward
  reprojection residual, gate 2), a plain normal dot product (gate 3), `SoftConfW` (smoothstep on
  the neighbour's own confidence around `minConfidence`, gate 4). Each neighbour contributes a
  fractional vote (`Kf += w`, `Pconf += w·cN`), so agreement degrades smoothly instead of a hard
  cliff.
- **`V` — free-space violations**: when a neighbour's own measured depth lies more than
  `VIOLATION_MARGIN·thDepth` behind the point along the same ray, that neighbour's line of sight
  passes through where the point claims to be — direct negative evidence, diluting the posterior's
  denominator.

`ConfRefine::Params` (`ConfidenceRefine.h`) carries the shape constants plus the runtime gate
thresholds shared with fusion (`minConfidence = 1 − fNCCThresholdKeep`, `thReproj =
fDepthReprojectionErrorThreshold`, `thDepth = fDepthDiffThreshold`), so the recalibrated confidence
predicts what `DenseFuseDepthMaps` will actually accept.

### Raw-neighbour-confidence invariant

`cN` (gate 4) and `Pconf` must be the neighbour's **raw** photometric confidence, never its
already-adjusted one — otherwise geometric agreement would be double-counted (the neighbour's
posterior already folded in its own neighbours, including this pixel) and the result would depend
on worker processing order. The design is a single Jacobi pass: every pixel's adjusted confidence
is a function of raw neighbour confidences only. This is enforced structurally:

- **Fused / epilogue**: neighbour normal/confidence maps are loaded by `InitViews` only on the last
  geometric-consistency iteration (`loadDepthMaps == 2`) into `DepthData::images[].confMap`, the
  *previous* iteration's snapshot; if that snapshot's own dmap already carries `CONF_ADJUSTED`,
  `InitViews` drops it instead of loading it (`SceneDensify.cpp:438-445`), so a re-run over
  already-adjusted dmaps cannot feed adjusted values back in as evidence — the neighbour then gates
  as "no confidence" (neutral), matching `ConfNeighborHost::conf == null`.
- **Standalone**: `AdjustConfidence(DepthData&, const IIndexArr&)` reads neighbours from the shared
  `arrDepthData[]`, which other references may be concurrently adjusting; the result is parked in
  `DepthData::confMapAdjusted` and only swapped into `confMap` by the `EVT_ADJUSTDEPTHMAP` handler
  after a whole-phase semaphore barrier confirms every reference has finished reading
  (`bDeferSwap=true` in `AdjustConfidenceSweep`).
- **Integrated CPU epilogue**: `AdjustConfidence(DepthData&)` reads this reference's own private
  `images[]` snapshot (never the shared, live state), so it writes `confMap` immediately
  (`bDeferSwap=false`).

### Where it runs

`OPTDENSE::nOptimize` (`--postprocess-dmaps`) bits: `1` `REMOVE_SPECKLES`, `2` `FILL_GAPS`, `4`
`ADJUST_CONFIDENCE_AUTO` (default), `8` `ADJUST_CONFIDENCE` (force on). `Scene::ComputeDepthMaps`
resolves `AUTO` once per call, once the estimation backend is known
(`SceneDensify.cpp:3218-3242`), to `ADJUST_CONFIDENCE` only when **all** of: a CUDA PatchMatch pool
exists, `OPTDENSE::bEstimateConfidenceCUDA` is true, `nFusionMode >= 0`, and
`nEstimationGeometricIters > 0` — otherwise it resolves to off. The resolution is scoped to that
call (restored on return), so a later `DenseReconstruction` in the same process re-resolves `AUTO`
for its own backend. Metal and CPU-only builds therefore always resolve `AUTO` to off.

When `ADJUST_CONFIDENCE` is set (auto-resolved or forced), three paths exist, tried in this order:

1. **Fused in-estimation** (`PatchMatch::EstimateDepthMap`, `PatchMatchCUDA.cpp:462-490`): fires
   only when `scaleNumber == 0`, `params.bGeomConsistency`, this is the last geometric-consistency
   iteration, and `nOptimize & OPTIMIZE` (speckle/gap filters) is **not** set — those filters run
   after estimation and would change the depth the confidence was computed from, so the fused path
   defers to the epilogue whenever they are enabled. Reads the reference depth/normal/cost straight
   from the PatchMatch instance's resident device buffers (`cudaDepthNormalEstimates`,
   `cudaDepthNormalCosts`); only the neighbours' raw previous-iteration maps are uploaded (or read
   from a resident depth texture when its resolution matches). On success `confMap` is written
   directly, skipping the cost→confidence conversion the caller would otherwise apply.
2. **Epilogue GPU re-upload** (`DepthMapsData::AdjustConfidenceCUDA`, called from the
   `EVT_SAVEDEPTHMAP` handler): runs when the fused launch did not run or failed (`bConfAdjusted`
   still false) — e.g. speckle/gap filters on, non-contiguous maps, or a CUDA error. Re-uploads the
   reference and neighbour maps and runs `MVS::CUDA::RunConfidenceCUDA`.
3. **Standalone CPU sweep** (`DepthMapsData::AdjustConfidence(DepthData&, const IIndexArr&)`,
   `--postprocess-dmaps 8`, CPU/Metal estimation, `--geometric-iters 0`, or re-adjusting already-saved
   dmaps): its own phase (`EVT_FILTERDEPTHMAP` / `EVT_ADJUSTDEPTHMAP`), using up to 8 neighbours per
   reference (`numMaxNeighbors` in `SceneDensify.cpp:3722`) through a phase-lifetime `DMapCache`
   (`g_pAdjustDMapCache`) shared across the whole phase.

Within one reference view, a fallback chain applies: fused → epilogue GPU
(`AdjustConfidenceCUDA`) → epilogue CPU (`AdjustConfidence(DepthData&)`), each only attempted if the
previous one did not set `bConfAdjusted`. `bEstimateConfidenceCUDA = 0` (dense config file only,
not a CLI flag) forces the CPU implementation even when CUDA estimates.

**Double-adjust guards.** In-process: the standalone phase's `AdjustConfidence` overload returns
`false` immediately when `depthDataRef.bConfAdjusted` is already set. Cross-process: every
recalibrated dmap carries a `CONF_ADJUSTED` flag bit in its header
(`HeaderDepthDataRaw::CONF_ADJUSTED`, `libs/MVS/Interface.h`), loaded back into
`DepthData::bConfAdjusted` on read; the standalone phase checks it before adjusting, and `InitViews`
checks it before trusting a neighbour's confidence (see above).

## 3. Parameters and defaults

| CLI option | struct field | default | meaning |
|---|---|---|---|
| `--postprocess-dmaps` | `OPTDENSE::nOptimize` | `4` (`ADJUST_CONFIDENCE_AUTO`) | `0` disabled, `1` remove-speckles, `2` fill-gaps, `4` auto (on for CUDA estimation, off otherwise), `8` force on |
| *(dense config file only)* | `OPTDENSE::bEstimateConfidenceCUDA` | `true` | when CUDA estimates, run the recalibration on the GPU; `0` forces the CPU sweep |
| — (shared with fusion) | `OPTDENSE::fNCCThresholdKeep` | `0.9` | `minConfidence = 1 − this` is gate 4's centre and the fusion keep threshold |
| `--fusion-reprojection-threshold` | `OPTDENSE::fDepthReprojectionErrorThreshold` | `1.0` | gate 2 (`thReproj`) divisor; also fusion's join gate |
| `--fusion-depth-diff-threshold` | `OPTDENSE::fDepthDiffThreshold` | `0.01` | gate 1 / plane-fit band (`thDepth`) divisor; also fusion's join gate |
| — (compile-time) | `ConfRefine::PRIOR_STRENGTH` | `2.0` | intra-map prior weight, as Beta pseudo-counts |
| — (compile-time) | `ConfRefine::CONFIRM_TAU` | `1.5` | softness of the confirmation gate |
| — (compile-time) | `ConfRefine::PRIOR_GATE` | `0.3` | prior's share of the gate when no neighbour confirms |
| — (compile-time) | `ConfRefine::PHOTO_FLOOR` | `0.7` | minimum multiplicative photometric weight |
| — (compile-time) | `ConfRefine::CONF_FLOOR` | `0.03` | anti-cascade floor (× photometric conf) once `Kf ≥ 1` |
| — (compile-time) | `ConfRefine::VIOLATION_W` | `2.0` | denominator weight of the violation count |
| — (compile-time) | `ConfRefine::VIOLATION_MARGIN` | `2.0` | how far behind (in units of `thDepth`) counts as a violation |

The compile-time constants live in `ConfidenceRefine.h` and are deliberately not runtime knobs: they
are one jointly ground-truth-calibrated operating point (BlendedMVS + ETH3D, 28 scene-levels, full-grid
sweep) — one global setting won on every scene-level, and moving one without re-sweeping the others
degrades the result. The gate *thresholds* (`fNCCThresholdKeep`, `fDepthReprojectionErrorThreshold`,
`fDepthDiffThreshold`) stay runtime because fusion shares them.

## 4. Invariants and constraints

- **Single Jacobi pass**: adjusted confidence is a pure function of raw neighbour data; never cache
  or read adjusted confidence as a neighbour's evidence (see §2).
- **AUTO requires all of**: a non-empty CUDA PatchMatch pool, `bEstimateConfidenceCUDA`,
  `nFusionMode >= 0`, `nEstimationGeometricIters > 0`. Metal-only and CPU-only builds always resolve
  `AUTO` to off.
- **The fused path only fires on the last geometric-consistency iteration, at full resolution, with
  the speckle/gap filters off.** Any post-estimation filter forces the epilogue path instead, since
  the fused kernel reads depth/normal that is about to change.
- **Standalone neighbours are capped at 8** (`numMaxNeighbors`), unlike the fused/epilogue paths
  which use every neighbour `InitViews` loaded for geometric consistency.
- **CONF_ADJUSTED is sticky and cross-process**: once set, no path re-adjusts that view's confidence
  again, in this run or a later one loading the same dmap.
- Gate thresholds are clamped away from zero (`MAXF(x, 1e-6f)`) before use as divisors, both on the
  CPU (`AdjustConfidenceSweep`) and in the GPU parameter snapshot (`MakeConfRefineParams`), so a
  degenerate `fDepthDiffThreshold == 0` or `fDepthReprojectionErrorThreshold == 0` cannot produce a
  divide-by-zero.
- The CPU and GPU paths share the exact same per-pixel math (`ConfidenceRefine.h`, compiled under
  both the host compiler and `nvcc`); the GPU path differs only in using single-precision `expf`.

## 5. Validation of the shipped defaults

Pooled inlier/outlier ROC-AUC and the contamination/completeness frontier, raw photometric
confidence vs. the shipped recalibration, measured against real ground-truth depth (BlendedMVS +
ETH3D, 28 scene-levels; a GT inlier is `|d_est − d_gt| ≤ 1%·d_gt`):

| pool | ROC-AUC (raw → adjusted) | completeness @ ≤1% contamination | contamination @ ≥90% completeness |
|---|---|---|---|
| ALL (28 scene-levels) | 0.844 → 0.926 | 31.5% → 57.9% | 10.4% → 7.1% |
| ETH3D (18) | 0.816 → 0.910 | 17.9% → 49.0% | 11.3% → 8.7% |
| BlendedMVS (10) | 0.895 → 0.956 | 58.6% → 73.9% | 8.7% → 4.1% |

ROC-AUC improves on 28/28 scene-levels; completeness at a ≤1% contamination budget improves on
27/27 comparable scene-levels. 11/28 scene-levels have no usable ≤1% operating point at all on raw
confidence (the highest raw confidences are already over 1% contaminated); recalibration gives most
of those a real one. At extreme completeness targets (≥99%) the frontier is largely unchanged, and a
few textureless/repetitive indoor levels regress at ≥95% completeness even as every ≤1–2% budget
point and the ROC improve (see §7 open items).

## 6. Rejected alternatives

- **Hard pass/fail gates** — loses the majority of the achievable ROC gain; continuous weights are
  the dominant lever.
- **Exposing the shape constants as CLI/config knobs** — they are one jointly-calibrated operating
  point, not seven independent dials; moving one without re-sweeping the others degrades it.
- **`CONF_FLOOR = 0.5`** — the sweep favoured a much smaller floor on ROC-flatness grounds; a value
  that high does not test the floor's actual job of protecting genuinely-confirmed few-view inliers.
- **Integrated (in-estimation) CPU mode as the always-on default** — ties or loses against the
  standalone phase's greater thread parallelism on CPU; only wins on the GPU, where the kernels are
  nearly free.
- **Raising `nPatchMatchCUDAInstances` to feed the inline sweep** — memory-bandwidth bound, so more
  workers barely reduce sweep cost while oversubscribing the GPU slows raw estimation.
- **A test-only determinism hash in neighbour-view selection** — reverted; randomness there is
  statistically fine and a bench concern, not something shipped code should carry as a crutch.
- **Exporting the estimator's geometric-consistency score as a fourth confidence feature** — a
  transient local already folded into the NCC score; plumbing a new per-pixel buffer through both
  the CPU estimator and PatchMatchCUDA was judged not worth it.
- **Monocular-foundation-model pseudo-GT (and mono-model completeness judges) for tuning** — retired
  once real GT was available: the pseudo-GT's own error floor was one to two orders of magnitude
  larger than the effects being tuned.

## 7. Open items

- The ≥95%-completeness tail regression on a few textureless/repetitive indoor scene-levels (§5) is
  unexplained; suspected cause is the confirmation term over-rewarding neighbours that are all wrong
  in the same way on repetitive surfaces.
- `CONF_FLOOR = 0.03` has no direct few-view-completeness proof, only ROC-flatness plus an improved
  recall at the fusion gate.
- The CPU standalone path is off by default because it costs a separate full-resolution sweep; the
  sweep is memory-bandwidth bound, so a real speedup would need better data layout/blocking, not more
  threads.
- Per-worker GPU allocation times the worker count can still be a pressure point on very large images;
  mitigated by resident-buffer reuse, with the CPU fallback as the backstop.
- The standalone phase and the fused/epilogue paths use different neighbour sets (8-cap vs. the full
  estimation neighbourhood), so their outputs are not interchangeable evidence — most visible at low
  view counts.
- A learned posterior (features → probability, trained on GT labels) instead of the hand-derived
  formula has never been tried.
