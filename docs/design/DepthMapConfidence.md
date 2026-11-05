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

**Code.** `libs/MVS/ConfidenceRefine.h` (shared host/device per-pixel math), `libs/Common/DepthGeometry.h`
(the depth-plane fit and its normal, shared with depth-map estimation), `libs/MVS/ConfidenceCUDA.{h,cu}`
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

- **`pGeo` — intra-map geometric prior** (`DepthMapsData::ComputeIntraMapPrior`): fits a local
  depth plane to the 3x3 neighbourhood of each pixel (depth-similar neighbours only,
  `FitDepthGradient`; its normal is `NormalFromDepthGradient`), scores the
  pixel by the plane-fit residual (`Pplane`), an inlier-count soft quorum (`gate`, ~4 inliers), and
  — when a normal map is available — the agreement between the plane-implied normal and the
  estimated normal (`Pnorm`). A correct surface is locally coherent in both; a photometric mismatch
  usually is not.
- **`Kf`, `Pconf` — multi-view confirmation**: the pixel is projected into each selected neighbour
  view and scored against that view's own depth/normal/confidence through four continuous weights —
  `SoftDepthW` (Gaussian relative-depth agreement at the estimation noise, gate 1), `AngleW`
  (independence of the vote, `min(1, sin θ / sin 20°)` of the triangulation angle θ, gate 2), a plain
  normal dot product (gate 3), `SoftConfW` (smoothstep on the neighbour's own confidence around
  `minConfidence`, gate 4). Each neighbour contributes a fractional vote (`Kf += w`,
  `Pconf += w·cN`), so agreement degrades smoothly instead of a hard cliff.
- **`V` — free-space violations**: when a neighbour's own measured depth lies more than
  `VIOLATION_MARGIN·thDepth` behind the point along the same ray, that neighbour's line of sight
  passes through where the point claims to be — direct negative evidence, diluting the posterior's
  denominator.

`ConfRefine::Params` (`ConfidenceRefine.h`, built once by `MakeConfRefineParams` for both the CPU
sweep and the CUDA kernel) carries the shape constants, the calibrated confirmation tolerance
(`thDepth = CONFIRM_DEPTH`, also the unit of the free-space margin and of the prior's plane-fit band)
and gate 4's centre `minConfidence = 1 − fNCCThresholdKeep`, the estimation's floor on the neighbours'
photometric confidence. Fusion applies its own floor to the recalibrated result
(`ConfRefine::FUSE_MIN_CONF`, see `DepthMapFusion.md`).

**Why the confirmation tolerances are not fusion's.** A confirmation is evidence that the depth is
correct; fusion's join thresholds decide how finely agreeing pixels merge into points, and a pixel
that fails to join one cluster can still seed its own. Measured against laser ground truth on four
Tanks-and-Temples scenes (projecting sampled pixels into every neighbour depth-map):

- correct depths disagree with their neighbours by a *relative* depth that is constant across
  triangulation angles (median 0.06–0.15%, 90th percentile 0.3–1%), so a relative depth tolerance
  is the right form, and a 0.25–0.5% width ranks correct against wrong pixels best on every scene;
- the forward-backward reprojection residual is no independent test: along the epipolar line it is
  the same depth error scaled by `f·sin θ` (a correct pixel shows 0.4 px at θ < 3° and 1.4–2.8 px
  beyond 20°), across it only pixel rounding (0.25 px at every angle). A fixed-pixel gate on it
  therefore discards the most informative, wide-baseline confirmations;
- what θ does carry is independence: a narrow-baseline neighbour sees nearly the same image content
  and shares the reference's mistakes, so weighting votes by `sin θ` separates correct from wrong
  pixels better than any residual gate.

Per-pixel confirmation AUC (correct vs wrong pixel), Meetingroom / Caterpillar / Truck / Church:
fusion's join gates (depth 1% × reprojection 1 px) 0.770 / 0.800 / 0.765 / 0.845; depth 0.5% only
0.810 / 0.848 / 0.791 / 0.885; depth 0.5% × `AngleW` (shipped) 0.835 / 0.861 / 0.809 / 0.906.
Gates tied to fusion's thresholds also make the confidence *scale* follow them: tightening fusion to
0.6 px / 0.5% left the ranking unchanged (AUC 0.726 both) but halved the median confidence, so a
fixed fusion floor dropped 27% of the admitted pixels, 37% of them correct.

### Raw-neighbour-confidence invariant

`cN` (gate 4) and `Pconf` must be the neighbour's **raw** photometric confidence, never its
already-adjusted one — otherwise geometric agreement would be double-counted (the neighbour's
posterior already folded in its own neighbours, including this pixel) and the result would depend
on worker processing order. The design is a single Jacobi pass: every pixel's adjusted confidence
is a function of raw neighbour confidences only. This is enforced structurally:

- **Fused / epilogue**: neighbour normal/confidence maps are loaded by `InitViews` only on the last
  geometric-consistency iteration (`loadDepthMaps == 2`) into `DepthData::images[].confMap`, the
  *previous* iteration's snapshot; if that snapshot's own dmap already carries `CONF_ADJUSTED`,
  `InitViews` drops it instead of loading it, so a re-run over already-adjusted dmaps cannot feed
  adjusted values back in as evidence — the neighbour then gates as "no confidence" (neutral),
  matching `ConfNeighborHost::conf == null`.
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
(`SceneDensify.cpp`), to `ADJUST_CONFIDENCE` only when **all** of: a CUDA PatchMatch pool
exists, `OPTDENSE::bEstimateConfidenceCUDA` is true, `nFusionMode >= 0`, and
`nEstimationGeometricIters > 0` — otherwise it resolves to off. The resolution is scoped to that
call (restored on return), so a later `DenseReconstruction` in the same process re-resolves `AUTO`
for its own backend. Metal and CPU-only builds therefore always resolve `AUTO` to off.

When `ADJUST_CONFIDENCE` is set (auto-resolved or forced), three paths exist, tried in this order:

1. **Fused in-estimation** (`PatchMatch::EstimateDepthMap`, `PatchMatchCUDA.cpp`): fires
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
   reference (`numMaxNeighbors` in `SceneDensify.cpp`) through a phase-lifetime `DMapCache`
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
| *(dense config file only)* | `OPTDENSE::fNCCThresholdKeep` | `0.9` | `minConfidence = 1 − this` is gate 4's centre (the estimation's photometric floor) |
| — (compile-time) | `ConfRefine::CONFIRM_DEPTH` | `0.005` | gate 1 relative-depth width; unit of the free-space margin and the prior's plane-fit band |
| — (compile-time) | `ConfRefine::CONFIRM_SIN_ANGLE` | `sin 20°` | gate 2: triangulation angle of a full vote |
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
degrades the result. Only `fNCCThresholdKeep` stays runtime, because it is the estimation's own
floor.

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
- The recalibration never reads fusion's join thresholds (`fDepthDiffThreshold`,
  `fDepthReprojectionErrorThreshold`): changing how finely fusion clusters must not move the
  confidence scale that fusion's floor is applied to.
- The CPU and GPU paths share the exact same per-pixel math and parameter snapshot
  (`ConfidenceRefine.h` and `Common/DepthGeometry.h`, compiled under both the host compiler and
  `nvcc`); the GPU path differs only in using single-precision `expf`.

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

- **Hard pass/fail gates** — lose most of the achievable ROC gain; continuous weights are the lever.
- **Confirmation gates on fusion's join thresholds, or any fixed-pixel reprojection gate** — the
  residual repeats the depth error scaled by `f·sin θ`, so it drops wide-baseline confirmations, and
  the confidence scale then moves with fusion's settings (§2).
- **Per-image depth widths from an estimated noise scale** — the scale is estimable (correlation
  0.77–0.97 with ground truth) but does not beat the fixed 0.5%.
- **Shape constants as CLI/config knobs** — one jointly calibrated operating point; moving one
  without re-sweeping the others degrades it.
- **`CONF_FLOOR = 0.5`** — too high to test the floor's real job (protecting confirmed few-view
  inliers); the sweep favoured a small floor.
- **The integrated CPU mode as default** — ties or loses against the standalone phase's thread
  parallelism; only the GPU makes it nearly free.
- **More `nPatchMatchCUDAInstances` to feed the inline sweep** — bandwidth-bound; oversubscribing
  the GPU slows estimation.
- **The estimator's geometric-consistency score as a confidence feature** — already folded into the
  NCC score; not worth a new per-pixel buffer through both estimators.
- **A pass-wide decoded cache of the neighbours' maps for `InitViews`** — cuts process reads 81–85%
  (Meetingroom R0) with no gain in wall time: the OS file cache already
  serves ~93% of those reads, and the decoded copy duplicates it, adding 2 GB peak and more disk reads.
- **Monocular-model pseudo-GT for tuning** — its error floor was one to two orders of magnitude above
  the effects being tuned.

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
