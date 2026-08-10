# CUDA confidence recalibration (Task 14) — 2026-07-11

GPU port of the fusion-faithful per-view confidence recalibration, and the switch that makes adaptive
confidence the default. Branch `feature/cuda-confidence` (off `feature/fusion-faithful-confidence`).

## What it does

The confidence recalibration (`AdjustConfidenceSweep`: intra-map prior + one-hop multi-view
confirmation → Beta posterior) is embarrassingly parallel over the H×W reference grid. Task 14 runs
it as two CUDA kernels (one thread/pixel) — `PriorKernel` + `SweepKernel` in `libs/MVS/ConfidenceCUDA.cu`
— integrated into the **last geometric-consistency iteration**, reusing the neighbor depth/normal/conf
already resident in host memory from that iteration's geometric scoring (no extra disk read, no second
estimation pass). The per-pixel math is shared *verbatim* with the CPU via `libs/MVS/ConfidenceRefine.h`
(`__host__ __device__` inlines: `DepthPlaneFit`, `Posterior`, soft-gate weights), so CPU and GPU
evaluate the identical closed form.

## Control model

- **Master switch = the `ADJUST_CONFIDENCE` postprocess flag, now default ON** for both CPU and GPU
  (`OPTDENSE::nOptimize` / `--postprocess-dmaps` default `0 → 4`). Adaptive confidence runs on every
  densify; opt out with `--postprocess-dmaps 0`.
- **`OPTDENSE::bEstimateConfidenceCUDA` (default 1):** when CUDA is used for depth-map estimation, run
  the recalibration on the **GPU** integrated into the last geometric-consistency iteration; `0`
  **forces the CPU version** (the standalone postprocess pass). No effect on the CPU estimation path.
- **Dispatch:** the last-geo-iter epilogue calls `AdjustConfidenceCUDA` (GPU) when CUDA-estimation +
  `bEstimateConfidenceCUDA`; on any CUDA error it falls back to the CPU integrated sweep, and the
  legacy `bEstimateConfidence` bool still selects the integrated-CPU path. The standalone postprocess
  phase is skipped whenever the integrated path ran (extended double-adjust guard). With
  `--geometric-iters 0` there is no last iteration, so confidence falls back to the standalone CPU pass.

## Release note — adaptive confidence is ON by default (cross-process double-adjust)

`--postprocess-dmaps` now defaults to `4` (`ADJUST_CONFIDENCE`), so every densify recalibrates
confidence. **Within a single process this is safe:** the in-process guard (`bIntegratedConfRan`) skips
the standalone phase whenever the integrated last-iteration recalibration already ran, so confidence is
adjusted exactly once. **The guard is per-process, though.** If you split the pipeline — estimate in
one invocation, then run a *separate* `--postprocess-dmaps 4` pass over the same saved `.dmap`s — the
confidence is recalibrated a second time (the posterior/gate/floor compounded on its own output). In a
split estimate → adjust → fuse workflow, pass `--postprocess-dmaps 0` on the later invocation(s) to opt
out. A single `DensifyPointCloud` call (estimate → adjust → fuse in one process) is unaffected.

**RESOLVED (2026-08-09):** the split-workflow hazard above is now guarded on disk. Every dmap whose
confidence was recalibrated carries a `CONF_ADJUSTED` flag bit in the D2 header `type` field (a
non-content bit the codec carries through and old readers ignore; `MvsUtils.py` masks it and exposes
`conf_adjusted`). The standalone postprocess phase checks the flag per view and **warns + skips**
instead of compounding (verified live: 38/38 already-adjusted views skipped, 0.00 ms/map compute,
files untouched). `--postprocess-dmaps 0` opt-out remains available but is no longer required.

## Parity (GPU vs CPU) — the recalibration is the same

**CUDA depth estimation is nondeterministic** (PatchMatchCUDA's curand: two runs of the same scene
differ on ~71% of depth pixels, max |Δdepth| ≫ 0). So a naive *separate-run* confmap comparison is
confounded — the *geometry* differs, not the confidence code. Parity therefore has to isolate the
confidence backend from the estimation noise, done two ways, both PASS:

1. **Identical-input harness (T14b):** run the GPU kernel on the *exact* `NeighborProj` + maps the CPU
   sweep just used, within one process (eth3d_meadow L3, soft gates, 8 neighbors/view). Result:
   mean |Δconf| ~1e-6, and end-to-end GT **ROC-AUC identical: 0.8614 = 0.8614** (PR/P@0.1/R@0.1/Brier/ECE
   all match to 4 decimals) → **|ΔROC| = 0.000**. Max |Δconf| ~0.1–0.27 on a minority of gate-boundary
   pixels (float-vs-double `exp` tipping the discrete `w>0.05` soft gate) — unbiased, zero aggregate effect.

2. **Real-pipeline, isolate by mechanism (T14d):** full densify with geometric iters, comparing the
   *integrated* GPU path to the *integrated* CPU path (not the standalone one):

   | run (eth3d_meadow L3, integrated) | GT ROC-AUC |
   |---|---|
   | GPU run 1 | 0.8266 |
   | GPU run 2 | 0.8261 |
   | CPU integrated (`Estimate Confidence CUDA=0` + `Estimate Confidence=1`) | **0.8275** |

   GPU-vs-CPU-integrated **|ΔROC| ≈ 0.001**, GPU run-to-run 0.0005 (pure estimation noise) — both
   inside the ≤0.005 gate. (CPU *standalone* scores 0.8603 — a *different* neighbor set + `cameraDepthMap`
   poses, i.e. the integrated-vs-standalone gap studied in Task 13, not a GPU effect.)

The CPU refactor onto the shared header is itself **byte-identical** (T14a): OLD vs refactored binary,
conf-adjust on eth3d_meadow L3 (5.6M pixels) → max |Δconf| = 0.0, 0 pixels differ.

## Timing

eth3d_meadow L3, 15 depth-maps, integrated recalibration (kernel + host↔device transfers):

| backend | ms/map | total |
|---|---|---|
| **GPU** | **~5.0 ms** | 75 ms |
| CPU integrated | 93 ms | 1399 ms |

GPU is **~18× faster** per map and **10× under the ≤50 ms/view** acceptance target; the ~75 ms it adds
across the whole run is a negligible fraction of estimation wall. (`VERBOSE`: "Integrated confidence
recalibration (GPU): … ms/map avg".)

## Fused kernel — resident-buffer reuse (T14e, 2026-08-09)

The confidence launch now runs **inside `PatchMatch::EstimateDepthMap`**, right after the last
geometric-consistency kernels on the same instance stream, reading the reference depth+normal from
the resident `cudaDepthNormalEstimates` (`Point4`: xyz=normal, w=depth) and the raw NCC confidence
from the resident `cudaDepthNormalCosts` (`1-cost`, the same conversion the host unpack applies).
Only the neighbors' **raw previous-iteration** depth/conf/normal snapshots are uploaded (the §4
raw-neighbor-conf invariant: the host `depthDataRef.images[]` copies loaded by `InitViews` are the
only copy that exists, and it is raw). The adjusted confidence is downloaded into a separate output
buffer straight into `confMap`; the unpack skips the cost→conf conversion; the `EVT_SAVEDEPTHMAP`
epilogue is skipped via the transient `DepthData::bConfAdjusted`. A failed launch falls back to the
epilogue re-upload GPU path, then the CPU sweep (chain intact). With the speckle/gap filters on
(`OPTIMIZE` bits), the fused path is skipped so the adjust keeps running after filtering.

The kernels are templated on a reference accessor (`RefLinearAcc` host-upload layout vs
`RefPackedAcc` resident layout), so the standalone/fallback launcher and the parity harness are
unchanged. Per-worker device footprint drops ~27% (no reference re-upload: 28 B/px + 20n B/px →
8 + 16n B/px, n = neighbors).

**Parity (§5 method 2, real pipeline, D2 codec):** eth3d_meadow L3, GPU-fused vs CPU-integrated,
GT ROC-AUC **0.824253 vs 0.824279 → |ΔROC| = 0.000026** (includes estimation noise; gate ≤0.005;
pre-refactor recorded ≈0.001). Full metric row (PR-AUC, Spearman, Brier, ECE) matches to ≤1e-3.

**Timing (fused vs epilogue re-upload, same binary):**

| scene / res | px/map | fused +tex2D | fused (upload nbr depth) | epilogue re-upload | CPU integrated |
|---|---|---|---|---|---|
| eth3d_meadow L3 | ~0.29 MP | **3.28 ms/map** | 3.59 ms/map | ~5.0 ms/map (T14 recorded) | 93.3 ms/map |
| eth3d_courtyard L1 | ~6.1 MP | **60.0 ms/map** | 65.6 ms/map | 75.0 ms/map | ~2364 ms/map (recorded) |

At full resolution the reference-buffer reuse saved **12.5%** — matching the design math (the
reference maps were ~13% of the transfer volume) — and the follow-up **neighbor-depth `tex2D`
reuse** (2026-08-10) another **8.6%**: each neighbor descriptor can carry the PatchMatch instance's
resident depth texture (`textureDepths[i-1]`, the same raw previous-iteration map geometric
consistency just used), skipping that upload + its `cudaMalloc`. The texture is linear-filtered, but
all confidence reads are exact texel-center fetches (`x+0.5`) whose interpolation weight is exactly
0 → bit-identical to the linear-buffer read (parity re-verified: meadow L3 GT ROC 0.8243 vs
0.824253 recorded). Guard: the texture is used per-neighbor only when it holds the UNRESIZED map
(`view.depthMap.size() == image.size()`); a mismatched neighbor falls back to the linear upload
(the fused upload path resizes with INTER_LINEAR in that case, which would diverge from the CPU
path's native-resolution sampling). The remaining upload (neighbor conf+normal, ~16n B/px) is
inherent. The original ≤50 ms/view target is met at L3/L2 scales and now missed by only 20% at
6 MP (60.0 ms); against the CPU integrated path the fused GPU is **~39× faster** at 6 MP.

## CUDA-OFF build

A `-DOpenMVS_USE_CUDA=OFF` build compiles and links cleanly (**verified**: fresh CUDA-off configure +
`DensifyPointCloud` target built 100%, no errors, `ConfidenceCUDA.cu.o` correctly excluded). The kernel
`.cu` is dropped by the CMake glob, `AdjustConfidenceCUDA` / `RunConfidenceCUDA` / `ConfNeighborHost` /
`pmCUDAPool` / `bEstimateConfidenceCUDA` are all referenced only inside `#ifdef _USE_CUDA`,
`ConfidenceCUDA.h`'s declaration is plain POD (no CUDA types), and `ConfidenceRefine.h`'s host path
(`CR_HD`=`inline`, double `std::exp`) compiles in the host passes.

## Reproduce

```bash
# build (CUDA): VCPKG_ROOT=/home/ubuntu/vcpkg cmake --build make --config Release --target DensifyPointCloud -j30
# (a new .cu needs a cmake reconfigure first: FILE(GLOB) runs at configure time)
LD=/usr/local/cuda/lib64
# GPU integrated (default): adaptive confidence on, GPU, in the last geometric iteration
LD_LIBRARY_PATH=$LD make/bin/Release/DensifyPointCloud scene.mvs -w WD --gpu-device 0 --geometric-iters 2 -v 2
#   -> "Using CUDA compute backend" + "Integrated confidence recalibration (GPU): …ms/map"
#   -> "skipping the postprocess confidence-adjust phase … already recalibrated …"
# force CPU integrated (for parity): dense-config-file with  Estimate Confidence CUDA = 0  and  Estimate Confidence = 1
# grade a confmap: scripts/python/EvalConfidence.py WD --gt-depth-dir <gt> --gt-format eth3d --scene-mvs scene.mvs
```

Files: `libs/MVS/ConfidenceRefine.h` (shared math), `libs/MVS/ConfidenceCUDA.{h,cu}` (kernels +
launcher), `libs/MVS/SceneDensify.cpp` (`AdjustConfidenceCUDA` + dispatch), `libs/MVS/DepthMap.{cpp,h}`
+ `apps/DensifyPointCloud/DensifyPointCloud.cpp` (flags/defaults).
