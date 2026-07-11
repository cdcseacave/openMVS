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

## CUDA-OFF build

A `-DOpenMVS_USE_CUDA=OFF` build is expected to compile: the kernel `.cu` is excluded by the CMake
glob, `AdjustConfidenceCUDA` / `RunConfidenceCUDA` / `ConfNeighborHost` / `pmCUDAPool` /
`bEstimateConfidenceCUDA` are all referenced only inside `#ifdef _USE_CUDA`, `ConfidenceCUDA.h`'s
declaration is plain POD (no CUDA types), and `ConfidenceRefine.h`'s host path (`CR_HD`=`inline`,
double `std::exp`) already compiles in the host passes. (A dedicated CUDA-off configure/build confirms
this — see `/home/ubuntu/virginia/build_nocuda`.)

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
