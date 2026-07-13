# T14 CUDA confidence — HANDOFF for the next agent (2026-07-13)

Branch **`feature/cuda-confidence`** (off `feature/fusion-faithful-confidence` @ `8b6c922`).
This file is the single source of truth for continuing the work. Read it fully before touching code.

---

## 1. TL;DR — where things stand

T14 (GPU port of the per-view confidence recalibration) is **implemented, validated, reviewed, and
committed** and WORKS. The remaining work is **one optimization the user explicitly requested**:
make the GPU confidence a *true extension of the last PatchMatch geometric-iteration* that **reuses the
device-resident buffers** instead of re-uploading them from the host. The current implementation
re-uploads data that is already on the GPU — correct, but wasteful in device memory and (at full
resolution) in PCIe transfer time.

**The user paused here specifically to lock the correctness invariant first (see §4). Do NOT start the
refactor until you have internalized §4 — it is the whole point of the pause.**

---

## 2. What is DONE (committed on this branch, `8b6c922..HEAD`)

| commit | what |
|---|---|
| `8fd0516` | `libs/MVS/ConfidenceRefine.h` — shared `__host__ __device__` per-pixel math (`DepthPlaneFit`, `Posterior`, `SoftDepthW/SoftReprojW/SoftConfW`, `IsDepthSimilarF`, `NormalFromGrad`, `CRexp`); CPU refactored onto it, **byte-identical** (0/5.6M px differ). |
| `fe34ae6` | `libs/MVS/ConfidenceCUDA.{h,cu}` — `PriorKernel` + `SweepKernel` (one thread/ref pixel) + host launcher `RunConfidenceCUDA` (host-in/host-out; RAII device cleanup; false→CPU fallback). Parity vs CPU on identical inputs: **|ΔROC| = 0.000**. |
| `55f7705` | Wiring: `ADJUST_CONFIDENCE` default ON (`nOptimize`/`--postprocess-dmaps` 0→4); new `OPTDENSE::bEstimateConfidenceCUDA` (default 1); `DepthMapsData::AdjustConfidenceCUDA(DepthData&)`; epilogue dispatch + double-adjust guard + `loadDepthMaps==2` trigger. |
| `248fd0e` | VERBOSE integrated-confidence timing line (GPU/CPU, ms/map). |
| `90ca9f7`,`fa93187`,`a89d8df` | docs (`gt_bench/CUDA_CONFIDENCE.md`), CUDA-OFF build confirmed, release note. |
| `32d04b4` | review-nit: clear per-thread CUDA error on the launcher failure path. |

**Validation already passed** (see `gt_bench/CUDA_CONFIDENCE.md` for the full writeup):
- Parity GPU vs CPU: |ΔROC| = 0.000 (identical-input harness) / ≈0.001 (real-pipeline integrated runs).
- Timing: GPU ~5.0 ms/map vs CPU-integrated 93 ms/map — **BUT measured at eth3d_meadow L3 (downscaled).
  Full-resolution timing was NOT measured and is a risk because of the re-uploads — re-measure.**
- CUDA-OFF build compiles + links clean.
- CPU refactor byte-identical.

**Control model (per user):** master switch = `ADJUST_CONFIDENCE` postprocess flag, default ON for CPU
+ GPU; `bEstimateConfidenceCUDA` (default 1) = when CUDA estimates, run confidence on GPU integrated,
else force CPU. Epilogue runs GPU (CPU fallback on error / legacy `bEstimateConfidence` opt-in);
standalone phase skipped when integrated ran.

---

## 3. THE NEXT TASK — resident-buffer reuse ("true fused kernel")

**Goal:** the GPU confidence must reuse what the last geometric iteration already left on the device,
uploading only what genuinely is NOT there.

**What is device-resident per PatchMatch worker after the last geometric iter** (verify in
`libs/MVS/PatchMatchCUDA.cpp` `AllocatePatchMatchCUDA`/`AllocateImageCUDA` ~181-225, `RunCUDA` ~774-817):
- `cudaDepthNormalEstimates` — `Point4` per pixel: **xyz = reference normal, w = reference depth**. REUSE.
- `cudaDepthNormalCosts` — reference **cost**; raw conf = `cost>=1 ? 0 : 1-cost` (see readback ~`PatchMatchCUDA.cpp:461`). REUSE (this is `confPhoto`).
- `cudaDepthArrays` / `cudaTextureDepths` — **neighbor depth** textures (geom mode). REUSE (via `tex2D`).
- `cudaImageArrays` — source images (not needed by confidence).

**What is NOT on the device and MUST be uploaded** (this is the irreducible ~70%):
- **neighbor CONF** and **neighbor NORMAL** — PatchMatch's geometric consistency only ever needs
  neighbor *depth*, so it never uploads neighbor conf/normal. But confidence gate G3 (normal) and
  G4/Pconf (neighbor conf) require them. Upload from the host `depthDataRef.images[i].confMap`/
  `.normalMap` (loaded by `InitViews` `loadDepthMaps==2`). See §4 for WHY the host copy is the correct
  (raw) one.
- the small `NeighborProj` matrix array (A/b/Ai/bi/Rrel per neighbor) — tiny; build on host as
  `AdjustConfidenceCUDA` already does (reuse that code).

**Memory math (per pixel, n≈9 neighbors):** current re-upload ≈ `28 + 20n` ≈ 208 B/px extra; after
reuse ≈ `8 + 16n` ≈ 152 B/px (prior + confOut + neighbor conf/normal). Reuse reclaims ≈ **56 B/px
(~27%)** and the matching transfers — dominated by dropping the reference maps + neighbor depth
re-upload. The neighbor conf+normal (~144 B/px) is inherent; it CANNOT be eliminated.

### Implementation approach
Move the confidence launch **into the PatchMatch instance's flow** so it reads the resident buffers:
- Add a method on `MVS::CUDA::PatchMatch` (in `PatchMatchCUDA.inl`/`.cpp`/`.cu`) that, after the
  estimation kernels and **before the conf D2H**, runs the confidence kernels using its own
  `cudaDepthNormalEstimates` (ref depth+normal), `cudaDepthNormalCosts` (ref cost→conf),
  `cudaTextureDepths` (neighbor depth), plus uploaded neighbor conf/normal + `NeighborProj`.
- The confidence math stays `ConfidenceRefine.h` (unchanged). Only the **data access** in the kernels
  changes: `PriorKernel`/`SweepKernel` need to read ref depth+normal from the `Point4` layout, ref
  conf from cost (`1-cost`), and neighbor depth from a **texture** (`tex2D`) instead of linear buffers.
  Consider kernel variants or template/param the accessors; keep the current linear-buffer kernels for
  the standalone/CPU-fallback path and the parity harness.
- **Output conversion gotcha:** the confidence kernel outputs an adjusted **conf ∈ [0,1]**, but the
  existing readback converts `cudaDepthNormalCosts` as a *cost* (`1-cost`). Write the adjusted conf to
  a **separate output buffer** D2H'd without the `1-cost` conversion (or adjust the unpack), or you will
  corrupt it.
- Dispatch: the confidence would then run *inside* estimation on the last geo iter — so the
  `EVT_SAVEDEPTHMAP` epilogue must NOT also call `AdjustConfidenceCUDA` (avoid double-adjust). Rework the
  dispatch so exactly one place does it. Thread "last geo iter + confidence on + neighbor conf/normal +
  NeighborProj" into the PatchMatch call. `EstimateDepthMap(depthData)` has `depthData.images[]` and the
  cameras, so everything needed is reachable.
- Keep the CPU path and `--geometric-iters 0` / CPU-estimation fallbacks working exactly as now.

### Neighbor-depth texture reuse is optional / fiddly
Reusing neighbor depth from `cudaTextureDepths` saves ~4n≈36 B/px but requires `tex2D` sampling and
care about the texture's **filter mode** (point vs linear — the confidence gates need exact
integer-pixel neighbor depth for the nearest sample and a controlled bilinear tap). If it adds too much
risk, the user said it's acceptable to **reuse only the reference buffers** and keep uploading neighbor
depth — that still removes the reference re-upload (the larger correctness-relevant win) at lower risk.
Decide based on the texture's descriptor.

---

## 4. ⚠️ THE CORRECTNESS INVARIANT YOU MUST PRESERVE — raw vs adjusted neighbor confidence

The confidence adjustment for reference R uses each neighbor N's confidence `cN` in **G4** (reject if
`cN < minConfidence`) and in **`Pconf`** (`Pconf += cN`, soft `+= w·cN`) — the multi-view confirmation
evidence in the posterior `(s·pGeo + Pconf)/(s + Pconf + λV)`.

**`cN` MUST be N's RAW NCC confidence, never N's ADJUSTED confidence.** Using adjusted would (a)
double-count geometric agreement (N's posterior already folded in N's neighbors, incl. R) and (b) make
the result depend on worker processing order (raw if N not yet adjusted, adjusted if it was) — a
nondeterministic cascade. The design is a **single Jacobi pass**: every pixel's adjusted confidence is a
function of *raw* neighbor confidences only.

**How rawness is guaranteed today (preserve this):** confidence is adjusted **only in the last geometric
iteration**, on the reference's **own** confMap (→ `geo.dmap`). A reference reads its neighbors via
`InitViews`/`loadDepthMaps==2` from each neighbor's **`.dmap` file = the PREVIOUS iteration's output**,
which was never adjusted; `geo.dmap→dmap` rename happens once, after all workers join. So no reference
ever reads another reference's this-iteration adjusted conf. (The standalone path uses the
`bDeferSwap`/`confMapAdjusted` barrier for the same reason.)

**How the fused kernel MUST preserve it:**
- Upload neighbor conf (and normal) from the **host `depthDataRef.images[i].confMap`** (the raw
  previous-iteration snapshot `InitViews` loaded). There is no resident neighbor conf on the device to
  accidentally reuse — which is convenient: the only copy that exists is also the only copy that's raw.
- Reference's own raw conf comes from the resident `cudaDepthNormalCosts` (`1-cost`) as `confPhoto`.
- Write the adjusted conf to the **output** buffer only; never feed a this-iteration adjusted conf back
  as a neighbor input. **Do NOT cache adjusted confidences on the device across views** — that would
  reintroduce exactly this ambiguity.

Result must remain the raw-neighbor-confidence, single-pass, order-independent semantics, bit-for-bit
with the CPU (within the established GPU float ULP band).

---

## 5. How to verify after the refactor

- **Parity (the reliable method):** CUDA depth estimation is **NONdeterministic** (PatchMatchCUDA
  curand — ~71% of depth pixels differ run-to-run), so a naive separate-run confmap diff is confounded
  by geometry. Isolate the confidence backend from estimation noise two ways:
  1. Reuse the T14b idea — an identical-input in-process comparison (GPU vs the CPU `AdjustConfidenceSweep`
     on the *same* NeighborProj + maps). The old harness lived in `AdjustConfidenceSweep` behind
     `MVS_CONF_CUDA_CHECK`/`MVS_CONF_CUDA_WRITEBACK` (removed in `fe34ae6`/`55f7705`; re-add temporarily
     if useful). Target: mean|Δconf|~1e-6, and GT ROC identical.
  2. Real pipeline, isolate by MECHANISM: GPU-integrated vs CPU-*integrated* (`Estimate Confidence CUDA =
     0` + `Estimate Confidence = 1`), full densify with geometric iters, EvalConfidence GT ROC on both.
     Expect |ΔROC| ≤ 0.005 (T14 got ≈0.001). Do NOT compare against CPU-*standalone* (different neighbor
     set + `cameraDepthMap` poses → ~0.03 gap, not a bug).
- **Timing:** re-measure at **full resolution** (not L3) — the whole point is the re-upload cost. Read the
  "Integrated confidence recalibration (GPU): …ms/map" line. Target ≤50 ms/view.
- **Memory:** confirm the per-worker device footprint drops (nvidia-smi during a run, or instrument).
- **Regressions:** CPU byte-identity still holds (T14a method); `-DOpenMVS_USE_CUDA=OFF` still builds;
  `--geometric-iters 0` and CPU-estimation still route to the standalone CPU pass.

---

## 6. Build & run (this machine)

- GPU: A100-40GB, CUDA 12.6 (`/usr/local/cuda`). Canonical build dir is **`make/`** (already configured,
  CUDA ON). Build: `VCPKG_ROOT=/home/ubuntu/vcpkg cmake --build make --config Release --target DensifyPointCloud -j30`.
  Binary `make/bin/Release/DensifyPointCloud`; run with `LD_LIBRARY_PATH=/usr/local/cuda/lib64`.
- **A new `.cu` file needs a cmake RECONFIGURE first** (`cmake make`) — `FILE(GLOB *.cu)` runs at
  configure time. (Editing an existing `.cu` does not.)
- GT benchmark data: `/home/ubuntu/virginia/gt_bench/` (21T mount — put ALL experiment outputs here, NOT
  on root `/dev/vda1` which runs near-full). ETH3D scene example: `runs/eth3d_meadow/scene.mvs` at
  `--resolution-level 3` (small/fast). `scripts/python/EvalConfidence.py … --gt-format eth3d` for ROC.
  numpy is only in `/home/ubuntu/miniconda3/bin/python`.
- Small full-pipeline smoke scene: `/home/ubuntu/data/ladita` (32 imgs; symlink `input`→`<wd>/images`,
  run with `-w <wd>`).

## 7. Key files

- `libs/MVS/ConfidenceRefine.h` — shared math (REUSE as-is; no change expected).
- `libs/MVS/ConfidenceCUDA.{h,cu}` — kernels + launcher (the refactor lives mostly here + PatchMatchCUDA).
- `libs/MVS/PatchMatchCUDA.{inl,cpp,cu}` — the estimation instance whose resident buffers you reuse; add
  the confidence launch to its flow. Kernel/launch pattern: `RunCUDA` ~774-817, allocs ~181-225.
- `libs/MVS/SceneDensify.cpp` — `AdjustConfidenceCUDA(DepthData&)` (~1466, host neighbor build to reuse),
  dispatch: epilogue (search `AdjustConfidenceCUDA(depthData)`), standalone guard (`bIntegratedConfRan`),
  `loadDepthMaps==2` trigger (`bWillAdjustConf`), timing line.
- `libs/MVS/DepthMap.{cpp,h}`, `apps/DensifyPointCloud/DensifyPointCloud.cpp` — flags/defaults.
- `gt_bench/CUDA_CONFIDENCE.md` — the feature writeup (update it when the refactor lands).

## 8. Open items (NOT blocking the refactor — user decisions)

- Minor review finding: **cross-process double-adjust** now reachable by default (release note added to
  `CUDA_CONFIDENCE.md`; a real guard = marking dmaps "already adjusted", deferred).
- Minor review finding: per-worker GPU alloc × workers can OOM on huge images → CPU fallback (safe). The
  reuse refactor reduces this by ~27%.
- Inherited from the parent branch (independent of T14): **revert the test-only determinism hash**
  (commit `763be0d` in `Scene::EstimateNeighborViewsPointCloud`) before release; and the **w2-vs-w3
  fusion default** decision (recommendation: w2). See the `gt-benchmark` / `confidence-recalibration-feature`
  memories.

## 9. Process notes

- Progress ledger: `.superpowers/sdd/progress.md` (this branch's task history).
- Memories (outside repo, `/home/ubuntu/.claude/projects/-home-ubuntu-openMVS/memory/`):
  `confidence-recalibration-feature.md` (has a T14 section), `gt-benchmark.md`, `openmvs-build-env.md`.
- The subagent-driven-development flow was used (implement → review → fix per task). The final
  whole-branch review of the current state came back clean (no Critical/Important).
