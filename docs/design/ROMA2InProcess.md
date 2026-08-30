# RoMa v2 In-Process — Retrieval and Dense Matching

## Overview

RoMa v2 (DINOv3 backbone + a coarse matcher head) runs **in-process** through ONNX Runtime, as an
optional supplement to the classical SFM pipeline — never a replacement for it. It plugs into two
independent seams:

```
CreateStructure --roma2 ...                    Scene::MatchPairs
  --roma2-retrieval ──> global descriptors ──> PairsMatcher::QueryRetrieval
                         (GlobalDescriptors.h)   (replaces the vocabulary tree as the ranking source;
                                                   RRF/mutual-top-K/bridging/feedback unchanged)
  --roma2-match ─────> coarse dense warps ────> guided sparse re-match
                         (MatchROMA2.h/cpp,        (ROMA2Warp.h/cpp: erosion, keypoint tracking,
                          RoMa2Matcher.h/cpp,       MatchFeaturesGeometric, replace-by-inliers)
                          OnnxRuntime.h/cpp)
```

No new `MatchMode`: everything downstream of pair ranking is backend-agnostic, and the
VOCABULARY→EXHAUSTIVE small-scene remap and the KNOWN_POSES unposed-image fallback keep working
unchanged. The in-process integration replaces the earlier NPZ-based ROMA2 import outright (deleted:
`ImportROMA2.{h,cpp}`, `--import-roma2`, depth-map import) — there is no dual path.

Source: `libs/SFM/OnnxRuntime.h/cpp` (ONNX Runtime session/tensor wrapper), `libs/SFM/RoMa2Matcher.h/cpp`
(the two RoMa v2 sessions), `libs/SFM/MatchROMA2.h/cpp` (describe pass, dense-matching pass,
`ROMA2Config`), `libs/SFM/ROMA2Warp.h/cpp` (warp coordinates, confidence erosion, keypoint tracking,
guided-pair store/replace), `libs/SFM/GlobalDescriptors.h/cpp` (retrieval pooling + cosine index). CLI:
`apps/CreateStructure/CreateStructure.cpp` (`--roma2*`, `--export-retrieval-csv`).

---

## Graph Contract

Per preset `S ∈ {320 (turbo), 512 (fast), 640 (base)}`, `G = S/16` patch grid, `C = S/4` warp cells:

| File | Inputs | Outputs |
|---|---|---|
| `roma_<setting>_descriptor_fp32.onnx` (+`.onnx.data`) | `image` `[1,3,S,S]` f32, RGB planar, values in [0,1], un-normalised (ImageNet mean/std applied in-graph) | `layers` `[1,2,G,G,1024]` f32 channels-last (blocks 11+17, patch tokens only, unchanged from the shipped engine); `value_facets` `[1,2,G,G,1024]` f32 channels-last (V projections of blocks 15+20, before attention weighting/`o_proj`) |
| `roma_<setting>_match_coarse_fp32.onnx` (+`.onnx.data`) | `descriptors_A`, `descriptors_B` `[1,2,G,G,1024]` f32; `img_A`, `img_B` `[1,3,S,S]` f32 (dead on the coarse graph, kept for contract compatibility with the published TensorRT graphs) | `warp` `[1,C,C,2]` f32, normalised (x,y) ∈ [-1,1] (align_corners=False); `confidence` `[1,C,C,1]` f32 raw overlap logit (sigmoid on host) |
| `roma_<setting>.json` | openMVS manifest: `format_version`, `model`, `setting`, `image_size`, `patch`, `layers`, `value_facet_blocks`, `warp_size`, `confidence_channels`, `retrieval_recipes` (`facets`/`layers` dim + GeM params), `files`, `io` (shapes), `opset`, checksums | |

fp32, static shapes, batch 1, opset 18. Coordinate conventions (`ROMA2Warp.h/cpp`, deliberate
asymmetry): pixel→grid `CoordFromTo` is align_corners=**true**; grid→pixel `DenormCoord` is
align_corners=**false** (`0.5*(n+1)*W - 0.5`). `RoMa2Manifest::Load` (`libs/SFM/RoMa2Matcher.h/cpp`)
rejects a manifest of another schema version, a missing/ill-typed key, or a declared graph I/O that
disagrees with the shapes derived from `image_size`/`warp_size`; the C++ loader (`OnnxModel::Load`,
`libs/SFM/OnnxRuntime.cpp`) separately rejects any negative (dynamic) dim in the graphs themselves.

Scope is descriptor + coarse-match only — the refiner ("regular") graph is out of scope (needs the
`local_corr` CUDA extension, VGG19-BN fine features, `grid_sample`); no depth maps are produced by the
in-process warps.

### Export tooling and provenance

Graphs are produced by `scripts/python/roma2/export.sh` (wraps `export.py onnx|check|manifest` and
`parity.py`), run inside the polyml export project environment (`uv run --project ~/polyml/romav2
--with onnxruntime-gpu==1.23.2`). The campaign spec that defines the `value_facets` tap and the
retrieval recipes is `EXPORT_REQUEST.md` (lives outside this repository, at
`~/megaloc-vs-dinov3-2026-08-28/EXPORT_REQUEST.md`).

Exported model sets live on the shared volume, one directory per export, e.g.
`~/virginia/models/roma2-onnx/roma2onnx-20260829-facets1520/` (the export current as of this task) —
referenced by `--roma2-model` or `$OPENMVS_ROMA2_MODEL_PATH`. The `.onnx` + `.onnx.data` + `.json` set
is byte-portable across OSs (external data is resolved relative to the model path on every platform).

---

## Retrieval Recipes

`SFM::PoolRetrievalDescriptor` (`libs/SFM/GlobalDescriptors.h/cpp`) pools one image's
descriptor-graph output into its global retrieval descriptor:

- **FACETS** (default, 2048-D): per-slice GeM p=3 (clamp 1e-6, cube, mean over the G·G patches, cbrt,
  accumulated in double) on `value_facets` → L2 per slice → concat (2048) → L2 →
  `sign(d)·|d|^0.3` (power normalization, `retrievalPower`) → L2.
- **LAYERS** (legacy/parity, 1024-D): GeM p=3 on `layers` slice 1 (block 17) → L2. Reproduces the
  shipped reference engine's own pooling.

The C++ GeM was matched to the campaign's Python implementation to 6e-8.

Measured against `~/virginia/models/roma2-onnx/roma2onnx-20260829-facets1520/CROSSCHECK.md`, seven
LiDAR captures (the engine consumed `keyframes/images`, not `corrected_images`):

| Measure | Value | Bound / reference |
|---|---|---|
| LAYERS pooled GeM cosine vs the shipped TensorRT dumps | ≥ 0.999966 | ≥ 0.999 |
| FACETS `value_facets` cosine vs the campaign's torch taps (t15v/t20v) | ≥ 0.999990 (worst per-image) | ≥ 0.999 |
| LAYERS recall@16 (non-temporal) | 0.6565 | shipped 0.656 ± 0.005 |
| FACETS recall@16 (non-temporal) | 0.7955 | shipped 0.797 ± 0.01 |
| Mean per-image top-16 overlap with the engine's own rankings | ≥ 97.6% (worst capture) | ≥ 95.0% |

Both recall figures were also confirmed through the real selection engine (`roma2-pair-eval`), not
only the numpy replay.

---

## Session Lifecycle

One `RoMa2Onnx` per `Scene::MatchPairs` call (`libs/SFM/Scene.cpp`): declared before `PairsMatcher` so
its destructor runs after (a `RoMa2Onnx*`/tensor held by `PairsMatcher` must not outlive the model).
`RoMa2Onnx::Load` loads only the descriptor graph; the coarse-match graph loads lazily on the first
`MatchCoarse` call, on the same execution provider the descriptor session got, and a load failure is
remembered so later calls fail fast. `layers` are **not** cached across the describe pass and the
matching pass (13 GB per 1000 images at base) — the matching pass re-describes on slot load.

`ROMA2Config::IsInProcessEnabled()` gates the whole feature: `enabled && (useRetrieval || useMatching)
&& !ResolveModelPath().empty()`. A requested-but-unavailable model is always an error, never a silent
fallback to the vocabulary tree — `Scene::MatchPairs` checks this before loading anything, and
`CreateStructure` checks it again during option validation so the user gets the hint before any
feature extraction runs.

---

## Execution-Provider Policy

`OnnxModel::Load` (`libs/SFM/OnnxRuntime.cpp`) tries providers in order, keeping the first that
constructs a working `Ort::Session`; a failed append is logged as a warning and the next candidate is
tried:

1. **CUDA** (Linux/Windows) — honours `--gpu-device` through `SEACAVE::CUDA::desiredDeviceIDs` when
   built with CUDA (device 0 otherwise); `--gpu-device -2`/CPU forces the CPU provider even when
   `--roma2-provider` defaults to `auto`.
2. **CoreML** (macOS) — `MLProgram` model format, `MLComputeUnits=ALL`.
3. **DirectML** (Windows builds of ONNX Runtime that include it).
4. **CPU** — always available; ~2-4 s/image at base, warned once.

`--roma2-provider auto|cuda|coreml|dml|cpu` overrides the candidate list. Partial op support on
CoreML/DML (falling back to CPU kernels for unsupported ops) is ONNX Runtime's own business and stays
correct, only speed differs (unmeasured on this repo's captures — see Limitations). Each session's
chosen provider is logged once when it loads (`"ONNX model '<name>' loaded on <provider>"`). CoreML/DML
provider headers are guarded with `__has_include(<coreml_provider_factory.h>)` /
`__has_include(<dml_provider_factory.h>)` so the code only compiles the branch where the header
exists. On Windows, model paths are converted to `std::wstring` for `Ort::Session` (`ORTCHAR_T` is
`wchar_t` there).

---

## Slot Plan (Dense Matching)

`MakeSlotPlan` (`libs/SFM/MatchROMA2.cpp`) schedules which image descriptors stay resident on the
device while running the coarse-match graph pair by pair: Belady's optimal replacement (evict the
slot whose next use is furthest away), over the candidate pairs sorted `(ID1,ID2)`, at most
`--roma2-slots` (default 64, 12.5 MiB each at base) slots. The plan is computed once, then replayed:
the describe-and-load step for a slot runs on the calling thread (ONNX Runtime sessions are used
through one `IoBinding` each, so all calls on one `RoMa2Onnx` must come from a single thread), while
each pair's guided re-match runs on the thread pool. The per-round log line reports the accounting:

```
ROMA2 slot plan: 6 pairs, 4 slots, 4 loads (0 reloads)
ROMA2 dense matching (first round): 6/6 pairs guided, 0 created, 6 replaced, 0 skipped healthy, ...
```

---

## Per-Round Replace Policy

Design decision 6 (polycpp `ShouldReplaceROMA2Pair`, `pose_refine.cpp:506-509`,
`roma2_retrieval.hpp:140`): the first round warps every candidate pair
(`skipHealthyInliers=0`, `maxReplaceInliers=0`); the verification-feedback round skips pairs already
having ≥100 inliers and only replaces pairs with <15 (`feedbackSkipHealthyInliers=100`,
`feedbackMaxReplaceInliers=15`).

A guided pair only replaces or creates a scene pair when (`libs/SFM/MatchROMA2.cpp`,
`ROMA2Warp.cpp:ApplyROMA2Pair`):

1. `MatchFeaturesGeometric` succeeded on the warp-tracked keypoints and produced a non-empty match set
   (a failed guide falls back to what descriptor matching already stored for that pair, which must
   never be offered as a "replacement" for itself);
2. the guided set then **re-verifies** the same way the descriptor path does —
   `PairsMatcher::GeometricFilter` when epipolar verification is enabled, or the plain
   `minMatches` size bar otherwise — so an unverified guided count is never compared against a
   RANSAC-verified existing count;
3. `ApplyROMA2Pair` then applies strict `new > existing` filtered-inlier count (ties keep the existing,
   descriptor-verified pair), and, in the feedback round, only when the existing pair is still below
   `feedbackMaxReplaceInliers`.

---

## Orientation

In-process warps are computed on openMVS *working-orientation* pixels for both images
(`Image::LoadPixels` rotates portrait to landscape; keypoints were extracted from the same pixels), so
no map rotation is ever needed. `RotateMapsForReference`, which existed for warps computed on raw
(un-rotated) files under the deleted NPZ import, is gone with it.

---

## Determinism

Warps are consumed in parallel (thread pool) but results are **applied serially** in `(ID1,ID2)` order
(`FOREACH(p, results)` in `MatchPairsROMA2`) — so which of two near-tied match sets a pair keeps never
depends on which thread-pool task happened to finish first. PoseLib RANSAC is seeded (default seed 0),
so a pair's guided match set is a pure function of its inputs. The coarse graph's `img_A`/`img_B`
inputs are dead (they only reach the DPT head, never used by the coarse output) but are still bound,
from **one shared zero-filled host tensor**, so no per-pair image upload is needed just to satisfy the
graph's I/O contract. Verified end-to-end: two identical `CreateStructure --roma2 ...` runs on the same
input produce byte-identical `--export-pairs-csv` output.

---

## Build and Provisioning

`OPTION(OpenMVS_USE_ONNXRUNTIME ... OFF)` (top-level `CMakeLists.txt`) — detection happens at the
top level (not lib-local) so `_USE_ONNXRUNTIME` lands in `ConfigLocal.h` for every translation unit,
including `apps/` and `apps/Tests/`. Two provisioning routes behind one `find_package(onnxruntime
CONFIG)`:

- **vcpkg manifest features** (`vcpkg.json`): `onnxruntime` (CPU, every platform) and
  `onnxruntime-cuda` (`"platform": "x64 & (linux | windows)"`), kept as separate features so a plain
  `cuda` build never triggers vcpkg's hours-long from-source ONNX Runtime build.
- **`ONNXRUNTIME_ROOT`** pointing at the official prebuilt package for the OS —
  `onnxruntime-linux-x64-gpu-1.23.2.tgz`, `onnxruntime-win-x64-gpu-1.23.2.zip` (or the DirectML NuGet
  layout), `onnxruntime-osx-arm64-1.23.2.tgz` (ships the CoreML EP). `CMakeLists.txt` repairs the
  tarball's `INTERFACE_INCLUDE_DIRECTORIES` when it points at the wrong subdirectory, works around a
  release tarball whose package config references files that do not exist in that tarball's actual
  layout (checked before calling `FIND_PACKAGE`, since CMake's generated targets file would otherwise
  abort configure with an unsuppressable `FATAL_ERROR`), and falls back to a manual
  `find_path`/`find_library` (setting `IMPORTED_IMPLIB` on Windows).

Shared libraries are copied next to the installed binaries either way (vcpkg's applocal step does it
for its own port; the tarball route does it explicitly via `install(FILES ...)` glob of
`onnxruntime*.so*`/`.dylib`/`DirectML.dll`), with an `rpath`/`$ORIGIN` entry added on Linux/macOS before
`libs`/`apps` are added so every target created afterwards inherits it (Windows instead needs the DLLs
copied next to each executable). CPU-only builds still work through the CPU provider.

**Runtime-dependency risk**: on Linux, an `LD_LIBRARY_PATH` that puts a cu13 cuDNN ahead of the cu12
libraries ONNX Runtime 1.23.2 needs makes the CUDA execution provider fail to construct and silently
fall back to CPU (with the warning logged by `OnnxModel::Load`, not a hard error) — use
`/usr/local/cuda-12.9/lib64` (or otherwise ensure the cu12 cuDNN resolves first) when running a CUDA
build.

---

## Memory (fp32, base preset)

| Item | Size |
|---|---|
| `image` tensor | 4.7 MiB |
| `layers` per slot | 12.5 MiB (turbo 3.1, fast 8.0) |
| `value_facets` scratch (+ host readback on the retrieval pass) | 12.5 MiB (+ 12.5 MiB host) |
| Descriptor session weights + arena | 1.22 GB + ≈0.5 GB |
| Match session weights + arena (joint attention over 3200 tokens) | ≈0.46 GB + ≈1.0-1.2 GB |
| Slot pool, 64 slots | 800 MiB |
| **Total** | **≈4 GB** |

## Measured Latencies (CUDA, fp32, median over 100 runs)

| Preset | Descriptor | Match coarse |
|---|---|---|
| base (640) | 42.3 ms | 40.7 ms |
| fast (512) | 25.3 ms | 20.4 ms |
| turbo (320) | 12.6 ms | 8.5 ms |

Source: `~/virginia/models/roma2-onnx/roma2onnx-20260829-facets1520/export.log`.

---

## Tests

- **`RoMa2PreprocessTest`** (`apps/Tests/TestsSFM.cpp`) — always runs; verifies
  `PreprocessImageRoMa2` (the bicubic-antialiased resize reproducing `F.interpolate` exactly) against
  a small synthetic fixture. Skipped only in an OFF build ("built without ONNX Runtime").
- **`RoMa2OnnxParityTest`** — runs only when `OPENMVS_ROMA2_MODEL_PATH` is set (skipped otherwise);
  `OPENMVS_ROMA2_PROVIDER` and `OPENMVS_ROMA2_SETTING` select which execution provider and preset(s)
  it exercises against the shipped `.reference/` dumps (preprocessing, `Describe`, `MatchCoarse`).
  Its full-size preprocessing check uses a **1e-4** bound, not the synthetic fixture's 1e-5: the
  shipped `in_image.npy` reference is itself up to 2.72e-5 away from an exact float64 evaluation of
  the same antialiased-Keys filter (large negative lobes cancel catastrophically on 0..255 samples),
  so no fp32 implementation can reach 1e-5 against it — 1e-4 still separates the right filter from
  every wrong one (plain bicubic ~2e-3 off, `align_corners=true` ~1e-2 off) by more than an order of
  magnitude.
- **`ROMA2ReconstructTest`** — runs only when `OPENMVS_ROMA2_MODEL_PATH` is set; same
  `OPENMVS_ROMA2_PROVIDER`/`OPENMVS_ROMA2_SETTING` coupling. Exercises the full in-process path on the
  bundled 4-image scene: global-descriptor computation, dense matching (created/replaced pairs vs a
  baseline run with matching off), determinism (two runs with the same config match identical pairs),
  and `.sfm` round-trip of `globalDescriptor`.

All three tests are coupled to the same production environment variable
(`OPENMVS_ROMA2_MODEL_PATH`) used by `CreateStructure --roma2-model`'s default — there is no separate
test-only model variable. In an OFF build (`-DOpenMVS_USE_ONNXRUNTIME=OFF`), all three report
"skipped (built without ONNX Runtime)"/"no ONNX Runtime support in this build" and pass trivially.

---

## Limitations

- **Coarse graph only.** No refiner ("regular") stage, and therefore no depth maps from the
  in-process warps — only descriptor/coarse-match are exported and consumed.
- **CoreML/DML speed is unmeasured.** Correctness follows from ONNX Runtime's own provider contract,
  but no latency numbers exist yet for either provider on this repo's hardware.
- **Self-calibration from warp-guided matches is weaker than from descriptor matches.** A guided match
  set is self-consistent with the *coarse* warp's own geometry (a smooth, low-resolution field), so it
  under-constrains focal length/distortion refinement in a way SIFT/AKAZE/ORB's independently-detected
  keypoints do not. Supply intrinsics (EXIF or Polycam priors) when using `--roma2` rather than relying
  on `--use-global-solver`/uncalibrated refinement alone; the effect is to be measured on real
  captures against Polycam poses.
- **The describe pass always runs when `--roma2-retrieval` is on**, even for `EXHAUSTIVE` or
  `SEQUENTIAL` matching where no retrieval ranking is needed for pair selection — the global
  descriptors are still computed and stored (`Image::globalDescriptor`), which costs the describe pass
  but not the (lazy) match-graph load.
