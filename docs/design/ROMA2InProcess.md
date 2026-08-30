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

**What `--roma2 true` does by default:** the retrieval seam only. `--roma2-retrieval` defaults to
**true**, `--roma2-match` to **false** — so a plain `--roma2 true` describes every image once, ranks
the candidate pairs by the global descriptors instead of the vocabulary tree, and leaves the matching
itself to SIFT/AKAZE/ORB. Dense matching is **experimental** and must be asked for
(`--roma2-match true`): it supplies far more pairs and inliers, but degraded pose accuracy on 3 of 5
validation captures when the intrinsics are self-calibrated (see Limitations, and prefer
`--roma2-skip-healthy 100 --roma2-max-replace 15` or imported intrinsics with it).

| Flag | Default | Effect |
|---|---|---|
| `--roma2` | `false` | master switch for the in-process model |
| `--roma2-model DIR` | `$OPENMVS_ROMA2_MODEL_PATH` | exported graphs + manifest |
| `--roma2-setting turbo\|fast\|base` | `base` | preset (320/512/640 px) |
| `--roma2-provider auto\|cuda\|coreml\|dml\|cpu` | `auto` | execution provider; a named one is required, not preferred |
| `--roma2-retrieval` | `true` | rank candidate pairs by the global descriptors |
| `--roma2-retrieval-recipe facets\|layers` | `facets` | pooling recipe (2048-D default, 1024-D parity) |
| `--roma2-match` | **`false`** | experimental: dense-match candidate pairs and replace weaker matches |
| `--roma2-slots N` | `64` | image descriptors resident on the device while dense matching |
| `--roma2-skip-healthy N` | `0` | round 1: skip pairs already at ≥ N inliers |
| `--roma2-max-replace N` | `0` | round 1: replace only pairs below N inliers |
| `--roma2-min-overlap F` | **`0`** (off) | create a pair the descriptor matcher did not verify only if ≥ F of the warp is confidently overlapping |
| `--roma2-cross-check B` | **`false`** | drop a guided match when a closer keypoint of A claims the same keypoint of B |
| `--export-retrieval-csv F` | — | per-image retrieval rankings (needs `--roma2-retrieval`) |

`--export-retrieval-csv` and `--export-pairs-csv` are both written by `Scene::Reconstruct()` right
after pair matching (`ReconstructionConfig::exportRetrievalCSV`/`exportPairsCSV`), before any
reconstruction step (largest-connected-component clustering, weak-image filtering, resection) can
drop pairs or leave images unregistered — the CSVs describe the matched scene, not whatever
reconstruction happened to keep. A failed export only logs a warning and never fails the
reconstruction, whose primary output is the scene itself.

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
  `sign(d)·|d|^p` (power normalization) → L2, where `p` is the manifest's own
  `retrieval_recipes.facets.power` (0.3 in the shipped exports). `ROMA2Config::retrievalPower`
  defaults to `0`, meaning "whatever the model was exported with"; a positive value overrides it.
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

`--roma2-provider auto|cuda|coreml|dml|cpu` overrides the candidate list, and a provider named
explicitly is a **requirement, not a preference**: if it is not in `Ort::GetAvailableProviders()` or
its session fails to construct, `OnnxModel::Load` returns false with an error naming it and the run
stops, instead of quietly finishing on the CPU at seconds per image. Only `auto` walks the chain
above and ends on CPU (with the one-time "running on the CPU execution provider" warning). Partial
op support on
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
ROMA2 dense matching (first round): 6/6 pairs guided, 0 created, 6 replaced, 0 gated, 0 skipped healthy, ...
```

---

## Per-Round Replace Policy

Design decision 6 (polycpp `ShouldReplaceROMA2Pair`, `pose_refine.cpp:506-509`,
`roma2_retrieval.hpp:140`): the first round warps every candidate pair
(`skipHealthyInliers=0`, `maxReplaceInliers=0`); the verification-feedback round skips pairs already
having ≥100 inliers and only replaces pairs with <15 (`feedbackSkipHealthyInliers=100`,
`feedbackMaxReplaceInliers=15`).

`--roma2-skip-healthy N` (default 0) and `--roma2-max-replace N` (default 0) expose the **round-1**
half of that policy on the command line (`skipHealthyInliers` / `maxReplaceInliers`; the feedback
round's 100/15 are not exposed), so `--roma2-skip-healthy 100 --roma2-max-replace 15` makes round 1
fill only where descriptor matching is weak instead of warping and replacing every candidate.

A guided pair only replaces or creates a scene pair when (`libs/SFM/MatchROMA2.cpp`,
`ROMA2Warp.cpp:ApplyROMA2Pair`):

0. the **confident-overlap gate** let it through: `--roma2-min-overlap F` (default 0 = off) refuses to
   *create* a pair the descriptor matcher did not verify unless at least F of the warp's cells survive
   the erosion with confidence ≥ `minConfidence`. Existence is read from the pass's `pairIndexMap`, so
   a pair the descriptor matcher verified is never gated whatever the warp says, and the gate fires
   before tracking and guided matching (the pair is counted in the summary line's `%u gated`).
   `overlapRatio`/`overlapArea` of a created pair still stay 0 — a gate is not a weight;
1. `MatchFeaturesGeometric` succeeded on the warp-tracked keypoints and produced a non-empty match set
   (a failed guide falls back to what descriptor matching already stored for that pair, which must
   never be offered as a "replacement" for itself). With `--roma2-cross-check` (default **false**)
   its forward selection is also **train-side cross-checked**: a match (i→j) survives only if, among
   all keypoints of A whose selected candidate is j, i has the smallest descriptor distance (ties keep
   the smaller queryIdx). The check is restricted to those forward candidate sets — no reverse
   epipolar pass is run — and it is the only mode in which the single-candidate case gets its
   descriptor distance computed at all, since a left-at-zero distance would win every collision;
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
`libs`/`apps` are added so every target created afterwards inherits it. CPU-only builds still work
through the CPU provider.

**Not implemented yet, tarball route on Windows:** Windows has no `rpath`, so `onnxruntime.dll` (and
`DirectML.dll`) must sit next to the executable that loads it. vcpkg's applocal step does that
automatically for its own port; the `ONNXRUNTIME_ROOT` route only `install(FILES ...)`s them into the
install `bin/`, and adds **no** `POST_BUILD` copy next to the executables in the build tree — the
`CMakeLists.txt` comment beside the `rpath` block says so explicitly. Until that step exists, running
a tarball-provisioned Windows *build tree* needs the DLLs copied next to the binaries by hand or
`ONNXRUNTIME_ROOT/lib` on `PATH`. Linux and macOS are unaffected (they get `$ORIGIN`/`@executable_path`
plus the library directory).

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
| **Total (analytic floor)** | **≈4 GB** |

Those are the tensors and weights the pipeline asks for. What the device actually holds is larger,
because ONNX Runtime's CUDA arena grows on demand and never returns memory. Measured with
`nvidia-smi --query-compute-apps` sampled at 2 s through a live `base` run with 64 slots (251 images,
A100 40 GB; `VALIDATION-20260830.md` §5):

| Stage | Device memory |
|---|---|
| SIFT extraction, before any ONNX session | ~1.0 GB |
| Descriptor session up (retrieval only) | 2.6 GB |
| Steady state through dense matching | 7.6 GB |
| Peak, at the round-1 → feedback-round transition | 12.0 GB |

So the analytic table is a **floor**, not a bound: budget from the measured column. Retrieval-only
(`--roma2-match false`, the default) never opens the coarse-match session and stays at the 2.6 GB
row — it runs comfortably on a 4 GB device at any slot count, `--roma2-slots` being a dense-matching
knob only. Dense matching needs headroom for the 12.0 GB transition peak, so on a 16 GB device drop
to `--roma2-slots 16` (200 MiB of slots instead of 800 MiB, at the cost of more re-describes — 34-55%
of slot loads are already reloads at 64) and expect the arena, not the slots, to dominate; below
~12 GB of free device memory, dense matching at `base` is not a good fit — use `--roma2-setting fast`
or `turbo`, whose tensors are 8.0/3.1 MiB per slot.

## Measured Latencies (CUDA, fp32, median over 100 runs)

| Preset | Descriptor | Match coarse |
|---|---|---|
| base (640) | 42.3 ms | 40.7 ms |
| fast (512) | 25.3 ms | 20.4 ms |
| turbo (320) | 12.6 ms | 8.5 ms |

Source: `~/virginia/models/roma2-onnx/roma2onnx-20260829-facets1520/export.log`.

---

## Tests

- **`ROMA2WarpTrackingTest`** (`apps/Tests/TestsSFM.cpp`) — always runs, no model needed: keypoint
  tracking through a synthetic identity warp (the pixel↔grid↔normalised coordinate conventions of
  `ROMA2Warp.h/cpp`), the confidence gate and the border erosion of the confidence map, and
  `ApplyROMA2Pair`'s store/replace-by-inlier-count policy (including the `maxReplaceInliers` ceiling).
- **`GlobalDescriptorsQueryTest`** — always runs, no model needed: the cosine ranking over
  `Image::globalDescriptor` and its deterministic tie order, the `PairsMatcher::QueryRetrieval`
  dispatch that ranks candidate pairs through the descriptors instead of the vocabulary tree, the
  `--export-retrieval-csv` rankings export, the `.sfm` round-trip of the descriptors, and both
  host-side pooling recipes against the export script's fixtures.
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

The three model-driven tests are coupled to the same production environment variable
(`OPENMVS_ROMA2_MODEL_PATH`) used by `CreateStructure --roma2-model`'s default — there is no separate
test-only model variable. In an OFF build (`-DOpenMVS_USE_ONNXRUNTIME=OFF`), all three report
"skipped (built without ONNX Runtime)"/"no ONNX Runtime support in this build" and pass trivially,
while the two always-on tests keep running (they exercise host-side code only).
`RoMa2OnnxParityTest` additionally skips — loudly, with "no reference dumps under '<dir>', parity not
checked" — when the model directory ships graphs but no `.reference/` dumps, since there is then
nothing to compare; naming a preset through `OPENMVS_ROMA2_SETTING` still fails hard in that case,
because the user then asked for a comparison that cannot be made.

---

## Compatibility (`.sfm` scene files)

Storing the per-image global descriptor changed the SFM project stream layout, so
`SFM_PROJECT_VERSION` (`libs/SFM/Scene.cpp`) went **0 → 1**. The loader accepts that exact version
only, and refuses anything else outright:

```
error: unsupported SFM project version 0 (this build reads only version 1) in '<file>'
```

There is no converter and none is planned: `.sfm` files written by an earlier build must be
regenerated by re-running the matching stage from the images (`CreateStructure -s <images> -o
scene.sfm ...`). `.mvs` files are unaffected.

---

## Limitations

- **Coarse graph only.** No refiner ("regular") stage, and therefore no depth maps from the
  in-process warps — only descriptor/coarse-match are exported and consumed.
- **CoreML/DML speed is unmeasured.** Correctness follows from ONNX Runtime's own provider contract,
  but no latency numbers exist yet for either provider on this repo's hardware.
- **Dense matching (`--roma2-match`) is experimental and off by default.** Measured end to end on five
  Polycam captures against Polycam GT (`VALIDATION-20260830.md` §3/§4/§6), the dense arm buys a lot of
  matching and loses on pose:
  - *What it buys:* +57 % to +111 % geometrically verified pairs, and **2.8×–5.0× the median inlier
    count** on the pairs both arms found (17×–82× more pairs improved than worsened). Non-temporal pair
    recall against the pseudo-GT rises from 0.46–0.76 to 0.76–0.88.
  - *What it costs:* measured **like-for-like** (alignment-free relative rotation, median over the
    images both arms registered) it is better on 1 capture (−11 %), a wash on 1 (+2 %) and **worse on
    3** (+9 %, +24 %, +85 %). The focal drifts **+1.2 to +6.2 px** (0.2–0.9 %) and only in this arm —
    the retrieval-only control tracks the SIFT baseline to within 1 px everywhere, which attributes the
    drift to the *replaced* warp-guided correspondences, not to the added pairs. Ceres reports
    `Linear solver failure ... dense Cholesky` **257–1194 times per capture** against 8–56 for the SIFT
    and retrieval-only arms, and reconstruction runs **15–18× longer** (one capture: ~18 min vs ~1 min).
  - *And on a repetitive scene the view graph fragments:* on capture `32265651` the retrieval-proposed,
    warp-verified pairs left the reconstruction on **146 of 377 images, out of 8 components**, where
    SIFT kept 309. The fill-only experiment below keeps every pair the dense arm added and still
    fragments (176/377, and a registered set *disjoint* from the first run's), which pins the
    fragmentation on the **added pairs themselves**, not on the replacement of SIFT match sets. The
    confident-overlap gate below attacks exactly those pairs and does *not* fix it either: alone it
    reaches 124/377, and together with the cross-check 192/377 — a spread that says more about how
    unstable this capture's reconstruction is than about either knob.
  - *Mitigations,* in order of how much they cost you:
    1. **Retrieval-only** — the default (`--roma2-retrieval true --roma2-match false`). It was better
       than or equal to SIFT on 4 of 5 captures, keeps the focal within 1 px of the SIFT arm, and costs
       one describe pass.
    2. **Fill-only dense matching** — `--roma2-match true --roma2-skip-healthy 100 --roma2-max-replace 15`
       makes round 1 behave like the feedback round: warp only where descriptor matching is weak,
       replace only the weakest pairs. This keeps **100 % of the pair-coverage gain** (identical pair
       sets, identical recall), removes the focal drift entirely (back to the SIFT value on all four
       captures tried), and cuts reconstruction time up to 3.5× where the replacement
       volume was largest (1.05–3.5× over the four captures). It does *not* close the pose gap
       (+5.7 %/+20.7 %/+35.6 % on three of four) and does *not* fix the fragmentation, which is why it
       is a knob and not the default (ruling R32).
    3. **Known intrinsics** — importing intrinsics (EXIF, `--import-poses-mode 1`, Polycam priors)
       removes the self-calibration failure mode the drift and the ill-conditioning come from. A guided
       match set is self-consistent with the *coarse* warp's own geometry — a smooth, low-resolution
       field — so it under-constrains focal/distortion refinement in a way independently-detected
       SIFT/AKAZE/ORB keypoints do not.
  - *Both follow-ups are now implemented*, each behind its own knob, and each was measured **on its
    own arm** against the pre-registered bar (follow-up campaign, 2026-08-30; run folders
    `<capture>/openmvs-roma2-20260830-roma2{gate-only,cc,gate,gate-diag,gate-diag-nocc}/`). **Both
    miss the bar and both stay off**, and both remain available as knobs:
    1. **A confident-overlap gate on created pairs** — `--roma2-min-overlap F` /
       `ROMA2Config::minCreatedOverlap` (default **0 = off**). A pair the descriptor matcher did not
       verify is created out of nothing but the warp, so the gate asks the eroded confidence map what
       fraction of the warp grid is confident (≥ `minConfidence`) and refuses to create the pair below
       `F`. Existing pairs are never gated. `F = 0.05` was chosen on `32265651` as the **knee** of a
       0.05–0.60 sweep over the 5774 created-pair attempts of a diagnostic pass, labelled against the
       capture's own ARKit depth (`scripts/python/tests/pair_gt_labels.py`, coverage mode: a pair is
       plausible when the optical axes agree to 90° *and* each frame sees ≥ 15 % of the other's valid
       depth pixels, implausible below 3 %, ambiguous in between and excluded). No threshold in that
       range met the pre-registered target of removing ≥ 90 % of the implausible created pairs while
       keeping ≥ 80 % of the plausible ones; at the knee `F = 0.05` it removes **86.6 %** of the
       implausible created pairs and keeps **86.3 %** of the plausible ones, and the confident-overlap
       fraction is a good discriminator of the two (ROC AUC **0.933**)
       (`openmvs-roma2-20260830-roma2gate-diag/GATE-SWEEP.md`).
       What it buys, measured alone (`…-roma2gate-only/RESULTS.md`): on the control capture
       `f7dbf861` it matches the plain dense arm and edges past it — **308 of 345** registered against
       309, rotation mean 0.899° vs 0.988° and median 0.393° vs 0.382° (+2.9 %, inside the ruling's
       +15 %), better position mean/median/max, a like-for-like alignment-free penalty of **+3.9 %**
       against the plain arm's +9.5 %, Ceres `Linear solver failure` **157 vs 257** and matching
       **389 s vs 588 s** — while creating 1163 pairs instead of 2205. On the repetitive `32265651`
       it does **not** fix the fragmentation: **124 of 377** registered (30 components), *fewer* than
       the plain dense arm's 146 and far from the **250** the ruling required, even though every
       `--compare-mvs` statistic there improves too (rotation 1.445/1.105/6.407 vs 1.712/1.229/6.682,
       like-for-like +7.4 % vs +23.7 %, 309 solver failures vs 452). That clause fails, so the default
       stays **0** — but on a capture like `f7dbf861` the knob costs nothing and buys speed.
       That 146 → 124 → 192 spread across three configurations of the same matcher (plain dense,
       gate-only at 0.05, gate + cross-check at 0.20) is itself the finding: on this capture the
       incremental reconstruction latches onto whichever self-consistent sub-graph it meets first, and
       any change to the pair set moves it to a different one — the `roma2` and `roma2gate` arms
       register **disjoint** image sets.
    2. **A train-side cross-check in guided matching** — `--roma2-cross-check B` /
       `ROMA2Config::guidedCrossCheck` (default **false**). `MatchFeaturesGeometric` is one-sided by
       construction: for each keypoint of A it keeps the descriptor-best keypoint of B among those
       near the epipolar line / the tracked point, subject to a ratio test *within that candidate
       set*, so several keypoints of A may claim the same keypoint of B. With the check on, a match
       (i→j) survives only if, among all keypoints of A whose selected candidate is j, i has the
       smallest descriptor distance (ties keep the smaller queryIdx); no reverse epipolar pass is run.
       A median pair loses **30.9 %** of its forward matches to such a collision — and removing them
       costs **no inliers**: measured against an otherwise identical pass with the check off, the
       verified inlier median per pair is **126 either way** on `32265651` (mean 221.9 vs 221.3,
       7170 pairs vs 7174) and **158 vs 157** on `f7dbf861` (5221 pairs vs 5226), so the ruling's
       inlier clause passes with a **0.0 % / +0.6 %** change. Measured alone
       (`…-roma2cc/RESULTS.md`), every `--compare-mvs` rotation and position statistic improves on
       both captures — `32265651` 1.304/0.982/4.143° and 0.0162/0.0122/0.0514 against
       1.712/1.229/6.682° and 0.0189/0.0129/0.0669; `f7dbf861` 0.459/0.361/5.451° and
       0.0109/0.0093/0.0448 against 0.988/0.382/25.517° and 0.0179/0.0102/0.3792, with the images over
       10° falling from 9 to 0. **But it registers far fewer images**: **137 of 377** on `32265651`
       (against 146) and **183 of 345** on `f7dbf861` (against 309), and on the latter its
       **like-for-like alignment-free rotation is worse** — **+18.1 %** against the SIFT baseline where
       the plain dense arm is +9.5 %.
       Ruling R-F10 settles how those are weighed: *"The cross-check clause's 'neither capture's
       `--compare-mvs` statistics worsen' is read over a comparable registered set — `--compare-mvs`
       scores each arm only on the images it registered, so a 41 % smaller (183 vs 309) and easier
       survivor set cannot be compared on those numbers; the one statistic that is comparable across
       arms, the like-for-like alignment-free rotation the brief asked for, is worse on `f7dbf861`
       (+18.1 % vs +9.5 % against sift). The clause fails on the control capture — default false,
       `--roma2-cross-check true` stays available."* So the default is **false**, and the knob is
       there for anyone who wants the higher-precision correspondences (it costs nothing at matching
       time and no verified inliers).
       *Footnote:* an earlier run of both changes together at `F = 0.20`
       (`…-roma2gate/RESULTS.md`, kept as a record) reached 192/377 on `32265651` and 332/345 on
       `f7dbf861` with the best like-for-like rotation of the whole campaign (**−2.1 %** against the
       SIFT baseline); it is not what either default rests on, since it cannot separate the two
       changes' contributions.
- **The describe pass always runs when `--roma2-retrieval` is on**, even for `EXHAUSTIVE` or
  `SEQUENTIAL` matching where no retrieval ranking is needed for pair selection — the global
  descriptors are still computed and stored (`Image::globalDescriptor`), which costs the describe pass
  but not the (lazy) match-graph load.
