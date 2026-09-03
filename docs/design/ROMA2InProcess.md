# RoMa v2 In-Process — Retrieval and Dense Matching

## Overview

RoMa v2 (DINOv3 backbone + a coarse matcher head) runs **in-process** through ONNX Runtime, as an
optional replacement for two stages of the classical SFM pipeline. It plugs into two independent
seams:

```
CreateStructure --roma2 ...                    Scene::MatchPairs
  --roma2-retrieval ──> global descriptors ──> PairsMatcher::QueryRetrieval
                         (GlobalDescriptors.h)   (replaces the vocabulary tree as the ranking source;
                                                   RRF/mutual-top-K/bridging/feedback unchanged)
  --roma2-match ─────> bidirectional warp ────> one-pass dense pair matching
                         (RoMa2Matcher.h/cpp,      (MatchROMA2.h/cpp: verdict, guided sparse match,
                          OnnxRuntime.h/cpp)         dense fill, one union fit, one store —
                                                      ROMA2Warp.h/cpp, MatchGeometric.h/cpp)
```

**What `--roma2 true` does by default:** the retrieval seam only. `--roma2-retrieval` defaults to
**true**, `--roma2-match` to **false** — so a plain `--roma2 true` describes every image once, ranks
the candidate pairs by the global descriptors instead of the vocabulary tree, and leaves the matching
itself to SIFT/AKAZE/ORB. `--roma2-match true` asks for one-pass dense pair matching instead: the
verdict on a pair comes from the bidirectional warp alone, with no union of SIFT-verified and
warp-verified pairs and no SIFT fallback for a pair the warp rejects (One-Pass Dense Pair Matching,
below). End-to-end reconstruction numbers for the one pass are not yet in this document (Limitations).

| Flag | Default | Effect |
|---|---|---|
| `--roma2` | `false` | master switch for the in-process model |
| `--roma2-model DIR` | `$OPENMVS_ROMA2_MODEL_PATH` | exported graphs + manifest |
| `--roma2-setting turbo\|fast\|base` | `base` | preset (320/512/640 px) |
| `--roma2-provider auto\|cuda\|coreml\|dml\|cpu` | `auto` | execution provider; a named one is required, not preferred |
| `--roma2-retrieval` | `true` | rank candidate pairs by the global descriptors |
| `--roma2-match` | **`false`** | one-pass dense pair matching (verdict, guided sparse match, dense fill, one union fit, one store) |
| `--roma2-slots N` | `64` | image descriptors resident on the device while dense matching |
| `--roma2-min-confidence F` | `0.1` | confidence at which a warp cell takes part in the verdict, the keypoint tracking and the dense fill |
| `--roma2-min-overlap F` | `0.10` | verdict: the pair is admitted iff min(inlier area A, inlier area B) ≥ F |
| `--roma2-dense-matches N` | `2000` | dense fill cap per pair |
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
(the two RoMa v2 sessions, `RoMa2Onnx::MatchCoarse`), `libs/SFM/MatchROMA2.h/cpp` (describe pass, the
verdict, pair assembly and storage, `MatchPairsROMA2`, `ROMA2Config`), `libs/SFM/ROMA2Warp.h/cpp` (warp
types, coordinate conventions, keypoint tracking, the coverage and complementary draws, dense append),
`libs/SFM/MatchGeometric.h/cpp` (guided sparse matching, `MatchFeaturesGuided`),
`libs/SFM/GlobalDescriptors.h/cpp` (cosine retrieval index over the graph-pooled descriptors). CLI:
`apps/CreateStructure/CreateStructure.cpp` (`--roma2*`, `--export-retrieval-csv`).

---

## Graph Contract

Per preset `S ∈ {320 (turbo), 512 (fast), 640 (base)}`, `G = S/16` patch grid, `C = S/4` warp cells.
Manifest `format_version` **3**:

| File | Inputs | Outputs |
|---|---|---|
| `roma_<setting>_descriptor_fp32.onnx` (+`.onnx.data`) | `image` `[1,3,S,S]` f32, RGB planar, values in [0,1], un-normalised (ImageNet mean/std applied in-graph) | `layers` `[1,2,G,G,1024]` f32 channels-last (blocks 11+17, patch tokens only); `value_facets` `[1,2,G,G,1024]` f32 channels-last (V projections of blocks 15+20, before attention weighting/`o_proj`); `retrieval` `[1, facetsDim]` f32, the FACETS recipe pooled end to end on device (**mandatory** — a graph without it fails to load, see Retrieval Recipe below) |
| `roma_<setting>_match_coarse_fp32.onnx` (+`.onnx.data`) | `descriptors_A`, `descriptors_B` `[1,2,G,G,1024]` f32 — the dead `img_A`/`img_B` inputs are dropped from the trace; if the wrapper cannot be traced without them the exporter keeps them and says so in its report, and the C++ binds them only when the loaded match graph still declares them (read from the ONNX session's own input metadata, not the manifest's `io` block — that block is `RoMa2Manifest::Load`'s optional shape-consistency check, not what decides binding; the one tolerated variation, decided by the exported graph, not a compatibility path) | `warp` `[1,C,C,2]`, `confidence` `[1,C,C,1]` f32 (A→B, normalised (x,y) ∈ [-1,1], align_corners=False; confidence a raw overlap logit, sigmoid on host); `warp_BA`, `confidence_BA`, the same for B→A, from the same forward pass (`bidirectional=True`) — one graph call states both directions of the pair |
| `roma_<setting>.json` | openMVS manifest: `format_version`, `model`, `setting`, `image_size`, `patch`, `layers`, `value_facet_blocks`, `warp_size`, `confidence_channels`, `retrieval_recipes` (`facets`/`layers` dim + GeM params), `files`, `io` (shapes), `opset`, checksums | |

fp32, static shapes, batch 1, opset 18. Coordinate conventions (`ROMA2Warp.h/cpp`, deliberate
asymmetry): pixel→grid `CoordFromTo` is align_corners=**true**; grid→pixel `DenormCoord` is
align_corners=**false** (`0.5*(n+1)*W - 0.5`). `RoMa2Manifest::Load` (`libs/SFM/RoMa2Matcher.h/cpp`)
rejects a manifest of any `format_version` other than 3, naming the version, and rejects a
missing/ill-typed key or a declared graph I/O that disagrees with the shapes derived from
`image_size`/`warp_size`; the C++ loader (`OnnxModel::Load`, `libs/SFM/OnnxRuntime.cpp`) separately
rejects any negative (dynamic) dim in the graphs themselves.

Scope is descriptor + coarse-match only — the refiners stay unexported: they buy match precision,
which the sparse inliers already supply wherever the scene has texture (One-Pass Dense Pair Matching,
below), and the export cost is the same argument it always was (the `local_corr` CUDA extension,
VGG19-BN fine features, `grid_sample`). No depth maps are produced by the in-process warps.

### Export tooling and provenance

Graphs are produced by `scripts/python/roma2/export.sh` (wraps `export.py onnx|check|manifest` and
`parity.py`), run inside the polyml export project environment (`uv run --project ~/polyml/romav2
--with onnxruntime-gpu==1.23.2`). `RoMa2OnnxParityTest`'s reference dumps (`*.reference`,
`save_reference`) and parity check cover all four `match_coarse` outputs.

Exported model sets live on the shared volume, one directory per export — the bidirectional export is
`~/virginia/models/roma2-onnx/roma2onnx-20260903-bidir/` (three presets, references, `export.log`,
`format_version` 3), referenced by `--roma2-model` or `$OPENMVS_ROMA2_MODEL_PATH` from then on. The
`.onnx` + `.onnx.data` + `.json` set is byte-portable across OSs (external data is resolved relative to
the model path on every platform). Earlier exports (`format_version` 1, no `retrieval` output;
`format_version` 2, unidirectional `match_coarse`) are unsupported and rejected at load, by name.

---

## Retrieval Recipe

The global retrieval descriptor (FACETS, 2048-D: per-slice GeM p=3 on `value_facets` → L2 per
slice → concat → L2 → `sign(d)·|d|^p` power normalization → L2, `p` = the export's `FACETS_POWER`)
is pooled end to end **on device**, inside the exported descriptor graph, and read back as the
`retrieval` output (`RoMa2Onnx::Describe`, `libs/SFM/RoMa2Matcher.h/cpp`) -- one path, no
runtime-configurable recipe or host-side pooling. A manifest that does not declare a `retrieval`
output (`format_version` 1) is an unsupported model and fails loudly at load, naming the model
directory and the missing output.

An earlier revision of this branch pooled FACETS (and a legacy LAYERS recipe: GeM p=3 on `layers`
slice 1) on the CPU (`SFM::PoolRetrievalDescriptor`); once the graph took over the pooling that CPU
path was kept only long enough to compare its output against the graph's, then deleted along with
the LAYERS recipe (task 1b of the matching redesign, 2026-08-31). The comparison it existed for
lives on as `RoMa2OnnxParityDescribe`'s retrieval check (`apps/Tests/TestsSFM.cpp`), which reads the
graph's `retrieval` output back and judges it against the independent Python `pool_retrieval`
reference (`scripts/python/roma2/graphs.py`) at cosine ≥ 0.99999 -- the C++ GeM was matched to that
reference to 6e-8 before the CPU path was retired.

Measured against `~/virginia/models/roma2-onnx/roma2onnx-20260829-facets1520/CROSSCHECK.md` (the
pre-GPU-pooling export), seven LiDAR captures (the engine consumed `keyframes/images`, not
`corrected_images`):

| Measure | Value | Bound / reference |
|---|---|---|
| FACETS `value_facets` cosine vs the campaign's torch taps (t15v/t20v) | ≥ 0.999990 (worst per-image) | ≥ 0.999 |
| FACETS recall@16 (non-temporal) | 0.7955 | shipped 0.797 ± 0.01 |
| Mean per-image top-16 overlap with the engine's own rankings | ≥ 97.6% (worst capture) | ≥ 95.0% |

The recall figure was also confirmed through the real selection engine (`roma2-pair-eval`), not
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

One graph call per pair states both directions (Graph Contract, above), so only the (A,B) pair with
A < B is ever warped — the candidate list holds each unordered pair once (`MakePairIdx` orders the
indices; `MatchPairsROMA2` asserts it and de-duplicates after sorting).

`MakeSlotPlan` (`libs/SFM/MatchROMA2.cpp`) schedules which image descriptors stay resident on the
device while running the coarse-match graph pair by pair: Belady's optimal replacement (evict the slot
whose next use is furthest away), over the candidate pairs sorted `(ID1,ID2)`, at most `--roma2-slots`
(default 64, 12.5 MiB each at base) slots. Processing pairs in `(ID1,ID2)` order — not, say, retrieval
rank — is what makes the slot plan cheap: consecutive pairs share their first image, so every use of
an image falls inside a short window and the plan keeps the rest resident up to the budget instead of
thrashing. The plan is computed once, then replayed: the describe-and-load step for a slot runs on the
calling thread (ONNX Runtime sessions are used through one `IoBinding` each, so all calls on one
`RoMa2Onnx` must come from a single thread), while each pair's verdict, guided match and assembly run
on the thread pool; assembled pairs are stored serially, in `(ID1,ID2)` order (the keypoint indices
`StorePairROMA2` hands out depend on what the two images already carry).

The plan is logged once it is computed, and the pass's own summary line (below) folds the same
slot/load/reload counts back in next to the matching result, so the cache cost of the order is always
visible beside what it bought:

```
ROMA2 slot plan: 3399 pairs, 139 slots, 218 loads (0 reloads)
```

Measured on a 225-image capture: 3399 candidate pairs needed 139 slots, 218 loads and 0 reloads — the
`(ID1,ID2)` order keeps almost every image's descriptor resident across the whole window it is needed
in, with essentially no re-describe cost.

---

## One-Pass Dense Pair Matching (`--roma2-match`)

With `--roma2-match true` the verdict on a candidate pair (A, B), A < B, comes from the bidirectional
warp alone, in one pass: no union of SIFT-verified and warp-verified pairs, and no fallback to SIFT for
a pair the warp rejects. Per pair:

1. **Warp, both ways, one graph call.** `RoMa2Onnx::MatchCoarse` (Graph Contract, above) returns the
   A→B warp and confidence and, from the same joint-ViT pass, the B→A warp and confidence
   (`PairWarps`, `ROMA2Warp.h`).
2. **Verdict** (`JudgePairROMA2`). One geometry is fitted, through `PairsMatcher::GeometricFilter`, on
   a coverage-uniform sample of A's confident cells (`SampleWarpByCoverage`, target 4000). Its inlier
   area is then measured over ALL of A's eligible cells and, under the same geometry, over all of B's
   eligible cells through the B→A warp. The pair is admitted iff `min(inlierAreaA, inlierAreaB) >=
   --roma2-min-overlap`; a rejected pair is dropped — no SIFT matching, no second chance in this round.
3. **Guided sparse matching** (`MatchFeaturesGuided`, `MatchGeometric.h`). The described keypoints of A
   are tracked into B through the warp (`TrackKeypointsByWarp`); for each, the candidates are B's
   described keypoints inside a disc of two warp cells around the prediction, and the winner is the
   best descriptor distance in the disc, accepted iff it beats the best descriptor distance OUTSIDE
   the disc by the matcher's ratio. This is where appearance (descriptor agreement) enters, and the
   only place it does.
4. **Dense fill** (`SampleWarpComplementary`). Correspondences are drawn from the verdict's inlier
   cells where the guided matches are not, up to `--roma2-dense-matches` per pair.
5. **One geometry for the pair** (`AssemblePairROMA2`). `PairsMatcher::GeometricFilter` runs once more
   on guided ∪ dense; the guided matches within the matcher's own epipolar tolerance become the pair's
   sparse (descriptor) evidence, the dense correspondences within the warp tolerance its dense segment.
   When the union fit fails, the verdict's own geometry stands and classification runs against it
   instead — an admitted pair is always stored, and a pair with zero sparse inliers is dense-only, its
   dense segment its whole evidence. The pair is stored once (`StorePairROMA2`): created, or replacing
   a same-key pair a previous `Match()` left.

### Why the min-side inlier area

A hallucinated warp is a smooth field, locally a homography, and every homography is explained exactly
by a family of fundamental matrices: a RANSAC inlier count or ratio on the warp's own cells cannot tell
a hallucination from a true pair — measured identical on the labelled none/good groups of all three
captures below, and a lower confidence floor only makes the count rule worse. The min-side inlier AREA
of one geometry does separate them: at floor 0.1 and θ = 0.10 it admitted zero hallucinated
(non-co-visible, warp-wrong) pairs on 8d2f4877, 38004114 and Truck, at both floors, without any
cycle-consistency test. The B side is what supplies that precision — it rejects the "whole of A onto a
few pixels of B" pairs, and 10 of Truck's 11 A-side-only admissions.

### Measured basis

Measured on 8d2f4877 (LiDAR interior), 38004114 (textureless interior) and Truck. Recall — the kept
share of the pairs whose true (GT) overlap falls in each band — at confidence floor 0.1, θ = 0.10,
against the SIFT baseline on the same candidate pairs:

| capture | rule | .15–.25 | .25–.40 | .40–.60 | ≥.60 |
|---|---|---|---|---|---|
| 8d2f4877 | one-pass | 0.88 | 0.91 | 1.00 | 1.00 |
| 8d2f4877 | SIFT | 0.47 | 0.60 | 0.75 | 1.00 |
| 38004114 | one-pass | 0.78 | 0.95 | 0.98 | 0.97 |
| 38004114 | SIFT | 0.50 | 0.38 | 0.78 | 0.90 |
| Truck | one-pass | 1.00 | 1.00 | 1.00 | 1.00 |
| Truck | SIFT | 0.53 | 0.55 | 0.75 | 0.97 |

The measured min-side inlier area is 0.55–0.75 of the true (GT) overlap, so θ = 0.10 asks for roughly
0.15–0.17 of true overlap and θ = 0.15 for about a quarter of it. Coarse θ = 0.08 measures the same
verdict as fine θ = 0.10; the refiners are not exported because match precision is
what they would buy, and the sparse (guided) inliers already supply that wherever the scene has
texture (Graph Contract, above).

### Interfaces

Headers, `libs/SFM/MatchROMA2.h` unless noted:

```cpp
struct PairVerdict {
    bool admitted = false;
    float confidentAreaA = 0.f, confidentAreaB = 0.f; // share of each warp grid at conf >= minConfidence, in-frame
    float inlierAreaA = 0.f, inlierAreaB = 0.f;       // share of each warp grid the fitted geometry explains
    std::vector<Point2f> inliersA, inliersB;          // A's inlier cells: the dense fill's population
    std::vector<float> confidences;
};

void JudgePairROMA2(const PairsMatcher&, const Image& imgA, const Image& imgB,
    const PairWarps& warps, const ROMA2Config& config, ImagePair& pair, PairVerdict& verdict);

// MatchGeometric.h
size_t MatchFeaturesGuided(PairsMatcher&, const Image& imgA, const Image& imgB,
    const std::vector<Point2f>& trackedB, const std::vector<uchar>& trackStatus,
    float discRadius, unsigned threadIdx, std::vector<DMatch>& matches);

struct DenseMatches { std::vector<Point2f> pointsA, pointsB; std::vector<float> confidences; };

bool AssemblePairROMA2(const PairsMatcher&, const Image& imgA, const Image& imgB,
    const PairVerdict& verdict, const std::vector<DMatch>& guided, const ROMA2Config& config,
    int warpSize, ImagePair& pair, DenseMatches& dense);

void StorePairROMA2(Scene&, std::unordered_map<PairIdx::PairIndex, IIndex>& pairIndexMap,
    ImagePair&& pair, const DenseMatches& dense, int warpSize);

bool MatchPairsROMA2(PairsMatcher&, RoMa2Onnx&, const PairIdxArr& candidatePairs,
    const ROMA2Config&, unsigned& numStored);
```

`PairsMatcher::GeometricFilter` gains a `(const Image&, const Image&, ImagePair&, float
maxEpipolarError)` overload; the existing three-argument form forwards `config.maxEpipolarError`. The
warp tolerance the verdict and the dense segment both classify against is half a warp cell in image
pixels (`WarpTolerance`, `ROMA2Warp.h`) — a function of the two image sizes and the grid, not a
setting.

### Per-pair log and summary

At verbosity 3, one line per judged pair, machine-parseable — a rejected pair stops after `REJECT`:

```
ROMA2 pair <ID1>-<ID2>: conf <areaA> <areaB> inl <inlA> <inlB> REJECT
ROMA2 pair <ID1>-<ID2>: conf <areaA> <areaB> inl <inlA> <inlB> ADMIT guided <n> sparse <n> dense <n> <ms>ms
```

and one summary line per pass:

```
ROMA2 one pass: <candidates> candidates, <judged> judged, <admitted> admitted, <stored> stored, <dense-only> dense-only;
  <slots> slots, <loads> loads, <reloads> reloads; <already stored> already stored, <unprepared> unprepared,
  <failed loads> failed loads, <failed matches> failed matches, <dense matches> dense matches (<time>)
```

The two skip counts mean opposite things and are therefore reported apart: `already stored` is a
candidate the scene already holds (a re-run, or the feedback round proposing a pair round 1 stored),
the ordinary outcome; `unprepared` is a candidate one of whose images carries no camera or no
descriptors, i.e. a failure of an earlier stage quietly shrinking the view graph.

`PairsMatcher::Match`, per round: with `roma2 && roma2Cfg.useMatching` the round IS
`MatchPairsROMA2(*this, *roma2, pairs, roma2Cfg, numStored)` and nothing else — no PreMatch, no
`OptimizePairsOrder`, no separate SIFT batch; otherwise the round is the existing SIFT flow, untouched.
The verification-feedback round proposes pairs from the stored pairs' verified matches as before and
runs the same one pass on them. `PairsMatcher::MatchStats::densePairs` is the pairs the one pass
stored.

### Defaults

| knob | default | meaning |
|---|---|---|
| `--roma2-min-confidence` | 0.1 | cell floor for the verdict, the tracking and the dense fill |
| `--roma2-min-overlap` | 0.10 | min-side inlier area; ≈ 0.15–0.17 true overlap; 0.15 ≈ a quarter of the frame |
| `--roma2-dense-matches` | 2000 | dense fill cap per pair |
| guided disc | 2 warp cells | fixed |
| warp tolerance | half a warp cell | fixed, in image pixels |
| sparse tolerance | `MatchConfig::maxEpipolarError` | the matcher's own |

### Known limit

Unchanged from the SIFT path: on a planar, textureless, small-baseline pair the essential matrix
degenerates (the warp is right, the pose is not) — `PairsMatcher` has no homography branch, and this
design does not add one.

---

## Orientation

In-process warps are computed on openMVS *working-orientation* pixels for both images
(`Image::LoadPixels` rotates portrait to landscape; keypoints were extracted from the same pixels), so
no map rotation is ever needed. `RotateMapsForReference`, which existed for warps computed on raw
(un-rotated) files under the deleted NPZ import, is gone with it.

---

## Determinism

Each pair's verdict, guided match and assembly run in parallel (thread pool), but the results are
**stored serially**, in `(ID1,ID2)` order — the keypoint indices `StorePairROMA2` hands out depend on
what the two images already carry, so which thread-pool task finished first must never change them.
PoseLib RANSAC is seeded (default seed 0), so a pair's verdict and guided match set are a pure function
of its inputs. The `match_coarse` graph's `img_A`/`img_B` inputs are dead (Graph Contract, above) and
dropped from the current export's trace; when the loaded match graph still declares them, the C++
binds them from **one shared zero-filled host tensor**, so no per-pair image upload is needed just to satisfy the
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

**Not revalidated for the one-pass matcher.** The device-memory table and the round-1/feedback-round
staging below it were measured under the previous multi-pass pipeline (gate, guided re-match, dense
infusion) on the pre-bidirectional export; the one-pass matcher has not been profiled on this metric
yet. In particular the "34-55% of slot loads are already reloads at 64" figure the `--roma2-slots 16`
advice rests on is a multi-pass-pipeline number: the Slot Plan section's own measurement on the
current pass (225 images, 64-slot budget, 218 loads, **0 reloads**) is the only reload rate taken
against the one-pass matcher, and it does not support that advice. Treat everything below as a floor
from the old pipeline, not a bound on the current one, until it is remeasured.

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

**Not revalidated for the one-pass matcher.** These numbers were measured on the pre-bidirectional
export (`roma2onnx-20260829-facets1520`, `format_version` 1 — unsupported and rejected at load by the
current loader; see Graph Contract and Export tooling above); the bidirectional `match_coarse` graph
the one-pass matcher actually runs (`roma2onnx-20260903-bidir`, two extra outputs from the same
forward pass) has not been re-timed. Real one-pass latency numbers are being produced by measurement
runs separately and are not yet in this document.

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
  `ROMA2Warp.h/cpp`), the confidence gate, and `AppendDenseMatches` — the appended keypoints landing
  past each image's described prefix with the warp-cell size and confidence convention, the appended
  matches landing *inside* the pair's filtered-inlier prefix (the only part `BuildTracks` reads), and
  a second append on the same images leaving the boundary where the first one put it.
- **`ROMA2CoverageSampleTest`** — the coverage-uniform warp sampling the verdict fits its geometry on
  (`SampleWarpByCoverage`): the sample budget, the spread the bucket stratification buys over a plain
  top-confidence selection, the coverage a genuinely one-sided sample reports, and the determinism of
  the draw.
- **`ROMA2ComplementaryDrawTest`** — the dense fill's own draw (`SampleWarpComplementary`): drawn only
  where a pair's guided sparse matches are NOT, capped at the pair's dense budget, thinned when over
  budget by an even stride rather than by confidence, identical across two pairs sharing an image, and
  deterministic.
- **`ROMA2VerdictTest`** (synthetic, no model needed) — the verdict in isolation (`JudgePairROMA2`):
  an exact bidirectional warp computed by projection between two pinhole cameras, admission and
  rejection by the min-side inlier-area rule (including the case a smooth-but-wrong warp — a
  homography of A's grid, unrelated to the cameras — is rejected although it is locally coherent),
  and the `minOverlap = 0` degenerate case that admits everything.
- **`ROMA2GuidedMatchTest`** (synthetic descriptors, no model needed) — guided sparse matching
  (`MatchFeaturesGuided`): a lookalike descriptor outside the disc is rejected, a keypoint whose only
  close descriptor is inside the disc is accepted, a scale-duplicate descriptor *inside* the disc does
  not block acceptance (the ratio-test failure the outside-the-disc rule removes), and the output is
  deterministic across runs.
- **`ROMA2AssemblyTest`** (synthetic, no model needed) — pair assembly and storage
  (`AssemblePairROMA2`, `StorePairROMA2`): the union fit's sparse/dense segments and single relative
  pose on an exact geometry, the dense-only fallback when the guided set is empty (the verdict's own
  geometry stands), and reproducible keypoint indices when a store appends past an image's described
  prefix.
- **`DenseKeypointBoundaryTest`** (`apps/Tests/TestsSFM.cpp`) — always runs, no model needed: the
  stored described-keypoint count surviving a descriptor release and an `.sfm` round-trip of an image
  whose `keypoints.size() > descriptors.rows`, `SelectTopKeypoints` staying inside the prefix, and
  `PairsMatcher::FilterRedundantKeypoints` moving the boundary through the same remap it applies to
  the indices — a removal *inside* the prefix shrinks it, described-wins keeps a described keypoint
  from losing to a coincident dense one, and the more confident of two coincident dense ones survives.
- **`SupplementEvidenceIsolationTest`** — dense evidence is real evidence, not noise to filter around:
  on a pair carrying a dense segment, `FilterMatches`' mean ray angle and the intrinsic weight's grid
  occupancy are both measured over the whole track-forming set (sparse + dense), the minimum-support
  floor reads that same set so a pair under the sparse-only bar still counts, and
  `GetNumWeightedInliers()` discounts the dense share by `DENSE_OBSERVATION_WEIGHT` rather than
  dropping or fully counting it.
- **`GlobalDescriptorsQueryTest`** — always runs, no model needed: the cosine ranking over
  `Image::globalDescriptor` and its deterministic tie order, the `PairsMatcher::QueryRetrieval`
  dispatch that ranks candidate pairs through the descriptors instead of the vocabulary tree, the
  `--export-retrieval-csv` rankings export, the `.sfm` round-trip of the descriptors, and both
  host-side pooling recipes against the export script's fixtures.
- **`RetrievalModeTest`** — the `RETRIEVAL` match mode: candidate selection ranks purely by the global
  descriptors with no `--roma2-match` opt-in needed, agrees pair-for-pair with `VOCABULARY` once that
  mode is pointed at the same backend, a missing descriptor is a hard error rather than a
  vocabulary-tree fallback, and the mode dispatches correctly end-to-end through `Match()`.
- **`RoMa2PreprocessTest`** (`apps/Tests/TestsSFM.cpp`) — always runs; verifies
  `PreprocessImageRoMa2` (the bicubic-antialiased resize reproducing `F.interpolate` exactly) against
  a small synthetic fixture. Skipped only in an OFF build ("built without ONNX Runtime").
- **`RoMa2OnnxParityTest`** — runs only when `OPENMVS_ROMA2_MODEL_PATH` is set (skipped otherwise);
  `OPENMVS_ROMA2_PROVIDER` and `OPENMVS_ROMA2_SETTING` select which execution provider and preset(s)
  it exercises against the shipped `.reference/` dumps (preprocessing, `Describe`, and all four
  `MatchCoarse` outputs — `warp`/`confidence`/`warp_BA`/`confidence_BA`). Its full-size preprocessing
  check uses a **1e-4** bound, not the synthetic fixture's 1e-5: the shipped `in_image.npy` reference
  is itself up to 2.72e-5 away from an exact float64 evaluation of the same antialiased-Keys filter
  (large negative lobes cancel catastrophically on 0..255 samples), so no fp32 implementation can
  reach 1e-5 against it — 1e-4 still separates the right filter from every wrong one (plain bicubic
  ~2e-3 off, `align_corners=true` ~1e-2 off) by more than an order of magnitude.
- **`ROMA2ReconstructTest`** — runs only when `OPENMVS_ROMA2_MODEL_PATH` is set; same
  `OPENMVS_ROMA2_PROVIDER`/`OPENMVS_ROMA2_SETTING` coupling. Exercises the full in-process path on the
  bundled 4-image scene through the one-pass matcher: global-descriptor computation, the pairs a run
  with dense matching on stores against a baseline run with it off, determinism (two runs with the
  same config store identical pairs), and `.sfm` round-trip of `globalDescriptor`.

The model-driven tests are coupled to the same production environment variable
(`OPENMVS_ROMA2_MODEL_PATH`) used by `CreateStructure --roma2-model`'s default — there is no separate
test-only model variable. In an OFF build (`-DOpenMVS_USE_ONNXRUNTIME=OFF`), they report "skipped
(built without ONNX Runtime)"/"no ONNX Runtime support in this build" and pass trivially, while the
always-on tests keep running (they exercise host-side code only). `RoMa2OnnxParityTest` additionally
skips — loudly, with "no reference dumps under '<dir>', parity not checked" — when the model
directory ships graphs but no `.reference/` dumps, since there is then nothing to compare; naming a
preset through `OPENMVS_ROMA2_SETTING` still fails hard in that case, because the user then asked for
a comparison that cannot be made.

---

## Compatibility (`.sfm` scene files)

This branch's stream layout is `SFM_PROJECT_VERSION` **1** (`libs/SFM/Scene.cpp`) — the layout that
carries the per-image global descriptor, `Image`'s described/dense keypoint boundary and the
per-pair dense-segment match count the one-pass matcher's `AppendDenseMatches` writes (One-Pass Dense
Pair Matching, above). The loader accepts that exact version and refuses anything else outright:

```
error: unsupported SFM project version 2 (this build reads only version 1) in '<file>'
```

In particular there is no "derive the keypoint boundary when the stream does not carry it" branch:
the boundary has to be *stored* rather than derived, and a derived value is exactly the silent
misclassification the stored count exists to prevent.

There is no converter and none is planned: a `.sfm` written by any other layout is regenerated by
re-running the matching stage from the images (`CreateStructure -s <images> -o scene.sfm ...`).
`.mvs` files are unaffected.

---

## Limitations

- **Coarse graph only.** No refiner ("regular") stage, and therefore no depth maps from the
  in-process warps — only descriptor/coarse-match are exported and consumed.
- **CoreML/DML speed is unmeasured.** Correctness follows from ONNX Runtime's own provider contract,
  but no latency numbers exist yet for either provider on this repo's hardware.
- **One-pass dense pair matching (`--roma2-match`) has pair-level measurements, not yet end-to-end
  ones.** The recall-per-overlap-band, zero-hallucinated-admission and area-to-overlap numbers in
  One-Pass Dense Pair Matching (above) are measured against SIFT on the same candidate pairs, on
  8d2f4877, 38004114 and Truck. Registration counts, pose accuracy against a pseudo-GT reconstruction,
  run time and peak memory for the whole reconstruction are being produced separately and are not yet
  in this document.
- **Repeated structure is not the verdict's job.** Two frames that see *different instances* of a
  repeated structure (doppelgängers) can still warp smoothly and locally coherently onto each other —
  the min-side inlier area cannot tell that apart from a true pair by construction, since both produce
  a geometrically consistent correspondence set. The triplet-based view-graph filter, downstream of
  matching, is what separates aliased pairs from true ones; the verdict is not a substitute for it.
- **Essential-matrix degeneracy on planar, small-baseline pairs** (Known limit, One-Pass Dense Pair
  Matching, above) — unchanged from the SIFT path: the warp can be right while the pose is not, and
  `PairsMatcher` has no homography branch to fall back on.
- **The describe pass always runs when `--roma2-retrieval` is on**, even for `EXHAUSTIVE` or
  `SEQUENTIAL` matching where no retrieval ranking is needed for pair selection — the global
  descriptors are still computed and stored (`Image::globalDescriptor`), which costs the describe pass
  but not the (lazy) match-graph load.
