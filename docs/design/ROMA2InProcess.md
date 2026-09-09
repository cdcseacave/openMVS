# RoMa v2 In-Process — Retrieval and Dense Matching

## Overview

RoMa v2 (DINOv3 backbone + a coarse matcher head) runs **in-process** through ONNX Runtime, as an
optional replacement for two stages of the classical SFM pipeline. It plugs into two independent
seams:

```
CreateStructure --roma2 ...                     Scene::MatchPairs
  --match-mode 4 ──────> global descriptors ──> PairsMatcher::CollectRetrievalPairs
   (RETRIEVAL)             (GlobalDescriptors.h,   (QueryRetrieval over the global-descriptor index;
                             computed by the         the vocabulary tree is never built; RRF/mutual-
                             describe pass, or        top-K/bridging/feedback shared with VOCABULARY)
                             already stored)
  --roma2-match true ──> bidirectional warp ────> one-pass dense pair matching
                           (RoMa2Matcher.h/cpp,      (MatchROMA2.h/cpp: verdict, guided sparse match,
                            OnnxRuntime.h/cpp)         dense fill, one union fit, one store —
                                                        ROMA2Warp.h/cpp, MatchGeometric.h/cpp)
```

**Two independent seams, neither one `--roma2`'s default effect.** `--roma2` on its own has nothing
to do — `CreateStructure` rejects it unless `--roma2-match true`, `--match-mode 4` (RETRIEVAL) or
`--export-retrieval-csv` also asks for something the model can produce. `--match-mode 4` is the
retrieval seam: it ranks candidate pairs by the DINOv3+GeM(p=3) 2048-D global descriptors instead of
the vocabulary tree, and needs `--roma2 true` only to compute those descriptors when the scene does
not already carry them — the matching itself stays SIFT/AKAZE/ORB. This is RETRIEVAL's headline use:
robust pair selection by DINOv3, with no dense matching at all. `--roma2-match true` is the
independent dense-matching seam, and composes with either match mode: the verdict on a pair comes
from the bidirectional warp alone, with no union of SIFT-verified and warp-verified pairs and no SIFT
fallback for a pair the warp rejects (One-Pass Dense Pair Matching, below).

| Flag | Default | Effect |
|---|---|---|
| `--roma2` | `false` | master switch for the in-process model |
| `--roma2-model DIR` | `$OPENMVS_ROMA2_MODEL_PATH` | exported graphs + manifest |
| `--roma2-setting turbo\|fast\|base` | `base` | preset (320/512/640 px) |
| `--roma2-provider auto\|cuda\|coreml\|dml\|cpu` | `auto` | execution provider; a named one is required, not preferred |
| `--roma2-match` | **`false`** | one-pass dense pair matching (verdict, guided sparse match, dense fill, one union fit, one store) |
| `--roma2-slots N` | `64` | image descriptors resident on the device while dense matching |
| `--roma2-min-confidence F` | `0.1` | confidence at which a warp cell takes part in the verdict, the keypoint tracking and the dense fill |
| `--roma2-min-overlap F` | `0.10` | verdict: the pair is admitted iff min(inlier area A, inlier area B) ≥ F |
| `--roma2-dense-matches N` | `2000` | dense correspondences per FULL FRAME of overlap (a density, not a per-pair count) |
| `--export-retrieval-csv F` | — | per-image retrieval rankings (needs `--roma2 true`) |

Which mode ranks candidate pairs how is a property of `--match-mode` alone, with no crossover: `1`
VOCABULARY always ranks with the SIFT/AKAZE vocabulary tree, `4` RETRIEVAL always ranks with the
global descriptors above. Neither can substitute itself into the other.

`--export-retrieval-csv` and `--export-pairs-csv` are both written by `Scene::Reconstruct()` right
after pair matching (`ReconstructionConfig::exportRetrievalCSV`/`exportPairsCSV`), before any
reconstruction step (largest-connected-component clustering, weak-image filtering, resection) can
drop pairs or leave images unregistered — the CSVs describe the matched scene, not whatever
reconstruction happened to keep. A failed export only logs a warning and never fails the
reconstruction, whose primary output is the scene itself.

`--match-mode 4` (RETRIEVAL) is its own `MatchMode`, not a modifier of another one:
`PairsMatcher::CollectRetrievalPairs` builds the global-descriptor cosine index
(`EnsureGlobalDescriptorsIndex`, `GlobalDescriptors.h`) and shares the same reciprocal-rank fusion,
mutual top-K agreement and connectivity bridging as VOCABULARY's `CollectVocabularyPairs`
(`CollectFusedRetrievalPairs`) — those are properties of the ranking, not of the backend. Everything
downstream of pair ranking stays backend-agnostic, and the VOCABULARY→EXHAUSTIVE small-scene remap is
unchanged.

**This changed the KNOWN_POSES unposed-image fallback.** `CollectKnownPosePairs`'s branch that adds
candidates for images with no imported pose (so an incomplete pose file does not leave them
unmatched) now always queries the vocabulary tree (`EnsureVocabularyTree`) — it used to be able to
query the global descriptors instead whenever a scene carried them, so "vocabulary retrieval" there
silently meant two different rankings depending on what else was enabled. It means exactly one thing
now: the SIFT/AKAZE vocabulary tree, `--roma2`/`--match-mode` notwithstanding.

The in-process integration replaces the earlier NPZ-based ROMA2 import outright (deleted:
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
Manifest `format_version` **1**:

| File | Inputs | Outputs |
|---|---|---|
| `roma_<setting>_descriptor_fp32.onnx` (+`.onnx.data`) | `image` `[1,3,S,S]` f32, RGB planar, values in [0,1], un-normalised (ImageNet mean/std applied in-graph) | `layers` `[1,2,G,G,1024]` f32 channels-last (blocks 11+17, patch tokens only); `value_facets` `[1,2,G,G,1024]` f32 channels-last (V projections of blocks 15+20, before attention weighting/`o_proj`); `retrieval` `[1, facetsDim]` f32, the FACETS recipe pooled end to end on device (**mandatory** — a graph without it fails to load, see Retrieval Recipe below) |
| `roma_<setting>_match_coarse_fp32.onnx` (+`.onnx.data`) | `descriptors_A`, `descriptors_B` `[1,2,G,G,1024]` f32 — the dead `img_A`/`img_B` inputs are dropped from the trace; if the wrapper cannot be traced without them the exporter keeps them and says so in its report, and the C++ binds them only when the loaded match graph still declares them (read from the ONNX session's own input metadata, not the manifest's `io` block — that block is `RoMa2Manifest::Load`'s optional shape-consistency check, not what decides binding; the one tolerated variation, decided by the exported graph, not a compatibility path) | `warp` `[1,C,C,2]`, `confidence` `[1,C,C,1]` f32 (A→B, normalised (x,y) ∈ [-1,1], align_corners=False; confidence a raw overlap logit, sigmoid on host); `warp_BA`, `confidence_BA`, the same for B→A, from the same forward pass (`bidirectional=True`) — one graph call states both directions of the pair |
| `roma_<setting>.json` | openMVS manifest: `format_version`, `model`, `setting`, `image_size`, `patch`, `layers`, `value_facet_blocks`, `warp_size`, `confidence_channels`, `retrieval_recipes` (`facets`/`layers` dim + GeM params), `files`, `io` (shapes), `opset`, checksums | |

fp32, static shapes, batch 1, opset 18. Coordinate conventions (`ROMA2Warp.h/cpp`, deliberate
asymmetry): pixel→grid `CoordFromTo` is align_corners=**true**; grid→pixel `DenormCoord` is
align_corners=**false** (`0.5*(n+1)*W - 0.5`). `RoMa2Manifest::Load` (`libs/SFM/RoMa2Matcher.h/cpp`)
rejects a manifest of any `format_version` other than 1, naming the version, and rejects a
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

Exported model sets live on the shared volume, one directory per export, referenced by
`--roma2-model` or `$OPENMVS_ROMA2_MODEL_PATH`. The manifest schema is unreleased and carries no
compatibility duty, so its `format_version` counter was reset to **1** for the bidirectional,
`retrieval`-carrying schema described above, rather than continuing to climb (nothing outside this
branch has ever consumed the higher numbers the counter briefly reached during development). An
export predating that reset is not what the current loader means by `format_version` 1, whatever
integer its own manifest happens to declare. `RoMa2Manifest::Load` accepts exactly `format_version`
1, naming the version it saw and rejecting everything else. The `.onnx` + `.onnx.data` + `.json` set
is byte-portable across OSs (external data is resolved relative to the model path on every platform).

---

## Retrieval Recipe

The global retrieval descriptor (FACETS, 2048-D: per-slice GeM p=3 on `value_facets` → L2 per
slice → concat → L2 → `sign(d)·|d|^p` power normalization → L2, `p` = the export's `FACETS_POWER`)
is pooled end to end **on device**, inside the exported descriptor graph, and read back as the
`retrieval` output (`RoMa2Onnx::Describe`, `libs/SFM/RoMa2Matcher.h/cpp`) -- one path, no
runtime-configurable recipe or host-side pooling. A manifest that does not declare a `retrieval`
output (an export predating the on-device pooling) is an unsupported model and fails loudly at
load, naming the model directory and the missing output.

The pooled output is checked against an independent implementation in `RoMa2OnnxParityDescribe`
(`apps/Tests/TestsSFM.cpp`): it reads the graph's `retrieval` output back and judges it against the
Python `pool_retrieval` reference (`scripts/python/roma2/graphs.py`) at cosine >= 0.99999.

---

## Session Lifecycle

One `RoMa2Onnx` per `Scene::MatchPairs` call (`libs/SFM/Scene.cpp`): declared before `PairsMatcher` so
its destructor runs after (a `RoMa2Onnx*`/tensor held by `PairsMatcher` must not outlive the model).
`RoMa2Onnx::Load` loads only the descriptor graph; the coarse-match graph loads lazily on the first
`MatchCoarse` call, on the same execution provider the descriptor session got, and a load failure is
remembered so later calls fail fast. `layers` are **not** cached across the describe pass and the
matching pass (13 GB per 1000 images at base) — the matching pass re-describes on slot load.

Whether any pass actually needs the model is decided entirely by the caller, not by `ROMA2Config`
itself: the config cannot see the match mode, so it exposes no "is this in process enabled" query
of its own. `Scene::MatchPairs` computes that itself: `needsDescriptors` (the match mode is
RETRIEVAL, or `--export-retrieval-csv` was requested, and the scene does not already carry global
descriptors) and `needsWarps` (`roma2Cfg.useMatching`) — the model loads only when at least one of
the two is true. A requested-but-unavailable model is always an error, never a silent fallback
to the vocabulary tree — `Scene::MatchPairs` checks this before loading anything, by name, and is
the only place that can, since only it knows whether the scene already carries global descriptors.
`CreateStructure` gives the same hint earlier, before any feature extraction runs, wherever it can
be sure without seeing the scene: unconditionally that `--roma2` has nothing to do at all without
`--roma2-match true`, `--match-mode 4` (RETRIEVAL) or `--export-retrieval-csv`, and unconditionally
that `--roma2-match true` needs a resolvable model (the dense pass always needs it, regardless of
what the scene already carries). It deliberately does not require a model for `--match-mode 4` or
`--export-retrieval-csv` alone, since a scene loaded from disk may already carry the descriptors
they need and then require no model at all — `Scene::MatchPairs`' own check covers that case.

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

The budget is a pure cost knob: the same candidate pairs are stored whatever the budget, so lowering
it only trades device memory for re-describes -- a slot per image needs no reload at all, regardless
of order. Processing pairs in `(ID1,ID2)` order is what keeps a small budget affordable: replaying the
same pairs in an arbitrary order costs several times more reloads at a tight budget than the run's own
processing order does under the same Belady policy.

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
4. **Dense fill** (`SampleWarpComplementary`). `--roma2-dense-matches` (2000, `denseMatchesPerFrame`)
   is a DENSITY — correspondences per FULL FRAME of overlap, not a count per pair. `DenseFillGridSide`
   turns that density into a fixed bucket pitch, `ceil(sqrt(density))` clamped to the warp side, over
   the WHOLE warp grid — the same pitch for every pair regardless of how much of the frame its overlap
   covers, so a pair's confident overlap holds `density x overlapArea` buckets whatever that overlap
   is, and the draw is the same density in every pair rather than the same flat count. Because
   `SampleWarpComplementary` already yields no point from a bucket the guided matches occupy
   (`occupiedA`), the fixed pitch turns that exclusion rule into an exact density over the UNCOVERED
   overlap with no separate coverage term to add: a pair whose guided matches already cover half its
   overlap draws from the other half at the same per-area rate as a pair with no guided coverage at
   all, instead of the old rule's flat count regardless of how much of the frame the guided matches
   left for it. A side effect worth stating because it changed what tracks look like: with a per-pair
   pitch, two pairs sharing image A used to stratify it on grids sized to each pair's own draw, so
   their A-side samples fell on different lattices and landed a few cells apart — the redundant-keypoint
   filter then saw two close keypoints where a shared track needed one. The fixed pitch removes that at
   the source (the same lattice, every pair), which is what lets dense matches chain into tracks longer
   than two views. `DenseFillCeiling` then bounds the draw at that same density over
   `min(inlierAreaA, inlierAreaB)`: the bucket grid lives in A's frame, so the pitch alone only bounds
   the density THERE; every dense correspondence also costs a keypoint in B, and the ceiling is what
   bounds it in B — it binds only when B is the smaller of the two inlier areas, sitting above the
   pitch's own draw otherwise, so it never re-charges the coverage discount the pitch already applied.
   The ceiling is the one term of the draw that is still per-pair, so ENFORCING it is where the
   agreement above could be lost and is not: `ThinSampleByLatticePriority` ranks the drawn sample on
   the same pair-independent key the bucket winners were picked on (lattice priority, the cell's
   scramble to break the level the prefix stops inside) and keeps a prefix of it, so two pairs whose
   ceilings differ keep NESTED samples of A — the tighter one's survivors are the looser one's, cell
   for cell — instead of two subsets of one agreement that need not meet. A stride through each
   pair's own list, which is what this was first written as, has survivors that depend on the pair's
   sample count and the pair's ceiling, and would have cut the chaining to roughly the product of the
   two pairs' thinning ratios on every pair the ceiling binds on — which, the perimeter of any
   compact overlap touching more buckets than the ceiling allows, is nearly all of them.
5. **One geometry for the pair** (`AssemblePairROMA2`). `PairsMatcher::GeometricFilter` runs once more
   on guided ∪ dense; the guided matches within the matcher's own epipolar tolerance become the pair's
   sparse (descriptor) evidence, the dense correspondences within the warp tolerance its dense segment.
   When the union fit fails, the verdict's own geometry stands and classification runs against it
   instead — an admitted pair is always stored, and a pair with zero sparse inliers is dense-only, its
   dense segment its whole evidence. The pair is stored once (`StorePairROMA2`): created, or replacing
   a same-key pair a previous `Match()` left.

A judged pair costs on the order of 100 ms on an A100 at the base preset, growing on a texture-rich
capture where the guided descriptor search dominates rather than the graph call, and less at the
faster presets. Bundle adjustment over the tracks the dense fill creates, not matching itself, is what
a whole reconstruction run actually spends its time on.

### Why the min-side inlier area

A hallucinated warp is a smooth field, locally a homography, and every homography is explained exactly
by a family of fundamental matrices: a RANSAC inlier count or ratio on the warp's own cells cannot tell
a hallucination from a true pair, and a lower confidence floor only makes the count rule worse. The
min-side inlier AREA of one geometry does separate them: measured across captures ranging from
well-textured outdoor scenes to textureless interiors, it admits no hallucinated (non-co-visible,
warp-wrong) pair at floor 0.1 and θ = 0.10, without any cycle-consistency test. The B side is what
supplies that precision — it rejects the "whole of A onto a few pixels of B" pairs that the A side
alone would let through.

Recall at low overlap -- where a retrieval-only pipeline most needs the pairs it can get -- is where
the verdict gains the most over a plain SIFT baseline on the same candidate pairs; at high overlap the
two agree. The measured min-side inlier area is 0.55–0.75 of the true (GT) overlap, so θ = 0.10 asks
for roughly 0.15–0.17 of true overlap and θ = 0.15 for about a quarter of it. Coarse θ = 0.08 measures
the same verdict as fine θ = 0.10; the refiners are not exported because match precision is what they
would buy, and the sparse (guided) inliers already supply that wherever the scene has texture (Graph
Contract, above).

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

// ROMA2Warp.h — the fixed pitch (density -> bucket grid side); bounds the draw in A
int DenseFillGridSide(unsigned denseMatchesPerFrame, int warpSide);

// MatchROMA2.h — density over the smaller inlier area; bounds the draw in B, which the grid cannot see
unsigned DenseFillCeiling(const ROMA2Config& config, const PairVerdict& verdict);

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
ROMA2 pair <ID1>-<ID2>: conf <areaA> <areaB> inl <inlA> <inlB> ADMIT cap <cap> grid <grid> guided <n> sparse <n> dense <n> <ms>ms
```

`cap` and `grid` are `DenseFillCeiling(config, verdict)` and `DenseFillGridSide(config.denseMatchesPerFrame,
warpSize)` — the same two calls `AssemblePairROMA2` feeds `SampleWarpComplementary`, read again at the
log site rather than plumbed out of it, so the two numbers that decided the draw are printed next to
what the draw actually did. A run's own log is then enough to replay the rule that produced it: e.g.

```
ROMA2 pair 0-1: conf 0.9523 0.9658 inl 0.9393 0.8929 ADMIT cap 1786 grid 45 guided 749 sparse 736 dense 1642 301ms
```

is a pair whose two inlier areas (0.9393, 0.8929) capped its draw at `round(2000 * 0.8929) = 1786`
correspondences (`cap`), stratified over a 45x45 pitch fixed for every pair in the run (`grid`), and
kept 1642 of them. `dense` is NOT the size of the draw: it is `result.dense.pointsA.size()` after
`AssemblePairROMA2` has classified the draw against the union fit's geometry and dropped whatever the
fit calls an outlier, so a pair thinned to the ceiling and then pruned prints the same line as a pair
that never reached it. `dense < cap` therefore says nothing about which term bound the draw, and a
readout asking whether the pitch or the ceiling binds cannot answer it from this record. A rejected
pair draws nothing, so it keeps the shorter form above with no `cap`/`grid` fields.

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
| `--roma2-dense-matches` | 2000 | dense correspondences per full frame of uncovered overlap (density, not a cap per pair) |
| guided disc | 2 warp cells | fixed |
| warp tolerance | half a warp cell | fixed, in image pixels |
| sparse tolerance | `MatchConfig::maxEpipolarError` | the matcher's own |

### Known limit

Unchanged from the SIFT path: on a planar, textureless, small-baseline pair the essential matrix
degenerates (the warp is right, the pose is not) — `PairsMatcher` has no homography branch, and this
design does not add one.

### Bundle adjustment's dense observation weight

The dense fill's correspondences are real geometric evidence, not noise to filter around — `FilterMatches`
and the intrinsic weight already treat them that way (`SupplementEvidenceIsolationTest`, Tests below) —
but a warp correspondence localizes a point several times less precisely than a described one, and a
bundle solve has to charge every residual for the precision of the measurement it minimizes.
`--ba-dense-weight` (`BAConfig::denseObservationWeight`, default `-1.0`, "measure it"; ANY value
`>= 0` pins it and is used unclamped, so use `[0,1]`, `1` turning the down-weight off) scales a dense
reprojection residual's loss weight by

```
w = (sigma_described / sigma_dense)^2
```

both sigmas the MEDIAN raw-pixel reprojection error of, respectively, the described and the dense
observation populations of the scene each solve is about to fit (`ComputeObservationSigmas`,
`Track.h/cpp`) — read off the RAW residuals, not the weighting the previous solve ran under, and
resolved fresh at the head of every solve (`EstimateDenseObservationWeight`, `BundleAdjustment.h/cpp`)
since a reconstruction runs fifty or more of them and the scene the estimate is measured on keeps
growing. `w` is clamped to `[0.01, 1]` — a dense correspondence is never a MORE precise measurement
than a described one, and never worth nothing to a textureless region that has no other evidence — and
falls back to the fixed `DENSE_OBSERVATION_WEIGHT` (0.25, `ImagePair.h`) when either population is
under 100 observations or either sigma is zero, the honest answer for an early incremental step whose
scene is a handful of tracks and for a scene that fits itself exactly. The same constant answers a
scene carrying no dense keypoints at all, which the estimator recognises before walking anything: the
stock `--roma2-match false` reconstruction has no dense observation to weight, and its solves must not
pay for a whole-scene reprojection pass to be told so.

Measured rather than configured because `k = sigma_dense / sigma_described` is a property of the
CAPTURE, not of the matcher: the dense sigma stays close to the warp's own sampling scale regardless
of the scene, while the described sigma moves several-fold between a well-textured outdoor capture and
a textureless interior, and `w = 1/k^2` moves with it. A constant tuned to one capture is wrong on the
other; nothing here is right in both places, which is why this weight is measured and the view graph's
is not.

This is deliberately a different quantity from the view graph's own dense discount
(`PairsWeightingConfig::denseObservationWeight`, `DENSE_OBSERVATION_WEIGHT` above, `ImagePair.h`), and
bundle adjustment's measured weight does not reach it. A warp correspondence localizes a point several
times less precisely than a descriptor one — which is exactly what bundle adjustment's weight charges
it for — but it says nearly as much as a descriptor correspondence about whether the two images
overlap, which is the only thing the view graph asks. Charging the precision penalty a second time in
the view graph would demote exactly the dense-only pairs that carry a capture the descriptor matcher
cannot match at all — on a sufficiently textureless capture those pairs are the difference between a
registered model and none. The view graph's constant therefore stays fixed and independent of bundle
adjustment's, which is free to move with every capture and every solve.

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

Building with ONNX Runtime enabled provisions the *library*; it does not fetch the exported *model*.
See `docs/RoMa2Model.md` for that — the `roma2-model` CMake target, `scripts/fetch_roma2_model.py`,
and the DINOv3 licence terms accepting the model implies.

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

Those are the tensors and weights the pipeline asks for; what the device actually holds is larger,
since ONNX Runtime's CUDA arena grows on demand and never returns memory, so budget on the order of
10 GB rather than the analytic floor once dense matching is running. Retrieval-only (`--roma2-match
false`, the default) never opens the coarse-match session and runs comfortably on a much smaller
device, `--roma2-slots` being a dense-matching knob only. On a memory-constrained device, lower
`--roma2-slots` (fewer resident descriptors, more re-describes) or use `--roma2-setting fast` or
`turbo`, whose tensors are smaller per slot.

---

## Scaling

**RETRIEVAL-only already runs the fast path.** With pair selection on `--match-mode 4` and
`--roma2-match false`, the coarse-match graph is never loaded: `RoMa2Onnx::Load` calls only
`LoadDescriptor`, and the match session is built lazily by `Impl::EnsureMatch()`, whose sole caller is
`MatchCoarse` (`RoMa2Matcher.cpp`). A retrieval-only run therefore never pays the ~448 MB of
coarse-graph weights, its session, or its four host warp tensors. `value_facets` is never read back
either: `Describe` binds it to `facetsScratch`, a device tensor, and the host copy (`facetsHost`) is
allocated lazily and asked for only by the parity test. One `layers` tensor is allocated for the whole
pass, not one per image (`ComputeGlobalDescriptorsROMA2`, `MatchROMA2.cpp`); the graph pools GeM(p=3)
→ concat → signed power → L2 on device and hands back the finished 2048-D vector as the `retrieval`
output, so only **8 KB per image** crosses the bus. Image load and preprocessing are pipelined on the
thread pool ahead of the single-threaded `Describe` call (`PrefetchRing`), measured at roughly 40 ms
per image on an A100 — an order of magnitude faster than the dense-matching pass over the same images.

The one thing a retrieval-only run still pays for and does not use is `layers` itself (blocks 11 and
17): the descriptor graph always emits it, because the same backbone forward pass that produces
`value_facets` (blocks 15, 20 — what the on-device pooling into `retrieval` actually consumes) has to
run through block 20 regardless, for the coarse matcher's `descriptors_A`/`descriptors_B` input, so no
transformer compute is saved either way. Not reading `layers` back leaves **12.5 MB of VRAM**
(`[1,2,40,40,1024]` fp32) allocated once for the whole pass rather than once per image — a memory
cost, not a time one, since the device-side write of that tensor is ~13 MB at roughly 1.5 TB/s, well
under 1 ms against a 43 ms ViT-L forward pass. A retrieval-only export that drops `layers` from the
descriptor graph would need a third exported graph, a third manifest key and a third parity fixture
set, for under 1% of the pass — not worth it; revisit only if VRAM, not time, becomes the binding
constraint.

**The retrieval index is linear in image count and needs no cache.** `GlobalDescriptors` holds one
`N x 2048` float matrix — 41 MB at 5 000 images, 164 MB at 20 000 — and `Query` is a single GEMV plus
a partial sort, O(N·D) per query with no N×N matrix anywhere: the vectors are small enough to stay
resident in RAM at any capture size this codebase targets. `Image::globalDescriptor` is 8 KB per image
and is serialized with the scene, so a second run over a scene that already carries descriptors skips
the describe pass entirely — no ONNX session is loaded at all, and pair ranking reads straight off the
stored vectors (Session Lifecycle, above).

**Dense matching's per-image cost, `layers`, cannot all live on the device — and does not need to.**
At 12.5 MB each, 5 000 images would be 61 GB. `MakeSlotPlan` (`MatchROMA2.cpp`) schedules which
descriptors stay resident with Belady's optimal replacement (evict the slot whose next use is
furthest away) over the candidate pairs sorted `(ID1,ID2)`, at most `--roma2-slots` (default 64,
12.5 MB each). The plan only grows to what a scene actually needs, so a capture smaller than the
budget never evicts, and an eviction costs a **43 ms re-describe**, not a correctness problem — there
is no disk spill. The pass reports `loads` and `reloads` separately (Slot Plan, above), so the cache's
own cost stays visible per run rather than hiding inside the matching wall time.

A **disk-backed descriptor cache** is a real option — trading 12.5 MB/image of disk I/O against the
43 ms of recompute an eviction currently costs — but nothing measured so far asks for one: a capture
that fits inside the slot budget sees no reloads at all. Whether a much larger capture (5 000+ images)
stays anywhere near that depends on its pair graph's own bandwidth, a property of the capture rather
than of the code, and is what a decision to add the cache would need measured first.

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
  budget by the pair-independent lattice key rather than by confidence -- so a tighter budget keeps a
  subset of what a looser one keeps -- identical across two pairs sharing an image, and deterministic.
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
- **One-pass dense pair matching (`--roma2-match`) is validated at the pair level.** The tests above
  cover the verdict, the guided match and the assembly in isolation; there is no automated end-to-end
  check of registration completeness or pose accuracy for a whole reconstruction run through it.
- **Repeated structure is not the verdict's job.** Two frames that see *different instances* of a
  repeated structure (doppelgängers) can still warp smoothly and locally coherently onto each other —
  the min-side inlier area cannot tell that apart from a true pair by construction, since both produce
  a geometrically consistent correspondence set. The triplet-based view-graph filter, downstream of
  matching, is what separates aliased pairs from true ones; the verdict is not a substitute for it.
- **Essential-matrix degeneracy on planar, small-baseline pairs** (Known limit, One-Pass Dense Pair
  Matching, above) — unchanged from the SIFT path: the warp can be right while the pose is not, and
  `PairsMatcher` has no homography branch to fall back on.
- **`--export-retrieval-csv` forces the describe pass under any match mode**, including
  `EXHAUSTIVE`/`SEQUENTIAL`/`VOCABULARY`/`KNOWN_POSES`, where no retrieval ranking is otherwise needed
  for pair selection — asking for the per-image rankings is asking for the global descriptors that
  produce them, so `Scene::MatchPairs` computes and stores them (`Image::globalDescriptor`) regardless
  of mode. This costs the describe pass but not the (lazy) match-graph load.
