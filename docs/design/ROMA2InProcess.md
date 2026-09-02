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
guided-pair store/replace), `libs/SFM/GlobalDescriptors.h/cpp` (cosine retrieval index over the
graph-pooled descriptors). CLI:
`apps/CreateStructure/CreateStructure.cpp` (`--roma2*`, `--export-retrieval-csv`).

---

## Graph Contract

Per preset `S ∈ {320 (turbo), 512 (fast), 640 (base)}`, `G = S/16` patch grid, `C = S/4` warp cells:

| File | Inputs | Outputs |
|---|---|---|
| `roma_<setting>_descriptor_fp32.onnx` (+`.onnx.data`) | `image` `[1,3,S,S]` f32, RGB planar, values in [0,1], un-normalised (ImageNet mean/std applied in-graph) | `layers` `[1,2,G,G,1024]` f32 channels-last (blocks 11+17, patch tokens only, unchanged from the shipped engine); `value_facets` `[1,2,G,G,1024]` f32 channels-last (V projections of blocks 15+20, before attention weighting/`o_proj`); `retrieval` `[1, facetsDim]` f32, the FACETS recipe pooled end to end on device (**mandatory** — a graph without it fails to load, see Retrieval Recipe below) |
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
`~/virginia/models/roma2-onnx/roma2onnx-20260831-retrieval/` (the export current as of this task,
`format_version` 2, `retrieval` output present) — referenced by `--roma2-model` or
`$OPENMVS_ROMA2_MODEL_PATH`. The `.onnx` + `.onnx.data` + `.json` set is byte-portable across OSs
(external data is resolved relative to the model path on every platform).
`~/virginia/models/roma2-onnx/roma2onnx-20260829-facets1520/` (`format_version` 1, no `retrieval`
output) is the export this task's own change makes unsupported; kept, read-only, only as a fixed
point for the load-time rejection.

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

## Dense Two-View Gate (`--roma2-validate`, off by default)

`ValidatePairsROMA2` (`libs/SFM/MatchROMA2.cpp`) runs **before** descriptor matching, on the pairs
the match mode selected, and drops the ones a single geometry cannot explain. Per pair: erode the
confidence map, draw a coverage-maximising sample of the warp (`SampleWarpByCoverage`,
`--roma2-dense-sample`, default 2000 — buckets are laid over the *whole* warp at a resolution
scaled up by the inverse eligible fraction, so that the buckets which do hold an eligible cell
number about the budget, and the most confident cell of each is taken; there is no fill-up, so a
pair with little overlap yields proportionally fewer points instead of crowding the budget into a
corner), then fit one geometry to that whole sample through `PairsMatcher::GeometricFilter` on
temporary `Image` copies whose keypoints are the dense points (the `MatchFeaturesGeometric`
precedent, so no second estimator exists). The pair passes when the fit's inlier subset still
covers at least `--roma2-min-inlier-coverage` (default 0.25) of *both* images. **A rejected pair is
dropped, not demoted** — it does not fall through to descriptor matching.

The gate originally thresholded the RANSAC inlier ratio (`--roma2-min-dense-ratio`, default 0.8).
That option is gone: measured across four Truck arms plus Meetingroom and Courthouse, the ratio
separated true from false pairs barely above chance once restricted to pairs the estimator actually
ran on (AUC 0.55–0.70), and it cost recall outright — at 0.5 warp-native px it rejected 89% of the
true pairs it was shown, while the same coverage threshold on the same run kept 98%. Inlier
coverage alone leaks 0.00–0.20% of normal false pairs at 90–99% recall on all three scenes.

Pairs that see *different instances of a repeated structure* (doppelgängers) are **not** this
gate's responsibility — coverage cannot separate them, and on Courthouse every non-co-visible pair
carrying 1000+ verified two-view inliers passes it. The triplet-based view-graph filter handles
those.

It shares `MakeSlotPlan`, the prefetch ring and the warp ordering with the dense matching pass
(`ForEachWarpROMA2`), so it costs the same per pair (~23 pair/s at base on an A100); unlike that
pass it needs no descriptors, only cameras. It is independent of `--roma2-match`: the gate decides
which pairs exist, the dense matcher re-matches the ones that do.

**The fit is the matcher's own — the gate holds no estimator settings.** `GeometricFilter` is called
with the `PairsMatcher`'s configuration, so `PairsMatcher::SelectGeometryBranch` picks the geometry
from what the two images actually carry — `ESSENTIAL` (5-DoF calibrated bearings plus cheirality,
where both cameras `TrustIntrinsics()`), `SHARED_FOCAL`, or `FUNDAMENTAL` (7-DoF F) — and the
epipolar threshold is `MatchConfig::maxEpipolarError`, the same precision the descriptor path
demands. Neither is worth a gate-specific setting, which is a measured result rather than an
assumption: run as both a forced-F and a forced-E arm at two thresholds a factor of 2.9 apart, the
gate's recall and leak barely moved (table below), so both branches and the whole threshold band
work. `SelectGeometryBranch` remains one named decision so that a caller logging the branch and the
estimator choosing it cannot disagree.

Why the default threshold is safe rather than lucky: the warp's precision is fixed in the network's
own square input frame (`ImageSize`, 640 at base) while `maxEpipolarError` is applied in
full-resolution pixels, so the same 4 px is a *different* demand on the warp per dataset —
2.9 warp-native px on a 1024x768 capture, 1.8 on a 1955x1089 one (converting by `sqrt(W*H)/ImageSize`,
the geometric mean, because `PreprocessImageRoMa2` resizes anisotropically into that square).
Measuring at 1.0 and 2.89 native px therefore brackets what 4 target px means across ordinary
capture resolutions, and the verdict is stable across that whole bracket.

**What the gate reads of an image.** Its `pCamera` (intrinsics, and `TrustIntrinsics()` through
them), its size, and the warp. Not its pose: the temporary `Image` copies the estimator sees are
built with an explicitly invalidated pose, so a scene carrying a ground-truth solution cannot leak
it into the gate's geometry. That is enforced structurally, not by convention, because a
pose-contaminated fit is exactly the circularity the gate exists to escape.

**Measured: the inlier ratio is the wrong decision variable** (2026-08-31, both 400-keyframe
Polycam captures, 9078 and 7604 candidates, run folders
`<capture>/openmvs-densegate-20260831-retrieval-gate/`). ROC AUC of the ratio as a pair-validity
predictor is 0.844 / 0.907 against the ARKit depth labels and 0.765 / 0.795 against Doppelgangers++,
but restricted to the candidates whose eroded warp offered a sample at all it falls to 0.677 / 0.627
and 0.733 / 0.603 — most of the separation is the pairs whose warp had no confident cell (ratio 0 by
construction), which is a coverage signal, not a coherence one. At the 0.8 bar, 70–87 % of the pairs
either reference calls wrong and whose warp yields a sample are still explained to ≥ 80 % by one
geometry (their median ratio 0.957–0.995, against 0.998–1.000 for the good pairs). A wrong warp is
*not* coherent only in patches. The coverage the same pass measures separates far better
(min-coverage AUC 0.984 / 0.988 and 0.793 / 0.852; median 0.023–0.062 on wrong pairs against
0.312–0.324 on good ones), as does the raw inlier count. Both captures are self-calibrated
(`0 trusted intrinsics`), so the fit is the 7-DoF fundamental branch of `GeometricFilter`, which is
the leading explanation for how permissive it is.

**Measured: inlier coverage at 0.25, on the population the gate is actually for** (2026-09-01,
Tanks-and-Temples Truck / Meetingroom / Courthouse, run folders
`<scene>/openmvs-densegate-2026090{1,1}-gate-{f,e}-native{1.0,2.89}/`, pseudo-GT from the COLMAP
reference reconstruction: a pair is co-visible when it shares >300 verified tracks, and separately
when the two frusta overlap by pose).

The measurement only became readable once the negatives were **partitioned** rather than pooled. Four
disjoint negative populations behave completely differently, and the earlier aggregates that lumped
them together read as a 10–14% "false-pair leak" that was almost entirely the *ambiguous* bucket —
pairs whose poses say the frusta see the same surface while the tracks found no support (occlusion
through a building body is the likely cause of most). Split out:

| population | definition | in scope |
|---|---|---|
| positives | shares >300 verified tracks | yes — recall |
| ambiguous | no shared tracks, but poses overlap | no — the label contradicts itself |
| aliased | no co-visibility, yet ≥30 independently verified two-view inliers | no — triplet filter |
| **normal false** | no shared surface and no appearance match | **yes — what the gate is for** |

Recall and normal-false leak at `minInlierCoverage = 0.25`:

| scene | branch, threshold | recall | normal-false leak |
|---|---|---|---|
| Truck | F, 1.0 native px | 98.99% | 0.00% (0/9688) |
| Truck | E, 1.0 native px | 99.23% | 0.00% |
| Truck | F, 2.89 native px | 99.31% | 0.04% (4/9688) |
| Truck | E, 2.89 native px | 99.48% | 0.20% (19/9688) |
| Meetingroom | F, 1.0 native px | 99.94% | 0.19% |
| Courthouse | F, 1.0 native px | 98.30% | 0.10% |

Two further guards worth reusing. First, an **aliasing-strength ladder** — bucketing non-co-visible
pairs by how many two-view inliers an independent matcher verified on them — is the cheap check that
a negative population actually contains the failure mode under test. Truck's tail tops out at 222
inliers, so Truck *cannot* test doppelgängers at all, and an early "rejects 100% of confusable pairs"
result taken from it was an artifact of that. Courthouse, whose tail reaches 2308, overturned it:
the leak there rises 3.3% → 28.8% → 35.5% → 37.9% → **100%** across strength buckets from 30–99 up
to 1000+ verified inliers. That is the triplet filter's problem, not this gate's, but it is only
visible on a scene that has the structure. Second, scene choice should follow the ladder rather
than intuition: Barn scored 8× Truck on Doppelgangers++ yet has Truck's tail shape (48 pairs ≥100,
one ≥200), so running it would have cost ~68 min of GPU and answered nothing.

---

## Dense Infusion (`--roma2-supplement`, off by default)

A pair the gate validated but whose descriptor matching is thin — too few correspondences, all of
them in one textured corner, or none that survive geometric verification at all — contributes
almost nothing to the reconstruction and often drops out of it. Dense infusion adds ROMAv2 dense
correspondences **alongside** whatever sparse matches that pair has, in the parts of the confident
overlap those matches left empty, so a weakly-textured pair contributes structure where it has none
instead.

It runs inside `MatchPairsROMA2`, on the warp that pass already has, and needs both
`--roma2-validate` and `--roma2-match`: the gate is what makes "validated" mean anything, and the
guided pass is where the warp is, so there is no third warp pass. The trigger is read off the
geometry the gate handed the guided pass — non-NULL exactly for a pair the gate judged and kept — so
with the gate off nothing is infused by construction rather than by a second check that could
disagree with the first.

**The decision is taken right after the guided SIFT pass and its geometric filter**, while the pair's
warp and the draw that would complement it are both still at hand — not inside the branch where the
SIFT pass survived, which is where it first shipped and which silently excluded the pairs the feature
exists for. It reads ONE quantity:

> **SIFT coverage of the valid disparity area.** `coverage = occupied / confident` over the
> complementary draw's own (fine) bucket grid: `confident` counts the buckets holding at least one
> confident, in-frame warp cell — the RoMa2 valid-disparity area the gate judged the pair on — and
> `occupied` counts those of them a verified SIFT inlier lands in (image-A frame). A pair whose SIFT
> pass failed covers nothing, so its coverage is 0.

Coverage, and not the two quantities the first version used. The inlier *count* says nothing about
where those inliers are: a pair with 2000 of them piled into one textured corner has an overlap whose
remainder is exactly what a dense draw is for. The warp's *confident fraction* says nothing about the
sparse matches at all — it is a property of the warp, and the old "overlap < 0.3" clause read it as
if it were a property of the matching.

```
infuse  <=>  the SIFT pass failed the geometric filter
             OR verified sparse inliers < --roma2-supplement-max-inliers   (default 500)
             OR coverage < --roma2-supplement-min-coverage                 (default 0.3)
budget   =   round(--roma2-supplement-total-matches * (1 - coverage))      (default total 2000)
```

The budget is the share of the total proportional to the part of the valid area still empty — the
part the draw can actually fill. `0` as the total keeps its meaning: no total cap, the draw bounded
by `--roma2-dense-sample` alone.

**The draw complements the sparse matches, it does not repeat them.** The supplement has its own
draw, `SampleWarpComplementary` (`ROMA2Warp.cpp`) — the gate keeps `SampleWarpByCoverage`, which must
stay blind to descriptors because its job is to *test* the warp. Three things separate the two:

1. **Occupied buckets are struck out.** The pair's verified sparse inliers are mapped, by their
   image-A keypoint positions, into the same n×n bucket grid the draw stratifies on, and every
   bucket one of them lands in yields no dense point at all. Occupancy is read in A's frame alone:
   the warp grid lives in A's frame, so that is the one frame where a keypoint position and a warp
   cell are directly comparable, and on a genuine pair — the only kind that reaches here — the warp
   carries that density over to B. A second grid in B would also need a B→A back-map the coarse warp
   does not carry.
2. **The grid is sized from the TOTAL, not from the budget.** n comes from
   `--roma2-supplement-total-matches` over the eligible cells, so one bucket stands for about one of
   the total correspondences — which is what makes the bucket census a *share of the valid area* and
   not a circular definition, since the budget it produces cannot also size the grid it is measured
   on. The draw takes one point per unoccupied confident bucket and is then thinned to the budget.
3. **Thinning preserves the spread.** An even stride through the warp raster order
   (`ThinSampleEvenly`, shared by the draw's own cap and by the caller thinning to the budget).
   Never by confidence — a confidence sort would re-cluster the survivors onto the warp's most
   certain region, which is the textured region the sparse matcher already covered, and undo the
   whole stratification.

**The in-bucket winner is a pair-independent lattice priority**, not the most confident cell:
`priority(x, y)` is the largest k for which both warp-cell coordinates are multiples of 2ᵏ, ties
going to raster order, and confidence stays the *eligibility* test (the eroded `minConfidence` bar).
The bucket grid is adaptive, so two pairs sharing image A stratify the same region of A on grids of
different pitch and phase; ranking by each pair's own confidences then puts their A-side points a few
cells apart, and the exact-position keypoint dedup below sees two keypoints where one surface point
was sampled twice. A lattice priority makes any two buckets covering a common region agree on the
cell whenever both find it eligible, so those samples land on the same pixel of A and chain into one
track. The rule is shared with `SampleWarpByCoverage` — one winner rule, not two. **Chaining is
through the A side only**: the B-side position is whatever the warp maps that cell to, a float two
pairs have no reason to agree on, so a chain grows along the images that play the A role of their
pairs and stops naturally wherever the confident overlaps stop coinciding. Expect chains of about
three, not long tracks.

Each drawn point carries its winning cell's own confidence — the value read where it was selected,
not a bilinear read-back of the map — which is what `Image::MakeDenseKeypoint` stamps as the point's
response.

**A validated pair whose SIFT pass failed is kept as a DENSE-ONLY pair.** Zero sparse inliers, the
gate's F/E and its relative pose as the pair's geometry, and the whole total as its draw (it covers
nothing, so `1 - coverage` is 1). Dropping it would drop exactly the textureless pairs the feature
exists for. Such a result may only **create**: it carries no descriptor evidence, so it can never be
the fair winner of a replace test against a pair that has some — if the pair already exists, the
existing one is kept and the dense-only result is counted and reported instead
(`ApplyROMA2Pair(..., bCreateOnly)`, and the pass's summary line). Its partition is
`numFilteredInliers == 0`, `numDenseInliers == N`, which `FilterMatches`' re-partition accepts.

**The relative pose after infusion.** An infused pair can hold two independent fits of one geometry:
the SIFT pose, fitted on its verified sparse inliers, and the gate's dense pose, fitted on its ~2000
spread warp samples (`PairsMatcher::ValidatedGeometry::relativePose`, present only on a branch that
produces a pose — `ESSENTIAL` and `SHARED_FOCAL` do, `FUNDAMENTAL` does not). The pair keeps the SIFT
pose, which is the more accurate one **when the two agree** (sub-pixel correspondences, even without
coverage), unless they differ substantially — rotation angle of `R_sift * R_dense^T` over
`--roma2-supplement-pose-max-rot` (2°) **or** angle between the unit translation directions over
`--roma2-supplement-pose-max-trans` (10°) — and then it takes the dense pose, which rests on evidence
spread over the whole overlap. Taking the dense pose takes the gate's F and E with it: they are one
fit, and a pair holding one fit's pose next to another fit's matrices would describe two geometries
as one. With no dense pose the policy is inert.

Both thresholds are **educated first guesses**, to be fitted on pseudo-GT. Every infused pair emits
one record at `-v 3`, into the working-folder log:

```
ROMA2 pose check pair 12 37, sparse 118, coverage 0.2043, dense 1591, dR 0.8312 deg, dt 3.2077 deg,
  choice sparse, qs(w x y z) ts(x y z), qd(w x y z) td(x y z)
```

fixed prefix, fixed field order, `nan` for a missing pose, one line per infused pair — so the two
thresholds can be swept offline against a scene's COLMAP relative poses without re-running the
matcher. `--roma2-supplement-refit-pose` (off) replaces the choice with a single re-estimate on all
of the pair's correspondences, sparse and dense together, through the matcher's own estimator (same
branch, same threshold); it is a hypothesis the sweep has to confirm before it can be a default, and
a refit on a draw that came from one of the two poses is not independent evidence about it.

The draw is deliberately **not** filtered again against the pair's fitted geometry: the gate already fit one
geometry to a sample of this same warp and required its inlier subset to cover both images, so a
second pass over the same evidence would confirm rather than test it. What bounds a wrong supplement
is the gate upstream, `ROMA2Config::minConfidence` on the warp, `FilterTracks`' reprojection bar, and the
bundle-adjustment down-weighting below.

**Keypoints beyond descriptors.** `AppendDenseMatches` (`ROMA2Warp.cpp`) puts each dense point at the
end of `Image::keypoints`, past the described prefix, with no new keypoint structure:
`keypoints.size() >= Image::NumDescribedKeypoints() == descriptors.rows`, and
`Image::IsDenseKeypoint(idx)` is the whole test. The boundary is a **stored, serialized count**, not
`descriptors.rows`, because the descriptors are released before `FilterRedundantKeypoints` runs — it
cannot remap descriptor rows, which is why it only runs there — and long before bundle adjustment, so
a derived boundary would report every keypoint as dense in exactly the two places that need it. The
append is serial, in the pass's `(ID1,ID2)` order, because the keypoint indices it hands out depend on
what the two images already carry.

**The inlier count means descriptor evidence.** `ImagePair::matches` carries a three-way partition:

```
[0, numFilteredInliers)                                    sparse matches that passed the strict
                                                           filter -- track-forming, and the pair's
                                                           descriptor evidence
[numFilteredInliers, numFilteredInliers+numDenseInliers)    the dense supplement -- track-forming
[numFilteredInliers+numDenseInliers, matches.size())        RANSAC inliers the strict filter rejected
                                                           -- not track-forming
```

`GetNumFilteredInliers()` is the **first segment only**: the pair's DESCRIPTOR evidence. What the
view graph ranks on is `GetNumWeightedInliers()` — `sparse + w × dense`, rounded, with `w` the dense
observation weight — because a dense-only pair is a pair, not a zero. **There is one notion of pair
evidence and one accessor for it**, and it is what `GetCompositeWeight`'s inlier factor (under its
1000 cap), `ComputePairsWeights`' connectivity normalisation, the triplet filter's edge strength,
`StarInitializer`'s degree and candidate ranking, rotation averaging's unweighted fallback,
`ViewGraphCalibrator`'s two "enough inliers to trust this F" guards, `GlobalAlignment`'s
`minCommonTracks` bar, the feedback round's `feedbackSkipHealthyInliers` skip and `ApplyROMA2Pair`'s
replace test all read.

`w` has **one definition** (`SFM::DENSE_OBSERVATION_WEIGHT`, `ImagePair.h`), shared by
`BAConfig::denseObservationWeight` and `PairsWeightingConfig::denseObservationWeight` and driven by
the single `--ba-dense-weight` option: the same statement about measurement precision cannot have two
values. The discounted count is *stored* on the pair (`ImagePair::weightedInliers`, written by
`ComputePairsWeights`, serialized with the other weights) because `GetCompositeWeight()` and its
callers have no access to a configuration, and a second hard-coded copy of the weight would be a
second answer. Before the weighting pass has run, the accessor answers with the sparse count — the
pre-supplement answer, which is what the matcher-internal readers always used.

The remaining sparse-only readers are the ones that genuinely mean "descriptor evidence": the
estimation bars inside `MatchFeaturesGeometric`/`GeometricFilter`/`MatchPair` (which run on pairs that
have no dense segment yet, and whose `minMatches` is a bar on descriptor matching), the supplement's
own occupancy gather and its `--roma2-supplement-max-inliers` trigger, and the diagnostics that report
what the descriptor matcher verified (the pairs CSV, `SceneAnalyzeSFM`, `BuildTracks`' skip counters
and its pose-consistency check, the per-pass statistics).

Both terms of `weightSpatial` run over the **track-forming** matches, supplement included. The
**area** score, because coverage asks where the pair has correspondences and a dense draw covers what
it was drawn over — a dense-only pair would otherwise score no area, hence no weight, and be cut from
the graph it was deliberately kept in. The **angle** term (`ComputeAngleBaselineWeight(meanRayAngle)`)
for a sharper reason: it is the only factor that can demote a pair for a degenerate baseline, and
`ComputeAngleBaselineWeight(0)` is not a neutral value but the function's **maximum** — so a
`meanRayAngle` accumulated over sparse matches alone hands every dense-only pair the best possible
baseline score by construction. A ray angle is a geometric quantity, not a sub-pixel one: a whole warp
cell of position error is ~0.05° of ray direction against baselines measured in degrees, so the
precision argument that keeps a dense position out of the reprojection-based checks does not reach it.
`FilterMatches` therefore accumulates the median over every accepted track-forming match, and
`AppendDenseMatches` — which changes that set without re-filtering the pair, deliberately — re-measures
it through `ImagePair::ComputeMeanRayAngle`. Only a pair with no relative pose at all still reads 0,
i.e. a baseline that was never measurable; nothing in the product demotes such a pair.

**The validity floor reads `GetNumTrackFormingMatches()`**, undiscounted. It asks "does this pair
carry enough verified correspondence to be considered at all", and a gate-validated infused pair does.
It has to be split from the magnitude because returning 0 there zeroes `weightSpatial`, hence
`GetCompositeWeight()`, hence `BuildTracks`' `minPairWeight` cut: an infused pair whose sparse
segment dips under 15 — normal once cross-pair dense reuse and the duplicate-match filter have run on
a pair infused at 60 sparse inliers — would contribute **zero** tracks, sparse or dense, and the
feature would be silently inert on exactly the weak pairs it exists to serve. One that then falls
under `minPairWeight` on the discounted magnitude is a correct drop. The alternatives were rejected:
exempting infused pairs from `minPairWeight` hides a real drop, and giving the gate a weight the view
graph does not rank on gives the pair two different authorities.

What forms tracks is the union of the first two segments, `GetNumTrackFormingMatches()`, and that is
what bounds `BuildTracks`' union-find (and `GlobalAlignment`'s cross-sub-scene equivalent). The
supplement is inside that prefix rather than at the end of `matches` precisely because a match past it
would form no track and the whole feature would be inert. The duplicate-match filter in
`FilterRedundantKeypoints` compacts **order-preservingly** and recomputes both counts by counting
survivors per original segment, so the partition is exact by construction: sorting `matches` there
would move every dense match (dense keypoints hold the largest indices) past the boundary while the
strict filter's rejects migrated into it — part of the supplement silently inert, an equal number of
deliberately rejected matches silently forming tracks.

**A pair the filter pushes below `minMatches` orphans its dense keypoints.** `FilterRedundantKeypoints`
calls `InvalidateMatches()` on it, and the dense keypoints that pair appended to both images stay in
`Image::keypoints` with nothing referencing them: counted in the boundary arithmetic, serialized to
`.sfm`, walked by `BuildTracks`' keypoint enumeration and by every later run of the filter, never
observed. **Cost only, no wrong answer**: step 3 of `BuildTracks` enumerates all keypoints regardless
of the partition, so an unreferenced one becomes a singleton union-find root that step 4's "fewer than
2 views" filter drops. The same is true of a supplemented pair replaced in a later round. Not worth
code — the alternative is reference-counting keypoints per pair, for memory a bounded draw already
caps.

**A re-verification holds a supplemented pair to the descriptor bar alone, so a focal update can drop
it whole.** `FilterMatches` returns the sparse count and its three callers invalidate the pair below
`minMatches`, which is right — that bar is a descriptor-evidence bar — but it means a pair that ended
matching with 55 sparse and 900 dense matches and comes out of a `ViewGraphCalibrator` focal update
with 48 sparse loses all 948, orphaning its dense keypoints in both images by the mechanism above at
900× the scale. Before supplementation the same call returned ~948 and the pair survived.

**A re-verification can also *raise* a supplemented pair's descriptor evidence.** The two sites that
maintain the partition classify a fully-collapsed supplement match — one whose *both* endpoints were
cross-pruned onto coincident described survivors — differently, by design: `FilterRedundantKeypoints`'
step 3 maintains the dense segment by **position** and leaves it there, while `FilterMatches`
re-derives the segment by **predicate** and calls it sparse, on the grounds that it now measures a
sub-pixel described position at both ends. So a pair reported at 55 sparse / 897 dense can come out of
a focal update at 58 / 894. This is also why `ImagePair::CheckSparseSegmentIsDescribed` asserts only
that no match in the sparse segment has a dense endpoint, and not the converse.

**Cross-pair identity is exact-position reuse, not proximity fusion.** There is no fusion radius:
the warp is sampled from a low-resolution disparity field, so averaging two nearby samples compounds
their error. What is reused is the identity `FilterRedundantKeypoints` already computes — two points
at the same position within its 0.1 px tolerance are one point, and it *keeps a survivor* rather than
averaging, so it adds no positional error. Because the draw comes from a deterministic stratified
grid, two pairs sharing an image often sample the same cell, which is how a dense-only track reaches
more than two views. Duplicate resolution is **described-wins**, stated explicitly against the stored
count: a dense point coinciding with a described keypoint collapses onto the described one, which
carries the descriptor and the sub-pixel position; between two dense points the higher warp
confidence wins. The survivors of an image carrying dense keypoints keep their original relative
order so the described prefix stays a prefix, and the stored count moves through the same remap.
`BuildTracks` reports the dense track-length histogram: a histogram sitting entirely at 2 means the
reuse never fired, which is itself a finding — and is the expected output under
`--release-descriptors false`, where the keypoint filter does not run at all.

**Dense observations are down-weighted in bundle adjustment** by `--ba-dense-weight`: a dense
keypoint's position was sampled from a low-resolution warp, a described one's is sub-pixel at full
resolution, and the two are not equally precise measurements. The weight follows the *keypoint*, not
the match that created it — after described-wins dedup an observation created by a dense match can
reference a described keypoint, and it then takes full weight. It models measurement precision only;
a wrong correspondence is the robust loss's and `FilterTracks`' job. **The 0.25 default is
provisional and was not measured**: it is `k = 2` in the `1/k^2` a ratio of robust sigmas implies,
picked at the mild end so that a wrong provisional value errs toward the pre-existing behaviour
rather than toward discarding the dense signal. The value that belongs there is the robust sigma of
dense versus described reprojection residuals on a ground-truth capture.

It is **exclusive with `BAConfig::useKeypointConfidence`**, which says the same thing by another route:
`ComputeKeypointPrecision`'s `SQUARE(2/max(size,1))` term reads measurement precision off the sampling
scale, and the dense `size` convention was chosen precisely so that it reports a dense point as the
less precise measurement. Multiplying the two charges the sampling scale twice — on the documented
values that is a ~116x ratio, `k ~ 10.8`, rather than the `k = 2` the default claims, and in the
opposite direction from "errs toward the pre-existing behaviour". So `SelectReprojectionLoss` applies
the flat weight only when the confidence term is off, and **the `--ba-dense-weight` measurement this
branch still owes must be made with `useKeypointConfidence` off** or it fits a quantity that already
contains the factor being fitted.

**Scale.** An infused pair costs up to `--roma2-supplement-total-matches` keypoints in *each* of
its two images — fewer, in proportion to the share of its valid disparity area the sparse matches
already cover — plus about one track per dense correspondence, less whatever the cross-pair lattice
coincidence chains together. `sizeof(cv::KeyPoint)` is 28 bytes, but the keypoints are the small part:
`BuildTracks` builds a union-find slot, a per-root image map, a `std::map` node and a `Track` over
every one of them, which analytically works out around 0.25 KB per appended keypoint at peak. At the
documented 500-image / 30-pairs-per-image scene with a fifth of pairs supplemented, a draw with no
total budget is ~10 M appended keypoints — ~0.3 GB of `cv::KeyPoint` and on the order of 2.5–3 GB
once the track bookkeeping is counted. Start a wide arm at `--roma2-supplement-total-matches 500` and
raise it once the contamination-by-track-length numbers are in.

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
  `ApplyROMA2Pair`'s store/replace-by-inlier-count policy (including the `maxReplaceInliers` ceiling),
  and `AppendDenseMatches` — the appended keypoints landing past each image's described prefix with
  the warp-cell size and confidence convention, the appended matches landing *inside* the pair's
  filtered-inlier prefix (the only part `BuildTracks` reads), and a second append on the same images
  leaving the boundary where the first one put it.
- **`ROMA2SupplementDrawTest`** (`apps/Tests/TestsSFM.cpp`) — always runs, no model needed: the
  complementary draw on a synthetic warp with the sparse inliers clustered in one corner. No dense
  point lands in a bucket those inliers hold, one does land in every *other* bucket that has a
  candidate, the bucket census matches an independent count of the same grid and the budget it buys
  keeps the draw inside the total, an over-budget draw thinned by an even stride still hits all four
  quadrants, two opposite confidence ramps over one region draw exactly the same points (confidence
  is eligibility, not ranking), two pairs sharing image A on different bucket grids land their A-side
  points on the same pixels wherever both cover and both are confident, the sample stays in warp
  raster order, and the draw is a pure function of its inputs.
- **`ROMA2DenseInfusionTest`** (`apps/Tests/TestsSFM.cpp`) — always runs, no model needed: the
  infusion decision and the pose choice. A validated pair whose SIFT pass failed is infused at the
  whole total and becomes a dense-only pair that keeps the gate's geometry, partitions as 0 sparse /
  N dense, carries a positive composite weight and forms tracks in `BuildTracks`; a strong but
  clustered pair (past `--roma2-supplement-max-inliers`, under `--roma2-supplement-min-coverage`) is
  infused with `round(total × (1 − coverage))` points, none of them in a bucket its inliers hold; a
  strong pair spread over the valid area gets nothing; and the pose choice keeps the SIFT pose while
  both angles are inside their thresholds, takes the dense pose once either is not, and falls back
  correctly when one of the two poses does not exist.
- **`DenseKeypointBoundaryTest`** (`apps/Tests/TestsSFM.cpp`) — always runs, no model needed: the
  stored described-keypoint count surviving a descriptor release and an `.sfm` round-trip of an image
  whose `keypoints.size() > descriptors.rows`, `SelectTopKeypoints` staying inside the prefix, and
  `PairsMatcher::FilterRedundantKeypoints` moving the boundary through the same remap it applies to
  the indices — a removal *inside* the prefix shrinks it, described-wins keeps a described keypoint
  from losing to a coincident dense one, and the more confident of two coincident dense ones survives.
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
while the always-on tests keep running (they exercise host-side code only).
`RoMa2OnnxParityTest` additionally skips — loudly, with "no reference dumps under '<dir>', parity not
checked" — when the model directory ships graphs but no `.reference/` dumps, since there is then
nothing to compare; naming a preset through `OPENMVS_ROMA2_SETTING` still fails hard in that case,
because the user then asked for a comparison that cannot be made.

---

## Compatibility (`.sfm` scene files)

This branch's stream layout is `SFM_PROJECT_VERSION` **1** (`libs/SFM/Scene.cpp`) — the layout that
carries the per-image global descriptor, `Image`'s described/dense keypoint boundary and the
per-pair dense-supplement match count. The loader accepts that exact version and refuses anything
else outright:

```
error: unsupported SFM project version 2 (this build reads only version 1) in '<file>'
```

In particular there is no "derive the keypoint boundary when the stream does not carry it" branch:
the boundary had to be *stored* rather than derived (see Dense Infusion), and a derived value
is exactly the silent misclassification the stored count exists to prevent.

There is no converter and none is planned: a `.sfm` written by any other layout is regenerated by
re-running the matching stage from the images (`CreateStructure -s <images> -o scene.sfm ...`).
`.mvs` files are unaffected.

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
