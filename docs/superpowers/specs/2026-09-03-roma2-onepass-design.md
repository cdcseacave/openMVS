# One-pass dense pair matching with RoMa v2 — design

Date: 2026-09-03. Branch `feature/roma2-onnx`. Approved by the user on 2026-09-03 ("go") on top of the
step-by-step plan presented the same morning; the measurements behind every number below are in the
ledger `.superpowers/sdd/roma2-matching-redesign-20260831/progress.md` (entries of 2026-09-03, rulings
R102–R105) and in `~/virginia/datasets/<capture>/openmvs-roma2-20260903-task7-probe-{model,sfm}/`.

This document is the binding authority for the implementation plan
`~/.claude/plans/roma2-onepass-20260903.md`. Where the plan and this document disagree, this document wins.

## 1. What changes

Today the ROMA2 side of `PairsMatcher::Match` is a chain of opt-in passes layered on top of the SIFT
batch: a dense two-view gate that *routes* pairs, a plain SIFT batch, a guided re-match that *replaces*
pairs by a per-round policy, and a dense infusion with its own weak-pair thresholds and pose choice. The
user rejected that shape: the verdict on an image pair must come from the ROMA2 dense matching alone, in
one pass, with no union of SIFT-verified and warp-verified pairs and no fallback to SIFT for a pair the
warp rejects (rulings R102, R104).

With `--roma2-match true` a matching round is now, per candidate pair (A, B), A < B:

1. **Warp, both ways, one graph call.** The coarse-match graph returns the A→B warp and confidence and,
   from the same pair-ViT pass, the B→A warp and confidence.
2. **Verdict.** One geometry is fitted on a coverage-uniform subsample of A's confident cells through
   PairsMatcher's own estimator. Its inlier area is measured on ALL of A's confident cells and, with the
   same geometry, on all of B's confident cells through the B→A warp. The pair is admitted iff
   `min(inlierAreaA, inlierAreaB) >= minOverlap`. A rejected pair is dropped: no SIFT matching, no
   second chance in this round.
3. **Guided sparse matching.** The described keypoints of A are tracked into B through the warp; for
   each, the candidates are B's described keypoints inside a disc of two warp cells around the
   prediction; the winner is the best descriptor distance in the disc, accepted iff it beats the best
   descriptor distance OUTSIDE the disc by the matcher's ratio. This is where appearance (descriptor
   agreement) enters, and the only place it does (R102).
4. **Dense fill.** Correspondences are drawn from the verdict's inlier cells where the sparse candidates
   are not (the complementary draw), up to `denseMatches` per pair.
5. **One geometry for the pair.** PairsMatcher's estimator runs once more on sparse ∪ dense; the sparse
   inliers at the matcher's own epipolar threshold become the pair's descriptor evidence, the dense
   points within the warp tolerance its dense segment. The pair is stored once (created, or replacing a
   same-key pair a previous Match() left).

Everything else of the old ROMA2 matching side is deleted: the gate and its coverage scale, the
routing, the replace policy, the infusion decision and pose choice, the cross-check, the erosion, and
every CLI option that drove them. No backward compatibility anywhere (config, CLI, `.sfm`, docs).

Refiners, resolutions above 640 and the cycle-consistency test are out (R105 and the model probe):
the coarse bidirectional warp at 640 is the whole model side.

## 2. Measured basis (why these rules)

- A hallucinated warp is a smooth field, locally a homography, and every homography is explained
  exactly by a family of fundamental matrices: RANSAC inlier COUNTS or RATIOS on the warp's own cells
  cannot tell a hallucination from a true pair (identical distributions on the labelled none/good
  groups of all three captures). Lower confidence floors make the count rule strictly worse.
- The min-side inlier AREA of one geometry does: at floor 0.1 and θ = 0.10 it admitted zero
  hallucinated (non-co-visible, warp-wrong) pairs on 8d2f4877, 38004114 and Truck, at both floors,
  without any cycle test. The B side is the precision: it rejects the "whole of A on a few pixels of
  B" pairs and 10 of Truck's 11 A-side-only admissions.
- Recall (kept share per GT overlap band .15–.25 / .25–.40 / .40–.60 / ≥.60), one-pass rule at floor
  0.1, θ 0.10, versus the SIFT baseline on the same pairs:
  8d2f4877 0.88/0.91/1.00/1.00 vs 0.47/0.60/0.75/1.00; 38004114 0.78/0.95/0.98/0.97 vs
  0.50/0.38/0.78/0.90; Truck 1.00/1.00/1.00/1.00 vs 0.53/0.55/0.75/0.97.
- The measured min-side inlier area is 0.55–0.75 of the GT overlap: θ 0.10 corresponds to ~0.15–0.17
  true overlap; θ 0.15 to the user's proposed 0.25 of the image surface.
- Coarse θ 0.08 ≈ fine θ 0.10 for the verdict; the refiners only buy match precision, which the SIFT
  inliers already supply where texture exists (user, R105).

## 3. Interfaces (binding)

All names below are the ones the code must use. Anything of the old surface not listed here is deleted.

### 3.1 ROMA2Config (`libs/SFM/MatchROMA2.h`)

```cpp
struct SFM_API ROMA2Config {
	bool enabled = false;          // enable the in-process ROMAv2 model (explicit opt-in)
	String modelPath;              // folder of the exported graphs (empty = $OPENMVS_ROMA2_MODEL_PATH)
	String setting = "base";       // preset: turbo|fast|base
	String provider = "auto";      // execution provider: auto|cuda|coreml|dml|cpu
	bool useRetrieval = true;      // rank candidate pairs with the ROMAv2 global descriptors
	bool useMatching = false;      // one-pass dense pair matching (verdict, guided, fill, store)
	float minConfidence = 0.1f;    // a warp cell takes part (verdict, tracking, fill) at this confidence or above
	float minOverlap = 0.10f;      // verdict: min(inlier area A, inlier area B) >= minOverlap
	unsigned denseMatches = 2000;  // dense fill cap per pair
	unsigned slotBudget = 64;      // image descriptors kept resident on the device
	bool useGPU = true;            // allow the GPU execution providers
	String ResolveModelPath() const;         // unchanged
	bool IsInProcessEnabled() const;         // enabled && (useRetrieval || useMatching) && path resolvable
	bool NeedsWarps() const { return useMatching; }
};
```

Deleted fields: `minErodeConfidence`, `erodeBorder`, `epipolarThreshold`, `maxReplaceInliers`,
`skipHealthyInliers`, `feedbackMaxReplaceInliers`, `feedbackSkipHealthyInliers`, `minCreatedOverlap`,
`guidedCrossCheck`, `useValidation`, `denseSampleSize`, `minInlierCoverage`, `useSupplement`,
`supplementMaxInliers`, `supplementMinCoverage`, `supplementTotalMatches`,
`supplementPoseMaxRotationDeg`, `supplementPoseMaxTranslationDeg`, `supplementRefitPose`.

CLI (`apps/CreateStructure/CreateStructure.cpp`): `--roma2`, `--roma2-model`, `--roma2-setting`,
`--roma2-provider`, `--roma2-retrieval`, `--roma2-match`, `--roma2-slots`, `--roma2-min-confidence`,
`--roma2-min-overlap`, `--roma2-dense-matches`. Every other `--roma2-*` option is deleted, and the
validation block only checks what remains (setting/provider names, ranges 0..1, slots >= 2).
`--ba-dense-weight` (BA weight of a dense observation) is unrelated and stays.

### 3.2 Graph contract (`scripts/python/roma2/`, `libs/SFM/RoMa2Matcher.h/.cpp`)

Manifest `format_version` **3**. The match graph `match_coarse`:

- inputs: `descriptors_A`, `descriptors_B` ([1,2,G,G,1024]) — the dead `img_A`/`img_B` inputs are
  dropped from the trace; if the wrapper cannot be traced without them the exporter keeps them and
  says so in its report, and the C++ binds them only when the manifest's `io` lists them (that is the
  ONE tolerated variation, decided by the exported manifest, not a compatibility path).
- outputs: `warp` [1,C,C,2], `confidence` [1,C,C,1] (A→B, unchanged), `warp_BA` [1,C,C,2],
  `confidence_BA` [1,C,C,1] (B→A, from the same forward pass, `bidirectional=True`).
- The reference dumps (`*.reference`, `save_reference`) and the parity check cover all four outputs.
- The loader rejects any `format_version` other than 3 with a message naming the version.

```cpp
// Run the coarse-match graph once on two descriptor tensors, returning both directions:
// the C x C normalized warp of A's cells into B with its confidence (sigmoid of the logit), and
// the same for B's cells into A
bool RoMa2Onnx::MatchCoarse(const OrtTensor& layersA, const OrtTensor& layersB,
	Image32F2& warpAB, Image32F& confidenceAB, Image32F2& warpBA, Image32F& confidenceBA);
```

The exported model lands in `~/virginia/models/roma2-onnx/roma2onnx-20260903-bidir/` (three presets,
references, `export.log`), and `OPENMVS_ROMA2_MODEL_PATH` points there from then on.

### 3.3 Warp types (`libs/SFM/ROMA2Warp.h`)

```cpp
struct SFM_API WarpMaps {            // one direction
	Image32F2 warp;                  // normalized (align_corners=false) target coordinates in [-1,1]
	Image32F confidence;             // matching confidence in [0,1]
	bool IsValid() const;            // non-empty and same size
};
struct SFM_API PairWarps {           // both directions of one (A,B) pair
	WarpMaps ab, ba;
	bool IsValid() const;            // both valid, same size
};
```

Kept (signatures unchanged unless noted): `CoordFromTo`, `DenormCoord`, `TrackKeypointsByWarp`,
`SampleWarpByCoverage`, `ThinSampleEvenly`, `SampleWarpComplementary` (the `occupiedA` form only —
the `ImagePair` form and `WarpDrawCoverage` are deleted), `AppendDenseMatches`.
Deleted: `ErodeConfidenceMap`, `ComputeSampleCoverage`, `WarpDrawCoverage`, `DENSE_COVERAGE_GRID` if
nothing uses it after the deletions, `ApplyROMA2Pair` (replaced by `StorePairROMA2`, §3.6).

Warp tolerance, used by the verdict and the dense segment (§3.4, §3.6), is a function, not a config:

```cpp
// Half a warp cell in image pixels: the accuracy a coarse-warp correspondence can claim, and the
// epipolar tolerance every test on warp cells uses (the probes measured at 2 px on the 640/160 grid)
inline float WarpTolerance(const cv::Size& sizeA, const cv::Size& sizeB, int warpSize) {
	return 0.5f * (float)MAXF(MAXF(sizeA.width, sizeA.height), MAXF(sizeB.width, sizeB.height)) / (float)warpSize;
}
```

### 3.4 The verdict (`libs/SFM/MatchROMA2.h/.cpp`)

```cpp
// What the verdict knows about one pair once it ran
struct SFM_API PairVerdict {
	bool admitted = false;
	float confidentAreaA = 0.f, confidentAreaB = 0.f; // share of each warp grid at conf >= minConfidence and landing in-frame
	float inlierAreaA = 0.f, inlierAreaB = 0.f;       // share of each warp grid the fitted geometry explains
	// A's inlier cells, raster order, index-parallel, in the pixels of the working orientation of
	// each image (SampleWarpByCoverage's convention), with each cell's confidence: the population the
	// dense fill draws from
	std::vector<Point2f> inliersA, inliersB;
	std::vector<float> confidences;
};

// Judge one candidate pair from its bidirectional warp alone. Fits one geometry on a coverage-uniform
// sample (SampleWarpByCoverage, target VERDICT_SAMPLE = 4000) of A's cells at conf >= config.minConfidence
// through pairsMatcher.GeometricFilter(tmpA, tmpB, pair, WarpTolerance(...)) on temporary Image copies
// carrying the sample as keypoints and no pose (the ValidateOnePairROMA2 precedent: a scene holding a
// ground-truth solution must not leak it into the fit); scores ALL of A's eligible cells against
// pair.F (Sampson distance <= WarpTolerance) -> inlierAreaA; scores all of B's eligible cells, each as
// the correspondence (DenormCoord(ba.warp[cell]) in A, cell centre in B), against the same pair.F ->
// inlierAreaB; admits iff min(inlierAreaA, inlierAreaB) >= config.minOverlap.
// On admission `pair` carries the fit's F/E/relativePose (whichever the branch produced) and no
// matches; verdict.inliersA/B are A's inlier cells. On rejection pair is Reset() and verdict says why
// (areas filled as far as they were computed). A sample under 8 cells is rejected without a fit.
// Pure function of its inputs; runs on the pool (GeometricFilter is const and thread-safe).
SFM_API void JudgePairROMA2(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const PairWarps& warps, const ROMA2Config& config, ImagePair& pair, PairVerdict& verdict);
```

`PairsMatcher::GeometricFilter` gains the overload
`bool GeometricFilter(const Image&, const Image&, ImagePair&, float maxEpipolarError) const;` the
existing three-argument form forwards `config.maxEpipolarError`. The branch choice
(`SelectGeometryBranch`), the strict filters and `minMatches` are the estimator's own, unchanged.

Eligible cell: confidence >= `minConfidence` AND its warped point lands inside the other image.
Areas are fractions of the C×C grid (not of the eligible cells), so a warp confident about a corner
of the frame reads as a corner.

### 3.5 Guided sparse matching (`libs/SFM/MatchGeometric.h/.cpp`)

```cpp
// Guided sparse matching of one admitted pair: for each described keypoint i of imgA that the warp
// tracked (trackStatus[i] == 1, prediction trackedB[i] in imgB's pixels), the candidates are imgB's
// described keypoints within discRadius of the prediction; the winner is the candidate with the
// smallest descriptor distance, accepted iff  d(winner) <= matchRatio * d(best keypoint of imgB OUTSIDE
// the disc). The outside distance comes from the thread's own descriptor matcher: the K_NN = 8 nearest
// neighbours of the query over all of imgB's described descriptors, the first of which not in the disc
// is "best outside"; when all K_NN are in the disc the K_NN-th distance stands in (a lower bound of the
// true outside distance, so the test can only get stricter). A keypoint with no candidate, or whose
// disc winner fails the test, yields no match. No geometry is estimated or applied here, no train-side
// collision is resolved, no descriptor-only fallback exists: geometry is the verdict's business
// (JudgePairROMA2) and the final fit's (AssemblePairROMA2).
// Output matches are (queryIdx = i, trainIdx = j, distance = d(winner)) in increasing i, so the result
// is a pure function of the inputs. Only the described prefix of either image takes part.
// Returns the number of matches.
SFM_API size_t MatchFeaturesGuided(PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const std::vector<Point2f>& trackedB, const std::vector<uchar>& trackStatus,
	float discRadius, unsigned threadIdx, std::vector<DMatch>& matches);
```

`discRadius` = two warp cells in imgB's pixels: `2 * MAXF(imgB.width, imgB.height) / warpSize`.
`MatchFeaturesGeometric` loses everything the ROMA2 path put in it — the `validatedGeometry`
parameter and its branches, the cross-check, `numSharedTrain` — and survives in that trimmed form for
the ONE caller the redesign does not touch: `KeyframeExtractor`, which matches two consecutive video
frames from optical-flow tracks, with no warp, no verdict and no supplied geometry (amendment of
2026-09-03, ruling R112 in the ledger). `PairsMatcher::ValidatedGeometry` is deleted.
The guided pass never calls it, and the two functions share only the file.

### 3.6 Pair assembly and storage (`libs/SFM/MatchROMA2.h/.cpp`, `libs/SFM/ROMA2Warp.h/.cpp`)

```cpp
// Turn an admitted pair's evidence into the pair the scene stores: draw the dense fill from the
// verdict's inlier cells where the guided candidates are not (SampleWarpComplementary with occupiedA =
// the A positions of `guided`, cap config.denseMatches); fit ONE geometry on guided ∪ dense through
// pairsMatcher.GeometricFilter at the matcher's own maxEpipolarError on temporary Image copies whose
// keypoints are those correspondences; then classify against pair.F: the guided matches within
// config.maxEpipolarError are the sparse segment (`pair.matches[0, numFilteredInliers)`, the pair's
// descriptor evidence), the dense correspondences within WarpTolerance are the dense segment (returned
// in `dense`, appended by StorePairROMA2 after the pair exists). When the union fit fails (too few
// inliers at the sparse tolerance, or the branch's strict filters), the verdict's geometry stands:
// the classification runs against it instead, so an admitted pair is always stored -- a pair with
// zero sparse inliers is a dense-only pair, its dense segment its whole evidence.
// Returns true when the pair carries at least one correspondence of either kind.
SFM_API bool AssemblePairROMA2(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const PairVerdict& verdict, const std::vector<DMatch>& guided, const ROMA2Config& config, int warpSize,
	ImagePair& pair, DenseMatches& dense);

struct SFM_API DenseMatches {            // one pair's dense segment before it is appended
	std::vector<Point2f> pointsA, pointsB;
	std::vector<float> confidences;
};

// Store one assembled pair: create it, or replace the same-key pair a previous Match() left, then
// append its dense segment (AppendDenseMatches). Serial, in (ID1,ID2) order -- the keypoint indices
// the append hands out depend on what the two images already carry.
SFM_API void StorePairROMA2(Scene& scene, std::unordered_map<PairIdx::PairIndex, IIndex>& pairIndexMap,
	ImagePair&& pair, const DenseMatches& dense, int warpSize);
```

The relative pose the pair carries is the one the kept geometry produced (the union fit's, else the
verdict's); it is never composed from another fit's F. `ImagePair::weightedInliers` and
`DENSE_OBSERVATION_WEIGHT` keep their meaning (sparse + 0.25 · dense).

### 3.7 The pass (`libs/SFM/MatchROMA2.h/.cpp`, `libs/SFM/PairsMatcher.cpp`)

```cpp
// One-pass dense pair matching of the given candidate pairs (each unordered pair once, i < j): sorts
// them in (ID1,ID2) order, plans the device slots (Belady, config.slotBudget), and on the calling
// thread describes the slots and runs the bidirectional coarse-match graph pair by pair while the pool
// judges each pair (JudgePairROMA2), guides its sparse matching (TrackKeypointsByWarp,
// MatchFeaturesGuided) and assembles it (AssemblePairROMA2); assembled pairs are stored serially in
// (ID1,ID2) order (StorePairROMA2). A candidate already in scene.pairs is skipped (the feedback round
// proposes only new pairs, and a re-run must not double-store). A pair whose image could not be
// described or whose graph call failed is dropped with a message.
// The summary line reports candidates, judged, admitted, stored, dense-only, and the slot plan's
// loads/reloads (the cache cost of the order).
// Returns the number of pairs stored.
SFM_API unsigned MatchPairsROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, const PairIdxArr& candidatePairs, const ROMA2Config& config);
```

Pair order and cache: pairs are processed in (ID1,ID2) order, so consecutive pairs share their first
image and every use of an image falls inside a short window; the slot plan (Belady over that order)
keeps the rest resident up to the budget. Only the (A,B) pair with A < B is ever warped: the B→A
direction comes out of the same call, and the candidate list holds each unordered pair once
(`MakePairIdx` orders the indices; the pass asserts it and de-duplicates after sorting).

`PairsMatcher::Match`, per round: with `roma2 && roma2Cfg.useMatching` the round is
`MatchPairsROMA2(*this, *roma2, pairs, roma2Cfg)` and nothing else — no PreMatch, no
`OptimizePairsOrder`, no `MatchPairsBatch`; otherwise the round is the existing SIFT flow, untouched.
The verification-feedback round proposes from the stored pairs' verified matches as today and runs
the same one pass on the proposed pairs. Deleted from PairsMatcher: `ValidatedGeometry`,
`SetValidatedGeometry`, `FindValidatedGeometry`, `validatedGeometries`, `warpCoverageScale` and its
accessors, `MatchStats::roma2Pairs` becomes `densePairs` (pairs the one pass stored),
the `ValidatePairsROMA2` call and the `roma2Candidates` snapshot.

### 3.8 Per-pair log record

At verbosity 3 one line per judged pair, machine-parseable, the format every measurement tool reads:

```
ROMA2 pair <ID1>-<ID2>: conf <areaA> <areaB> inl <inlA> <inlB> <ADMIT|REJECT> guided <n> sparse <n> dense <n> <ms>ms
```

Rejected pairs stop after `REJECT`. `<ms>` is the pool time of the pair.

## 4. Deleted code (exhaustive)

`MatchROMA2.h/.cpp`: `InfusedPoseChoice`, `InfusedPoseChoiceName`, `DenseSupplement`,
`DrawDenseSupplement`, `RefitInfusedPose`, `RelativePoseDifference`, `SelectInfusedPose`,
`ChooseInfusedPose`, `LogInfusedPoseCheck`, `DensePairValidation`, `ValidateOnePairROMA2`,
`ValidatePairsROMA2`, the replace policy and the `bFeedbackRound` parameter, the "ROMA2 pose check"
and gate records. `ROMA2Warp.h/.cpp`: see §3.3. `MatchGeometric.h/.cpp`: see §3.5.
`PairsMatcher.h/.cpp`: see §3.7. `CreateStructure.cpp`: see §3.1. Tests: `ROMA2SupplementDrawTest`
adapted to the kept draw, `ROMA2DenseInfusionTest` deleted (replaced by the assembly test).
Tests of the deleted knobs go with them: `GuidedCrossCheckTest` and both supplied-geometry tests of
`MatchFeaturesGeometric`; `MatchGeometricSphericalTest` stays, as the keyframe path's cover.
Docs: `docs/design/ROMA2InProcess.md` sections "Dense Two-View Gate", "Dense Infusion", "Per-Round
Replace Policy" replaced by one section "One-Pass Dense Pair Matching"; "Graph Contract" rewritten
for format 3; "Tests", "Compatibility", "Limitations" updated; `docs/features_catalog.md` entry
rewritten.

## 5. Tests

- `RoMa2OnnxParityTest`: all four match outputs against the reference dumps.
- `ROMA2VerdictTest` (synthetic): two pinhole cameras looking at a non-planar surface; the exact
  bidirectional warp computed by projection with confidence 1 on a chosen region and 0 elsewhere.
  (a) region covering ~30 % of both frames → admitted, both inlier areas within 0.05 of the region's
  share; (b) confidence on 30 % of A but B's confident cells mapping into A only over 3 % → rejected
  (min side); (c) a random smooth warp unrelated to the cameras (a homography of A's grid) with the
  same B-side confidence → rejected; (d) minOverlap 0 admits (a) and (b).
- `ROMA2GuidedMatchTest` (synthetic descriptors): B = A's keypoints under a known offset plus
  distractors; a keypoint whose lookalike (equal descriptor) sits outside the disc is rejected; one
  whose only close descriptor is in the disc is accepted; a scale-duplicate keypoint INSIDE the disc
  (equal descriptor, same location) does not block acceptance (the classic ratio-test failure the
  outside-the-disc rule removes); output order and content are deterministic across two runs.
- `ROMA2AssemblyTest` (synthetic): guided ∪ dense on an exact geometry → sparse and dense segments
  as expected, one relative pose; with zero guided matches → dense-only pair stored with the
  verdict's geometry; `StorePairROMA2` appends past the described prefix with reproducible indices.
- `ROMA2ReconstructTest`: the bundled 4-image scene through the one pass (skipped without the model).

## 6. Measurement plan (after the code is green)

Arms on 8d2f4877 (LiDAR interior), 38004114 (textureless interior), Truck: `onepass`
(`--match-mode 4 --roma2 true --roma2-retrieval true --roma2-match true`, defaults) against the
existing `sift` and `gate-relative` runs of 2026-09-02, with the existing instruments in
`~/virginia/datasets/openmvs-roma2-20260901-task6-tools/`: per-band GT-good pair recall and stored
pair precision (`pair_eval.py`), registration counts, relative-rotation / centre error against the
pseudo-GT (`pose_eval.py`), run time and peak RSS. Then the hard `~/virginia/datasets/lidar`
captures, last. Results go to `<capture>/openmvs-roma2-20260903-onepass/` with the log beside them.

## 7. Defaults and what they mean

| knob | default | meaning |
|---|---|---|
| `--roma2-min-confidence` | 0.1 | cell floor for verdict, tracking and fill |
| `--roma2-min-overlap` | 0.10 | min-side inlier area; ≈ 0.15–0.17 true overlap; 0.15 ≈ the user's 0.25 |
| `--roma2-dense-matches` | 2000 | dense fill cap per pair |
| guided disc | 2 warp cells | fixed |
| warp tolerance | half a warp cell | fixed, in image pixels |
| sparse tolerance | `MatchConfig::maxEpipolarError` | the matcher's own |

Known limit, unchanged from the SIFT path: on a planar, textureless, small-baseline pair the
essential matrix degenerates (the warp is right, the pose is not); PairsMatcher has no homography
branch, and this design does not add one.
