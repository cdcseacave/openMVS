# The dense fill's budget and its weight in bundle adjustment — design

The one-pass matcher (`2026-09-03-roma2-onepass-design.md`) fills every admitted pair with up to
`--roma2-dense-matches 2000` warp correspondences and hands bundle adjustment a flat
`--ba-dense-weight 0.25` for each of them. Both numbers are wrong in the same way: they are
constants where the quantity they stand for is not.

The campaign that measured the one pass found the consequences. Two million tracks against fifty
thousand for the descriptor pipeline, 52-69 full bundle adjustments, and run times of one to five
hours of which matching is 269-343 s — the fill is cheap to compute and expensive to reconstruct.
On Tanks and Temples / Truck, the capture where the descriptor matcher already works, the dense
fill made every pose metric worse: it is the evidence for `--roma2-match` shipping off.

This design replaces both constants with the quantities they were standing in for.

## 1. What changes

1. **The dense fill's budget becomes a density over the overlap the sparse matches did not cover.**
   A pair draws `--roma2-dense-matches` correspondences per full frame of *uncovered* overlap
   instead of per pair. The fill's bucket grid becomes a constant of the configuration rather than
   a function of the budget, which makes the coverage discount fall out of the draw itself, bounds
   the dense keypoint density in both images of the pair, and — because two pairs sharing an image
   now stratify it on the same pitch and phase — chains their A-side samples into tracks longer
   than two.
2. **Bundle adjustment estimates the dense observation weight from the residuals instead of being
   told it.** The weight is `(sigma_described / sigma_dense)^2`, measured on the reconstruction the
   solve is about to fit. `--ba-dense-weight` becomes a manual override of an estimate.
3. **The view graph stops sharing that number.** The evidence a dense match carries for
   connectivity is not the precision of its position, and tying the two would demote exactly the
   pairs that carry a textureless capture.

Nothing else about the one pass changes: the verdict, the guided sparse pass, the storage order,
the slot plan and every default but the two above stay as they are.

## 2. Measured basis (why these rules)

Every number here comes from the three 2026-09-03 one-pass runs, scored with the campaign's own
instruments; nothing in this section is an estimate of a quantity that was not measured.

**The flat cap binds nearly everywhere, and it is a count where the thing it controls is a
density.** Replaying every admitted pair's logged verdict areas (`dense_cap_predict.py`):

| capture | min-side inlier area p10/med/p90 | dense matches per pair |
|---|---|---|
| 8d2f4877 (LiDAR interior, 225) | .122 / .221 / .492 | 1964 of 2000 |
| 38004114 (textureless, 311) | .126 / .251 / .561 | 1982 of 2000 |
| Truck (textured outdoor, 251) | .184 / .474 / .688 | 1627 of 2000 |

A pair whose overlap is a tenth of the frame receives the same 2000 correspondences as one that
overlaps completely, so its dense keypoint density is ten times higher. Density, not count, is what
the reconstruction pays for.

**The sparse matches already cover much of that overlap.** Truck's admitted pairs carry a mean of
**1677 guided matches** against 1627 dense ones, at a median sparse overlap fill of 0.9231: on that
capture nearly the whole dense fill is redundant with the descriptor evidence it is supposed to
complement. The two interiors are the opposite case — median sparse fill 0.6154 and 0.3107.

Modelling the rule of section 3.1 on the same logs (coverage estimated as Poisson over the fixed
pitch, which *over*-states coverage because real guided matches cluster on texture, so these are
lower bounds on the dense yield):

| capture | dense/pair now | proportional to overlap | proportional to **uncovered** overlap |
|---|---|---|---|
| 8d2f4877 | 1964 | 536 | **466** (4.2x fewer) |
| 38004114 | 1982 | 605 | **562** (3.5x fewer) |
| Truck | 1627 | 852 | **380** (4.3x fewer) |

The uncovered-overlap rule is the one that discriminates: it takes Truck down 4.3x, where the fill
is redundant, and the textureless interior only to 562, where it is the reconstruction. No pair of
any capture loses its fill entirely, so dense-only pairs keep their evidence.

**One flat observation weight cannot be right for two captures.** `pair_eval.py` has been recording
each stored pair's median Sampson distance to the ground-truth epipolar line, per segment. Reading
the precision ratio k off it, within pairs that carry both segments (`dense_precision_ratio.py`):

| capture | sparse Sampson med | dense Sampson med | k | 1/k^2 |
|---|---|---|---|---|
| 8d2f4877 | 0.589 px | 1.136 px | 1.93 | 0.27 |
| 38004114 | 1.152 px | 1.789 px | 1.55 | 0.41 |
| Truck | **0.324 px** | 1.303 px | **4.02** | **0.062** |

The dense sigma barely moves — 1.14 to 1.79 px, the warp's sampling scale. What moves by 3.6x is
the *described* sigma: SIFT is sub-pixel on Truck and worse than a warp cell on the textureless
interior. So k is a property of the capture, and the shipped 0.25 is about right on the interiors
while being four times too generous on Truck. That is the accuracy loss the campaign measured
there, and no constant fixes it in both places.

## 3. Interfaces (binding)

### 3.1 The dense fill's budget (`libs/SFM/ROMA2Warp.h/.cpp`, `libs/SFM/MatchROMA2.h/.cpp`)

**`ROMA2Config::denseMatches` becomes `denseMatchesPerFrame`** (`unsigned`, default 2000
unchanged): dense correspondences the fill may draw per FULL FRAME of overlap. It is a density, and
a pair's draw is that density over the part of its overlap the guided matches did not already
cover. The CLI keeps the name `--roma2-dense-matches`; its help text states the new meaning.

**The fill's bucket grid becomes a constant of the configuration.** New exported function in
`ROMA2Warp.h`:

```cpp
// Side of the square bucket grid the dense fill stratifies the WHOLE warp grid on: one bucket per
// dense match at full overlap, so the draw's density is denseMatchesPerFrame per frame whatever the
// pair's overlap is. Clamped to the warp side (a bucket is never finer than a cell).
SFM_API int DenseFillGridSide(unsigned denseMatchesPerFrame, int warpSide);
// == MINF(warpSide, MAXF(1, (int)CEIL(SQRT((double)denseMatchesPerFrame))))
```

`SampleWarpComplementary` gains an `int bucketGridSide` parameter, placed immediately before
`maxSamples`, and stops calling `WarpBucketGridSide` — that function stays, unchanged, as
`SampleWarpByCoverage`'s (the verdict sampler's grid is a different question and is not touched).
`maxSamples` keeps its present meaning: the ceiling `ThinSampleEvenly` applies at the end.

Everything else in the sampler is unchanged, and this is what makes the coverage discount exact
rather than an extra term: the draw already takes one winner per UNOCCUPIED bucket, occupancy
already being "a bucket holding a guided match's A position". With a pitch fixed at
`sqrt(denseMatchesPerFrame)` over the frame, a pair's overlap holds `denseMatchesPerFrame *
inlierAreaA` buckets, the guided matches remove the covered ones, and the yield is

```
denseMatchesPerFrame * (overlap area of A that the guided matches did not cover)
```

with no second coverage pass and no double charge. One property does not follow from the grid,
because the grid lives in A's frame: a dense match costs a keypoint in image B as well, and a warp
that puts a large part of A onto a small part of B would concentrate them there. That is what the
ceiling is for. New inline in `MatchROMA2.h`:

```cpp
// The most dense correspondences a pair may keep: the configured density over the SMALLER of the
// two inlier areas the verdict measured, so the density is bounded in image B as well as in A.
// It binds only when B is the constraining frame -- with areaA <= areaB it is above the draw the
// fixed pitch produces, so it never charges the coverage discount a second time.
inline unsigned DenseFillCeiling(const ROMA2Config& config, const PairVerdict& verdict) {
    return (unsigned)ROUND2INT(config.denseMatchesPerFrame *
        MINF(verdict.inlierAreaA, verdict.inlierAreaB));
}
```

`AssemblePairROMA2` passes `DenseFillGridSide(config.denseMatchesPerFrame, warpSize)` and
`DenseFillCeiling(config, verdict)` to `SampleWarpComplementary`. No floor: at the default
`--roma2-min-overlap 0.10` the ceiling is never below 200, and a configuration that asks for fewer
dense matches than a fit needs is asking for that.

**Deleted:** the comment block at `ROMA2Warp.cpp:324-327` explaining the per-draw grid as a feature
("a pair whose sparse matches already fill most of its budget asks for few dense points and gets a
coarse grid") — the fixed pitch is the mechanism now and the reasoning is replaced, not amended.

**A consequence to record, not an aim:** `WarpCellLatticePriority` exists because two pairs sharing
image A stratified it on grids of *different* pitch and phase, so their A-side samples landed a few
cells apart and the 0.1 px dedup saw two keypoints where one surface point was sampled twice. A
pitch that no longer depends on the pair removes that difference at the source: two pairs sharing A
now agree on the buckets, so they agree on the winners, so their A-side keypoints are the same
pixel and `FilterRedundantKeypoints` chains them. Longer tracks are measured (section 6), not
assumed, and the lattice priority stays — it is what makes two pairs with *different overlaps* on A
still agree within a shared bucket.

### 3.2 The dense observation weight (`libs/SFM/BundleAdjustment.h/.cpp`, `libs/SFM/Track.h/.cpp`)

**`BAConfig::denseObservationWeight` becomes an override, default `-1` = estimate.** A negative
value means "measure it on the scene this solve is about to fit"; a value in `[0, 1]` pins it, which
is what a sweep or a regression test uses. `--ba-dense-weight` default becomes `-1` and its help
says so.

New exported function, next to the reprojection helpers it reuses (`Track.h`):

```cpp
// Robust reprojection sigma of the two observation populations of the current solution, in pixels:
// the MEDIAN reprojection error over observations on described keypoints and over those on dense
// (warp-sampled) ones, each computed with ComputeReprojectionErrorPixels -- the same formula
// FilterTracks uses. The median of an already non-negative error is used as the scale directly,
// with no 1.4826 consistency factor: the only consumer takes a ratio of the two, in which any
// common factor cancels. An observation whose projection is invalid is skipped by both counts.
// Counts are returned so the caller can refuse a sample too small to be a sigma.
SFM_API void ComputeObservationSigmas(const Scene& scene,
    double& sigmaDescribed, size_t& numDescribed, double& sigmaDense, size_t& numDense);
```

and, in `BundleAdjustment.cpp`, the weight it feeds:

```cpp
// The weight a dense reprojection residual carries relative to a described one, measured rather
// than configured: 1/k^2 for k = sigma_dense/sigma_described, the ratio of the two populations'
// robust reprojection sigmas on the scene as it stands. Both sigmas are read off the RAW pixel
// residuals, so the estimate does not carry the weighting the previous solve ran under; it is
// recomputed at the head of every solve, and a reconstruction runs 50+ of them, so it settles.
// Falls back to the configured constant when either population is too small to give a sigma
// (fewer than MIN_SIGMA_OBSERVATIONS = 100) or when there are no dense keypoints at all, and is
// clamped to [0.01, 1]: a dense match is never worth more than a described one, and never worth
// nothing -- on a capture where the descriptor matcher is very good the ratio can run away, and a
// weight of zero would discard the only evidence a textureless region has.
double EstimateDenseObservationWeight(const Scene& scene, const BAConfig& config);
```

Both `BundleAdjustment::Adjust` and the local bundle adjustment resolve the effective weight once,
at the head of the routine, before the problem is built:

```cpp
const double denseWeight = config.denseObservationWeight >= 0.0 ?
    config.denseObservationWeight : EstimateDenseObservationWeight(scene, config);
```

and pass it to `SelectReprojectionLoss` in place of `config.denseObservationWeight`. The estimator
runs over the whole scene in both cases — a local window's described population is often too small
to give a sigma, and two different weights inside one reconstruction would be worse than a slightly
stale one. The existing DEBUG line reporting how many residuals are dense reports the effective
weight and, when it was estimated, the two sigmas it came from.

`SelectReprojectionLoss` keeps its signature except that the weight arrives as a parameter rather
than being read from the config, and its exclusivity with `useKeypointConfidence` is unchanged:
when that term is on it supersedes this one, and no estimation runs.

**Rewritten, not amended:** the PROVISIONAL comment block at `BundleAdjustment.cpp:531-538` and the
matching paragraphs of `BAConfig` — they describe how a value that was never measured was picked,
and there is no such value any more.

### 3.3 The view graph's dense discount (`libs/SFM/ImagePair.h`, `libs/SFM/PairsWeighting.h`, `apps/CreateStructure/CreateStructure.cpp`)

`ComputePairsWeights` scores a pair's evidence as `numFilteredInliers + w * numDenseInliers`, and
today `w` is the same number bundle adjustment uses (`CreateStructure.cpp:391`). It must stop
being: the two are different quantities, and section 2 makes the difference load-bearing. Bundle
adjustment's weight is `1/sigma^2`, a statement about how precisely a correspondence locates a
point. The view graph's is a statement about how much a correspondence says two images see the same
thing, and a warp correspondence says nearly as much as a descriptor one however coarsely it is
localized. Letting the estimated weight reach the view graph would tell it, on a capture like
Truck, that a dense-only pair carrying 200 correspondences is worth 12 — demoting precisely the
pairs that take 38004114 from 0 registered images to 248.

So: `CreateStructure.cpp:391` is deleted, `PairsWeightingConfig::denseObservationWeight` keeps
`DENSE_OBSERVATION_WEIGHT = 0.25` as its own constant and its own default (unchanged, which also
holds the view graph fixed across the measurement of section 6), and it is settable through the
library and Python API but has no CLI flag — no measurement asks for one. `DENSE_OBSERVATION_WEIGHT`
stays in `ImagePair.h` as the view graph's constant and as bundle adjustment's fallback, and the
comments that assert the two are one quantity are rewritten to say why there are two:
`ImagePair.h:116-124` (the `weightedInliers` paragraph), the include comment at
`PairsWeighting.h:14`, and `BundleAdjustment.h:63-70`.

### 3.5 Documentation

Four places state the old rules and are rewritten to the new ones, not amended alongside them:
`libs/SFM/README.md:188` (the `--roma2-match` paragraph's "up to `--roma2-dense-matches` (2000)
warp correspondences drawn where the sparse matches are not"), and in
`docs/design/ROMA2InProcess.md` the flag tables at `:39` and `:370` ("dense fill cap per pair") and
the latency paragraph at `:521` ("runs at the `--roma2-dense-matches` cap (2000) on essentially
every admitted pair"), which the measurement of section 6 replaces with what it finds. The
`ROMA2InProcess.md` sections on the fill and on bundle adjustment gain the density rule, the
estimated weight and the reason the view graph keeps its own constant.

### 3.4 Per-pair log record

The record gains the two numbers that now decide the draw, between the verdict areas and the
counts, so a run's own log remains sufficient to replay the rule that produced it:

```
ROMA2 pair 0-1: conf 0.9523 0.9658 inl 0.9393 0.8929 ADMIT cap 1786 grid 45 guided 749 sparse 736 dense 1642 301ms
```

`dense_cap_predict.py` and `task6lib.py`'s one-pass parsers are updated to the new field list.

## 4. Deleted code (exhaustive)

Deleted outright:

- `ROMA2Warp.cpp:324-327` — the per-draw grid rationale, superseded by the fixed pitch.
- `BundleAdjustment.cpp:531-538` — the PROVISIONAL default's derivation. There is no such value.
- `CreateStructure.cpp:391` — `cfg.matchCfg.weightingCfg.denseObservationWeight = OPT::baDenseWeight`.
- The `WarpBucketGridSide` call in `SampleWarpComplementary` (the function itself stays, as
  `SampleWarpByCoverage`'s).
- Every use of the name `ROMA2Config::denseMatches`, including `PythonWrapper.cpp:313`'s
  `dense_matches` binding, which takes the new field's name.

Rewritten so that no text claims the superseded rule alongside the new one — the passages naming
the bundle adjustment weight and the view-graph discount as one quantity (`BundleAdjustment.h:63-70`,
`ImagePair.h:116-124`, the include comment at `PairsWeighting.h:14`), and the four documentation
sites of section 3.5.

## 5. Tests (`apps/Tests/TestsSFM.cpp`)

1. `DenseFillGridSideTest` — the pitch is a pure function of the density (2000 -> 45), clamps to the
   warp side, and never falls below 1.
2. `DenseFillDensityTest` — one synthetic warp, two verdicts differing only in inlier area: the draw
   scales with the area rather than staying at the cap, and neither draw exceeds `DenseFillCeiling`.
3. `DenseFillCoverageTest` — the same warp and verdict drawn three times, with guided matches
   covering none, half and all of the overlap: the yield falls roughly with the uncovered fraction
   and reaches zero when the overlap is covered.
4. `DenseFillCeilingTest` — a verdict with `inlierAreaB << inlierAreaA` is capped by B's area.
5. `DenseFillCrossPairChainTest` — two pairs sharing image A, over a common region of A, draw A-side
   points at identical positions (the fixed pitch's chaining property, which is what turns two
   length-2 tracks into one length-3 track downstream).
6. `DenseObservationWeightEstimateTest` — a synthetic scene whose dense observations carry a known
   k times the described ones' reprojection error: the estimator returns ~1/k^2, clamps at both
   ends, and falls back to the configured constant when either population is under 100 observations
   or the scene has no dense keypoints.

Both existing suites must stay green, and the build warning-free.

## 6. Measurement plan (after the code is green)

Three captures, the campaign's own: 8d2f4877 (225 images, LiDAR interior), 38004114 (311,
textureless interior, the capture the descriptor pipeline cannot reconstruct at all) and
Tanks and Temples / Truck (251, well-textured outdoor, the capture the dense fill hurts). Two new
arms each, against the recorded 2026-09-03 `onepass` runs:

- `openmvs-roma2-20260904-capdensity` — section 3.1 only, `--ba-dense-weight 0.25` pinned.
- `openmvs-roma2-20260904-capdensity-autoweight` — 3.1 and 3.2 together.

Pinning the weight in the first arm is what separates the two changes; the view graph is held fixed
by 3.3 in both.

Read out with the existing instruments (`pair_eval.py`, `pose_eval.py`, `arm_summary.py`,
`onepass_compare.py`, `pose_compare.py`): registered images and components, stored pairs, tracks and
observations, **mean and median track length split by keypoint kind** (new — this is where 3.1's
chaining shows), full/local bundle adjustment counts and wall, total wall and peak RSS, and the
per-pair correctness panel — passRate, sparse and dense GT-inlier fractions, joint-fit fractions,
two-view pose errors, and **overlap fill**, which is what 3.1 could cost and the number to watch on
38004114 (0.9279 today).

The result this design is accountable for: Truck's pose accuracy moving toward `sift`'s without
38004114 losing registered images. If the first arm alone achieves it, the second is a refinement;
if neither does, the finding is that the dense fill's problem on well-textured captures is not its
budget, and that belongs in the report as plainly as a success would.

Runs go to `<capture>/openmvs-roma2-20260904-<arm>/` with their logs, per the storage rule; the
per-capture comparison notes and `PAIR-CORRECTNESS-REPORT.md` gain the new arms.

## 7. Defaults and what they mean

| knob | default | meaning |
|---|---|---|
| `--roma2-dense-matches` | 2000 | dense correspondences per full frame of UNCOVERED overlap; a pair's draw is that density over the part of its overlap its guided matches did not cover, capped by the smaller of the verdict's two inlier areas |
| `--ba-dense-weight` | -1 | estimate `(sigma_described/sigma_dense)^2` at the head of every solve; a value in [0,1] pins it |
| `PairsWeightingConfig::denseObservationWeight` | 0.25 | the connectivity evidence one dense match carries in the view graph — a separate quantity from the one above, no CLI flag |
| `--roma2-match` | off | unchanged: the dense pass is for captures the descriptor matcher cannot handle |
