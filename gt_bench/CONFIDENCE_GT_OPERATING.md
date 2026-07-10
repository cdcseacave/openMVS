# Adaptive confidence — GT operating-frontier eval (contamination vs completeness), 2026-07-10

Closes the gap that the two GT datasets (ETH3D + BlendedMVS) were integrated to close: grading the
adaptive per-view confidence (`DepthMapsData::AdjustConfidence`) on the **same two surface-GT metrics
as fusion** — outlier **contamination** and **completeness** — instead of only ROC-AUC. ROC is a
threshold-free ranking score; it never told us what a confidence-*thresholding* consumer (TSDF, any
"keep depth where conf ≥ t" stage) actually gets.

## What is measured, and why it is a fair raw-vs-adjusted comparison

`AdjustConfidence` rewrites only the confidence channel; the depth is untouched. So raw (pre-adjust,
`raw_dmaps/`) and adjusted confidence sit on the **identical** set of GT-labelled pixels — same
inliers, same outliers, only the confidence *values* (hence the ranking) differ. A pixel is a
GT-inlier iff `|d_est − d_gt|/d_gt ≤ 1%` (EvalConfidence.py GT mode, `GtUtils.gt_labels`).

Treat the confidence as a retention score: keep pixel iff `conf ≥ t`. Among kept pixels,
- **contamination** = kept_outliers / kept_total (= 1 − precision),
- **completeness** = kept_inliers / all_inliers (= recall).

Sweeping `t` traces a frontier (`EvalConfidence.operating_points`, evaluated at distinct-confidence
group boundaries so tied confidences are kept/dropped together). We read off two duals:
- **completeness kept at a fixed contamination budget** — the per-depth-map analogue of fusion's
  "completeness at a gross-outlier budget";
- **contamination admitted at a fixed completeness target**.

Both are invariant to any monotone rescaling of the confidence, so raw and adjusted compare **fairly
without picking a shared threshold** — the naive "compare P@0.1 of each" is unfair because
recalibration shifts the numeric scale (the old fusion gate t=0.1 keeps ~everything: R@0.1≈1.0 on
every view, so the single-point P/R was blind to the operating region that matters).

Reproduce: `bash gt_bench/run_confop.sh` (raw+adjusted × 28 scene-levels → `results_confop/`) then
`python gt_bench/aggregate_confop.py`. Uses the already-estimated dmaps; no re-estimation.

## The 2×2 is now complete

Both branch features, both GT metrics (✅ = measured here / previously):

| | outlier contamination | completeness |
|---|---|---|
| **fusion** (`fFusePriorWeight`) | ✅ gross-outlier% (`FINAL_2026-07.md` [2], `W2_W3_RECHECK.md`) | ✅ completeness@tol |
| **adaptive confidence** (`AdjustConfidence`) | ✅ contamination@completeness (below) | ✅ completeness@contamination (below) |

*(The two rows are not in identical units — fusion = 3D surface coverage at metric/diagonal
tolerances; confidence = per-depth-map pixel recall at a contamination fraction. Same two axes, same
spirit; magnitudes are not cross-row comparable, as with all cross-dataset tolerances in this bench.)*

## Aggregate result — all 28 deterministic scene-levels

**Completeness kept at a fixed contamination budget** (higher = better; adjusted should win):

| contamination ≤ | raw | adjusted | mean Δ | scenes adj ≥ raw |
|---|---|---|---|---|
| 0.5% | 19.8% | **40.5%** | **+21.9pp** | 27/27 |
| 1%   | 31.0% | **51.3%** | **+20.4pp** | 27/28 |
| 2%   | 46.8% | **65.5%** | **+18.8pp** | 27/28 |
| 5%   | 66.3% | **80.6%** | **+14.3pp** | 19/28 |

**Contamination admitted at a fixed completeness target** (lower = better):

| completeness ≥ | raw | adjusted | mean Δ | scenes adj ≤ raw |
|---|---|---|---|---|
| 90% | 10.3% | **8.4%** | **−1.9pp** | 25/28 |
| 95% | 12.7% | 13.8% | +1.1pp (worse) | 18/28 |
| 99% | 15.7% | 16.1% | +0.5pp (worse) | 8/28 |

ROC-AUC (macro-avg of scene-level pooled ROC, for continuity only — this is *not* the Task-17
pixel-pooled 0.9463→0.9598 micro-average): 0.844 → 0.903.

Per-dataset the direction is identical (ETH3D 18 levels: completeness@≤1% 18.6%→40.3%, +21.7pp, 17/18;
BlendedMVS 10 levels: 53.2%→71.2%, +18.0pp, 10/10).

## Findings (honest)

**1. Decisive win where it matters — clean-subset selection.** In the tight-contamination region a
quality-conscious consumer actually uses, the recalibration keeps **~20pp more inlier surface at the
same outlier budget**, near-universally (27/28 at ≤1%; the lone regressor is courtyard-L1 at −0.5pp).
This is the direct completeness-vs-outlier answer the user asked for, and it is strongly positive.

**2. Raw confidence is *unusable for thresholding* on hard scenes; adjusted makes it usable.** On 6
scene-groups (ETH3D office, meadow, pipes, facade-L2/L3; and it is marginal on courtyard-L3) raw
completeness@≤1%-contam = **0.0%** — the raw map's highest confidences are already >1% contaminated,
so *no* clean subset exists at any threshold. Recalibration fixes this qualitatively:
office-L1 0.0→40.4%, pipes-L1 0.0→43.0%, facade-L2 0.0→78.7%, courtyard-L3 0.2→41.9%. Turning a
confidence map that cannot gate at all into one that can is the single strongest argument for the
feature.

**3. A real tradeoff at the high-completeness tail.** To retain ≥95% of inliers, adjusted admits
slightly *more* contamination on average (+1.1pp), concentrated on textureless / low-signal scenes:
office (−7.8/−11.0/−8.8pp across L1/L2/L3), meadow-L1/L2 (−2.9/−3.7pp), pipes-L1/L2 (−3.2/−3.9pp),
and the 375-view aerial `bmvs_5b7a3890` (−6.0pp). The recalibration sharpens the *top* of the ranking
(the clean-subset gains above) at the cost of marginally reordering the *bottom*. This tail is not the
TSDF operating point — and on office both raw (35%) and adjusted (46%) contamination-to-keep-95% are
garbage anyway (the scene is just hard near-saturation); the usable office operating point is the
tight end, where adjusted goes 0→40%. ROC *improved* on every one of these scenes, so a single AUC
would have hidden this tail entirely — the frontier is what exposes it.

**4. One scene regresses in a way worth noting:** `bmvs_5b7a3890` (aerial, hardest, ROC 0.82→0.90)
gains almost nothing at the tight end (compl@1% 0→0.8%) and loses 6pp at the high-completeness end.
Not a blocker (it is one pathological scene and the feature is opt-in), but recorded.

## Verdict

The adaptive confidence is **validated on both GT metrics**, not just ROC: it delivers a large,
near-universal completeness gain at fixed low outlier contamination (the clean-subset / TSDF regime),
and it rescues thresholdability on the hardest scenes where the raw NCC-confidence is unusable. The
only cost is a modest contamination increase at the ≥95%-completeness tail on textureless scenes —
outside the regime the feature is meant to serve. This matches the branch goal ("better differentiate
accurate inliers from noisy outliers") measured, at last, in the operational units that goal implies.
