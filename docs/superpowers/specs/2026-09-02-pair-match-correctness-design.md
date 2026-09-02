# Pair-level match correctness — design

Date: 2026-09-02
Branch: `feature/roma2-onnx`
Campaign: roma2-matching-redesign

## Why

The end-to-end arms answered "how many pairs and how good is the model", never "is a pair's match
set correct". The gate-floor sweep on the LiDAR interior 5828945d made that gap load-bearing:
lowering `--roma2-min-inlier-coverage` from 0.25 to 0.10 doubled the matched pairs (935 -> 1948) and
made the reconstruction *worse* (rotation error 1.19 deg -> 2.04 deg). Pairs admitted below the floor
carry guided-SIFT and dense content that is not explained by the pair's true relative pose. Nothing
in the pipeline measures that, so nothing can be tuned against it.

This design adds the missing measurement: for a matched pair, are the dense matches that fill an
under-matched SIFT pair correct, well spread over the true overlap, and explained by one relative
pose together with the sparse matches — and are all good pairs matched and passing.

## Scope

Measurement only. No pipeline change. The fixes it motivates are a separate design.

## Datasets

| key | capture | GT | notes |
|---|---|---|---|
| truck | `~/virginia/datasets/tnt/training/Truck` | COLMAP `sparse` (251 imgs, 128k points) | easy outdoor baseline, no depth |
| lidar8d2f | `~/virginia/datasets/polycam/lidar/8d2f4877-…` | GlueMap oracle `bench_base_aug19/gluemap_aba` (225 imgs, RADIAL) + LiDAR depth 640x480 mm | easy indoor |
| normal38 | `~/virginia/datasets/polycam/normal/400/38004114-…` | `oracle/frames.npz` poses + depth samples, 11 segments | problematic indoor (10 ARKit breaks) |

Arms per dataset: `sift`, `roma2`, `roma2-supp` (Truck also has the existing task6 runs; the LiDAR
gate probes `roma2-cov010` / `roma2-cov000` are evaluated as the sweep that motivated the work).

## 1. Export (C++)

`SceneAnalyzeSFM` gains the per-match dump. It already loads a saved project read-only and writes
CSVs to `--out`; the pair evidence it omits is exactly what the test needs.

* `matches.bin` — 16-byte header (`OMVSMTC1`, uint32 version, uint32 rows), then rows of
  `float32 x1,y1,x2,y2; uint32 idx1,idx2` (keypoint pixel coordinates in each image plus the two
  keypoint indices). One contiguous block per pair, in `pairs.csv` row order, holding that pair's
  `matches` array followed by its `outlierMatches` array. The three-way partition of `matches`
  (sparse / dense / loose) falls out of the counts already in the row.
* `pairs.csv` gains `numLooseInliers`, `numOutlierMatches`, `matchOffset`, `overlapRatio`,
  `overlapArea`, `meanRayAngle`, `hasPose`, `R00..R22`, `tx,ty,tz` (the stored relative pose in its
  own convention: `x2 = R*x1 + t`, `t = Pose3D::GetT()`).
* `images.csv` gains `width,height,ppx,ppy` so the metrics never guess a resolution.

Input is the run's `scene_pre_reconstruction.sfm` — the state after matching, before reconstruction,
which keeps every matched pair.

## 2. Ground truth (python)

`pairgt.py` in `~/virginia/datasets/openmvs-roma2-20260901-task6-tools/`, one loader per dataset
behind one interface: per image name, `K` (+ radial distortion), world-to-camera `R`, centre `C`,
size, segment id, and a depth sampler where depth exists.

* truck: pycolmap on `sparse`. No depth: match error is the Sampson distance under the GT relative
  pose; covisibility comes from shared `points3D`.
* lidar8d2f: oracle poses (RADIAL distortion applied on projection) with the LiDAR depth converted to
  oracle units by one robust scale factor `s`. `s` is fitted from the oracle's **own structure** —
  the median ratio of each triangulated point's z-depth in a camera to the LiDAR depth at that pixel
  — not from the ARKit trajectory: ARKit gives 0.3063 units/m with an 11 % relative MAD (its drift is
  exactly what the oracle exists to correct), the structure fit gives 0.3140 with 4.6 %, and that
  4.6 % is the honest accuracy of this ground truth.
* normal38: `oracle/frames.npz` (`R_wc` camera-to-world, centre `C`, `K_depth`, per-frame depth
  samples) plus `keyframes/depth/<ts>.Clean.png`. Metric and self-consistent, so reprojection error
  directly. Poses are consistent only inside a `segment`; a cross-segment pair has **no GT** and is
  reported as `unknown`, never as pass or fail.

Overlap is an 8x8 grid per image: a cell is in the overlap when at least half its GT depth samples
project inside the other image and are not occluded there (truck: the cell holds a `points3D`
observation also seen by the other image). A pair is **good** when the overlap covers >= 15 % of both
frames (truck: and >= 20 shared tracks). Every result is also bucketed by overlap, so the threshold
is never load-bearing.

## 3. Metrics (per pair)

1. **GT consistency** — a match is correct when it lies on the GT epipolar geometry *and*, wherever
   the GT depth can say so, in the right place along it:

       sampson <= tau   AND   (no depth  OR  reprojection <= tau + sigma * parallax)

   with tau = 4 px — the pipeline's own `maxEpipolarError`, so "wrong" means wrong by the standard of
   the filter that kept the match — sigma = 0.35, and *parallax* the
   match's own translational parallax — how far translation alone moved the point, measured against
   the same ray at infinity. The depth term is not a fixed pixel budget because a depth error is
   relative: it shows up in pixels as a fraction of the parallax, so a fixed tolerance would fail
   every correct wide-baseline match.

   **sigma is calibrated, not assumed.** Over 20176 plain-SIFT matches on 8d2f4877 that sit within
   tau of their GT epipolar line — matches that are certainly correct — the ratio
   reprojection/parallax has p50 0.07, p90 0.26, p95 0.36; restricting to locally smooth depth does
   not move it (93 % of matches are already on smooth depth, so this is the depth's accuracy, not a
   discontinuity artefact). sigma = 0.35 is that p95. The consequence has to be stated plainly: on
   this LiDAR the depth half is a **coarse** two-dimensional check that catches a match sliding far
   along its epipolar line and nothing finer. The sharp half is the epipolar one — 0.3-0.5 px on
   correct matches at every keyframe gap — and it is reported on its own as `*EpiFrac` so no headline
   ever rests on the coarse half alone. Reported per segment: GT-inlier fraction (both halves),
   epipolar-only fraction, Sampson median and p90, reprojection median, parallax median, depth
   coverage.

   Where there is no depth at all (Truck) the two-view epipolar test is the only one available, and a
   match displaced along its own epipolar line passes it. The reported counterweight is
   **triangulated distance**: the matches are triangulated under the GT pose and their median
   distance to the GT point cloud is reported in units of the cloud's own nearest-neighbour spacing.
2. **Pose** — stored relative pose against GT: rotation angle and translation-direction angle.
3. **One-pose explanation (GT-free)** — one essential matrix fitted by RANSAC over sparse ∪ dense at
   the pipeline's own epipolar threshold; the inlier fraction of each segment under that single
   model, and that model's own error against GT.
4. **Distribution** — overlap cells covered by GT-inlier sparse matches, by dense matches, and by
   both; `fill = covered / overlap` per image, plus the largest single-cell share as a clustering
   indicator.
5. **Under-matched flag** — from the pipeline's own record (supplemented, sparse count, SIFT cell
   coverage), so "did the filling help the pairs it was meant to help" is answerable.

**Pass rule (per pair):** sparse and dense GT-inlier fractions (rule 1 above) >= 0.9; joint-fit inlier fractions
>= 0.9; stored pose within 1 deg / 5 deg of GT; and, for supplemented pairs, `fill >= 0.6` on both
images. Thresholds are named constants at the top of the script.

## 4. Completeness

Over the GT-good pairs, a funnel per gap bucket and per overlap bucket: retrieval candidate (from the
gate records in the run log) -> gate-validated -> guided kept -> present in the scene -> passing.
Precision is the share of kept pairs whose GT overlap is under 5 %.

## 5. Outputs

Per run, under `<run>/analysis/`: `pair_eval.csv` (one row per pair), `pair_eval.json` (summary),
`pair_eval.md` (tables by segment, gap, overlap, under-matched status, the funnel, pass rates, and
the 30 worst pairs). Never in the repo tree.

## 6. Testing

* A C++ check that the export round-trips a scene: counts in `pairs.csv` sum to the block length in
  `matches.bin`, and the exported coordinates equal the keypoints they index.
* A python self-test on synthetic cameras, points and a synthetic depth image: exact matches score
  100 %, perturbed matches are caught, the grid marks the right overlap cells.
* A GT sanity gate before any number is trusted: on Truck's `sift` arm the sparse matches must reach
  >= 0.95 median GT-inlier fraction. If they do not, the GT convention is wrong and is fixed first.
  The same gate runs per dataset on its sparse matches. **Result:** Truck/sift scores a median sparse
  GT-inlier fraction of 1.000, a joint-fit inlier fraction of 0.999 and a stored relative pose 0.11
  deg / 0.35 deg from GT. On 8d2f4877 the first gate run failed (0.12) and was traced to the
  tolerance, not the conventions: the pose fitted from that arm's own matches agrees with the oracle
  to 0.11-0.25 deg at every gap, and the oracle reprojects its own points through the reader at 0.83
  px, so the GT is sound and the fixed pixel tolerance was not. That is the measurement that produced
  the rule above.

## Out of scope

Fixing the filling. Retrieval changes. Anything that writes into a capture's `keyframes/`.
