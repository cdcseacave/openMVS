# Two-View Verification, Lens Fit and Focal Calibration

## Purpose and scope

Every candidate pair of images (`PairSelection.md`) is matched and then verified geometrically: a robust estimate of
the relative pose (or fundamental matrix) separates inlier matches from outliers, and the surviving matches become the
pair's evidence for everything downstream. That verification depends on the cameras. A camera whose focal is a guess and
whose lens is unmodelled makes every pair it belongs to come out with a wrong pose, so after all pairs are verified one
stage fixes the cameras and verifies again the pairs it changed:

1. `GeometricFilter`: the per-pair verification (RANSAC, the E-or-F choice, the strict match filter).
2. `CalibrateFocalsAndFitLenses` (`LensFit.h/cpp`): the radial lens (k1, k2) of every pinhole camera that has none, with
   the focal check that corrects a focal the pairs clearly reject (`FitLensesAndReverifyPairs`).
3. `ViewGraphCalibrator` (`ViewGraphCalibrator.h/cpp`): the focal of the cameras the lens fit could not take, from the
   whole graph of fundamental matrices.

The stage runs in `Scene::MatchPairs` after matching, before the planar twin switch (`DenseMatching.md`), the pairs
export, the triplet scores (`TripletScores.md`) and every reconstruction stage. Spherical cameras take part only in the
first step (see Spherical branch). Where the intrinsics come from (EXIF, forced values, trusted or not) is in
`ImportAndCameras.md`.

## Per-pair verification (`PairsMatcher::GeometricFilter`)

A pair needs at least 8 matches. The estimator is chosen once per pair by `SelectGeometryBranch`:

| branch | chosen when | estimator | result |
|---|---|---|---|
| ESSENTIAL | both cameras trust their intrinsics (and `forceFundamental` is off) | `poselib::estimate_relative_pose_bearings` on unit bearings (5-point RANSAC with cheirality) | relative pose, E, and F when both cameras are pinhole |
| FUNDAMENTAL | anything else | `poselib::estimate_fundamental` (7-point RANSAC) | F; the pose only if both cameras trust their intrinsics or `forceFundamentalDecomposition` is set |
| SHARED_FOCAL | `forceFundamentalWithFocal` and one shared pinhole camera | `poselib::estimate_shared_focal_relative_pose` with the real-focal check | pose at the estimated focal |

`forceFundamental`, `forceFundamentalWithFocal` and `forceFundamentalDecomposition` are `MatchConfig` fields with no
command-line option; the default flow uses ESSENTIAL for trusted cameras and FUNDAMENTAL for the rest.

- **RANSAC**: 100 to 10000 iterations; `max_error` is `MatchConfig::maxEpipolarError` (4 px, 0 turns verification off
  and every match is an inlier). The ESSENTIAL branch converts the pixel bar to an angle per camera
  (`Camera::PixelErrorToAngular`) and averages the two, the unit of the unit-norm Sampson residual on the sphere. A pair
  that already carries a pose or an F seeds RANSAC with it.
- **Acceptance**: the number of inliers must reach `minMatches` (50 for AKAZE/ORB, 15 for SIFT and SIFTGPU, from
  `DefaultsForFeatureType`); otherwise the pair's matches are invalidated.
- **FUNDAMENTAL to pose** (`DecomposeFundamentalToPose`): E = K2^T F K1, then `ImagePair::RecoverPose` takes, of the four
  factorizations of E, the one that puts the most correspondences in front of both cameras (the correspondences are
  unit bearings, each unprojected by its own camera). Fewer than `minMatches` in front invalidates the pair. If the
  intrinsics are inaccurate the decomposition keeps very few inliers.
- **Strict filter** (`ImagePair::FilterMatches`, on when `IsMatchesFilterOn()`): drops inliers by cheirality, by
  triangulation angle (`minTriangulationAngle`, 0.5 deg), by reprojection error (`reprojThreshold`, 6 px) and by
  epipole proximity (`epipoleFilterThreshold`, 0 = off), and measures the pair's median ray angle. Fewer than
  `minMatches` survivors invalidate the pair. The survivors are the sparse (descriptor) segment of `matches`.
- **F is composed** from E and the two K only when both cameras are pinhole; consumers branch on `pair.F.has_value()`
  and otherwise use E on bearings.
- `RefineRelativePose` polishes a calibrated pose over weighted matches with the estimator's own bearing refinement
  (used by the dense pair assembly, `DenseMatching.md`); it acts only on the ESSENTIAL branch.

`PairsMatcher::ComputeRelativePoses` verifies pairs again after a camera changes: all pairs, or those of a given set of
cameras, optionally only those whose cameras trust their intrinsics or only those missing a pose. A pair from the dense
one-pass flow is re-verified by `ReverifyPairROMA2` (sparse and dense segments kept apart), every other pair by
`GeometricFilter` with its outliers rejoining the candidates. The pair weights are recomputed afterwards.

## Stage order (`CalibrateFocalsAndFitLenses`)

Before this stage, a camera whose focal is an estimate (an EXIF 35 mm-equivalent focal or the default ratio of the
long side) does not trust its intrinsics, so its pairs were verified through F. Then:

1. Every untrusted pinhole camera with no lens given (`HasDistortion` and `HasAdditionalDistortion` both false) is made
   trusted at its import focal, and each of its pairs is posed from its F there (`PosePairsFromF`: E = K2^T F K1, the
   pose that puts the most sparse inliers in front of both views; a pair with fewer than 5 sparse inliers is left
   as it is). This only runs when `MatchConfig::lensFitEnabled` (default on, no command-line flag).
2. `FitLensesAndReverifyPairs` fits the lens of every trusted pinhole camera that has none, checks the focals, and
   verifies again every pair of a camera whose lens was fitted or whose focal moved (`ComputeRelativePoses`). A camera
   from step 1 whose focal the fit did not take goes back to untrusted, with no lens and its pairs back to their F.
3. If `MatchConfig::viewGraphCalibrationEnabled` (default on) and any pinhole camera is still untrusted, the
   `ViewGraphCalibrator` estimates its focal; the pairs of the cameras it updated are verified again through E.
   Cameras the lens fit cannot take: too few eligible pairs, a lens given, an unsolved or folding fit, the lens fit off.
   The calibration runs after the lens fit because it reads an uncorrected lens as a wrong focal.

A scene resumed already matched carries the lenses it was fitted, which makes them given: those cameras are not fitted
again, a camera that still has no lens is.

## Lens fit (`FitLensesFromPairs`)

The model is the radial lens of `PinholeCamera` that OpenCV and COLMAP's OPENCV share: `x_d = x_u (1 + k1 r^2 + k2 r^4)`
on coordinates normalized by the focal around the principal point. The principal point is held, and so is the focal
unless the focal check moves it. A strong lens (corners moved by tens of pixels) turns pairs that barely move into
poses that are degrees short with reversed baselines, and every reader of the pairs inherits that.

**Which cameras.** A pinhole camera that trusts its focal and has no lens is fitted. A camera whose lens is given as k1,
k2 alone, one to one past the image corner, is held, and its pairs inform the others. Anything else (spherical, focal not
trusted, a given lens with more terms, or one that folds within the frame) is excluded together with its pairs, since
held at a wrong lens it would bias every camera it pairs with.

**Input.** Every verified pair with a pose whose rotation is between `lensFitMinRotation` (5 deg) and
`lensFitMaxRotation` (45 deg) and which holds at least `lensFitMatchesPerPair` (200) sparse inliers, between two fitted
or held cameras, heaviest first by composite weight until every fitted camera has `lensFitMaxPairs` (150). From each,
200 sparse inliers are drawn by a generator seeded from the pair's IDs. A fitted camera with fewer than
`lensFitMinPairs` (20) eligible pairs leaves the fit with its pairs.

**Loss.** The first-order distance of each match's two measured, distorted pixels to the pair's epipolar constraint, each
side through its own lens and focal, under a soft L1 at `lensFitLossScale` (1 px). Each pair's pose (5 unknowns) is
eliminated in the joint fit, leaving 2 unknowns per fitted camera.

**Order of one fit.**
1. The poses alone refined at the starting lenses; their median Sampson error is the baseline.
2. Lenses and poses together from the stored poses.
3. Each pair's pose re-estimated by RANSAC through the fitted lens (inlier bar 1 px) and kept where it explains the pair
   better, then the joint fit again, until a round replaces no pose, for at most `lensFitRounds` (10).
4. A check on the fit: a fitted camera passes when at least 85% of its pairs keep the fit's median error within a ratio
   of 1.5 of the median they reach refined alone from the fit (pose and a private copy of the lenses free). A fit that
   fails is started again from its lenses doubled, the poses re-estimated through that start first, and the fit of least
   cost is kept.
5. A lens is kept when it stays one to one past the image corner (with a 5% margin) and its median error falls
   `lensFitMinErrorDrop` (10%) below the baseline; a fit the solver cannot complete keeps none. A refused camera keeps
   no lens, and the others are fitted again without its pairs. The outcome per camera is one of EXCLUDED, HELD,
   TOO_FEW_PAIRS, FOLDS, NO_ERROR_DROP, FITTED. A median error under 0.01 px means there is nothing for a lens to explain
   and none is kept.

**Focal check.** A focal the pairs clearly reject (an import guess, a forced focal off, a calibration's) is corrected.
After the fit with every focal held, the joint fit and its rounds run again with every fitted camera's focal free beside
its lens (a log scale of the focal), with the same check. A camera takes the free fit's focal when its median error falls
`lensFitFocalMinErrorDrop` (10%) below the held fit's, and the fit is built again at the moved focal from the free fit's
poses. Estimated focals (the cameras made trusted in step 1 above) are judged first and take the free fit's focal with no
bar on the error drop; the trusted ones are judged on a fit whose estimated focals already moved. `fx` and `fy` scale
alike.

The pairs of a camera whose lens was fitted or whose focal moved (kept lens or not) are verified again through them; the
log reports the inliers before and after and how many pairs that formed tracks were invalidated.

## View-graph calibration (`ViewGraphCalibrator`)

For the cameras still untrusted, the focal is estimated globally across all pairs with the Fetzer method (direct focal
length calibration from two views): each pair's F, decomposed with the principal points, gives two residuals on the
focals of its two cameras (one parameter when both images share a camera). A pair gives a residual only if it has an F,
at least 15 weighted inliers and a composite weight of at least `minPairWeight` (0.9, `PairWeight.md`), and both
cameras are pinhole. The problem is solved with Ceres under an arctan loss (`lossThreshold` 0.1), at most 100
iterations, trusted cameras held constant, focals bounded below by 1e-3. An estimate outside `minFocalRatio` (0.1) to
`maxFocalRatio` (10) times the starting focal is rejected as degenerate; an accepted one sets `fx = fy` and marks the camera
trusted. `CreateStructure` sets `maxTwoViewError` to 0, so the optional pair filtering by residual after the calibration
is off.

## Spherical branch

Spherical cameras (`SphericalCameras.md`) always trust their intrinsics, so their pairs take the ESSENTIAL branch on unit
bearings, with `F` absent. They are excluded from the lens fit (they have no radial lens model) and from the
view-graph calibration (only untrusted pinhole cameras are calibrated); a pair that mixes a spherical and a pinhole
camera is verified on bearings (ESSENTIAL branch, when the pinhole camera trusts its intrinsics) and its F is not composed.

## Parameters and defaults

| parameter | default | where |
|---|---|---|
| `maxEpipolarError` | 4 px | `MatchConfig`; 0 = no verification |
| `minMatches` | 50 AKAZE/ORB, 15 SIFT/SIFTGPU | `MatchConfig::DefaultsForFeatureType` |
| `minTriangulationAngle`, `reprojThreshold`, `epipoleFilterThreshold` | 0.5 deg, 6 px, 0 | `MatchConfig` strict filter |
| `lensFitEnabled`, `viewGraphCalibrationEnabled` | true, true | `MatchConfig` |
| `lensFitMaxPairs`, `lensFitMinPairs`, `lensFitMatchesPerPair` | 150, 20, 200 | constants of `MatchConfig` |
| `lensFitMinRotation`, `lensFitMaxRotation` | 5 deg, 45 deg | constants |
| `lensFitLossScale`, `lensFitRounds` | 1 px, 10 | constants |
| `lensFitMinErrorDrop`, `lensFitFocalMinErrorDrop` | 0.10, 0.10 | constants |
| `ViewGraphCalibratorConfig` | min pair weight 0.9, loss 0.1, 100 iterations, focal ratio 0.1 to 10 | `ViewGraphCalibrator.h` |

## Invariants

- Each pair is verified with the cameras as they are at that moment; after a camera changes, every pair of that camera is
  verified again before anything reads the poses.
- A camera is trusted only where its intrinsics are measured, forced or calibrated; the lens stage makes an estimated
  camera trusted temporarily and returns it to untrusted when the fit does not take its focal, with no lens set.
- Lens coefficients are kept only while the lens is one to one out to the image corner; its undistortion has one answer.
- A given lens (forced `--k1`/`--k2` or imported) is never fitted again.

## Validation

| quantity | value | basis |
|---|---|---|
| focal check bar (10% error drop) | an accurate focal's free fit lowers the median error by at most 0.7%; a wrong focal's by at least 16% (most by 32-71%) | eleven real captures |
| fit check (85% of pairs within ratio 1.5) | fits that reached their lens keep 0.975-1.000 of their pairs, fits held short of it (more than 10 px off) keep 0.27-0.73; the five test captures keep 0.967-1.000 | 100 synthetic scenes per lens test (barrel 164 and 20 px at the corner, pincushion 80 and 160 px) |
| `lensFitMinPairs` 20 | fits stop failing at 8 to 10 eligible pairs, 20 leaves a margin | prototype pairs |
| `lensFitRounds` 10 | up to 5 rounds were needed over 100 scenes of a 164 px lens at the corner | synthetic scenes |
| restart from the doubled lens | every fit held short reached its lens over 100 scenes of each lens test; restarts from fixed radial changes of -5, -10 and +5% leave the 160 px pincushion short in 14 of 35 scenes | synthetic scenes |

## Rejected alternatives

- A fixed focal from the EXIF 35 mm-equivalent: the tag describes the lens on the full sensor, which a cropped video
  frame does not use; the focal is fitted with the lens instead.
- An absolute bar on the share of pairs with a small median error as the fit check: at 0.25 px of noise a fit held short
  still keeps up to 0.92 of them; the check compares each pair against its own refinement instead.
- Calibrating the focal by the view graph before the lens fit: it reads the uncorrected lens as a wrong focal.

## Open items

- The pair filtering by residual in `ViewGraphCalibrator` (`maxTwoViewError`, default 2 px) is disabled by
  `CreateStructure`; library users who leave it on get it.
