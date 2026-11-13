# Bundle Adjustment

The bundle adjustment (BA) refines camera poses, 3D points and, at some levels, camera intrinsics
by minimising the reprojection error of the inlier tracks, together with a relative-pose residual
per verified pair and, when configured, GPS position priors. It is a Ceres problem built by
`BundleAdjustment` (`BundleAdjustment.h/cpp`, cost functors in `BundleAdjustmentCostFunctions.h`)
from the scene as it stands; there is one entry point per scope:

- `BundleAdjustment::Adjust`: the global adjustment of every posed image and every inlier track.
- `BundleAdjustment::AdjustLocal(viewIDs, fixedViewIDs)`: the views in `viewIDs` and the points
  they observe are refined; the views in `fixedViewIDs` contribute observations but stay constant;
  every other view and point is held out of the problem. Intrinsics are never refined.

Both are configured by `BAConfig`. Who calls which, and when, is owned by the callers: the star
initializer (`StarInitialization.md`), the resection (`Resection.md`: local and full cadence), the
merge (`HierarchicalSFM.md`) and the reconstruction's final adjustment (`SfMPipeline.md`).

## Parameters being refined

- Pose: 7 values per image `[qw, qx, qy, qz, Cx, Cy, Cz]` on a product manifold (unit quaternion,
  Euclidean camera centre in world coordinates). The rotation and position parts can be held
  separately (`refinePosesRotation`, `refinePosesPosition`).
- Point: the track position, 3 values.
- Intrinsics, pinhole cameras only, one 12-value block per camera shared by its images:
  `[fx, fy/fx, cx, cy, k1, k2, k3, p1, p2, k4, k5, k6]`. A spherical camera has no intrinsic block.

## Cost terms

### Reprojection

One residual per observation of an inlier track in a posed image (`Track::IsInlier`, the inlier
prefix), through a Huber loss at `BAConfig::robustThreshold` (2 px).

- Pinhole (`PinholeReprojectionErrorAnalytic`, hand-written Jacobians, checked against the
  auto-differentiated functor by `PinholeReprojectionJacobianTest`): the 2D difference in pixels
  between the projection through the radial (k1-k3, with the rational k4-k6), tangential and
  intrinsic model and the keypoint. A point at depth under 1e-8 gets a constant residual of 2 and a
  zero Jacobian.
- Spherical (`SphericalAngularReprojectionError`): the predicted unit ray projected on the tangent
  plane at the observed ray, scaled by `width / 2 pi` so the two residuals are in pixels; no depth
  test, since a spherical camera sees in every direction.

### The dense observation weight

A dense (warp-sampled, descriptor-less) keypoint localises a point several times less precisely
than a described one, so its reprojection residual carries a loss weight below 1 (a
`ceres::ScaledLoss`). The weight follows the keypoint, not the match that created it: a dense
match whose endpoint was replaced by a described keypoint has the described keypoint's full weight.
The weight of image `i`'s dense residuals is

    w_i = min( w * N / (N + n_i), maxDenseObservationWeight )

- `w`, the scene weight (`EstimateDenseObservationWeight`), is resolved at the head of every
  solve, local solves included (by the whole scene, so two windows of one reconstruction never use
  different weights):

      w = clamp( (sigma_described / sigma_dense)^2, 0.01, 1 )

  both sigmas the median raw-pixel reprojection error of the described and of the dense
  observation populations of the inlier tracks of the scene about to be fitted
  (`ComputeObservationSigmas`, `Track.h`), read off the raw residuals and not off the weighting of
  the previous solve. It is measured and not configured because `sigma_dense / sigma_described` is
  a property of the capture: the dense sigma stays near the warp's own sampling scale, while the
  described sigma moves several-fold between a well-textured outdoor capture and a textureless
  interior.
- `w` falls back to the constant `DENSE_OBSERVATION_WEIGHT` (0.25) when either population holds
  fewer than 100 observations or either sigma is 0 (an early incremental step, a scene that fits
  itself exactly). A scene with no dense keypoint at all is recognised before anything is walked and
  gets the constant too; its solves pay no whole-scene reprojection pass.
- `--ba-dense-weight` (`BAConfig::denseObservationWeight`, default -1 = measure) pins `w` when it is
  `>= 0`, unclamped; use [0,1], 1 turning the down-weighting off.
- `N` = `denseHalfWeightObservations` (300; 0 = the same weight for every image) and `n_i` is the
  number of described observations of image `i` in the inlier tracks of the whole scene: an image
  with hundreds of detections has its pose fixed by them, and the warp samples of its matchings,
  whose errors neighbouring samples share, can only pull it; an image with few detections keeps most
  of its dense say.
- `maxDenseObservationWeight` (0.01; 0 = no ceiling) bounds an image's dense weight. The weight the
  scenes measure (0.06 to 0.15) is above what the warp samples are worth against the detections, so
  nearly every image is held to the ceiling and only one rich in detections goes below it.
- `BAConfig::useKeypointConfidence` (off, no flag) weights an observation by keypoint response and
  size instead; it expresses the same statement by another route, so it supersedes the dense weight
  and the two never compose.

This weight is the bundle adjustment's own and is deliberately a different quantity from the view
graph's dense evidence (`PairsWeightingConfig::denseFrameInliers`, `PairWeight.md`): a warp
correspondence localises a point imprecisely, which is what this weight charges it for, but says
nearly as much as a descriptor correspondence about whether two images overlap, which is all the
view graph asks. Charging the precision penalty a second time there would demote the dense-only
pairs that carry a capture the descriptor matcher cannot match.

### The observation cap

What a solve costs is the number of residuals it fits, and one image can bring thousands: warp
samples where it was matched densely, detections where it was not. A few hundred spread over the
frame already fix its pose, so each image contributes at most `maxObservationsPerImage` (1500,
`--ba-max-obs`, 0 = no cap) observations of any kind to a solve (`BuildObservationCap`).

- The budget runs only once the scene's observations (every inlier track's observations in every
  posed image, summed over the whole scene) reach `minObservationsForCap` (1,000,000,
  `--ba-cap-min-obs`, 0 = always): a scene small enough to be solved whole is solved whole, and once
  it is not, every solve of it is budgeted, a local window included.
- An over-budget image keeps its described observations first, since a detected position is the
  precise measurement, and the warp samples fill what is left of the budget: an image with few
  detections spends most of it on warp samples, one with many spends little.
- Each kind is taken round-robin across a grid of `ceil(sqrt(cap))` cells per side (at most 256)
  scaled to the image, one observation per cell per round, so the survivors cover the frame instead
  of clustering where the matcher was densest. Inside a cell the longest track comes first, and the
  remaining ties are ordered by a key drawn for that solve alone (a counter advanced on every solve),
  so successive adjustments take different subsets and over a reconstruction most observations take
  part in some solve.
- A track left with a single view is given a dropped observation back, a described one first; a
  track left with none stays out of the solve and its position stands until the next triangulation.

### The pair residuals

Every verified pair with a relative pose and a valid weight (`IsPoseLinkPair`) whose two images are
both in the solve adds a six-value residual (`RelativePoseError`) that compares the model's relative
pose with the pair's, which the reprojection residuals never see: where the tracks joining two parts
of the model are few or two-view only, they leave the joint between them free to bend, and the
pair's measurement from hundreds of correspondences holds it.

- Rotation (3 values): the angle-axis of `R^T (R_2 R_1^T)`, `R` the pair's relative rotation, in
  degrees divided by the rotation sigma.
- Baseline direction (3 values): the model's unit baseline `R_1 (C_2 - C_1)` minus the pair's own
  unit baseline, scaled by 180/pi (the chord between unit vectors is the angle in radians to third
  order) and divided by the direction sigma. A pair whose two images share a viewpoint (zero
  baseline) measured no direction: only its rotation counts.
- Sigmas: `--ba-pair-sigma` (`ReconstructionConfig::relativeRotationSigma`, 1 degree) for the
  rotation and twice that (`relativeTranslationSigmaFactor`) for the direction, the weaker of a
  pair's two measurements; copied onto every `BAConfig` the reconstruction derives
  (`SetBAPairConstraints`). Each is divided by `sqrt(min(WI, 500) / 100)`, `WI` the pair's weighted
  inliers (`PairWeight.md`), so a pair of 500 inliers counts `sqrt(5)` times one of 100. Either sigma
  at 0 switches that half off; both at 0 leaves the solve fitting reprojections alone.
- All pair residuals share one Huber loss at 3 standard deviations: past that the pair and the model
  disagree about more than measurement noise, and the pull stops growing.

### GPS position priors

With `gpsPositionWeight` or `gpsPositionWeightZ` above 0 and a geo-aligned scene (`GEO_ALIGN`), a
global solve adds `GPSPositionError` per image with GPS metadata: the camera centre against its ENU
position, divided by the image's accuracy (10 m horizontal and 20 m vertical when the metadata has
none), scaled by `sqrt(weight * scale * pixel_scale)` with `pixel_scale` the median track depth over
the median focal length so that the metric terms balance the pixel terms; no robust loss. Local
solves add none. The priors anchor all seven degrees of freedom of the gauge (`PoseUncertainty.md`).

## The gauge

- Without GPS priors a global solve holds the pose of the image with the most reprojection residuals
  constant. The reference pose leaves the scale free, and the iterations of a solve drift it; once
  the cap leaves tracks out of a solve their points stay at the old scale and their observations land
  pixels off. So a second camera, the best measured among those at least the median distance from the
  reference, keeps one centre coordinate where it is (the coordinate of its largest offset from the
  reference), as COLMAP does; the camera gets its whole pose back after the solve. A capped solve then
  changes the scale by about 0.01%.
- A local solve fixes the views in `fixedViewIDs`; when there are none it fixes its best-connected
  local view.

## Intrinsics by level

`--refine-intrinsics` (`ReconstructionConfig::baIntrinsicFlags`, default 1) says what the
reconstruction's adjustments may refine; every adjustment derives its `BAConfig` from it and may
restrict it further.

| Level | Refined |
|---|---|
| 0 (`INTRINSIC_NONE`) | nothing |
| 1 (`INTRINSIC_MAIN`) | focal length (`fy/fx` held), k1, k2 |
| 2 (`INTRINSIC_MAIN_EXTRA`) | those plus k3, the principal point and the tangential terms |

The aspect ratio and k4-k6 are reached only through the flags (`ba_intrinsic_flags` in Python,
`INTRINSIC_ALL`); k4-k6 are refined only for a camera with `useAdditionalDistortion` set, which no
stage sets.

| Adjustment | Intrinsics |
|---|---|
| local (resection) | never |
| star initializer | the focal length alone (`StarInitConfig::refineFocalLength`), none for a forced focal; distortion is left to the resection once the model can constrain it |
| full (resection) | the main set the level allows (focal length, k1, k2), until `minRefineExtIntrs` (100) images are registered |
| extended (resection, from `minRefineExtIntrs` registered images and in its closing adjustment), final adjustment, merged scene | every intrinsic the level allows |
| known-poses finetune | the level's flags; the main set also when a camera's intrinsics are untrusted |
| the adjustment that records the pose covariance | none (intrinsics already converged) |

## Repeated full solves

A full adjustment of the resection is repeated (`AdjustRepeated`, `Resection.h`) because one solve
fits the observations of the model it started from, and a camera moved by a few degrees has its
ties to the rest of the model a few pixels past the bar at its old pose.

1. The model is filtered at `fullBAInputErrorFactor` (3) times `maxReprojError` (4 px), so the solve
   fits the observations within 12 px.
2. The solve runs. The model is then filtered at `maxReprojError` and its tracks left without a
   position are triangulated again.
3. Another solve follows, up to `fullBARepeats` (4) after the first, only while some camera that
   held at least `fullBARepeatMinObservations` (100) observations in the input has at least
   `fullBARepeatMinChange` (0.05) of them added or removed by the input of the next solve.
4. After the last solve allowed the model is filtered and its pointless tracks triangulated again, so
   a capped adjustment ends as a converged one does.

The merge uses a single solve: the merged scene is filtered at the loose bar (3 x `maxReprojError`),
triangulated and adjusted once (`AdjustMergedScene`).

## The solver

- Linear solver: `DENSE_SCHUR` below 500,000 reprojection residuals; above, `SPARSE_SCHUR` with inner
  iterations in an order that depends on the problem alone (the order Ceres picks itself depends on
  allocator addresses). A local solve is always `SPARSE_SCHUR`. A failed sparse solve is retried once
  with `ITERATIVE_SCHUR` and the `SCHUR_JACOBI` preconditioner.
- Iterations: `maxIterations` 100 by default; the star initializer's two solves use 10 and 25, the
  resection's local solve 20, its full solves 40 and its closing adjustment 100, the known-poses
  path 25 and then 100;
  `functionTolerance` 1e-6.
- Threads: every solve clamps itself to the scene's thread budget, since sub-scenes solve
  concurrently.
- `Adjust` refuses a scene with fewer than two posed images or fewer than 50 tracks.
- An image no residual reaches is left out of the problem rather than added as a free block.
- A solve that succeeded keeps its Ceres problem alive for the pose covariance
  (`ComputePoseUncertainty`, `PoseUncertainty.md`).

## Parameters

| Parameter (`BAConfig`) | CLI flag | Default |
|---|---|---|
| `robustThreshold` | none | 2 px |
| `denseObservationWeight` | `--ba-dense-weight` | -1 (measure) |
| `denseHalfWeightObservations` | none | 300 |
| `maxDenseObservationWeight` | none | 0.01 |
| `maxObservationsPerImage` | `--ba-max-obs` | 1500 |
| `minObservationsForCap` | `--ba-cap-min-obs` | 1,000,000 |
| `relativeRotationSigma`, `relativeTranslationSigma` | `--ba-pair-sigma` | 1 degree, 2 degrees |
| intrinsic flags | `--refine-intrinsics` | 1 |
| `gpsPositionWeight`, `gpsPositionWeightZ` | `--gps-position-weight`, `--gps-position-weight-z` | 0 (off) |
| `maxIterations`, `functionTolerance` | none | 100, 1e-6 |

## Invariants

- Only inlier tracks and posed images take part; every inlier observation of a track is posed.
- The unit quaternion of a pose stays unit by the manifold; the cost functors use the
  no-normalisation rotation.
- The cap never reduces a track that has two observations in the solve to one.
- The dense weight is resolved once per solve, not per residual, and by the whole scene.
- A local solve changes only the poses in `viewIDs`; it never changes an intrinsic.

## Validation

The per-image dense weight, on the eleven captures with verified reference poses, against the same
dense weight for every image:

| per-image dense weight | outcome |
|---|---|
| half weight at 300 described observations, no ceiling | 55 wrong images against 39, a 46-image tail left out in every run |
| half weight at 300 and a ceiling of 0.01 (shipped) | completeness/accuracy F1 +0.009 [+0.002, +0.017] (three runs of each; 95%, paired bootstrap over images and runs), the gain in accuracy, no capture worse; an interior's real failure (15 images) avoided in 4 of 6 runs against none of 9 |

## Rejected alternatives

- A constant dense weight tuned to one capture: the ratio of the two sigmas moves several-fold
  between captures, so no constant is right on both; pinning the weight at 0.03 or 0.01 instead of
  measuring it moved the whole-scene accuracy only a little either way.
- Borrowing the view graph's dense evidence number for the bundle adjustment, or the reverse: two
  questions, two numbers.
- A ceiling used as a floor (the per-image weight held at 0.01 or more): a tie with the same dense
  weight for every image (+0.004 [-0.002, +0.010]).
- A sigma per keypoint kind re-estimated every solve (a per-observation uncertainty), with and
  without a correlation inflation per image and kind: ties at best (-0.006 to -0.001 F1); the
  correlation variants misplace 131 to 154 images of an interior in single runs.
- `ITERATIVE_SCHUR` for the large problems: slower and far less predictable on the captures measured,
  since the conjugate gradients a step costs depend on a conditioning the dense observations
  thin.
- The power-series-expansion preconditioner (`SCHUR_POWER_SERIES_EXPANSION`) instead of
  `SCHUR_JACOBI`: 1.2 to 3.9 times slower on an 83-camera scene, 1.6 to 1.7 times slower on a
  1106-camera one, and it did not converge in over 81 minutes on a 6.3-million-residual bundle.
- Combining the keypoint-confidence weight with the dense weight: it charges the sampling scale twice
  (an effective sigma ratio of about 10.8 against the 1.55 to 4.02 the captures measure).

## Open items

- The cap's tie-break key is drawn from a process-wide counter advanced on every solve, so with
  sub-scenes solving concurrently the subsets drawn depend on the order in which solves start.
- `BAConfig::useKeypointConfidence` and the intrinsic flags beyond level 2 have no command-line
  flag.
