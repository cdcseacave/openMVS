# PoseLib Bearing-Vector Pose Estimation

## 1. Purpose and scope

Absolute (PnP) and relative pose estimation in `libs/SFM` run on **3D unit bearing vectors**
(`Camera::UnprojectNormalized`), not 2D normalized-plane coordinates, so that camera models whose
field of view exceeds a hemisphere — principally `SphericalCamera` — never lose the hemisphere
sign that a 2D projection discards. The RANSAC/refinement machinery itself is not
OpenMVS-authored: the vendored PoseLib (`vcpkg.json` package `poselib`) ships native bearing-vector
robust estimators (`poselib::estimate_absolute_pose_bearings`,
`poselib::estimate_relative_pose_bearings`, declared in `PoseLib/robust.h`) that OpenMVS calls
directly. There is no OpenMVS-side wrapper layer; for pinhole cameras the bearing path is used
unconditionally too (`UnprojectNormalized` reduces to `normalize((x/z, y/z, 1))`), so there is a
single pose-estimation code path for every camera type instead of a pinhole/spherical fork.

## 2. Algorithm as implemented

### PoseLib entry points used

`PoseLib/robust.h` (vcpkg `poselib` package) exposes, alongside its pixel-space estimators:

```cpp
RansacStats estimate_absolute_pose_bearings(const std::vector<Point3D> &bearings, const std::vector<Point3D> &points3D,
                                             const AbsolutePoseOptions &opt, CameraPose *pose, std::vector<char> *inliers);
RansacStats estimate_relative_pose_bearings(const std::vector<Point3D> &bearings_1, const std::vector<Point3D> &bearings_2,
                                             const RelativePoseOptions &opt, CameraPose *relative_pose,
                                             std::vector<char> *inliers, bool check_cheirality = true);
```

Both run LO-RANSAC over the same minimal solvers as the pixel-space estimators (`p3p` for
absolute, the generalized 5-point solver for relative), score candidates on the unit sphere
(chord-distance-squared for absolute via `compute_msac_score_bearing`; unit-norm symmetric
Sampson-on-sphere for relative via `compute_sampson_msac_score_bearing`, `PoseLib/robust/utils.h`),
and locally refine the winner with the matching bearing-native bundle routine
(`bundle_adjust_bearing` / `refine_relpose_bearing`, `PoseLib/robust/bundle.h`) — refinement is not
a stub, it runs inside every call. `AbsolutePoseOptions` / `RelativePoseOptions`
(`PoseLib/types.h`) each wrap a `RansacOptions` + `BundleOptions` pair and a single `max_error`
field whose unit is contextual: pixels for the pixel-space estimators, **radians** for the
bearing-vector ones (converted internally to chord distance / `sin(angle)`).

Cheirality is bearing-native rather than a pinhole `Z > 0` test: absolute pose requires
`b_pred . b_obs > 0`; relative pose (`check_cheirality`, default `true`) requires the
midpoint-triangulation parameter along each ray to be positive. Both formulations accept
back-hemisphere spherical observations — the test is about ray direction, not a depth sign — and
without it the four essential-matrix decompositions would score identically under Sampson error
and RANSAC would keep an arbitrary one.

### Call sites

- **`Resection::RegisterImage`** (`libs/SFM/Resection.cpp:68-121`) — absolute pose / PnP. Builds
  `bearings` via `img.pCamera->UnprojectNormalized(kp)` for every inlier-track observation on the
  image being registered, converts `config.ransac.threshold` to radians via
  `img.pCamera->PixelErrorToAngular(threshold * img.pCamera->GetFeatureNoiseScale())`, and calls
  `estimate_absolute_pose_bearings`.
- **`PairsMatcher::GeometricFilter`** (`libs/SFM/PairsMatcher.cpp:439-505`), calibrated branch
  (both images `TrustIntrinsics()`) — relative pose. Converts the pixel Sampson threshold to
  radians per camera via `PixelErrorToAngular` and averages the two (`0.5*(angle1+angle2)`, valid
  because the residual unit is an angle regardless of each camera's pixel resolution), builds
  `bearings1`/`bearings2` via `UnprojectNormalized`, seeds RANSAC with an existing pose from
  `PreMatch` when available (`opt.ransac.score_initial_model = true`), and calls
  `estimate_relative_pose_bearings`. The fundamental matrix `pair.F` is only composed afterward
  when **both** cameras are pinhole (`SphericalCamera::GetK()` is identity, so `F` is not
  meaningful for a mixed or spherical pair); downstream consumers branch on `pair.F.has_value()`.

### Consumers that use bearing vectors directly (not through PoseLib)

- **`StarInitializer`** (`libs/SFM/StarInitializer.cpp:90-98`) — scale-averaging triangulation:
  bearings from `UnprojectNormalized`, midpoint triangulation via `TriangulatePoint3D`, angular
  (not pixel) reprojection gate via `PixelErrorToAngular`.
- **`ImagePair::FilterMatches`** (`libs/SFM/ImagePair.cpp:220-269`) — epipole-proximity and
  reprojection filtering on bearings; cheirality is `cosErr <= 0` on the bearing dot product
  instead of a depth sign.
- **`MatchGeometric.cpp`** (`libs/SFM/MatchGeometric.cpp:110-216`) — descriptor-guided candidate
  matching branches on `pair.F` vs `pair.E`: pinhole pairs keep the pixel-space epipolar-line
  distance test unchanged; spherical/mixed pairs (`pair.E` set, `pair.F` absent) precompute unit
  bearings for every keypoint once and test candidates with the same
  Sampson-on-sphere residual `r² = (b2·(E b1))² / (‖(Eb1)_xy‖² + ‖(Eᵀb2)_xy‖²)` against an
  angular threshold — a local re-implementation of the same residual PoseLib's
  `compute_sampson_msac_score_bearing` uses, kept separate from the F-matrix path so pinhole
  behavior has zero regression risk.

## 3. Parameters and defaults

| Parameter | Location | Default | Meaning |
|---|---|---|---|
| `opt.max_error` (absolute) | `Resection::RegisterImage` | `PixelErrorToAngular(config.ransac.threshold * GetFeatureNoiseScale())` | Angular inlier threshold, radians |
| `opt.max_error` (relative) | `PairsMatcher::GeometricFilter` | `0.5*(cam1.PixelErrorToAngular(px) + cam2.PixelErrorToAngular(px))` | Angular inlier threshold, radians, averaged across the pair |
| `GetFeatureNoiseScale()` | `Camera` (`libs/SFM/Camera.h`) | pinhole 1, spherical 2 | Multiplier on pixel-space thresholds before angular conversion, for camera models with noisier feature localization (cube-face SIFT seam/warp effects) |
| `check_cheirality` | `estimate_relative_pose_bearings` call | `true` | Disambiguates the four essential-matrix decompositions; bearing-native, valid for back-hemisphere features |
| `opt.ransac.max_iterations` / `min_iterations` | `config.ransac` (Resection/PairsMatcher config) | pipeline-configured | Forwarded unchanged from the existing pixel-space RANSAC config |
| `opt.ransac.success_prob` | `config.ransac.confidence` | pipeline-configured | RANSAC confidence |
| `minInliers` | `ResectionConfig` | 12 | Minimum PnP inliers to accept a pose |
| `minCorrespondences` | `ResectionConfig` | 15 | Minimum 2D-3D correspondences to attempt resection |
| `minMatches` | `PairsMatcherConfig` | 50 (AKAZE/ORB), 15 (SIFT) | Minimum inlier matches to accept a pair |

## 4. Invariants and constraints

- Bearing vectors passed to PoseLib are always unit-norm, produced only by
  `Camera::UnprojectNormalized` — never by the 2D `Unproject()` form, which is front-hemisphere
  biased and singular near the equator/back-pole for `SphericalCamera`.
- `opt.max_error` for a bearing estimator is always an angle in radians; pixel-space thresholds
  are converted per-camera via `PixelErrorToAngular` before being passed in, never compared
  directly against a bearing residual.
- `pair.F` (fundamental matrix) is only populated when both cameras in a pair are pinhole;
  everything downstream that used to assume `F` is always present must check
  `pair.F.has_value()` and fall back to the bearing/essential-matrix path.
- For pinhole cameras, the bearing path is first-order equivalent to the corresponding 2D
  pixel-space PoseLib estimator (`normalize((X/Z, Y/Z, 1))` vs. `(X/Z, Y/Z)`), so switching
  every caller to bearings costs nothing numerically on the pinhole side; the two differ only at
  `O(error^3)` (different LM objective curvature), which is why the pinhole regression suite is
  unaffected.
- Cheirality stays bearing-native (dot-product sign test) in every caller; there is no separate
  pinhole depth-sign branch left in `Resection.cpp` / `PairsMatcher.cpp`.

## 5. Validation of the shipped defaults

none recorded

`apps/Tests/TestsSFM.cpp` exercises the bearing-vector relative-pose path end-to-end on synthetic
spherical scenes: `PairsMatcherSphericalTest` (rotation error ≤ 0.5°, translation direction
similarity ≥ 0.99) and `MatchGeometricSphericalTest` (rotation error ≤ 0.5°, translation
similarity ≥ 0.99, ≥ 80% of tracked correspondences retained as inliers, requires ≥ 20
back-hemisphere matches to be a meaningful test). Absolute-pose bearing correctness
(`estimate_absolute_pose_bearings` itself, including its Jacobian) is left to PoseLib's own
upstream test suite rather than duplicated in OpenMVS (`TestsSFM.cpp` comment at the
`PairsMatcherSphericalTest` definition points to `ports/poselib/source/tests/optim_bearing_test.cc`).

## 6. Rejected alternatives

- **OpenMVS-side RANSAC wrapper reusing PoseLib's low-level minimal solvers** (`p3p`,
  `gen_relpose_5p1pt`) plus a hand-written estimator class plugged into PoseLib's templated
  `ransac<Solver>()` — this was the original design once PoseLib exposed only pixel-space robust
  entry points. Superseded: PoseLib itself now ships `estimate_absolute_pose_bearings` /
  `estimate_relative_pose_bearings` as first-class robust entry points, making a parallel
  OpenMVS-maintained estimator layer unnecessary.
- **Converting bearings to 2D normalized-plane coordinates and reusing the generalized-camera
  estimators** (`estimate_generalized_relative_pose` / `estimate_generalized_absolute_pose`) —
  rejected because the conversion is lossy: it discards the hemisphere sign the whole feature
  exists to preserve, and the generalized-camera rig model doesn't fit a single spherical camera
  semantically (it would require passing the same camera center multiple times).

## 7. Open items

none recorded
