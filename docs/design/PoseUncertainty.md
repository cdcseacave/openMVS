# Pose Uncertainty — Per-Image Quality from the BA Covariance

## Overview

Every bundle adjustment implicitly knows how well each camera is localized: the inverse of the
Gauss-Newton Hessian at the solution is the covariance of the estimated parameters. This feature
reads that covariance off the **last global bundle adjustment** run during reconstruction, records
it per image on the scene, exports it as a CSV quality report, and visualizes it in the Viewer as
per-camera error ellipsoids.

```
Scene::Reconstruct (estimatePoseUncertainty)      CreateStructure --export-pose-quality
  final BA ──┐                                             │
  GPS-prior BA (supersedes) ──> Scene::poseUncertainty ──> quality.csv ──> Viewer --pose-quality-file
```

The primary use case is geo-referenced accuracy: on a GPS-aligned scene refined with GPS priors,
the reported values are absolute 1-sigma camera-position accuracies in **ENU meters**
(East/North/Up).

---

## The Estimator (`BundleAdjustment::ComputePoseUncertainty`)

Computed on the live instance after `Adjust()` succeeded (the solved `ceres::Problem` is kept
alive). Math adapted from COLMAP's covariance estimator:

1. Evaluate the sparse Jacobian `J` over `[poses, points]` (intrinsics excluded — the result is
   **conditioned on fixed intrinsics**, adequate for a per-image quality signal).
2. Schur-eliminate the 3D points: `S = H_cc − H_cp H_pp⁻¹ H_pc` with `H_pp` exactly 3x3
   block-diagonal (points are conditionally independent given the cameras).
3. Sparse **selected inverse** of `S` via the Takahashi recursion over its simplicial LDLT factor —
   no dense inverse; only entries on the factor pattern are computed, which always includes the
   per-pose 6x6 diagonal blocks.

Per image, `PoseUncertainty` stores:

| Field | Meaning | Units / frame |
|---|---|---|
| `rotVar` | rotation variance about the camera x/y/z axes | rad², body frame (quaternion tangent) |
| `posVar` | camera-center variance along the world X/Y/Z axes | world-units² |
| `posCov` | camera-center covariance off-diagonals (XY, XZ, YZ) | world-units² |

The pose block is parameterized `[quaternion, C]` with a
`ProductManifold<QuaternionManifold, EuclideanManifold<3>>`: the position tangent is the **plain
world-frame camera center**, so `posVar`/`posCov` form a genuine world-frame 3x3 covariance —
`GetPositionCovariance()` eigen-decomposes directly into an oriented error ellipsoid, with no frame
change needed. Sentinels: not-computed = `-1` (unregistered image, pose absent/partially fixed);
gauge datum = exactly `0` on all axes.

### Gauge semantics

A monocular BA has a 7-DOF gauge freedom (similarity). Two regimes:

- **No GPS priors** — the BA holds one reference pose constant (or the estimator picks the
  best-connected pose as datum and removes it from the system). This fixes 6 DOF; the **global
  scale stays unanchored**, so variances saturate at the regularization ceiling along the scale
  mode. Values are then a *relative* trust signal (compare images to each other), not absolute
  accuracies. The datum reports exactly 0.
- **GPS priors present** (`numGPSResiduals > 0`) — the priors anchor all 7 DOF, so no datum is
  designated and the covariances are **absolute** in the ENU frame.

---

## Pipeline Integration (`Scene::Reconstruct`)

Gated on `ReconstructionConfig::estimatePoseUncertainty` (set by CreateStructure when
`--export-pose-quality` is given):

1. The **final global BA** runs in instance form and records `Scene::poseUncertainty`.
   The covariance must be read **before** `FilterTracks`: the solved Ceres problem holds raw
   pointers into the track array, which filtering invalidates.
2. If GPS alignment succeeded and GPS weights are configured, the **GPS-prior BA** runs after
   `AlignToGPS` and **supersedes** the record with absolute ENU covariances (also covering images
   resected between the two BAs; images resected after the recorded BA keep not-computed entries).
3. `Scene::Transform` keeps the record consistent with the world frame across any subsequent
   similarity transform (including `AlignToGPS` itself when the GPS-prior BA does not run):
   `Cov' = scale² · R · Cov · Rᵀ`; the rotation variance is body-frame and unaffected.
4. `poseUncertainty` is serialized with the `.sfm` scene, so a saved reconstruction retains its
   quality record.

### GPS-prior bundle adjustment

`GPSPositionError` constrains each camera center to its GPS position converted to the scene ENU
frame (origin = the ECEF centroid stored by `AlignToGPS`, so both frames coincide by construction).
Residuals are divided by the per-image accuracy metadata (`positionAccuracy` /
`positionAccuracyZ`, with 10 m / 20 m fallbacks when EXIF provides none) and scaled by
`sqrt(weight · scaleFactor · pixel_scale)` where `pixel_scale = median_depth / median_focal`
balances the metric GPS terms against the pixel-unit reprojection terms — which is why this BA is
only meaningful **after** the scene is metric (post-alignment); earlier BAs gate the residuals off
via the `GEO_ALIGN` state. Enabled by `--gps-position-weight` / `--gps-position-weight-z`
(default 0 = disabled). Validated for pinhole cameras; spherical scenes use angular reprojection
residuals the weighting does not account for.

---

## The CSV Quality Report (`ExportPoseUncertaintyCSV`)

`CreateStructure --export-pose-quality quality.csv` dumps `Scene::poseUncertainty`, one row per
image:

```
# pose uncertainty (1-sigma): position in ENU meters (East/North/Up) (frame: ENU, gauge: absolute); ...
ID,name,valid,datum,sigmaPosX,sigmaPosY,sigmaPosZ,covPosXY,covPosXZ,covPosYZ,sigmaRotX,sigmaRotY,sigmaRotZ,numObs,gpsAccuracyXY,gpsAccuracyZ
```

- `ID` — the SFM image ID; `ExportMVS` writes it into `Interface::Image::ID`, so the report
  correlates with the `.mvs` project by ID (no filename matching). Vertex views keep referencing
  images-array positions — the ID is a parallel, purely external identifier.
- `sigmaPos*` are `sqrt` of the position variances; together with the raw `covPos*` off-diagonals
  the full 3x3 position covariance is reconstructible. `sigmaRot*` are degrees about the camera
  axes. The header comment states the frame (ENU vs local world units) and gauge (absolute vs
  datum-relative).
- `numObs` (inlier observations) and the a-priori GPS accuracies allow comparing the estimated
  accuracy against the sensor claim.

## Viewer Display

`Viewer scene.mvs --pose-quality-file quality.csv` matches rows to images by ID and renders a
translucent shaded-solid **error ellipsoid** at each camera center: axes/orientation from the
eigen-decomposition of the 3x3 position covariance, radii = 1-sigma times a log-scale magnification
slider (Render Settings), per-vertex color = jet from blue (best localized) to red (worst),
normalized at the 95th-percentile sigma. The surfaces are lit and drawn semi-transparent (alpha
0.6, depth-write off, sorted back-to-front) so the camera frustum at each center and overlapping
ellipsoids remain visible through the shell. Selecting a camera shows its per-axis position and
rotation sigmas; the gauge datum is labeled "reference".

## Validation

- `PipelineTest` (Test 1): covariance present, finite, Cauchy-Schwarz-consistent, exactly one datum.
- `GPSPriorPoseUncertaintyTest`: GPS-prior BA on a synthetic geo-aligned scene → datum-free
  (absolute) covariances, no NaNs with missing accuracy metadata, poses within GPS accuracy.
- `PoseUncertaintyExportTest`: CSV write/re-read, `.mvs` image-ID roundtrip, `Scene::Transform`
  covariance mapping against a random Sim(3), `.sfm` serialization roundtrip.
