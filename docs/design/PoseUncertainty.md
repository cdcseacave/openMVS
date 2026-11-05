# Pose Uncertainty — Per-Image Quality from the BA Covariance

## 1. Purpose and scope

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

## 2. Algorithm as implemented

### The estimator — `BundleAdjustment::ComputePoseUncertainty` (`libs/SFM/BundleAdjustment.h/.cpp`)

Computed on the live instance after `Adjust()` (or `AdjustLocal()`) succeeded, while the solved
`ceres::Problem` is still kept alive:

1. Evaluate the sparse Jacobian `J` over `[poses, points]` (intrinsics excluded — the result is
   **conditioned on fixed intrinsics**, adequate for a per-image quality signal).
2. Schur-eliminate the 3D points: `S = H_cc − H_cp H_pp⁻¹ H_pc` with `H_pp` exactly 3x3
   block-diagonal (points are conditionally independent given the cameras).
3. Sparse **selected inverse** of `S` via the Takahashi recursion over its simplicial LDLT factor —
   no dense inverse; only entries on the factor pattern are computed, which always includes the
   per-pose 6x6 diagonal blocks.

Per image, `PoseUncertainty` (`libs/SFM/BundleAdjustment.h`) stores:

| Field | Meaning | Units / frame |
|---|---|---|
| `rotVar` | rotation variance about the camera x/y/z axes | rad², body frame (quaternion tangent) |
| `posVar` | camera-center variance along the world X/Y/Z axes | world-units² |
| `posCov` | camera-center covariance off-diagonals (XY, XZ, YZ) | world-units² |

The pose block is parameterized `[quaternion, C]` with a
`ProductManifold<QuaternionManifold, EuclideanManifold<3>>`: the position tangent is the **plain
world-frame camera center**, so `posVar`/`posCov` form a genuine world-frame 3x3 covariance —
`PoseUncertainty::GetPositionCovariance()` eigen-decomposes directly into an oriented error
ellipsoid, with no frame change needed. Sentinels: not-computed = `posVar.x < 0` (unregistered
image, pose absent/partially fixed, `IsValid()` false); gauge datum = exactly `0` on all axes.

A second entry point, `BundleAdjustment::ComputePoseUncertaintyCeres()`, computes the same
per-image covariance from the same solved problem using `ceres::Covariance` (`DENSE_SVD`) instead
of the custom Schur + selected-inverse path, with matched conditioning (intrinsics fixed, points
marginalized, same gauge/datum). It is `O(n^3)` — a validation-only cross-check
(`GPSPriorPoseUncertaintyTest`, §5), not part of the pipeline.

### Gauge semantics

A monocular BA has a 7-DOF gauge freedom (similarity). Two regimes:

- **No GPS priors** — the BA holds one reference pose constant (or the estimator picks the
  best-connected pose as datum and removes it from the system). This fixes 6 DOF; the **global
  scale stays unanchored**, so variances saturate at the regularization ceiling along the scale
  mode. Values are then a *relative* trust signal (compare images to each other), not absolute
  accuracies. The datum reports exactly 0.
- **GPS priors present** (`numGPSResiduals > 0`) — the priors anchor all 7 DOF, so no datum is
  designated and the covariances are **absolute** in the ENU frame.

### Pipeline integration — `Scene::Reconstruct`

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
via the `GEO_ALIGN` state. Enabled by `--gps-position-weight` / `--gps-position-weight-z`.
Validated for pinhole cameras; spherical scenes use angular reprojection residuals the weighting
does not account for.

### The CSV quality report — `ExportPoseUncertaintyCSV` (`libs/SFM/BundleAdjustment.h/.cpp`)

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
- Not-computed entries are written as `-1`; the gauge datum (if any) as all-zero with `datum=1`.

### Viewer display

`Viewer scene.mvs --pose-quality-file quality.csv` matches rows to images by ID and renders a
translucent shaded-solid **error ellipsoid** at each camera center: axes/orientation from the
eigen-decomposition of the 3x3 position covariance, radii = 1-sigma times a log-scale magnification
slider (Render Settings), per-vertex color = jet from blue (best localized) to red (worst),
normalized at the 95th-percentile sigma. The surfaces are lit and drawn semi-transparent (alpha
0.6, depth-write off, sorted back-to-front) so the camera frustum at each center and overlapping
ellipsoids remain visible through the shell. Selecting a camera shows its per-axis position and
rotation sigmas; the gauge datum is labeled "reference".

## 3. Parameters and defaults

| Parameter | CLI flag | Default | Meaning |
|---|---|---|---|
| `estimatePoseUncertainty` | `CreateStructure --export-pose-quality <file>` | disabled | Gates covariance estimation during reconstruction and enables the CSV export |
| GPS horizontal weight | `--gps-position-weight` | 0 (disabled) | Horizontal weight of GPS position priors in the GPS-prior BA |
| GPS vertical weight | `--gps-position-weight-z` | 0 (disabled) | Vertical weight of GPS position priors in the GPS-prior BA |
| horizontal accuracy fallback | `GPSPositionError` | 10 m | Used when EXIF provides no `positionAccuracy` |
| vertical accuracy fallback | `GPSPositionError` | 20 m | Used when EXIF provides no `positionAccuracyZ` |
| Viewer input | `Viewer --pose-quality-file <file>` | none | Loads the CSV and enables the uncertainty-ellipsoid display |

## 4. Invariants and constraints

- The covariance must be read before `FilterTracks` runs, since the solved Ceres problem holds
  raw pointers into the track array that filtering invalidates.
- `ComputePoseUncertainty()` conditions on fixed intrinsics (intrinsics excluded from the
  Jacobian); the result is a per-image quality signal, not a full-parameter covariance.
- Without GPS priors, the global scale mode is gauge-unanchored: variances saturate at the
  regularization ceiling along that mode, so values compare images to each other but are not
  absolute accuracies. Only GPS-prior BA (`numGPSResiduals > 0`) yields absolute ENU covariances.
- `Scene::Transform` must remap `poseUncertainty` (`Cov' = scale² · R · Cov · Rᵀ`) across every
  similarity transform applied after the covariance was recorded, or the record goes stale
  relative to the world frame.
- The GPS-prior BA is only meaningful after the scene is metric (post-`AlignToGPS`); earlier BAs
  gate `GPSPositionError` residuals off via the `GEO_ALIGN` state.
- GPS-prior weighting is validated for pinhole cameras only; spherical scenes use angular
  reprojection residuals that the `pixel_scale` balancing term does not account for.
- CSV image IDs, not filenames, are the correlation key back to the `.mvs` project.

## 5. Validation of the shipped defaults

none recorded

Correctness (not default-tuning) is exercised by three tests in `apps/Tests/TestsSFM.cpp`:
`PipelineTest` (covariance present, finite, Cauchy-Schwarz-consistent, exactly one datum),
`GPSPriorPoseUncertaintyTest` (GPS-prior BA on a synthetic geo-aligned scene: datum-free absolute
covariances, no NaNs with missing accuracy metadata, poses within GPS accuracy, and agreement with
the `ComputePoseUncertaintyCeres()` cross-check), and `PoseUncertaintyExportTest` (CSV write/re-read,
`.mvs` image-ID roundtrip, `Scene::Transform` covariance mapping against a random Sim(3), `.sfm`
serialization roundtrip).

## 6. Rejected alternatives

none recorded

## 7. Open items

- GPS-prior residual weighting does not account for the angular reprojection residuals used by
  spherical cameras, so absolute covariances from the GPS-prior BA are not validated for that
  camera type.
