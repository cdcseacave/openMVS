# Spherical Camera Support in OpenMVS SfM

## 1. Purpose and scope

`libs/SFM` supports equirectangular (360°) still images and video as a first-class camera model
end to end through sparse reconstruction: EXIF auto-detection, feature extraction, matching,
pose estimation, triangulation, and bundle adjustment all dispatch on `CameraType::SPHERICAL`
alongside `PINHOLE`. Dense reconstruction (`libs/MVS`) has no spherical camera model at all; the
SFM→MVS export bridges the gap by rendering each spherical image to N tangent pinhole cube-map
faces that downstream PatchMatch/texturing consume as ordinary pinhole images.

## 2. Algorithm as implemented

### Camera model — `libs/SFM/Camera.h`, `libs/SFM/Camera.cpp:216-278`

`SphericalCamera : public Camera` implements equirectangular projection/unprojection. `Project`
maps a 3D camera-space point to equirectangular pixel coordinates. Two unprojection forms exist:
`Unproject(x)` returns `(tan θ, tan φ / cos θ)` — algebraically the pinhole-normalized plane, so
it is only valid for the front hemisphere and diverges at the equator/back pole — while
`UnprojectNormalized(x)` returns a genuine 3D unit bearing vector and is singularity-free over the
whole sphere. `PixelErrorToAngular` converts a pixel threshold to radians; `GetFeatureNoiseScale`
returns 2 (cube-face SIFT has roughly twice the pixel-space localization noise of a pinhole
pipeline, from face-seam sampling and off-center descriptor warping). `GetK()` returns identity
(no traditional intrinsics); `TrustIntrinsics()` is always `true`.

Every geometric caller in the SFM library that needs whole-sphere correctness — triangulation,
track filtering, pose estimation, matching — goes through `UnprojectNormalized`, not `Unproject`.
Two call sites still use the raw 2D form: `View::Ray()` and `View::UnprojectPoint()`
(`libs/SFM/View.h`); `View::RayNormalized()` is the spherical-safe sibling kept alongside them.

### Feature extraction — `FeaturesExtractor::ExtractImage` / `ExtractImageSpherical` (`libs/SFM/FeaturesExtractor.cpp`)

When `image.pCamera->GetType() == CameraType::SPHERICAL`, extraction is dispatched to
`ExtractImageSpherical`: the equirectangular image is rendered to `config.cubemapFaces` tangent
pinhole faces (default 6; `config.cubemapFaceSize`, default 0 = `max(1024, equirect_width/4)`) via
`SphereCubeMap::MakeTangentFacesGeometry` + `SphericalToTangentialFaces`. Each face is wrapped as
a synthetic pinhole `Image` and run back through the normal `ExtractImage` path — SIFT/AKAZE/
SiftGPU detection logic is not duplicated per camera type. Resulting keypoints are reprojected to
equirectangular pixel coordinates (face pixel → bearing in the face frame → body frame via the
face rotation → `SphericalCamera::Project`); the keypoint `octave` field is repurposed to record
the source face ID. Duplicate keypoints straddling a face seam are suppressed by angular
non-max-suppression over unit bearings (a 3D octree, radius set by `cubemapDedupAngleDeg`,
default 0.25°), keeping the highest-response keypoint per angular neighborhood.

### Matching and pose estimation

- **`PairsMatcher::GeometricFilter`** (`libs/SFM/PairsMatcher.cpp`) — relative pose from bearing
  vectors via PoseLib's native bearing-vector RANSAC estimator; see
  `docs/design/PoseLibBearingVector.md` for the shared pinhole/spherical pose-estimation design.
  `pair.F` (fundamental matrix) is only composed when both cameras are pinhole, since
  `SphericalCamera::GetK()` is identity and makes `F` geometrically meaningless for a spherical or
  mixed pair; `pair.E` (essential matrix) is set instead.
- **`MatchGeometric.cpp`** — descriptor-guided candidate matching branches on `pair.F` vs
  `pair.E`: pinhole pairs keep the unchanged pixel-space epipolar-line distance test; spherical or
  mixed pairs precompute unit bearings for every keypoint once and score candidates with a
  Sampson-on-sphere residual `r² = (b2·(Eb1))² / (‖(Eb1)_xy‖² + ‖(Eᵀb2)_xy‖²)` against an angular
  threshold derived per-camera via `PixelErrorToAngular` and averaged across the pair.
- **`StarInitializer`** and **`ImagePair::FilterMatches`** triangulate and filter on unit bearings
  (`UnprojectNormalized`) with angular (not pixel) reprojection and epipole-proximity gates, so
  the same code path handles pinhole and spherical inputs.
- **`PairsWeighting::ComputeIntrinsicWeight`** (`libs/SFM/PairsWeighting.cpp`) — the spatial
  feature-coverage score bins spherical keypoints by **equal-solid-angle** cells (bearing azimuth
  `atan2(b.x, b.z)` and `b.y` each binned across the grid, wrapping the equirectangular seam)
  instead of the uniform pixel grid used for pinhole; everything else in pair weighting (triplet
  cycle-consistency, connectivity, baseline/angle scoring) is camera-agnostic.

### Triangulation — `libs/SFM/Triangulation.cpp`

`TriangulateTracks` calls `TriangulateSkewLLS` unconditionally for every track regardless of
camera type — it operates on bearing vectors and is singularity-free, so no per-camera branch is
needed. `TriangulateDLT` mixes a 2D pinhole-normalized coordinate with the 3×4 projection matrix
and is guarded by `ASSERT(img.pCamera->GetType() == CameraType::PINHOLE)`; it is not called from
anywhere in the codebase (production or tests) today.

### Track filtering — `libs/SFM/Track.cpp`

`ComputeTracksMeanReprojectionError` and `FilterTracks` both use `UnprojectNormalized` bearings
gated by `PixelErrorToAngular`, so back-hemisphere spherical observations are neither
misreported nor silently discarded as outliers.

### Bundle adjustment — `libs/SFM/BundleAdjustment.cpp`, `libs/SFM/BundleAdjustmentCostFunctions.h:357-412`

`AddReprojectionResidual` dispatches per `img.GetCameraType()`: pinhole observations get
`PinholeReprojectionError` with pose + 12-parameter intrinsics + point blocks; spherical
observations get `SphericalAngularReprojectionError::Create(kp.pt.x, kp.pt.y, width, height)` with
only pose + point blocks — spherical cameras never receive an intrinsics parameter block, so
nothing about the spherical model itself is refined by BA. The cost functor precomputes a
per-observation tangent-plane basis on the unit sphere, pre-scaled by `pixel_scale = width/(2π)`
so residuals come out in pixel-equivalent units; it has no z/cheirality check (valid across the
full sphere) and returns the two tangent-plane dot products as residuals rather than minimizing
angular (`acos`) error directly, for numerical stability near zero residual.

### EXIF auto-detection and keyframe extraction

`Image.cpp` sets `isSpherical` from EXIF `ProjectionType == 2` (equirectangular) and constructs a
`SphericalCamera(cv::Size(w, h))` (warning if `w != 2*h`). `KeyframeExtractor::ExtractFromVideo`
dispatches on `config.cameraType` (`PINHOLE` / `SPHERICAL`) to build the matching camera model.
`apps/ExtractKeyframes` exposes `--camera-type 0|1` (pinhole/spherical) and `--cubemap-faces`
(default 6, feature-extraction face count only).

### MVS export (cube-map bridge) — `libs/SFM/InterfaceMVS.cpp`

`SFM::ExportMVS` detects any `CameraType::SPHERICAL` camera in the scene, builds one
`SphereCubeMap::TangentFacesGeometry` for `(config.sphericalNumFaces, config.sphericalFaceSize)`
(defaults 6 / 1024px, `ExportMVSConfig` in `libs/SFM/InterfaceMVS.h`; must be one of 4/6/8/12/20 or
export fails), then runs four phases in `namespace SFM::SphereCubeMap` (scoped to
`InterfaceMVS.cpp`):

1. `EmitSphericalPlatforms` — one MVS `Platform` per spherical SFM camera with N face `Camera`
   records sharing the geometry's intrinsics and per-face rotations.
2. `AppendSphericalFaceImages` — N `Interface::Image` records per spherical source image, named
   `<stem>_face<k><ext>`; face image IDs are drawn from a synthetic counter past the maximum SFM
   image ID (pinhole images keep their original SFM ID).
3. `RenderAndSaveSphericalFaces` — renders and writes the N face images to disk.
4. `ProjectTrackOntoSphericalFaces` — for each 3D track point, projects into every face's pinhole
   model and adds a `Vertex::View` per face where the point is in front of that face's camera.

`UndistortDepthMaps` (`InterfaceMVS.cpp`) remains pinhole-only (explicitly skips any camera whose
type is not `PINHOLE`) — consistent with there being no native spherical depth-map path.

No CLI flag exposes `sphericalNumFaces` / `sphericalFaceSize` in `CreateStructure` or any other
app; only the Python bindings (`libs/SFM/PythonWrapper.cpp`) and hand-constructed
`ExportMVSConfig` (tests) can change them from the 6-face/1024px defaults.

## 3. Parameters and defaults

| Parameter | Location | Default | Meaning |
|---|---|---|---|
| `GetFeatureNoiseScale()` | `SphericalCamera` | 2 | Pixel-threshold multiplier before angular conversion (pinhole = 1) |
| `cubemapFaces` | `FeaturesExtractor` config | 6 | Tangent-face count for feature extraction (4/6/8/12/20) |
| `cubemapFaceSize` | `FeaturesExtractor` config | 0 (auto = `max(1024, width/4)`) | Tangent-face resolution for feature extraction |
| `cubemapDedupAngleDeg` | `FeaturesExtractor` config | 0.25° | Angular non-max-suppression radius for cross-face duplicate keypoints |
| `sphericalNumFaces` | `ExportMVSConfig` (`InterfaceMVS.h`) | 6 | Tangent-face count for MVS export (4/6/8/12/20; not exposed on any app's CLI) |
| `sphericalFaceSize` | `ExportMVSConfig` | 1024 px | Tangent-face resolution for MVS export (square; not exposed on any app's CLI) |
| `--camera-type` | `apps/ExtractKeyframes` | 0 (pinhole) | `0` = pinhole, `1` = spherical |
| `--cubemap-faces` | `apps/ExtractKeyframes` | 6 | Feature-extraction face count (does not affect MVS export) |

## 4. Invariants and constraints

- `SphericalCamera` requires `width == 2 * height` (equirectangular aspect ratio), enforced by an
  `ASSERT` in the constructor.
- Whole-sphere-correct callers must use `UnprojectNormalized`; the 2D `Unproject()` form is only
  valid in the front hemisphere and must never be used for RANSAC/DLT/angle inputs. `View::Ray()`
  and `View::UnprojectPoint()` are exceptions that still use the 2D form — callers passing a
  spherical image through them get front-hemisphere-biased results.
- `pair.F` is populated only when both cameras in a pair are pinhole; every consumer that used to
  assume `F` is always present must check `pair.F.has_value()` and fall back to the `pair.E` /
  bearing path.
- BA never adds an intrinsics parameter block for a spherical observation — no spherical-model
  parameter (e.g. a lens or orientation offset) is refinable by BA today.
- Pose-quality / uncertainty export (`docs/design/PoseUncertainty.md`) is validated for pinhole
  cameras only; spherical scenes use angular reprojection residuals its GPS-weight balancing term
  does not account for.
- `libs/MVS` has no `CameraType` concept at all — every dense-reconstruction consumer (PatchMatch,
  texturing, `UndistortDepthMaps`) only ever sees the rendered pinhole cube-map faces, never a
  spherical camera object.

## 5. Validation of the shipped defaults

| Test | File | Checks |
|---|---|---|
| `ReconstructSphericalSyntheticTest` | `apps/Tests/TestsSFM.cpp` | ≥95% of tracks recovered within 0.1 m of ground truth; mean reprojection ≤1.0 px; mean angular error ≤1.0°; same bounds post-BA |
| `PairsMatcherSphericalTest` | `apps/Tests/TestsSFM.cpp` | Relative-pose rotation error ≤0.5°; translation direction similarity ≥0.99 |
| `MatchGeometricSphericalTest` | `apps/Tests/TestsSFM.cpp` | Rotation error ≤0.5°; translation similarity ≥0.99; ≥80% of tracked correspondences retained as inliers; requires ≥20 back-hemisphere matches to be meaningful |
| `PipelineTest` (spherical BA sub-test) | `apps/Tests/TestsSFM.cpp` | Mean reprojection error ≤1.0 px post-BA |
| `CubeMapBridgeDropTopBottomTest` | `apps/Tests/TestsSFM.cpp` | 4-face geometry shows 0 views for zenith/nadir points vs. 6-face geometry showing the expected face |
| `CubeMapFaceRenderTest`, `CubeMapBridgeGeometryTest`, `CubeMapBridgeEndToEndTest`, `CubeMapBridgeMVSLoadTest`, `CubeMapBridgeMixedSceneTest` | `apps/Tests/TestsSFM.cpp` | Cube-map rendering, geometry, MVS-export round trip, and mixed pinhole/spherical scenes (no numeric tolerances recorded beyond pass/fail) |

## 6. Rejected alternatives

- **Stereographic projection from the back pole** as a 2D unprojection replacement — well-defined
  everywhere except the back pole, but still has a singularity there and changes the return value
  semantics, breaking pinhole callers.
- **Returning raw `(θ, φ)` angles** instead of a 2D plane coordinate — well-defined everywhere,
  but pinhole callers would silently misinterpret the result as normalized-plane coordinates.
- **Deleting `Camera::Unproject` entirely** — cleanest long-term, but a larger refactor across
  every pinhole caller; not done.

## 7. Open items

- `View::Ray()` and `View::UnprojectPoint()` still use the front-hemisphere-biased 2D `Unproject()`
  form rather than `UnprojectNormalized`; whether any live caller invokes them on a spherical image
  has not been audited.
- Dense reconstruction has no native spherical camera model; PatchMatch and texturing only ever
  operate on the rendered pinhole cube-map faces, so per-face seams and the fixed face
  count/resolution are inherent to the current MVS path, not a configurable trade-off from any
  app's CLI.
- Pose-uncertainty/covariance estimation is not validated for spherical cameras (see
  `docs/design/PoseUncertainty.md` §4).
