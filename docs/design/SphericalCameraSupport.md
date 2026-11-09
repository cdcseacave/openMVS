# Spherical Camera Support in OpenMVS SfM

## 1. Purpose and scope

`libs/SFM` supports equirectangular (360°) still images and video as a first-class camera model
end to end through sparse reconstruction: EXIF auto-detection, feature extraction, matching,
pose estimation, triangulation, and bundle adjustment all dispatch on `CameraType::SPHERICAL`
alongside `PINHOLE`. The SFM→MVS export writes spherical cameras natively (Interface v8
`SPHERICAL` model) by default; `libs/MVS` loads and saves them, and `DensifyPointCloud` estimates
their depth-maps natively on the CPU, CUDA and Metal and fuses them (`SphericalPatchMatch.md`). The
camera models are described in `CameraModels.md`. Meshing, refinement and texturing remain pinhole
only. The cube-map bridge, which renders
each spherical image to N tangent pinhole faces that the whole MVS pipeline consumes as ordinary
pinhole images, stays available as an export option.

## 2. Algorithm as implemented

### Camera model — `libs/Math/Camera/CameraModels.h`, `libs/SFM/Camera.h/.cpp`

The projection math lives in the header-only, scalar-templated `SEACAVE::Spherical<T>` model
(`libs/Math/Camera/CameraModels.h`, shared with `Pinhole<T>` and `BrownDistortion<T>` and usable on
the host, in CUDA and on Ceres Jets). `Spherical<T>::FromSize(W, H)` sets `fx = W/2π`,
`fy = H/π`, `cx = (W-1)/2`, `cy = (H-1)/2`: pixel centers lie at integer coordinates (the
OpenCV/MVS convention), so column 0 is centered at longitude `-π + π/W` and the image spans
`[-0.5, W-0.5] x [-0.5, H-0.5]`. Axes are the pinhole camera frame (x-right, y-down, z-forward):
longitude `θ = atan2(X.x, X.z)` grows with the column (`u = fx·θ + cx`, forward +Z at the
center column, +X at 3/4 width), elevation `φ = asin(-X.y/|X|)` is `+π/2` on the top edge
(`v = cy - fy·φ`, computed as `fy·atan2(X.y, |X.xz|) + cy`), so an upright camera's sky is at the
top. `Ray(x)` returns the unit bearing `(cos φ sin θ, -sin φ, cos φ cos θ)`, `Depth(X) = |X|`,
`Project` reports a point visible iff `|X| > 0`.

`SphericalCamera : public Camera` is a thin wrapper over that model (`GetModel()`), converting
`TPoint` ↔ Eigen at the boundary. Two unprojection forms exist: `Unproject(x)` returns the bearing
scaled to `|z| = 1` — the pinhole-normalized plane in the front hemisphere, divergent where the
bearing is orthogonal to the optical axis — while `UnprojectNormalized(x)` returns the unit
bearing and is singularity-free over the whole sphere. `PixelErrorToAngular` converts a pixel threshold to radians; `GetFeatureNoiseScale`
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
`SphereCubeMap::MakeTangentFacesGeometry` + `SphericalToTangentialFaces`
(`libs/Math/Camera/CameraUndistort.h/.cpp`, shared with the MVS export). Each face is wrapped as
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

### Bundle adjustment — `libs/SFM/BundleAdjustment.cpp`, `libs/SFM/BundleAdjustmentCostFunctions.h`

`AddReprojectionResidual` dispatches per `img.GetCameraType()`: pinhole observations get
`PinholeReprojectionError` with pose + 12-parameter intrinsics + point blocks; spherical
observations get `SphericalAngularReprojectionError::Create(kp.pt.x, kp.pt.y, width, height)` with
only pose + point blocks — spherical cameras never receive an intrinsics parameter block, so
nothing about the spherical model itself is refined by BA. The cost functor precomputes a
per-observation tangent-plane basis on the unit sphere at the observed bearing (longitude and
elevation from `Spherical<double>::Angles`, so the observation follows the same pixel-center
convention as `Project`), pre-scaled by the model's `fx = width/(2π)` (longitude) and
`fy = height/π` (elevation) so residuals come out in pixel-equivalent units; it has no z/cheirality check (valid across the
full sphere) and returns the two tangent-plane dot products as residuals rather than minimizing
angular (`acos`) error directly, for numerical stability near zero residual.

### EXIF auto-detection and keyframe extraction

`Image.cpp` sets `isSpherical` from EXIF `ProjectionType == 2` (equirectangular) and constructs a
`SphericalCamera(cv::Size(w, h))` (warning if `w != 2*h`). `KeyframeExtractor::ExtractFromVideo`
dispatches on `config.cameraType` (`PINHOLE` / `SPHERICAL`) to build the matching camera model.
`apps/ExtractKeyframes` exposes `--camera-type 0|1` (pinhole/spherical) and `--cubemap-faces`
(default 6, feature-extraction face count only).

### MVS export — `libs/SFM/InterfaceMVS.cpp`

`SFM::ExportMVS` writes an Interface v8 project (`libs/Math/Interface.h`, format in
`docs/wiki/Interface.md`). Each SFM camera becomes one platform with one camera:

- **Pinhole without distortion:** `PINHOLE`, the original images.
- **Pinhole with distortion:** `BROWN` with `k1,k2,p1,p2,k3` (plus `k4,k5,k6` when the rational
  terms are used) and the original image paths; no image is written. The images of a camera the
  SfM rotated to landscape get the camera of their on-disk orientation: `View::RevertRotation` for
  `K`, and `p1' = -p2`, `p2' = p1` for the tangential terms (the radial ones are invariant).
  `libs/MVS` undistorts on load (below). Only if `ExportMVSConfig::undistortImageDir` is set
  (`CreateStructure --undistort-extension <ext>`) are the images undistorted on export by
  `Scene::UndistortImages` (cubic remap, `undistortAlpha`) and written as `PINHOLE` cameras of
  `UndistortedK(K, distortion, size, undistortAlpha)`; the images of the other cameras keep their paths.
- **Spherical, `Spherical::NATIVE` (default, `--spherical-export native`):** `SPHERICAL` with the
  angular `K` of `Spherical<REAL>::FromSize(W, H)` and the original image paths.
- **Spherical, `Spherical::CUBEMAP` (`--spherical-export cubemap`):** the cube-map bridge below.

Observations are written when `ExportMVSConfig::includeObservations` is set (off by default, no
CLI flag): the keypoint of every view in the exported image file (through `UndistortedK` for an
undistorted file, in the on-disk orientation for a rotated one, and through the feature ray
projected into each face for the cube-map faces); the export fails if a track observation has no
keypoint.

The cube-map bridge builds one `SphereCubeMap::TangentFacesGeometry` for
`(config.sphericalNumFaces, config.sphericalFaceSize)` (defaults 6 / 1024px; the face count must be
one of 4/6/8/12/20, `SphereCubeMap::IsSupportedNumFaces`, or the export fails), then runs four phases
implemented as file-local helpers of `InterfaceMVS.cpp`:

1. `EmitSphericalPlatforms` — one MVS `Platform` per spherical SFM camera with N face `Camera`
   records sharing the geometry's intrinsics and per-face rotations.
2. `AppendSphericalFaceImages` — N `Interface::Image` records per spherical source image, named
   `<stem>_face<k><ext>`; face image IDs are drawn from a synthetic counter past the maximum SFM
   image ID (pinhole images keep their original SFM ID).
3. `RenderAndSaveSphericalFaces` — renders (`SphereCubeMap::SphericalToTangentialFaces`: each
   face pixel's ray is projected by the `Spherical` model and sampled bilinearly, wrapping across
   the seam) and writes the N face images to disk.
4. `ProjectTrackOntoSphericalFaces` — for each 3D track point, projects into every face's pinhole
   model and adds a `Vertex::View` per face where the point is in front of that face's camera.

`UndistortDepthMaps` (`InterfaceMVS.cpp`, the ROMA2 depth-maps of `CreateStructure`) is
pinhole-only (it skips any camera whose type is not `PINHOLE`). It writes each depth-map in the
frame of the undistorted camera of its image, `UndistortedK(K, distortion, imageSize, alpha)` scaled
to the depth-map resolution, and stores that `K'`: `alpha` is `UNDISTORT_ALPHA` for a native export
(the frame `libs/MVS` loads the images in) and `--undistort-alpha` for an undistorted one. Its remap
tables and depth-aware resampling are `SEACAVE::UndistortMaps` (`libs/Math/Camera/CameraUndistort.h`),
shared with `Scene::UndistortImages` and `apps/InterfaceMetashape`.

No CLI flag exposes `sphericalNumFaces` / `sphericalFaceSize` in `CreateStructure` or any other
app; only the Python bindings (`libs/SFM/PythonWrapper.cpp`) and hand-constructed
`ExportMVSConfig` (tests) can change them from the 6-face/1024px defaults.

### MVS load — `libs/MVS/Scene.cpp`, `libs/MVS/Image.cpp`

`Scene::LoadInterface` keeps the model of every camera in `MVS::Platform::Camera::model`:

- **`BROWN`:** the lens of the image files (`CameraLens`: `K` at the file resolution, the
  distortion, the resolution, which a Brown camera must carry) is kept, and the camera `K` every
  consumer sees is `K' = UndistortedK(lens.K, lens.distortion, lens.size)`, i.e.
  `cv::getOptimalNewCameraMatrix` with `UNDISTORT_ALPHA = 0.6` at the same size: the rule of the SfM
  undistort-on-export path, computed by the one function in `libs/Math/Camera`. `Image::UpdateCamera`
  copies the model and lens into each `MVS::Image`.
- **`Image::ReadPixels`** reads an image file and, for a Brown camera, remaps it at its original
  resolution (linear) through the `CameraUndistortMaps` of the process-wide cache
  (`GetCameraUndistortMaps`: keyed by the lens and the resolution, shared by the images of a
  camera, thread-safe, least recently used maps dropped beyond 1 GiB), before any resize.
  `LoadImage`, `ReloadImage`, the densify on-demand decode and the Viewer observations panel all
  read through it, so every app sees the same pixels; depth-maps live in the `K'` frame.
  `Image::LoadMask` remaps a segmentation mask with nearest-neighbour sampling.
- The undistorted pixels whose bilinear source footprint leaves the image file are unset in
  `CameraUndistortMaps::mask`; `DepthEstimator::ImportIgnoreMask` ANDs it into the ignore mask
  of a Brown image whatever the ignore label (densify CPU and CUDA, refine), and the texturing
  validity mask detects them as the black regions it already handles.
- Observations (`HAS_OBSERVATIONS`) are undistorted into `K'` (`BrownDistortion::Undistort`) into
  `PointCloud::pointObservations`; `Scene::SaveInterface` distorts them back and writes the camera
  back as it was read (`K`, resolution and coefficients).
- **`SPHERICAL`:** loaded and saved as is; `DensifyPointCloud` estimates the depth-maps with the CPU,
  CUDA or Metal PatchMatch and fuses them on the CPU, the fusion dispatching on the image model
  through `ModelCamera` (`libs/MVS/Image.h`); see `SphericalPatchMatch.md`. `Scene::DenseReconstruction`
  refuses the scene for SGM (`HasSphericalImages`, `libs/MVS/SceneDensify.cpp`) and switches the
  confidence recalibration off for it.

## 3. Parameters and defaults

| Parameter | Location | Default | Meaning |
|---|---|---|---|
| `GetFeatureNoiseScale()` | `SphericalCamera` | 2 | Pixel-threshold multiplier before angular conversion (pinhole = 1) |
| `cubemapFaces` | `FeaturesExtractor` config | 6 | Tangent-face count for feature extraction (4/6/8/12/20) |
| `cubemapFaceSize` | `FeaturesExtractor` config | 0 (auto = `max(1024, width/4)`) | Tangent-face resolution for feature extraction |
| `cubemapDedupAngleDeg` | `FeaturesExtractor` config | 0.25° | Angular non-max-suppression radius for cross-face duplicate keypoints |
| `sphericalExport` / `--spherical-export` | `ExportMVSConfig` / `CreateStructure` | `NATIVE` / `native` | Spherical cameras exported natively or as the cube-map bridge (`cubemap`) |
| `sphericalNumFaces` | `ExportMVSConfig` (`InterfaceMVS.h`) | 6 | Tangent-face count for the cube-map export (4/6/8/12/20; not exposed on any app's CLI) |
| `sphericalFaceSize` | `ExportMVSConfig` | 1024 px | Tangent-face resolution for the cube-map export (square; not exposed on any app's CLI) |
| `undistortImageDir` / `--undistort-extension` | `ExportMVSConfig` / `CreateStructure` | empty | Undistort the Brown cameras on export into this folder / with this image format, instead of exporting them as `BROWN` |
| `includeObservations` | `ExportMVSConfig` | false | Write the feature observation of every view (no CLI flag) |
| `UNDISTORT_ALPHA` | `libs/Math/Camera/CameraUndistort.h` | 0.6 | Free scaling of `K'` for the cameras undistorted on load (fixed, not an option) |
| `--camera-type` | `apps/ExtractKeyframes` | 0 (pinhole) | `0` = pinhole, `1` = spherical |
| `--cubemap-faces` | `apps/ExtractKeyframes` | 6 | Feature-extraction face count (does not affect MVS export) |

## 4. Invariants and constraints

- `SphericalCamera` requires `width == 2 * height` (equirectangular aspect ratio), enforced by an
  `ASSERT` in the constructor.
- Every equirectangular pixel ↔ direction mapping goes through `Spherical<T>`; no SFM code maps
  pixels to angles by hand. Keypoint columns therefore span `(-0.5, W-0.5]` (the seam is the
  meridian `θ = ±π`, at `u = -0.5 ≡ W-0.5`).
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
- `libs/MVS` keeps the camera model on `Platform::Camera` and `Image` (`model`, `lens`); the
  `MVS::Camera` is a `K`, R, C triple (the undistorted `K'` of a Brown camera, the angular `K` of a
  spherical one). Densification and the consumers of its depth-maps (fusion, filters, point colors and
  normals) build the `Spherical` model from `Image::model` and that `K` (`ModelCamera`,
  `MakeCameraModel`). Mesh reconstruction, refinement and texturing are pinhole only by decision:
  they contain no model dispatch and treat a spherical `K` as a pinhole one.
- `K'` of a Brown camera depends only on its lens and resolution (`UndistortedK`); the MVS loader,
  the SfM export and the depth-map undistortion all call that one function.

## 5. Validation of the shipped defaults

| Test | File | Checks |
|---|---|---|
| `TestCameraModels` | `apps/Tests/TestsMath.cpp` | `Spherical`: axis/seam/pole conventions exact to 1e-9 px, column 0 at longitude `-π+π/W`, `Ray`∘`Project` round trip ≤1e-8 px over random pixels, the seam columns, both hemispheres and the pole rows; Ceres Jet evaluation matches double values (1e-9 px) and central-difference derivatives (1e-5 relative) |
| `ReconstructSphericalSyntheticTest` | `apps/Tests/TestsSFM.cpp` | ≥95% of tracks recovered within 0.1 m of ground truth; mean reprojection ≤1.0 px; mean angular error ≤1.0°; same bounds post-BA; +X/-X exactly at 3/4 and 1/4 width (pixel-center convention) |
| `PairsMatcherSphericalTest` | `apps/Tests/TestsSFM.cpp` | Relative-pose rotation error ≤0.5°; translation direction similarity ≥0.99 |
| `MatchGeometricSphericalTest` | `apps/Tests/TestsSFM.cpp` | Rotation error ≤0.5°; translation similarity ≥0.99; ≥80% of tracked correspondences retained as inliers; requires ≥20 back-hemisphere matches to be meaningful |
| `PipelineTest` (spherical BA sub-test) | `apps/Tests/TestsSFM.cpp` | Mean reprojection error ≤1.0 px post-BA |
| `CubeMapBridgeDropTopBottomTest` | `apps/Tests/TestsSFM.cpp` | 4-face geometry shows 0 views for zenith/nadir points vs. 6-face geometry showing the expected face |
| `CubeMapFaceRenderTest`, `CubeMapBridgeGeometryTest`, `CubeMapBridgeEndToEndTest`, `CubeMapBridgeMVSLoadTest`, `CubeMapBridgeMixedSceneTest` | `apps/Tests/TestsSFM.cpp` | Cube-map rendering, geometry, MVS-export round trip, and mixed pinhole/spherical scenes (no numeric tolerances recorded beyond pass/fail) |
| `InterfaceV8Test` | `apps/Tests/TestsMVS.cpp` | Interface v8 round trip of every model (pinhole, Brown 5/8 coefficients, spherical) with and without observations; a v7 stream loads; v8 size = v7 + 1 byte per camera + 1; `MVS::Scene` keeps the Brown lens exactly and the observations within 1e-3 px; `K'` identical between `UndistortedK`, `UndistortMaps::Init`, the map cache and the loaded image camera |
| `BrownUndistortOnLoadTest` | `apps/Tests/TestsSynthetic.cpp` | Synthetic Brown views read through `MVS::Image` against the pinhole render at `K'`: mean abs difference 0.96 / 0.89 grey levels (bound 1.5; supersampled render vs bilinear remap), 4.61 % invalid pixels (bound 1-10 %), ignore mask equal to the undistortion mask, observation error 2.1e-5 px (bound 1e-3 px) |
| v7 load / re-save identity | `TransformScene --convert 1`, develop vs this code | `apps/Tests/data` `scene.mvs` and `reconstruct_test.mvs`, EPFL fountain-P11 and Herz-Jesu-P8, ETH3D pipes (Metashape import): content equal, v8 size = v7 + cameras + 1 byte (+3/+2/+2/+2/+2), the v8 file rewritten as v7 by `MvsUtils.py` byte-identical to the develop re-save |
| Native Brown export vs undistort-on-export | `CreateStructure` once, re-exported with `--undistort-extension .png`; `DensifyPointCloud --resolution-level 1 --max-resolution 3072`; in-house ETH3D scorer (Sim3 from the camera centres) | pipes (14/14 images): F1@2cm 0.7068 native / 0.7043 undistorted, F1@5cm 0.7800 / 0.7791; office (the SfM registers 6/26 images, 1.1° camera rotation error, so only the ICP-refined score is meaningful): F1@2cm 0.3106 / 0.3104, F1@5cm 0.3520 / 0.3526 |
| Undistortion cost on load | `Tests undistortbench`, RelWithDebInfo, ETH3D 6048x4032 JPEG | remap 10.5 ms per image over a 108.5 ms decode (pipes, 14 images; office 10.8 / 120.6 ms, 6 images), 34-43 ms once per camera to build the maps |

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
- The confidence recalibration of the depth-maps is not available for spherical scenes (off).
- ReconstructMesh, RefineMesh, TextureMesh and the Viewer do not check the camera model; with a
  spherical scene they treat the angular `K` as a pinhole one. Supporting spherical cameras there is a
  separate piece of work.
- The semi-global matcher does not apply the undistortion validity mask (the ignore mask) of the
  Brown images.
- The other Interface producers (InterfaceCOLMAP import, InterfaceMetashape, InterfacePolycam)
  still undistort on their own instead of passing the distortion through; InterfaceCOLMAP exports a
  Brown camera as `OPENCV`/`FULL_OPENCV`.
- Pose-uncertainty/covariance estimation is not validated for spherical cameras (see
  `docs/design/PoseUncertainty.md` §4).
