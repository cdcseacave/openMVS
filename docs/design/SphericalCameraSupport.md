# Spherical Camera Support in OpenMVS SfM

## 1. Purpose and scope

`libs/SFM` supports equirectangular (360°) still images and video as a first-class camera model through
sparse reconstruction: EXIF auto-detection, feature extraction, matching, pose estimation, triangulation
and bundle adjustment dispatch on `CameraType::SPHERICAL` alongside `PINHOLE`. The SFM to MVS export writes
spherical cameras natively (Interface v8, `SPHERICAL` model); `libs/MVS` loads and saves them and
`DensifyPointCloud` estimates and fuses their depth-maps natively (`SphericalPatchMatch.md`, which also lists
the stages that refuse a spherical scene). The camera math is in `CameraModels.md`. The cube-map bridge,
which renders each spherical image to N tangent pinhole faces that the whole MVS pipeline consumes as
pinhole images, stays available as an export option.

## 2. Algorithm as implemented

### Camera wrapper — `libs/SFM/Camera.h/.cpp`

`SphericalCamera : public Camera` is a thin wrapper over `Spherical<T>` (`GetModel()`) converting `TPoint`
and Eigen at the boundary. Two unprojection forms exist: `Unproject(x)` returns the bearing scaled to
`|z| = 1` (the pinhole-normalized plane, divergent where the bearing is orthogonal to the optical axis);
`UnprojectNormalized(x)` returns the unit bearing, valid over the whole sphere. `PixelErrorToAngular`
converts a pixel threshold to radians. `GetFeatureNoiseScale()` returns 2 (cube-face SIFT has about twice
the pixel-space localization noise of a pinhole pipeline). `GetK()` is identity; `TrustIntrinsics()` is true.

Every SFM caller that needs whole-sphere correctness (triangulation, track filtering, pose estimation,
matching) uses `UnprojectNormalized`. `View::Ray()` and `View::UnprojectPoint()` (`libs/SFM/View.h`) use the
2D form; `View::RayNormalized()` is the spherical-safe sibling.

### Feature extraction — `FeaturesExtractor::ExtractImage` / `ExtractImageSpherical`

A spherical image is rendered to `config.cubemapFaces` tangent pinhole faces (`cubemapFaceSize`, 0 =
`max(1024, equirect_width/4)`) by `SphereCubeMap::MakeTangentFacesGeometry` +
`SphericalToTangentialFaces` (`CameraModels.md`). Each face is run through the normal `ExtractImage` path
as a synthetic pinhole `Image`. Keypoints are reprojected to equirectangular pixels (face pixel, bearing in
the face frame, body frame, `SphericalCamera::Project`); the keypoint `octave` field records the source face
ID. Duplicates straddling a face seam are suppressed by angular non-max-suppression over unit bearings (a 3D
octree, radius `cubemapDedupAngleDeg`), keeping the highest response.

### Matching and pose estimation

- `PairsMatcher::GeometricFilter` (`libs/SFM/PairsMatcher.cpp`): relative pose from bearing vectors by PoseLib's
  bearing-vector RANSAC (`docs/design/PoseLibBearingVector.md`). `pair.F` is composed only when both cameras
  are pinhole; `pair.E` is set instead for spherical or mixed pairs. Consumers check `pair.F.has_value()`
  and use the `pair.E` / bearing path otherwise.
- `MatchGeometric.cpp`: descriptor-guided matching branches on `pair.F` vs `pair.E`. Pinhole pairs keep the
  pixel-space epipolar-line distance; spherical or mixed pairs precompute unit bearings and score candidates
  with the Sampson-on-sphere residual `r² = (b2·(Eb1))² / (‖(Eb1)_xy‖² + ‖(Eᵀb2)_xy‖²)` against an angular
  threshold derived per camera by `PixelErrorToAngular`, averaged across the pair.
- `StarInitializer` and `ImagePair::FilterMatches` triangulate and filter on unit bearings with angular
  reprojection and epipole-proximity gates, one code path for both models.
- `PairsWeighting::ComputeIntrinsicWeight` bins spherical keypoints by equal-solid-angle cells (azimuth
  `atan2(b.x, b.z)` and `b.y`, wrapping the seam) instead of the uniform pixel grid; the rest of pair
  weighting is camera-agnostic.

### Triangulation and track filtering

`TriangulateTracks` calls `TriangulateSkewLLS` for every track: it works on bearings, so no per-camera
branch. `TriangulateDLT` is pinhole-only (ASSERTed) and unused. `ComputeTracksMeanReprojectionError` and
`FilterTracks` use `UnprojectNormalized` bearings gated by `PixelErrorToAngular`, so back-hemisphere
observations are neither misreported nor discarded as outliers.

### Bundle adjustment — `libs/SFM/BundleAdjustment.cpp`, `BundleAdjustmentCostFunctions.h`

`AddReprojectionResidual` dispatches per `img.GetCameraType()`. Pinhole observations get
`PinholeReprojectionError` (pose, 12-parameter intrinsics, point). Spherical observations get
`SphericalAngularReprojectionError` (pose and point only): a spherical camera has no intrinsics block, so
BA refines nothing of the spherical model. The functor precomputes a tangent-plane basis on the unit sphere
at the observed bearing (angles from `Spherical<double>::Angles`, the same pixel-center convention as
`Project`), pre-scaled by `fx = width/(2π)` and `fy = height/π` so residuals are pixel-equivalent. It has no
cheirality check and returns the two tangent-plane dot products rather than an `acos` angle, for numerical
stability near zero.

### EXIF auto-detection and keyframes

`Image.cpp` sets `isSpherical` from EXIF `ProjectionType == 2` and constructs a `SphericalCamera(cv::Size(w,
h))`. `KeyframeExtractor::ExtractFromVideo` dispatches on `config.cameraType`; `apps/ExtractKeyframes`
exposes `--camera-type 0|1` and `--cubemap-faces` (feature-extraction face count only).

### MVS export — `libs/SFM/InterfaceMVS.cpp`

`SFM::ExportMVS` writes an Interface v8 project (`libs/IO/Interface.h`, `docs/wiki/Interface.md`), one
platform with one camera per SFM camera and orientation of its image files: the SfM rotates a portrait
file to landscape on load (`View::IsRotated`), so the portrait and landscape images of a camera share it,
and the export gives the portrait ones a platform of their own whose camera is rotated back to their
on-disk orientation (`View::RevertRotation` for `K`; `p1' = -p2`, `p2' = p1`; the depth-map undistortion of
`UndistortDepthMaps` keys its maps the same way):

- Pinhole without distortion: `PINHOLE`, the original images.
- Pinhole with distortion: `BROWN` (`k1,k2,p1,p2,k3`, plus `k4,k5,k6` for the rational model) and the original
  image paths; no image is written. `libs/MVS` undistorts on
  load (`CameraModels.md`). Only with `ExportMVSConfig::undistortImageDir` (`CreateStructure
  --undistort-extension <ext>`) are images undistorted on export by `Scene::UndistortImages` and written as
  `PINHOLE` cameras of `UndistortedK(K, distortion, size, undistortAlpha)`.
- Spherical, `NATIVE` (default, `--spherical-export native`): `SPHERICAL` with the angular `K` of
  `Spherical<REAL>::FromSize(W, H)` and the original image paths.
- Spherical, `CUBEMAP` (`--spherical-export cubemap`): the bridge below.

Observations are written when `ExportMVSConfig::includeObservations` is set: the keypoint of every view in
the exported image file (through `UndistortedK` for an undistorted file, in the on-disk orientation for a
rotated one, through the feature ray projected into each face for cube-map faces). The export fails if a
track observation has no keypoint.

The cube-map bridge builds one `SphereCubeMap::TangentFacesGeometry` for `(sphericalNumFaces,
sphericalFaceSize)` (the face count must be 4, 6, 8, 12 or 20, `IsSupportedNumFaces`, else the export
fails), then runs four file-local helpers of `InterfaceMVS.cpp`:
1. `EmitSphericalPlatforms`: one MVS `Platform` per spherical SFM camera with N face `Camera` records.
2. `AppendSphericalFaceImages`: N `Interface::Image` records per source image, named `<stem>_face<k><ext>`,
   with IDs from a synthetic counter past the maximum SFM image ID.
3. `RenderAndSaveSphericalFaces`: renders the faces and writes them to disk.
4. `ProjectTrackOntoSphericalFaces`: each 3D track point gets a `Vertex::View` in every face whose camera
   has it in front.

`UndistortDepthMaps` (`InterfaceMVS.cpp`, the ROMA2 depth-maps of `CreateStructure`) is pinhole-only. It
writes each depth-map in the frame of the undistorted camera of its image, `UndistortedK` scaled to the
depth-map resolution, and stores that `K'`: `alpha` is `UNDISTORT_ALPHA` for a native export and
`--undistort-alpha` for an undistorted one. It uses `UndistortMaps` (`CameraModels.md`).

### MVS load

`Scene::LoadInterface` keeps the camera model; `BROWN` is undistorted on load (`CameraModels.md`).
`SPHERICAL` is loaded and saved as is; `Scene::HasSphericalImages` drives the refusals and switches
described in `SphericalPatchMatch.md` §1. `MvgMvsPipeline.py` exports cube-maps when it runs a mesh step.

### Viewer

- A spherical camera is drawn as a globe (`AppendCameraGlyphLines`, `apps/Viewer/Renderer.cpp`): the great
  circles of its equator (the image row of elevation 0) and of the meridians of longitude 0/180 and -90/90
  degrees, at half the frustum depth, with the same look-at and up indicator as a pinhole camera.
- Camera view mode (`Camera::SetCameraFromSceneData`): the view is placed at the image center, looking along
  the image axis with its own field of view, the navigation target close enough to look around in place.
  The image overlay is the panorama on the sphere of its rays (`Renderer::SetupImageOverlayBuffers`,
  `RenderImageOverlays`): one unit sphere with the image positions as texture coordinates, placed at the
  image pose by the overlay shader's `model` matrix, only its far side drawn. Seen from the center every
  scene point lies on its pixel. Once loaded, a panorama stays on its globe outside the camera view mode
  too: the camera shows as a ball textured with it, depth tested with the scene.
- Point projections (selection info, track-based neighbors, Point Observations) go through `ModelCamera`.
  A Point Observations patch of a panorama continues across the seam and, mirrored, across a pole
  (`CropPatch`, `apps/Viewer/PointObservations.cpp`).
- `--screenshot-show i` with `--view-camera` captures the camera view mode with its image overlay.

## 3. Parameters and defaults

| Parameter | Location | Default | Meaning |
|---|---|---|---|
| `GetFeatureNoiseScale()` | `SphericalCamera` | 2 | pixel-threshold multiplier before angular conversion (pinhole 1) |
| `cubemapFaces` | `FeaturesExtractor` config | 6 | tangent-face count for feature extraction (4/6/8/12/20) |
| `cubemapFaceSize` | `FeaturesExtractor` config | 0 (auto) | tangent-face resolution for feature extraction |
| `cubemapDedupAngleDeg` | `FeaturesExtractor` config | 0.25° | angular NMS radius for cross-face duplicates |
| `sphericalExport` / `--spherical-export` | `ExportMVSConfig` / `CreateStructure` | `NATIVE` / `native` | native cameras or the cube-map bridge (`cubemap`) |
| `sphericalNumFaces`, `sphericalFaceSize` | `ExportMVSConfig` | 6, 1024 px | cube-map export faces (no CLI flag; Python bindings and tests only) |
| `undistortImageDir` / `--undistort-extension` | `ExportMVSConfig` / `CreateStructure` | empty | undistort Brown cameras on export instead of exporting `BROWN` |
| `undistortAlpha` / `--undistort-alpha` | `ExportMVSConfig` / `CreateStructure` | 0.6 | free scaling of the undistorted export |
| `includeObservations` | `ExportMVSConfig` | false | write the feature observation of every view (no CLI flag) |
| `--camera-type` | `apps/ExtractKeyframes` | 0 | 0 pinhole, 1 spherical |
| `--cubemap-faces` | `apps/ExtractKeyframes` | 6 | feature-extraction face count |

## 4. Invariants and constraints

- `SphericalCamera` requires `width == 2 * height` (ASSERT in the constructor).
- Every pixel/direction mapping goes through `Spherical<T>`; no SFM code maps pixels to angles by hand.
  Keypoint columns span `(-0.5, W-0.5]` (the seam is `θ = ±π`, `u = -0.5 ≡ W-0.5`).
- Whole-sphere-correct callers use `UnprojectNormalized`; the 2D `Unproject()` is valid only in the front
  hemisphere and must not feed RANSAC, DLT or angle computations.
- `pair.F` is populated only when both cameras of a pair are pinhole.
- BA never adds an intrinsics block for a spherical observation.
- Pose-uncertainty export (`docs/design/PoseUncertainty.md`) is validated for pinhole cameras only.

## 5. Validation

| Test | File | Checks |
|---|---|---|
| `ReconstructSphericalSyntheticTest` | `apps/Tests/TestsSFM.cpp` | ≥95 % of tracks within 0.1 m of ground truth; mean reprojection ≤1.0 px; mean angular error ≤1.0°, also after BA; +X/-X at 3/4 and 1/4 width |
| `PairsMatcherSphericalTest` | `TestsSFM.cpp` | relative-pose rotation error ≤0.5°; translation direction similarity ≥0.99 |
| `MatchGeometricSphericalTest` | `TestsSFM.cpp` | rotation error ≤0.5°; translation similarity ≥0.99; ≥80 % of correspondences kept as inliers (needs ≥20 back-hemisphere matches) |
| `PipelineTest` (spherical BA sub-test) | `TestsSFM.cpp` | mean reprojection error ≤1.0 px after BA |
| `CubeMapBridgeDropTopBottomTest` | `TestsSFM.cpp` | 4-face geometry shows 0 views for zenith/nadir points, 6-face the expected face |
| `CubeMapFaceRenderTest`, `CubeMapBridgeGeometryTest`, `CubeMapBridgeEndToEndTest`, `CubeMapBridgeMVSLoadTest`, `CubeMapBridgeMixedSceneTest` | `TestsSFM.cpp` | cube-map rendering, geometry, MVS round trip, mixed pinhole/spherical scenes (pass/fail) |
| `InterfaceV8Test`, `BrownUndistortOnLoadTest` | `TestsMVS.cpp`, `TestsSynthetic.cpp` | the Interface v8 round trip and the Brown undistort-on-load (`CameraModels.md`) |
| Native Brown export vs undistort-on-export, ETH3D F1@2cm (pipes, office) | `CreateStructure`, `DensifyPointCloud --resolution-level 1 --max-resolution 3072` | 0.7068 vs 0.7043 (pipes), 0.3106 vs 0.3104 (office, ICP-refined) |

## 6. Rejected alternatives

- Stereographic projection from the back pole as the 2D unprojection: still singular at the back pole and
  changes the return semantics for pinhole callers.
- Returning raw `(θ, φ)`: pinhole callers would misread it as normalized-plane coordinates.
- Deleting `Camera::Unproject`: a refactor across every pinhole caller, not worth it.

## 7. Open items

- `View::Ray()` and `View::UnprojectPoint()` still use the front-hemisphere 2D `Unproject()`; whether a live
  caller invokes them on a spherical image is not audited.
- The semi-global matcher does not apply the undistortion validity mask (the ignore mask) of Brown images.
- InterfaceCOLMAP handles pinhole and Brown cameras only (COLMAP has no equirectangular model);
  InterfacePolycam undistorts on its own.
- Pose-uncertainty estimation is not validated for spherical cameras (`PoseUncertainty.md` §4).
- Mesh stages do not support spherical cameras (a 360 dataset meshes through `--spherical-export cubemap`).
- The Viewer navigation in the camera view mode of a panorama turns the view about a target close to the
  image center, not exactly in place; the first-person controls do.
