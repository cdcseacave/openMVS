# Spherical Cameras

## 1. Purpose and scope

`libs/SFM` supports equirectangular (360 degree) still images and video frames as a camera model next to the pinhole,
through sparse reconstruction: metadata detection, feature extraction, matching, pose estimation, triangulation and bundle
adjustment all dispatch on `CameraType::SPHERICAL`. Every geometric computation that must hold over the whole sphere works
on 3D unit bearing vectors, which is the design of section 7 (bearing-vector pose estimation). The SfM to MVS export
writes spherical cameras natively (Interface v8, `SPHERICAL` model); `libs/MVS` loads and saves them and
`DensifyPointCloud` estimates and fuses their depth-maps natively (`SphericalPatchMatch.md`, which also lists the stages
that refuse a spherical scene). The camera math is in `CameraModels.md`. The cube-map bridge, which renders each spherical
image to tangent pinhole faces that the whole MVS pipeline consumes as pinhole images, is an export option.

## 2. Camera model and consequences

`SphericalCamera` (`libs/SFM/Camera.h/cpp`) is a thin wrapper over `Spherical<T>` (`libs/Math/Camera/CameraModels.h`;
`GetModel()` returns `Spherical<REAL>::FromSize(width, height)`), converting `TPoint` and Eigen at the boundary. It is the
equirectangular projection with the Y-down convention of the pinhole: longitude `theta = atan2(x, z)` maps to the
horizontal axis, elevation `phi = asin(-y / r)` to the vertical one. Pixel centres lie at integer coordinates; longitude 0
is at column `(width - 1) / 2` and elevation 0 at row `(height - 1) / 2`, with `width / (2 pi)` and `height / pi` pixels
per radian. The camera has no intrinsics (`GetK()` is the identity), always trusts its intrinsics, and requires
`width == 2 * height` (asserted by the constructor). It is created from the EXIF projection tag (`ProjectionType == 2`,
`Image.cpp`) or by `--camera-type 1` (`ImportAndCameras.md`).

- `UnprojectNormalized(x)` returns the unit bearing `Spherical<REAL>::Ray(x)`, `(cos phi sin theta, sin phi,
  cos phi cos theta)`, without singularity over the whole sphere. `Unproject(x)` is derived from it: the same direction
  scaled so that `|z| = 1` with the sign of `cos theta`; it is singular at the equator sides (`cos theta = 0`) and the
  poles and is not used for estimation.
- `PixelErrorToAngular` converts a pixel threshold to radians (`2 pi / width` per pixel); `GetFeatureNoiseScale()` is 2
  (pinhole 1): features from the cube faces localize about twice as noisily in pixels (face-seam sampling and off-centre
  descriptor warping), and thresholds calibrated on pinhole pipelines are widened by it (`AngularError`, `Link2D3D.h`).
- Every caller that needs whole-sphere correctness (triangulation, track filtering, pose estimation, matching) goes
  through `UnprojectNormalized`. `View::Ray()` and `View::UnprojectPoint()` (`libs/SFM/View.h`) use the 2D form.

**Two-view verification and lens.** A pair of spherical cameras takes the calibrated (ESSENTIAL) branch of
`GeometricFilter` on bearings and carries E only: `pair.F` is composed only when both cameras are pinhole, because `GetK()`
is the identity. Spherical cameras are excluded from the lens fit and from the view-graph calibration (they have no radial
lens model and are always trusted). See `TwoViewVerificationAndLens.md`.

**Matching.** Descriptor-guided matching branches on `pair.F` against `pair.E`: pinhole pairs keep the pixel-space
epipolar-line test; spherical or mixed pairs precompute unit bearings once and score candidates with the Sampson-on-sphere
residual `r^2 = (b2.(E b1))^2 / (|(E b1)_xy|^2 + |(E^T b2)_xy|^2)` against an angular threshold, derived per camera with
`PixelErrorToAngular` and averaged over the pair.

**Pair weight.** The spatial coverage term bins spherical keypoints in equal-solid-angle cells (azimuth `atan2(b.x, b.z)`
and `sin(latitude)`, wrapping the seam) instead of the pixel grid; the rest of the weight is camera-agnostic
(`PairWeight.md`).

**Triangulation and tracks.** `TriangulateTracks` calls `TriangulateSkewLLS` for every track whatever the camera type; it
works on unit bearings and weighs every direction of a ray the same, a ray 90 degrees off the forward axis included
(`TrackBuilding.md`). `TriangulateDLT` mixes a 2D pinhole-normalized coordinate with a 3 x 4 projection matrix, asserts
pinhole cameras, and is called by the tests only, not by any production code. `ComputeTracksMeanReprojectionError` and
`FilterTracks` use bearings gated by `PixelErrorToAngular`, so back-hemisphere observations are neither misreported nor
silently discarded (`TrackBuilding.md`).

**Bundle adjustment.** A spherical observation gets `SphericalAngularReprojectionError` with pose and point blocks only:
no intrinsics block exists for a spherical camera, so nothing of the model is refined (`BundleAdjustment.md`). The
functor precomputes a tangent-plane basis on the unit sphere at the observed bearing (angles from
`Spherical<double>::Angles`, the same pixel-centre convention as `Project`), pre-scaled by `fx = width / (2 pi)` and
`fy = height / pi` so residuals are in pixel-equivalent units, has no cheirality test, and returns the two tangent-plane
dot products rather than the `acos` angle, for numerical stability near zero residual.

## 3. Feature extraction: cube-map faces and the bottom band

`FeaturesExtractor::ExtractImageSpherical` renders the equirectangular image to `cubemapFaces` tangent pinhole faces
(`SphereCubeMap::MakeTangentFacesGeometry`, `SphericalToTangentialFaces`, `libs/Math/Camera/CameraUndistort.h/cpp`), runs
the normal pinhole `ExtractImage` on each face (every detector, the 3 x 3 grid, RootSIFT and the SiftGPU dispatch are
reused, `FeatureExtraction.md`), reprojects the keypoints to equirectangular pixels (face pixel to bearing in the face
frame, to the body frame by the face rotation, then `SphericalCamera::Project`), and records the source face in the
keypoint's `octave` field. Keypoints duplicated across a face seam are suppressed by angular non-maximum suppression over
unit bearings (a 3D octree ball query of chord radius set by `cubemapDedupAngleDeg`), keeping the highest-response
keypoint per neighbourhood. Face counts 4, 6, 8, 12 and 20 are supported (4 is equatorial and sees no zenith or nadir);
any other count fails extraction.

**Bottom band.** A keypoint whose equirectangular pixel lies in the bottom `sphericalBottomBand` fraction of the image
height is dropped before the seam suppression, so it neither becomes a feature nor suppresses a neighbour. The operator,
the tripod and the ground right under the camera sit there and give wrong, look-alike or moving matches. The decision is
the single predicate `Image::IsInSphericalBottomBand(pt, band)`, read by `FeaturesExtractor::IsMasked`. Detection still
runs on the whole face, so the rows above the band keep exactly the features they had. Perspective images never reach
the predicate.

The dense matchers (`DenseMatching.md`) honour the same band through the same predicate: a warp cell is eligible only
when neither its end in the spherical source image nor its end in the spherical target image lies in the band
(`IsWarpCellEligible`), so no dense keypoint is created in the band and no dense match has an end in it. The value is
`DenseMatchConfig::sphericalBottomBand`, which `Scene::Reconstruct` copies from `FeatureExtractionConfig::sphericalBottomBand`
(the scene stores no band, so on a reloaded scene whose features exist the dense matching takes the band of the
current configuration, while the stored features keep the filter they were extracted with). A perspective end is never
filtered, so a spherical and a perspective image lose only the spherical end's band.

Spherical images are decoded inline on the extraction thread, not prefetched: they are decoded in colour and split into
faces. `KeyframeExtractor::ExtractFromVideo` dispatches on `config.cameraType`; `apps/ExtractKeyframes` exposes
`--camera-type 0|1` and `--cubemap-faces` (the feature-extraction face count) for video input.

## 4. MVS export

`SFM::ExportMVS` (`libs/SFM/InterfaceMVS.cpp`) writes an Interface v8 project (`libs/IO/Interface.h`,
`docs/wiki/Interface.md`), one platform with one camera per SfM camera and orientation of its image files: the SfM
rotates a portrait file to landscape on load (`View::IsRotated`), so the portrait and landscape images of a camera share
it, and the export gives the portrait ones a platform of their own whose camera is rotated back to their on-disk
orientation (`View::RevertRotation` for `K`; `p1' = -p2`, `p2' = p1`; the depth-map undistortion of `UndistortDepthMaps`
keys its maps the same way):

- Pinhole without distortion: `PINHOLE`, the original images.
- Pinhole with distortion: `BROWN` (`k1, k2, p1, p2, k3`, plus `k4, k5, k6` for the rational model) and the original
  image paths; no image is written. `libs/MVS` undistorts on load (`CameraModels.md`). Only with
  `ExportMVSConfig::undistortImageDir` (`CreateStructure --undistort-extension <ext>`, which writes into the folder
  `undistorted`) are images undistorted on export by `Scene::UndistortImages`, written as `<index>_<stem><ext>` (the SfM
  image index keeps equal stems from different folders apart), and exported as `PINHOLE` cameras of
  `UndistortedK(K, distortion, size, undistortAlpha)`; the export fails if a posed image can not be undistorted.
- Spherical, `NATIVE` (default, `--spherical-export native`): `SPHERICAL` with the angular `K` of
  `Spherical<REAL>::FromSize(W, H)` and the original image paths.
- Spherical, `CUBEMAP` (`--spherical-export cubemap`): the bridge below.

Observations are written when `ExportMVSConfig::includeObservations` is set: the keypoint of every view in the exported
image file (through `UndistortedK` for an undistorted file, in the on-disk orientation for a rotated one, through the
feature ray projected into each face for cube-map faces). The export fails if a track observation has no keypoint.

The cube-map bridge builds one `SphereCubeMap::TangentFacesGeometry` for `(sphericalNumFaces, sphericalFaceSize)` (the
face count must be 4, 6, 8, 12 or 20, `IsSupportedNumFaces`, else the export fails). Three helpers in `InterfaceMVS.cpp`
share that geometry:

1. `EmitSphericalPlatforms`: one MVS platform per spherical SfM camera with one calibrated camera per face, sharing the
   geometry's intrinsics and per-face rotations.
2. `ExportSphericalFaces`: renders the face files and registers their paths as `cube<ID>_<stem>_face<k><ext>`, with IDs
   from a counter past the largest SfM image ID (pinhole images keep their IDs). The faces share the source pose.
3. `ProjectTrackOntoSphericalFaces`: each 3D track point adds a view for every face that contains its feature ray or
   projected point.

No command-line option of `CreateStructure` or another app sets the bridge's face count or size; only the Python bindings
and a hand-built `ExportMVSConfig` can.

`UndistortDepthMaps` remaps pinhole-family depth maps to the exported image frame. Native Brown exports use the on-disk
orientation and `UNDISTORT_ALPHA`; undistorted exports use the working orientation and `--undistort-alpha`. The header
stores intrinsics at image resolution; the pixel remapping uses those intrinsics scaled to the depth-map resolution.

## 5. MVS load

`Scene::LoadInterface` keeps the camera model; `BROWN` is undistorted on load (`CameraModels.md`). `SPHERICAL` is loaded
and saved as is; `Scene::HasSphericalImages` drives the refusals and switches described in `SphericalPatchMatch.md`
section 1. `scripts/python/MvgMvsPipeline.py` exports cube-maps when it runs a mesh step.

## 6. Viewer

- A spherical camera is drawn as a globe (`AppendCameraGlyphLines`, `apps/Viewer/Renderer.cpp`): the great circles of its
  equator (the image row of elevation 0) and of the meridians of longitude 0/180 and -90/90 degrees, at half the frustum
  depth, with the same look-at and up indicator as a pinhole camera.
- Camera view mode (`Camera::SetCameraFromSceneData`): the view is placed at the image centre, looking along the image
  axis with its own field of view, the navigation target close enough to look around in place. The image overlay is the
  panorama on the sphere of its rays (`Renderer::SetupImageOverlayBuffers`, `RenderImageOverlays`): one unit sphere with
  the image positions as texture coordinates, placed at the image pose by the overlay shader's `model` matrix, only its
  far side drawn. Seen from the centre every scene point lies on its pixel. Once loaded, a panorama stays on its globe
  outside the camera view mode too: the camera shows as a ball textured with it, depth tested with the scene.
- Point projections (selection info, track-based neighbours, Point Observations) go through the image cameras
  (`MVS::Camera`, model-aware). A Point Observations patch of a panorama continues across the seam and, mirrored, across a
  pole (`CropPatch`, `apps/Viewer/PointObservations.cpp`).
- `--screenshot-show i` with `--view-camera` captures the camera view mode with its image overlay.

## 7. Bearing-vector pose estimation

Absolute (PnP) and relative pose estimation run on 3D unit bearing vectors (`Camera::UnprojectNormalized`), not on 2D
normalized-plane coordinates, so that a camera whose field of view exceeds a hemisphere never loses the sign that a 2D
projection discards. The robust estimation itself is the vendored PoseLib (vcpkg package `poselib`): its native
bearing-vector estimators `poselib::estimate_absolute_pose_bearings` and `poselib::estimate_relative_pose_bearings`
(`PoseLib/robust.h`) are called directly, with no wrapper layer. For pinhole cameras the bearing path is used too
(`UnprojectNormalized` reduces to `normalize((x/z, y/z, 1))`), so one pose-estimation path serves every camera type.

Both run LO-RANSAC over the same minimal solvers as the pixel-space estimators (P3P for absolute pose, the generalized
5-point solver for relative pose), score candidates on the unit sphere (chord-distance squared for absolute; unit-norm
symmetric Sampson-on-sphere for relative), and refine the winner with the matching bearing-native bundle routine inside
every call. `AbsolutePoseOptions` and `RelativePoseOptions` carry a single `max_error`, an angle in **radians** for the
bearing estimators (converted internally to chord distance or `sin(angle)`). Cheirality is bearing-native: absolute pose
requires `b_pred . b_obs > 0`; relative pose (`check_cheirality`, on) requires the midpoint-triangulation parameter along
each ray to be positive. Both accept back-hemisphere observations; without cheirality the four decompositions of the
essential matrix score identically and RANSAC keeps an arbitrary one.

| call site | use |
|---|---|
| `Resection::RegisterImage` | PnP of an image against the structure: bearings of the inlier-track observations, threshold `PixelErrorToAngular(ransac.threshold * GetFeatureNoiseScale())` |
| `PairsMatcher::GeometricFilter`, ESSENTIAL branch | relative pose: the pixel bar converted per camera by `PixelErrorToAngular` and averaged over the pair; seeded with an existing pose when present |
| `PairsMatcher::RefineRelativePose` | weighted bearing polish of a relative pose (`refine_relpose_bearing`) |
| `GlobalAlignment` (block merge) | `estimate_generalized_absolute_pose_scale_bearings`: a rig of one block's cameras against another block's points, solving rotation, translation and scale from bearings alone; the scale is observable only from points seen out of at least two distinct rig centres (`HierarchicalSFM.md`) |

Consumers that use bearings directly rather than through PoseLib: `StarInitializer` (midpoint triangulation of
bearings, angular reprojection gate), `ImagePair::FilterMatches` (epipole proximity and reprojection on bearings;
cheirality is the sign of the bearing dot product) and the guided matching of `MatchGeometric.cpp` (the Sampson-on-sphere
residual above, a local re-implementation kept separate from the F path).

| bearing-estimation parameter | default | meaning |
|---|---|---|
| angular inlier threshold | `PixelErrorToAngular(px * noise scale)` | radians; relative pose averages the two cameras' |
| `GetFeatureNoiseScale()` | pinhole 1, spherical 2 | multiplier on pixel thresholds before the angular conversion |
| `check_cheirality` | true | resolves the four decompositions; valid for back-hemisphere features |
| `ResectionConfig::minInliers`, `minCorrespondences` | 12, 15 | PnP acceptance (the inliers weighed with a dense one at `PairsWeightingConfig::DenseScale`, 1/40) |
| `MatchConfig::minMatches` | 50 AKAZE/ORB, 15 SIFT/SIFTGPU | pair acceptance |

For pinhole cameras the bearing estimators equal the pixel-space ones to first order; they differ at third order in the
error, so the pinhole results are unchanged.

## 8. Parameters and defaults

| Parameter | Location | Default | Meaning |
|---|---|---|---|
| `cubemapFaces` | `FeatureExtractionConfig` | 6 | tangent-face count for feature extraction (4, 6, 8, 12, 20) |
| `cubemapFaceSize` | `FeatureExtractionConfig` | 0 (auto: `max(1024, width / 4)`) | tangent-face resolution for feature extraction |
| `cubemapDedupAngleDeg` | `FeatureExtractionConfig` | 0.25 deg | angular non-maximum-suppression radius across face seams |
| `sphericalBottomBand` | `FeatureExtractionConfig`, `CreateStructure --spherical-bottom-band` | 0.2 | fraction of the image height, from the bottom, discarded from the features and the dense matches (`DenseMatchConfig::sphericalBottomBand`, set from it); in [0,1), 0 = off |
| `sphericalExport` | `ExportMVSConfig`, `CreateStructure --spherical-export` | `NATIVE` (`native`) | native `SPHERICAL` cameras or the cube-map bridge (`cubemap`) |
| `sphericalNumFaces` | `ExportMVSConfig` | 6 | face count for the cube-map export (4, 6, 8, 12, 20); no command-line option |
| `sphericalFaceSize` | `ExportMVSConfig` | 1024 px | face resolution for the cube-map export; no command-line option |
| `undistortImageDir` | `ExportMVSConfig`, `CreateStructure --undistort-extension` | empty | undistort the Brown cameras' images on export instead of exporting `BROWN` cameras of the original images |
| `undistortAlpha` | `ExportMVSConfig`, `CreateStructure --undistort-alpha` | 0.6 | free scaling of the undistorted export |
| `includeObservations` | `ExportMVSConfig` | false | write the feature observation of every view; no command-line option |
| `--camera-type`, `--cubemap-faces` | `apps/ExtractKeyframes` | 0, 6 | video input camera model (0 pinhole, 1 spherical) and extraction face count |

`CreateStructure` exposes the bottom band and the spherical export mode but not the extraction face count or size.

## 9. Invariants and constraints

- `SphericalCamera` requires `width == 2 * height`. `Image::LoadMetadata` fails for a spherical image of another shape
  with an error naming it, and the import keeps that image without a camera.
- Every pixel/direction mapping goes through `Spherical<T>`; no SfM code maps pixels to angles by hand. Keypoint columns
  span `(-0.5, W-0.5]` (the seam is `theta = +-pi`, `u = -0.5` is `u = W-0.5`).
- Estimation inputs are unit bearings from `UnprojectNormalized` only; `Unproject()` must never feed RANSAC, DLT or angle
  computations.
- `pair.F` exists only for a pair of pinhole cameras; every consumer checks `pair.F.has_value()` and otherwise uses
  `pair.E` on bearings.
- A bearing estimator's `max_error` is an angle in radians; pixel thresholds are converted per camera first.
- No spherical-model parameter (lens, orientation offset) is refinable by bundle adjustment.
- Pose-quality export (`PoseUncertainty.md`) is validated for pinhole cameras only.

## 10. Validation

Tests: `ReconstructSphericalSyntheticTest` (`apps/Tests/TestsSFMBundleAdjustment.cpp`: at least 95% of the tracks within
0.1 m of the truth, mean reprojection error at most 1.0 px and mean angular error at most 1.0 degree before and after the
bundle adjustment, +X at 3/4 and -X at 1/4 of the width, no left-right mirror), `BundleAdjustmentBasicsTest` (same file:
the spherical solve within 1.0 px), `SphericalPairGeometryTest` (`TestsSFMFeatures.cpp`: relative pose within 0.5 degrees
and translation direction within `acos(0.99)`; the guided matching leaves F empty, keeps 80% of the tracked
correspondences and admits back-hemisphere inliers), `SphericalBottomBandTest` (`TestsSFMFeatures.cpp`) and
`DenseWarpSamplingTest` part 4 (`TestsSFMDense.cpp`) for the bottom band, `CameraTypeImportTest` (`TestsSFMImport.cpp`: a
non-2:1 spherical image refused), `CubeMapExportTest` (`TestsSFMPipeline.cpp`: face rendering, the rig platform, the MVS
load round trip, a mixed pinhole and spherical export, the 4-face export without the zenith and nadir tracks),
`UndistortDepthMapsTest` (`TestsSFMPipeline.cpp`), `InterfaceV8Test` (`TestsMVS.cpp`, the Interface v8 round trip) and
`BrownUndistortOnLoadTest` (`TestsSynthetic.cpp`, the Brown undistort-on-load, `CameraModels.md`).

Native Brown export against undistort-on-export, ETH3D F1 at 2 cm (`CreateStructure`, then `DensifyPointCloud
--resolution-level 1 --max-resolution 3072`): 0.7068 against 0.7043 on pipes, 0.3106 against 0.3104 on office
(ICP-refined).

Bottom band 0.2 against 0 on Holo360D panoramas (`--camera-type 1`), SIFT matching, full runs from images at 8 threads,
scored on the LiDAR ground truth (completeness = share of trusted images posed within the bar, accuracy = mean pose score
of those, F1 of the two). Outdoor_004: three runs per arm (the three agree to 0.0003 F1); Indoor_044: one run per arm,
whose F1 difference (0.0018) is below the 0.008 noise of comparing different matchings.

| Capture, band | registered / right / wrong | completeness | accuracy | F1 | keypoints (removed by the band) |
|---|---|---|---|---|---|
| Outdoor_004, 0.2 | 777 / 777 / 0 of 824 | 0.943 | 0.586 | 0.7228-0.7231 | 69.58 M (4.21 M, 5.7%) |
| Outdoor_004, 0 | 767 / 767 / 0 of 824 | 0.931 | 0.586 | 0.7190 | 73.79 M |
| Indoor_044, 0.2 | 1087 / 1087 / 0 of 1087 | 1.000 | 0.553 | 0.7120 | 46.35 M (5.16 M, 10.0%) |
| Indoor_044, 0 | 1087 / 1087 / 0 of 1087 | 1.000 | 0.555 | 0.7138 | 51.51 M |

## 11. Rejected alternatives

- Stereographic projection from the back pole as the 2D unprojection: still singular at the pole and changes the return
  semantics for pinhole callers.
- Returning raw `(theta, phi)` from the 2D unprojection: pinhole callers would read it as normalized-plane coordinates.
- Deleting `Camera::Unproject`: cleanest, but a refactor across every pinhole caller.
- An OpenMVS-side RANSAC wrapper over PoseLib's minimal solvers: PoseLib ships the bearing estimators.
- Converting bearings to normalized-plane coordinates for the generalized-camera estimators: the conversion discards the
  hemisphere sign, and the rig model does not fit a single spherical camera.

## 12. Open items

- `View::Ray()` and `View::UnprojectPoint()` use the 2D `Unproject()` form (singular at the equator sides and the poles).
  Whether a live caller invokes the 2D form on a spherical image is not audited.
- `TriangulateDLT` is production-unused and pinhole-only; it is kept as public API with test coverage.
- The mesh stages do not support spherical cameras: a 360 dataset meshes through `--spherical-export cubemap`, whose
  seams and fixed face count and size are inherent to the bridge.
- The semi-global matcher does not apply the undistortion validity mask (the ignore mask) of Brown images.
- InterfaceCOLMAP handles pinhole and Brown cameras only (COLMAP has no equirectangular model); InterfacePolycam
  undistorts on its own.
- Pose-uncertainty estimation is not validated for spherical cameras (`PoseUncertainty.md` section 4).
- The Viewer navigation in the camera view mode of a panorama turns the view about a target close to the image centre,
  not exactly in place; the first-person controls do.
