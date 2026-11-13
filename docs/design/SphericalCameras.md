# Spherical Cameras

## 1. Purpose and scope

`libs/SFM` supports equirectangular (360 degree) still images and video frames as a camera model next to the pinhole,
through sparse reconstruction: metadata detection, feature extraction, matching, pose estimation, triangulation and bundle
adjustment all dispatch on `CameraType::SPHERICAL`. Dense reconstruction (`libs/MVS`) has no spherical camera model; the
SfM to MVS export renders each spherical image to tangent pinhole faces that PatchMatch and texturing consume as ordinary
images. Every geometric computation that must hold over the whole sphere works on 3D unit bearing vectors, which is the
design of section 5 (bearing-vector pose estimation).

## 2. Camera model and consequences

`SphericalCamera` (`Camera.h/cpp`) is the equirectangular projection with the Y-down convention of the pinhole: longitude
`theta = atan2(x, z)` maps to the horizontal axis, elevation `phi = asin(-y / r)` to the vertical one. It has no
intrinsics (`GetK()` is the identity), always trusts its intrinsics, and requires `width == 2 * height` (asserted by the
constructor). It is created from the EXIF projection tag or by `--camera-type 1` (`ImportAndCameras.md`).

- `UnprojectNormalized(x)` returns the unit bearing, `(cos phi sin theta, sin phi, cos phi cos theta)`, without
  singularity over the whole sphere. `Unproject(x)` returns the same direction scaled so that `|z| = 1` with the sign of
  `cos theta`; it is singular at the equator sides (`cos theta = 0`) and the poles and is not used for estimation.
- `PixelErrorToAngular` converts a pixel threshold to radians (`2 pi / width` per pixel); `GetFeatureNoiseScale()` is 2
  (pinhole 1): features from the cube faces localize about twice as noisily in pixels (face-seam sampling and off-centre
  descriptor warping), and thresholds calibrated on pinhole pipelines are widened by it (`AngularError`, `Link2D3D.h`).
- Every caller that needs whole-sphere correctness (triangulation, track filtering, pose estimation, matching) goes
  through `UnprojectNormalized`.

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
functor precomputes a tangent-plane basis on the unit sphere per observation, pre-scaled by `width / (2 pi)` so residuals
are in pixel-equivalent units, has no cheirality test, and returns the two tangent-plane dot products rather than the
`acos` angle, for numerical stability near zero residual.

## 3. Feature extraction: cube-map faces and the bottom band

`FeaturesExtractor::ExtractImageSpherical` renders the equirectangular image to `cubemapFaces` tangent pinhole faces
(`SphereCubeMap::MakeTangentFacesGeometry`, `SphericalToTangentialFaces`), runs the normal pinhole `ExtractImage` on each
face (every detector, the 3 x 3 grid, RootSIFT and the SiftGPU dispatch are reused, `FeatureExtraction.md`), reprojects the
keypoints to equirectangular pixels (face pixel to bearing in the face frame, to the body frame by the face rotation, then
`SphericalCamera::Project`), and records the source face in the keypoint's `octave` field. Keypoints duplicated across a
face seam are suppressed by angular non-maximum suppression over unit bearings (a 3D octree ball query of chord radius
set by `cubemapDedupAngleDeg`), keeping the highest-response keypoint per neighbourhood. Face counts 4, 6, 8, 12 and 20 are
supported (4 is equatorial and sees no zenith or nadir); any other count fails extraction.

**Bottom band.** A keypoint whose equirectangular pixel lies in the bottom `sphericalBottomBand` fraction of the image
height is dropped before the seam suppression, so it neither becomes a feature nor suppresses a neighbour. The operator,
the tripod and the ground right under the camera sit there and give wrong, look-alike or moving matches. The decision is
the single predicate `FeaturesExtractor::IsMasked(image, pt)`. Detection still runs on the whole face, so the rows above
the band keep exactly the features they had. Perspective images never reach the predicate.

Spherical images are decoded inline on the extraction thread, not prefetched: they are decoded in colour and split into
faces. `apps/ExtractKeyframes` exposes `--camera-type 0|1` and `--cubemap-faces` for video input.

## 4. MVS export (cube-map bridge)

`SFM::ExportMVS` (`InterfaceMVS.cpp`) detects spherical cameras and builds one `TangentFacesGeometry` for
`(ExportMVSConfig::sphericalNumFaces, sphericalFaceSize)`, then:

1. `EmitSphericalPlatforms`: one MVS platform per spherical SfM camera with one camera record per face, sharing the
   geometry's intrinsics and per-face rotations.
2. `AppendSphericalFaceImages`: one image record per face, named `<stem>_face<k><ext>`, with ids from a counter past the
   largest SfM image id (pinhole images keep their ids).
3. `RenderAndSaveSphericalFaces`: renders and writes the face images.
4. `ProjectTrackOntoSphericalFaces`: each 3D track point is projected into every face's pinhole model and adds a view per
   face that sees it in front.

No command-line option of `CreateStructure` or another app sets the export's
face count or size; only the Python bindings and a hand-built `ExportMVSConfig` can.

## 5. Bearing-vector pose estimation

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

## 6. Parameters and defaults

| Parameter | Location | Default | Meaning |
|---|---|---|---|
| `cubemapFaces` | `FeatureExtractionConfig` | 6 | tangent-face count for feature extraction (4, 6, 8, 12, 20) |
| `cubemapFaceSize` | `FeatureExtractionConfig` | 0 (auto: `max(1024, width / 4)`) | tangent-face resolution for feature extraction |
| `cubemapDedupAngleDeg` | `FeatureExtractionConfig` | 0.25 deg | angular non-maximum-suppression radius across face seams |
| `sphericalBottomBand` | `FeatureExtractionConfig`, `CreateStructure --spherical-bottom-band` | 0.2 | fraction of the image height, from the bottom, discarded; in [0,1), 0 = off |
| `sphericalNumFaces` | `ExportMVSConfig` | 6 | face count for the MVS export (4, 6, 8, 12, 20) |
| `sphericalFaceSize` | `ExportMVSConfig` | 1024 px | face resolution for the MVS export |
| `--camera-type`, `--cubemap-faces` | `apps/ExtractKeyframes` | 0, 6 | video input camera model and extraction face count |

`CreateStructure` exposes the bottom band but not the extraction face count or size.

## 7. Invariants and constraints

- `SphericalCamera` requires `width == 2 * height`. `Image::LoadMetadata` fails for a spherical image of another shape
  with an error naming it, and the import keeps that image without a camera.
- Estimation inputs are unit bearings from `UnprojectNormalized` only; `Unproject()` must never feed RANSAC, DLT or angle
  computations.
- `pair.F` exists only for a pair of pinhole cameras; every consumer checks `pair.F.has_value()` and otherwise uses
  `pair.E` on bearings.
- A bearing estimator's `max_error` is an angle in radians; pixel thresholds are converted per camera first.
- No spherical-model parameter (lens, orientation offset) is refinable by bundle adjustment.
- Pose-quality export (`PoseUncertainty.md`) is validated for pinhole cameras only.
- `libs/MVS` never sees a spherical camera: only the rendered pinhole faces.

## 8. Validation

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

## 9. Rejected alternatives

- Stereographic projection from the back pole as the 2D unprojection: still singular at the pole and changes the return
  semantics for pinhole callers.
- Returning raw `(theta, phi)` from the 2D unprojection: pinhole callers would read it as normalized-plane coordinates.
- Deleting `Camera::Unproject`: cleanest, but a refactor across every pinhole caller.
- An OpenMVS-side RANSAC wrapper over PoseLib's minimal solvers: PoseLib ships the bearing estimators.
- Converting bearings to normalized-plane coordinates for the generalized-camera estimators: the conversion discards the
  hemisphere sign, and the rig model does not fit a single spherical camera.

## 10. Open items

- `View::Ray()` and `View::UnprojectPoint()` use the 2D `Unproject()` form (singular at the equator sides and the poles). Whether a live caller invokes the 2D form on a spherical image is not
  audited.
- `TriangulateDLT` is production-unused and pinhole-only; it is kept as public API with test coverage.
- Dense reconstruction works only on the rendered faces, so seams and the fixed face count and size are inherent to the
  current MVS path.
- Pose-uncertainty estimation is not validated for spherical cameras.
