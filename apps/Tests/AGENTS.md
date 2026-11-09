# Tests Application

Unit and integration tests for the SFM and MVS libraries. No test framework — tests are dispatched manually from `main()` with early-exit on first failure.

## Test Dispatch

```
main(argv)
  argv[1] == 0 or missing → UnitTests()      — data structures & math
  argv[1] == 1            → SFM smoke tests   — multiple sequential tests
  argv[1] >= 2            → MVS::PipelineTest() — full dense reconstruction
  argv[1] not a number    → MVS::SynthTool()    — tool modes (see Synthetic Depth GT)
```

Each test prints `VERBOSE` progress and returns `false` on failure, causing `main()` to `return EXIT_FAILURE` immediately.

## Test Data

Located in `data/` (path compiled as `_DATA_PATH`):
- `scene.mvs` — binary MVS scene for pipeline testing
- `images/00000.jpg` through `images/00003.jpg` — 4 photographs for SFM reconstruction

## Unit Tests (`Tests.cpp`)

| Test | What It Validates |
|------|-------------------|
| `cListTest<true>(100)` | Custom vector container operations |
| `OctreeTest<double,2>(100)` | 2D spatial octree indexing |
| `OctreeTest<float,3>(100)` | 3D spatial octree indexing |
| `TestRayTriangleIntersection<float>(1000)` | Ray-triangle intersection (float) |
| `TestRayTriangleIntersection<double>(1000)` | Ray-triangle intersection (double) |
| `TestLeastAbsoluteDeviationSolver()` | Robust L1 solver |
| `TestConfidenceInterval()` | Statistical confidence interval |
| `MVS::MeshVertexColorsPLYTest()` | Vertex-colored PLY round-trip, ASCII and binary |

Tests needing scratch files use `ScopedTempDir` (`Tests.h`), which creates a uniquely
named temporary directory, reports the failure itself, and removes the tree on scope exit:

```cpp
const ScopedTempDir tmpDir(_T("MyTest"));
if (!tmpDir.IsValid())
    return false;
const String path = tmpDir(_T("scene.mvs"));
```

## SFM Tests (`TestsSFM.h` / `TestsSFM.cpp`)

All in `namespace SFM`. Called sequentially when `argv[1] == 1`.

### Synthetic Scene Generator

`SceneConfig` (line ~244) drives most SFM tests:
- `CameraType`: PINHOLE or SPHERICAL
- `PoseMode`: SIMPLE_TRANSLATION, RANDOM_POSES, CIRCULAR_ARRANGEMENT
- `PerturbOptions`: Bitmask — PERTURB_POSES, PERTURB_POINTS, PERTURB_INTRINSICS, PERTURB_KEYPOINTS, PERTURB_PAIR_POSES, PERTURB_ALL
- `GenerateTestScene()` creates a fully synthetic scene with configurable cameras, points, noise, and distortion

### Test Catalog

| Test | Purpose | Key Tolerance |
|------|---------|---------------|
| `VocabularyTreeTest()` | VocTree build/save/load/query roundtrip (RootSIFT-like + binary descriptors) | Top matches contain expected images |
| `KnownPosesImportTest()` | frames.json/CSV pose import, intrinsics trust, duplicate and malformed-row handling | Unique images imported; invalid rows make no partial updates |
| `KnownPosePairSelectionTest()` | Pose-guided selection with partial pose coverage | Unique candidate set includes the unposed image |
| `AlignToPriorPosesTest()` | Sim(3) restoration of a transformed known-pose scene | Camera centers and rotations return to their prior frame |
| `AlignToPriorPosesCollinearTest()` | Restoration of a straight-line capture (collinear centers) | Rotation-averaging fallback recovers the roll the centers cannot |
| `BAPinholeReprojectionJacobianTest()` | Analytical vs AutoDiff Jacobian validation for pinhole BA | Gradient agreement |
| `PipelineTest()` | 6-subtest BA suite: quaternion poses + pose covariance, spherical camera, focal refinement, radial distortion, GPS constraints, scene transform | Reprojection < 1.0 px; focal < 5%; k1/k2 < 0.01; covariance finite/PSD, one datum |
| `GPSPriorPoseUncertaintyTest()` | GPS-prior BA on a geo-aligned scene: absolute (datum-free) pose covariance + missing-accuracy fallback | No gauge datum; all finite; mean position error < 0.2 m |
| `PoseUncertaintyExportTest()` | Pose-quality report roundtrip: covariance recorded on the scene, CSV export re-read, ExportMVS image-ID preservation, `Scene::Transform` covariance mapping, `.sfm` serialization | 1 datum row; IDs match; Cov' = s²RCovRᵀ; save/load identical |
| `TripletStarInitTest()` | 3-view initialization via `StarInitializer` with track building and intrinsic refinement | >75% tracks recovered; focal < 5%; k1/k2 < 0.01 |
| `TwoViewTest()` | Epipolar geometry: essential/fundamental matrix, pose recovery, distortion roundtrip | Rotation < 0.1 rad; translation dot > 0.95; distortion reproj < 1e-4 |
| `ReconstructTest()` | Full SFM on real images: import → AKAZE features → exhaustive matching → geometric filter → tracks → BA | 4 images loaded; BA converges; tracks non-empty |
| `RotationEstimatorTest()` | Global rotation averaging (16 circular cameras, 1 disconnected) | Relative rotation < 5 deg |
| `ScaleEstimatorTest()` | Global scale averaging from pairwise ratios (auto + fixed gauge) | Scale ratio error < 1e-4 |
| `TranslationEstimatorTest()` | Global translation averaging from pairwise constraints | Translation error < 1e-4 |
| `PairsWeightingTest()` | Spatial, connectivity, and triplet weight computation for image pairs | Spread > clumped; valid triplets > 0 |
| `ViewGraphCalibratorTest()` | Focal length refinement via view graph (8 images, +30% perturbation) | Focal < 2% error |
| `PairMatcherTest()` | Sequential matching mode (5 images, overlap=2, 10 expected pairs) | Exact pair count and membership |
| `PreMatchTest()` | Pre-matching threshold filtering (3 images, manual descriptors) | Correct accept/reject per threshold |

### Key Helpers

- `GenerateRandomRotation()` / `GenerateRandomTranslation()` — synthetic pose generation
- `ComputeTracksMeanReprojectionError()` — BA quality metric
- `TriangulateTracks()` / `BuildTracks()` — track construction and triangulation
- `ComputePairsWeights()` — pair importance scoring
- `ComputeAngle()` — angle between rotation matrices

## MVS Test (`TestsMVS.h` / `TestsMVS.cpp`)

Single integration test: `MVS::PipelineTest()`.

```
Load scene.mvs
  → DenseReconstruction()        — point cloud >= 50,000 points
  → SceneSplitTest()             — on the depth-maps: ROI-independent depth term, forced split exported
  → ReconstructMesh()            — faces in [40,000 – 100,000]
  → Mesh::Clean(decimate=0.7)    — faces in [28,000 – 70,000]
  → TestMeshProjectionMT()       — (if OpenMP enabled)
  → ComputeVertexColors()        — every vertex colored, most of them sampled
  → TextureMesh()                — texturing succeeds
  → ComputeReconstructionQuality() — score >= 43.0
```

The face/quality bounds are deliberately wide plausibility windows: they bracket the
spread of both the CPU and GPU PatchMatch backends, which differ by design.

Keeps the depth maps on disk (`OPTDENSE::bRemoveDmaps = false`) for `SceneSplitTest` and deletes them right after it, whatever its outcome. Optionally saves `.ply` outputs when verbose (into the data folder: remove them after a verbose run).

## Synthetic Depth GT (`TestsSynthetic.h` / `TestsSynthetic.cpp`)

Deterministic depth ground truth without data files, used for the spherical PatchMatch work:
- **Scene:** closed room [-8,8]x[-5,5]x[0,5] m (z up) with 3 spheres, a slanted ramp and a thin
  pillar, solid world-space value-noise texture (6-octave fbm + sharp-edged spots, Lambertian),
  one small low-texture patch on the y=+5 wall.
- **Views** (`SynthGenerate(SynthConfig, SynthViewArr&)`): per panorama on an 8-pose trajectory
  the equirect view (`panoWidth` x `panoWidth/2`, default 512), its 6 `SphereCubeMap` faces
  (`panoWidth/4`), plus Brown-distorted pinhole views between panoramas. Exact ray casting
  through `CameraModels.h`; color SxS supersampled; GT lambda (z for pinhole, z of the
  undistorted ray for Brown, range for spherical) and camera-frame normals at pixel centers.
- `SynthCamera` = model + `Params()` (the planned Interface v8 description), `R`, `C`
  (X_cam = R (X - C)).
- `SynthGTMismatch(a, b)`: median relative lambda mismatch of a reprojected into b
  (faces ~2e-8, Brown ~4e-6, panorama targets ~2e-5 from the bilinear interpolation).
- `SynthBuildScene`: MVS scene of the face views (one platform per panorama, faces as its
  cameras), PNGs written to a folder, GT sparse points.
- `SynthSave`: PNG + GT `.dmap` (lambda + normals; the spherical K holds fx,fy,cx,cy) + depth
  preview per view, `cameras.json` for every view, `scene.mvs` of the faces.
- `DepthEvaluator`: abs-rel mean/median, % < 1/2/5 %, completeness, normal median angle, per
  band: all; equirect equator (|elev| < 30), north/south (|elev| > 70), seam (16 px from
  column 0 / W-1); pinhole center / border (16 px).
- `SyntheticDepthBaselineTest()` (MVS suite, CPU forced): GT consistency across the models,
  then CPU PatchMatch on the 18 128x128 faces of 3 panoramas, asserted against calibrated
  thresholds (measured numbers in the code). ~25 s in Debug.

- `BrownUndistortOnLoadTest()` (MVS suite): the two Brown views as an Interface v8 `BROWN` camera
  of their PNG files; each image read through `MVS::Image` against the pinhole render at `K'` of
  the same pose (mean abs difference over the valid pixels), the invalid area equal to the
  `ImportIgnoreMask` output, and the observations of GT points against their `K'` projections.

## Camera models (`TestsMVS.cpp`)

`InterfaceV8Test()` (MVS suite, before the pipeline): Interface v8 round trip of every camera
model (pinhole, Brown 5 and 8 coefficients, spherical) with and without observations; a v7 stream
written and loaded back; the size identity (v8 = v7 + 1 byte per camera + 1 flags byte without
observations); `MVS::Scene` load/save of the models (Brown lens kept exactly, observations within
1e-3 px); `K'` identical between `UndistortedK`, the SfM `UndistortMaps::Init`, the map cache and the
loaded image camera.

Tool modes (argv[1] is not a number; they always write `Tests-*.log` in the current directory, and
only a Debug `_HEADLESS_DEBUG` build also prints to stdout):
```
Tests synth <outDir> [panoWidth=512]          # write the dataset
Tests evaldepth <gt.dmap> <est.dmap> [model]  # band table (GT resampled to the estimate size)
Tests synthtest [verbosity]                   # run SyntheticDepthBaselineTest alone
Tests cameratest [verbosity]                  # run InterfaceV8Test and BrownUndistortOnLoadTest alone
Tests undistortbench <scene.mvs> [numImages=8] # decode vs remap time per image undistorted on read
```

## Logging Convention

Each file defines its own log name:
```cpp
DEFINE_LOG_NAME(lt, "Test    ")  // Tests.cpp
DEFINE_LOG_NAME(lt, "TestSFM ")  // TestsSFM.cpp
DEFINE_LOG_NAME(lt, "TestMVS ")  // TestsMVS.cpp
```

## Build

Links against both SFM and MVS libraries. `_DATA_PATH` is set at compile time to `${CMAKE_CURRENT_SOURCE_DIR}/data/`, so the test binary locates data files relative to the source tree. Installed to `${INSTALL_BIN_DIR}`.
