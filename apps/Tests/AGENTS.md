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

Ray-cast ground truth without data files, for the camera models and the spherical PatchMatch:
- **Scene:** closed room [-8,8]x[-5,5]x[0,5] m (z up) with 3 spheres, a ramp and a thin pillar, a solid
  world-space procedural texture and one low-texture patch.
- **Views** (`SynthGenerate`): per panorama of an 8-pose trajectory the equirectangular view (`panoWidth` x
  `panoWidth/2`), its 6 `SphereCubeMap` faces (`panoWidth/4`), and Brown-distorted views between panoramas;
  GT lambda (z for pinhole, z of the undistorted ray for Brown, range for spherical) and normals at the
  pixel centers. `SynthBuildScene` makes an `MVS::Scene` of face/panorama views with GT sparse points.
- **Metrics:** `DepthEvaluator` (completeness, abs-rel, % within 1/2/5 %, normal angle) per band: all;
  equator (|elev| < 30), north/south (|elev| > 70), seam and its antipode (16 px); pinhole center/border
  (16 px). `CloudEvaluator`: precision/recall/F of a cloud against GT range maps at three relative
  thresholds, per band.

Tests (MVS suite 2), each bound being the measured accuracy less a margin:

| Test | Checks |
|------|--------|
| `BrownUndistortOnLoadTest()` | Brown views read through `MVS::Image` match the pinhole render at `K'`; ignore mask = undistortion mask; observations on their `K'` projections, also after `Scene::ScaleImages` (files stored distorted, lens scaled); the MeshLab raster of a Brown image holds its undistorted pixels and the camera at `K'` |
| `NeighborViewsPointCloudRigTest()` | `Scene::EstimateNeighborViewsPointCloud` on two rigs (a panorama and its 6 faces at one center each) without sparse points: no assertion, every point finite and seen by the views of two rigs |
| `EstimatePointCloudNormalsSphericalTest()` | `Scene::EstimatePointCloudNormals` on 3 panoramas at half resolution: the scene images keep their resolution, camera and pixels; every point whose target view projects within 4 columns of the seam is refined (points next to a pole left out), its mean normal error within 1.5x that of the others |
| `TowerSphericalTest()` | `Scene::InitTowerScene` on 24 panoramas in three rings around the pillar, on their seam: every tower point has a horizontal normal and two panoramas seeing it from the side it faces |
| `SyntheticDepthBaselineTest()` | GT consistency across the models, then CPU PatchMatch on the 18 128x128 faces of 3 panoramas |
| `SyntheticSphericalTest()` | CPU PatchMatch on 3 512x256 panoramas, overall and per band (seam held to its antipodal band) |
| `MetalModelMathTest()` | Apple only: the Metal mirror of `CameraModels.h` / `DepthGeometry.h` against the C++ models |
| `SyntheticSphericalGPUTest()` | the same panoramas on CUDA or Metal, moved 1e6 m from the origin (as accurate as there: the estimator only relates the views); skipped without a GPU or with `forceCPU` |
| `SyntheticMixedGPUTest()` | CUDA or Metal on the 6 96x96 faces of the middle of 3 rigs with the 384x192 panoramas of the 2 others as their only neighbors (pinhole reference, spherical neighbors: the `MODEL_PINHOLE_REF` kernels), held to the CPU accuracy on the same views; skipped without a GPU or with `forceCPU` |
| `GPUConfidenceTest()` | the confidence recalibration on CUDA or Metal against the CPU sweep, on noisy GT depth-maps, for a cube face then a panorama against the faces and panoramas of two others (every pair of camera models); both separate the outliers; skipped without a GPU or with `forceCPU` |
| `SyntheticSphericalFusionTest()` | CPU PatchMatch + dense fusion of the panoramas, cloud precision/recall per band |

## Spherical seam and model consistency (`TestsMVS.cpp`)

MVS suite, before the synthetic tests: `GapInterpolationNormalsTest()` (`GapInterpolation` fills a gap
between two normals with unit normals within their angle, pinhole and spherical, also across the seam of a
spherical row), `WrapPixelTest()` (`ModelCamera::WrapPixel` wraps the columns of a spherical camera only,
never rows), `ConfidenceSeamTest()` (`EstimateConfidenceFromDepth/Normal` read across the seam of a spherical
depth-map), `ComputeFOVTest()` (`Image::ComputeFOV`: 2π x π for a full panorama, the pinhole angles otherwise).

## Camera models (`TestsMVS.cpp`)

`InterfaceV8Test()` (MVS suite, before the pipeline): Interface v8 round trip of every camera
model (pinhole, Brown 5 and 8 coefficients, spherical) with and without observations; a v7 stream
written and loaded back; the size identity (v8 = v7 + 1 byte per camera + 1 flags byte without
observations); `MVS::Scene` load/save of the models (Brown lens kept exactly, observations within
1e-3 px), also through the native boost archive (interface -> native -> interface); `K'` identical
between `UndistortedK`, the SfM `UndistortMaps::Init`, the map cache and the loaded image camera; a
standalone spherical `.dmap` loaded by `Scene::LoadDMAP` keeps the SPHERICAL model and its points at
the stored range.

## Running

```
Tests <suite> [verbosity] [forceCPU]   # suite 0/1/2; verbosity > 0 writes Tests-*.log in the current directory
```
Debug builds define `_HEADLESS_DEBUG`: capture stderr (`2> err.txt`), a failed `ASSERT` prints `[ASSERT]` there.

Tool modes (argv[1] is not a number; an unknown mode prints the usage). They always
write `Tests-*.log` in the current directory; the result tables go to the log and, for most modes, stdout:
```
Tests synth <outDir> [panoWidth=512]                       # the dataset: PNG, GT .dmap, cameras.json, scene.mvs of the faces
Tests sphtest <verbosity> [panoWidth=512] [numPanos=3] [faces=1] [maxElevation] [maxViews] [maxThreads] [gpu=0]
Tests conftest <verbosity>                                 # GPUConfidenceTest alone
Tests sphfuse <verbosity> [panoWidth=512] [numPanos=3] [faces=1] [fusionFilter=2] [maxThreads]   # faces 2: mixed scene
Tests scorebench <verbosity> [panoWidth=1536] [rounds=4] [maxElevation] [faceWidth=panoWidth/4]  # ns per ScorePixel
Tests evaldepth <gt.dmap|gtFolder> <est.dmap|estFolder> [pinhole|spherical]
Tests evalcloud <cloud.ply|scene.mvs> <gtFolder> [evalWidth] [th1% th2% th3%]
Tests facescene <scene.mvs> <gtFolder> <outFolder> <panoWidth> [faceSize]   # cube-face baseline of a spherical scene
Tests undistortbench <scene.mvs> [numImages=8]             # decode vs remap time of images undistorted on read
```
`maxElevation` overrides `DepthEstimatorBase::fSphericalWarpMaxElevation` (0: exact warp everywhere).

## Logging Convention

Each file defines its own log name:
```cpp
DEFINE_LOG_NAME(lt, "Test    ")  // Tests.cpp
DEFINE_LOG_NAME(lt, "TestSFM ")  // TestsSFM.cpp
DEFINE_LOG_NAME(lt, "TestMVS ")  // TestsMVS.cpp
DEFINE_LOG_NAME(lt, "TestSyn ")  // TestsSynthetic.cpp
```

## Build

Links against both SFM and MVS libraries. `_DATA_PATH` is set at compile time to `${CMAKE_CURRENT_SOURCE_DIR}/data/`, so the test binary locates data files relative to the source tree. Installed to `${INSTALL_BIN_DIR}`.
