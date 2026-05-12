# Tests Application

Unit and integration tests for the SFM and MVS libraries. No test framework — tests are dispatched manually from `main()` with early-exit on first failure.

## Test Dispatch

```
main(argv)
  argv[1] == 0 or missing → UnitTests()      — data structures & math
  argv[1] == 1            → SFM smoke tests   — multiple sequential tests
  argv[1] >= 2            → MVS::PipelineTest() — full dense reconstruction
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
| `BAPinholeReprojectionJacobianTest()` | Analytical vs AutoDiff Jacobian validation for pinhole BA | Gradient agreement |
| `PipelineTest()` | 5-subtest BA suite: quaternion poses, spherical camera, focal refinement, principal point, radial distortion | Reprojection < 1.0 px; focal < 5%; k1/k2 < 0.01 |
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
  → ReconstructMesh()            — faces >= 25,000
  → Mesh::Clean(decimate=0.7)    — faces in [18,000 – 30,000]
  → TestMeshProjectionMT()       — (if OpenMP enabled)
  → TextureMesh()                — texturing succeeds
  → ComputeReconstructionQuality() — score >= 45.0
```

Sets `OPTDENSE::bRemoveDmaps = true` to clean intermediate depth maps. Optionally saves `.ply` outputs when verbose.

## Logging Convention

Each file defines its own log name:
```cpp
DEFINE_LOG_NAME(lt, "Test    ")  // Tests.cpp
DEFINE_LOG_NAME(lt, "TestSFM ")  // TestsSFM.cpp
DEFINE_LOG_NAME(lt, "TestMVS ")  // TestsMVS.cpp
```

## Build

Links against both SFM and MVS libraries. `_DATA_PATH` is set at compile time to `${CMAKE_CURRENT_SOURCE_DIR}/data/`, so the test binary locates data files relative to the source tree. Installed to `${INSTALL_BIN_DIR}`.
