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
| `MatchPairsFailureTest()` | `Scene::MatchPairs()` fails the stage when matching two or more images leaves the view graph empty, and does not report failure on a single-image scene | Reports failure with no pairs and no `MATCHED` state; single image still reports success |
| `PreMatchTest()` | Pre-matching threshold filtering (3 images, manual descriptors) | Correct accept/reject per threshold |

### ROMAv2 Dense Matching Catalog

The one-pass dense matcher (`libs/SFM/MatchROMA2.h`, `libs/SFM/ROMA2Warp.h`), in the order
`Tests.cpp` dispatches it. The last two need a model: they report themselves skipped unless
`OPENMVS_ROMA2_MODEL_PATH` points at an exported model folder, and `OPENMVS_ROMA2_PROVIDER`
(`auto|cuda|coreml|dml|cpu`) and `OPENMVS_ROMA2_SETTING` (`turbo|fast|base`) narrow what they run.
`RoMa2PreprocessTest()` needs neither: it is gated only on ONNX Runtime being compiled in, and
runs entirely off the bundled fixture files.

| Test | Purpose | Key Tolerance |
|------|---------|---------------|
| `ROMA2WarpTrackingTest()` | Keypoint tracking through an identity warp: the pixel/grid/normalized conventions, the confidence gate, the dense append | Tracked positions exact; gated cells dropped |
| `ROMA2CoverageSampleTest()` | The coverage-uniform sample the verdict fits on: budget, bucket stratification against a top-confidence pick, one-sided coverage, determinism | One winner per bucket; identical draw on a repeat |
| `ROMA2ComplementaryDrawTest()` | The dense fill of an admitted pair: drawn only where the guided matches are not, capped, thinned by an even stride, chained across pairs sharing an image | >= 2/5 of the common-region points coincide exactly; repeat draw identical |
| `ROMA2VerdictTest()` | `JudgePairROMA2` on the exact bidirectional warp of a two-camera wedge fixture: ~30% of both frames admitted, a 3% B side rejected by the min-side rule, a homography warp rejected in the calibrated AND the forced-fundamental branch, `minOverlap` 0 admitting both | Inlier areas within 0.05 of the confident region; pose < 0.05 deg and t-dot > 0.9999 off the two cameras |
| `ROMA2GuidedMatchTest()` | `MatchFeaturesGuided`: the ratio taken against the best descriptor OUTSIDE the search disc, so a lookalike elsewhere still rejects and a scale duplicate inside no longer blocks | Exact accept/reject per case; same matches in the same order on a repeat |
| `ROMA2AssemblyTest()` | `AssemblePairROMA2` + `StorePairROMA2`: one fit over guided u dense splitting into the sparse and dense segments, a dense-only pair, a fill too small to refit, and the store's dense append past each image's described prefix and ahead of a pair's rejected tail | Sparse/dense/outlier counts exact; assembled pose < 0.05 deg off; keypoint indices reproducible across two runs |
| `DenseKeypointBoundaryTest()` | The described/dense keypoint boundary survives a descriptor release and an `.sfm` round-trip | Stored described count preserved exactly |
| `SupplementEvidenceIsolationTest()` | Dense supplementation as evidence: ray angle, grid occupancy and the support floor measured over sparse + dense, `GetNumWeightedInliers()` discounting the dense share | Degenerate all-dense baseline still demoted |
| `GlobalDescriptorsQueryTest()` | Cosine ranking of the per-image global descriptors, its tie order, the `PairsMatcher` dispatch through them, the rankings CSV and the `.sfm` round-trip | Deterministic ranking; descriptors bit-identical after round-trip |
| `RetrievalModeTest()` | `RETRIEVAL` match mode: ranks purely on the global descriptors with no ROMAv2 opt-in, agrees pair-for-pair with `VOCABULARY`, a missing descriptor is a hard error | Identical candidate set to VOCABULARY; no vocabulary-tree fallback |
| `RoMa2PreprocessTest()` | CPU preprocessing: constant image to constant planes with the R/G/B swap, and resampling against torch's `F.interpolate(bicubic, align_corners=False, antialias=True)` | Within 1e-5 of the torch reference |
| `RoMa2OnnxParityTest()` | The exported descriptor and coarse-match graphs through `RoMa2Onnx` against the Python reference dumps shipped with the models | Per-preset parity thresholds of the dumps |
| `ROMA2ReconstructTest()` | The full in-process path on the bundled 4-image scene through `Scene::MatchPairs`, four times: the 2048-D retrieval descriptors, a dense run compared against a baseline run with dense matching off, an `.sfm` round-trip, `ReconstructTest`'s reconstruction stage, and two runs of one awkward configuration (1 thread, 2 slots) | Same pair set as the baseline, a dense segment on every pair and a differing sparse count on most; tracks in [5000, 12000] (measured, see the comment); max distortion < 20 px; the two awkward runs bit-identical |

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
  → ReconstructMesh()            — faces in [40,000 – 100,000]
  → Mesh::Clean(decimate=0.7)    — faces in [28,000 – 70,000]
  → TestMeshProjectionMT()       — (if OpenMP enabled)
  → ComputeVertexColors()        — every vertex colored, most of them sampled
  → TextureMesh()                — texturing succeeds
  → ComputeReconstructionQuality() — score >= 43.0
```

The face/quality bounds are deliberately wide plausibility windows: they bracket the
spread of both the CPU and GPU PatchMatch backends, which differ by design.

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
