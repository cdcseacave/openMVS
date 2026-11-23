# Tests Application

Unit and integration tests for the SFM and MVS libraries. No test framework library: `main()` dispatches the tests by hand and stops at the first failure.

## Test Dispatch

```
main(argv)
  argv[1] == 0 or missing → UnitTests()      — data structures, math, codecs, mesh and image units
  argv[1] == 1            → the SFM table    — 88 entries, file by file in pipeline order
  argv[1] >= 2            → the MVS tests    — PipelineTest and the mesh/neighbor/cache tests after it
```

A test returns `false` on failure, and `main()` then prints `ERROR: <name> failed!` and returns `EXIT_FAILURE` at once.

## Test Data

Located in `data/` (path compiled as `_DATA_PATH`):
- `scene.mvs` — binary MVS scene for pipeline testing
- `images/` — the four bundled photographs (`00000.jpg`, `00001.heic`, `00002.heic`, `00003.jpg`) of the SFM real-images tests
- `exif_samples/`, `roma2/` — the fixtures of the metadata and RoMa2 preprocessing tests
- the other files (`*.ply`, `depth*`, `reconstruct_test.*`, `scene_pre_reconstruction.sfm`) are outputs of test runs, not committed

## Running

Three ctest entries (`CommonUnitTests`, `SFMPipelineTest`, `MVSPipelineTest`), from the build tree:
```bash
cd make
OPENMVS_ROMA2_MODEL_PATH=<roma2 model folder> OPENMVS_FLEXGLUE_MODEL_PATH=<flexglue bundle folder> ctest -C Release --output-on-failure
ctest -C Debug --output-on-failure      # the checked build: slow, about an hour
```
Without a model variable the half of `DenseMatcherReconstructTest` that needs it reports itself skipped, and `RoMa2OnnxParityTest` is skipped unless the model folder holds the reference dumps; `OPENMVS_ROMA2_PROVIDER` (`auto|cuda|coreml|dml|cpu`) and `OPENMVS_ROMA2_SETTING` (`turbo|fast|base`) narrow what the RoMa2 tests run. A Debug build of this tree defines `_HEADLESS_DEBUG` (see the root AGENTS.md): a passing Debug run has no `[ASSERT]` on stderr and no `ASSERTION FAILED` in `make/Testing/Temporary/LastTest.log`. There is no test filter: to run one test, call the binary (`Tests 1`) with a local, uncommitted edit of the table.

## Unit Tests (`Tests.cpp`, `UnitTests()`)

Containers and math (`cListTest`, `OctreeTest`, `OctreeLODTest`, ray-triangle intersection, the L1 and confidence-interval solvers, `TestTetraFlow`, `TestLBPPotts`), UTF-8 conversion, the MVS units (vertex-colored PLY and the PLY codec, the Delaunay bipyramid and tetra fixtures, half-mesh processing, `PlatformArr` growth, the mesh refinement step and window statistics, per-vertex mesh cleaning, image resize with its camera), `SFM::HEIFMetadataTest` and the HEIF and TIFF readers' own tests (when built in).

Tests needing scratch files use `ScopedTempDir` (`Tests.h`), which creates a uniquely
named temporary directory, reports the failure itself, and removes the tree on scope exit:

```cpp
const ScopedTempDir tmpDir(_T("MyTest"));
if (!tmpDir.IsValid())
    return false;
const String path = tmpDir(_T("scene.mvs"));
```

Tests showing that a result does not depend on where the allocator places memory use
`ScopedDescendingHeap` (`Tests.h`): while one is alive, every `new` of up to 256 bytes comes from a
pool handed out from its top down, so each block lies below the previous one. `Tests.cpp` replaces
the global `operator new`/`operator delete` for this; outside such a scope they forward to
`malloc`/`free`. It reaches the libraries' allocations where they bind to the executable's operator
new (shared objects on Linux), and has no effect where they do not.

## SFM Tests (`TestsSFM.h`, ten topic files `TestsSFM<Topic>.cpp`)

All in `namespace SFM`, declared in `TestsSFM.h` (one section per file, in the order below) and run in that order by the table in `Tests.cpp` (`SFM_TEST(Name)` rows, one comment line per file). A test must not depend on another having run.

### Shared framework (`TestsSFMFramework.h` / `.cpp`)

Every block states its contract with `ASSERT`; a call outside it is a bug in the test. The header documents each block with an example.

- **Frame**: `TestScope scope("XTest")`, `TEST_CHECK(cond, fmt, ...)` (a failed check prints `XTest FAILED: <file>:<line>: <cond>: <message>` and returns false, so it stops the test), `return scope.Pass(fmt, ...)` (prints `XTest PASSED: <message> (<time>)`); `LogCapture` (the log text while alive, with `Contains` and `Count`).
- **Scenes without geometry or from a generator**: `SceneConfig` / `GenerateTestScene()` / `RingConfig()` (the configurable synthetic scene), `GenerateTwoClusterScene()`, `PairGraph` (a view graph of weighted or matched pairs: `Pair`, `Matched`, `Hub`, `Chain`, `Clique`), `MakeRig()` (posed pinhole rig), `MakeSphereScene()` (panoramas looking at a sphere of points), `BundledImagesConfig()` / `BundledImagesFeatures()` (the four bundled photographs with their AKAZE features, computed once per process).
- **Scenes of the resection and the merge**: `ArcScene` (posed cameras on an arc: `Tracks`, `TrackAt`, `Correspondences`, `Displace`, `Pair` with `ArcLinkSpec`, `Unregister`, `Triangulate`), `RingSceneConfig` / `GenerateRingScene()` / `SplitOnTruth()` / `RingFixture` (a ring of camera blocks around a wall, split as the reconstructions hand them out, merged with `RingFixture::Merge`).
- **Measures and shared helpers**: `PoseErrors()` (rotation and position error after the best similarity), `RotationErrorDeg()`, `ArmPair()`, `HasDensePartition()`, `ReconstructMatchedScene()`, `ExtractBlocks()`, `Median()`. A builder or check used by one file is a `static` helper at the top of that file.

### Adding a test

1. Write `bool XTest()` in the topic file of its stage: open with `TestScope scope("XTest");`, make every check a `TEST_CHECK` with a message that says what was measured and what was expected, with the numbers, and end with `return scope.Pass(...)`. Every random draw comes from a `std::mt19937` with a literal seed (never `std::random_device`).
2. Declare it in `TestsSFM.h` under its file's section, with a comment of two to five lines, in the position it has in the file.
3. Add one `SFM_TEST(XTest),` row to the table in `Tests.cpp`, under the comment line of its file, in the same position.
4. A test that needs a model or a device skips itself with a `VERBOSE` line when the variable is not set, and still returns true.

### Test Catalog (in table order)

**`TestsSFMImport.cpp`**

| Test | Covers |
|------|--------|
| `HEIFMetadataTest` | HEIF/HEIC at the SFM layer: decoded resolution, EXIF bridge, orientation guard, pixel fallback (unit-test suite, when HEIF is built in) |
| `ImportMetadataDeterminismTest` | repeated parallel `Scene::Import` of the bundled folder gives the same finite cameras every time |
| `ExifFocalUntrustedTest` | an EXIF 35 mm-equivalent focal seeds the camera's focal untrusted |
| `CameraTypeImportTest` | the camera model an import gives: from the metadata, forced spherical or pinhole, a non-2:1 spherical image refused |
| `KnownPosesImportTest` | frames.json and pose CSV import (names, intrinsics, duplicates, invalid rows) and the pose-frame detection from the matched pairs |

**`TestsSFMFeatures.cpp`**

| Test | Covers |
|------|--------|
| `VocabularyTreeTest` | vocabulary tree build, save, load and query round trip |
| `BuildTracksConflictTest` | the track builder on a component holding one image twice: cut versus veto, merge distance |
| `RetrievalTest` | global-descriptor retrieval: cosine ranking, the RETRIEVAL match mode, VOCABULARY against it, the `.sfm` round trip |
| `PairSelectionTest` | how a candidate pair is made or pruned: known poses, sequential prior and mode, pre-match threshold |
| `MatchOutcomeSignalsTest` | `Scene::MatchPairs()` fails on an empty view graph; the fatal-round signal is not "nothing stored" |
| `SphericalBottomBandTest` | the bottom band of equirectangular feature extraction |
| `DetectorConfigTest` | `FeatureTypeFromString` words and the shared default detector with its thresholds |
| `SphericalPairGeometryTest` | the relative pose of two panoramas from bearing vectors |
| `TwoViewGeometryTest` | `PairsMatcher` and `ImagePair` matrix operations, two-view calibration refinement, pose from the fundamental matrix |

**`TestsSFMDense.cpp`**

| Test | Covers |
|------|--------|
| `DenseWarpSamplingTest` | the warp draws: the coverage-uniform sample, the complementary fill, determinism |
| `DenseWarpGridConventionTest` | the warp grid's pixel and cell convention end to end |
| `DenseVerdictTest` | `JudgePairDense` on an exact bidirectional warp: admit, min-side and homography rejects, `minOverlap` |
| `DenseGuidedMatchTest` | `MatchFeaturesGuided`: ratio outside the search disc, epipolar band, fallbacks |
| `DenseAssemblyTest` | `AssemblePairDense`, `StorePairDense`, `AppendDenseMatches`: sparse and dense segments under one pose |
| `PlanarTwinSwitchTest` | the planar twin switch of a floor pair, its gain margin and plane gate |
| `DenseReverifyPartitionTest` | the re-verification a camera change sets off keeps a RoMa2 pair's sparse, dense and outlier partition |
| `DenseKeypointBoundaryTest` | the described/dense keypoint boundary survives a descriptor release, an `.sfm` round trip and the keypoint remap |
| `RoMa2PreprocessTest` | CPU preprocessing against torch's bicubic resampling (within 1e-5) |
| `DenseModelFilesTest` | the RoMa2 and FlexGlue manifests and the model path precedence of both matchers (no model needed) |
| `RoMa2OnnxParityTest` | the exported graphs against the reference dumps shipped with the model (skipped without them) |
| `DenseMatcherReconstructTest` | both dense matchers in the whole import, AKAZE, RETRIEVAL pipeline on the bundled images, each half run when its model variable is set |

**`TestsSFMCalibration.cpp`**

| Test | Covers |
|------|--------|
| `LensMatchingTest` | the matchers and the re-verification through a lens the camera knows |
| `LensFitRecoveryTest` | `FitLensesFromPairs` recovers the lenses that rendered the pairs |
| `LensFitFocalTest` | the lens fit's focal check corrects a rejected trusted focal and keeps a right one |
| `LensFitLimitsTest` | what the lens fit refuses or leaves alone |
| `LensFitStageTest` | the lens stage of the matching (`FitLensesAndReverifyPairs`) brings pairs to their true rotation |
| `LensFitUntrustedFocalTest` | an untrusted focal fitted with its lens from the import focal, and its fallbacks |
| `ViewGraphCalibratorTest` | focal refinement over the view graph |

**`TestsSFMViewGraph.cpp`**

| Test | Covers |
|------|--------|
| `PairWeightTest` | the pair weights: spatial, connectivity and triplet terms, the composite, dense inliers counted in proportion |
| `TripletFilterTest` | the camera-triplet view-graph filter: scores, yield rule, both modes |
| `RotationAveragingTest` | the rotation estimator on the 16-camera ring and its independence of the weight scale |
| `ContradictionCutTest` | the contradiction cut: its rotation and translation stages on symmetric rings, roofs, bridges |
| `FilteredSceneReloadTest` | a matched scene is filtered once, on load as on a fresh run |
| `PairScalesTest` | the flat pair-scale build equals the node-map one |
| `ScaleAveragingTest` | the scale estimator solves each connected component with its own gauge |
| `TranslationAveragingTest` | the translation estimator's normal-equations solve against SparseQR |
| `ScaleConjugateGradientsTest` | the scale solve by conjugate gradients against sparse Cholesky |
| `RobustAveragingTest` | the robust averaging ignores a wrong chord in scales and translations |

**`TestsSFMCluster.cpp`**

| Test | Covers |
|------|--------|
| `SceneClusterBasicsTest` | clustering into sub-scenes: limits, size constraints, memory protocol, ID remapping |
| `SceneClusterSmallClusterPassTest` | the small-cluster pass: weak images rescued, a community under the floor held to the coupling test |
| `SceneClusterRefinePassesTest` | the balance pass and the thin-waist check on hand-made graphs |
| `SceneClusterSeamTest` | clustering on chains: sizes aimed at the target, a strong seam between neighbours, a weak tail merged |

**`TestsSFMBlock.cpp`**

| Test | Covers |
|------|--------|
| `TripletStarInitTest` | three-view star initialization with track building and bundle adjustment |
| `StarInitTest` | the star initializer: reference view, neighbour listing, scale gauge |
| `NearDuplicateYardstickTest` | a near-duplicate pair is never the yardstick of connectivity |
| `ResectionLinkChecksTest` | the checks a resected pose passes: inlier share, composed rotation, quorum, link direction and rotation |
| `ResectionCandidateOrderTest` | the next image of the resection is the candidate whose verified pairs weigh most |
| `ResectionFullAdjustmentTest` | when the resection loop runs a full adjustment (`Resection::GetStats()`) |
| `ResectionDenseCorrespondencesTest` | dense correspondences in the resection: bar, polish weight, inlier weight |
| `LinkWalksTest` | the shared walks: 2D-3D correspondences of an image and the links of a model |
| `ContradictingImagesTest` | images removed for contradicting the model are told from those removed for want of evidence |
| `SceneBlocksInsertTest` | `Insert` then `Release` returns a block's cameras, images and pairs byte for byte |
| `PoolResectionTest` | an image put in the wrong cluster ends registered in the right block; pool offers only to blocks it has links in |

**`TestsSFMBundleAdjustment.cpp`**

| Test | Covers |
|------|--------|
| `PinholeReprojectionJacobianTest` | analytical against automatic Jacobian of the pinhole residual (defined in the library) |
| `BADenseWeightTest` | the weights a bundle adjustment gives dense observations |
| `BAIntrinsicFlagsTest` | `SetBAIntrinsicFlags` and `ResectionConfig::DeriveBAConfigs` |
| `ReconstructSphericalSyntheticTest` | full-sphere reconstruction, equirectangular convention and handedness |
| `BundleAdjustmentBasicsTest` | the plain and spherical solve, focal and distortion refinement, `Scene::Transform` |
| `PoseUncertaintyTest` | the GPS-prior covariance and the quality report round trip |
| `RepeatedFullAdjustmentTest` | a full adjustment fits the observations within the factor of the bar and repeats while inputs differ |
| `FilterTracksTest` | the track filter and triangulation on the arc: dense multiple of the bar, single-view and two-view tracks |
| `TriangulationTest` | a failed triangulation returns 0 and is taken up again; panorama triangulation |
| `BundleAdjustmentPairConstraintTest` | the relative-pose residuals of the verified pairs straighten a bent chain |
| `ReprojectionReportOverUsedTest` | the mean reprojection error of a budgeted solve is over the observations it used |
| `BAObservationCapTest` | the per-image observation cap does not move the solution |
| `BAInnerIterationOrderTest` | a solve by inner iterations ends at the same bits when its blocks are laid out in reverse |

**`TestsSFMMerge.cpp`**

| Test | Covers |
|------|--------|
| `SeamGraphSelectionTest` | the selection of the seam graph over planted block graphs (false chords, rivals, one-camera sides, seams far from the blocks' origins) |
| `RingMergeTest` | a ring of six blocks placed whole on its seams, honest, bent and with a camera of a block far from its place, taken to the truth by the final adjustment |
| `MergeTrackJoinTest` | the cross-block track join: seam matches first, the join guard, the 255 cap |
| `PooledSeamReadingTest` | a block pair read from light cross pairs together, and the bar on their total |
| `MergeRoundsTest` | the merge in rounds: kept whole, regrouping, release, `MergeInRounds`, the anchor |
| `MergeWeakImagesTest` | the weak-image filter before the merge: a weak image leaves its block, a block of weak images alone goes to the pool, no block left, a weak image of the merged model loses its pose in the scene and, registered again by the pool step, does not hold the rounds open |
| `UnobservableScaleTest` | blocks of two cameras a millimetre apart are placed at the ring's scale |
| `PairAgreementMergeTest` | verified pairs across a seam decide by inlier weight |
| `RingClosingSeamTest` | the closing of a ring on a biased, thin or copied seam, a planted chord, a single bridge |
| `SeamCameraSupportTest` | a seam held by one camera weighs one camera; one-camera directions and rigs |
| `FoldedBlockTest` | a block folded by half a turn or 45 degrees leaves a seam off and the rest right |
| `UnplacedBlocksReportedTest` | a block with nothing to be placed against is left out, named in the report and keeps its tracks |
| `BentRingTest` | a ring of bent blocks keeps its seams and the final adjustment straightens it, and its far matches join by the closure alone and by the cross-block join alone |
| `MergeRoundtripTest` | a scene split at the true poses and merged back, in the true frame and moved by random similarities |
| `HierarchicalCameraAlignmentTest` | the camera alignment of two sub-scenes from each seam direction alone and both together |

**`TestsSFMPipeline.cpp`**

| Test | Covers |
|------|--------|
| `SceneAlignmentTest` | the similarity estimate and the alignment onto prior poses and GPS |
| `CubeMapExportTest` | the cube-map bridge of `SFM::ExportMVS`: faces, rig platform, wiring, mixed and 4-face exports |
| `RealImagesPipelineTest` | the pipeline on the bundled images: refusals, match-only run and CSV, SKIP from saved matches, the global solver, reconstruction from wrong intrinsics |

## MVS Tests (`TestsMVS.h` / `TestsMVS.cpp`)

The MVS unit tests run in `UnitTests()`. The integration test `MVS::PipelineTest()` runs first when `argv[1] >= 2`, followed by `MeshRefineSyntheticTest` and `MeshRefineEnergyGradientTest` (the photometric mesh refinement against a textured plane), `MeshUnseenFacesTest`, `SelectNeighborViewsScaleTest` and `DMapCacheKeepsRequestedImageTest`.

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

## Logging Convention

Each file defines its own log name:
```cpp
DEFINE_LOG_NAME(lt, "Test    ")  // Tests.cpp
DEFINE_LOG_NAME(lt, "TestSFM ")  // every TestsSFM*.cpp
DEFINE_LOG_NAME(lt, "TestMVS ")  // TestsMVS.cpp
```

## Build

Links against both SFM and MVS libraries. `_DATA_PATH` is set at compile time to `${CMAKE_CURRENT_SOURCE_DIR}/data/`, so the test binary locates data files relative to the source tree. Installed to `${INSTALL_BIN_DIR}`.
