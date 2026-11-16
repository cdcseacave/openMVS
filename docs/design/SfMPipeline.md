# SfM Pipeline

## Purpose and scope

This document describes the Structure-from-Motion pipeline as one unit: the stages `CreateStructure` runs, in the order
`SFM::Scene::Reconstruct` (`libs/SFM/Scene.cpp`) calls them, what each stage takes and produces, the options that switch
or size it, and which choices can be combined. Each stage has its own design document; this one only fixes the order and
the contracts between stages. A reader is expected to know SfM (features, pairs, tracks, resection, bundle adjustment)
but not this code.

Terms used throughout. A *pair* is two images with matches and, once verified, a relative pose. A *view graph* is the
images as nodes and the verified pairs as edges. A *track* is one 3D point with its observations in several images. A
*block* is a subset of the images that is reconstructed on its own and merged later. The *pool* is the set of images that
belong to no block. The *dense matcher* is an optional neural network (RoMa v2 or FlexGlue, run in-process through ONNX
Runtime) that gives a warp between two images; `--roma2` switches it on, whichever of the two networks is selected.

The program is `apps/CreateStructure/CreateStructure.cpp`. Its `main` reads the options into one `ReconstructionConfig`
(`libs/SFM/Scene.h`), calls `Scene::Reconstruct(source, cfg)` once, saves the scene with `Scene::Save`, then writes the
optional exports. Everything below is inside that one call unless stated.

## Algorithm as implemented

### Stage order

`Scene::Reconstruct` runs the stages below in this order. A scene loaded from a `.sfm` file skips the stages its state
flags say are done (see "Saved-scene resume").

| # | Stage | Takes | Produces | Options that switch or size it | Runs in | Design document |
|---|---|---|---|---|---|---|
| 1 | Import and cameras | an image folder, a semicolon-separated list of images, or one `.sfm` file; optionally a poses file | images with EXIF metadata, one shared camera per distinct intrinsics, forced focal/distortion applied, imported poses | `--source`, `--camera-type`, `--default-focal-ratio`, `--focal-length`, `--k1`, `--k2`, `--image-indices`, `--import-poses-file`, `--import-poses-mode`, `--known-poses-convention` | `Scene.cpp:Scene::Import` | `ImportAndCameras.md`, `SphericalCameras.md` |
| 2 | Features | the images | keypoints and descriptors per image | `--detector-type` (AKAZE, ORB, SIFT, SIFTGPU; default SIFTGPU), `--max-features-per-cell`, `--min-features-per-cell`, `--siftgpu-cuda`, `--siftgpu-darkness-adaptivity`, `--spherical-bottom-band`, `--import-openmvg-dir`, `--export-openmvg-dir` | `Scene.cpp:Scene::ExtractFeatures` (`FeaturesExtractor.cpp:FeaturesExtractor::Extract`) | `FeatureExtraction.md`, `SiftGPUFeatureExtraction.md` |
| 3 | Retrieval and pair selection | features (and, for RETRIEVAL, global descriptors) | the candidate pairs to match | `--match-mode` (default 1 VOCABULARY), `--vocab-max-pairs` (50), `--match-sequence-overlap` (3), `--match-verification-feedback` (1), `--roma2` with `--match-mode 4` | `PairsMatcher.cpp:PairsMatcher::Match` (`CollectVocabularyPairs`, `CollectRetrievalPairs`, `CollectKnownPosePairs`, `CollectVerificationFeedbackPairs`) | `PairSelection.md` |
| 4 | Matching | the candidate pairs | for each pair, matches between keypoints | `--roma2-match` (default 0) selects the dense pass; otherwise descriptor matching. The dense pass is sized by `--roma2-*`, `--dense-matcher`, `--flexglue-model`, `--dense-min-covisibility` | descriptors: `PairsMatcher.cpp:PairsMatcher::MatchPairsBatch`; dense: `MatchROMA2.cpp:MatchPairsROMA2`; both from `Scene.cpp:Scene::MatchPairs` | `PairSelection.md`, `DenseMatching.md` |
| 5 | Pair verification, lens fit, focal calibration | matched pairs | inlier matches and a relative pose per verified pair; the radial lens of every pinhole camera without one; focals of cameras that do not trust theirs | `--release-descriptors`; the lens fit has no option | `PairsMatcher.cpp:PairsMatcher::GeometricFilter` (inside stage 4), then `LensFit.cpp:CalibrateFocalsAndFitLenses` and `MatchROMA2.cpp:SwitchPlanarTwinsROMA2` (both from `Scene::MatchPairs`) | `TwoViewVerificationAndLens.md`, `DenseMatching.md` (planar twin switch) |
| 6 | Pair weighting | verified pairs | the composite weight of each pair | `--export-pairs-csv` writes the pairs right after this step | `PairsWeighting.cpp:ComputePairsWeights`, called by `ExportMatchingCSVsAndFilterPairs` in `Scene.cpp` | `PairWeight.md` |
| 7 | View-graph filtering and the contradiction cut | the weighted view graph | triplet scores per pair, the seed views, and a view graph with the contradicted pairs removed (or repaired) | `--filter-triplets` (1), `--triplet-cut` (0), `--triplet-min-score` (0.3), `--triplet-second-face-score` (0.75), `--max-pair-direction-residual` (45; 0 = rotation stage only), `--repair-contradicted-pairs` (1) | `ViewGraphTriplets.cpp:FilterPairsByTriplets`, then `GlobalRotationAveraging.cpp:FilterPairsByContradiction`; both from `ExportMatchingCSVsAndFilterPairs` | `TripletScores.md`, `ContradictionCut.md`; weights again in `PairWeight.md` |
| 8 | Solver choice | the filtered view graph | a calibrated scene | known poses imported (modes 1 and 2) take `ReconstructKnownPoses`; else `--use-global-solver` takes `ReconstructGlobal`; else `ReconstructHierarchical` (default) | `Scene.cpp:Scene::Reconstruct`, the ternary after the `matchImagesOnly` return | this document |
| 8a | Clustering into blocks | the view graph | a partition: blocks and the pool; or one whole block | `--max-views-per-cluster` (200; 0 = no clustering; a scene of at most that many images is one block) | `SceneCluster.cpp:SceneCluster::Partition`, `SceneBlocks.cpp:SceneBlocks::Extract` or `TakeWhole`; from `Scene::ReconstructHierarchical` | `HierarchicalSFM.md` |
| 8b | Block start and incremental growth | one block's pairs | tracks, a star of registered images, then every image the block can register, with local and full bundle adjustment | `--max-link-direction-error` (5; 0 = off), `--refine-intrinsics`, `--ba-*` | `Track.cpp:BuildTracks`, `StarInitializer.cpp:StarInitializer::Initialize`, `Resection.cpp:Resection::RegisterImages`; blocks run in parallel on the scene's thread pool | `TrackBuilding.md`, `IncrementalReconstruction.md`, `BundleAdjustment.md` |
| 8c | Pool resection | the images no block registered and the images in no block | those images registered into the block they connect to best | `--max-link-direction-error` | `SceneBlocks.cpp:SceneBlocks::ResectPool` | `HierarchicalSFM.md` |
| 8d | Merge of the blocks | the blocks | one model | `--cluster-alignment` (1) | `SceneBlocks.cpp:SceneBlocks::MergeInRounds` | `HierarchicalSFM.md` |
| 9 | Final bundle adjustment | the reconstruction | tracks filtered, triangulated again and filtered; one bundle adjustment of the whole scene | `--refine-intrinsics` (1), `--ba-dense-weight`, `--ba-max-obs`, `--ba-cap-min-obs`, `--ba-pair-sigma` | `Scene.cpp:Scene::Reconstruct` after the solver returns (`BundleAdjustment::Adjust`) | `BundleAdjustment.md` |
| 10 | Resection of leftover images | the registered scene | weakly connected images filtered, then every unregistered image tried again, closed by a full bundle adjustment | `--max-link-direction-error` | `Scene.cpp:Scene::Reconstruct` (`FilterWeaklyConnectedImages`, `Resection::RegisterImages`) | `IncrementalReconstruction.md` |
| 11 | Geo-alignment, pose quality, colors | the reconstruction | alignment to the imported poses (known poses) or to GPS; GPS-prior refinement; per-image pose covariance; point colors | `--align-gps-threshold` (5), `--gps-position-weight`, `--gps-position-weight-z`, `--export-pose-quality`, `--extract-colors` | `Scene.cpp:Scene::AlignToPriorPoses`, `Scene::AlignToGPS`, `SampleColors` | `PoseUncertainty.md`, `BundleAdjustment.md` |
| 12 | Save and export | the scene | the `.sfm`, and optionally poses CSV, pose-quality CSV, an `.mvs` with undistorted images | `--output-file` (default `scene.sfm`), `--export-poses-csv`, `--export-pose-quality`, `--export-mvs`, `--undistort-alpha`, `--undistort-extension`, `--compare-mvs` | `CreateStructure.cpp:main`, `InterfaceMVS.cpp:ExportMVS` | `ImportAndCameras.md` (poses file), `SphericalCameras.md` (cube-map export) |

Stage details that the table cannot hold.

**Import.** A source folder is listed for `.jpg .jpeg .png .tif .tiff .jxl .exr .webp .heic .heif`, sorted by path, two
files whose stems are both numbers being compared as numbers. The camera of each image comes from its EXIF data and `--camera-type`
(-1 reads an equirectangular projection tag; 0 pinhole; 1 spherical, which requires width = 2 x height). Identical
cameras are merged into one shared camera. Imported poses are applied before that merge, so identical per-frame
intrinsics collapse too. Forced focal and distortion (`-f`, `--k1`, `--k2`) are applied after the merge, on a clone of
the camera when other images share it.

**Features.** `Scene::ExtractFeatures` returns at once when `FEATURES_EXTRACTED` is set. It stores the detector type in
`Scene::Status::nFeaturesType`.

**Pair selection and matching.** `PairsMatcher::Match` first replaces VOCABULARY by EXHAUSTIVE when the scene has fewer
than `1.2 * --vocab-max-pairs` images. VOCABULARY and RETRIEVAL add the sequential prior (`--match-sequence-overlap`).
When verification feedback is on and `--vocab-max-pairs` is at least 10, matching runs in two rounds (VOCABULARY,
RETRIEVAL and KNOWN_POSES only): the first round matches 80 percent of the budget, the second re-invests the rest in pairs
suggested by the verified matches. With `--roma2-match` each round is the dense pass (`MatchPairsROMA2`): a pair the
dense verdict rejects is dropped, never descriptor-matched. RETRIEVAL ranks by global descriptors computed once by the
dense matcher's descriptor graph (`Scene::ComputeGlobalDescriptors`, flag `GLOBAL_DESCRIPTORS`); a scene that already
carries descriptors of the right size does not load the model for it.

**Verification and weighting.** `GeometricFilter` picks one of three branches per pair: shared-focal fundamental matrix,
essential matrix on bearings (both cameras trust their intrinsics), else the seven-degree-of-freedom fundamental matrix.
After all pairs are verified, `CalibrateFocalsAndFitLenses` and `SwitchPlanarTwinsROMA2` run once on the whole graph,
before anything reads the pairs' geometry.

**View-graph filtering.** Scene::Reconstruct runs this step after matching, for a fresh scene and for a resumed one
(both paths call `ExportMatchingCSVsAndFilterPairs`). The weights are computed first, then the pairs CSV is written, then
the triplet filter, then the contradiction cut. The triplet filter removes nothing by default: it scores the pairs, and
the scores weigh the contradiction cut and select the seed views. With `--filter-triplets 0` the contradiction cut still
runs, unweighted. `--match-images-only` returns after this step; it first resolves an automatic `frames.json` camera-axes
convention when known poses were imported, so the saved scene carries resolved poses.

**Hierarchical solver.** `ReconstructHierarchical` extracts the blocks, then reconstructs every block in parallel (build
tracks at `ReconstructionConfig::minPairWeight` 1.22, which has no option; star initializer; incremental resection with its closing full
bundle adjustment; track filter). The seed views of stage 7 are handed to each block's star initializer. When the scene
is not split (no clustering, or at most `--max-views-per-cluster` images), the only block becomes the scene and stages 8c
and 8d do not run. Otherwise the pool resection runs, then the merge.

**Merge.** `MergeInRounds` places the blocks in one frame by a robust averaging over the block graph (readings from
generalized-camera pose estimation of one block's cameras against the other's tracks, or from 3D-3D similarity with
`--cluster-alignment 0`; a reading enters only with two supporting cameras on its weaker side, an observed scale and
the verified image pairs across it not against it), merges tracks of the placed blocks, adjusts the merged scene, resects the images that belong to
no block against the merged model, and repeats until a round places no block and enters no image. Blocks not placed are
kept whole between rounds and released to the final resection at the end. The merge internals are described in
`HierarchicalSFM.md`; this document does not repeat them.

**Global solver.** `ReconstructGlobal` runs global rotation averaging twice (the second time only when the first removed
pairs), builds tracks, solves global positioning (`GlobalPositioner`), filters tracks at 6 px, marks the scene calibrated,
then runs two bundle adjustments (12 iterations with rotations fixed, then 25 with everything free). It has no design
document of its own.

**Known-poses solver.** `ReconstructKnownPoses` requires poses on at least 20 percent of the images and at least 2, else
it fails; resolves the `frames.json` convention; stores the imported poses in `Scene::priorPoses`; builds tracks and
triangulates them with the imported poses at four times `maxReprojError`; marks the images calibrated; runs a first
bundle adjustment of 25 iterations, triangulates again, filters, and runs a second one. It forces refinement of the main
intrinsics when any camera does not trust its intrinsics. The tail of `Reconstruct` then aligns the result back to the
imported pose frame.

**Tail common to every solver.** After the solver returns, `Reconstruct` filters and triangulates tracks, sets
`CALIBRATED`, runs one bundle adjustment, filters weakly connected images, resects the images still unregistered (the
whole-scene resection), aligns to the imported poses or to GPS, refines with GPS priors when `--gps-position-weight` is
set and the scene is geo-aligned, estimates pose uncertainty when `--export-pose-quality` is given, and samples colors
when `--extract-colors` is given.

### Which choices combine

The code behaves as follows. "Supported" means no check refuses or ignores the combination. "Ignored option" means the
code runs without reading the option. "Refused" means the program stops with an error. Each row cites the line.

Dense matcher and match mode.

| Combination | Behaviour | Where |
|---|---|---|
| no dense matcher, match modes 0, 1, 2, 3 | supported | `PairsMatcher.cpp:2088-2178` |
| no dense matcher, mode 4 RETRIEVAL | supported only when every image already carries a global descriptor (a saved scene from an earlier `--roma2` run); otherwise the matching fails with "RETRIEVAL matching needs a global descriptor on every image" | `PairsMatcher.cpp:741` |
| mode -1 SKIP | supported only for a saved scene (`.sfm`) whose pairs hold matches (after `--match-images-only` or ExtractKeyframes): no matching runs and the scene's own matches are used; refused at start for any other source and with `--roma2 1`, and refused by `Scene::Reconstruct` for a scene in which no pair holds a match | `CreateStructure.cpp:Application::Initialize`, `Scene.cpp:Scene::Reconstruct`, `Scene.cpp:Scene::MatchPairs` |
| `--roma2 1` without `--roma2-match` and with mode other than 4 | refused: "--roma2 has nothing to do without --roma2-match true or --match-mode 4" | `CreateStructure.cpp:401` |
| `--roma2 1`, mode 4, no `--roma2-match` | supported: dense descriptors rank the pairs, ordinary descriptor matching verifies them | `Scene.cpp:596-647` |
| `--roma2-match 1` without `--roma2 1` | refused | `CreateStructure.cpp:391` |
| `--roma2 1 --roma2-match 1`, modes 0, 1, 2, 3, 4 | supported: the dense pass is the matching round of each candidate list | `PairsMatcher.cpp:2205` |
| `--dense-matcher flexglue` without `--roma2 1` | refused | `CreateStructure.cpp:345` |
| `--flexglue-model` with `--dense-matcher roma2` | refused | `CreateStructure.cpp:349` |
| `--roma2-model` given with `--dense-matcher flexglue` | refused naming the option (the FlexGlue folder is `--flexglue-model`) | `CreateStructure.cpp:Application::Initialize` |
| `--dense-min-covisibility` given with `--dense-matcher roma2` or with `--roma2 0` | refused naming the option (only FlexGlue gives a covisibility score; RoMa v2 is never gated) | `CreateStructure.cpp:Application::Initialize` |
| `--roma2-model`, `--roma2-slots`, `--roma2-min-confidence`, `--roma2-min-overlap`, `--roma2-dense-matches` or `--roma2-provider` given with `--roma2 0` | refused naming the option (the dense matcher is off, nothing reads it) | `CreateStructure.cpp:Application::Initialize` |
| `--roma2 1` in a build without ONNX Runtime | refused | `CreateStructure.cpp:357` |
| `--roma2-match 1` with no model folder resolvable | refused before any work | `CreateStructure.cpp:417` |
| `--roma2 1 --roma2-match 1` on a scene already `MATCHED` | the model is not loaded and the dense options are ignored, but a missing model path is still refused | `Scene.cpp:618-627` |

Dense matcher and camera type.

| Combination | Behaviour | Where |
|---|---|---|
| dense matcher, perspective images | supported | `MatchROMA2.cpp:SampsonTest` |
| dense matcher, spherical images | supported: the dense code has no camera-type branch; the networks read the equirectangular raster as an image, and a spherical or mixed pair is judged on bearings through the essential matrix | `MatchROMA2.cpp:470-500` (no `CameraType` test in `MatchROMA2.cpp`, `DenseMatcher.cpp`, `GlobalDescriptors.cpp`) |

Camera type and the rest of the pipeline.

| Combination | Behaviour | Where |
|---|---|---|
| spherical images and `--focal-length`, `--k1`, `--k2` | refused: "spherical cameras not supported for forced parameters" | `Scene.cpp:510` |
| spherical image whose width is not twice its height | that image fails to load ("equirectangular input requires width == 2 * height") and is left without a camera; the import itself does not return an error for it | `Image.cpp:331`, `Scene.cpp:405-407` |
| spherical cameras and the lens fit, view-graph focal calibration | ignored: both skip every non-pinhole camera | `LensFit.cpp:837`, `ViewGraphCalibrator.cpp:266` |
| spherical cameras and `--refine-intrinsics` | ignored for them: a spherical camera has no intrinsics block in the bundle adjustment | `SphericalCameras.md` |
| perspective images and `--spherical-bottom-band` | ignored option | option help text |
| spherical images and `--export-pose-quality` | supported, with a warning that the rows do not correlate with the cube-map faces of the MVS export | `BundleAdjustment.cpp:441-444` |
| a scene mixing pinhole and spherical images | supported by the two-view verification (a mixed pair takes the bearing branch and carries no fundamental matrix) | `PairsMatcher.cpp:508-520` |

Solver.

| Combination | Behaviour | Where |
|---|---|---|
| hierarchical (default), spherical or perspective | supported | `Scene.cpp:852-855` |
| `--use-global-solver 1`, spherical or perspective | supported; no check refuses spherical cameras | `Scene.cpp:853` |
| `--use-global-solver 1` with `--max-views-per-cluster`, `--cluster-alignment` | ignored options: the global solver never clusters or merges | `Scene.cpp:1014-1071` |
| `--use-global-solver 1`: the triplet seed views | ignored: only `ReconstructHierarchical` receives them | `Scene.cpp:852-855` |
| known poses (import mode 1 or 2 with a poses file) with `--use-global-solver 1` | ignored option: `ReconstructKnownPoses` is chosen first | `Scene.cpp:852-854` |
| known poses with the cluster options or the star initializer | ignored: the known-poses solver never clusters, merges or builds a star | `Scene.cpp:1073-1200` |
| known poses with fewer than 20 percent of the images posed (at least 2) | refused with an error, after matching has run | `Scene.cpp:1088-1112` |
| known poses and `--match-mode` left at its default | the match mode becomes 3 KNOWN_POSES | `CreateStructure.cpp:528-531` |
| known poses and an explicit `--match-mode` | supported, the explicit mode is used | `CreateStructure.cpp:528` |
| import mode 3 (positions only) | not a known-poses reconstruction: the standard solver runs, the match mode stays at its default | `Scene.h:ReconstructionConfig::HasKnownPoses` |
| known poses with `--roma2` | supported | no check |
| known poses on a resumed scene with import mode 1 | the intrinsics in the file are ignored (warning), poses only | `Scene.cpp:458-468` |
| `--match-images-only` | returns after stage 7; no solver runs | `Scene.cpp:824-834` |
| a `.sfm` already `CALIBRATED` | `Reconstruct` returns false at once; `main` treats it as a partial result, writes the optional exports and does not rewrite the `.sfm` | `Scene.cpp:793-796`, `CreateStructure.cpp:535-545` |

### Saved-scene resume

`--source` can be a single `.sfm` file. `Scene::Import` then calls `Scene::Load` instead of reading images, and the stages
whose flags are set are skipped. The `.sfm` stream carries a layout version (`SFM_PROJECT_VERSION`); `Load` refuses any
other (`Scene.cpp:Scene::Load`). The serialized members are `cameras`, `images`, `pairs`, `tracks`, `colors`,
`poseUncertainty`, `transform`, `obb` and `status` (`Scene.h:Scene::serialize`). `Scene::priorPoses` is not serialized.

The state flags (`Scene::Status::STATE`) and what they decide:

| Flag | Value | Set by | Effect when set on a loaded scene |
|---|---|---|---|
| `FEATURES_EXTRACTED` | 1 | `ExtractFeatures` | feature extraction is skipped ("features already extracted"); `--detector-type`, `--max-features-per-cell`, `--min-features-per-cell`, `--siftgpu-*`, `--spherical-bottom-band` are not read for this scene |
| `MATCHED` | 2 | `MatchPairs` | pair selection and matching are skipped; `--match-mode`, `--vocab-max-pairs`, `--match-sequence-overlap`, `--match-verification-feedback`, `--roma2-*` matching options, `--dense-matcher` are not read (but a missing model path is still refused, see the table above); relative poses are recomputed and the lens fit, focal calibration and planar twin switch run again; stage 6 and 7 run again |
| `CALIBRATED` | 4 | the end of the solver, after `Reconstruct` filters and triangulates | `Reconstruct` returns false at once; see the table above |
| `GEO_ALIGN` | 8 | `AlignToGPS` | GPS-prior refinement runs only when this is set and `--gps-position-weight` is positive |
| `GLOBAL_DESCRIPTORS` | 16 | `ComputeGlobalDescriptors` | RETRIEVAL does not describe the images again, unless the stored descriptors have the size of another dense matcher |

Options read only for a fresh scene (they act in `Scene::Import` on a new image list): the image source list, `--camera-type`,
`--default-focal-ratio`, the camera merge. Options applied to a resumed scene too: `-f`, `--k1`, `--k2`,
`--image-indices` (applied to the loaded cameras, cloning a camera shared with unselected images), and the poses file
(poses only; intrinsics are dropped). Options read in every run, resumed or not: everything from stage 6 onward
(`--filter-triplets` and the other view-graph options, track building, clustering, solver choice,
bundle adjustment, resection, GPS, exports; the track weight bar `ReconstructionConfig::minPairWeight` has no option).

On resume `main` warns for each given option the saved scene does not read: `--camera-type` (its cameras are kept),
`--detector-type` and `--spherical-bottom-band` (when it already holds features). `main` derives the match defaults
(`minMatches`, `matchRatio`, `matchDistance`, `maxDescriptorsPerImage`) from `--detector-type`; `Scene::Reconstruct`
replaces them with the defaults of the stored `nFeaturesType` when a resumed scene holds features of another type (a
scene whose features were imported without a type keeps the configured ones), with a warning.

## Parameters and defaults

The defaults are the ones `CreateStructure --help` prints. The ones without a command-line option live in the config
structs of `libs/SFM`:

| Setting | Default | Defined in |
|---|---|---|
| minimum composite weight for a pair to form tracks and to enter the contradiction cut | 1.22 | `Scene.h:ReconstructionConfig::minPairWeight` |
| minimum composite weight for a pair to be an edge of the clustering graph | 0.9 | `SceneCluster.h:ClusterConfig::minPairWeight` |
| minimum composite weight for a pair to give a focal residual | 0.9 | `ViewGraphCalibrator.h:ViewGraphCalibratorConfig::minPairWeight` |
| triangulation reprojection bar, coarse and fine | 4 px and 2 px | `Scene.h:ReconstructionConfig::maxReprojError`, `maxFineReprojError` |
| minimum triangulation angle | 1.5 degrees | `Scene.h:ReconstructionConfig::minAngleThreshold` |
| clusters aim at | 133 images (two thirds of 200), floor 53 | `SceneCluster.h:ClusterConfig` |
| star size | 4 to 36 views | `StarInitializer.h:StarInitConfig` |
| resection RANSAC bar | 8 px | `Resection.h:ResectionConfig::ransac` |
| local bundle adjustment every, window | 10 images, 25 images | `Resection.h:ResectionConfig` |
| full bundle adjustment every | 25, 50, 100 registered images | `Resection.h:ResectionConfig::fullBAEvery` |
| pair constraints in bundle adjustment: allowed distance of the model's relative pose from a verified pair | 1 degree rotation, 2 degrees baseline direction | `Scene.h:ReconstructionConfig::relativeRotationSigma` |

`CreateStructure` sets `cfg.viewgraphCfg.maxTwoViewError = 0` (`CreateStructure.cpp:502`), which disables the pair
filtering of the view-graph focal calibration.

## Invariants

- The order of stages 1 to 7 is the same for every solver; the solver choice (stage 8) is made once, after the view graph
  is final.
- Pair weights are recomputed at the start of stage 6, so a saved scene's weights never reach a reconstruction.
- Every consumer of pair geometry runs after the lens fit and the planar twin switch (`Scene::MatchPairs`).
- Stages 8c and 8d run only when stage 8a produced more than one block (`SceneBlocks::IsSplit`).
- `CALIBRATED` is set by the tail of `Reconstruct` whenever the solver returned true, whatever number of images it
  registered.

## Rejected alternatives

- No fallback from the dense pass to descriptor matching for a rejected pair (`PairsMatcher.cpp:2205`).
- No automatic switch of the match mode when `--roma2` is given without the dense pass or mode 4: the parser refuses it.

## Open items

- The global solver (`ReconstructGlobal`, `GlobalRotationAveraging`, `GlobalPositioning`) has no design document.
- When `Reconstruct` fails after a calibrated state, `main` exports but does not save the `.sfm`.
