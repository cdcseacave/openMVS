# Global Solver

## Purpose and scope

The global solver is the alternative to the hierarchical solver (`HierarchicalSFM.md`, `IncrementalReconstruction.md`).
It is selected with `CreateStructure --use-global-solver 1` (`ReconstructionConfig::useGlobalSolver`) and runs as
`Scene::ReconstructGlobal` (`libs/SFM/Scene.cpp`), the solver stage of `SfMPipeline.md`. Instead of growing a model
image by image inside blocks, it averages the relative rotations of the whole view graph at once, places every camera
centre and every track point in one positioning solve over rays, and refines the result with two bundle adjustments. It
has no blocks, no star initializer, no resection and no merge.

Scope: the solver itself. The stages before it (matching, pair weighting, the triplet filter and the contradiction cut)
and the tail every solver shares (final adjustment, resection of the leftover images, alignment, colors) are in
`SfMPipeline.md`. The code is adapted from GLOMAP ("GLOMAP: Global Structure-from-Motion Revisited", arXiv:2407.20219).

## Algorithm as implemented

`Scene::ReconstructGlobal(config)` takes the filtered view graph (`scene.pairs` with relative poses and composite
weights) and the config of the run. It returns at once, true, for a scene already `CALIBRATED`. Otherwise:

1. **Global rotation averaging** (`GlobalRotationAveraging.cpp:GlobalRotationEstimator::EstimateRotations(Scene&)`).
   - The input is every pair for which `IsPoseLinkPair` holds (a relative pose and a valid composite weight), each as a
     relative rotation weighted by `ImagePair::GetCompositeWeight`.
   - Initialization: the maximum spanning tree of the weights (`RotationsFromMaximumSpanningTree`), rotations carried
     out from its best connected node at the identity, over the largest connected component only. The images outside
     that component receive no rotation and so stay unregistered.
   - L1 stage (`SolveL1Regression`): at most `maxNumL1Iterations` (5) rounds of a least-absolute-deviation solve (ADMM,
     10 iterations each) on the linearised system, ended when the average step is under 0.001.
   - IRLS stage (`SolveIRLS`): at most `maxNumIrlsIterations` (100) rounds of iteratively reweighted least squares with
     the Geman-McClure weight at `irlsLossParameterSigma` = 5 degrees, ended when the average step is under 0.001.
   - Filter (`FilterRelativeRotations`): every pair whose stored relative rotation is more than
     `maxRelativeRotationAngle` (12 degrees) from the one the averaged rotations give gets its weight invalidated
     (`ImagePair::InvalidateWeight`), which takes it out of every later read of the view graph.
   - When the filter invalidated at least one pair the estimator runs a second time from the start (the tree is built
     again on the surviving pairs) and filters once more, without a further run. A failure of either run fails the
     solver.
   - Every image that received a rotation is valid from here on, with its centre set to the origin.
2. **Tracks** (`Track.cpp:BuildTracks`, `TrackBuilding.md`): over the pairs whose weight exceeds
   `ReconstructionConfig::minPairWeight`, with the default `TrackConflictConfig` (the cut rule).
3. **Global positioning** (`GlobalPositioning.cpp:GlobalPositioner::Solve`), with `GlobalPositionerOptions` at their
   defaults: constraint type ONLY_POINTS.
   - Every valid image starts at a random centre in [-100, 100]^3, every track of at least `minNumViewPerTrack` (3)
     views at a random position in the same cube (seed 123, so the start is the same every run).
   - Every observation of such a track in a valid image is one residual `ray - s * (X - C)`: the unit world-frame ray of
     the keypoint (`Image::Ray`, undistorted by the image's camera, rotated by the averaged rotation) against the
     direction from the camera centre `C` to the point `X`, with one free scale `s >= 1e-5` per observation.
     The residual is robustified by a Huber loss with threshold `thLossFunction` = 0.1. Observations of a camera that
     does not trust its intrinsics (`Image::TrustIntrinsics`) weigh half.
   - The first scale is held constant, which removes the gauge (position, scale of the scene).
   - Solved by Ceres: SPARSE_SCHUR with the CLUSTER_TRIDIAGONAL preconditioner, scales eliminated first, then points,
     then cameras; at most 200 iterations, function tolerance 1e-5; a CUDA build offloads the dense algebra to the GPU
     when the scene has at least 50 images and Ceres supports it. The camera-to-camera constraint types of the options (ONLY_CAMERAS,
     POINTS_AND_CAMERAS, POINTS_AND_CAMERAS_BALANCED) are implemented (`AddCameraToCameraConstraints`) and not selected.
   - The solve fails when it has no usable solution.
4. **Filter and state**: `FilterTracks` at 6 px and 1 degree (the depth multipliers at their defaults), the number of
   calibrated images is recounted (`RecomputeCalibratedImages`), and the scene is marked `CALIBRATED`.
5. **Two bundle adjustments** (`BundleAdjustment.md`), from a copy of `config.baConfig` with the minimum triangulation
   angle set to `minAngleThreshold`: first 12 iterations with the rotations held fixed (positions and structure only),
   `FilterTracks` at `maxReprojError`, then 25 iterations with the rotations free, `FilterTracks` again, and a
   triangulation of every track (`TriangulateTracks`). The intrinsics are not refined by these two (the BAConfig default
   refines none; `SetBAIntrinsicFlags` is applied only to the adjustments of the shared tail).

Then `Scene::Reconstruct` runs the tail common to every solver (`SfMPipeline.md`): filtering and triangulation of the
tracks, the final adjustment with the intrinsics of `--refine-intrinsics`, `FilterWeaklyConnectedImages`, the resection
of every unregistered image and the closing full adjustment, alignment, colors. That resection is what registers the
images the rotation averaging did not reach, when it can.

## Parameters and defaults

The solver reads these options through the config; the ones of the solver itself (`GlobalRotationEstimatorOptions`,
`GlobalPositionerOptions`) have no command-line option and are at the values above.

| Setting | Default | Defined in |
|---|---|---|
| pair weight bar for the tracks | 1.22 | `Scene.h:ReconstructionConfig::minPairWeight` |
| relative-rotation filter after the averaging | 12 degrees | `GlobalRotationAveraging.h:GlobalRotationEstimatorOptions::maxRelativeRotationAngle` |
| tracks entering the positioning: minimum views | 3 | `GlobalPositioning.h:GlobalPositionerOptions::minNumViewPerTrack` |
| bundle adjustment of the solver: iterations | 12 (rotations fixed), then 25 | `Scene.cpp:ReconstructGlobal` |
| reprojection bar after the positioning | 6 px, 1 degree | `Scene.cpp:ReconstructGlobal` |
| reprojection bar after the adjustments | `maxReprojError` (4 px) | `Scene.h:ReconstructionConfig::maxReprojError` |

Options the global solver does not read, and which `CreateStructure` therefore refuses when they are given together with
`--use-global-solver 1`: `--max-views-per-cluster` and `--cluster-alignment` (the solver never clusters or merges). Known
poses (`--import-poses-mode 1` or `2`) take `ReconstructKnownPoses` before the solver choice reaches
`--use-global-solver`, so the combination is refused as well. The triplet filter's seed views are read by the star
initializer alone, which this solver does not run; the options of the triplet filter still act on the view graph before
the solver, so they are not refused.

## Invariants

- The view graph the solver reads has relative poses and valid weights on the pairs it uses (`IsPoseLinkPair`); the
  rotation averaging asserts that both images of such a pair have a camera.
- An image is registered by the solver only when it lies in the largest connected component of the weighted view graph
  after the rotation filter; every other image is left to the tail's resection.
- Positioning starts from random centres and points, with a fixed seed: a run is repeatable on the same input.
- `CALIBRATED` is set before the two adjustments, whatever number of images the positioning placed.

## Known limits

- There is one validation only: on one 40-image capture the global solver registered 13 images where the default
  hierarchical pipeline registers 27. No other capture was measured, and no accuracy comparison exists.
- Nothing in the solver checks a direction or a loop: a wrong pair that survives the rotation filter (a repeated
  structure whose rotation is consistent) places images wrongly, and the two adjustments then fit to that placement.
  The hierarchical solver's direction check of every resection has no counterpart here.
- The positioning uses points only: an image that sees no track of three views has no constraint and is not placed
  (the camera-to-camera constraint types exist but are not selected).
- Intrinsics are held during the solver's own adjustments; a scene whose focal is wrong is refined only in the tail.

## Rejected alternatives

- Camera-to-camera constraints from the relative translations: not selected, ONLY_POINTS is the one the GLOMAP paper
  recommends (`GlobalPositionerOptions::ConstraintType`).

## Open items

- A measurement of the solver on more captures, and a decision whether it stays a first-class option.
- `GlobalSolverTest` (`apps/Tests/TestsSFM.cpp`) runs `ReconstructGlobal` on the four bundled images and asserts that
  every image registers with points; no test measures the accuracy of the result or runs it through the
  `--use-global-solver` route of `Reconstruct`.
