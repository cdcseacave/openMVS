# SFM Library

Structure from Motion pipeline that reconstructs 3D scenes from multiple images. Converts feature matches into calibrated camera poses and sparse 3D point clouds. Supports incremental, global, and hierarchical reconstruction strategies.

## Core Data Structures

### Scene (`Scene.h`)
Central container for all SFM data:
```cpp
class Scene {
    CameraArr cameras;        // Shared camera models (polymorphic)
    ImageArr images;          // Per-image features, descriptors, poses
    ImagePairArr pairs;       // Pairwise matches and geometry
    TrackArr tracks;          // 3D points with multi-view observations
    ColorArr colors;          // Per-track RGB colors
    PoseUncertaintyArr poseUncertainty; // Optional per-image pose covariance from the last global BA
    Transform transform;      // Similarity transform (GPS alignment)
    OBB3f obb;               // Scene bounding box
    Status status;            // Pipeline state tracking
    BS::light_thread_pool threadPool;
};
```
`poseUncertainty` is filled during `Scene::Reconstruct` when `ReconstructionConfig::estimatePoseUncertainty`
is set, serialized with the scene, and kept consistent with the world frame by `Scene::Transform`
(position covariance maps as `scale^2 * R * Cov * R^T`; rotation variance is about the camera axes,
untouched). Exported as a per-image CSV quality report by `ExportPoseUncertaintyCSV`.

`priorPoses` (`std::unordered_map<IIndex, Pose3D>`, keyed by image ID) snapshots the poses as imported,
before any refinement, and is consumed by `Scene::AlignToPriorPoses`. It is **transient**: deliberately
not serialized, but preserved by regular `Scene` copies and moves.

### Camera Hierarchy (`Camera.h`) - Polymorphic
- **Camera** (abstract base): Virtual `Project()`, `Unproject()`, `GetK()`, `AccumulateIntrinsics()`, `ScaleIntrinsics()`
- **PinholeCamera**: `fx, fy, cx, cy` + Brown-Conrady distortion `k1-k6, p1, p2`. Flag `useAdditionalDistortion` for k4-k6
- **SphericalCamera**: Equirectangular 360 projection, no distortion params

### Image / View (`Image.h`, `View.h`)
```cpp
class Image : public View {
    KeypointArr keypoints;    // cv::KeyPoint features
    cv::Mat descriptors;      // CV_8U or CV_32F descriptors
    String fileName;
    Metadata metadata;        // EXIF, GPS, timestamp
};
class View : public Pose3D {
    uint32_t cameraID;
    CameraPtr pCamera;        // Shared camera pointer
};
```

### Pose3D (`Pose.h`)
```cpp
class Pose3D {
    RMatrix R;   // 3x3 rotation (world-to-camera)
    CMatrix C;   // 3D camera center (world coordinates)
    // Operators: * (compose), / (relative pose)
    // TransformPointW2C(), TransformPointC2W()
};
```

### Track (`Track.h`)
3D point with multi-view observations:
```cpp
class Track {
    Point3 position;
    ObservationArr observations;  // (imageID, featureID) pairs
    uint8_t numInliers;           // First N observations are inliers (max 255)
};
```
`BuildTracks` unions the track-forming matches of every pair above `minPairWeight` plainly, then
settles each component holding one image twice on its own links (`TrackConflictConfig`,
`ReconstructionConfig::trackConflictCfg`, mirrored in `ClusterConfig`): the least-supported link
(fewest triangles, then the lighter pair) on the path between the two keypoints is cut until every
piece is a track (`cut`, default), or the union is vetoed in pair order (`cut` false, and always for
a component above `maxComponentSize`, 5000); two keypoints of one image within `mergeDistance`
(3 px) are one feature, the one with more links stands for the image. On alameda the veto keeps the
wrong keypoint in 22% of the judgeable conflicts, the cut in 17%.

### ImagePair (`ImagePair.h`)
```cpp
class ImagePair {
    MatchArr matches;              // Inlier feature matches
    MatchArr outlierMatches;
    std::optional<Matrix3> F, E, H; // Fundamental/Essential/Homography
    Pose3D relativePose;
    float weightSpatial, weightConnectivity, weightTriplet;
    float overlapRatio, meanRayAngle;
};
```

## Pipeline Workflows

### Incremental Reconstruction (`Scene::Reconstruct`)
```
Extract features (AKAZE/ORB/SIFT/SIFTGPU) -> Match pairs (VOCABULARY/EXHAUSTIVE/SEQUENTIAL/KNOWN_POSES)
-> Geometric verification (E/F/H + RANSAC) -> Lens fit from pairs (k1, k2 of every camera with none, the focal of every untrusted one from its import focal; its pairs verified again through it) -> View graph calibration (focal estimation of the cameras the lens fit could not take)
-> Build tracks (union-find) -> Filter tracks + weak images
-> Star initialization (reference view) -> Resect remaining images (incremental)
-> Bundle adjustment (global + local)
-> Optional GPS alignment (rigid Sim(3) to metric ENU), or, in known-poses mode, Sim(3)
   re-alignment back to the imported pose frame (AlignToPriorPoses; no GEO_ALIGN)
-> Optional GPS-prior BA (BAConfig::gpsPositionWeight/Z > 0; must run post-alignment:
   the residuals are gated on GEO_ALIGN and their meters-vs-pixels weighting assumes ENU)
-> Optional pose-uncertainty recording (estimatePoseUncertainty): covariance read off the
   last global BA (final BA, superseded by the GPS-prior BA when it runs) BEFORE FilterTracks
   invalidates the parameter blocks the solved ceres problem references
```

### Hierarchical Reconstruction (`Scene::ReconstructHierarchical`)
```
[Pipeline up to track building]
-> Cluster scene (aggregative partitioning)
-> For each cluster: extract sub-scene -> full pipeline
-> Global alignment (block-graph averaging + merge + loose-bar adjustment) -> Final BA
```

### Global Reconstruction (`Scene::ReconstructGlobal`)
```
[Match + build tracks] -> Compute relative poses
-> Global rotation averaging -> Global positioning (translations + points)
```

### Known-Poses / Finetune Reconstruction (`Scene::ReconstructKnownPoses`)
Selected by `ReconstructionConfig::HasKnownPoses()` (a poses file is configured with
`PoseImportMode::POSES_INTRINSICS` or `POSES`); the imported poses are initialization only.
```
[Match + build tracks] -> Validate >=20% of the images got a pose (fail loudly, listing the
   unmatched names - never silently fall back to standard SfM; the gate catches a file-name
   mismatch, partially covered captures are legitimate)
-> Resolve the frames.json camera-axes convention (ResolveFramesConvention, only when AUTO)
-> Snapshot the imported poses into Scene::priorPoses
-> BuildTracks -> TriangulateTracks with 4x maxReprojError -> FilterTracks
-> RecomputeCalibratedImages + set CALIBRATED
-> Finetune BA (forces RefineMainIntrinsics when any camera has !TrustIntrinsics)
-> Re-triangulate outliers + FilterTracks -> second BA
```
Then falls through to the shared tail of `Scene::Reconstruct` (pre-final BA,
filtering, final BA, `FilterWeaklyConnectedImages`, resection of the images missing from the
poses file, alignment, colors). A failed final similarity alignment back to the imported frame
is reported as a warning and leaves the scene in the refined (arbitrary-gauge) frame; when prior
poses exist the alignment takes precedence over GPS. Clustering is never involved.

## Key Algorithms

### Feature Extraction (`FeaturesExtractor.h`)
- Detectors: **AKAZE** (default), **ORB**, **SIFT**, **SiftGPU** (optional)
- 3x3 spatial grid for even distribution: `maxFeaturesPerCell` (default 3000, ~27k total)
- Keypoint weighting: `ComputeKeypointWeight()`, `ComputeKeypointPrecision()`

### Feature Matching (`PairsMatcher.h`, `MatchGeometric.h`)
- **Matching modes**: `EXHAUSTIVE` (all pairs), `VOCABULARY` (reciprocal-rank-fused retrieval,
  mutual top-K + connectivity bridges), `SEQUENTIAL` (video),
  `KNOWN_POSES` (pose-guided: `CollectKnownPosePairs` scores baseline - normalized by the median
  nearest-neighbor camera distance - times viewing-direction agreement, rejects optical-axis angles
  > 75 deg, keeps the pairs present in the candidate lists of BOTH endpoints, plus each image's
  2 nearest cameras ungated (occlusion safeguard) and max-score component bridges; incomplete
  pose sets use vocabulary retrieval for pairs touching unposed images; falls back to exhaustive
  if fewer than two poses are usable or their camera centers are degenerate.
  No camera-center cheirality test: orbit and nadir captures put the neighbor centers tangential /
  perpendicular to the view direction, so the strongest pairs would fail it)
- **Verification feedback** (`CollectVerificationFeedbackPairs`, on by default for
  VOCABULARY/KNOWN_POSES with `maxPairsPerImage >= 10`): matching runs in two rounds - the first
  collects from uninflated per-image lists at 80% of the target, then the remaining budget (up to
  `maxPairsPerImage*N/2` total pairs) goes to pairs suggested by the geometrically verified matches: KNOWN_POSES closes
  the triangles of the verified pair graph (ranked by common verified neighbors), VOCABULARY
  propagates each verified pair to its endpoints' top-5 retrieval candidates; the images with the
  weakest verified connectivity refill any leftover budget (2 pairs/image) from their next
  best-ranked first-round candidates
- Lowe's ratio test, cross-check, FLANN (LSH/KDTree)
- **ROMAv2 one-pass dense matching** (`MatchROMA2.h`, `ROMA2Warp.h` helpers, `--roma2-match`): the
  bidirectional dense warps are computed in-process (no external files) and, when enabled, ARE the
  matching round - `JudgePairROMA2` admits a pair on the smaller of the two inlier areas one fitted
  geometry explains (`--roma2-min-overlap`), `TrackKeypointsByWarp` + `MatchFeaturesGuided` select
  its sparse matches in an epipolar band centred on each warp prediction, `AssemblePairROMA2` fills the rest
  of the overlap densely and fits the pair's geometry, `StorePairROMA2` stores it. A rejected pair
  is dropped, never descriptor-matched
- **Geometric verification**: RANSAC for E (calibrated) or F (uncalibrated), optional H
- Threshold: `maxEpipolarError` (pixels), min inliers (default 50)

### Pair Weighting (`PairsWeighting.h`)
Composite weight = `spatial x connectivity x triplet`
- **Spatial**: Grid-based feature coverage across image
- **Connectivity**: Relative importance in local graph
- **Triplet**: 3-view loop consistency (most reliable)

### Triangulation (`Triangulation.h`)
- **DLT** (Direct Linear Transform): Fast, linear, assumes inliers
- **Skew-Symmetric LLS**: More robust, returns inlier count
- Filters: reprojection error, triangulation angle, depth bounds

### Bundle Adjustment (`BundleAdjustment.h`, `BundleAdjustmentCostFunctions.h`)
Ceres Solver-based non-linear optimization.
- Refines: poses (R, C), points, intrinsics (focal, principal point, distortion k1-k6, p1-p2)
- Parameterization: angle-axis or quaternion for rotation
- Robust loss: Huber with configurable threshold
- Variants: Global BA, Local BA (windowed)
- Solver: DENSE_SCHUR under 500000 reprojection residuals, SPARSE_SCHUR with inner iterations past it;
  the inner iterations get their block order from `ComputeInnerIterationOrdering` (Ceres's own
  rounds of independent sets, ties broken by the order the residuals name the blocks), since Ceres
  left to itself breaks those ties by the heap addresses of its block records and the same scene
  then solves differently from one run to the next
- Optional GPS position constraints (`GPSPositionError`: camera center vs GPS in ENU, weighted by
  per-image accuracy metadata with 10 m / 20 m fallbacks; gated on the GEO_ALIGN scene state; when
  present the gauge is anchored, so no reference pose is held fixed)
- **Pose covariance** (`ComputePoseUncertainty`, call on the instance after `Adjust`): Schur-eliminates
  the 3D points, reads the per-pose 6x6 marginal blocks off a sparse selected inverse (Takahashi).
  `PoseUncertainty` = rotation variance about the camera axes (rad^2) + full 3x3 world-frame
  camera-center covariance (diag + off-diag; the position tangent is the plain world camera center).
  Gauge semantics: with GPS priors the covariance is absolute (ENU); otherwise it is relative to a
  datum image (reported as exactly 0) and the global SCALE gauge stays unanchored, so magnitudes
  saturate at the damping ceiling along the scale mode — treat non-GPS values as relative trust only.

### Star Initializer (`StarInitializer.h`)
Reference view selection (highest connectivity) + star configuration initialization.
Config: `minViews` (4), `maxViews` (36).

### Incremental Resection (`Resection.h`)
Register images one at a time via 2D-3D PnP + RANSAC. Periodic local/global BA.

### Scene Clustering (`SceneCluster.h`)
Aggregative clustering on covisibility graph for hierarchical reconstruction.
- Bottom-up: merge highest-weight edges, aiming for `targetViewsPerCluster` (two thirds of `maxViewsPerCluster`, default 150), never past the ceiling itself
- `maxOverCapacity` (20): extra images allowed over the ceiling when absorbing a small cluster
- Refinement (seven passes): local search, merge small clusters, balance load across clusters, split disconnected components, split thin-waisted clusters, merge away or widen any cluster boundary the merge could not register against (`IsStrongSeam`), rescue orphans
- Data split: keypoints/descriptors MOVED (not copied) to sub-scenes

### Global Alignment (`GlobalAlignment.h`)
Places the independently reconstructed blocks ("sub-scenes") in one frame by one robust averaging of the block graph, the view-graph filter's pattern over blocks:
1. **Readings** (`EstimateSeamCandidates`/`EstimateSeamPair`): every block pair the cross pairs join, read once per direction by generalized-camera PnP with scale of one block's cameras against the other's tracks, refined over its inliers (`ALIGN_CAMERAS`, default; a one-camera rig is a door read by its camera's own pose), or by one 3D-3D similarity (`ALIGN_POINTS`, `--cluster-alignment 0`); the two directions stay two readings. Each scored on every correspondence of the pair (the supporting cameras of its weaker side, `minCameraInliers` 10) and read against the verified pairs across it (`minPairInliers` 30, `maxPairRotationResidual` 5 deg); weight = max(cameras, 1) x the pairs' share
2. **Selection** (`SelectSeamGraph` / `AverageBlockPoses`): every component of the kept readings averaged (rotations by `GlobalRotationEstimator`, log-scales and translations of the camera centroids under the inverse-error reweighting of the contradiction cut), every reading judged against its component's poses at 5 deg / 1.20 / 0.20 of the pair footprint, repeated until the kept set settles (`kRobustRounds`); a bridge keeps a reading only with `minSeamWeight` (2) cameras on its weaker side and its verified pairs not against it; each component left is a model
3. **Merge** (`MergeScenes`): apply the poses of the model holding the most images, average shared camera intrinsics, union-find the tracks (every correspondence of every placed block pair within the loose bar, 3 x 4 px, joins by construction); every other block is merged without a pose, for the resection to recover its images; then `AdjustMergedScene` (Scene.h) filters at the loose bar and adjusts once
Known limits: a non-rigid block can be placed wrong; a heavy wrong chord with no verified pairs, two equal readings of a bridge, and a copy heavier than the truth on a door's triangle are not caught (`docs/design/HierarchicalSFM.md`)

### Rotation Averaging (`GlobalRotationAveraging.h`)
- MST initialization (weighted by match counts)
- L1 minimization (tangent space, angle-axis)
- IRLS refinement (Geman-McClure or Half-Norm loss)
- Options: `maxRelativeRotationAngle`, `skipInitialization` (warm-start)

### View Graph Calibration (`ViewGraphCalibrator.h`)
Global focal length estimation across all image pairs. Uses Fetzer method with robust loss. Only for a
camera still untrusted after the lens fit (`CalibrateFocalsAndFitLenses`): it reads the uncorrected lens as focal.

### Lens Fit From Pairs (`LensFit.h`)
Once the pairs are verified, `CalibrateFocalsAndFitLenses` trusts every untrusted pinhole camera the lens fit
can take (no lens given) at its import focal (an EXIF 35 mm-equivalent focal, the default ratio of the long
side), poses its pairs from their F there (E = K2^T F K1, pairs under 5 sparse inliers left) and has
`FitLensesAndReverifyPairs` fit the radial lens (k1, k2) of every pinhole camera that has none and trusts
its focal, from its verified pairs (`FitLensesFromPairs`): the heaviest `MatchConfig::lensFitMaxPairs` (150)
that turn 5-45 deg, 200 sparse inliers each, the Sampson error in measured pixels under soft L1 at 1 px,
each pair's 5-dof pose eliminated, up to `lensFitRounds` (10) rounds of each pose re-estimated through the
lens, the check of the fit and its restart from the lens doubled (these settings are `static constexpr`
members of `MatchConfig`). Then the focal check: from the lenses and poses that fit reached, the joint fit
runs again with focals free (a log scale, a third number beside k1, k2), with the same check and restart,
first for the estimated focals (the untrusted starts), taken with no bar, then for the trusted ones,
judged on the fit whose estimated focals already moved: a trusted camera takes the free fit's focal (fx, fy
scaled alike) when its median error falls 10% below the held fit's (`MatchConfig::lensFitFocalMinErrorDrop`,
a constant); the fit is then built again at a moved focal from the free fit's poses and runs again, and a
focal kept leaves the held fit as the fit. A camera whose focal the lens fit does not take (too few pairs,
an unsolved or folding fit) goes back to untrusted, with no lens and its pairs to their F, before the pairs
of the changed cameras are verified again (so those it shares with them through F, not through an E at its
start), and every camera still untrusted
(also one with a lens given, and all with the lens fit off) is estimated by the view-graph calibration, its
pairs then verified through E. A lens is kept when it is one to one past the image corner (1.05x) and lowers the median error 10% below the poses' own refinement; a
camera under 20 eligible pairs, spherical, untrusted, or with a given lens past k1, k2 or that folds
within the frame (not one to one out to 1.05x its corner) is left as it is, its pairs out of the fit.
Every pair of a camera whose lens was fitted or whose focal moved (`LensFitReport::CamerasToVerifyAgain`)
is then verified again through `PairsMatcher::ComputeRelativePoses` (a RoMa2 pair through
`ReverifyPairROMA2`), and the summary line counts the pairs that verification invalidated.
The bundle adjustments refine k1, k2 from the fitted lens. No command-line flag;
`MatchConfig::lensFitEnabled` turns it off. A lens given with `--k1/--k2` or imported is held, and a
resumed scene's fitted lens counts as given; a resumed scene fits again every camera that still has
no lens (refused, under the pair minimum, or matched by an older build), so a camera refused beside
one fitted the first time, now held, can be kept.

### Relative Pose Refinement (`RelativePoseRefine.h`)
Joint refinement of focal length + distortion (k1, k2) with relative pose via Ceres.

## External Format Support
- **COLMAP import** (`ImportCOLMAP.h`): Binary reconstruction, selective import
- **frames.json import** (`PoseIO.h`): Polycam-style array of `{name, transform[16], params?}`;
  `transform` is a column-major 4x4 camera-to-world matrix, `params` an optional OPENCV intrinsics block
  (`w,h,fx,fy,cx,cy,k1,k2,p1,p2`) declared for its own resolution and rescaled to the image.
  Entries are matched to images by file name (full name, then stem, both case-insensitive).
  Runs before the camera de-duplication in `Scene::Import`, so identical per-frame intrinsics collapse
  into one shared `Camera`. Two gotchas it handles: the file does not declare its camera-axes
  convention (`FramesConvention::ARKIT|OPENCV`, differing by a pi rotation about X - see
  `DetectFramesConvention` / `FlipFramesConvention`), and EXIF-portrait images are rotated 90 deg
  clockwise on load, so both `R` (composed with `Rz(+90 deg)`, the inverse of `View::RevertRotation`)
  and the imported intrinsics are rotated to match, while the camera center is unchanged
- **Pose CSV import/export** (`PoseIO.h`: `ImportPosesCSV` / `ExportPosesCSV`): `PoseImportMode`
  = `NONE` / `POSES_INTRINSICS` (intrinsics when the row has them, plus R + C, marks
  `trustIntrinsics`) / `POSES` (R + C) / `POSITIONS` (C only). Rows match by file-name stem
  (case-insensitive, ambiguous stems rejected) and `score <= 0` invalidates the pose.
  `ImportPoses` (`PoseIO.h`) dispatches on extension: `.csv` here, `.json` to `ImportFramesJSON`
- **MVS export** (`InterfaceMVS.h`): Conversion to MVS binary format. The SFM image ID is written
  into `Interface::Image::ID` (the external/global-ID field) so per-image data keyed by SFM ID —
  e.g. the pose-quality CSV — correlates after import; `Vertex::View::imageID` stays the
  images-ARRAY-POSITION (hard invariant), tracked separately from the IDs. Spherical cube-map faces
  get fresh unique IDs past the largest SFM ID (depth-map file names are derived from the ID and
  must not collide)
- **PLY export**: Tracks as 3D points with optional colors

## Memory Management
- Camera models shared via `CameraPtr` (reference counted)
- Keypoints/descriptors MOVED between global and sub-scenes during split/merge
- Lazy pixel loading: `LoadPixels()` / `ReleasePixels()`
- `Scene::Release()` for manual cleanup

## Build & Dependencies
- **Required**: Common, Math, IO, Ceres Solver, PoseLib, TinyEXIF
- **Optional**: SiftGPU (CUDA/OpenGL)
- **Inherited**: Eigen3, OpenCV, Boost
- **Precompiled header**: `Common.h`

## Debugging / fast iteration
When iterating on the reconstruction stages (clustering, sub-scene recon, global
alignment/merge, BA, weak-image filtering), **skip feature extraction + matching** by
resuming from the post-matching scene — **pass the matched `.sfm` as the input source
instead of the images folder**. No code change needed:

- A normal `-v 3` run writes `scene_pre_reconstruction.sfm` to the working folder (the
  post-matching scene, carrying the `FEATURES_EXTRACTED` + `MATCHED` state flags).
- **Rename it first** (e.g. to `matches.sfm`) so the new run cannot overwrite it.
- Feed it as the source: `CreateStructure matches.sfm -o scene.sfm -v 3`. `Scene::Import`
  detects the single `.sfm` and `Load`s it; `ExtractFeatures` and `MatchPairs` then
  early-return on the `FEATURES_EXTRACTED` / `MATCHED` flags, so reconstruction starts
  immediately.
- For pure reconstruction debugging, omit `--export-mvs` and `--extract-colors` to skip
  the slow undistortion / MVS export / color-sampling tail.
- This turns a ~full pipeline run into just the reconstruction (e.g. on Tanks&Temples
  Courthouse, 1106 imgs: skips ~33s features + several min matching).
- To capture extra debug state mid-pipeline, temporarily add a `Save(MAKE_PATH("dbg.sfm"))`
  and feed `dbg.sfm` back in the same way.
