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
    Transform transform;      // Similarity transform (GPS alignment)
    OBB3f obb;               // Scene bounding box
    Status status;            // Pipeline state tracking
    BS::light_thread_pool threadPool;
};
```

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
Extract features (AKAZE/ORB/SIFT/SIFTGPU) -> Match pairs (VOCABULARY/EXHAUSTIVE/SEQUENTIAL)
-> Geometric verification (E/F/H + RANSAC) -> View graph calibration (focal estimation)
-> Build tracks (union-find) -> Filter tracks + weak images
-> Star initialization (reference view) -> Resect remaining images (incremental)
-> Bundle adjustment (global + local)
-> Optional GPS alignment
```

### Hierarchical Reconstruction (`Scene::ReconstructHierarchical`)
```
[Pipeline up to track building]
-> Cluster scene (aggregative partitioning)
-> For each cluster: extract sub-scene -> full pipeline
-> Global alignment (5-stage merge) -> Final BA
```

### Global Reconstruction (`Scene::ReconstructGlobal`)
```
[Match + build tracks] -> Compute relative poses
-> Global rotation averaging -> Global positioning (translations + points)
```

## Key Algorithms

### Feature Extraction (`FeaturesExtractor.h`)
- Detectors: **AKAZE** (default), **ORB**, **SIFT**, **SiftGPU** (optional)
- 3x3 spatial grid for even distribution: `maxFeaturesPerCell` (default 3000, ~27k total)
- Keypoint weighting: `ComputeKeypointWeight()`, `ComputeKeypointPrecision()`

### Feature Matching (`PairsMatcher.h`, `MatchGeometric.h`)
- **Matching modes**: `EXHAUSTIVE` (all pairs), `VOCABULARY` (top-K retrieval), `SEQUENTIAL` (video)
- Lowe's ratio test, cross-check, FLANN (LSH/KDTree)
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
- Optional GPS position constraints

### Star Initializer (`StarInitializer.h`)
Reference view selection (highest connectivity) + star configuration initialization.
Config: `minViews` (4), `maxViews` (36), `minTracksPerView` (50).

### Incremental Resection (`Resection.h`)
Register images one at a time via 2D-3D PnP + RANSAC. Periodic local/global BA.

### Scene Clustering (`SceneCluster.h`)
Aggregative clustering on covisibility graph for hierarchical reconstruction.
- Bottom-up: merge highest-weight edges until clusters <= `maxViewsPerCluster` (200)
- `maxOverCapacity` (20): allows clusters to exceed `maxViewsPerCluster` when absorbing orphan views that have no other viable cluster
- Refinement: merge small clusters, local search, split disconnected components
- Data split: keypoints/descriptors MOVED (not copied) to sub-scenes

### Global Alignment (`GlobalAlignment.h`)
5-stage merge for hierarchical reconstruction:
1. **Relative similarity transforms**: 7-DOF Sim(3) between sub-scene pairs via RANSAC over 3D-3D point correspondences (`SimilarityTransform.h::EstimateSimilarityTransform`). Correspondences are collected from cross-sub-scene image-pair matches whose endpoints both lie on existing inlier tracks in the two sub-scenes, so the per-sub-scene triangulated points are paired directly — no assumption that the sub-scene rigs share a common scale. `ScenePair` carries the full `Transform` (R, t, scale); downstream stages read scale off `relativeTransform.scale` instead of recomputing it.
2. **Rotation averaging** (`GlobalRotationAveraging.h`): MST init + L1-ADMM + IRLS on SO(3)
3. **Scale averaging** (`GlobalScaleAveraging.h`): Log-space least-squares, fed directly from `relativeTransform.scale`
4. **Translation averaging** (`GlobalTranslationAveraging.h`): Linear system solve
5. **Merge**: Apply similarity transforms, average shared camera intrinsics, union-find on tracks

### Rotation Averaging (`GlobalRotationAveraging.h`)
- MST initialization (weighted by match counts)
- L1 minimization (tangent space, angle-axis)
- IRLS refinement (Geman-McClure or Half-Norm loss)
- Options: `maxRelativeRotationAngle`, `skipInitialization` (warm-start)

### View Graph Calibration (`ViewGraphCalibrator.h`)
Global focal length estimation across all image pairs. Uses Fetzer method with robust loss.

### Relative Pose Refinement (`RelativePoseRefine.h`)
Joint refinement of focal length + distortion (k1, k2) with relative pose via Ceres.

## External Format Support
- **COLMAP import** (`ImportCOLMAP.h`): Binary reconstruction, selective import
- **ROMA2 import** (`ImportROMA2.h`): Robust optical matching .npz files
- **MVS export** (`InterfaceMVS.h`): Conversion to MVS binary format
- **PLY export**: Tracks as 3D points with optional colors

## Memory Management
- Camera models shared via `CameraPtr` (reference counted)
- Keypoints/descriptors MOVED between global and sub-scenes during split/merge
- Lazy pixel loading: `LoadPixels()` / `ReleasePixels()`
- `Scene::Release()` for manual cleanup

## Build & Dependencies
- **Required**: Common, Math, IO, Ceres Solver, PoseLib, TinyEXIF, TinyNPY
- **Optional**: SiftGPU (CUDA/OpenGL)
- **Inherited**: Eigen3, OpenCV, Boost
- **Precompiled header**: `Common.h`
