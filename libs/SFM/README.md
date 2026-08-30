# SFM Library

The SFM (Structure from Motion) library provides complete photogrammetric reconstruction from images or video. It takes a collection of images, finds feature correspondences, estimates camera poses, and produces a sparse 3D point cloud -- the input that the MVS library needs for dense reconstruction.

## What You Need to Know First

### SFM vs MVS: two halves of a pipeline

The SFM library operates **before** the MVS library in the photogrammetry pipeline. SFM answers: "Where was each camera, and where are the sparse 3D points?" MVS then answers: "What does the dense surface look like?"

```
Images → [SFM: poses + sparse points] → [MVS: dense mesh + texture]
```

### SFM::Scene vs MVS::Scene

These are **different classes in different namespaces**. `SFM::Scene` stores feature descriptors, image pairs, and tracks. `MVS::Scene` stores depth maps, meshes, and textures. At the handoff point, `SFM::Scene` is converted to `MVS::Scene` via the export functions in `InterfaceMVS.h`.

### Four reconstruction strategies

The library supports four approaches, each suited to different scenarios:

1. **Incremental** (`Scene::Reconstruct`): Registers images one at a time. Most robust, handles difficult cases, but O(N) bundle adjustments.
2. **Hierarchical** (`Scene::ReconstructHierarchical`): Splits into clusters, reconstructs each independently, then merges. Best for large datasets (1000+ images).
3. **Global** (`Scene::ReconstructGlobal`): Solves all rotations and translations simultaneously. Fastest when it works, but less robust to outliers.
4. **Known-poses finetune** (`Scene::ReconstructKnownPoses`): Starts from camera poses that are already known (AR capture, drone flight log, another SfM), triangulates with them and refines. Not a way to reconstruct an unknown scene -- it *requires* the poses.

## Architecture

### Camera System (`Camera.h`)

The camera model is **polymorphic** -- an abstract `Camera` base class with two implementations:

**PinholeCamera** (most common):
- Intrinsics: focal lengths (fx, fy), principal point (cx, cy)
- Brown-Conrady distortion: radial (k1-k6) and tangential (p1, p2)
- `useAdditionalDistortion` flag enables k4-k6 (off by default)
- `trustIntrinsics` flag indicates calibration reliability (affects matching strategy)

**SphericalCamera** (360 imagery):
- Equirectangular projection
- No distortion parameters

Cameras can be **shared** between images (same physical camera). During bundle adjustment, shared cameras are optimized once and the result applies to all images using that camera.

### Pose Convention (`Pose.h`)

```cpp
class Pose3D {
    RMatrix R;   // 3x3 rotation: world → camera coordinates
    CMatrix C;   // 3D camera center in world coordinates
};
```

This follows the OpenMVS convention: `P = KR[I|-C]`. The camera "looks down" the Z axis. Operators allow composition (`A * B`) and relative pose computation (`A / B`).

### Image and Features (`Image.h`)

Each Image stores:
- **Keypoints**: Detected feature locations (`cv::KeyPoint` array)
- **Descriptors**: Feature vectors (`cv::Mat`, either `CV_8U` binary or `CV_32F` float)
- **Metadata**: EXIF data (focal length, GPS, timestamp, sensor size)
- **View**: Camera model reference + pose

Features are extracted with a **3x3 spatial grid** to ensure even distribution across the image. Each cell targets up to 3000 features, giving ~27k features per image.

### Tracks (`Track.h`)

A track is a 3D point observed in multiple images:

```cpp
class Track {
    Point3 position;              // 3D world coordinates
    ObservationArr observations;  // [(imageID, featureID), ...]
    uint32_t numInliers;          // First N observations are inliers
};
```

Tracks are built using **union-find** (disjoint sets): if feature A in image 1 matches feature B in image 2, and feature B matches feature C in image 3, then A, B, C all belong to the same track.

### Image Pairs (`ImagePair.h`)

Stores the geometric relationship between two images:

```cpp
class ImagePair {
    MatchArr matches;              // Inlier feature correspondences
    std::optional<Matrix3> F, E, H; // Estimated geometry matrices
    Pose3D relativePose;           // Relative camera pose
    float weightSpatial;           // How well features cover the image
    float weightConnectivity;      // Importance in the view graph
    float weightTriplet;           // 3-view consistency score
};
```

The **composite weight** (`spatial × connectivity × triplet`) ranks pairs by reliability. Triplet weight is the strongest quality signal -- it measures consistency across three-view loops.

## The Reconstruction Pipeline

All reconstruction workflows share a common front-end that extracts features, matches images, and builds tracks. They diverge after that: the **hierarchical** workflow clusters the scene and uses incremental reconstruction per cluster, the **global** workflow solves all poses simultaneously, and the **known-poses** workflow skips pose estimation entirely and refines the poses it was given.

```
Input: Images (or video keyframes)          [+ optional poses file]
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│                    COMMON FRONT-END                      │
│                                                          │
│  1. Feature Extraction (AKAZE/ORB/SIFT)                 │
│  2. Feature Matching (Vocabulary/Exhaustive/             │
│                       Sequential/Known-Poses)            │
│  3. Geometric Verification (RANSAC: E/F/H matrices)      │
│  4. View Graph Calibration (focal length estimation)     │
│  5. Track Building (union-find on matches)               │
│  6. Track & Image Filtering (outliers, weak views)       │
│                                                          │
└──────────────────────┬──────────────────────────────────┘
                       │
      ┌────────────────┼────────────────────┐
      ▼                ▼                    ▼
┌───────────────┐ ┌──────────────┐ ┌──────────────────────┐
│ HIERARCHICAL  │ │   GLOBAL     │ │     KNOWN POSES      │
│(robust,slower)│ │(fast,simpler)│ │      (finetune)      │
│               │ │              │ │                      │
│Scene          │ │Rotation      │ │Validate pose         │
│  Clustering   │ │  Averaging   │ │  coverage (>=20%)    │
│      │        │ │      │       │ │      │               │
│      ▼        │ │      ▼       │ │      ▼               │
│Per-cluster:   │ │Global        │ │Resolve camera-axes   │
│ Star Init     │ │  Positioning │ │  convention          │
│ Resection+BA  │ │(translations │ │      │               │
│      │        │ │ + points,    │ │      ▼               │
│      ▼        │ │ rotations    │ │Triangulate with the  │
│Global         │ │ held fixed)  │ │  imported poses      │
│  Alignment    │ │      │       │ │      │               │
│(5-stage merge)│ │      ▼       │ │      ▼               │
│      │        │ │Optional      │ │Finetune BA ->        │
│      ▼        │ │  final BA    │ │ re-triangulate -> BA │
│  Final BA     │ │              │ │                      │
└───────┬───────┘ └──────┬───────┘ └──────────┬───────────┘
        │                │                    │
        └────────────────┼────────────────────┘
                         ▼
         Shared tail: pre-final BA, filtering, final BA,
         weak-image filtering, resection of unposed images,
         GPS alignment *or* re-alignment to the prior poses
                         │
                         ▼
              Export to MVS::Scene
              (dense reconstruction)
```

---

### Common Front-End

These steps are shared by all reconstruction workflows.

#### 1. Feature Extraction (`FeaturesExtractor.h`)

Supported detectors:
- **AKAZE** (default): Fast binary descriptors, good for most cases
- **ORB**: Lighter weight, binary descriptors
- **SIFT**: Highest quality, float descriptors (slower)
- **SiftGPU**: CUDA-accelerated SIFT (optional)

The 3x3 grid extraction ensures features aren't concentrated in textured areas while ignoring featureless regions. Each cell targets up to 3000 features, giving ~27k features per image.

#### 2. Feature Matching (`PairsMatcher.h`, `MatchGeometric.h`)

Four matching strategies:
- **VOCABULARY** (recommended): Build a visual vocabulary tree and query each image's ranked similar-image list. The two directed rankings are fused with symmetric reciprocal-rank fusion (each direction contributes `1/(k0+rank)`, so a pair both images retrieve early outranks a pair only one image scores high, and the rank-based fusion is immune to per-query score-scale drift), then only the pairs present in the fused top-K lists of *both* endpoints are kept — mutual agreement suppresses the one-sided, mostly false tail of each retrieval list that wastes matching budget at small `maxPairsPerImage`. Finally the connected components of the selected pair graph are bridged with the best-scoring cross-component pairs, so a sparse selection cannot silently split the view graph. O(N log N) instead of O(N²).
- **EXHAUSTIVE**: Match all pairs. Only practical for small datasets (<100 images).
- **SEQUENTIAL**: Match consecutive frames only. For ordered video sequences.
- **KNOWN_POSES**: Pick the pairs geometrically, from poses that were imported rather than estimated. Each pair is scored by baseline (normalized by the median nearest-neighbor camera distance, so the score is independent of the units the poses came in) times viewing-direction agreement; pairs whose optical axes diverge by more than 75° are rejected outright. Selection mirrors VOCABULARY: a pair is kept only if each image ranks the other within its own top candidates (mutual agreement), every posed image additionally keeps its 2 nearest posed cameras with *no* angle gating (under occlusion — e.g. an indoor camera turning back at the end of a corridor — all top covisible partners can exceed the gate), and remaining components are bridged by the best-scoring cross pairs. Pair selection itself needs no descriptors and therefore builds no vocabulary tree when every image is posed. If the poses file is incomplete, vocabulary retrieval adds pairs touching the unposed images so the reconstruction tail can resect them. Auto-selected when poses were imported and `--match-mode` was not passed explicitly; falls back to exhaustive if fewer than two images are posed.

  Note that the score deliberately has **no** camera-center cheirality (mutual-frustum) test: in an orbit capture the neighboring camera centers lie tangentially to the view direction, and in a nadir aerial capture perpendicularly to it, so the strongest overlapping pairs are exactly the ones such a test would discard. Baseline is a ranking preference, not a rejection criterion -- the distance at which two views still overlap varies by orders of magnitude between close-range and aerial captures.

**Verification feedback** (VOCABULARY and KNOWN_POSES, on by default when `maxPairsPerImage >= 10`, disable with `--match-verification-feedback 0`): matching runs in two rounds. The first round collects and matches candidates from *uninflated* per-image lists at 80% of the target (single-round matching inflates the lists to compensate the strictness of mutual agreement; here the second round does that job instead); the remaining budget — everything up to `maxPairsPerImage*N/2` total attempted pairs, including the part the strict mutual-agreement rule leaves unspent — is then re-invested in pairs suggested by the geometrically *verified* matches of the first round: KNOWN_POSES closes the triangles of the verified pair graph (two images sharing verified neighbors most likely overlap too, ranked by the number of common neighbors — this recovers true pairs the view-angle gate or the baseline preference mis-ranked), while VOCABULARY propagates each verified pair to the top-5 retrieval candidates of its endpoints (the retrieval analogue of triangle closing). Images left with the weakest verified connectivity refill any leftover budget from their next best-ranked first-round candidates (2 pairs per image).

The matching pipeline:
1. **Descriptor matching**: FLANN (LSH for binary, KDTree for float) or brute-force
2. **Lowe's ratio test**: Keep match only if best/second-best distance ratio < 0.8
3. **Cross-check** (optional): Both images must agree on the match
4. **Geometric verification**: RANSAC to estimate E (calibrated) or F (uncalibrated) matrix
5. **Cheirality check**: Points must be in front of both cameras

**RoMa v2 (optional, `CreateStructure --roma2`)**: an in-process ONNX Runtime deployment of RoMa v2 (a DINOv3 descriptor graph + a coarse-match graph, no refiner) can supplement — never replace — the classical matching above in two independent ways: `--roma2-retrieval` (default on) replaces the vocabulary tree as the per-image ranking source `PairsMatcher::QueryRetrieval` feeds to pair selection, pooling the descriptor graph's output into a global descriptor per image (`GlobalDescriptors.h`, FACETS 2048-D default or LAYERS 1024-D legacy); `--roma2-match` (**default off, experimental**) dense-matches every candidate pair through the coarse-match graph and turns each warp into a guided sparse re-match that replaces the descriptor match only when it has strictly more inliers. End-to-end validation found the dense arm supplies 2.8-5x the median inliers and +57-111% verified pairs, but degrades like-for-like pose accuracy on 3 of 5 captures and drifts the self-calibrated focal, so it is opt-in: enable it with `--roma2-match true`, preferably together with `--roma2-skip-healthy 100 --roma2-max-replace 15` or with imported intrinsics. Needs an exported model (`--roma2-model DIR`, default `$OPENMVS_ROMA2_MODEL_PATH`) and a build with `-DOpenMVS_USE_ONNXRUNTIME=ON`. Flags: `--roma2`, `--roma2-model`, `--roma2-setting turbo|fast|base`, `--roma2-retrieval`, `--roma2-match`, `--roma2-slots N`, `--roma2-skip-healthy N`, `--roma2-max-replace N`, `--roma2-retrieval-recipe facets|layers`, `--roma2-provider auto|cuda|coreml|dml|cpu`, `--export-retrieval-csv` (like `--export-pairs-csv`, written right after matching, before reconstruction). See `docs/design/ROMA2InProcess.md` for the full design.

#### 3. View Graph Calibration (`ViewGraphCalibrator.h`)

If camera intrinsics aren't fully trusted (no EXIF or imprecise calibration), this stage estimates focal lengths globally across all image pairs using the Fetzer method.

#### 4. Track Building & Filtering (`Track.h`)

Union-find merges matched features across all image pairs into tracks. Filtering removes:
- Tracks with too few observations
- Images with spatially clustered tracks (likely degenerate geometry)
- Images with small triangulation angles

---

### Hierarchical Workflow (`Scene::ReconstructHierarchical`)

The hierarchical workflow is the **recommended default**. It splits the scene into manageable clusters, reconstructs each independently using incremental SFM, and then stitches them together with global alignment. When the dataset is small enough to fit in a single cluster, it degrades gracefully to a pure incremental reconstruction -- so it works well for any scene size.

```
Input: Images + matched pairs (from common front-end)
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 1. Scene Clustering                                      │
│    SceneCluster.h/cpp                                    │
│    Aggregative clustering on covisibility graph          │
│    Partition into clusters of ≤200 images                │
│    (if scene ≤ maxViewsPerCluster → 1 cluster = pure    │
│     incremental reconstruction, no alignment needed)     │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 2. Per-Cluster Incremental Reconstruction                │
│    (each cluster runs independently, can be parallelized)│
│                                                          │
│    ┌───────────────────────────────────────────────┐     │
│    │ a. Star Initialization (StarInitializer.h)    │     │
│    │    Select most-connected reference view       │     │
│    │    Register multiple views simultaneously     │     │
│    │    Triangulate initial points                 │     │
│    │    Estimate global scale from median depths   │     │
│    └───────────────────┬───────────────────────────┘     │
│                        ▼                                 │
│    ┌───────────────────────────────────────────────┐     │
│    │ b. Incremental Resection (Resection.h)        │     │
│    │    For each unregistered image:               │     │
│    │      Find 2D-3D correspondences with tracks   │     │
│    │      Solve PnP + RANSAC (PoseLib)             │     │
│    │      Triangulate new points                   │     │
│    │      Local BA on nearby cameras               │     │
│    │      Periodic global BA to correct drift      │     │
│    └───────────────────┬───────────────────────────┘     │
│                        ▼                                 │
│    ┌───────────────────────────────────────────────┐     │
│    │ c. Bundle Adjustment (BundleAdjustment.h)     │     │
│    │    Final global BA with Ceres Solver           │     │
│    │    Refine: poses + points + intrinsics         │     │
│    │    Optional GPS position constraints           │     │
│    └───────────────────────────────────────────────┘     │
│                                                          │
└─────────────────────────────────────────────────────────┘
  │
  │  (skip if only 1 cluster)
  ▼
┌─────────────────────────────────────────────────────────┐
│ 3. Global Alignment -- 5-stage merge                     │
│    GlobalAlignment.h/cpp                                 │
│                                                          │
│    a. Estimate relative poses between sub-scene pairs    │
│       (PoseLib generalized absolute pose from            │
│        cross-cluster 2D-3D correspondences)              │
│                        ▼                                 │
│    b. Rotation averaging (GlobalRotationAveraging.h)     │
│       MST init → L1-ADMM → IRLS on SO(3)                │
│                        ▼                                 │
│    c. Scale averaging (GlobalScaleAveraging.h)           │
│       Log-space least-squares on pairwise scale ratios   │
│                        ▼                                 │
│    d. Translation averaging (GlobalTranslationAveraging) │
│       Linear system solve with gauge constraint          │
│                        ▼                                 │
│    e. Merge sub-scenes into reference scene              │
│       Apply similarity transforms to each cluster        │
│       Average shared camera intrinsics                   │
│       Merge tracks via union-find + 3D proximity guards  │
│                                                          │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 4. Final Bundle Adjustment (optional)                    │
│    Global BA on the merged scene                         │
│    Optional GPS alignment (SimilarityTransform.h)        │
└─────────────────────────────────────────────────────────┘
  │
  ▼
Output: Calibrated poses + sparse point cloud
```

**Key implementation details**:

- **Scene Clustering** (`SceneCluster.h`): Builds a covisibility graph (nodes = images, edges = inlier match counts), then uses bottom-up aggregative clustering. It merges the highest-weight edge at each step until all clusters have ≤ `maxViewsPerCluster` images (default 200). A refinement pass merges small clusters, moves boundary images for better modularity, and splits disconnected components.

- **Data protocol**: Keypoints and descriptors are **moved** (not copied) from the global scene into sub-scenes to save memory. Cross-cluster image pairs remain in the global scene for use during alignment. After merge, data is moved back.

- **Star Initialization** (`StarInitializer.h`): Instead of the classic two-view initialization (sensitive to baseline selection), OpenMVS uses a star configuration: the most-connected image becomes the reference, and multiple views are registered simultaneously. This averages over multiple baselines for a more stable initial estimate.

- **Bundle Adjustment** (`BundleAdjustment.h`): Uses Ceres Solver. **Local BA** optimizes a window of cameras + their points with fixed intrinsics (fast, used during resection). **Global BA** optimizes everything including intrinsics (slower, used at end). GPS constraints can be added when EXIF GPS data is available.

---

### Global Workflow (`Scene::ReconstructGlobal`)

The global workflow bypasses incremental reconstruction entirely. Instead of registering images one by one, it solves for all camera rotations and translations simultaneously using averaging algorithms. This is fundamentally different from the incremental approach.

```
Input: Images + matched pairs (from common front-end)
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 1. Compute Relative Poses                                │
│    Extract relative rotations and translation directions │
│    from E/F matrices in all verified image pairs         │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 2. Global Rotation Averaging                             │
│    GlobalRotationAveraging.h/cpp                         │
│                                                          │
│    Input: pairwise relative rotations R_ij               │
│    Solve: find global R_i for each image such that       │
│           R_ij ≈ R_j × R_i^T for all pairs              │
│                                                          │
│    Algorithm:                                            │
│      a. MST initialization (propagate from root)         │
│      b. L1 minimization (tangent space, angle-axis)      │
│      c. IRLS refinement (Geman-McClure robust loss)      │
│                                                          │
│    Output: global rotations for all images               │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 3. Global Positioning                                    │
│    GlobalPositioning.h/cpp                               │
│                                                          │
│    Rotations are now FIXED.                              │
│    Solve for translations + 3D points simultaneously     │
│    using point-to-camera reprojection constraints        │
│                                                          │
│    Uses Ceres Solver with optional GPU acceleration      │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 4. Optional Bundle Adjustment                            │
│    Refine all poses + points + intrinsics jointly         │
│    Optional GPS alignment                                │
└─────────────────────────────────────────────────────────┘
  │
  ▼
Output: Calibrated poses + sparse point cloud
```

**Key implementation details**:

- **Rotation averaging** operates on the SO(3) manifold. It parameterizes rotations in tangent space (angle-axis vectors) and solves a linear system. The IRLS refinement uses robust loss functions (Geman-McClure or Half-Norm) to downweight inconsistent pairs that are likely wrong matches.

- **Global positioning** treats rotations as fixed and solves only for translations and 3D point positions. This makes the problem linear (or nearly so), which is what gives the global approach its speed advantage. The downside is that any errors in the rotation averaging stage are baked in and cannot be corrected.

- **No incremental registration**: Images are not added one at a time. All poses are estimated in one shot. This means there's no opportunity for the system to detect and reject problematic images during reconstruction.

---

### Known-Poses Workflow (`Scene::ReconstructKnownPoses`)

Sometimes the poses are not the unknown. An AR capture (ARKit/ARCore, Polycam), a drone flight log, or a previous reconstruction already knows where every camera was; what is missing is an accurate, densification-ready sparse reconstruction in *that* coordinate frame. This workflow treats the given poses as the initialization and spends all its effort on refinement -- a "finetune" rather than a reconstruction.

It is selected by `ReconstructionConfig::HasKnownPoses()`, which is true when a poses file is configured with a mode that brings in extrinsics (`PoseImportMode::POSES_INTRINSICS` or `POSES`).

```
Input: Images + matched pairs (from common front-end)
       + poses imported during Scene::Import
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 1. Validate Pose Coverage                                │
│    At least 20% of the images must have received a pose  │
│    Otherwise: fail loudly, listing the unmatched names   │
│    (never silently fall back to standard SfM -- that     │
│     would mask a file-name mismatch in the poses file)   │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 2. Resolve the Camera-Axes Convention (frames.json only) │
│    PoseIO.h::ResolveFramesConvention                     │
│    Compare imported vs match-verified relative rotations │
│    Inconclusive -> fail, ask for an explicit convention  │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 3. Snapshot the Imported Poses                           │
│    Scene::priorPoses (transient, keyed by image ID)      │
│    The BA below refines freely; this is what the final   │
│    re-alignment brings the result back to                │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 4. Triangulate with the Imported Poses                   │
│    BuildTracks -> TriangulateTracks -> FilterTracks      │
│    Permissive threshold (4x maxReprojError): the poses   │
│    are approximate and the intrinsics may be EXIF-only,  │
│    so the strict threshold would reject correct tracks   │
│    before BA ever gets to fix the geometry               │
└─────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│ 5. Finetune Bundle Adjustment                            │
│    BA -> re-triangulate outliers -> filter -> BA         │
│    (same convergence pattern the star initializer uses)  │
└─────────────────────────────────────────────────────────┘
  │
  ▼
Output: Refined poses + sparse point cloud, handed to the
        shared tail of Scene::Reconstruct
```

**Key implementation details**:

- **The poses are initialization only**. There are no soft or hard pose priors in the bundle adjustment -- BA refines the poses as freely as it would in any other workflow. The input frame is restored afterwards by `Scene::AlignToPriorPoses`, not by constraining the optimizer.

- **Intrinsics are refined even when you asked for no refinement**, but only if some camera reports `!TrustIntrinsics()`. A poses-only import leaves the focal length coming from EXIF, which is by far the weakest prior in play; known poses make the bundle adjustment over focal length well conditioned, so it is worth solving for.

- **The front-end still verifies image evidence.** Features are extracted and every selected pair is descriptor-matched and geometrically verified; only candidate selection changes to use the imported poses (plus visual retrieval for unposed images). This is deliberate: the E-decomposed relative poses are what convention auto-detection compares the imported poses against, so they must stay independent of them.

- **Clustering never runs.** This path does not go through `ReconstructHierarchical`; `Scene::priorPoses` is transient (not serialized) but is preserved by regular scene copies and moves.

- **The shared tail still runs.** After this method returns, `Scene::Reconstruct` continues through final bundle adjustment, filtering, and `Resection::RegisterImages()`, which registers images that had no entry in the poses file.

#### The frames.json format (`PoseIO.h`)

A JSON array of frames, the format Polycam-style AR captures export:

```json
[
  {
    "name": "frame_00001.jpg",
    "transform": [ /* 16 numbers: column-major 4x4 camera-to-world */ ],
    "params": { "camera_model": "OPENCV",
                "w": 2048, "h": 1534,
                "fx": 1600.0, "fy": 1600.0, "cx": 1024.0, "cy": 767.0,
                "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0 }
  }
]
```

- **Matching to images** is by file name: full name first, then stem, both case-insensitive. Entries with no matching image are reported; images with no entry stay unposed and are picked up by the tail's resection.
- **`params` is optional** and only read in `POSES_INTRINSICS` mode. It may be declared for a different resolution than the images on disk (a downscaled preview, typically); `fx, fy, cx, cy` are rescaled by the width ratio, and the height ratio must agree within 0.1% or the entry is rejected. `OPENCV` is the only accepted `camera_model`; its `k1, k2, p1, p2` map directly onto `PinholeCamera`'s Brown-Conrady subset. Without `params`, the EXIF-derived intrinsics from `Image::LoadMetadata` are kept.
- **The import runs before camera de-duplication** in `Scene::Import`, so a capture whose frames all declare identical intrinsics collapses into a single shared `Camera`.
- **Rotations are sanitized, not trusted**: the 4x4 must have a `(0,0,0,1)` last row, and a rotation block within 1e-3 of orthonormal is re-orthonormalized via SVD. Anything outside that is rejected with the frame name.

**Two things the format does not tell you**, both handled by the importer:

1. **Camera-axes convention.** The `transform` is camera-to-world, but whether its camera axes are ARKit/OpenGL (X right, Y up, Z backward) or OpenCV (X right, Y down, Z forward) is not declared, and the two differ by a π rotation about the camera X axis. Choosing wrong reverses every optical axis and triangulation collapses. `DetectFramesConvention` decides after matching: it compares the imported relative rotations `R_j · R_iᵀ` against the match-verified ones under both hypotheses and takes the lower median angular error, requiring the winner to be 3× better. If no pair carries a verified relative pose (an F-only, uncalibrated scene) or the rotation margin is ambiguous (a low-rotation forward walk barely changes under conjugation), it falls back to two-view triangulating the highest-weighted pairs under both hypotheses and counting cheirality-positive, low-reprojection inliers. A genuinely ambiguous scene returns `AUTO`, and the caller fails asking for an explicit convention rather than guessing. `FlipFramesConvention` applies the flip as `img.R ← diag(1,-1,-1) · img.R` — or `diag(-1,1,-1)` for EXIF-rotated images, whose stored pose composes the in-plane rotation — leaving the camera centers alone.

2. **EXIF portrait rotation.** OpenMVS rotates EXIF-portrait images 90° clockwise on load (`View::ToWorkingOrientation`), but the imported pose describes the camera of the image *as stored on disk*. The importer composes the same in-plane rotation into the pose (`R ← Rz(+90°) · R`, the inverse of what `View::RevertRotation` undoes on export) and rotates the imported intrinsics to match (`fx ↔ fy`, `cx, cy` remapped, `p1, p2` rotated; the radial coefficients are invariant). The camera center is unchanged.

#### Re-aligning to the input frame (`Scene::AlignToPriorPoses`)

Bundle adjustment leaves the gauge free, so the refined reconstruction drifts off the input frame -- and, without GPS priors, its scale is unanchored entirely. `AlignToPriorPoses` estimates a similarity transform from the refined camera centers to their `priorPoses` counterparts (`EstimateSimilarityTransformWithRotations`, which falls back to rotation averaging plus a least-squares scale/translation when the centers are near-collinear -- a straight-line capture leaves the roll unconstrained by centers alone) and applies it with `Scene::Transform`, which also re-maps the track positions and the pose covariances. Images resected along the way have no prior and simply ride along with the transform. Prior-pose alignment takes precedence over GPS: preserving the input frame is the point of this workflow.

The RANSAC threshold is expressed as a *fraction* of the median distance between neighboring prior camera centers (default 0.5) rather than in absolute units, because the prior frame may be in any units at all. If this final similarity cannot be estimated, the failure is reported as a warning and the finished reconstruction is kept in the refined (arbitrary-gauge) frame rather than discarded.

Unlike `AlignToGPS`, this does not touch `Scene::transform` and does not set the `GEO_ALIGN` state: the prior frame is the dataset's own frame, not a geo-referenced one. It runs as the `else` branch of the GPS-alignment block, so a scene with both GPS metadata and known poses still prefers GPS when the user asks for it.

The log line it emits -- median and maximum camera-center delta, median and maximum rotation delta against the priors -- is the headline diagnostic for this workflow: it answers "how much did the finetune actually move the poses".

---

### Choosing Between Workflows

| Aspect | Hierarchical | Global |
|--------|-------------|--------|
| **Speed** | Slower -- runs full incremental SFM per cluster, plus alignment | Faster -- solves rotations and translations in closed form |
| **Robustness** | More robust -- incremental registration can detect and reject bad images; BA continuously corrects drift | Less robust -- relies on pairwise relative poses being correct; a few bad pairs can corrupt the entire solution |
| **Difficult scenes** | Handles well -- star initialization and incremental resection adapt to varying baselines, repeated structures, and challenging geometry | Can fail -- rotation averaging may not converge if many pairs have wrong relative poses (e.g., repetitive textures, symmetric structures) |
| **Completeness** | Higher -- incremental approach tries hard to register every image, running BA after each addition | Lower -- images that don't fit the global solution are simply lost; no mechanism to retry or adjust |
| **Scalability** | Excellent -- clustering + parallel reconstruction handles 1000+ images; memory bounded by cluster size | Good for medium scenes -- the global solve is O(N) but can struggle numerically with very large systems |
| **Small scenes** | Works well -- with 1 cluster, degrades to pure incremental reconstruction | Works well -- fastest option for well-connected small datasets |
| **Drift** | Controlled -- periodic global BA during resection prevents drift accumulation | No drift -- all poses solved simultaneously (but errors are global rather than local) |

**Practical guidance**:

- **Start with hierarchical** (`Scene::ReconstructHierarchical`). It's the safer default. For small scenes (< 200 images) it automatically runs as a single incremental reconstruction with no clustering overhead.
- **Try global** (`Scene::ReconstructGlobal`) when you need speed and your dataset is well-connected with reliable matches (e.g., drone surveys with good overlap, indoor scans with distinctive features). If the global result has missing or misaligned cameras, fall back to hierarchical.
- **Global is not "better hierarchical"**. The two approaches have fundamentally different failure modes. Hierarchical fails gracefully (some images may not register, but the rest are correct). Global can fail catastrophically (a few bad rotations corrupt the entire solution).
- **Known-poses is not a competitor to either** -- it is the answer to a different question. Use it only when you already have the poses and want them refined in their own frame; it cannot reconstruct a scene whose poses are unknown. It avoids the incremental/global pose solver, and bad or badly named poses stop the run instead of silently selecting a different reconstruction strategy.

## External Format Integration

| Module | Format | Direction |
|--------|--------|-----------|
| `ImportCOLMAP.h` | COLMAP binary | Import cameras, poses, tracks |
| `PoseIO.h` | OpenMVS pose CSV and Polycam-style frames.json | Import/export per-image intrinsics and poses |
| `InterfaceMVS.h` | OpenMVS .mvs | Export to MVS pipeline |
| Scene::ExportPLY | PLY | Export sparse point cloud |

The pose CSV schema is `filename,fx,fy,cx,cy,qx,qy,qz,qw,Cx,Cy,Cz,score` per row. Both pose importers share the `PoseImportMode` selector: `POSES_INTRINSICS` applies the intrinsics (when the row/entry carries them, marking them trusted) *and* the rotation + camera center, `POSES` applies only the rotation + center, `POSITIONS` only the center. `ImportConfig::importPosesFile` dispatches on the file extension -- `.csv` to `ImportPosesCSV`, `.json` to `ImportFramesJSON`.

RoMa v2 correspondences are not an external format: the descriptor and coarse-match ONNX graphs run in-process through `OnnxRuntime.h` (the ONNX Runtime session/tensor wrapper, CUDA/CoreML/DirectML/CPU) and `RoMa2Matcher.h` (the two RoMa v2 sessions); the dense warps are turned into pooled global retrieval descriptors (`GlobalDescriptors.h`) and guided sparse re-matches configured and orchestrated by `MatchROMA2.h`, and consumed through `ROMA2Warp.h` (warp coordinate conventions, confidence erosion, keypoint tracking, guided-pair store/replace policy). See `docs/design/ROMA2InProcess.md` for the full design.

### Scene-file compatibility (`.sfm`)

The SFM project stream carries a layout version (`SFM_PROJECT_VERSION`, `Scene.cpp`) and
`Scene::Load` accepts **that exact version only**. Storing the per-image RoMa v2 global descriptor
bumped it **0 → 1**, so a `.sfm` written by an earlier build is refused with
`error: unsupported SFM project version 0 (this build reads only version 1) in '<file>'`. There is no
converter: regenerate the scene by re-running the matching stage from the images
(`CreateStructure -s <images> -o scene.sfm ...`). `.mvs` files are unaffected.

## Usage Examples

### Keyframe Extraction from Video

```cpp
KeyframeConfig config;
config.detectorType = "AKAZE";
config.overlapThreshold = 0.8f;     // 80% overlap between keyframes

Scene scene;
KeyframeExtractor::ExtractFromVideo("video.mp4", config, scene);
```

### Full Reconstruction

```cpp
Scene scene;
// ... load images, extract features ...

// Match image pairs (builds the vocabulary tree on demand in VOCABULARY mode)
scene.MatchPairs(matchConfig);

// Build tracks and reconstruct
BuildTracks(scene);
StarInitializer().Initialize(scene);
// ... incremental resection + BA ...

// Export to MVS format
SFM::ExportMVS("scene.mvs", scene);
```

### Finetune from Known Poses

```cpp
ReconstructionConfig config;
config.importCfg.importPosesFile = "frames.json";              // or a pose .csv
config.importCfg.importPosesMode = PoseImportMode::POSES;      // POSES_INTRINSICS to also take params
config.importCfg.framesConvention = FramesConvention::AUTO;    // resolved after matching
config.matchCfg.mode = MatchConfig::KNOWN_POSES;               // pose-guided pair selection
config.thAlignGPS = 0.f;                                       // keep the imported frame, not ENU

// HasKnownPoses() is now true, so Reconstruct() dispatches to ReconstructKnownPoses()
// and finishes by re-aligning the result to the imported poses
Scene scene;
scene.Reconstruct("images_folder", config);
```

## Performance Considerations

| Strategy | Complexity | Best for |
|----------|-----------|----------|
| Vocabulary matching | O(N log N) | Default for most datasets |
| Exhaustive matching | O(N²) | Small datasets (<100 images) |
| Sequential matching | O(N) | Ordered video frames |
| Pose-guided matching | O(N²) scoring, O(N·K) pairs matched | Datasets with known camera poses |
| Scene clustering | Enables parallel reconstruction | 200+ images |
| Local BA | O(window size) | During incremental registration |
| Global BA | O(all cameras + points) | Final refinement |

**Memory**: Lazy image loading (`LoadPixels()` / `ReleasePixels()`) and feature data movement (not copy) during clustering keep memory usage bounded.

## File Organization

```
libs/SFM/
├── Common.h/cpp                        # Library init
│
│ # Core data structures
├── Camera.h/cpp                        # Pinhole + Spherical camera models
├── Pose.h/cpp                          # 3D pose (R, C)
├── View.h/cpp                          # Pose + Camera reference
├── Image.h/cpp                         # Features, descriptors, metadata
├── ImagePair.h/cpp                     # Pairwise matches and geometry
├── Track.h/cpp                         # 3D tracks + union-find builder
├── Scene.h/cpp                         # Central container
│
│ # Feature pipeline
├── FeaturesExtractor.h/cpp             # AKAZE/ORB/SIFT extraction
├── VocabularyTree.h/cpp                # Visual vocabulary for retrieval
├── GlobalDescriptors.h/cpp             # RoMa v2 global-descriptor pooling + cosine retrieval index
├── PairsMatcher.h/cpp                  # Matching strategies
├── MatchGeometric.h/cpp                # RANSAC geometric verification
├── OnnxRuntime.h/cpp                   # ONNX Runtime session/tensor wrapper (CUDA/CoreML/DML/CPU)
├── RoMa2Matcher.h/cpp                  # RoMa v2 ONNX graphs: describe + coarse-match sessions
├── MatchROMA2.h/cpp                    # In-process RoMa v2 describe pass, dense-matching pass, config
├── ROMA2Warp.h/cpp                     # RoMa v2 warp coordinates, erosion, keypoint tracking
├── PairsWeighting.h/cpp                # Composite pair quality scores
│
│ # Reconstruction
├── StarInitializer.h/cpp               # Star-config initialization
├── Resection.h/cpp                     # Incremental PnP registration
├── Triangulation.h/cpp                 # Multi-view triangulation
├── BundleAdjustment.h/cpp              # Ceres-based optimization
├── BundleAdjustmentCostFunctions.h     # Reprojection error residuals
├── ViewGraphCalibrator.h/cpp           # Global focal estimation
├── RelativePoseRefine.h/cpp            # Two-view calibration refinement
│
│ # Hierarchical / Global reconstruction
├── SceneCluster.h/cpp                  # Aggregative scene clustering
├── GlobalAlignment.h/cpp               # 5-stage sub-scene merging
├── GlobalRotationAveraging.h/cpp       # SO(3) rotation averaging
├── GlobalScaleAveraging.h/cpp          # Log-space scale averaging
├── GlobalTranslationAveraging.h/cpp    # Linear translation solving
├── GlobalPositioning.h/cpp             # Translation refinement
├── SimilarityTransform.h/cpp           # 7-DOF transform + GPS / prior-pose alignment
│
│ # Video support
├── KeyframeExtractor.h/cpp             # Keyframe selection from video
│
│ # External format support
├── ImportCOLMAP.h/cpp                  # COLMAP import
├── PoseIO.h/cpp                        # CSV/frames.json pose I/O + convention detection
└── InterfaceMVS.h/cpp                  # MVS format export
```

## Conventions

- **Namespace**: `SFM` (separate from `MVS`)
- **Coordinate system**: Right-handed, X right, Y down, Z forward
- **Pose**: `P = KR[I|-C]`, R is world-to-camera, C is camera center in world
- **Pixel origin**: Integer coordinates at pixel center, (-0.5, -0.5) at top-left corner
- **Thread pool**: `Scene::threadPool` (`BS::light_thread_pool`) for parallel algorithms

## Dependencies

- **Common, Math, IO** (required): OpenMVS internal libraries
- **Ceres Solver** (required): Bundle adjustment
- **PoseLib** (required): Pose estimation (E/F/H matrices, PnP)
- **TinyEXIF** (required): EXIF metadata parsing
- **OpenCV** (inherited): Feature detection, matching, image I/O
- **Eigen3** (inherited): Linear algebra
- **Boost** (inherited): Serialization
- **SiftGPU** (optional): GPU-accelerated SIFT
