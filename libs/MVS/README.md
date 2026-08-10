# MVS Library

The MVS (Multi-View Stereo) library is the heart of OpenMVS. It implements the complete reconstruction pipeline that transforms sparse camera poses and point clouds (from SFM) into dense, textured 3D meshes. This is the largest library in the project.

## What You Need to Know First

### Scene is the central data hub

Everything revolves around the `Scene` class. It holds all images, cameras, point clouds, and meshes. The typical workflow in any OpenMVS application is:

```cpp
MVS::Scene scene;
scene.Load("input.mvs");     // Load data
scene.DenseReconstruction();  // Process
scene.Save("output.mvs");    // Save results
```

Each pipeline stage reads from and writes to the same Scene object. You won't find data being passed between stages through function arguments -- it all lives in Scene.

### Two different Camera types exist

There's a two-tier camera system that can be confusing at first:

- **`CameraIntern`**: Stores the raw intrinsics (K matrix) and extrinsics (R rotation + C center). This is the base representation.
- **`Camera`** (extends `CameraIntern`): Adds a cached **projection matrix P** (3x4) for fast point projection. Call `ComposeP()` after changing K, R, or C.

**Convention**: `P = K[R|t]` where `t = -RC`. The rotation R transforms from **world to camera** coordinates. C is the camera center in **world** coordinates. Pixel center is at integer coordinates (0,0), with the top-left image corner at (-0.5, -0.5).

### Platform = camera rig

A **Platform** represents a physical camera rig (e.g., a drone with multiple cameras). It has:
- A list of mounted cameras (with fixed relative positions)
- A trajectory of poses (one per capture time)

Each Image stores `platformID`, `cameraID`, and `poseID` to reconstruct its absolute pose. For single-camera setups, there's typically one platform with one camera.

## Data Structures

### Scene (`Scene.h`)
```cpp
class Scene {
    PlatformArr platforms;        // Camera rigs with trajectories
    ImageArr images;              // All images with metadata
    PointCloud pointcloud;        // Sparse or dense 3D points
    Mesh mesh;                    // Reconstructed triangle mesh
    OBB3f obb;                    // Region of interest
    unsigned nCalibratedImages;   // How many images have valid poses
    unsigned nMaxThreads;         // Thread limit for algorithms
};
```

### Image (`Image.h`)
```cpp
class Image {
    uint32_t platformID, cameraID, poseID;  // Which camera took this
    String name;                             // File path
    Camera camera;                           // Full camera model (cached)
    uint32_t width, height;                  // Resolution
    Image8U3 image;                          // Pixel data (loaded on demand!)
    ViewScoreArr neighbors;                  // Best stereo partner views
};
```

**Important**: Image pixels are **lazy-loaded**. They're not in memory until an algorithm actually needs them. This is essential for handling datasets with thousands of images.

### PointCloud (`PointCloud.h`)
```cpp
class PointCloud {
    PointArr points;               // 3D positions (float)
    PointViewArr pointViews;       // Which images see each point
    PointWeightArr pointWeights;   // Per-view confidence weights
    NormalArr normals;             // Surface normals (optional)
    ColorArr colors;               // RGB colors (optional)
};
```

The point cloud starts sparse (from SFM) and becomes dense after depth map fusion. It includes an **octree** for fast spatial queries.

### Mesh (`Mesh.h`)
```cpp
class Mesh {
    VertexArr vertices;              // 3D vertex positions
    FaceArr faces;                   // Triangle indices (3 per face)

    // Topology (computed on demand)
    VertexVerticesArr vertexVertices; // Adjacent vertices per vertex
    VertexFacesArr vertexFaces;      // Incident faces per vertex
    FaceFacesArr faceFaces;          // Adjacent faces per face

    // Texturing
    TexCoordArr faceTexcoords;       // UV coordinates (3 per textured face)
    Image8U3Arr texturesDiffuse;     // Texture atlas images
};
```

The Mesh class supports both manifold and non-manifold topology. Adjacency data is built lazily when needed.

## The Pipeline

The full reconstruction pipeline has five stages, each implemented in a separate large source file:

```
Input: Sparse point cloud + calibrated camera poses (.mvs file)
  │
  ▼
┌─────────────────────────────────────────────┐
│ 1. Neighbor Selection                        │
│    SelectNeighborViews()                     │
│    Scores view pairs by geometric overlap    │
└─────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────┐
│ 2. Dense Depth Estimation                    │
│    SceneDensify.cpp (98 KB)                  │
│    PatchMatch stereo + SGM refinement        │
│    Multi-view consistency filtering          │
│    Depth map fusion into dense point cloud   │
└─────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────┐
│ 3. Mesh Reconstruction                       │
│    SceneReconstruct.cpp (43 KB)              │
│    CGAL Delaunay/Poisson reconstruction      │
│    Free-space support for occlusion handling │
└─────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────┐
│ 4. Mesh Refinement                           │
│    SceneRefine.cpp (49 KB) - CPU             │
│    SceneRefineCUDA.cpp (89 KB) - GPU         │
│    Multi-resolution image-guided deformation │
│    Topology repair (hole closing, decimation)│
└─────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────┐
│ 5. Texture Mapping                           │
│    SceneTexture.cpp (82 KB)                  │
│    Per-face view selection + blending        │
│    Atlas packing + seam leveling             │
└─────────────────────────────────────────────┘
  │
  ▼
Output: Textured mesh (.ply, .obj, .glb)
```

### Stage 2: Dense Depth Estimation (the most complex stage)

This is where the heavy computation happens. For each reference image:

1. **Select neighbor views** with good stereo geometry (baseline, overlap, angle)
2. **Initialize depth estimates** randomly across the image
3. **PatchMatch iteration**: For each pixel, check if neighbors' depth/normal produce a better NCC (Normalized Cross-Correlation) score, then randomly perturb to explore
4. **Optional SGM pass**: Semi-Global Matching refines depth using path-based optimization with penalties for depth discontinuities
5. **Confidence filtering**: Check multi-view consistency -- a depth is only accepted if multiple views agree

All depth maps are then **fused** into a single dense point cloud by projecting each confident pixel into 3D and merging nearby points.

**Confidence recalibration**: the confidence stored with each depth starts out as a photometric score (`1-NCC`), which measures how well a patch matched rather than whether the depth is right -- a repetitive facade matches well and is often wrong. When enabled, the recalibration replaces it with a posterior that predicts whether the depth will survive fusion, combining a local plane-fit prior over the depth-map, continuous (not pass/fail) agreement weights against each neighbour view, and free-space violations where a neighbour's own depth lies behind our point. Against ground truth this lifts the inlier/outlier ROC-AUC from 0.844 to 0.926 and roughly doubles the depths retained at a fixed 1% contamination budget. It is on by default only when CUDA estimates the depth-maps, where it rides along on the already-resident device buffers; see `ConfidenceRefine.h` for the shared CPU/GPU math.

**Memory management**: Large datasets can produce hundreds of depth maps. The `DMapCache` (LRU disk cache) automatically writes depth maps to disk and reloads them when needed, keeping memory usage bounded. The images are bounded the same way: `ImageCache` decodes them on demand during depth estimation and `DMapCache` holds the color pixels of the depth-maps it caches during fusion, so neither stage needs every image resident. The depth-maps are estimated in view-graph order (`SortImagesByViewLocality`) so that consecutive ones are computed from mostly the same views, which is what keeps that cache from having to decode an image more than once.

**GPU acceleration**: `PatchMatchCUDA` provides a CUDA implementation for the depth estimation step, running per-pixel matching in parallel on the GPU.

### Stage 3: Mesh Reconstruction

Takes the dense point cloud and produces a watertight triangle mesh using CGAL:
- **Delaunay triangulation** of the 3D points
- **Graph-cut labeling** to classify tetrahedra as inside/outside
- **Free-space constraints**: Uses visibility information (which camera saw each point) to carve away occluded volumes

### Stage 4: Mesh Refinement

Improves the mesh by deforming it to better match the images. This is a multi-resolution process:

1. Start at a coarse resolution level
2. For each level: subdivide the mesh, project it to images, compute photometric gradients, and deform vertices to reduce image error while maintaining smoothness
3. Close small holes and decimate to control mesh complexity
4. Move to the next finer level

Key parameters you'll see:
- `nResolutionLevel`: How many coarse-to-fine levels
- `fDecimateMesh`: Target decimation ratio
- `nCloseHoles`: Maximum hole size to close (in edges)
- `fRegularityWeight`: How much to penalize non-smooth surfaces
- `fGradientStep`: Step size for vertex deformation

### Stage 5: Texture Mapping

Creates texture atlases by:
1. **Projecting** each mesh face to all images that see it
2. **Selecting** the best view per face (based on angle, distance, resolution)
3. **Packing** face textures into atlas images using `RectsBinPack`
4. **Seam leveling**: Adjusting colors at face boundaries to prevent visible seams (both global and local blending)

## GPU (CUDA) Support

Several stages have GPU-accelerated variants:

| Component | File | What it accelerates |
|-----------|------|-------------------|
| PatchMatch stereo | `PatchMatchCUDA.h/cpp/inl` | Per-pixel depth estimation |
| Mesh refinement | `SceneRefineCUDA.cpp` | Face normal computation, vertex deformation |
| Camera operations | `CUDA/Camera.h` | Projection/unprojection on GPU |
| Math utilities | `CUDA/Maths.h` | Vector/matrix operations |

GPU code targets compute capabilities 5.0, 7.2, and 7.5+. When CUDA is not available, everything falls back to CPU implementations transparently.

## File Formats

| Format | Extension | Usage |
|--------|-----------|-------|
| MVS native | `.mvs` | Boost binary serialization -- stores complete Scene state |
| PLY | `.ply` | Point clouds and meshes (binary or ASCII) |
| OBJ | `.obj` | Textured mesh export (with .mtl and texture images) |
| glTF | `.gltf`/`.glb` | Modern 3D format for interchange |
| Interface | various | COLMAP, OpenMVG import/export via `Interface.h` |
| Depth-map | `.dmap` | quantized depth/normal/confidence/views, 11 bytes per pixel; the codec lives in `Interface.h`, which is self-contained and can be dropped into any project |

## Performance and Threading

- **OpenMP**: Used for simple loop parallelism (depth estimation, normal computation)
- **`BS::light_thread_pool`**: Task-based parallelism for more complex scheduling
- **`nMaxThreads`**: Scene-level thread limit that algorithms respect
- **Octree**: Spatial acceleration for point/mesh queries
- **DMapCache** / **ImageCache**: LRU caches prevent out-of-memory on large datasets
- **Multi-resolution**: Coarse-to-fine processing reduces computation at each level

## File Organization

```
libs/MVS/
├── Common.h/cpp              # Library init, precompiled header
│
│ # Core data structures
├── Scene.h                   # Central container
├── Scene.cpp                 # Scene management, I/O, transforms (103 KB)
├── Image.h/cpp               # Image/view representation
├── Camera.h/cpp              # Camera intrinsics/extrinsics
├── Platform.h/cpp            # Camera rig + trajectory
├── PointCloud.h/cpp          # Point cloud with attributes
├── Mesh.h/cpp                # Triangle mesh with topology
├── DepthMap.h/cpp            # Depth/normal/confidence maps
├── Interface.h               # External format definitions + the .dmap codec
│
│ # Pipeline stages (one file per stage)
├── SceneDensify.cpp          # Dense depth estimation (98 KB)
├── SceneDensify.h            # Depth estimation config
├── SceneReconstruct.cpp      # Mesh from points (43 KB)
├── SceneRefine.cpp           # CPU mesh refinement (49 KB)
├── SceneRefineCUDA.cpp       # GPU mesh refinement (89 KB)
├── SceneTexture.cpp          # Texture mapping (82 KB)
│
│ # Supporting algorithms
├── SemiGlobalMatcher.h/cpp   # SGM stereo algorithm
├── DMapCache.h/cpp           # LRU depth map disk cache
├── ImageCache.h/cpp          # LRU on-demand image decoding cache
├── RectsBinPack.h/cpp        # Texture atlas packing
│
│ # CUDA components
├── PatchMatchCUDA.h/cpp/inl  # GPU depth estimation
├── CUDA/Camera.h             # GPU camera operations
└── CUDA/Maths.h              # GPU math utilities
```

## Dependencies

- **Common, Math, IO** (required): OpenMVS internal libraries
- **CGAL** (required): Computational geometry (Delaunay, Poisson reconstruction)
- **OpenCV** (required): Image processing
- **Eigen3** (required): Linear algebra
- **Boost** (required): Serialization for .mvs format
- **Ceres Solver** (optional): Non-linear optimization
- **CUDA Toolkit** (optional): GPU acceleration
- **Python** (optional): Python bindings (`pyOpenMVS`)
