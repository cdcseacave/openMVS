# MVS Library

Core multi-view stereo reconstruction engine. Implements the complete pipeline from sparse point clouds to textured meshes: dense depth estimation, surface reconstruction, mesh refinement, and texture mapping. This is the largest and most important library in OpenMVS.

## Central Data Structure: Scene (`Scene.h`)

```cpp
class Scene {
    PlatformArr platforms;        // Camera rigs with trajectories
    ImageArr images;              // All images/views
    PointCloud pointcloud;        // Sparse or dense 3D points
    Mesh mesh;                    // Reconstructed surface
    OBB3f obb;                    // Region of interest
    Matrix4x4 transform;         // Coordinate system transform
    unsigned nCalibratedImages;   // Count of valid images
    unsigned nMaxThreads;         // Thread limit
};
```

All data flows through Scene. Applications load a scene, process it, and save it back.

## Key Classes

### Image (`Image.h`)
```cpp
class Image {
    uint32_t platformID, cameraID, poseID;  // Platform attachment
    String name, maskName;                   // File paths
    Camera camera;                           // Pose + intrinsics
    uint32_t width, height;
    Image8U3 image;                          // Pixels (lazy-loaded)
    ViewScoreArr neighbors;                  // Scored neighbor views
    float scale, avgDepth;
};
```

### Camera (`Camera.h`)
Two-tier system:
- **CameraIntern**: K (3x3 intrinsic), R (3x3 rotation world-to-camera), C (3x1 camera center in world)
- **Camera** extends CameraIntern: Adds cached P (3x4 projection matrix)
- Convention: `P = K[R|t]` where `t = -RC`. R maps world->camera. Pixel center at (0,0).

### Platform (`Platform.h`)
Camera rig with multiple mounted cameras and a trajectory of poses. Each image references a platform + camera + pose index.

### PointCloud (`PointCloud.h`)
```cpp
class PointCloud {
    PointArr points;            // 3D positions
    PointViewArr pointViews;    // Which images see each point
    PointWeightArr pointWeights;// Per-view weights
    NormalArr normals;          // Surface normals (optional)
    ColorArr colors;            // RGB colors (optional)
};
```
Includes octree spatial acceleration. Methods: `GetAABB()`, `EstimateNormals()`.

### Mesh (`Mesh.h`)
```cpp
class Mesh {
    VertexArr vertices;              // 3D positions
    FaceArr faces;                   // Triangle indices
    NormalArr vertexNormals, faceNormals;
    ColorArr vertexColors;           // Per-vertex color
    VertexVerticesArr vertexVertices; // Adjacency
    VertexFacesArr vertexFaces;      // Incident faces
    FaceFacesArr faceFaces;          // Face adjacency
    TexCoordArr faceTexcoords;       // UV coordinates
    Image8U3Arr texturesDiffuse;     // Texture atlases
};
```
Key method (`MeshHalfMesh.cpp`, the bridge to the halfmesh library):
- `Clean(CleanParams)`: spurious-component removal, spike removal, QEM decimation, hole closing, Taubin smoothing, isotropic remeshing, then a finalize pass (degenerate faces, unreferenced vertices, non-manifold repair) — every enabled stage running on ONE halfmesh instance, so the mesh is converted in and out exactly once per call. The same stages are also exposed individually (`RemoveSpuriousComponents`, `RemoveSpikes`, `Simplify`, `CloseHoles`, `Smooth`), each paying its own conversion, so prefer `Clean` when running more than one. OpenMVS adjacency caches are rebuilt only when they existed before the operation. Authored per-vertex normals are carried across by halfmesh, which keeps them through the operations that only renumber vertices and drops them in the ones that move a vertex; only then are they recomputed, and only for a caller that had them.

### DepthData (`DepthMap.h`)
Per-image depth estimation data:
```cpp
struct DepthData {
    struct ViewData { Camera camera; Image32F image; DepthMap depthMap; };
    ViewDataArr images;       // Reference + neighbor views
    DepthMap depthMap;         // Estimated depth
    NormalMap normalMap;       // Surface normals
    ConfidenceMap confMap;     // Confidence scores
    float dMin, dMax;         // Depth range
};
```

## Pipeline Stages

### 1. Scene Loading
```cpp
Scene::Load()                     // Load .mvs, .ply, or interface formats
Scene::SelectNeighborViews()      // Geometric scoring of view pairs
```

### 2. Dense Depth Estimation (`SceneDensify.cpp`, 98KB)
```cpp
Scene::DenseReconstruction(nFusionMode, ...)
  -> SelectViews() -> InitViews() -> EstimateDepthMap() -> FuseDepthMaps()
```
- **PatchMatch stereo**: Random init + iterative propagation + sub-pixel refinement
- **Semi-Global Matching** (`SemiGlobalMatcher.h`): Optional SGM refinement pass
- **Confidence filtering**: Multi-view consistency checks
- **Confidence recalibration** (`ConfidenceRefine.h`, `ConfidenceCUDA.cu`): replaces the photometric `1-NCC` confidence with a posterior predicting fusion survival (intra-map plane prior + soft multi-view confirmation + free-space violations). Controlled by the `ADJUST_CONFIDENCE` bit of `nOptimize`, whose default `ADJUST_CONFIDENCE_AUTO` resolves in `ComputeDepthMaps` to ON only for CUDA estimation (fused into the last geometric iteration, ~3ms/map) and OFF for CPU. The posterior's shape constants are deliberately compile-time in `ConfidenceRefine.h` -- one jointly GT-calibrated operating point, not independent knobs. Design, GT evidence, rejected alternatives and open threads: `docs/design/DepthMapConfidence.md`.
- **DMapCache** (`DMapCache.h`): LRU disk cache for large-scale processing; also holds the color pixels of the cached depth-maps during fusion
- **ImageCache** (`ImageCache.h`): LRU cache decoding images on demand during estimation, storing the gray intensities the estimator consumes
- **View-locality order** (`SortImagesByViewLocality`): the depth-maps are estimated walking the view graph, so consecutive ones share views and the cache decodes each image once
- **PatchMatchCUDA** (`PatchMatchCUDA.h`): GPU-accelerated depth estimation

### 3. Mesh Reconstruction (`SceneReconstruct.cpp`, 43KB)
```cpp
Scene::ReconstructMesh(distInsert, bUseFreeSpaceSupport, ...)
```
Uses CGAL Poisson reconstruction or Delaunay-based method. Integrates free-space support for occlusion handling.

### 4. Mesh Refinement (`SceneRefine.cpp`, 49KB; `SceneRefineCUDA.cpp`, 89KB)
```cpp
Scene::RefineMesh(nResolutionLevel, ...)     // CPU
Scene::RefineMeshCUDA(...)                    // GPU
```
Multi-resolution loop: subdivide -> project to images -> deform vertices using image gradients -> regularize -> close holes -> decimate.

Key params: `nResolutionLevel`, `fDecimateMesh`, `nCloseHoles`, `fRegularityWeight`, `fGradientStep`.

### 5. Texture Mapping (`SceneTexture.cpp`, 82KB)
```cpp
Scene::TextureMesh(nResolutionLevel, ...)
```
Project faces to images -> compute blending weights -> spatial patch grouping -> atlas packing (`halfmesh::PackRectangles`, bounded multi-page skyline packing over `cv::Rect` with rotation) -> global seam leveling -> local seam blending.

`Scene::ComputeVertexColors(...)` shares the same view selection, but instead of building an atlas it samples each vertex in the views texturing the faces around it and stores the weighted average in `mesh.vertexColors`; it releases each image as soon as its faces are consumed.

### 6. Reconstruction Quality Assessment (`SceneQuality.cpp`)
```cpp
Scene::ComputeReconstructionQuality(nMaxResolution)
```
Renders the textured mesh from each camera viewpoint and compares against the original photograph. Returns a `ReconstructionQuality` struct containing:
- `Score`: `completeness` (fraction of image covered by mesh [0,1]), `ssim` (SSIM in covered region [0,1]), `psnr` (PSNR in dB), `score()` (composite 0–100: `100 * completeness * ssim`)
- `ImageScore`: Per-image score with `idxImage`
- `ReconstructionQuality`: Aggregate score + array of `ImageScore`

## File Format Support
- **Native**: `.mvs` (Boost serialization)
- **Point clouds**: `.ply` (binary/ASCII), `.gltf`
- **Meshes**: `.ply`, `.obj` (with MTL), `.gltf`
- **Interface**: COLMAP, OpenMVG via `Interface.h`
- **Depth-maps**: `.dmap`, read and written by `Interface.h` alone
  (`Export/ImportDepthDataRaw`); `DepthMap.cpp` and `SFM/InterfaceMVS.cpp` only adapt
  the scene types to it.

`Interface.h` is a drop-in header: any project can read and write our scenes and
depth-maps with it alone. Keep it that way — no `Common/` include, no OpenMVS type in it.
It works two ways, and the rule for both is *use OpenCV whenever it is there*:
- `_USE_OPENCV` (what we build with): includes `<opencv2/core.hpp>` and uses the real
  `cv::Mat`/`Matx`/`Point3_`, `convertTo` for the float16/uint8 packing, `minMaxIdx` for
  the scales. Maps are passed as `cv::Mat&` and created in place, so nothing is copied —
  our `TImage`s *are* `cv::Mat_`s and bind straight through.
- `_USE_CUSTOM_CV` (set automatically when the former is not): minimal in-header
  `cv::Matx`/`Point3_`/`Mat` and plain-C IEEE-754 conversions in `namespace DEPTHDATA`.

Both paths must write the same bytes. That is not assumed, it is tested: the scratchpad
carries an exhaustive check of the conversions against F16C over all 2^32 float patterns,
plus a round-trip that compiles the header both ways over real `.dmap` files and compares
the output byte for byte. Re-run those after touching the codec.

## GPU/CUDA Components
- `PatchMatchCUDA.h/cpp/inl` - GPU-parallel depth estimation
- `SceneRefineCUDA.cpp` - GPU mesh refinement with CUDA kernels
- `CUDA/Camera.h`, `CUDA/Maths.h` - GPU utility types
- GPU selection via `desiredDeviceID`, compute capabilities 5.0+

## Performance Optimizations
- **Parallelization**: OpenMP + `BS::light_thread_pool`
- **Memory**: Reference counting, DMapCache disk caching, configurable resolution levels
- **Spatial**: Octree acceleration for mesh/point queries
- **Multi-resolution**: Coarse-to-fine pyramid processing

## Build & Dependencies
- **Required**: Common, Math, IO, CGAL, OpenCV, Eigen3, Boost
- **Optional**: Ceres Solver, CUDA Toolkit, Python (bindings)
- **Precompiled header**: `Common.h`
