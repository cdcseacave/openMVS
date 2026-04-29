# OrthoMap Step 1: Grid Initialization & DEM Generation

## Context

We are building Step 1 of a new orthomap (true orthophoto) pipeline for OpenMVS. Given a reconstructed mesh with Z-up orientation, this step computes the scene's XY bounding box, determines the target GSD, decomposes the output into tiles, and rasterizes the mesh into per-tile depth maps under orthographic projection. This depth map (G-buffer) is the foundation for all subsequent steps (view selection, blending, export).

The pipeline lives in new files (`SceneOrthoMap.h/cpp`) and reuses the existing orthographic rasterization infrastructure (`TRasterMesh`, `ProjectOrtho`, `Camera` ortho transforms, octree spatial queries).

---

## Files to Create

| File | Purpose |
|------|---------|
| `libs/MVS/SceneOrthoMap.h` | Data structures: `OrthoConfig`, `OrthoGrid`, `OrthoTile`, `OrthoMapContext` |
| `libs/MVS/SceneOrthoMap.cpp` | All logic: GSD computation, grid/tile setup, spatial index, G-buffer rasterization, `Scene::ComputeOrthoMap()` |
| `apps/OrthoMap/OrthoMap.cpp` | Standalone app entry point |
| `apps/OrthoMap/CMakeLists.txt` | Build rules for the app |

## Files to Modify

| File | Change |
|------|--------|
| `libs/MVS/Scene.h:165` | Add `ComputeOrthoMap()` declaration after `TextureMesh()` |
| `apps/CMakeLists.txt:12` | Add `ADD_SUBDIRECTORY(OrthoMap)` after TextureMesh line |

Note: `libs/MVS/CMakeLists.txt` uses `FILE(GLOB ...)` so new `.cpp`/`.h` files are auto-detected. No change needed.

---

## Implementation Steps

### Step 1: `libs/MVS/SceneOrthoMap.h` — Data Structures

Define in namespace `MVS`:

**`OrthoConfig`** — pipeline parameters with defaults:
```cpp
struct MVS_API OrthoConfig {
    float targetGSD;              // 0 = auto from cameras; >0 = user override (m/px)
    unsigned tileSize;            // tile dimension in pixels (default 4096)
    unsigned tileOverlap;         // overlap for blending seams (default 16)
    float nadirThreshold;         // abs(dir.z) filter for GSD cameras (default 0.3)
    float marginFactor;           // AABB expansion: max(0.1f, extent * factor) (default 0.01)
    OrthoConfig() : targetGSD(0), tileSize(4096), tileOverlap(16),
       nadirThreshold(0.3f), marginFactor(0.01f) {}
};
```

**`OrthoGrid`** — computed grid layout:
```cpp
struct MVS_API OrthoGrid {
    AABB3f sceneAABB;    // expanded mesh AABB
    float gsd;           // final GSD (m/px)
    cv::Size sizePx;     // total orthomap dimensions in pixels
    Point2u tiles;       // tile counts (define Point2u similar to Point2i if does not exist for unsigned)
    OrthoGrid() : gsd(0), tiles(0,0) {}
};
```

**`OrthoTile`** — per-tile data:
```cpp
struct MVS_API OrthoTile {
    Point2u tile;   // tile grid indices
    Point2u p0;       // start pixel in global grid
    cv::Size size;           // tile pixel dimensions (may be smaller at edges)
    AABB3f worldBounds;      // world XY region this tile covers
    Camera camera;           // orthographic camera for this tile
    DepthMap depthMap;       // G-buffer: depth (camera-space Z)
};
```
Use `cList<OrthoTile, const OrthoTile&, 2, 16, uint32_t>` as `OrthoTileArr` — needs `useConstruct=2` because `OrthoTile` contains non-trivial members (`Camera`, `DepthMap`).

**`OrthoMapContext`** — orchestrator (takes `Scene&`, not a Scene member):
```cpp
class MVS_API OrthoMapContext {
public:
    Scene& scene;
    OrthoConfig config;
    OrthoGrid grid;
    OrthoTileArr tiles;
    Mesh::Octree octree;

    OrthoMapContext(Scene&, const OrthoConfig& = OrthoConfig());
    float ComputeGSD() const;
    bool ComputeGrid();
    Camera SetupTileCamera(const OrthoTile&) const;
    void BuildSpatialIndex();
    void RasterizeTile(OrthoTile&, const Mesh::FaceIdxArr&) const;
    bool RasterizeTiles();
};
```

Create a image cache system in `ImageCache.h/cpp` based on `libs/Common/ListFIFO.h` needed for `RasterizeTile()` to avoid redundant image reloading (see similar cache implementations in depth-map estimation code).

### Step 2: `libs/MVS/SceneOrthoMap.cpp` — Core Logic

Include pattern follows `SceneTexture.cpp`:
```cpp
#include "Common.h"
#include "Scene.h"
```

**`ComputeGSD()`**:
1. If `config.targetGSD > 0`, return it immediately.
2. If any `image.avgDepth == 0`, call `scene.ComputeDistanceCameras2Scene()`.
3. For each valid image: get `camera.Direction()` (returns `R.row(2)` — Camera.h:73), skip if `abs(dir.z) < nadirThreshold`, compute `gsd_k = avgDepth / GetFocalLength()` (Camera.h:78).
4. Return median via `FloatArr::GetMedian()`.
5. If no cameras pass nadir filter, log error and return 0.

**`ComputeGrid()`**:
1. Get mesh AABB: `scene.mesh.GetAABB()`.
2. Expand XY by `max(0.1f, max(extentX, extentY) * marginFactor)`. Do NOT expand Z.
3. Compute GSD via `ComputeGSD()`. Return false if 0.
4. Grid dimensions: `ceil(sizeX / gsd)` x `ceil(sizeY / gsd)`.
5. Tile layout: `effectiveStride = tileSize - tileOverlap`, `tilesX = ceil(widthPx / effectiveStride)`.
6. Allocate tiles and compute per-tile pixel bounds and world bounds.

**Critical Y-axis convention**: Pixel row 0 = north (max world Y), rows increase southward:
```
worldBounds.ptMax.y = sceneAABB.ptMax.y - py0 * gsd       // north edge
worldBounds.ptMin.y = sceneAABB.ptMax.y - pyEnd * gsd     // south edge
```

**`SetupTileCamera()`** — follows `ProjectOrthoTopDown` (Mesh.cpp:3116-3136):
```cpp
camera.R.SetFromDirUp(Vec3(Point3(0,0,-1)), Vec3(Point3(0,1,0)));
camera.C = Point3(centerX, centerY, sceneAABB.ptMax.z + 1.0);
camera.K = KMatrix::IDENTITY;
camera.K(0,0) = camera.K(1,1) = 1.0 / gsd;    // pixels per meter
camera.K(0,2) = (REAL)(tile.size.width - 1) / 2;
camera.K(1,2) = (REAL)(tile.size.height - 1) / 2;
```

**`BuildSpatialIndex()`**: Reuse `Mesh::FacesInserter::CreateOctree(octree, scene.mesh)` (Mesh.h:101-114).

**`RasterizeTile()`** — define local CRTP rasterizer following `ProjectOrtho` (Mesh.cpp:3047-3061):
```cpp
struct RasterOrthoGBuffer : TRasterMesh<RasterOrthoGBuffer> {
    typedef TRasterMesh<RasterOrthoGBuffer> Base;
    RasterOrthoGBuffer(const VertexArr& _v, const Camera& _c, DepthMap& _d)
        : Base(_v, _c, _d) {}
    inline bool ProjectVertex(const Mesh::Vertex& pt, int v, Triangle& t) {
        return (t.ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt))).z > 0 &&
            depthMap.isInsideWithBorder<float,3>(t.pti[v] = camera.TransformPointOrthoC2I(t.ptc[v]));
    }
    void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary) {
        const Depth z(ComputeDepth(t, bary));
        ASSERT(z > Depth(0));
        Depth& depth = depthMap(pt);
        if (depth == 0 || depth > z)
            depth = z;
    }
};
```

Calling code:
1. `tile.depthMap.create(tile.size); tile.depthMap.memset(0);`
2. Iterate `tileFaces`.
3. Call `rasterer.Project(face, triangleRasterizer)` for each passing face.

**`RasterizeTiles()`** — main parallel loop:
1. `BuildSpatialIndex()`.
3. `#pragma omp parallel for schedule(dynamic)` over tiles.
4. Per tile: query octree with `FacesInserterAABB` (enlarged by `gsd * 16`), then `RasterizeTile()`.
5. Thread safety: octree read-only, each thread has local `FaceIdxArr`, writes only to its own tile's `depthMap`.

**`Scene::ComputeOrthoMap()`** — entry point:
1. Validate mesh not empty.
2. Create `OrthoConfig` from parameters, create `OrthoMapContext`.
3. `ctx.ComputeGrid()`, `ctx.RasterizeTiles()`.
4. Log summary with `VERBOSE()`.

### Step 3: `libs/MVS/Scene.h` — Declaration

After line 165 (after `TextureMesh` declaration), add:
```cpp
// Orthomap generation
bool ComputeOrthoMap(float orthoResolution = 0, unsigned tileSize = 4096,
    unsigned tileOverlap = 16);
```

No `#include "SceneOrthoMap.h"` needed in Scene.h — the method signature uses only primitive types. The `OrthoMapContext` class lives entirely in the `.cpp` file.

### Step 4: `apps/OrthoMap/OrthoMap.cpp` — App

Follow TextureMesh.cpp pattern exactly (OPT namespace, Application class, boost::program_options):

**Options**:
- `--input-file,-i`: input .mvs file
- `--mesh-file,-m`: optional mesh override
- `--output-file,-o`: output filename
- `--ortho-resolution`: GSD in m/px (0 = auto)
- `--tile-size`: tile size in pixels (default 4096)
- `--tile-overlap`: tile overlap in pixels (default 16)
- `--skip-vertical-threshold`: face normal.z filter (default 0.1)
- Standard generic options (help, working-folder, config-file, archive-type, process-priority, max-threads, verbosity)

**main()**:
1. Initialize application.
2. Load scene, optionally load mesh override.
3. Validate mesh not empty.
4. Call `scene.ComputeOrthoMap(...)`.
5. Log completion time.

### Step 5: `apps/OrthoMap/CMakeLists.txt`

Follow TextureMesh/CMakeLists.txt pattern exactly:
```cmake
if(MSVC)
    create_rc_files(OrthoMap)
    FILE(GLOB LIBRARY_FILES_C "*.cpp" "${CMAKE_CURRENT_BINARY_DIR}/*.rc")
else()
    FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")
cxx_executable_with_flags(OrthoMap "Apps" "${cxx_default}" "MVS" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})
INSTALL(TARGETS OrthoMap EXPORT OpenMVSTargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
```

### Step 6: `apps/CMakeLists.txt` — Registration

Add `ADD_SUBDIRECTORY(OrthoMap)` after line 11 (`TextureMesh`), before line 12 (`TransformScene`).

---

## Key Reuse References

| Component | File | Line |
|-----------|------|------|
| `TRasterMesh` CRTP base | `libs/MVS/Mesh.h` | 288-380 |
| `ProjectOrtho` (depth-only rasterizer) | `libs/MVS/Mesh.cpp` | 3045-3068 |
| `ProjectOrthoTopDown` (camera setup) | `libs/MVS/Mesh.cpp` | 3116-3136 |
| `TransformPointOrthoC2I` | `libs/MVS/Camera.h` | 414 |
| `FacesInserter::CreateOctree` | `libs/MVS/Mesh.h` | 101-114 |
| `FacesInserterAABB` | `libs/MVS/Mesh.h` | 116-123 |
| `Camera::Direction()` | `libs/MVS/Camera.h` | 73 |
| `Camera::GetFocalLength()` | `libs/MVS/Camera.h` | 78 |
| `Scene::ComputeDistanceCameras2Scene()` | `libs/MVS/Scene.h` | 131 |
| TextureMesh app pattern | `apps/TextureMesh/TextureMesh.cpp` | 1-337 |

---

## Verification

1. **Build**: `cmake --build make/ -j4` — new files compile and link.
2. **Smoke test**: Run `./bin/Debug/OrthoMap -i scene.mvs` on a test scene with mesh. Verify:
   - Auto-computed GSD is reasonable (0.01-0.1 m/px for drone datasets).
   - Grid dimensions match `ceil(extent / gsd)`.
   - Tile count matches `ceil(sizePx / effectiveStride)`.
3. **Depth map check**: Non-zero depth values inside mesh footprint, zero outside, all non-zero values positive.
4. **Edge tiles**: Last row/column tiles have correct reduced dimensions.
5. **Y-axis**: Point at max world Y appears at pixel row 0.

---

## Notes

- **Do NOT remove** `ProjectOrthoTopDown` from TextureMesh — leave backward compat until full pipeline (Steps 1-4) is complete.
- **Do NOT include** `SceneOrthoMap.h` from `Scene.h` — the header is only consumed by `SceneOrthoMap.cpp`.
- The `OrthoMapContext` lives on the stack and is discarded after Step 1. Later steps will extend the entry point to pass the context forward.
