This document specifies an orthomap (true orthophoto) generation pipeline for OpenMVS, designed to maximally reuse the infrastructure of the Advanced Texturing Pipeline. Given an already-reconstructed mesh with Z-up orientation, the pipeline produces a geometrically correct, seamless top-down image of the scene — an orthomap where every pixel maps to a real-world XY coordinate with uniform scale.

The pipeline is structured in 4 steps:

Step 1 — Grid Initialization & DEM Generation: Compute the scene's XY bounding box, target GSD, tile decomposition, and rasterize the mesh into a G-buffer (depth map) under orthographic projection.

Step 2 — Ortho Projection per View & Pixel-block-Based View Selection: Per tile, ortho project based on the DEM all images seeing the tile. Next, per-pixel-block view scoring with nadir preference, BRIEF-based outlier removal, and alpha-expansion MRF on a regular 2D grid.

Step 3 — Multi Band Blending: Apply multi-band blending between inlier ortho views.

Step 4 — Tile Export & Assembly: Export tile one-by-one with georeference data, and merge them into the final orthomap image with optional DSM export and georeferencing metadata.

Help me design and plan Step 1 of the new ortho-map generation pipeline:

# Step 1: Grid Initialization & DEM Generation — Implementation Plan

## Context

We are building Step 1 of a new orthomap (true orthophoto) pipeline for OpenMVS. Given a reconstructed mesh with Z-up orientation, Step 1 computes the scene's XY bounding box, determines the target ground sampling distance (GSD), decomposes the output into tiles, and rasterizes the mesh into a per-tile G-buffer (depth map) under orthographic projection. This G-buffer is the foundation for all subsequent steps (view selection, blending, export).

The pipeline lives in new files (`SceneOrthoMap.h/cpp`) and reuses the existing orthographic rasterization infrastructure (`TRasterMesh`, `ProjectOrtho`, `Camera` ortho transforms, etc).

## Implementation Steps

### Step 1.1 — Data Structures (`SceneOrthoMap.h`)

Define these structs in namespace `MVS`:

**`OrthoConfig`** — pipeline parameters with defaults:
```
targetGSD: float = 0          // 0 = auto-compute from cameras; >0 = user override (m/px)
tileSize: unsigned = 4096     // tile dimension in pixels
tileOverlap: unsigned = 16    // overlap for blending
skipVerticalThresh: float = 0.1f  // skip faces with abs(N.z) < threshold
nadirThreshold: float = 0.3f     // camera nadir filter for GSD computation
marginFactor: float = 0.01f      // expand AABB by max(0.1m, extent * marginFactor)
```

**`OrthoGrid`** — computed grid layout:
```
sceneAABB: AABB3f             // expanded mesh AABB
gsd: float                    // final GSD (m/px)
sizePx: cv::Size              // total grid dimensions
tiles: Point2u                // tile counts
```

**`OrthoTile`** — per-tile data:
```
tile: Point2u        // tile grid indices
p0: Point2u           // start pixel in global grid
size: cv::Size        // tile pixel dimensions
worldBounds: AABB3f            // world XY region this tile covers
camera: Camera                 // orthographic camera for this tile
depthMap: DepthMap             // G-buffer: depth (camera-space, min-z = highest world surface)
```

**`OrthoMapContext`** — orchestrator (not a Scene member; takes `Scene&`):
```
scene: Scene&
config: OrthoConfig
grid: OrthoGrid
tiles: cList<OrthoTile>
octree: Mesh::Octree
```

### Step 1.2 — GSD Computation

**Function**: `float OrthoMapContext::ComputeGSD() const`

Algorithm:
1. If `config.targetGSD > 0`, return it (user override).
2. Ensure camera distances are computed: call `scene.ComputeDistanceCameras2Scene()` if any `image.avgDepth == 0`.
3. For each valid image:
   - Get viewing direction: `image.camera.Direction()` → returns `R.row(2)` (Camera.h:73)
   - Nadir test: `abs(dir.z)` (since down-looking has dir=(0,0,-1), `abs(dir.z)` measures nadir-ness). Skip if `< config.nadirThreshold`.
   - Compute `gsd_k = image.avgDepth / image.camera.GetFocalLength()` — this is world-meters-per-pixel, add a new function in `Image` if needed. Reuse `Camera::GetFocalLength()` which returns `K(0,0)` (Camera.h:78).
4. If no cameras pass nadir filter, return error.
5. Return **median** of collected GSD values (use `cList::GetMedian()`).

**Reuses**: `Camera::Direction()`, `Camera::GetFocalLength()`, `Scene::ComputeDistanceCameras2Scene()`, `Image::avgDepth`.

### Step 1.3 — Grid & Tile Setup

**Function**: `void OrthoMapContext::ComputeGrid()`

Algorithm:
1. Compute mesh AABB: `AABB3f box = scene.mesh.GetAABB()` (Mesh.cpp uses `AABB3f(vertices.data(), vertices.size())`).
2. Expand XY by margin: `max(0.1f, extent * config.marginFactor)` on each side.
3. Compute GSD: `grid.gsd = ComputeGSD()`.
4. Grid dimensions: `widthPx = ceil(sizeX / gsd)`, `heightPx = ceil(sizeY / gsd)`.
5. Tile layout: `effectiveStride = tileSize - tileOverlap`, `tilesX = ceil(widthPx / effectiveStride)`, similarly for Y.
6. Allocate tiles and compute per-tile pixel bounds and world bounds.

**Critical Y-axis convention**: Pixel row 0 = north (max world Y), pixel row increases southward. So:
```
tile.worldMinY = sceneAABB.ptMax.y - tile.pyEnd * gsd   // south edge
tile.worldMaxY = sceneAABB.ptMax.y - tile.py0 * gsd     // north edge
```
This matches the camera's Y-flip from `SetFromDirUp((0,0,-1), (0,1,0))` which produces `R = [[1,0,0],[0,-1,0],[0,0,-1]]`.

### Step 1.4 — Tile Camera Setup

**Function**: `Camera OrthoMapContext::SetupTileCamera(const OrthoTile& tile) const`

Follow the pattern or reuse `ProjectOrthoTopDown` (Mesh.cpp:3116-3136):
```
R.SetFromDirUp(Vec3(0,0,-1), Vec3(0,1,0))    // look down, Y-up
C = (tile world center X, tile world center Y, sceneAABB.ptMax.z + 1.0)
K = Identity
K(0,0) = K(1,1) = 1.0 / gsd                  // pixels per meter
K(0,2) = (tile.width - 1) / 2.0              // principal point at tile center
K(1,2) = (tile.height - 1) / 2.0
```

**Verified coordinate mapping**:
- `TransformPointOrthoC2I` (Camera.h:414) uses `K(0,2) + K(0,0) * cam.x`, ignoring Z.
- World X → camera X (direct), world Y → camera -Y (R flips Y), world Z → camera -Z (R flips Z).
- Higher world Z → smaller camera Z → wins `depth == 0 || depth > z` test. Correct: min camera-Z = highest world surface.

### Step 1.5 — Spatial Index

**Function**: `void OrthoMapContext::BuildSpatialIndex()`

Reuse `Mesh::FacesInserter::CreateOctree(octree, scene.mesh)` (Mesh.h:100-114) — builds octree from face centroids.

Per-tile query using `FacesInserterAABB` (Mesh.h:116-122):
```cpp
Mesh::FaceIdxArr tileFaces;
AABB3f queryBounds = tile.worldBounds;
queryBounds.Enlarge(grid.gsd * 16);  // safety margin for faces straddling tile edges
Mesh::FacesInserterAABB inserter(tileFaces, queryBounds);
octree.Collect(inserter, inserter);  // Octree.h:144 — INSERTER+COLLECTOR pattern
```

The `Enlarge` accounts for large faces whose centroid is outside the tile but whose geometry overlaps it. The 16-pixel margin matches the tile overlap.

### Step 1.6 — G-Buffer Rasterization

**Function**: `void OrthoMapContext::RasterizeTile(OrthoTile& tile, const Mesh::FaceIdxArr& tileFaces) const`

Define a CRTP rasterizer following `Mesh::ProjectOrtho` (Mesh.cpp:3045-3068):

```cpp
struct RasterOrthoGBuffer : TRasterMesh<RasterOrthoGBuffer> {
    Mesh::FIndex idxFace;

    // Override ProjectVertex for orthographic projection (same as ProjectOrtho)
    bool ProjectVertex(const Mesh::Vertex& pt, int v, Triangle& t) {
        return (t.ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt))).z > 0 &&
            depthMap.isInsideWithBorder<float,3>(t.pti[v] = camera.TransformPointOrthoC2I(t.ptc[v]));
    }

    // Raster callback: depth test + store world-space face normal
    void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary) {
        const Depth z = ComputeDepth(t, bary);  // No perspective correction for ortho
        Depth& depth = depthMap(pt);
        if (depth == 0 || depth > z)
            depth = z;
    }
};
```

Calling code iterates `tileFaces`, sets `idxFace`, and calls `Project(face, triangleRasterizer)`.

**Key design decisions**:
- **No perspective-correct barycentrics**: Orthographic projection is affine — raw bary coords are correct. Matches existing `ProjectOrtho` which does NOT call `PerspectiveCorrectBarycentricCoordinates`.

### Step 1.7 — Scene Entry Point

**In `Scene.h`** (after line 165, the `TextureMesh` declaration):
```cpp
bool ComputeOrthoMap(float orthoResolution = 0, unsigned tileSize = 4096,
    unsigned tileOverlap = 16, float skipVerticalThresh = 0.1f);
```

**In `SceneOrthoMap.cpp`**:
```cpp
bool Scene::ComputeOrthoMap(float orthoResolution, unsigned tileSize,
    unsigned tileOverlap, float skipVerticalThresh) {
    // Build config, create context, compute grid, rasterize G-buffer
    // Log summary with VERBOSE()
    // Return tiles for subsequent pipeline steps
}
```

### Step 1.8 — OrthoMap Pipeline Integration

**Create a new app called `OrthoMap.cpp`**:
1. Add option: `("ortho-resolution", value(&OPT::fOrthoResolution)->default_value(0), "orthomap GSD in meters/pixel (0=auto)")`.
2. Add any other options needed to manage the ortho map generation (tile size, overlap, image resolution, etc).
3. remove `ProjectOrthoTopDown` call from `TextureMesh` (no backward compat).

### Step 2.1 — Main Rasterization Loop (not to implement yet, but design for parallelization)

**Function**: `void OrthoMapContext::RasterizeTiles()`

1. Build spatial index once.
2. Create image caches for view images to avoid reloading across tiles (use existing cache mechanism using `ListFIFO.h`).
2. Parallel loop over tiles with `#pragma omp parallel for schedule(dynamic)`:
   - Rasterize G-buffer for each tile independently:
     1. Each thread queries octree (read-only, thread-safe) with local `FaceIdxArr`.
     2. Setup tile camera.
     3. Rasterize faces into tile's `DepthMap`.
   - Ortho project all views seeing the tile (read-only, thread-safe) to generate a list of ortho images seen by the tile.
   - Find similarities between pixel blocks in ortho views using BRIEF descriptors and remove outlier blocks for blending (per-tile MRF on regular grid, no shared state).
   - Multi band blending of the clean views only on 3 layers pyramid.
   - No shared mutable state between threads.

Memory: ~960 MB per tile at 4096^2 (~64 depth-map, ~48 ortho images x ~20 numViewsSeingTile). Peak = `nThreads * 960 MB`. Acceptable for 4-8 threads on most machines, check free memory size to set the number of threads.

---

## Verification Plan

1. **Build**: `cmake --build make/ -j4` — confirms new files compile and link.
2. **Unit test**: Add test in `apps/Tests/` that:
   - Loads a small `.mvs` scene with mesh.
   - Calls `ComputeOrthoMap()` with a known GSD.
   - Verifies grid dimensions match expected `ceil(extent / gsd)`.
   - Verifies depth map is non-zero inside the mesh footprint and zero outside.
   - Verifies normal map Z-component is positive (upward-facing faces) for non-zero pixels.
3. **Integration test**: Run `OrthoMap --ortho-resolution 0.05` on a test dataset, check that tiles are generated with correct dimensions.
4. **Visual check**: Export a tile's depth map as a grayscale image and compare against the mesh viewed from above in the Viewer.

---

## Key Reuse Reference

| Component | Source File | Line |
|-----------|------------|------|
| `TRasterMesh` CRTP base | `libs/MVS/Mesh.h` | 347 |
| `ProjectOrtho` (ortho rasterizer reference) | `libs/MVS/Mesh.cpp` | 3045 |
| `ProjectOrthoTopDown` (camera setup reference) | `libs/MVS/Mesh.cpp` | 3116 |
| `TransformPointOrthoC2I` | `libs/MVS/Camera.h` | 414 |
| `TransformPointOrthoW2I` | `libs/MVS/Camera.h` | 418 |
| `SetFromDirUp` | `libs/Common/Rotation.inl` | 1203 |
| `FacesInserter::CreateOctree` | `libs/MVS/Mesh.h` | 100 |
| `FacesInserterAABB` | `libs/MVS/Mesh.h` | 116 |
| `Octree::Collect(inserter, collector)` | `libs/Common/Octree.h` | 144 |
| `Mesh::GetAABB()` | `libs/MVS/Mesh.cpp` | ~180 |
| `Mesh::ComputeNormalFaces()` | `libs/MVS/Mesh.h` | 170 |
| `Camera::Direction()` | `libs/MVS/Camera.h` | 73 |
| `Camera::GetFocalLength()` | `libs/MVS/Camera.h` | 78 |
| `Scene::ComputeDistanceCameras2Scene()` | `libs/MVS/Scene.h` | 131 |
| `Image::avgDepth` | `libs/MVS/Image.h` | ~50 |
