This document specifies an orthomap (true orthophoto) generation pipeline for OpenMVS, designed to maximally reuse the infrastructure of the Advanced Texturing Pipeline. Given an already-reconstructed mesh with Z-up orientation, the pipeline produces a geometrically correct, seamless top-down image of the scene — an orthomap where every pixel maps to a real-world XY coordinate with uniform scale.

The pipeline is structured in 4 steps:

Step 1 — Grid Initialization & DEM Generation: Compute the scene's XY bounding box, target GSD, tile decomposition, and rasterize the mesh into a G-buffer (depth map) under orthographic projection.

Step 2 — Ortho Projection per View & Pixel-block-Based View Selection: Per tile, ortho project based on the DEM all images seeing the tile. Next, per-pixel-block view scoring with nadir preference, BRIEF-based outlier removal, and alpha-expansion MRF on a regular 2D grid.

Step 3 — Multi Band Blending: Apply multi-band blending between inlier ortho views.

Step 4 — Tile Export & Assembly: Export tile one-by-one with georeference data, and merge them into the final orthomap image with optional DSM export and georeferencing metadata.

Help me design and plan next Step 2 of the new ortho-map generation pipeline. Start by understanding the current code, and then design the implementation to be efficient, leveraging parallel processing where possible, and should handle memory constraints effectively. Research state of the art orthomap generation pipelines to inform design decisions, especially for view scoring and outlier removal.

# Step 2: Ortho Projection per View & Pixel-block-Based View Selection — Implementation Plan

## Context

We are building Step 2 of a new orthomap (true orthophoto) pipeline for OpenMVS. Given a generated G-buffer from Step 1, Step 2 performs ortho projection per view and pixel-block-based view selection. This involves projecting all images that see a given tile based on the DEM, scoring views per pixel block with nadir preference, removing outliers using BRIEF descriptors, and applying alpha-expansion MRF on a regular 2D grid. Step 2 operates on a per-tile basis, where each tile corresponds to a portion of the orthomap defined in Step 1.

## Main Rasterization Loop (containing all sub-steps for a global understanding of the process)

**Function**: `void OrthoMapContext::RasterizeTiles()`

1. Build spatial index once.
2. Create image caches for view images to avoid reloading across tiles (use existing cache mechanism using `libs/Common/ListFIFO.h`).
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

## Implementation Steps

### Step 2.1 — Ortho Project Views per Tile

Ortho project all views that see the tile based on the DEM. This involves:
- For each tile, identify which views see it
- For each view, use the DEM to compute the orthographic projection of the view onto the tile's plane
- Split the ortho-projected images into pixel blocks for subsequent scoring and filtering; default 4x4 pixel blocks (compile time configurable)
- For each pixel block, compute a weight: NADIR weight (the viewing angle of the block center ray with the Z axis, with a strong preference for nadir views) + gradient weight (optional sum of absolute Sobel gradients in the block which encapsulates image information, with a tunable weight to balance against the NADIR weight); research if this helps in state of the art orthomap generation pipelines

**Nadir exponent**: Quadratically favors cameras that look straight down. For purely nadir drone captures, this has minimal effect (all views are near-nadir). For convergent or mixed oblique/nadir captures, it strongly prefers nadir views for ground surfaces, preventing wall-facing cameras from being selected.

**Image caching**: Starting with this stage we will use extensively the view images, so we also need to create an image cache to avoid reloading them across tiles. We can use the existing cache mechanism using `libs/Common/ListFIFO.h` for this purpose. See other parts of the codebase for examples of how to use this cache.

### Step 2.2 — BRIEF Descriptor Computation

Transient removal (people, cars, shadows) operates identically to the texturing pipeline in concept, but adapted for the grid-based sampling:

Compute a BRIEF descriptor to capture local image structure for outlier detection. For each pixel block in the ortho-projected views, compute a BRIEF descriptor. This will be used to identify and remove outlier blocks that do not match well across views, which can cause artifacts in the final blended orthomap.

**Sample 3D points for the BRIEF descriptor**:

For each pixel block, sample on the DEM surface in a local neighborhood around the block center:

```
For block b at center P:
    Generate 16 points within the block's footprint (4×4 regular sub-grid on the DEM surface)
    Generate 16 points in the surrounding blocks (8-connected neighbors, 2 points per neighbor)
    Total: 32 sample points S_0...S_31 in 3D (X, Y from grid, Z from DEM)
```

**Compute BRIEF descriptor**: Standard BRIEF computation — sample grayscale intensity, apply the 256 fixed comparison pairs, etc.

To save memory, an idea to investigate is to compute BRIEF descriptors for the same pixel block across all views and store only the similarity indices instead of the full descriptors. For example, for each block, compute the BRIEF descriptor in the first view and then compute the Hamming distance to that descriptor for the same block in all other views. This way we only store one full descriptor per block and a set of similarity scores for the other views, which can be used for outlier detection.

### Step 2.3 — MRF-Based Outlier Removal

At this stage, we have for each pixel block a set of ortho-projected views that see it, along with their weights (nadir + gradient) and BRIEF-based similarity scores. We want to remove outlier blocks that do not match well across views to prevent blending artifacts. For example we want to remove blocks that see a transient object (e.g., a car) in one view but not in others, which would cause ghosting in the blended orthomap.

Simply filtering out the blocks in the view with not enough similar blocks in other views is possible, but it can lead to inconsistent block selection across the pixel blocks in the tile, which can cause artifacts in the blended result. Instead, we can formulate this as an MRF optimization problem where we want to select a subset of views for each block that maximizes the overall similarity while also enforcing spatial consistency (neighboring blocks should have similar view selections).

For the orthomap, the output lives on a *regular pixel grid*, so the MRF should operate on that same grid. This is different from the texturing pipeline where the MRF operates on an irregular mesh. The regular grid structure allows us to use a more efficient MRF optimization 2D based method, such as alpha-expansion graph cuts, which can handle large label spaces (many views) and enforce spatial smoothness effectively.

**Goal**: For each pixel block, select a subset of views that are consistent with each other (high BRIEF similarity), while also enforcing that neighboring blocks have similar view selections to avoid blending artifacts. Remove the pixel blocks in views that are deemed outliers based on this optimization before blending.

### Step 2.4 Alpha-Expansion MRF on 2D Grid

**Graph structure**: Regular 4-connected grid of blocks. Each block `b` has 4 neighbors: `(bx±1, by)` and `(bx, by±1)`. No adjacency list is needed — neighbors are computed by index arithmetic.

**Data term**:

```
E_data(b, l) = 1.0 − w_total(b, l)
```

where `w_total` includes the nadir bias. If view `l` is not in block `b`'s candidate set, `E_data(b, l) = ∞`.

**Smoothness term**:

```
E_smooth(b, q, l_b, l_q) = {
    0                                               if l_b = l_q
    color_diff(b, q, l_b, l_q) × flatness_factor    if l_b ≠ l_q
}
```

Where:
- `color_diff(b, q, l_b, l_q)` = average RGB color difference between views `l_b` and `l_q` sampled at the shared block boundary (5–10 sample points along the boundary, projected into both views).
- `flatness_factor = 1.0 + max(0, min_dihedral(b, q) / 10° − 1) × 0.5` — extra penalty for seams on flat terrain where they are more visible. Computed from the normal map: `min_dihedral ≈ acos(dot(N_b, N_q))`.

**Lambda** (smoothness weight): Default 1.0, same as texturing. Increase to 2.0–3.0 to reduce number of seams (larger coherent regions, possibly at the cost of sub-optimal view quality at some pixels).

**Alpha-expansion solver**: Instantiate `AlphaExpansion<>` from `AlphaExpansion.h` with the 2D grid graph. The solver template is agnostic to the graph structure — it only needs node count, neighbor iteration, and energy callbacks.

**Convergence**: Typically 3–6 passes through all labels (faster than mesh-based MRF due to the regular grid and spatial coherence of aerial imagery).

### Step 2.5 Per-Tile MRF

The strategie for how tiles relate to the MRF to implment is:

**Per-tile MRF with overlap**:
- Run independent MRFs on each tile with 32-block (128 pixel) overlap at borders.
- Faster and lower memory, but may have slight inconsistencies at tile boundaries.
- For 13×13 tiles (5 km scene): each tile has ~1 M nodes — trivial to solve.
