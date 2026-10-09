# Scene Split (scalable dense reconstruction)

## 1. Purpose and scope

A scene too large for one machine's memory is reconstructed in sub-scenes: the depth-maps are
computed for the whole scene, the scene is then cut into boxes whose depth-map fusion and mesh
reconstruction each fit a memory budget, every box is fused and meshed on its own, and the box meshes
are merged. Library code: `Scene::Split` and `Scene::ExportChunks` (`libs/MVS/SceneSplit.cpp`); CLI:
`DensifyPointCloud --sub-scene-memory <GB>` (after `--fusion-mode 1`); driver and merge:
`scripts/python/MvsScalablePipeline.py`, `scripts/python/MvsMergeChunks.py`.

## 2. Algorithm as implemented

### 2.1 Samples
Every `depthMapStep`-th pixel (default 8) of every depth-map is back-projected; samples outside the
scene ROI (when set) are dropped. Each sample remembers its image; each image counts its samples.

### 2.2 Cost model (`SplitCost::Predict`)
The predicted peak of a box is the larger of the two stages:
- fusion: `3.8 + 0.026 * Mdepths + 0.18 * Mpoints` GB, where `Mdepths` is the millions of depth-map
  pixels of the box's images (all of them: an image brings its whole map, not only the part in the
  box) and `Mpoints` the fused points, `0.05` per depth-map pixel in the box (`SplitPointsPerDepth`);
- Delaunay mesh: `1.7 + 1.0 * 0.35 * Mpoints` GB (`SplitVerticesPerPoint` = 0.35 Delaunay vertices per
  fused point after the 1.5 px merge).

### 2.3 Partition
Balanced recursive bisection of the samples into K parts: a box allotted K parts is cut on the
longest axis of its samples' extent at the sample quantile floor(K/2)/K, so every leaf holds the same
number of samples. K starts at the count the points alone need and grows until the largest leaf's
predicted cost fits `maxMemory`; when four more parts stop lowering the largest leaf by 1%, the best
K so far is kept with a warning (the depth-maps an image brings put a floor under a box's cost).

### 2.4 Images of a box
An image belongs to a box if it has samples there and its share of its own samples in the box is at
least 30% of the best share any image has in that box. An image whose samples fall >90% in a box with
more images is then removed from the smaller box.

### 2.5 Export and merge
`ExportChunks` writes `scene_XXXX.mvs` per box with the box as the sub-scene ROI.
`ReconstructMesh --roi-border` can grow the ROI to reconstruct with context across the boundary.
`MvsMergeChunks.py scene_XXXX.mvs scene_XXXX_mesh.ply merged.ply` keeps of each sub-scene mesh the
faces whose centroid lies in its box (half-open on the upper side, so a boundary face goes to exactly
one box) and concatenates them (geometry only, no stitching).

## 3. Parameters

| option | default | meaning |
|---|---|---|
| `--sub-scene-memory` | 0 | budget per sub-scene in GB (0 disables, < 0 uses 85% of the physical memory) |

Fixed: sample step 8 px, cost constants above, image contribution 0.3, containment 0.9.

## 4. Invariants

- Every leaf box is non-empty and the boxes tile the root box (ROI or sample extent) without overlap.
- `maxMemory > 0`, `depthMapStep > 0` (asserted).

## 5. Validation

Constants fitted on measured peak commit of fusion-only `DensifyPointCloud` and `ReconstructMesh`
runs on T&T and EPFL scenes (median relative error 14% fusion, 4% mesh).

T&T Church (507 views, 50M points), same depth-maps, fusion `--number-views 24`, default
`ReconstructMesh`, merged with `MvsMergeChunks.py`:

| split | sub-scenes | fusion peak (GB) | mesh peak (GB) | mesh F1 (P / R) |
|---|---|---|---|---|
| octree area split (`--sub-scene-area 3000000`, removed) | 5 | 9.0-9.7 | 2.2-2.7 | 0.5967 (0.6649 / 0.5412) |
| memory split, 12 GB | 4 | 10.0-11.5 (predicted 9.7) | 2.7-3.4 | 0.6085 (0.6648 / 0.5610) |
| memory split, 12 GB, `--roi-border 10` | 4 | 9.0-11.6 | 4.5-6.8 | 0.6087 (0.6613 / 0.5639) |

The area split's boxes spread 1.6-2.4x in load (max/median) with near-empty boxes next to over-full
ones, and no area value maps to a memory budget.

## 6. Rejected alternatives

- Octree area split (sampling area as a proxy of memory): see §5.
- Overlapping boxes (`--roi-border 10`) by default: +0.0002 F1 for 1.5-2x mesh memory and time.

## 7. Open items

- The fusion keeps a depth-map cache sized from the free memory, so its measured peak follows the
  machine as much as the box; the mesh stage, at ~3 GB for a 12 GB fusion box, is far from its budget.
- No cost term for `RefineMesh`/`TextureMesh` per box, and no textured merge.
