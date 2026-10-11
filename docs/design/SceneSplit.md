# Scene Split (scalable dense reconstruction)

## 1. Purpose and scope

A scene too large for one machine's memory is reconstructed in sub-scenes: the depth-maps are
computed for the whole scene, the scene is then cut into boxes whose depth-map fusion and mesh
reconstruction each fit a memory budget, every box is fused, meshed and refined on its own, the box
point-clouds and meshes are merged, and the merged mesh is textured from all the images. A scene that
fits the budget is not split and goes through the same modules as an unsplit run. Library code:
`Scene::Split` and `Scene::ExportChunks` (`libs/MVS/SceneSplit.cpp`); CLI: `DensifyPointCloud
--sub-scene-memory` (after `--fusion-mode 1`); driver and merge: `scripts/python/MvsScalablePipeline.py`
(also run by `MvgMvsPipeline.py` for its steps 18-21, `--sub-scene-memory auto` by default),
`scripts/python/MvsMergeChunks.py`.

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
longest axis of its samples' extent at the sample quantile floor(K/2)/K, so every leaf holds the
same number of samples. K starts at a lower bound, the count at which a leaf's share of the points
fits either stage with no depth-map at all, and grows by one until the largest leaf's predicted cost
fits `maxMemory`; when four more parts stop lowering the largest leaf by 1%, the best K so far is
kept with a warning (the depth-maps an image brings put a floor under a box's cost, and no split
goes below it).

### 2.4 Images of a box
An image belongs to a box if it has samples there and its share of its own samples in the box is at
least 30% of the best share any image has in that box. An image whose samples fall >90% in a box with
more images is then removed from the smaller box.

### 2.5 Budget
`--sub-scene-memory <0` (the driver's `auto`) fits the machine at the time of the split: the physical
memory available (`Util::GetMemoryInfo().freePhysical`, page cache included on every OS: `MemAvailable`
on Linux) less a reserve of 10% of the total and at least 2 GB, for the rest of the system and the
error of the cost model. `Split` returns false if the budget is not above a sub-scene's fixed cost (3.8
GB) or no depth-map gives a sample, and leaves the chunk list empty when the largest predicted stage of
the whole scene fits: `DensifyPointCloud` then exports nothing and exits successfully.

### 2.6 Export and merge
`ExportChunks` writes `scene_XXXX.mvs` per box with the box as the sub-scene ROI; each sub-scene is
fused with `--geometric-iters 0` (the depth-maps exist, only the fusion runs; on a whole scene that
fits, the result is byte-identical to an unsplit run's on the same depth-maps) and its point-cloud is
cropped to the box.
`ReconstructMesh --roi-border` can grow the ROI to reconstruct with context across the boundary.
`MvsMergeChunks.py scene_XXXX.mvs scene_XXXX_<suffix>.ply merged.ply` keeps of each sub-scene mesh the
faces whose centroid lies in its box, of each point-cloud the points inside it (half-open on the upper
side, closed on the scene box's own upper faces, so a boundary element goes to exactly one box), and
concatenates them with all their vertex properties, the per-point view lists included (no stitching, no
texture coordinates).

### 2.7 Driver
`MvsScalablePipeline.py scene.mvs [--memory auto|off|GB]` runs in the scene's folder: depth-maps
(`--fusion-mode 1`), split into `subscenes/`, then per sub-scene `DensifyPointCloud`, `ReconstructMesh`,
`RefineMesh`, the merges into `scene_dense.ply`, `scene_dense_mesh.ply`, `scene_dense_mesh_refined.ply`,
and `TextureMesh scene.mvs -m scene_dense_mesh_refined.ply`. Unsplit (`--memory off`, or a scene that
fits) it runs the four modules on the whole scene. Extra module options go through `--densify`,
`--mesh`, `--refine`, `--texture`; `--steps` selects stages (without `densify`, the sub-scenes an
earlier run wrote are reused, and only the products of the selected stages are merged), `--resume`
skips existing outputs.

## 3. Parameters

| option | default | meaning |
|---|---|---|
| `DensifyPointCloud --sub-scene-memory` | 0 | budget per sub-scene in GB (0 disables, < 0 fits this machine, §2.5) |
| `MvsScalablePipeline.py --memory`, `MvgMvsPipeline.py --sub-scene-memory` | auto | auto, off or GB |

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

EPFL Herz-Jesu-P8 through `MvsScalablePipeline.py` (`--refine "--scales 1"`), refined mesh F1 (1 cm):

| run | sub-scenes | wall | F1 (P / R) |
|---|---|---|---|
| `--memory off` | 1 | 2m02 | 0.5225 (0.5729 / 0.4803) |
| `auto` (10.3 GB budget, scene predicted 4.2 GB) | 1, not split | 1m58 | 0.5218 (0.5715 / 0.4800) |
| `--memory 4.2` (forced) | 2 | 1m51 | 0.5133 (0.5600 / 0.4738) |

The `auto` row differs from `off` only by the CUDA depth-map estimation, which is not bit-reproducible
between runs: re-fusing the `off` run's own depth-maps with `--geometric-iters 0` reproduces its point-cloud
byte for byte. The forced split of an 8-view scene costs 0.009 F1 at the seam.

## 6. Rejected alternatives

- Octree area split (sampling area as a proxy of memory): see §5.
- Overlapping boxes (`--roi-border 10`) by default: +0.0002 F1 for 1.5-2x mesh memory and time.

## 7. Open items

- The fusion keeps a depth-map cache sized from the free memory, so its measured peak follows the
  machine as much as the box; the mesh stage, at ~3 GB for a 12 GB fusion box, is far from its budget.
- No cost term for `RefineMesh` per box (it runs on the box mesh with the box's images) and none for
  `TextureMesh`, which runs once on the merged mesh with all the images.
- The boundaries are not stitched: the merged mesh has seams where two boxes meet.
