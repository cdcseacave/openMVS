# CUDA Texturing and Orthomap (user guide)

Two opt-in CUDA backends added on top of `TextureMesh`. Both require
`OpenMVS_USE_CUDA=ON` at build time and are activated by runtime flags — default
behavior is unchanged.

| Branch | What it adds | Activation flag |
|---|---|---|
| `cuda-texture` (PR #1268) | Alternative texturing backend | `--cuda-blending` |
| `cuda-ortho` (PR #1269)   | Direct top-down orthomap, no texturing | `--ortho-only` |

---

## 1. `cuda-texture` — CUDA texture-atlas backend

### Goal
Provide an alternative to the existing CPU `TextureMesh` pipeline. The CPU path
assigns each face to a single best view (mapmap-LBP) and stitches the patches
together with Poisson seam-leveling. The CUDA path takes a different approach:
re-parametrize the whole mesh and blend many views per atlas region on the GPU.

### Method (high level)
1. **Re-parametrize the mesh with [xatlas](https://github.com/jpcy/xatlas)** —
   ignore any existing UVs, generate a new chart layout from the geometry.
2. **Pack charts into texture atlases** (one or more, bounded by `--max-texture-size`).
3. **For each input image**: load → run a CUDA rasterizer (CURAST module) to
   project the mesh into image-space and composite the visible faces into the
   corresponding atlas regions.
4. **Sequential per-view blending** with best-views averaging and outlier removal.

### Example
```bash
TextureMesh -w <scene_dir> \
  -i scene_dense.mvs \
  -m scene_dense_mesh.ply \
  -o textured.mvs \
  --cuda-blending \
  --max-texture-size 8192
```

Useful additional flags: `--max-img-size N` (cap input image resolution),
`--force-param` (force xatlas re-parametrization even if UVs exist),
`--force-pack` (force atlas re-packing).

### Observations from validation runs
- **Tanks-and-Temples Truck** (251 images, 900k-face mesh): CUDA took **32 min**
  vs CPU **2.5 min**; quality score 42.1 (CUDA) vs 48.5 (CPU).
- The single biggest cost is **xatlas `ComputeCharts`**, which is single-threaded
  and dominated wall-time at ~24 min on the 900k-face mesh.
- Quality on Truck is lower than CPU — expected, since the algorithms differ:
  xatlas-based reparametrization introduces more UV distortion than mapmap-LBP's
  per-face best-view assignment, and sequential blending averages images more
  softly than CPU's Poisson seam-leveling.
- **Coverage (mesh→camera completeness) is identical to CPU** — that's purely
  geometric and doesn't depend on the texturing backend.
- The CUDA path is best thought of as an **alternative**, not a faster
  replacement: it's intended for very large meshes or streaming scenarios where
  the GPU per-image rasterizer's structure pays off.

---

## 2. `cuda-ortho` — direct CUDA orthomap

### Goal
Produce a single top-down (or other planar) **orthographic image** of the
reconstructed surface — typically used on geo-referenced reconstructions where
you want a 2D map of the scene rather than a textured 3D mesh. The pipeline
skips full mesh texturing entirely.

### Method (high level)
1. **Per-view mesh projection** — for each input image, project the mesh and
   compute per-face visibility + view scores + resolution scores.
2. **Tile-size auto-search** — binary-search the world-space tile size that
   maximizes resolution while fitting within `--max-tile-res` and
   GPU memory.
3. **Per-tile multi-view rasterization** — for each tile, load each
   contributing view's image, build a Laplacian image pyramid on the GPU, and
   sample the image into a per-view tile stack via orthographic projection.
4. **Per-tile blending** — best-views selection, outlier removal,
   Laplacian-pyramid recombination, and per-pixel normal-map masking to
   limit blending to the flattest areas.
5. **Final assembly** — blend all per-tile PNGs into the final orthomap with
   overlap feathering.

### Example
```bash
TextureMesh -w <scene_dir> \
  -i scene_dense.mvs \
  -m scene_dense_mesh.ply \
  -o ortho.png \
  --ortho-only \
  --decimate 0.3 \
  --tile-size 0 \
  --max-tile-res 8192
```

`-o` may be either a relative filename (resolved against `-w`) or an absolute
path; both work. Tile PNGs (`tile_X_Y.png`) and intermediate caches are always
written to the working folder.

Useful additional flags: `--resume-orthomap` (skip re-rasterization if tile
PNGs already exist on disk), `--max-img-size N`.

### Observations from validation runs
- **Tanks-and-Temples Truck** (251 images, 900k-face mesh, auto tile size):
  total wall time **~11 min**, peak RSS 5.1 GB, output `truck_ortho.png` 1 MB.
- Auto tile-size binary search converged in ~30 iterations to 8.21 m
  world-space (with 5% overlap → 2 tiles).
- The bulk of the time is the per-view mesh projection + per-tile rasterization,
  not the binary search.

---

## Stacking note

`cuda-ortho` is built **on top of** `cuda-texture` and shares the CURAST CUDA
module. If both PRs are merged, all three flags (`--cuda-blending`,
`--ortho-only`, plus tile-related flags) are available together. Until then, PR
#1269 targets `cuda-texture` so its diff is just the ortho work — when #1268
merges, #1269 auto-rebases to `develop`.
