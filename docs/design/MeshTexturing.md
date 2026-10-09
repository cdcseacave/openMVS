# Mesh Texturing

## 1. Purpose and scope

`TextureMesh` maps every face of a mesh to the image that should texture it, levels the colours
across the seams between faces taken from different images, and packs the image regions into one or
more texture atlases. Library code: `libs/MVS/SceneTexture.cpp` (`MeshTexture`, `Scene::TextureMesh`,
`Scene::ComputeVertexColors`); CLI driver: `apps/TextureMesh/TextureMesh.cpp`. Texture coordinates are
stored per face (three per face, `Mesh::faceTexcoords`) with one texture index per face
(`Mesh::faceTexindices`, empty for a single atlas).

## 2. Algorithm as implemented

### 2.1 Face-view data (`MeshTexture::ListCameraFaces`)

For every view (in parallel, one view per thread):

1. The image is loaded at the working resolution (`--resolution-level`, `--min-resolution`).
2. The faces inside the view frustum (mesh octree) are rasterized at the **full image resolution**
   into a face-id map and a depth map (closest face wins). Faces whose pixels fall on the image's
   invalid area (`ComputeValidityMask`, lens-distortion border / ignore-mask label) are marked
   invalid for the view. Every pixel a face covers is also counted before the depth test
   (`RasterMesh::projArea`), occluded or not.
3. The **quality** of a face in the view is the number of its visible pixels: the texture resolution
   the view offers it, which already falls with distance and obliqueness. The pixels are counted
   (and their colours summed) per thread; only the faces the view sees are appended to the shared
   face-view lists, inside the one critical section.
4. A view that sees under 90% of the face's projection, with more than 2 pixels missing, keeps a
   tenth of that quality: it would copy the occluder into the face's texture, so it is chosen only
   where nothing sees the face whole.
5. With `--outlier-threshold` > 0 the mean colour of the face in each view is kept and
   `FaceOutlierDetection` (Gaussian clamping in YCbCr) removes views whose colour disagrees with the
   others (e.g. a passer-by in one image).

### 2.2 View selection (`MeshTexture::FaceViewSelection`)

A labeling over the face adjacency graph (label 0 = no view, label i+1 = view i), solved by loopy
belief propagation (`Math/LBP.h`):

- **Data cost** of view v for face f: `(1 - q_fv / max_v q_fv) * MaxEnergy * area_f/avgArea`, i.e.
  each face is scored relative to its own best view (cost 0); views below 0.2 of the best are not
  candidates. A face no view sees can only take label 0, at cost `MaxEnergy*(area_f/avgArea+0.5)`.
- **Smoothness**: Potts (0 for equal labels, `LBPMaxEnergy` otherwise), weighted per edge by
  `0.5 + edgeLength/avgEdgeLength`.
- `MaxEnergy = --cost-smoothness-ratio` (default 0.1) sets the data/smoothness balance.
- The Potts model lets each LBP message be computed in O(labels) (`LBPInference::SetSmoothCostPotts`):
  the sender's energy for a label is one of two values (same label: smoothness 0, otherwise the Potts
  cost), so the message of receiver label j is min(E0[j], min_k E1[k]); the energies are summed in the
  generic loop's order, so the messages are bit-identical to the O(labels^2) loop.
- `--virtual-face-images` > 0 runs the same labeling on virtual faces (coplanar groups of faces
  sharing at least that many views) instead of individual faces.

Connected faces with the same label form a texture patch; faces without a view go to the last
patch, textured with `--empty-color`.

### 2.3 Patch geometry and seam leveling (`GenerateTexture`)

- Each patch's faces are projected in its view; the patch rectangle is their bounding box plus a
  2 px border of real image content.
- **Global seam leveling** (`GlobalSeamLeveling`, `--global-seam-leveling`): one additive YCbCr
  correction per (vertex, patch), solved in the least-squares sense so the two sides of every seam
  vertex agree (colours sampled along the seam edges), with a Tikhonov term (lambda 0.1) keeping the
  corrections smooth inside a patch; conjugate gradient per channel; the correction is interpolated
  over each patch's triangles.
- **Local seam leveling** (`LocalSeamLeveling`, `--local-seam-leveling`): Poisson blending of a strip
  along each patch border so the border continues smoothly into the patch.
- A patch whose rectangle lies inside another patch's rectangle of the same view joins it.

### 2.4 Atlas

- Patches are grouped spatially (`SplitPatchesSpatially`) and each group is packed by halfmesh's
  skyline packer (`PackTexturePages`, rotation allowed, no padding) into square pages grown to fit,
  at most `--max-texture-size` (default 8192); a group that does not fit opens another page.
- The patch pixels are copied into the pages; texels no patch covers are filled by push-pull over a
  box pyramid of the covered texels (`PushPullFill`), so a mipmapped lookup near a patch border
  averages its own surroundings instead of the empty colour (which only the unseen faces' patch
  keeps).
- `--sharpness-weight` > 0 applies an unsharp mask (sigma 1.5) to the pages; disabled by default.

### 2.5 Vertex colours (`ComputeVertexColors`, `--vertex-colors`)

Same view selection; each vertex takes the weighted average of its colour in the views texturing the
faces around it; images are released as soon as their faces are consumed.

## 3. Parameters and defaults

| option | default | meaning |
|---|---|---|
| `--resolution-level` / `--min-resolution` | 0 / 640 | working image resolution |
| `--outlier-threshold` | 0.06 | colour-outlier view removal (0 disables) |
| `--cost-smoothness-ratio` | 0.1 | data vs. smoothness balance of the labeling |
| `--virtual-face-images` | 0 | label virtual faces of coplanar triangles (0 disables) |
| `--global-seam-leveling` / `--local-seam-leveling` | 1 / 1 | seam colour leveling |
| `--texture-size-multiple` | 0 | page size multiple (0 = power of two) |
| `--max-texture-size` | 8192 | page size cap; more pages when exceeded |
| `--empty-color` | 0x00FF7F27 | colour of faces no view sees |
| `--sharpness-weight` | 0 | unsharp-mask weight |
| `--remove-unseen-faces` | 0 | drop faces seen by fewer than N images before texturing |

Fixed: full-resolution raster, partial-visibility threshold 90% / 2 px and factor 0.1, per-face
candidate cut-off 0.2, LBP min edge weight 0.5, global leveling lambda 0.1.

## 4. Invariants

- Every face has three texture coordinates; library code never uses a per-vertex layout.
- A face's data cost exists only for views within 0.2 of its best; the best view of every seen face
  has cost 0, so a face is never forced into label 0 while some view sees it.
- The visible-pixel count of a face never exceeds its projected-pixel count (asserted).

## 5. Validation of the shipped defaults

Same-input bench `bench/cmp_a3d.py` (stages T/X) with `bench/eval_texture.py`: the refined mesh of each
scene textured with every 4th view left out, the textured mesh ray-cast into the held-out cameras
and compared with the photos (T&T: only the surface inside the evaluation crop; faces a run leaves
unseen are excluded for every arm).

| hold-out PSNR / SSIM | HJ8 | F11 | HJ25 | Ignatius |
|---|---|---|---|---|
| previous defaults (gradient data term, 2x raster, sharpening 0.5) | 25.84 / .9088 | 28.25 / .9332 | 24.15 / .8811 | 16.81 / .7234 |
| current defaults | 26.55 / .9222 | 28.80 / .9451 | 24.52 / .8989 | 17.47 / .7527 |
| Acute3D FinishMesh, same mesh and views | 26.29 / .9223 | 28.99 / .9448 | 23.84 / .8902 | 16.38 / .7060 |

TextureMesh peak RAM (HJ25 hold-out): 2.5-3.0 GB vs 5.8 GB with the 2x raster.

## 6. Rejected alternatives

- Gradient-magnitude data term (blurred Sobel x cos^2, global 95th-percentile normalization):
  replaced, see §5; each of its replacements' parts alone measured worse except the partial-visibility
  penalty.
- Crease-aware seam cost (edge weight x (1+n_a.n_b)/2): HJ8 -0.08 dB.
- Multiplicative, ratio-gated global seam leveling in log-RGB (Acute3D's gain model): HJ8 +0.05,
  F11 -0.09, HJ25 -0.2 dB.
- Merging same-view patches whose rectangles overlap (not only contained ones): -4% patch pixels but
  the page size never changed and the atlas stage was 20% slower.
- Unsharp mask by default: lowered PSNR and SSIM on every scene.

## 7. Open items

- Atlas fill: triangles cover only 13-28% of the page texels (rectangle packing of irregular patches,
  power-of-two square pages); Acute3D packs exact patch masks for 33-56%.
- The images stay loaded from view selection to atlas generation; peak RAM still grows with the view
  count (the face/depth rasters are per thread).
- Faces no view sees get the empty colour; a colour diffused from the neighbouring faces would be
  closer to the surface's appearance.
