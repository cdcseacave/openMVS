# Mesh Refinement

`RefineMesh` implements the Vu et al. (PAMI 2012) variational mesh refinement: alternate a
multi-scale subdivision pass with a gradient-descent optimization that pulls the surface toward
photo-consistency while a Laplacian regularizer keeps it smooth. This document is the design
reference for that stage — what the pipeline does today (§1), what it measures against ground
truth (§2), how those numbers were produced (§3), what was tried and rejected on the way (§4),
and the constraints a future change has to respect (§5).

Two implementations exist side by side (`libs/MVS/AGENTS.md`'s guidance on platform-specialized
code applies): the CPU path in `libs/MVS/SceneRefine.cpp` (`MeshRefine`/`Scene::RefineMesh`) and
the CUDA path split across `libs/MVS/SceneRefineCUDA.cpp` (`MeshRefineCUDA`/
`Scene::RefineMeshCUDA`) and `libs/MVS/SceneRefineCUDA.cu`/`.inl` (device kernels and launch
wrappers). One file both backends share outright: `libs/MVS/SceneRefineCommon.h/.cpp` (shared
scalar math, the `OPTREFINE` configuration space, per-view image/mask preparation, and the
vertex-position stepper `MeshRefineStep` — one implementation, no CUDA twin to drift). There is no
`SceneRefine.h`; `MeshRefine`/`MeshRefineCUDA` are translation-unit-local
classes. `apps/RefineMesh/RefineMesh.cpp` is the CLI driver; every default below is that file's
`boost::program_options` default unless marked as an `OPTREFINE`/`MeshRefineStep` constant.

---

## 1. Pipeline as shipped

### 1.1 Entry point, CLI defaults, and the `-m`/output-naming trap

`main()` loads the scene, optionally attaches per-image masks, loads (or defaults) the mesh, and
picks a backend through `SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs)`
(`libs/Common/UtilCUDA.cpp`) — true for an empty string (the `--gpu-device` default), `-2`, `cpu` or
`none` (case-insensitive), false otherwise (`-1` = best GPU, `>=0` = comma-separated device IDs).
Refinement mutates the mesh in place, so `main()` snapshots `scene.mesh.vertices`/`faces` before a
CUDA attempt; if `RefineMeshCUDA` returns `false` (no device, OOM, a kernel launch failure, a
poisoned context after a mid-run error), it logs
`"CUDA mesh refinement failed: falling back to the CPU implementation"`, restores the snapshot, and
the CPU path runs on the caller's original input instead of re-refining (and re-decimating)
whatever the half-finished CUDA run left behind.

`--mesh-file/-m` defaults to `<input-file-without-extension>.ply` **only when the archive type is
MVS**. With `-i scene_dense.mvs` that default resolves to `scene_dense.ply` — the dense **point
cloud's** ply, not a mesh — which produces `mesh.IsEmpty()` and `"error: empty initial mesh"`. `-m`
must always be passed explicitly when refining a coarse mesh built from the same stem as the input
scene. The refined mesh is written to `<out-stem><export-type>` unconditionally (default `.ply`);
the `.mvs` sidecar is written only when the archive type isn't `ARCHIVE_MVS` or the input wasn't
loaded as `Scene::SCENE_INTERFACE`.

Per-view masking: `--mask-path` points at a folder of `<image>.mask.png` files (assigned to
`Image::maskName` if not already set from the `.mvs`) and `--ignore-mask-label` (default `-1`,
disabled) is the label value those masks encode to drop. Both are validated once at startup — a
missing mask file is logged once per image there, not repeated every scale by the hot per-view
loader (§1.3). `OPTREFINE::init()`/`update()` load the `OPTREFINE` defaults and merge in
`--refine-config-file` if given; the CLI, including this app's own `RefineMesh.cfg`, always wins
over that file, and a configuration file naming an option that does not exist is refused by name
rather than silently ignored. An option the CLI defines with a default of its own has to be applied
only when it was actually given (`vm[...].defaulted()`), or the default overwrites what the
configuration file set and the file silently does nothing -- `--simplify-tolerance` and
`--adaptive-face-size` both go through that check.

**Trading accuracy for speed.** `--fast` is the measured fast configuration behind one switch: it
prepares the mesh at twice the default face area (`--max-face-area 32`) and raises the decimation
tolerance to 0.5 px, which runs 1.3x faster (0.78x wall) for −0.005 mean F1 on Tanks & Temples and
delivers a mesh 0.40x the size of the default's already-decimated one (§2.7, §2.9). Until 2026-09
the preset cut the view budget instead (`--max-views 4`); measured against each other in one cell,
the face area dominates it on every axis at once — −0.005 against −0.010 mean F1, 0.40x against
0.60x faces, 0.78x against 0.83x wall — because a coarser mesh removes per-pixel work and mesh work
together while a smaller view budget removes only the first and costs Barn and Ignatius 0.015-0.021
F1 on its own. It is a preset over `--max-face-area` and `--simplify-tolerance` and nothing else,
and an explicitly given one of those wins over it — `--fast --max-face-area 16` keeps the default
density and only decimates. What no preset can do is go faster than that: 38 % of the wall is the mesh
preparation, which neither the view count nor the working resolution touches, and the levers that
do go faster (a coarser `--resolution-level`, `--max-views 2`) cost an order of magnitude more
accuracy (§4 #46).

### 1.2 Multi-scale subdivision loop

`Scene::RefineMesh`/`RefineMeshCUDA` run `nScales` passes coarse-to-fine (`--scales`, default 2;
`--scale-step`, default 0.5). At loop index `nScale` (0-based, 0 = first = coarsest):

```
scale = fScaleStep ^ (nScales - nScale - 1)   // image downsample factor
step  = 2 ^ (nScales - nScale)                // used only for the blur sigma below
sigma = 0.09 * step + 0.15                    // pre-blur before resizing/gradient
```

identical on both backends. With the shipped defaults: scale 0 runs at half resolution with
`sigma=0.51`, scale 1 (finest) at full resolution with `sigma=0.33` px — a noise-robust pre-blur
ahead of the derivative stencil (§1.3), not a negligible one. The multiplier on this sigma was
swept in 2026-09 (§2.6): it is coupled to `MeshRefineStep::StepGrow` and the two only work as a
pair — see the entry there before changing either.

Each scale re-inits images (`InitImages`, one worker per view): the shared `PrepareRefineImage`
(`SceneRefineCommon.cpp`) loads, gray-converts, Gaussian-blurs at the scale's `sigma` and resizes;
`ComputeRefineImageGradient` (§1.3) builds the derivative image both backends read; then
`PrepareRefineImageMask` builds the per-view keep-mask at the working size. `ListVertexFacesPre`
re-lists incident faces, then `SubdivideMesh` runs — one shared body on both backends since
2026-09 (`PrepareRefineMesh`, `SceneRefineCommon.h`), so the two can no longer drift. At the first
scale it projects the input mesh into every camera (`ListCameraFaces`, `ListFaceAreas`: for every
face the *tightest-pair area*, the larger over the image pairs of the smaller of the two
rasterized pixel counts, 0 for a face no pair sees; and, because those counts are pixels and read
1 on the sub-pixel faces of every dense input mesh, the analytic mean area of a stride sample of
the seen faces), and decimates the mesh (`--decimate` 0) straight to the density the refinement
wants: a mean tightest-pair area of **half `--max-face-area`** in that scale's pixels — 8 px² at
the default 16, which the finest scale's doubled resolution makes 32 px², twice the cap, so its
split lands the mesh at the cap. What arrives is a constant 70-79 % of that nominal target, the
remesh band's own equilibrium (§2.9), so the cap reads as a split threshold rather than a delivered
density. The ratio is the measured mean over the target (floor 0.002: it guards a degenerate
target only, and a floor near the ratios the usable caps ask for silently saturates the knob,
§2.9),
handed to one `Mesh::Clean` pass that decimates (halfmesh's QEM), closes holes up to 30 edges and
remeshes isotropically in a band around the mean edge the decimation left (`edgeLength` −1,
10 iterations): the remesh evens the rings out for the umbrella operator without moving the
density. With `--adaptive-face-size` (default on since 2026-09) that remesh instead grades the
density per vertex: the decimation runs alone, the decimated mesh is projected once more, and every
face states its own scale — it covers `seenArea` pixels for its world area, so the world area that
would project to the target is `area·target/seenArea`, and a vertex takes the equilateral edge of
the mean over its incident seen faces (`SeenAreasToEdgeTargets`, handed to halfmesh as
`RemeshParams::vertexSizing`). The decimation ratio is one number for the whole mesh and can only
set the total; the remesh's split and collapse are local, so the field is what moves faces from
where the cameras over-resolve the surface to where they under-resolve it. On a surface seen from a
constant distance the field comes out flat and the preparation is the uniform one — which is why
Tanks & Temples measures it as +0.0007 F1 at 1.00x faces (§2.10), and why the mechanism is aimed at
scenes whose camera-to-surface distance varies, which that benchmark does not contain. Every scale then projects again and splits 1-to-4 every face whose tightest-pair area
exceeds **twice** `--max-face-area` (`Mesh::Subdivide` tests `2*maxArea`, 32 px² at the default).
Until 2026-09 the preparation was two passes and two relative heuristics — decimate to
`max(0.1, 6·median/cap)` over every face, unseen zeros included, then remesh to 2.25x the mean
edge *after* the split — and the remesh, not the split or the decimation, set the density: on
Truck it collapsed 4x more faces than the decimation had left, uniformly in world space, and
the coarse scale iterated on 85 % of the fine mesh's faces at ~3 px² each. §2.8 measured the
alternatives; the explicit target became the default on the speed and mesh-size criteria: the
shipped one-pass composition is +0.0001 F1 at 0.85x faces and 0.83x wall on Tanks & Temples against
the previous defaults (the preparation itself at 0.67-0.86x of its previous wall: the two Clean
passes become one, and the second projection runs on a fifth of the faces), the same target with
the remesh in a second pass after the split −0.0004 at 0.79x faces and 0.85x wall. The log lines `"Mesh projected (input|prepared): …"`, `"Mesh split: …"` and
`"Mesh subdivided: %u/%u -> %u/%u vertices/faces"` are identical on both backends.

**Post-refinement decimation (on by default since 2026-09).** The face count the refinement needs (every
face under `--max-face-area` in its tightest pair) is not the face count a deliverable needs.
`--simplify-tolerance T` (px, default 0.25, 0 = off; `OPTREFINE::fSimplifyTolerance`) decimates the
refined mesh once the last scale ends, the same host-side pass on both backends
(`SimplifyMeshWithinTolerance`): every vertex gets its pixels per scene unit from the same
tightest-pair areas the split rule reads (`SeenAreasToPixelFactors`: a face covering `seenArea`
pixels over its world area resolves `sqrt(seenArea/area)`, averaged over a vertex's incident seen
faces), so `T` is in pixels of the working resolution -- the unit `--max-face-area` already uses.
Until 2026-09 this one measurement was the odd one out, taken as f/depth in each vertex's single
best view: that ignores foreshortening and occlusion, which a rasterized area carries, and it can
pick an image no refinement pair uses. Unifying it removes 19-20 % more faces at unchanged F1, and
at accuracy and completeness identical to four decimals on the EPFL scenes (§2.10). The bound
`(T/pf)²` is handed to
`Mesh::Clean` as `CleanParams::vertexMaxError`, and halfmesh's exact QEM decimation collapses an
edge only while the mean squared distance of its collapse point to the planes its merged quadric
holds stays within the smaller bound of its endpoints, running until no edge passes. The bound is
a mean plane distance, not the raw QEM sum, so a vertex that absorbed many collapses is held to
the same pixel distance as a fresh one (with the raw sum the same tolerance removed a tenth of the
vertices, normalized it removes half). Measured on the shipped configuration (§2.7): 0.25 px is
F1-neutral on all seven scenes at −32..−65 % faces, 0.5 px costs up to −0.004 (Ignatius) at
−55..−83 %, and every doubling of the tolerance roughly halves the vertex count.

**Why 0.25 px is the default.** F1 at τ cannot see a change that stays inside τ by construction, so
the default rests on the threshold-free distances the EPFL evaluator also records. In one cell
(CUDA, all three scenes against the same baseline), 0.25 px moves the accuracy mean by
−0.03 % on fountain-P11 (0.00796), −0.04 % on Herz-Jesu-P8 (0.01478) and +0.11 % on P25
(0.01467→0.01468) — the fifth decimal, inside the run-to-run band the CPU cell shows for the same
arm — with rms and p95 in the same band and no consistent sign. Completeness moves
**+0.030 / +0.044 / +0.078 %**: small, but the only quantity that moves the same way on all three
scenes and monotonically in the tolerance, so that is the cost, for a third to two thirds of the
faces and ~1-2 % wall. The tolerance is in working-resolution pixels, so at the default
`--resolution-level 0` it is *tighter* than everything above, which was measured at level 1: the
same 0.5 px removes 49 % of the vertices at level 0 against 66-72 % at level 1. What no measurement
here covers is appearance — both metrics score distance to ground-truth points, and neither
penalizes faceting on the large flat regions where a reprojection bound collapses hardest. Pass
`--simplify-tolerance 0` for the refinement's full output.

Two consequences for the numbers elsewhere in this document: §2.1 and §2.7 were measured on the
undecimated output, so the shipped default now differs from them by the ±0.0004 the 0.25 px column
records, and a bench baseline taken after this change already includes the decimation.

### 1.3 Photo-consistency energy

For every ordered image pair `(A,B)` — view-graph neighbors gathered and filtered by the shared
`SelectRefineNeighbors` (`SceneRefineCommon.cpp`: recovers `Image::neighbors` via
`Scene::SelectNeighborViews` if a mesh was handed to the refiner directly, then
`Scene::FilterNeighborViews` at fixed thresholds — min area 0.1, scale [0.2, 3.2], angle
[2.5°, 45°] — capped at `--max-views`, default 8) — `ThProcessPair`/`MeshRefineCUDA::ProcessPair`
does, per pixel of A:

1. **Warp, with masks.** Project A's depth at `(i,j)` into B; a pixel masked out of A never seeds a
   sample. Otherwise keep the sample only if the nearest-tap relative visibility test accepts
   (§1.5) **and** the same rounded B-side tap is kept by B's own mask. Invalid pixels are
   **zero-filled**, not seeded with a copy of A: CPU's `MeshRefine::ImageMeshWarp` memsets `imageAB`
   to 0 before the warp; CUDA's `kernelImageMeshWarp` writes 0 for every rejected pixel — a
   rejected pixel contributes nothing to any window sum downstream, instead of biasing the local
   statistics toward `ZNCC=1` right at occlusion boundaries the way an `imageA.copyTo(imageAB)`
   seed does.
2. **Masked local statistics, 7x7 window, shared (`Refine::HalfSize=3`, `SceneRefineCommon.h`).**
   `MeshRefine::ComputeWindowStats` (CPU: six `cv::boxFilter` passes over the masked A/B products)
   and `kernelComputeWindowStats` (CUDA: one thread per pixel, the block's 22x22 overlapping-window
   tile staged into `__shared__ sA/sB/sW` once instead of re-read 49x per thread) both reduce the
   same six masked sums — over VALID pixels only, normalized by their count `n` — through two shared
   inline functions. `Refine::WindowStatsFromSums` rejects the pixel if
   `n < Refine::MinWindowCount` (25), floors both variances at `1e-4`, and applies two rejection
   gates: `|muA-muB| > OPTREFINE::fGateMeanDiff` (default 0.4) or a variance ratio exceeding
   `OPTREFINE::fGateVarRatio` (default 8) — a specular highlight, a shadow boundary or a missed
   occlusion, not a photo-consistency measurement of the same surface.
   `Refine::ZnccAndDerivative` forms `zncc = cov/sqrt(varA*varB)`, its derivative
   `dZ = (pixA-muA)/sqrt(varA*varB) - zncc*(pixB-muB)/varB`, the reliability weight
   `conf = ZnccReliability(varA,varB) = min(varA,varB)/(min(varA,varB)+0.0015)`, and
   `dzncc = -conf*dZ*(WindowArea/n)` (exactly 1 when every pixel of the window is valid). The
   pair-direction's reliability sums `sumR += conf`, `sumRZ += conf*(1-zncc)` accumulate into
   `S = sumRZ/sumR` (§1.7) — the reliability-weighted mean of `1-ZNCC`, invariant to scene scale,
   contrast, resolution and pair count. `MinWindowCount` and the two gates are the only
   pixel-rejection machinery.
3. **Per-pixel photometric gradient.** `MeshRefine::ComputePhotometricGradient` (CPU) and
   `kernelComputeWindowStats` (CUDA, one thread per pixel, right after the pixel's own ZNCC)
   compute the face normal `N`, the camera-A ray `dA`, and
   `Nd = N.dA`; skip the pixel if `Nd > -0.1` (a one-sided grazing/back-face gate). They
   back-project to the 3D point, project into B with a Jacobian `J` (`MeshRefine::ProjectVertex`),
   sample B's precomputed image gradient there, and form

   ```
   sg = (gB . (J . dA)) * dzncc * RegularizationScale / Nd
   ```

   distributed to the face's three vertices weighted by barycentric coordinates and the face
   normal. `RegularizationScale = avgDepthA*avgDepthB / (fA*fB)` (identical formula on both
   backends) converts an image-space photometric gradient into a 3D-consistent one (§1.8). This
   magnitude term with the pair-count normalizer below is the only photometric formulation the
   pipeline computes; bounded, vote-based and plain-sum alternatives were measured and rejected
   (§4).
4. **Image gradient stencil, shared.** `gB` above comes from a per-view gradient image built once
   per scale by `ComputeRefineImageGradient` (`SceneRefineCommon.cpp`), selected by
   `OPTREFINE::nImageGradient` (default **1**, central differences `[-1,0,1]/2`; 0 = the
   noise-robust separable 3x5 `[1,2,1]^T (x) [-1,-2,0,2,1]/32`, `CreateDerivativeKernel3x5`; 2 =
   Sobel-3 `/8`; 3 = the derivative of the bilinear interpolant the warp samples). CPU builds the
   image once with `cv::filter2D` and samples it bilinearly (integer coordinates = pixel centres);
   CUDA uploads the identical host-computed image as two float textures and samples with hardware
   bilinear `tex2D`, whose non-normalized-coordinate convention places a texel's centre at `+0.5` —
   every CUDA fetch reading a CPU-convention coordinate (this sample, and the warp's color fetch in
   step 1) adds that offset explicitly so both backends sample the same point. Central differences
   are the measured winner over the wider stencils (§2.3): after the per-scale Gaussian pre-blur
   (§1.2) a wider stencil only adds blur on top of an already-blurred image.
5. **Per-vertex accumulation.** Each pixel's `sg` is added to `photoGrad[v]` for the face's three
   vertices; `photoGradNorm[v]` (`c_v`) is incremented **once per pair-direction that touched `v`,
   not per pixel**; `footprint[v]` (`Camera::GetFootprintWorld(depthA) = depthA/focalLengthA`) is
   min-reduced over every pixel/pair-direction that touched `v`, resolved from an `FLT_MAX` sentinel
   to 0 for a vertex none saw (`footprint[v] > 0` iff `photoGradNorm[v] > 0` on both backends). On
   CPU this merge happens per-pair under a lock at the end of `ThProcessPair`, so summation order —
   and the float result — is not reproducible run to run unless `--max-threads 1`. On CUDA it is
   **deterministic**: `kernelAccumulateFacePhoto` is face-parallel — one thread per face the
   reference view owns walks its clipped bounding box in fixed scan order, folding the per-pixel
   terms the window-statistics kernel left behind (weighted by the barycentrics), the pixel count
   and the footprint minimum into the face's private slots, which accumulate across the
   evaluation's pair-directions, no atomics, and counts the direction once per vertex through a
   per-vertex direction stamp (the `photoGradNorm += 1` per direction); and once per
   evaluation `kernelGatherVertexPhoto` is vertex-parallel, walking each vertex's flattened
   incident-face list (`Mesh::ListIncidentFaces` order) to sum the contribution and resolve the
   sentinel exactly like the CPU's post-loop pass. `photoGrad[v]/photoGradNorm[v]` — the pair-count
   average — is what the
   stepper and the Ceres energy mode (§1.9) both read as `g_v`; there is no other normalizer.
6. **Boundary vertices get the photometric term but zero smoothing.** Nothing in `ScoreMesh`/
   `ComputeWindowStats`/`ComputePhotometricGradient` special-cases a boundary vertex — the
   photometric pull above applies to it like any interior vertex — but
   `ComputeSmoothnessGradient1`/`2` (§1.4) zero both smoothing terms there unconditionally. Freezing
   the boundary and a rim-Laplacian treatment were both measured and rejected (§4).

### 1.4 Regularization term

`ComputeSmoothnessGradient1` (CPU) / `kernelComputeSmoothnessGradient(mode=0)` (CUDA) compute the
discrete umbrella-operator Laplacian `L(v) = mean(1-ring neighbors) - v`, zeroed at boundary
vertices. `ComputeSmoothnessGradient2` / `kernelComputeSmoothnessGradient(mode=1)` form the
"level 2" operator from Hernandez (2004, p.105) — a valence-normalized combination of `L` over the
same ring — also zeroed at boundary vertices; on both backends the valence used to weight a
neighbour is that neighbour's TRUE valence (`vertexVertices[idxVert].GetSize()` on CPU, the
uploaded `vertSizes[]` on CUDA), including a boundary neighbour's, so an interior vertex next to
the boundary does not divide by a corrupted weight.

The final per-vertex gradient combines them (CPU `ScoreMesh`, only when a caller asks for the
combination — the Ceres arm and the planar-vertex hook; the stepper reads the terms separately):

```
ratioRigidityElasticity >= 1:   photoGrad[v]/photoGradNorm[v] + smoothGrad2[v]*weightRegularity
ratioRigidityElasticity <  1:   photoGrad[v]/photoGradNorm[v] + smoothGrad2[v]*elasticity - smoothGrad1[v]*rigidity
                                 rigidity   = (1-ratio)*weightRegularity
                                 elasticity =    ratio *weightRegularity
```

`--regularity-weight` defaults to 0.2, `--rigidity-elasticity-ratio` to 0.9; both are validated at
the entry point against the stepper's explicit-flow stability bound
`weightRegularity * MeshRefineStep::StepMax <= 1` (§1.7), rather than left to fail inside the
optimizer. The ratio is not forced to 1 for a fixed fraction of a fixed iteration count — see §1.7
for the two-phase schedule.

### 1.5 Visibility test (occlusion)

CPU `MeshRefine::IsDepthSimilar` and CUDA `kernelImageMeshWarp` both round the projected point in B
to its nearest depth-map texel and accept it iff `depth > 0 && depth*1.0002f >= z` — a one-sided
occlusion test (rejects only when B's own measured depth is significantly closer than the
transformed point) that is **exactly invariant under a uniform scene rescale**, since both depths
are multiplied by the same factor. There is no scene-unit tolerance left in the test; the
multiplier is a tuned operating point and the curve is not monotone (§4).

Both backends also test the B-side keep-mask at the same rounded tap, and both range-check the
projected coordinate in **float, before** any integer conversion: a grazing projection can put the
target arbitrarily far outside the image, and converting to `int` first lets a saturated/wrapped
value slip past a `<` comparison and index the depth map out of bounds.

### 1.6 CPU/CUDA: what is shared vs what legitimately differs

Identical **by construction**, not by measurement: the shared scalar header (`SceneRefineCommon.h` —
`ZnccReliability`, `WindowStatsFromSums`, `ZnccAndDerivative`), the 7x7 window, the visibility test,
the image-gradient stencil (built once, host-side, shared by both), the rasterizer (both keep only
front faces where `EdgeFunction(p0,p1,p2) > 0` and use perspective-correct barycentric coordinates —
CUDA's two-pass `kernelProjectMesh` is itself deterministic run to run), and the stepper
(`MeshRefineStep`, one implementation). What legitimately differs:

| Aspect | CPU | CUDA |
|---|---|---|
| Camera projection precision | double (`Camera::TransformPointW2C`) | float (`MVS::CUDA::Camera`, built once per call by `MakeCUDACamera`) — measured bit-identical face maps on the `Tiny` fixture despite the precision drop |
| Rasterizer input | the faces an octree frustum cull lists per camera, rebuilt every evaluation; the first face rasterised wins an exact depth tie | the whole mesh into every view, the kernel rejecting what a view does not see (the cull cost more host time than it saved on the GPU, §1.6.1); the lower face id wins the tie — the only pixels the two face maps can differ on sit exactly on a shared edge |
| Photometric accumulation order | per-pair, under a lock, in whatever order threads complete — not bit-reproducible run to run unless `--max-threads 1` | face-parallel accumulate + vertex-parallel gather over a fixed order, no atomics — **bit-reproducible** |
| Planar-vertex removal | implemented (§1.7) | refused at the entry (a loud error, not a silent no-op) — caller falls back to CPU |
| Ceres arm (`--use-ceres`) | implemented, gated on `_USE_CERES` | refused at the entry, same fallback |
| Float reassociation | `cv::boxFilter`/`filter2D`, MSVC `/fp:precise`, no FMA | explicit-rounding intrinsics (`__fmul_rn` etc.) so nvcc cannot silently fuse an FMA the CPU wouldn't — residual disagreement ~0.1-2% per vertex at the tail, the documented approximation floor |

The two backends were brought to this state by dumping one image pair's per-vertex gradients and
per-pixel maps from each and comparing them side by side; that diagnostic found the six CUDA-only
defects (window size, `dZNCC` form, image-gradient sampling, border margins, bi-Laplacian valence
at a boundary neighbour, back-face winding) and was removed from the tree once they were closed.

#### 1.6.1 What one CUDA evaluation does, and where its time goes

`MeshRefineCUDA::ScoreMesh` makes two host round trips, one after the rasterization and one at the
end:

1. The vertex upload and the rasterization of the whole mesh into every view (`kernelProjectMesh`
   x2 per view, no host frustum cull — see the table above); the second pass also leaves, per view,
   one *owner bit* per face (a warp ballot), set iff the face wrote a pixel there.
2. The owner bits packed into dense per-view *owner lists* (`kernelCountOwners`,
   `kernelScanViewCounts`, `kernelCompactOwners`), each view's slice then counting-sorted by the
   64x8-pixel image tile the face's clipped box starts in (`kernelSortOwnersTile`). The views'
   offsets come down here — the one synchronization before the terms — and the list is kept at
   the largest total so far.
3. The face normals and both smoothness terms (`kernelComputeFaceNormal`,
   `kernelComputeSmoothnessGradient` x2), which need only the vertices.
4. Per pair-direction: the warp (`kernelImageMeshWarp`); the window statistics with the pixel's
   photometric term (`kernelComputeWindowStats`, one thread per pixel, the block's 22x22 tile
   staged in shared memory, a tile without a masked sample leaving at once); and the accumulation
   (`kernelAccumulateFacePhoto`): one thread per face the reference view owns walks the face's
   clipped box in row chunks, folds its pixels' terms weighted by the barycentrics — recomputed
   with the rasterizer's own `pixelBary`, bit for bit — into the face's private slots, which
   persist across the directions, and counts the direction once per vertex through a per-vertex
   direction stamp (`atomicExch`; an integer count, so reproducible).
5. One reduction of every direction's block partials into `S` (`kernelReduceBlockSums`), one
   per-vertex gather over the fixed incident-face order (`kernelGatherVertexPhoto`), and ONE
   download of all per-vertex terms into a pinned host buffer the stepper reads in place.

Steps 4 and 5 are thousands of launches whose every argument is a constant of the scale (the
accumulation reads its view's slice offsets on the device and strides over a grid sized at the
recording), so they are recorded once per scale — per parity when the pairs alternate — into a CUDA
graph by stream capture and replayed by one `cudaGraphLaunch` per evaluation (Ignatius: 533
launches and one graph per evaluation against 11,974 issued one by one, at ~10 us each under
WDDM, as much as the coarse scale's GPU time). The graphs are dropped when a buffer they
reference moves (a new scale, a grown list); if a recording fails the directions are issued one
by one. The rasterization, compaction and memsets stay eager ahead of the graph, the terms
download behind it. Between two evaluations the host does only the stepper and the next vertex
upload; the next scale's images are prepared on a thread while the current scale runs
(`PrefetchImages`, its OpenMP team capped at half the cores so it does not starve the thread
feeding the GPU), so a scale switch pays only the upload.

Why it is shaped this way:

- **No atomics, fixed orders.** Float addition is not associative, and an `atomicAdd` scatter
  gives a different per-vertex gradient on every run. Every float sum has one writer and a fixed
  order: a face reduces its own pixels in raster order, the slots accumulate in launch order, the
  gather walks `Mesh::ListIncidentFaces`' order, `S` is folded from per-block partials in slot
  order. The list order and the tile sort never touch the result.
- **Owner bits and lists.** In a scene of hundreds of views any one view sees a fraction of the
  mesh, so a face-parallel kernel over the whole mesh per direction is O(faces x views) of early
  exits that leave the warps nearly empty (Nsight Compute on the accumulation: 4.4 active lanes
  per warp-instruction, two thirds of the cycles waiting on scattered loads). One thread per owner
  face, in tile order so that a warp's 32 boxes share cache lines (L1 hit rate 52 -> 78 %), and
  the box rows loaded in chunks so that a chunk's loads are in flight together, took the
  accumulation from 139 to 35 us per direction at scale 1 on Ignatius.
- **No barycentric map.** Barycentrics and depth are recomputed from the face's projection where
  they are needed, in float like the CPU's `BaryMap`, instead of a half-precision map per pixel of
  every view: 21 bytes per pixel per view (float image 4, its two gradient textures 8, depth 4,
  face 4, a keep-mask byte when masks are used) instead of 29.
- **Projected face areas on the GPU.** The preparation's `ListFaceAreas` counts pixels per face
  per view (`kernelFaceHistogram`, integer atomics, exact) and folds each pair's min into the max
  over the pairs (`kernelReduceFaceAreasPair`), the pairs walked grouped by their first view; only
  the reduced `uint16_t` areas come down, truncated exactly like the host's counters, and a Debug
  build recomputes the host path and asserts equality on every face. A Debug build also checks
  the rasterizer's payloads against the keys, and the owner lists against the bits, on every
  evaluation.

Measured with Nsight Systems on Ignatius at level 1 (263 views of 960x540, 1,430 pairs = 2,860
pair-directions per evaluation, 125k/281k faces at the two scales), the first CUDA build of this
branch against the shipped one:

| per evaluation, Ignatius level 1 | first build | shipped |
|---|---|---|
| GPU kernels, scale 0 / scale 1 | 276 / 638 ms | 102 / 246 ms |
| host gap to the next evaluation, scale 0 / scale 1 | 70 / 135 ms | 2-6 / 3-7 ms |
| `kernelAccumulateFacePhoto`, scale 0 / scale 1 | 55 / 139 us | 15 / 35 us |
| `kernelComputeWindowStats`, scale 0 / scale 1 | 10 / 24 us | 11 / 25 us |
| D2H copies per run | 1,351 / 1.68 GB | 41 / 188 MB |
| scale switch (host image preparation + upload) | 1.17 s + 0.47 s | 0.01 s + 0.44 s |
| device memory per view | 29 B/px | 21 B/px |

Walls back to back in one machine state: Ignatius 72.5 s -> 34.5 s, Truck, Barn and Meetingroom
2.1-2.7x (§2.2); the owner lists and the graph then took another 8-15 % off the four (Ignatius
35.3 -> 32.3 s), and the host-side mesh preparation shared with the CPU path is now most of what
is left (Barn: 38 s of 81 s). On Herz-Jesu-P8 (8 views, 54 pair-directions), where none of the
many-view machinery matters, the refinement loop takes 1.1 s for 50 evaluations and the wall
6.9 s, 5.4 s of it that preparation: the end-to-end CPU/CUDA ratio on small scenes stays near 3x
while the loop alone runs about 15x faster than the CPU's 0.33 s per evaluation. What is left of
a scale-1 evaluation on Ignatius is 246 ms of kernels (accumulation 42 %, window statistics
29 %, rasterization 19 %, warp 8 %), ~24 ms of host stepper and ~10 ms of transfers, memsets and
graph launch.

Measured and rejected, all on Ignatius at level 1: a host-side frustum cull shortlisting the faces
per view (an octree over the mesh rebuilt for every evaluation cost more host time than the
rasterizer's early exits cost on the GPU, and the per-view face lists were uploads, allocations
and frees the GPU waited on); eight lanes sharing a face in the accumulation, each taking every
eighth pixel of the box with a fixed shuffle tree over the partials (49 / 88 us against 26 / 64
us at the time: on the small boxes most lanes idle, and the extra threads cost more than the
divergence they remove); a launch bound forcing the accumulation to 64 registers from 71 (spills,
2 ms slower per evaluation); 128x16 tiles for the owner sort (1.4-2.6 % slower than 64x8); a
per-reference-view barycentric scratch (unnecessary once the lists were dense); and a per-pair
octree or BVH preselection of faces, which would only duplicate what the rasterizer's owner bits
already give exactly, occlusion included, at 1-2 ms per evaluation.

**Device memory is the next large-scene limit.** At 21 bytes per pixel per view, Ignatius at
level 0 (263 views of 1920x1080 at the finest scale) needs about 11 GB and does not fit a 12 GB
RTX 4070: the driver pages and a scale-1 evaluation goes from 0.3 s to minutes. Courthouse (1,106
images) needs the same at level 1. Beyond that point the design has to stream views — a resident
working set of views, pinned host copies, the pair-directions ordered by view, a view
re-rasterized when it comes back — which is a change of memory architecture, not of kernels.

### 1.7 Optimization schedule

Both backends drive the same shared stepper, `MeshRefineStep` (`SceneRefineCommon.h/.cpp`). The
stepper works in **pixels** (through each vertex's own footprint, §1.3) and in **ZNCC** (`S`,
§1.3), so its trajectory does not depend on scene scale, image resolution or pair count (§1.8).

**Per-evaluation update**, `rho` the phase's rigidity/elasticity ratio, `w = --regularity-weight`:

```
gamma_v = |g_v| / s_v                  g_v = photoGrad_v / photoGradNorm_v   (c_v >= 2 only)
m       = median gamma_v over vertices with c_v >= 2    (computed ONCE per scale, then held)
P_v     = g_v / (Kappa * m)            zero if c_v < 2 or m == 0
R_v     = rho * bilap_v - (1-rho) * lap_v
D_v     = -eta * (P_v + w * R_v)
delta_v = |D_v| / s_v                  the per-vertex step actually applied, in px
```

`m` is fixed at the scale's first evaluation: recomputing it every iteration would renormalize the
median vertex back to the same step every time and defeat the stop rule. It is a GLOBAL conversion
factor, not a per-vertex one — the median seen vertex moves `eta/Kappa` px at the first iteration
and every other vertex moves in **proportion** to its own gradient. A per-vertex
normalize-and-clamp variant measured a 0.0215 mean F1 regression (§4) because clamping flattens the
gradient distribution; the header documents why the normalization must stay global.

**Constants** (`MeshRefineStep`, pixel/ZNCC quantities, deliberately not CLI-exposed):
`StepInit = 0.5` px (`eta` at the start of every scale; opening the later scales at twice the `eta` the previous one ended with, which skips the fine scale's 3-4 opening rejections, was +0.0011 F1 on the old preparation and −0.008 on Ignatius with the shipped one, §2.8 and #52), `StepMax = 1` px (`eta_max`),
`StepGrow = 1.05`, `StepShrink = 0.5`, `StepStop = 0.05` px (median-step-at-full-stride convergence
floor), `ProgressTol = 1e-3` (relative `S` decrease counted as stalled), `Kappa = 2`,
`Patience = 3` (consecutive stalled iterations that end the scale), `MaxRejects = 4` (consecutive
rejections that end the scale), `MinIters = 3` (no stop rule before this many ACCEPTED
iterations), `MaxIters = 45` (the coarsest scale's evaluation budget, below). `ProgressTol` and
`Patience` were swept and sit on a flat optimum (§2.4); every other constant here was swept in
2026-09 (§2.6). Two results from that sweep matter when reading this list: `StepGrow` is COUPLED to
the pre-blur sigma of §1.2 and the two may only be changed together, and `StepMax` never binds —
`eta` never exceeds 0.550 px in any trace on any scene, so the cap is documentation, not a rule.

**Accept/reject.** An evaluation whose `S` is worse than the last accepted `S` is REJECTed: every
vertex moves back to exactly `v_prev + stepPrev/2` (undoing half the offending step),
`eta *= StepShrink`, and the scale STOPs after 4 consecutive rejections. An accepted evaluation
becomes the new reference, resets the reject streak, grows `eta = min(eta*StepGrow, StepMax)`, and the
scale STOPs once `numAccepted >= MinIters` and `Patience` consecutive iterations failed to improve
`S` by `ProgressTol`, or once the median per-vertex step **at a full stride**
(`medianPx * StepMax/eta`, not the step just taken — an `eta` ratcheted down by repeated
accept/reject cycles would otherwise report false convergence) drops below `StepStop`.

**Per-scale two-phase schedule.** `eta` starts every scale at `StepInit`. The per-scale evaluation
**cap** is `MeshRefineStep::Budget(nScale) = max(MaxIters/(nScale+1), 8)` (`nScale` 0-based,
coarsest first — the coarse scale gets the larger cap); it is a safety net, not the operating stop:
raising `MaxIters` from 45 to 1000 changes no result (§2.4), which is why the `--gradient-step`
option that used to set it and the initial step no longer exists.
**Phase A** runs up to `cap` evaluations at the caller's `--rigidity-elasticity-ratio` (default
0.9), with the CPU-only planar-vertex hook eligible from the 4th accepted evaluation onward, every
3rd accepted evaluation thereafter, provided more than 5 evaluations remain in the phase's budget.
**Phase B** runs a fresh, smaller budget `capB = max(3, 3*numAcceptedInPhaseA/7)` — a 70/30 split
taken against phase A's ACCEPTED count rather than its raw budget, since a rejected evaluation buys
no convergence — at `rho = 1` (pure elasticity) with the planar hook off; `eta` and the
accepted-`S` references carry over from phase A unchanged, only `MeshRefineStep::BeginSecondPhase()`
runs between phases (it hands back that budget and resets the stall counter; the reject streak does
**not** reset — resetting it too measured −0.0048 mean F1, §4). Either phase's loop exits early on
`MeshRefineStep::STOP`.
Phase B's budget is the one part of the schedule that is not convergence-driven, and deliberately
so: letting it run under phase A's stop rules over-smooths (§2.4).

**Planar-vertex removal (CPU only, `--planar-vertex-ratio`, default 0 = disabled).** When eligible
and the evaluation was APPLYed, every vertex whose combined-gradient magnitude and smoothing
residual both fall below `fThPlanarVertex * footprint[v] * medianViewFocalLength` is removed via
`Mesh::RemoveVerticesAndFill` — **after** the step was applied, since removal permutes the vertex
indexing (swap-with-last) and everything the stepper indexes by vertex must already have been
consumed that evaluation. The threshold is a fraction of the vertex's own depth (`footprint[v]`
times the median focal length of the views scoring this scale), not a whole-image average depth.
`MeshRefineStep::TopologyChanged()` runs after a removal so the next evaluation is not rejected
against an `S` measured on a different vertex set. CUDA refuses `--planar-vertex-ratio > 0` at the
entry rather than silently skipping it. One evaluation removes only vertices whose one-rings are
pairwise disjoint and do not touch the mesh's open boundary: `Mesh::RemoveVerticesAndFill` spans a
hole only when its boundary is one simple loop that does not meet an existing boundary loop, so a
removed planar patch (or two holes touching at a shared ring vertex) would otherwise stay open and
leave the next scale's `Mesh::Subdivide` a non-manifold mesh. With the rule every hole is one
fan-filled ring (measured on `Tiny`: exactly 2 faces lost per removed vertex, Euler characteristic
and boundary-edge count unchanged, no assertion in Debug at ratio 1e-3, which removes 30 % of the
vertices over the two scales), and the rest of the patch goes in the following evaluations.

### 1.8 Scale invariance

The refinement stage is scale-invariant by construction. The visibility test is exactly invariant
(§1.5). The stepper measures every step in pixels through the per-vertex footprint (`s_v`, §1.3,
§1.7) and judges convergence on `S` (dimensionless, [0,2]), so a scene scaled by 100x produces the
identical sequence of accept/reject decisions and the identical `eta` trajectory.
`RegularizationScale = avgDepthA*avgDepthB / (fA*fB)` (§1.3) is not itself scale-invariant and is
not meant to be: it is the paper's homogenization term, converting an image-space (pixel)
photometric gradient into a 3D-consistent one, and it is expected to scale with the scene — that is
what makes the photometric and regularization gradients commensurate in scene units before the
stepper converts them back to pixels.

What is **not** exactly reproducible under rescaling is the mesh preparation, and it bounds what a
scale test can assert. Measured on `Tiny` L0: on the CPU the identity control itself fails an
exact-iteration-match criterion — thread-order noise in the accumulation flips the stepper's
accept/stall decisions, so identical inputs take 20-37 evaluations on the coarse scale and land
0.0007 apart (mean symmetric distance, 0.8 % of an edge). On CUDA two unit runs are identical, and
the ×100 and ×0.01 scenes rescaled back land 0.0034 from the unit result (4 % of the mean edge,
3e-4 of the bounding-box diagonal): the decimation is identical at every scale (18969/37335), but
the isotropic remesh resolves float ties differently under rescaled coordinates (4715 / 4758 vs
4721 vertices; every threshold inside it is relative), and the refinement then follows a different
trajectory. That residual is a property of float arithmetic in the mesh preparation, not of a
scene-unit constant in the refinement.

### 1.9 Ceres arm (opt-in, CPU only)

`--use-ceres` selects a first-order Ceres solve instead of the stepper, gated on `_USE_CERES`
(`OpenMVS_USE_CERES`, ON by default — Ceres is a mandatory SFM dependency). It is a research and
reference arm, not a default: it is slower and no better (§2.5). `MeshRefine` runs in **energy
mode**: `ScoreMesh` returns the exact energy

```
E = Sum_pairs RegScale_p * Sum_pixels r*(1-ZNCC)  +  w * (1/2) * Sum_{v interior} ||L(v)||^2
```

and fills its exact gradient — the raw per-vertex photometric sum (no pair-count division, since the
gradient of a sum is not the gradient of a per-vertex average) plus `w * L^T L * v`
(`ComputeSmoothnessGradientLtL`, the transpose of the umbrella operator, dividing each contribution
by the *neighbour's* valence — not the Hernandez level-2 operator the stepper uses). The
photometric half's exact per-pixel derivative (`ComputeWindowStats(..., bExactDerivative=true)`)
replaces the pointwise `dzncc` with the derivative of the **whole window sum**,
`dE/dB_p = -(A_p*S1 - S2 - B_p*S3 + S4) + (B_p*S5 - S6)` from six extra box-filtered sums (the
last two carry the reliability weight's own dependence on the warped window's variance — about a
fifth of the gradient where `1-ZNCC` is 0.3, nothing on a perfect match), and a rejected-but-still-
summed pixel stays in the mask (not dropped) because its value still enters its accepted
neighbours' window sums. A finite-difference gate (`Scene::RefineMeshEnergyProbe`,
`MeshRefineEnergyGradientTest`) validates this pair against a 5 % bar, probing both from the
fixture and from a surface displaced by a 2.5 px bump so `1-ZNCC` is not near 0: photometric
1.79 / 2.38 % perturbed (0.52 / 0.74 % unperturbed), smoothness 0.000 / 0.005 %. The same gate
established that the derivative consistent with this energy is the derivative of the *bilinear
interpolant* the warp samples (`MeshRefine::BilinearGradient`), not any precomputed stencil image —
the stencils mismatch it by 21-109 % — so the energy mode always uses the bilinear derivative
regardless of `OPTREFINE::nImageGradient`.

`ceres::MeshProblem` wraps this as a `FirstOrderFunction`, solved with `GradientProblemSolver` at
the Ceres 2.2 defaults for the direction and the line search: L-BFGS rank 20, Wolfe with cubic
interpolation (mandatory with L-BFGS) and 20 step-size iterations, no Oren-Luenberger scaling of
the initial inverse Hessian — Ceres documents that scaling as harmful where the sensitivity to
different parameters varies widely, which a vertex seen by twenty pair-directions next to one
carrying only the smoothness term is. Those defaults are kept **after** a sweep of every direction
and line search the solver offers on this energy, plus the rank, the scaling, the iteration cap and
the function tolerance, over three ground-truth scenes: no configuration wins on more than one
scene, and the within-family differences are the size of the arm's own run-to-run spread (§2.5).
`max_num_iterations = 100` per scale is a safety net that never binds (the longest observed scale
is 88); `function_tolerance = 1e-4` is the actual stop;
`gradient_tolerance = parameter_tolerance = 0` because both would be absolute thresholds on
scene-scaled quantities.

`Evaluate` returns false for a surface the energy cannot score (no pair-direction contributed a
pixel) or a non-finite energy, which makes the line search contract instead of ranking a scoreless
surface best; it applies every probe to the mesh directly (the solver's parameter block is only ever
written by Ceres) and keeps the lowest-cost surface any evaluation reached, which is what a solve
ending in FAILURE (a numerical failure of the line search, which this only-piecewise-smooth energy
can produce) leaves behind, so one scale's failure does not abort the whole refinement. The arm
refuses `--alternate-pair 1` (its energy would change every iteration) and `--planar-vertex-ratio
> 0` (the parameter count is fixed for the whole solve), forces `--rigidity-elasticity-ratio` to 1
(pure thin-plate), and is exempt from the stepper's `w <= 1` stability cap (a line search has no
explicit-flow bound). The parameters are the vertex coordinates in scene units: a pixel-unit
parameterization measured worse, because Ceres's first line search moves the largest-gradient
coordinate by one parameter unit — one pixel for one outlier vertex, next to nothing for the bulk —
and the first curvature pair L-BFGS builds from that is noise.

Two normalizations, fixed at every scale's calibration evaluation, are unconditional in the arm and
are worth +0.015 Truck / +0.038 Ignatius over the raw energy: each pair-direction is weighted by its
scale-start reliability sum over its current one (the raw photometric sum runs over a
mesh-dependent pixel set, so a line search could otherwise lower it by dropping scored pixels), and
the photometric term is divided by `kappa` times the scale-start median of `|g_v|/s_v` (its raw
gradient is `c_v` times stronger than the stepper's normalized direction at the same `w`, so `w`
keeps its stepper meaning). A calibration line per scale logs the median `|g|/s`, the mean
pair-directions per seen vertex and the energy scale; the per-iteration line logs energy, accepted
step, `S` and the reliability sum, so a run shows whether the score improves and whether the scored
domain shrinks.

---

## 2. Measured results

Every number below comes from the harness of §3: one run per cell unless stated, F1 against ground
truth at the scene's tolerance, refinement compared against the coarse mesh it was given and
against the previous `develop` behaviour on the identical frozen input.

### 2.1 Shipped configuration against the previous behaviour

Defaults only: two scales, the bold driver from its 0.5 px initial step, regularity
0.2, rigidity/elasticity ratio 0.9, magnitude photometric term with the pair-count normalizer,
central-difference stencil, masked 7x7 window statistics with the 0.4 / 8 rejection gates,
nearest-tap relative visibility. `develop` is commit `c410c9d4`, CPU, same inputs, same evaluator.

| scene | τ | coarse input | develop | **shipped CPU** | **Δ vs develop** | shipped CUDA | evals CPU/CUDA | wall CPU s | wall CUDA s |
|---|---|---|---|---|---|---|---|---|---|
| Ignatius | 0.003 | 0.7427 | 0.6489 | **0.7734** | **+0.1245** | 0.7735 | 28 / 23 | 363 → 217 | 45 |
| Truck | 0.005 | 0.6606 | 0.6235 | **0.6667** | **+0.0432** | 0.6666 | 14 / 14 | 407 → 197 | 56 |
| Barn | 0.01 | 0.6310 | 0.6531 | **0.6663** | **+0.0132** | 0.6673 | 31 / 34 | 798 → 508 | 161 |
| Meetingroom | 0.01 | 0.4026 | 0.4033 | **0.4105** | **+0.0072** | 0.4110 | 34 / 31 | 573 → 515 | 78 |
| **Tanks & Temples mean** | | | | | **+0.0470** | | | total **0.67x** | |
| fountain-P11 | 0.005 | 0.3338 | 0.3197 | **0.3431** | **+0.0234** | 0.3429 | 57 / 60 | 41 → 39 | 13 |
| Herz-Jesu-P8 | 0.01 | 0.4743 | 0.4231 | **0.4675** | **+0.0444** | 0.4676 | 16 / 16 | 28 → 12 | 6 |

Every scene improves on both backends. Two of them change sign: with the previous defaults,
refinement **lost** 0.094 against its own coarse input on Ignatius and 0.037 on Truck, and it now
ends above the input on both (0.7734 vs 0.7427, 0.6667 vs 0.6606). The CPU is 1.5-2x faster than
before because the stepper stops on convergence instead of running a fixed 67 evaluations.

This table predates the 2026-09 default sweep (§2.6), which moved the pre-blur sigma and
`StepGrow` and adds a further +0.0031 on Tanks & Temples; the comparison against `develop` it makes
is therefore conservative by that much. It is not re-measured here because the CPU cells behind it
cost 363-798 s per scene and the conclusion only strengthens — §2.6 carries the shipped numbers.

Two scenes still end below their coarse input and bound what this stage can claim. On the EPFL
scenes the coarse input is far better than anything the refinement produces (fountain 0.3338 input
vs 0.3431 refined is a gain, but Herz-Jesu-P8's 0.4743 input vs 0.4675 refined is still a net
loss), because the `--decimate 0` auto-decimation removes 12-15x of the faces before any
photo-consistency step runs. That is a mesh-density limit, not a convergence limit (§5.1).

### 2.2 The two backends produce the same surface

Same binary, same protocol, machine otherwise idle:

| scene | CPU | CUDA | CUDA − CPU | evaluations CPU/CUDA | speedup | host peak RSS |
|---|---|---|---|---|---|---|
| Ignatius | 0.7734 | 0.7735 | −0.0001 | 28 / 23 | 4.8x | 0.50x |
| Truck | 0.6667 | 0.6666 | +0.0001 | 14 / 14 | 3.5x | 0.81x |
| Barn | 0.6663 | 0.6673 | −0.0010 | 31 / 34 | 3.2x | 0.87x |
| Meetingroom | 0.4105 | 0.4110 | −0.0005 | 34 / 31 | 6.6x | 0.41x |
| fountain-P11 | 0.3431 | 0.3429 | +0.0002 | 57 / 60 | 3.0x | 0.43x |
| Herz-Jesu-P8 | 0.4675 | 0.4676 | −0.0001 | 16 / 16 | 1.9x | 0.33x |

The backends agree to 0.001 everywhere and take the same number of evaluations to get there — they
make the same accept/reject decisions on the same surface. Host peak RSS drops because the maps and
images live on the device. The residual ±0.001 is the documented floating-point difference (§5.6),
not a disagreement worth chasing.

The table above was measured before `--adaptive-face-size` and `--simplify-tolerance` became
defaults, at an operating point that stopped much earlier (16 evaluations on Herz-Jesu-P8). Re-run
on the shipped defaults the agreement is a little wider, because the run is longer and the two
backends get more chances to split a marginal accept/reject:

| scene | CPU | CUDA | CPU − CUDA | evaluations CPU/CUDA | wall CPU / CUDA | speedup |
|---|---|---|---|---|---|---|
| Herz-Jesu-P8 | 0.45111 | 0.45110 | +0.0000 | 57 / 51 | 25.5 s / 9.0 s | 2.8x |
| fountain-P11 | 0.34520 | 0.34655 | −0.0014 | 54 / 59 | 35.9 s / 11.4 s | 3.2x |

The evaluation counts no longer match exactly, so the surfaces are compared at slightly different
stopping points rather than at the same one; F1 tracks the iteration count closely enough (§2.4)
that this accounts for the gap on its own. Before the GPU rework of §1.6.1 the CUDA build
reproduced §2.10's shipped combination — the sizing field plus the tightest-pair decimation,
measured there as an arm — to the fifth decimal (0.45312 / 0.34618); the rework moved it inside
the band through the rasterizer's tie-break, the summation order and the float barycentrics the
CPU also uses, so the gap is the CPU's own trajectory and not a regression. Walls are only
comparable within one machine state (the CPU column was measured in a different session); the
end-to-end speedup on these small scenes is capped by the host-side mesh preparation both
backends share (5.3 s of the 8.4 s on Herz-Jesu-P8), while the refinement loop itself runs about
15x faster on the GPU (§1.6.1).

On the many-view scenes, against the CUDA build before §1.6.1's rework (the same source
otherwise), each pair run in the same machine state:

| scene | previous CUDA build: F1 / evaluations / wall | this build: F1 / evaluations / wall | wall |
|---|---|---|---|
| Ignatius | 0.7793 / 36 / 75.3 s | 0.7801 / 36 / 34.9 s | 2.2x |
| Truck | 0.6660 / 16 / 85.2 s | 0.6660 / 16 / 39.9 s | 2.1x |
| Barn | 0.6768 / 34 / 241.6 s | 0.6757 / 33 / 89.2 s | 2.7x |
| Meetingroom | 0.4135 / 26 / 110.7 s | 0.4134 / 35 / 79.9 s | 1.4x (2.4x per scale-1 evaluation: 3.6 s to 1.5 s) |
| Herz-Jesu-P8, back to back, no evaluator | 51 / 10.0 s | 51 / 8.4 s | 1.2x |
| fountain-P11, back to back, no evaluator | 50 / 16.8 s | 59 / 14.4 s | 1.2x |

F1 moves within the run-to-run band of a changed trajectory (the float barycentrics and the
summation order change the bits, not the surface); the walls fall where the views are many and
stay where the host-side preparation dominates (the EPFL pairs).

**CUDA is bit-reproducible.** Three identical runs on Ignatius produce F1 0.7730 / 0.7730 / 0.7730,
413,865 faces and 23 evaluations each — a run-to-run spread of exactly 0. The CPU is not
(§5.3): its noise floor is 0.0001-0.0009 depending on the scene.

### 2.3 Where the gain comes from

Each component measured on its own, in the order it landed, mean over the four Tanks & Temples
scenes:

| component | mean ΔF1 | worst scene | notes |
|---|---|---|---|
| Pixel-unit bold-driver stepper (replaces the fixed 67-evaluation schedule) | **+0.0130** | +0.0016 (Barn) | 6 of 6 scenes positive; Truck +0.0351 at 0.37x the wall |
| Nearest-tap relative visibility test | **+0.0138** | −0.0021 (Barn) | Ignatius +0.0538 |
| Central-difference image derivative | **+0.0086** | +0.0014 (Truck) | 6 of 6 positive at identical wall |
| Masked window statistics + the two rejection gates | **+0.0028** | −0.0004 (Truck) | Ignatius +0.0098; CPU also 0.70x the wall |
| Parity and crash fixes | — | — | what made the CUDA backend usable at all |

The parity work closed six CUDA-only defects (window size, `dZNCC` form, image-gradient sampling,
border margins, bi-Laplacian valence at a boundary neighbour, back-face winding) plus an
out-of-bounds depth-map read reachable on both backends from a grazing projection, and replaced
float atomics with a face-parallel accumulate and an ordered gather, which is what makes the CUDA
result bit-reproducible.

### 2.4 Step size and iteration count are selected automatically

The stepper chooses both the step and the number of iterations from the trajectory itself; the
question is whether any published rule does it better. Two sweeps on the small ground-truth scenes
answer it, both against the shipped driver.

**Nothing beats the bold driver as the step rule.** Fourteen arms, fountain-P11 + Herz-Jesu-P8,
CUDA, F1 against the shipped configuration (0.3435 / 0.4677); the acceptance gate is a mean
≥ +0.002 with no scene below −0.002:

| arm | fountain | Herz-Jesu-P8 | mean | wall |
|---|---|---|---|---|
| Barzilai-Borwein, global secant step | −0.0011 | +0.0027 | **+0.0008** | 0.89x |
| Barzilai-Borwein, fine scale only | −0.0014 | +0.0017 | +0.0002 | 0.87x |
| Trust ratio, fine scale only | −0.0002 | +0.0003 | +0.0001 | 0.70x |
| Trust ratio | −0.0007 | +0.0003 | −0.0002 | 0.88x |
| Trust ratio + predictive/restore stop rules | +0.0001 | −0.0049 | −0.0024 | 0.72x |
| Predictive and restore-best stop rules alone | 0.0000 | −0.0053 | −0.0027 | 0.75x |
| Momentum (β 0.5) / with self-referenced clip, fine scale | +0.0015 / +0.0014 | −0.0096 / −0.0100 | −0.0041 / −0.0043 | 0.92x |
| Momentum / with clip, all scales | +0.0010 / +0.0010 | −0.0196 / −0.0203 | −0.0093 / −0.0097 | 0.93x |
| Fixed step, never reject (control) | −0.0365 | −0.1416 | −0.0891 | 1.20x |

Nothing passes. The best arm is +0.0008 against a +0.002 gate, and the control confirms the
accept/reject machinery is carrying the result: without it the refinement collapses. A structural
property of the stepper explains most of the family: it modulates the step on the ACCEPT path only
and a REJECT returns early, so a trust-ratio arm is exactly the bold driver until two consecutive
accepts and a momentum arm until three. Over roughly 440 recorded per-scale traces the longest
consecutive-accept run is 1 in 38 % of them, so on many scenes these arms never diverge from the
driver they are meant to replace.

That objection is answered on the scene that does exercise them. Herz-Jesu-P25 runs 46 evaluations
with consecutive-accept runs of 8 and 7, against Herz-Jesu-P8's 1 and 3, so every history-based arm
is fully active there — and both survivors lose on it:

| arm | fountain-P11 | Herz-Jesu-P8 | Herz-Jesu-P25 | mean | wall |
|---|---|---|---|---|---|
| Barzilai-Borwein | −0.0004 | +0.0028 | **−0.0015** | +0.0003 | 0.93x |
| Trust ratio | +0.0013 | +0.0014 | **−0.0032** | −0.0002 | 1.08x |

The arms, the extra stop rules and the fixed-step control are therefore removed from the tree;
only the bold driver ships.

**Nothing beats the shipped stop rules either, and the one budget-limited phase must stay that
way.** Same two scenes, iterations per scale in brackets:

| arm | fountain | Herz-Jesu-P8 | mean | wall |
|---|---|---|---|---|
| Patience 2 | +0.0001 [18, 26] | 0.0000 [8, 8] | +0.0001 | 0.93x |
| **shipped** (Patience 3, ProgressTol 1e-3) | — [36, 28] | — [8, 8] | — | 1.00x |
| Patience 5 | −0.0004 [43, 28] | 0.0000 | −0.0002 | 0.98x |
| ProgressTol 1e-4 | −0.0007 [60, 28] | 0.0000 | −0.0003 | 1.01x |
| ProgressTol 1e-2 | −0.0014 [16, 17] | 0.0000 | −0.0007 | 0.91x |
| Phase B under phase A's stop rules | +0.0002 [36, 39] | **−0.0108** [28, 27] | −0.0053 | 1.16x |
| … and ProgressTol 1e-4 | +0.0004 [67, 44] | **−0.0151** [50, 44] | −0.0073 | 1.30x |

Three conclusions. The per-scale evaluation cap is **not** binding — raising `MaxIters` from
45 to 100, 200 and 1000 changes nothing (all within ±0.0006) — so scales already end on the rules,
not on a budget. The stall thresholds sit on a flat optimum: a stricter tolerance runs fountain's
coarse scale 67 % longer to a *lower* score and a lower F1, a looser one halves the iterations at
parity. And phase B is the exception that proves the design: letting the pure-elasticity phase run
to convergence instead of its `3·nA/7` budget over-smooths Herz-Jesu-P8 by 0.011 (27 elasticity
iterations instead of 3). That budget is doing regularization work and stays.

### 2.5 The Ceres arm against the stepper

The opt-in Ceres solve (§1.9) minimizes the exact energy with a proper line search, so it is the
reference against which the hand-written stepper is judged. ΔF1 against the stepper in the same
cell, CPU, all three ground-truth scenes for the configurations that survived the first two, one
run each unless the entry says otherwise (the arm's own run-to-run spread is 0.0008 on fountain-P11
and 0.0013 on Herz-Jesu-P8, against the stepper's 0.0000 and 0.0009):

| Ceres configuration | fountain-P11 | Herz-Jesu-P8 | Herz-Jesu-P25 | wall |
|---|---|---|---|---|
| L-BFGS rank 20, Wolfe, tolerance 1e-4 (Ceres 2.2 defaults, n=3) | +0.0006 | −0.0278 | −0.0180 | 2.3-6.7x |
| Fletcher-Reeves nonlinear CG, Wolfe (n=3) | +0.0011 | **−0.0201** | **−0.0237** | 1.3-3.6x |
| function tolerance 1e-3 (n=2) | +0.0019 | −0.0267 | — | 1.3-3.6x |
| Polak-Ribière + Armijo (n=2) | +0.0001 | −0.0153 | — | 1.7-5.4x |
| L-BFGS rank 5 / rank 50 | +0.0018 / +0.0011 | −0.0256 / −0.0292 | — | 2.0-6.1x |
| Oren-Luenberger scaling on | +0.0013 | −0.0252 | — | 2.3-4.6x |
| function tolerance 1e-5 / iteration cap 300 | +0.0016 / +0.0016 | −0.0279 / −0.0268 | — | 2.4-5.8x |
| Polak-Ribière / Hestenes-Stiefel nonlinear CG | −0.0007 / −0.0008 | −0.0179 / −0.0219 | — | 1.6-6.0x |
| steepest descent / with Armijo | −0.0057 / −0.0045 | −0.0108 / −0.0092 | — | 1.7-6.4x |
| regularity weight 0.5 / 1.0 | −0.0005 / −0.0022 | −0.0228 / −0.0235 | — | 2.1-5.5x |

**The stepper wins, and no solver configuration changes that.** On fountain-P11 the arm is within
±0.002 of the stepper; on both Herz-Jesu scenes every configuration is 0.009 to 0.029 below it, at
1.3 to 6.7 times the wall. The arm stays opt-in and keeps the Ceres 2.2 defaults.

**No configuration survives a third scene, which is why the sweep needed one.** Fletcher-Reeves
looked like a clear winner after two scenes — equal-best on fountain-P11 and 0.0077 ahead of
L-BFGS on Herz-Jesu-P8, both above the spread — and then lost 0.0057 to L-BFGS on Herz-Jesu-P25.
Ranked over all three, the two are tied (mean −0.0147 against −0.0152). The same is true of the
rest: nothing separates the L-BFGS rank, the Oren-Luenberger scaling, a tolerance of 1e-5 or a
raised iteration cap from the defaults by more than the arm's spread. Only two configurations are
robustly worse — steepest descent (−0.0057 on fountain) and Polak-Ribière on its own, whose
fountain coarse scale stalls after 9 iterations. All 53 runs were numerically clean: no
line-search failure, and no scale ever reached the iteration cap, so `function_tolerance` is what
ends every scale.

**Raising the regularity weight for the arm does not help**, which refutes the obvious reading of
the Herz-Jesu gap. The arm's energy has no phase-B elasticity pass and forces a pure thin-plate
regularizer, so an under-regularized energy was the natural suspect; `w` at 0.5 and 1.0 is worse on
both scenes tried. `w` stays at its 0.2 default.

**Why the arm loses where it loses.** On every scene Ceres reaches a *lower* score than the
stepper — final `S` 0.064 against 0.067 on fountain-P11, 0.085 against 0.099 on Herz-Jesu-P8 — and
lowers the energy 12-16 % per scale. It is minimizing the stated energy better and scoring worse
against ground truth. On the Herz-Jesu scenes refinement is a net loss whatever the optimizer (the
coarse inputs score 0.4743 and 0.6523, above every refined result), so within the family the
configurations that change the mesh least tend to rank highest, and the ranking is partly a ranking
of restraint. The practical conclusion is the one the stepper already encodes: the stopping rule
and the step cap are doing quality work that minimizing this energy further does not buy.

---

### 2.6 The default-parameter sweep (2026-09)

Every remaining number in §1 was screened, not just the ones an idea had touched: 127 arms over
mesh preparation, regularization and schedule, the stepper constants, the photometric term, and
view/scale selection, on the three EPFL ground-truth scenes, with the survivors put through the §3
gate on the four Tanks & Temples scenes. **Two defaults moved, and they move together.**

Measured on the shipping build (CUDA, resolution level 1), against the same frozen inputs at the
previous defaults:

| scene | previous | **shipped** | Δ | wall s |
|---|---|---|---|---|
| Truck | 0.6667 | **0.6679** | +0.0012 | 71.6 → 69.4 |
| Barn | 0.6673 | **0.6706** | +0.0033 | 199.1 → 198.6 |
| Ignatius | 0.7731 | **0.7799** | +0.0068 | 58.8 → 60.7 |
| Meetingroom | 0.4119 | **0.4129** | +0.0010 | 158.1 → 138.7 |
| **Tanks & Temples mean** | | | **+0.0031** | |
| fountain-P11 | 0.3430 | **0.3442** | +0.0012 | |
| Herz-Jesu-P8 | 0.4674 | **0.4700** | +0.0026 | |
| Herz-Jesu-P25 | 0.6298 | **0.6345** | +0.0047 | |
| **EPFL mean** | | | **+0.0028** | |

Positive on all seven scenes, no scene below the gate's −0.002, pooled wall 0.91-0.95x, peak
working set 1.00x, face counts flat to 0.1 %.

The change is the pre-blur sigma multiplied by 0.75 (`0.12·step + 0.2` → `0.09·step + 0.15`,
§1.2) together with `StepGrow` 1.1 → 1.05 (§1.7). **Neither may be reverted alone.** Sharpening
the images makes the gradient more informative but the objective more locally rugged, and the
old growth factor then over-steps that finer landscape; slowing growth without sharpening just
spends evaluations more slowly on the blurrier one. On Tanks & Temples the parts measure +0.0011
(blur alone, and it misses the gate) and **−0.0022** (`StepGrow` alone, Ignatius −0.0087) against
+0.0033 for the pair — an interaction worth +0.0044, fifteen times Ignatius's noise floor. On EPFL
the two are merely additive, so what generalizes is the weaker statement: the pair is positive on
all seven scenes across both datasets while neither knob is.

**What the sweep confirmed rather than changed.** `--max-face-area` 16, `--max-views` 8, `--scales`
2 × 0.5, `--regularity-weight` 0.2, `--rigidity-elasticity-ratio` 0.9, the 3·nA/7 phase-B budget,
remesh crease angle 20°, 10 remesh iterations, the two-heuristic preparation of the time (since
replaced by the explicit target of §1.2, §2.8), `StepInit` 0.5, `StepShrink`, `MaxRejects` 4,
`Kappa`, `HalfSize` 3, `MinWindowCount` 25, `VarFloor`, the reliability offset and both occlusion
gates. Most now have a measured response curve behind them for the first time.

**`StepMax` never binds.** `StepInit` is 0.5 and the first accepted evaluation grows it to 0.550 px;
0.550 is the largest step in every trace on all seven scenes. Reaching the 1.0 px cap would need
about seven consecutive accepts, which do not happen. `StepStop` and `MinIters`, by contrast, are
both live — but only on Tanks & Temples: on Truck, `StepStop` is what ends the scale and `MinIters`
is what stops it firing two evaluations early, while on EPFL every scale ends on `MaxRejects`
instead, so EPFL alone cannot see either rule.

**Two traps that any repeat of this exercise will hit.** *(a) The chaotic floor is ~0.003 F1 on
Herz-Jesu-P25*, proven by a pure rounding control: writing the remesh band `[4/5, 4/3]` as
`[0.8, 1.3333]` — a 3e-5 relative change — moved F1 by +0.0028 off a 22-vertex difference. Single-
scene evidence below that is not evidence. *(b) The do-less artifact.* Refinement **degrades** F1
against its own input mesh on Herz-Jesu-P8 (−0.0070) and P25 (−0.0225), so on those scenes an arm
can "win" purely by refining less — corr(Δiterations, ΔF1) is −0.93 on P8. Rank on fountain-P11,
the one scene where refinement improves F1; treat P8 as do-no-harm evidence only; and always report
Δfaces and Δiterations beside ΔF1. The artifact **reverses sign** on Tanks & Temples, where
refinement does improve F1, which is why `--max-views` looked like the campaign's biggest win on
EPFL (+0.0099) and failed the gate at −0.0058 (#31).

**Method note.** Three of the four EPFL "winners" were withdrawn by their own follow-up runs: two
by bracketing (a knob whose best value sits on the edge of the tested range is not yet measured —
both the remesh crease angle and `StepGrow` looked like standalone wins until their neighbours were
filled in and the curve turned out flat or non-monotone), and one by the T&T gate. A three-scene
*mean* hides a non-monotone shape; a knob is a winner only when the curve is monotone on
fountain-P11 and the neighbours on both sides have been measured.

---

### 2.7 Comparison with a reference implementation (2026-09)

A commercial implementation of the same Vu et al. energy differs from the shipped pipeline in
twelve places; the ones the registry had not already measured were implemented as default-off
knobs on both backends, screened on the three EPFL scenes and gated on Tanks & Temples (§3
protocol, CUDA, resolution level 1). Every arm ran alone: the mesh2mesh evaluator takes 65 s by
itself and ~700 s beside any build, so a screen never overlaps a compile.

| reference mechanism | shipped OpenMVS counterpart | measured | outcome |
|---|---|---|---|
| Image pairs voted by the tie-point tracks (cells of 8 px, angle weight peaking at 24.6°, top-4 per view with a dominance test) | best neighbours of each view (`--max-views` 8) | EPFL: Herz-Jesu-P25 −0.011..−0.015 for every K; Tanks & Temples K=4 **−0.0221** mean (Ignatius −0.0445) at 0.49-0.64x wall, K=2 −0.0250 at 0.46-0.53x — with the face count halved wherever F1 fell, because the pair set drives the subdivision; re-measured with the pair-independent area rule below | removed (#44) |
| Photometric support masks (8 px around every reconstructed point) | none (every pixel scores) | inert on EPFL, Tanks & Temples −0.0042 at +0.7 GB | removed (#42) |
| Umbrella smoothing switched to the bi-Laplacian where the data opposes it | rigidity/elasticity blend | inert at the default weight, fountain-P11 loses at higher weights; Tanks & Temples −0.0008 mean at the default weight (Barn −0.0024), +0.0005 at weight 1.0, where the plain weight 1.0 already earns +0.0009 without the switch | removed (#40) |
| End-of-level relaxation `v += 0.45 (L − mean L)` | none | Herz-Jesu-P25 +0.0101 but Barn −0.0119; Barn's loss is a schedule artifact, without it Ignatius loses instead (§2.8) | removed (#43, #49) |
| Per-vertex step cap 2 px, halved on reversal | global median normalizer (#1) | Herz-Jesu-P8 −0.025 for every cap | removed (#38) |
| Fixed 20 evaluations per level, no rejection | bold driver with accept/reject | EPFL −0.016 mean | removed (#39) |
| Edge-cap subdivision to 10 px in the best view | 1-to-4 split at 16 px² in the tightest pair + edge band | as a pixel-graded isotropic remesh: worse than the split at equal face count | removed (#41) |
| Post-refinement decimation within 0.5 px | none | 0.25 px F1-neutral on all seven scenes at −32..−65 % faces, and flat on the threshold-free distances too; 0.5 px −0.004 worst at −55..−83 % | ships as `--simplify-tolerance`, **0.25 px by default** (§1.2) |
| Colour ZNCC, GPU job partitioning, 3 mip levels | grey ZNCC, whole-scene CUDA, 2 scales | not measured (out of scope here; scale count is in #4-#37) | — |

EPFL screen of the schedule and smoothing arms (ΔF1 against the same pin's baseline, P25 / P8 /
fountain-P11): switch +0.0016 / −0.0000 / −0.0002 (weight 1.0: +0.0066 / +0.0005 / −0.0037);
end relax 0.45: +0.0101 / −0.0010 / −0.0027; caps 1 / 2 / 4 px: −0.008..−0.012 / −0.024..−0.026 /
−0.0002..−0.0007; fixed schedule −0.0155 / −0.0329 / +0.0001; sizing remesh 10 px −0.0226 /
−0.0079 / −0.0008. Tanks & Temples for the survivors (Truck / Barn / Ignatius / Meetingroom):
support 8 px −0.0002 / −0.0130 / −0.0003 / −0.0034; end relax +0.0001 / −0.0119 / −0.0008 /
+0.0025; simplify 0.25 px −0.0001 / +0.0001 / −0.0004 / +0.0004 at 0.35 / 0.52 / 0.40 / 0.68x
faces; simplify 0.5 px +0.0009 / +0.0002 / −0.0039 / +0.0002 at 0.17 / 0.30 / 0.22 / 0.45x.
The shipped tolerance was then re-measured on the **CPU** backend (fountain-P11 and Herz-Jesu-P8,
where the pixel factors are gathered host-side instead of downloaded from the device): −0.0001 mean
at 0.25 px and −0.0004 at 0.5 px, against the CUDA screen's −0.0003 and −0.0011 on the same two
scenes, at the same 0.46-0.60x / 0.28-0.34x face counts.

The voting was re-measured with the mesh resolution decoupled from the pair set. The subdivision
follows the projected face area, and the shipped rule takes the *smaller* projection of the best
pair, so dropping pairs coarsens the mesh: every voting arm above carried 0.45-1.03x the baseline
face count, and F1 fell exactly where the faces did. A temporary `Face Area Rule` 1 (the largest
projection over the images, which no pair set can change) was added to both backends to separate
the two effects. EPFL, same pin, ΔF1 P25 / P8 / fountain-P11:

| arm | ΔF1 | mean | faces | wall |
|---|---|---|---|---|
| area rule alone | −0.0188 / −0.0266 / +0.0010 | −0.0148 | 1.46-1.54 | 1.25-1.47 |
| area rule + `--max-views 4` | +0.0027 / −0.0012 / +0.0118 | +0.0044 | 1.46-1.51 | 1.13-1.30 |
| area rule + `--max-views 2` | −0.0006 / +0.0068 / +0.0161 | +0.0074 | 1.45-1.50 | 1.08-1.24 |
| voting K=4 + area rule | −0.0293 / +0.0020 / −0.0025 | −0.0099 | 1.46-1.56 | 1.63-1.96 |
| voting K=2 + area rule | −0.0362 / +0.0036 / +0.0026 | −0.0100 | 1.45-1.56 | 1.58-2.04 |

At an equal view budget and an equal face count the voting scores **−0.0143** (K=4) and **−0.0174**
(K=2) against its own control and costs 1.5-1.6x that control's wall, so the selection itself is the
loss and not the resolution it induced (#44). The area rule is no default either: alone it is
−0.0148 for +50 % faces, and its reduced-view arms win on EPFL only — on Tanks & Temples they are
−0.0034 (mv4) and −0.0133 (mv2), because there the pair set is dense enough that the two rules
nearly coincide (1.05-1.24x faces). It was an instrument, and it is removed with the voting (#45).

**The fast mode.** The user-facing question the comparison had to answer is what to trade for
speed. Three levers were gated on Tanks & Temples against the same baseline (ΔF1 Truck / Barn /
Ignatius / Meetingroom, wall as a fraction of the baseline's refinement wall):

| lever | ΔF1 | mean | wall | faces |
|---|---|---|---|---|
| `--max-views 4` | −0.0034 / −0.0174 / +0.0022 / −0.0030 | −0.0054 | 0.59 / 0.83 / 0.61 / 0.59 | 0.64-1.00 |
| `--max-views 4 --simplify-tolerance 0.5` | −0.0033 / −0.0169 / −0.0009 / −0.0027 | −0.0060 | 0.60 / 0.84 / 0.59 / 0.61 | 0.07-0.35 |
| `--max-views 2` | −0.0054 / −0.0185 / −0.0498 / −0.0027 | −0.0191 | 0.45-0.57 | 0.46-0.98 |
| `--resolution-level` +1 (`--min-resolution 320`) | −0.0155 / −0.0392 / **−0.1804** / −0.0102 | −0.0613 | 0.57 / 0.39 / 0.53 / 0.34 | 0.33-0.35 |

Only the first two stay inside the acceptance rule for a fast mode (mean ΔF1 ≥ −0.01, no scene
below −0.02); neither reaches the 0.5x wall the rule also asked for, and none of the levers gets
close to the 2x the request implied. **The reason is that a large part of the wall is not
per-pixel work at all.** On Truck the two mesh preparation passes (`--decimate 0` auto plus the
hole closing and edge-size remesh, one per scale) take 10.8 s and 11.9 s of a 59.9 s refinement:
38 % of it, invariant under both the view count and the working resolution. Cutting views or
resolution only attacks the remaining 62 %, which is why `--max-views 4`, `--max-views 2` and one
resolution level down all saturate in the same 0.4-0.6x band while their accuracy costs differ by
an order of magnitude.

So the fast mode is one switch over knobs that already exist — since 2026-09 the face area, not
the view budget (§2.9):

```
RefineMesh ... --fast        # --max-face-area 32 --simplify-tolerance 0.5
```

The rows above measured the view budget on the preparation of the time, where it was the only
lever that did not also cost an order of magnitude in accuracy. §2.9 re-measured the composition
once the face area became sweepable and the view budget lost: the shipped preset is
`--max-face-area 32 --simplify-tolerance 0.5` at −0.005 mean F1, 0.40x faces and 0.78x wall
against the default. The preset fills in only what the command line did not state, so an explicit
`--max-face-area` or `--simplify-tolerance` overrides it; the Tiny functional runs pin that
`--fast` and the two options spelled out produce the same mesh byte for byte, that
`--fast --max-face-area 16` reproduces `--simplify-tolerance 0.5` alone, and that `--fast` does
not rescue an invalid tolerance. Raising `--resolution-level` instead is recorded as #46: it
is the cheapest lever per second and by far the most expensive per unit of quality, because it
also cuts the face count the subdivision reaches (0.33-0.35x) — on Ignatius, whose gap to its own
input mesh is already the dominant term (§5.1), that removes 0.18 F1.

### 2.8 Preparation, schedule, relaxation and visibility arms on Tanks & Temples (2026-09)

Five follow-ups to §2.7, all measured on Tanks & Temples directly (Truck / Barn / Ignatius /
Meetingroom, CUDA, level 1, one arm at a time, ΔF1 against the same build's baseline; the EPFL
screen is a do-less instrument for anything that changes the mesh preparation, and the stepper
arms below turn on behaviour Truck alone exhibits). The baseline of this build reproduces §2.7's:
0.6678 / 0.6707 / 0.7795 / 0.4133.

**What the optimizer asks of its input mesh.** The energy is evaluated per pixel and scattered
onto the three vertices of the face the pixel lands on; the per-vertex direction is that raw
scattered sum, the stride is one global pixel-unit eta, and the regularizer is the umbrella or
bi-Laplacian of the 1-ring. Five requirements follow. *Resolution*: a face must be small enough
in every view that scores it for the surface to bend where the images say it bends, and the 7x7
ZNCC window is the unit of evidence — a face much smaller than a window buys nothing the window
does not already blur. *Signal*: below ~1-2 px² in its best pair a face receives no pixel centre
and its vertices move on smoothing alone; between 2 and ~10 px² the gradient is the average of a
handful of pixels. *Regularity*: the umbrella approximates the mean-curvature normal only on
isotropic, valence-6 rings; a skinny ring gives it a tangential component that slides vertices
along the surface. *Persistence*: vertices the coarse scale placed must survive to the fine scale
(a resampling pass between scales throws away what the coarse scale bought, #41). *Cost*: every
vertex costs the same per evaluation, and every `Mesh::Clean` pass a fixed amount.

Against that, the shipped preparation (§1.2) stacks two relative heuristics — decimate to six
times the median area, then remesh to 2.25x the mean edge — and only the 32 px² split states a
pixel size. The remesh, run after the split, is what sets the density, uniformly in world space:
near regions end up over-resolved in pixels and far ones are caught by the split. On Truck the
prepared coarse mesh carries 425 k faces at ~3 px² per face for its half-resolution images
(11.7 px² mean, median 8, at full resolution), 85 % of the fine mesh's count; the coarse scale is
coarse for the images only. The preparation is also 40 s of Truck's 75 s. Three arms, single
factor each, were measured as default-off knobs of the shared `PrepareRefineMesh` and removed
again: an *explicit pixel target*
(`Prepare Target Area` T: decimate straight to a mean tightest-pair area of T px² over the seen
faces, measured analytically on a stride sample because the rasterized counts quantize to 1 px²
on a sub-pixel mesh, then remesh in a band around the *resulting* mean edge so the remesh
regularizes without coarsening; T = 4 keeps today's density, 8 and 12 make a genuinely coarse
first scale that the 1-to-4 split refills); *remesh before the split* (`Prepare Order` 1, so the
split's vertices survive the remesh and the cap holds on the first scale too); and a
*butterfly-interpolated split* (`Prepare Split` 1: the new vertices land on the modified-butterfly
surface through the coarse vertices and their wings instead of on the flat facets, without moving
a coarse vertex). Not pursued: curvature-adaptive sizing allocates faces by the *input* mesh's
curvature, which the refinement is about to change; pixel-graded sizing at both scales already
lost (#41); anisotropic remeshing breaks the umbrella's ring assumption.

**What the baseline schedule actually does on these scenes.** Every scale opens at
`StepInit` 0.5 px, and on all four scenes that opening is rejected three to five times in a row
before the first step is accepted on its merit (`Arrrr…`): 8 of Truck's 14 evaluations, 8 of
Barn's 34, 8 of Ignatius's 26 and 8 of Meetingroom's 31 are opening cascades. On Truck and
Meetingroom the coarse scale accepts *nothing* beyond its free first evaluation — the cascade
runs into `MaxRejects` and the scale ends with the surface 1/32 of a 0.5 px step from where it
started — and the fine scale accepts 3 of 8 (Truck) at strides of 0.02-0.03 px. The usable
stride on Truck is ~0.01-0.02 px, fifty times below the opening. The gain over the input mesh
(+0.0072 on Truck) is therefore the preparation's, not the iterations'.

**The first-order model of S is not usable.** The arms below that predict S along the applied
step use dS ≈ ⟨g, D⟩ / photoNorm, with g the raw photometric gradient (`Terms::photoNorm`, both
backends). Measured against the actual change (the `step search` debug line), the ratio
measured/predicted has a median of 0.16-0.22 over the accepted steps and does **not** tend to 1
as the step shrinks (0.10-1.06 at 0.025 px): the ZNCC energy is not predictable from its
gradient at any stride the driver uses, so a Levenberg-Marquardt gain ratio cannot be
calibrated on it and a quadratic backtrack degenerates to the shipped halving except at the
opening.

| arm (knob) | Truck | Barn | Ignatius | Meetingroom | mean | wall | faces | outcome |
|---|---|---|---|---|---|---|---|---|
| Opening rejections exempt from `MaxRejects` (`Step Search` 16) | −0.0013 | −0.0037 | **+0.0031** | −0.0030 | −0.0012 | 1.22x | 1.08x | removed |
| Quadratic backtracking on a rejection (`Step Search` 1; 17 with the exemption is identical) | −0.0051 | −0.0037 | −0.0038 | −0.0048 | **−0.0043** | 1.12x | 1.08x | removed |
| End relaxation 0.45 between scales only, opening exempt (`End Relax` 0.45, mode 1 + 16) | −0.0001 | +0.0003 | −0.0024 | −0.0017 | −0.0010 | 1.16x | 0.93x | removed |
| End relaxation 0.45 after the last scale only, opening exempt (mode 2 + 16) | +0.0004 | −0.0018 | −0.0021 | −0.0008 | −0.0011 | 1.23x | 0.81x | removed |
| Remove the faces no image sees, after the refinement (measured as a RefineMesh post-pass; now `TransformScene --remove-unseen-faces`) | +0.0000 | −0.0002 | +0.0000 | +0.0003 | +0.0000 | 1.01x | 1.00x | inert: 15 / 53 / 21 / 734 faces of 172-500 k removed |

Three readings. **Letting a scale run past its opening loses on three scenes of four.** The
exemption turns Truck's coarse scale from 1 accepted evaluation into 13 (strides of 0.01 px, S
−1.8 %) and Meetingroom's into 24, and both lose F1; only Ignatius, the one object scene, gains.
This is §4's law (#12, #13, #20) in its Tanks & Temples form: the more the coarse scale moves the
mesh, the worse the result, and the shipped driver's early death at `MaxRejects` is what protects
it. The quadratic backtrack loses for the same reason from the other side: it reaches an
acceptable stride in fewer rejections (retain 0.31 instead of 0.5 on the first one), so the
scale stays alive and moves more (30 evaluations on Truck instead of 14). **The Barn loss of the
relaxation (#43) was the schedule artifact it was suspected to be** — under the exemption Barn
reads +0.0003 instead of −0.0119 — but with it removed the relaxation is a low-pass filter that
Ignatius pays for (−0.0024 / −0.0021) and nothing else earns enough on; against the exemption
alone it is +0.0003 mean, with the same Ignatius loss. **OpenMVS meshes have no unseen faces to
remove.** The reference implementation's visibility test drops 2.8 % of its facets and lifts its
Truck precision 0.467 → 0.521 because its mesher produces undersides and interiors; the Delaunay
graph-cut here carves the surface along the visibility rays, so the same test finds 0.01-0.15 %
of the faces and precision does not move. The pass stays as an opt-in for imported meshes, in
TransformScene (and ReconstructMesh), not in the refinement.

The second cell, same scenes, the same tree plus the two arms marked †:

| arm (knob) | Truck | Barn | Ignatius | Meetingroom | mean | wall | faces | outcome |
|---|---|---|---|---|---|---|---|---|
| Coarse scales prepare the mesh but run no evaluation († `Skip Coarse Evaluations` 1) | +0.0017 | **−0.0182** | **+0.0129** | +0.0001 | −0.0009 | 0.85x | 0.94x | removed |
| Warm start: a scale after the first opens at twice the eta the previous one ended with († `Step Search` 4) | +0.0030 | −0.0004 | +0.0009 | +0.0007 | +0.0011 | 0.97x | 1.02x | removed (under the gate) |
| Fixed opening: the first 3 evaluations of every scale accepted unconditionally (`Step Search` 8) | −0.5435 | −0.1426 | −0.7170 | −0.2873 | **−0.4226** | 16x | 1.50x | removed |
| Quadratic backtracking + Levenberg-Marquardt gain-ratio growth (`Step Search` 3) | +0.0003 | −0.0111 | −0.0098 | −0.0015 | −0.0055 | 0.98x | 0.97x | removed |
| Explicit pixel target 4 px² + regularizing remesh (`Prepare Target Area` 4, `Prepare Remesh Edge` −1) | −0.0022 | +0.0014 | −0.0037 | −0.0034 | −0.0020 | 1.02x | 1.07x | removed |
| Explicit pixel target 8 px² (a coarse first scale) | −0.0030 | +0.0017 | −0.0004 | +0.0000 | −0.0004 | 0.85x | 0.79x | removed (nothing on top of `--fast`, below) |
| Explicit pixel target 12 px² | −0.0044 | +0.0008 | −0.0082 | +0.0013 | −0.0026 | 0.73x | 0.65x | removed (nothing on top of `--fast`, below) |
| Remesh before the split (`Prepare Order` 1) | +0.0000 | −0.0026 | −0.0018 | −0.0006 | −0.0012 | 0.96x | 1.01x | removed |
| Butterfly-interpolated split (`Prepare Split` 1) | +0.0004 | −0.0034 | +0.0003 | +0.0004 | −0.0006 | 0.98x | 1.00x | removed |
| Relaxation 0.45 after the last scale only, shipped schedule (`End Relax` 0.45, mode 2) | +0.0006 | +0.0004 | −0.0062 | +0.0025 | −0.0007 | 1.00x | 0.76x | removed |
| Relaxation 0.25 after the last scale only | +0.0007 | +0.0010 | −0.0005 | +0.0017 | +0.0007 | 0.99x | 0.86x | removed (under the gate) |
| Unseen-face filter on the input mesh as well (`--remove-unseen-faces-pre` 1) | −0.0005 | −0.0033 | +0.0002 | −0.0016 | −0.0013 | 1.00x | 1.00x | removed |

**Preparation.** The explicit target at today's density (T = 4, 0.93-1.23x the faces) loses on
three scenes: decimating a dense, noisy mesh straight to the target with the shape-preserving
QEM keeps the noise the shipped path removes, where a mild decimation (ratio 0.375 on Truck) is
followed by the strong isotropic remesh to 2.25x the mean edge. The remesh is a denoiser, not only
a regularizer, and that is the property the stacked heuristics have that a stated pixel size
does not. The coarser first scales (T = 8, 12) trade F1 for wall along the density line — 0.85x
wall at −0.0004, 0.73x at −0.0026, the losses exactly where faces were removed (Ignatius 0.55x
faces, −0.0082) — so they fail as accuracy defaults, and as fast-mode components they add
nothing: on top of `--fast` (`--max-views 4 --simplify-tolerance 0.5`, itself −0.0059 mean at
0.57-0.85x wall on this pin) the target at 8 / 12 px² costs −0.0086 / −0.0098 mean (Ignatius
−0.0129 / −0.0181) for the **same** 0.65x / 0.63x wall — the first `Mesh::Clean` costs 15-25 s
whatever it is asked to keep, and the fewer views already took the per-evaluation part. Remeshing
before the split and the butterfly split are inert on three scenes and lose on
Barn, where the fine scale stopped after 20 evaluations instead of 26. **Barn's column is a
lottery on the stop rule**: for every arm that perturbs the mesh by a few faces its F1 tracks how
many evaluations the fine scale got before a streak of four rejections ended it (26 → 0.6707,
20 → 0.6673-0.6681, 15 → 0.6674, 8 → 0.6525), which is also what #43/#49 and #40's Barn
losses were. The scene is kept in the gate because that fragility is the shipped behaviour, but
a Barn-only loss of a few thousandths is not evidence about the arm.

**Schedule.** The fixed opening — the hybrid idea of spending a budget of unconditional steps
before managing the stride — is catastrophic without a per-vertex cap: three 0.5 px steps along
the raw gradient double S each time and blow the mesh up (the 16-31x wall is the rasterization of
the exploded faces). The reference's fixed schedule survives only because of its 2 px cap, and #38
measured that the cap loses on its own. The Levenberg-Marquardt gain ratio loses −0.0055 as the
calibration predicted: with rho ≈ 0.2 on most accepted steps it halves eta where the bold driver
grows it. The warm start is the one stepper change that is positive, +0.0011 mean with no scene
below −0.0004 at 0.97x wall: the fine scale skips its opening cascade (Ignatius 21 evaluations
instead of 26) but then runs longer wherever it can (Truck 13 instead of 8, +0.0030), so it is not
a speed lever and stays under the +0.002 gate; removed, like #35. Skipping the coarse scale's
evaluations says how scene-dependent that scale's contribution is: Ignatius gains **+0.0129**
without it (its two accepted coarse steps cost that much), Truck +0.0017 at 0.91x wall,
Meetingroom is inert at 0.87x, and Barn loses **−0.0182** because the fine scale, opening from
the unrefined surface, rejects four times and dies after 8 evaluations. The coarse scale is
neither the do-nothing it is on Truck nor the guard it is on Barn by design; no arm here can tell
the two cases apart, which is why the knob goes and the finding stays (§6).

**Relaxation.** Applied after the last scale only, where it cannot touch the schedule, the
bi-Laplacian relaxation is a pure post-smoothing: 0.45 lifts three scenes by +0.0004..+0.0025 and
costs Ignatius −0.0062, and 0.25 halves both sides (+0.0007 mean, Ignatius −0.0005). It cannot be
made an always-win because it is a low-pass filter: the noisy large scenes like it and the
detailed object pays for it, and the fraction only sets where between the two it sits. Filtering
the input mesh as well as the output removes a few faces and rolls Barn's dice (−0.0033).

**What became the default, and why the warm start did not.** Two of the losers above were
re-opened on the speed and mesh-size criteria rather than F1 alone (a user decision: the
deliverable is a mesh people wait for and store). The explicit target at 8 px² and the warm start
were composed as the new defaults and measured as a 2x2 on a third build (ΔF1 against the
previous defaults, Truck / Barn / Ignatius / Meetingroom; that cell's own wall column is unusable —
the machine was in a degraded state that ran the unchanged old-default binary at 1.55-1.86x its
usual wall for bit-identical meshes — so the shipped composition's wall was measured against the
previous defaults re-run in the same state (0.83x: Truck 0.66, Barn 0.80, Ignatius 0.86,
Meetingroom 1.00; the preparation alone 0.67-0.86x), the two-pass arm's is the earlier arm's,
measured alone, and the warm-start rows have none):

| composition | Truck | Barn | Ignatius | Meetingroom | mean | faces | wall |
|---|---|---|---|---|---|---|---|
| one Clean pass (decimate to the target + band remesh), fixed opening — **shipped** | −0.0028 | +0.0022 | +0.0010 | +0.0001 | **+0.0001** | 0.85x | 0.83x |
| two passes (decimate; split; remesh), fixed opening — the measured arm, reproduced | −0.0030 | +0.0017 | −0.0004 | +0.0000 | −0.0004 | 0.79x | 0.85x |
| two passes + warm start | +0.0023 | +0.0011 | **−0.0081** | +0.0011 | −0.0009 | 0.80x | — |
| one pass + warm start | +0.0020 | −0.0012 | **−0.0094** | +0.0007 | −0.0020 | 0.82x | — |

The single pass is the better preparation: it keeps 6-15 % more faces on the coarse scale than
the two-pass arm (the remesh no longer runs over the split's fresh vertices) and gains on three
scenes. The warm start, +0.0011 on the old preparation, costs Ignatius −0.008 on the coarse one:
its fine scale opens at 0.05 px instead of the cascade's 0.03-0.04 px, accepts nine larger strides
and hits the patience rule after 11 evaluations instead of 20-24, reaching the same S (0.2212
against 0.2213) on a visibly worse surface — the accepted steps carry the smoothing, and Ignatius
wants them small and many. It stays out (#52). `--fast` on the shipped defaults, measured in the
same state: −0.0104 mean against the previous defaults (Truck −0.0026, Barn −0.0129, Ignatius
−0.0203, Meetingroom −0.0057) at 0.68x wall and 0.51x faces, which against the shipped defaults is
−0.0105 at 0.81x wall (0.73-0.96x) and 0.60x faces (§2.7): the preset's definition (4 views, 0.5 px)
was set on the old preparation and is open to revision.

### 2.9 The face-area cap swept, and what it did to the fast preset (2026-09)

`--max-face-area` had never been measured: 16 was a guess carried since the option was added, and
it is the only knob the preparation reads, since the decimation target is half of it (§1.2). Swept
on the four Tanks & Temples scenes against the shipped default, one cell per curve, walls in-cell:

| cap | target px² | ΔF1 | faces | wall | delivered px² (% of target) |
|---|---|---|---|---|---|
| 8 | 4 | −0.0044 | 1.72x | 1.23x | 3.2 (79 %) |
| **16 (default)** | 8 | — | 1.00x | 1.00x | 6.0 (75 %) |
| 24 | 12 | −0.0053 | 0.68x | 0.82x | 8.8 (73 %) |
| 32 | 16 | −0.0036 | 0.57x | 0.78x | 11.6 (72 %) |
| 48 | 24 | −0.0105 | 0.38x | 0.67x | 17.2 (72 %) |
| 64 | 32 | −0.0175 | 0.30x | 0.63x | 22.5 (70 %) |

**The default stays at 16.** Denser is worse on every axis at once — cap 8 costs F1 *and* 1.72x the
faces *and* 1.23x the wall — so the shipped density is not too coarse. Coarser trades F1 for size
and wall monotonically past 32, so there is no free win to take in either direction.

**Two things the sweep found on the way.** First, the auto decimation floored its ratio at 2 % of
the input faces, and from cap 32 upward most scenes ask for less than that: the target was silently
clamped and the cap stopped coarsening the mesh except through the split. The floor is now 0.2 %,
which never binds at the default (the ratios there are 3.5-13 %) and only restores the knob's reach
above it — the delivered density in the table is a constant 70-79 % of nominal at every cap now,
where it used to drift down to 61 % as the clamp bit. Second, that constant undershoot is the
remesh band's own equilibrium, not the decimation arithmetic: it survives every way of deriving the
target (below), so `--max-face-area 16` delivers a mean projected area near 22 px² at the finest
scale rather than 16, and the parameter is a split threshold, not a delivered density.

**Reading the F1 column.** Most of the per-scene spread in this cell is the schedule's stopping
rule, not the density. Across the six arms of the first cell Ignatius' F1 is a strictly monotone
function of the evaluations the schedule reached before the patience rule fired — 0.7600 at 24,
0.7714 at 26, 0.7743 at 28, 0.7805 at 32 — while the face counts over those same runs run from 83k
to 240k in no order at all. A changed mesh moves where the stop lands, and that motion is larger
than the density effect anywhere inside 0.5-2x of the default. It is the same lottery §2.8 recorded
for Barn, and it is why the cap was not chosen on F1 differences of ±0.005.

**The threshold-free check disagrees, mildly.** On the three EPFL scenes, where accuracy and
completeness are distances rather than a count at τ, coarser is *better*: cap 24 and 32 improve the
mean accuracy by 2.2 % and 2.6 % and F1 by +0.004 and +0.008, at 0.67x and 0.52x faces, with
completeness flat (+0.2 %). Cap 8 is worse there too (+3.6 % accuracy error). The two datasets agree
that denser is wrong and disagree on how much coarser is right, so the default stays where the gate
dataset puts it.

**Preparation mechanisms, both rejected.** §1.2's preparation reaches the target in two conceptual
steps — decimate by a face-count ratio, then remesh the rings even — and two more direct
formulations were measured against it on the same cell (Truck/Barn/Ignatius/Meetingroom):

| arm | ΔF1 | faces | wall |
|---|---|---|---|
| remesh pinned to the derived target length (decimation keeps the bulk) | −0.0032 | 0.96x | 0.93x |
| one remesh carries the mesh to the target, no decimation at all | −0.0010 | 1.11x | 1.06x |

Both derive the target honestly: the sampled seen faces convert pixels to scene units, and an
equilateral triangle of edge L covers √3/4·L², so the world edge whose projection is the target
area needs no constant. Neither earns its place. Pinning the remesh changes nothing except on
Ignatius, where it loses; dropping the decimation costs 11 % more faces and 6 % more wall for no F1,
because a pure remesh undershoots the target *further* (62 % of nominal against 75 %) and the
halfmesh pass gets no cheaper for having one stage instead of two. The shipped two-step preparation
stands, and the temporary `Prepare Mode` knob was removed with them.

**What the sweep did change: the fast preset.** `--fast` was `--max-views 4 --simplify-tolerance
0.5`. Measured against compositions built on the cap, in one cell against the same baseline:

| preset | ΔF1 | faces | wall |
|---|---|---|---|
| `--max-views 4 --simplify-tolerance 0.5` (the old `--fast`) | −0.0105 | 0.60x | 0.83x |
| `--max-face-area 32 --max-views 4` | −0.0141 | 0.53x | 0.64x |
| **`--max-face-area 32 --simplify-tolerance 0.5` (the new `--fast`)** | **−0.0046** | **0.40x** | **0.78x** |
| all three together | −0.0147 | 0.37x | 0.65x |

The face area dominates the view budget on every axis at once: less than half the F1 cost, a third
fewer faces, and slightly less wall. The reason is §2.7's own finding read the other way — a large
part of the wall is mesh work that the view count cannot touch, so cutting views buys only the
per-pixel part and pays for it in accuracy (Barn −0.015, Ignatius −0.021 on its own), while a
coarser mesh cuts both halves at once. `--max-views` leaves the preset entirely; it remains
available on its own for anyone who needs the memory back.

### 2.10 One measurement for the whole pipeline, and the track-voted pair selection (2026-09)

Four places asked "how big does this face project?" and three of them agreed. `ListFaceAreas`
reduces the per-view rasterized pixel counts over **the refinement's own pairs** — the smaller of a
pair's two views, the larger over the pairs — and the split rule, the decimation target
(`SampleSeenFaceArea`, the same reduction on analytic areas) and the sizing field all read it. The
post-refinement decimation read something else: `f/depth` in each vertex's single best view. The
work below unified the fourth, tried the alternatives for the reduction itself, and closed the pair
vote.

**The decimation's units (shipped).** `f/depth` is what a camera would resolve looking at the
surface head-on with nothing in the way: it carries neither the foreshortening nor the occlusion
that a rasterized area carries, and the view it picks need not appear in any refinement pair.
Reading the tolerance off the tightest-pair areas instead is a controlled A/B — the refinement is
bit-identical, only the last pass differs:

| | Truck | Barn | Ignatius | Meetingroom |
|---|---|---|---|---|
| ΔF1 | +0.0000 | +0.0003 | −0.0006 | +0.0005 |
| faces | 0.84x | 0.80x | 0.80x | 0.81x |

and on the three EPFL scenes the threshold-free accuracy **and** completeness are identical to four
decimals at 0.85x faces. The removed faces carry no geometry; F1-at-τ cannot see that, which is why
the density questions in this document are cross-checked there.

**The reduction rule (rejected, `max` stays).** A face is seen by many pairs at wildly different
sizes, and `max` keeps a face as fine as the *best* pair can exploit. `median` and `mean` — the
typical pair decides — are not a different rule so much as a different scale: they report ~5x
smaller areas, so `--max-face-area 16` under them means what cap 80 means under `max`, and
comparing at a fixed cap only re-measures §2.9's sweep. Density-matched at cap 2 and 3, `mean` gave
+0.0020 mean F1 and `median` −0.0003 at 1.31x faces, but both swing the evaluation counts hard
(Truck 14 → 28, Barn 33 → 46) and this benchmark's F1 tracks that count, so the gain is not
attributable to the rule; and `median` at its matched cap puts the working numbers at 1-2 px, where
counting whole pixels stops being a measurement. Not enough to redefine `--max-face-area`, whose
value 16 was tuned against `max` (§2.9). The reduction now lives once, in
`ReduceFaceAreasOverPairs`, called by both backends instead of being copied into each.

**The track-voted pair selection (rejected).** The reference implementation's pair vote,
reimplemented from the reconstructed points: thin them to one
cell per 8 px in the view that resolves them best, let every cell give 1 to the pair of views
seeing it at the best triangulation angle (`((1+cosθ)/2)¹⁰·sinθ`, peaking at 24.6°) and less to the
rest, then keep a pair that ranks among a view's best unless a third view is better connected to
both ends. It is a global, symmetric choice and it thins hard — **2.62 pairs per image on Barn,
3.16 Meetingroom, 3.36 Ignatius, 4.56 Truck**, against eight neighbours each — and the refinement
runs at 0.69x wall. It is also worse by −0.0100 mean F1, and not through the evaluation count: at
*equal* counts Barn scores 0.6532 against 0.6728 (31 evaluations both), Meetingroom 0.4013 against
0.4134, Ignatius 0.7486 against 0.7795 (26 both). Barn's voted run even produced more faces than
the default. Too few pairs under-constrain the surface, and the vote thins hardest on the scenes
with the most images. As a *view-budget* lever it beats `--max-views 4` (0.69x wall for the same
−0.010, against 0.83x), but the view budget is the wrong lever: the face-area cap buys 0.71-0.79x
for −0.0035 and halves the mesh at the same time (§2.9).

**Ending a scale sooner (`Max Evaluations`, kept as a bound, not a speed lever).** The convergence
rules run well past the point of return on one scene: Meetingroom's fine scale spends evaluations 8
to 26 moving the energy by 0.05-0.17 % each. A hard per-scale cap at 12 costs −0.0013 mean F1 for
−7 % wall, but Truck never reaches it, Ignatius saves 3 %, and the whole benefit is Meetingroom's
−24 %. Raising the stall tolerance instead (`Progress Tol`, three consecutive evaluations below a
relative decrease) saves the same wall for twice the loss, −0.0026, and is untunable — 0.002 and
0.003 give bit-identical results on all four scenes, since the energy sequence holds no stall value
between them at the deciding point. That knob was removed; `Max Evaluations` stays at its default 0
(the convergence rules decide) for callers who need a bounded run time rather than a faster one.

**A structural note on the post-refinement decimation.** It is a cliff, not a dial. Two runs of one
scene at the same tolerance, with meshes 0.3 % apart in size, decimated to 146148 and 50669 faces.
The preparation remeshes to a uniform density, so nearly every edge carries the same
QEM-cost-to-bound ratio and the whole mesh sits either above the tolerance or below it. This is why
delivered face counts have looked erratic between arms differing in nothing else, and it is worth
remembering before reading any single face count as a property of an arm.

## 3. How these numbers were produced

The benchmark harness is not part of the repository; this section records what it does, so a
number in §2 or §4 can be reproduced or challenged.

**Scoring.** One `RefineMesh` cell per (scene, variant, backend): the harness samples 10 M
area-uniform seeded points from the result inside the frozen scene-to-ground-truth crop and scores
precision/recall/F1 at the scene's tolerance τ. Tanks & Temples scenes go through the official
toolbox; the ground-truth-mesh scenes through a mesh-to-mesh evaluator (exact point-to-triangle
distance to the ground truth for accuracy; 2 M ground-truth samples restricted to surface some
scene camera sees for completeness). The per-scale, per-iteration trace — score, step, applied
median step in pixels, accept/reject — is parsed out of the application log, so a verdict can cite
the trajectory and not just the final number. All evaluation
is seeded and deterministic: identical meshes score identically, so the spread between repeated
runs measures the refinement, not the evaluator.

**Datasets.** Four Tanks & Temples scenes (Ignatius, Truck, Barn, Meetingroom) for acceptance, and
the EPFL/Strecha dense-MVS set for development. The EPFL set is much cheaper per run and has a
laser-scanned ground-truth mesh for fountain-P11 (11 views) and Herz-Jesu-P8 (8 views); the
25-view release of the same building has no published mesh and is scored against the P8 scan
registered into its frame (feature matching then point-to-plane ICP: rotation 0.079°, pure
translation, and a median reconstruction-to-ground-truth distance of 1.62 cm against 1.48 cm for
P8's own reconstruction, so the registration adds nothing measurable). entry-P10, castle-P19 and
castle-P30 have no public ground truth and are registered for robustness only. Camera conventions
are pinned per dataset before any reconstruction: a self-check renders the ground truth into camera
A, warps it into camera B and requires a real photometric correlation, which catches a
plausible-but-wrong convention that a purely geometric check passes.

**Development practice.** Screen on fountain-P11 and Herz-Jesu-P8, promote survivors to the other
ground-truth scenes, and go to Tanks & Temples only to validate a working theory. The small scenes
are roughly 4x cheaper per run end to end, and Truck in particular is a poor instrument for
step-rule work because its traces never contain two consecutive accepted evaluations.

**Noise floor and acceptance gate.** Three identical runs per scene on the CPU give a maximum
pairwise F1 spread of 0.0001 (Truck, Barn, Meetingroom, fountain) to 0.0009 (Ignatius, Herz-Jesu-P8),
mean 0.00028. The gate used throughout: mean ΔF1 over the four Tanks & Temples scenes ≥ +0.002, no
scene below −0.002, uncleaned ΔF1 ≥ −0.003 everywhere, pooled median wall ≤ 1.25x and peak working
set ≤ 1.15x, with a speed route at |ΔF1| ≤ 0.002 everywhere and wall ≤ 0.85x. One run per arm is
enough at that floor. **A wall time measured beside another job is not evidence** (§5.4).

**Diagnostics that are not gates.** A degraded-ground-truth oracle (take the ground truth, decimate
and displace it, refine, and measure how much of the displacement comes back) and a scene-rescale
test both exist and are reported where they contradict a verdict. Neither is an acceptance
criterion: the oracle measures convergence on an input real coarse meshes never produce, and the
rescale test's exact-match criteria are unattainable for reasons in §1.8.

---

## 4. Rejected ideas

**Do not retry without new evidence.** Everything below was measured with the protocol of §3 and
judged against that gate. Losers are removed from the tree, not left behind a switch, unless the
"kept" column says otherwise. Numbers are mean ΔF1 over the four Tanks & Temples scenes unless the
entry says otherwise.

| # | idea | result | kept? |
|---|---|---|---|
| 1 | Per-vertex normalized, unit-clamped photometric direction (every vertex moves at most η px along its own gradient) | **−0.0215** (Truck −0.0427, Barn −0.0241, Meetingroom −0.0379) | no |
| 2 | Rescuing #1 by retuning the regularity weight | monotone in `w` up to the stability limit and still below the old optimizer; the arm does best with its photometric term suppressed | no |
| 3 | Running fewer iterations of the old fixed schedule as a speed arm | −0.0015 at 2.06x faster, but Ignatius −0.0203 | no |
| 4 | Fixed-step control arm (never rejects, never adapts η) | −0.0891 on the ground-truth scenes | removed |
| 5 | Depth-proportional, grazing-aware visibility tolerance | **−0.0047** at base 1, **−0.0056** at base 5 (Ignatius −0.0153 / −0.0197); loosening it makes it worse, so it is not a tuning problem | no |
| 6 | 2x2 any-passing depth tap instead of nearest tap | +0.0002 legacy bias, +0.0000 exact, +0.0008 receiver bias — all inside noise, at three extra depth loads and a branchy search | no |
| 7 | Exact shadow comparison `depth >= z` at the nearest tap | +0.0140 mean but **Barn −0.0076** | no |
| 8 | Shadow receiver bias from the local depth spread | +0.0048, dominated by the simple relative test | no |
| 9 | Relative visibility multiplier other than 1.0002 | 1.0001 +0.0125 (**Barn −0.0105**), 1.0005 +0.0091, 1.0010 +0.0044 — the curve is not monotone | 1.0002 |
| 10 | Summing both warp directions every evaluation instead of alternating | −0.0002, identical evaluation counts; on the CPU it would double the pair work | no, default stays alternating |
| 11 | Pure thin-plate regularizer (rigidity/elasticity ratio 1) | −0.0006 with **Barn −0.0032**; the 10 % first-order share does useful work | no |
| 12 | Resetting the reject streak between phase A and phase B | **−0.0048** (Ignatius −0.0137) — a scale that gave up on its rejections keeps stepping into a worse trajectory | no, and a unit test pins the carried streak |
| 13 | Smaller initial step (0.1 / 0.2 px instead of 0.5) | **−0.0104 / −0.0045**, 30-70 % more evaluations | no |
| 14 | Dividing the per-vertex photometric sum by the confidence-weighted pixel count instead of the pair count | −0.0090, and it doubles the oracle error | no |
| 15 | The paper's literal plain sum, no per-vertex division | **−0.0200**, catastrophic on Ignatius (−0.0869) where the statue is seen by far more pairs than the ground | no |
| 16 | Sign-vote photometric direction (bounded, ±1 per pixel) | **−0.0426** (Ignatius −0.0815); lands below the coarse input | no |
| 17 | Saturating `tanh` photometric direction | **−0.0338** (Ignatius −0.0955) | no |
| 18 | Freezing boundary vertices | −0.0008 with Ignatius −0.0050, Herz-Jesu-P8 −0.0134 | no |
| 19 | Rim-Laplacian boundary treatment plus a second-ring fix | −0.0016 with Ignatius −0.0057 | no |
| 20 | A single full-resolution scale | **−0.0217** (Ignatius −0.0689); the first step is rejected four times and the scale ends before anything is accepted | no |
| 21 | Sobel-3 image derivative | +0.0039 but **Barn −0.0020** | no |
| 22 | 3x5 separable image derivative (the previous default) | −0.0086 against central differences, 6 of 6 scenes | no |
| 23 | Derivative of the bilinear interpolant as the stencil | −0.0007 (Truck −0.0042, Herz-Jesu-P8 −0.0055), although it converges a displaced ground truth better | selectable; it is what the Ceres energy uses |
| 24 | Trust-ratio, Barzilai-Borwein, momentum and momentum-with-clip step rules; predictive and restore-best stop rules | best arm +0.0008 against a +0.002 gate, momentum −0.0093, and the two survivors both lose on Herz-Jesu-P25, the one scene whose accept runs exercise them (§2.4) | no |
| 25 | Retuned stall thresholds (Patience 2/5, ProgressTol 1e-2/1e-4) | ±0.0007, all inside noise (§2.4) | no |
| 26 | Phase B under phase A's stop rules | −0.0053, Herz-Jesu-P8 −0.0108 (§2.4) | no |
| 27 | Raising the per-scale evaluation cap (45 → 100 / 200 / 1000) | ±0.0006; the cap is not what ends a scale | no |
| 28 | Non-default Ceres solver configurations: direction, line search, L-BFGS rank, Oren-Luenberger scaling, iteration cap, function tolerance (§2.5) | no configuration wins on more than one of the three ground-truth scenes; Fletcher-Reeves led after two and lost 0.0057 on the third | no |
| 29 | Pixel-unit parameterization for the Ceres arm | the first line search moves one outlier vertex a pixel and the bulk nothing; the scale dies after one iteration | no |
| 30 | Raising the regularity weight for the Ceres arm (0.5 / 1.0) | −0.0005 / −0.0022 on fountain-P11, −0.0228 / −0.0235 on Herz-Jesu-P8 | no |
| 31 | Lowering `--max-views` (4 or 2 instead of 8) | the largest EPFL win of the whole sweep (+0.0099 at 4) and a **−0.0058 / −0.0191** loss on Tanks & Temples, negative on all four scenes (Ignatius −0.0498 at 2). EPFL scenes hold 8-25 images, so cutting to 4 there filters bad views; T&T scenes hold 150-300, so it discards good ones — and fewer views also mean fewer pairs, a smaller max-over-pairs projected area and a coarser prepared mesh, which flatters the EPFL scenes and costs on T&T (§2.6) | no, 8 |
| 32 | `--max-face-area` 12 or 9 instead of 16 | +0.0005 / +0.0007 on EPFL for +13 % / +26 % faces and 1.19x / 1.33x wall; still only +0.0007 when re-tested at 4 views, where a coarser pairing might have justified a finer target | no, 16 |
| 33 | Remesh crease angle other than 20° | monotone and harmful above it (30° −0.0018, 45° −0.0031, 90° −0.0099 on fountain-P11); below it 5°/10°/15° are all worth the same +0.0015 — half the noise floor — while inflating the mesh up to +50 % | no, 20° |
| 34 | Lowering the auto-decimate floor to 0.05 | +0.0020 EPFL mean carried entirely by Herz-Jesu-P25, which it earns by deleting 29 % of the faces on the two scenes where refining less scores better; also a change to the delivered mesh density, not a tuning change | no, 0.1 |
| 35 | `--regularity-weight` 0.5 or 0.8 instead of 0.2 | +0.0011 mean on Tanks & Temples, never negative on any scene but never reaching the +0.002 gate, and flat between the two values | no, 0.2 |
| 36 | Neighbour-selection thresholds (min common area 0.1, angle band 2.5-45°) | inert: 0.05 and 0.2 produce **byte-identical** output on EPFL, and every angle-band variant is inside the noise floor. Only the neighbour *count* matters, not the thresholds that picked them | unchanged |
| 37 | Pre-blur multipliers below 0.75 (0.6, 0.5) | best on fountain-P11 and Herz-Jesu-P25 but negative on Herz-Jesu-P8 with face counts flat and iterations *up*, so a real loss rather than a do-less effect | no, 0.75 |
| 38 | Per-vertex step cap in pixels on the bold driver (the reference's rule: 2 px, halved at every direction reversal, and never past a quarter of the longest ring edge) | EPFL: Herz-Jesu-P8 **−0.024..−0.026** at every cap (1, 2, 4 px), Herz-Jesu-P25 −0.008..−0.012, fountain-P11 inert; the global median normalizer (#1) already sets the stride, a per-vertex clamp only holds back the vertices with the most signal | removed |
| 39 | The reference's fixed schedule: 20 evaluations per level, every step applied, no rejection, the cap as the only adaptivity (alone, with a 1 px cap, with the end relaxation) | EPFL mean **−0.016** / −0.013 / −0.006 (Herz-Jesu-P8 −0.033 / −0.028 / −0.019); the same verdict as #4 in the reference's form | removed |
| 40 | The reference's smoothing switch: the umbrella term, swapped for the bi-Laplacian wherever the photometric pull opposes it | inert at the default weight (+0.0004 EPFL mean); at weight 0.5 / 1.0 Herz-Jesu-P25 +0.0037 / +0.0066 but fountain-P11 −0.0013 / **−0.0037**. Re-measured on Tanks & Temples (Truck / Barn / Ignatius / Meetingroom) in case the textured or the textureless scenes liked it: weight 0.2 +0.0000 / **−0.0024** / −0.0001 / −0.0006, weight 0.5 +0.0001 / −0.0025 / +0.0000 / +0.0016, weight 1.0 +0.0002 / −0.0011 / +0.0014 / +0.0017 — while the plain weight 0.5 / 1.0 without the switch scores +0.0007 / +0.0009 mean with no negative scene (#35 again, still under the gate). Inert on Truck, and Barn's loss is the opening artifact of §2.8: the switch costs one more rejection at the start of the fine scale, which hits `MaxRejects` and ends it at 15-16 evaluations instead of 25 | removed |
| 41 | Pixel-graded isotropic remesh (every vertex to a 5 / 7 / 10 / 14 px edge in its best view, halfmesh sizing field) in place of the 1-to-4 split and the edge band | worse than the split at equal face count: 10 px Herz-Jesu-P25 **−0.0226**, Herz-Jesu-P8 −0.0079, fountain-P11 −0.0008 at 0.93-1.10x faces; 14 px −0.0206 / −0.0050 / −0.0030 at 0.6-0.8x; 5 px +0.0029 on fountain only, at 2.6-3.0x faces and 1.6-2.7x wall. The remesh re-samples the surface at every scale, the split keeps it | removed; the halfmesh sizing field it used (`RemeshParams::vertexSizing`) is back for `--adaptive-face-size` (#68), which grades the preparation once instead of resampling every scale |
| 42 | Photometric support masks from the dense point-cloud (the reference's rule: score only the pixels within 8 px of a projected reconstructed point; 4 / 8 / 16 px measured) | inert on EPFL (+0.0005 at 8 px: the dense cloud covers nearly every pixel); Tanks & Temples **−0.0042** (Barn −0.0130, Meetingroom −0.0034) at +0.7 GB peak memory | removed |
| 43 | End-of-scale bi-Laplacian relaxation `v += 0.45 (L(v) − mean L)` (the reference's last step of every level; 0.25 measured too) | Herz-Jesu-P25 +0.0101 and Meetingroom +0.0025, but Barn **−0.0119** and fountain-P11 −0.0027: mean −0.0025 on Tanks & Temples | removed |
| 44 | Image pairs voted by the tie-point tracks (the reference's pair vote: 8 px cells, angle weight peaking at 24.6°, top-4 of a view with a transitive-redundancy test) | **−0.0221** at K=4 (Ignatius −0.0445), −0.0250 at K=2; with the mesh resolution held fixed by the area rule below it still loses −0.0143 / −0.0174 against the shipped selection at the same view budget, at 1.5-1.6x its wall | no, removed |
| 45 | Subdividing by the largest projection over the images instead of the smaller projection of the best pair (`Face Area Rule` 1) | EPFL −0.0148 alone at +50 % faces; with `--max-views` 4/2 it wins on EPFL (+0.0044 / +0.0074) and loses on Tanks & Temples (−0.0034 / −0.0133) | no, removed — it existed to decouple #44 |
| 46 | Raising `--resolution-level` by one as the fast mode (with `--min-resolution 320`, else the level is clamped back) | **−0.0613** (Ignatius **−0.1804**, Barn −0.0392) at 0.34-0.57x wall and 0.33-0.35x faces | no |
| 47 | Exempting the opening cascade from `MaxRejects` (the rejections before a scale's first merit acceptance no longer end it; cap 8) | Tanks & Temples **−0.0012** mean: Ignatius +0.0031 but Truck −0.0013, Barn −0.0037, Meetingroom −0.0030 at 1.22x wall and 1.08x faces; Truck's coarse scale goes from 1 accepted evaluation to 13 at 0.01 px strides and loses (§2.8) | removed |
| 48 | Quadratic backtracking on a rejection (the trial retreats to the minimizer of the parabola through the reference, the model slope and the rejected S, clamped to [0.1, 0.5] of the step) | **−0.0043** mean on Tanks & Temples, every scene between −0.0037 and −0.0051; the first rejection retreats to 0.31 of the step instead of 0.5, so the scale reaches an acceptable stride within `MaxRejects` and keeps moving (Truck 30 evaluations instead of 14) | removed |
| 49 | End-of-scale relaxation (#43) with its schedule confound removed (#47's exemption), between scales only or after the last scale only | Barn's −0.0119 was the confound (now +0.0003 / −0.0018) but Ignatius −0.0024 / −0.0021 and the means −0.0010 / −0.0011; against the exemption alone +0.0003 mean, same Ignatius loss | removed |
| 50 | Removing the faces no image sees after the refinement (z-buffer render into every camera; the reference's visibility test, precision +0.054 on its Truck) | +0.0000 mean: 15 / 53 / 21 / 734 faces of 172-500 k are unseen on Truck / Barn / Ignatius / Meetingroom, the Delaunay graph-cut mesher never produced the undersides the reference's did | inert on OpenMVS meshes; kept opt-in for imported meshes as `--remove-unseen-faces` of TransformScene and ReconstructMesh |
| 51 | Coarse scales that prepare the mesh but run no evaluation (`Skip Coarse Evaluations`) | Ignatius **+0.0129** and Truck +0.0017 at 0.85-0.91x wall, but Barn **−0.0182**: its fine scale, opening from the unrefined surface, rejects four times and ends after 8 evaluations; mean −0.0009 (§2.8) | removed — the coarse scale's value is scene-dependent and nothing here can tell the cases apart |
| 52 | Warm start: every scale after the first opens at twice the eta the previous one ended with (`Step Search` 4) | +0.0011 mean (Truck +0.0030, Barn −0.0004) at 0.97x wall: the fine scale skips its 3-4 opening rejections and then runs longer where it can, so no speed is gained; under the +0.002 gate like #35 | removed; re-measured on the shipped one-pass preparation: Ignatius **−0.0094** (fine scale stops after 11 evaluations instead of 24 at the same S), mean −0.0021 — the coarse mesh wants the small strides the cascade leaves behind |
| 53 | Fixed opening: the first three evaluations of every scale applied unconditionally, the stride managed only afterwards (`Step Search` 8; the hybrid schedule) | **−0.42** mean, wall 16x: three 0.5 px steps along the raw gradient double S each and blow the mesh up. Without a per-vertex cap the idea is not viable, and #38 measured that the cap loses too | removed |
| 54 | Quadratic backtracking with Levenberg-Marquardt gain-ratio growth (`Step Search` 3; the lmfit-style rule on rho = measured/predicted decrease) | −0.0055 mean (Barn −0.0111, Ignatius −0.0098): the first-order model over-predicts the decrease 4-6x and does not converge to it as the step shrinks (rho 0.10-1.06 at 0.025 px), so rho ≈ 0.2 halves eta on most accepted steps | removed |
| 55 | Explicit pixel target for the decimation with a regularizing remesh in place of the two stacked relative heuristics (`Prepare Target Area` T, `Prepare Remesh Edge` −1) | T = 4 (today's density) −0.0020 mean at 0.93-1.23x faces: the shape-preserving QEM keeps the dense mesh's noise that the shipped strong remesh averages out; T = 8 / 12 −0.0004 / −0.0026 at 0.85x / 0.73x wall and 0.79x / 0.65x faces, the losses where faces were removed | **became the default** on the speed/size criteria, as one Clean pass with the target at half the face cap: +0.0001 mean at 0.85x faces and 0.83x wall against the previous defaults (§2.8); a coarser target on top of `--fast` still buys nothing (same 0.63-0.65x wall for −0.003..−0.004 more) |
| 56 | Remesh before the split (`Prepare Order` 1), so the split's vertices survive the remesh | −0.0012 mean, inert on three scenes, Barn −0.0026 with its fine scale ending at 20 evaluations instead of 26 | removed |
| 57 | Butterfly-interpolated 1-to-4 split (new vertices on the modified-butterfly surface through the coarse vertices and their wings) | −0.0006 mean: +0.0003/+0.0004 on Truck, Ignatius, Meetingroom and Barn −0.0034 (the same 20-evaluation stop) | removed |
| 58 | Bi-Laplacian relaxation after the last scale only (a post-smoothing that cannot touch the schedule), 0.45 and 0.25 | 0.45: three scenes +0.0004..+0.0025, Ignatius **−0.0062**; 0.25: +0.0007 mean, Ignatius −0.0005 — a low-pass filter the noisy scenes like and the detailed object pays for, never an always-win and under the gate at every fraction | removed |
| 59 | Unseen-face filter on the input mesh as well as the output (`--remove-unseen-faces-pre`) | −0.0013 mean, Barn −0.0033 by the stop-rule lottery; the input meshes have as few unseen faces as the outputs | removed (the post filter stays opt-in, #50) |
| 60 | `--max-face-area` swept 8/16/24/32/48/64 on Tanks & Temples, the first measurement of a value guessed when the option was added | 8: −0.0044 at 1.72x faces and 1.23x wall, worse on every axis at once; 24 / 32: −0.0053 / −0.0036 at 0.68x / 0.57x faces; 48 / 64: −0.0105 / −0.0175. On the three EPFL scenes the threshold-free metrics prefer coarser (24 / 32 improve mean accuracy 2.2 % / 2.6 %, F1 +0.004 / +0.008) | **default stays 16**: denser loses on quality, size and wall together, coarser trades quality for both past 32, and the F1 spread inside 0.5-2x of the default is the stop-rule lottery (§2.9) |
| 61 | The auto decimation's ratio floor (2 % of the input faces) | from cap 32 up most scenes ask for a smaller ratio, so the target was clamped and the cap stopped coarsening the mesh except through the split; the delivered density drifted from 75 % of nominal at the default to 61 % at cap 32 | **fixed**: floor 0.002, never binding at the default (ratios there are 3.5-13 %), delivered density now a constant 70-79 % of nominal at every cap |
| 62 | Preparation reaching the target more directly: the remesh pinned to the derived world edge length (the sampled faces convert px to scene units, √3/4·L² for an equilateral face), and one remesh carrying the mesh with no decimation at all | pinned −0.0032 mean at 0.96x faces (inert but for Ignatius, which loses); remesh-only −0.0010 at 1.11x faces and 1.06x wall, undershooting the target further (62 % of nominal against 75 %) | both removed; the two-step preparation of §1.2 stands |
| 63 | The fast preset rebuilt on the face area instead of the view budget | `--max-face-area 32 --simplify-tolerance 0.5` = −0.0046 mean at 0.40x faces and 0.78x wall, against the old `--max-views 4 --simplify-tolerance 0.5` at −0.0105 / 0.60x / 0.83x; adding `--max-views 4` back costs −0.010 more for 0.13x wall | **became `--fast`**: a coarser mesh cuts the mesh work and the per-pixel work together, a smaller view budget only the second (§2.9) |
| 64 | The reference's pair vote reimplemented from the tracks (8 px point cells, angle weight ((1+cosθ)/2)¹⁰·sinθ, per-cell normalisation, top-K with a transitive-redundancy test) in place of the per-image neighbour selection | 2.6-4.6 pairs per image against eight neighbours each, 0.69x wall, **−0.0100 mean F1** — and not the evaluation-count artefact: at equal counts Barn 0.6532 vs 0.6728, Meetingroom 0.4013 vs 0.4134, Ignatius 0.7486 vs 0.7795. Better than `--max-views 4` as a view-budget lever (same loss, 0.69x vs 0.83x wall) | removed: too few pairs under-constrain the surface, and the view budget is the wrong lever next to the face-area cap (§2.10) |
| 65 | The tightest-pair reduction taken as the median or the mean over the pairs that see a face, instead of the largest | not a rule change but a scale change (~5x smaller areas, so cap 16 means cap 80); density-matched at cap 2/3, mean +0.0020 and median −0.0003 at 1.31x faces, both swinging the evaluation counts hard (Truck 14 → 28), and median puts the working numbers at 1-2 px where whole-pixel counting stops measuring | removed, `max` stays: no evidence strong enough to redefine `--max-face-area`, whose 16 was tuned against it (§2.10) |
| 66 | `Progress Tol`, ending a scale on three evaluations below a larger relative decrease, as a fast-mode stop | same wall as a hard evaluation cap for twice the F1 cost (−0.0026 vs −0.0013), and untunable: 0.002 and 0.003 are bit-identical on all four scenes | removed; `Max Evaluations` stays at default 0 as a bound on run time, not a speed lever (§2.10) |
| 67 | The post-refinement decimation measured in the tightest-pair areas the split rule uses, instead of f/depth in each vertex's best view | T&T +0.0001 mean F1 (worst scene −0.0006) at 0.81x faces and 1.01x wall; EPFL accuracy **and** completeness identical to four decimals at 0.85x faces | **shipped unconditionally**, and the f/depth path deleted: foreshortening and occlusion are in a rasterized area and not in f/depth (§2.10) |
| 68 | Per-vertex sizing field for the preparation (`--adaptive-face-size`), each face graded to the area it projects to in the pair that refines it | +0.0007 mean F1 at 1.00x faces and 0.92x wall (the graded coarse scale is cheaper than the extra projection costs); EPFL accuracy +2.6 % / −1.3 % / unchanged. Neutral by construction on Tanks & Temples, whose viewing distance is near-constant | **default on**: the correct formulation at no cost, aimed at scenes with varying camera-to-surface distance that this benchmark does not contain (§2.10) |

Three of these carry a mechanism worth stating, because they look like independent ideas and are
not.

**Any per-vertex flattening of the gradient distribution costs F1** (#1, #14, #16, #17). Dividing
each vertex's direction by its own footprint and clipping the tail gives low-confidence vertices
the same authority as high-confidence ones. Four controls eliminated the alternatives: running the
old optimizer to the same evaluation count reversed the sign on 3 of 4 scenes; decoupling the stop
rule bought ≤ 0.0014 for 50-70 % more evaluations; disabling the accept/reject machinery was worse
still; and retuning the regularity weight was worse on both scenes tried. The shipped fix keeps the
median-based conversion factor **global** and drops the clamp.

**Every change that lets the coarse scale move the mesh more makes the final result worse** (#12,
#13, and by contradiction #20). A smaller initial step, or a reset reject streak, both let the
half-resolution scale accept steps it otherwise rejects, and both lose. Removing the coarse scale
entirely loses much more, so what it contributes is not its own motion but the decimate-and-
subdivide preparation and a second chance for the driver at the scale change.

**The oracle and the acceptance gate disagree on the bounded photometric terms, and the gate is
right** (#16, #17). The sign-vote arm looks like it almost fixes the fountain fixed point (it
leaves a displaced ground truth only 13 % worse instead of 141 % worse) for the same reason it
loses 0.08 on Ignatius: a bounded direction under a pixel-capped step rule barely moves the mesh at
all. "Moves less" reads as "converges better" on the oracle and as a large loss on real inputs.

---

## 5. Durable constraints and limitations

1. **Refinement cannot repair mesh density.** The `--decimate 0` (auto) step reduces the coarse
   mesh to the working face budget *before* any photo-consistency iteration — 5-10x on the Tanks &
   Temples scenes at level 1, 12-15x on the EPFL scenes — and no optimizer recovers what that
   removes. Ignatius keeps a −0.087 gap to its coarse input even with everything shipped here.
   Judging an optimizer by its distance to the input mesh rather than against a fixed baseline
   measures mostly this.
2. **Visibility has no scene-unit tolerance.** Both backends use the nearest-tap relative test
   `depth*1.0002 >= z` (§1.5), which is invariant under uniform scene scaling.
3. **The CPU backend is not bit-reproducible run to run** (per-pair contributions summed under a
   lock in completion order, unless `--max-threads 1`); the trajectory is chaotic at the vertex
   level — 1e-7 at the first iteration grows to 6.7e-4 by the 44th — so every CPU F1 carries a
   run-to-run term of 0.0001-0.0009. **The CUDA backend is bit-reproducible** (face-parallel
   accumulation, ordered gather, ordered score reduction, no float atomics).
4. **Wall-time cells must run alone.** A 7-10 GB working-set refinement measured beside another job
   reads high by a large factor: one cell measured 2.12x the baseline under contention and 0.84x
   when re-measured solo on the same binary with the same output. The harness serializes its own
   cells with a lock, but nothing stops other processes.
5. **`S` is photometric only.** The regularizer can raise it near convergence, producing rejections
   and an early stop — the intended end of a scale, not a defect. Adding the smoothness energy to
   `S` would break its dimensionless [0,2] range and its scale invariance.
6. **CPU/CUDA parity is a bug finder, not a gate.** 32-bit texture-unit bilinear weights and
   accumulation order keep the two backends ~0.1-2 % apart per vertex at the tail; chasing that
   residue further has no measured payoff. Large disagreements are bugs (six were found this way);
   small ones are the documented approximations.
7. **The scope is the PAMI 2012 formulation.** The decimate-and-remesh preparation is by design.
   Mechanism changes drawn from later literature (Sobolev-preconditioned directions, area
   normalization, robust per-pixel weights, depth-consistent window masks, adaptive regularization)
   are deliberately out of scope here and belong to a separate effort.

---

## 6. Open items

1. **The fountain fixed point.** Starting from the ground truth itself, the pipeline still lands
   about 0.011 away from it: the coarse-scale photometric iterations roughly triple an error the
   decimation leaves well inside τ. No in-scope variant fixed it — pair schedule, regularizer
   share, initial step, scale count, bounded/vote/plain-sum photometric terms, boundary modes, and
   every stencil including the energy-consistent one were all measured (§4). The remaining
   suspects change the mechanism and are out of scope (§5.7).
2. **Decimation policy.** Constraint §5.1 is the largest single term left on the object-like
   scenes. A policy that keeps a coarse mesh already finer than the pixel footprint has never been
   measured; it would have to pay for its face count under the wall and memory criteria.
3. **Mesh preparation is 38 % of the wall, and the density it sets is not a stated pixel size.** On Truck the
   two `Mesh::Clean` passes (auto-decimation, hole closing, edge-size remesh; one per scale) take
   10.8 s and 11.9 s of a 59.9 s refinement, invariant under `--max-views` and
   `--resolution-level` — which is why every speed lever saturates near 0.5x (§2.7). A faster
   preparation is the only remaining route to a fast mode worth the name; it is a halfmesh
   question, not a refinement one. §2.8 measured the preparation itself: the strong isotropic
   remesh after the split is what sets the density and it doubles as a denoiser, so replacing the
   stacked heuristics by an explicit pixel target loses at equal density and buys speed by
   delivering fewer faces — which is what the shipped default now does, 0.85x faces and 0.83x wall for a flat
   F1, and none at all on top of `--fast`. The first `Mesh::Clean` (the QEM
   decimation of the multi-million-face input) costs the same 15-25 s whatever it is asked to
   keep; a cheaper first pass is the untouched lever.
4. **The coarse scale's contribution is scene-dependent, and the stop rule is fragile.** With no
   evaluation on the coarse scale Ignatius gains +0.0129 and Barn loses −0.0182 (§2.8, #51), and
   on Barn every arm that perturbs the mesh by a few faces scores by how many evaluations its
   fine scale gets before four consecutive rejections end it. A schedule that decides per scene
   whether the coarse scale should move, and a stop rule that does not turn one extra rejection
   into a 0.018 loss, are the largest levers left on these scenes; both need a signal that is not
   the benchmark's F1.
5. **The Ceres arm remains opt-in.** §2.5 records why: on both ground-truth scenes it reaches a
   lower energy than the stepper and never a better F1.
