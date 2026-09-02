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
wrappers). Two files both backends share outright: `libs/MVS/SceneRefineCommon.h/.cpp` (shared
scalar math, the `OPTREFINE` configuration space, per-view image/mask preparation) and
`libs/MVS/SceneRefineStep.h/.cpp` (the vertex-position stepper — one implementation, no CUDA twin
to drift). There is no `SceneRefine.h`; `MeshRefine`/`MeshRefineCUDA` are translation-unit-local
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
rather than silently ignored.

### 1.2 Multi-scale subdivision loop

`Scene::RefineMesh`/`RefineMeshCUDA` run `nScales` passes coarse-to-fine (`--scales`, default 2;
`--scale-step`, default 0.5). At loop index `nScale` (0-based, 0 = first = coarsest):

```
scale = fScaleStep ^ (nScales - nScale - 1)   // image downsample factor
step  = 2 ^ (nScales - nScale)                // used only for the blur sigma below
sigma = 0.12 * step + 0.2                     // pre-blur before resizing/gradient
```

identical on both backends. With the shipped defaults: scale 0 runs at half resolution with
`sigma=0.68`, scale 1 (finest) at full resolution with `sigma=0.44` px — a noise-robust pre-blur
ahead of the derivative stencil (§1.3), not a negligible one.

Each scale re-inits images (`InitImages`, one worker per view): the shared `PrepareRefineImage`
(`SceneRefineCommon.cpp`) loads, gray-converts, Gaussian-blurs at the scale's `sigma` and resizes;
`ComputeRefineImageGradient` (§1.3) builds the derivative image both backends read; then
`PrepareRefineImageMask` builds the per-view keep-mask at the working size. `ListVertexFacesPre`
re-lists incident faces, then `SubdivideMesh` runs — CPU and CUDA remain independent but
mechanically identical: optionally decimates (`--decimate`, default 0 = auto: projects the mesh
into every camera, measures the median face area across image pairs, decimates only if that median
exceeds `--max-face-area` (default 16 px²) by more than 6x), runs `Mesh::Clean` with a negative
(relative) target edge length `-2.25` so the [0.5x, 4x] edge-length remesh rides the same `Clean`
pass (`remeshIterations=10`), then subdivides any face whose projected area in the tightest camera
pair exceeds `--max-face-area` (`Mesh::Subdivide`, 1-to-4 split). The log line
`"Mesh subdivided: %u/%u -> %u/%u vertices/faces"` is identical on both backends.

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
   pixel-rejection machinery; `dzRaw`/`conf` exist only for the parity debug export (§1.6).
3. **Per-pixel photometric gradient.** `MeshRefine::ComputePhotometricGradient` (CPU) and
   `kernelAccumulateFacePhoto` (CUDA) compute the face normal `N`, the camera-A ray `dA`, and
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
   **deterministic**: `kernelAccumulateFacePhoto` is face-parallel — one thread per mesh face walks
   its clipped bounding box in fixed scan order, reducing the corner sums, pixel count and footprint
   minimum in registers, no atomics — and `kernelGatherVertexPhoto` is vertex-parallel, walking each
   vertex's flattened incident-face list (`Mesh::ListIncidentFaces` order) to sum the contribution
   and absorb the `photoGradNorm += 1`; `kernelFinalizePhotoGrad` resolves the sentinel exactly like
   the CPU's post-loop pass. `photoGrad[v]/photoGradNorm[v]` — the pair-count average — is what the
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

The final per-vertex gradient combines them (`ScoreMesh`/`CombineGradients` — only computed when a
caller asks for the combination; the stepper reads the terms separately):

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
CUDA's two-pass `kernelProjectMesh` resolves depth ties the same way the CPU's face-list traversal
order does, and is itself deterministic run to run), and the stepper (`SceneRefineStep.cpp`, one
implementation). What legitimately differs:

| Aspect | CPU | CUDA |
|---|---|---|
| Camera projection precision | double (`Camera::TransformPointW2C`) | float (`MVS::CUDA::Camera`, built once per call by `MakeCUDACamera`) — measured bit-identical face maps on the `Tiny` fixture despite the precision drop |
| Photometric accumulation order | per-pair, under a lock, in whatever order threads complete — not bit-reproducible run to run unless `--max-threads 1` | face-parallel accumulate + vertex-parallel gather over a fixed order, no atomics — **bit-reproducible** |
| Planar-vertex removal | implemented (§1.7) | refused at the entry (a loud error, not a silent no-op) — caller falls back to CPU |
| Ceres arm (`--gradient-step 0`) | implemented, gated on `_USE_CERES` | refused at the entry, same fallback |
| Float reassociation | `cv::boxFilter`/`filter2D`, MSVC `/fp:precise`, no FMA | explicit-rounding intrinsics (`__fmul_rn` etc.) so nvcc cannot silently fuse an FMA the CPU wouldn't — residual disagreement ~0.1-2% per vertex at the tail, the documented approximation floor |

An env-var-gated diagnostic, `RefineDebug` (`SceneRefineCommon.h/.cpp`, `OMVS_REFINE_DEBUG_DIR`/
`_PAIR`, no CLI flag), dumps one image pair's per-vertex gradients and per-pixel maps from either
backend for direct comparison — the tool that found the six CUDA-only defects (window size, `dZNCC`
form, image-gradient sampling, border margins, bi-Laplacian valence at a boundary neighbour,
back-face winding) now closed.

### 1.7 Optimization schedule

Both backends drive the same shared stepper, `MeshRefineStep` (`SceneRefineStep.h/.cpp`). The
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

**Constants** (`SceneRefineStep.h`, pixel/ZNCC quantities, deliberately not CLI-exposed):
`StepMax = 1` px (`eta_max`), `StepGrow = 1.1`, `StepShrink = 0.5`, `StepStop = 0.05` px
(median-step-at-full-stride convergence floor), `ProgressTol = 1e-3` (relative `S` decrease counted
as stalled), `Kappa = 2`, `Patience = 3` (consecutive stalled iterations that end the scale),
`MaxRejects = 4` (consecutive rejections that end the scale), `MinIters = 3` (no stop rule before
this many ACCEPTED iterations). `ProgressTol` and `Patience` were swept and sit on a flat optimum
(§2.4).

**Accept/reject** (`OPTREFINE::nOptimizer`, default 0 = bold driver; 1 = a fixed-step control arm
that never rejects and never grows/shrinks `eta`, kept only to isolate whether the accept/reject
machinery matters). An evaluation whose `S` is worse than the last accepted `S` is REJECTed: every
vertex moves back to exactly `v_prev + stepPrev/2` (undoing half the offending step),
`eta *= StepShrink`, and the scale STOPs after 4 consecutive rejections. An accepted evaluation
becomes the new reference, resets the reject streak, grows `eta = min(eta*1.1, StepMax)`, and the
scale STOPs once `numAccepted >= MinIters` and `Patience` consecutive iterations failed to improve
`S` by `ProgressTol`, or once the median per-vertex step **at a full stride**
(`medianPx * StepMax/eta`, not the step just taken — an `eta` ratcheted down by repeated
accept/reject cycles would otherwise report false convergence) drops below `StepStop`.

**Per-scale two-phase schedule.** `--gradient-step N.s` sets `N = floor(...)` and the initial step
`eta0 = (fGradientStep-N)*10` px (shipped default `45.05` → `N=45`, `eta0=0.5` px; both validated at
entry — `eta0` must lie in `(0, StepMax]`). The per-scale evaluation **cap** is
`max(N/(nScale+1), 8)` (`nScale` 0-based, coarsest first — the coarse scale gets the larger cap);
it is a safety net, not the operating stop: raising `N` from 45 to 1000 changes no result (§2.4).
**Phase A** runs up to `cap` evaluations at the caller's `--rigidity-elasticity-ratio` (default
0.9), with the CPU-only planar-vertex hook eligible from the 4th accepted evaluation onward, every
3rd accepted evaluation thereafter, provided more than 5 evaluations remain in the phase's budget.
**Phase B** runs a fresh, smaller budget `capB = max(3, 3*numAcceptedInPhaseA/7)` — a 70/30 split
taken against phase A's ACCEPTED count rather than its raw budget, since a rejected evaluation buys
no convergence — at `rho = 1` (pure elasticity) with the planar hook off; `eta` and the
accepted-`S` references carry over from phase A unchanged, only `MeshRefineStep::ResetStall()` runs
between phases (the stall counter resets, the reject streak does **not** — resetting it too
measured −0.0048 mean F1, §4). Either phase's loop exits early on `MeshRefineStep::STOP`.
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

`--gradient-step 0` selects a first-order Ceres solve instead of the stepper, gated on `_USE_CERES`
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

Defaults only: two scales, `--gradient-step 45.05` (bold driver, initial step 0.5 px), regularity
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

The arms and the extra stop rules are therefore removed from the tree; `OPTREFINE::nOptimizer`
keeps only the shipped bold driver and the fixed-step control.

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

Three conclusions. The per-scale evaluation cap is **not** binding — raising `--gradient-step` from
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

## 3. How these numbers were produced

The harness lives under the gitignored `bench/` tree; this section records what it does, so a
number in §2 or §4 can be reproduced or challenged.

**Scoring.** `bench/run_refine.py` runs one `RefineMesh` cell per (scene, variant, backend),
samples 10 M area-uniform seeded points from the result inside the frozen scene-to-ground-truth
crop, and scores precision/recall/F1 at the scene's tolerance τ. Tanks & Temples scenes go through
the official toolbox; the ground-truth-mesh scenes go through `bench/eval_mesh2mesh.py` (exact
point-to-triangle distance to the ground truth for accuracy; 2 M ground-truth samples restricted to
surface some scene camera sees for completeness). `bench/refine_log.py` parses the per-scale,
per-iteration trace out of the application log — score, step, applied median step in pixels,
accept/reject — so a verdict can cite the trajectory and not just the final number. All evaluation
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
| 4 | Fixed-step control arm (never rejects, never adapts η) | −0.0891 on the ground-truth scenes | kept as a control only |
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
6. **The CPU/CUDA parity diagnostic is a bug finder, not a gate.** 32-bit texture-unit bilinear
   weights and accumulation order keep the two backends ~0.1-2 % apart per vertex at the tail;
   chasing that residue further has no measured payoff. Large disagreements are bugs (six were
   found this way); small ones are the documented approximations.
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
3. **The Ceres arm remains opt-in.** §2.5 records why: on both ground-truth scenes it reaches a
   lower energy than the stepper and never a better F1.
