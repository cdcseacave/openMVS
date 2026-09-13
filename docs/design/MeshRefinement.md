# Mesh Refinement

## 1. Purpose and scope

`RefineMesh` implements the Vu et al. (PAMI 2012) variational mesh refinement: alternate a
multi-scale subdivision pass with a gradient-descent optimization that pulls the surface toward
photo-consistency while a Laplacian regularizer keeps it smooth.

Two implementations exist side by side: the CPU path in `libs/MVS/SceneRefine.cpp`
(`MeshRefine`/`Scene::RefineMesh`) and the CUDA path split across `libs/MVS/SceneRefineCUDA.cpp`
(`MeshRefineCUDA`/`Scene::RefineMeshCUDA`) and `libs/MVS/SceneRefineCUDA.cu`/`.inl` (device kernels
and launch wrappers). One file both backends share outright: `libs/MVS/SceneRefineCommon.h/.cpp` —
the scalar math both must compute identically, the `OPTREFINE` configuration space, the per-view
image/mask preparation, the shared mesh-preparation template (`PrepareRefineMesh`), and the
vertex-position stepper (`MeshRefineStep`). There is no `SceneRefine.h`; `MeshRefine`/
`MeshRefineCUDA` are translation-unit-local classes. `apps/RefineMesh/RefineMesh.cpp` is the CLI
driver.

The CPU path is the reference implementation. The CUDA path is required to reach the same surface
(§2.9 lists where it legitimately differs); a CUDA/CPU disagreement outside the documented
floating-point tolerance is a bug, not an acceptable variant.

Scope is deliberately the PAMI 2012 formulation: photo-consistency scored on a masked 7x7 ZNCC
window, an umbrella/bi-Laplacian regularizer, and a decimate-and-remesh mesh preparation. Mechanism
changes drawn from later literature (Sobolev-preconditioned directions, area normalization, robust
per-pixel weights, depth-consistent window masks, adaptive regularization) are out of scope for this
stage.

## 2. Algorithm as implemented

### 2.1 Entry point and backend selection

`apps/RefineMesh/RefineMesh.cpp:main` loads the scene, optionally attaches per-image masks
(`--mask-path`, `--ignore-mask-label`), loads (or requires) the input mesh, and picks a backend via
`SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs)` (true for an empty `--gpu-device`,
`-2`, `cpu` or `none`; false for `-1` = best GPU or `>=0` = device id list). Refinement mutates the
mesh in place, so `main` snapshots `scene.mesh.vertices`/`faces` before a CUDA attempt; if
`Scene::RefineMeshCUDA` returns `false`, it restores the snapshot and runs `Scene::RefineMesh` (CPU)
on the original input.

`--mesh-file/-m` defaults to `<input-file-without-extension>.ply` only when the archive type is
`ARCHIVE_MVS`; with a `.mvs` input that resolves to the dense point cloud's `.ply`, not a mesh, and
refinement fails with "empty initial mesh" — `-m` must be given explicitly in that case. The refined
mesh is always written to `<out-stem><export-type>` (default `.ply`); the `.mvs` sidecar is written
only when the archive type isn't `ARCHIVE_MVS` or the input wasn't `Scene::SCENE_INTERFACE`.

`OPTREFINE::init()`/`update()` load the `OPTREFINE` defaults and merge in `--refine-config-file` if
given. The CLI always wins for an option actually passed on the command line; an option whose CLI
default would otherwise silently overwrite what the file set (`--ignore-mask-label`,
`--simplify-tolerance`, `--adaptive-face-size`) is applied only when
`variables_map[...].defaulted()` is false, and a config file naming an unrecognized option is
refused by name rather than ignored. `--fast` fills in `--max-face-area 32` and
`--simplify-tolerance 0.5` only where those options were not explicitly given.

### 2.2 Multi-scale subdivision loop

`Scene::RefineMesh`/`RefineMeshCUDA` run `--scales` passes coarse-to-fine (default 2,
`--scale-step` 0.5). At loop index `nScale` (0-based, 0 = coarsest):

```
scale = scaleStep ^ (scales - nScale - 1)   // image downsample factor
step  = 2 ^ (scales - nScale)               // used only for sigma below
sigma = 0.09 * step + 0.15                  // pre-blur before resizing/gradient
```

identical on both backends. With the shipped defaults, scale 0 runs at half resolution with
`sigma=0.51`, scale 1 (finest) at full resolution with `sigma=0.33`. `sigma`'s multiplier is coupled
to `MeshRefineStep::StepGrow` (§2.7) — the two are one operating point, not two independent knobs.

Each scale re-inits images (`InitImages`, one worker per view, `MeshRefine::ThInitImage`): the
shared `PrepareRefineImage` (`SceneRefineCommon.cpp`) loads, gray-converts, Gaussian-blurs at the
scale's `sigma` and resizes; `ComputeRefineImageGradient` builds the per-view derivative image both
backends read (skipped when `OPTREFINE::nImageGradient == 3`, which samples the bilinear
interpolant directly instead); `PrepareRefineImageMask` builds the per-view keep-mask. `SubdivideMesh`
then runs the shared mesh preparation (§2.3), and `ListVertexFacesPost` lists incident/boundary
vertices before the optimization phase (§2.7) runs.

### 2.3 Mesh preparation

`MeshRefine::SubdivideMesh`/`MeshRefineCUDA::SubdivideMesh` both forward to the template
`MVS::PrepareRefineMesh` (`SceneRefineCommon.h`), so the two backends cannot drift here.

At the first scale (`fDecimate == 0`, the `--decimate` auto default) the mesh is projected into
every camera (`ListCameraFaces`, `ListFaceAreas`) to get, per face, its *tightest-pair area*: the
larger over the refinement's image pairs of the smaller of the pair's two rasterized pixel counts (0
for a face no pair sees). `SampleSeenFaceArea` additionally computes the analytic mean seen area over
a stride sample, since the rasterized counts saturate at 1 px on the sub-pixel faces of a dense
input mesh. The mesh is decimated (`Mesh::Clean`, `simplifyTarget` = the ratio, floor 0.002) to a
target mean tightest-pair area of half `--max-face-area` in that scale's pixels, then regularized in
the same `Clean` pass: `maxHoleEdges = --close-holes` (default 30), and, when `--ensure-edge-size >
0`, an isotropic remesh (`edgeLength = -1`, a band around the mesh's own current mean edge;
`remeshIterations = 10`) that evens the vertex rings out without moving the density.

With `--adaptive-face-size` (default on) the density-setting remesh instead grades a per-vertex
target: after the plain decimation, the mesh is re-projected once more and every face states its own
scale (`SeenAreasToEdgeTargets`: a face covering `seenArea` pixels for its world area implies a world
area of `area*targetArea/seenArea` for the target pixel count, `targetArea` = half `--max-face-area`;
a vertex averages the equilateral edge length of its incident seen faces). That field is passed to
`Mesh::Clean` as `CleanParams::vertexSizing`, in its own call, separate from the decimation (§4 #4).
On a surface seen from roughly constant camera-to-surface distance the field is flat and this reduces
to the uniform remesh.

Every scale then subdivides: `Mesh::Subdivide(maxAreas, maxArea)` splits 1-to-4 every face whose
tightest-pair projected area exceeds `--max-face-area` (default 16 px² at that scale's resolution),
and, when `--ensure-edge-size > 1` (force mode), the edge-band remesh above also runs after the
split on every scale, not only the first.

### 2.4 Photo-consistency energy

Refinement pairs are gathered by `MVS::SelectRefineNeighbors` (`SceneRefineCommon.cpp`): recover
`Image::neighbors` via `Scene::SelectNeighborViews` if the mesh was handed to the refiner directly,
then `Scene::FilterNeighborViews` at fixed thresholds (min shared area 0.1, scale range [0.2, 3.2],
angle range [2.5°, 45°]), capped at `--max-views` (default 8).

For every ordered pair `(A,B)`, `MeshRefine::ThProcessPair` (CPU) / the CUDA per-direction kernels do,
per pixel of A:

1. **Warp.** `ImageMeshWarp` projects A's depth into B; a pixel masked out of A never seeds a
   sample. A sample is kept only if the visibility test accepts (§2.6) and B's own keep-mask accepts
   the same rounded tap. Rejected/invalid pixels are zero-filled, not seeded with a copy of A.
2. **Masked local window statistics, 7x7 (`Refine::HalfSize=3`, `SceneRefineCommon.h`).**
   `ComputeWindowStats` reduces six masked box-filtered sums (over valid pixels only) through
   `Refine::WindowStatsFromSums`: rejects a pixel if its window holds fewer than
   `Refine::MinWindowCount` (25) valid samples, floors both variances at `Refine::VarFloor` (1e-4),
   and rejects a pixel pair whose means differ by more than `OPTREFINE::fGateMeanDiff` (default 0.4)
   or whose variance ratio exceeds `OPTREFINE::fGateVarRatio` (default 8). `Refine::ZnccAndDerivative`
   then forms `zncc`, its derivative `dzncc`, and the reliability weight
   `conf = min(varA,varB)/(min(varA,varB)+ReliabilityVarOffset)` (`ReliabilityVarOffset=0.0015`).
   The pair-direction's `sumR += conf`, `sumRZ += conf*(1-zncc)` accumulate into the reliability-
   weighted score `S = sumRZ/sumR` (§2.7).
3. **Per-pixel photometric gradient.** `ComputePhotometricGradient` computes the face normal `N`,
   camera-A ray `dA`, `Nd = N.dA`, and skips the pixel if `Nd > -0.1` (a grazing/back-face gate).
   It back-projects to 3D, projects into B with a Jacobian (`MeshRefine::ProjectVertex`), samples B's
   image-gradient stencil, and forms `sg = (gB . (J . dA)) * dzncc * RegularizationScale / Nd`,
   distributed to the face's three vertices by barycentric weight.
   `RegularizationScale = avgDepthA*avgDepthB / (focalA*focalB)` (identical formula on both backends)
   converts the image-space gradient into a 3D-consistent one.
4. **Image gradient stencil.** Selected by `OPTREFINE::nImageGradient` (default 1, central
   differences; 0 = separable 3x5; 2 = Sobel-3; 3 = the derivative of the bilinear interpolant the
   warp itself samples, computed on demand instead of from a precomputed stencil image).
5. **Per-vertex accumulation.** `photoGrad[v]` sums `sg` over every touching pixel; `photoGradNorm[v]`
   (`c_v`) counts pair-directions that touched `v`, once per direction, not per pixel;
   `footprint[v]` (`depthA/focalA`) is min-reduced over every contributing pixel/direction, resolved
   from a sentinel to 0 for a vertex no direction saw — `footprint[v] > 0` iff `photoGradNorm[v] > 0`
   is an invariant checked on both backends. `photoGrad[v]/photoGradNorm[v]` is the pair-count
   average `g_v` the stepper (§2.7) reads.
6. **Boundary vertices** receive the photometric term like any interior vertex; only the smoothing
   terms (§2.5) are zeroed there.

### 2.5 Regularization term

`ComputeSmoothnessGradient1` computes the umbrella-operator Laplacian `L(v) = mean(1-ring
neighbors) - v`, zeroed at boundary vertices. `ComputeSmoothnessGradient2` forms the Hernandez
(2004, p.105) "level 2" operator, a valence-normalized combination of `L` over the same ring, also
zeroed at the boundary — both use the neighbour's true valence, including a boundary neighbour's.
The combined per-vertex gradient (computed only when a caller needs it summed — the Ceres arm and
the planar-vertex hook; the stepper reads the terms separately):

```
ratioRigidityElasticity >= 1:   g_v + smoothGrad2[v]*weightRegularity
ratioRigidityElasticity <  1:   g_v + smoothGrad2[v]*elasticity - smoothGrad1[v]*rigidity
                                 rigidity   = (1 - ratio) * weightRegularity
                                 elasticity =       ratio * weightRegularity
```

`--regularity-weight` (`w`, default 0.2) and `--rigidity-elasticity-ratio` (`rho`, default 0.9) are
validated at the entry point of `Scene::RefineMesh`/`RefineMeshCUDA` against the stepper's
explicit-flow stability bound `w * MeshRefineStep::StepMax <= 1` rather than left to fail inside the
optimizer.

### 2.6 Visibility test (occlusion)

`MeshRefine::IsDepthSimilar` (CPU) and the equivalent CUDA warp kernel both round the projected
point in B to its nearest depth-map texel and accept it iff `depth > 0 && depth*1.0002f >= z` — a
one-sided occlusion test, exactly invariant under a uniform scene rescale. There is no scene-unit
tolerance. Both backends also test B's keep-mask at the same rounded tap, and range-check the
projected coordinate in float before any integer conversion (a grazing projection can put the target
far outside the image; converting to `int` first can wrap a saturated value past a `<` check).

### 2.7 Optimization schedule

Both backends drive the same shared stepper, `MeshRefineStep` (`SceneRefineCommon.h/.cpp`), working
in pixels (through each vertex's footprint `s_v`) and in ZNCC (`S`), so its trajectory does not
depend on scene scale, image resolution, or pair count.

**Per-evaluation update**, `rho` the phase's rigidity/elasticity ratio, `w` the regularity weight:

```
gamma_v = |g_v| / s_v                                     (c_v >= 2 only)
m       = median gamma_v over vertices with c_v >= 2       (computed ONCE per scale, then held)
P_v     = g_v / (Kappa * m)                                (zero if c_v < 2 or m == 0)
R_v     = rho * bilap_v - (1 - rho) * lap_v
D_v     = -eta * (P_v + w * R_v)
```

`m` is a global conversion factor fixed at the scale's first evaluation, not a per-vertex one — the
median seen vertex moves `eta/Kappa` px at the first evaluation and every other vertex moves in
proportion to its own gradient.

**Constants** (`MeshRefineStep`, not CLI-exposed, listed in §3): `StepInit`, `StepMax`, `StepGrow`,
`StepShrink`, `StepStop`, `ProgressTol`, `Kappa`, `Patience`, `MaxRejects`, `MinIters`, `MaxIters`.

**Accept/reject.** An evaluation whose `S` is worse than the last accepted `S` is rejected: every
vertex moves back to `v_prev + stepPrev/2`, `eta *= StepShrink`, and the scale stops after
`MaxRejects` consecutive rejections. An accepted evaluation becomes the new reference, resets the
reject streak, grows `eta = min(eta*StepGrow, StepMax)`; the scale stops once `numAccepted >=
MinIters` and `Patience` consecutive evaluations fail to improve `S` by `ProgressTol`, or once the
median per-vertex step at a full stride (`medianPx * StepMax/eta`) drops below `StepStop`.

**Per-scale two-phase schedule.** `eta` resets to `StepInit` every scale. The per-scale cap is
`MeshRefineStep::Budget(nScale) = OPTREFINE::nMaxEvaluations > 0 ? nMaxEvaluations :
max(MaxIters/(nScale+1), 8)` (`nScale` 0-based, coarsest first) — a safety net, not the operating
stop rule. **Phase A** runs up to that cap at the caller's `--rigidity-elasticity-ratio` (default
0.9); on the CPU only, the planar-vertex hook (below) becomes eligible from the 4th accepted
evaluation onward, every 3rd accepted evaluation thereafter, provided more than 5 evaluations remain
in the phase's budget. **Phase B** runs a fresh budget `capB = max(3, 3*numAcceptedInPhaseA/7)` at
`rho = 1` (pure elasticity) with the planar hook off; `eta` and the accepted-`S` reference carry over
from phase A unchanged (only `MeshRefineStep::BeginSecondPhase()` runs between phases: it hands back
the budget and resets the stall counter — the reject streak does not reset). Either phase's loop
exits early on `MeshRefineStep::STOP`.

**Planar-vertex removal (CPU only, `--planar-vertex-ratio`, default 0 = disabled).** When eligible
and the evaluation was applied, every vertex whose combined gradient magnitude and smoothing
residual both fall below `fThPlanarVertex * footprint[v] * medianViewFocalLength` is removed via
`Mesh::RemoveVerticesAndFill`, run *after* the step (removal permutes the vertex indexing, so
everything the stepper indexes by vertex must already have been consumed). Only vertices whose
one-rings are pairwise disjoint and do not touch the mesh's open boundary are removed in one
evaluation — `RemoveVerticesAndFill` spans a hole only when its boundary is one simple loop, so the
rest of a planar patch goes in later evaluations. `MeshRefineStep::TopologyChanged()` runs after a
removal so the next evaluation is not judged against an `S` measured on a different vertex set.
CUDA refuses `--planar-vertex-ratio > 0` at the entry with an error rather than silently skipping it.

### 2.8 Post-refinement simplification

When `OPTREFINE::fSimplifyTolerance > 0` (default 0.25 px), both backends end with
`MeshRefine::SimplifyMesh`/`MeshRefineCUDA::SimplifyMesh`: re-project the final mesh
(`ListCameraFaces`, `ListFaceAreas`), convert the same tightest-pair seen areas to per-vertex pixels-
per-scene-unit (`SeenAreasToPixelFactors`), then decimate within that reprojection tolerance
(`SimplifyMeshWithinTolerance`, `Mesh::Clean` with a per-vertex `vertexMaxError` bound derived by
`PixelFactorsToErrorBounds`). This measures "how big is this face on screen" the same way the
preparation's split rule does, so it never keeps a face no refinement pair can see.

### 2.9 Ceres arm (opt-in, CPU only)

`--use-ceres` (default off, gated on `_USE_CERES`/`OpenMVS_USE_CERES`) replaces the stepper with a
first-order Ceres solve. `MeshRefine` runs in "energy mode": `ScoreMesh` returns the exact energy
`E = Sum_pairs RegScale_p * Sum_pixels r*(1-ZNCC) + w * (1/2) * Sum_{v interior} ||L(v)||^2` and
fills its exact gradient — the raw per-vertex photometric sum (not divided by `photoGradNorm`) plus
`w * L^T L * v` (`ComputeSmoothnessGradientLtL`, the transpose of the umbrella operator). The
photometric term's exact per-pixel derivative comes from `ComputeWindowStats(...,
bExactDerivative=true)`, always using the bilinear-interpolant derivative
(`MeshRefine::BilinearGradient`) regardless of `OPTREFINE::nImageGradient`, since that is what makes
the gradient consistent with the energy. Two normalizations, fixed at each scale's calibration
evaluation, keep the arm's `w` meaning the same as the stepper's: each pair-direction is weighted by
its scale-start reliability sum over its current one, and the photometric term is divided by `Kappa`
times the scale-start median of `|g_v|/s_v`.

`ceres::MeshProblem` wraps this as a `ceres::FirstOrderFunction`, solved with
`GradientProblemSolver` at the Ceres 2.2 defaults (L-BFGS rank 20, Wolfe line search, no
Oren-Luenberger scaling), `max_num_iterations=100` (a safety net), `function_tolerance=1e-4` (the
actual stop), `gradient_tolerance=parameter_tolerance=0`. The arm refuses `--alternate-pair 1` and
`--planar-vertex-ratio > 0` (the parameter count is fixed for the whole solve), forces
`--rigidity-elasticity-ratio` to 1, and is exempt from the stepper's `w <= 1` stability cap.
`Evaluate` keeps the lowest-cost surface any evaluation reached, so a solve ending in `FAILURE` does
not abort the whole refinement. A finite-difference gate, `Scene::RefineMeshEnergyProbe`, validates
the analytic gradient against a numerical one and is exercised by the test suite.

### 2.10 CPU/CUDA: what is shared vs what legitimately differs

Identical by construction: the shared scalar header (`SceneRefineCommon.h`), the 7x7 window, the
visibility test, the image-gradient stencil (built once, host-side, shared by both), the rasterizer
convention (front faces only, perspective-correct barycentric coordinates), and the stepper
(`MeshRefineStep`, one implementation, §2.7). What legitimately differs:

| Aspect | CPU | CUDA |
|---|---|---|
| Camera projection precision | double (`Camera::TransformPointW2C`) | float (`MVS::CUDA::Camera`) |
| Rasterizer input | an octree-frustum-culled face list per camera, rebuilt every evaluation | the whole mesh into every view; the kernel rejects what a view does not see |
| Photometric accumulation order | per-pair, under a lock, in thread-completion order — not bit-reproducible run to run unless `--max-threads 1` | face-parallel accumulate + vertex-parallel gather over a fixed order — bit-reproducible |
| Planar-vertex removal (§2.7) | implemented | refused at the entry (loud error); caller falls back to CPU |
| Ceres arm (§2.9) | implemented | refused at the entry, same fallback |
| Float reassociation | `cv::boxFilter`/`filter2D`, no FMA | explicit-rounding intrinsics so nvcc cannot silently fuse an FMA the CPU wouldn't; residual disagreement ~0.1-2% per vertex at the tail |

CUDA also prefetches the next scale's images on a host thread while the current scale's optimization
runs (`MeshRefineCUDA::PrefetchImages`), so a scale switch pays only the vertex/image upload.

## 3. Parameters and defaults

| CLI option | struct field | default | meaning |
|---|---|---|---|
| `--mesh-file/-m` | `OPT::strMeshFileName` | `<input>.ply` if archive is MVS, else required | mesh to refine |
| `--mask-path` | `OPT::strMaskPath` | empty | folder of `<image>.mask.png` files |
| `--ignore-mask-label` | `OPTREFINE::nIgnoreMaskLabel` | `-1` (disabled) | mask label value to drop |
| `--resolution-level` | `OPT::nResolutionLevel` | `0` | image downscale steps before refinement |
| `--min-resolution` | `OPT::nMinResolution` | `640` | floor on the downscaled image's longest side |
| `--max-views` | `OPT::nMaxViews` | `8` | neighbor-view cap per image (`SelectRefineNeighbors`) |
| `--decimate` | `OPT::fDecimateMesh` | `0` (auto) | input decimation ratio; `1` disables it |
| `--close-holes` | `OPT::nCloseHoles` | `30` | max boundary edges of a hole the prep closes |
| `--ensure-edge-size` | `OPT::nEnsureEdgeSize` | `1` (auto) | `0` disabled, `1` first scale only, `2` every scale |
| `--max-face-area` | `OPT::nMaxFaceArea` | `16` (px²) | split threshold; decimation target is half this |
| `--adaptive-face-size` | `OPTREFINE::bAdaptiveFaceSize` | `true` | per-vertex sizing field instead of one scalar density |
| `--simplify-tolerance` | `OPTREFINE::fSimplifyTolerance` | `0.25` (px) | post-refinement decimation tolerance; `0` disables it |
| `--fast` | preset | `false` | fills unset `--max-face-area 32`/`--simplify-tolerance 0.5` |
| `--scales` | `OPT::nScales` | `2` | coarse-to-fine passes |
| `--scale-step` | `OPT::fScaleStep` | `0.5` | per-scale image scale factor |
| `--alternate-pair` | `OPT::nAlternatePair` | `0` (both) | `1` alternate, `2` left only, `3` right only |
| `--regularity-weight` | `OPT::fRegularityWeight` (`w`) | `0.2` | photo-consistency vs. regularization balance |
| `--rigidity-elasticity-ratio` | `OPT::fRatioRigidityElasticity` (`rho`) | `0.9` | phase-A umbrella/bi-Laplacian blend |
| `--use-ceres` | `OPT::bUseCeres` | `false` | Ceres line search instead of the stepper (CPU only) |
| `--planar-vertex-ratio` | `OPT::fThPlanarVertex` | `0` (disabled) | planar-vertex removal threshold (CPU only) |
| `--refine-config-file` | `OPT::strRefineConfigFileName` | empty | optional `OPTREFINE` overrides file |
| (config file only) | `OPTREFINE::nImageGradient` | `1` (central) | `0` 3x5 separable, `2` Sobel, `3` bilinear interpolant |
| (config file only) | `OPTREFINE::fGateMeanDiff` | `0.4` | reject a window pair whose local mean differs more |
| (config file only) | `OPTREFINE::fGateVarRatio` | `8.0` | reject a window pair whose variance ratio exceeds this |
| (config file only) | `OPTREFINE::nMaxEvaluations` | `0` (convergence decides) | hard per-scale evaluation cap |
| (fixed, `MeshRefineStep`) | `StepInit` | `0.5` px | `eta` at the start of every scale |
| (fixed, `MeshRefineStep`) | `StepMax` | `1.0` px | `eta` ceiling (never observed to bind) |
| (fixed, `MeshRefineStep`) | `StepGrow` / `StepShrink` | `1.05` / `0.5` | `eta` growth on accept / shrink on reject |
| (fixed, `MeshRefineStep`) | `StepStop` | `0.05` px | full-stride median-step convergence floor |
| (fixed, `MeshRefineStep`) | `ProgressTol` | `1e-3` | relative `S` decrease counted as stalled |
| (fixed, `MeshRefineStep`) | `Kappa` | `2` | median seen vertex moves `eta/Kappa` px at the first evaluation |
| (fixed, `MeshRefineStep`) | `Patience` / `MaxRejects` / `MinIters` | `3` / `4` / `3` | stall / reject / minimum-accepted stop conditions |
| (fixed, `Refine`) | `HalfSize` / `WindowSize` | `3` / `7` | ZNCC window half-size / size |
| (fixed, `Refine`) | `MinWindowCount` | `25` | valid-sample floor per window |
| (fixed, `Refine`) | `VarFloor` / `ReliabilityVarOffset` | `1e-4` / `0.0015` | variance floor / reliability saturation scale |
| (fixed, `SelectRefineNeighbors`) | `fMinArea`/`fMinScale,fMaxScale`/`fMinAngle,fMaxAngle` | `0.1` / `[0.2,3.2]` / `[2.5°,45°]` | neighbor-pair filter thresholds |
| (fixed, visibility) | occlusion multiplier | `1.0002` | `IsDepthSimilar` acceptance margin |

## 4. Invariants and constraints

1. **Refinement cannot repair mesh density.** The `--decimate 0` (auto) step reduces the coarse mesh
   to the working face budget before any photo-consistency iteration runs, and no amount of
   optimization recovers what that removes.
2. **`footprint[v] > 0` iff `photoGradNorm[v] > 0`.** Both backends resolve the footprint sentinel
   the same way at the end of `ScoreMesh`; nothing downstream may read one without the other.
3. **Visibility has no scene-unit tolerance.** The relative test `depth*1.0002 >= z` (§2.6) is
   invariant under uniform scene rescale by construction; there is no absolute-distance visibility
   parameter.
4. **`CleanParams::vertexSizing` must not follow a vertex-set-changing stage in the same `Mesh::Clean`
   call.** The sizing field is indexed by vertex, so a decimation that changes the vertex count has
   to be its own, earlier `Clean` call (§2.3).
5. **`w * MeshRefineStep::StepMax <= 1` is required** (explicit-flow stability of the stepper's
   explicit update), enforced at the entry of `Scene::RefineMesh`/`RefineMeshCUDA`; the Ceres arm is
   exempt (a line search has no such bound).
6. **The CPU backend is not bit-reproducible run to run** unless `--max-threads 1` (per-pair
   contributions are summed under a lock in whatever order threads complete). **The CUDA backend is
   bit-reproducible** (face-parallel accumulation, ordered gather, ordered score reduction, no float
   atomics).
7. **CPU/CUDA parity is a bug finder, not an exact-equality gate.** Camera-projection precision and
   float reassociation keep the two backends ~0.1-2% apart per vertex at the tail; a large
   disagreement is a bug, a small one is the documented approximation floor (§2.10).
8. **`S` is photometric only**, in `[0,2]`. The regularizer is not folded into it — doing so would
   break its dimensionless range and its scale invariance, which the stepper depends on.
9. **The Ceres arm requires `--alternate-pair 0` and `--planar-vertex-ratio 0`**, and forces
   `--rigidity-elasticity-ratio` to 1 (§2.9); these are validated, not merely documented.
10. **`OPTREFINE::nImageGradient` must be in `[0,3]`**; an unrecognized value is refused at the entry
    rather than silently falling back to the default.

## 5. Validation of the shipped defaults

F1 against ground truth (Tanks & Temples: official toolbox; EPFL/Strecha: mesh-to-mesh evaluator),
one `RefineMesh` run per cell, CPU/CUDA as noted.

| scene | metric | value |
|---|---|---|
| Truck | F1 at shipped defaults (CUDA) | 0.6679 |
| Barn | F1 at shipped defaults (CUDA) | 0.6706 |
| Ignatius | F1 at shipped defaults (CUDA) | 0.7799 |
| Meetingroom | F1 at shipped defaults (CUDA) | 0.4129 |
| fountain-P11 | F1 at shipped defaults (CUDA) | 0.3442 |
| Herz-Jesu-P8 | F1 at shipped defaults (CUDA) | 0.4700 |
| Herz-Jesu-P25 | F1 at shipped defaults (CUDA) | 0.6345 |
| CPU vs. CUDA | max \|F1 diff\| over 6 ground-truth scenes | ≤ 0.001 |
| CPU vs. CUDA | evaluation counts | identical on every scene tested |
| `--adaptive-face-size` (default on) | ΔF1 mean, Tanks & Temples, at 1.00x faces | +0.0007 |
| `--simplify-tolerance 0.25` (default) | ΔF1, all 7 ground-truth scenes | F1-neutral, −32..−65% faces |
| `--max-face-area 16` (default) | cap swept 8/16/24/32/48/64 on Tanks & Temples | 16 dominates on F1, face count and wall jointly |
| `--fast` (`--max-face-area 32 --simplify-tolerance 0.5`) | ΔF1 mean, Tanks & Temples | −0.0046 at 0.40x faces, 0.78x wall |
| tightest-pair area used for post-refinement decimation (vs. `f/depth`) | ΔF1 mean, Tanks & Temples | +0.0001 at 0.81x faces |
| noise floor | max pairwise F1 spread, 3 identical CPU runs | 0.0001 (Truck/Barn/Meetingroom/fountain) to 0.0009 (Ignatius) |

## 6. Rejected alternatives

- Per-vertex normalized, unit-clamped photometric direction: clamping flattens the gradient
  distribution and loses on every scene tested.
- Retuning the regularity weight to rescue the per-vertex clamp: still loses; the arm does best with
  the photometric term suppressed.
- Running fewer iterations of the old fixed 67-evaluation schedule as a speed arm: faster but loses
  accuracy on the object scene.
- Fixed-step control arm (never rejects, never adapts `eta`): large loss; the accept/reject machinery
  is load-bearing.
- Depth-proportional/grazing-aware visibility tolerance, a 2x2 any-passing depth tap, an exact
  `depth >= z` shadow comparison, a receiver bias from the local depth spread, and multipliers other
  than 1.0002: all inside noise or scene-inconsistent (one wins on average but loses on Barn); the
  plain relative test at 1.0002 dominates every variant tried.
- Scoring both warp directions every evaluation instead of alternating: no gain, doubles CPU pair
  work.
- Pure thin-plate regularizer (rigidity/elasticity ratio forced to 1): the first-order share does
  useful work; forcing it to 1 loses.
- Resetting the reject streak between phase A and phase B: a scale that gave up on its rejections
  keeps stepping into a worse trajectory.
- Smaller initial step (0.1/0.2 px instead of 0.5): loses and costs more evaluations.
- Dividing the per-vertex photometric sum by the confidence-weighted pixel count instead of the pair
  count: loses and doubles the displaced-ground-truth oracle error.
- The paper's literal plain sum with no per-vertex division: catastrophic on the highest-pair-count
  scene.
- Sign-vote (bounded ±1) photometric direction: lands below the coarse input.
- Saturating `tanh` photometric direction: same failure mode as the sign vote.
- Freezing boundary vertices: loses, worst on the small-view scenes.
- Rim-Laplacian boundary treatment plus a second-ring fix: loses.
- A single full-resolution scale (no multi-scale schedule): the first step is rejected repeatedly and
  the scale ends before anything is accepted.
- Sobel-3 image derivative: gains on average but loses on Barn.
- 3x5 separable image derivative (the previous default): loses against central differences on every
  scene.
- Derivative of the bilinear interpolant as the general-purpose stencil: loses slightly overall,
  though it converges a displaced ground truth better — kept only as what the Ceres energy mode uses.
- Trust-ratio, Barzilai-Borwein, momentum and momentum-with-clip step rules; predictive and
  restore-best stop rules: none clears the acceptance gate.
- Retuned stall thresholds (`Patience`, `ProgressTol`): response is flat, inside noise.
- Phase B run under phase A's stop rules instead of its own smaller budget: loses; the 70/30 split is
  deliberate.
- Raising the per-scale evaluation cap (45 to 100/200/1000): no effect — the cap is a safety net, not
  the operating stop rule.
- Non-default Ceres solver configurations (direction, line search, L-BFGS rank, Hessian scaling,
  iteration cap, tolerance), a pixel-unit parameterization (the first line search moves one outlier
  vertex and the bulk nothing), and a raised Ceres regularity weight: none wins on more than one
  ground-truth scene.
- Lowering `--max-views` to 4 or 2: the largest EPFL win of any arm tried, and a loss on Tanks &
  Temples on every scene — sparse-view scenes filter bad views by cutting the cap, dense-view scenes
  lose good ones and a coarser induced mesh besides; stays at 8.
- `--max-face-area` 12 or 9: marginal EPFL gain for a large face-count and wall cost; stays at 16.
- Remesh crease angle other than 20°: monotone and harmful above it, flat (but face-inflating) below
  it; stays at 20°.
- Lowering the auto-decimate ratio floor to 0.05: the apparent gain is entirely a do-less artifact on
  one scene, not a tuning change; stays at the fixed 0.002 floor (§2.3).
- `--regularity-weight` 0.5 or 0.8 instead of 0.2: never negative but never clears the gate; stays at
  0.2.
- Loosening the neighbor-selection thresholds (min area, angle band): produces byte-identical or
  noise-level output — only the neighbor count matters, not the thresholds that pick them.
- Pre-blur multipliers below 0.75: real loss on one scene with iteration count actually rising, not a
  do-less effect.
- Per-vertex step cap in pixels (halved at every direction reversal): loses at every cap tried — the
  global median normalizer already sets the stride.
- The reference's fixed schedule (fixed evaluations per level, no rejection): loses, same failure
  mode as the fixed-step control arm.
- Swapping the umbrella term for the bi-Laplacian wherever the photometric pull opposes it: inert at
  the default weight, mixed and net-negative at higher weights.
- Pixel-graded isotropic remesh at every scale in place of the 1-to-4 split: worse than the split at
  equal face count — a remesh re-samples the surface every scale, the split preserves it. (Its
  sizing-field machinery is what `--adaptive-face-size` reuses, once, in the preparation only.)
- Photometric support masks around reconstructed points: inert on small scenes, loses on Tanks &
  Temples for extra memory.
- End-of-scale bi-Laplacian relaxation as a post-smoothing step: helps noisy scenes, costs the
  detailed object scene; no fraction is an always-win.
- Image pairs voted by tie-point tracks in place of the per-image neighbor selection: thins pairs
  hard and loses even at equal evaluation counts and equal view budget.
- Subdividing by the largest per-image projection instead of the tightest pair's smaller one: loses
  more faces than it's worth; existed only to isolate the pair-vote experiment above.
- Raising `--resolution-level` as a fast-mode lever: by far the most expensive lever per unit of
  quality (it also shrinks the face count the subdivision reaches).
- Exempting the opening-cascade rejections from `MaxRejects`, and quadratic backtracking on a
  rejection (which reaches an acceptable stride in fewer rejections): both net losses — either lets
  a scale move the mesh more before it stops, which this stage's evidence consistently penalizes.
  Re-measuring the end-of-scale relaxation with that confound removed still loses.
- Removing faces no image sees, as part of refinement itself: inert — the Delaunay graph-cut mesher
  this pipeline consumes does not produce unseen undersides; kept opt-in in `TransformScene`/
  `ReconstructMesh` instead, for imported meshes.
- Skipping evaluation on the coarse scale (prepare only): scene-dependent — gains on the object
  scene, loses on a wide scene whose fine scale then opens unrefined and dies early; no arm can tell
  the two cases apart in advance.
- Warm start (each scale after the first opens at twice the previous scale's ending `eta`): positive
  on the old preparation, a net loss re-measured on the shipped one-pass preparation.
- Fixed opening (first three evaluations of every scale applied unconditionally): catastrophic
  without a per-vertex step cap, and the cap itself already lost on its own.
- Quadratic backtracking with Levenberg-Marquardt gain-ratio growth: the first-order energy model
  over-predicts the accepted decrease and does not converge to it as the step shrinks, so the rule
  mis-sizes `eta` on most accepted steps.
- Explicit pixel-size decimation target with a two-pass (decimate; split; remesh) preparation,
  remeshing before the split, and a butterfly-interpolated 1-to-4 split: all inert to slightly
  negative against the shipped single-`Clean`-pass preparation.
- Preparation reaching the target directly (remesh pinned to the derived edge length; one remesh with
  no decimation step at all): both undershoot the target further than the shipped two-step
  preparation and buy nothing.
- The old `--fast` built on `--max-views 4` instead of `--max-face-area`: a smaller view budget only
  removes per-pixel work, a coarser face cap removes per-pixel and mesh-preparation work together —
  the face-area version is `--fast` now.
- The reference's tie-point-track pair vote, reimplemented in full (cell voting, angle weighting,
  transitive-redundancy top-K): too few pairs under-constrain the surface; loses even against a
  matched-view-budget control.
- Tightest-pair reduction taken as the median or mean over the pairs that see a face, instead of the
  max: not a different rule so much as a ~5x rescale of what the face-area cap means; no evidence
  strong enough to redefine the cap around it.
- `ProgressTol`-based early stop as a fast-mode lever: same wall savings as a hard evaluation cap for
  roughly twice the F1 cost, and the threshold is untunable (bit-identical results across a 50%
  change in it).
- Coarse mesh-preparation shortcuts that trade density for wall time beyond the shipped
  `--adaptive-face-size`/`--max-face-area` combination were measured and did not clear the
  quality/size/wall gate together; the shipped one-`Clean`-pass preparation stands.

## 7. Open items

1. **The fixed-point gap.** Starting refinement from the ground truth itself, the pipeline still
   drifts measurably away from it — the coarse-scale photometric iterations amplify decimation noise
   that started well inside the scene's tolerance. No in-scope variant (pair schedule, regularizer
   share, initial step, scale count, bounded/vote/plain-sum photometric terms, boundary modes,
   alternate stencils) closes this; the remaining candidates change the mechanism and are out of
   scope (§1).
2. **Decimation policy.** The `--decimate 0` auto step is the largest single accuracy loss on
   object-like scenes (§4 #1) and has never been measured against a policy that keeps a denser coarse
   mesh; any such policy has to pay for its face count under the wall and memory budget.
3. **Mesh preparation is roughly two-fifths of total wall time** and is not itself expressed as a
   stated pixel size — it is a relative decimation ratio plus a remesh-band equilibrium that
   delivers about 70-79% of the nominal `--max-face-area` target. It is invariant under `--max-views`
   and `--resolution-level`, which is why those levers saturate early as speed knobs. A cheaper first
   `Mesh::Clean` pass is the untouched lever, and it is a halfmesh question, not a refinement one.
4. **The coarse scale's contribution is scene-dependent, and the stop rule is fragile to it.**
   Skipping the coarse scale's evaluations helps some scenes and hurts others by a wide margin
   (§6), and on at least one scene every small mesh perturbation re-rolls how many evaluations the
   fine scale gets before `MaxRejects` ends it. A per-scene signal for whether the coarse scale
   should move, and a stop rule less sensitive to it, are the largest levers left unexplored.
5. **The Ceres arm stays opt-in.** It reaches a lower energy than the stepper but never a better F1
   on the scenes it was tested against (§2.9), so it remains a research/reference arm rather than a
   default.
6. **CUDA device memory scales with resident view count** (roughly 21 bytes/pixel/view: image,
   gradient textures, depth, face id, optional mask). A very large or very high-resolution scene can
   exceed a single GPU's memory at the finest scale; the fix is to stream views in and out of
   residency, which is unimplemented.
