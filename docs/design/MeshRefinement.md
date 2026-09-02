# Mesh Refinement

Record of the `RefineMesh` improvement campaign (branch `refine-improvements`, off `develop`,
started 2026-08-30): what the variational refinement stage does today (§1), the measurement
harness built to gate every change against it (§2), and — as each work package lands — the noise
floor, baseline, accepted arms and rejected ideas that follow from running that harness (§3-§7).
`RefineMesh` was explicitly left untouched by the Delaunay-mesh and depth-fusion campaigns
(`DelaunayMeshReconstruction.md` §8's last guardrail); that exclusion is superseded here, for the
refinement code only.

## 0. Scope and adjudication rule

**Branch.** `refine-improvements`, branched from `develop`. This document is written and updated
on that branch; nothing here describes a shipped `develop` behavior change until the branch merges.

**Adjudication rule.** Every accept/reject verdict recorded in this document is backed by a row (or
a set of rows, medianed) in `bench/out_refine/results.csv`, produced by `bench/run_refine.py`
against the frozen inputs and evaluators described in §2. No verdict in §3-§7 rests on eyeballing a
screenshot, a single unmeasured run, or an argument from first principles about what "should" be
faster or more accurate — the same discipline `DepthMapFusion.md` §3 and `DelaunayMeshReconstruction.md`
§2 apply to their own campaigns. A number without a `results.csv` row behind it does not go in this
document; a claim that cannot yet be measured (because the harness or the build it depends on isn't
ready) is recorded as an open item (§8), not as a verdict.

**State (2026-09-02).** §1 describes the pipeline as it ships on this branch; §2 the harness;
§3 the noise floor and gate; §4 the develop baseline; §5 the accepted arms and the shipped
configuration measured end to end against develop; §6 every rejected candidate with its numbers;
§7 durable constraints; §8 the chronological log (newest first) and open items.

---

## 1. Pipeline as shipped

`RefineMesh` implements the Vu et al. (PAMI 2012) variational mesh refinement: alternate a
multi-scale subdivision pass with a gradient-descent optimization that pulls the surface toward
photo-consistency while a Laplacian regularizer keeps it smooth. What follows describes the
pipeline **as it ships on `refine-improvements` today** — after Part A (CPU/CUDA parity) and Part B
(the shared stepper) landed — not the pre-campaign `develop` algorithm; see §5/§8 for how it got
here and §6 for what was tried and rejected on the way. Two independent implementations still exist
side by side (`libs/MVS/AGENTS.md`'s guidance on platform-specialized code applies), but the
campaign closed most of what used to separate them: the CPU path in `libs/MVS/SceneRefine.cpp`
(`MeshRefine`/`Scene::RefineMesh`), the CUDA path split across `libs/MVS/SceneRefineCUDA.cpp`
(`MeshRefineCUDA`/`Scene::RefineMeshCUDA`) and `libs/MVS/SceneRefineCUDA.cu`/`.inl` (device kernels
and launch wrappers), and two files both backends now share outright: `libs/MVS/SceneRefineCommon.h/.cpp`
(shared scalar math, the `OPTREFINE` configuration space, per-view image/mask preparation) and
`libs/MVS/SceneRefineStep.h/.cpp` (the vertex-position stepper — one implementation, no CUDA twin to
drift). There is no `SceneRefine.h`; `MeshRefine`/`MeshRefineCUDA` are translation-unit-local
classes. `apps/RefineMesh/RefineMesh.cpp` is the CLI driver; every default below is that file's
`boost::program_options` default unless marked as an `OPTREFINE`/`MeshRefineStep` constant.

### 1.1 Entry point, CLI defaults, and the `-m`/output-naming trap

`main()` loads the scene, optionally attaches per-image masks, loads (or defaults) the mesh, and
picks a backend through `SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs)`
(`libs/Common/UtilCUDA.cpp`) — true for an empty string (the `--gpu-device` default), `-2`, `cpu` or
`none` (case-insensitive), false otherwise (`-1` = best GPU, `>=0` = comma-separated device IDs).
Refinement mutates the mesh in place, so `main()` snapshots `scene.mesh.vertices`/`faces` before a
CUDA attempt; if `RefineMeshCUDA` returns `false` (no device, OOM, a kernel launch failure, a
poisoned context after a mid-run error), it now logs
`"CUDA mesh refinement failed: falling back to the CPU implementation"` — no longer silent — restores
the snapshot, and the CPU path runs on the caller's original input instead of re-refining (and
re-decimating) whatever the half-finished CUDA run left behind.

`--mesh-file/-m` defaults to `<input-file-without-extension>.ply` **only when the archive type is
MVS**. With `-i scene_dense.mvs` that default resolves to `scene_dense.ply` — the dense **point
cloud's** ply, not a mesh — which produces `mesh.IsEmpty()` and `"error: empty initial mesh"`. `-m`
must always be passed explicitly when refining a coarse mesh built from the same stem as the input
scene. Output naming is unchanged: the refined mesh is written to `<out-stem><export-type>`
unconditionally (default `.ply`); the `.mvs` sidecar is written only when the archive type isn't
`ARCHIVE_MVS` or the input wasn't loaded as `Scene::SCENE_INTERFACE`.

Per-view masking is new since the pre-campaign pipeline: `--mask-path` points at a folder of
`<image>.mask.png` files (assigned to `Image::maskName` if not already set from the `.mvs`) and
`--ignore-mask-label` (default `-1`, disabled) is the label value those masks encode to drop. Both
are validated once at startup — a missing mask file is logged once per image here, not repeated
every scale by the hot per-view loader (§1.3). `OPTREFINE::init()`/`update()` load the `OPTREFINE`
defaults and merge in `--refine-config-file` if given; the CLI, including this app's own
`RefineMesh.cfg`, always wins over that file.

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
`"Mesh subdivided: %u/%u -> %u/%u vertices/faces"` (identical text on both backends) is what
`refine_log.py`'s `RE_SCALE` regex keys per-scale boundaries on.

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
   statistics toward `ZNCC=1` right at occlusion boundaries the way the old
   `imageA.copyTo(imageAB)` seed did.
2. **Masked local statistics, 7x7 window, shared (`Refine::HalfSize=3`, `SceneRefineCommon.h`).**
   `MeshRefine::ComputeWindowStats` (CPU: six `cv::boxFilter` passes over the masked A/B products)
   and `kernelComputeWindowStats` (CUDA: one thread per pixel, the block's 22x22 overlapping-window
   tile staged into `__shared__ sA/sB/sW` once instead of re-read 49x per thread — a naive version
   cost 1.4-1.8x the wall of the five kernels it replaced) both reduce the same six masked sums —
   over VALID pixels only, normalized by their count `n` — through two shared inline functions:
   `Refine::WindowStatsFromSums(n, sA, sB, sAA, sBB, sAB, gateMeanDiff, gateVarRatio, stats)` rejects
   the pixel if `n < Refine::MinWindowCount` (25), floors both variances at `1e-4`, and applies the
   two rejection gates ported from the Acute3D shader (`correlation.frag`): `|muA-muB| >
   OPTREFINE::fGateMeanDiff` (default 0.4) or a variance ratio exceeding `OPTREFINE::fGateVarRatio`
   (default 8) — a specular highlight, a shadow boundary or a missed occlusion, not a
   photo-consistency measurement of the same surface. `Refine::ZnccAndDerivative(stats, n, pixA,
   pixB, zncc, dzncc, conf, dzRaw)` forms `zncc = cov/sqrt(varA*varB)`, its derivative
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
   shipped pipeline computes.
4. **Image gradient stencil, shared.** `gB` above comes from a per-view gradient image built once
   per scale by `ComputeRefineImageGradient` (`SceneRefineCommon.cpp`), selected by
   `OPTREFINE::nImageGradient` (default **1**, central differences `[-1,0,1]/2`; 0 = the
   noise-robust separable 3x5 `[1,2,1]^T (x) [-1,-2,0,2,1]/32`, `CreateDerivativeKernel3x5`; 2 =
   Sobel-3 `/8`). CPU builds it once with `cv::filter2D` and samples it bilinearly (integer
   coordinates = pixel centres); CUDA uploads the identical host-computed image as two float
   textures and samples with hardware bilinear `tex2D`, whose non-normalized-coordinate convention
   places a texel's centre at `+0.5` — every CUDA fetch reading a CPU-convention coordinate (this
   sample, and the warp's color fetch in step 1) adds that offset explicitly so both backends sample
   the same point. WP7 measured all three stencils and found central differences the clear winner
   (T&T mean +0.0086 F1 over the 3x5 default, every scene positive, identical evaluation counts)
   once the per-scale Gaussian pre-blur (§1.2) is accounted for — the wider stencil only adds blur
   on top of an already-blurred image.
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
   stepper and the Ceres energy mode (§1.7) both read as `g_v`; there is no other normalizer shipped.
6. **Boundary vertices get the photometric term but zero smoothing.** Nothing in `ScoreMesh`/
   `ComputeWindowStats`/`ComputePhotometricGradient` special-cases a boundary vertex — the
   photometric pull above applies to it like any interior vertex — but
   `ComputeSmoothnessGradient1`/`2` (§1.4) zero both smoothing terms there unconditionally.

`--reduce-memory` still exists on the CLI for compatibility but its help text now reads
"deprecated: no effect since the masked window statistics" — the memory/compute trade-off it used
to gate is gone now that every window statistic above is masked and recomputed fresh from six box
filters.

### 1.4 Regularization term

Unchanged operators. `ComputeSmoothnessGradient1` (CPU) / `kernelComputeSmoothnessGradient(mode=0)`
(CUDA) compute the discrete umbrella-operator Laplacian `L(v) = mean(1-ring neighbors) - v`, zeroed
at boundary vertices. `ComputeSmoothnessGradient2` / `kernelComputeSmoothnessGradient(mode=1)` form
the "level 2" operator from Hernandez (2004, p.105) — a valence-normalized combination of `L` over
the same ring — also zeroed at boundary vertices; on both backends the valence used to weight a
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
optimizer. Unlike the pre-campaign pipeline, the ratio is no longer forced to 1 for a fixed
fraction of a fixed iteration count — see §1.7 for the two-phase schedule that replaced it.

### 1.5 Visibility test (occlusion)

CPU `MeshRefine::IsDepthSimilar` and CUDA `kernelImageMeshWarp` both round the projected point in B
to its nearest depth-map texel and accept it iff `depth > 0 && depth*1.0002f >= z` — a one-sided
occlusion test (rejects only when B's own measured depth is significantly closer than the
transformed point) that is **exactly invariant under a uniform scene rescale**, since both depths
are multiplied by the same factor. This replaced a fixed scene-unit bias after two campaign
results: a depth-proportional, grazing-aware tolerance regressed every scene (WP3, up to −0.020 on
Ignatius) and a plain nearest-tap-vs-2x2-tap sweep found nearest-tap sampling itself worth +0.009
mean F1 before any tolerance was touched; the `1.0002` multiplier was then tuned (+0.0138 mean,
non-monotonic — `1.0001` loses 0.0105 on Barn despite a small Ignatius gain), and the old fixed
bias has no compatibility path left in the tree.

Both backends also test the B-side keep-mask at the same rounded tap, and both range-check the
projected coordinate in **float, before** any integer conversion: a grazing projection can put the
target arbitrarily far outside the image, and converting to `int` first let a saturated/wrapped
value slip past a `<` comparison and index the depth map out of bounds — caught by
compute-sanitizer as a multi-GB out-of-range read on CUDA, silent heap-adjacent reads on CPU, before
the float-domain range test closed both.

### 1.6 CPU/CUDA: what is shared vs what legitimately differs

The pre-campaign divergence table no longer applies. Identical **by construction**, not by
measurement: the shared scalar header (`SceneRefineCommon.h` — `ZnccReliability`,
`WindowStatsFromSums`, `ZnccAndDerivative`), the 7x7 window, the visibility test, the image-gradient
stencil (built once, host-side, shared by both), the rasterizer (both keep only front faces where
`EdgeFunction(p0,p1,p2) > 0` and use perspective-correct barycentric coordinates — CUDA's two-pass
`kernelProjectMesh` resolves depth ties the same way the CPU's face-list traversal order does, and
is itself deterministic run to run), and the stepper (`SceneRefineStep.cpp`, one implementation).
What legitimately differs:

| Aspect | CPU | CUDA |
|---|---|---|
| Camera projection precision | double (`Camera::TransformPointW2C`) | float (`MVS::CUDA::Camera`, built once per call by `MakeCUDACamera`) — measured bit-identical face maps on the `Tiny` fixture despite the precision drop |
| Photometric accumulation order | per-pair, under a lock, in whatever order threads complete — not bit-reproducible run to run unless `--max-threads 1` | face-parallel accumulate + vertex-parallel gather over a fixed order, no atomics — **bit-reproducible** |
| Planar-vertex removal | implemented (§1.7) | refused at the entry (a loud error, not a silent no-op) — caller falls back to CPU |
| Ceres arm (`--gradient-step 0`) | implemented, gated on `_USE_CERES` | refused at the entry, same fallback |
| Float reassociation | `cv::boxFilter`/`filter2D`, MSVC `/fp:precise`, no FMA | explicit-rounding intrinsics (`__fmul_rn` etc.) so nvcc cannot silently fuse an FMA the CPU wouldn't — residual disagreement ~0.1-2% per vertex at the tail, the documented approximation floor |

An env-var-gated diagnostic, `RefineDebug` (`SceneRefineCommon.h/.cpp`, `OMVS_REFINE_DEBUG_DIR`/
`_PAIR`, no CLI flag), lets a Python harness (`bench/refine_parity.py`) dump one image pair's
per-vertex gradients and per-pixel maps from both backends for direct comparison — the tool the
campaign's parity work used to find the six real CUDA-only bugs (window size, `dZNCC` form,
image-gradient sampling, border margins, bi-Laplacian valence at a boundary neighbour, back-face
winding) that are gone from the tree now, not merely narrowed.

### 1.7 Optimization schedule

Both backends drive the same shared stepper, `MeshRefineStep` (`SceneRefineStep.h/.cpp`) — the
legacy fixed-iteration, `gstep *= 0.98`-per-iteration loop is gone entirely. The stepper works in
**pixels** (through each vertex's own footprint, §1.3) and in **ZNCC** (`S`, §1.3), so its
trajectory does not depend on scene scale, image resolution or pair count (§1.8).

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
and every other vertex moves in **proportion** to its own gradient; a per-vertex
normalize-and-clamp variant was tried first and measured a 0.0215 mean F1 regression (§6.1) because
clamping flattens the gradient distribution.

**Constants** (`SceneRefineStep.h`, pixel/ZNCC quantities, not CLI-exposed): `StepMax = 1` px
(`eta_max`), `StepGrow = 1.1`, `StepShrink = 0.5`, `StepStop = 0.05` px (median-step-at-full-stride
convergence floor), `ProgressTol = 1e-3` (relative `S` decrease counted as stalled), `Kappa = 2`,
`Patience = 3` (consecutive stalled iterations that end the scale), `MaxRejects = 4` (consecutive
rejections that end the scale), `MinIters = 3` (no stop rule before this many ACCEPTED iterations).

**Accept/reject** (`OPTREFINE::nOptimizer`, default 0 = bold driver; 1 = a fixed-step control arm
that never rejects and never grows/shrinks `eta`, kept only to isolate whether the accept/reject
machinery matters). An evaluation whose `S` is worse than the last accepted `S` is REJECTed: every
vertex moves back to exactly `v_prev + stepPrev/2` (undoing half the offending step),
`eta *= StepShrink`, and the scale STOPs after 4 consecutive rejections. An accepted evaluation
becomes the new reference, resets the reject streak, grows `eta = min(eta*1.1, StepMax)` (bold
driver only), and the scale STOPs once `numAccepted >= MinIters` and `Patience` consecutive
iterations failed to improve `S` by `ProgressTol`, or once the median per-vertex step **at a full
stride** (`medianPx * StepMax/eta`, not the step just taken — an `eta` ratcheted down by repeated
accept/reject cycles would otherwise report false convergence) drops below `StepStop`.

**Per-scale two-phase schedule**, replacing the legacy fixed 70/30 split of a fixed iteration
count: `--gradient-step N.s` sets `N = floor(...)` and the initial step `eta0 = (fGradientStep-N)*10`
px (shipped default `45.05` → `N=45`, `eta0=0.5` px; both validated at entry — `eta0` must lie in
`(0, StepMax]`). The per-scale evaluation **cap** is `max(N/(nScale+1), 8)` (`nScale` 0-based,
coarsest first — the coarse scale gets the larger cap). **Phase A** runs up to `cap` evaluations at
the caller's `--rigidity-elasticity-ratio` (default 0.9), with the CPU-only planar-vertex hook
eligible from the 4th accepted evaluation onward, every 3rd accepted evaluation thereafter,
provided more than 5 evaluations remain in the phase's budget. **Phase B** runs a fresh, smaller
budget `capB = max(3, 3*numAcceptedInPhaseA/7)` — the legacy 70/30 split preserved against phase
A's ACCEPTED count rather than its raw budget, since a rejected evaluation buys no convergence — at
`rho = 1` (pure elasticity) with the planar hook off; `eta` and the accepted-`S` references carry
over from phase A unchanged, only `MeshRefineStep::ResetStall()` runs between phases (the stall
counter resets, the reject streak does **not** — resetting it too was tried and measured −0.0048
mean F1, since it lets a scale that already gave up on its rejections keep stepping into a worse
trajectory). Either phase's loop exits early on `MeshRefineStep::STOP`.

**Planar-vertex removal (CPU only, `--planar-vertex-ratio`, default 0 = disabled).** When eligible
and the evaluation was APPLYed, every vertex whose combined-gradient magnitude and smoothing
residual both fall below `fThPlanarVertex * footprint[v] * medianViewFocalLength` is removed via
`Mesh::RemoveVerticesAndFill` — **after** the step was applied, since removal permutes the vertex
indexing (swap-with-last) and everything the stepper indexes by vertex must already have been
consumed that evaluation. The threshold is a fraction of the vertex's own depth (`footprint[v]`
times the median focal length of the views scoring this scale), not the whole-image average depth
the pre-campaign hook used. `MeshRefineStep::TopologyChanged()` runs after a removal so the next
evaluation is not rejected against an `S` measured on a different vertex set. CUDA refuses
`--planar-vertex-ratio > 0` at the entry rather than silently skipping it.

**Ceres arm (CPU only, `--gradient-step 0`, gated on `_USE_CERES`; `OFF` in the reference build, so
unmeasured by this campaign's own `results.csv`).** `MeshRefine` runs in **energy mode**: `ScoreMesh`
returns the exact energy
```
E = Sum_pairs RegScale_p * Sum_pixels r*(1-ZNCC)  +  w * (1/2) * Sum_{v interior} ||L(v)||^2
```
and fills its exact gradient — the raw per-vertex photometric sum (no pair-count division, since the
gradient of a sum is not the gradient of a per-vertex average) plus `w * L^T L * v`
(`ComputeSmoothnessGradientLtL`, the transpose of the umbrella operator, dividing each contribution
by the *neighbour's* valence — not the Hernandez level-2 operator the stepper uses). The
photometric half's exact per-pixel derivative (`ComputeWindowStats(..., bExactDerivative=true)`)
replaces the pointwise `dzncc` with the derivative of the **whole window sum**,
`∂E/∂B_p = -(A_p*S1 - S2 - B_p*S3 + S4)` from four extra box-filtered sums, and a
rejected-but-still-summed pixel stays in the mask (not dropped) because its value still enters its
accepted neighbours' window sums. A finite-difference gate (`Scene::RefineMeshEnergyProbe`,
`MeshRefineEnergyGradientTest`) validates this pair to 0.32-0.95% on the photometric term and 0.001%
on the smoothness term against a 5% bar; the same gate found that the derivative consistent with
this energy is the derivative of the *bilinear interpolant* the warp samples
(`MeshRefine::BilinearGradient`), not any precomputed stencil image — the stencils mismatch it by
21-109% — so the energy mode always uses the bilinear derivative regardless of
`OPTREFINE::nImageGradient`. `ceres::MeshProblem` wraps this as a `FirstOrderFunction`, solved with
`GradientProblemSolver`: LBFGS rank 5, WOLFE line search, approximate eigenvalue BFGS scaling,
`max_num_iterations` = the stepper's own per-scale `cap`, `function_tolerance = 1e-4`,
`gradient_tolerance = parameter_tolerance = 0` (both would be absolute thresholds on scene-scaled
quantities, so only the relative tolerance and the iteration cap are honest stopping rules), and an
iteration callback that snapshots the lowest-cost iterate seen — a line search that exhausts its
step-size budget (common on this only-piecewise-smooth energy) leaves the parameter block at a
probe point the solver never accepted, and the snapshot is applied instead so one scale's solver
failure doesn't abort the whole refinement. `--rigidity-elasticity-ratio` is forced to 1 for the
whole solve (pure thin-plate); the arm also refuses `--planar-vertex-ratio > 0`, since the solver's
parameter count is fixed for the whole solve.

### 1.8 Scale-dependent constants

Both of the pre-campaign pipeline's scene-unit-dependent terms are gone. The visibility test is
exactly scale-invariant by construction (§1.5). The stepper measures every step in pixels through
the per-vertex footprint (`s_v`, §1.3, §1.7) and judges convergence on `S` (dimensionless, [0,2]) —
a scene scaled by 100x produces the identical sequence of accept/reject decisions and the identical
`eta` trajectory (`SceneRefineStep.h`'s own header comment states this as the design intent, and a
100x synthetic rescale test reproduces the same vertex/face counts at every stage to 1.3%, the
residual attributed to 100 not being an exact power of two). `RegularizationScale =
avgDepthA*avgDepthB / (fA*fB)` (§1.3) is not itself scale-invariant, and is not meant to be: it is
the paper's homogenization term, converting an image-space (pixel) photometric gradient into a
3D-consistent one, and it is expected to scale with the scene — that is what makes the photometric
and regularization gradients commensurate in scene units before the stepper converts them back to
pixels. A footprint-proportional visibility tolerance that tried folding a similar scale-covariant
idea into the occlusion test instead of the stepper was measured and rejected (WP3, −0.0047 to
−0.0056 mean F1) — the lesson being that scale-covariance belongs in the step, not the test.

The scene-rescale test (H7, `bench/scale_test.py`) has not been re-run to a PASS against the shipped
defaults: the last recorded run was the pre-campaign `develop` baseline, which failed as expected —
the now-removed fixed-bias visibility term was one identified cause. Closing that failure mode is
recorded as leaving H7 itself open: any remaining scale sensitivity would now have to come from
mesh-processing decisions made in image/pixel space (the `--max-face-area` subdivision threshold,
the decimation policy), not from the photometric or regularization math itself.

---

## 2. Scoring protocol

Work packages H1 (`bench/refine_log.py`) and H2 (`bench/run_refine.py`) landed the RefineMesh
measurement harness; both files were read directly for this section (not the campaign plan alone).
`bench/refine_scenes.json` (six T&T entries, four enabled) and `bench/refine_variants.json`
(`baseline` plus three exploratory regularity/scale arms) both exist on disk and are loaded at
import time (`_load_refine_scenes`/`load_refine_variants`, `run_refine.py:125-140,153-157`); the
built-in fallback registries described below (`_DEFAULT_TNT_SCENES`, `_REFINE_BUILTIN_VARIANTS`)
only activate if one of those files is later deleted or moved.

### 2.1 `bench/refine_log.py` (H1) — pure log parser

`parse_refine_log`/`parse_refine_log_file` (`refine_log.py:129-236`) turn a `RefineMesh-*.log`'s
text into a per-scale/per-iteration trace dict; it is a pure function of the log text — no
subprocess launches, no filesystem writes beyond the one read. Key regexes, all anchored on the
`HH:MM:SS [AppType]` line prefix every OpenMVS log line carries (`_PREFIX`, `:46`):

- `RE_SCALE` matches `"Mesh subdivided: %u/%u -> %u/%u vertices/faces"` (`SceneRefine.cpp:572`,
  identical text from the CUDA path) — starts a new scale block.
- `RE_ITER` matches both iteration-line shapes in one pattern with optional groups: CPU carries a
  leading `f: ... (...)` cost field and a trailing `v: %5u` removed-vertex count
  (`SceneRefine.cpp:1430`); CUDA carries neither (`SceneRefineCUDA.cpp:916`). **Backend
  classification** (`refine_log.py:138-149`) is by which iteration-line shape(s) appear: only
  `f:`-bearing lines -> `cpu`; only bare `g:`/`s:` lines -> `cuda`; both shapes in one log ->
  `cuda_fallback_to_cpu` (RefineMesh.cpp's silent CUDA->CPU fallback, §1.1, leaves exactly this
  fingerprint — there is no explicit "falling back" log line anywhere in the app).
- `RE_COMPLETE` matches `"Mesh refinement completed: %u vertices, %u faces (%s)"`
  (`RefineMesh.cpp:253`); `RE_CMDLINE` matches the logged command line (`RefineMesh.cpp:169`);
  `RE_CUDA_DEVICE` matches the CUDA device-init line (from `libs/Common/UtilCUDA.cpp`).
- Per-scale wall time is derived by diffing consecutive scale-start timestamps (or the completion
  line's, for the last scale) — no explicit per-scale duration line exists — with a midnight-wrap
  correction (`_time_diff_seconds`, `:94-105`) since RefineMesh log lines carry no date field.

`test_refine_log.py` exercises this against synthetic CPU/CUDA/fallback logs (not re-read for this
document; its existence and role are recorded in the plan's work-package table, H1).

### 2.2 `bench/run_refine.py` (H2) — T&T, CPU backend only

Scope as declared in the file's own docstring (`run_refine.py:1-48`): Tanks & Temples scenes, CPU
backend (`--backend` currently only accepts `cpu`, `:819-821`). CUDA backend selection is a later
work package (H4); non-T&T datasets/evaluators (H5/H8/H9), oracle mode (H6) and the scale-invariance
test (H7) are all out of scope here — the CSV already carries every column those work packages will
fill (§2.4), and this file only ever writes `None`/empty into them.

**Frozen coarse mesh.** Every RefineMesh cell in this file starts from one frozen mesh per scene:
`ensure_coarse_mesh` (`:408-485`) builds it once with `ReconstructMesh.exe -i <scene_dense.mvs> -o
<scene_dense_mesh.mvs> -w <work> -v 2` at defaults (no ReconstructMesh flags), scores it exactly
like a refined mesh (`sample_and_score`, `:362-393`, the same in-crop sample -> evaluate ->
visibility-clean pipeline every refine cell uses), and caches the score in
`bench/out_refine/<scene>/mesh_in_f1.json` keyed on a fingerprint of `{dense_mvs stat, coarse_ply
stat, ReconstructMesh binary stat}` (`_fingerprint_coarse`, `:400-406`). A cache hit skips both the
rebuild and the rescoring. Rebuilding an existing coarse mesh requires `--refreeze-coarse` and logs
a loud warning that every earlier CSV row for that scene becomes incomparable (`:415-419`) — the
frozen mesh is the one thing every arm and every run must share. `--input-mesh` bypasses the cache
entirely for a one-off custom starting mesh (`get_input_mesh`, `:488-501`; requires exactly one
`--scenes` entry, `:881-883`).

**Scene registry.** `bench/refine_scenes.json` (loaded by `_load_refine_scenes`, `:125-137`) holds
six T&T entries — Truck/Barn/Ignatius/Meetingroom enabled, Courthouse/Caterpillar disabled — the
same four/six-scene split `bench/run_mesh.py` already uses for the Delaunay/fusion campaigns, plus
a per-scene `tau_hint` and an `evaluator`/`dataset` field that is a seam for H5/H8/H9 (always `"tnt"`
today). Each JSON entry is expanded through the shared `_tnt_scene()` helper and extended
(`_extend_scene`, `:115-122`) with the frozen coarse-mesh paths and score-cache location RefineMesh
needs that `run_mesh.py`'s own scene dicts don't carry. `_DEFAULT_TNT_SCENES` (`:111-112`) is a
built-in fallback used only if the JSON file is absent.

**Refine cell** (`run_refine_cell`, `:553-687`). Builds and runs, in the scene's frozen `-w` work
directory:
```
RefineMesh.exe -i <scene_dense.mvs> -m <frozen coarse .ply> -o <run_dir>/refined.mvs \
    -w <work> -v {2|3} --resolution-level <level> [--max-views V] [--max-threads T] <variant args>
```
(`:574-587`) — `-m` is always present (§1.1's trap), `--gpu-device` is never passed in H2 (CPU only).
After the process exits: the log is located by "new file since a pre-run snapshot"
(`snapshot_logs`/`latest_log`, reused from `run_mesh.py`) and parsed with `refine_log.py`; any
`-v 3` dump plys (`MeshRefine*.ply`/`MeshRefined*.ply`, written into `<work>` by the app itself, not
the run directory) are moved into `<run_dir>/dumps/`; the working directory's listing is compared
before and after and any file besides the log and the dumps just moved out is flagged as a
`listing_drift` warning **and** recorded in the CSV row itself, not just the console (`:606-620`,
`:751`) — an unattended overnight run must not need someone watching the log for this. `--backend
cpu` requested but a `cuda`/`cuda_fallback_to_cpu` `backend_det` in the parsed log is flagged as
`backend_mismatch=1` (`:650-657`) rather than silently recorded as a clean CPU row.

**Refusal on a merged config file.** `_cfg_blocks` (`:284-298`) checks for `<work>/RefineMesh.cfg`
before any subprocess for that scene runs (including under `--dry-run`) and, if found, refuses the
entire scene outright rather than warning: `boost::program_options` merges that file into **every**
`RefineMesh.exe` invocation silently (`RefineMesh.cpp:96,152-157` — the config-file option itself
defaults to exactly this filename, `APPNAME ".cfg"`, resolved relative to `-w`), with no log line
announcing the merge. A refusal, not a warning, because a merged `.cfg` would make every arm run in
that session incomparable to every other, and nothing in the log would reveal it happened.

**Single-instance lock.** `bench/out_refine/.lock`, PID-checked with a stale-lock takeover if the
holder is dead (`single_instance_lock`, `:305-353`) — skipped entirely under `--dry-run`, which
touches nothing on disk (`main`, `:959-969`).

### 2.3 Example commands (from the module's own docstring, `run_refine.py:26-30`)

```
python run_refine.py --scenes Ignatius --dry-run          # print every command, touch nothing
python run_refine.py --scenes Ignatius                    # baseline, 1 run, resolution-level 0
python run_refine.py --scenes Truck,Barn --noise-floor     # baseline x3, paired-delta rows
python run_refine.py --scenes Ignatius --variants baseline,regweight-01
```
`--noise-floor` (optional `N`, default 3, `:815-818`) forces the `baseline` variant only, repeated
`N` times per scene — the mode H3 will run to derive the noise floor (§3). `--dry-run` prints every
command (coarse-mesh build and every refine cell) and performs no subprocess launches, directory
creation, locking, or CSV writes (`:860-863`) — the safe way to check a command line before
committing to a real run, and the only mode this campaign's work packages are allowed to invoke for
real in an automated context ahead of a validated build (see the campaign's standing rules).

### 2.4 CSV schema (`bench/out_refine/results.csv`)

`REFINE_CSV_COLUMNS` (`run_refine.py:167-205`), grouped exactly as declared in the source (comments
in the same block), with an additive-migration `append_csv` (`:218-249`, mirrors `run_mesh.py`'s
logic: a CSV whose header is a strict prefix of the current column list is migrated in place with
old rows padded; any other mismatch is archived rather than corrupted).

| Group | Columns (representative) | Populated by H2 today? |
|---|---|---|
| identity | `ts tag scene dataset evaluator variant backend_req backend_det run level max_views max_threads args env bin_version bin_commit cmdline` | yes |
| metric | `tau metric_note` | yes |
| reference | `cloud_f1` (the T&T dense-cloud F1, shared cache with `run_mesh.py`) | yes |
| input (cached coarse mesh) | `in_p in_r in_f1 in_vis_p in_vis_r in_vis_f1 in_verts in_faces in_faces_incrop` | yes |
| refined | `ref_p ref_r ref_f1 ref_vis_p ref_vis_r ref_vis_f1 ref_verts ref_faces ref_faces_incrop vis_points vis_kept_pct` | yes |
| deltas | `d_in_f1 d_in_vis_f1 base_f1 base_vis_f1 d_base_f1 d_base_vis_f1` | yes (`d_in_*` always; `base_*`/`d_base_*` only when a `baseline` variant ran in the same invocation, §2.5) |
| trace | `scales iters_total iters_per_scale verts_per_scale faces_per_scale f_first f_last f_rel_last g_first g_last g_mean_last s_last scale_wall_s load_s refine_s refine_wall_s dump_files` | yes (`load_s` always empty — no distinct load-duration log line exists yet) |
| resources | `peak_ws_mb peak_ws_log_mb peak_ws_src` | yes |
| scoring cost | `sample_points sample_wall_s eval_wall_s vis_wall_s` | yes |
| status | `rc_refine rc_sample rc_eval rc_vis backend_mismatch listing_drift seed refine_log run_dir trace_json` | yes |
| mesh2mesh / DTU / ETH3D | `acc_{mean,median,rms,p90,p95} comp_{mean,median,rms,p90,p95} comp_vis_frac gt_samples f_curve dtu_overall_mm` | **no — always `None`**, reserved for H5 (mesh2mesh/EPFL), H8 (DTU/BlendedMVS), H9 (ETH3D) |
| oracle | `oracle degrade_spec in_dist_mean ref_dist_mean recovery_pct recovery_per_scale` | **no — always `None`**, reserved for H6 |
| scale-test | `scale_factor ref_dist_to_unit_mean ref_dist_to_unit_p95 iters_match counts_match` | **no — always `None`**, reserved for H7 |

The three reserved groups are enumerated verbatim as `_FUTURE_WP_COLUMNS` (`:207-215`) and are
`setdefault(col, None)`'d onto every row (`_build_refine_row`, `:757-758`) precisely so that no
later work package needs a CSV schema migration to start filling them — they exist as placeholders
today, not as implemented functionality.

### 2.5 Primary/secondary metric and baseline deltas

Per the campaign plan's CSV-schema section, `d_base_vis_f1` (visibility-cleaned F1, arm minus
baseline) is the **primary** gating quantity once a `baseline` variant exists in the invocation
being compared; `d_base_f1` (uncleaned) is the secondary guard. Refinement can only move
camera-observed surface, so uncleaned F1 dilutes any real effect with hole-filled surface no arm
can touch — the same reasoning `DepthMapFusion.md` §3 applies to its own mesh-visibility cleaning
step.

`_fill_baseline_deltas` (`run_refine.py:766-780`) computes this **within one invocation only**: it
groups this run's rows by `(scene, backend_req, level)`, takes the median `ref_f1`/`ref_vis_f1`
over that group's `baseline` rows (`rc_refine==0` only) as `base_f1`/`base_vis_f1`, and sets
`d_base_f1`/`d_base_vis_f1` for every row in the group against that median. If no `baseline` variant
ran in this invocation, those four columns stay `None` for every row — permanently, unless a later
invocation with a `baseline` row is what produced them. Cross-invocation deltas against a
persisted, multi-run noise-floor table are `refine_report.py`'s job (H3, not written yet — see §8).

### 2.6 T&T evaluator reuse

`run_refine.py` imports the following directly from `bench/run_mesh.py` as a sibling module
(`:66-72`) rather than reimplementing any of it — RefineMesh output needs exactly the treatment
ReconstructMesh output already gets:

- `_eval_tnt` (`run_mesh.py:515`) — the official T&T toolbox subprocess wrapper.
- `evaluate_cloud` (`run_mesh.py:591`) — dispatches to the right evaluator by the scene's
  `evaluator` key (today always `"tnt"`; `mesh2mesh`/`dtu`/`eth3d` register into the same
  `EVALUATORS` dict once H5/H8/H9 add them, so `evaluate_cloud` itself needs no change).
- `cloud_metrics` (`run_mesh.py:626`) — the cached dense-cloud F1 (`cloud_f1` column, §2.4).
- `load_scene_crop` (`run_mesh.py:674`) — the frozen scene->GT crop/alignment transform
  (`<Scene>_final_transform.npy`), shared rather than duplicated with `bench/out_mesh/<scene>/cloud_f1.json`.
- `run_sample_python` (`run_mesh.py:821`) — the in-crop, area-uniform, seeded (default seed 42)
  10 M-point sampler.
- `clean_and_score` (`run_mesh.py:907`) — the optional visibility-clean arm (mesh rendered into
  every scene camera; samples no camera sees are dropped before scoring), togglable per run via
  `--no-vis-clean`.

`sample_and_score` (`run_refine.py:362-393`) is the thin wrapper tying these together for both the
coarse mesh and every refined mesh, so both get bit-for-bit identical sampling/evaluation treatment.

---

### 2.7 Mesh-GT datasets, camera self-check, scale test (H5/H7 tooling, validated 2026-08-30)

All under gitignored `bench/`, all run with the `tnt` venv python (open3d, scipy, PIL+pillow-heif).
Each tool was validated on the `Tiny` fixture before touching any real dataset.

| Tool | What it does | Validation on `Tiny` |
|---|---|---|
| `datasets/camcheck.py --mvs --gt --out [--perturb]` | Camera-convention self-check run **before** any dense reconstruction of an imported scene: (1) geometric — 50 k area-uniform GT samples, per-camera positive-depth / inside-image fractions, PASS median ≥ 0.95 / ≥ 0.3; (2) **photometric, decisive** — the 3 camera pairs with the largest GT overlap, GT depth rendered into A (`mesh_visibility.render_depth`), back-projected, projected into B with an occlusion test against B's GT depth, masked 9×9 ZNCC between A and warped B, PASS median ≥ 0.4 over the more-textured half of the valid pixels for every pair; (3) footprint — median `depth/f` at level 0 → `tau_from_footprint_rule = round-nice(2·footprint)`. | correct cameras: ZNCC 0.94 / 0.91 / 0.94 → PASS; `--perturb transpose-R` / `negate-C`: geometric FAIL (inside 0.000); `--perturb jitter-C` (centres shifted 2 % of the bbox diagonal): geometric PASS, photometric 0.65 / **0.36** / 0.66 → FAIL — the photometric test is the one that catches a plausible-but-wrong convention. |
| `eval_mesh2mesh.py --points|--mesh --gt --mvs --tau --out-dir [--self-test]` | Mesh-vs-mesh evaluator for EPFL/BlendedMVS/oracle: our samples cropped to the GT AABB + 4τ; accuracy = exact point-to-triangle distance to the GT (`RaycastingScene.compute_distance`); completeness = 2 M seeded GT samples restricted to the GT surface some scene camera sees (`mesh_visibility.clean_points`, `comp_vis_frac` reported, unrestricted `recall_all` too) measured exactly to our mesh when `--mesh` is given; P/R/F at τ + curve at {0.5, 1, 2, 4}τ, mean/median/RMS/p90/p95 both ways; stdout in the T&T `key : value` format `parse_eval_stdout` reads + `mesh2mesh.json`. | `--self-test` (seeds 42/43, N = 2 M): \|ΔP\|,\|ΔR\|,\|ΔF\| = 0.0000, curve ≤ 4e-5 → sampling floor far below the 0.001 requirement; 4 s per evaluation. |
| `mesh_compare.py --a --b [--tau]` | Symmetric exact mesh-to-mesh distances (2 M samples each way): the measurement behind the oracle recovery numbers, the scale test and the identical-run distance noise. | CPU-refined vs its coarse input: symmetric mean 0.0050; **CUDA-refined vs the same input: 0.0123** (2.5× farther — the cull bug of §8 degrades, it does not refine). |
| `scale_scene.py --mvs --out --scale [--mesh …] [--cloud …] [--verify]` | Rescales every length field of the MVSI (`cameras[].C`, `poses[].C`, `images[].{min,avg,max}_depth`, `vertices[].X`, lines, `transform[:3,3]`, `obb.pt_min/max`) plus mesh/cloud vertices; K, R, names untouched; asserts the stream version round-trips. The only trusted rescaler: `TransformScene.exe` leaves the persisted `avgDepth` (which `RegularizationScale` reads) unscaled. | `--verify` at s = 100: every scaled field = 100× input, rotations/K/names unchanged, version 7 preserved; s = 1 output is **byte-identical** to the input; dense cloud with per-point view lists scaled to float32 precision (max \|Δ\| 3e-5 at magnitude 600). |
| `scale_test.py --work --out [--scales 100,0.01]` | H7 driver: N unit runs → `d_noise` (max pairwise symmetric mean); identity control (s = 1 through the tool); per s: scaled scene refined in its own folder (junction to the image folder, frozen work dir untouched), result divided by s, compared with unit run 0 — PASS iff mean ≤ 1.5·d_noise, p95 ≤ 2·p95_noise, `iters_match`, `counts_match` (per-scale counts within 2× the spread the unit runs show, 1e-4 floor — identical multi-threaded runs differ by a few vertices at the finer scale); relative f/g sequences reported. Holds the harness lock. **The shipped baseline is expected to FAIL** (§4 records it) — the test is trusted only after that. | 20 s on `Tiny`: 3 unit runs `d_noise` 1e-6; identity control PASS (dist 0.0, counts identical, `f_rel` dev 7e-5); s = 100 FAIL (0.00218, counts differ), s = 0.01 FAIL (0.00225) — the baseline fails as it must, the tool does not (§8). |
| `oracle_degrade.py --gt --mvs --spec --seed --out [--rho]` | H6 degraded-GT input generator (deterministic): GT cropped to faces whose **centroid** some camera sees (`clean_points` z-buffer test — a pixel-ray hit test shreds a sub-pixel-face GT into thousands of components), components < 1 % of the area dropped, quadric decimation to `A/(64ρ²)` faces (ρ = level-0 footprint from `camcheck.json`), then the spec: `dec64` / `gt-fixedpoint` (nothing more), `dec64-smooth10/30` (Laplacian λ 0.5), `dec64-noise-hf` (i.i.d. normal offset σ = ρ), `dec64-noise-lf` (per-vertex noise diffused 16 one-ring iterations ≈ 8-edge waves, rescaled to σ = 3ρ), `standard` (smooth10 → noise-lf 2ρ). JSON sidecar with counts and applied steps. | `Tiny` (ρ 0.013): 99,561 → 5,342 faces (median edge 8ρ); input→GT mean distance dec64 0.0023 (0.18ρ), noise-hf 0.0076 (0.58ρ), noise-lf 0.030 (2.3ρ), standard 0.023 (1.75ρ) — monotone in the intended order, hf ≪ lf. |
| `datasets/{mvsnet_writer,import_epfl,prepare_scene}.py` | MVSNet `cams/` writer derived from `InterfaceMVSNet.cpp`'s parser (world→cam extrinsic, raw-pixel K); EPFL `.camera` importer that emits **all four** R/Rᵀ × centre/translation hypotheses as `runOMVS/hyp{0..3}/scene.mvs`; frozen `Densify (L1) → ReconstructMesh` pipeline that refuses to run without a PASS `camcheck.json` and records `frozen.json`. | EPFL fountain-P11 (11 × 3072×2048) and Herz-Jesu-P8 (8) downloaded from the still-live `documents.epfl.ch` mirror (SHA-256 in `C:\Pro\Datasets\EPFL\SOURCES.md`); GT laser meshes 26.0 M / 36.2 M faces (ASCII, converted once to `runOMVS/gt_binary.ply`). camcheck picked `hyp2` on both scenes (§8); `prepare_scene.py` froze both (§8). |

## 3. Noise floor and derived gate thresholds

Measured 2026-08-30 with the pinned develop binary (`bench/bin_refine_develop`, commit `c410c9d4`),
CPU backend, `--resolution-level 1`, default options, 3 identical `baseline` runs per scene (tag
`noisefloor`; Ignatius has a 4th identical run from the smoke test), `bench/out_refine/results.csv`
rows 2-13, aggregated by `bench/refine_report.py`. nf = max pairwise |Δ| over the identical runs.

| scene | n | nf `ref_vis_f1` | nf `ref_f1` | wall s (min-max) | peak RSS MB |
|---|---|---|---|---|---|
| Truck | 3 | 0.0001 | 0.0001 | 406-413 | 5006-5034 |
| Barn | 3 | 0.0001 | 0.0001 | 783-813 | 9875 |
| Ignatius | 4 | 0.0008 | 0.0009 | 349-363 | 5063 |
| Meetingroom | 3 | 0.0001 | 0.0001 | 573-596 | 6972-6992 |
| **nf_mean (CPU, L1, T&T acceptance set)** | | **0.00028** | | | |
| fountain-P11 (EPFL, diagnostic set) | 3 | 0.00001 | 0.00001 | 41-48 | 2280-2284 |
| Herz-Jesu-P8 (EPFL, diagnostic set) | 3 | 0.0009 | 0.0009 | 27-28 | 2082-2108 |

The floor is run-to-run thread-order non-determinism of the CPU pair-gradient accumulation
(`--max-threads` default; the sampler and evaluator are seeded and deterministic, so identical meshes
score identically). Ignatius is 8× noisier than the others because its τ (0.003) is the tightest.
Wall time varies ≤ 4 % between identical runs.

**Derived gate (CPU, L1) — replaces the plan's placeholders; every arm is judged against it by
`refine_report.py`:**

1. runs per arm: **1** (nf_mean 0.00028 ≤ 0.001);
2. criterion 1, mean `d_base_vis_f1` over the 4 scenes ≥ max(0.002, 2·nf_mean) = **≥ +0.002**;
3. criterion 2, no scene `d_base_vis_f1` < −max(0.002, nf_scene) = **> −0.002 on every scene**
   (nf never exceeds the floor, so the 0.002 floor is the binding number everywhere);
4. criterion 3, uncleaned `d_base_f1` ≥ −0.003 everywhere (unchanged);
5. criterion 4, pooled median `refine_wall_s` ≤ 1.25× baseline (baseline medians: Truck 407 s,
   Barn 798 s, Ignatius 363 s, Meetingroom 573 s) and `peak_ws_mb` ≤ 1.15× (5028 / 9875 / 5063 /
   6974 MB);
6. criterion 5, `rc_refine = 0`, `backend_mismatch = 0`, `iters_total` = 67 unless a schedule
   change is declared;
7. **speed route (re-derived)**: the plan's "every scene within ±nf_scene" is unsatisfiable at
   nf = 0.0001 — any arm whose arithmetic differs at all moves F1 by more than that — so the
   two-sided tolerance is the same floor criterion 2 uses: **|`d_base_vis_f1`| ≤ max(0.002,
   nf_scene) on every scene and pooled median wall ≤ 0.85× baseline**.

The 0.002 floor is the Delaunay-campaign magnitude the plan started from; the measurement shows it
is ≥ 2.5× the largest per-scene nf, i.e. conservative, and it is kept rather than tightened because
the uncleaned/cleaned evaluators and the 10 M-point sampler add their own sub-0.001 variation when a
*different* mesh is scored (see the eval_mesh2mesh self-test in §2.7 for the sampling floor).

**CUDA noise floor on the final binary (2026-09-02, `bench/bin_refine_final`, tag `final-nf`,
Ignatius L1, three identical runs): F1 0.7730 / 0.7730 / 0.7730, 413,865 faces each, 23
evaluations each — nf = 0 exactly.** The deterministic accumulation (§8) removed the CUDA
run-to-run term entirely; the paragraph below records the state before it, when a CUDA floor could
not even be measured.

**CUDA noise floor: still NOT measurable (measured 2026-08-30 23:19, pin `bench/bin_refine_wp1`,
Ignatius L1, 3 runs each backend).** The attempt produced the finding at the top of §8 instead:

| run | CPU `ref_f1` | CPU wall s | CUDA `ref_f1` | CUDA wall s | CUDA `CUDA error` lines |
|---|---|---|---|---|---|
| 0 | 0.6495 | 357.6 | 0.4257 | 184.2 | 85,364 |
| 1 | 0.6497 | 357.9 | 0.4256 | 183.3 | 85,364 |
| 2 | 0.6496 | 347.7 | **0.6500** | **75.3** | **0** |

Two of three CUDA runs hit `CUDA_ERROR_ILLEGAL_ADDRESS` at the scale-0 → scale-1 transition; their
0.4257 is a half-refined mesh and their extra wall time is the 85 k failed calls, not work. A CUDA
noise floor cannot be derived from a backend that corrupts its own context two runs in three, so the
CUDA rows of §4 stay empty until that closes. **The one clean CUDA run is the useful measurement:
F1 0.6500 against the CPU's 0.6496 — a +0.0004 difference, inside the CPU's own spread — at 75.3 s
against 347.7 s, i.e. 4.6× faster.** That is the first end-to-end evidence that the WP1/WP2 parity
work actually produces an equivalent surface on a real scene, and it sets the prize for fixing the
illegal access.

**CPU re-baseline on the same pin:** 0.6495 / 0.6497 / 0.6496, nf = **0.0002** against the develop
pin's 0.0008 on this scene, and the median F1 moved 0.6489 → 0.6496 (+0.0007, i.e. within a single
noise-floor width). The WP1 border/partial-face changes and the warp-rule fix therefore left the
production CPU path where it was; the gate above stands unchanged.

## 4. Baseline table (per scene / backend / level)

Same rows as §3 (medians over the identical runs). `in_*` = the frozen ReconstructMesh coarse mesh
scored with the identical protocol; `d_in` = refined − coarse.

| scene | backend | L | `in_f1` | `ref_f1` | `ref_vis_f1` | `d_in_f1` | wall s | peak MB | iters | faces coarse → refined |
|---|---|---|---|---|---|---|---|---|---|---|
| Truck | cpu | 1 | 0.6606 | 0.6235 | 0.6235 | **−0.0371** | 407 | 5028 | 67 (45+22) | 4,717,300 → 492,895 (in-crop 3,235,366 → 301,323) |
| Barn | cpu | 1 | 0.6310 | 0.6531 | 0.6531 | **+0.0221** | 798 | 9875 | 67 | 6,989,793 → 896,926 (4,147,284 → 567,868) |
| Ignatius | cpu | 1 | 0.7427 | 0.6489 | 0.6489 | **−0.0938** | 363 | 5063 | 67 | 3,372,579 → 410,930 (915,310 → 61,629) |
| Meetingroom | cpu | 1 | 0.4026 | 0.4033 | 0.4033 | +0.0007 | 573 | 6974 | 67 | 3,449,638 → 705,852 (3,441,855 → 703,193) |
| fountain-P11 (EPFL, mesh GT, τ 0.005) | cpu | 1 | 0.3338 | 0.3197 | 0.3197 | **−0.0141** | 41-48 | 2280 | 67 | 2,676,610 → 179,311 |
| Herz-Jesu-P8 (EPFL, mesh GT, τ 0.01) | cpu | 1 | 0.4743 | 0.4230 | 0.4231 | **−0.0513** | 27-28 | 2083 | 67 | 1,770,021 → 143,296 |

Herz-Jesu-P8 detail: coarse F at {0.5, 1, 2, 4}τ = 0.244 / 0.474 / 0.673 / 0.762, accuracy mean 0.0146;
refined 0.223 / 0.423 / 0.634 / 0.748, accuracy mean 0.0165 — again below the coarse mesh at every
τ, with a 12× decimation.

fountain-P11 detail (mesh2mesh evaluator, 3 identical runs, nf 0.00001): coarse mesh accuracy
mean/median 0.0081/0.0058 (≈ 2 level-1 footprints of 0.006), completeness mean 0.166 (the scanned
GT includes surface the 11-image densify never reconstructs — 82 % of the GT is camera-visible, our
recall of it is 0.27), F at {0.5, 1, 2, 4}τ = 0.179 / 0.334 / 0.585 / 0.757. Refined: accuracy
mean 0.0083 (worse), F curve 0.165 / 0.320 / 0.570 / 0.748 — **below the coarse mesh at every τ**:
a third scene where the shipped default is a net loss, and here the decimation is 15× (2.68 M →
179 k faces at 1536×1024 working resolution).

Reading: with the shipped defaults at L1, refinement is a net **loss** on the two object-like scenes
with tight τ (Ignatius τ 0.003, Truck 0.005) and a gain only on Barn (τ 0.01); Meetingroom is flat.
The mechanism is the `--decimate 0` auto-decimation to the half-resolution face budget before any
photo-consistency step (§8): the coarse meshes lose 5-10× of their faces, and on Ignatius the
non-uniform coarse mesh (dense on the statue, coarse on the ground) is uniformised, so the in-crop
statue keeps 6.7 % of its faces. Uncleaned and visibility-cleaned F1 coincide to 4 decimals on every
scene (the coarse meshes already cover only camera-observed surface), so `d_base_vis_f1` and
`d_base_f1` will move together for arms that do not add surface. Consequences for the campaign: the
gate compares arms against *this* baseline, so a decimation-policy arm (`--decimate 1`, larger
`--max-face-area`) is a legitimate candidate but must pay for its face count under criterion 4; the
confirm set at L0 will show whether the loss is a resolution artefact.

CUDA rows: deferred to after Part A WP1 (see §3).

**Scale-test baseline (H7, develop binary) — FAIL as required.** `Tiny` L0: identity PASS, s = 100
FAIL (dist-to-unit 0.00218, counts differ), s = 0.01 FAIL (0.00225). **Ignatius L1** (3 unit runs
6-7 min each, `d_noise` 2.1e-5, p95 3.8e-5; per-scale vertex counts of identical runs already spread
by 0-16, so `counts_match` = within 2× that spread with a 1e-4 floor): identity control PASS
(dist 1.8e-5 ≤ 3.2e-5, p95 3.2e-5 ≤ 7.6e-5, counts within tolerance); **s = 100: dist-to-unit
0.00136 (65× noise), p95 0.0042; s = 0.01: 0.00152, p95 0.0048; both with the first-scale
subdivision itself moving (182,533 / 182,257 vs 182,310 vertices)** — the scene-unit constants
(`MESHOPT_DEPTHCONSTBIAS`, `gstep`, `RegularizationScale`) change the result and even the mesh
topology. The test is therefore trusted: a scale-normalised build must PASS it.

**Oracle baseline (H6, develop binary).** `TinyGT --oracle standard` (GT = the scene's own coarse
mesh, ρ 0.013): degraded input 5,342 faces at mean distance 0.0228 (1.75ρ) from the GT, F 0.868;
after refinement 0.0072 (0.55ρ), F 0.953 → **recovery 68.5 %** — the shipped optimizer does
converge when the input error is pure displacement, which is exactly what real coarse meshes do
*not* give it (§8: the L1 losses are decimation/structure, not convergence). **fountain-P11
`--oracle standard` (real GT, ρ from camcheck, τ 0.005): degraded input F 0.502 at mean distance
0.0058 from the GT; after refinement (L1, 67 iterations, 2 scales, 42 s) F 0.416, mean distance
0.0116 → recovery −99.6 %** — on a real scene the shipped optimizer *doubles* the error of a
smooth-plus-noise displacement of the ground truth (precision 0.443, recall 0.393; 18,634 faces
out). This is the strongest diagnostic in the baseline: it is not a decimation artefact (the input
already has the target density) and not a convergence failure (TinyGT converges) — the photometric
gradient itself pushes the surface away from the truth on fountain, consistent with the −0.014 L1
baseline loss on the same scene. Part A must turn this number positive.

**fountain-P11 `--oracle gt-fixedpoint` — the ground truth is not a fixed point.** Input = the
(camera-visible) GT itself: 285,272 faces, F 0.9997, mean distance 0.00058 (self-distance of the
sampling). Output after the shipped pipeline (L1, 67 iterations, 2 scales, 15 s): **23,826 faces,
F 0.413** (precision 0.433, recall 0.395), mean distance 0.0113 → recovery −1851 %. The output
is within noise of the `standard` result (0.416) and of the regular L1 baseline: whatever mesh
fountain starts from, the pipeline lands on the same ≈0.41 surface. Two effects are confounded
here — the scale-0 auto-decimation (`MAXF(0.1, medianArea/maxArea)` took 285 k → ≈24 k faces)
and the photometric iterations — and the `--dumps` per-scale split of the same cell (mean
refined→GT distance after each stage, `mesh_compare` 2 M samples) separates them:

| stage (fountain-P11, GT in, develop pin, L1) | mean distance to GT | step |
|---|---|---|
| input (visible GT, 285,272 faces) | 0.00058 | — |
| coarse scale, after `SubdivideMesh` (decimation to ≈24 k faces) | 0.00283 | ×4.9 |
| coarse scale, after its iterations | 0.01003 | **×3.5 (−255 %)** |
| fine scale, after `SubdivideMesh` | 0.01004 | — |
| fine scale, after its iterations = output | 0.01127 | ×1.12 (−12 %) |

The decimation costs a factor 4.9 but leaves the mesh at 0.0028 — far inside τ = 0.005 (F would
still be high); **the coarse-scale photometric iterations then triple the error** and the fine
scale adds a further 12 %. The loss is the optimization, not the mesh density: starting from the
truth, the shipped photometric gradient (or the step it is applied with) moves the surface away
from it. This is the campaign's primary target — Part A's photometric term and Part B's stepper
must make the truth a fixed point (iter_recovery ≈ 0 at both scales) before anything else is
tuned; a decimation policy that keeps a mesh already finer than the footprint is Part B's
secondary item.

## 5. Candidate table (accepted arms)

### ACCEPTED — arm 2, the pixel-unit bold-driver stepper with a proportional photometric step

`OPTREFINE::nOptimizer = 0` (default), shipped as the CPU path's only optimizer; `SceneRefineStep.h`
holds the equations and the single operating point. Measured 2026-08-31 09:19-10:23, CPU, L1, one run
per scene (the derived gate allows n=1 at nf_mean 0.00028), tag `arm2-prop`, against the §4 baseline.

| scene | `in_f1` | baseline `ref_vis_f1` | **arm 2** | **`d_base_vis_f1`** | `d_base_f1` | evals | wall | peak RSS |
|---|---|---|---|---|---|---|---|---|
| Ignatius | 0.7427 | 0.6490 | 0.6557 | **+0.0067** | +0.0065 | 67 → 33 | 1.06x | 1.01x |
| Truck | 0.6606 | 0.6235 | 0.6586 | **+0.0351** | +0.0351 | 67 → 14 | **0.37x** | 1.00x |
| Barn | 0.6310 | 0.6531 | 0.6547 | **+0.0016** | +0.0015 | 67 → 33 | 0.81x | 1.00x |
| Meetingroom | 0.4026 | 0.4033 | 0.4118 | **+0.0085** | +0.0082 | 67 → 32 | 2.12x (see below) | 1.00x |
| **T&T mean** | | | | **+0.0130** | +0.0128 | | pooled median **0.93x** | max 1.015x |
| fountain-P11 (EPFL) | 0.3338 | 0.3197 | 0.3310 | **+0.0113** | +0.0113 | 67 → 60 | 1.01x | 0.98x |
| Herz-Jesu-P8 (EPFL) | 0.4743 | 0.4231 | 0.4418 | **+0.0187** | +0.0184 | 67 → 34 | 0.87x | 1.02x |

**Verdict: PASSES every criterion of the §3 gate.** (1) mean `d_base_vis_f1` +0.0130 ≥ +0.002;
(2) no scene below −0.002 — the *minimum* is +0.0016, i.e. every scene improved; (3) uncleaned
`d_base_f1` ≥ −0.003 everywhere (min +0.0015); (4) pooled median wall 0.93x ≤ 1.25x and peak RSS
1.015x ≤ 1.15x; (5) `rc_refine` 0 and `backend_mismatch` 0 on every cell, with the schedule change
declared (that is the arm). It is the first arm in this campaign to move the primary metric at all,
and it does so on **6 of 6 scenes including the two mesh-GT diagnostic scenes it was not tuned on**.

Two results worth separating from the headline. **Truck is the largest single gain (+0.0351) at 14
evaluations and 0.37x the wall** — and it changes the baseline's sign: refinement used to *lose*
0.0371 against its own coarse mesh there (§4) and now lands within 0.002 of it, so most of the gain
is a loss that stopped happening. **Ignatius keeps a large negative `d_in` (−0.0873 against −0.0938)**:
the decimation-driven structural loss identified in §4 is untouched by the optimizer, exactly as that
analysis predicted — this arm fixes convergence, not mesh density.

**Meetingroom's 2.12x wall was measurement contention, now confirmed and corrected.** Per
*evaluation* at the fine scale the other three scenes agreed closely — Ignatius 9.6 → 10.3 s, Truck
9.8 → 10.6 s, Barn 18.2 → 19.8 s, i.e. **+7 to +9 %**, what the extra per-vertex stepper work and
1-3 % more faces predict — while Meetingroom read 15.0 → 40.9 s (+173 %) on 4 % more faces. That cell
ran 09:46-10:17 with five other agents working on this machine. **Re-measured solo on the final
binary (tag `arm2-confirm`, 12:09): 479 s against the baseline's 573 s = 0.84x**, in line with the
other three scenes, and `ref_vis_f1` 0.4110 against the contended cell's 0.4118 — inside the noise
floor. The same cell doubles as the **binary-equivalence check**: the post-10:30 validation and
error-path fixes did not move the numbers, so the §5 table stands as measured. Corrected pooled
median wall: **0.81x**. (Recorded in §7.4 as a standing rule: a wall time measured beside other jobs
is not evidence.)

The arm also **redistributes its budget from the coarse scale to the fine one**: the legacy schedule
spent a fixed 45 + 22 evaluations, while the stepper converges the coarse scale in 6-8 and spends the
rest where the surface detail is (Ignatius 6+27, Truck 6+8, Barn 8+25, Meetingroom 6+26). That is the
intended behaviour of a convergence-driven stop rule, and it is why the arm can be both better and
faster.

### CUDA backend on the same stepper (B2) — equivalent surface, 2.5-4.7x faster

Measured on the same binary and protocol, tag `arm2-cuda`, one run per scene, machine otherwise idle:

| scene | CPU baseline | **CPU new** | **CUDA new** | CUDA − CPU | speedup | peak RSS |
|---|---|---|---|---|---|---|
| Ignatius | 0.6487 / 381 s / 67 | 0.6557 / 383 s / 33 | 0.6555 / **85 s** / 31 | −0.0002 | **4.5x** | 0.49x |
| Truck | 0.6235 / 406 s / 67 | 0.6586 / 151 s / 14 | 0.6587 / **60 s** / 14 | +0.0001 | **2.5x** | 0.81x |
| Barn | 0.6531 / 809 s / 67 | 0.6547 / 646 s / 33 | 0.6542 / **179 s** / 34 | −0.0005 | **3.6x** | 0.87x |
| Meetingroom | 0.4033 / 573 s / 67 | 0.4110 / 479 s / 31 | 0.4108 / **102 s** / 32 | −0.0002 | **4.7x** | 0.41x |
| fountain-P11 | 0.3197 / 41 s / 67 | 0.3310 / 45 s / 60 | 0.3300 / **14 s** / 55 | −0.0010 | 3.3x | 0.43x |
| Herz-Jesu-P8 | 0.4234 / 28 s / 67 | 0.4418 / 24 s / 34 | 0.4420 / **8 s** / 34 | +0.0003 | 3.0x | 0.32x |

CUDA's T&T mean `d_base_vis_f1` is **+0.0126** against the CPU's +0.0130. **The two backends agree to
0.0005 on the acceptance set and 0.0010 overall, and take the same number of evaluations to get there**
(33/31, 14/14, 33/34, 31/32, 34/34) — they make the same accept/reject decisions on the same surface,
which is the parity campaign's payoff stated as an end-to-end result rather than a gradient cosine.
Host peak RSS drops to 0.32-0.87x because the maps and images live on the device. The residual
±0.001 is the documented fp/atomic difference (§7.3, §7.6), not a disagreement worth chasing.

### Caveat that does NOT clear: the oracle diagnostic (gate criterion 6 of the plan)

The plan's gate also asked that the fountain oracle recovery not regress. It does regress on mean
distance while improving on the threshold metric, and both halves belong in the record:

| oracle spec | metric | baseline | **arm 2** | verdict |
|---|---|---|---|---|
| `standard` | mean dist to GT | 0.01163 | **0.01637** | worse |
| | `recovery_pct` | −99.6 % | **−181.0 %** | **worse** |
| | `ref_vis_f1` at τ | 0.4163 | **0.4344** | better (+0.018) |
| `gt-fixedpoint` | mean dist to GT | 0.01127 | **0.01187** | worse |
| | `recovery_pct` | −1851 % | **−1955 %** | **worse** |
| | `ref_vis_f1` at τ | 0.4129 | **0.4765** | better (+0.064) |

**Reading:** arm 2 puts *more* surface within τ (F at τ up substantially, +0.064 starting from the
GT itself) while the *mean* distance — a tail-sensitive statistic — gets worse. It also runs 88-89
evaluations on these degraded-GT inputs against 33-60 on the real scenes, i.e. the stop rule does not
fire early there, so the photometric term is applied for longer. Since §4 established that on
fountain the photometric term itself moves the surface away from the truth, running a *better step
rule* on a *still-wrong descent direction* buys threshold accuracy and costs tail accuracy — exactly
the shape observed. **The campaign's stated target from §4 — "make the truth a fixed point" — is
therefore NOT met by this arm, and was never in its power to meet:** arm 2 changes only how far to
step, not which way. The direction is Part A WP6 (bounded/vote photometric term), and the density
loss is the decimation policy (§7.1); both are open (§8). The arm is accepted on the §3 gate, which
is the campaign's codified acceptance criterion on real inputs, with this diagnostic recorded
against it.

### ACCEPTED — WP4, masked window statistics with the two rejection gates

`MeshRefine::ComputeWindowStats` (CPU) and `kernelComputeWindowStats` (CUDA), both reducing the
same six sums through `Refine::WindowStatsFromSums` / `Refine::ZnccAndDerivative`
(`SceneRefineCommon.h`). Three changes in one arm, because they are one mechanism:

1. **The six window sums are accumulated over the VALID pixels only** and normalized by that
   count `n`, instead of over all 49 pixels of the window. Previously `imageAB` was initialized to
   a *copy of image A* and the statistics were unmasked integral images, so every window
   straddling an occlusion boundary compared image A against image A there and read ZNCC ≈ 1
   exactly where the surface is least certain. Unwarped pixels are now zero-filled on both
   backends and masked out of every sum; `dZNCC` carries a `WindowArea/n` factor so a partially
   valid window keeps the magnitude a full one would have had.
2. **A window with fewer than `Refine::MinWindowCount` = 25 valid samples is rejected** — below
   that the mean/variance/covariance are too noisy to steer a vertex.
3. **The two rejection gates ported from the Acute3D shader** (`correlation.frag`): reject the
   pixel if `|muA - muB| > OPTREFINE::fGateMeanDiff` (0.4) or if the variances differ by more than
   `OPTREFINE::fGateVarRatio` (8). A pixel whose two windows disagree that much on brightness or
   contrast is a specular highlight, a shadow boundary or an occlusion the depth test missed, not
   a photo-consistency measurement of the same surface.

The five CUDA kernels this replaces (mean/var/cov/zncc/dzncc) and their six full-image device
buffers are gone; the CPU's two double-precision integral images are replaced by six float box
filters. Measured CUDA, L1, one run per scene, tag `wp4-tiled`/`wp4-masked`, against the §5 arm-2
CUDA numbers on the identical frozen coarse mesh:

| scene | arm 2 (CUDA) | **WP4** | **delta** | arm 2 wall | WP4 wall |
|---|---|---|---|---|---|
| Ignatius | 0.6555 | **0.6653** | **+0.0098** | 85 s | 99 s |
| Truck | 0.6587 | 0.6583 | −0.0004 | 60 s | **57 s** |
| Barn | 0.6542 | 0.6559 | +0.0017 | 179 s | **163 s** |
| Meetingroom | 0.4108 | 0.4108 | 0.0000 | 102 s | **83 s** |
| **T&T mean** | | | **+0.0028** | | median **0.91x** |
| fountain-P11 | 0.3300 | 0.3322 | +0.0022 | 14 s | 19 s |
| Herz-Jesu-P8 | 0.4420 | 0.4433 | +0.0013 | 8 s | 11 s |

CPU, same protocol (tag `wp4-cpu`): Ignatius 0.6557 → **0.6590** (+0.0033) at **268 s against
383 s (0.70x)**, Truck 0.6586 → 0.6584 (−0.0002) at 147 s against 151 s. The CPU gets *faster*
because six float box filters cost less than two double-precision integral images.

**Verdict: PASSES the §3 gate.** Mean `d_base` +0.0028 ≥ +0.002; the worst scene is −0.0004,
inside the noise floor; peak RSS unchanged (the device-side saving does not show in the host
figure, which is dominated by decimation). Improves 5 of 6 scenes including both mesh-GT scenes.
A confirmation run on the final tree (tag `wp4-final`, after the WP3 revert below) reproduces the
quality figures — 0.6652 / 0.6583 / 0.6557 / 0.4115 — so the accepted numbers are the shipped
ones. **Wall is at or below baseline but noisy on this machine**: Barn measured 163 s and 272 s on
two runs of the same binary, and Ignatius 91-99 s, so the honest statement is "no wall regression"
rather than a speedup factor; the CPU's 0.70x on Ignatius (268 s against 383 s) is the one clean
speed number, and it comes from six float box filters replacing two double-precision integral
images.

**The fused kernel had to be tiled to get there.** The first CUDA version read the mask and the
two surfaces directly inside the 7x7 loop — 147 uncached loads per thread — and cost **1.4-1.8x**
the wall of the five kernels it replaced (Truck 109 s, Barn 277 s), failing gate criterion 4 while
passing every quality criterion. Staging the 22x22 tile the block's windows span into shared
memory once (`sA`/`sB`/`sW`, invalid samples staged as zeros so the accumulation needs no branch)
brought it to 0.91x with bit-identical results. Recorded because the naive version looks obviously
cheaper than five kernel launches and is not.

### SHIPPED CONFIGURATION — end-to-end against develop (2026-09-02, pin `bench/bin_refine_wp6b`, defaults only)

Two scales, `--gradient-step 45.05` (bold driver, 0.5 px), regularity 0.2 / ratio 0.9, boundary
mode 0, magnitude photometric term with the pair-count normalizer, central-difference stencil,
masked 7x7 window statistics with the 0.4 / 8 rejection gates, nearest-tap relative visibility.
CPU = the production path (tag `final-cpu`, `--max-threads` default, one run per scene, walls
contended by a concurrent Debug build); CUDA = the same binary (tag `wp6b-arms` baseline rows);
develop = the §4 baseline (commit c410c9d4, CPU).

| scene | coarse `in_f1` | develop CPU | **shipped CPU** | Δ vs develop | shipped CUDA | CPU − CUDA | evals CPU/CUDA | wall CPU s (develop → shipped) | wall CUDA s |
|---|---|---|---|---|---|---|---|---|---|
| Ignatius | 0.7427 | 0.6489 | **0.7734** | **+0.1245** | 0.7735 | −0.0001 | 28 / 23 | 363 → 217 | 45 |
| Truck | 0.6606 | 0.6235 | **0.6667** | **+0.0432** | 0.6666 | +0.0001 | 14 / 14 | 407 → 197 | 56 |
| Barn | 0.6310 | 0.6531 | **0.6663** | **+0.0132** | 0.6673 | −0.0010 | 31 / 34 | 798 → 508 | 161 |
| Meetingroom | 0.4026 | 0.4033 | **0.4105** | **+0.0072** | 0.4110 | −0.0005 | 34 / 31 | 573 → 515 | 78 |
| **T&T mean** | | | | **+0.0470** | | | | **0.66x** | |
| fountain-P11 | 0.3338 | 0.3197 | **0.3431** | **+0.0234** | 0.3429 | +0.0002 | 57 / 60 | 41 → 39 | 13 |
| Herz-Jesu-P8 | 0.4743 | 0.4231 | **0.4675** | **+0.0444** | 0.4676 | −0.0001 | 16 / 16 | 28 → 12 | 6 |

Every scene improves against develop, on both backends, and for the first time in the campaign
refinement ends ABOVE the coarse input on Ignatius (0.7734 vs 0.7427) and Truck (0.6667 vs
0.6606) — the two object-like scenes where the develop defaults were a net loss (§4). The two
backends agree to 0.001 everywhere with the same evaluation counts (±5), and the CUDA path is
4-6x faster than the shipped CPU. Where the gain comes from, in order of landing: the pixel-unit
bold-driver stepper (arm 2, +0.0130), the masked window statistics with the Acute3D gates (WP4,
+0.0028), the nearest-tap relative visibility test (§6.0b, +0.0138), the central-difference
stencil (WP7, +0.0086), plus the parity/crash fixes that made the CUDA backend usable at all.
Everything else tried on the way is in §6 with its numbers.

### ACCEPTED — WP7, central-difference image derivative (`OPTREFINE::nImageGradient = 1`)

The photometric gradient samples the derivative of image B at the warped position; the stencil that
produces that derivative image is an implementation choice the paper does not specify. Three were
compared on the identical pinned binary (`bench/bin_refine_head`, HEAD e4a74c4a + nothing else),
CUDA, L1, one run per scene, tag `wp7-stencil`, baseline re-run in the same invocation: 0 = the
noise-robust 3x5 separable `[1 2 1]^T (x) [-1 -2 0 2 1]/32` (develop), 1 = central `[-1 0 1]/2`,
2 = Sobel-3 /8. All three see the same pre-blurred image (sigma 0.44-0.68 px, §1.2).

| scene | 3x5 (baseline) | **central** | delta | Sobel | delta | evals 3x5/central | wall s |
|---|---|---|---|---|---|---|---|
| Ignatius | 0.7494 | **0.7736** | **+0.0242** | 0.7659 | +0.0165 | 26 / 23 | 46 / 44 |
| Truck | 0.6653 | 0.6667 | +0.0014 | 0.6659 | +0.0006 | 14 / 14 | 58 / 57 |
| Barn | 0.6604 | **0.6673** | **+0.0069** | 0.6584 | −0.0020 | 33 / 34 | 158 / 161 |
| Meetingroom | 0.4121 | 0.4140 | +0.0019 | 0.4126 | +0.0005 | 34 / 34 | 83 / 84 |
| **T&T mean** | | | **+0.0086** (min +0.0014) | | +0.0039 (min −0.0020) | | 1.00x |
| fountain-P11 | 0.3420 | 0.3431 | +0.0011 | 0.3425 | +0.0005 | 54 / 60 | 13 / 13 |
| Herz-Jesu-P8 | 0.4566 | **0.4675** | **+0.0109** | 0.4679 | +0.0113 | 36 / **16** | 16 / **10** |

**Verdict: central differences PASS the §3 gate** — mean +0.0086 ≥ +0.002, every scene positive
(minimum +0.0014, inside the floor), 6 of 6 scenes including both mesh-GT scenes, at identical wall
and evaluation counts (Herz-Jesu converges in 16 evaluations instead of 36). Sobel is positive on
average but loses 0.0020 on Barn and is dominated by central everywhere except Herz-Jesu (equal).
The prior written into the plan — that the 3x5 stencil would win because the pre-blur is "too
little for raw central differences" — is refuted: after the Gaussian pre-blur the wider stencil only
adds blur to the derivative, and the tighter one tracks the sub-pixel image gradient better.

Two things the table also shows: the CUDA baseline re-run moved Ignatius 0.7506 → 0.7494, Barn
0.6591 → 0.6604 and Meetingroom 0.4111 → 0.4121 against the previous day's rows on the same
binary, so the CUDA run-to-run floor on this set is ≈ 0.0013 (float `atomicAdd` order, §7.3) —
the 0.002 gate floor still covers it, with less margin than the CPU's 0.0001-0.0009. Code status:
`nImageGradient` default flipped to 1 on both backends (the stencil is shared,
`ComputeRefineImageGradient`); the 3x5 and Sobel stencils stay selectable for the record.

## 6. Failed and rejected ideas

**Do not retry without new evidence.** Every entry below was benchmarked with the §2 protocol and
judged against the §3 gate; the numbers that killed it are kept here so it is not re-proposed. Code
status is given per entry — losers are removed from the tree, not left behind a switch.

### 6.0 WP3 — scale-free, grazing-aware visibility tolerance (REJECTED, −0.0047 / −0.0056)

The plan's Part A WP3: replace the scene-unit `Refine::DepthConstBias = 0.05` in the warp's
occlusion test with `tol = z/f_B * (base + 1.5*tan(theta))`, `theta` the incidence angle of the
surface in camera B (`|N.d|` floored at 0.1), so the test follows the scene scale and widens where
one pixel of B's depth map spans a lot of depth. Implemented on both backends (the warp gains the
face map of A and the face normals to get `N`), measured CUDA, L1, against `wp4-tiled`:

| scene | WP4 reference | `base = 1` | `base = 5` |
|---|---|---|---|
| Ignatius | 0.6653 | 0.6500 (**−0.0153**) | 0.6456 (**−0.0197**) |
| Truck | 0.6583 | 0.6571 (−0.0012) | 0.6567 (−0.0016) |
| Barn | 0.6559 | 0.6549 (−0.0010) | 0.6538 (−0.0021) |
| Meetingroom | 0.4108 | 0.4095 (−0.0013) | 0.4118 (+0.0010) |
| **mean** | | **−0.0047** | **−0.0056** |

Every scene regresses at `base = 1`, and Ignatius — the scene that gains most from WP4 — loses
0.015-0.020. **Loosening the constant makes it worse, not better**, which is what rules out
"the magnitude just needs tuning": at `base = 5` the tolerance is comparable to the old 0.05 on
these scenes' scored surfaces and Ignatius is still 0.020 down, so the regression comes from the
tolerance being *depth-proportional* (distant surface gets slack the near, scored surface does
not), not from its overall size. The stop rule notices: Ignatius runs 31 → 29 evaluations and
Meetingroom 31 → 20, i.e. fewer pixels survive the test and the energy converges on less evidence.

Code status: reverted, both backends. `Refine::DepthConstBias = 0.05f` stays, documented at its
definition as the last scene-unit constant and as a rejected candidate. The consequence stands:
**the H7 scale test cannot pass on the shipped defaults** (§7.2), and closing that now needs a
different idea than a footprint-proportional tolerance.

### 6.0a Acute3D-style visibility and depth sampling (PARTIALLY ACCEPTED)

Acute3D's reprojection shader uses a hardware shadow comparison: its projected receiver depth is
compared against the rendered depth texture, and the pixel is discarded only when the comparison
reports occlusion. To isolate that idea from OpenMVS's own depth-map representation, all candidates
kept the same one-sided meaning — a farther B depth is not an occluder — and crossed the following
three predicates with two depth footprints:

| predicate | depth footprint |
|---|---|
| legacy `depth + 0.05 >= z` | 2x2 any-passing tap (old OpenMVS) |
| exact `depth >= z` | nearest texel |
| shadow receiver bias `depth + 0.5*(maxTap-minTap) + 1e-6*max(|depth|,|z|) >= z` | |

CUDA L1, four T&T scenes, measured against the final WP4 tree:

| candidate | Ignatius | Truck | Barn | Meetingroom | mean delta | worst delta |
|---|---:|---:|---:|---:|---:|---:|
| legacy, 2x2 | +0.0004 | +0.0001 | +0.0003 | -0.0002 | +0.0002 | -0.0002 |
| exact, 2x2 | -0.0003 | +0.0009 | +0.0010 | -0.0017 | +0.0000 | -0.0017 |
| receiver bias, 2x2 | +0.0015 | +0.0005 | +0.0007 | +0.0006 | +0.0008 | +0.0005 |
| **legacy, nearest** | **+0.0293** | **+0.0025** | **+0.0037** | **+0.0014** | **+0.0092** | **+0.0014** |
| exact, nearest | +0.0598 | +0.0072 | **-0.0076** | -0.0033 | +0.0140 | -0.0076 |
| receiver bias, nearest | +0.0156 | +0.0014 | +0.0018 | +0.0003 | +0.0048 | +0.0003 |

**Decision at this stage: retain nearest-tap sampling.** It removes three depth loads and the
branchy 2x2 search from every visibility query. Exact shadow comparison is rejected despite its high
mean because it damages Barn by 0.0076; the receiver-bias variant is also removed. The visibility
bias itself is superseded by the scale-invariant refinement below. This answers the sampling
question: bilinear/PCF-like 2x2 depth testing is not needed here.

### 6.0b Simple relative receiver bias (ACCEPTED, +0.0138 mean)

The simplest scale-invariant replacement after §6.0a needs no extra maps, branches or state:
nearest-tap one-sided visibility accepts iff `depth > 0 && depth*1.0002 >= z`. Under a uniform
scene scale, both depths are multiplied by the same factor, making the inequality exactly invariant.
CUDA L1 tuning against the final nearest-plus-0.05 tree:

| multiplier | Ignatius | Truck | Barn | Meetingroom | mean delta | worst delta |
|---|---:|---:|---:|---:|---:|---:|
| `1.0010` | +0.0311 | +0.0029 | -0.0143 | -0.0022 | +0.0044 | -0.0143 |
| `1.0005` | +0.0432 | +0.0037 | -0.0089 | -0.0016 | +0.0091 | -0.0089 |
| **`1.0002`** | **+0.0538** | **+0.0046** | **-0.0021** | **-0.0013** | **+0.0138** | **-0.0021** |
| `1.0001` | +0.0564 | +0.0048 | -0.0105 | -0.0008 | +0.0125 | -0.0105 |

The curve is non-monotonic at fine tolerances: tightening from `1.0002` to `1.0001` loses 0.0105
on Barn despite a small Ignatius gain, so `1.0002` is the selected point. It improves the mean by
0.0138 while keeping the worst regression at 0.0021; it is also a single multiply at the existing
nearest tap. Code status: retained on both backends; the former fixed-bias rule has no compatibility
path.

### 6.1 Arm 1 — per-vertex normalised, unit-clamped photometric direction (REJECTED, −0.0215)
The design as originally planned: `d_v = g_v/(kappa*m*s_v)` clamped to `|d_v| <= 1`, so every vertex
moves at most `eta` px along its own gradient direction. Measured CPU, L1, 4 T&T scenes:

| scene | baseline | arm 1 | `d_base` | arm 1b (stop rule fixed) | `d_base` |
|---|---|---|---|---|---|
| Ignatius | 0.6490 | 0.6679 | +0.0189 | 0.6685 | +0.0195 |
| Truck | 0.6235 | 0.5808 | **−0.0427** | 0.5822 | −0.0413 |
| Barn | 0.6531 | 0.6290 | **−0.0241** | 0.6287 | −0.0244 |
| Meetingroom | 0.4033 | 0.3654 | **−0.0379** | 0.3661 | −0.0372 |
| **mean** | | | **−0.0215** | | **−0.0209** |

**Mechanism (established by elimination, four controls):** dividing each vertex's direction by its
own footprint and then clipping the tail flattens the gradient distribution — every vertex past the
clamp moves the same `eta` px however much larger its gradient is — so low-confidence vertices get
the same authority as high-confidence ones. The fix that ships (§5) keeps `kappa*m` as a **global**
conversion factor and drops the clamp. The four controls, each refuting a different hypothesis:

| # | hypothesis | control | verdict |
|---|---|---|---|
| 1 | "it just refines less" | legacy optimizer at 30 evaluations | refuted: opposite sign on 3 of 4 scenes |
| 2 | "it stops too early" | stop rule decoupled from `eta` (arm 1b) | refuted: +50-70 % evaluations, ΔF1 ≤ 0.0014 |
| 3 | "`S`-rejection blocks smoothness" | bold driver off (`nOptimizer=1`) | refuted: worse, −0.0641 on Truck |
| 4 | "photometric/regularisation imbalance" | `w` 0.2 → 0.02 | refuted: worse on both scenes tested |

Code status: replaced in `MeshRefineStep::Evaluate`; the header documents why the normalisation must
stay global so it is not reintroduced.

### 6.2 Regularity-weight retuning as a rescue for arm 1 (REJECTED)

Truck bracket at fixed arm 1: `w` = 0.02 / 0.2 / 0.5 / 1.0 → F1 0.5730 / 0.5822 / 0.5884 / 0.5982,
monotone up to the `StepMax*w <= 1` stability limit, and still below the legacy optimizer's 0.6235 at
`w = 0.2`. Extrapolated, arm 1 did best with its photometric term suppressed entirely — a verdict on
the formulation, not a tuning result. `fRegularityWeight` keeps its 0.2 default.

### 6.3 "Just run fewer legacy iterations" as a speed arm (REJECTED)

Legacy optimizer at `--gradient-step 20.05` (30 evaluations instead of 67): mean `d_base` −0.0015 at
2.06x faster — but Ignatius −0.0203, far outside the §3 speed route's ±0.002 per-scene tolerance. Not
shippable; recorded because it is the obvious cheap alternative to a real optimizer and it does not
work.

### 6.4 Fixed-step control arm (`OPTREFINE::nOptimizer = 1`) (KEPT AS A CONTROL, NOT A DEFAULT)

Never rejects, never grows or shrinks the step. On arm 1 it was clearly worse (Truck 0.5594 vs
0.5822, Barn 0.6107 vs 0.6287), which is what exonerated the bold driver. Retained as the only
alternative arm because it answers "does the accept/reject machinery matter?" in one flag; every
other planned arm (rprop, adam, bb) is unimplemented and now **rejected at the entry point** rather
than silently collapsing onto this one.

### 6.5 Plan premises refuted during implementation

- **"`ComputeLocalZNCC` has no zero guard → inf/NaN on flat patches."** False: `ComputeLocalVariance`
  already floors both variances at 1e-4 unconditionally, so `varA*varB >= 1e-8`. No guard was added;
  the invariant is asserted at the point of division instead.
- **"`vertexDepth` is dead, remove it."** False: it is the live input to the planar-vertex removal
  pass (`fThPlanarVertex`), and it is a different quantity from the new per-vertex `footprint`
  (whole-image `avgDepth` min vs per-vertex `depth/focal`). Kept.
- **"CUDA's 10-px border is needed."** False: it was leftover over-conservatism; both backends now
  share `Refine::Border = HalfSize = 3`.

### 6.6 Paper-faithfulness checks E1/E3 (NEUTRAL / REJECTED, 2026-09-02)

Reading PAMI 2012 against the code found two places where the shipped defaults deviate from the
paper as written, both a CLI flag away. Measured on the pinned HEAD binary (`bench/bin_refine_head`),
CUDA, L1, one run per scene, baseline re-run in the same invocation (tag `e1e3`):

| scene | baseline | **E1** `--alternate-pair 0` (all ordered pairs every evaluation, eq. 10) | delta | **E3** `--rigidity-elasticity-ratio 1` (thin-plate only, no Hernandez mix) | delta |
|---|---|---|---|---|---|
| Ignatius | 0.7493 | 0.7497 | +0.0004 | 0.7496 | +0.0003 |
| Truck | 0.6653 | 0.6653 | 0.0000 | 0.6653 | 0.0000 |
| Barn | 0.6602 | 0.6594 | −0.0008 | 0.6570 | **−0.0032** |
| Meetingroom | 0.4116 | 0.4111 | −0.0005 | 0.4119 | +0.0003 |
| **T&T mean** | | | **−0.0002** | | **−0.0006** |
| fountain-P11 | 0.3420 | 0.3419 | 0.0000 | 0.3419 | 0.0000 |
| Herz-Jesu-P8 | 0.4571 | 0.4572 | +0.0001 | 0.4567 | −0.0004 |

**E1 — neutral, default kept at 1.** Summing both warp directions in every evaluation instead of
alternating them changes no scene by more than the noise floor and leaves the evaluation counts
identical (26/14/33/34 → 26/14/33/34); the "different energy on odd and even evaluations" of the
alternating schedule is therefore not costing anything the gate can see. On CUDA the wall is also
unchanged (the per-evaluation cost there is dominated by rasterizing every view, not by the pair
loop), on the CPU it would double the pair work; the alternating default stays. The oracle cells
(§8) test the remaining hypothesis that a one-sided warp biases the fountain fixed point.
**E3 — rejected.** The paper's pure thin-plate regularizer loses 0.0032 on Barn (gate criterion 2)
and gains nothing elsewhere: the 10 % first-order Laplacian share of the shipped Hernandez mix is
doing useful work on the scene with the most hole-fill surface. Default `0.9` stays.

**Oracle cells (fountain-P11, same pin, tag `e1e3-oracle`) — the one-sided-warp hypothesis is
refuted.** The survey's sharpest explanation for the fountain fixed-point defect (§4) was that the
alternating one-sided warp resamples only image B and so biases the minimizer; summing both
directions in every evaluation would then have to move the ground truth less. It does not:

| oracle spec | metric | baseline | E1 `--alternate-pair 0` | E3 `ratio 1` |
|---|---|---|---|---|
| `standard` | mean dist to GT (input 0.00583) | 0.01367 (−134.6 %) | 0.01313 (−125.4 %) | 0.01491 (−155.8 %) |
| | F at τ | 0.4347 | 0.4404 | 0.4163 |
| `gt-fixedpoint` | mean dist to GT (input 0.00058) | 0.01136 (−1866 %) | 0.01152 (−1894 %) | 0.01057 (−1730 %) |
| | F at τ | 0.4727 | 0.4705 | 0.4833 |

Starting from the truth itself, the symmetric energy lands 0.0115 away from it exactly like the
alternating one (0.0114); the `standard` case improves by 4 % of the distance and +0.006 F, which is
the size of the oracle's own run-to-run variation on 89 evaluations. E3 is mixed on the oracle
(worse from a degraded input, better from the truth) and stays rejected on Barn. Whatever pushes
the fountain GT away by 20x its input distance is neither the pair-direction schedule nor the
first-order regularizer share; the remaining in-scope suspect is the per-pixel photometric term
itself (WP6, and E4's plain-sum normalization).

### 6.7 F08 — resetting the reject streak between the two phases (REJECTED, −0.0048 mean)

Found by the implementation review as a smell: `ResetStall()` between phase A (rigidity mix) and
phase B (pure bi-Laplacian) reset the stall counter but not the consecutive-reject streak, so a
scale whose phase A ended on `MaxRejects` gave phase B a single evaluation. Resetting both looked
like the consistent choice and was measured as part of the WP-F batch on the HEAD pin's frozen
inputs, CUDA L1 (tag `wpf-confirm` vs `wp7-stencil`; the other batch items are provably inert on
this path — the traces are identical until phase B of the first scale):

| scene | streak carried (HEAD) | streak reset (F08) | delta |
|---|---|---|---|
| Ignatius | 0.7494 | 0.7357 | **−0.0137** |
| Truck | 0.6653 | 0.6611 | −0.0042 |
| Barn | 0.6604 | 0.6591 | −0.0013 |
| Meetingroom | 0.4121 | 0.4122 | +0.0001 |
| **mean** | | | **−0.0048** |

Mechanism from the Ignatius trace: after phase A's four rejections the vertices sit at the
half-undone position; with the streak reset, phase B accepts two more tiny steps there
(S 0.1003 → 0.0986 at 0.02 px), the next scale subdivides 0.5 % differently and ends at a worse S
and F1. The calibration (§8) shows an ordinary trajectory change is worth ≈ 0.002, so the loss is
systematic: continuing a scale that has already given up on its rejections harms the fine scale
more than the coarse S improvement is worth — consistent with the heavy-tailed first steps (§8),
where the accepted tiny steps mostly move the strongest-gradient vertices. Code status: reverted
to `ResetStall()` with the measured result recorded at its definition; the unit test asserts the
carried streak.

### 6.8 E5 — a smaller initial step (`eta0` 0.1 / 0.2 px instead of 0.5) (REJECTED, −0.0104 / −0.0045)

Motivated by the traces in §8 (the 0.55-px first step of every scale triples S and costs four
rejections). `--gradient-step 45.01` (eta0 = 0.1 px) and `45.02` (0.2 px) on the HEAD pin, CUDA
L1, one run per scene, against the median of the three same-pin baseline runs (tag `e5-step`):

| scene | baseline | eta0 = 0.1 px | delta | eta0 = 0.2 px | delta | evals base / 0.1 / 0.2 |
|---|---|---|---|---|---|---|
| Ignatius | 0.7493 | 0.7320 | **−0.0173** | 0.7349 | **−0.0144** | 26 / 38 / 47 |
| Truck | 0.6653 | 0.6578 | −0.0075 | 0.6669 | +0.0016 | 14 / 36 / 28 |
| Barn | 0.6603 | 0.6507 | −0.0096 | 0.6545 | −0.0058 | 33 / 48 / 49 |
| Meetingroom | 0.4119 | 0.4046 | −0.0073 | 0.4125 | +0.0006 | 31 / 45 / 37 |
| **T&T mean** | | | **−0.0104** | | **−0.0045** | |
| fountain-P11 | 0.3420 | 0.3417 | −0.0002 | 0.3419 | −0.0001 | 59 / 58 / 60 |
| Herz-Jesu-P8 | 0.4569 | 0.4489 | −0.0080 | 0.4505 | −0.0063 | 36 / 53 / 54 |

Fountain oracle recovery moves the other way (`standard` −135 % → −124 % / −116 %;
`gt-fixedpoint` −1866 % → −1642 % at 0.1 px with F 0.4727 → 0.4905, −1903 % at 0.2 px), i.e. the
smaller step does converge a displaced GT slightly better, but on real inputs it is worse on 5 of
6 scenes and uses 30–70 % more evaluations. **Reading, together with §6.7:** what the large first
step does on a real scene is make the *coarse* scale accept almost nothing (four rejections, then
the phase budget runs out), and every change that lets the coarse scale move the mesh more — a
reset reject streak (§6.7), a smaller step that gets accepted — makes the final result worse. The
half-resolution scale looks harmful on these inputs; `--scales 1` is the next free experiment
(E6). Default `45.05` stays.

### 6.9 WP6 — bounded / re-normalized photometric terms (ALL REJECTED)

The plan's WP6 (from the Acute3D shader) and the paper-faithfulness item E4, implemented as
`OPTREFINE::nPhotoTerm` × `nPhotoNorm` arms on one binary (`bench/bin_refine_wp6`: HEAD + WP-F +
central stencil default; the `(0,0)` combination is byte-identical to the pre-WP6 code on the
CPU). Per valid pixel `m = (gB·(J·dA))·dz_raw`; term 0 = legacy `conf·m·RegScale/Nd`, term 1 =
sign vote `conf·sign(sg)`, term 2 = saturating `−conf·tanh(m/τ)` (τ = median non-zero |m| of the
pair-direction); norm 0 = divide the vertex sum by the pair-direction count (legacy), 1 = by the
confidence-weighted pixel sum `Σ conf·b_v` (makes terms 1/2 a bounded direction |·| ≤ 1 fed to the
stepper's `bounded` path, no median normalizer), 2 = the paper's plain sum (eq. 19, no per-vertex
division). CUDA, L1, one run per scene, baseline in the same invocation (tags `wp6-arms`,
`wp6-oracle`):

| scene | baseline (0,0) | (0,1) conf-pixel-sum | (0,2) plain sum = E4 | (1,1) sign vote |
|---|---|---|---|---|
| Ignatius | 0.7733 | 0.7601 (−0.0132) | **0.6864 (−0.0869)** | **0.6918 (−0.0815)** |
| Truck | 0.6667 | 0.6588 (−0.0079) | 0.6666 (−0.0001) | 0.6313 (−0.0354) |
| Barn | 0.6672 | 0.6610 (−0.0062) | 0.6705 (+0.0033) | 0.6363 (−0.0309) |
| Meetingroom | 0.4110 | 0.4024 (−0.0086) | 0.4148 (+0.0038) | 0.3884 (−0.0226) |
| **T&T mean** | | **−0.0090** | **−0.0200** | **−0.0426** |
| fountain-P11 | 0.3431 | 0.3444 (+0.0013) | 0.3428 (−0.0003) | 0.3376 (−0.0055) |
| Herz-Jesu-P8 | 0.4674 | 0.4408 (−0.0265) | 0.4690 (+0.0016) | 0.4658 (−0.0016) |
| evals (Ign/Truck/Barn/Mr) | 23/14/34/19 | 38/23/52/43 | 34/14/30/34 | 19/23/18/26 |

| fountain oracle | metric | (0,0) | (0,1) | (0,2) | (1,1) |
|---|---|---|---|---|---|
| `standard` (input 0.00583) | mean dist / recovery | 0.01491 / −156 % | 0.02073 / −256 % | 0.01403 / −141 % | **0.00656 / −13 %** |
| | F at τ | 0.4287 | 0.3380 | 0.4666 | 0.4838 |
| `gt-fixedpoint` (input 0.00058) | mean dist / recovery | 0.01044 / −1708 % | 0.02055 / −3456 % | 0.00698 / −1108 % | **0.00283 / −390 %** |
| | F at τ | 0.4897 | 0.3372 | 0.6481 | 0.8209 |

**Readings.** (0,1): dividing by the pixel weight instead of the pair count is worse on every T&T
scene and doubles the oracle error — the pair count is the better per-vertex normalizer of the
magnitude term. (0,2), the paper's literal sum: neutral-to-positive on the three scenes whose
views are spread evenly, catastrophic on Ignatius (−0.087), where the statue is seen by far more
pairs than the ground: without the per-vertex division the many-view vertices carry gradients an
order of magnitude larger than the rest and overshoot under a global step; the paper's fixed tiny
step never had that problem and never had the campaign's convergence speed either. (1,1), the
Acute3D vote: the oracle numbers that look like a fixed point (−13 %, F 0.82 from the GT) are the
same fact as the T&T losses — a bounded direction under the pixel-capped stepper moves every
vertex at most η px per evaluation and stops after 12–26 evaluations, so the mesh barely leaves
its input; on Ignatius it lands BELOW the coarse input's 0.7427. A longer schedule or a larger η
for the bounded arm would be a new optimizer design, out of this campaign's scope. The (2,1)
saturating arm, measured on the `wp6b` pin (tag `wp6b-arms`, τ = median non-zero |m| per
pair-direction), behaves like the vote: Ignatius 0.7735 → 0.6780 (−0.0955), Truck −0.0152, Barn
−0.0135, Meetingroom −0.0112, **T&T mean −0.0338**; fountain −0.0064, Herz-Jesu +0.0036; oracle
identical to the vote (`standard` −12.7 %, `gt-fixedpoint` −390 %, 12 evaluations).

**Verdict: `(nPhotoTerm, nPhotoNorm) = (0,0)` stays — the magnitude term with the pair-count
division is the best formulation on real inputs by a wide margin, and the fountain fixed-point
defect is not a property of the photometric term's shape.** Code status: the arms stay selectable
behind `--refine-config-file` for the record (the per-pixel scalar is shared between backends in
`Refine::PhotoPixelTerm`); the default path is the legacy one and is byte-identical to it.

### 6.10 WP8 — boundary-vertex modes (REJECTED: freeze −0.0008 / rim −0.0016 mean, Ignatius −0.005)

`OPTREFINE::nBoundaryMode` on both backends (CPU/CUDA parity of the new terms: cosine 1.000000):
1 = freeze (boundary vertices lose their photometric term too, so they never move), 2 = rim
Laplacian (a boundary vertex with exactly two boundary neighbours gets the curve umbrella
`(b1+b2)/2 − v` and the matching level-2 term along the rim; other boundary vertices freeze) plus
the second-ring fix (interior vertices sum the level-2 operator over interior neighbours only,
renormalized). Measured CUDA L1, one run per scene, baseline in the same invocation
(tag `wp6b-arms`):

| scene | mode 0 (legacy) | 1 freeze | 2 rim + second-ring fix |
|---|---|---|---|
| Ignatius | 0.7735 | 0.7685 (**−0.0050**) | 0.7678 (**−0.0057**) |
| Truck | 0.6666 | 0.6668 (+0.0002) | 0.6667 (+0.0001) |
| Barn | 0.6673 | 0.6670 (−0.0003) | 0.6664 (−0.0009) |
| Meetingroom | 0.4110 | 0.4128 (+0.0018) | 0.4110 (0.0000) |
| **T&T mean** | | **−0.0008** | **−0.0016** |
| fountain-P11 | 0.3429 | 0.3434 (+0.0005) | 0.3434 (+0.0005) |
| Herz-Jesu-P8 | 0.4676 | 0.4542 (**−0.0134**) | 0.4685 (+0.0009) |
| oracle `standard` / `gt-fixedpoint` recovery | −141 % / −1736 % | −142 % / −1760 % | −140 % / −1690 % |

Neither mode passes criterion 2 (Ignatius −0.005 both; freeze also −0.013 on Herz-Jesu). The
legacy treatment — photometric pull, no smoothing, on boundary vertices — is what the gate
prefers: on Ignatius the mesh boundary is the cut where the statue meets the unreconstructed
ground, and letting it follow the images beats pinning it or smoothing it along the rim. Code
status: both modes deleted (the shared `FindRimNeighbors` and the mode plumbing go with them);
`nBoundaryMode` is removed from `OPTREFINE`.

### 6.11 E6 — a single full-resolution scale (`--scales 1`) (REJECTED, −0.0217 mean)

The hypothesis from §6.7/§6.8 — the coarse scale harms — is refuted the direct way: dropping it
loses on every T&T scene (Ignatius **−0.0689**, Barn −0.0143, Meetingroom −0.0021, Truck −0.0014;
fountain +0.0016, Herz-Jesu −0.0035) and the single scale stops after 6–8 evaluations: the first
0.55-px step at full resolution is rejected four times in a row and the reject streak ends the
scale before any step is accepted. What the coarse scale contributes is not its own tiny
movement but the decimation-to-budget plus subdivision that prepares the mesh and, on the scenes
where the full-resolution scale's first steps are also rejected, the second chance the scale
change gives the driver. The oracle prefers the single scale (`standard` −98 % vs −141 %,
`gt-fixedpoint` −1491 % vs −1736 %) — again a "moves less" reading, not a convergence one.
Defaults unchanged (`--scales 2`).

### 6.12 E7 — the derivative of the bilinear interpolant as the image gradient (`nImageGradient = 3`) (REJECTED, −0.0007 mean)

B5's finite-difference gate showed that the derivative consistent with the energy is the
derivative of the bilinear interpolant the warp samples (§8). Measured as a stencil mode on the
final binary (`bench/bin_refine_final`, tag `final-cuda`), CUDA L1, baseline in the same
invocation:

| scene | central (default) | bilinear interpolant | delta |
|---|---|---|---|
| Ignatius | 0.7730 | 0.7753 | +0.0023 |
| Truck | 0.6667 | 0.6625 | **−0.0042** |
| Barn | 0.6673 | 0.6674 | +0.0001 |
| Meetingroom | 0.4136 | 0.4127 | −0.0009 |
| **T&T mean** | | | **−0.0007** |
| fountain-P11 | 0.3435 | 0.3423 | −0.0012 |
| Herz-Jesu-P8 | 0.4677 | 0.4622 | **−0.0055** |
| oracle `standard` / `gt-fixedpoint` recovery | −137 % / −1766 % | **−112 % / −1611 %** | better |

The energy-consistent derivative converges a displaced ground truth better (both oracle specs)
but is not a better descent direction on real inputs: the central-difference stencil's slight
smoothing of the derivative is worth more than its inconsistency. Kept selectable (it is the
derivative the Ceres energy mode uses, where consistency is what matters); default stays 1.

## 7. Durable constraints and limitations

1. **Refinement cannot repair mesh density.** The `--decimate 0` (auto) step reduces the coarse mesh
   to the working face budget *before* any photo-consistency iteration — 5-10x on the T&T scenes at
   L1, 15x on fountain — and no optimizer recovers what that removes. Ignatius keeps `d_in` ≈ −0.087
   even with the accepted arm (§5). A decimation-policy candidate is a separate, still-open item
   (§8); judging an optimizer by `d_in` rather than `d_base` measures mostly this.
2. **Visibility now has no scene-unit tolerance.** Both backends use the nearest-tap relative test
  `depth*1.0002 >= z` (§1.5), which is invariant under uniform scene scaling. The H7 scale test
  can still expose other scale-dependent terms (§1.8), but no longer fails because of visibility.
3. **The CPU backend is not bit-reproducible run to run** (per-pair contributions summed under a
   lock in completion order, unless `--max-threads 1`); the trajectory is chaotic at the vertex
   level — 1e-7 at iteration 0 grows to 6.7e-4 by iteration 44 — so every CPU F1 carries a
   run-to-run term, quantified as the §3 noise floor (0.0001-0.0009). **The CUDA backend IS
   bit-reproducible since 2026-09-02** (face-parallel accumulation + ordered gather + ordered score
   reduction, no float atomics left in the refinement; §8).
4. **Wall-time cells must run alone.** A 7-10 GB working-set refine cell measured beside other jobs
   reads high by a large factor (§5, Meetingroom). The harness enforces one cell at a time via
   `bench/out_refine/.lock`, but nothing stops *other* processes; a wall number measured next to a
   build or another agent is not evidence.
5. **`S` is photometric only.** The regulariser can raise it near convergence, producing rejections
   and an early stop — the intended end of a scale, not a defect. Adding the smoothness energy to `S`
   would break its dimensionless [0,2] range and its scale invariance.
6. **The parity diagnostic is a bug finder, not a gate.** fp32 texture-unit bilinear weights and
   atomic ordering keep the two backends ~1-2 % apart per vertex at the tail; chasing that residue
   further has no measured payoff. Large disagreements are bugs (six were found this way); small ones
   are the documented approximations.

## 8. Open items

**FINAL STATUS (2026-09-02 09:50, read this first; the blocks below are the chronological log,
newest first).** The campaign's goal — the PAMI 2012 refinement implemented correctly and
efficiently, both backends — is met and measured end to end (§5 "shipped configuration"):
against develop, +0.125 / +0.043 / +0.013 / +0.007 F1 on Ignatius / Truck / Barn / Meetingroom and
+0.023 / +0.044 on the two mesh-GT scenes, CPU and CUDA within 0.001 of each other, CPU 1.5-2x
faster than develop and CUDA 4-6x faster than the CPU, the CUDA backend bit-reproducible (nf = 0,
§3). The final binary (`bench/bin_refine_final`, tag `final-cuda`) reproduces the §5 table within
trajectory noise (max |Δ| 0.0026, Meetingroom) and the CPU lands at 0.7730 on Ignatius in 171 s.
Shipped on this branch: the parity fixes (six CUDA deviations + the warp overflow), the pixel-unit
bold-driver stepper, masked window statistics with the two rejection gates, the relative
nearest-tap visibility test, the central-difference stencil, per-view keep masks, deterministic
CUDA accumulation, the fix batch (WP-F: planar hook ordering, behind-camera warp, `OPTREFINE`
initialization in Viewer/Python, CUDA neighbour recovery, a `ScoreMesh` thread-wait deadlock, the
static thread list, `nCalibratedImages`), a consistent energy/gradient pair for the opt-in Ceres
arm with a finite-difference gate, and the synthetic end-to-end and pure-function tests. Every
rejected candidate is in §6 with its numbers and is removed from the tree.

Open items after this campaign:
1. **Fountain fixed-point defect (§4)** — not fixed by any in-scope variant (pair schedule,
   regularizer share, initial step, scale count, bounded/vote/plain-sum photometric terms,
   boundary modes, stencil incl. the energy-consistent one). The remaining suspects change the
   mechanism (area normalization, Sobolev preconditioning, robust per-pixel weights, depth-consistent
   masks) and live in the follow-up plan `refine-followup-literature.md` for a new branch.
2. **Ceres arm benchmark** — the reference build has `OpenMVS_USE_CERES=OFF`; the arm is consistent
   (FD gate 0.3-0.9 %) but unmeasured; `bench/refine_log.py` does not parse its `E:` line.
3. **H7 scale test on the shipped tree** — not re-run to a PASS (the visibility and the stepper are
   scale-invariant by construction; `RegularizationScale` is the paper's scale-covariant term).
4. **Planar-vertex hook** (CPU, default off) — can leave a non-manifold mesh that the next scale's
   `Mesh::Subdivide` asserts on in Debug; its useful ratio is ≈ 1e-4..1e-3, not the Viewer's 0.02.
5. The `--reduce-memory` CLI option is a documented no-op (kept for compatibility).


**2026-09-02 — WP-F fix batch LANDED (uncommitted, RelWithDebInfo rebuild + 4-scene re-measure
pending), and WP7 ACCEPTED (§5).** An independent review of the whole branch (19 findings) was
folded into one batch, every item fixed at its producer, Debug builds of MVS/RefineMesh/Tests/Viewer/
pyOpenMVS clean, `Tests.exe 0 1` and `2 1` green, Tiny functional on both backends:
- **F01** the planar-vertex hook removed vertices BEFORE the stepper consumed the per-vertex arrays it
  had just permuted (swap-with-last) — now removal follows the applied step; the threshold moved from
  the whole-image `avgDepth` (`vertexDepth`, deleted) to the vertex's own depth (`footprint × median
  focal`), and the log's `v:` field reports the true vertex-count change.
- **F02** the CPU warp accepted points BEHIND camera B (`depth*1.0002 >= z` is vacuous for `z <= 0`;
  CUDA already rejected them) — a genuine CPU/CUDA divergence, fixed at the warp.
- **F03** `OPTREFINE::init()` ran only in the RefineMesh app: the Viewer and the Python bindings
  refined with every knob at ZERO, i.e. with the accepted WP4 gates switched off. Initialized next to
  `OPTDENSE::init()` at all three entry points.
- **F04** the CUDA constructor never recovered missing neighbour views (the CPU did), so CUDA was
  unreachable on a scene handed to the refiner without `SelectNeighborViews` having run; one shared
  `SelectRefineNeighbors` now serves both constructors.
- **Found on the way, not in the review: `MeshRefine::ScoreMesh` waited for `threads.GetSize()`
  smoothing jobs but queued `ceil(V/ceil(V/T))` of them — fewer whenever the vertex count is not a
  multiple of the chunking, a deadlock reachable from the CLI (it hung the first planar-hook run with
  every thread idle). Fixed by waiting for the jobs actually queued.**
- Also: `ResetBudget()` resets the reject streak between phases (F08; a `MaxRejects` stop in phase A
  used to end phase B after one evaluation — a declared schedule change); `--reduce-memory` is a
  documented no-op at the CLI and gone from the library API and the Viewer (F14); the CUDA entry
  refuses `--planar-vertex-ratio` loudly and the app falls back to the CPU hook (F15); an unknown
  `nImageGradient` is rejected instead of silently measuring the default twice; the Ceres arm is
  validated like the stepper arm; the combined-gradient pass is skipped unless something reads it
  (F16); dead `numVert > 0` guard replaced by the adjacency-symmetry ASSERT on both backends (F05);
  new suite-0 tests `MeshRefineWindowStatsTest` (closed-form identities, gate boundaries, the
  `WindowArea/n` factor and a central-difference check of `dzncc`) and `RefineStepResetBudgetTest`.
- Known and recorded, not fixed: the planar hook (`--planar-vertex-ratio`, default off, CPU only)
  can leave a non-manifold mesh that the next scale's `Mesh::Subdivide` ASSERTs on in Debug and
  survives in Release; its ratio is a fraction of the vertex depth, so useful values are ≈ 1e-4 to
  1e-3, not the 0.02 the Viewer tooltip suggests. `--gpu-device -2` still routes through
  `RefineMeshCUDA` and logs one fallback line.

**2026-09-02 02:30 — WP-F re-measure: F08 REGRESSES; two stepper defects found in the traces.**
The fix-batch binary (`bench/bin_refine_wpf`, defaults unchanged) against the HEAD pin, CUDA L1
(tag `wpf-confirm` vs `wp7-stencil`): Ignatius 0.7494 → **0.7357 (−0.0137)**, Truck −0.0042,
Barn −0.0013, Meetingroom +0.0001; with the central stencil −0.0095 / −0.0008 / +0.0004 / −0.0018.
CPU Ignatius on the same binary: 0.7347 baseline, 0.7684 central (the stencil gain holds on the
CPU: +0.034). The per-evaluation traces of the two Ignatius CUDA runs are identical through the
first six evaluations of scale 0 — same S, same accept/reject, same first-scale subdivision — so
the pair set (F04) and the warp (F02) are unchanged; they diverge exactly where F08 acts: phase B
of scale 0 used to end after one rejected evaluation (the reject streak carried over), and with the
streak reset it accepts two more tiny steps (S 0.1003 → 0.0986), after which the scale-1
subdivision differs by 0.5 % (413,600 vs 415,751 faces) and the fine scale ends at a worse S
(0.1952 vs 0.1945) and F1. **Calibrated (tag `decision-noise`, HEAD pin, CUDA): perturbing the initial step by ±2 %
(`--gradient-step 45.049` / `45.051`, a different trajectory with 26 vs 30 evaluations) moves
Ignatius by +0.0021 / +0.0017 and Truck by +0.0001 / +0.0001 — a changed trajectory is worth
≈ 0.002, the same size as the identical-binary floor, so F08's −0.0137 is a real effect. F08 is
REVERTED (§6.7): the reject streak carries over between phases as it did.**

The same traces expose two properties of the stepper on a real scene that Tiny never showed:
1. **The first step of every scale is ~10× too large.** Ignatius scale 0: S 0.0998 → **0.2894**
   (+190 %) at the 0.55-px median step, then four halvings (0.275 → 0.017 px) all rejected; the
   accepted steps later in the run are 0.01–0.05 px. Scale 1 repeats it (0.201 → 0.289). Every
   scale burns 4–5 evaluations of its budget backing off from `eta0`, and on Ignatius scale 0
   never accepts a single step. The initial step is CLI-selectable (`--gradient-step N.s`, `s×10`
   px), so `eta0` = 0.1 / 0.2 px is a free experiment (E5) queued on the HEAD pin.
2. **A scale that gives up on rejections ends at an unevaluated state.** After the reject streak
   the vertices sit at `v_prev + Δ/2^k` (each REJECT undoes half), whose S was never measured;
   Ignatius scale 0 ended at S 0.1003, above its own start 0.0998. The principled ending is either
   a restore to the last accepted state or one more evaluation of the half-undone one; F08's extra
   phase-B evaluations are the second option by accident — decided with the calibration.

**2026-09-02 05:10 — B4 synthetic end-to-end test LANDED (`MVS::MeshRefineSyntheticTest`, suite
2, right after `PipelineTest`).** The photometric pipeline finally has automated coverage: a
textured plane (blurred seeded noise + checker), four 640×480 cameras at f = 600 (footprint
5e-3), a 10×10 input grid displaced by Gaussian noise (3 px) plus a 10-px sinusoid, refined with
the shipped defaults on `Scene(1)`. Measured: CPU RMS to the plane 0.0312 → **0.000344** (0.07
footprints; bar 0.5), CUDA 0.000355 (ratio 1.033 to the CPU; bar ±10 %), 421 vertices, 45+7 / 22+3
evaluations, 33 s; the same scene at 100× scale subdivides to the identical vertex count and
converges to RMS/100 within 1.3 % (tolerance 2 %: 100 is not a power of two, so the rescale
perturbs the last bits and the accept/reject path drifts — counts bit-identical at every stage).
Two library defects it exposed, queued for the next fix batch: `MeshRefine`'s static thread list
is never cleared after the destructor joins it, so the second CPU refinement in one process
(Viewer, Python) trips `ASSERT(threads.IsEmpty())` (harmless under `_HEADLESS_DEBUG`, a violated
contract nonetheless); and `Scene::nCalibratedImages` is uninitialized for a scene built without
`LoadInterface`, which makes `SelectNeighborViews` clamp its neighbour count to garbage.

**2026-09-02 05:40 — WP5 per-view keep masks LANDED (not benchable on T&T, proven functionally).**
`RefineMesh` gains `--mask-path` / `--ignore-mask-label` with DensifyPointCloud's semantics (no `-m`
short alias: taken by `--mesh-file`); a non-mutating `DepthEstimator::ImportKeepMask` replaces the
`const_cast` import at the densify call sites (the refiner reloads every image at several scales,
so the old in-place resize of `Image::mask` could not serve it; the segmentation reader loads its
own full-resolution mask, so nothing depended on that side effect); `PrepareRefineImageMask` builds
the per-view keep-mask at the working size for both backends; the CPU warp skips masked A pixels
before back-projection and rejects a projection whose rounded B tap is masked (same tap as the
occlusion test); `kernelImageMeshWarp` gets two null-tolerant pointers and applies the same two
tests. Proof: default path byte-identical (`refined.ply`, 57 identical `S:` lines); with a
left-half mask on Tiny the exported pair mask has 0 of 153,280 left-half pixels set on both
backends (71,171 without), CUDA within 1 px of the CPU on the right half.

**2026-09-02 06:20 — deterministic CUDA accumulation LANDED (the queued §7.3 item).** The
pixel-parallel scatter with float `atomicAdd`s is replaced by a face-parallel accumulation
(`kernelAccumulateFacePhoto`: one thread per mesh face walks its clipped bounding box in fixed
scan order and reduces the corner sums, the pixel count, the pixel-weight normalizer and the
footprint minimum in registers — iterating all mesh faces removes any per-view face index map,
since `faceMap` only ever holds ids the rasterizer wrote) and a vertex-parallel gather over the
uploaded incident-face lists in fixed order (`kernelGatherVertexPhoto`, which also absorbs the
per-direction `photoGradNorm += 1` and retires the `photoGradPixels` buffer). The last float
atomic — the per-block `sumR`/`sumRZ` reduction of the window-statistics kernel — became per-block
slots folded by a single-block kernel in block order. Cost: 20 B per face (32 with the WP6
pixel-weight normalizer) plus the flattened incident-face adjacency. Verified on Tiny (Debug):
before, two identical runs diverged at scale-0 evaluation 7 and ended at different meshes
(338,524 vs 339,791 bytes); after, three runs give byte-identical `refined.ply` and 57 identical
`S:` lines. Equivalence with the old scatter (one binary, both paths, iteration 0, 8,120
vertices): `photoGradNorm` bit-identical, `photo` cosine 1.000000000000, max relative difference
2.3e-5 on a vertex whose sum cancels to 0.4 % of the median magnitude (reassociation), CPU-vs-CUDA
cosine unchanged (0.99998). The host-side mesh processing between scales was already
reproducible. The CUDA noise floor of §3 (≈ 0.0013 on Ignatius/Barn) should now be exactly 0; it
is re-measured with the final binary (§8 next steps).

**2026-09-02 08:10 — B5 LANDED as a consistent energy/gradient pair (CPU energy mode; Ceres
itself stays unmeasured: `OpenMVS_USE_CERES=OFF` in this build).** `MeshRefine::ScoreMesh` gains
an energy mode, used only by the `--gradient-step 0` arm, that returns exactly the functional
whose gradient it fills: `E_photo = Σ_pairs RegScale_p·Σ_c r_c(1−ZNCC_c)` with the raw per-vertex
gradient sum (no pair-count division) and the EXACT per-pixel derivative of the windowed sum
(four extra box filters: `∂E/∂B_p = −(A_p·S1 − S2 − B_p·S3 + S4)`), plus
`E_smooth = w·½Σ_interior‖L v‖²` with `∇ = w·LᵀL v` (`ComputeSmoothnessGradientLtL`, dividing by
the neighbour's valence — not the Hernandez level-2 operator the stepper uses). A finite-difference
gate test (`MeshRefineEnergyGradientTest`, suite 2, on the synthetic plane fixture, central
differences at 0.2 and 0.1 footprints) passes at **0.32 % / 0.95 % (photometric), 0.001 %
(smoothness), 0.32 % / 0.95 % (combined)** against a 5 % bar. Three things the gate exposed, all
energy-mode only: (1) **the derivative consistent with the energy is the derivative of the
bilinear interpolant the warp samples, not a precomputed stencil image** — the stencils mismatch
it by 109 % (3x5), 21 % (central), 60 % (Sobel), the same ordering WP7 measured, so a fourth
stencil mode (the interpolant's own derivative) is added for one measurement (§8 next steps);
(2) a pixel rejected by a gate still enters its neighbours' window sums, so the energy depends on
it and its derivative support must not be pruned; (3) the boundary row of `LᵀL` must not be
zeroed (a boundary vertex moves its interior neighbours' umbrellas). The Ceres block was updated
by inspection only: LBFGS rank 5, Wolfe, approximate eigenvalue scaling, the stepper's per-scale
cap as `max_num_iterations`, an iteration callback that snapshots the best state so a solver
FAILURE applies the snapshot and continues to the next scale instead of aborting the refinement;
a per-iteration `E:` log line `bench/refine_log.py` does not yet parse. Also fixed on the way
(found by the synthetic test): `MeshRefine`'s static thread list is released after the join (the
second CPU refinement in one process no longer trips `ASSERT(threads.IsEmpty())`), and
`Scene::nCalibratedImages` is initialized to 0 in the constructor with its contract documented.

**CURRENT STATUS (2026-08-31, read this first; everything below the arm-2 blocks is the
chronological log, oldest last).** Part 0 complete. Part A WP0–WP2 complete (CPU/CUDA parity reached
and reviewed; six CUDA deviations found and fixed). Part B B0/B1/B2/B4 complete: both backends run
the shared pixel-unit stepper, and **arm 2 is accepted (§5)** — mean `d_base_vis_f1` +0.0130 (CPU) /
+0.0126 (CUDA) with no scene regressing, the two backends agreeing to 0.0005 on the acceptance set,
and CUDA 2.5-4.7x faster than the CPU at 0.32-0.87x the host RSS. The branch's own remaining work, in
the order the plan puts it:

1. **Part A WP3** — scale-free grazing-aware visibility: **implemented, measured and REJECTED**
  (§6.0, −0.0047 mean at base 1 and −0.0056 at base 5). The simpler relative nearest-tap rule
  was later accepted (§6.0b); the H7 scale test can still fail through the other terms in §1.8.
2. **Part A WP4** — masked window statistics + rejection gates: **implemented and ACCEPTED**
   (§5, +0.0028 mean CUDA / +0.0033 Ignatius CPU, at 0.91x CUDA and 0.70x CPU wall).
3. **Part A WP5/WP6/WP7/WP8** — per-view keep masks, the bounded/vote photometric term (the
   `Terms::bounded` path the stepper already supports), the derivative stencil A/B, boundary
   vertices. Still open; WP6 is the one the oracle diagnostic points at.
3. **Part B B5/B6** — the Ceres arm on a consistent cost/gradient pair, and the per-vertex arms
   (rprop/adam/bb). Both are now *rejected at the entry point* rather than silently falling back, so
   implementing one means removing its rejection.
4. **Decimation policy — CLOSED by design decision (user, 2026-09-02).** The decimate-then-remesh
   pass is the deliberate preparation of the input mesh to the face size the refinement expects
   (the paper's 16-px subdivision rule), not a deviation to be swept; §7.1 stays as a description
   of its cost (Ignatius `d_in` ≈ −0.087), and no `--decimate`/`--max-face-area` arm is run.
5. **The oracle regression (§5, last table)** — arm 2 makes the fountain oracle's mean-distance
   recovery worse (−99.6 % → −181 % on `standard`, −1851 % → −1955 % on `gt-fixedpoint`) while
   improving F at τ on both. The optimizer cannot fix this by construction; it is evidence for
   items 1/2/4 above and the number to re-measure after WP6 lands. Both cells were re-run on the
   final binary, so the comparison is like-for-like.
6. **Queued small items** — fixed-point `photoGrad` atomics for CUDA determinism (§7.3); the
   `vertexDepth` → `footprint` migration of the planar-vertex hook.

Done since the arm-2 blocks below were written: the Meetingroom wall was re-measured solo (0.84x,
contention confirmed) and doubles as the final-binary confirmation cell; the CUDA backend was
measured on all six GT scenes (§5).

**ARM 2 (proportional pixel step) IMPLEMENTED 2026-08-31 09:15 — the reformulation the
four-controls table pointed at.** `MeshRefineStep::Evaluate` no longer normalizes per vertex: the
photometric step is `P_v = g_v/(kappa*m)` with `m` the per-scale median of `|g_v|/s_v` — one GLOBAL
conversion factor that makes `eta` the pixel step of the *median* seen vertex — and the unit clamp
is gone, so every vertex moves in proportion to its own raw gradient exactly as the legacy update
did. `s_v` now only converts lengths to pixels for the `eta` cap and the stop rule. Relative to
arm 1 this deletes the two mechanisms the controls left standing (per-vertex `1/s_v` in the
direction, and the `|d_v| <= 1` clamp that flattened the gradient distribution); everything else —
bold driver, `S`, phases, planar hook, `--gradient-step` mapping — is unchanged. Functional check
(Tiny L0): Debug run clean (0 `[ASSERT]` lines over both scales), and the driver visibly does more
work than under arm 1 — the unclamped first 0.55 px step overshoots (S 0.0704 → 0.0922, +31 %,
rejected twice) before settling, then S falls monotonically across accepted evaluations to 0.0665;
55 evaluations against the legacy 67 and arm 1's 24, wall 1.97 s vs the legacy pin's 1.72 s.

**ARM 2 PASSES THE GATE — the campaign's first accepted arm (2026-08-31 10:23).** Mean
`d_base_vis_f1` **+0.0130** over the 4 T&T scenes with **no scene below +0.0016**, and it also gains
on both EPFL mesh-GT scenes it was not tuned on (fountain +0.0113, Herz-Jesu +0.0187) — 6 of 6
positive, at a pooled median **0.93x** the wall and 1.015x peak RSS. Full table, per-criterion
verdict and the two caveats (Meetingroom's contended wall measurement; Ignatius's untouched
structural `d_in`) are in §5. The one-line mechanism: **restoring proportionality to `g_v` while
keeping the pixel unit turns a −0.0215 regression into a +0.0130 gain**, which is the four-controls
table's prediction confirmed — every other component of the design (bold driver, `S`, phases, stop
rule) was already right, and the per-vertex normalisation was the whole defect.

The measurement's provenance, stated for the record: the numbers come from the RelWithDebInfo binary
built at 09:14 carrying B0 + B1 + arm 2 + the CUDA/CPU border fixes. The silent-failure and
comment fixes applied after 10:30 (entry validation for `--regularity-weight`/`nOptimizer`/Ceres-less
`--gradient-step 0`, the `sumR == 0` runtime failure, export error reporting, the mesh snapshot
before a CUDA attempt, Viewer slider ranges) are all validation and error-path changes that cannot
alter a run whose inputs are valid, so the table stands; a confirmation cell on the final binary is
listed in the open items below.

**ROOT CAUSE FOUND AND FIXED 2026-08-31 00:20 (the bug described below).** `compute-sanitizer
memcheck` reproduced it on the real trajectory and named it outright:

```
Invalid __global__ read of size 4 bytes
  at MVS::CUDA::kernelImageMeshWarp+0x8d0
  Access to 0xd21fff0fc is out of bounds
  and is 3,396,604,021 bytes after the nearest allocation at 0xc57400000 of size 4,972,680 bytes
```

A 4-byte (`float`) read 3.4 GB past a 4,972,680-byte allocation — i.e. `depthMapB` indexed with a
wrapped offset. The hypothesis was right: **the B-side border guard converted the projection to int
before range-checking it.** `pz` can be positive but arbitrarily small at a grazing projection, which
makes `xB`/`yB` enormous; `__float2int_rd` then saturates to `INT_MAX`, and the upper-bound test
`ixB + 1 < size - Border` overflows to `INT_MIN` and *passes*. `idxB = iyB*widthB + ixB` is then a
wrapped garbage index. It is intermittent because whether any projection reaches that regime depends
on the vertex trajectory, which drifts run to run on both backends.

**Fix (cause, not symptom): range-check in float, before the int conversion.** Since `Border` and the
image sizes are integers, `xB >= Border && xB < size-Border-1` is *exactly* the floor-based test it
replaces, and it rejects huge values and NaN instead of wrapping; the int conversion is followed by
an `ASSERT` restating the contract. **The identical hole exists on the CPU** in
`MeshRefine::IsDepthSimilar` and is fixed the same way — there an out-of-bounds read only returned
whatever heap followed the depth map, silently, which is why it survived in production for years.
**Silent-failure fix:** `Scene::RefineMeshCUDA` now calls `cuCtxSynchronize()` once per evaluation
and gives up when the context is poisoned, so the app falls back to the CPU path instead of writing
a half-refined mesh with a zero exit code.

**Verified 2026-08-31 00:56 — 0 faults in 9 runs, against a pre-fix rate of 2 in 3.** Six short runs
(`--gradient-step 8.05`, the minimum 8 iterations per scale, which still crosses the scale-0 → 1
transition where every observed fault happened) at 43-44 s, plus three at the exact shipped default
that had produced 0.4257 / 0.4256 / 0.6500, at 76-77 s — matching the single clean pre-fix run's
75.3 s. Every run reached its "Mesh refinement completed" line with a real face count.
*Harness lesson worth keeping:* the first version of that verification script reported a triumphant
"0/9" while the binary had never launched (`rc=127`, `wall=0s`, `-w` wrongly pointing at the scratch
directory instead of the dataset, so image paths did not resolve) — it counted only CUDA errors, so
a run that never started scored as a pass. A verification harness must require positive evidence of
success (exit 0 **and** the completion line **and** no error lines), never merely the absence of a
failure signal; that is the same silent-failure shape as the product bug it was written to check.

**BLOCKING BUG, found 2026-08-30 23:05 — the CUDA backend dies on any real scene.** On Ignatius L1
the CUDA run raises `CUDA_ERROR_ILLEGAL_ADDRESS` (code 700) at the scale-0 → scale-1 transition and
then logs it **85,364 times**: once the context is poisoned every subsequent CUDA call fails, the
per-scale loop keeps iterating on a mesh nothing is updating any more, and the app still exits
`rc=0` with a plausible-looking mesh. The resulting F1 is **0.4257 against the CPU's 0.6495** on the
same frozen input — so no CUDA benchmark number can be trusted until this closes, and a
silent-failure fix (an illegal access must abort the refinement, not produce a mesh) is part of the
work. **It is INTERMITTENT: 2 of 3 identical runs faulted, the third completed cleanly** — which is
what rules out a systematic indexing mistake and points at a race or a trajectory-dependent access
(both backends drift run to run, subdividing to 414,426 / 414,470 / 414,458 faces on CPU and
414,460 / 414,360 / 414,364 on CUDA). Almost certainly **pre-existing, not introduced by WP1/WP2**:
`results.csv` shows CUDA had only ever been run on `Tiny` before tonight, and `Tiny` is clean
(0 errors, both scales). WP1's winding-cull fix plausibly unmasked it — before that only 0.1 % of
faces rasterised at all. Diagnosis is running under `compute-sanitizer memcheck` (2-in-3 should
reproduce; note memcheck serialises kernels and can *hide* a race, so a clean run is not an
acquittal). One hypothesis, explicitly **unverified**, is an integer-overflow hole in a border
guard: `ixB = __float2int_rd(xB)` saturates to `INT_MAX` for a projection with a tiny positive `pz`,
and the upper-bound test `ixB + 1 < size - Border` then wraps to `INT_MIN` and *passes*, so
`depthMapB[idxB…]` is indexed with garbage; the CPU carries the same shape in `IsDepthSimilar`, but
an out-of-bounds read there only returns garbage, which is why it has survived in production while
CUDA dies. **The prize:** the one clean CUDA run scored **F1 0.6500 vs the CPU's 0.6496 at 75 s vs
348 s** (§3) — equivalent surface, 4.6× faster.

**ARM 1 FAILS THE GATE, 2026-08-31 01:34.** CPU, L1, one run per scene (the derived gate allows n=1
at nf ≤ 0.0002); baseline = the `noisefloor` rows of §4.

| scene | baseline `ref_vis_f1` | arm 1 | **d_base** | baseline `d_in` | arm 1 evals / wall s | baseline evals / wall s |
|---|---|---|---|---|---|---|
| Ignatius | 0.6490 | 0.6679 | **+0.0189** | −0.0937 | 20 / 179 | 67 / 381 |
| Truck | 0.6235 | 0.5808 | **−0.0427** | −0.0371 | 19 / 187 | 67 / 406 |
| Barn | 0.6531 | 0.6290 | **−0.0241** | **+0.0221** | 21 / 386 | 67 / 809 |
| Meetingroom | 0.4033 | 0.3654 | **−0.0379** | +0.0007 | 32 / 391 | 67 / 573 |
| **mean** | | | **−0.0215** | | | |

Gate criterion 2 wants mean `d_base` ≥ +0.002 and criterion 3 wants no scene below −0.002; this is
−0.0215 with three scenes 10-20x past the floor. **Arm 1 is a clear regression and does not ship as
the default in this form.** It is uniformly ~2x faster (19-32 evaluations against 67), so the speed
half of the design works.

**The Ignatius result was me reading one scene as a trend — it is the outlier, not the rule.** The
sign of `d_base` tracks the sign of the *baseline's* `d_in` almost perfectly: refinement HURTS on
Ignatius (−0.0937) and arm 1 gains there; refinement HELPS on Barn (+0.0221) and arm 1 loses most
there. That looked like the signature of an arm that simply **refines less**.

**That explanation is REFUTED by the control (2026-08-31 02:08).** The legacy optimizer at
`--gradient-step 20.05` (30 evaluations instead of 67, the develop pin, so the *only* change from the
baseline is how long it runs):

| scene | baseline (67 evals) | **arm 1** (19-32) | d_base | **ctrl: legacy at 30 evals** | d_base |
|---|---|---|---|---|---|
| Ignatius | 0.6490 | 0.6679 | **+0.0189** | 0.6287 | **−0.0203** |
| Truck | 0.6235 | 0.5808 | **−0.0427** | 0.6387 | **+0.0152** |
| Barn | 0.6531 | 0.6290 | **−0.0241** | 0.6503 | −0.0028 |
| Meetingroom | 0.4033 | 0.3654 | **−0.0379** | 0.4052 | +0.0019 |
| **mean** | | | **−0.0215** | | **−0.0015** |
| mean wall | 542 s | 286 s (1.90x) | | 263 s (2.06x) | |

**The two disagree in sign on 3 of 4 scenes, including both scenes that drove the story**: simply
running the legacy optimizer for fewer iterations makes Ignatius *worse* (−0.0203) where arm 1 was
better, and Truck *better* (+0.0152) where arm 1 was much worse. So arm 1's per-scene pattern is not
an artefact of doing less work, and F1 is not monotone in iteration count (Ignatius: 0.6287 at 30
evaluations is below *both* 0.6679 at arm 1's 20 and 0.6490 at the shipped 67).

**Revised reading: arm 1 is worse per evaluation.** At a comparable budget the legacy optimizer is
nearly gate-neutral (mean −0.0015) while arm 1 is −0.0215.

**DEFECT FOUND in the per-iteration log (2026-08-31 02:20) — `eps_stop` and the step controller are
coupled, and the controller strangles itself.** Truck L1, scale 0, verbatim:

```
 1. S: 0.12498            step: 0.550px  med: 0.278px  acc
 2. S: 0.12467 (-2.4e-03) step: 0.605px  med: 0.204px  acc
 3. S: 0.12518 (+4.0e-03) step: 0.302px  med: 0.000px  rej
 4. S: 0.12447 (-1.7e-03) step: 0.333px  med: 0.091px  acc
 5. S: 0.12466 (+1.6e-03) step: 0.166px  med: 0.000px  rej
 6. S: 0.12446 (-3.1e-05) step: 0.183px  med: 0.044px  acc
 7. S: 0.12452 (+4.2e-04) step: 0.092px  med: 0.000px  rej
 8. S: 0.12446 (-2.5e-05) step: 0.101px  med: 0.023px  acc   <- STOP: med < eps_stop 0.05
```

Once `S` reaches a plateau the driver oscillates accept/reject, and because a rejection multiplies
`eta` by 0.5 while an acceptance multiplies by only 1.1, **each cycle ratchets `eta` down by 0.55**:
0.605 → 0.101 px in five evaluations. `medianPx` is proportional to `eta`, so `eps_stop` then reads
the controller's own collapse as convergence and ends the scale with `S` improved by 0.4 %. The scale
is not converged; the optimizer has throttled itself and then mistaken that for success. This is a
design bug in the stepper as specified — the plan's stop rule reads a controller state, not a surface
property — and it explains the whole arm-1 pattern without any appeal to the direction being wrong.

**Fix applied: measure the stop rule at a full step.** `medianAtFullStep = medianPx * StepMax / eta`
is what the median vertex *would* move at `eta_max`, i.e. a pure property of the direction field,
independent of whatever the driver has done to `eta`.

**It is a real bug fix and it is NOT the cause of the deficit (arm 1b, 2026-08-31 02:52):**

| scene | baseline | arm 1 | **arm 1b (stop rule fixed)** | d_base (1b) | evals 1 → 1b |
|---|---|---|---|---|---|
| Ignatius | 0.6490 | 0.6679 | 0.6685 | +0.0195 | 20 → 30 |
| Truck | 0.6235 | 0.5808 | 0.5822 | −0.0413 | 19 → 32 |
| Barn | 0.6531 | 0.6290 | 0.6287 | −0.0244 | 21 → 34 |
| Meetingroom | 0.4033 | 0.3654 | 0.3661 | −0.0372 | 32 → 37 |
| **mean** | | **−0.0215** | **−0.0209** | | |

The scales now run 50-70 % more evaluations, and F1 moves by at most 0.0014. **Running arm 1's step
longer changes essentially nothing, so the deficit is in the step itself.** Both the "it refines
less" and the "it stops too early" explanations are now dead, each killed by its own control. What
remains is the step formulation — the median normalisation, the unit clamp, and `S`-based rejection.
**The bold driver is EXONERATED — switching it off makes things worse (`nOptimizer=1`, 2026-08-31
03:11):** Truck 0.5594 (`d_base` −0.0641, against the bold arm's −0.0413) and Barn 0.6107 (−0.0424
against −0.0244). Rejecting steps that raise `S` is *helping*; the accept/reject machinery is not
the defect, and the `S`-blocks-smoothness worry does not survive either. Three controls, three
explanations killed:

| explanation | control | verdict |
|---|---|---|
| "it just refines less" | legacy optimizer at 30 evaluations | refuted: opposite sign on 3 of 4 scenes |
| "it stops too early" | stop rule decoupled from `eta` | refuted: +50-70 % evaluations, ΔF1 ≤ 0.0014 |
| "`S`-rejection blocks the smoothness term" | bold driver off | refuted: without it, −0.0641 on Truck |

**What is left is the normalisation itself, and there is a concrete unit argument for it.** Legacy
moved `delta = -gstep*(g_v + w*R_v)` — both energies scaled by one common factor. Arm 1 moves
`delta = -eta*(s_v*d_v + w*R_v)` with `d_v = g_v/(kappa*m*s_v)`, so the **photometric** half is
divided by `kappa*m` and the **regularisation** half is not: the ratio between the two energies is
`1/(kappa*m)` times the legacy one. With `kappa*m > 1` the arm is systematically over-smoothed
relative to the shipped optimizer — which is exactly the shape of the results, since it loses most
on the scenes where refinement *adds* detail (Barn, Truck) and "wins" only where refinement is
destructive (Ignatius). `w` was tuned for the legacy ratio and was never re-derived for the new
units; the plan changed the photometric normalisation without touching `fRegularityWeight`.
**REFUTED as well (2026-08-31 03:34).** `w = 0.02` (10x lower, so 10x *less* smoothing) makes both
scenes **worse**: Truck 0.5730 against 0.5822 at the default `w`, Barn 0.6044 against 0.6287. The arm
is not over-smoothed; if anything it is under-smoothed, and the regularisation is doing useful work.

**Four controls, four refuted explanations** — worth listing because each one narrows the target and
none of them is the answer the plan assumed:

| # | explanation | control | verdict |
|---|---|---|---|
| 1 | it just refines less | legacy optimizer at 30 evaluations | refuted: opposite sign on 3 of 4 scenes |
| 2 | it stops too early | stop rule decoupled from `eta` | refuted: +50-70 % evaluations, ΔF1 ≤ 0.0014 |
| 3 | `S`-rejection blocks the smoothness term | bold driver off (`nOptimizer=1`) | refuted: worse, −0.0641 on Truck |
| 4 | photometric/regularisation imbalance → over-smoothing | `w` 0.2 → 0.02 | refuted: worse on both scenes |

**Where that leaves it: the normalised, unit-clamped photometric direction is itself the defect, and
the regularisation is what keeps the arm from being worse still.** Control 4 turned the imbalance
argument on its head — reducing smoothing hurts, so the photometric half is contributing error, not
detail. The mechanism that fits every result is the clamp: `d_v` is divided by a per-scale median and
then clipped to `|d_v| <= 1`, so every vertex whose gradient exceeds `kappa*m` moves the *same*
`eta` px regardless of how much larger its gradient is. The legacy update moved each vertex in
proportion to its gradient; arm 1 flattens that distribution, giving low-confidence vertices the same
authority as high-confidence ones. That predicts exactly what is observed — worse per evaluation,
unhelped by running longer, and partly rescued by smoothing.

**The Truck bracket confirms the sign (2026-08-31 03:47), and it is damning for the photometric
half:**

| `w` | 0.02 | 0.2 (default) | 0.5 | 1.0 (the `eta_max*w <= 1` limit) | legacy baseline (`w` = 0.2) |
|---|---|---|---|---|---|
| Truck `ref_vis_f1` | 0.5730 | 0.5822 | 0.5884 | 0.5982 | **0.6235** |

F1 rises **monotonically** as the regularisation is strengthened, right up to the stability limit —
the more arm 1's photometric term is relatively suppressed, the better it does. And yet the legacy
optimizer, using *its* photometric term at `w = 0.2`, beats every point on that curve. So this is not
"the smoothing weight needs retuning": **arm 1's photometric direction contributes error where the
legacy one contributes signal**, and the only things separating the two are the per-scale median
normalisation and the unit clamp. Extrapolated, arm 1 would do best with its photometric term turned
off entirely, which is a verdict on the formulation rather than a tuning result.

**Recommended next step (not yet run): drop the clamp and the median normalisation, keep the pixel
unit.** Move each vertex proportionally to `g_v` as the legacy update does, and use `s_v` only to
convert the resulting step into pixels for the step-length bookkeeping, the `eta` cap and the stop
rule — never to rescale the direction by a median or to clip its tail. That keeps everything Part B
actually wanted (scene-invariant step lengths, a pixel-unit stop rule, `S` as the reported score)
while restoring the proportionality the evidence says is load-bearing; `kappa` then only sets the
initial step and the bold driver — which these controls showed is helping — adapts it from there.

Note in passing, since it is a measured number and not a candidate: the control is 2.06x faster at
mean −0.0015, but it fails the §3 speed route on Ignatius (|−0.0203| >> 0.002), so "just run fewer
legacy iterations" is not a shippable speed arm either.

For the record the other two rows from the same session: post-fix **CUDA (still the legacy loop,
B2 pending) scores 0.6500 at 77.5 s** against the CPU baseline's 0.6496 at 348 s — equivalent
surface, 4.5x faster, which is the parity campaign's payoff and is unaffected by the arm-1 verdict.
Caveat on arm 1 that remains either way: it also carries the `IsDepthSimilar` overflow fix, which is
exactly equivalent to the old test on every finite projection but is not separated by an experiment.

**Status 2026-08-30 22:40 (historical — superseded by the arm-2 block at the top of this section;
the bullets below are the chronological log):**
Part 0 complete. Part A WP0–WP2 landed and reviewed: the CUDA backend now reproduces the CPU's
per-vertex gradient on `Tiny` to cosine 1.000000 / rel-RMS 0.0008 (WP2 table at the end of this
section); six CUDA deviations were found and fixed on the way (image-gradient stencil,
`smoothGrad1` sign, half-float storage, rasterizer far-edge off-by-one, rasterizer payload race,
warp depth rule). **All 12 directed pairs verified in RelWithDebInfo** (see the 12-pair sweep
bullet below) and an independent code review of the whole WP2 diff is closed (review bullet
below). Known, documented, not chased: fp32 texture-unit bilinear weights and float `atomicAdd`
order (CUDA is not bit-reproducible run to run; the CPU multi-threaded path has the same
property; fixed-point accumulation is the cheap fix, queued). Binaries pinned at
`bench/bin_refine_wp1/` (see its `MANIFEST.txt`); with that pin `Tiny` L0 runs **67 iterations over
2 scales on both backends** (CPU 1.72 s, CUDA 0.813 s).

**End of session 2026-08-31 03:50 (historical — the "next steps" listed here were done the
following morning; see the arm-2 block at the top of this section).** B0 landed (CPU producers `S` and
per-vertex `footprint`; the plan's variance-guard premise refuted). B1 landed (stepper +
`--gradient-step` re-interpretation + `bold`/`fixed` arms + dual-format log parser). **The CUDA
backend was found to be crashing on every real scene and is now fixed and verified** (0 faults in 9
runs, from 2-in-3), which is what finally made a CUDA number measurable at all: **post-fix CUDA
scores F1 0.6500 in 77 s against the CPU baseline's 0.6496 in 348 s — an equivalent surface 4.5x
faster, and the payoff of the whole Part A parity effort.** **Arm 1 (the new optimizer) FAILS the
gate at mean `d_base` −0.0215 and does not ship in this form**; four controlled experiments each
refuted an explanation, and the surviving one is that the median normalisation + unit clamp on the
photometric direction is itself the defect — see the arm-1 block above for the table, the controls
and the recommended reformulation. Next, in order: reformulate the photometric step per that
recommendation and re-run the 4-scene gate; then B4 tests, B2 (CUDA stepper), B3 docs; then Part A
WP3+. Two small items still queued from earlier: fixed-point `photoGrad` atomics for CUDA
determinism, and the `vertexDepth` → `footprint` migration of the planar-vertex hook.

- **B0 (CPU producers) landed 2026-08-30 23:00**, `SceneRefine.cpp` only — no `Scene.h`, no CUDA
  change needed (`ComputeLocalZNCC`/`ComputePhotometricGradient`/`vertexDepth` are all file-local to
  `MeshRefine`; `MeshRefineCUDA` has its own independent implementations).
  - **The plan's variance-guard premise is REFUTED.** `ComputeLocalVariance` already floors both
    variances unconditionally: `imageVar(r,c) = MAXF(sumSq-SQUARE(imageMean(r,c)), Real(0.0001))`.
    With a floor of 1e-4, `varA*varB >= 1e-8`, so `invSqrtVAVB = 1/sqrt(varA*varB)` can never be
    `inf`/`NaN` — not even on a perfectly flat image — and flooring only inflates the denominator,
    so `|ZNCC| <= 1` still holds. No guard was added; the invariant is now stated as an `ASSERT` at
    the point of division instead. The `#if 1`/`#else` pair in `ComputeLocalZNCC` was collapsed to
    its live branch at the same time (the dead `#else` was the windowed/box-averaged `dZNCC`).
  - `S = sumRZ/sumR` is produced: `ComputeLocalZNCC` returns a two-field `PairScore{sumRZ, sumR}`
    (a struct return rather than an out-parameter — that signature already takes nine images), and
    `ThProcessPair` adds both under the **existing** lock, over exactly the pixel set that already
    feeds the returned score. `sumR`/`sumRZ` are the **raw** sums, deliberately *not* scaled by
    `RegularizationScale` the way `scorePhoto` is — that scaling is what makes the legacy cost
    scene-dependent. `ScoreMesh` resets them, asserts `sumR > 0` and `S ∈ [0,2]`.
    **Why `S ∈ [0,2]`:** `r = minVar/(minVar+0.0015) ∈ (0,1)` strictly (`minVar >= 1e-4 > 0`), and
    `1−ZNCC ∈ [0,2]`, so `0 <= sumRZ <= 2·sumR`.
  - Per-vertex `footprint` (scene units per pixel) = min over pair-directions of
    `cameraA.GetFootprintWorld(depthA)` = `depth/focal`, the existing `Camera.h` helper. It is
    accumulated in the **same** three-vertex loop that increments `photoGradNorm`, at the point where
    `depthA` is already read and asserted positive — no extra projection pass. A `FLT_MAX` sentinel
    is resolved to 0 after all pair-directions have run, so `footprint[v] > 0` exactly where
    `c_v = photoGradNorm[v] > 0` (asserted). `photoGrad`/`photoGradNorm` semantics are untouched.
  - **`vertexDepth` was NOT removed, and the plan is wrong to call it dead.** It is the live input to
    the planar-vertex-removal pass in `Scene::RefineMesh` (`fThPlanarVertex`), which is a shipped
    CPU-only feature. It is also a *different* quantity from `footprint`: a min over pair-directions
    of the whole-image `avgDepth`, not a per-vertex `depth/focal` ratio. Migrating that consumer to
    `footprint` means changing the per-scale loop, which is B1's territory, so B0 left it alone
    rather than break the shipped loop. Carried into B1 as an explicit task.
  - Repo gotcha found on the way: `if (cond) ASSERT(x); else ...` does not compile (MSVC C2181,
    "illegal else without matching if") — `ASSERT` expands to a brace-enclosed block, so the trailing
    semicolon creates a dangling `else`. Brace both branches.
- **B1 (stepper + CPU loop) landed 2026-08-31 00:00**, new `libs/MVS/SceneRefineStep.h/.cpp` plus the
  per-scale loop rewrite in `Scene::RefineMesh`. The fixed-iteration `gstep *= 0.98` decay is gone;
  the loop is now the bold-driver stepper described in the header, working in pixels and in `S`.
  - `--gradient-step` is re-interpreted with **no new option**: `N.s` gives `cap = MAXF(N/(nScale+1), 8)`
    as before and `eta0 = s*10` **pixels**, so the shipped `45.05` still means 45/22 iterations and
    now also 0.5 px. Because the step is a pixel quantity bounded by `StepMax`, the fractional part
    has a real valid range of `(0, 0.1]`; `Scene::RefineMesh` rejects anything else at the entry
    point with a message naming the offending value, instead of letting an unhonourable step reach
    the optimizer (the legacy silent fallback to `iters=75, gstep=0.4` for `fGradientStep <= 1` is
    deliberately gone). Verified: `--gradient-step 45.5` exits 1 with
    "asks for an initial step of 5 px, outside (0, 1]".
  - `OPTREFINE::nOptimizer` selects `0 = bold` (default) or `1 = fixed`, the control arm that never
    rejects and never grows/shrinks the step — it exists to answer whether the bold driver is what
    matters, so it differs in exactly that respect and nothing else.
  - Two phases per scale as planned (A at the caller's rho with the planar hook, B at rho = 1 with
    the planar hook off, `cap_B = MAXF(3, 3*nA/7)`, step and the `S` references carried over and only
    the stall counter reset). The legacy "last iteration forces `nAlternatePair = 0`" is dropped.
  - First functional run (Tiny L0, CPU, Debug) — the bold driver is visibly working:
    ```
     1. S: 0.07036 (+0.00e+00)  step: 0.550px  med: 0.282px  v:     0  acc
     3. S: 0.06792 (-9.12e-03)  step: 0.665px  med: 0.175px  v:     0  acc
     4. S: 0.06875 (+1.23e-02)  step: 0.333px  med: 0.000px  v:     0  rej
     5. S: 0.06732 (-8.77e-03)  step: 0.366px  med: 0.077px  v:     0  acc
    ```
    `S` falls monotonically across accepted evaluations, a rise is rejected and halves the step, and
    the step never exceeds `StepMax`. **Tiny converged in 24 evaluations against the legacy 67**
    (9 at scale 0 against a cap of 45, 15 at scale 1 against 22) — the plan's "expect fewer evals
    than 45/22" holds on this scene. No F1 on Tiny; the real verdict needs the T&T arms.
  - Note for the A/B: phase B often stops on its first evaluation *when phase A converged*, because
    `numAccepted` and the step carry over so `MinIters` is already satisfied and `medianPx` is
    already below `eps_stop`. That is correct behaviour for a converged scale, not a wiring bug —
    on a scene where phase A exhausts its cap instead, phase B gets its full budget.
  - `bench/refine_log.py` parses both the legacy `f:/g:` line and the new `S:` line (41 tests green).
- **12-pair sweep, RelWithDebInfo (2026-08-30 22:07-22:15)**: every one of the 4×3 directed pairs
  on `Tiny` L0 now reports **mask IoU = 1.0000 and face-map identical = 100.000%**; per-pair
  `imageAB` rms ≈ 1.6e-4, `dzncc` rms ≈ 0.016, `sg` rms 1.5e-6…4.0e-6. The pair that used to carry
  the warp defect, `0_3`, was 0.9994 with 108 CPU-only pixels before the `DepthConstBias` fix and is
  exactly 1.0000 after — i.e. the fix is confirmed outside Debug, on every pair, not just the one it
  was diagnosed on. (Harness note: two of the twelve `mv`s of the per-pair dump directory failed with
  `Permission denied` — a transient Windows handle on the directory — so the driving script's final
  PNG-based gate could not find `parity_pairs/0_3` and the chain exited 1. The measurements
  themselves are complete and in the run log; only the automated gate step was lost.)
- **WP2 code review closed (2026-08-30 22:20-22:35)**: an independent review of the full WP2 diff
  (`SceneRefineCUDA.cu/.cpp/.inl`, `SceneRefine.cpp`, `SceneRefineCommon.h/.cpp`) against the CPU
  primitives confirmed the parity-critical math — edge-function argument order and cull sign, the
  b0/b1/b2 ↔ v1/v2/v3 mapping, perspective-correct weights and depth blend, the first-face-wins
  tie-break through the `(depthBits<<32)|tid` key, the `Border` clamp, the one-sided
  `DepthConstBias` occlusion test, and the whole `sg = (gB·(J·dA))·dZNCC·RegScale/Nd` chain. Two
  substantive findings, both about what happens when an asserted invariant is violated rather than
  about the math, and both fixed:
  1. `kernelComputePhotometricGradient` had a `pz <= 0` fallback that set `projB = (-1,-1)` and then
     *kept computing* with `pz2` — Inf/NaN into three vertices' `atomicAdd`, silently frozen for the
     rest of the run by the host's per-vertex `ISFINITE` guard. The branch is unreachable (`mask==1`
     means `kernelImageMeshWarp` already required `pz > 0` on the identical chain), so it is now the
     contract the CPU states: `ASSERT(pz > 0.f)`.
  2. `ProjectMesh` preset `faceMap` to `NO_ID` but left `depthMap` stale, so a pixel that ever missed
     its pass-2 payload would pair a previous-iteration depth with a `NO_ID` face and index
     `faces[0xFFFFFFFF]`. `depthMap` is now preset to 0 alongside, and the kernel asserts
     `faceID != NO_ID`; a missed key degrades to "uncovered pixel" instead of an OOB read.
  Also fixed: four stale/incorrect comments in the parity-critical files (an orphaned
  `MESHOPT_DEPTHCONSTBIAS` "uncomment to…" block, an `ExportPairMask` doc claiming a 0/255 caller
  that does not exist, three hard-coded `SceneRefine.cpp:NNN` line references that had already
  rotted — line numbers rot exactly like issue numbers, which the repo already bans in comments —
  and a `#if 1`/`#else` reference to a conditional that no longer exists), plus three minors (a
  defensive `!vertexBoundary.IsEmpty()` in the debug export, a redundant `#ifdef _DEBUG` around an
  `ASSERT`, and the dead `WindowSize`/`WindowArea`/`MinWindowCount` constants, removed). Re-verified
  after the fixes (Debug, 22:26, `parity_debug_review`): **all four WP2 targets still PASS** — `g`
  cosine 1.000000 / rel-RMS 0.0008, `pnorm` identical 100%, pair `0_1` mask IoU 1.0000 — with zero
  `ASSERTION FAILED` lines in any of the three run logs, so the two new device-side contracts hold
  on real data.

- **RelWithDebInfo build DONE (2026-08-30 14:11), pinned to `bench/bin_refine_develop/`** (commit
  `c410c9d4`, see that folder's `MANIFEST.txt`). One real smoke-test cell confirmed the harness
  end-to-end: `Ignatius, cpu, L1, baseline, run0` — `rc_refine=0`, `backend_det=cpu`, 67 iterations
  over 2 scales, wall 349s, peak RSS 5.06 GB (`bench/out_refine/results.csv`, tag `smoketest`).
- **First measured number, not yet a verdict (n=1, no noise floor yet): the shipped default
  REGRESSES F1 at L1 on Ignatius** — coarse-mesh (ReconstructMesh, before any refinement)
  `in_f1=0.7427` vs refined `ref_f1=0.6489` (`d_in_f1=-0.0938`, τ=0.003). Traced to source, not a
  harness bug: `SubdivideMesh` (`SceneRefine.cpp:557-558`) captures its `numVertsOld`/`numFacesOld`
  log fields *after* its own `--decimate 0` (auto) step already ran (`:505-544`) — the coarse mesh's
  3,372,579 faces get auto-decimated to ~1,270,941 *before* any photo-consistency optimization
  starts (median projected face area at L1's half-resolution images exceeded `--max-face-area`(16
  px²) by >6x, triggering the auto-decimate branch, `:526-532`), then area-based subdivision brings
  the finest scale to 410,956 faces — an 8.2x reduction from the ReconstructMesh input. A plausible
  explanation for the F1 drop: at τ=0.003 (a tight tolerance), losing this much fine surface detail
  to decimation costs more precision/recall than the photo-consistency term's positional correction
  gains back. Needs confirming at L0 (no such aggressive downsample-driven decimation expected) and
  across more scenes before it's a real finding — recorded here as the first concrete number this
  campaign has produced, not yet gated on anything (H3 hasn't run).
- **H3 DONE (2026-08-30 14:29-17:15)**: 12 noise-floor cells, all `rc_refine = 0`, dataset dirs
  clean afterwards; nf and the derived gate are in §3, the baseline table in §4. **H4 DONE**
  (`--backend cpu|cuda|both`, `--gpu-device`, `backend_mismatch`, nvidia-smi pre-flight, `--parity`
  stub, no-GT `Tiny` and mesh-GT `TinyGT` scenes; 36 unit tests). Real checks on `Tiny` (17:16,
  tags `h4test`/`h5test` in `results.csv`): `--backend cuda --gpu-device 99` → the app silently ran
  the CPU path (no "falling back" line, as documented in §1.1) and the harness flagged the row
  `backend_req=cuda backend_det=cpu backend_mismatch=1` with a console banner — a failed cell, never
  a CPU row in disguise; `--backend both` → one cpu row (16,657 faces, 1.77 s) and one cuda row
  (3,507 faces, 0.59 s, `backend_mismatch=0`); `TinyGT` (mesh2mesh path, GT = the scene's own
  coarse mesh) → coarse scores F = 1.0 against itself, the refined mesh F = 0.9986 (acc mean
  0.0051, `comp=exact-mesh`, `comp_vis_frac` 1.0). **H7 positive control on `Tiny`** (`scale_test.py`,
  baseline binary, 3 unit runs → `d_noise` 1e-6; identity control PASS bit-exact): s = 100 →
  dist-to-unit mean 0.00218 / p95 0.0070, `counts_match` **False** (4715/8574 vs 4721/8651 vertices
  per scale); s = 0.01 → 0.00225 / 0.0073, `counts_match` False → **FAIL, as required** — the
  baseline is not scale-invariant, and not even its subdivision is (the 0.05 scene-unit visibility
  bias changes which faces the face maps see, hence the projected areas). Caveat for the later PASS
  criterion: with `d_noise` = 1e-6 on a 4-image scene the `1.5·d_noise` tolerance is below float32
  round-off of the mesh code's own epsilons; on the real run (Ignatius L1, multi-threaded noise) it
  is larger, and if it is still unattainable a floor of 1 % of a pixel footprint will be added and
  documented.
- **Functionality-check scene `Tiny`** (user decision 2026-08-30: the repo's own test fixture, not
  `openMVS_sample`): `apps/Tests/data/scene.mvs` (4 images ~640 px, two HEIC) copied to gitignored
  `bench/tiny/` with `scene_dense.mvs/.ply` and `scene_dense_mesh.ply` (50,084 vertices / 99,561
  faces) generated by the pinned develop binaries (`DensifyPointCloud -v 2 --resolution-level 0`
  1 s, `ReconstructMesh` 3 s). `RefineMesh` on it: CPU 3.6 s, CUDA 0.8 s. Every functional check of
  a code change runs here first; T&T cells are only for verdicts.
- **H6 oracle mode: tool + harness plumbing done (2026-08-30 17:45), first real cell pending.**
  `oracle_degrade.py` validated on `Tiny` (§2.7); `run_refine.py --oracle <spec>` (mesh-GT scenes
  only, mutually exclusive with `--input-mesh`) degrades the GT once per (spec, seed) into
  `<scene>/oracle/<spec>-<seed>.ply`, scores it as the input (`in_*`), runs the cell on it under
  `<backend>-L<level>/<variant>/oracle-<spec>-<seed>/run<i>/`, and fills `oracle=1`,
  `degrade_spec`, `in_dist_mean` (cached input→GT), `ref_dist_mean` (refined→GT),
  `recovery_pct = 100·(1 − ref/in)`, `recovery_per_scale` from `-v 3` dumps when present
  (`mesh_compare.py`, 2 M samples); baseline deltas group by `degrade_spec`. 51 unit tests. First
  real cell (`TinyGT --oracle standard`, 18:28): recovery 68.5 % (§4). One trap found on the way:
  `oracle_degrade.py` takes ρ from the scene's `camcheck.json`, so every mesh-GT scene — the tiny
  fixture included — needs one (`bench/tiny/camcheck.json` now exists; the first attempt was a
  failed-cell row, `rc_refine = 1`, tag `h6test`, kept as the record). fountain-P11
  `standard` on the develop pin (18:48): **recovery −99.6 %** (§4) — the refinement moves a
  displaced real GT *away* from the truth; `gt-fixedpoint` (19:04): **recovery −1851 %**, the GT
  (285 k faces, F 0.9997) comes out as 24 k faces at F 0.413 — same surface the `standard` input
  and the regular baseline converge to (§4). Per-scale split (`--dumps`, 19:23): decimation
  0.00058 → 0.00283, **coarse-scale iterations 0.00283 → 0.01003 (−255 %)**, fine-scale
  iterations −12 % → the optimization itself is the loss (§4 table). The harness now scores both
  dumps per scale (`subdivided_dist_mean`, `subdivide_recovery_pct`, `iter_recovery_pct` in
  `recovery_per_scale`), so every future oracle cell carries this split.
- **CUDA rasterizer culls the wrong side — the CUDA backend has almost no photometric signal
  (found 2026-08-30 on `Tiny`, verified numerically).** Same input mesh, first scale: CPU
  `Mesh subdivided: 18969/37335 -> 4721/8838`, CUDA `5246/10054 -> 1973/3507`, and CUDA's second
  scale subdivides nothing (`1973/3507 -> 1973/3507`). `SubdivideMesh` and `ListFaceAreas` are
  verbatim copies on both sides (`SceneRefine.cpp:443-578` vs `SceneRefineCUDA.cpp:404-540`), so
  the difference is the face map. `kernelProjectMesh` (`SceneRefineCUDA.cu:108-112`) rejects a face
  when its screen-space determinant `det = e10.x·e20.y − e20.x·e10.y` is ≤ 0; the CPU rasterizer
  (`TImage::RasterizeTriangleBary`, `Types.inl:2612-2614`) culls too, keeping a triangle iff
  `EdgeFunction(p0,p1,p2) = (p2−p0)×(p1−p0) > 0` (`Util.inl:699`) — which is exactly `−det`, so
  the CPU keeps `det < 0` (front faces) and the CUDA kernel kept the opposite sign. A Python replication of
  `ListFaceAreas` on `Tiny` (raycast face ids per pixel at scale 0.5, pair-min/max, `6·median`,
  `max(0.1, ·/16)`; script `bench/tiny_face_areas_check.py`) gives: all faces → `6·median = 6.00
  px`, Clean ratio **0.375** (log: 50084→18969 vertices = **0.379**); `det>0` faces only → **0.1 %
  of the faces pass** in every camera (1-2 pixels rastered per 320×240 image), median 0, ratio
  floored at **0.100** (log: 50084→5246 = **0.105**). With the mesh's outward face orientation and
  y-down pixel coordinates, front-facing triangles project with a *negative* determinant, so the
  kernel keeps only back faces seen through holes. Consequences: (1) every CUDA `RefineMesh` result
  to date is essentially Laplacian smoothing of an over-decimated mesh; (2) Part A WP1 gets a new
  first item — cull with the CPU's sign (`det >= 0 → reject`), i.e. the same back-face cull on both
  backends (first drafted as "no cull" before the CPU's `EdgeFunction` sign was verified — a
  back face seen through a hole must not be refined against images it does not face, and the
  cull halves the fragment work); (3) the CUDA noise floor / baseline of §3-§4 is measured on the *fixed* kernel,
  not on this one — the pre-fix CUDA row is kept only as the "before" reference.
- **`bench/refine_scenes.json` and `bench/refine_variants.json` already exist** (six T&T scenes;
  `baseline` plus three exploratory regularity/scale variants) and are what actually runs today —
  the built-in fallback registries in §2.2 are a safety net, not the active path. Adding more
  scenes/variants (or real Part A/B algorithmic arms once H3's noise floor is known) is just editing
  these two files.
- **EPFL imported, camera convention settled by camcheck (2026-08-30).** fountain-P11: of the four
  `.camera` interpretations exactly one passes — `hyp2` (file `R` is cam→world, the 3-vector is the
  camera centre; world→cam = `[Rᵀ | −Rᵀ·C]`), photometric ZNCC 0.977 / 0.968 / 0.982 on the three
  best-overlap pairs; `hyp0`/`hyp1` fail geometrically (nothing in front of the cameras), `hyp3`
  (Rᵀ with the vector read as a translation) passes the geometric test with 9-37 % overlap yet
  scores ZNCC −0.04 / −0.01 / −0.01 — the case only the photometric test catches. Level-0 pixel
  footprint 0.0030 scene units (metres) → the plan's τ = 0.005 stands (rule value 2×0.003 = 0.006).
  Installed as `runOMVS/scene.mvs` + `camcheck.json` + `convention.json`. Herz-Jesu-P8: same
  outcome — `hyp2` alone passes (ZNCC 0.978 / 0.982 / 0.966; `hyp0`/`hyp1` geometric FAIL, `hyp3`
  ZNCC −0.002 / 0.021 / −0.018), τ = 0.01 per the plan. Frozen inputs produced 17:18-17:22 by
  `prepare_scene.py` with the pinned binaries (`DensifyPointCloud --resolution-level 1` →
  `ReconstructMesh`, recorded in each `runOMVS/frozen.json`): fountain-P11 coarse mesh 1,344,909
  vertices / 2,676,610 faces, Herz-Jesu-P8 891,565 / 1,770,021. Both stay `enabled: false` in
  `refine_scenes.json` (they are the diagnostic/oracle set, always named explicitly with `--scenes`;
  the default selection is the T&T acceptance set). Baseline ×3 cells (nf + first mesh-GT numbers)
  launched right after. DTU/BlendedMVS/ETH3D (H8/H9) not started.
- **Part A started 2026-08-30 17:xx (H3 and H4 done).** WP0 (scaffolding) is in the working tree,
  reviewed: `libs/MVS/SceneRefineCommon.h` (`MVS::Refine::{HalfSize=3, WindowSize=7, WindowArea=49,
  MinWindowCount=25, Border=3}`, `REFINE_HD`, `ZnccReliability` hoisted from the two identical
  expressions — `Real` is `float` on the CPU side, so bit-identical), `SceneRefineCommon.cpp`
  (`OPTREFINE` built like `OPTDENSE`: `nIgnoreMaskLabel nPhotoTerm nPhotoNorm nImageGradient
  nBoundaryMode fGateMeanDiff fGateVarRatio nOptimizer`, nothing reads them yet;
  `PrepareRefineImage` = the load/gray/blur/resize/`UpdateCamera` block both `ThInitImage` and
  `InitImages` used to duplicate, same order and flags), `--refine-config-file` in `RefineMesh.cpp`
  mirroring `--dense-config-file` (`DensifyPointCloud.cpp:270-295`), CUDA `HalfSize` 2 → 3 (7×7
  window on both backends). Debug build of MVS/RefineMesh/Tests green, `Tests.exe 2 1` green.
  Functional check on `Tiny` with the Debug binaries: both backends load the images through the
  hoisted `PrepareRefineImage`, decimate to the develop counts (CPU `Cleaned mesh: 18969/37335`,
  CUDA `5246/10054`), and the CUDA run with `--refine-config-file` logs `OPTREFINE: … photoTerm=1 …`
  at `-v 3` (config loading works) — then **both stopped at a Debug-only assertion in
  `Mesh::Subdivide` (`Mesh.cpp:1335`)** that has nothing to do with WP0: it estimated the initial
  face count as `faces.capacity()/3`, which is wrong whenever the array was shrunk before the call
  (RefineMesh always decimates first), and in Debug a failed ASSERT is a *modal dialog* — the run
  looks hung at its last log line. Fixed in `Mesh.cpp` (real initial count, `mapSplits.empty()`
  guard; the invariant protects the swap-with-last `RemoveAt` loop). The refine loop itself under the
  shared `HalfSize` is verified by WP1's functional run (same Debug build, one run per backend).
  **WP1 landed (2026-08-30 18:1x, reviewed):** (1) `kernelProjectMesh` culls with the CPU's sign
  (`det >= 0 → reject`, see the cull bullet above) and drops faces with any vertex `z ≤ 0`;
  (2) perspective-correct barycentrics + depth, stored in `baryMap` (same formula as
  `Util.inl:843`); (3) `+0.5` half-texel offset on all four `tex2D` fetches; (4) one border rule on
  both backends — the whole 2×2 read inside `Refine::Border` (the CPU's per-corner
  `isInsideWithBorder<int,3>` let a corner at Border−1 through; CUDA's 10-px `borderMin` gone);
  (5) partially visible faces: CUDA clips the bbox to `[HalfSize, size−HalfSize)` instead of 5 px
  (in fact inclusive at `size−HalfSize` until the 20:15 fix below — one extra row/column),
  the CPU `RasterMesh` gets its own `ProjectVertex` (accept any `z > 0`, per-pixel border test in
  `Raster`) so a face is no longer dropped because one vertex sits within 3 px of the edge — note
  this and (4) are the two WP1 items that change the *CPU* (production) behaviour; their effect is
  what the CPU column of the re-baseline against the develop pin measures;
  (6) `MakeCUDACamera(cameraB, views[idxImageB].size)` at both call sites; (7) CUDA dZNCC is the
  CPU's live pointwise form, same sign, the CPU's dead windowed `#else` deleted; (8) gradient
  kernel iterates `[HalfSize, size−HalfSize)` so border pixels no longer inflate `photoGradNorm`;
  (9) true valences uploaded for every vertex + a `vertBoundary` flag — the bi-Laplacian of every
  interior vertex next to the boundary was `0` before (÷0 → inf); (10) dead `vertexVertices`
  upload removed. Debug builds 0 warnings, `Tests.exe 2 1` green (8 min in Debug; it never calls
  RefineMesh). Functional on `Tiny` (Debug): CUDA first scale `18969/37335 → 4721/8838` — **identical
  to the CPU** (was `5246/10054 → 1973/3507`), second scale `→ 8781/16904` vs CPU `→ 8673/16694`
  (+1.3 %, sub-pixel rasterizer residue), both complete without NaN; CPU second scale moved from
  `8651/16657` to `8673/16694` (+0.25 %) through fixes 4/5 — the CPU baseline moves at WP1 as
  planned, `bench/bin_refine_develop` stays the frozen reference. RelWithDebInfo build done
  19:13 (58 min for `MVS.vcxproj`), pinned as `bench/bin_refine_wp1/` (87 files + MANIFEST; it also
  carries WP2's env-gated export, inert without `OMVS_REFINE_DEBUG_DIR`). That pin was superseded at
  22:44 by a rebuild carrying the warp-rule fix and the post-review contract fixes, and re-pinned to
  the same path with an updated MANIFEST — `bench/bin_refine_wp1/` is that 22:44 binary, and it is
  the reference every §3/§4 baseline number and every arm comparison below is measured against. The
  re-baseline it drove (parity `Tiny` L0 → `Tiny` both backends → Ignatius L1 ×3 both) completed at
  23:19; results in §3.
- **WP2 parity diagnostic landed (2026-08-30 18:40, reviewed).** No public option: `RefineDebug`
  (`SceneRefineCommon.h/.cpp`) is gated on `OMVS_REFINE_DEBUG_DIR` (+ `OMVS_REFINE_DEBUG_PAIR=A,B`),
  the only env-var read in the refine code, zero cost when unset (both backends guard with
  `Dir().empty()` before doing any extra work). Exports per iteration: `refine_grad_s<S>_i<I>.ply`
  (pos, combined gradient, photo gradient + `pnorm`, both smoothness terms, boundary flag — the CUDA
  side downloads the terms *before* `CombineGradients()` overwrites `photoGrad` in place) and, for
  the one named pair, `pair_A_B_s<S>_i<I>_{imageA,imageAB,zncc,dzncc,conf}.pfm` + `_mask.png`, where
  `conf = Refine::ZnccReliability(varA, varAB)` on both sides. `bench/refine_parity.py` compares
  the two dumps (cosine / relative RMS / offender fraction per field, over all vertices and over
  `pnorm > 0` on both sides; per-pair map RMS on the mask intersection, confidence-gated too; mask
  IoU; side-by-side PNGs) and prints the WP1 calibration targets as a report, not a gate.
  `run_refine.py --parity` implements the frozen-mesh protocol: one CPU run `-v 3 --scales 1
  --max-threads 1` freezes the post-`SubdivideMesh` `MeshRefine0.ply` (cached on coarse-mesh +
  binary fingerprint), then each backend runs it with `--decimate 1 --max-face-area 0
  --ensure-edge-size 0 --close-holes 0 --scales 1` — `SubdivideMesh` returns before touching the
  mesh (`bNoDecimation && bNoSimplification`, `SceneRefine.cpp:525-568`) while the scale loop still
  does `ListVertexFacesPre/Post`, so iteration 0 on both backends scores byte-identical vertices
  (`refine_parity.py` aborts otherwise). No CSV row is written. 12 unit tests + 8 subtests
  (`bench/test_refine_parity.py`). First attempt (Debug binary, 19:04) died after the freeze run
  with two latent harness bugs, both fixed: `run_mesh.PeakMemSampler` named its event `_stop`,
  shadowing `threading.Thread._stop()` that `join()` calls on Python ≤ 3.12 (the `tnt` venv is
  3.12, the session's `python` is 3.13, which is why no earlier cell hit it) → renamed
  `_stop_evt`; and the `-v 3` dump pickup was a name-set difference, blind to a same-named dump
  overwritten in place by a later run → `_snapshot_dumps()/_new_dumps()` key on (size, mtime_ns)
  at all four sites (normal cells, oracle `--dumps` per-scale, freeze, parity). Harness suite 94
  green. **First real cell, `Tiny` L0, Debug binaries, both backends (19:17):** frozen mesh
  8,120 vertices, xyz max diff 0 (protocol holds), 45 gradient dumps + 270 pair files per side,
  CPU 161 s / CUDA 4.2 s.

  | field | cosine | rel-RMS | offenders | reading |
  |---|---|---|---|---|
  | `g` combined | 0.8607 | 0.906 | 85 % | driven by `p` below |
  | `p` photo | 0.8850 | 1.022 | 95 % | per-vertex cosine median 0.998, but \|cuda\|/\|cpu\| median **1.68**, p10 0.64, p90 5.3; global LS scale 1.59 leaves 47 % residual → a per-pixel factor differs, not a constant |
  | `pnorm` | 0.99996 | 0.0085 | 0.1 % | integer-identical on 99.88 % (0 visibility disagreements): the same pixels reach the same vertices |
  | `s1` | **−1.000000** | 2.000 | 92 % | exact negation = convention only: CUDA computes `v − mean(nbrs)` and combines `+rigidity·s1`, the CPU `mean − v` and `−rigidity·s1`; `s2` identical (1e-6) because the sign cancels twice — CUDA flipped to the CPU convention, no numeric effect |
  | `s2` | 1.000000 | 0.0000 | 0 % | identical |
  | pair 0,1 mask IoU | 0.9978 | | | |
  | `imageAB` | | rms 2.7e-4 | | fp16 texture vs fp32 |
  | `dzncc` | corr 0.9999, LS scale 1.000 | rms 0.059 (edge ≤ 4 px: 0.19, interior 0.036) | | window statistics at mask edges only |
  | `zncc`/`conf` | | rms 4e-3 / 2e-3 | | same edge pixels |

  Verdict: rasterisation, warp, ZNCC and both smoothness terms agree; **the per-pixel photometric
  gradient does not** — with `dz`, bary weights and the pixel→vertex assignment matching, the
  difference is inside `sg = (gB·(J·dA))·dz·RegScale/Nd`: the image-gradient estimate (CPU:
  bilinear sample of a Sobel/8 gradient image; CUDA: forward differences of the linear-filtered
  texture) and/or the projection Jacobian. Policy (user, 2026-08-30): the CPU path is the mature
  production implementation, so a CPU/CUDA disagreement is resolved toward the CPU behaviour unless
  the CPU is shown to be wrong — every WP1/WP2 deviation so far was CUDA-side. Resolved by reading
  the code: the CPU's stencil is
  the noise-robust 3×5 Holoborodko derivative (`CreateDerivativeKernel3x5`, unit scale, then
  bilinear sampling at `xB`); CUDA's forward difference of the linearly-filtered texture is an
  unsmoothed estimate, hence the per-pixel factor rising with local texture frequency. Fix (landed
  19:27, corrected 20:00): `ComputeRefineImageGradient()` in `SceneRefineCommon`
  computes the derivative images once (honours `OPTREFINE::nImageGradient`: 0 = 3×5, 1 = central,
  2 = Sobel/8), the CPU merges them into its `imageGrad` as before, the CUDA backend uploads them
  as two **float** arrays per view (`View::imageGrad[2]`, `ArrayRT32F`, 8 B/px on top of the
  16 B/px a view already holds in image + depth/face/bary maps) and the photometric kernel samples
  them at `proj + 0.5` — the same estimator and the same bilinear sampling on both backends.
  The first version stored them as half-float (`ArrayRT16F`): the Debug CUDA run then aborted
  (exit 3, no message — the pre-`_HEADLESS_DEBUG` popup path) right after "Refine mesh at";
  the headless Debug build showed the cause in one line, `HalfFloat.h:111` firing 52,753 times:
  a derivative stencil leaves rounding residue of ~1e-8 on flat regions, below the half-float
  normal minimum 6.1e-5 that `hfloat::fromFloat` asserts on (it flushes to zero, but the contract
  says the caller must not rely on it). The CPU keeps its gradient in float, so the CUDA side now
  does the same — the value range is the CPU's, not an approximation of it. Note the earlier
  "aborts on both backends" reading was wrong: `--gpu-device -1` means *best GPU*, the CPU path
  is `-2`/`cpu`; the harness CPU cell of that run had in fact completed. The Ignatius step of the
  re-baseline was cancelled until this is verified; Tiny both-backends and the parity with the pin
  follow.
- **Parity re-run after the gradient unification (`Tiny` L0, Debug, 20:07, `parity_debug_grad32f`):**
  `p` cosine 0.8850 → **0.9971**, rel-RMS 1.022 → **0.076**; `g` 0.8607 → 0.9963 / 0.906 → 0.086;
  `s1` now 1.000000 (rel-RMS 2e-5), `s2` unchanged, `pnorm` identical on 99.95 %, mask IoU 0.9978
  (unchanged). Per vertex the photometric gradient now agrees: cosine median 1.00000 (p1 0.9966),
  |cuda|/|cpu| median 1.0003, p10–p90 0.991–1.010. The remaining rel-RMS is **10 vertices** (95 % of
  the squared difference; without the worst 50 the field is at rel-RMS 0.0038 / cosine 0.99999, i.e.
  inside the calibration targets); they have identical `pnorm` and a 5–10× magnitude gap, so a
  single pixel's `dzncc` differs. The pair maps locate it: the 419 mask pixels that differ are
  **all CUDA-only, all on row `y = H−3`** (5 runs of 12–236 px) — `kernelProjectMesh` clamped its
  inclusive bbox at `size − HalfSize`, rasterising the one row/column the CPU's per-pixel
  `Raster()` test rejects (`>= rows − Border`); the warped values written there then leak into
  every 7×7 window statistic within `HalfSize` of that row: `dzncc` error within 4 px of a
  differing pixel is 0.54–0.62 rms (rel 11–12 %) and carries 63 % of the map's squared error.
  Fixed: the CUDA bbox now ends at `size − Border − 1`. What is left after that is the fp16
  image storage: `imageAB` rms 2.7e-4 = half an fp16 ulp at the median grey level, amplified by
  `1/sqrt(varA·varB)` in low-variance windows into a `dzncc` floor of 0.036 rms (rel 0.9 %; rel
  1.9 % at conf < 0.2, 0.7 % at conf > 0.8) — a deliberate CUDA memory trade-off, not a logic
  difference, and it contributes ≤ 0.4 % to the per-vertex gradient. **Post-border-fix parity
  (20:19, `parity_debug_border`):** mask IoU **1.0000** (0 differing pixels), `dzncc` rms 0.059 →
  **0.036**, max 4.4 → **0.31** (the fp16 floor, and nothing else, is left in the pair maps);
  `pnorm` 99.95 % identical. Yet `p` is unchanged (cosine 0.9971, rel-RMS 0.076): the same 10
  vertices, same magnitudes (e.g. v584 CPU 0.145 / CUDA 0.015, v197 0.010 / 0.082), so they never
  were the border row. Their discrepancy is the size of one whole pixel contribution (0.07–0.13,
  vs |p| p99 0.146) — which the fp16 floor *can* produce: `dzncc` error ≤ 0.31 × the `1/(N·d)`
  weight (up to 10× at the −0.1 cut both backends apply) × `gB·(J·d)·RegScale`. Both kernels
  are otherwise the same formula (`sg = gB·(J·dA)·dz·RegScale/Nd`, face normal, same cut). Since
  fp16 rounding is deterministic the iteration-0 outlier set is reproducible (10/10 in both runs)
  and cannot be separated from a structural cause by re-running; the decisive experiment is
  also the principled fix under the CPU-reference policy: **the CUDA image and the warped image
  are now float (`ArrayRT32F`, `readSurfFloat`, warp writes float)** — +2 B/px per view on top of
  the 26 B/px a view holds, and the last approximation the CPU does not make is gone (`baryMap`
  stays fp16: weights in [0,1], 5e-4 relative). **Parity with fp32 images (20:33,
  `parity_debug_fp32`):** the bulk tightened again — |cuda|/|cpu| p10–p90 0.9965–1.0032 (was
  0.991–1.010), p1/p99 0.972/1.030, without the worst 50 rel-RMS 0.0021; `imageAB` rms 2.7e-4 →
  1.7e-4 (what is left is the texture unit's 9-bit-fraction bilinear weights vs the CPU's exact
  ones), `dzncc` 0.036 → 0.016, max 0.19. **And the same 10 vertices are still there with the
  same magnitudes to four digits** (v584 0.1448/0.0153, v197 0.0102/0.0822 …) — untouched by
  fp16→fp32 and by the border row, so structural, and not in the pair-(0,1) maps. Two facts
  narrow it: `pnorm` counts *pairs* (ThProcessPair adds 1 per pair a vertex was seen in;
  `pnorm = 9` = 9 directed pairs of 12), so an outlier is one pair's contribution differing;
  and both kernels are the same formula (`sg = gB·(J·dA)·dz·RegScale/Nd`, unit face normal,
  same −0.1 cut). Tooling added to settle it instead of guessing: the debug pair now also
  exports `sg` (the per-pixel scalar each backend hands to its face's 3 vertices; 0 where cut)
  and `face` (rasterised face id, −1 outside the mask); `refine_parity.py` reports `sg`
  rms/max and `face` identical-%; `sweep_pairs.sh` runs the parity once per directed pair with
  the RelWithDebInfo binaries (seconds per cell) into `Tiny/parity_pairs/<A>_<B>/`, and
  `parity_sg.py` walks each outlier vertex → its faces → the pixels rasterised to them in
  every pair, printing the CPU/CUDA pixel sets, Σsg and the pixels where `sg` differs. First
  use (20:44, `parity_debug_sg`, pair (0,1) only): the export works — `sg` rms 4.4e-6 / max
  1.8e-3, `face` 99.999 % identical — and every outlier's Σsg in pair (0,1) is identical, so
  the culprit is one of the other 11 pairs; 8 of the 10 outliers share faces (28, 87, 177,
  1695, 2596, 13571 …): one patch around pixel (305–309, 305–315) of image 0. The sweep that
  followed localised it to the `(·,3)` pairs and led to the warp-rule root cause below; after that
  fix a fresh 12-pair sweep is clean on every pair (see the status block at the top of this section).
- **CUDA was not deterministic (found 20:47 by comparing two runs of identical code):** the
  CPU dumps of `parity_debug_fp32` and `parity_debug_sg` are bit-identical, the CUDA ones are not —
  `pnorm` differs on 11 vertices (6 ↔ 9: a visibility flip in one A-image propagates to its 3
  pairs) and `p` by up to 6e-3, with `max|dxyz| = 0`. Cause in `kernelProjectMesh`: the depth
  `atomicMin` (CAS loop) and the `faceMap`/`baryMap` payload writes were not one atomic — two
  faces can both win the depth race in sequence and the farther one's payload can land last.
  Rewritten as two race-free passes: pass 1 does a single 64-bit `atomicMin` per covered pixel
  of the key `(depth bits << 32) | position-in-cameraFaces` (nearest face wins; on an exact tie
  the face that comes first in the camera's list — the CPU's `RasterMesh` keeps the first face
  it rasterises, `depth > z` strict, walking that same octree-order list, so the tie-break is
  the CPU's); pass 2 is the *same* face-parallel kernel (`template<bool RESOLVE>`): each thread
  redoes its arithmetic and the one holding a pixel's winning key writes depth/face/bary — one
  writer per pixel; a per-pixel `kernelResolveProjection` then clears the uncovered pixels
  (0 / NO_ID / 0) and, in Debug, ASSERTs every covered pixel got its payload (faceMap is preset
  to NO_ID so a stale id cannot satisfy it). The first attempt recomputed the winner's bary in
  a *separate per-pixel kernel* and the device ASSERT fired on many pixels: nvcc contracts
  `a*b+c` into FMAs differently in different kernels, so "the same inline function" is not
  bit-identical across kernels — `pixelBary` now uses explicit-rounding intrinsics
  (`__fmul_rn`/`__fadd_rn`/`__fsub_rn`/`__fdiv_rn`) which the compiler may not fuse, which is
  what makes the two passes agree by construction. Scratch: one 64-bit key per pixel of the
  largest view (`projKey`, +8 B/px transient); `atomicMinFloat` and `kernelCrossCheckProjection`
  are gone. Debug parity with it (21:09, `parity_debug_raster`): device ASSERT silent over 45
  iterations, `pnorm` identical on 99.99 % (cosine 1.000000, rel-RMS 0.0009; the one vertex left
  is v584, 9 vs 8 pairs), `face` 99.999 %, everything else unchanged — and the same 10 outliers,
  as expected. Determinism check with the fixed rasterizer (21:14, two RelWithDebInfo CUDA runs
  on the frozen mesh, `determinism.sh`): at iteration 0 positions, `pnorm`, masks, face maps
  and pair maps are byte-identical — the rasterizer race is gone — but `p` differs by 1.0e-7
  (3e-7 of max|p|): the float `atomicAdd` accumulation order in `kernelComputePhotometricGradient`.
  That noise is amplified by the optimisation: max|Δxyz| 1.2e-7 at iteration 1, 5.8e-5 at 10,
  **6.7e-4 at 44**, where `p` differs by 61 % of max|p| and `refined.ply` is not reproducible.
  So the refinement trajectory is chaotic at the vertex level and any CUDA F1 number carries a
  run-to-run term; the CPU multi-threaded path has the same class of nondeterminism (per-pair
  sums added under a lock in completion order). Follow-up, cheap and order-independent:
  accumulate `photoGrad` in 64-bit fixed point (integer atomics commute; 2^-32 resolution is
  finer than float's 1e-7 relative at these magnitudes) — after the outlier root cause.
- **Outlier root cause (21:30, `sweep_pairs.sh` + `parity_sg.py` on all 12 directed pairs):**
  the `sg` maps agree to ≤ 2e-3 max in every pair except those with **image 3 as B** ((0,3)
  0.0119, (1,3) 0.0082, (2,3) 0.0035), and there the difference is the *mask*: over the outlier
  patch CUDA's warp rejects pixels the CPU keeps (v584 in pair (0,3): CPU 33 pixels, CUDA 3;
  (2,3): 10 vs 0 — `face 2596/-1 mask 1/0`), exactly the grazing pixels with |dzncc| 8–15.
  Those pixels project into image 3 onto a strip of **sliver faces** (15333: bbox 1.8 × 8.7 px,
  8641, 12917 — front-facing, z ≈ 7.2, mid-image) that the CPU rasterises (7, 6 px) and CUDA
  **not at all**, so CUDA's view-3 depth map has holes there and the B-side depth check fails.
  The same shows in the portrait view 2 (8641/1398 CPU-only, 12917 CUDA-only). Cause: CUDA
  computed `b0 = 1 − b1 − b2` — not watertight: on a sliver strip a pixel centre within an ulp of
  a shared edge is rejected by *both* neighbours; the CPU's `RasterizeTriangleBary` evaluates
  each barycentric as the edge function of the pixel against the two other vertices, and the
  shared edge's function is the same value for both faces, so the pixel is inside at least one.
  Fix: `pixelBary` is now the CPU's test expression for expression (`edgeFunction` =
  `(x2−x0).cross(x1−x0)` in the CPU's operation order, `b_i = ef_i·invArea`, reject as soon as
  one is negative, cull iff area ≤ 0, `pb_i = b_i·z_j·z_k` each divided by the sum, depth as
  `ComputeDepth`), all in explicit-rounding intrinsics so the CUDA coverage is bit-compatible
  with the CPU's given the same projected vertices. **Result: no change** — the re-sweep's warp
  masks are identical pair by pair to the previous build's (0_3 still 108 CPU-only, 1_3 68,
  2_3 28, 3_0 34/17), the 10 outliers unchanged to four digits; so the formulation was not the
  mechanism (a 7-pixel face has interior centres ≥ 0.3 px from any edge — I should have
  checked that arithmetic before rebuilding). A float64 ray-cast of the mesh then showed the
  real geometry: the outlier patch (faces 9075/14576/10414, A ≈ (304–307, 300–301)) projects
  into view 3 at (274.6, 298.3) onto a **depth discontinuity** — nearest surface f15584 at
  7.96, behind it f7884/f12682 at 8.28, `pz` 8.11–8.19 in between (the patch is a sliver seen
  edge-on in view 3) — so an exact depth check *rejects* those pixels: CUDA rejects, the CPU
  accepts. Which of the two depth maps deviates from the exact surface is a matter of which
  sliver each rasterizer covers, and that is decided by the projected vertex coordinates: the
  CPU projects in **double** (`ptc = R*(X−C)`, `pti = (float)(K02 + K00·x/z)`, depth
  `(float)ptc.z`), CUDA in float — 1e-5 px is enough to flip a sliver's coverage. Next step:
  a double-precision `RasterCamera` (the CPU camera copied verbatim) and `projectVertex` on the
  device with the CPU's expressions in non-fusing double intrinsics, so the float pixel
  coordinates the edge functions see are the CPU's bit for bit. Raw (unmasked) face maps are
  now exported (`face` = −1 only where nothing was rasterised) to separate rasterisation from
  warp differences. **Those raw maps settled it before the double camera was even needed
  (21:48, `parity_debug_A3_rawface`, view 3): CPU and CUDA face maps are identical on all
  197,624 covered pixels and both match the float64 rasterisation everywhere, slivers
  included (CUDA does have 15333 at (438, 416–422)).** The masked export had shown warp
  rejections as missing faces; rasterisation was never the problem, and the double-precision
  camera was reverted (the CPU edge-function formulation stays — it is the CPU's, and costs
  nothing). With identical depth maps and identical `pz`, the only thing left was the test
  itself — and it is: `SceneRefine.cpp` defines `MESHOPT_DEPTHCONSTBIAS 0.05f`, so the CPU's
  live `IsDepthSimilar` is the **one-sided occlusion test** `depth + 0.05 < z → skip` (any
  corner at or *behind* the point is accepted; the symmetric `|depth − z| > 1 %` sits in an
  `#ifndef` that is never compiled), while CUDA implemented the dead symmetric branch. At the
  patch, the corners behind the point (8.28 vs 8.19) are accepted by the CPU and rejected by
  CUDA. Fix (CPU-reference policy): `Refine::DepthConstBias = 0.05f` in `SceneRefineCommon.h`,
  `kernelImageMeshWarp` uses `depthB > 0 && depthB + DepthConstBias >= pz` on the 2×2 block,
  the CPU reads the same constant and its dead `#ifndef` branch and the macro are gone. (Note
  for Part B: a 0.05 scene-unit constant is scale-dependent — the CPU has always behaved this
  way and it is now shared, not changed.) **Verification (Debug parity, `Tiny` L0, 21:59,
  `parity_debug_warp`) — WP2 parity reached:**

  | field | cosine | rel-RMS | offenders | before WP2 (19:17) |
  |---|---|---|---|---|
  | `g` combined | **1.000000** | **0.0008** | 0.04 % | 0.8607 / 0.906 |
  | `p` photo | 1.000000 | 0.0009 | 0.30 % | 0.8850 / 1.022 |
  | `s1`, `s2` | 1.000000 | 2e-5 | 0 | −1.0 (sign) / 1.0 |
  | `pnorm` | identical on **100 %** | | 0 visibility disagreements | 99.88 % |
  | pair (0,1) mask IoU | **1.0000** | `face` 99.999 % identical | `sg` rms 4.4e-6, max 1.8e-3 | 0.9978 |

  All four calibration targets pass (cosine ≥ 0.999, rel-RMS ≤ 0.02, pnorm ≥ 99 %, IoU ≥
  0.995). **Confirmed a second time in RelWithDebInfo with the pinned binaries** (step 0 of the
  re-baseline chain, 22:45, `bench/bin_refine_wp1`, output kept as `Tiny/parity_wp1_pin`):
  `combined_gradient_cosine` 1.000000 ≥ 0.999 PASS, `combined_gradient_rel_rms` 0.000826 ≤ 0.02
  PASS, `pnorm_identical_pct` 1.000000 ≥ 0.99 PASS, `mask_iou` 1.0000 ≥ 0.995 PASS, with pair
  (0,1) `face` 99.999 % identical and 0 visibility disagreements over 8,120 vertices — so the
  verdict is not an artefact of the Debug build, and the pin that the baseline table rests on is
  the binary that produced it. Per vertex |cuda|/|cpu| p1–p99 0.980–1.019; the worst vertex now differs by 2 %
  (v8102: 0.0840 vs 0.0825), i.e. the texture unit's bilinear weights and the float `atomicAdd`
  order — the two approximations the CPU does not make, both documented above. The
  disagreements WP2 found and removed, in order: forward-difference image gradient (CUDA →
  shared 3×5 stencil), `smoothGrad1` sign convention, half-float gradient/image storage,
  rasterizer bbox off-by-one at the far edge, rasterizer payload race (nondeterminism), and the
  warp depth test (dead symmetric branch vs the CPU's live one-sided test) — every one
  CUDA-side, as the CPU-reference policy predicted. RelWithDebInfo determinism check,
  12-pair sweep and the pin/re-baseline follow.
