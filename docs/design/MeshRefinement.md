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

**Today's state (2026-08-30).** §1 and §2 are complete — they describe the shipped `develop`
algorithm and the harness that now exists (work packages H1/H2). §3-§7 are stubs: no
`RefineMesh` benchmark row exists yet anywhere on this branch (see §8).

---

## 1. Pipeline as shipped

`RefineMesh` implements the Vu et al. (PAMI 2012) variational mesh refinement: alternate a
multi-scale subdivision pass with a gradient-descent optimization that pulls the surface toward
photo-consistency while a Laplacian regularizer keeps it smooth. Two independent implementations
exist side by side and are expected to diverge (`libs/MVS/AGENTS.md`'s guidance on
platform-specialized code applies): the CPU path in `libs/MVS/SceneRefine.cpp`
(`MeshRefine`/`Scene::RefineMesh`) and the CUDA path split across `libs/MVS/SceneRefineCUDA.cpp`
(`MeshRefineCUDA`/`Scene::RefineMeshCUDA`) and `libs/MVS/SceneRefineCUDA.cu` (device kernels). There
is no `SceneRefine.h`; `MeshRefine` is a translation-unit-local class defined inside the `.cpp`.
`apps/RefineMesh/RefineMesh.cpp` is the CLI driver; every default cited below is that file's
`boost::program_options` default (lines 92-136), unless marked as a compile-time constant.

### 1.1 Entry point, CLI defaults, and the `-m`/output-naming trap

`main()` (`RefineMesh.cpp:206-265`) loads the scene, loads (or defaults) the mesh, tries CUDA first
if `--gpu-device` selects a device, and silently falls back to the CPU path if the CUDA call
returns `false`:

```
#ifdef _USE_CUDA
if (SEACAVE::CUDA::desiredDeviceIDs.empty() || !scene.RefineMeshCUDA(...))
#endif
if (!scene.RefineMesh(...))
    return EXIT_FAILURE;
```
(`RefineMesh.cpp:231-252`.) When `--gpu-device` is unset the default is the empty string
(`RefineMesh.cpp:111`), which short-circuits the `||` and skips the CUDA call entirely — the app
runs on CPU unless a device is explicitly requested (`-1` = best GPU, `-2`/`cpu`/empty = CPU,
`>=0` = comma-separated device IDs). If `RefineMeshCUDA` returns `false` for any reason (no device,
OOM, kernel launch failure) execution falls through to the full CPU path **in the same process and
log**, with no log line announcing the fallback — the only way to tell from the log is that CPU
iteration lines carry an `f:` cost field and CUDA lines never do (§2.1, `refine_log.py`).

`--mesh-file/-m` defaults to `<input-file-without-extension>.ply` **only when the archive type is
MVS** (`RefineMesh.cpp:185-186`). With `-i scene_dense.mvs` that default resolves to
`scene_dense.ply` — the dense **point cloud's** ply, not a mesh — which produces `mesh.IsEmpty()`
and `"error: empty initial mesh"` (`RefineMesh.cpp:226-228`). `-m` must always be passed explicitly
when refining a coarse mesh built from the same stem as the input scene; this is why
`bench/run_refine.py` never omits it (§2.3).

Output naming: the refined mesh is written to `<out-stem><export-type>` unconditionally
(`RefineMesh.cpp:256-257`, default export type `.ply`); the `.mvs` sidecar is written **only** when
the archive type isn't `ARCHIVE_MVS` or the input wasn't loaded as `Scene::SCENE_INTERFACE`
(`RefineMesh.cpp:262-263`) — for the harness's normal MVS-in/MVS-out invocation, the only mesh
artifact on disk after a run is the `.ply`.

### 1.2 Multi-scale subdivision loop

`Scene::RefineMesh`/`RefineMeshCUDA` (`SceneRefine.cpp:1290-1445`,
`SceneRefineCUDA.cpp:849-930`) run `nScales` passes coarse-to-fine (`--scales`, default 2;
`--scale-step`, default 0.5, `RefineMesh.cpp:128-129`). At loop index `nScale` (0-based, 0 = first
= coarsest):

```
scale = fScaleStep ^ (nScales - nScale - 1)   // image downsample factor
step  = 2 ^ (nScales - nScale)                // used only for the blur sigma below
sigma = 0.12 * step + 0.2                     // pre-blur before resizing/gradient
```
(`SceneRefine.cpp:1310-1313`, identical in `SceneRefineCUDA.cpp:868-871`.) With the shipped
defaults (`nScales=2`, `fScaleStep=0.5`): scale 0 runs at half image resolution with `sigma=0.68`,
scale 1 (the last, finest) at full resolution with `sigma=0.12*2+0.2=0.44` px — a noise-robust
pre-blur, not a negligible one, ahead of the derivative stencil (§1.3).

Each scale re-inits images at that scale (`MeshRefine::InitImages`, `SceneRefine.cpp:392-401`),
re-lists incident faces (`ListVertexFacesPre`), then calls `SubdivideMesh` (`:486-578`, CUDA
near-verbatim at `SceneRefineCUDA.cpp:448-540` — any change to this logic must be made in both
places or hoisted). `SubdivideMesh` first optionally decimates (`--decimate`, default 0 = auto:
projects the mesh into every camera, measures the median face area across image pairs, and
decimates only if that median exceeds `--max-face-area` by more than 6x, `:526-538`), runs
`Mesh::Clean` with `simplifyTarget`/`edgeLength=-2.25` (a negative, i.e. relative, target edge
length so the [0.5x, 4x] edge-length remesh rides the same `Clean` pass, `remeshIterations=10`,
`:492-500`), then subdivides any face whose projected area in the tightest camera pair exceeds
`--max-face-area` (default 16 px², `Mesh::Subdivide`, 1-to-4 split, `:559`). The subdivision-count
log line (`"Mesh subdivided: %u/%u -> %u/%u vertices/faces"`, `:572`, identical text from the CUDA
path) is what `refine_log.py`'s `RE_SCALE` regex keys per-scale boundaries on (§2.1).

### 1.3 Photo-consistency energy

For every ordered image pair `(A, B)` in the pair list (view-graph neighbors filtered by
`Scene::FilterNeighborViews`, `SceneRefine.cpp:1053-1073`), `ThProcessPair` (`:1126-1204`, CUDA
`ProcessPair`, `SceneRefineCUDA.cpp:645-664`) does, per pixel of A:

1. **Warp.** Project A's depth at `(i,j)` into B; keep the sample only if `IsDepthSimilar` accepts
   (§1.5/§1.6 — visibility test). Invalid (occluded/out-of-view) pixels are **not zeroed**: CPU
   seeds the warped buffer `imageAB` with a straight copy of image A (`imageA.copyTo(imageAB)`,
   `:1147`) before overwriting only the pixels that pass the warp (`ImageMeshWarp`, `:759-783`), so
   an invalid pixel's warped value equals A's own value — biasing the local window statistics
   toward `ZNCC=1` right at occlusion boundaries, since the box-sum window (below) is not masked
   against the integral image's contents, only against which output pixels get written.
2. **Local statistics, 7x7 window (CPU `HalfSize=3`, `SceneRefine.cpp:239`; CUDA `HalfSize=2` =
   5x5, `SceneRefineCUDA.cpp:169`).** `ComputeLocalVariance` (`:786-828`) uses `cv::integral` box
   sums over the **whole** image/mean/variance arrays (not masked at the sum level, only the
   per-pixel write is gated by `mask(r,c)`); `ComputeLocalZNCC` (`:831-908`) forms
   `ZNCC = (cov - meanA*meanB) / sqrt(varA*varB)` and its analytic pointwise derivative — the
   active branch (`#if 1`, `:873-877`):
   ```
   dZNCC = A(r,c)*invSqrtVAVB - B(r,c)*ZNCC/varB + meanB*ZNCC/varB - meanA*invSqrtVAVB
   ```
   folded with a reliability weight `ReliabilityFactor = min(varA,varB) / (min(varA,varB)+0.0015)`
   into `imageDZNCC(r,c) = -ReliabilityFactor * dZNCC` and the accumulated score
   `+= ReliabilityFactor*(1-ZNCC)` (`:900-903`) — there is no separate confidence output; the
   reliability weight is baked into `imageDZNCC` before it ever reaches the gradient step. A dead
   `#else` branch (`:878-897`) computes a *windowed* average of the same quantity instead of the
   pointwise value at `(r,c)` — this is the form the CUDA kernel actually implements (below).
   `imageVar` is clamped to a `1e-4` floor (`:824`) but `imageInvSqrtVAVB = 1/sqrt(varA*varB)` has
   **no zero guard** (`:862`) — a flat patch (`varA` or `varB` at the floor from both sides) can
   still produce `inf`/`NaN` here; nothing downstream currently checks for it on CPU.
3. **Per-pixel photometric gradient.** `ComputePhotometricGradient` (`:911-965`) computes, per
   valid pixel, the face normal `N`, the camera-A ray direction `dA`, and `Nd = N.dA`; skips the
   pixel if `Nd > -0.1` (a one-sided grazing/back-face gate, `:938-940`, unconditionally compiled).
   It back-projects to the 3D point, projects into B with a Jacobian `xJac` (`ProjectVertex`,
   `:672-709`), bilinearly samples B's precomputed image gradient `gB` there, and forms
   ```
   sg = (gB . (xJac * dA)) * dZNCC * RegularizationScale / Nd            (SceneRefine.cpp:953)
   ```
   distributed to the face's three vertices weighted by barycentric coordinates and the face
   normal (`:957-962`); `photoGradNorm[vert]` is incremented **once per pixel that touched it in
   this pair**, but that per-pixel count is only ever tested for `>0` back in `ThProcessPair`
   (`:1186-1201`) — the value actually accumulated into the global `photoGradNorm[v]` is `+=1` per
   pair-direction that saw `v` at all, not per pixel and not weighted by how many pixels or how
   reliable they were. `RegularizationScale = avgDepthA*avgDepthB / (fA*fB)` (`:1180`, camera focal
   lengths; identical formula on CUDA, `SceneRefineCUDA.cpp:662`) is the scene-scale-dependent term
   that converts an image-space photometric gradient into a 3D-consistent one (§1.7).
4. **Image gradient stencil.** `gB` above comes from a precomputed per-view gradient image
   (`View::imageGrad`), built once per scale in `ThInitImage` (`:1074-1115`) with the noise-robust
   separable `[1,2,1]^T (x) [-1,-2,0,2,1]/32` kernel (`CreateDerivativeKernel3x5`, `Types.inl:3133-3140`,
   applied via `cv::filter2D` at `SceneRefine.cpp:1106-1108`; a plain Sobel-3 and a 5x7 variant of
   the same family exist in the source but are `#if 0`-disabled, `:1102-1113`). `gB` is sampled at
   the sub-pixel projection `xB` with a bilinear `Sampler::Linear` (`:950`), which in
   `TImage::sample` (`Types.inl:2337-2339` -> `Sampler.inl:236-241`) treats pixel `(0,0)`'s value as
   located at grid coordinate `(0,0)` (`floor(pt)` + fractional weight, no half-texel offset).
5. **`--reduce-memory` (default 1, `RefineMesh.cpp:135`).** Counter-intuitively, the *default*
   value **disables** the per-view precompute: `ThInitImage` only fills `view.imageMean/imageVar`
   when `!nReduceMemory` (`:1095-1098`), so by default those buffers are never populated; instead
   `ThProcessPair` recomputes A's local mean/variance from scratch for every pair, but *masked by
   that pair's actual occlusion mask* (`:1151-1156`). The `nReduceMemory==0` (opt-out) path
   precomputes once per scale over an all-valid mask (`BitMatrix(img.size(), 0xFF)`, `:1097`) and
   reuses it for every pair regardless of that pair's own occlusion pattern — trading a
   per-pair-correct statistic for a cached, pair-agnostic one. As shipped, the flag is a genuine
   memory/compute trade-off, not a no-op (a later work package flattens this, see §5).

### 1.4 Regularization term

`ComputeSmoothnessGradient1` (`SceneRefine.cpp:969-993`) computes the discrete umbrella-operator
Laplacian `L(v) = mean(1-ring neighbors) - v`, **zeroed at boundary vertices**
(`if (vertexBoundary[idxV]) continue;`, `:979-981`, unconditionally compiled). `ComputeSmoothnessGradient2`
(`:996-1023`) forms the "level 2" operator from Hernandez (2004, p.105) — a valence-normalized
combination of `L` over the same ring — **also zeroed at boundary vertices** (`:1004-1007`). Crucially,
the photometric term (§1.3 step 3) has **no boundary check at all**: a boundary vertex receives the
full photometric pull with zero smoothing counter-pull (plan-tracked issue; not yet fixed on this
branch).

The final per-vertex gradient (`ScoreMesh`, `:650-666`) combines them as:
```
ratioRigidityElasticity >= 1:   photoGrad[v]/photoGradNorm[v] + smoothGrad2[v]*weightRegularity
ratioRigidityElasticity <  1:   photoGrad[v]/photoGradNorm[v] + smoothGrad2[v]*elasticity - smoothGrad1[v]*rigidity
                                 rigidity   = (1-ratio)*weightRegularity
                                 elasticity =    ratio *weightRegularity
```
`--regularity-weight` defaults to 0.2, `--rigidity-elasticity-ratio` to 0.9 (`RefineMesh.cpp:131-132`);
`ratioRigidityElasticity` is itself forced to `1.f` for the last 30% of each scale's iterations
(`iter <= iters*7/10`, `SceneRefine.cpp:1381,1389`, same on CUDA `SceneRefineCUDA.cpp:896,903`) —
so the rigidity (level-1) term only contributes early in each scale's schedule.

### 1.5 Visibility test (occlusion)

CPU `IsDepthSimilar` and CUDA `kernelImageMeshWarp` both round the projected point in B to its
nearest depth-map texel and accept it iff `depth > 0 && depth*1.0002 >= z`. This is a **one-sided**
occlusion test: it rejects only when B's own measured depth is significantly *closer* to the camera
than the transformed point. A farther B surface is accepted because it does not occlude A's point.
The relative tolerance is invariant under a uniform scene rescale because both camera-space depths
are multiplied by the same factor (§6.0b).

### 1.6 CPU/CUDA divergences (beyond the visibility test above)

| Aspect | CPU | CUDA | Citation |
|---|---|---|---|
| Window size | `HalfSize=3`, 7x7 (49 px) | `HalfSize=2`, 5x5 (25 px) | `SceneRefine.cpp:239`; `SceneRefineCUDA.cpp:169` |
| dZNCC form | pointwise analytic (`#if 1` active branch) | windowed average of the same quantity over the 5x5 neighborhood (the CPU's dead `#else`) | `SceneRefine.cpp:873-877` vs `.cu:337-386` (loop body `:359-377`) |
| Image gradient | precomputed 3x5 separable stencil image, bilinearly sampled | no gradient image; `tex2D(x+1,y)-tex2D(x,y)` forward differences on fp16 texels, sampled at the *unshifted* projected coordinate | `SceneRefine.cpp:1106-1108` vs `.cu:451-453` |
| Sample-point convention | `floor(pt)+frac`, texel center at integer coords | CUDA linear-filtered `tex2D` treats a non-normalized coordinate's texel center at `+0.5` (hardware convention) — both the gradient forward-diff fetches (`.cu:451-453`) and the color fetch in the warp kernel (`.cu:213`) pass the projected coordinate unshifted, a systematic ~0.5-px bias not present on CPU | `Sampler.inl:236-241` vs `.cu:213,451-453` |
| Border margin | effective B-side margin 3 px (`isInsideWithBorder<int,3>`); A-side loops `[HalfSize, size-HalfSize)` | hard-coded 10-px margin on the warp target (`borderMin=10.f`, well past any window/derivative need) | `SceneRefine.cpp:719` vs `.cu:195-199` |
| `photoGradNorm` margin | the `[HalfSize,size-HalfSize)` loop bounds implicitly exclude border pixels from ever setting `photoGradNorm` | `kernelComputePhotometricGradient` has **no** `HalfSize` margin check (only `mask[pixIdx]!=1` return) — border pixels with `dzncc=0` can still increment a vertex's count, diluting `photoGradNorm` toward zero contribution | `SceneRefine.cpp:928-930` vs `.cu:406-411` |
| Bi-Laplacian valence | guards `numVert>0` before dividing (`:1016`) | `totalWeight += invN / vertSizes[ni]` with **no zero guard** (`.cu:516`); boundary vertices are uploaded with `vertSizes=0` (`SceneRefineCUDA.cpp:355-358`), so any interior vertex adjacent to a boundary vertex divides by zero -> `inf` -> its own `smoothGrad2` collapses to 0 | `SceneRefine.cpp:1011-1018` vs `.cu:506-520` |
| Camera-B size for the warp/gradient kernels | not applicable (per-view sizes used directly) | `MakeCUDACamera(cameraB, size)` is called with **image A's** size, not B's, in both `ImageMeshWarp` and `ComputePhotometricGradient` | `SceneRefineCUDA.cpp:678,769` |
| Rasterization | `PerspectiveCorrectBarycentricCoordinates` (perspective-correct); a face is drawn only if all 3 vertices project inside the image with a 3-px border | screen-space barycentrics (no perspective correction), face bbox clamped to a 5-px border, drawn if any pixel's barycentrics are non-negative regardless of whether all 3 vertices are in-frame | `SceneRefine.cpp:120-121`, `Mesh.h:357-360` vs `.cu:83-144` |
| Cost / scale of the schedule | `iters=75, gstep=0.4` when `--gradient-step<=1`; emits a per-iteration cost `f:` field and a removed-vertex count `v:` | `iters=25, gstep=0.05`; **no cost is ever computed** (`MeshRefineCUDA::ScoreMesh` returns `void`); no planar-vertex removal; float (not double) gradients; a defensive `if (!ISFINITE(grad)) continue;` the CPU loop does not have | `SceneRefine.cpp:1374-1375,1394,1411,1430` vs `SceneRefineCUDA.cpp:889-890,905,911-916` |
| Dead upload | — | `vertexVertices.Reset(scene.mesh.vertexVertices)` uploads a host `cList` structure nothing downstream ever reads (only the flattened `vertexVerticesCont/Sizes/Pointers` triple is used) | `SceneRefineCUDA.cpp:347` |
| Image masks (`Image::maskName`) | ignored — no reference to a per-image mask anywhere in `SceneRefine.cpp` | ignored, same | `Image.h:63-76` defines the feature; unused by either backend |
| Winding cull (found during the campaign, §8) | `RasterizeTriangleBary` keeps `EdgeFunction = (p2−p0)×(p1−p0) > 0` = front faces | kept `det = (p1−p0)×(p2−p0) > 0` = the opposite sign → back faces only, ~0.1 % of the faces | `Types.inl:2612-2614`, `Util.inl:699` vs `.cu:108-112` |

**Status after Part A WP0/WP1 (2026-08-30, see §8):** the table above describes develop. On the
branch, the window size (both 7×7), the dZNCC form (both pointwise), the sample-point convention
(`+0.5` on every `tex2D`), the border margin (one 2×2-block rule on both sides), the `photoGradNorm`
margin, the bi-Laplacian valences, the camera-B size, the rasterization (perspective-correct
barycentrics on CUDA, one partial-face rule on both) and the winding cull are unified, and the dead
upload is gone. Still divergent and scheduled: the image-derivative stencil (WP7), the visibility
test (§1.5, WP3), the schedule/cost (Part B), image masks (WP5), the warp fill (WP4).

### 1.7 Optimization schedule

Both backends run the same fixed-iteration gradient-descent loop, once per scale, with a
per-iteration exponential step decay:

```
iters = FLOOR2INT(fGradientStep)  if fGradientStep > 1   (CPU 45, CUDA formula identical)
gstep = (fGradientStep - iters) * 10
iters = max(iters / (nScale+1), 8)                        # nScale is 0-based, coarsest first
for iter in [0, iters):
    vertex -= gradient * gstep
    gstep *= 0.98                                          # SceneRefine.cpp:1431; SceneRefineCUDA.cpp:917
```
With the shipped CLI default `--gradient-step 45.05` (`RefineMesh.cpp:133`): `iters=45` at the
*coarsest* scale (`nScale=0`, divisor 1) and `iters=22` at the *finest* scale (`nScale=1`, divisor
2 — the coarse scale runs more iterations than the fine one, as coded); `gstep0 = (45.05-45)*10 =
0.5`, decaying 2% per iteration. If `--gradient-step <= 1` the loop instead uses `iters=75, gstep=0.4`
on CPU (`:1374-1375`) or `iters=25, gstep=0.05` on CUDA (`SceneRefineCUDA.cpp:889-890`) — divergent
un-normalized defaults that only the shipped `45.05` value papers over. `--gradient-step 0` selects
a third path on CPU only, described below; CUDA has no equivalent and always runs the fixed loop.

On the **last** iteration of every scale, `nAlternatePair` is forced to `0` ("both directions")
regardless of the configured `--alternate-pair`, on both backends (`SceneRefine.cpp:1388`,
`SceneRefineCUDA.cpp:902`) — a final full-coverage evaluation baked into the schedule.

**Planar-vertex removal (CPU only).** If `--planar-vertex-ratio` (`fThPlanarVertex`) is nonzero
(default 0 = disabled, `RefineMesh.cpp:134`), every third iteration starting at 40% through the
scale's budget (`iterStart=iters*4/10`, `:1382,1390`) the loop additionally tracks a per-vertex
minimum-seen depth (`refine.vertexDepth`) and removes vertices whose gradient magnitude and
smoothing residual are both below `depth * fThPlanarVertex` (`:1393-1420`), via
`Mesh::RemoveVerticesAndFill` — a halfmesh round trip that does not preserve vertex order or index
stability. CUDA implements none of this.

**Ceres path (CPU only, `--gradient-step 0`, gated by `MESHOPT_CERES`, itself gated on
`_USE_CERES`).** `ceres::MeshProblem` (`SceneRefine.cpp:1237-1283`) wraps `ScoreMesh` as a
`FirstOrderFunction` and solves with `GradientProblemSolver` (LBFGS, default rank 20, WOLFE line
search; explicit options `function_tolerance=1e-3, gradient_tolerance=1e-7,
max_num_line_search_step_size_iterations=10`, `:1350-1352`); `ratioRigidityElasticity` is forced to
`1.f` for the whole solve (`:1334`). The cost `ScoreMesh` returns
(`(nAlternatePair?0.2:0.1)*scorePhoto + 0.01*scoreSmooth`, `:666`, where `scoreSmooth` is the
level-1 umbrella-magnitude sum) and the gradient it fills
(`photoGrad/photoGradNorm + weightRegularity(=0.2 by default)*smoothGrad2`, `:653-654`) are **not
the gradient of that cost** — different constants (0.1/0.2 vs 0.2), a different smoothing
functional (L1 umbrella-magnitude sum in the cost vs the bi-Laplacian in the gradient), and an
implicit `1/photoGradNorm[v]` per-vertex reweighting with no corresponding term in the cost. A
solver whose line search relies on cost/gradient consistency (Wolfe conditions) is therefore
operating on an inconsistent pair; any `TerminationType` other than `NO_CONVERGENCE`/`CONVERGENCE`/
`USER_SUCCESS` aborts the **entire refinement** for that scene (`:1358-1367`, `return false`).

### 1.8 Scale-dependent constants

Two quantities carry scene absolute-scale dependence baked in, neither normalized against the
scene's own coordinate units:

- `RegularizationScale = avgDepthA*avgDepthB / (fA*fB)` — both backends, converts the image-space
  photometric gradient into a 3D one; depends on `Image::avgDepth`, which is persisted in the
  `.mvs` file and is **not** touched by `Scene::Transform`/`TransformScene.exe` (a Python-side
  rescale tool must recompute it by hand; the planned scale-invariance test, H7, rescales it
  explicitly for this reason).
- `gstep` (§1.7) is subtracted directly from vertex positions in scene units, so its effective size
  in pixels (what actually matters for convergence) changes with scene scale even though the
  iteration/step schedule itself is scale-blind.

The relative occlusion tolerance (`depth*1.0002 >= z`, §1.5) is scale-invariant by construction.

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

## 7. Durable constraints and limitations

1. **Refinement cannot repair mesh density.** The `--decimate 0` (auto) step reduces the coarse mesh
   to the working face budget *before* any photo-consistency iteration — 5-10x on the T&T scenes at
   L1, 15x on fountain — and no optimizer recovers what that removes. Ignatius keeps `d_in` ≈ −0.087
   even with the accepted arm (§5). A decimation-policy candidate is a separate, still-open item
   (§8); judging an optimizer by `d_in` rather than `d_base` measures mostly this.
2. **Visibility now has no scene-unit tolerance.** Both backends use the nearest-tap relative test
  `depth*1.0002 >= z` (§1.5), which is invariant under uniform scene scaling. The H7 scale test
  can still expose other scale-dependent terms (§1.8), but no longer fails because of visibility.
3. **Neither backend is bit-reproducible run to run.** CUDA accumulates `photoGrad` with float
   `atomicAdd` (order varies); the CPU sums per-pair contributions under a lock in completion order.
   The trajectory is chaotic at the vertex level — 1e-7 at iteration 0 grows to 6.7e-4 by iteration
   44 — so every F1 carries a run-to-run term, quantified as the §3 noise floor (0.0001-0.0009).
   Fixed-point accumulation would remove the CUDA half and is queued, not done.
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
4. **Decimation policy** — the largest single effect in the baseline table and untouched by any
   optimizer (§7.1): `--decimate 0` (auto) removes 5–15x of the coarse mesh before the first
   photometric iteration, and Ignatius still carries `d_in` ≈ −0.087 with the accepted arm.
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
