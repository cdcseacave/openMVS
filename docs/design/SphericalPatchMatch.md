# Spherical PatchMatch

## 1. Purpose and scope

`DensifyPointCloud` estimates the depth-maps of equirectangular (360x180 degree) images natively,
without rendering them to cube faces. The reference camera model is a template parameter of the CPU
PatchMatch (`DepthEstimatorT<Pinhole>`, `DepthEstimatorT<Spherical>`, `libs/MVS/DepthMap.h`), a kernel
template argument of the CUDA PatchMatch (`libs/MVS/PatchMatchCUDA.cu`) and a function constant of the
Metal PatchMatch (`libs/MVS/PatchMatchMetal.metal`). Each resolves the model of every neighbor view once
per (pixel, hypothesis, view), so pinhole, Brown and spherical views can be mixed. The models are in
`CameraModels.md`; Brown images are undistorted on load.

Scope: depth-maps (CPU with `--gpu-device -2`, CUDA, Metal), the photometric confidence, and the fusion of
the depth-maps into the dense cloud (CPU). Not supported for a scene with spherical images, with a clear
error: SGM (`--fusion-mode` < 0) and the neighbor views selected from a mesh. The tower mode
(`Scene::InitTowerScene`) sees its points through the image cameras and runs on spherical images as on pinhole ones.

The confidence recalibration runs on spherical references and neighbours as on pinhole ones, fused
into the last geometric-consistency iteration on CUDA and Metal (`DepthMapConfidence.md`, camera
models).

The stages past the dense cloud project through the pinhole camera only and call
`Scene::CheckPinholeOnly(stage)`, an error that names the stage and `--spherical-export cubemap`:
`Scene::ReconstructMesh`, `RefineMesh`, `RefineMeshCUDA`, `TextureMesh`, `ComputeVertexColors`,
`ExportMeshToDepthMaps`, and the apps before `RemoveUnseenMeshFaces`. `MvgMvsPipeline.py` passes
`--spherical-export cubemap` to `CreateStructure` when the run includes a mesh step;
`MvsScalablePipeline.py` takes a scene as it is. `Scene::Split` back-projects depth-maps through the image cameras.

## 2. Algorithm as implemented

### Depth and geometry

A depth is the distance λ along the model's ray, `X_cam = λ·Ray(x)`: λ = z for a pinhole (the pinhole
estimator is unchanged, depth-maps byte-identical), the range for a spherical camera. The depth geometry
(`InterpolatePlaneDepth`, `NormalFromDepthGradient`, `PlaneRayHomography`, `PlanePatchWarp`, depth
similarity) is written once over `Ray`/`RayFrame`/`PixelHomography` in `libs/Common/DepthGeometry.h`.

### Patch warp per target view (`DepthEstimatorT::ScorePixelImage`)

Every warp hands the target texels one by one, in the order of the reference texels, to a
`PatchSums` (the NCC terms, as the GPU kernels accumulate them); `ScorePatch` turns the sums into
the cost. The samplers copy the sums in and out, and the pinhole path has its own: the sums stay in
registers through the texel loop whether the sampler is inlined or not (a texel buffer and an
out-of-line sampler cost the pinhole estimator 15 % on clang/arm64).

- Pinhole reference and target: the cached plane homography (`PinholePatchWarp`, `DepthMap.cpp`).
- Otherwise, Method B (tangent warp): `M = Trg.PixelHomography(H·r0)·H·Ref.RayFrame(x0)`, with
  `H·F = Rrel·F + t·(nᵀF)/(n·X)` and `Rrel·F` cached per pixel and view. `M` feeds the same texel loop as the
  pinhole homography. A pinhole target must see the patch center in front of it. For a spherical target the
  four patch corners decide: a corner behind the tangent plane rejects the view; a patch off the seam is
  sampled by the plain loop, a patch across it by the wrapping one (`SampleTangentPatch`).
- Method A (exact warp), where the target sees the patch center above
  `DepthEstimatorBase::fSphericalWarpMaxElevation`: each texel ray is cut with the plane and projected by
  the target model. A texel whose intersection is behind the reference camera scores the view `thRobust`,
  as an out-of-image texel does. The texel rays are structure-of-arrays padded to whole SIMD packets;
  intersections and projections run on Eigen packet functions, lane by lane identical to the scalar code.
- B's error grows as 1/cosφ of the target elevation of the patch center. The B/A choice is one branch per
  (pixel, hypothesis, view), never per texel.
- Spherical projections use `Spherical::ProjectFast` (`FastAtan2`, `FastSqrt`: `CameraModels.md`).
  Method B computes ρ = √(q_x²+q_z²) once for the center projection and `PixelHomography(q, x, ρ)`.
- The 3x3 products of the warps and of the geometric-consistency transforms are written out on the Eigen
  storage (`MulMat3`): MSVC does not inline the fixed-size Eigen products in these builds.

### Sampling and the seam

- Spherical target images wrap their columns (`SampleTexel<true>`) and keep the 1-pixel border on the rows;
  target depth-maps of the geometric consistency wrap the same way (`SampleDepth<true>`). The estimator pads no image (the transient normal
  refinement of `Scene::EstimatePointCloudNormals` pads its copies, see the consumers).
- A spherical reference is estimated across the seam (only the 4 rows next to each pole are skipped); the
  reference patch and the propagation neighbors wrap their columns (`Col0`).
- Pixel order, pinhole and spherical alike: the estimators pull pixels in zigzag order and work on a copy of
  the pixel's estimate. A reader briefly locks a pending neighbor to copy its previous estimate.
  The writer acquires the same lock to publish depth, normal and score together. Completed pixels are
  immutable until the pass ends and can be read without locking. Only at the
  corners of the zigzag stripes, where the anti-diagonals are shorter than the number of estimators, does a
  pixel wait for a lower neighbor still in flight fewer than `DepthEstimatorBase::nZigzagMinStride` (64)
  pixels before it. Elsewhere a lower neighbor is a whole anti-diagonal back, or its estimator is preempted
  and the pixel reads the previous pass's estimate rather than stall. The first and last column of a
  spherical reference are neighbors like any other two; nobody waits for the one across the seam.

### Geometric consistency

`GeometricConsistency<TRG>`: the reference point is moved into the frame of the target depth-map and
projected, the target depth sampled (similar depths only), the target point at that depth
(`X1·depth/Depth(X1)`) projected back. The pixel distance to x0, columns wrapped for a spherical
reference, is mapped to [0,4]. The pinhole pair keeps its cached form.

### Normals

Random and perturbed normals are drawn as directions (`Dir2Normal`) in a frame whose z axis is the pixel
ray: the camera frame for pinhole, `normalFrame0` = (e_lon, e_down, ray) for spherical. `CorrectNormal` is
ray-based. `EstimateNormalMap` of a spherical map fits the depth gradient with wrapped columns.

### Reference trigonometry

A spherical reference tabulates, per estimator, the sine and cosine of the longitude of every column and
the elevation of every row (`lonSinCos0`, `elSinCos0`). The ray frame at x0, the normal frame, the texel
rays of Method A and the propagation rays are built from them: no trigonometry per pixel.

### Initialization and view selection

- Sparse init of a spherical reference: every sparse point it sees is splatted as a 5x5 square of its range
  (columns wrapping), other pixels start random; dMin/dMax are those ranges x0.9/x1.1. The Delaunay
  triangulation of the sparse points stays pinhole only.
- `Scene::SelectNeighborViews` and the ROI camera-to-scene distance use the range as depth of a spherical
  image (`Image::PointDepth`). The covered area of a spherical reference bins the pixel of each shared
  point, its column wrapped (the projection spans [-0.5, size-0.5)).
- `Scene::ROIPointWeights`: a panorama contributes every observation at its range, with the image-center
  weight of a central pixel (it has no image center). `PointCloud::SaveWithScale` takes the footprint at
  the range.
- `Scene::EstimateGravityDirection`: a panorama supports only gravity candidates along its image-down axis.
- Every size of a spherical image (`--max-resolution`, resolution levels, neighbor scale, pyramid) comes from
  `Image::ComputeResize`: even width, height from the same uniform factor, so a 2:1 panorama stays 2:1 and
  its red/black propagation gets the even width it needs. K is scaled per axis, so a spherical view keeps
  fx = W/2π, fy = H/π, c = (size−1)/2; the estimator ASSERTs it.
- `Scene::EstimateNeighborViewsPointCloud` (a scene without sparse points): a pair with a panorama gets a
  grid over the whole first image, at depths log-uniform in [0.1, 10] x the depth where directions normal to
  the baseline meet at `fOptimAngle`, kept where the second view sees them.

### Per-pixel view gate

The per-view triangulation angle of `Scene::FilterNeighborViews` (averaged over shared sparse points) says
little about a panorama, which sees a neighbor along its baseline in some directions and across it in
others. When a spherical camera is involved (`bModelWarp`), `ScorePixel` gates every (hypothesis, view) at
the hypothesis depth:
1. X = depth·Ray(x0); a view is gated where the angle at X between the reference center and the view center
   (`viewCenters0`) is below `OPTDENSE::fMinAngle` (3°): X·(X−C) > cos(fMinAngle)·|X|·|X−C|;
2. if that gates every view, none is gated;
3. a gated view scores `thRobust`, like a view that does not see the patch, and drops out of the min-mean
   aggregation; the others score their NCC.

Step 2 keeps the ranking of hypotheses fair next to an epipole: there the true depth gates every view while
a closer depth sees some view at a larger angle. Pinhole-only estimators do not gate. The CUDA and Metal
kernels of a scene with a spherical view apply the same rule.

### Depth-map files

The `.dmap` header flag `SPHERICAL` (`HeaderDepthDataRaw`, `libs/IO/Interface.h`, `MvsUtils.py`) is set by
`DepthData::Save` from the reference camera model; `DepthData::Load` and the depth-map loads of `InitViews`
reject a file whose flag disagrees with the view's model.

### CUDA estimator

- The estimation kernels (`InitializeScore`, `BlackPixelProcess`, `RedPixelProcess`) are templates on `<GEOM, MODEL>`:
  `MODEL_PINHOLE` (every view pinhole), `MODEL_PINHOLE_REF` (pinhole reference, spherical views),
  `MODEL_SPHERICAL_REF`. `RunCUDA` picks it from `Params::sphericalViews`. Every MODEL runs the same code;
  the spherical branches and the view gate are compiled out of `MODEL_PINHOLE`.
- `__constant__ g_cameras` holds each view's `CUDA::Camera` (`CUDA/Camera.h`): the intrinsics in the fields
  of the `CameraModels.h` models, the pose and the image size; the model is not stored, the kernel builds it
  with `Camera::Model<PinholeModel>()` or `Model<SphericalModel>()`. `__constant__ g_views` holds each view's
  `ModelView`: its pose relative to the reference, its center in the reference frame and its texture scale.
- `RefPixel` holds the reference model and its `RayFrame` at the pixel. An equirectangular reference builds
  the frame and every ray (Method A texels, propagation, surface normal) from the per-column/row tables
  `Params::refTrig`, as the CPU does; the table (`ComputeSphericalTrig`, `DepthMap.h`, double-precision
  angles stored in float) is the one of the CPU estimator, of the Metal estimator and of every confidence
  recalibration, and the fused CUDA recalibration reads the estimation's own copy before it is freed.
- `ScorePlane`: the warp `M = PixelHomography·H·F` of the CPU (`PlanePatchWarp`), `H·F = R·F + t·(nᵀF)/(n·X)`
  from the relative pose: into a pinhole target `K·H·F`, into an equirectangular one warp B with the
  4-corner check, warp A above `fSphericalWarpMaxElevation`. A plane not facing the reference camera, or
  whose patch center is behind a pinhole target, scores the bad cost. The texel walk and ZNCC are shared
  (`SampleWarpedPatch`, `ScoreNCC`).
- `GeometricConsistencyWeight` is the CPU round trip through the relative pose, with the wrapped column
  distance for an equirectangular reference; a point behind a pinhole camera scores the maximum distance.
- Sampling: an equirectangular view's image and depth textures take normalized coordinates, wrap on the
  columns (`cudaAddressModeWrap`) and clamp on the rows (the CPU rejects the view there).
- Propagation wraps its columns; red/black parity agrees across the seam only for an even width (ASSERTed in
  `RunCUDA`); the poles are not crossed. The 4 rows next to each pole are not estimated (cost `FLT_MAX`).
- Normals are camera-facing against the pixel ray (`RefPixel::F` column 2, any length); surface normal and
  propagation use the geometry of `DepthGeometry.h` (its pinhole closed forms for a pinhole reference).
- View gate (`MultiViewScorePlane`, scenes with a spherical view): the CPU rule on a bit mask over the
  neighbor views: `SelectViews` keeps at most `OPTDENSE::MAX_ESTIMATION_NEIGHBORS` (31), the Metal shader's
  limit too. Pinhole-only scenes run without it, as the pinhole CPU estimator does.
- `ProcessPixel` is `__forceinline__` into the kernels with a spherical view: under `-rdc` a callee is
  compiled with its own register count and must fit the 128 registers of the launch bounds. The pinhole
  kernels leave the inlining to the compiler, which keeps them free of spills.
- The host hands the kernels (CUDA and Metal, every model) the camera centers relative to the reference
  center, subtracted in double precision: the kernels only relate the views to each other, and single
  precision centers of a scene far from its origin round the baselines and the points of the geometric
  consistency.
- The confidence recalibration kernels (`ConfidenceCUDA.cu`) are templates on the reference model
  (`PinholeModel`, or `SphericalModel` with its trig table).
- `SmoothPlanes`, which builds the smoothed planes of the smoothness coupling before every pass
  (`PatchMatchSmoothness.md`), reads only the reference: it is a template on `MODEL_PINHOLE` (also for the
  `MODEL_PINHOLE_REF` passes) or `MODEL_SPHERICAL_REF`, whose 3x3 neighbourhood wraps across the seam.

### Metal estimator

`PatchMatchMetal.metal` mirrors the CUDA kernels by hand (MSL cannot include Eigen).
- Function constant `MODEL` (0, 1, 2 as in CUDA) next to `GEOM`; one set of functions serves every MODEL
  (`InitializePixelScore`, `ProcessPixel`, `ScorePlane`, `GeometricConsistencyWeight`,
  `MultiViewScorePlane`), the spherical branches and the view gate dead for `MODEL_PINHOLE`. `PatchMatch`
  builds the pipelines of every (GEOM, MODEL) pair; `EstimateDepthMap` picks MODEL from the spherical mask.
- One delimited block mirrors `CameraModels.h` and `DepthGeometry.h`, the pinhole closed forms of
  `InterpolatePlaneDepth` and `NormalFromDepthGradient` included. The test kernel `TestModelMath` runs it
  for `MetalModelMathTest` (Tests suite 2, Apple only), which compares it with the C++ models.
- Buffer 0 holds each view's `Camera` {fx, fy, cx, cy, size} (`AsPinhole`/`AsSpherical` build the models);
  buffer 8 the per-view `ModelView` {R, t, C, texScale}; buffer 9 the reference trig table (W+H (sin, cos)
  pairs, a dummy for a pinhole reference). A second sampler (`coord::normalized`, repeat on columns, clamp
  on rows) reads the textures of an equirectangular view. Buffer 10 holds the smoothed planes, written by
  `SmoothPlanes` (pipelines for `MODEL_PINHOLE` and `MODEL_SPHERICAL_REF`) before every black and red pass.
- Kernel logic as in CUDA, including the view gate and the smoothness coupling.
- Differences from CUDA: the relative view poses are composed in double precision (CUDA composes them in
  single precision from the reference-centered cameras); random numbers come from PCG, bilateral weights
  from `exp`.
- Kernels are compiled from source at runtime, with `ConfidenceMetal.metal` into one library; macOS caches
  the library and the pipelines across processes. CMake also compiles both offline (target
  `PatchMatchMetalCheck`) when the toolchain has a working Metal compiler.
- `DensifyPointCloud` gives Metal builds the CUDA defaults (`--number-views` 8, `--iters` 4); a CPU run in a
  GPU build (`--gpu-device cpu`) keeps the CPU defaults for the options not given explicitly.

### Fusion, filters and other consumers of the depth-maps

Everything downstream of the estimator reaches a view's geometry through its `Camera` (`libs/MVS/Camera.h`,
`Image::camera` or `DepthData::ViewData::camera`), which carries the projection model: the pinhole branch
of each call is the pinhole formula, the spherical one goes through `Spherical<REAL>`.
- `TransformPointI2C/I2W(x, depth)`, `ProjectPointP(X)` (pixel and range), `ProjectPointC(Xc)`,
  `TransformPointW2I`, `RayPoint`; `WrapPixel(ir, size)` (integer pixel, column wrapped, false if
  outside), the seam primitive for integer pixels; `GetPixel(x, size, ir)` (rounded, then `WrapPixel`);
  `ColumnOffset(a, b, width)` (shortest offset around a panorama).
- `WrapColumn(c, width)` (`DepthGeometry.h`, `HOST_DEVICE`) is the one column wrap of the host code; the
  CUDA and Metal kernels keep their own copy.
- Consumers, each with the model of every view it touches (mixed scenes fuse as such):
  `DepthMapsData::FuseDepthMaps`, `DenseFuseDepthMaps`, `MergeDepthMaps`, `ComputeIntraMapPrior`;
  the filters `RemoveSmallSegments`, `GapInterpolation` (a row of a panorama is a circle, so a gap across the
  seam is filled like any other; columns do not cross the poles) and `FilterDepthMap`;
  `EstimateNormalMaps`, `DepthData::GetNormal`, `EstimatePointColors`, `EstimatePointSegmentation`,
  `ExportPointCloud`, `Scene::EstimatePointCloudNormals`;
  `Camera::WrapPixel` also serves `EstimateConfidenceFromDepth/Normal` (their window across the seam) and
  `InterfaceMetashape::AssignPoints`; `Scene::EstimatePointCloudNormals` works on 1024 px copies of the scene
  images, a panorama's copy padded with a `cv::BORDER_WRAP` margin of `patchRadius+2` columns on each side and
  its `cx` moved by the margin, so the pinhole sampling code reads across the seam;
  `Scene::PointCloudFilter` (visibility cone aperture = the angle a pixel spans, 1/fx on the equator).

## 3. Parameters

| parameter | default | meaning |
|---|---|---|
| `DepthEstimatorBase::fSphericalWarpMaxElevation` | 70° | target elevation above which Method A replaces Method B |
| `OPTDENSE::fMinAngle` | 3° | view-gate angle (shared with the pinhole triangulation-angle test) |
| `--number-views` / `--iters` (Metal and CUDA builds) | 8 / 4 | GPU defaults; CPU builds 5 / 3 |

All other PatchMatch options apply unchanged.

## 4. Invariants

- A spherical image is 2:1 with `fx = W/2π`, `fy = H/π`, `c = (size−1)/2` at every pyramid level and
  neighbor scale; its width is even, and at least `DepthData::MIN_SPHERICAL_WIDTH` (32) at the coarsest
  level it is estimated at (`InitViews` refuses to estimate it otherwise: the coarsest sub-resolution
  level in the photometric pass, full resolution in the geometric ones): the propagation reaches 23 columns.
- Pinhole scenes run the unchanged code paths: the CPU depth-maps are byte-identical to the pre-model
  estimator.
- A `.dmap` carries `SPHERICAL` iff its view is spherical.
- Every estimate is read whole, from one pass.
- The hot loop has no model branch per texel.

## 5. Validation

The synthetic room has ray-cast ground truth. Holo360D Outdoor_004 uses 40 frames at
1440 px. Depth completeness is the fraction of ground-truth pixels with an estimate.
Fused F is the harmonic mean of precision and recall at the stated range tolerance.

| Dataset and shipped configuration | Depth completeness | Median relative depth error | Fused F |
|---|---:|---:|---:|
| Synthetic 1536, CPU | 0.911 | 0.00050 | 0.975 at 1% |
| Synthetic 1536, CUDA | 0.983 | 0.00041 | |
| Synthetic 1536, Metal | 0.982 | 0.00041 | |
| Mixed 96 px faces against two 384 px panoramas, CPU / CUDA / Metal | 0.838 / 0.991 / 0.990 | 0.00247 / 0.00226 / 0.00225 | |
| Holo360D, CPU | 0.954 | 0.201 | 0.522 at 5% |
| Holo360D, CUDA / Metal | 0.681 | 0.0471 | 0.617 at 5% |

Pinhole scenes run the same GPU kernels with the spherical branches compiled out: Tanks and Temples
Ignatius (263 images, CUDA, default options) fuses to F 0.803 at its 3 mm tolerance (precision 0.730,
recall 0.892).

The synthetic suite checks seam accuracy against the antipodal band and exercises CUDA or Metal
with the scene translated 1e6 m from the origin along every axis. `TestSphericalPatchWarp` measures
a 99th-percentile maximum texel error of 0.140 px below the 70-degree switch on 2880 x 1440 panoramas.

## 6. Rejected alternatives

- An affine approximation introduces median patch errors of 1 to 3.6 px.
- Using the tangent warp at the poles increases depth error. Using the exact warp everywhere costs more.
- Skipping columns near the poles reduces the fraction of accurate south-pole estimates.
- Rejecting a hypothesis when every neighbor falls below the triangulation threshold biases epipolar depths.
- An upper triangulation-angle gate at 65 degrees increased Holo360D median relative error.

## 7. Open items

- CUDA clamps the rows at the poles where the CPU rejects the view.
- The shape constants of the confidence recalibration were calibrated on pinhole ground truth; a
  spherical calibration (Holo360D) is untested.
- The view gate still ranks hypotheses that leave a pixel different views; a soft weight by the
  triangulation angle is untested.
- The corner waits cover up to 64 concurrent estimators; with more threads the corner anti-diagonals between
  64 and the thread count are read from the previous pass.
- `GapInterpolation` does not cross the poles; the rows next to the poles stay unestimated.
- `Scene::EstimatePointCloudNormals` (`SceneGeometry.cpp`): the columns of a panorama wrap through a margin,
  the rows do not. A view whose patch crosses a pole row is not chosen as a point's target, and as a neighbor it scores the largest
  residual; a point whose every view has its patch cross a pole row stays unrefined.
