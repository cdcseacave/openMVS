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
error or a warning:
- SGM (`--fusion-mode` < 0) and the neighbor views selected from a mesh: error;
- confidence recalibration and tower mode (`Scene::InitTowerScene`): switched off, with a warning.

The stages past the dense cloud project through the pinhole camera only and call
`Scene::CheckPinholeOnly(stage)`, an error that names the stage and `--spherical-export cubemap`:
`Scene::ReconstructMesh`, `RefineMesh`, `RefineMeshCUDA`, `TextureMesh`, `ComputeVertexColors`,
`ExportMeshToDepthMaps`, and the apps before `RemoveUnseenMeshFaces`. `MvgMvsPipeline.py` passes
`--spherical-export cubemap` to `CreateStructure` when the run includes a mesh step;
`MvsScalablePipeline.py` takes a scene as it is. `Scene::Split` back-projects depth-maps through `ModelCamera`.

## 2. Algorithm as implemented

### Depth and geometry

A depth is the distance λ along the model's ray, `X_cam = λ·Ray(x)`: λ = z for a pinhole (the pinhole
estimator is unchanged, depth-maps byte-identical), the range for a spherical camera. The depth geometry
(`InterpolatePlaneDepth`, `NormalFromDepthGradient`, `PlaneRayHomography`, `PlanePatchWarp`, depth
similarity) is written once over `Ray`/`RayFrame`/`PixelHomography` in `libs/Common/DepthGeometry.h`.

### Patch warp per target view (`DepthEstimatorT::SamplePatch`)

- Pinhole reference and target: the cached plane homography (`ComputePatchWarp`).
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
  target depth-maps of the geometric consistency wrap the same way (`SampleDepth<true>`). No padding.
- A spherical reference is estimated across the seam (only the 4 rows next to each pole are skipped); the
  reference patch and the propagation neighbors wrap their columns (`Col0`).
- Pixel order, pinhole and spherical alike: the estimators pull pixels in zigzag order and work on a copy of
  the pixel's estimate, written back as a whole between two state changes of its flag (`PixelPending`: 1 not
  estimated yet in this pass, 2 being written, 0 done). A neighbor's estimate is read as a whole
  (`ReadEstimate`): this pass's if that neighbor is done, else the previous pass's, never a mix. Only at the
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
  image (`Image::PointDepth`).
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
model kernels apply the same rule.

### Depth-map files

The `.dmap` header flag `SPHERICAL` (`HeaderDepthDataRaw`, `libs/Math/Interface.h`, `MvsUtils.py`) is set by
`DepthData::Save` from the reference camera model; `DepthData::Load` and the depth-map loads of `InitViews`
reject a file whose flag disagrees with the view's model.

### CUDA estimator

- The kernels (`InitializeScore`, `BlackPixelProcess`, `RedPixelProcess`) are templates on `<GEOM, MODEL>`:
  `MODEL_PINHOLE` (every view pinhole; the unchanged pinhole kernels), `MODEL_PINHOLE_REF` (pinhole
  reference, spherical views), `MODEL_SPHERICAL_REF`. `RunCUDA` picks it from `Params::sphericalViews`.
- `RefPixel` holds the reference model and its `RayFrame`; `__constant__ g_views` holds the per-view pose and
  texture scale. An equirectangular reference builds the frame and the Method A texel rays from the
  per-column/row tables `Params::refTrig`, as the CPU does.
- `ScorePlaneModel`: the CPU warps (B with the 4-corner check, A above `fSphericalWarpMaxElevation`, pinhole
  target `K·H·F`); a plane not facing the reference camera scores the bad cost. The texel walk and ZNCC are
  the pinhole ones (`SampleWarpedPatch`, `ScoreNCC`).
- Sampling: an equirectangular view's image and depth textures take normalized coordinates, wrap on the
  columns (`cudaAddressModeWrap`) and clamp on the rows (the CPU rejects the view there).
- Propagation wraps its columns; red/black parity agrees across the seam only for an even width (ASSERTed in
  `RunCUDA`); the poles are not crossed. The 4 rows next to each pole are not estimated (cost `FLT_MAX`).
- Normals are camera-facing against the pixel ray; surface normal and propagation use the generic geometry.
  `GeometricConsistencyWeightModel` is the CPU round trip with the wrapped column distance.
- View gate (`MultiViewScorePlane`, model kernels): the CPU rule on a bit mask over the neighbor views, at
  most `MAX_VIEWS` (32) including the reference.
- The model kernels are `__forceinline__`: under `-rdc` a callee is compiled with its own register count and
  must fit the 128 registers of the launch bounds.
- Pinhole scenes run the unchanged kernels; depth-maps are byte-identical.

### Metal estimator

`PatchMatchMetal.metal` mirrors the CUDA model kernels by hand (MSL cannot include Eigen).
- Function constant `MODEL` (0, 1, 2 as in CUDA) next to `GEOM`; the model kernels are separate functions
  (`InitializePixelScoreModel`, `ProcessPixelModel`, `ScorePlaneModel`, `GeometricConsistencyWeightModel`).
  `PatchMatch` builds the pipelines of every (GEOM, MODEL) pair; `EstimateDepthMap` picks MODEL from the
  spherical mask.
- One delimited block mirrors `CameraModels.h` and `DepthGeometry.h`. The test kernel `TestModelMath` runs it
  for `MetalModelMathTest` (Tests suite 2, Apple only), which compares it with the C++ models.
- Buffer 8 holds the per-view `ModelView` {R, t, C, texScale}; buffer 9 the reference trig table (W+H
  (sin, cos) pairs, a dummy for a pinhole reference). A second sampler (`coord::normalized`, repeat on
  columns, clamp on rows) reads the textures of an equirectangular view.
- Kernel logic as in CUDA, including the view gate (`MultiViewScorePlaneModel`).
- Differences from CUDA: plane propagation and surface normal take their rays from the trig tables; view
  poses are composed in double precision; random numbers come from PCG, bilateral weights from `exp`.
- Kernels are compiled from source at runtime; CMake also compiles them offline (target
  `PatchMatchMetalCheck`) when the toolchain has a working Metal compiler.
- `DensifyPointCloud` gives Metal builds the CUDA defaults (`--number-views` 8, `--iters` 4).

### Fusion, filters and other consumers of the depth-maps

Everything downstream of the estimator reaches a view's geometry through `ModelCamera`
(`libs/MVS/Image.h`, from `Image::GetModelCamera()` or `DepthData::ViewData::GetModelCamera()`): pinhole
calls are the camera's own (bit-identical), spherical calls go through `Spherical<REAL>`.
- `TransformPointI2C/I2W(x, depth)`, `ProjectPointP(X)` (pixel and range), `ProjectPointC(Xc)`,
  `TransformPointW2I`, `RayPoint`; `GetPixel(x, size, ir)` (rounded pixel, column wrapped, false if
  outside); `ColumnOffset(a, b, width)` (shortest offset around a panorama).
- `WrapColumn(c, width)` (`DepthGeometry.h`, `HOST_DEVICE`) is the one column wrap of the host code; the
  CUDA and Metal kernels keep their own copy.
- Consumers, each with the model of every view it touches (mixed scenes fuse as such):
  `DepthMapsData::FuseDepthMaps`, `DenseFuseDepthMaps`, `MergeDepthMaps`, `ComputeIntraMapPrior`;
  the filters `RemoveSmallSegments`, `GapInterpolation` (a row of a panorama is a circle, so a gap across the
  seam is filled like any other; columns do not cross the poles) and `FilterDepthMap`;
  `EstimateNormalMaps`, `DepthData::GetNormal`, `EstimatePointColors`, `EstimatePointSegmentation`,
  `ExportPointCloud`, `Scene::EstimatePointCloudNormals`;
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
  neighbor scale; its width is even.
- Pinhole scenes run the unchanged code paths: depth-maps are byte-identical to the pre-model estimator.
- A `.dmap` carries `SPHERICAL` iff its view is spherical.
- Every estimate is read whole, from one pass.
- The hot loop has no model branch per texel.

## 5. Validation

Synthetic room (`Tests sphtest`, `Tests sphfuse`, `Tests scorebench`; 3 panoramas 1536x768, cube faces
384x384) and Holo360D Outdoor_004 (40 frames at `--max-resolution 1440`, `Tests evaldepth`, `Tests evalcloud`).
A = exact warp everywhere, B = tangent warp with A above 70° (default), C = panoramas rendered to cube faces
and the pinhole PatchMatch (`CreateStructure --spherical-export cubemap`, `Tests facescene`).

Depth-maps, fraction of GT pixels within 1 / 5 % and abs-rel median:

| scene | method | device | completeness | abs-rel median | <1% | <5% |
|---|---|---|---|---|---|---|
| synthetic 1536 | A | CPU | 0.911 | 0.00047 | 0.983 | 0.994 |
| | B | CPU | 0.911 | 0.00050 | 0.982 | 0.994 |
| | C | CPU | 0.913 | 0.00079 | 0.913 | 0.952 |
| | A / B / C | CUDA | 0.983 / 0.983 / 0.970 | 0.00039 / 0.00041 / 0.00056 | 0.987 / 0.986 / 0.945 | 0.995 / 0.995 / 0.966 |
| | B | Metal | 0.982 | 0.00041 | 0.986 | 0.995 |
| synthetic 512x256 | B | CPU | 0.956 | 0.00147 | 0.919 | 0.986 |
| | B | CUDA / Metal | 0.964 | 0.00139 | 0.931 | 0.988 |
| Holo360D 1440 | A / B / C | CPU | 0.954 / 0.954 / 0.774 | 0.182 / 0.201 / 0.130 | | 0.352 / 0.346 / 0.375 |
| | A / B / C | CUDA | 0.680 / 0.681 / 0.644 | 0.0460 / 0.0471 / 0.0945 | | 0.513 / 0.509 / 0.406 |
| | B | Metal | 0.681 | 0.0471 | | 0.509 |

Fused clouds (precision / recall / F within th x range of the GT):

| scene | method | F@1% | F@2% | F@5% | F@10% |
|---|---|---|---|---|---|
| synthetic 1536 CPU | B | 0.975 | 0.991 | | |
| | C | 0.903 | 0.949 | | |
| synthetic 512 CPU | B / C | 0.806 / 0.637 | 0.931 / 0.757 | | |
| Holo360D 1440 CPU | A / B / C | | 0.346 / 0.338 / 0.308 | 0.535 / 0.522 / 0.562 | 0.679 / 0.667 / 0.734 |
| Holo360D 1440 CUDA | A / B / C | | 0.412 / 0.404 / 0.264 | 0.631 / 0.617 / 0.480 | 0.755 / 0.744 / 0.646 |
| Holo360D 1440 Metal | B | | 0.404 | 0.617 | 0.745 |

Synthetic 1536 B CPU: precision 0.999, recall 0.952 at 1 %; per-band F@1% equator 0.963, north 1.000, south
0.964, seam 0.974. Against C the panoramas win on every band, most at the south pole (C: 0.711).

Reading the numbers:
- B matches A in accuracy and costs 0.8x of it on CPU, 0.7x on CUDA. C is less accurate and less complete on the synthetic
  room and on Holo360D CUDA, recovers more ground recall only on Holo360D CPU at the loose thresholds, loses
  whole faces on real captures (Cocarascu: 160 of 720 reference views without neighbors) and needs a face
  export (176-635 s on the real captures).
- On Holo360D the GPU estimators keep fewer pixels than the CPU one and are more accurate on every band;
  the pixels only the CPU keeps are wrong (abs-rel median 0.81). The cause is the estimators (view
  selection, the cost threshold of `FilterPlanes`), not the spherical model: on synthetic data CUDA matches
  or beats the CPU on every band.
- Holo360D GT and poses agree with the images to a few pixels only; all methods are bounded by it.
- The synthetic panoramas look along their trajectory, so the epipoles of their neighbors lie in the seam
  band and its antipodal band, where the view gate acts. `SyntheticSphericalTest` requires seam <5 % above
  center − 0.01 and completeness above 0.99 x center.

Cost, synthetic 1536:

| measure | result |
|---|---|
| `scorebench`, ns per ScorePixel: A / B / pinhole faces | 231 / 156 / 127 (1.82x / 1.23x) |
| CPU whole run, µs per GT pixel: A / B / C | 5.08 / 4.07 / 3.9 |
| CUDA whole run, µs per GT pixel: A / B / C | 0.197 / 0.141 / 0.183 |
| Metal (M3 Max) whole run, µs per GT pixel | 0.107 (CPU on that machine 2.43) |
| CUDA kernel, panorama vs faces, 1024 px / 3840 px | 1.62x / 0.70x photometric, 1.27x / 0.47x geometric |
| pinhole fountain-P11 CPU depth-maps + fusion vs the pre-model estimator | 202.1 s vs 201.4 s (+0.3 %) |

The CUDA excess at small sizes is Method A near the poles (~4.3x the cost of B on ~12 % of the
evaluations) and the per-hypothesis model math; at real panorama sizes the spherical kernels are cheaper
per pixel than the faces. Real captures (7680x3840 keyframes, `--max-resolution 1440`): CUDA B, Cocarascu
120 frames 165 s, 2.36 M points, depth share 0.898; MaidenLane 91 frames 105 s, 5.99 M points, 0.972.

Warp accuracy (`TestSphericalPatchWarp`, random poses and slanted planes, 2880x1440, 9x9 patch): B against A,
per-patch max texel error p99 0.042 / 0.079 / 0.140 / 0.757 px for target elevations below 30 / 50 / 70 / 90°;
seam-crossing patches max 0.23 px; the affine warp has a median error of 1.0-3.6 px.

Seam and pixel order (synthetic 512, B, 24 threads): the seam band matches its antipodal band within 0.003
on <5 %. Without the corner waits, the bottom-right corner of the zigzag order has 145-285 wrong estimates
per run, with them 5-31 (single thread 11-54).

Reference numbers for the regression run (`Tests sphtest 2 512 3 1 70 0 0 0` CPU, `... 0 0 1` CUDA):
per-band <5 % CPU all 0.985-0.988, seam 0.996-0.997; CUDA all 0.964 completeness, abs-rel 0.00139.

## 6. Rejected alternatives

- Affine warp (M with its projective row frozen at the patch center): 1-3.6 px median texel error.
- Method B everywhere: 1/cosφ error at the poles (south band abs-rel 0.00064 vs 0.00042).
- Method A everywhere: 1.4-1.5x the cost of B.
- Exact warp only above 80°: pole bands lose 20-35 % abs-rel at 512x256 for 1.32x instead of 1.62x the CUDA
  cube-face cost at 1024 px.
- Padding equirectangular images with wrapped columns: patches off the seam take the plain loop.
- `std::atan2` in the warps: the exact warp spends most of its time there.
- Pixels waiting for all their preceding neighbors: 5-8 % multi-threaded wall, idle cores.
- Setting aside a pixel whose preceding neighbors are in progress: +40 % wall.
- Eigen fixed-size expressions in the per-hypothesis warp code: not inlined by MSVC, ~10 ns per 3x3 product.
- Pole stride (estimate ~1/cos(elevation) columns per row, fill the rest): south <5 % 0.98 -> 0.44.
- Isotropic angular patch; filling the 4 pole rows from the nearest estimated row: fused F within noise.
- An upper triangulation-angle gate (65°): Holo360D abs-rel median 0.28 -> 0.74.
- Dropping the `FilterNeighborViews` angle filter for panoramas: no change with the per-pixel gate.
- View-gate variants (an all-gated hypothesis scoring `thRobust`; gated views left out of the aggregation;
  one view set per pixel and pass; dropping estimates every view sees below `fMinAngle`): F@5 % equal or
  lower; the shipped rule has the fewest wrong estimates among pixels every neighbor sees below 3°.

## 7. Open items

- Per-hypothesis cost is ~1.23x pinhole against a 1.2x target: the center projection (two `FastAtan2`, a
  square root, a reciprocal) and the per-pixel ray frames, amortized over few hypotheses in `scorebench`.
- Metal compiles the kernels from source at the first estimation of every process (~3 s on an M3 Max).
- CUDA clamps the rows at the poles where the CPU rejects the view. Mixed pinhole/spherical scenes run the
  `MODEL_PINHOLE_REF` kernels (CUDA, Metal), untested.
- The confidence recalibration (`ConfidenceRefine.h`, `ConfidenceCUDA.cu`) is pinhole only
  (`AdjustConfidenceSweep` ASSERTs a pinhole reference).
- The view gate still ranks hypotheses that leave a pixel different views; a soft weight by the
  triangulation angle is untested.
- The corner waits cover up to 64 concurrent estimators; with more threads the corner anti-diagonals between
  64 and the thread count are read from the previous pass.
- `GapInterpolation` does not cross the poles; the rows next to the poles stay unestimated.
