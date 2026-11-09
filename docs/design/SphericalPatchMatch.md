# Spherical PatchMatch

## 1. Purpose and scope

`DensifyPointCloud` estimates the depth-maps of equirectangular (360x180 degree) images natively,
without rendering them to cube faces: the CPU PatchMatch takes the reference camera model as a
template parameter (`DepthEstimatorT<Pinhole>`, `DepthEstimatorT<Spherical>`, `libs/MVS/DepthMap.h`),
the CUDA PatchMatch as a kernel template argument (`libs/MVS/PatchMatchCUDA.cu`, "CUDA estimator" below),
the Metal PatchMatch as a function constant (`libs/MVS/PatchMatchMetal.metal`, "Metal estimator" below),
and all resolve the camera model of each neighbor view once per (pixel, hypothesis, view). Pinhole,
Brown (undistorted on load, see `SphericalCameraSupport.md`) and spherical views can be mixed.

Scope: depth-maps and their fusion into the dense cloud, with the photometric confidence: the
depth-maps on the CPU (`--gpu-device -2`), CUDA or Metal, the fusion on the CPU. `Scene::DenseReconstruction`
refuses a scene with spherical images for SGM (`--fusion-mode` < 0), with a clear error; the confidence
recalibration is switched off for it.

## 2. Algorithm as implemented

### Depth and camera models

A depth is the distance λ along the model's ray, `X_cam = λ·Ray(x)`. Pinhole rays lie on the z=1
plane, so λ = z and the pinhole estimator is unchanged (byte-identical depth-maps); spherical rays
are unit vectors, so λ is the range. The depth geometry (plane propagation `InterpolatePlaneDepth`,
`NormalFromDepthGradient`, depth similarity) is written once over `Ray`/`RayFrame`
(`libs/Common/DepthGeometry.h`).

`Spherical<T>` (`libs/Math/Camera/CameraModels.h`, `HOST_DEVICE`, templated on the scalar) has,
like `Pinhole<T>`:
- `RayFrame(x0)`: columns ∂r/∂x = (cosφcosθ, 0, −cosφsinθ)/fx, ∂r/∂y = (sinφsinθ, cosφ, sinφcosθ)/fy
  and r(x0): pixel offsets (dx,dy,1) around x0 to rays, to first order;
- `PixelHomography(q[, x])`: rays near q to homogeneous pixels, the gnomonic projection on the plane
  tangent at q scaled by the equirectangular Jacobian, written on the unnormalized q (rows
  `fx·|q|²/ρ²·(q_z,0,−q_x) + x_c·q`, `fy·(−q_x·q_y/ρ, ρ, −q_z·q_y/ρ) + y_c·q`, `q`, with
  ρ = |q|·cosφ and (x_c, y_c) = Project(q), which the caller may pass); exact at q, first order
  around it.

### Patch warp per target view (`DepthEstimatorT::SamplePatch`)

- Pinhole reference and pinhole target: the cached plane homography (`ComputePatchWarp`).
- Otherwise Method B: `M = Trg.PixelHomography(H·r0)·H·Ref.RayFrame(x0)`, with `H·F` built as
  `Rrel·F + t·(nᵀF)/(n·X)` from `Rrel·F` cached per pixel and view, in single precision; M feeds the
  same base/dx/dy texel loop as the pinhole homography. A pinhole target must see the patch center
  in front of it. For a spherical target the four patch corners decide: a corner behind the tangent
  plane (a texel ray more than 90 degrees off the center) rejects the view, a patch off the seam is
  sampled by the plain pinhole loop, a patch across it by the wrapping one (`SampleTangentPatch`).
- Method A, the exact warp, where the target sees the patch center above
  `DepthEstimatorBase::fSphericalWarpMaxElevation`: each texel ray (cached per pixel, and turned
  into the view frame per pixel and view) is cut with the plane and projected by the target model.
  A texel whose plane intersection is behind the reference camera scores the view `thRobust`, as an
  out-of-image texel does. The texel rays are stored as structure-of-arrays padded to whole SIMD
  packets, and the intersections and projections run on Eigen's packet functions
  (`Eigen::internal::p*`, SSE or NEON), lane by lane identical to the scalar code; only the bilinear
  samples are taken per texel.
- The spherical projections of both warps use `Spherical::ProjectFast` through `FastAtan2`
  (`CameraModels.h`, shared with CUDA: a minimax polynomial, max error 1.3e-7 rad, the precision of
  the float atan2; `DepthMap.cpp` evaluates the same polynomial on SIMD packets) and `FastSqrt` (the
  SIMD square root: `std::sqrt` is a library call under `/fp:strict`). Method B computes
  ρ = √(q_x²+q_z²) once for the projection of the patch center and for
  `Spherical::PixelHomography(q, x, ρ)`.
- The 3x3 products of the warps (`Rrel·F` per pixel and view, `H·F`, `P·(H·F)`, the patch warp and
  the geometric-consistency transforms) are written out on the Eigen storage (`MulMat3`): MSVC does
  not inline the fixed-size Eigen products in these builds (`EIGEN_STRONG_INLINE=inline`), and they
  were the main cost of the model warps.

The error of B grows as 1/cosφ of the target elevation of the patch center, not of the reference
one. The choice between B and A is one branch per (pixel, hypothesis, view); there is no per-texel
branch on the model.

### Sampling and the seam

- Spherical target images: the columns wrap (bilinear across the seam), the rows keep the 1-pixel
  border (`SampleTexel<true>`); no padding. Target depth-maps of the geometric consistency wrap the
  same way (`SampleDepth<true>`).
- Spherical reference: patches are estimated across the seam (only the 4 rows next to each pole
  are skipped); the reference patch and the propagation neighbors wrap their columns (`Col0`).
- Pixel order (pinhole and spherical alike): the estimators pull pixels in the zigzag order and work
  on a copy of the pixel's estimate, written back to the maps as a whole at the end of the pixel
  between two state changes of its per-pixel flag (`PixelPending`: 1 not estimated yet in this pass,
  2 being written, 0 done). A neighbor's estimate is read as a whole (`ReadEstimate`, read again if
  the flag changed meanwhile): the one of this pass if that neighbor is done, else the previous
  pass's; never a mix of the two. A single thread reads its lower neighbors final and the higher
  ones from the previous pass. Several threads wait only on the corners of the zigzag stripes: there
  the anti-diagonals are shorter than the number of estimators, which then work on adjacent pixels,
  so a pixel waits for a lower neighbor still in flight fewer than
  `DepthEstimatorBase::nZigzagMinStride` (64, the smallest stripe height) pixels before it in the
  order, and reads it as a single thread does. Elsewhere a lower neighbor is a whole anti-diagonal
  back and done, or its estimator is preempted and the pixel reads the previous pass's estimate
  rather than stall. The corners of a panorama lie on the seam next to the poles. The first and the
  last column of a spherical reference are neighbors like any other two (the one across the seam
  comes later in the order, so nobody waits for it).

### Geometric consistency

`GeometricConsistency<TRG>`: the reference point moved into the frame of the target depth-map,
projected, the target depth sampled (similar depths only), the target point at that depth
(`X1·depth/Depth(X1)`, along the same ray) projected back; the pixel distance to x0, its columns
wrapped for a spherical reference, mapped to [0,4]. The pinhole pair keeps its cached form.

### Normals

Random and perturbed normals are drawn as directions (`Dir2Normal`) in a frame whose z axis is the
pixel ray: the camera frame for pinhole (unchanged), the frame (e_lon, e_down, ray) of the pixel
for spherical (`normalFrame0`, e_lon = (cos lon, 0, −sin lon), e_down = (sin el·sin lon, cos el,
sin el·cos lon)), where the polar angle of the turned normal is taken by atan2 (the turned vector is
unit only to the rounding). `CorrectNormal` is ray-based. `EstimateNormalMap` of a spherical map
fits the depth gradient with wrapped columns.

### Reference trigonometry

A spherical reference tabulates, per estimator, the sine and cosine of the longitude of every
column and of the elevation of every row (`lonSinCos0`, `elSinCos0`): the ray frame at x0
(`Spherical::RayFrame(sinLon, cosLon, sinEl, cosEl)`), the normal frame, the texel rays of Method A,
and the rays of the propagation (`InterpolatePlaneDepth` on rays, `ComputeNeighborPoint`) are built
from them, with no trigonometry per pixel.

### Initialization and view selection

- Sparse init of a spherical reference: every sparse point it sees is splatted as a 5x5 square of
  its range (columns wrapping), the other pixels start random; dMin/dMax are those ranges x0.9/x1.1.
  The Delaunay triangulation of the sparse points stays pinhole only.
- `Scene::SelectNeighborViews` and the ROI's camera-to-scene distance take the range as the depth
  of a spherical image (`Image::PointDepth`); every direction projects inside a panorama.
- `Scene::EstimateGravityDirection`: a panorama supports only the gravity candidates along its
  image-down axis (any horizontal direction is perpendicular to its x axis, so the pinhole roll
  test would accept its forward direction).
- Pyramid levels and neighbor scaling scale K per axis, so a spherical view keeps
  fx = W/2π, fy = H/π, c = (size−1)/2 on its grid; the estimator ASSERTs it.
- Per-pixel view gate: the per-view triangulation angle of `Scene::FilterNeighborViews` (an average
  over the shared sparse points) says little about a panorama, which sees a neighbor along its
  baseline in some directions and across it in others. When a spherical camera is involved
  (`bModelWarp`), `ScorePixel` also gates every (hypothesis, view), at the hypothesis depth:
  1. X = depth·Ray(x0), the hypothesis point in the reference frame; a view is gated where the
     angle at X between the reference center (the origin) and the view's center (`viewCenters0`,
     cached per estimator) is below `OPTDENSE::fMinAngle` (3°): X·(X−C) > cos(fMinAngle)·|X|·|X−C|;
  2. if that gates every view, none is gated;
  3. a gated view scores `thRobust`, like a view that does not see the patch (the min-mean
     aggregation leaves it out); the others score their NCC as usual.

  Step 2 keeps the ranking of the hypotheses of a pixel next to an epipole fair: its true depth
  gates every view, while a closer depth sees some view at a larger angle; scoring the all-gated
  hypothesis `thRobust` made the closer ones win for that alone (synthetic: 94 % of those pixels
  too close by 19–26 % median). The views a pixel sees above fMinAngle at some depth thus still
  decide alone there, and the pixels no view sees above it are estimated from all of them.
  Pinhole-only estimators do not gate (bit-identical depth-maps). The CUDA and Metal model kernels
  apply the same rule (below).

### Depth-map files

The `.dmap` header carries the flag bit `SPHERICAL` (`HeaderDepthDataRaw`, `Interface.h`,
`MvsUtils.py`): `DepthData::Save` sets it from the reference camera model; `DepthData::Load` and the
depth-map loads of `InitViews` reject a file whose flag disagrees with the view's model.

### CUDA estimator

The kernels (`InitializeScore`, `BlackPixelProcess`, `RedPixelProcess`) are templates on `<GEOM, MODEL>`:
`MODEL_PINHOLE` when every view is pinhole (the plane homography of `ComputeHomography`, unchanged),
`MODEL_PINHOLE_REF` for a pinhole reference with spherical views, `MODEL_SPHERICAL_REF` for an
equirectangular reference; `RunCUDA` picks it from the per-view mask `Params::sphericalViews`.
- Per pixel, `RefPixel` holds the reference model and its `RayFrame` F; per view, `__constant__
  g_views` holds the pose relative to the reference (R, t) and the texture-coordinate scale. An
  equirectangular reference builds F and the texel rays of the exact warp from the sines and cosines
  of each column's longitude and each row's elevation (`Params::refTrig`, filled per level on the
  host), as the CPU estimator does: no transcendental per pixel or texel, no per-pixel ray cache.
- `ScorePlaneModel`: H·F = R·F + t·(nᵀF)/(n·X) (a plane not facing the reference camera at the pixel,
  n·X ≥ 0, scores the bad cost); into a spherical target the tangent-plane warp
  `PixelHomography(q0, ProjectFast(q0))·H·F` with the 4-corner check, or the exact per-texel warp
  above `fSphericalWarpMaxElevation`, the same per (pixel, hypothesis, view) choice as the CPU; into a
  pinhole target K·H·F
  with the patch center in front and inside. The texel walk and the ZNCC are the pinhole ones
  (`SampleWarpedPatch`, `ScoreNCC`).
- Sampling: an equirectangular view's image and depth textures take normalized coordinates
  (`cudaAddressModeWrap` on the columns, the only addressing the hardware wraps, clamp on the rows);
  the texel-center offset and 1/W, 1/H are folded into the warp rows. The hardware bilinear weights
  keep their 1/256 quantization: on a random texture the mean interpolation error is 6.9e-4 at
  3840 px and 6.5e-4 at 1440 px against 6.4e-4 with unnormalized coordinates, the same across the
  seam. Rows outside the image are clamped (the pole row), where the CPU rejects the view.
- Propagation: the 8 sampling directions and the 4-neighbors wrap their columns; the red/black
  parity of a pixel and its wrapped neighbors agrees only for an even width (ASSERTed in
  `RunCUDA`); the poles are not crossed (a crossing moves the column by W/2, which flips the parity
  when W = 2H). The 4 rows next to each pole are not estimated (cost `FLT_MAX`, never a neighbor).
- The per-pixel view gate of the CPU estimator, per (pixel, hypothesis) on every scoring call: the
  views are gated together first (a bit mask; none if all would be), a gated view scores the bad cost
  as a view that does not see the patch, the others are scored as usual.
- Normals: random and perturbed normals are isotropic, made camera-facing against the pixel ray
  F.col(2), so they are the same in the ray's local frame; the surface normal and the plane
  propagation use the generic `NormalFromDepthGradient`/`InterpolatePlaneDepth`.
- Geometric consistency (`GeometricConsistencyWeightModel`): the CPU round trip through both
  models, the column distance wrapped for a spherical reference.
- View gate (`MultiViewScorePlane`, the model kernels): per (pixel, hypothesis), as `ScorePixel`, a view
  whose center C (`g_views`, reference frame) sees X = depth·r at X·(X−C) > cos(`fMinAngle`)·|X||X−C|
  scores the bad cost of a view that does not see the patch, unless every view is gated; its
  geometric-consistency term is added as for any unseen view. At most 31 views (ASSERT).
- The model kernels inline the whole scoring path (`__forceinline__`): under `-rdc` a callee is
  compiled with its own register count and must fit the 128 registers of the launch bounds.
- Pinhole scenes run the unchanged kernels: their SASS is identical but for the stack layout of the
  local arrays of the two geometric-consistency kernels, and the depth-maps are byte-identical.

### Metal estimator

`PatchMatchMetal.metal` mirrors the CUDA model kernels by hand: MSL cannot include Eigen or the shared
HOST_DEVICE headers.
- Specialization: the function constant `MODEL` next to `GEOM`: 0 (`MODEL_PINHOLE`) every view pinhole,
  the pinhole kernels textually unchanged; 1 (`MODEL_PINHOLE_REF`) a pinhole reference with spherical
  views; 2 (`MODEL_SPHERICAL_REF`) an equirectangular reference. The model kernels are separate functions
  (`InitializePixelScoreModel`, `ProcessPixelModel`, `ScorePlaneModel`, `GeometricConsistencyWeightModel`)
  that the kernels select by `MODEL`. `PatchMatch` builds the pipelines of every (GEOM, MODEL) pair when it
  is created; `EstimateDepthMap` picks MODEL from the per-view spherical mask, as `RunCUDA` does.
- Camera models: one delimited block mirrors `CameraModels.h` function by function (`FastAtan2` with the same
  coefficients; Spherical `Ray` and `RayFrame` from sines and cosines, `ProjectFast` with rho, `Depth`,
  `PixelHomography` with rho; Pinhole `Ray`, `Project`, `RayFrame`, `PixelHomography`) and `DepthGeometry.h`
  (`InterpolatePlaneDepth` on rays, the `DepthPlaneFit` gradient of the 4-neighbors, the generic
  `NormalFromDepthGradient`). A pinhole reference uses the existing `InterpolatePixel` and
  `ComputeDepthGradient`, which are the closed forms of the pinhole overloads. The test kernel
  `TestModelMath` evaluates the block for `MetalModelMathTest` (Tests suite 2, Apple only), which compares
  it with the C++ models on random inputs.
- Buffers: `Params` adds `sphericalViews`, `fSinSqWarpMaxElevation` and `cosMinViewAngle`. Buffer 8 holds
  the per-view `ModelView` {R, t, C, texScale}, where C is the view center in the reference frame. Buffer 9
  holds the reference trig table: W+H (sin, cos) pairs, or a one-entry dummy for a pinhole reference. Both
  are filled per level on the host.
- Sampling: a second constexpr sampler (`coord::normalized`, linear, `s_address::repeat`,
  `t_address::clamp_to_edge`) reads the image and depth textures of an equirectangular view, chosen per view
  by the mask, at `(x+0.5)*texScale`.
- The kernel logic is that of the CUDA estimator: `ScorePlaneModel` (Method B with the 4-corner check,
  Method A above the elevation threshold, the pinhole-target branch); the wrapped propagation without pole
  crossing and an even-width ASSERT; the pole rows skipped; F.col(2) as the view direction of the random
  normals; the model geometric consistency with the wrapped column distance.
- The per-pixel view gate of the CPU estimator (`MultiViewScorePlaneModel`): per (pixel, hypothesis), on
  every scoring call, X = F.col(2)*depth and each view v with `X.(X-C_v) > cos(fMinAngle)|X||X-C_v|` sets
  its bit in a mask; a full mask is cleared; a masked view scores the bad cost without its warp, as a view
  that does not see the patch (the geometric term is added to it as to any such view). The host ASSERTs
  fewer than 32 neighbor views.
- Differences from CUDA: the plane propagation and the surface normal of an equirectangular reference take
  their rays and ray frame from the trig tables, as the CPU estimator does (CUDA evaluates `Ray` and
  `RayFrame` from the float angles there). The view poses are composed in double precision. The random
  numbers come from PCG instead of curand, and the bilateral weights from `exp` instead of the LUT, as in
  the pinhole Metal kernels. The model kernels skip an invalid neighbor in the plane comparison (cost
  `FLT_MAX`, as CUDA); the pinhole Metal kernels aggregate its unwritten costs and reject it afterwards.
- The host applies the ignore mask, with the validity mask of undistorted images, as CUDA does. The kernels
  are compiled from source at runtime; CMake also compiles `PatchMatchMetal.metal` with the offline Metal
  compiler (target `PatchMatchMetalCheck`) when the toolchain has a working one. The configure step that
  embeds the source re-runs when the file changes.

### Fusion, filters and the other consumers of the depth-maps

Everything downstream of the estimator reaches a view's geometry through `ModelCamera`
(`libs/MVS/Image.h`, from `Image::GetModelCamera()` or `DepthData::ViewData::GetModelCamera()`): a
camera with the model of its view, whose pinhole calls are the camera's own (the pinhole results are
bit-identical) and whose spherical calls go through `Spherical<REAL>` built from the angular K:
- `TransformPointI2C/I2W(x, depth)`: the point of pixel x at the depth along the model's ray;
  `ProjectPointP(X)`: pixel and depth (range) of a world point; `TransformPointW2I`, `RayPoint`;
- `GetPixel(x, size, ir)`: the rounded pixel, its column wrapped for a panorama, false if outside;
  `ColumnOffset(a, b, width)`: the shortest column offset around a panorama; `WrapColumn`.

The consumers, each with the model of every view it touches, so mixed pinhole and spherical
scenes fuse as such:
- `DepthMapsData::FuseDepthMaps`, `DenseFuseDepthMaps` (back-projection, projection into the
  neighbor depth-maps with wrapped columns, the reprojection-error test on the wrapped column
  offset), `MergeDepthMaps`, the intra-map prior of the dense fusion (`ComputeIntraMapPrior`: the
  depth-gradient fit and the 3x3 window wrap, the gradient normal of the model);
- the depth-map filters: `RemoveSmallSegments` (4-neighbors wrap), `GapInterpolation` (a row of a
  panorama is a circle, walked once around from its first valid pixel, so a gap across the seam
  is filled like any other; columns do not cross the poles), `FilterDepthMap` (per pixel);
- `EstimateNormalMaps` (the spherical `EstimateNormalMap`), `DepthData::GetNormal` (the window
  wraps), `EstimatePointColors` (the range picks the closest view; nearest-pixel sampling next to
  the seam), `EstimatePointSegmentation`, `ExportPointCloud`, `Scene::EstimatePointCloudNormals`;
- `Scene::PointCloudFilter`: the visibility cone of a point is the ray from the camera center to
  it, its aperture the angle a pixel spans (1/fx on the equator of a panorama);
- `Scene::EstimateNeighborViewsPointCloud` (a scene without sparse points): a pair with a panorama
  gets a grid over the whole first image, at depths log-uniform in [0.1, 10] x the depth where
  directions normal to the baseline meet at `fOptimAngle`, kept where the second view sees them.

## 3. Parameters

| parameter | default | meaning |
|---|---|---|
| `DepthEstimatorBase::fSphericalWarpMaxElevation` | 70 deg | target elevation above which Method A replaces Method B |

All other PatchMatch options apply unchanged.

## 4. Validation

Synthetic room (`Tests sphtest`, `Tests scorebench`, `TestSphericalPatchWarp`; CPU unless marked CUDA,
RelWithDebInfo), Holo360D Outdoor_004 (40 frames at `--max-resolution 1440` unless noted, `Tests evaldepth`):

| case | completeness | abs-rel median | <1% | <5% | notes |
|---|---|---|---|---|---|
| synthetic 3 panoramas 1536x768, A only | 0.911 | 0.00047 | 0.982 | 0.994 | equator 0.00069, north 0.00030, south 0.00042, seam 0.00051 |
| same, B + A above 70°, no view gate | 0.911 | 0.00050 | 0.983 | 0.995 | equator 0.00072, north 0.00030, south 0.00041, seam 0.00055 |
| same, B + A above 80° | 0.911 | 0.00051 | 0.982 | 0.994 | south 0.00049 |
| same, B only | 0.911 | 0.00053 | 0.982 | 0.994 | south 0.00064, seam 0.00062 |
| their 18 cube faces 384x384, pinhole | 0.913 | 0.00079 | 0.912 | 0.952 | border band completeness 0.69 |
| synthetic 512x256, A only / B70, no view gate | 0.956 / 0.956 | 0.00116 / 0.00145 | 0.926 / 0.920 | 0.984 / 0.985 | faces 128x128: 0.854, 0.00273, 0.756, 0.921 |
| synthetic 512x256, B70, view gate (default), 24 threads, 8 runs | 0.956 | 0.00145–0.00150 | 0.919–0.922 | 0.985–0.988 | <5%: equator 0.965–0.974, south 0.978–0.987, seam 0.996–0.997, center (longitude 0) 0.994–0.996; completeness seam 0.951, center 0.957 |
| Holo360D, B + A above 70°, no view gate | 0.956 | 0.203 | 0.131 | 0.343 | equator 0.974 / 0.111 / 0.149 / 0.390, seam 0.951 / 0.208 / 0.136 / 0.333, south (the ground under the handheld camera, behind the operator in the neighbors) 0.945 / 1.94 |
| Holo360D, cube faces 416x416, pinhole | 0.774 | 0.130 | 0.126 | 0.374 | same pipeline on the rendered faces |
| synthetic 3 panoramas 1536x768, B + A above 70°, view gate (default) | 0.911 | 0.00050 | 0.982 | 0.994 | equator 0.00072 / <5% 0.987, north 0.00031 / 1.000, south 0.00040 / 0.992, seam 0.00054 / 0.998, center 0.00054 / 1.000 |
| Holo360D, B + A above 70°, view gate (default), 2 runs | 0.954 | 0.193–0.197 | 0.133–0.134 | 0.341–0.344 | equator 0.972 / 0.111–0.112 / 0.150–0.151 / 0.388–0.389, seam 0.947–0.949 / 0.198–0.213 / 0.136–0.137 / 0.333–0.334 |
| CUDA: synthetic 512x256, B70, view gate | 0.964 | 0.00139 | 0.931 | 0.988 | equator 0.991 / 0.00218, north 0.857 / 0.00068, south 0.850 / 0.00092, seam 0.968 / 0.00153 / <5% 0.994, center 0.966 / 0.00163 / <5% 0.991 (CPU 0.956 / 0.00147 / 0.919 / 0.986, seam 0.951 / 0.00166, center 0.958 / 0.00168); normal median 3.2° (CPU 4.0°); without the gate 0.964 / 0.00140 |
| CUDA: Holo360D 10 frames at 720, B70, view gate | 0.711 | 0.056 | 0.209 | 0.483 | CPU with the gate on the same: 0.958 / 0.284 / 0.090 / 0.269; CUDA without the gate 0.718 / 0.057 / 0.203 / 0.481; the CPU's best 71.8 % by confidence (no gate) 0.173 / 0.113 / 0.338 |

On Holo360D the CUDA estimator keeps fewer pixels than the CPU one and is more accurate on every band
(equator 0.79 / 0.047 against 0.97 / 0.20, south 0.62 / 0.51 against 0.95 / 1.99): the pixels only the
CPU keeps (25 % of the GT pixels, none in the pole rows, which the GT masks) are wrong (abs-rel median
0.81, 9 % within 5 %, low CPU confidence), and on the pixels both keep CUDA is still better (0.056
against 0.189). The difference is the estimators' (ACMH view selection, the cost threshold of
`FilterPlanes`), not the spherical model, as on synthetic data, where CUDA matches or beats the CPU on
every band.

The pole bands (|elevation| > 70°) lose the 4 rows next to each pole and the floor below the
synthetic cameras; their accuracy matches the equator's. Holo360D is a handheld capture whose
LiDAR-derived GT and poses agree with the images to a few pixels only (a GT-predicted
correspondence lands within 1 px of the image match in 12–48 % of the cases at 2880 px): both the
spherical and the cube-face estimates are bounded by it alike.

The synthetic panoramas look along their trajectory, so the epipoles of their neighbors lie in the
seam band and in its antipodal band at longitude 0 (two of the three panoramas, at columns 15 / 271
and 3–4 / 259–260 of 512), where the neighbors see the scene at small angles and the view gate
acts: the seam band is compared with that antipodal band, not with the equator
(`SyntheticSphericalTest`: seam <5% above center − 0.01, completeness above 0.99 x center). The two
agree within 0.003; target patches across the seam scored unseen cost the seam 0.06–0.08 (0.904–0.920
against 0.983–0.984, completeness 0.945 against 0.957), and no propagation across the seam fails the
south band (<5% 0.933–0.947 against 0.978–0.987). Of the 1983 pixels whose true point both
neighbors see below 3° (2.0 % of the seam band, 1.3 % of the equator), 1976–1979 are estimated and
57–60 of them off by more than 5 % (71 without the view gate). Without the corner waits of the
pixel order, 24 threads left 145–285 wrong estimates per run in the bottom-right corner of the
zigzag order (the seam next to the south pole) against 11–54 single-threaded, seam <5%
0.984–0.991 against 0.995 without the view gate; with them 5–31 and 0.997.

Patch warp, B against A over random poses and slanted planes (2880x1440, 9x9 patch step 2):
per-patch max texel error p99 0.042 / 0.079 / 0.140 / 0.757 px for target elevations below
30 / 50 / 70 / 90°, seam-crossing patches max 0.23 px; the affine warp: median 1.0–3.6 px.

Cost (`scorebench`, one ScorePixel against one view, all texels seen, the pixel prepared once per 4
hypotheses; min of 3x5 alternating runs, timing-exclusive machine): panorama 1536x768 B70 158 ns,
B only 155 ns, A only 225 ns; cube faces of the same image size 126–129 ns: B70 1.22–1.25x, B only
~1.21x, A only ~1.8x the pinhole cost. Whole runs (`sphtest 1536`, 24 threads, B70): the 3 panoramas
take 9.6 s, 2.71 µs per GT pixel, their 18 cube faces 384x384 7.4 s, 2.80 µs per GT pixel (the
panoramas hold 1.33x the pixels of their faces).

CUDA kernel time (PMTIME events, black+red ms per MP, view and iteration at full resolution; synthetic,
4 panoramas and their cube faces, 3 views each, min of 3, one exclusive run): at 1024x512 against
256x256 faces, photometric 2.37 vs 1.46 ms (1.62x), geometric 2.51 vs 1.98 (1.27x); Method B everywhere
1.72 (1.18x) and 2.27 (1.15x), Method A everywhere 7.43. The excess over B is Method A near the poles
(~4.3x the cost of B on ~12 % of the evaluations: two FastAtan2, a square root, a division and a 3x3
rotation per texel); normalized wrapped texture coordinates cost 2 % (patch-walk micro-benchmark), the
rest is the per-hypothesis model math and register spills (178/420 B against none for pinhole, at the
same 128 registers). At 3840x1920 against 960x960 faces the panoramas cost 1.91 vs 2.73 ms (0.70x)
photometric and 2.30 vs 4.95 (0.47x) geometric: the larger faces sample their neighbors less locally.
At real panorama sizes the spherical kernels are therefore cheaper per pixel than the cube faces, and
the 1.62x at 1024 px is a small-image effect; the default threshold stays at 70° rather than trading
pole accuracy for it (B80: 1.32x photometric at 1024 px, see section 5).

The pinhole estimator keeps its cost: fountain-P11 CPU depth-maps + fusion (default 24 threads,
P-core pinned, timing-exclusive machine, one discarded warm-up run, min of 3 rotating runs):
200.2 s against 199.4 s for the estimator before the camera models (+0.4 %, 3054 / 3046 CPU-s);
with per-pixel waits instead of whole-estimate reads it was 208.6 s (+4.6 %, 3163 CPU-s). The
corner waits and the model-dispatched fusion leave it there: 202.1 s against 201.4 s for the
estimator without them in the same session (3080 / 3065 CPU-s; depth-maps 195.9 / 195.3 s, fusion
4.8 / 4.6 s).

### Fused clouds

`Tests sphfuse` (synthetic, the GT ray-cast from the panorama centers at 4x the width, at most
4096) and `Tests evalcloud` (Holo360D, the GT range maps of the frames, masked pixels excluded on
both sides), CPU, dense fusion, default options. Precision: the share of the cloud within th x
range of a GT point; recall: the share of the GT points on the grid of the estimated images within
th x range of a cloud point; the range and the band from the nearest panorama; seam band = the
longitudes 16 pixels from the seam.

| case | points | P / R / F @1% | P / R / F @2% | per band |
|---|---|---|---|---|
| synthetic 3 panoramas 1536x768 | 784178 | 0.999 / 0.952 / 0.975 | 1.000 / 0.982 / 0.991 | F@1% equator 0.963, north 1.000, south 0.964, seam 0.973 |
| their 18 cube faces 384x384 | 592042 | 0.997 / 0.821 / 0.901 | 1.000 / 0.902 / 0.948 | F@1% equator 0.939, north 1.000, south 0.715, seam 0.923 |
| synthetic 3 panoramas 512x256 | 81705 | 0.979 / 0.685 / 0.806 | 0.997 / 0.874 / 0.931 | F@1% equator 0.714, north 0.981, south 0.646, seam 0.802 |
| their 18 cube faces 128x128 | 43452 | 0.973 / 0.474 / 0.637 | 0.996 / 0.618 / 0.757 | F@1% equator 0.618, north 0.980, south 0.227, seam 0.651 |
| the panoramas and faces fused together (mixed scene) | 112856 | 0.984 / 0.793 / 0.878 | 0.998 / 0.930 / 0.963 | |
| panoramas 512, FuseDepthMaps / MergeDepthMaps | 96059 / 375841 | F 0.838 / 0.943 | F 0.967 / 0.987 | merge keeps every depth, so its recall is the depth-map completeness |
| panoramas 512 without sparse points (virtual cloud) | 80975 | 0.984 / 0.684 / 0.807 | 0.997 / 0.871 / 0.930 | depth-maps: completeness 0.956, <5% 0.984 |

Holo360D, 40 frames at 1440 px, the GT of the 2048x1024 range maps (thresholds 2 / 5 / 10 %):

| case | points | P / R / F @2% | P / R / F @5% | P / R / F @10% | F@5% per band |
|---|---|---|---|---|---|
| 40 panoramas, view gate (2 runs) | 324781–330978 | 0.792–0.797 / 0.208–0.212 / 0.330–0.335 | 0.908–0.916 / 0.360–0.365 / 0.516–0.521 | 0.968–0.973 / 0.504–0.509 / 0.663–0.668 | equator 0.396–0.403, north 0.898–0.904, south 0.279–0.295, seam 0.517–0.519 |
| their 240 cube faces 416x416 | 287846 | 0.760 / 0.194 / 0.310 | 0.932 / 0.401 / 0.561 | 0.971 / 0.596 / 0.738 | equator 0.434, north 0.859, south 0.489, seam 0.573 |

The panoramas give the more precise cloud, the faces the higher recall at the loose thresholds,
most of it on the ground below the handheld camera (south band) and the far equator.

Holo360D, its first 10 frames at 720 px (thresholds 2 / 5 / 10 %, F; precision @5 %):

| case | F@2% | F@5% | F@10% | P@5% | depth abs-rel median / <5% |
|---|---|---|---|---|---|
| panoramas, no view gate (3 runs) | 0.204–0.214 | 0.356–0.364, mean 0.360 | 0.471–0.474 | 0.850–0.863 | 0.299–0.333 / 0.260–0.277 |
| panoramas, view gate (5 runs) | 0.222–0.232 | 0.367–0.379, mean 0.373 | 0.473–0.490 | 0.860–0.901, mean 0.881 | 0.276–0.326 / 0.257–0.273 |
| 60 cube faces 208x208 | 0.157 | 0.330 | 0.472 | 0.901 | |

The seam band of Holo360D has a lower recall than the equator for the panoramas and the cube faces
alike: it looks behind the camera, at the operator (masked) and the path walked.

## 5. Rejected alternatives

- Affine warp (M with its projective row frozen at the patch center): 1–3.6 px median texel error.
- Method B everywhere: 1/cosφ error at the poles (south band 0.00064 vs 0.00042 with A).
- Method A everywhere: ~3x the cost of B.
- Exact warp only above 80° (B80): at 512x256 the pole bands lose 20-35 % abs-rel (CUDA north 0.00090 vs
  0.00069, south 0.00129 vs 0.00096; CPU north 0.00091 vs 0.00075, south 0.00134 vs 0.00107) for 1.32x
  instead of 1.62x the CUDA cube-face cost at 1024 px; on Holo360D (10 frames at 720) it is within the
  GT/pose noise (CUDA all 0.719 / 0.0571 vs 0.718 / 0.0572, CPU 0.960 / 0.300 vs 0.958 / 0.305).
- Padding the equirectangular images with wrapped columns: patches off the seam already take the
  plain pinhole loop.
- std::atan2 in the warps: the exact warp spends most of its time there.
- Pixels waiting for all their preceding neighbors (reads exactly as a single thread, the first race
  fix): 5–8 % multi-threaded wall on fountain-P11 with the same CPU work, idle cores while waiting;
  zigzag strides T·8 to T·64 and sleeping instead of yielding change nothing.
- Setting aside a pixel whose preceding neighbors are in progress and estimating it later: +40 %.
- Eigen fixed-size expressions in the per-hypothesis warp code: not inlined by MSVC, ~10 ns per 3x3
  product.
- Pole stride (~1/cos(elevation) columns estimated per row, the others filled from them after every
  pass): synthetic 512 px, 3 panoramas: south band <5% 0.98 -> 0.44, north 0.999 -> 0.70, for ~30 %
  fewer estimated pixels; the filled pixels starve the horizontal propagation.
- Isotropic angular patch (the patch columns stepped by round(1/cos(elevation)), at most 8):
  synthetic south completeness 0.797 -> 0.851, fused F@1% +0.004 (within the run-to-run noise of
  0.003); Holo360D fused precision@2% 0.672 -> 0.631 alone, neutral with the view gate.
- Filling the rows next to the poles a patch does not fit (the plane of the nearest estimated row
  along their rays): synthetic completeness north 0.857 -> 1.0, south 0.797 -> 0.933 (the 4 rows
  are 14 % of the pole band's pixels, 0.06 % of the sphere), fused F@1% +0.002; Holo360D neutral.
- An upper triangulation-angle gate (65°) beside the lower one: Holo360D depth abs-rel median
  0.28 -> 0.74.
- Dropping the per-view angle filter of `FilterNeighborViews` for panoramas: no change (Holo360D
  F@2% 0.232 / 0.229 vs 0.228; synthetic F@1% 0.809 vs 0.807): with the per-pixel gate it filters
  nothing that matters.
- View gate variants (synthetic 512: wrong estimates among the 1983 pixels whose true point every
  neighbor sees below 3°; Holo360D 10 frames at 720 px: F@5%):
  - an all-gated hypothesis scoring `thRobust` (no step 2): 1518–1540 wrong, median 24 % too
    close; F@5% mean 0.375 (4 runs), 40 frames at 1440 px 0.519: the same as the current rule
    within the noise;
  - gated views left out of the aggregation, an all-gated hypothesis the maximum cost: the same;
  - one view set per pixel and pass, gated at the depth the pixel starts the pass with (all passes,
    or from the second on): 370–410 wrong, F@5% 0.349–0.360 (all) and 0.360–0.370 (second on);
  - dropping the estimates every view sees below fMinAngle at the end of each pixel: only the ones
    at a depth that passes the gate survive, 1083–1113 wrong with the current rule (F@5%
    0.367–0.380), 434–511 with a view set per pixel and pass (F@5% 0.359–0.381).

## 6. Open items

- Per-hypothesis cost ~1.24x pinhole against the 1.2x target (section 4): what remains is the
  projection of the patch center (two FastAtan2, one square root, the reciprocal of ρ) and the
  per-pixel ray frames, amortized over few hypotheses in `scorebench`.
- Metal: not yet run on Apple hardware; still to be done there: `MetalModelMathTest`, the parity with
  the CPU and CUDA estimators (synthetic panoramas per band, Holo360D) and the cost per pixel.
- CUDA: an equirectangular reference needs an even width at every pyramid level (ASSERT); the
  rows are clamped at the poles where the CPU rejects the view; mixed pinhole/spherical scenes run
  the `MODEL_PINHOLE_REF` kernels, untested (Metal: the same).
- The confidence recalibration (`ConfidenceRefine.h`, `ConfidenceCUDA.cu`) is pinhole only and
  stays off for scenes with spherical images (`AdjustConfidenceSweep` ASSERTs a pinhole reference).
- The view gate still ranks hypotheses that leave a pixel different views: a closer depth that
  some views see above fMinAngle is scored by those alone, a farther one that no view sees above
  it by all of them. On the synthetic scene and Holo360D that costs nothing measurable; a soft
  weight by the triangulation angle is untested.
- The corner waits of the pixel order cover up to 64 concurrent estimators; with more threads the
  corner anti-diagonals between 64 and the thread count are read from the previous pass.
- `GapInterpolation` does not cross the poles; the rows next to the poles stay unestimated.
