# Spherical PatchMatch

## 1. Purpose and scope

`DensifyPointCloud` estimates the depth-maps of equirectangular (360x180 degree) images natively,
without rendering them to cube faces: the CPU PatchMatch takes the reference camera model as a
template parameter (`DepthEstimatorT<Pinhole>`, `DepthEstimatorT<Spherical>`, `libs/MVS/DepthMap.h`),
the CUDA PatchMatch as a kernel template argument (`libs/MVS/PatchMatchCUDA.cu`, "CUDA estimator" below), and
both resolve the camera model of each neighbor view once per (pixel, hypothesis, view). Pinhole,
Brown (undistorted on load, see `SphericalCameraSupport.md`) and spherical views can be mixed.

Scope: depth-maps only (`--fusion-mode 1`) on the CPU or CUDA, with the photometric confidence.
`Scene::DenseReconstruction` refuses a scene with spherical images otherwise, with a clear error: the
fusion, SGM, the Metal estimator and a scene without sparse points; the confidence recalibration is
switched off for it.

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
  out-of-image texel does.
- The spherical projections of both warps use `Spherical::ProjectFast`, through `FastAtan2`
  (`CameraModels.h`, a minimax polynomial, max error 1.3e-7 rad, the precision of the float atan2).

The error of B grows as 1/cosφ of the target elevation of the patch center, not of the reference
one. The choice between B and A is one branch per (pixel, hypothesis, view); there is no per-texel
branch on the model.

### Sampling and the seam

- Spherical target images: the columns wrap (bilinear across the seam), the rows keep the 1-pixel
  border (`SampleTexel<true>`); no padding. Target depth-maps of the geometric consistency wrap the
  same way (`SampleDepth<true>`).
- Spherical reference: patches are estimated across the seam (only the 4 rows next to each pole
  are skipped); the reference patch and the propagation neighbors wrap their columns (`Col0`).
- Pixel order: estimators pull pixels in the zigzag order and a pixel waits for its 4-neighbors
  that precede it. The first and the last column are neighbors: in each pass the one visited second
  waits for the other, so the first reads the last as the previous pass left it and the last reads
  the first finished, as a single thread would; every wait points backwards in the order, so there
  is no deadlock.

### Geometric consistency

`GeometricConsistency<TRG>`: the reference point moved into the frame of the target depth-map,
projected, the target depth sampled (similar depths only), the target point at that depth
(`X1·depth/Depth(X1)`, along the same ray) projected back; the pixel distance to x0, its columns
wrapped for a spherical reference, mapped to [0,4]. The pinhole pair keeps its cached form.

### Normals

Random and perturbed normals are drawn as directions (`Dir2Normal`) in a frame whose z axis is the
pixel ray: the camera frame for pinhole (unchanged), the frame (e_lon, e_down, ray) of the pixel
for spherical (`normalFrame0`), where the polar angle of the turned normal is taken by atan2 (the
turned vector is unit only to the rounding). `CorrectNormal` is ray-based. `EstimateNormalMap` of a
spherical map fits the depth gradient with wrapped columns.

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
- Normals: random and perturbed normals are isotropic, made camera-facing against the pixel ray
  F.col(2), so they are the same in the ray's local frame; the surface normal and the plane
  propagation use the generic `NormalFromDepthGradient`/`InterpolatePlaneDepth`.
- Geometric consistency (`GeometricConsistencyWeightModel`): the CPU round trip through both
  models, the column distance wrapped for a spherical reference.
- The model kernels inline the whole scoring path (`__forceinline__`): under `-rdc` a callee is
  compiled with its own register count and must fit the 128 registers of the launch bounds.
- Pinhole scenes run the unchanged kernels: their SASS is identical but for the stack layout of the
  local arrays of the two geometric-consistency kernels, and the depth-maps are byte-identical.

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
| same, B + A above 70° (default) | 0.911 | 0.00050 | 0.983 | 0.995 | equator 0.00072, north 0.00030, south 0.00041, seam 0.00055 |
| same, B + A above 80° | 0.911 | 0.00051 | 0.982 | 0.994 | south 0.00049 |
| same, B only | 0.911 | 0.00053 | 0.982 | 0.994 | south 0.00064, seam 0.00062 |
| their 18 cube faces 384x384, pinhole | 0.913 | 0.00079 | 0.912 | 0.952 | border band completeness 0.69 |
| synthetic 512x256, A only / B70 | 0.956 / 0.956 | 0.00116 / 0.00145 | 0.926 / 0.920 | 0.984 / 0.985 | faces 128x128: 0.854, 0.00273, 0.756, 0.921 |
| Holo360D, B + A above 70° | 0.956 | 0.203 | 0.131 | 0.343 | equator 0.974 / 0.111 / 0.149 / 0.390, seam 0.951 / 0.208 / 0.136 / 0.333, south (the ground under the handheld camera, behind the operator in the neighbors) 0.945 / 1.94 |
| Holo360D, cube faces 416x416, pinhole | 0.774 | 0.130 | 0.126 | 0.374 | same pipeline on the rendered faces |
| CUDA: synthetic 512x256, B70 | 0.964 | 0.00141 | 0.932 | 0.989 | equator 0.991 / 0.00218, north 0.857 / 0.00069, south 0.849 / 0.00097, seam 0.969 / 0.00149 (CPU 0.956 / 0.00147 / 0.919 / 0.985); normal median 3.2° (CPU 4.0°); faces 128x128 on CUDA 0.949 |
| CUDA: Holo360D 10 frames at 720, B70 | 0.718 | 0.057 | 0.203 | 0.481 | CPU on the same: 0.960 / 0.295 / 0.088 / 0.268; the CPU's best 71.8 % by confidence 0.173 / 0.113 / 0.338 |

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

Patch warp, B against A over random poses and slanted planes (2880x1440, 9x9 patch step 2):
per-patch max texel error p99 0.042 / 0.079 / 0.140 / 0.757 px for target elevations below
30 / 50 / 70 / 90°, seam-crossing patches max 0.23 px; the affine warp: median 1.0–3.6 px.

Cost (`scorebench`, one ScorePixel against one view, all texels seen, min of 5 alternating runs):
panorama 1536x768 B70 ~250 ns, B only ~215 ns, A only ~750 ns; cube faces of the same image size
~150 ns: B70 1.6–1.7x, B only ~1.4x the pinhole cost. The difference is the per-hypothesis model
math (two atan2, a square root and the 3x3 products of the warp) and Method A near the poles.

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

The pinhole estimator keeps its cost: fountain-P11 CPU depth-maps + fusion, P-core pinned, min of N, 212.4 / 225.0 s against 211.9 / 224.3 s for the pinhole-only estimator it extends (two sessions; the pinhole-only scenes take the cached-homography loop without the model dispatch).

## 5. Rejected alternatives

- Affine warp (M with its projective row frozen at the patch center): 1–3.6 px median texel error.
- Method B everywhere: 1/cosφ error at the poles (south band 0.00064 vs 0.00042 with A).
- Method A everywhere: ~3x the cost of B.
- Exact warp only above 80° (B80): at 512x256 the pole bands lose 20-35 % abs-rel (CUDA north 0.00090 vs
  0.00069, south 0.00129 vs 0.00096; CPU north 0.00091 vs 0.00075, south 0.00134 vs 0.00107) for 1.32x
  instead of 1.62x the CUDA cube-face cost at 1024 px.
- Padding the equirectangular images with wrapped columns: patches off the seam already take the
  plain pinhole loop.
- std::atan2 in the warps: the exact warp spends most of its time there.

## 6. Open items

- CPU per-pixel cost above the 1.2x target (see section 4).
- Per-pixel view weighting by the ray-to-ray triangulation angle (P6).
- Fusion, the depth-map filters' seam wrap, the confidence recalibration (P6).
- Metal port (P8) of the model kernels, the wrapped sampling and the model math (hand mirror).
- CUDA: an equirectangular reference needs an even width at every pyramid level (ASSERT); the
  rows are clamped at the poles where the CPU rejects the view; mixed pinhole/spherical scenes run
  the `MODEL_PINHOLE_REF` kernels, untested.
