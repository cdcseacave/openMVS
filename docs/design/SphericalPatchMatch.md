# Spherical PatchMatch (CPU)

## 1. Purpose and scope

`DensifyPointCloud` estimates the depth-maps of equirectangular (360x180 degree) images natively,
without rendering them to cube faces: the CPU PatchMatch takes the reference camera model as a
template parameter (`DepthEstimatorT<Pinhole>`, `DepthEstimatorT<Spherical>`, `libs/MVS/DepthMap.h`)
and resolves the camera model of each neighbor view once per (pixel, hypothesis, view). Pinhole,
Brown (undistorted on load, see `SphericalCameraSupport.md`) and spherical views can be mixed.

Scope: depth-maps only (`--fusion-mode 1`) on the CPU (`--gpu-device -2`), with the photometric
confidence. `Scene::DenseReconstruction` refuses a scene with spherical images otherwise, with a
clear error: the fusion, SGM, the CUDA/Metal estimators and a scene without sparse points; the
confidence recalibration is switched off for it.

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
- The spherical projections of both warps use `FastAtan2` (a minimax polynomial, max error
  1.3e-7 rad, the precision of the float atan2; a scalar and a packet version) and `FastSqrt` (the
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
  pass's; never a mix of the two. Nobody waits. A single thread reads its lower neighbors final and
  the higher ones from the previous pass; with several threads a lower neighbor still in progress
  is read from the previous pass. The first and the last column of a spherical reference are
  neighbors like any other two.

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

### Depth-map files

The `.dmap` header carries the flag bit `SPHERICAL` (`HeaderDepthDataRaw`, `Interface.h`,
`MvsUtils.py`): `DepthData::Save` sets it from the reference camera model; `DepthData::Load` and the
depth-map loads of `InitViews` reject a file whose flag disagrees with the view's model.

## 3. Parameters

| parameter | default | meaning |
|---|---|---|
| `DepthEstimatorBase::fSphericalWarpMaxElevation` | 70 deg | target elevation above which Method A replaces Method B |

All other PatchMatch options apply unchanged.

## 4. Validation

Synthetic room (`Tests sphtest`, `Tests scorebench`, `TestSphericalPatchWarp`; CPU, RelWithDebInfo),
Holo360D Outdoor_004 (40 frames, `--max-resolution 1440`, `Tests evaldepth`):

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

The pole bands (|elevation| > 70°) lose the 4 rows next to each pole and the floor below the
synthetic cameras; their accuracy matches the equator's. Holo360D is a handheld capture whose
LiDAR-derived GT and poses agree with the images to a few pixels only (a GT-predicted
correspondence lands within 1 px of the image match in 12–48 % of the cases at 2880 px): both the
spherical and the cube-face estimates are bounded by it alike.

Patch warp, B against A over random poses and slanted planes (2880x1440, 9x9 patch step 2):
per-patch max texel error p99 0.042 / 0.079 / 0.140 / 0.757 px for target elevations below
30 / 50 / 70 / 90°, seam-crossing patches max 0.23 px; the affine warp: median 1.0–3.6 px.

Cost (`scorebench`, one ScorePixel against one view, all texels seen, the pixel prepared once per 4
hypotheses; min of 3x5 alternating runs, timing-exclusive machine): panorama 1536x768 B70 158 ns,
B only 155 ns, A only 225 ns; cube faces of the same image size 126–129 ns: B70 1.22–1.25x, B only
~1.21x, A only ~1.8x the pinhole cost. Whole runs (`sphtest 1536`, 24 threads, B70): the 3 panoramas
take 9.6 s, 2.71 µs per GT pixel, their 18 cube faces 384x384 7.4 s, 2.80 µs per GT pixel (the
panoramas hold 1.33x the pixels of their faces).

The pinhole estimator keeps its cost: fountain-P11 CPU depth-maps + fusion (default 24 threads,
P-core pinned, timing-exclusive machine, one discarded warm-up run, min of 3 rotating runs):
200.2 s against 199.4 s for the estimator before the camera models (+0.4 %, 3054 / 3046 CPU-s);
with per-pixel waits instead of whole-estimate reads it was 208.6 s (+4.6 %, 3163 CPU-s).

## 5. Rejected alternatives

- Affine warp (M with its projective row frozen at the patch center): 1–3.6 px median texel error.
- Method B everywhere: 1/cosφ error at the poles (south band 0.00064 vs 0.00042 with A).
- Method A everywhere: ~3x the cost of B.
- Padding the equirectangular images with wrapped columns: patches off the seam already take the
  plain pinhole loop.
- std::atan2 in the warps: the exact warp spends most of its time there.
- Pixels waiting for their preceding neighbors (reads exactly as a single thread, the first race
  fix): 5–8 % multi-threaded wall on fountain-P11 with the same CPU work, idle cores while waiting;
  zigzag strides T·8 to T·64 and sleeping instead of yielding change nothing.
- Setting aside a pixel whose preceding neighbors are in progress and estimating it later: +40 %.
- Eigen fixed-size expressions in the per-hypothesis warp code: not inlined by MSVC, ~10 ns per 3x3
  product.

## 6. Open items

- Per-hypothesis cost ~1.24x pinhole against the 1.2x target (section 4): what remains is the
  projection of the patch center (two FastAtan2, one square root, the reciprocal of ρ) and the
  per-pixel ray frames, amortized over few hypotheses in `scorebench`.
- Seam/pole-aware propagation and red/black parity on CUDA (P6/P7), per-pixel view weighting by
  the ray-to-ray triangulation angle (P6).
- Fusion, the depth-map filters' seam wrap, the confidence recalibration (P6).
- CUDA (P7) and Metal (P8) ports of `RayFrame`/`PixelHomography` and the wrapped sampling.
