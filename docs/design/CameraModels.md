# Camera Models

## 1. Purpose and scope

`libs/Math/Camera/` is the one implementation of the camera projection math of OpenMVS, shared by
the SfM library, bundle adjustment, the Metashape importer, the MVS library and the PatchMatch
backends. It holds three models (pinhole, Brown-Conrady distortion, equirectangular spherical), the
resampling between them (undistortion maps, depth-map undistortion, equirectangular to tangent
faces). The scene-file camera description is in `libs/IO/Interface.h` (version 8).

It does not hold the per-library camera classes: `SFM::PinholeCamera` / `SFM::SphericalCamera`
(`libs/SFM/Camera.h`) and `MVS::Camera` / `MVS::Image` (`libs/MVS/Camera.h`, `Image.h`) keep their
own state and call the models. The depth-map algorithms built on the models are described in
`SphericalPatchMatch.md`; the SfM side of spherical cameras and the MVS scene loading in
`SphericalCameraSupport.md`.

## 2. Algorithm as implemented

### Conventions (`libs/Math/Camera/CameraModels.h`)

- Pixel centers lie at integer coordinates (OpenCV/MVS): pixel (0,0) covers `[-0.5,0.5]^2`.
- Camera frame x-right, y-down, z-forward, for every model.
- `Ray(x)` is the camera-space direction of pixel `x`; `Project(X, x)` is its inverse and returns
  whether `X` is visible; `Depth(X)` is the scale of `Ray(x)` that reaches `X`:
  - `Pinhole`: `Ray` lies on the z=1 plane, so the depth is `z`;
  - `Spherical`: `Ray` is a unit vector, so the depth is the range `|X|`.
- `RayFrame(x)` maps pixel offsets `(dx,dy,1)` around `x` to rays (columns: the ray derivatives
  along x and y, then `Ray(x)`); `PixelHomography(r)` maps rays near `r` to homogeneous pixels. The
  plane-induced patch warp between two views is composed from these two (`SphericalPatchMatch.md`).
  Both are exact for `Pinhole` (`RayFrame` is `[1/fx 0 r.x; 0 1/fy r.y; 0 0 1]`,
  `PixelHomography` is `K`) and first order for `Spherical`.

### Models

- `Pinhole<T>{fx,fy,cx,cy}`: skew-free; `PixelToNormalized`, `NormalizedToPixel`, `Perspective`,
  `Ray`, `Project` (visible iff `z>0`), `ProjectFast` (= `Project`), `Depth`, `RayFrame`,
  `PixelHomography`.
- `BrownDistortion<T>{k1,k2,p1,p2,k3,k4,k5,k6}`: OpenCV coefficient order and meaning, on normalized
  image-plane coordinates. `DistortBrown` always evaluates the rational radial form
  `(1+k1 r2+k2 r4+k3 r6)/(1+k4 r2+k5 r4+k6 r6)` plus the tangential terms; the plain polynomial model
  is the case `k4=k5=k6=0`. `Jacobian(p)` is the analytic 2x2 Jacobian of `Distort`.
  `Undistort(p)` is Newton's method from `p` on `Distort(u)-p`, stopped when both coordinates move by
  less than 1e-12 or after 20 steps; it `ASSERT`s a positive Jacobian determinant at every step (the
  distortion must be invertible between `p` and the solution).
- `Spherical<T>{fx,fy,cx,cy}`, equirectangular 360x180 degrees: `FromSize(W,H)` sets `fx=W/(2*pi)`,
  `fy=H/pi`, `cx=(W-1)/2`, `cy=(H-1)/2`. Longitude `atan2(x,z)` grows with the column and is 0 at
  `cx` (forward +Z); elevation `asin(-y/|X|)` is `pi/2` on the top row (up = -Y, an upright
  camera's sky is at the top), so the frame is the pinhole frame and no mirror is involved.
  `Ray(x) = (cos(el) sin(lon), -sin(el), cos(el) cos(lon))`; `Project` computes
  `fy*atan2(y, hypot(x,z)) + cy`. Overloads take the sines and cosines of the pixel's longitude and
  elevation (they depend on the column and the row only, so callers tabulate them): `Ray`, `RayFrame`.
  `PixelHomography(ray[, x[, rho]])` is the gnomonic projection onto the plane tangent at the ray
  scaled by the equirectangular Jacobian; exact at the ray, error growing as `1/cos(el)` towards the
  poles; `ASSERT(rho > 0)`.
- `FastAtan2` (single precision, minimax polynomial, max error 1.3e-7 rad) backs
  `Spherical::ProjectFast`, the entry of the PatchMatch hot loops, on the CPU, in the SIMD packet path
  of `DepthMap.cpp` (same polynomial lane by lane) and on the GPU.
- `CameraModelType` (`PINHOLE=0, BROWN=1, SPHERICAL=2`) is the id stored in scene files; its values
  equal `Interface::Platform::Camera::Model` (`static_assert`s in `libs/MVS/Scene.cpp`).

### Sharing: scalar templating, CUDA, Metal

- Every model and `DistortBrown` is a template on the scalar type and written with Eigen fixed-size
  types, so the same code runs on `double`/`float`, on `ceres::Jet` (bundle adjustment cost functors,
  `libs/SFM/BundleAdjustmentCostFunctions.h`) and in CUDA. The including context provides Eigen,
  `ASSERT` and `HOST_DEVICE` (`libs/Common/Config.h`, or `libs/MVS/CUDA/Maths.h` in `.cu` files).
- The numeric constants of the code Ceres differentiates (`DistortBrown`, `Pinhole::Perspective`) are
  written in `ScalarOf<T>::type`, the plain scalar of `T` (the value type of a Jet): a constant Jet would
  turn each `1 + ...` or `2 * ...` into a full Jet operation. These two helpers are force-inlined.
- `HOST_DEVICE` functions are included unchanged by CUDA code (`libs/MVS/CUDA/Camera.h` includes
  `CameraModels.h`; `LinearCameraModel::AsPinhole()` yields the shared model), so the host and the
  device evaluate the same source. The depth geometry written over the models is
  `libs/Common/DepthGeometry.h`.
- Metal Shading Language cannot include Eigen: `libs/MVS/PatchMatchMetal.metal` carries one delimited
  hand mirror of `CameraModels.h` and `DepthGeometry.h`, covered by a unit test that evaluates the
  mirror against the C++ models (`SphericalPatchMatch.md`, Metal estimator).

### Resampling (`libs/Math/Camera/CameraUndistort.h/.cpp`)

- `UndistortedK(K, distortion, size, alpha=UNDISTORT_ALPHA)`: `cv::getOptimalNewCameraMatrix` with free
  scaling `alpha` (0.6 by default), same image size. The one rule shared by the SfM export, the MVS
  loader and the depth-map undistortion; deterministic for given inputs.
- `UndistortMaps` (`Init`, `Remap`, `RemapDepth`): fixed-point remap tables from the undistorted image
  to the distorted source; `RemapDepth` interpolates only depths similar to the nearest one
  (`IsDepthSimilar`) and writes 0 where no valid source depth exists.
- `CameraUndistortMaps` adds the mask of the undistorted pixels with a source (255 where the bilinear
  footprint of the source position lies inside the source image), and `GetCameraUndistortMaps(K,
  distortion, size)` returns them from a process-wide, thread-safe cache keyed by `K`, the 8
  coefficients and the source size (concurrent requests for one camera wait for one computation).
  Beyond a fixed budget of 1 GiB the least recently requested entries are dropped from the cache
  (holders keep theirs alive).
- `SphereCubeMap`: `MakeTangentFacesGeometry(numFaces, faceSize)` (4, 6, 8, 12 or
  20 faces, `IsSupportedNumFaces`) and `SphericalToTangentialFaces` (bilinear, wraps across the seam;
  `Pixel8U`, `Pixel32F`), used by SfM feature extraction and the cube-map export.

### Scene-file camera, Interface version 8 (`libs/IO/Interface.h`, `MVSI_PROJECT_VER 8`)

`Interface.h` stays a drop-in header (plain data, no dependency on the libraries).

- `Platform::Camera` has `uint8_t model` (`PINHOLE`, `BROWN`, `SPHERICAL`) and `params`: for `BROWN`,
  5 (`k1,k2,p1,p2,k3`) or 8 (`+k4,k5,k6`) coefficients in OpenCV order. Serialized as
  `if (version > 7) { ar & model; if (model == BROWN) ar & params; }`: one byte per camera, the
  coefficients only for Brown. The `SPHERICAL` `K` is the angular camera matrix of
  `Spherical::FromSize`, so normalization, resolution scaling and the `.dmap` `K` work as for a
  pinhole `K`.
- `Vertex::View::x` is the optional observation: the feature position in the image file as it is on disk
  (distorted for a Brown camera), pixel-center convention. `Interface::flags` (`HAS_OBSERVATIONS`) is
  written before the vertices; `View::serialize` reads or writes `x` only when the archive carries the
  flag (`ar.flags`), so an unset observation costs 0 bytes. The flag is set by the producer.
- `.dmap` header flag `HeaderDepthDataRaw::SPHERICAL` marks depth-maps of an equirectangular view.
- v7 files load as `PINHOLE`; v8 files are rejected by older readers.

### Users of the library

- SfM: `PinholeCamera::GetModel()/GetDistortion()` and `SphericalCamera::GetModel()` (`libs/SFM/Camera.h`)
  make `Project`, `Unproject`, `Distort`, `Undistort` thin wrappers (`libs/SFM/Camera.cpp`).
- Bundle adjustment: `libs/SFM/BundleAdjustmentCostFunctions.h` (`ProjectPinhole`) evaluates
  `Pinhole<T>::Perspective` and `DistortBrown` on Jets and applies `fx, fy, cx, cy` from the parameter
  block in place (no `Pinhole` of Jets is built); the spherical angular residual takes its tangent basis
  from `Spherical<double>`.
- `SFM::Scene::UndistortImages` and `SFM::UndistortDepthMaps` (`libs/SFM/Scene.cpp`, `InterfaceMVS.cpp`)
  use `UndistortMaps`; SfM feature extraction uses `SphereCubeMap`.
- Importers pass the distortion through, with the feature observations of the sparse points:
  `InterfaceCOLMAP` maps SIMPLE_RADIAL, RADIAL, OPENCV and FULL_OPENCV to BROWN (the fisheye, FOV and
  thin-prism models are refused) and reads the observations from `images.bin/txt`; on export it writes a
  Brown camera as `OPENCV`/`FULL_OPENCV` and the stored observations, else the projections through the
  distortion (always the projections for a cloud replaced by `--pointcloud-file`), and fails on a camera
  COLMAP can not hold. `InterfaceMetashape` makes the distorted Metashape and BlocksExchange sensors BROWN through
  `Platform::Camera::SetBrownLens` (their `p1`/`p2` are OpenCV's `p2`/`p1`, `b1`/`b2` go into `fx` and the
  skew, the Agisoft r^8 `k4` is refused; the cameras XML `cx`/`cy` are offsets from the image center with
  the pixel centers at +0.5), makes the spherical sensors SPHERICAL (Metashape projects them exactly as
  `Spherical`; a BlocksExchange spherical photogroup has no `CameraModelType` and the angular focal
  length), and reads the BlocksExchange tie-point measurements as observations, mapped into the camera
  frame by `ObservationMap` (`libs/MVS/Image.h`).
- SfM export (`SFM::ExportMVS`, `libs/SFM/InterfaceMVS.cpp`) writes Brown cameras natively with the
  original image paths and no undistortion, unless `CreateStructure --undistort-extension` is set;
  spherical cameras are written natively (`--spherical-export native`, default) or through the
  cube-map bridge (`--spherical-export cubemap`). Details: `SphericalCameraSupport.md`.
- MVS: `Scene::LoadInterface` (`libs/MVS/Scene.cpp`) keeps `model` and the Brown lens in
  `MVS::Platform::Camera`; for `BROWN` the camera `K` every consumer sees is
  `UndistortedK(lens.K, lens.distortion, lens.size)` (alpha 0.6). `Image::ReadPixels` remaps the image
  through `GetCameraUndistortMaps` before any resize, `Image::LoadMask` remaps a mask with nearest
  sampling, and `DepthEstimatorBase::ImportIgnoreMask` ANDs the validity mask into the ignore mask
  (CPU, CUDA and Metal densify). Observations are undistorted into `K'` on load and distorted back on
  save, and the camera is saved as it was read. A spherical camera is loaded only with the angular `K`
  of its image size. `Scene::LoadDMAP` gives a standalone `.dmap` the camera model of its `SPHERICAL` flag.
- The native boost archive carries the observations (`PointCloud::pointObservations`) and the model and
  lens of `Platform::Camera` too. Only the interface format is backward compatible: `PROJECT_VER`
  (`Scene.cpp`) names the native layout, and an archive of another version is refused.
- The observations follow their points and views: removed with them, subset by `Scene::SubScene`, and
  released as a whole where points without features join the cloud (the tower points of
  `Scene::InitTowerScene`).
- PatchMatch: `MakeCameraModel<MODEL>(camera)` (`libs/MVS/DepthMap.h`) builds `Pinhole`/`Spherical`
  from a view's `K`; `ModelCamera` (`libs/MVS/Image.h`) dispatches the pixel/world transforms of fusion
  and the depth-map consumers on the image model (`SphericalPatchMatch.md`).

## 3. Parameters and defaults

| Parameter | Location | Default |
|---|---|---|
| `UNDISTORT_ALPHA` | `CameraUndistort.h` | 0.6, fixed (not an option) |
| cache budget | `CameraUndistort.cpp` (`CameraUndistortCache::budget`) | 1 GiB (~7 bytes per cached pixel) |
| `Undistort` iterations / step tolerance | `BrownDistortion::Undistort` | 20 / 1e-12 |
| `--spherical-export` | `CreateStructure` | `native` (`cubemap` for the bridge) |
| `--undistort-extension` | `CreateStructure` | empty: Brown cameras exported natively |

## 4. Invariants

- Pixel centers at integer coordinates in every model; `Spherical::FromSize` requires positive sizes.
- `Pinhole` rays are on z=1 (depth = z); `Spherical` rays are unit (depth = range); `PixelHomography`
  of `Spherical` requires `rho > 0`; the `rho` given to `ProjectFast` is `sqrt(x^2+z^2)` of the same
  point up to rounding (relative 1e-5: compilers that contract `a*a+b*b` into an fma, clang, GCC and
  nvcc, round the caller's and the check's expression differently, MSVC does not).
- `Undistort` requires an invertible distortion between the distorted point and the solution.
- `MakeCameraModel` requires zero skew.
- `K'` of a Brown camera depends only on its lens and resolution.
- A Brown camera in a v8 scene carries its resolution and 5 or 8 coefficients (`LoadInterface` refuses
  the scene otherwise); a spherical `K` is the angular `K` of its image size.

## 5. Validation

| Check | Result |
|---|---|
| `Undistort(Distort(p))` residual (`TestCameraModels`, `apps/Tests/TestsMath.cpp`) | 3e-16 |
| `TestCameraModels`: `Ray`/`Project` round trip, seam, poles, Jets | `Ray`∘`Project` ≤1e-8 px; axis/seam/pole conventions exact to 1e-9 px; Jet values 1e-9 px, derivatives 1e-5 relative |
| Remap cost on load (`Tests undistortbench`, ETH3D 6048x4032 JPEG) | 10.5-10.8 ms per image vs 108-121 ms decode; 34-43 ms once per camera for the maps |
| ETH3D F1@2cm, native Brown vs undistorted-on-export (pipes / office ICP-refined) | 0.7068 vs 0.7043 / 0.3106 vs 0.3104 |
| v7 to v8 file size | + one byte per camera + 1 |
| SfM reconstruct-only on 12 scenes against the pre-model code (same matches) | identical registrations, points within 0.02 % (CasaBatllo within its run-to-run spread), equal reprojection errors, equal BA cost and time |
| Spherical SfM from video against the pre-model code (`ExtractKeyframes` + `CreateStructure`, 91 keyframes) | 91/91 registered, 977k vs 981k points, reprojection 1.03 vs 1.01 px |

## 6. Rejected alternatives

- Undistorting once into an on-disk cache folder: every app re-derives the same pixels from the maps,
  no disk state.
- `K' = K` instead of `getOptimalNewCameraMatrix(alpha)`: the SfM export, the loader and the depth-map undistortion share one rule (alpha 0.6).
- A flat side array of observations: costs a uint64 when unset and is awkward to index.
- The spherical axis flip (Y-up): the Y-down frame is identical to the pinhole frame and mirror-free.

## 7. Open items

- Spherical keypoints in `.sfm` projects extracted with the older half-pixel convention sit half a pixel off in
  latitude in the current model (a spherical camera has no principal point to absorb it): such a project
  reconstructs with a higher reprojection error and should be re-extracted.
- InterfacePolycam still undistorts on its own instead of passing the distortion through.
- Metal runs the hand mirror on Apple hardware only; it is checked there by `MetalModelMathTest`.
- Nothing bounds the Brown coefficients in the SfM bundle adjustment (no parameter bounds, no check
  after it), so BA can converge to a distortion that folds inside the image, typically in the corners
  with few observations; `BrownDistortion::Undistort` (Newton) ASSERTs a positive Jacobian
  determinant and does not handle such a fold.
