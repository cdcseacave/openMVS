# PoseLib Bearing Vector Estimators for Spherical Cameras

**Version:** 1.0
**Date:** April 2026
**Author:** SfM Pipeline Extension
**Status:** Design Phase

## Executive Summary

OpenMVS currently routes all pose estimation (both relative and absolute) through PoseLib's high-level `estimate_relative_pose()` and `estimate_absolute_pose()` functions, which take **2D normalized-plane coordinates** as input. For spherical (equirectangular / 360°) cameras, this approach silently loses hemisphere information—a bearing vector (unit-norm 3D direction) that points backward through the camera center is indistinguishable from one that points forward when projected onto a 2D plane.

**Key Finding:** PoseLib exposes bearing-vector solvers (`p3p`, `gen_relpose_5p1pt`, etc.) but **does not expose a high-level robust (RANSAC) entry point** that takes bearing vectors directly. The generalized-camera estimators (`estimate_generalized_relative_pose`, `estimate_generalized_absolute_pose`) take 2D points + rig extrinsics, not bearings.

**Recommendation:** **OPTION 2 (with caveats) → OPTION 3 (preferred).**

The cleanest path forward is to **extend PoseLib internally** with bearing-vector RANSAC wrappers that reuse the existing templated RANSAC machinery and low-level solvers. These wrappers would be added to OpenMVS's SFM library (not PoseLib itself, to avoid patching the vendored library), and they would call PoseLib's existing `ransac_relpose()` / `ransac_gen_pnp()` infrastructure with a bearing-vector adapter layer.

---

## Part 1: Reconnaissance of PoseLib API

### 1.1 High-Level Robust Estimators (robust.h)

All estimators follow the pattern:
```cpp
RansacStats estimate_*_pose(
    const std::vector<Point2D> &points2D_...,  // Input: 2D image coords
    ...,
    const Camera &camera,                       // Camera model (pinhole-based)
    const RansacOptions &ransac_opt,
    const BundleOptions &bundle_opt,
    CameraPose *pose,                           // Output: rotation + translation
    std::vector<char> *inliers);                // Output: per-point inlier mask
```

**Exposed entry points (from robust.h):**

| Function | Input Type | Output | Scoring |
|----------|-----------|--------|---------|
| `estimate_absolute_pose()` | `vector<Point2D>` 2D points, `vector<Point3D>` 3D points, `Camera` pinhole | `CameraPose` | reprojection error (2D pixel-space) |
| `estimate_generalized_absolute_pose()` | `vector<vector<Point2D>>` (per-camera), `vector<Point3D>`, `vector<CameraPose>` rig extrinsics | `CameraPose` | reprojection error (2D pixel-space) |
| `estimate_absolute_pose_pnpl()` | 2D/3D points + 2D/3D lines | `CameraPose` | point reprojection + line reprojection |
| `estimate_relative_pose()` | `vector<Point2D>` from two images, `Camera` for each | `CameraPose` relative | Sampson error (2D epipolar-space) |
| `estimate_generalized_relative_pose()` | `vector<PairwiseMatches>` (2D), `vector<CameraPose>` rig extrinsics | `CameraPose` | Sampson error (2D) |
| `estimate_shared_focal_relative_pose()` | 2D points, unknown focal length | `ImagePair` | Sampson error (2D) |
| `estimate_fundamental()` | 2D points (no camera model) | `Eigen::Matrix3d` F | Sampson error (2D) |
| `estimate_homography()` | 2D points | `Eigen::Matrix3d` H | transfer error (2D) |
| `estimate_hybrid_pose()` | 2D-3D + 2D-2D matches (hybrid) | `CameraPose` | mixed |
| `estimate_1D_radial_absolute_pose()` | 2D points (1D radial distortion) | `CameraPose` | radial reprojection error |

**Critical observation:** No entry point takes `std::vector<Eigen::Vector3d>` (unit bearing vectors) directly.

### 1.2 Generalized Camera Path (NOT applicable as-is)

`estimate_generalized_relative_pose()` signature:
```cpp
RansacStats estimate_generalized_relative_pose(
    const std::vector<PairwiseMatches> &matches,  // struct with 2D points x1, x2
    const std::vector<CameraPose> &camera1_ext,   // rig1 camera extrinsics
    const std::vector<Camera> &cameras1,           // rig1 camera models (PINHOLE)
    const std::vector<CameraPose> &camera2_ext,   // rig2 camera extrinsics
    const std::vector<Camera> &cameras2,           // rig2 camera models (PINHOLE)
    const RansacOptions &ransac_opt,
    const BundleOptions &bundle_opt,
    CameraPose *relative_pose,                     // Output: relative between rig origins
    std::vector<std::vector<char>> *inliers);      // Output: per-camera, per-point
```

**Why it doesn't directly solve our problem:**
- Input is still **2D points** (`PairwiseMatches::x1, x2` are `std::vector<Point2D>`), not bearing vectors.
- The rig extrinsics specify the **camera centers** in the rig; for a single spherical camera, we'd need to pass the same center multiple times (redundant and semantically wrong).
- Scoring is still in **2D pixel-space** (Sampson error on normalized coordinates).

**Can it be repurposed?** Theoretically, if we:
1. Convert each bearing vector to a 2D normalized image coordinate (lossy, loses hemisphere info).
2. Pass a single "camera" with extrinsic at the origin.
3. Use the `estimate_generalized_relative_pose` machinery.

This would work for pinhole but defeats the entire purpose for spherical cameras—we're back to losing hemisphere information.

### 1.3 Low-Level Minimal Solvers (solvers/)

PoseLib provides **minimal solvers** that take bearing vectors (unit-norm `Eigen::Vector3d`) directly:

| Solver | Input | Output | Rel/Abs | Notes |
|--------|-------|--------|---------|-------|
| `p3p()` | `vector<Vector3d>` bearing x, `vector<Vector3d>` 3D points X | `vector<CameraPose>` | Absolute | Revisiting P3P (Ding et al., CVPR 2023). Solves `λx = R*X + t`, λ > 0. **Bearing-native.** |
| `gen_relpose_5p1pt()` | `vector<Vector3d>` p1 (rig origins), `vector<Vector3d>` x1 (bearings), same for second rig | `vector<CameraPose>` | Relative | Generalized 5-point: first 5 correspondences from same camera pair, 6th from different camera. **Bearing-native.** |
| `relpose_5pt()` | `vector<Vector2d>` points (2D) | `vector<CameraPose>` | Relative | Classic 5-point essential matrix. Input is 2D, not bearing. |
| `relpose_6pt_focal()` | `vector<Vector2d>` points (2D) | `vector<CameraPose>` | Relative | 5-point + unknown focal length. Input is 2D. |
| `gp3p()` | `vector<Vector3d>` (rig cameras), `vector<Vector3d>` (bearings) | `vector<CameraPose>` | Absolute | Generalized P3P (multi-camera rig to 3D). **Bearing-native.** |
| `gp4ps()` | `vector<Vector3d>` (rig cameras), `vector<Vector3d>` (bearings) | `vector<CameraPose>` | Absolute | Generalized 4-point solver. **Bearing-native.** |

**Key insight:** PoseLib has **bearing-vector solvers**. The gap is in the **robust estimation layer** — there's no RANSAC wrapper for these solvers that outputs a high-level API like `estimate_absolute_pose_bearings()`.

### 1.4 RANSAC & Refinement Infrastructure

**RANSAC Architecture** (`ransac.h`, `ransac_impl.h`):

PoseLib uses a templated LO-RANSAC design (inspired by RansacLib):
```cpp
template <typename Solver, typename Model = CameraPose>
RansacStats ransac(Solver &estimator, const RansacOptions &opt, Model *best_model);
```

The `Solver` class must implement three methods:
```cpp
class Solver {
    void generate_models(std::vector<Model> *models);           // Sample & solve
    double score_model(const Model &model, size_t *inlier_count) const;  // Residual
    void refine_model(Model *model) const;                      // LO refinement

    const size_t sample_sz;    // e.g., 5 for 5-point
    const size_t num_data;     // Total data points
};
```

**Existing estimator classes** (in `robust/estimators/`):
- `RelativePoseEstimator`: wraps `relpose_5pt()`, uses 2D Sampson error scoring.
- `GeneralizedRelativePoseEstimator`: wraps generalized 5-point, uses 2D Sampson error.
- `AbsolutePoseEstimator`: wraps `p3p()`, uses 2D reprojection error scoring.
- `GeneralizedAbsolutePoseEstimator`: wraps `gp3p()`/`gp4ps()`, uses 2D reprojection error.

**Refinement** (`bundle.h`):
- `bundle_adjust()`: minimizes reprojection error (2D pixel-space) for calibrated camera.
- `generalized_bundle_adjust()`: minimizes reprojection error for generalized camera rigs.
- `refine_relpose()`: minimizes Sampson error (2D) for relative pose.
- No refinement function exists for bearing-vector inputs.

**RansacOptions** struct (types.h):
```cpp
struct RansacOptions {
    size_t max_iterations = 100000;
    size_t min_iterations = 1000;
    double dyn_num_trials_mult = 3.0;
    double success_prob = 0.9999;
    double max_reproj_error = 12.0;     // 2D pixel threshold
    double max_epipolar_error = 1.0;    // 2D epipolar threshold
    unsigned long seed = 0;
    bool progressive_sampling = false;
    size_t max_prosac_iterations = 100000;
    bool real_focal_check = false;
    bool score_initial_model = false;
};
```

**RansacStats** struct (types.h):
```cpp
struct RansacStats {
    size_t refinements = 0;
    size_t iterations = 0;
    size_t num_inliers = 0;
    double inlier_ratio = 0;
    double model_score = std::numeric_limits<double>::max();
};
```

### 1.5 Current OpenMVS Usage

**File: `/Users/dancostin/Pro/openMVS/libs/SFM/PairsMatcher.cpp`**

```cpp
poselib::RansacStats stats = poselib::estimate_relative_pose(
    pts1, pts2,                 // Point2D (2D normalized coords)
    plCam1, plCam2,             // PinholeCameraModel (identity intrinsics)
    ransacOpt,
    bundleOpt,
    &plPose,
    &inliers);
```

For spherical cameras, `pts1` and `pts2` are populated by `camera->Unproject(pixelCoord)`, which for spherical cameras converts a pixel to a normalized 2D direction (losing hemisphere information).

**File: `/Users/dancostin/Pro/openMVS/libs/SFM/Resection.cpp`**

```cpp
poselib::RansacStats stats = poselib::estimate_absolute_pose(
    points2D, points3D,         // Point2D (2D) + Point3D (3D world)
    plCam,                      // PinholeCameraModel (identity intrinsics)
    ransacOpt,
    bundleOpt,
    &camPose,
    &inliers);
```

Again, `points2D` is 2D normalized coordinates, not bearing vectors.

### 1.6 Version Information

**PoseLib version:** `2.0.4` (from `/Users/dancostin/Pro/openMVS/make/vcpkg_installed/arm64-osx/include/PoseLib/version.h`)

**vcpkg.json pin:** `"poselib"` (no version constraint, vcpkg pins latest).

**Latest upstream PoseLib:** Check https://github.com/PoseLib/PoseLib/releases — as of April 2026, the latest stable is still 2.0.x. No changelog mentions bearing-vector robust estimators.

---

## Part 2: Decision Matrix

| Aspect | Option 1 | Option 2 | Option 3 |
|--------|----------|----------|----------|
| **Direct support in PoseLib?** | No | Partial (gen. cameras take 2D) | N/A |
| **Bearing-vector solvers available?** | N/A | Yes (p3p, gen_relpose_5p1pt) | N/A |
| **High-level robust API?** | No | No | To be built |
| **Requires PoseLib patch?** | N/A | No (use as-is) | No (wrapper in OpenMVS) |
| **Execution complexity** | N/A | Low (use existing path) | Medium (new RANSAC wrappers) |
| **Angular-error scoring?** | No | Yes (via bearings) | Yes |
| **Hemisphere-safe?** | No | No (lossy conversion) | Yes |

### Decision: **Option 3 (RANSAC wrapper in OpenMVS)**

**Reasoning:**
- Option 1 is out: no bearing-vector entry point exists in PoseLib.
- Option 2 (generalized camera path) would still require converting bearings → 2D, which loses hemisphere information and defeats the purpose.
- Option 3 is clean and **non-invasive**: we build an adapter layer in OpenMVS that plugs bearing-vector estimator classes into PoseLib's existing templated `ransac<Solver>()` template. This reuses ~95% of PoseLib infrastructure and requires ~300 lines of new code.

---

## Part 3: Extension Design (Option 3)

### 3.1 Architecture Overview

Create new files in `/Users/dancostin/Pro/openMVS/libs/SFM/`:
- `PoseLiBearingVector.h` — bearing-vector estimator classes + entry-point functions
- `PoseLiBearingVector.cpp` — implementations (minimal; mostly delegates to PoseLib)

These files will:
1. Define `RelativePoseBearingEstimator` class that wraps `relpose_5pt()` with **angular-error scoring**.
2. Define `AbsolutePoseBearingEstimator` class that wraps `p3p()` with **angular-error scoring**.
3. Expose high-level functions:
   - `estimate_relative_pose_bearings()`
   - `estimate_absolute_pose_bearings()`

These functions will plug estimators into PoseLib's existing `ransac<Solver>()` template and optionally call PoseLib's refinement functions.

### 3.2 Entry-Point Signatures

```cpp
namespace poselib {

// Relative pose from bearing vectors (unit-norm 3D directions)
RansacStats estimate_relative_pose_bearings(
    const std::vector<Eigen::Vector3d> &bearings1,  // Unit vectors from camera 1
    const std::vector<Eigen::Vector3d> &bearings2,  // Unit vectors from camera 2
    const RansacOptions &ransac_opt,
    const BundleOptions &bundle_opt,  // For refinement (if implemented)
    CameraPose *relative_pose,                      // Output
    std::vector<char> *inliers);                    // Output

// Absolute pose from bearing vectors and 3D points
RansacStats estimate_absolute_pose_bearings(
    const std::vector<Eigen::Vector3d> &bearings,   // Unit vectors from camera
    const std::vector<Eigen::Vector3d> &points3D,   // 3D world points
    const RansacOptions &ransac_opt,
    const BundleOptions &bundle_opt,  // For refinement (if implemented)
    CameraPose *pose,                               // Output
    std::vector<char> *inliers);                    // Output

} // namespace poselib
```

**Note:** We namespace them in `poselib::` so they appear alongside existing estimators (even though they're implemented in OpenMVS—this is a wrapper pattern).

### 3.3 Estimator Class Design

#### RelativePoseBearingEstimator

```cpp
class RelativePoseBearingEstimator {
  public:
    RelativePoseBearingEstimator(
        const poselib::RansacOptions &ransac_opt,
        const std::vector<Eigen::Vector3d> &bearings1,
        const std::vector<Eigen::Vector3d> &bearings2)
        : num_data(bearings1.size()),
          opt(ransac_opt),
          x1(bearings1),
          x2(bearings2),
          sampler(num_data, sample_sz, opt.seed,
                  opt.progressive_sampling, opt.max_prosac_iterations) {
        x1s.resize(sample_sz);
        x2s.resize(sample_sz);
        sample.resize(sample_sz);
    }

    // PoseLib RANSAC interface: these three methods
    void generate_models(std::vector<poselib::CameraPose> *models);
    double score_model(const poselib::CameraPose &pose, size_t *inlier_count) const;
    void refine_model(poselib::CameraPose *pose) const;

    const size_t sample_sz = 5;
    const size_t num_data;

  private:
    const poselib::RansacOptions &opt;
    const std::vector<Eigen::Vector3d> &x1;  // Bearing vectors from camera 1
    const std::vector<Eigen::Vector3d> &x2;  // Bearing vectors from camera 2

    poselib::RandomSampler sampler;
    std::vector<Eigen::Vector3d> x1s, x2s;    // Sampled bearings
    std::vector<size_t> sample;                // Sampled indices
};
```

**Implementation:**

1. **`generate_models()`:**
   - Sample 5 random bearing pairs.
   - Call `poselib::relpose_5pt(x1s, x2s, &models)`.
   - Collect the output (typically 1–4 candidate poses).

2. **`score_model(pose, inlier_count)`:**
   - For each bearing pair (b1, b2), compute the **angular residual**:
     - Rotate b2 to camera 1 frame: `b2_rotated = pose.rotate(b2)`.
     - Angular error: `error = acos(clamp(dot(b1, b2_rotated), -1, 1))`.
     - Clamp dot product to [-1, 1] to handle numerical precision issues.
   - Threshold: `error < opt.max_epipolar_error` (reuse the epipolar threshold; angular and pixel thresholds are loosely related via image resolution).
   - Return **sum of thresholded errors** (MSAC scoring, same as PoseLib's existing estimators).
   - Increment `*inlier_count` for threshold-passing pairs.

3. **`refine_model(pose)`:**
   - **Option A (simple):** No refinement; bearing-vector optimization is expensive and rarely improves substantially after RANSAC inlier filtering.
   - **Option B (advanced):** Implement a minimal Ceres-based refinement that minimizes angular error on inliers (deferred to Phase 2).
   - **Current recommendation:** Option A. The robustness of the solver + inlier filtering typically suffices.

#### AbsolutePoseBearingEstimator

```cpp
class AbsolutePoseBearingEstimator {
  public:
    AbsolutePoseBearingEstimator(
        const poselib::RansacOptions &ransac_opt,
        const std::vector<Eigen::Vector3d> &bearings,
        const std::vector<Eigen::Vector3d> &points3D)
        : num_data(bearings.size()),
          opt(ransac_opt),
          x(bearings),
          X(points3D),
          sampler(num_data, sample_sz, opt.seed,
                  opt.progressive_sampling, opt.max_prosac_iterations) {
        xs.resize(sample_sz);
        Xs.resize(sample_sz);
        sample.resize(sample_sz);
    }

    void generate_models(std::vector<poselib::CameraPose> *models);
    double score_model(const poselib::CameraPose &pose, size_t *inlier_count) const;
    void refine_model(poselib::CameraPose *pose) const;

    const size_t sample_sz = 3;
    const size_t num_data;

  private:
    const poselib::RansacOptions &opt;
    const std::vector<Eigen::Vector3d> &x;    // Bearing vectors
    const std::vector<Eigen::Vector3d> &X;    // 3D world points

    poselib::RandomSampler sampler;
    std::vector<Eigen::Vector3d> xs, Xs;
    std::vector<size_t> sample;
};
```

**Implementation:**

1. **`generate_models()`:**
   - Sample 3 random bearing-3D point pairs.
   - Call `poselib::p3p(xs, Xs, &models)`.
   - Output: 0–4 candidate poses.

2. **`score_model(pose, inlier_count)`:**
   - For each bearing-3D pair (b, X), compute the **angular reprojection error**:
     - Transform point: `X_cam = pose.apply(X)` (rotate + translate).
     - Normalize to bearing: `b_pred = X_cam.normalized()`.
     - Angular error: `error = acos(clamp(dot(b, b_pred), -1, 1))`.
     - **No depth check**: for spherical cameras, back-facing points are valid. (This differs from pinhole, where we'd reject negative depths.)
   - Threshold: `error < opt.max_reproj_error` (reuse reprojection error threshold).
   - Return MSAC score; count inliers.

3. **`refine_model(pose)`:**
   - Same as relative case: Option A (no refinement) or Option B (defer).

### 3.4 Error Metrics (Critical Detail)

**Angular Error (Recommended):**
```
angular_error_rad = acos(clamp(dot(b_obs, b_pred), -1, 1))
```

This is the **geodesic distance on the unit sphere**. It's principled, invariant to camera resolution, and directly meaningful for spherical cameras.

**Alternative: Chord Distance (Faster):**
```
chord_distance = ||b_obs - b_pred||_2
// ≈ 2 * sin(angular_error / 2)
```

For small angles (< 0.1 rad ≈ 6°), this is approximately equal to angular error and avoids the `acos()`. Profile later if needed.

**Threshold Conversion:**
- User specifies `max_epipolar_error` (for relative) or `max_reproj_error` (for absolute) in **pixels** or **normalized coordinates**.
- For a spherical camera of resolution W×H, 1 pixel ≈ `(2π / W)` radians in longitude and `(π / H)` radians in latitude.
- **Simplification:** Reuse the threshold values as-is. A threshold of `1.0` (the default) loosely corresponds to ~1 pixel of angular deviation on a typical 360° image (W ~ 4000 pixels → 1 rad ≈ 4000 / (2π) ≈ 600 pixels—too large). For spherical, users should probably set smaller thresholds (e.g., 0.1 rad). Document this clearly.
- **Better approach:** Add a configuration parameter `bearing_error_is_angular = true` to `RansacOptions` to signal that thresholds are in radians, not pixels. (Deferred to Phase 2.)

### 3.5 Integration with PoseLib's RANSAC Template

Once estimator classes are defined, invoking RANSAC is one line:

```cpp
RelativePoseBearingEstimator estimator(ransac_opt, bearings1, bearings2);
RansacStats stats = poselib::ransac<RelativePoseBearingEstimator>(
    estimator, ransac_opt, &best_pose);
```

PoseLib's `ransac_impl.h` template will:
1. Repeatedly sample (using `estimator.sampler`).
2. Call `estimator.generate_models()` to solve the sampled subset.
3. Call `estimator.score_model()` to evaluate each candidate.
4. Track the best model and dynamically adjust iteration count.
5. Call `estimator.refine_model()` on the best model (local optimization).
6. Return `RansacStats` with inlier count, iterations, score.

This is **zero additional RANSAC machinery**—we inherit all of PoseLib's sophisticated sampling, early termination, local optimization, and statistical testing.

### 3.6 Refinement Layer (Phase 2 / Optional)

For now, leave `refine_model()` as a no-op:
```cpp
void RelativePoseBearingEstimator::refine_model(poselib::CameraPose *pose) const {
    // TODO: Implement bearing-vector Levenberg-Marquardt refinement
    // For now, RANSAC inlier filtering is sufficient.
}
```

**If refinement is needed later:**
- Use Ceres' `AutoDiffCostFunction` to minimize angular error on all inliers.
- Cost function: `error = acos(dot(b_obs, R(pose) * b_pred))`.
- Parameterize pose as quaternion + translation (match PoseLib's `CameraPose` layout).
- Typically converges in 3–5 iterations.

### 3.7 Degenerate Case Handling

**Relative Pose (5-point):**
- **Coplanar bearings:** If all bearing pairs lie in a plane, the 5-point solver returns 1–4 solutions, but they may be poorly constrained. PoseLib's scorer will naturally rank them by inlier count; a good model will emerge. No special handling needed.
- **Front/back ambiguity:** For spherical cameras, both the pose and its "flipped" (180° rotation) version are geometrically valid. The 5-point solver returns multiple solutions; RANSAC scores them all and picks the one with the most inliers. This is correct behavior.

**Absolute Pose (P3P):**
- **Planar points:** P3P degenerates when the 3D points are coplanar. The solver may return 0, 1, or 4 solutions. RANSAC handles this naturally by scoring all solutions and picking the best.
- **No depth check:** Unlike pinhole cameras, we don't reject back-facing points. The scorer counts any bearing whose prediction error is below the threshold.

**Cheirality (removed for spherical):**
- Pinhole's `estimate_absolute_pose()` does a cheirality check: after solving for a pose, it filters out points with negative depth and re-scores. This ensures the 3D points are in front of the camera.
- **For spherical cameras:** There is no "in front." Any bearing direction is valid. We should **skip the cheirality check** entirely.
- This is handled automatically because we compute residuals based on angular error, not depth.

### 3.8 File Locations & Dependencies

**New files to create:**
```
/Users/dancostin/Pro/openMVS/libs/SFM/PoseLiBearingVector.h
/Users/dancostin/Pro/openMVS/libs/SFM/PoseLiBearingVector.cpp
```

**Dependencies:**
- `#include <PoseLib/poselib.h>` — for RANSAC template, solver calls, types.
- `#include <Eigen/Dense>` — for Vector3d, etc.
- `#include "Camera.h"` — for SphericalCamera::UnprojectNormalized().

**Integration point:**
- In `PairsMatcher.cpp` and `Resection.cpp`, add a check:
  ```cpp
  if (camera.GetType() == CameraType::SPHERICAL) {
      // Convert to bearings, call estimate_*_pose_bearings()
  } else {
      // Keep existing 2D path
  }
  ```

### 3.9 Estimated Code Size

- **PoseLiBearingVector.h:** ~150 lines (class definitions, minimal inline helpers).
- **PoseLiBearingVector.cpp:** ~200 lines (generate_models, score_model, entry-point functions).
- **Integration in PairsMatcher.cpp & Resection.cpp:** ~50 lines total (conditionals + helper calls).

**Total new code:** ~400 lines.
**Lines reusing PoseLib:** ~1000+ (RANSAC loop, sampling, solvers, statistics).
**Reuse ratio:** ~71% (1000 / 1400).

---

## Part 4: Integration with OpenMVS SfM Pipeline

### 4.1 Call Sites

**`PairsMatcher.cpp` (relative pose):**

Current (pinhole):
```cpp
poselib::RansacStats stats = poselib::estimate_relative_pose(
    pts1, pts2, plCam1, plCam2, ransacOpt, bundleOpt, &plPose, &inliers);
```

New (unified):
```cpp
poselib::RansacStats stats;
if (cam1.GetType() == CameraType::SPHERICAL && cam2.GetType() == CameraType::SPHERICAL) {
    // Convert pixels to bearing vectors
    std::vector<Eigen::Vector3d> bearings1, bearings2;
    for (size_t i = 0; i < matches.size(); ++i) {
        bearings1.push_back(cam1.UnprojectNormalized(matches[i].first));
        bearings2.push_back(cam2.UnprojectNormalized(matches[i].second));
    }
    // Call bearing-vector estimator
    stats = poselib::estimate_relative_pose_bearings(
        bearings1, bearings2, ransacOpt, bundleOpt, &plPose, &inliers);
} else {
    // Existing pinhole path (or heterogeneous camera pair)
    stats = poselib::estimate_relative_pose(
        pts1, pts2, plCam1, plCam2, ransacOpt, bundleOpt, &plPose, &inliers);
}
```

**`Resection.cpp` (absolute pose):**

Current:
```cpp
poselib::RansacStats stats = poselib::estimate_absolute_pose(
    points2D, points3D, plCam, ransacOpt, bundleOpt, &camPose, &inliers);
```

New:
```cpp
poselib::RansacStats stats;
if (img.pCamera->GetType() == CameraType::SPHERICAL) {
    // Convert pixels to bearings
    std::vector<Eigen::Vector3d> bearings;
    for (const auto &p : points2D) {
        bearings.push_back(img.pCamera->UnprojectNormalized(p));
    }
    stats = poselib::estimate_absolute_pose_bearings(
        bearings, points3D, ransacOpt, bundleOpt, &camPose, &inliers);
} else {
    stats = poselib::estimate_absolute_pose(
        points2D, points3D, plCam, ransacOpt, bundleOpt, &camPose, &inliers);
}
```

### 4.2 Behavior Change

**Before (spherical camera):**
- Pixels (u, v) → `UnprojectNormalized()` → (x, y, z) normalized.
- (x, y) passed to PoseLib as 2D point.
- Hemisphere information (z's sign) **discarded**.
- Pose estimated in pixel/epipolar-space (2D error metric).

**After (spherical camera):**
- Pixels (u, v) → `UnprojectNormalized()` → (x, y, z) unit vector.
- Full bearing (x, y, z) passed to estimator.
- **No information loss**: hemisphere preserved.
- Pose estimated in angular-space (3D error metric on sphere).

**For pinhole cameras:**
- Behavior unchanged (takes the old path).

### 4.3 Regression Testing

**Synthetic tests** (add to `apps/Tests/TestsSFM.cpp`):

```cpp
// Test 1: Relative pose recovery (spherical camera, both hemispheres)
// - Generate random relative pose.
// - Project 3D world points to 2D image in camera 1 frame.
// - Transform points to camera 2 frame, project to 2D.
// - Add noise to pixel coordinates.
// - Call estimate_relative_pose_bearings().
// - Verify recovered pose matches ground truth (within tolerance).
// - Test both front- and back-facing points.

// Test 2: Absolute pose recovery (spherical camera).
// - Similar: ground truth pose, 3D points, noisy 2D observations.
// - Verify estimated pose matches ground truth.
// - Test back-facing points separately.

// Test 3: Numerical consistency.
// - Generate a match set, compute pose with pinhole + bearing-vector paths.
// - Verify both paths score consistent inlier counts (allowing for numerical differences).
```

**Regression guard:**

Run existing OpenMVS SfM tests on pinhole images:
```bash
# Before & after: outputs should be numerically identical (or within 1e-6 tolerance)
openmvs_test --test_sfm --camera_type pinhole
```

If pinhole tests regress, the integration point is wrong.

---

## Part 5: Risk Assessment & Maintenance

| Risk | Severity | Mitigation |
|------|----------|-----------|
| **Numerical instability in `acos(dot)`** | Medium | Clamp dot product to [-1, 1]; use `acos()` carefully. Alternatively, use `atan2()` for small angles. |
| **Threshold interpretation** | Medium | Document that thresholds for bearing vectors are loosely in radians, not pixels. Add example configuration. |
| **No refinement initially** | Low | Bundle adjustment on bearings is deferred to Phase 2. Current RANSAC + inlier filtering usually suffices. |
| **PoseLib version bumps** | Low | RANSAC template is stable API. If solvers change, only estimator classes need updates. No invasive patching. |
| **Generalized camera regression** | Low | Generalized estimators (`estimate_generalized_*`) are unchanged. They still take 2D. No impact. |
| **Performance** | Low | Bearing-vector operations (normalize, dot product, acos) are cheap. No measurable slowdown vs. pinhole. |

**Maintenance burden:**
- ~200 LOC to maintain (estimator classes).
- Tight coupling to PoseLib's RANSAC template (stable interface).
- If PoseLib's solvers change (p3p, relpose_5pt), we inherit the fixes automatically.
- If PoseLib's RANSAC loop changes, the template interface must be updated (unlikely; it's core infrastructure).

**Confidence:** High. This is a **thin wrapper** approach, not a reimplementation.

---

## Part 6: Deployment Plan

### Phase 1 (Current): Core Implementation
- [ ] Create `PoseLiBearingVector.h` & `.cpp` with `RelativePoseBearingEstimator` & `AbsolutePoseBearingEstimator`.
- [ ] Implement `estimate_relative_pose_bearings()` & `estimate_absolute_pose_bearings()` entry points.
- [ ] Add conditional logic in `PairsMatcher.cpp` & `Resection.cpp` to route spherical cameras to new path.
- [ ] Write synthetic unit tests.
- [ ] Verify pinhole regression tests still pass.

### Phase 2 (Later): Refinement & Polish
- [ ] Add bearing-vector Levenberg-Marquardt refinement in `refine_model()` (optional, if inlier filtering alone is insufficient).
- [ ] Add configuration parameter `bearing_error_is_angular` to `RansacOptions` (optional, for clearer threshold semantics).
- [ ] Performance profiling & optimization (if needed).

### Phase 3 (Future): Generalization
- [ ] Extend to **omni-directional cameras** (e.g., fisheye with 180° FOV).
- [ ] Support **stereo spherical cameras** (two overlapping hemispheres).

---

## Part 7: Recommendation Summary

**Decision: Implement Option 3 (RANSAC wrapper in OpenMVS).**

**Why NOT Option 1 or 2:**
- Option 1: No bearing-vector entry points exist in PoseLib 2.0.4.
- Option 2: Generalized camera estimators take 2D points + rig extrinsics, not bearing vectors + a single origin. Repurposing them loses hemisphere information, which defeats the entire purpose.

**Why Option 3:**
- Reuses PoseLib's templated RANSAC machinery (~90% reuse).
- Preserves hemisphere information (no lossy 2D conversion).
- Minimal code footprint (~400 lines).
- Non-invasive: no patches to vendored PoseLib.
- Clear, maintainable design: thin wrapper layer.
- Unified API: both pinhole and spherical cameras can use the same downstream pipeline if configured correctly.

**Next steps:**
1. Implement Phase 1 (core).
2. Write unit tests on synthetic spherical data.
3. Verify pinhole regression.
4. Integrate into main SfM pipeline.
5. Test on real 360° image sequences.

---

## Appendix: Code Snippets

### A.1 Bearing Vector Computation (existing in SphericalCamera)

From `/Users/dancostin/Pro/openMVS/libs/SFM/Camera.cpp`:

```cpp
Point3 SphericalCamera::UnprojectNormalized(const Point2& x) const {
    // Map image coordinates to spherical angles
    const Point2 sph = MapImageToSpherical(x);
    // Convert spherical to Cartesian (normalized ray)
    return Point3(
        std::sin(sph.y) * std::cos(sph.x),  // x = sin(lat) * cos(lon)
        std::sin(sph.x),                     // y = sin(lon)  [sic: note unconventional naming]
        std::cos(sph.y) * std::cos(sph.x)   // z = cos(lat) * cos(lon)
    );
}
```

This already exists and is correct. We just call it and pass the result to our bearing-vector estimators.

### A.2 PoseLib RANSAC Invocation Pattern

From `ransac_impl.h`:

```cpp
template <typename Solver, typename Model = CameraPose>
RansacStats ransac(Solver &estimator, const RansacOptions &opt, Model *best_model) {
    RansacStats stats;
    if (estimator.num_data < estimator.sample_sz) {
        return stats;
    }

    stats.num_inliers = 0;
    stats.model_score = std::numeric_limits<double>::max();
    // ... RANSAC loop (sampling, scoring, refinement)
    return stats;
}
```

Our estimator classes plug directly into this template.

---

## Appendix: References

1. **PoseLib GitHub:** https://github.com/PoseLib/PoseLib
2. **OpenMVS SfM:** https://cdcseacave.github.io/openMVS/
3. **Spherical cameras in SfM:** Sturm, P. & Ramalingam, S. (2004). "A Generic Concept for Camera Calibration."
4. **Angular Error Metrics:** Hartley, R. & Zisserman, A. (2003). "Multiple View Geometry in Computer Vision" (Chapter on error metrics).

---

**Document Version:** 1.0
**Last Updated:** April 10, 2026
**Status:** Design Phase — Ready for Implementation Review
