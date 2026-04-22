# Patching the vcpkg PoseLib port with PR #178 (bearing-vector relative pose)

## Context

During Phase 3 planning for spherical camera support in OpenMVS's SfM pipeline, we identified that PoseLib 2.0.5 (the vendored version at [make/vcpkg_installed/arm64-osx/include/PoseLib/](../../make/vcpkg_installed/arm64-osx/include/PoseLib/)) exposes low-level bearing-vector solvers (`relpose_5pt`, `p3p`) but **no high-level RANSAC entry point that accepts bearing vectors directly**. Every `poselib::estimate_*` wrapper takes 2D normalized-plane coordinates and internally unprojects via a `poselib::Camera` object — which cannot represent spherical back-hemisphere features because the 2D `(x, y)` form drops the `sign(z)` hemisphere bit.

An alternative design doc ([PoseLibBearingVectorPlan.md](PoseLibBearingVectorPlan.md)) proposed writing our own ~400-line RANSAC wrapper in OpenMVS that reuses PoseLib's `ransac<Solver>()` template. The user then pointed at [PoseLib PR #178](https://github.com/PoseLib/PoseLib/pull/178) ("Add EQUIRECTANGULAR camera model support for 360° images") by @zalo, which adds exactly the missing `BearingRelativePoseEstimator` upstream. This document is the plan for pulling that PR into OpenMVS via a vcpkg port patch instead of reinventing it locally.

**Intended outcome:** A patched local vcpkg port of PoseLib that includes PR #178's bearing-vector relative pose API, so `PairsMatcher.cpp` can feed unit bearing vectors directly to PoseLib and unify pinhole + spherical on the same code path — with a clear record of the residual gaps the PR does *not* close (absolute pose / PnP).

## PR #178 inventory

### Metadata
- **Upstream PR**: https://github.com/PoseLib/PoseLib/pull/178
- **Author**: @zalo
- **Title**: "Add EQUIRECTANGULAR camera model support for 360° images"
- **State**: `open` (not merged)
- **Mergeable**: `false` / `mergeable_state: dirty` — has conflicts against the current `master`. Last updated 2026-01-22.
- **Base branch**: `master` at `f917bbd584927efcafabe3b12247582bbdc36297`
- **Head commit**: `c18a978bb95196b835d1018b3825e4f21cddf12a` (on `zalo/PoseLib:feature/equirectangular`)
- **Diff URL**: https://github.com/PoseLib/PoseLib/pull/178.diff
- **Patch URL**: https://github.com/PoseLib/PoseLib/pull/178.patch
- **Scope**: 7 files modified, +550/-1 lines, all additions.

### Files changed (with line counts)

| File | Additions | Deletions | What it adds |
|---|---|---|---|
| `PoseLib/misc/colmap_models.h` | 14 | 1 | `EquirectangularCameraModel` class decl; `is_spherical()`, `unproject_bearing()`, `project_bearing()`, `project_bearing_with_jac()` methods on `Camera`; `SETUP_CAMERA_SHARED_DEFS(EquirectangularCameraModel, "EQUIRECTANGULAR", 14)` macro and new `SWITCH_CAMERA_MODELS` case. |
| `PoseLib/misc/colmap_models.cc` | 216 | 0 | Equirectangular `project`/`unproject` (2D normalized — front hemisphere only; PR openly notes this) plus `project_with_jac`. Full implementations of `Camera::is_spherical()`, `unproject_bearing()` (pixel → unit bearing), `project_bearing()` (unit bearing → pixel, handles full sphere), `project_bearing_with_jac()` with an explicit singular-pole branch (`Z² + X² ≈ 0` → `theta = 0`). |
| `PoseLib/robust/estimators/relative_pose.h` | 32 | 0 | New class `BearingRelativePoseEstimator` at the end of the existing `RelativePoseEstimator` / `FundamentalEstimator` file, with `generate_models()`, `score_model()`, `refine_model()`, `sample_sz = 5`, holding `const std::vector<Point3D>& b1, b2;` and a `RandomSampler`. Note: file ends with `\n\\ No newline at end of file` — the patch will need a trailing newline fixup. |
| `PoseLib/robust/estimators/relative_pose.cc` | 91 | 0 | Implementation of the three methods: `generate_models` samples 5 points and calls `relpose_5pt(x1s, x2s, models)` on pre-normalized bearings; `score_model` computes `b2ᵀ·E·b1` epipolar residual with MSAC scoring (inlier if `err² < max_angular_error_sq`, where `max_angular_error = opt.max_epipolar_error * 0.01`); `refine_model` extracts inliers and calls `refine_relpose_bearing(...)`. |
| `PoseLib/robust/bundle.h` | 6 | 0 | Forward declaration of `BundleStats refine_relpose_bearing(const std::vector<Point3D>&, const std::vector<Point3D>&, CameraPose*, const BundleOptions&, const std::vector<double>& weights)`. |
| `PoseLib/robust/bundle.cc` | 35 | 0 | Template `refine_relpose_bearing<WeightType, LossFunction>` that runs `lm_impl<>` with a new `BearingRelativePoseJacobianAccumulator`; switch over loss types via existing `SWITCH_LOSS_FUNCTIONS` macro; uniform-weight entry point. |
| `PoseLib/robust/jacobian_impl.h` | 156 | 0 | New class `BearingRelativePoseJacobianAccumulator<LossFunction, ResidualWeightVector>` mirroring the existing `RelativePoseJacobianAccumulator` but operating on `std::vector<Point3D>` instead of `std::vector<Point2D>`. Computes Jacobians of the epipolar residual w.r.t. pose parameters. |

### What the PR provides (summary)

```cpp
// Public API added by PR #178

namespace poselib {

// In colmap_models.h — new camera model + bearing-vector methods on Camera class
class Camera {
    // ... existing fields ...
    bool is_spherical() const;
    void unproject_bearing(const Eigen::Vector2d& xp, Eigen::Vector3d* bearing) const;
    void project_bearing(const Eigen::Vector3d& bearing, Eigen::Vector2d* xp) const;
    void project_bearing_with_jac(const Eigen::Vector3d& bearing, Eigen::Vector2d* xp,
                                   Eigen::Matrix<double, 2, 3>* jac) const;
};

// In bundle.h — new bearing-vector bundle adjustment
BundleStats refine_relpose_bearing(
    const std::vector<Point3D>& b1,
    const std::vector<Point3D>& b2,
    CameraPose* pose,
    const BundleOptions& opt = BundleOptions(),
    const std::vector<double>& weights = std::vector<double>());

// In robust/estimators/relative_pose.h — new RANSAC estimator (not a free function)
class BearingRelativePoseEstimator {
public:
    BearingRelativePoseEstimator(const RansacOptions& ransac_opt,
                                  const std::vector<Point3D>& bearings_1,
                                  const std::vector<Point3D>& bearings_2);
    void generate_models(std::vector<CameraPose>* models);
    double score_model(const CameraPose& pose, size_t* inlier_count) const;
    void refine_model(CameraPose* pose) const;
    const size_t sample_sz = 5;
    const size_t num_data;
private:
    const RansacOptions& opt;
    const std::vector<Point3D>& b1;
    const std::vector<Point3D>& b2;
    RandomSampler sampler;
    std::vector<Eigen::Vector3d> x1s, x2s;
    std::vector<size_t> sample;
};

} // namespace poselib
```

### Critical gaps in PR #178

These are the reasons this plan exists alongside — not instead of — a small adapter in OpenMVS:

1. **No top-level `estimate_relative_pose_bearings(...)` entry point.** The PR adds the `BearingRelativePoseEstimator` class but does *not* add a free function in `robust.h` that instantiates the estimator, runs `ransac<Estimator>()`, and returns `RansacStats`. OpenMVS would need to instantiate the class and call PoseLib's internal `ransac()` template directly, *or* we add a 5-line wrapper to our patch.

2. **No absolute pose (PnP) support.** PR #178 covers relative pose only. There is no `BearingAbsolutePoseEstimator`. Our [Resection.cpp](../../libs/SFM/Resection.cpp) call site — which uses `poselib::estimate_absolute_pose` — gets no help from this PR. To close Group B fully we would need either (a) a second custom estimator class in OpenMVS that wraps `p3p` with angular scoring, or (b) a follow-up PR to PoseLib adding `BearingAbsolutePoseEstimator`. See §"Residual Group B work" at the end of this doc.

3. **Scoring function is epipolar, not true angular.** The `score_model()` computes `err = b2.dot(E * b1)` and thresholds `err² < (opt.max_epipolar_error * 0.01)²`. The `0.01` factor is a hardcoded `kEpipolarToAngularErrorFactor` in both `score_model` and `refine_model`, with a code comment openly admitting:
   > The scalar factor below was chosen empirically to give reasonable inlier thresholds for typical omnidirectional image resolutions; tune `opt.max_epipolar_error` to adjust behavior.

   This is load-bearing for threshold calibration. Not a principled `acos(dot)` angular error — it's an epipolar constraint residual scaled by a magic number. Acceptable for RANSAC voting (monotone in the true angular error near zero), but OpenMVS's `PairsMatcher` will need to scale its `maxEpipolarError` config differently for spherical pairs. Worth documenting loudly in the call-site comment.

4. **PR is unmerged and conflicts with `master`.** The branch `zalo:feature/equirectangular` was created against `master` at `f917bbd5...` and is now marked `dirty` — some file on master has moved on. We need to either rebase manually or pin to commit `c18a978b` and hope the conflicting file(s) aren't ones we care about (most likely a macro list in `colmap_models.h` where `SWITCH_CAMERA_MODEL_CASE(...)` lines were added by both the PR and upstream).

5. **Missing trailing newline in `relative_pose.h`.** The patch ends with `\n\\ No newline at end of file`. Harmless but produces a warning on strict compilers.

6. **EquirectangularCameraModel uses model ID 14, not 10.** PR body claims "Model ID 10 (matching COLMAP convention)" but the macro in the diff is `SETUP_CAMERA_SHARED_DEFS(EquirectangularCameraModel, "EQUIRECTANGULAR", 14)`. Cosmetic — OpenMVS doesn't serialize PoseLib camera IDs — but worth fixing if we're already patching.

## Current vcpkg port state

- **Port location**: `/Users/dancostin/Pro/vcpkg/ports/poselib/` (upstream vcpkg registry at `$VCPKG_ROOT`)
- **Pinned version**: 2.0.5 (in `vcpkg.json`)
- **Source fetch**: `vcpkg_from_github(REPO PoseLib/PoseLib, REF "v${VERSION}", SHA512 ed56d8cd..., HEAD_REF master)` in `portfile.cmake`
- **Existing patches**: One (`fatal-errors.patch`) applied in the `PATCHES` list.
- **How the build consumes it**: `openmvs/vcpkg.json` lists `"poselib"` as a dependency; `make/CMakeCache.txt` sets `CMAKE_TOOLCHAIN_FILE=/Users/dancostin/Pro/vcpkg/scripts/buildsystems/vcpkg.cmake`, so builds pull from the upstream vcpkg tree unmodified.

**The challenge:** we cannot simply edit files under `/Users/dancostin/Pro/vcpkg/ports/poselib/` — that's the shared vcpkg registry, and any local edits would be wiped by the next `vcpkg update`. We need to create an **overlay port** under `openmvs/ports/poselib/` and tell vcpkg to prefer it via `vcpkg-configuration.json` or the `VCPKG_OVERLAY_PORTS` environment variable.

OpenMVS already has an overlay port at [ports/siftgpu/](../../ports/siftgpu/), so the mechanism is in place — we just need to add a second entry.

## Implementation plan

### Step 1 — Fetch the PR as a patch file

Download the diff that GitHub generates for PR #178 and save it into the new overlay port directory:

```bash
mkdir -p /Users/dancostin/Pro/openMVS/ports/poselib
curl -fsSL https://github.com/PoseLib/PoseLib/pull/178.patch \
    > /Users/dancostin/Pro/openMVS/ports/poselib/0001-pr178-equirectangular-bearing-relative-pose.patch
```

**Pin to a specific commit**, not the "live" PR URL, to keep the patch deterministic. The PR head commit at the time of writing is `c18a978bb95196b835d1018b3825e4f21cddf12a`. If `curl` against the live `.patch` URL produces a file whose header references that sha, we're fine. If the PR author pushes more commits later, we want to know; re-fetch and re-verify by hand.

**Alternative — inline the patch**: if GitHub's generated `.patch` is flaky or includes non-source noise, we can instead write a hand-curated patch by downloading each of the 7 changed files from the `c18a978b` commit via raw URLs and `git diff`-ing them against vcpkg's 2.0.5 tarball. More work; reserve for a rebase situation.

### Step 2 — Handle the merge conflict

PR #178 is marked `mergeable_state: dirty`. The conflict is almost certainly in `PoseLib/misc/colmap_models.h` — specifically the `SWITCH_CAMERA_MODELS` macro list, which grows as upstream adds new camera models. A textual resolution is straightforward:

1. Download `PoseLib/misc/colmap_models.h` at tag `v2.0.5` (what vcpkg pulls).
2. Compare against the PR diff's "before" hunks.
3. If the conflict is only "new case inserted in the middle of a macro list", the patch can be re-anchored to the 2.0.5 version by hand. Update the patch file's context lines to match 2.0.5.
4. If the conflict is structural (e.g. upstream renamed a class), escalate to §"Alternative: vendor the PR source tree" below.

**Verification that the conflict is tractable**: before we spend time rebasing, first try applying the patch dry-run:
```bash
cd /tmp && git clone --depth 1 --branch v2.0.5 https://github.com/PoseLib/PoseLib.git poselib-2.0.5
cd poselib-2.0.5
git apply --check /Users/dancostin/Pro/openMVS/ports/poselib/0001-pr178-equirectangular-bearing-relative-pose.patch
```
If `git apply --check` succeeds, no rebase needed — PR was born against a commit close enough to 2.0.5 that the hunks still anchor. If it reports rejected hunks, rebase manually against each rejected file.

### Step 3 — Create the overlay port

Copy the three files from the upstream vcpkg port as a starting point, then modify:

```bash
mkdir -p /Users/dancostin/Pro/openMVS/ports/poselib
cp /Users/dancostin/Pro/vcpkg/ports/poselib/{vcpkg.json,portfile.cmake,fatal-errors.patch} \
   /Users/dancostin/Pro/openMVS/ports/poselib/
```

Edit [ports/poselib/vcpkg.json](../../ports/poselib/vcpkg.json) (new file) to bump the port-version so vcpkg rebuilds instead of reusing a cached 2.0.5 binary:

```json
{
  "name": "poselib",
  "version": "2.0.5",
  "port-version": 1,
  "description": "Minimal solvers for calibrated camera pose estimation (patched with PR #178 for equirectangular bearing-vector relative pose)",
  "homepage": "https://github.com/PoseLib/PoseLib",
  "license": "BSD-3-Clause",
  "dependencies": [
    "eigen3",
    { "name": "vcpkg-cmake", "host": true },
    { "name": "vcpkg-cmake-config", "host": true }
  ]
}
```

Edit [ports/poselib/portfile.cmake](../../ports/poselib/portfile.cmake) to add the new patch alongside the existing `fatal-errors.patch`:

```cmake
if(VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO PoseLib/PoseLib
    REF "v${VERSION}"
    SHA512 ed56d8cd6a3073776edbfe9d11e2ebf8e2bed4065f7f53a02541323c1631632bf6c161d305fc09674175351b024bf019211dfa9d7a48e74e3c5563941099f1ef
    HEAD_REF master
    PATCHES
        fatal-errors.patch
        0001-pr178-equirectangular-bearing-relative-pose.patch
)

vcpkg_cmake_configure(
    SOURCE_PATH ${SOURCE_PATH}
    OPTIONS
        -DMARCH_NATIVE=OFF
        -DWITH_BENCHMARK=OFF
        -DPYTHON_PACKAGE=OFF
)
vcpkg_cmake_install()
vcpkg_copy_pdbs()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/PoseLib)

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
```

No other edits to `portfile.cmake` are needed — the PR adds files that are already picked up by PoseLib's existing `CMakeLists.txt` globbing (all `.cc` files under `PoseLib/robust/estimators/` are auto-included). Verify this assumption by grepping the PoseLib `CMakeLists.txt` after unpack.

### Step 4 — Register the overlay with OpenMVS

Two options — option A is preferred because it's explicit and persists across developers.

**Option A — `vcpkg-configuration.json` in repo root (preferred)**

Create or edit [vcpkg-configuration.json](../../vcpkg-configuration.json) at the OpenMVS repo root:

```json
{
  "default-registry": {
    "kind": "builtin",
    "baseline": "<pin current vcpkg baseline>"
  },
  "overlay-ports": [
    "./ports"
  ]
}
```

The `"./ports"` entry tells vcpkg to scan `openmvs/ports/` for overlay ports before consulting the default registry. Because `./ports/poselib/` exists with `port-version: 1`, vcpkg picks it in preference to the registry's 2.0.5/port-version 0.

**Option B — `VCPKG_OVERLAY_PORTS` environment variable (developer-local)**

Each developer exports:
```bash
export VCPKG_OVERLAY_PORTS=/Users/dancostin/Pro/openMVS/ports
```
Simpler but not version-controlled, so CI and fresh clones won't pick it up without additional setup.

### Step 5 — Rebuild and verify headers land correctly

```bash
cd /Users/dancostin/Pro/openMVS/make
rm -rf vcpkg_installed/arm64-osx/{lib,include,share}/PoseLib  # force re-extract
cmake --build . --target Tests -j4
```

After the rebuild, verify the new symbols are present in the installed headers:
```bash
grep -l BearingRelativePoseEstimator \
    /Users/dancostin/Pro/openMVS/make/vcpkg_installed/arm64-osx/include/PoseLib/robust/estimators/relative_pose.h
grep -l is_spherical \
    /Users/dancostin/Pro/openMVS/make/vcpkg_installed/arm64-osx/include/PoseLib/misc/colmap_models.h
grep -l refine_relpose_bearing \
    /Users/dancostin/Pro/openMVS/make/vcpkg_installed/arm64-osx/include/PoseLib/robust/bundle.h
```

All three should return the header path. If any fails, the patch didn't land — check vcpkg's buildtree log at `buildtrees/poselib/apply-patches-out.log`.

### Step 6 — Integrate in OpenMVS SFM

The PR does not provide a top-level free function, so OpenMVS has two paths for calling the new estimator:

**Path A — Add a thin wrapper inside OpenMVS (recommended)**

Create a new small header [libs/SFM/PoseLibBearingAdapter.h](../../libs/SFM/) (new file, ~40 lines) that exposes a single free function mirroring `poselib::estimate_relative_pose`'s signature:

```cpp
namespace SFM {
// Thin wrapper around BearingRelativePoseEstimator + PoseLib's internal ransac<> loop.
// Mirrors poselib::estimate_relative_pose but consumes 3D unit bearing vectors.
// Used for spherical cameras and any other case where hemisphere information must be preserved.
poselib::RansacStats EstimateRelativePoseBearings(
    const std::vector<Eigen::Vector3d>& bearings1,
    const std::vector<Eigen::Vector3d>& bearings2,
    const poselib::RansacOptions& ransac_opt,
    const poselib::BundleOptions& bundle_opt,
    poselib::CameraPose* pose,
    std::vector<char>* inliers);
}
```

Implementation body instantiates `poselib::BearingRelativePoseEstimator` and calls PoseLib's `poselib::ransac<>()` template (exposed internally via `PoseLib/robust/ransac.h`) directly. This is ~20 lines of glue, isolated to one file, and keeps the call sites in [PairsMatcher.cpp](../../libs/SFM/PairsMatcher.cpp) clean.

Rationale: a wrapper inside OpenMVS is cleaner than adding the wrapper to our vcpkg patch because (a) it's code we can iterate on without rebuilding PoseLib every time, (b) if PR #178 is eventually merged upstream with a free-function wrapper, we can delete ours, and (c) it keeps the patch minimal and therefore easier to rebase.

**Path B — Call the estimator class directly from `PairsMatcher.cpp`**

Skip the wrapper; `PairsMatcher.cpp` instantiates `poselib::BearingRelativePoseEstimator` inline. Marginally fewer files but couples the call site to PoseLib's estimator class layout.

### Step 7 — Update `PairsMatcher.cpp` for spherical cameras

Dispatch on camera type at the top of the relative-pose block. For pinhole pairs, keep the existing `poselib::estimate_relative_pose(pts1, pts2, plCam1, plCam2, ...)` call byte-identical (zero risk of pinhole regression). For spherical or mixed pairs:

```cpp
// Drop-in replacement for the spherical branch
std::vector<Eigen::Vector3d> bearings1, bearings2;
bearings1.reserve(pair.matches.size());
bearings2.reserve(pair.matches.size());
for (const auto& m : pair.matches) {
    const cv::Point2f& pt1 = img1.keypoints[m.queryIdx].pt;
    const cv::Point2f& pt2 = img2.keypoints[m.trainIdx].pt;
    if (minFeatureDistanceSq > 0 && normSq(pt1 - pt2) < minFeatureDistanceSq)
        continue;
    const Point3 r1 = cam1.UnprojectNormalized(Cast<REAL>(pt1));  // unit bearing
    const Point3 r2 = cam2.UnprojectNormalized(Cast<REAL>(pt2));
    bearings1.emplace_back(r1.x, r1.y, r1.z);
    bearings2.emplace_back(r2.x, r2.y, r2.z);
}

// Note: BearingRelativePoseEstimator scores the epipolar residual b2'*E*b1
// with a hardcoded 0.01 scale factor (see PR #178 score_model). To get a
// RANSAC threshold equivalent to ~1 pixel on a 2048-wide equirectangular,
// scale maxEpipolarError accordingly — the scalar is NOT a true angular error.
ransacOpt.max_epipolar_error = config.maxEpipolarError;  // may need retuning
EstimateRelativePoseBearings(bearings1, bearings2, ransacOpt, bundleOpt, &pl_pose, &inliers);
```

### Step 8 — Write a regression test

Extend `apps/Tests/TestsSFM.cpp` with a new test `PairsMatcherSphericalTest` that:

1. Generates a synthetic spherical scene (2 views, radius-5 point cloud, ~50% back-hemisphere coverage — reuse the generator from `ReconstructSphericalSyntheticTest`).
2. Builds a `Scene` with matches between the two images using ground-truth observations.
3. Calls `PairsMatcher::EstimateRelativePose` (or whatever the entry function is) on the pair.
4. Asserts recovered rotation matches ground truth within 0.5° and translation direction matches within 0.5°.

This is the failing test that pins correctness for the new path. Write it *first*, watch it fail (current pinhole-normalized path can't recover spherical relative pose), then wire up the bearing path and watch it pass. Classic TDD cycle, same as Phase 1.

## Residual Group B work not covered by PR #178

Even with PR #178 applied, **absolute pose (PnP) for spherical cameras is still an open problem**. The PR adds zero support for `poselib::estimate_absolute_pose` bearing-vector variants. Options for closing that gap:

1. **Custom estimator in OpenMVS** (recommended for now): write an `AbsolutePoseBearingEstimator` class in OpenMVS that wraps `poselib::p3p(bearings, points3D, solutions)` (already exposed as a bearing-vector solver in PoseLib 2.0.5), scores candidates with angular error, and plugs into PoseLib's `ransac<>` template. Mirror the structure of `BearingRelativePoseEstimator` from PR #178. ~150 lines.

2. **Upstream follow-up PR to PoseLib**: write a `BearingAbsolutePoseEstimator` mirroring the same pattern and submit it. Longer feedback loop; not blocking.

3. **Defer until Phase 4**: spherical resection is only needed once the scene has 3+ calibrated images. If we ship Phase 3 with PR #178 handling two-view + star initialization, we can live without spherical PnP for initial validation on small scenes.

Recommend option 1 in parallel with the PR #178 port patch — they're independent enough to land together.

## Risks and rollback

- **PR author abandons the branch**: `c18a978b` stays pinned in our patch regardless. We own the fork's behavior from the moment we patch. Rollback = delete the overlay port, vcpkg falls back to upstream 2.0.5.
- **Next PoseLib release bumps version past 2.0.5**: we'll need to re-anchor the patch to the new version. Same process as step 2. Low risk because the bearing-vector code lives in files rarely touched by upstream.
- **CI doesn't pick up the overlay**: Option A (the `vcpkg-configuration.json` approach) is version-controlled so CI picks it up automatically. Double-check after first rebuild on a fresh clone.
- **Epipolar scoring factor 0.01 turns out wrong**: if the spherical test in step 8 fails because RANSAC rejects too many inliers (or too few), the fix lives in our `ransacOpt.max_epipolar_error` value — not in PoseLib. Leave a comment at the call site so the next reader knows why it differs from the pinhole threshold.
- **Pinhole regression**: zero if the dispatch in step 7 keeps the pinhole branch byte-identical. Verified by running the existing Tests suite unchanged.

## Verification checklist

End-to-end checks before declaring the port patch done:

- [ ] `git apply --check` on the patch against a fresh `v2.0.5` checkout succeeds.
- [ ] `cmake --build . --target Tests -j4` completes with the overlay port active.
- [ ] `grep BearingRelativePoseEstimator` in the installed headers under `vcpkg_installed/arm64-osx/include/PoseLib/` returns a hit.
- [ ] All pre-existing SFM tests (`./bin/Debug/Tests 1`) still pass — pinhole path unchanged.
- [ ] New `PairsMatcherSphericalTest` passes.
- [ ] `ReconstructSphericalSyntheticTest` from Phase 1 still passes (triangulation + BA path unaffected by the change).
- [ ] A real equirectangular dataset end-to-end (once available) runs through `PairsMatcher` → `StarInitializer` → `Triangulation` → `BA` with non-zero inlier counts on spherical pairs.

## Decision: do we actually do this?

**Yes, patch the vcpkg port** — it's the right call. Reasoning:

- Reuses ~550 lines of PoseLib-maintainer-quality code instead of re-inventing it in OpenMVS. The `BearingRelativePoseJacobianAccumulator` alone is the kind of thing you want written once, carefully, and never again.
- Keeps OpenMVS's SfM code leaner — one thin adapter file instead of a 400-line RANSAC wrapper.
- Aligns with upstream direction: if PR #178 is eventually merged, we can drop our overlay port entirely and fall back to the normal 2.0.6 (or whatever) vcpkg port with a one-line change.
- The two gaps (no top-level wrapper, no absolute pose) are both straightforward to close with small amounts of OpenMVS-side code — a wrapper for the first, a mirror estimator class for the second.

The only scenario where the local-RANSAC-wrapper alternative ([PoseLibBearingVectorPlan.md](PoseLibBearingVectorPlan.md)) wins is if the PR's `c18a978b` commit has been force-pushed/deleted by the author by the time we try to fetch it. If that happens, the wrapper in step 6 can still be built against PoseLib 2.0.5's low-level solvers as originally planned — the wrapper code is the same either way, only the RANSAC driver would differ.

## Step-by-step execution order

1. Fetch PR #178 as a patch file, pin to commit `c18a978b`.
2. Dry-run `git apply --check` against `v2.0.5`. Rebase by hand if it fails.
3. Create overlay port at `ports/poselib/` with `port-version: 1`, copy portfile, add patch to `PATCHES` list.
4. Create/update `vcpkg-configuration.json` at repo root with `"overlay-ports": ["./ports"]`.
5. Clean and rebuild. Verify installed headers contain the new symbols.
6. Write the failing `PairsMatcherSphericalTest` first (TDD RED).
7. Add the thin `EstimateRelativePoseBearings` adapter in `libs/SFM/PoseLibBearingAdapter.{h,cpp}`.
8. Update `PairsMatcher.cpp` with the spherical dispatch branch.
9. Run the full SFM test suite (`./bin/Debug/Tests 0 && ./bin/Debug/Tests 1`). Confirm zero pinhole regressions, new spherical test passes.
10. Commit the port patch, the adapter, and the test as a single feature branch.
11. (Follow-up, not blocking) Write `AbsolutePoseBearingEstimator` for `Resection.cpp` using PoseLib's existing `p3p` bearing-vector solver, mirroring the PR #178 estimator structure.

Estimated effort: 4-6 hours for steps 1-10 if the patch applies cleanly; add 2-3 hours if step 2 requires a manual rebase.
