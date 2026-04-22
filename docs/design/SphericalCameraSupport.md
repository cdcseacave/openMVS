# Spherical Camera Support in OpenMVS SfM — Design Document

## Context

OpenMVS already has substantial spherical (equirectangular / 360°) camera infrastructure in the SFM library — polymorphic `Camera` base class with `SphericalCamera` subclass, a Ceres `SphericalAngularReprojectionError` cost functor, BA dispatch on `CameraType`, EXIF auto-detection, `ExtractKeyframes` CLI `--camera-type 1` flag, and a synthetic scene generator that can emit spherical views. Roughly 75% of the plumbing is done, but **no test actually exercises the spherical path end-to-end**, and several hot spots still assume pinhole semantics. Specifically, `SphericalCamera::Unproject()` returns `(tan θ, tan φ / cos θ)` which diverges at θ = ±π/2 — meaning callers that use `Unproject()` (not `UnprojectNormalized()`) silently break for features near the "back" of the equirectangular image. Downstream, the MVS library has no spherical camera class at all, so we adopt a cube-map bridge (6 virtual pinhole faces) to feed dense reconstruction.

The intended outcome: a reliable end-to-end SFM pipeline for spherical images and video keyframes, validated by synthetic and real-dataset tests, with a documented cube-map export path for MVS downstream.

## Current State — What Already Works

Verified against the current `develop` branch:

- [libs/SFM/Camera.h](../../libs/SFM/Camera.h) — `SphericalCamera` inherits from polymorphic `Camera` base with `Project`, `UnprojectNormalized`, `PixelErrorToAngular`, `AccumulateIntrinsics` (no-op), Boost serialization export.
- [libs/SFM/Camera.cpp:216-278](../../libs/SFM/Camera.cpp#L216) — Equirectangular projection/unprojection.
- [libs/SFM/BundleAdjustmentCostFunctions.h:355-416](../../libs/SFM/BundleAdjustmentCostFunctions.h#L355) — `SphericalAngularReprojectionError` tangent-plane residual (pre-scaled by `pixel_scale = width/(2π)` for direct pixel-equivalent residuals; no z-check because spherical cameras see all directions).
- [libs/SFM/BundleAdjustment.cpp:153-181](../../libs/SFM/BundleAdjustment.cpp#L153) and [:608](../../libs/SFM/BundleAdjustment.cpp#L608) — Dispatch in both global and local BA selects the correct cost functor per `CameraType`. Intrinsics manifold at [:242](../../libs/SFM/BundleAdjustment.cpp#L242) is correctly gated to pinhole.
- [libs/SFM/ViewGraphCalibrator.cpp:266,299,349,400](../../libs/SFM/ViewGraphCalibrator.cpp#L266) — Focal estimation skips spherical (correct, N/A).
- [libs/SFM/PairsMatcher.cpp:500-501](../../libs/SFM/PairsMatcher.cpp#L500) — Skips fundamental-matrix composition for spherical pairs.
- [libs/SFM/Image.cpp](../../libs/SFM/Image.cpp) — Reads EXIF `ProjectionType=equirectangular` and instantiates `SphericalCamera`.
- [libs/SFM/KeyframeExtractor.cpp:444-465](../../libs/SFM/KeyframeExtractor.cpp#L444) — `ExtractFromVideo` already handles `config.cameraType == SPHERICAL` and constructs `SphericalCamera(frameWidth, frameHeight)`.
- [apps/ExtractKeyframes/ExtractKeyframes.cpp:119,227](../../apps/ExtractKeyframes/ExtractKeyframes.cpp#L119) — CLI flag `--camera-type 0|1` (pinhole/spherical) already wired; `config.cameraType = (CameraType)(nCameraType+1)`.
- [apps/Tests/TestsSFM.cpp:249,328](../../apps/Tests/TestsSFM.cpp#L249) — Synthetic scene generator's `CameraSpec::type` supports `SPHERICAL` and instantiates a real `SphericalCamera`, but no test fixture currently passes a spherical spec.

## Phase 1 Findings (live results from the regression test)

Phase 1 added [apps/Tests/TestsSFM.cpp::ReconstructSphericalSyntheticTest](../../apps/Tests/TestsSFM.cpp) — a synthetic spherical scene with 6 cameras clustered near the origin and 80 3D points placed on a sphere of radius 5, guaranteeing ~50% back-hemisphere observations. Running the test revealed:

1. **Triangulation is already correct for spherical cameras.** `TriangulateTracks` at [libs/SFM/Triangulation.cpp:257](../../libs/SFM/Triangulation.cpp#L257) calls `TriangulateSkewLLS`, which uses `UnprojectNormalized` (3D unit bearing vectors) and is singularity-free. The test gets 80/80 tracks recovered with **0.0000 m mean 3D error** and **0.00 pixel reprojection error**. G2 from the original plan was a red herring.

2. **`TriangulateDLT` is dead code** — declared in [Triangulation.h](../../libs/SFM/Triangulation.h) and defined in [Triangulation.cpp:15](../../libs/SFM/Triangulation.cpp#L15) but **never called** anywhere in the live pipeline. Phase 2 added a comment + runtime `ASSERT(img.pCamera->GetType() == CameraType::PINHOLE)` at [Triangulation.cpp:36](../../libs/SFM/Triangulation.cpp#L36) to document that it's pinhole-only, in case a future caller re-activates it.

3. **The real G1 instance is a different pattern: `Unproject(px).homogeneous()`.** This synthesizes a 3D ray `(tan θ, tan φ/cos θ, 1)` with `z` hard-coded to `+1`, which always points into the front hemisphere. For back-hemisphere observations the synthesized ray points the wrong way, so `acos(dot(observed, Xcam))` returns ~π radians. Averaged across a full-sphere scene this produces exactly the **90° mean angular error** the test caught.

4. **Angular reprojection metric is broken for spherical scenes.** `ComputeTracksMeanReprojectionError` at [libs/SFM/Track.cpp:225](../../libs/SFM/Track.cpp#L225) and `FilterTracks` at [libs/SFM/Track.cpp:278](../../libs/SFM/Track.cpp#L278) both use the `Unproject(px).homogeneous()` pattern. `FilterTracks` is the more serious one because it **rejects observations** based on the broken angular metric — so for spherical scenes, valid back-hemisphere observations get silently discarded as outliers.

## Inventory of `Camera::Unproject()` 2D-form call sites

A grep across the SFM library classified every call site of the 2D `Camera::Unproject()` form:

### Not a bug (leave alone)
- [libs/SFM/Camera.cpp:61](../../libs/SFM/Camera.cpp#L61) — `PinholeCamera::UnprojectNormalized()` internally calls `normalized(Unproject(x).homogeneous())`. This is the **correct** implementation for pinhole specifically: `Unproject` returns `(X/Z, Y/Z)` on the z=1 plane, and `(X/Z, Y/Z, 1)` normalized is the unit bearing vector.
- [libs/SFM/Triangulation.cpp:42](../../libs/SFM/Triangulation.cpp#L42) — `TriangulateDLT`, now gated pinhole-only by ASSERT.

### Group A — Single-line `.homogeneous()` bugs (safe local fixes)
These sites construct a 3D ray via `Unproject(px).homogeneous()` and feed it into an angle/dot-product computation. For pinhole the result is algebraically correct; for spherical it's front-hemisphere-biased. Replacing with `UnprojectNormalized()` fixes spherical without changing pinhole semantics.

| # | Site | Function | Notes |
|---|---|---|---|
| A1 | [libs/SFM/Track.cpp:225](../../libs/SFM/Track.cpp#L225) | `ComputeTracksMeanReprojectionError` | **Caught by the regression test.** Metric-only; doesn't affect reconstruction correctness, but every existing test that logs angular reprojection error is wrong for spherical scenes. |
| A2 | [libs/SFM/Track.cpp:278](../../libs/SFM/Track.cpp#L278) | `FilterTracks` | **Reconstruction-critical.** Uses the angular threshold to reject observations as outliers. For spherical scenes, back-hemisphere observations get discarded. |
| A3 | [libs/SFM/View.h:158](../../libs/SFM/View.h#L158) | `View::Ray(x)` | Returns a world-space ray from a pixel. Only caller is `GlobalPositioning.cpp:301` which `normalized()`s the result anyway. Dead sibling `View::RayNormalized` at line 160 is zero-callers and can be deleted. |
| A4 | [libs/SFM/ImportROMA2.cpp:599-600](../../libs/SFM/ImportROMA2.cpp#L599) | `ImportROMA2` depth computation | Uses ray cross-product for depth. Likely pinhole-focused import, but should still be fixed for consistency. |

### Group B — Multi-line refactors (feed 2D into external solvers)
These sites pass the 2D-form output into PoseLib or custom triangulators that expect pinhole-normalized-plane coordinates. Fixing requires switching to the bearing-vector solver variants and possibly adapting the downstream code. Out of Phase 2 scope — tracked for Phase 3.

| # | Site | Function | Downstream |
|---|---|---|---|
| B1 | [libs/SFM/StarInitializer.cpp:93-94](../../libs/SFM/StarInitializer.cpp#L93) | Scale averaging inner loop | `TriangulatePoint3D(..., p1Cam.homogeneous(), p2Cam.homogeneous(), ...)` |
| B2 | [libs/SFM/ImagePair.cpp:249-250](../../libs/SFM/ImagePair.cpp#L249) | `ImagePair::Triangulate` (Linear LS) | Local DLT triangulation |
| B3 | [libs/SFM/Resection.cpp:81](../../libs/SFM/Resection.cpp#L81) | PnP input | PoseLib `PinholeCameraModel` PnP |
| B4 | [libs/SFM/PairsMatcher.cpp:453-454](../../libs/SFM/PairsMatcher.cpp#L453) | Geometric verification | PoseLib relative pose / E-matrix |

### Group C — Semantics mismatch (needs API decision)
- [libs/SFM/View.h:151](../../libs/SFM/View.h#L151) — `View::UnprojectPoint(x, d)` builds `Point3(rayC.x*d, rayC.y*d, d)`. For pinhole, `d` is depth along the z-axis. For spherical, this gives a point that is **not** at distance `d` along the bearing — it's at distance `d` along the z-axis of the aliased front-facing ray. Used by the synthetic scene generator and some Image-space helpers. Fixing requires either a semantics change (d = depth along bearing) or a new overload. Out of Phase 2 scope.

## Gaps to Close (revised)

### G1 — `Unproject(px).homogeneous()` pattern aliases back-hemisphere rays onto the front hemisphere (correctness bug)
[libs/SFM/Camera.cpp:247-257](../../libs/SFM/Camera.cpp#L247) returns `(TAN(theta), TAN(phi)/COS(theta))`. Algebraically this equals `(X/Z, Y/Z)` — the standard pinhole-normalized plane — so for any 3D point in the **front hemisphere** (Z > 0) the 2D form works and even agrees with pinhole semantics. The problem is topological, not numerical: **any smooth map S² → ℝ² must have at least one singularity** (the 2-sphere is not homeomorphic to the plane). There is no division-free reformulation that returns a 2D vector and covers the whole sphere. The concrete failure modes of the current formula are:

- **Z = 0 (equator, features at longitude ±π/2)** — `cos(θ)` vanishes and the second component blows up.
- **Z < 0 (back hemisphere, longitude > ±π/2)** — `X/Z` is finite but has the wrong sign; front and back features alias onto the same 2D point, so downstream RANSAC/DLT cannot distinguish them.
- **θ = ±π (back pole of the equirectangular image)** — `tan(π)` hits the `atan2` branch cut.

**The fix is to stop asking `SphericalCamera` to implement a 2D unproject at all** and route every geometric caller through `Camera::UnprojectNormalized` (already defined on the base class, already returns a 3D unit bearing vector, already singularity-free). The existing virtual is the right API — we just have callers that still use the 2D form. No new `GetBearingVector` function needed.

Options considered and rejected:
- *Stereographic projection from the back pole*: well-defined everywhere except the back pole, still has a singularity, and changes the meaning of the return value so pinhole callers break.
- *Return raw `(θ, φ)` angles*: well-defined everywhere (the θ = ±π branch cut is periodic, not singular), but the return value is no longer "normalized plane coordinates" — pinhole callers would silently misinterpret it.
- *Delete `Camera::Unproject` entirely*: cleanest long-term design, but it's a larger refactor across every pinhole caller and their call sites in `TriangulateDLT`. Defer to a follow-up.

**Chosen approach**: keep `SphericalCamera::Unproject` as-is (front-hemisphere convenience) but add an `ASSERT` or `DEBUG_EXTRA` warning when |θ| approaches π/2, and **audit every call site and switch it to `UnprojectNormalized` when the camera might be spherical**. The 3D bearing-vector path is numerically equivalent to the 2D plane path for pinhole (just `normalized((x,y,1))` vs `(x,y)`) so changing the callers costs nothing on the pinhole side.

Confirmed callers of `Unproject` (not `UnprojectNormalized`) that must switch for spherical scenes:
- [libs/SFM/StarInitializer.cpp:93-94](../../libs/SFM/StarInitializer.cpp#L93) — `img1.pCamera->Unproject(pt1)` → feeds PoseLib's relative pose.
- [libs/SFM/PairsMatcher.cpp](../../libs/SFM/PairsMatcher.cpp) — same pattern.
- [libs/SFM/Triangulation.cpp:62,77,80-82,108-110](../../libs/SFM/Triangulation.cpp#L62) — DLT builds `A*X=0` from `pt.x * P(2,j) - P(0,j)` assuming the 2D coords are pinhole-normalized. `ProjectPoint` at line 77 is polymorphic and OK, but the DLT itself is not.

### G2 — Triangulation DLT uses pinhole projection-matrix form ~~(live gap)~~ RESOLVED: dead code, guarded by ASSERT
[libs/SFM/Triangulation.cpp:15-124](../../libs/SFM/Triangulation.cpp#L15) `TriangulateDLT` mixes the normalized pixel with the 3×4 camera projection matrix `P`. This is mathematically valid only when the input is on the pinhole normalized plane (x/z, y/z). For spherical cameras it would fail silently for features outside the front hemisphere.
**However, `TriangulateDLT` is never actually called** — the live triangulator in `TriangulateTracks` is `TriangulateSkewLLS`, which operates on bearing vectors from `UnprojectNormalized` and is singularity-free. Phase 2 added a header comment and runtime `ASSERT(GetType() == CameraType::PINHOLE)` to document the constraint in case the function is ever re-activated. No further work needed for this gap.

### G3 — MatchGeometric RANSAC threshold is pure pixels
[libs/SFM/MatchGeometric.cpp](../../libs/SFM/MatchGeometric.cpp) uses `config.maxEpipolarError` in pixels. For a 4000-pixel equirectangular image, 1 pixel ≈ 0.09°; for a 640-pixel pinhole image with focal 500, 1 pixel ≈ 0.11°. The two aren't equivalent. Need a per-camera angular threshold via `camera->PixelErrorToAngular(px)`.

### G4 — MVS downstream has no spherical camera model
[libs/SFM/InterfaceMVS.cpp:134](../../libs/SFM/InterfaceMVS.cpp#L134) `UndistortDepthMaps` explicitly bails on non-pinhole (`// only pinhole supported`). The MVS library (`libs/MVS/Camera.h`, `Scene`, `DepthMap`) has no `SphericalCamera` equivalent. The **cube-map bridge** approach is adopted: convert each spherical view to 6 virtual pinhole cube-map faces before exporting to `.mvs` format.

### G5 — No spherical test coverage
[apps/Tests/TestsSFM.cpp](../../apps/Tests/TestsSFM.cpp) has dozens of pinhole tests but none that exercise the spherical path. The scene generator supports `CameraSpec::SPHERICAL` but is never invoked that way.

### G6 — Documentation gap
Users have no guide for how to run SFM on 360° photos or 360° video.

## Critical Files (what changes where)

### Fix G1 (Unproject divergence)
No new API is needed — `Camera::UnprojectNormalized` already returns a 3D unit bearing vector for both `PinholeCamera` and `SphericalCamera`. The fix is to route every geometric caller through it. For pinhole cameras the 2D and 3D forms are algebraically equivalent (`(x, y)` vs `normalized((x, y, 1))`), so switching has no numerical cost on the pinhole side. Concrete changes:

- **[libs/SFM/Camera.cpp:247-257](../../libs/SFM/Camera.cpp#L247)** — `SphericalCamera::Unproject`: add an `ASSERT` (or `DEBUG_EXTRA` warning) when the longitude is near the equator-back singularity, and leave a short comment explaining that the 2D form is a front-hemisphere convenience; callers that need whole-sphere correctness must use `UnprojectNormalized`.
- **[libs/SFM/StarInitializer.cpp:91-94](../../libs/SFM/StarInitializer.cpp#L91)** — switch to `UnprojectNormalized()`. Then verify PoseLib's relative-pose entry point accepts 3D bearing vectors; if the current call path goes through a 2D normalized-plane input, switch to PoseLib's bearing-vector solver (e.g. `relpose_5pt` variant that takes unit vectors) for spherical pairs. Keep the 2D PoseLib path for pinhole-pinhole pairs if it's meaningfully faster; otherwise unify both to the 3D path.
- **[libs/SFM/PairsMatcher.cpp](../../libs/SFM/PairsMatcher.cpp)** — audit every `Unproject()` call site; switch to `UnprojectNormalized()` for spherical pairs.
- **[libs/SFM/Resection.cpp:96-100](../../libs/SFM/Resection.cpp#L96)** — currently passes "normalized coordinates" to PoseLib's `PinholeCameraModel`. For spherical images this breaks on back-facing features. Switch the PnP 2D→3D input to bearing vectors via `UnprojectNormalized` and use PoseLib's bearing-vector absolute-pose entry point for spherical cameras.

### Fix G2 (bearing-vector DLT)
- **[libs/SFM/Triangulation.cpp:15-124](../../libs/SFM/Triangulation.cpp#L15)** — add a branch: if any observation's camera is spherical, use `TriangulateSkewLLS` (lines 126+) which already operates on normalized rays, OR implement a cross-product DLT: for each view stack `[r]_× (R X + t) = 0` where `r` is the unit bearing vector from `UnprojectNormalized`. Simpler choice: for spherical observations, call `TriangulateSkewLLS` directly from `TriangulateTracks` and keep `TriangulateDLT` as the pinhole-only fast path.
- Reprojection error at [:81-82](../../libs/SFM/Triangulation.cpp#L81) stays in pixels (both camera types produce pixel-space projections), but the threshold interpretation is image-resolution-dependent — add a camera-aware helper `img.pCamera->PixelErrorToAngular(reprojThreshold)` for logging/comparison. Acceptable compromise for v1: leave the threshold as-is; document that for high-res equirectangular images the threshold should be scaled up proportionally.

### Fix G3 (angular RANSAC)
- **[libs/SFM/MatchGeometric.cpp](../../libs/SFM/MatchGeometric.cpp)** — inside the RANSAC inlier evaluation, convert `config.maxEpipolarError` from pixels to radians via `PixelErrorToAngular` of each camera, and compare against the angular distance between the observed bearing vector and the epipolar great circle for spherical pairs. For pinhole-pinhole pairs, keep the existing pixel path.

### Fix G4 (cube-map bridge)
- **New file: [libs/SFM/CubeMapBridge.h](../../libs/SFM/) + .cpp** — public API:
  ```cpp
  namespace SFM {
      // Expand a scene so that every spherical image is replaced by 6 virtual
      // pinhole cube-map faces (+X, -X, +Y, -Y, +Z, -Z) sharing the same camera
      // center but with rotated poses. Features and tracks are re-projected to
      // the appropriate face. Returns a new Scene; original is unchanged.
      bool ExpandSphericalToCubeMap(const Scene& sphericalScene, Scene& cubeMapScene,
                                    int faceSize = 1024);
  }
  ```
  Mechanics:
  1. For each spherical `Image`, render 6 virtual `PinholeCamera` faces with 90° FOV (`fx = fy = faceSize/2`, `cx = cy = faceSize/2`), each face pose being the original pose composed with a fixed rotation (identity, Rx(90°), Rx(-90°), Ry(90°), Ry(-90°), Rz(180°)).
  2. Write each face as a PNG/JPG to disk (sampling the equirectangular image with the existing `TImage::sample` bilinear helper described in `CLAUDE.md`).
  3. For every track observation whose original image was spherical, reproject the 3D point via each of the 6 virtual pinhole cameras and keep the face where the projection is valid and inside the image bounds. Create new observations with (face_imageID, new_featureID) pointing at the matching keypoint in the face.
  4. Preserve GPS/EXIF metadata on each face.
- **[libs/SFM/InterfaceMVS.cpp](../../libs/SFM/InterfaceMVS.cpp)** — in the SFM→MVS export entry (likely `ExportScene` or similar; scan for the function that writes `.mvs`), if the scene contains any spherical cameras, call `ExpandSphericalToCubeMap` into a temp scene first, then serialize that. Add a CLI flag to skip the expansion for callers who want raw spherical output.
- **[apps/InterfaceCOLMAP](../../apps/InterfaceCOLMAP)** etc. — unaffected for v1; document that COLMAP interchange doesn't round-trip spherical.

### Keyframe extraction for spherical video (G6 partial — already mostly wired)
- Verify [apps/ExtractKeyframes/ExtractKeyframes.cpp:119,227](../../apps/ExtractKeyframes/ExtractKeyframes.cpp#L119) end-to-end by extracting keyframes from a spherical video with `--camera-type 1`. No code changes expected here unless the test finds issues.
- Add a docs page showing the CLI usage.

### Tests (G5)
- **[apps/Tests/TestsSFM.cpp](../../apps/Tests/TestsSFM.cpp)** — add a new test `ReconstructSphericalSyntheticTest` that:
  1. Uses the existing scene generator at line ~249 with `CameraSpec::type = SPHERICAL`, `width = 2048`, `height = 1024`, 6–8 viewpoints around a cluster of ground-truth 3D points with **good coverage of all hemispheres** (place points at ±X, ±Y, ±Z around each camera center so features map to all regions of the equirectangular image — this is the test that catches G1).
  2. Exercises: synthetic observations → tracks → `StarInitializer` → `Resection` → `Triangulation` → global BA → compare reconstructed poses/points to ground truth within an angular tolerance.
  3. Asserts reprojection error < 1 pixel across all observations, including features with |longitude| > π/2.
- **[apps/Tests/TestsSFM.cpp](../../apps/Tests/TestsSFM.cpp)** — add `BundleAdjustmentSphericalTest` that constructs a minimal 2-view spherical scene, perturbs the poses, runs BA, and checks convergence.
- **[apps/Tests/TestsSFM.cpp](../../apps/Tests/TestsSFM.cpp)** — add `CubeMapBridgeTest` that takes a synthetic spherical scene, runs `ExpandSphericalToCubeMap`, verifies that every original track observation appears in at least one face observation, and that the total reprojection error across the cube-map scene matches the original spherical scene.
- **Real dataset** — provided separately. Plan a manual smoke-test script (shell) that runs: `ExtractKeyframes --camera-type 1` → feature extraction → SFM reconstruct → cube-map export → MVS densify → mesh. No automated assertions for the real data; visual inspection only.

### Documentation
- **New file: [docs/spherical_camera_workflow.md](../spherical_camera_workflow.md)** — usage guide covering EXIF-based auto-detection, video keyframe extraction with `--camera-type 1`, the cube-map export step for MVS, current limitations (no COLMAP spherical round-trip, no native MVS spherical depth maps), and references to the relevant library entry points.
- **Update [libs/SFM/CLAUDE.md](../../libs/SFM/CLAUDE.md)** — add a short "Spherical camera notes" section summarizing the polymorphic entry points and the cube-map bridge.

## Existing Functions to Reuse (no new code needed)

- `SphericalCamera::Project` / `UnprojectNormalized` / `PixelErrorToAngular` — [libs/SFM/Camera.cpp:216-278](../../libs/SFM/Camera.cpp#L216).
- `SphericalAngularReprojectionError::Create` — [libs/SFM/BundleAdjustmentCostFunctions.h:408](../../libs/SFM/BundleAdjustmentCostFunctions.h#L408).
- `TriangulateSkewLLS` — [libs/SFM/Triangulation.cpp:126](../../libs/SFM/Triangulation.cpp#L126) — already ray-based, reuse for spherical.
- `TImage::sample` with `Sampler::Linear` — documented in [CLAUDE.md](../../CLAUDE.md) — for cube-map face rasterization.
- `TImage::isInsideWithBorder` — for cube-map face bounds checking.
- `Pose3D` composition operators — for composing face rotations with the original spherical pose.
- Scene generator in [apps/Tests/TestsSFM.cpp:249-400](../../apps/Tests/TestsSFM.cpp#L249) — already supports `SphericalCamera`; just instantiate with the right `CameraSpec`.
- `ExtractFromVideo` — [libs/SFM/KeyframeExtractor.cpp:412-680](../../libs/SFM/KeyframeExtractor.cpp#L412) — already dispatches on `cameraType`.

## Implementation Order (updated after Phase 1)

1. **Phase 1 — Write the failing test first.** ✅ Done. `ReconstructSphericalSyntheticTest` exists and fails on the angular reprojection assertion with a 90° mean error. It also uncovered that G2 is dead code.
2. **Phase 2 — Close Group A (`.homogeneous()` single-line fixes).** Four sites: A1 `ComputeTracksMeanReprojectionError`, A2 `FilterTracks`, A3 `View::Ray`, A4 `ImportROMA2`. All replace `Unproject(px).homogeneous()` with `UnprojectNormalized(px)`. Each fix is presented to the user before editing. After all four, re-run `ReconstructSphericalSyntheticTest` — A1 should turn the test green.
3. **Phase 3 — Close Group B (multi-line refactors).** StarInitializer, ImagePair, Resection, PairsMatcher — switch their PoseLib / solver input from 2D-normalized to 3D-bearing and verify with an expanded integration test. This is the biggest unit of work.
4. **Phase 4 — Fix G3 (angular RANSAC).** Convert MatchGeometric thresholds per-camera.
5. **Phase 5 — Cube-map bridge (G4).** Implement `CubeMapBridge.h/.cpp` + `CubeMapBridgeTest`. Wire into `InterfaceMVS` export path.
6. **Phase 6 — Verify keyframe video path end-to-end.** Run a manual smoke test with a sample spherical video.
7. **Phase 7 — Docs.** Write `docs/spherical_camera_workflow.md` and update `libs/SFM/CLAUDE.md`.

## Verification

End-to-end checks that must pass before claiming completion:

- `cd make && cmake --build . -j4 && ctest -R Spherical` — runs the new synthetic tests; all pass.
- `./bin/Debug/Tests` — all pre-existing pinhole tests still pass (no regression in the pinhole path).
- `./bin/Debug/ExtractKeyframes --input sample360.mp4 --camera-type 1 --output-dir keyframes/` — produces equirectangular keyframes with a `SphericalCamera` in the output scene; load the resulting `.mvs` in the `Viewer` and confirm the camera is rendered as a sphere/360 placeholder.
- Synthetic dataset end-to-end: generate a synthetic spherical scene, run SFM, compare reconstructed camera centers to ground truth with translation error < 1% of scene diameter and rotation error < 0.5°.
- Real dataset: user-provided 360° capture runs through `ExtractKeyframes → SFM → cube-map bridge → DensifyPointCloud → ReconstructMesh → TextureMesh` without errors. Inspect the mesh in `Viewer`.
- Open `docs/spherical_camera_workflow.md` and follow the instructions verbatim to reproduce the real-dataset run.

## Out of Scope (explicit)

- Native `SphericalCamera` in the MVS library (PatchMatch, MRF, texturing). The cube-map bridge is the sanctioned workaround for v1.
- COLMAP spherical round-trip (COLMAP has no native spherical model).
- Fisheye / Kannala-Brandt / OpenCV omnidir camera models — unrelated to this task.
- Cube-map-aware feature detection (e.g., rendering tangent images before SIFT) to improve pole regions. v1 runs SIFT/AKAZE directly on the equirectangular image and accepts degraded match density near the poles.
