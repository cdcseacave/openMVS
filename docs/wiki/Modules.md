## Keyframe Extraction

The pipeline can ingest video directly. The keyframe extraction module, exposed as the `ExtractKeyframes` app, walks a video file and selects a well-spaced, motion-blur-free subset of frames suitable for Structure-from-Motion. Frame selection is driven by feature overlap: for each incoming frame, features are matched against the last accepted keyframe and the pair is geometrically verified; a new keyframe is committed when the matched-feature overlap drops below a configurable threshold (0.85 by default). Optional Gaussian blurring on the optical-flow-tracked features rejects motion-blurred frames. The output is a `.sfm` scene containing the accepted images together with an initial intrinsic calibration, ready to be passed to `CreateStructure`.

Equirectangular (360°) video is supported natively: setting `--camera-type 1` switches the internal feature extractor to a tangent-pinhole cube-map of the spherical frame (4, 6, 8, 12 or 20 faces, see `--cubemap-faces`), and the resulting scene is tagged with a `SphericalCamera` model so downstream stages treat the images as equirectangular panoramas.

## Structure-from-Motion

The SfM module, implemented in [libs/SFM](https://github.com/cdcseacave/openMVS/blob/master/libs/SFM) and exposed as the `CreateStructure` app, performs a full incremental reconstruction from an unordered set of images. It includes feature detection (SIFT / AKAZE / ORB / SIFTGPU), pairwise matching (exhaustive, vocabulary-tree or sequential), geometric verification, incremental pose estimation and triangulation, followed by Ceres-based bundle adjustment. The reconstruction can be seeded from a CSV file of known camera poses (full poses, extrinsics only or positions only) and its result can be exported either as a native `.sfm` file or directly as a `.mvs` project for the downstream dense-reconstruction step.

Scenes with GPS metadata can be rigidly aligned to a metric ENU (East/North/Up) frame and optionally refined with GPS position priors in a final bundle adjustment (`--gps-position-weight`). The pose covariance of the last bundle adjustment can be recorded per image and exported as a quality report (`--export-pose-quality`) — 1-sigma position and rotation accuracy per camera, absolute ENU meters when GPS priors are used — which the `Viewer` renders as per-camera error ellipsoids (see [design/PoseUncertainty.md](https://github.com/cdcseacave/openMVS/blob/master/docs/design/PoseUncertainty.md)).

Both pinhole and spherical (equirectangular) cameras are first-class camera models. For spherical images, matching skips the fundamental-matrix epipolar check (the epipolar geometry degenerates for 360° rays) and bundle adjustment uses an angular reprojection cost scaled to pixel-equivalent residuals.

## Spherical Cameras

OpenMVS models a spherical camera as an equirectangular image whose width is exactly twice its height, covering 360° horizontally and 180° vertically. The `SphericalCamera` class in [libs/SFM/Camera.h](https://github.com/cdcseacave/openMVS/blob/master/libs/SFM/Camera.h) handles projection (longitude/latitude → pixel) and back-projection (pixel → unit bearing vector) without a calibration matrix; `K` is the identity. Panoramic images tagged with EXIF `ProjectionType=equirectangular` are auto-detected on load.

Because the downstream MVS modules (`DensifyPointCloud`, `ReconstructMesh`, `RefineMesh`, `TextureMesh`) operate natively on pinhole views, spherical images are internally converted to a six-face cube-map of virtual pinhole views via [libs/SFM/SphereCubeMap.h](https://github.com/cdcseacave/openMVS/blob/master/libs/SFM/SphereCubeMap.h). This bridge is transparent to the user — the cube-map faces carry the correct relative poses and intrinsics and are stitched back into a single reconstruction.

## Dense Point-Cloud Reconstruction

The goal of this module is to provide the functionality of obtaining a complete and accurate as possible point-cloud at reasonable speeds. Since the final goal is to obtain a mesh representation, and since there is a module to refine the mesh, the completeness and speed of estimating the dense point-cloud is more important than the accuracy. Therefore, the current implementation is based on the Patch-Match algorithm: *PatchMatch: A Randomized Correspondence Algorithm for Structural Image Editing* C. Barnes et al. 2009.

A second option for estimating the dense point-cloud is using Semi-Global Matching algorithm, implemented as described in: *Memory Efficient Semi-Global Matching* H. Hirschmüller et al. 2012. This method is still experimental, thus sometimes the speed and completeness might not be as good as the Path-Match approach, though the accuracy could be better.

### Depth-Map Confidence

Every depth estimate carries a confidence in `[0,1]`, stored in the `.dmap` files next to the depth and used to weight fusion, to order points, and to drive the visibility weights of the mesh step. By default this starts as a photometric score (`1 - NCC`), which answers *"how well does this patch match?"* — a question that is only loosely related to the one that actually matters downstream, *"is this depth correct?"*. A patch on a repetitive facade or in a textureless region can match beautifully and still be wrong.

The optional recalibration replaces that photometric score with a posterior that predicts **whether a depth will survive fusion as an inlier**, combining three sources of evidence per pixel:

- **an intra-map geometric prior** — a local plane is fitted to the depth-map around the pixel, and the pixel is scored by how well its neighbourhood agrees with that plane and by whether the plane's implied normal agrees with the estimated normal. A correct surface is locally coherent in both; a photometric mismatch usually is not;
- **multi-view confirmation** — the pixel is projected into each neighbouring view and compared against that view's own depth estimate, through continuous (rather than pass/fail) agreement weights on depth, forward-backward reprojection, surface normal and the neighbour's own confidence. Every neighbour contributes a fractional vote, so agreement degrades smoothly instead of falling off a threshold;
- **free-space violations** — when a neighbour's own measured depth lies well *behind* our point along the same ray, that neighbour's line of sight passes *through* where we claim a surface is. This is direct negative evidence, and is counted separately from mere occlusion (a neighbour seeing something closer says nothing about our point).

These are combined in a closed form (see [libs/MVS/ConfidenceRefine.h](https://github.com/cdcseacave/openMVS/blob/master/libs/MVS/ConfidenceRefine.h)): a Beta-style posterior mean whose evidence is diluted by the violation count, gated by the total confirmation weight and scaled by the photometric term. Its shape constants are not exposed as options — they are a single operating point calibrated jointly against ground truth, and moving one without re-deriving the others degrades the result.

**Expected accuracy.** Measured against ground-truth depth on 28 scene-levels of BlendedMVS and ETH3D, the recalibration raises the pooled inlier/outlier ROC-AUC from **0.844 to 0.926**, improving every scene-level tested. The practical consequence is on the completeness/contamination trade-off: thresholding confidence to admit at most 1% contaminated points retains **57.9%** of the depths versus **31.5%** with the raw photometric score — close to twice the usable surface at the same error budget.

**Cost and defaults.** When CUDA estimates the depth-maps, the recalibration runs fused into the last geometric-consistency iteration, reading the depth, normal and cost buffers already resident on the device — about 3 ms per depth-map, which is why it is **enabled by default on GPU**. On the CPU there is no such free ride: it costs a separate full-resolution sweep comparable to a fusion pass, so it is **off by default** and enabled explicitly with `--postprocess-dmaps 8`. Recalibrated depth-maps are flagged in the `.dmap` header so a second pass never adjusts an already-adjusted map.

## Mesh Reconstruction

This module aims at estimating a mesh surface that explains the best the input point-cloud, and to be robust to outliers. The input point-cloud could be dense or sparse, and hence the algorithm used should be able to perform well in both cases. For these reasons, the algorithm currently implemented is based on the paper: *Exploiting Visibility Information in Surface Reconstruction to Preserve Weakly Supported Surfaces* M. Jancosek et al. 2014.

## Mesh Refinement

Rough meshes obtained by the previous module are in general a good enough starting point for a variational refinement step. Such algorithms are relatively fast and able to recover the true surface even in cases when only a coarse input mesh is provided (as in the case of meshes estimated from a sparse point-cloud, or texture-less scenes). The algorithm employed for solving this task is based on the paper: *High Accuracy and Visibility-Consistent Dense Multiview Stereo* HH. Vu et al. 2012.

## Mesh Texturing

In the case of having a perfect mesh reconstruction and ground-truth camera poses, obtaining the texture is relatively a straight-forward step. In reality however both the mesh and the camera poses contain slight variations/errors at best, and hence the mesh texturing module should be able to cope with them. A very good paper describing such an algorithm, implemented in *OpenMVS*, is: *Let There Be Color! - Large-Scale Texturing of 3D Reconstructions* M. Waechter et al. 2014.