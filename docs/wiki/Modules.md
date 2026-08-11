## Keyframe Extraction

The pipeline can ingest video directly. The keyframe extraction module, exposed as the `ExtractKeyframes` app, walks a video file and selects a well-spaced, motion-blur-free subset of frames suitable for Structure-from-Motion. Frame selection is driven by feature overlap: for each incoming frame, features are matched against the last accepted keyframe and the pair is geometrically verified; a new keyframe is committed when the matched-feature overlap drops below a configurable threshold (0.85 by default). Optional Gaussian blurring on the optical-flow-tracked features rejects motion-blurred frames. The output is a `.sfm` scene containing the accepted images together with an initial intrinsic calibration, ready to be passed to `CreateStructure`.

Equirectangular (360°) video is supported natively: setting `--camera-type 1` switches the internal feature extractor to a tangent-pinhole cube-map of the spherical frame (4, 6, 8, 12 or 20 faces, see `--cubemap-faces`), and the resulting scene is tagged with a `SphericalCamera` model so downstream stages treat the images as equirectangular panoramas.

## Structure-from-Motion

The SfM module, implemented in [libs/SFM](https://github.com/cdcseacave/openMVS/blob/master/libs/SFM) and exposed as the `CreateStructure` app, performs a full incremental reconstruction from an unordered set of images. It includes feature detection (SIFT / AKAZE / ORB / SIFTGPU), pairwise matching (exhaustive, vocabulary-tree, sequential or pose-guided), geometric verification, incremental pose estimation and triangulation, followed by Ceres-based bundle adjustment. The reconstruction can also be run as a *finetune* of already-known camera poses, imported from an OpenMVS pose CSV or a Polycam-style `frames.json`, with optional intrinsics: the known poses replace the incremental pose search, the pairs to match are selected from them geometrically, and the refined result is aligned back to the input coordinate frame. The result can be exported either as a native `.sfm` file or directly as a `.mvs` project for the downstream dense-reconstruction step.

Scenes with GPS metadata can be rigidly aligned to a metric ENU (East/North/Up) frame and optionally refined with GPS position priors in a final bundle adjustment (`--gps-position-weight`). The pose covariance of the last bundle adjustment can be recorded per image and exported as a quality report (`--export-pose-quality`) — 1-sigma position and rotation accuracy per camera, absolute ENU meters when GPS priors are used — which the `Viewer` renders as per-camera error ellipsoids (see [design/PoseUncertainty.md](https://github.com/cdcseacave/openMVS/blob/master/docs/design/PoseUncertainty.md)).

Both pinhole and spherical (equirectangular) cameras are first-class camera models. For spherical images, matching skips the fundamental-matrix epipolar check (the epipolar geometry degenerates for 360° rays) and bundle adjustment uses an angular reprojection cost scaled to pixel-equivalent residuals.

## Spherical Cameras

OpenMVS models a spherical camera as an equirectangular image whose width is exactly twice its height, covering 360° horizontally and 180° vertically. The `SphericalCamera` class in [libs/SFM/Camera.h](https://github.com/cdcseacave/openMVS/blob/master/libs/SFM/Camera.h) handles projection (longitude/latitude → pixel) and back-projection (pixel → unit bearing vector) without a calibration matrix; `K` is the identity. Panoramic images tagged with EXIF `ProjectionType=equirectangular` are auto-detected on load.

Because the downstream MVS modules (`DensifyPointCloud`, `ReconstructMesh`, `RefineMesh`, `TextureMesh`) operate natively on pinhole views, spherical images are internally converted to a six-face cube-map of virtual pinhole views via [libs/SFM/SphereCubeMap.h](https://github.com/cdcseacave/openMVS/blob/master/libs/SFM/SphereCubeMap.h). This bridge is transparent to the user — the cube-map faces carry the correct relative poses and intrinsics and are stitched back into a single reconstruction.

## Dense Point-Cloud Reconstruction

The goal of this module is to provide the functionality of obtaining a complete and accurate as possible point-cloud at reasonable speeds. Since the final goal is to obtain a mesh representation, and since there is a module to refine the mesh, the completeness and speed of estimating the dense point-cloud is more important than the accuracy. Therefore, the current implementation is based on the Patch-Match algorithm: *PatchMatch: A Randomized Correspondence Algorithm for Structural Image Editing* C. Barnes et al. 2009.

A second option for estimating the dense point-cloud is using Semi-Global Matching algorithm, implemented as described in: *Memory Efficient Semi-Global Matching* H. Hirschmüller et al. 2012. This method is still experimental, thus sometimes the speed and completeness might not be as good as the Path-Match approach, though the accuracy could be better.

## Mesh Reconstruction

This module aims at estimating a mesh surface that explains the best the input point-cloud, and to be robust to outliers. The input point-cloud could be dense or sparse, and hence the algorithm used should be able to perform well in both cases. For these reasons, the algorithm currently implemented is based on the paper: *Exploiting Visibility Information in Surface Reconstruction to Preserve Weakly Supported Surfaces* M. Jancosek et al. 2014.

## Mesh Refinement

Rough meshes obtained by the previous module are in general a good enough starting point for a variational refinement step. Such algorithms are relatively fast and able to recover the true surface even in cases when only a coarse input mesh is provided (as in the case of meshes estimated from a sparse point-cloud, or texture-less scenes). The algorithm employed for solving this task is based on the paper: *High Accuracy and Visibility-Consistent Dense Multiview Stereo* HH. Vu et al. 2012.

## Mesh Texturing

In the case of having a perfect mesh reconstruction and ground-truth camera poses, obtaining the texture is relatively a straight-forward step. In reality however both the mesh and the camera poses contain slight variations/errors at best, and hence the mesh texturing module should be able to cope with them. A very good paper describing such an algorithm, implemented in *OpenMVS*, is: *Let There Be Color! - Large-Scale Texturing of 3D Reconstructions* M. Waechter et al. 2014.
