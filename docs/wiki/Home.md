![logo](https://github.com/cdcseacave/openMVS/blob/master/docs/assets/logo.png)

# OpenMVS
**open Multi-View Stereo reconstruction library**

## Introduction

[OpenMVS (Multi-View Stereo)](https://github.com/cdcseacave/openMVS) is a library for computer-vision scientists, targeted to the photogrammetry and Multi-View Stereo reconstruction community. It provides a complete end-to-end pipeline that takes a set of images (or a video) and produces a textured 3D mesh: a native Structure-from-Motion module recovers the camera poses and a sparse point-cloud, and the downstream MVS modules densify, mesh, refine and texture the scene. *OpenMVS* remains fully interoperable with external SfM solutions — projects calibrated by [OpenMVG](https://github.com/openMVG/openMVG), [COLMAP](https://colmap.github.io), Agisoft *Metashape* / Bentley *iTwin Capture Modeler* and [Polycam](https://poly.cam) can be imported directly. The main topics covered by this project are:

- **video keyframe extraction** for selecting a stable, well-spaced subset of frames (including 360° / spherical video) suitable for reconstruction
- **Structure-from-Motion** for recovering camera poses and a sparse 3D point-cloud from unordered images, with native support for both pinhole and **spherical (equirectangular 360°)** cameras
- **dense point-cloud reconstruction** for obtaining a complete and accurate as possible point-cloud
- **mesh reconstruction** for estimating a mesh surface that explains the best the input point-cloud
- **mesh refinement** for recovering all fine details
- **mesh texturing** for computing a sharp and accurate texture to color the mesh

## Build

See the [[building|Building]] page.

## License

See the [copyright](https://github.com/cdcseacave/openMVS/blob/master/COPYRIGHT.md) file.

## Citation

If you use this project for your research, please cite:

```
@Unpublished{openmvs2020,
    author = {Cernea, Dan},
    title = {{OpenMVS}: Multi-View Stereo Reconstruction Library},
    year = {2020},
    url = {https://cdcseacave.github.io}
}
```

## Contact

openmvs[AT]googlegroups.com
