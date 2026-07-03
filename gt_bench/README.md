# gt_bench — real-GT evaluation benchmark for depth-map confidence and fusion

Scripts and scene lists for evaluating OpenMVS depth-map confidence adjustment and
depth-map fusion against real ground truth (BlendedMVS rendered depth maps + textured
meshes, ETH3D laser scans). Only scripts/lists/docs live in the repo; **all data stays
on the large mount** — never under the repository or the root filesystem.

## Data root

```
/home/ubuntu/virginia/gt_bench/
├── blendedmvs/            # BlendedMVS scenes (one dir per 24-hex-char scene ID)
│   ├── <sceneID>/
│   │   ├── blended_images/          # 00000000.jpg + 00000000_masked.jpg, 768x576
│   │   ├── cams/                    # 00000000_cam.txt ... + pair.txt (MVSNet convention)
│   │   ├── rendered_depth_maps/     # 00000000.pfm ... GT depth (Pf, little-endian, bottom-up)
│   │   └── textured_mesh/           # GT mesh: tiled OBJ (tile_<r>_<c>.obj + .mtl + *_tex0.jpg)
│   ├── _dl/                         # low-res archive parts + combined full.zip (all 113 scenes)
│   └── _dl_mesh/                    # textured_meshes.zip (all 113 scenes)
├── eth3d/                 # ETH3D high-res multi-view scenes (Task 2)
│   ├── <scene>/
│   │   ├── images/dslr_images_undistorted/          # <ImgName>.JPG, undistorted, per-camera resolution
│   │   ├── masks_for_images/dslr_images/            # <ImgName>.png binary masks, same count as images
│   │   ├── dslr_calibration_undistorted/            # cameras.txt, images.txt, points3D.txt (COLMAP text format)
│   │   ├── ground_truth_depth/dslr_images/          # <ImgName>.JPG **misnamed** — actually raw 4-byte-float
│   │   │                                             #   binary depth dumps, row-major, infinity = no depth
│   │   ├── occlusion/                               # splats.ply, surface_mesh.ply (occlusion handling for
│   │   │                                             #   re-rendering depth/normals from the laser scan)
│   │   └── dslr_scan_eval/                          # scan1.ply [scan2.ply ...] + scan_alignment.mlp
│   │                                                 #   (per-scan 4x4 global_T_mesh transforms)
│   └── _dl/                          # the 24 downloaded *.7z archives (4 kinds x 6 scenes)
├── tools/
│   └── multi-view-evaluation/        # ETH3D/multi-view-evaluation, built (build/ETH3DMultiViewEvaluation);
│                                      #   not committed to the repo, rebuild from GitHub if this dir is gone
├── runs/                  # imported scene.mvs + per-resolution work dirs (Task 3+)
└── results/               # evaluation JSONs (Task 7+)
```

Verified per-scene invariant: `N images == N *_cam.txt == N *.pfm` (masked images are a
second copy per view, not counted).

### Camera file format (`cams/*_cam.txt`, MVSNet convention)

`extrinsic` = 4x4 world-to-camera matrix; `intrinsic` = 3x3 K at the 768x576 image
resolution; final line = `depth_min depth_interval depth_num depth_max`.
`pair.txt`: first line = #views, then per view: view id, then `k id1 score1 id2 score2 ...`.

### Textured mesh (GT for fusion eval, Task 6)

Path: `blendedmvs/<sceneID>/textured_mesh/tile_*_*.obj` — a scene is TILED into many OBJ
files (16–255 tiles across the validation scenes), each with a `.mtl` and `*_tex0.jpg`
texture. `mesh_list.txt` enumerates the tiles (with the original authors' absolute paths —
use the basenames only). A GT-mesh loader must concatenate all `tile_*_*.obj` vertices and
faces; geometry-only eval can ignore `.mtl`/textures.

### ETH3D ground-truth depth maps pair with DISTORTED images, not undistorted (Task 2)

Confirmed from ETH3D's own documentation page (https://www.eth3d.net/documentation,
"Training data" section): *"The rendered depth maps ... contain depth images that match
the original (distorted) versions of the images, not the pre-undistorted ones."* Verified
empirically too: `courtyard/ground_truth_depth/dslr_images/DSC_0286.JPG` is 97,542,144
bytes = 6048×4032×4 (float32), which does not match any of the four undistorted
per-camera resolutions in `dslr_calibration_undistorted/cameras.txt` (e.g. camera 1 is
6205×4135) — 6048×4032 is the original DSLR sensor resolution before undistortion.
**Consequence:** GT depth maps cannot be directly diffed against depth maps OpenMVS
computes from the undistorted images/calibration in `dslr_calibration_undistorted/`
without either re-distorting OpenMVS's output or re-projecting the GT through the known
distortion model first. The `dslr_raw`/`dslr_jpg` archives (not downloaded here) hold the
matching distorted images if a depth-map-level comparison is ever needed; this benchmark
only downloaded `dslr_undistorted` (for reconstruction) + `dslr_depth` (for reference) +
`dslr_scan_eval` + `dslr_occlusion`. Point-cloud/mesh evaluation via
`ETH3DMultiViewEvaluation` (Task 6) is unaffected by this — it consumes the laser-scan
PLYs, not the depth maps.

### ETH3D `ETH3DMultiViewEvaluation` build notes and coordinate frames (Task 2)

`gt_bench/tools/multi-view-evaluation/CMakeLists.txt` hardcodes `-std=c++11`, which fails
to compile against PCL 1.14's headers (generic lambdas need C++14+) on this box's PCL/Ubuntu
version. Fixed locally (not upstream) by changing that line to `-std=c++17`; redo this if
the `tools/` dir is ever deleted and re-cloned. Also needed `sudo apt-get install -y
libboost-all-dev libpcl-dev` beyond `libeigen3-dev` (already present) — cmake's Boost
config search fails without the full boost-all-dev metapackage.

The tool applies each scan's `global_T_mesh` transform (from `scan_alignment.mlp`) only to
the *ground-truth* point clouds it loads — **not** to `--reconstruction_ply_path`, which is
read as-is. This is correct for real use: a reconstruction produced from
`dslr_calibration_undistorted` camera poses is already in the same global/aligned frame
that `scan_alignment.mlp` registers the raw laser scans into. It only matters as a trap for
self-tests: comparing a raw scan `.ply` (in the laser scanner's own local frame) directly
against the full `scan_alignment.mlp` (which transforms that same scan into the global
frame) gives near-zero completeness/accuracy — looks broken but isn't. The smoke test used
instead: a custom single-scan `.mlp` with an **identity** transform pointed at
`courtyard/dslr_scan_eval/scan1.ply`, evaluated against raw `scan1.ply` as the
reconstruction — gives completeness = accuracy = F1 = 1.0 exactly.

## Selected scenes

`scenes_blendedmvs.txt` — 5 scenes chosen from the 7-scene community validation split
(`project_lists/validation_list.txt` in the BlendedMVS repo) after visual inspection of
`blended_images`, maximizing category diversity: aerial/large, sculpture/statue (drone
monument + close-range fountain), building facades (backgrounds of both sculpture scenes
plus the aerial roofs — the validation split has no dedicated facade-only scene), indoor,
and small object. Excluded: `5b950c71608de421b1e7318f` (indoor sneaker — duplicates the
indoor small-object slot) and `5c189f2326173c3a09ed7ef3` (studio turntable figurine —
duplicates small object, atypical capture).

`scenes_eth3d.txt` (Task 2) — all 6 scenes ETH3D's high-res multi-view "training" split
offers ground truth for that fit the brief's scene list (`courtyard facade meadow office
delivery_area pipes`); no exclusions needed. 3 outdoor (courtyard, facade, meadow) + 3
indoor (office, delivery_area, pipes), 14–76 images/scene. The other 7 training scenes
(`electro kicker playground relief relief_2 terrace terrains`) and all "test" split
scenes (no public GT) were left out per the brief's explicit scene list.

## Download sources (used 2026-07-03)

- BlendedMVS low-res set (27.5 GB, all 113 scenes): GitHub release mirror —
  `https://github.com/YoYo000/BlendedMVS/releases/download/v1.0.0/BlendedMVS.z01..z15 + BlendedMVS.zip`
  (split zip; recombine with `zip -s 0 BlendedMVS.zip --out full.zip`). Archive paths are
  prefixed `BlendedMVS/<sceneID>/...`.
- Textured meshes (9.42 GB, all 113 scenes): OneDrive link from the BlendedMVS README,
  fetched headless via the `onedrivedownloader` pip package (plain `curl`/`wget` on the
  share URL gets 403/HTML). Archive paths are prefixed `dataset_textured_meshes/<sceneID>/`.
- ETH3D (Task 2): per-scene `.7z` archives from `https://www.eth3d.net/data/<scene>_<kind>.7z`
  (confirmed exact names via https://www.eth3d.net/datasets, "High-res multi-view training
  data" section) — 4 kinds per scene: `dslr_undistorted` (images + undistorted calibration),
  `dslr_depth` (rendered GT depth, distorted-image space, see below), `dslr_scan_eval`
  (laser-scan PLYs + `scan_alignment.mlp` for `ETH3DMultiViewEvaluation`), `dslr_occlusion`
  (splat/mesh + masks for occlusion-aware re-rendering). 24 archives, 6.4 GB compressed,
  33 GB extracted. `ETH3DMultiViewEvaluation` built from
  `https://github.com/ETH3D/multi-view-evaluation` (see build notes above).

Only the selected scenes are extracted; the full BlendedMVS archives are kept in `_dl`/
`_dl_mesh`, and the ETH3D `.7z` archives in `eth3d/_dl/`, so more scenes/kinds can be added
without re-downloading.

## Licenses

- **BlendedMVS / BlendedMVG**: Creative Commons Attribution 4.0 International (CC-BY-4.0).
  Cite Yao et al., "BlendedMVS: A Large-scale Dataset for Generalized Multi-view Stereo
  Networks", CVPR 2020.
- **ETH3D**: Creative Commons Attribution-NonCommercial-ShareAlike 4.0 (CC-BY-NC-SA-4.0).
  Cite Schöps et al., "A Multi-View Stereo Benchmark with High-Resolution Images and
  Multi-Camera Videos", CVPR 2017. Benchmark-internal use only; do not redistribute
  derived data commercially.
