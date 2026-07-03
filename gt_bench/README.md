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

## Selected scenes

`scenes_blendedmvs.txt` — 5 scenes chosen from the 7-scene community validation split
(`project_lists/validation_list.txt` in the BlendedMVS repo) after visual inspection of
`blended_images`, maximizing category diversity: aerial/large, sculpture/statue (drone
monument + close-range fountain), building facades (backgrounds of both sculpture scenes
plus the aerial roofs — the validation split has no dedicated facade-only scene), indoor,
and small object. Excluded: `5b950c71608de421b1e7318f` (indoor sneaker — duplicates the
indoor small-object slot) and `5c189f2326173c3a09ed7ef3` (studio turntable figurine —
duplicates small object, atypical capture).

`scenes_eth3d.txt` (Task 2) — ETH3D high-res multi-view scene names.

## Download sources (used 2026-07-03)

- BlendedMVS low-res set (27.5 GB, all 113 scenes): GitHub release mirror —
  `https://github.com/YoYo000/BlendedMVS/releases/download/v1.0.0/BlendedMVS.z01..z15 + BlendedMVS.zip`
  (split zip; recombine with `zip -s 0 BlendedMVS.zip --out full.zip`). Archive paths are
  prefixed `BlendedMVS/<sceneID>/...`.
- Textured meshes (9.42 GB, all 113 scenes): OneDrive link from the BlendedMVS README,
  fetched headless via the `onedrivedownloader` pip package (plain `curl`/`wget` on the
  share URL gets 403/HTML). Archive paths are prefixed `dataset_textured_meshes/<sceneID>/`.
- ETH3D (Task 2): per-scene `.7z` archives from `https://www.eth3d.net/data/`.

Only the 7 validation-split scenes are extracted; the full archives are kept in `_dl`/
`_dl_mesh` so more scenes can be added without re-downloading.

## Licenses

- **BlendedMVS / BlendedMVG**: Creative Commons Attribution 4.0 International (CC-BY-4.0).
  Cite Yao et al., "BlendedMVS: A Large-scale Dataset for Generalized Multi-view Stereo
  Networks", CVPR 2020.
- **ETH3D**: Creative Commons Attribution-NonCommercial-ShareAlike 4.0 (CC-BY-NC-SA-4.0).
  Cite Schöps et al., "A Multi-View Stereo Benchmark with High-Resolution Images and
  Multi-Camera Videos", CVPR 2017. Benchmark-internal use only; do not redistribute
  derived data commercially.
