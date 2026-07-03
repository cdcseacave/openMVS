# GT Benchmark + Confidence Speed + Fusion Accuracy — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a real-GT evaluation benchmark (BlendedMVS + ETH3D), make per-view confidence adjustment ≥10× faster per map (plus an integrated geometric-consistency-stage mode), and improve confidence/fusion accuracy (FSV evidence, soft gates, GT-recalibrated parameters, fusion rescue guards) — per spec `docs/superpowers/specs/2026-07-03-conf-fusion-gt-benchmark-design.md`.

**Architecture:** Three sequential phases. Phase 1 (Tasks 1–8) creates datasets, importers, GT eval scripts, a bench runner, and baseline numbers — every later task is judged against it. Phase 2 (Tasks 9–14) rewrites the `AdjustConfidence` hot path (fused single-precision projection matrices, DMapCache batching, prior reuse) and adds an integrated mode that computes confidence during the last geometric-consistency iteration of depth estimation. Phase 3 (Tasks 15–20) adds free-space-violation evidence, soft gates, GT-driven recalibration, and fusion rescue guards.

**Tech Stack:** C++ (libs/MVS), CUDA (Task 14 only), Python/numpy eval scripts (`scripts/python/`), bash runners, ETH3D official C++ eval tool, MapAnything (existing, conda env `sam3`).

## Global Constraints

- **Build:** `VCPKG_ROOT=/home/ubuntu/vcpkg cmake --build make --config Release --target <APP> -j30`. Binaries land in `make/bin/Release/`. Run all binaries with `LD_LIBRARY_PATH=/usr/local/cuda/lib64`. Never use the Debug binary for runs that reuse `.dmap` files (SIGTRAP assert). Do NOT create new build dirs.
- **Python:** use `/home/ubuntu/miniconda3/bin/python` for everything in `scripts/python/` (system python3 has no numpy). MapAnything runs only in conda env `sam3` (`conda run -n sam3 python ...`); if sam3's numpy got bumped to ≥2, fix with `pip install 'numpy<2'` inside sam3.
- **Disk (critical):** root `/dev/vda1` is near-full and shared with `/tmp`; filling it bricks the session. ALL datasets, dmaps, clouds, results go under `/home/ubuntu/virginia/gt_bench/`. The repo only gets scripts and docs — never data.
- **GPU is shared** (single A100): run `nvidia-smi` before estimation/MapAnything jobs; wait if another job holds >30 GB.
- **No multi-threading inside per-view confidence code** (user requirement — views are already processed in parallel by the thread pool). No OpenMP/threads in `AdjustConfidence`'s pixel loop, ever.
- **Deferred-swap semantics:** within the adjust phase, a view's neighbor confidence reads must see PRE-adjustment values. Any change to the write-back path must preserve this (see Task 10).
- **`.dmap` reuse** happens only when flags match the original resolution (`--resolution-level`, `--max-resolution`). Fusion re-runs on adjusted dmaps must pass `--postprocess-dmaps 0` or the confidence gets adjusted twice (postprocess re-saves dmaps).
- **Acceptance thresholds (from spec):** rewrite parity = GT ROC-AUC within ±0.002; integrated-mode adoption = ROC-AUC drop <0.01 vs standalone; WS3 items adopted only if GT completeness improves (or cost drops) with gross-outlier % increase ≤ +0.05 pp and per-scene conf ROC not regressing >0.005.
- **Speed targets:** per-map compute ≥10× faster than the Task-9 reference at 2560-wide resolution; adjust-phase wall ≥5× faster than fusion wall on every benchmark row.
- **Commits:** C++ → `dense: ...`, python/bench scripts → `scripts: ...`, docs → `doc: ...`. Commit after every task (scripts + docs; never data).
- **Key code anchors** (verify before editing — lines drift): `libs/MVS/SceneDensify.cpp` — `AdjustConfidence` :1225-1366, gate constants :1239-1242, posterior :1328-1333, `ComputeIntraMapPrior` :1083-1160, adjust-phase dispatch :2609-2638, neighbor load/unload :2843-2879, `adjusted.cmap` round-trip :2889-2899, `DenseFuseDepthMaps` rescue :1940-2023, keep-rule :2021-2023, `DMapCache` usage :1841, geometric iterations :2570-2596, CUDA/Metal pools :2563-2567; `libs/MVS/DepthMap.cpp` — OPTDENSE defvars :129-131 area, geometric consistency score :588-602, `DepthGradientEstimator` :1601-1640.

---

# Phase 1 — WS1: GT benchmark

### Task 1: Directory scaffold + BlendedMVS download + scene selection

**Files:**
- Create: `/home/ubuntu/virginia/gt_bench/{blendedmvs,eth3d,runs,results}/` (data, not committed)
- Create: `gt_bench/README.md` (in repo root `gt_bench/` — new committed folder for bench scripts)

**Interfaces:**
- Produces: `/home/ubuntu/virginia/gt_bench/blendedmvs/<sceneID>/{blended_images,cams,rendered_depth_maps,textured_mesh}` per selected scene; `gt_bench/scenes_blendedmvs.txt` (list of 5 selected scene IDs, one per line) consumed by Tasks 3/7.

- [ ] **Step 1: Scaffold + download BlendedMVS low-res + textured meshes**

```bash
mkdir -p /home/ubuntu/virginia/gt_bench/{blendedmvs,eth3d,runs,results}
df -h /home/ubuntu/virginia   # must have >100G free
```

Download links are in the README of https://github.com/YoYo000/BlendedMVS (verified live 2026-07-03): `BlendedMVS.zip` (low-res, 27.5 GB, OneDrive link "low-res set") and the textured-mesh archive (9.42 GB). OneDrive links can be fetched with `wget/curl` after resolving the share URL (append `&download=1`), or via a HuggingFace mirror if the direct fetch stalls (search `hf datasets BlendedMVS`). Download into `/home/ubuntu/virginia/gt_bench/blendedmvs/` and unzip there. If only per-scene archives are available, fetching just the validation-split scenes (Step 3) is sufficient — we do not need all 113.

- [ ] **Step 2: Verify structure**

```bash
ls /home/ubuntu/virginia/gt_bench/blendedmvs | head   # 24-hex-char scene IDs
S=$(ls /home/ubuntu/virginia/gt_bench/blendedmvs | head -1)
ls /home/ubuntu/virginia/gt_bench/blendedmvs/$S      # expect: blended_images cams rendered_depth_maps (+ textured mesh dir)
ls /home/ubuntu/virginia/gt_bench/blendedmvs/$S/cams | head -3   # 00000000_cam.txt ... + pair.txt
```

Expected: each scene has N images ⇔ N `_cam.txt` ⇔ N `.pfm` depth maps. Record where the textured mesh lives (`textured_mesh/*.obj` or `.ply`) — Task 6 needs the exact path/format.

- [ ] **Step 3: Select 5 scenes, write the list**

Start from the community validation split (in the BlendedMVS repo `validation_list.txt`): pick 5 covering sculpture/statue, building/facade, aerial/large, indoor, small object. Inspect a few `blended_images` per candidate to classify. Write chosen IDs to `gt_bench/scenes_blendedmvs.txt` with a `# comment` per line describing the scene type. Also write `gt_bench/README.md` documenting the data root, dataset licenses (BlendedMVS CC-BY-4.0, ETH3D CC-BY-NC-SA-4.0) and this layout.

- [ ] **Step 4: Commit**

```bash
git add gt_bench/README.md gt_bench/scenes_blendedmvs.txt && git commit -m "scripts: GT bench scaffold + BlendedMVS scene selection"
```

### Task 2: ETH3D download + build official eval tool

**Files:**
- Create: `/home/ubuntu/virginia/gt_bench/eth3d/<scene>/` for 6 scenes (data)
- Create: `/home/ubuntu/virginia/gt_bench/tools/multi-view-evaluation/` (built tool, not committed)
- Create: `gt_bench/scenes_eth3d.txt`

**Interfaces:**
- Produces: per scene `images/`, `dslr_calibration_undistorted/{cameras,images,points3D}.txt`, `ground_truth_depth/` (or `..._dslr_depth` layout), `dslr_scan_eval/scan_alignment.mlp` + scan PLYs; binary `ETH3DMultiViewEvaluation` consumed by Task 6.

- [ ] **Step 1: Download 6 scenes**

ETH3D serves per-scene archives at `https://www.eth3d.net/data/<scene>_dslr_undistorted.7z`, `<scene>_dslr_depth.7z`, `<scene>_dslr_scan_eval.7z`, `<scene>_dslr_occlusion.7z` (check exact names on https://www.eth3d.net/datasets — the "High-res multi-view" training section lists them). Scenes: `courtyard facade meadow office delivery_area pipes`.

```bash
cd /home/ubuntu/virginia/gt_bench/eth3d
for s in courtyard facade meadow office delivery_area pipes; do
  for k in dslr_undistorted dslr_depth dslr_scan_eval dslr_occlusion; do
    wget -c "https://www.eth3d.net/data/${s}_${k}.7z"; done; done
sudo apt-get install -y p7zip-full 2>/dev/null || true
for f in *.7z; do 7z x -y "$f"; done
ls courtyard/   # expect images/, dslr_calibration_undistorted/, ground_truth_depth/, dslr_scan_eval/, ...
```

Write `gt_bench/scenes_eth3d.txt` (6 lines, `# indoor`/`# outdoor` comments).

- [ ] **Step 2: Build the official eval tool**

```bash
cd /home/ubuntu/virginia/gt_bench/tools
git clone https://github.com/ETH3D/multi-view-evaluation && cd multi-view-evaluation
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8
./ETH3DMultiViewEvaluation --help   # expect usage listing --reconstruction_ply_path, --ground_truth_mlp_path, --tolerances
```

If cmake misses deps: `sudo apt-get install -y libeigen3-dev libboost-all-dev libpcl-dev` and retry.

- [ ] **Step 3: Smoke the tool + commit**

Run it on the scan itself (perfect reconstruction) to validate wiring:

```bash
GT=/home/ubuntu/virginia/gt_bench/eth3d/courtyard/dslr_scan_eval
./ETH3DMultiViewEvaluation --reconstruction_ply_path $GT/scan1.ply \
  --ground_truth_mlp_path $GT/scan_alignment.mlp --tolerances 0.02
```

Expected: completeness/accuracy near 1.0 (scan vs itself). Adjust ply filename to what the archive contains.

```bash
git add gt_bench/scenes_eth3d.txt && git commit -m "scripts: ETH3D bench scenes list"
```

### Task 3: Import all scenes to scene.mvs

**Files:**
- Modify: none (uses existing `apps/InterfaceMVSNet`, `apps/InterfaceCOLMAP`)
- Create: `gt_bench/import_scenes.sh`
- Create (data): `/home/ubuntu/virginia/gt_bench/runs/<scene>/scene.mvs` for all 11 scenes

**Interfaces:**
- Consumes: scene lists from Tasks 1–2.
- Produces: `runs/<scene>/scene.mvs` + `runs/<scene>/images` (symlink), scene naming convention `bmvs_<first8ofID>` and `eth3d_<name>` used by every later task.

- [ ] **Step 1: Build the interface apps**

```bash
VCPKG_ROOT=/home/ubuntu/vcpkg cmake --build make --config Release --target InterfaceMVSNet InterfaceCOLMAP -j30
LD_LIBRARY_PATH=/usr/local/cuda/lib64 make/bin/Release/InterfaceMVSNet --help
LD_LIBRARY_PATH=/usr/local/cuda/lib64 make/bin/Release/InterfaceCOLMAP --help
```

Read the help text (and skim `apps/InterfaceMVSNet/InterfaceMVSNet.cpp` main parse) to confirm the expected folder names — MVSNet convention wants `cams/` + `images/`; BlendedMVS names its image folder `blended_images`, so symlink `images -> blended_images` inside each scene dir before importing.

- [ ] **Step 2: Write `gt_bench/import_scenes.sh`**

```bash
#!/bin/bash
# Imports all benchmark scenes to /home/ubuntu/virginia/gt_bench/runs/<scene>/scene.mvs
set -e
ROOT=/home/ubuntu/virginia/gt_bench
BIN=make/bin/Release
export LD_LIBRARY_PATH=/usr/local/cuda/lib64
grep -v '^#' gt_bench/scenes_blendedmvs.txt | awk '{print $1}' | while read id; do
  s=$ROOT/blendedmvs/$id; out=$ROOT/runs/bmvs_${id:0:8}; mkdir -p $out
  [ -e $s/images ] || ln -s blended_images $s/images
  $BIN/InterfaceMVSNet -i $s -o $out/scene.mvs -w $out
done
grep -v '^#' gt_bench/scenes_eth3d.txt | awk '{print $1}' | while read name; do
  s=$ROOT/eth3d/$name; out=$ROOT/runs/eth3d_$name; mkdir -p $out
  # InterfaceCOLMAP wants a COLMAP workspace: sparse model + images
  mkdir -p $s/colmap/sparse
  for f in cameras images points3D; do
    [ -e $s/colmap/sparse/$f.txt ] || ln -s ../../dslr_calibration_undistorted/$f.txt $s/colmap/sparse/$f.txt; done
  [ -e $s/colmap/images ] || ln -s ../images $s/colmap/images
  $BIN/InterfaceCOLMAP -i $s/colmap -o $out/scene.mvs -w $out
done
```

Adapt flags to the actual `--help` output (e.g. `--image-folder`); ETH3D undistorted calibration uses PINHOLE cameras, which InterfaceCOLMAP supports directly.

- [ ] **Step 3: Run + verify every scene loads**

```bash
bash gt_bench/import_scenes.sh
for m in /home/ubuntu/virginia/gt_bench/runs/*/scene.mvs; do
  /home/ubuntu/miniconda3/bin/python scripts/python/MvsReadMVS.py "$m" | head -5; done
```

Expected: each prints platform/image/vertex counts; image count equals the dataset's image count (ETH3D DSLR scenes have ~10–76 images; BlendedMVS 20–1000). If `MvsReadMVS.py` needs different args, check its source (2 lines — it calls `MvsUtils`).

- [ ] **Step 4: Commit**

```bash
git add gt_bench/import_scenes.sh && git commit -m "scripts: import BlendedMVS+ETH3D scenes to MVS projects"
```

### Task 4: GT depth utilities (`GtUtils.py`)

**Files:**
- Create: `scripts/python/GtUtils.py`
- Create: `scripts/python/tests/test_gtutils.py`

**Interfaces:**
- Produces (consumed by Tasks 5, 6, 8):
  - `read_pfm(path) -> np.ndarray (H,W) float32` — BlendedMVS GT depth.
  - `read_eth3d_depth(path, width, height) -> np.ndarray (H,W) float32` — raw binary GT; invalid → NaN.
  - `remap_eth3d_depth_to_undistorted(depth_distorted, cam_distorted, cam_pinhole) -> (H,W) float32` — only if Step 3 shows GT depth is registered to distorted images.
  - `resize_depth_nearest(d, (h2, w2)) -> np.ndarray` — GT to dmap grid.
  - `gt_labels(d_est, d_gt, rel_tol=0.01, abs_tol=0.0) -> (inlier, outlier, valid)` bool arrays.
  - `view_image_names(scene_mvs) -> list[str]` — dmap index → image filename (via `MvsUtils`).
  - `load_ply_xyz(path) -> np.ndarray (N,3) float64` — vertex-only PLY reader that tolerates OpenMVS's per-vertex variable-length list properties.

- [ ] **Step 1: Write the failing test**

`scripts/python/tests/test_gtutils.py` (plain asserts, run directly):

```python
import sys, os, struct, io, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import GtUtils

def test_pfm_roundtrip(tmp='/tmp/claude_test.pfm'):
    d = np.random.rand(7, 5).astype(np.float32)
    with open(tmp, 'wb') as f:  # write little-endian bottom-up PFM
        f.write(b'Pf\n5 7\n-1.0\n'); np.flipud(d).astype('<f4').tofile(f)
    r = GtUtils.read_pfm(tmp)
    assert r.shape == (7, 5) and np.allclose(r, d)

def test_gt_labels():
    d_gt  = np.array([[10.0, 10.0, np.nan, 10.0]])
    d_est = np.array([[10.05, 11.0, 10.0, 0.0]])   # 0 = no estimate
    inl, outl, valid = GtUtils.gt_labels(d_est, d_gt, rel_tol=0.01)
    assert inl.tolist()  == [[True, False, False, False]]
    assert outl.tolist() == [[False, True, False, False]]
    assert valid.tolist()== [[True, True, False, False]]

def test_resize_nearest():
    d = np.arange(16, dtype=np.float32).reshape(4, 4)
    r = GtUtils.resize_depth_nearest(d, (2, 2))
    assert r.shape == (2, 2)

if __name__ == '__main__':
    test_pfm_roundtrip(); test_gt_labels(); test_resize_nearest(); print('OK')
```

- [ ] **Step 2: Run to verify it fails**

`/home/ubuntu/miniconda3/bin/python scripts/python/tests/test_gtutils.py` → expect `ModuleNotFoundError`/`AttributeError`.

- [ ] **Step 3: Implement `GtUtils.py`**

```python
"""GT depth utilities for the gt_bench evaluation (BlendedMVS PFM, ETH3D raw depth)."""
import numpy as np, struct, re, os

def read_pfm(path):
    with open(path, 'rb') as f:
        header = f.readline().decode().rstrip()
        if header not in ('Pf', 'PF'): raise ValueError('not PFM: ' + path)
        w, h = map(int, f.readline().decode().split())
        scale = float(f.readline().decode().rstrip())
        data = np.fromfile(f, '<f4' if scale < 0 else '>f4', w * h * (3 if header == 'PF' else 1))
    img = data.reshape((h, w, 3) if header == 'PF' else (h, w))
    return np.flipud(img).copy()  # PFM stores rows bottom-up

def read_eth3d_depth(path, width, height):
    d = np.fromfile(path, '<f4')
    assert d.size == width * height, f'{path}: {d.size} != {width}x{height}'
    d = d.reshape(height, width).copy()
    d[~np.isfinite(d)] = np.nan
    d[d <= 0] = np.nan
    return d

def resize_depth_nearest(d, shape):
    h2, w2 = shape; h, w = d.shape
    yi = (np.arange(h2) * (h / h2) + 0.5 * h / h2).astype(int).clip(0, h - 1)
    xi = (np.arange(w2) * (w / w2) + 0.5 * w / w2).astype(int).clip(0, w - 1)
    return d[yi][:, xi]

def gt_labels(d_est, d_gt, rel_tol=0.01, abs_tol=0.0):
    valid = np.isfinite(d_gt) & (d_gt > 0) & (d_est > 0)
    tol = np.maximum(rel_tol * np.where(np.isfinite(d_gt), d_gt, 0), abs_tol)
    err = np.abs(np.where(valid, d_est - d_gt, 0))
    inlier = valid & (err <= tol)
    return inlier, valid & ~inlier, valid

def view_image_names(scene_mvs):
    from MvsUtils import loadMVS   # verify actual loader name in MvsUtils.py
    mvs = loadMVS(scene_mvs)
    return [os.path.basename(im['name']) for im in mvs['images']]

def load_ply_xyz(path):
    # Vertex-only reader tolerating variable-length list properties (OpenMVS clouds).
    with open(path, 'rb') as f:
        n, props, fmt = 0, [], None
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if line.startswith('format'): fmt = line.split()[1]
            elif line.startswith('element vertex'): n = int(line.split()[-1]); props = []
            elif line.startswith('property') and n:
                props.append(line.split())
            elif line == 'end_header': break
        assert fmt == 'binary_little_endian', fmt
        SZ = {'float': 4, 'double': 8, 'uchar': 1, 'uint8': 1, 'int': 4, 'uint': 4,
              'uint32': 4, 'ushort': 2, 'uint16': 2, 'float32': 4, 'float64': 8}
        fixed = [p for p in props if p[1] != 'list']
        lists = [p for p in props if p[1] == 'list']
        if not lists:  # fast path: one big fromfile
            rec = sum(SZ[p[1]] for p in fixed)
            off = 0; offs = {}
            for p in fixed: offs[p[2]] = (off, p[1]); off += SZ[p[1]]
            raw = np.fromfile(f, np.uint8, n * rec).reshape(n, rec)
            xyz = np.empty((n, 3))
            for i, name in enumerate(('x', 'y', 'z')):
                o, t = offs[name]
                xyz[:, i] = raw[:, o:o + SZ[t]].copy().view('<f4' if SZ[t] == 4 else '<f8')[:, 0]
            return xyz
        # slow path: sequential parse (lists have per-vertex length)
        pre = sum(SZ[p[1]] for p in fixed)
        xyz = np.empty((n, 3), np.float64)
        buf = f.read()
        pos = 0
        for i in range(n):
            xyz[i] = struct.unpack_from('<3f', buf, pos)[:3]
            pos += pre
            for p in lists:
                cnt = struct.unpack_from('<' + {'uchar': 'B', 'uint8': 'B', 'int': 'i', 'uint': 'I', 'uint32': 'I'}[p[2]], buf, pos)[0]
                pos += SZ[p[2]] + cnt * SZ[p[3]]
        return xyz
```

(`remap_eth3d_depth_to_undistorted` is added in Step 5 only if needed.)

- [ ] **Step 4: Run tests to verify they pass**

`/home/ubuntu/miniconda3/bin/python scripts/python/tests/test_gtutils.py` → `OK`. Also smoke on real data: read one BlendedMVS PFM (shape = image size, positive finite depths where valid) and one ETH3D depth file.

- [ ] **Step 5: Resolve the ETH3D depth convention empirically (do not skip)**

ETH3D `ground_truth_depth` may be registered to the DISTORTED images and may store z-depth or along-ray distance. Decide with data: load `dslr_calibration_undistorted/images.txt` GT pose for one image, load the laser scan (`load_ply_xyz` on the scan PLY, subsample 1M pts), project points into the camera (world→cam with COLMAP convention `x_cam = R*X + t`, pixel via PINHOLE K), and compare projected z against the GT depth file sampled at those pixels, under 4 hypotheses: {undistorted grid, distorted grid} × {z-depth, ray distance}. Pick the hypothesis with median relative error < 1%; write the answer as a comment + implement accordingly (if distorted-grid: implement `remap_eth3d_depth_to_undistorted` — for each undistorted pixel, apply the distorted camera's THIN_PRISM_FISHEYE forward-distortion to its normalized ray from `dslr_calibration_jpg` params, nearest-sample the distorted GT; z values transfer unchanged). Add a regression test with the measured median error threshold to `test_gtutils.py`.

- [ ] **Step 6: Commit**

```bash
git add scripts/python/GtUtils.py scripts/python/tests/test_gtutils.py
git commit -m "scripts: GT depth utilities (BlendedMVS PFM, ETH3D depth, labels)"
```

### Task 5: EvalConfidence.py GT mode

**Files:**
- Modify: `scripts/python/EvalConfidence.py`
- Test: `scripts/python/tests/test_evalconf_gt.py`

**Interfaces:**
- Consumes: `GtUtils.gt_labels`, `GtUtils.read_pfm`, `GtUtils.read_eth3d_depth`, `GtUtils.view_image_names`, `GtUtils.resize_depth_nearest`.
- Produces: CLI `EvalConfidence.py <dmap_dir> --gt-depth-dir <dir> --gt-format {blendedmvs,eth3d} --scene-mvs <scene.mvs> [--rel-tol 0.01 --abs-tol 0.0] [--json out.json]` emitting the SAME metric block as the existing labels mode (ROC-AUC, PR-AUC, P@0.1, R@0.1, Spearman, Brier, ECE) plus `n_labeled`; `--json` output schema `{scene, mode, per_view: [...], pooled: {roc_auc, pr_auc, p_at_01, r_at_01, spearman, brier, ece}}` consumed by Task 7's aggregator.

- [ ] **Step 1: Read the existing script**

Read `scripts/python/EvalConfidence.py` fully (249 lines). Identify: (a) how it loads dmaps + confidence (via `MvsUtils`), (b) the function computing metrics from `(conf, labels)` arrays, (c) the STRICT/LENIENT split. The GT mode reuses (a) and (b) unchanged; GT labels replace the `.flabel` source; keep STRICT semantics (per-pixel inlier/outlier from `gt_labels`), skip LENIENT for GT mode.

- [ ] **Step 2: Write the failing test**

```python
import sys, os, numpy as np, subprocess
# synthetic: 1 fake dmap-array + GT where conf perfectly ranks -> AUC 1.0
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import EvalConfidence as E
def test_gt_label_metrics():
    d_gt  = np.full((10, 10), 5.0); d_gt[0, :] = np.nan
    d_est = np.full((10, 10), 5.0); d_est[5:, :] = 6.0     # bottom half = outliers
    conf  = np.where(d_est == 5.0, 0.9, 0.1)
    m = E.metrics_from_gt(d_est, conf, d_gt, rel_tol=0.01)
    assert m['roc_auc'] > 0.99 and m['n_labeled'] == 90
if __name__ == '__main__': test_gt_label_metrics(); print('OK')
```

- [ ] **Step 3: Run to verify it fails; implement**

Add to `EvalConfidence.py`: `metrics_from_gt(d_est, conf, d_gt, rel_tol, abs_tol=0.0)` — calls `GtUtils.gt_labels` (resizing GT with `resize_depth_nearest` when shapes differ), then feeds inlier/outlier masks into the existing metric function; plus the CLI flags from the Interfaces block wiring per-view GT files: for `blendedmvs`, GT file = `rendered_depth_maps/<imagestem>.pfm`; for `eth3d`, GT file = `ground_truth_depth/.../<imagename>` (path shape learned in Task 4 Step 5); match dmap index → image name via `view_image_names(--scene-mvs)`.

- [ ] **Step 4: Verify test passes + real smoke**

Test → `OK`. Real smoke needs dmaps and is deferred to Task 7 (runner produces them); note this in the commit message.

- [ ] **Step 5: Commit**

```bash
git add scripts/python/EvalConfidence.py scripts/python/tests/test_evalconf_gt.py
git commit -m "scripts: EvalConfidence GT-depth label mode (BlendedMVS/ETH3D)"
```

### Task 6: Fusion-vs-GT evaluation (`EvalFusionGT.py` + ETH3D wrapper)

**Files:**
- Create: `scripts/python/EvalFusionGT.py`
- Create: `gt_bench/eth3d_eval.sh`
- Test: `scripts/python/tests/test_evalfusion_gt.py`

**Interfaces:**
- Consumes: `GtUtils.load_ply_xyz`; existing `scripts/python/CompletenessGT.py` (read it first — reuse its NN/voxel-hash machinery if it fits, else scipy `cKDTree` if available in miniconda).
- Produces:
  - `EvalFusionGT.py <fused.ply> --gt-mesh <mesh.obj|.ply> [--n-samples 10000000] [--json out.json]` → `{completeness: {tol: frac}, accuracy: {tol: frac}, gross_outlier_frac, tol_abs: {...}, diag}` where tolerances = {0.25%, 0.5%, 1%} of GT bbox diagonal, gross outliers at 5% diag. Used for BlendedMVS.
  - `gt_bench/eth3d_eval.sh <fused.ply> <scene_dir> <out.json>` → parses `ETH3DMultiViewEvaluation` stdout (tolerances 0.01,0.02,0.05,0.1 m) into the same JSON shape (+`f1`). Used for ETH3D.

- [ ] **Step 1: Write the failing test**

```python
import sys, os, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import EvalFusionGT as E
def test_cube():
    # GT = unit cube surface; recon = same but one face missing + 100 far outliers
    v, f = E.make_cube_mesh()                       # helper returned for tests
    gt = E.sample_mesh(v, f, 200000)
    rec = gt[gt[:, 2] < 0.999]                      # drop top face samples (~1/6)
    out = np.random.RandomState(0).rand(100, 3) * 10 + 5
    m = E.score_cloud(np.vstack([rec, out]), gt, tols=[0.01], gross_tol=0.5)
    assert 0.79 < m['completeness'][0.01] < 0.88    # ~5/6 of faces present
    assert m['gross_outlier_frac'] > 0.0003         # the 100 planted floaters
if __name__ == '__main__': test_cube(); print('OK')
```

- [ ] **Step 2: Run to verify failure; implement core**

```python
import numpy as np

def sample_mesh(V, F, n, seed=0):
    a = V[F[:, 1]] - V[F[:, 0]]; b = V[F[:, 2]] - V[F[:, 0]]
    area = 0.5 * np.linalg.norm(np.cross(a, b), axis=1)
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(F), n, p=area / area.sum())
    r1, r2 = rng.random((2, n)); s = np.sqrt(r1)
    return (V[F[idx, 0]] * (1 - s)[:, None] + V[F[idx, 1]] * (s * (1 - r2))[:, None]
            + V[F[idx, 2]] * (s * r2)[:, None])

def load_obj(path):
    V, F = [], []
    for line in open(path):
        if line.startswith('v '): V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            ix = [int(t.split('/')[0]) - 1 for t in line.split()[1:]]
            for k in range(1, len(ix) - 1): F.append([ix[0], ix[k], ix[k + 1]])
    return np.asarray(V), np.asarray(F)

def nn_dist(query, ref):
    try:
        from scipy.spatial import cKDTree
        return cKDTree(ref).query(query, workers=-1)[0]
    except ImportError:
        import CompletenessGT as C     # reuse its voxel-hash NN (check exact fn name)
        return C.nn_dist(query, ref)

def score_cloud(rec, gt, tols, gross_tol):
    d_gt2rec = nn_dist(gt, rec)        # completeness
    d_rec2gt = nn_dist(rec, gt)        # accuracy / outliers
    return {'completeness': {t: float((d_gt2rec <= t).mean()) for t in tols},
            'accuracy':     {t: float((d_rec2gt <= t).mean()) for t in tols},
            'gross_outlier_frac': float((d_rec2gt > gross_tol).mean()),
            'n_rec': len(rec), 'n_gt': len(gt)}
```

CLI: load fused via `GtUtils.load_ply_xyz`, GT via `load_obj`/PLY (Task 1 recorded the mesh format), tolerances from bbox diagonal of GT samples; `make_cube_mesh()` = 8 verts/12 tris helper. `eth3d_eval.sh`: run the tool, `grep`/`awk` the "Completenesses/Accuracies/F1-scores" lines into JSON with the same keys.

- [ ] **Step 3: Verify test passes**

`/home/ubuntu/miniconda3/bin/python scripts/python/tests/test_evalfusion_gt.py` → `OK`. Check whether miniconda has scipy (`python -c "import scipy"`); if not, verify the CompletenessGT fallback path is exercised by the test too.

- [ ] **Step 4: Commit**

```bash
git add scripts/python/EvalFusionGT.py gt_bench/eth3d_eval.sh scripts/python/tests/test_evalfusion_gt.py
git commit -m "scripts: fusion-vs-GT eval (mesh sampling + ETH3D official tool wrapper)"
```

### Task 7: Bench runner + aggregator + BASELINE run

**Files:**
- Create: `gt_bench/run_scene.sh`, `gt_bench/aggregate_gt.py`
- Create (data): `/home/ubuntu/virginia/gt_bench/results/*.json`, `/home/ubuntu/virginia/gt_bench/AGGREGATE_GT.md`

**Interfaces:**
- Consumes: everything from Tasks 3–6.
- Produces: `run_scene.sh <scene> <reslevel>` writing `results/<scene>_L<r>_<tag>.json` for tags `{conf_raw, conf_adj, fuse_w0, fuse_w3, timing}`; `aggregate_gt.py` → `AGGREGATE_GT.md` tables. THE BASELINE (this branch, pre-Phase-2) that all later tasks diff against.

- [ ] **Step 1: Write `gt_bench/run_scene.sh`**

```bash
#!/bin/bash
# Usage: run_scene.sh <scene> <reslevel>   (scene = runs/<scene> dir name)
set -e
SCENE=$1; L=$2
ROOT=/home/ubuntu/virginia/gt_bench; WD=$ROOT/runs/$SCENE/L$L; RES=$ROOT/results
BIN="env LD_LIBRARY_PATH=/usr/local/cuda/lib64 $(pwd)/make/bin/Release/DensifyPointCloud"
PY=/home/ubuntu/miniconda3/bin/python
mkdir -p $WD $RES; cd $WD
[ -e scene.mvs ] || cp ../scene.mvs .
[ -e images ] || ln -s ../images images 2>/dev/null || true
# 1) estimate depth-maps only (GPU), raw confidence
$BIN scene.mvs --resolution-level $L --max-resolution 3200 --fusion-mode 1 --postprocess-dmaps 0 -v 2
mkdir -p raw_dmaps && cp *.dmap raw_dmaps/
# 2) adjust confidence in place (CPU, timed by the code)
$BIN scene.mvs --resolution-level $L --max-resolution 3200 --fusion-mode 1 --postprocess-dmaps 4 -v 2
# 3) fusion on adjusted dmaps: default rescue (w3) and disabled (w0)
printf 'Fuse Prior Weight = 0\n' > fuse_w0.cfg
$BIN scene.mvs --resolution-level $L --max-resolution 3200 --postprocess-dmaps 0 -v 2 \
     --dense-config-file fuse_w0.cfg -o cloud_w0.mvs
$BIN scene.mvs --resolution-level $L --max-resolution 3200 --postprocess-dmaps 0 -v 2 -o cloud_w3.mvs
# 4) evaluations
SRC=$ROOT/runs/$SCENE/scene_src   # per-dataset GT dir symlink, set by import_scenes.sh
for tag in raw adj; do
  D=$([ $tag = raw ] && echo raw_dmaps || echo .)
  $PY $(pwd)/../../../../scripts/python/EvalConfidence.py $D --scene-mvs scene.mvs \
      --gt-depth-dir $SRC --gt-format ${SCENE%%_*} --json $RES/${SCENE}_L${L}_conf_${tag}.json
done
# fusion eval: dataset-specific (bmvs -> EvalFusionGT vs mesh; eth3d -> official tool)
```

Finish the fusion-eval branch per dataset (mesh path for bmvs scenes, `eth3d_eval.sh` for eth3d), and a `timing` JSON extracted from the freshest `DensifyPointCloud-*.log` in `$WD`: grep the `Confidence-maps adjusted:` line (wall + compute + ms/map) and the fusion phase wall (`Depth-maps fused`/equivalent — check exact log text in `SceneDensify.cpp` :2636 area and the fusion end log). NOTE: `--fusion-mode 1` is assumed to mean "estimate dmaps, skip fusion" — verify against `apps/DensifyPointCloud/DensifyPointCloud.cpp` before first run and adjust. Verify `-o cloud_w0.mvs` controls the output PLY name (`cloud_w0.ply`); else rename after each run.

- [ ] **Step 2: Write `gt_bench/aggregate_gt.py`**

Scan `results/*.json` → emit `AGGREGATE_GT.md` with 4 tables: [1] conf ROC/PR/P@0.1 raw vs adjusted per scene×res; [2] fusion completeness/accuracy/F1 per tol, w0 vs w3; [3] gross-outlier %; [4] timing (per-map adjust ms, adjust wall, fusion wall, ratio). Plain string formatting, no deps beyond json/os.

- [ ] **Step 3: Run the BASELINE (long, GPU)**

Resolutions: ETH3D scenes at L∈{3,2,1} (24 MP images → capped 3200px wide at L1), BlendedMVS at L∈{1,0} (768×576 native). Run overnight-style, sequential:

```bash
for s in $(ls /home/ubuntu/virginia/gt_bench/runs); do case $s in
  eth3d_*) for L in 3 2 1; do bash gt_bench/run_scene.sh $s $L; done;;
  bmvs_*)  for L in 1 0; do bash gt_bench/run_scene.sh $s $L; done;; esac; done
/home/ubuntu/miniconda3/bin/python gt_bench/aggregate_gt.py
```

Expected: `AGGREGATE_GT.md` filled; sanity-check adjusted-conf ROC ≫ raw (pseudo-GT bench gave +0.05…+0.17 — real GT should confirm the direction), w3 completeness ≥ w0, gross-outliers small. **Visually inspect one fused cloud per dataset** (top-down render — the MapAnything episode taught us metrics can't see a broken GT/cloud).

- [ ] **Step 4: Commit scripts + baseline table copy**

```bash
cp /home/ubuntu/virginia/gt_bench/AGGREGATE_GT.md gt_bench/BASELINE_2026-07.md
git add gt_bench/run_scene.sh gt_bench/aggregate_gt.py gt_bench/BASELINE_2026-07.md
git commit -m "scripts: GT bench runner + aggregator + baseline numbers"
```

### Task 8: MapAnything-vs-GT calibration study

**Files:**
- Create: `scripts/python/MapAnyVsGT.py`
- Create: `/home/ubuntu/virginia/gt_bench/VERDICT_MAPANYTHING.md` (+ copy `gt_bench/VERDICT_MAPANYTHING.md` committed)

**Interfaces:**
- Consumes: existing `MvsSparseDepth.py`, `MapAnyInferMV.py` (env knobs `MA_MASK_EDGES=0 MA_RES=910`), `MapAnyVoxelFuse.py` (`MINVIEWS=2`); `GtUtils`, `EvalFusionGT.score_cloud`.
- Produces: the written verdict on pseudo-GT trustworthiness (used to decide MapAnything's future role).

- [ ] **Step 1: Generate pseudo-GT on 3 bench scenes** (GPU; `nvidia-smi` first)

Scenes: `eth3d_courtyard` (outdoor), `eth3d_office` (indoor), 1 BlendedMVS scene. Use the L-level with existing dmaps from Task 7. Follow the exact pipeline from the existing scripts (sparse depth → `conda run -n sam3 ... MapAnyInferMV.py` → `MapAnyVoxelFuse.py` gated ≥2 views), outputs under `/home/ubuntu/virginia/gt_bench/mapany/<scene>/`.

- [ ] **Step 2: Write + run `MapAnyVsGT.py`**

Per view: load MapAnything depth npz + GT depth (GtUtils), report median |d_MA − d_GT|/d_GT and fractions >1 %, >3 %, >10 % on jointly-valid pixels, pooled per scene. Per cloud: `score_cloud(mapany_cloud, gt_samples, ...)` at the Task-6 tolerances, alongside the same numbers for the OpenMVS `cloud_w3.ply` for context. Output one JSON + printed table per scene.

- [ ] **Step 3: Write the verdict**

`VERDICT_MAPANYTHING.md`: pseudo-GT depth error stats, cloud precision/completeness vs real GT, and a decision: (a) retire for tuning, keep for GT-less visual QA; or (b) keep with stated error bars. Cross-reference how far the old pseudo-GT-based conclusions (w3 completeness gains, gross-outlier claims) hold under real GT (compare Task 7 tables vs `/home/ubuntu/virginia/mvs_bench/AGGREGATE.md`).

- [ ] **Step 4: Commit**

```bash
cp /home/ubuntu/virginia/gt_bench/VERDICT_MAPANYTHING.md gt_bench/
git add scripts/python/MapAnyVsGT.py gt_bench/VERDICT_MAPANYTHING.md
git commit -m "scripts: MapAnything pseudo-GT calibration vs real GT + verdict"
```

---

# Phase 2 — WS2: Confidence speed

### Task 9: Fused-matrix single-precision rewrite of the confirmation sweep

**Files:**
- Modify: `libs/MVS/SceneDensify.cpp` (`AdjustConfidence` :1225-1366)
- Create (data): `/home/ubuntu/virginia/gt_bench/refbin/` (reference binary + feature snapshots)

**Interfaces:**
- Consumes: baseline numbers (Task 7).
- Produces: `struct NeighborProj` + the rewritten pixel loop, reused verbatim by Tasks 12/15/16. Per-map compute expected ≥3× faster (cumulative ≥10× by Task 11).

- [ ] **Step 1: Snapshot the reference BEFORE editing**

```bash
cp make/bin/Release/DensifyPointCloud /home/ubuntu/virginia/gt_bench/refbin/
cp make/bin/Release/*.so /home/ubuntu/virginia/gt_bench/refbin/
```

On 2 scenes (`eth3d_courtyard` L2, one bmvs L0), with dmaps from Task 7: run the reference binary with `--postprocess-dmaps 4 --export-conf-features 1 --fusion-mode 1` into a copy of the workdir; keep the `cfeatK/cfeatPconf/cfeatPrior/cfeatPhoto` sidecars as `ref_features/`. Also record the reference `Confidence-maps adjusted:` timing line.

- [ ] **Step 2: Read the current code, then rewrite**

Read `AdjustConfidence` (:1225-1366) and the gate constants (:1239-1242) carefully — the rewrite must port every threshold and test VERBATIM, only changing the math path. Add before the pixel loop (double-precision compose, float storage; verify helper names in `libs/MVS/Camera.h` — `Camera::K`, `R`, `C`, `GetInvK()`):

```cpp
struct NeighborProj { // fused ref->neighbor projection (see plan 2026-07-03)
    Matrix3x3f A;     // Kn*Rn*Rr^T*Kr^-1 : (u*d,v*d,d) -> nbr cam h-coords (z = nbr depth)
    Point3f b;        // Kn*Rn*(Cr-Cn)
    Matrix3x3f Ai;    // Kr*Rr*Rn^T*Kn^-1 : back-projection for the fwd-bwd gate
    Point3f bi;       // Kr*Rr*(Cn-Cr)
    Matrix3x3f Rrel;  // Rn*Rr^T : ref-cam normal -> nbr-cam normal
    const DepthMap* depthMap; const ConfidenceMap* confMap; const NormalMap* normalMap;
};
```

Pixel loop template (adapt names/thresholds to the real code; note ImageRef is (x=col, y=row)):

```cpp
const Depth d = depthMap(r,c); // valid, checked as before
const Point3f xd((float)c*d, (float)r*d, d);
unsigned K = 0; float Pconf = 0;
for (const NeighborProj& np : neighborProjs) {
    const Point3f q = np.A * xd + np.b;
    if (q.z <= 0) continue;
    const ImageRef x(ROUND2INT(q.x/q.z), ROUND2INT(q.y/q.z));
    if (!np.depthMap->isInsideWithBorder<int,0>(x)) continue;
    const Depth dn = (*np.depthMap)(x);
    if (dn <= 0 || !IsDepthSimilar(q.z, dn, thDepthDiff)) continue;      // G1 (same fn/args as current)
    const Point3f qr = np.Ai * Point3f(x.x*dn, x.y*dn, dn) + np.bi;      // G2 fwd-bwd
    if (qr.z <= 0) continue;
    const float du = qr.x/qr.z - (float)c, dv = qr.y/qr.z - (float)r;    // px error in ref
    if (du*du + dv*dv > thReprojSq) continue;                            // same threshold as current
    // G3 normal (only now rotate; same test as current) ; G4 conf gate (same order as current!)
    ...
    ++K; Pconf += cN;
}
```

Keep the posterior block (:1328-1333) untouched. Delete the per-pixel `TransformPointI2W`/`TransformPointW2C`/`ProjectPointP` calls and the per-pixel world-normal precompute (:1284) — the normal now rotates by `np.Rrel` lazily inside G3. **Preserve the gate ORDER of the current code** (if G4 currently precedes G3, keep that) so K/Pconf semantics match exactly. No threads.

- [ ] **Step 3: Rebuild + equivalence check**

```bash
VCPKG_ROOT=/home/ubuntu/vcpkg cmake --build make --config Release --target DensifyPointCloud -j30
```

Re-run the two feature exports with the new binary into `new_features/`. Compare with a short throwaway python check (loadable via `MvsUtils` raw-map loaders, format 'LMAP'/raw sidecars — see how `SweepConfParams.py` loads them): require ≥99.5 % of labeled pixels with identical `cfeatK`, max |ΔPconf| ≤ 1e-3 where K matches (float-vs-double boundary flips are expected at gate thresholds). Then end-to-end: `--postprocess-dmaps 4` on both scenes, `EvalConfidence.py --gt-depth-dir` → |ΔROC-AUC| ≤ 0.002 vs the Task-7 `conf_adj` baseline.

- [ ] **Step 4: Timing + commit**

Compare the `Confidence-maps adjusted: ... ms/map` line vs Step 1's reference on the ETH3D L1 (largest) run — expect ≥3×. Record both numbers in the commit message.

```bash
git add libs/MVS/SceneDensify.cpp && git commit -m "dense: fused single-precision projection in AdjustConfidence (~Nx per-map)"
```

### Task 10: DMapCache batching + in-memory adjusted confidence

**Files:**
- Modify: `libs/MVS/SceneDensify.cpp` (adjust dispatch :2609-2638, neighbor load :2843-2879, cmap round-trip :1360 + :2889-2899), `libs/MVS/DepthMap.h` (DepthData member if needed)

**Interfaces:**
- Consumes: Task 9's rewritten `AdjustConfidence` (unchanged signature).
- Produces: adjust phase reads each `.dmap` from disk exactly once; `adjusted.cmap` files no longer written.

- [ ] **Step 1: Read `DMapCache.h` (:48-110) and the fusion usage** (`SceneDensify.cpp` :1841, :2060) to mirror the pattern.

- [ ] **Step 2: Implement**

(a) In the adjust phase, replace per-reference `IncRef`/`DecRef` of neighbors (:2859, :2874-2878) with a phase-lifetime `DMapCache` sized via `GetAvailableMemory` (as fusion does at :2060): each worker asks the cache for its neighbors; the cache keeps hot dmaps resident across references. If `DMapCache` isn't thread-safe for concurrent workers (check for locking in DMapCache.h), guard its map with a mutex — contention is negligible vs disk reads.
(b) Replace the `adjusted.cmap` write→reload→delete: store the new conf into a side buffer `newConfMap` kept on the `DepthData` (add member `ConfidenceMap confMapAdjusted;`), and make the `EVT_ADJUSTDEPTHMAP` handler swap `confMap = std::move(confMapAdjusted)` + save the dmap. **The deferred swap is load-bearing** (Global Constraints): neighbors must keep reading pre-adjustment conf during the phase — the swap event for image i must only run after every reference that uses i as a neighbor has finished its sweep. The current two-phase event design already guarantees this ordering (all EVT_FILTERDEPTHMAP before EVT_ADJUSTDEPTHMAP — verify in the dispatcher :2609-2638); preserve it.

- [ ] **Step 3: Verify**

Same two-scene equivalence as Task 9 Step 3 (identical outputs — this task must be a pure I/O change), plus: run a full ETH3D scene and confirm via the cache counter (`numImageRead`, log it at phase end) that dmap disk reads ≈ #maps (was ≈9×#maps); record adjust-phase WALL improvement.

- [ ] **Step 4: Commit**

```bash
git add libs/MVS/SceneDensify.cpp libs/MVS/DepthMap.h
git commit -m "dense: DMapCache-batched adjust phase; drop adjusted.cmap round-trip"
```

### Task 11: Intra-map prior reuse + nested-OpenMP fix + full timing table

**Files:**
- Modify: `libs/MVS/SceneDensify.cpp` (`ComputeIntraMapPrior` :1083-1160 signature; call sites :1253, :1962), `libs/MVS/DepthMap.h` (DepthData member)

**Interfaces:**
- Produces: `const TImage<float>& DepthData::GetIntraMapPrior(...)` or equivalent cached accessor used by BOTH AdjustConfidence and DenseFuseDepthMaps; `ComputeIntraMapPrior(..., bool bParallel)`.

- [ ] **Step 1: Implement**

(a) Cache: add `TImage<float> priorMap` to `DepthData` (or a scene-level array indexed by image); both call sites compute-if-absent, reuse otherwise. Released with the DepthData by DMapCache ejection — verify `DMapCache::Eject` clears it too (add to the release path if not).
(b) Threading: add `bool bParallel` param to `ComputeIntraMapPrior` — the `#pragma omp parallel for` (:1097) runs only when `bParallel`; AdjustConfidence passes `false` (inside pool workers — no intra-view threading), DenseFuseDepthMaps passes `true` (serial caller, cores idle).

- [ ] **Step 2: Verify + full timing table**

Equivalence run (Task 9 Step 3 protocol) — outputs unchanged. Then re-run the FULL bench (Task 7 Step 3 loop — estimation reused, only adjust+fusion re-execute) and re-aggregate. **Acceptance gates:** per-map adjust compute ≥10× faster than the Task-9 Step-1 reference at the largest resolution; adjust wall ≥5× below fusion wall on every scene/res row; conf ROC within ±0.002 of baseline everywhere. Put the new timing table in `AGGREGATE_GT.md` (aggregator handles it) and copy to `gt_bench/TIMING_AFTER_WS2A.md`.

- [ ] **Step 3: Commit**

```bash
git add libs/MVS/SceneDensify.cpp libs/MVS/DepthMap.h gt_bench/TIMING_AFTER_WS2A.md
git commit -m "dense: share intra-map prior between adjust and fusion; fix nested OMP"
```

### Task 12: Integrated geometric-stage confidence (CPU path)

**Files:**
- Modify: `libs/MVS/SceneDensify.cpp` (estimation worker path — find where a view's geometric iteration completes and neighbors are still IncRef'd; comment anchor :378, dispatcher :727-778, geometric loop :2570-2596), `libs/MVS/DepthMap.cpp`/`.h` (new OPTDENSE knob)

**Interfaces:**
- Consumes: Task 9-11 `AdjustConfidence(depthData, idxNeighbors)` (already takes loaded neighbors — reuse as-is).
- Produces: `OPTDENSE::bEstimateConfidence` (title `"Estimate Confidence"`, default `"0"`); when on, dmaps saved by the LAST geometric iteration already carry adjusted confidence; `--postprocess-dmaps 4` not needed.

- [ ] **Step 1: Trace the call site**

Read the estimation worker (`DenseReconstructionEstimate` / `EstimateDepthMap` flow) and answer two questions in a code comment: (a) at the point a view's last-geometric-iteration estimation finishes, which neighbor DepthDatas are loaded (the geometric-consistency inputs), and (b) does geometric-consistency scoring read neighbor confMap (check `DepthMap.cpp` :588-602)? If (b) is yes, note that integrated adjustment can leak adjusted conf into later views' scoring within the last iteration — accepted iff the Task-13 A/B passes anyway (it is the same staleness class the spec accepts).

- [ ] **Step 2: Implement**

```cpp
// DepthMap.cpp, near the other OPTDENSE defvars (:129 area):
DEFVAR_OPTDENSE_bool(bEstimateConfidence, "Estimate Confidence",
    "adjust per-pixel confidence during the last geometric-consistency iteration (fusion-faithful)", "0")
```

At the traced call site (view finished last geometric iter, neighbors resident, before DecRef/save):

```cpp
if (OPTDENSE::bEstimateConfidence &&
    nGeometricIter+1 == (int)OPTDENSE::nEstimationGeometricIters)
    AdjustConfidence(depthData, idxNeighborsLoaded); // same core as postprocess path
```

Ensure the adjusted conf ends up in the dmap that iteration saves (the standalone deferred-swap concern doesn't apply — here each view adjusts only itself, from neighbors' as-loaded conf). Guard against double-adjust: when `bEstimateConfidence` is on and `--postprocess-dmaps 4` is also set, log a warning and skip the postprocess adjustment.

- [ ] **Step 3: Verify + commit**

On 3 scenes (courtyard, office, 1 bmvs): full estimation with `--dense-config-file <(echo 'Estimate Confidence = 1')`-style config (write a real cfg file; SML `Estimate Confidence = 1`), `--postprocess-dmaps 0`; then `EvalConfidence.py` GT mode. Report ROC vs the standalone `conf_adj` baseline (the A/B decision is Task 13). Confirm zero additional dmap reads (no new IncRef) and estimation wall unchanged within noise (+the per-map sweep cost, now ~tens of ms).

```bash
git add libs/MVS/SceneDensify.cpp libs/MVS/DepthMap.cpp libs/MVS/DepthMap.h
git commit -m "dense: optional confidence estimation in last geometric-consistency iteration"
```

### Task 13: Integrated-vs-standalone A/B decision

**Files:**
- Create: `gt_bench/AB_INTEGRATED.md` (committed)

- [ ] **Step 1: Run both modes on the full bench** (estimation re-runs for integrated mode — GPU time; reuse Task 12's 3 scenes plus the remaining 8 at one representative resolution each).

- [ ] **Step 2: Decide per spec**

Pooled + per-scene ROC-AUC delta (integrated − standalone). If drop < 0.01 everywhere: document integrated mode as the recommended default for confidence consumers (update `AdjustConfidence`'s block comment + `gt_bench/README.md`), keep standalone for reuse-existing-dmaps workflows. Else: keep standalone as primary, record why. Optional micro-experiment while here: export the estimator's per-pixel geometric-consistency score (DepthMap.cpp :602) as a 5th feature sidecar on one scene and report its standalone AUC vs labels in the same doc (informs Task 17's sweep; do not wire into the formula yet).

- [ ] **Step 3: Commit** `gt_bench/AB_INTEGRATED.md` (`doc: integrated-confidence A/B results`).

### Task 14: Integrated confidence on the CUDA path (only if Task 13 adopts integrated mode)

**Files:**
- Modify: `libs/MVS/PatchMatchCUDA.cpp/.cu/.h` (read first — structure discovery is step 1), `libs/MVS/SceneDensify.cpp` (CUDA pool dispatch :2563-2567)

**Interfaces:**
- Consumes: gate constants + posterior formula (identical to CPU); `OPTDENSE::bEstimateConfidence`.
- Produces: adjusted confMap computed on GPU during the last geometric iteration; CPU result remains the reference implementation.

- [ ] **Step 1: Read `PatchMatchCUDA.*`** — identify: which per-neighbor buffers are already device-resident during geometric iterations (depth certainly; check normals/conf), how results download. Write a 10-line plan comment in the .cu file.

- [ ] **Step 2: Implement kernel**

One thread per ref pixel; loops the ≤8 neighbors with the same fused `A/b/Ai/bi/Rrel` float matrices (upload one small struct array per view); same 4 gates + posterior + floor; prior: port `ComputeIntraMapPrior`'s 3×3 plane fit into the kernel OR compute on CPU and upload (measure both if trivial; else CPU-upload is fine — it's ~10 % of the work). Upload neighbor confMaps if not already resident (small). Write adjusted conf to the existing conf buffer before download.

- [ ] **Step 3: Verify + commit**

Parity vs CPU-integrated on 2 scenes: |ΔROC| ≤ 0.005, visual conf-map diff sane; report kernel+transfer time per map (expect ≤50 ms incl. transfers at bench res). `dense: CUDA confidence estimation in geometric-consistency stage`.

---

# Phase 3 — WS3: Accuracy

### Task 15: Free-space-violation evidence in the confidence sweep

**Files:**
- Modify: `libs/MVS/SceneDensify.cpp` (sweep from Task 9; feature export :1351-1354), `libs/MVS/DepthMap.cpp/.h` (knobs)

**Interfaces:**
- Produces: `OPTDENSE::fConfViolationWeight` (λ, title `"Conf Violation Weight"`, default `"0"` = exact no-op), `OPTDENSE::fConfViolationMargin` (title `"Conf Violation Margin"`, default `"3"`); violation count `V` exported as `cfeatV` (uint16) in feature mode; posterior denominator gains `+ λ·V`. Consumed by Tasks 17/18.

- [ ] **Step 1: Implement**

In the sweep, where G1 fails (using Task 9's variables):

```cpp
if (dn <= 0) continue;
if (!IsDepthSimilar(q.z, dn, thDepthDiff)) {
    // neighbor sees a surface well BEHIND our point => its ray passes through us: violation
    if (dn > q.z * (1.f + OPTDENSE::fConfViolationMargin * thDepthDiff))
        ++V;
    continue; // dn << q.z => we are occluded in that view: neutral
}
```

Posterior: `posterior = (s*pGeo + Pconf) / (s + Pconf + OPTDENSE::fConfViolationWeight * (float)V);` — with λ=0 this is bit-identical to before. Feature export writes `cfeatV`.

- [ ] **Step 2: Verify no-op + probe**

λ=0 run → byte-identical adjusted conf vs Task 11 output (compare cfeat + final cmaps). Probe λ=0.5 on courtyard+office+1 bmvs: GT ROC/P@0.1 vs λ=0 — record in commit message (adoption happens in Task 17's sweep, not here).

- [ ] **Step 3: Commit** — `dense: free-space-violation evidence in confidence posterior (opt-in)`.

### Task 16: Soft gates + bilinear neighbor sampling (opt-in)

**Files:**
- Modify: `libs/MVS/SceneDensify.cpp`, `libs/MVS/DepthMap.cpp/.h` (knob)

**Interfaces:**
- Produces: `OPTDENSE::bConfSoftGates` (title `"Conf Soft Gates"`, default `"0"`); soft path accumulates `K_soft = Σ w`, `Pconf += w·cN` with `w = wD·wR·wN`; feature export writes the soft K/Pconf when on.

- [ ] **Step 1: Implement**

Bilinear validity-aware depth sample (member helper next to the sweep):

```cpp
static inline bool SampleDepthBilinear(const DepthMap& dm, float px, float py,
                                       float thDepthDiff, Depth& d) {
    const int x0=FLOOR2INT(px), y0=FLOOR2INT(py);
    if (x0 < 0 || y0 < 0 || x0+1 >= dm.width() || y0+1 >= dm.height()) return false;
    const Depth d00=dm(y0,x0), d01=dm(y0,x0+1), d10=dm(y0+1,x0), d11=dm(y0+1,x0+1);
    if (d00<=0 || d01<=0 || d10<=0 || d11<=0) return false;   // caller falls back to nearest
    const Depth dmin(MINF(MINF(d00,d01),MINF(d10,d11))), dmax(MAXF(MAXF(d00,d01),MAXF(d10,d11)));
    if (!IsDepthSimilar(dmin, dmax, thDepthDiff)) return false; // never interpolate across an edge
    const float wx=px-(float)x0, wy=py-(float)y0;
    d = (d00*(1.f-wx)+d01*wx)*(1.f-wy) + (d10*(1.f-wx)+d11*wx)*wy;
    return true;
}
```

Soft weights when `bConfSoftGates` (σ derived from existing thresholds so the sweep can scale them): `wD = expf(-sq((q.z-dn)/(0.5f*thDepthDiff*q.z)))`, `wR = expf(-(du*du+dv*dv)/sq(0.5f*thReproj))`, `wN = max(0, cosAngle)` (same cos as G3). A sample contributes if `w > 0.05`; `Ksoft += w; Pconf += w*cN`. Gate/posterior consume `Ksoft` in place of `K` (float). Violations stay hard (Task 15 logic on the nearest sample). Hard path must remain bit-identical when the knob is off.

- [ ] **Step 2: Verify + A/B**

Knob-off byte-identity; knob-on GT eval on 4 scenes (2 ETH3D, 2 bmvs): report ROC/P@0.1 deltas. Adoption criterion (Task 17 sweep confirms): improves pooled ROC and no scene regresses > 0.005.

- [ ] **Step 3: Commit** — `dense: soft gates + bilinear neighbor sampling for confidence (opt-in)`.

### Task 17: GT-driven recalibration sweep → new defaults

**Files:**
- Modify: `scripts/python/SweepConfParams.py`
- Modify: `libs/MVS/DepthMap.cpp` (DEFVAR defaults, only per sweep outcome)
- Create: `gt_bench/SWEEP_GT.md`

**Interfaces:**
- Consumes: feature sidecars (`cfeatK/Pconf/Prior/Photo/V`, soft variants) via 2 export runs per scene (hard + soft); GT labels via `GtUtils.gt_labels`; the offline posterior recompute (formula = SceneDensify.cpp posterior block — keep in sync).
- Produces: tuned defaults for `fConfPriorStrength, fConfConfirmTau, fConfPhotoFloor, fConfFloor, fConfViolationWeight, fConfViolationMargin, bConfSoftGates` committed to the DEFVARs; `SWEEP_GT.md` with the grid results.

- [ ] **Step 1: Extend `SweepConfParams.py`** — add GT-label mode (reuse Task 5 loaders) + new sweep dims: λ∈{0,.25,.5,1,2}, margin∈{2,3,5}, τ∈{1.5,2,3}, floor∈{0.03,0.05,0.1}, photoFloor∈{0.3,0.5,0.7}, s∈{0.5,1,2}, hard-vs-soft features. Everything recomputes OFFLINE from exported features (fast; the margin dim needs V exported per margin — export V at margins {2,3,5} in one run as 3 sidecars, or re-export per margin).

- [ ] **Step 2: Run** on 6+ scenes (all ETH3D + 2 bmvs at their bench res). Selection rule: maximize pooled GT ROC-AUC subject to (a) no scene below its current-default ROC − 0.005, (b) P@0.1 not worse pooled. Coordinate descent from current defaults is acceptable if the full grid is too big; log the path. Also check whether the winner differs materially between resolution levels (ETH3D L3 vs L1): if pooled ROC at a shared setting is >0.01 below per-resolution winners, propose per-resolution defaults (spec allows it); otherwise keep one global default.

- [ ] **Step 3: Adopt + end-to-end check** — set winning DEFVAR defaults, rebuild, re-run `--postprocess-dmaps 4` + GT eval on all sweep scenes: end-to-end must reproduce the offline prediction within ±0.003 (guards formula drift between C++ and the offline recompute).

- [ ] **Step 4: Commit** — `dense: GT-recalibrated confidence defaults` + `gt_bench/SWEEP_GT.md`.

### Task 18: Fusion FSV guard on rescued points

**Files:**
- Modify: `libs/MVS/SceneDensify.cpp` (`DenseFuseDepthMaps` :1940-2023), `libs/MVS/DepthMap.cpp/.h` (knob)

**Interfaces:**
- Produces: `OPTDENSE::nFuseViolationMax` (title `"Fuse Violation Max"`, default `"-1"` = disabled; `0` = strict) — rescued points (kept only thanks to `w·pGeo` virtual support) require `V ≤ nFuseViolationMax`.

- [ ] **Step 1: Implement**

In the fusion seed loop, where each neighbor projection is tested (read the loop; the projected-depth vs stored-depth comparison exists for the join gate): count `V` with the SAME classification as Task 15 (`dn > dproj*(1+fConfViolationMargin*thDepthDiff)`). At the keep-rule (:2021-2023):

```cpp
const bool rescued = fusedPoints.First().size() < nMinPixelsFuse ||
                     fusedViews.size() < nMinViewsFuse;   // passes only via virtualSupport
if (rescued && OPTDENSE::nFuseViolationMax >= 0 && V > (unsigned)OPTDENSE::nFuseViolationMax)
    continue; // reject: virtual-support point contradicted by a free-space ray
```

(Adapt container names to the real code.) Non-rescued points are untouched — the guard can only tighten the rescue.

- [ ] **Step 2: Measure on the bench** (fusion-only re-runs, CPU, dmaps reused): w3 with guard∈{off,0,1} — completeness + gross-outlier tables. Adopt default `0` iff gross-outliers drop (or hold) and completeness loss ≤ 0.5 pp on every scene; else keep `-1` and document.

- [ ] **Step 3: Commit** — `dense: free-space-violation guard on fusion-rescued points`.

### Task 19: Second-chance fusion pass (opt-in)

**Files:**
- Modify: `libs/MVS/SceneDensify.cpp` (`DenseFuseDepthMaps`), `libs/MVS/DepthMap.cpp/.h` (knob)

**Interfaces:**
- Produces: `OPTDENSE::bFuseSecondChance` (title `"Fuse Second Chance"`, default `"0"`).

- [ ] **Step 1: Read the rejection path first** — in the seed loop, determine exactly what happens to a seed that fails the keep-rule (are its contributing pixels marked in `useMask`/invalidated, or left available?). The second-chance design depends on it; write the answer in a comment.

- [ ] **Step 2: Implement** — during pass 1, record seeds failing the keep-rule with `fusedViews.size() >= 2 && priorMap(i,j) >= 0.5` (store image idx + pixel + nothing else; memory trivial). After the main loop, if `bFuseSecondChance`: re-run the same seed-assembly for recorded seeds with `nMinPixelsFuse-1` (floor 3) and require `V == 0` (Task 18 counting) — only unconsumed pixels can contribute (respect useMask state discovered in Step 1).

- [ ] **Step 3: Measure + adopt-or-not** — bench fusion re-runs, knob on vs off: completeness (target: visible gain on the "discarded-recoverable" scenes, e.g. Sceaux-like) vs gross-outliers (gate: ≤ +0.05 pp). Document in `gt_bench/` and set the default accordingly.

- [ ] **Step 4: Commit** — `dense: second-chance fusion pass for prior-supported discarded seeds (opt-in)`.

### Task 20: Retune fFusePriorWeight on GT + final aggregate + docs

**Files:**
- Modify: `libs/MVS/DepthMap.cpp` (w default, only if the knee moved)
- Create: `gt_bench/FINAL_2026-07.md`; update memory files

- [ ] **Step 1: Weight sweep** — fusion-only re-runs at w∈{0,1,2,3,4,5} with the adopted Task-18/19 settings, all scenes. Pick max completeness subject to gross-outliers ≤ w0 + 0.05 pp at the mid tolerance (0.5 % diag / 2 cm). Update the DEFVAR default iff ≠ 3.

- [ ] **Step 2: Final full benchmark** — re-run the whole Task-7 loop on the final build; `aggregate_gt.py`; copy to `gt_bench/FINAL_2026-07.md` next to `BASELINE_2026-07.md`; verify every Global-Constraints acceptance gate and list them checked in the doc.

- [ ] **Step 3: Docs + memory** — update the `AdjustConfidence` block comment (new formula terms), `gt_bench/README.md` (how to re-run everything); update `/home/ubuntu/.claude/projects/-home-ubuntu-openMVS/memory/confidence-recalibration-feature.md` and add a `gt-benchmark` memory (datasets, runner, verdicts, final numbers) + MEMORY.md index line.

- [ ] **Step 4: Commit** — `doc: final GT benchmark results + tuned defaults`.
