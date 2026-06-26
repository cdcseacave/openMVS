<!-- Draft reply for cdcseacave/openMVS#1279 -->

@cdcseacave thanks! I've added a fully reproducible benchmark harness under
[`eval/tnt/`](https://github.com/leNeo/openMVS-metal/blob/metal-patchmatch-backend/eval/tnt/README.md) and ran the first comparison.

### Methodology (one variable changed)

SfM is run once per scene → a single shared `scene.mvs`; every backend densifies
*that identical file* with *identical* fixed parameters, and the *same* GT + the
official [TanksAndTemples](https://github.com/isl-org/TanksAndTemples) toolbox
scores every result. So the F-score isolates the backend, not the configuration.

### Metal vs CPU vs CUDA — Tanks-and-Temples training set (F-score @ official τ)

All three backends densify the **same** shared `scene.mvs` with the **same**
params (`-R1 --min-resolution 640 --max-resolution 3200 --number-views 5
--fusion-mode 0`), scored by the same toolbox at the official per-scene τ. The
Metal/CPU columns ran on an Apple M4 Max (48 GB) so their wall-times are directly
comparable; the CUDA column ran on an NVIDIA GPU (i7-13700KF host), so CUDA
wall-times are **not** cross-architecture comparable — the F-score is the
hardware-independent signal.

| Scene | Backend | F1 ↑ | Precision | Recall | Points | Wall (s) |
|---|---|---|---|---|---|---|
| Barn | metal | 0.409 | 0.378 | 0.446 | 6.87M | 64.7 |
| Barn | cpu | 0.371 | 0.326 | 0.430 | 7.87M | 481.0 |
| Barn | **cuda** | **0.424** | 0.387 | 0.471 | 7.16M | 253.8 |
| Ignatius | metal | 0.657 | 0.592 | 0.738 | 4.39M | 38.6 |
| Ignatius | cpu | 0.608 | 0.525 | 0.720 | 5.07M | 348.7 |
| Ignatius | **cuda** | **0.668** | 0.602 | 0.750 | 4.55M | 71.6 |
| Meetingroom | metal | 0.239 | 0.417 | 0.168 | 2.68M | 55.2 |
| Meetingroom | cpu | 0.215 | 0.331 | 0.159 | 3.40M | 470.3 |
| Meetingroom | **cuda** | **0.253** | 0.403 | 0.184 | 3.11M | 105.9 |
| Truck | metal | 0.647 | 0.597 | 0.707 | 4.97M | 40.2 |
| Truck | cpu | 0.593 | 0.519 | 0.693 | 5.49M | 366.4 |
| Truck | **cuda** | **0.649** | 0.600 | 0.708 | 5.09M | 69.9 |
| **mean** | metal | 0.488 | | | | 49.7 |
| **mean** | cpu | 0.447 | | | | 416.6 |
| **mean** | **cuda** | **0.499** | | | | |

**Takeaways**
- **CUDA and Metal land on top of each other** — within ~0.01–0.015 F1 on every
  scene (mean **0.499 vs 0.488**), CUDA a hair ahead on all four, with
  near-identical precision/recall splits (e.g. Truck 0.600/0.708 cuda vs
  0.597/0.707 metal). This is the confirmation the PR was missing: the Metal port
  reproduces the **CUDA reference**, not just the separate CPU `DepthEstimator`.
- Both GPU backends clearly beat the CPU `DepthEstimator` (mean 0.447), including
  the hard indoor scene (Meetingroom), where absolute recall is low for all.
- The CPU path produces more raw points but with lower precision; the GPU clouds
  are leaner and slightly more accurate after fusion.

This corroborates the earlier depth-map analysis in this PR (median depth
agreement ~0.1% where both are valid; Metal depth-maps actually *more* complete).

### CUDA column — how it was filled

The CUDA numbers above densify the *same* published `scene.mvs` files with a CUDA
build, so they drop straight into the table on equal footing. The trajectory the
toolbox needs was derived from each shared `scene.mvs`
(`InterfaceCOLMAP --binary 0` export → `colmap_to_tnt_log.py`); the resulting
entry counts match the official `_COLMAP_SfM.log` exactly for all four scenes
(Barn 410, Ignatius 263, Meetingroom 371, Truck 251).

<details>
<summary>Cross-check — CUDA on an independent SfM (local Metashape `scene.mvs`)</summary>

Same densify params and toolbox, but a **different** SfM front-end (Metashape,
not the COLMAP shared `scene.mvs`). Not comparable to the table above — included
to show the backend is stable across a second, independent reconstruction of the
same scenes:

| Scene | F1 ↑ | Precision | Recall | Points | Wall (s) | τ |
|---|---|---|---|---|---|---|
| Barn | 0.559 | 0.543 | 0.575 | 7.43M | 108.9 | 0.010 |
| Ignatius | 0.706 | 0.668 | 0.749 | 4.43M | 76.0 | 0.003 |
| Meetingroom | 0.345 | 0.523 | 0.257 | 3.83M | 158.1 | 0.010 |
| Truck | 0.695 | 0.672 | 0.720 | 5.26M | 79.0 | 0.005 |

The absolute F1 is higher here purely because the Metashape SfM is a stronger
starting point — which is exactly why the main table pins a single shared
`scene.mvs`.
</details>

Shared inputs (scene.mvs + undistorted images), one self-contained tarball per
scene — [**release: tnt-bench-inputs**](https://github.com/leNeo/openMVS-metal/releases/tag/tnt-bench-inputs):
[Barn](https://github.com/leNeo/openMVS-metal/releases/download/tnt-bench-inputs/Barn_mvs.tar.gz) ·
[Ignatius](https://github.com/leNeo/openMVS-metal/releases/download/tnt-bench-inputs/Ignatius_mvs.tar.gz) ·
[Meetingroom](https://github.com/leNeo/openMVS-metal/releases/download/tnt-bench-inputs/Meetingroom_mvs.tar.gz) ·
[Truck](https://github.com/leNeo/openMVS-metal/releases/download/tnt-bench-inputs/Truck_mvs.tar.gz)

> The shared `scene.mvs` bundles undistorted frames derived from the
> [Tanks and Temples](https://www.tanksandtemples.org/) training dataset
> (Knapitsch et al., *ACM ToG* 2017), redistributed for non-commercial
> research under [CC BY-NC-SA](https://creativecommons.org/licenses/by-nc-sa/3.0/).
