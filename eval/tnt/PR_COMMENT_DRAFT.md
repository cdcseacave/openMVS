<!-- Draft reply for cdcseacave/openMVS#1279 -->

@cdcseacave thanks! I've added a fully reproducible benchmark harness under
[`eval/tnt/`](eval/tnt/README.md) and ran the first comparison.

### Methodology (one variable changed)

SfM is run once per scene → a single shared `scene.mvs`; every backend densifies
*that identical file* with *identical* fixed parameters, and the *same* GT + the
official [TanksAndTemples](https://github.com/isl-org/TanksAndTemples) toolbox
scores every result. So the F-score isolates the backend, not the configuration.

### Metal vs CPU — Tanks-and-Temples training set (F-score @ default τ)

Both backends ran on the **same machine** (Apple M4 Max, 48 GB), so here the
wall-times *are* directly comparable; the CPU path is the multithreaded
`DepthEstimator`, Metal is the new PatchMatch backend.

| Scene | Backend | F1 ↑ | Precision | Recall | Points | Wall (s) |
|---|---|---|---|---|---|---|
| Barn | metal | **0.409** | 0.378 | 0.446 | 6.87M | 64.7 |
| Barn | cpu | 0.371 | 0.326 | 0.430 | 7.87M | 481.0 |
| Ignatius | metal | **0.657** | 0.592 | 0.738 | 4.39M | 38.6 |
| Ignatius | cpu | 0.608 | 0.525 | 0.720 | 5.07M | 348.7 |
| Meetingroom | metal | **0.239** | 0.417 | 0.168 | 2.68M | 55.2 |
| Meetingroom | cpu | 0.215 | 0.331 | 0.159 | 3.40M | 470.3 |
| Truck | metal | **0.647** | 0.597 | 0.707 | 4.97M | 40.2 |
| Truck | cpu | 0.593 | 0.519 | 0.693 | 5.49M | 366.4 |
| **mean** | **metal** | **0.488** | | | | **49.7** |
| **mean** | **cpu** | 0.447 | | | | 416.6 |

**Takeaways**
- The Metal backend **matches or slightly exceeds** the CPU `DepthEstimator`
  F-score on every scene (mean **0.488 vs 0.447**), at **~8× faster** wall time
  on identical hardware.
- Parity holds even on the hard indoor scene (Meetingroom), where both backends
  have low absolute recall — Metal is still ahead.
- The CPU path produces more raw points but with lower precision; Metal's clouds
  are leaner and slightly more accurate after fusion.

This corroborates the earlier depth-map analysis in this PR (median depth
agreement ~0.1% where both are valid; Metal depth-maps actually *more* complete).

### CUDA column — call for help

I don't have an NVIDIA GPU, so I can't produce the CUDA PatchMatch numbers
locally. To keep it perfectly controlled, the harness publishes the shared
`scene.mvs` files and a one-shot
[`contribute_cuda.sh`](eval/tnt/contribute_cuda.sh): a volunteer densifies the
*same* `scene.mvs` with a CUDA build and sends back the fused PLY + timing — no
GT or toolbox needed on their side; I run the identical F1 evaluation here so the
result drops straight into the table. If you or anyone with a CUDA box can run it
on even one or two of these scenes, the comparison is complete.

Shared inputs (scene.mvs + undistorted images): _<link>_.

> The shared `scene.mvs` bundles undistorted frames derived from the
> [Tanks and Temples](https://www.tanksandtemples.org/) training dataset
> (Knapitsch et al., *ACM ToG* 2017), redistributed for non-commercial
> research under [CC BY-NC-SA](https://creativecommons.org/licenses/by-nc-sa/3.0/).
