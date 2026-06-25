# Tanks-and-Temples benchmark — Metal PatchMatch vs CPU vs CUDA

Reproducible evaluation harness for PR
[cdcseacave/openMVS#1279](https://github.com/cdcseacave/openMVS/pull/1279)
(Metal PatchMatch backend). It produces an objective **F-score** quality
comparison between the Metal backend, the CPU `DepthEstimator`, and the CUDA
PatchMatch backend on the Tanks-and-Temples **training** set (the split with
public ground truth, so F1 is computable locally).

## Methodology — one variable changed

The whole point is that the table compares **the densification backend and
nothing else**:

1. **SfM is run once per scene** (`prepare_scene.sh`) → a single shared
   `scene.mvs`.
2. Every backend densifies **that identical `scene.mvs`** with the **identical
   fixed parameters** in `lib.sh` (`DENSIFY_PARAMS`). Metal is the default on
   Apple; `OPENMVS_DISABLE_METAL=1` forces the CPU path.
3. **Evaluation is held constant**: the *same* ground truth, crop volume and
   *same* toolbox version (`isl-org/TanksAndTemples`) score every backend's
   fused cloud (`evaluate_f1.sh`).

Timings are wall-clock on each contributor's machine and are deliberately
**not** treated as cross-architecture comparable (per the maintainer's note);
the F-score is the hardware-independent quality signal.

## Local run (Metal + CPU, Apple Silicon)

```bash
./setup.sh                 # build apps, install colmap + TnT toolbox
./download_tnt.sh          # fetch training images + public ground truth
./run_local.sh             # SfM once, densify Metal & CPU, evaluate, emit table
# -> $TNT_ROOT/RESULTS.md
```

Scene list is configurable: `SCENES="Barn Truck" ./run_local.sh`.

## Filling the CUDA column (community help)

We cannot run CUDA on Apple Silicon. So the CUDA column is crowd-sourced with a
**minimal ask**: a volunteer densifies the shared `scene.mvs` we publish and
sends back the fused PLY + timing — they need **no** ground truth and **no**
evaluation toolbox. We then run the *same* F1 evaluation on their cloud, so the
result drops straight into the table on equal footing.

Volunteer side:

```bash
./contribute_cuda.sh /path/to/DensifyPointCloud_CUDA Truck ./Truck/scene.mvs 0
# -> Truck_cuda_result.tar.gz  (attach to the PR thread)
```

Our side, when a result comes in:

```bash
tar -xzf Truck_cuda_result.tar.gz -C $TNT_ROOT/Truck/out_cuda/
./evaluate_f1.sh Truck $TNT_ROOT/Truck/out_cuda/scene_dense.ply cuda
python3 make_table.py $TNT_ROOT $SCENES   # CUDA column now populated
```

`evaluate_f1.sh` derives the reconstruction's TnT trajectory from the shared
`scene.mvs` itself (`InterfaceCOLMAP --binary 0` → `colmap_to_tnt_log.py`), so it
works from just the published tarball + ground truth — no `colmap/sparse/0`
needed. Note: the `scene.mvs` lives in its own COLMAP frame, so do **not** pass
the GT's `<Scene>_COLMAP_SfM.log` as the trajectory — it has the same entry count
but different poses, which silently breaks the alignment and collapses F1 (~0.01).

## Results — CUDA column filled

The CUDA column was produced on an NVIDIA GPU (i7-13700KF host) by densifying the
**exact published shared `scene.mvs`** with the **identical** `DENSIFY_PARAMS`,
and scored by the same toolbox at the same per-scene τ. The CUDA trajectory was
derived from the shared `scene.mvs` (`InterfaceCOLMAP --binary 0` export →
`colmap_to_tnt_log.py`); entry counts match the official `_COLMAP_SfM.log`
exactly for all four scenes.

F-score @ official τ (only the PatchMatch backend differs):

| Scene | metal | cpu | cuda |
|---|---|---|---|
| Barn | 0.409 | 0.371 | **0.424** |
| Ignatius | 0.657 | 0.608 | **0.668** |
| Meetingroom | 0.239 | 0.215 | **0.253** |
| Truck | 0.647 | 0.593 | **0.649** |
| **mean** | 0.488 | 0.447 | **0.499** |

CUDA and Metal agree to within ~0.01–0.015 F1 on every scene (CUDA marginally
ahead), with near-identical precision/recall — confirming the Metal port
reproduces the CUDA reference, not merely the separate CPU `DepthEstimator`. Full
P/R/points/wall and an independent-SfM cross-check are in the PR discussion.

## Files

| Script | Role |
|---|---|
| `lib.sh` | shared paths, scene list, **fixed densify params** |
| `setup.sh` | build apps, install colmap + TnT toolbox |
| `download_tnt.sh` | fetch training images + ground truth |
| `prepare_scene.sh` | SfM once → **shared `scene.mvs`** |
| `densify.sh` | densify+fuse one scene/backend, record time+points |
| `evaluate_f1.sh` | TnT toolbox → precision/recall/F1 |
| `run_local.sh` | orchestrate Metal+CPU, emit markdown table |
| `contribute_cuda.sh` | **standalone** script for NVIDIA volunteers |
| `pack_scene_mvs.sh` | bundle shared `scene.mvs` + images for volunteers |
| `make_table.py` | collate JSON → markdown (grows as CUDA arrives) |

## Attribution

The datasets and ground truth are from the
[Tanks and Temples](https://www.tanksandtemples.org/) benchmark (Knapitsch,
Park, Zhou, Koltun, *"Tanks and Temples: Benchmarking Large-Scale Scene
Reconstruction"*, ACM Transactions on Graphics, 2017). Any shared `scene.mvs`
bundles undistorted frames **derived** from the TnT training set, redistributed
for non-commercial research under
[CC BY-NC-SA](https://creativecommons.org/licenses/by-nc-sa/3.0/). Evaluation
uses the official [isl-org/TanksAndTemples](https://github.com/isl-org/TanksAndTemples)
Python toolbox.
