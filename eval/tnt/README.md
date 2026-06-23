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
