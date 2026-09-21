# Tanks & Temples: tuned parameters

Best-performing OpenMVS configurations on the Tanks & Temples *training* set, and the measurements
behind them. Screening runs on **Truck (tau 5 mm), Ignatius (3 mm) and Barn (10 mm)** - three scenes
spanning the tolerance range - at `--resolution-level 0`. Poses are frozen Metashape poses
throughout, so these numbers isolate the dense pipeline; our own SfM is a separate arm.

## 1. How the benchmark scores

The official `python_toolbox` evaluator voxel-downsamples **both** the reconstruction and the ground
truth at **tau/2** before matching (`run.py` -> `evaluation.py`). Precision and recall are therefore
area-weighted, and raw point density above ~4 samples per tau/2 surface cell buys nothing. Meshes are
sampled inside the official crop volume only; sampling the whole mesh spends the budget on background
the evaluator discards.

Repeat-eval noise with a frozen scene->GT transform is ~1e-4, so deltas above ~0.002 are real.

## 2. Recall is saturated; precision is the binding constraint

This is the single fact that determines every recommendation below. At the shipped defaults the
cloud already recalls **0.90 mean** (0.97 on Ignatius) while precision sits at **0.65**:

| `default_r0` cloud | precision | recall | F1 |
|---|---|---|---|
| Truck | 0.6507 | 0.8573 | 0.7399 |
| Ignatius | 0.7427 | 0.9695 | 0.8411 |
| Barn | 0.5643 | 0.8680 | 0.6840 |
| **mean** | **0.6526** | **0.8983** | **0.7550** |

There is almost no recall left to win, so every parameter that buys completeness with precision
loses F1. Measured on the same three scenes, `--fusion-recycle-dropped 1` (+9.4 M points) costs
-0.0104 and `--fusion-prior-weight 4` (+17.1 M points) costs -0.0145. The winning direction is the
opposite one: **fewer, cleaner points**.

A related corollary, tested directly: a noisier but more complete cloud does **not** make a better
mesh. Across 19 densify configurations the mesh F1 tracks the cloud F1 at **r = +0.97**, while point
count correlates with mesh F1 at **r = -0.58**. The best cloud is also the best mesh input.

## 3. Point-cloud mode

```
DensifyPointCloud --resolution-level 0 --number-views 16 --fusion-prior-weight 0 \
                  --fusion-reprojection-threshold 0.6 --fusion-depth-diff-threshold 0.005 \
                  --estimate-roi 0 --crop-to-roi 0 --tower-mode 0
```

| scene | tau | P | R | **F1** | points | vs default | densify |
|---|---|---|---|---|---|---|---|
| Truck | 5 mm | 0.7343 | 0.7912 | **0.7617** | 22.3 M | +0.0218 | 564 s |
| Ignatius | 3 mm | 0.8548 | 0.9207 | **0.8865** | 17.8 M | +0.0454 | 445 s |
| Barn | 10 mm | 0.6682 | 0.7691 | **0.7151** | 27.3 M | +0.0311 | 661 s |
| **mean** | | **0.7524** | **0.8270** | **0.7878** | | **+0.0328** | |

It reaches a higher F1 with **36 % fewer points** than the default (22.3 M vs 34.8 M on Truck) and is
no slower. Precision rises 0.10 for 0.07 of recall.

Per-knob contribution, measured one at a time against `default_r0`:

| knob | dF1 cloud | note |
|---|---|---|
| `--number-views 16` | +0.0173 | strongest single knob; 12 gives +0.0094, 24 only +0.0224 for +40 % densify time |
| `--fusion-prior-weight 0` | +0.0092 | monotonic: 4 -> -0.0145, 3 (default) -> 0, 2 -> +0.0070, <=1 -> +0.0092 |
| `--fusion-reprojection-threshold 0.6` | +0.0079 | 0.8 gives +0.0039 |
| `--fusion-depth-diff-threshold 0.005` | +0.0034 | |
| ROI/tower off | +0.0014 | |

`--fusion-prior-weight` **1 is byte-identical to 0**: the fusion keep-rule is
`fusedViews.size() + weight*prior >= nMinViewsFuse(2)`, so with `prior < 1` a weight of 1 can never
rescue a one-view cluster. Its shipped default of 3 is documented as favouring completeness "when a
mesh reconstruction step follows"; on this benchmark that costs F1 in **both** modes.

No-ops worth knowing: `--number-views-fuse 3` (the dense-fuse path ignores it - it changed the cloud
by 5 points out of 34.8 M), `--sub-resolution-levels 3` (clamped by `--min-resolution 640`),
`--geometric-iters 4`.

## 4. Mesh mode

Same densify as section 3. `--smooth 0` is the only `ReconstructMesh` knob that pays, and only when
the mesh is the final product: after `RefineMesh` the two settings tie, and stock smoothing gets
there with ~30 % fewer faces.

**Mesh only** (`ReconstructMesh --smooth 0`, no refinement):

| scene | P | R | **F1** | vertices | faces | in-crop faces | in-crop area |
|---|---|---|---|---|---|---|---|
| Truck | 0.7168 | 0.7129 | **0.7148** | 4.35 M | 8.63 M | 5.55 M | 212.3 |
| Ignatius | 0.8532 | 0.8649 | **0.8590** | 3.34 M | 6.64 M | 1.40 M | 95.0 |
| Barn | 0.6370 | 0.7027 | **0.6683** | 6.81 M | 13.54 M | 7.20 M | 1566.0 |
| **mean** | **0.7357** | **0.7602** | **0.7474** | | | | |

Taubin smoothing shrinks the surface - in-crop area falls from 212.3 to 181.6 on Truck - and the
lost area is recall (+0.0050 F1 for `--smooth 0`). Nine other `ReconstructMesh` knobs were swept over
one frozen cloud and all land within +-0.004, except `--free-space-support 1` at **-0.0398**.

**Mesh + refinement** (`ReconstructMesh` and `RefineMesh` both at defaults):

| scene | P | R | **F1** | vertices | faces | refine |
|---|---|---|---|---|---|---|
| Truck | - | - | **0.7185** | 0.18 M | 0.31 M | ~9 min |
| Ignatius | - | - | **0.8612** | 0.26 M | 0.48 M | ~13 min |
| Barn | - | - | **0.6965** | 0.84 M | 1.57 M | ~50 min |
| **mean** | **0.7714** | **0.7467** | **0.7587** | | | |

Refinement adds **+0.0163** over the mesh it receives and cuts the face count by ~20x (the 0.25 px
simplify tolerance). Adding `--smooth 0` on top scores 0.7590 - the same within the 0.002 noise floor
- but keeps 0.45 M faces on Truck instead of 0.31 M, so stock smoothing is the better trade once
refinement runs.

The mesh stage is not where the gains are: switching the *cloud* from `default_r0` to `x_max` is worth
+0.0198 on mesh F1 with stock mesh parameters, more than any mesh knob.

## 5. Summary and which mode to submit

3-scene mean F1 at `--resolution-level 0`:

| config | cloud | mesh | mesh + refine |
|---|---|---|---|
| `default_r0` (shipped defaults) | 0.7550 | 0.7226 | 0.7463 |
| `c_v24` (`--number-views 24` only) | 0.7774 | 0.7393 | 0.7547 |
| **`x_max`** (section 3) | **0.7878** | 0.7424 | **0.7587** |
| `x_max` + `--smooth 0` | - | **0.7474** | 0.7590 |

**Point-cloud mode wins outright, 0.7878 vs 0.7587**, and is by far the cheapest: it skips both the
mesh and the refinement stage. Submit a mesh only where a surface is required rather than a point
set, and then run refinement - it is worth +0.0163 and shrinks the result ~20x.

Every arm gains from the same densify parameters, so there is no separate "cloud-tuned" and
"mesh-tuned" configuration to maintain.
