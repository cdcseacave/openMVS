# Tanks & Temples: tuned parameters

Best-performing OpenMVS configurations on the Tanks & Temples *training* set, and the measurements
behind them. Screening runs at `--resolution-level 0` on the **six training scenes that can be
reconstructed and scored here** - Truck (tau 5 mm), Ignatius (3 mm), Barn (10 mm), Caterpillar
(5 mm), Meetingroom (10 mm) and Church (25 mm). Poses are frozen Metashape poses throughout, so
these numbers isolate the dense pipeline; our own SfM is a separate arm.

## Recommended settings

Best configuration found for the mean F1 over the six scenes, current code, every product
re-aligned to the ground truth by the official evaluator (Courthouse, the seventh training scene with
ground truth, did not densify at `--resolution-level 0` on a 32 GB machine, §7):

| product | command | mean F1 |
|---|---|---|
| **point cloud** | `DensifyPointCloud --resolution-level 0 --number-views 24` | **0.7145** |
| **mesh** | the same densify, then `ReconstructMesh` at defaults | **0.6607** |

- **Densify:** `--number-views 24` is the only flag to add; every other fusion and confidence
  parameter is already at its best value by default (`--fusion-reprojection-threshold 0.6`,
  `--fusion-prior-weight 4` with the contradiction guard, the recalibrated confidence floor). It is
  the one knob that never loses on any scene (§2). `--number-views 16` is the cheaper setting: most
  of the gain for a third of the extra time (§3).
- **Mesh:** no `ReconstructMesh` parameter beats its defaults (§4); the mesh uses the same densify
  as the cloud.
- **Refinement:** `RefineMesh` at defaults added +0.0070 mean F1 at the previous densify defaults
  (§5, not re-measured since); its value is mostly an 8-20x smaller mesh.
- **Submit the point cloud** when the score is the goal: it leads the mesh by 0.054 mean F1 and skips
  both the mesh and the refinement stages (§6).

## 1. How the benchmark scores

The official `python_toolbox` evaluator voxel-downsamples **both** the reconstruction and the ground
truth at **tau/2** before matching (`run.py` -> `evaluation.py`). Precision and recall are therefore
area-weighted, and raw point density above ~4 samples per tau/2 surface cell buys nothing. Meshes are
sampled inside the official crop volume only; sampling the whole mesh spends the budget on background
the evaluator discards.

Every arm is aligned to the ground truth by the evaluator's own ICP, every run - a matrix refined
against one cloud would score later arms in a frame fitted to a different product, and per-arm ICP is
what the leaderboard does anyway. Two noise floors follow, and they are not the same number:

| repeat | spread | what it covers |
|---|---|---|
| re-score one fixed cloud | **0.0001** | the evaluator's ICP alone |
| re-run the whole cell | **0.002 - 0.003** | densify included; fusion output depends on free RAM |

**Rank configurations against the 0.003 figure, not the 0.0001 one.** For reference, freezing the
alignment instead of refining it per run shifts F1 by only +0.0005 (Truck/`default_r0`: 0.7399 frozen
vs 0.7404/0.7405 realigned).

## 2. The governing mechanism: where a scene sits on its own P/R curve

Every recommendation below follows from one fact. The scenes do **not** share a precision/recall
balance, and a knob's value depends entirely on which side of it a scene is on (measured at the
defaults before fusion's contradiction guard; the ordering is a property of the scenes):

| cloud at the previous defaults | P | R | slack (R - P) |
|---|---|---|---|
| Caterpillar | 0.5009 | 0.8651 | +0.3642 |
| Barn | 0.5643 | 0.8680 | +0.3037 |
| Ignatius | 0.7427 | 0.9695 | +0.2268 |
| Truck | 0.6507 | 0.8573 | +0.2066 |
| **Church** | 0.6229 | 0.6178 | **-0.0051** |
| **Meetingroom** | 0.5414 | 0.4728 | **-0.0686** |

Correlating each knob's per-scene gain against that slack separates the knobs into three kinds:

| knob | mean dF1 | r(slack, dF1) | worst scene | kind |
|---|---|---|---|---|
| `--number-views 24` | **+0.0223** | -0.50 | **+0.0020** | better estimate |
| `--number-views 16` | **+0.0182** | -0.49 | **+0.0024** | better estimate |
| `x_max` (9 flags stacked) | +0.0126 | **+0.97** | -0.0624 | filter |
| `--fusion-depth-diff-threshold 0.005` | +0.0003 | **+0.98** | -0.0116 | filter |
| `--fusion-prior-weight 0` | -0.0000 | +0.93 | -0.0373 | filter |
| `--fusion-recycle-dropped 1` | -0.0085 | **-0.93** | -0.0247 | completeness |
| `--fusion-prior-weight 4` | -0.0110 | -0.91 | -0.0314 | completeness |

**Filtering knobs have r ~ +0.9.** They discard points, which is pure profit only where recall is
surplus; they relocate a scene along its own P/R curve rather than reconstructing anything better.
**Completeness knobs are their mirror image at r ~ -0.9** and lose on average. Either kind is a bet
on the scene - unless the knob tells good points from bad ones. That is what fusion's contradiction
guard does: it keeps a weakly supported point only when no view disputes it, which turned prior
weight 4 into a gain on every scene, and with the confidence recalibration decoupled from fusion's
thresholds a 0.6 px reprojection threshold wins everywhere too. Both are defaults now
(`docs/design/DepthMapFusion.md` §5).

**`--number-views` is neither.** It changes the point count in *opposite directions* depending on the
scene - Truck -21 %, Ignatius -12 %, Barn -5 %, Caterpillar -6 %, but Meetingroom **+26 %** and
Church **+12 %** - and the two scenes where it adds points are exactly the two recall-bound ones. More
neighbour views make the depth estimate more reliable, so it removes false surface where there was
surplus and recovers real surface where there was a deficit. That is why it raises P and R together
(Meetingroom: P 0.5414 -> 0.5449, R 0.4728 -> 0.5193) and why it is the only knob that never loses on
any scene.

Screening on scenes that share a slack sign is therefore actively misleading: on Truck/Ignatius/Barn
alone the stacked filter `x_max` ranks **1st of 16**; over six scenes it ranks **5th** and is worse
than the shipped defaults on the two scenes that were missing.

## 3. Point-cloud mode

```
DensifyPointCloud --resolution-level 0 --number-views 24
```

| scene | tau | P | R | **F1** | vs the same flag at the previous defaults |
|---|---|---|---|---|---|
| Truck | 5 mm | 0.7167 | 0.8151 | **0.7627** | +0.0207 |
| Ignatius | 3 mm | 0.8430 | 0.9585 | **0.8970** | +0.0147 |
| Barn | 10 mm | 0.6452 | 0.8565 | **0.7359** | +0.0281 |
| Caterpillar | 5 mm | 0.5624 | 0.8398 | **0.6737** | +0.0372 |
| Meetingroom | 10 mm | 0.6307 | 0.4897 | **0.5513** | +0.0195 |
| Church | 25 mm | 0.7096 | 0.6284 | **0.6665** | +0.0083 |
| **mean** | | | | **0.7145** | **+0.0214** |

Full densify per scene (mean 17.3 min); re-fusing one set of depth-maps per scene at these defaults
gives 0.7148, within the 0.003 repeat spread.

The gain over the previous defaults is fusion's contradiction guard, the angle-weighted confidence
recalibration and the 0.6 px join threshold (`docs/design/DepthMapFusion.md` §5); it comes almost
entirely from precision (+0.02 to +0.09), while recall gives back at most 0.03 (Meetingroom).

`--number-views 16` is the cheaper setting and 24 the "score is all that matters" one: at the
previous defaults 16 got **82 % of the gain for 35 % of the extra densify time**.

| config | mean densify | vs default | mean dF1 | worst scene |
|---|---|---|---|---|
| `default_r0` | 10.1 min | 1.00x | - | - |
| **`--number-views 16`** | 11.9 min | 1.18x | +0.0182 | +0.0024 |
| **`--number-views 24`** | 15.3 min | 1.51x | +0.0223 | +0.0020 |

Everything else is a no-op or a bet. Genuine no-ops: `--number-views-fuse 3` (the dense-fuse path
ignores it - it changed the cloud by 5 points out of 34.8 M), `--sub-resolution-levels 3` (clamped by
`--min-resolution 640`), `--geometric-iters 4`, ROI/tower off (+0.0008).

## 4. Mesh mode

Same densify as section 3 (the same run), `ReconstructMesh` at defaults:

| scene | P | R | **F1** | faces | in-crop faces | recon |
|---|---|---|---|---|---|---|
| Truck | 0.6882 | 0.6858 | **0.6870** | 11.09 M | 7.03 M | 450 s |
| Ignatius | 0.8864 | 0.8665 | **0.8763** | 9.74 M | 2.16 M | 635 s |
| Barn | 0.6565 | 0.6796 | **0.6678** | 20.66 M | 10.38 M | 889 s |
| Caterpillar | 0.4545 | 0.7307 | **0.5605** | 11.44 M | 6.82 M | 404 s |
| Meetingroom | 0.6431 | 0.4184 | **0.5070** | 11.99 M | 11.93 M | 408 s |
| Church | 0.7788 | 0.5813 | **0.6657** | 15.61 M | 15.61 M | 595 s |
| **mean** | | | **0.6607** | | | 564 s |

The six-scene mean at the same flag and the previous defaults was 0.6542, so the mesh keeps a
smaller share of the cloud gain (+0.0065 against +0.0214): it is recall-bound, and the cloud gain is
precision.

The mesh stage itself is not a lever: ten `ReconstructMesh` knobs swept over one frozen cloud all
land within +-0.004 except `--free-space-support 1` at **-0.0398**. `--smooth 0` was the one apparent
gain (+0.0082 on three scenes) but it is a recall bet of the same kind as section 2's filters, and it
collapses refinement (-0.0423 mean); stock smoothing is the safe setting.

The mesh **damps** whatever the cloud does, which is why a cloud bet looks safer here than it is: on
Meetingroom `x_max` costs -0.0624 of cloud F1 but only -0.0021 of mesh F1. Damping a loss is not
avoiding it - `c_v24`, which never takes the bet, still wins the arm.

## 5. Mesh + refinement

`RefineMesh` at defaults on the section 4 mesh, measured at the previous densify defaults.

| scene | **F1** | vs its own mesh | vertices | faces | refine |
|---|---|---|---|---|---|
| Truck | **0.7019** | +0.0203 | 0.22 M | 0.39 M | 522 s |
| Ignatius | **0.8646** | -0.0044 | 0.28 M | 0.49 M | 770 s |
| Barn | **0.6976** | +0.0304 | 0.89 M | 1.65 M | 1910 s |
| Caterpillar | **0.5496** | +0.0014 | 0.60 M | 1.09 M | 1950 s |
| Meetingroom | **0.4990** | -0.0053 | 0.97 M | 1.61 M | 1887 s |
| Church | **0.6545** | -0.0002 | 0.96 M | 1.66 M | 4584 s |
| **mean** | **0.6612** | **+0.0070** | | | |

Refinement's value is mostly **size**, not score: it cuts the face count by 8-20x (the 0.25 px
simplify tolerance) for +0.0070 mean F1, and on three of six scenes it is neutral or slightly
negative. Run it when a compact surface is the product, not to chase F1.

## 6. Summary and which mode to submit

Six-scene mean F1 at `--resolution-level 0` (all rows but the first at the previous defaults):

| config | cloud | mesh | mesh + refine |
|---|---|---|---|
| **`--number-views 24`, shipped defaults** | **0.7145** | **0.6607** | - |
| `default_r0` | 0.6708 | 0.6327 | 0.6483 |
| `--number-views 24` | 0.6931 | 0.6542 | 0.6612 |
| `--number-views 16` | 0.6892 | 0.6481 | 0.6527 |
| `x_max` (9 stacked filter flags) | 0.6834 | 0.6509 | 0.6589 |

**`--number-views 24` wins every arm**, and it is one flag against `x_max`'s nine. In cloud mode the
margin over `x_max` is a clear +0.0097; in mesh (+0.0033) and refine (+0.0023) the two are level
within the 0.003 repeat spread, and `c_v24` wins on simplicity - `x_max` also disables ROI
estimation, cropping and tower mode, which have consequences beyond F1.

**Point-cloud mode wins outright** (0.7145 vs 0.6607 for the mesh at the shipped defaults; 0.6931 vs
0.6612 with refinement at the previous ones), and is by far the cheapest: it skips both the mesh and
the refinement stage. Build a mesh only where a surface is required rather than a point set.

## 7. Scenes that do not run here

- **Courthouse** (1106 images) cannot densify at `--resolution-level 0` on a 32 GB machine:
  `DensifyPointCloud` dies with `0xC0000005` after ~38 min in fusion, the log repeating
  "not enough memory to cache depth-maps (2772MB needed, ~1.2GB available)" - an out-of-memory
  condition surfacing as an access violation rather than a clean failure. It scores at
  `--resolution-level 1` (cloud 0.4939 at defaults).
- **Church** needed a dataset repair first. The toolbox pairs the reconstruction's camera trajectory
  with `Church_COLMAP_SfM.log` index-by-index, but that log has 644 poses while the Metashape project
  here aligned only the 507 images present, so alignment raised `unequal length 507 != 644`. The 507
  are a named subset - `metashape.xml` records each pose's original `images/000NNN.jpg`, so reference
  entry NNN-1 is its counterpart. `bench/tnt_church_reference.py` writes that subset over the
  reference log (the shipped file is kept as `Church_COLMAP_SfM.orig.log`); the pairing is confirmed
  by a 0.988 RANSAC fitness. Because only 507 of 644 images are reconstructed, Church's absolute F1
  is pessimistic - its *ranking* of configurations is unaffected, since every config sees the same
  images.
- **Palace** has no ground truth in this dataset copy.
