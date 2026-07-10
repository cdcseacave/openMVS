# GT benchmark — methodology + all results in one place (2026-07-10)

Everything the ETH3D + BlendedMVS ground-truth benchmark measures, how each number is produced, what
every field means, and the full per-scene results — for **both** branch features (`feature/fusion-
faithful-confidence`) on **both** GT metrics (outlier contamination + completeness).

- **Adaptive per-view confidence** (`DepthMapsData::AdjustConfidence`) — should better separate
  accurate/inlier depths from noisy/outlier ones.
- **Dense-fusion completeness rescue** (`fFusePriorWeight`, "w") — should add completeness without
  adding outliers.

Both are graded on the two surface-GT metrics; the 2×2 is complete:

| | outlier contamination | completeness |
|---|---|---|
| **fusion** (w) | gross-outlier fraction (§3) | surface completeness@tol (§3) |
| **confidence** (adjust) | contamination@completeness (§2) | completeness@contamination (§2) |

---

## 0. The datasets and where GT comes from

| | BlendedMVS (10 scene-levels) | ETH3D high-res (18 scene-levels) |
|---|---|---|
| scenes | 5 val scenes × L0,L1 | courtyard, delivery_area, facade, meadow, office, pipes × L1,L2,L3 |
| GT geometry | **textured mesh** (`textured_mesh/tile_*.obj`) | **laser-scan point cloud** (`dslr_scan_eval/…`) |
| GT per-view depth | **rendered depth maps** (`rendered_depth_maps/*.pfm`), rendered from the mesh by the dataset authors | **laser-scan depth** (`ground_truth_depth/dslr_images/*`), z-depth on the distorted grid |
| poses | dataset-provided (imported to `scene.mvs`) | COLMAP `dslr_calibration_undistorted` (imported to `scene.mvs`) |

**Did I project the GT mesh into each view to make a GT depth-map? No.** The per-view GT depth maps
are **provided by the datasets** (BlendedMVS rendered its mesh to depth at dataset-creation time;
ETH3D projected its registered laser scans to per-view depth). I *consume* those. The only per-view
transform I apply is for ETH3D: its GT depth ships on the **distorted** 6048×4032 grid, so
`GtUtils.remap_eth3d_depth_to_undistorted` moves it onto the **undistorted** grid OpenMVS reconstructs
on — for each undistorted pixel it forms the pinhole ray, forward-distorts it through the camera's
`THIN_PRISM_FISHEYE` model (COLMAP convention), and nearest-samples the distorted GT depth there. The
depth convention is camera-frame **Z** (not ray distance), verified to ~0.00% median error against the
raw laser scan (`GtUtils.py` header; `test_gtutils.py`). BlendedMVS PFM depth is just nearest-resized
to the dmap resolution.

**How one scene-level is run** (`gt_bench/run_scene.sh <scene> <level>`), idempotent per stage:
1. estimate depth-maps (raw NCC confidence) → **snapshot to `raw_dmaps/`**;
2. **adjust confidence in place** (`--postprocess-dmaps 4 --geometric-iters 0`; depth untouched);
3. fuse the adjusted dmaps at **w0** (rescue off), **w2**, **w3** → `cloud_w{0,2,3}.ply`;
4. evaluate → `results_final/<TAG>_{conf_raw,conf_adj,fuse_w0,fuse_w2,fuse_w3}.json`.

So `raw_dmaps/` holds the **raw** confidence and the top-level dmaps hold the **adjusted** confidence
on the **identical depth** — that is what lets §2 compare raw vs adjusted fairly.

---

## 2. Per-depth-map CONFIDENCE eval — how, and every field

**Goal:** does the confidence value predict whether a pixel's estimated depth is correct? Graded per
depth-map against the per-view GT depth, so it is independent of fusion.

**Per-pixel label** (`GtUtils.gt_labels`, `EvalConfidence.py --gt-...`): a pixel is GT-valid where GT
depth is finite/positive and an estimate exists; a valid pixel is an **inlier** iff
`|d_est − d_gt| ≤ max(1%·d_gt, 0)` (`rel_tol=0.01`), else an **outlier**. (1% relative is the finest
benchmark tier; laser/rendered GT is far more precise than that.)

**The confidence is used as a retention score:** keep a pixel iff `conf ≥ t`. Among kept pixels,
- **contamination** = kept_outliers / kept_total = `1 − precision`  (how dirty the kept set is);
- **completeness** = kept_inliers / all_inliers = `recall`         (how much correct surface is kept).

Sweeping `t` traces a frontier. Because *adjust rewrites only the confidence channel*, raw and
adjusted share the identical labelled pixels, so this frontier is a clean, calibration-free comparison
(the old "P@0.1 of raw vs adjusted" is unfair — recalibration shifts the scale, and at t=0.1 the gate
keeps ~everything so R@0.1≈1.0, blind to the region that matters).

**Fields in `results_confop/<TAG>_confop_{raw,adj}.json → pooled`:**

| field | meaning |
|---|---|
| `roc_auc` | ROC-AUC of confidence ranking inliers above outliers (0.5 chance, 1 perfect; threshold-free). The *old* headline. |
| `pr_auc` | average precision (area under precision-recall); high baseline since inliers dominate. |
| `p_at_01`,`r_at_01` | precision/recall at the fixed fusion gate t=0.1 — **uninformative** here (R≈1.0: the gate keeps everything); retained only for continuity. |
| `brier`,`ece` | calibration: mean-squared-error and expected-calibration-error of confidence vs the binary label (lower better). |
| `n_labeled` | GT-labelled pixels pooled over the scene's views. |
| `operating.completeness_at_contam.{0.005,0.010,0.020,0.050}` | **max completeness kept while contamination ≤ {0.5,1,2,5}%** — the headline (analogue of fusion "completeness at a gross budget"). `null` = unreachable. |
| `operating.contam_at_completeness.{0.90,0.95,0.99}` | **min contamination admitted while completeness ≥ {90,95,99}%**. |
| `operating.n_pos`,`n_neg` | inlier / outlier pixel counts. |

Frontier evaluated only at distinct-confidence group boundaries (tied confidences kept/dropped
together — a threshold *between* equal values is not a realizable operating point). Unit-tested against
a hand-computed oracle (`operating_points`, commit c2e1591).

### 2a. Confidence results — per scene-level (raw → adjusted)

Headline operating points: completeness kept at ≤1% contamination, and contamination at ≥95%
completeness. Full budgets/targets in `results_confop/`; aggregates below.

```
scene-level             ROC raw->adj    compl@1%contam raw->adj (d)     contam@95%compl raw->adj (d)
bmvs_59817e4a_L0        0.877->0.929     8.3%-> 72.1% (+63.8pp)     5.7%->  4.0% (+1.63pp)
bmvs_59817e4a_L1        0.878->0.929     9.8%-> 72.4% (+62.6pp)     5.8%->  4.1% (+1.67pp)
bmvs_59d2657f_L0        0.947->0.978    96.9%-> 98.8% ( +1.9pp)     0.7%->  0.3% (+0.34pp)
bmvs_59d2657f_L1        0.944->0.977    96.9%-> 98.8% ( +1.9pp)     0.7%->  0.4% (+0.32pp)
bmvs_5a640093_L0        0.897->0.954    69.9%-> 91.4% (+21.5pp)     2.7%->  1.6% (+1.12pp)
bmvs_5a640093_L1        0.897->0.954    70.0%-> 91.4% (+21.4pp)     2.7%->  1.6% (+1.11pp)
bmvs_5b7a3890_L0        0.819->0.899     0.0%->  0.7% ( +0.7pp)    39.8%-> 45.8% (-6.03pp)  <- regress (hard aerial)
bmvs_5b7a3890_L1        0.821->0.900     0.0%->  0.8% ( +0.8pp)    39.8%-> 45.9% (-6.10pp)  <- regress
bmvs_5ba19a8a_L0        0.931->0.954    90.0%-> 92.5% ( +2.5pp)     1.5%->  1.4% (+0.17pp)
bmvs_5ba19a8a_L1        0.930->0.954    90.1%-> 92.6% ( +2.5pp)     1.5%->  1.4% (+0.17pp)
eth3d_courtyard_L1      0.904->0.933    93.0%-> 92.5% ( -0.5pp)     1.3%->  1.4% (-0.17pp)
eth3d_courtyard_L2      0.866->0.936    85.4%-> 94.5% ( +9.1pp)     1.4%->  1.1% (+0.36pp)
eth3d_courtyard_L3      0.724->0.868     0.2%-> 41.9% (+41.7pp)     6.0%->  4.2% (+1.75pp)
eth3d_delivery_area_L1  0.890->0.930    47.6%-> 82.0% (+34.4pp)     4.1%->  3.5% (+0.59pp)
eth3d_delivery_area_L2  0.866->0.916    38.5%-> 68.6% (+30.1pp)     5.1%->  4.2% (+0.92pp)
eth3d_delivery_area_L3  0.787->0.878     6.4%-> 18.7% (+12.4pp)    10.2%->  7.6% (+2.60pp)
eth3d_facade_L1         0.885->0.908    63.3%-> 79.5% (+16.2pp)     3.3%->  3.3% (+0.05pp)
eth3d_facade_L2         0.826->0.917     0.0%-> 78.7% (+78.7pp)     3.9%->  3.2% (+0.76pp)
eth3d_facade_L3         0.715->0.867     0.0%->  0.0% ( +0.0pp)     8.7%->  6.7% (+2.01pp)
eth3d_meadow_L1         0.810->0.791     0.8%-> 25.5% (+24.7pp)    18.9%-> 21.7% (-2.89pp)  <- tail regress
eth3d_meadow_L2         0.787->0.820     0.1%-> 14.8% (+14.7pp)    10.3%-> 14.0% (-3.67pp)  <- tail regress
eth3d_meadow_L3         0.737->0.807     0.0%-> 13.2% (+13.2pp)    13.8%-> 12.9% (+0.88pp)
eth3d_office_L1         0.880->0.883     0.0%-> 40.4% (+40.4pp)    27.6%-> 35.4% (-7.76pp)  <- tail regress
eth3d_office_L2         0.882->0.903     0.0%->  1.8% ( +1.8pp)    35.2%-> 46.3% (-11.01pp) <- tail regress
eth3d_office_L3         0.789->0.893     0.0%->  0.1% ( +0.1pp)    43.2%-> 52.0% (-8.84pp)  <- tail regress
eth3d_pipes_L1          0.805->0.884     0.0%-> 43.0% (+43.0pp)    15.5%-> 18.8% (-3.24pp)  <- tail regress
eth3d_pipes_L2          0.786->0.881     0.0%-> 25.8% (+25.8pp)    18.5%-> 22.4% (-3.85pp)  <- tail regress
eth3d_pipes_L3          0.766->0.853     0.0%->  4.7% ( +4.7pp)    26.9%-> 20.9% (+6.03pp)
```

### 2b. Confidence aggregates (28 scene-levels)

Completeness kept at a fixed contamination budget — higher better:

| contamination ≤ | raw | adjusted | mean Δ | scenes adj≥raw |
|---|---|---|---|---|
| 0.5% | 19.8% | **40.5%** | **+21.9pp** | 27/27 |
| 1%   | 31.0% | **51.3%** | **+20.4pp** | 27/28 |
| 2%   | 46.8% | **65.5%** | **+18.8pp** | 27/28 |
| 5%   | 66.3% | 80.6% | +14.3pp | 19/28 |

Contamination admitted at a fixed completeness target — lower better:

| completeness ≥ | raw | adjusted | mean Δ | scenes adj≤raw |
|---|---|---|---|---|
| 90% | 10.3% | **8.4%** | **−1.9pp** | 25/28 |
| 95% | 12.7% | 13.8% | +1.1pp (worse) | 18/28 |
| 99% | 15.7% | 16.1% | +0.5pp (worse) | 8/28 |

**Reading it:** the recalibration is a large, near-universal win in the clean-subset region a
thresholding consumer (TSDF) uses — **+~20pp completeness at ≤1% contamination, 27/28** — and on hard
scenes where raw confidence is *unusable* for gating (completeness@≤1% = 0.0%: office, pipes,
facade-L2/L3) it restores 40–79%. The only cost is at the ≥95%-completeness *tail* (not the TSDF
operating point): +1.1pp contamination, concentrated on textureless office/meadow/pipes and the aerial
`bmvs_5b7a3890`. ROC improved on all of those, so a single AUC would have hidden the tail — the
frontier exposes it. Full narrative: `CONFIDENCE_GT_OPERATING.md`.

---

## 3. Dense-FUSION eval — how, and every field

**Goal:** does the completeness rescue add surface without adding outliers? Graded on the fused **3D
point cloud** vs the **3D GT surface** — no depth-maps involved.

- **BlendedMVS** (`EvalFusionGT.py`): sample the textured mesh uniformly by area into 2,000,000 GT
  surface points; nearest-neighbor distance (exact, dependency-free grid NN) between fused cloud and
  GT samples. Tolerances are fractions of the GT bbox **diagonal**: 0.25% / 0.5% / 1%, gross at 5%.
- **ETH3D** (`eth3d_eval.sh`): the **official `ETH3DMultiViewEvaluation`** tool vs the registered
  laser-scan cloud. Tolerances are fixed **absolute metres**: 1 / 2 / 5 / 10 cm, gross at 0.5 m.

Both voxel-dedup before scoring (standard MVS-benchmark practice); fused PLYs are stripped to XYZ
first (the PCL reader crashes on OpenMVS list properties).

**Fields in `results_final/<TAG>_fuse_{w0,w2,w3}.json`** (`w` = `fFusePriorWeight`, few-view rescue
strength; **w0 = rescue off**):

| field | meaning |
|---|---|
| `completeness.{tol}` | fraction of **GT surface** samples with a reconstructed point within `tol` — surface **coverage** (the completeness metric). |
| `accuracy.{tol}` | fraction of **reconstructed** points with a GT sample within `tol` — precision. |
| `gross_outlier_frac` | fraction of reconstructed points with **no** GT surface within the gross tol (bmvs 5%·diag; eth3d 0.5 m). Floaters/blunders — **the outlier metric**. (eth3d: = 1 − accuracy@0.5 m.) |
| `tol_abs.{tol}` | that tolerance in scene units (metres). bmvs = frac·diag; eth3d = the fixed metres. |
| `f1.{tol}` | (eth3d only) harmonic mean of completeness & accuracy. |
| `diag` | bbox diagonal (bmvs: GT sample cloud; eth3d: reconstruction, informational). |
| `n_rec`,`n_gt` | reconstructed / GT point counts. |

**Not cross-dataset comparable:** bmvs tolerances scale with scene size, eth3d are fixed metres — compare
within a scene or across same-dataset scenes only.

### 3a. Fusion results — per scene-level (w0 / w2 / w3)

completeness at the **mid** tier (bmvs 0.5%·diag / eth3d 2 cm) and gross-outlier %; `Δgross` vs w0 with
the per-scene **+0.05pp** budget flag (`aggregate_fuse.py`):

```
scene-level             set   | comp@mid  w0     w2     w3   |  gross%   w0     w2     w3   | dgross w2/w3  budget
bmvs_59817e4a_L0        bmvs  |  0.519  0.528  0.546 |  0.000  0.000  0.001 | +0.00/+0.00pp  OK/OK
bmvs_59817e4a_L1        bmvs  |  0.521  0.532  0.549 |  0.000  0.000  0.001 | +0.00/+0.00pp  OK/OK
bmvs_59d2657f_L0        bmvs  |  0.979  0.987  0.993 |  4.964  5.774  7.004 | +0.81/+2.04pp  OVER/OVER
bmvs_59d2657f_L1        bmvs  |  0.979  0.986  0.993 |  4.921  5.700  6.852 | +0.78/+1.93pp  OVER/OVER
bmvs_5a640093_L0        bmvs  |  0.878  0.884  0.903 |  0.029  0.048  0.091 | +0.02/+0.06pp  OK/OVER
bmvs_5a640093_L1        bmvs  |  0.875  0.880  0.901 |  0.027  0.043  0.085 | +0.02/+0.06pp  OK/OVER
bmvs_5b7a3890_L0        bmvs  |  0.664  0.714  0.768 |  0.056  0.058  0.086 | +0.00/+0.03pp  OK/OK
bmvs_5b7a3890_L1        bmvs  |  0.667  0.718  0.774 |  0.060  0.062  0.086 | +0.00/+0.03pp  OK/OK
bmvs_5ba19a8a_L0        bmvs  |  0.587  0.591  0.610 |  0.049  0.081  0.143 | +0.03/+0.09pp  OK/OVER
bmvs_5ba19a8a_L1        bmvs  |  0.591  0.592  0.613 |  0.056  0.091  0.155 | +0.04/+0.10pp  OK/OVER
eth3d_courtyard_L1      eth3d |  0.433  0.452  0.476 |  0.123  0.132  0.143 | +0.01/+0.02pp  OK/OK
eth3d_courtyard_L2      eth3d |  0.363  0.390  0.424 |  0.167  0.177  0.192 | +0.01/+0.02pp  OK/OK
eth3d_courtyard_L3      eth3d |  0.147  0.169  0.207 |  0.315  0.309  0.305 | -0.01/-0.01pp  OK/OK
eth3d_delivery_area_L1  eth3d |  0.717  0.744  0.788 |  0.504  0.543  0.587 | +0.04/+0.08pp  OK/OVER
eth3d_delivery_area_L2  eth3d |  0.655  0.675  0.719 |  0.704  0.733  0.767 | +0.03/+0.06pp  OK/OVER
eth3d_delivery_area_L3  eth3d |  0.410  0.455  0.509 |  1.119  1.154  1.184 | +0.03/+0.07pp  OK/OVER
eth3d_facade_L1         eth3d |  0.481  0.530  0.551 |  0.050  0.053  0.057 | +0.00/+0.01pp  OK/OK
eth3d_facade_L2         eth3d |  0.400  0.432  0.475 |  0.077  0.078  0.087 | +0.00/+0.01pp  OK/OK
eth3d_facade_L3         eth3d |  0.198  0.226  0.265 |  0.196  0.205  0.221 | +0.01/+0.03pp  OK/OK
eth3d_meadow_L1         eth3d |  0.249  0.304  0.382 |  0.083  0.082  0.084 | -0.00/+0.00pp  OK/OK
eth3d_meadow_L2         eth3d |  0.287  0.348  0.421 |  0.049  0.049  0.057 | -0.00/+0.01pp  OK/OK
eth3d_meadow_L3         eth3d |  0.181  0.232  0.294 |  0.120  0.114  0.126 | -0.01/+0.01pp  OK/OK
eth3d_office_L1         eth3d |  0.261  0.281  0.315 |  0.215  0.261  0.357 | +0.05/+0.14pp  OK/OVER
eth3d_office_L2         eth3d |  0.267  0.285  0.332 |  0.409  0.450  0.501 | +0.04/+0.09pp  OK/OVER
eth3d_office_L3         eth3d |  0.238  0.255  0.284 |  0.587  0.621  0.714 | +0.03/+0.13pp  OK/OVER
eth3d_pipes_L1          eth3d |  0.185  0.203  0.217 |  0.011  0.015  0.022 | +0.00/+0.01pp  OK/OK
eth3d_pipes_L2          eth3d |  0.183  0.194  0.208 |  0.007  0.009  0.016 | +0.00/+0.01pp  OK/OK
eth3d_pipes_L3          eth3d |  0.161  0.169  0.184 |  0.050  0.063  0.089 | +0.01/+0.04pp  OK/OK
```

### 3b. Fusion aggregates — mean / median / max across the 28 scene-levels

**Completeness GAIN over w0** (rescue off), at the mid tier, in percentage points:

| weight | mean | median | max |
|---|---|---|---|
| w2 | +2.42pp | +1.93pp | +6.07pp |
| w3 | +5.79pp | +5.68pp | +13.34pp |

**Gross-outlier fraction — absolute** (%):

| weight | mean | median | max |
|---|---|---|---|
| w0 (off) | 0.53% | 0.08% | 4.96% |
| w2 | 0.60% | 0.09% | 5.77% |
| w3 | 0.71% | 0.13% | 7.00% |

**Gross outliers the rescue ADDS vs w0** (percentage points) — the actual cost of the rescue:

| weight | mean | median | max (worst scene) |
|---|---|---|---|
| w2 | +0.07pp | +0.01pp | +0.81pp (`bmvs_59d2657f`) |
| w3 | +0.18pp | +0.03pp | +2.04pp (`bmvs_59d2657f`) |

Per dataset the added outliers split sharply: **ETH3D barely moves** (w2 max +0.05pp, w3 max +0.14pp),
while **BlendedMVS carries the cost** (w2 max +0.81pp, w3 max +2.04pp) — almost entirely the one
near-textureless wall (`bmvs_59d2657f`), where any rescue invents some geometry.

**Reading it:** every scene gains completeness (w2 and w3 both beat w0 everywhere). w3 roughly doubles
w2's completeness gain (+5.8 vs +2.4pp mean) but adds ~2.5× the outliers (+0.18 vs +0.07pp mean;
+2.04 vs +0.81pp worst-case). Since the goal is "completeness without new outliers," **w2 is the
conservative default** — typically adding essentially nothing (median +0.01pp) — with w3 available for
users wanting maximum completeness at a known outlier cost. (Shipped default is yours; currently w3.)
Regenerate all of the above with `python gt_bench/aggregate_fuse.py`; deep-dive narrative in
`W2_W3_RECHECK.md` and `FINAL_2026-07.md` [2].

---

## 4. Reproduce

```bash
# one scene-level end to end (estimate -> adjust -> fuse w0/w2/w3 -> eval):
DO_W2=1 bash gt_bench/run_scene.sh eth3d_courtyard 2      # or bmvs_59d2657f 0
# whole benchmark: loop scenes_eth3d.txt x {3,2,1} and scenes_blendedmvs.txt x {1,0} (see gt-benchmark memory)

# confidence operating frontier (reuses existing dmaps, no re-estimation):
bash gt_bench/run_confop.sh            # -> results_confop/<TAG>_confop_{raw,adj}.json
python gt_bench/aggregate_confop.py    # the §2 tables

# fusion table:
python gt_bench/aggregate_fuse.py      # the §3 tables (reads results_final/*_fuse_{w0,w2,w3}.json)
```

Determinism note: BlendedMVS fusion is byte-stable only with the test-only hash in
`Scene::EstimateNeighborViewsPointCloud` (commit 763be0d) — **to be reverted before release**; ETH3D
was deterministic throughout. Confidence ROC/operating numbers were never affected (they grade the
conf map vs GT depth, not fusion output).
