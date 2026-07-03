# Design: GT benchmark + faster per-view confidence + safer fusion completeness

Branch: `feature/fusion-faithful-confidence` (WS3 learned-calibrator variant will live on a separate branch, out of scope here).
Date: 2026-07-03. Status: approved by user (WS1/WS3 as proposed; WS2 amended: no intra-view threading, add integrated geometric-stage mode).

## Problem

1. Per-view confidence recalibration (`DepthMapsData::AdjustConfidence`, `--postprocess-dmaps 4`) works
   (ROC-AUC 0.72–0.90 → 0.87–0.96 on the 5-scene pseudo-GT bench) but costs ~0.2–2.0 s/view single-core
   (~0.4–0.56 ms/Mpx·view at up to 8 neighbors); ~2 s/view at high resolution is too slow for what it computes.
   The confidence is consumed by stages that do not build a dense cloud (e.g. TSDF integration), so it must be
   decisively cheaper than dense fusion (≥2×, ideally ≥5× on wall-clock, and cheap per view).
2. Fusion few-view rescue (`fFusePriorWeight=3`) visibly extends completeness but not fully, and introduces
   some outliers. Both effects are currently judged only against MapAnything pseudo-GT, which is itself noisy.
3. No trustworthy ground truth: all tuning so far used fusion-consensus labels and mono-depth pseudo-GT.
   We need datasets with complete GT geometry + GT poses to (a) measure truth-faithful confidence,
   (b) measure fusion completeness vs outliers honestly, (c) calibrate how much the MapAnything witness can be trusted.

## WS1 — GT evaluation benchmark (foundation; do first)

Datasets (downloads under `/home/ubuntu/virginia/gt_bench/`):
- **BlendedMVS** (low-res set ~27.5 GB + textured meshes ~9.4 GB; CC-BY, direct download):
  113 scenes (indoor/outdoor/aerial/objects), GT depth map (PFM) for every image, poses exactly consistent
  with the meshes. Import per scene with the existing `InterfaceMVSNet` app → `scene.mvs`.
- **ETH3D high-res multi-view training** (~8.5 GB: undistorted images + laser GT + occlusion masks):
  13 real DSLR scenes, GT poses, per-image laser-rendered GT depth. Import with `InterfaceCOLMAP`.
  Build the official `ETH3D/multi-view-evaluation` C++ tool (accuracy/completeness/F1 at tolerances,
  correctly handling laser-shadow/unobserved space).

Recurring benchmark (~11 scenes, 3 resolutions `--resolution-level 2/1/0`):
- ETH3D: courtyard, facade, meadow (outdoor/vegetation), office, delivery_area, pipes (indoor/textureless/reflective).
- BlendedMVS: 5 diverse scenes chosen after download (sculpture, building, aerial, indoor, small object),
  preferring the community 7-scene validation split.

Evaluation layers:
1. **Confidence vs GT depth** — extend `scripts/python/EvalConfidence.py` with a GT mode (`--gt-depth-dir`):
   per-pixel label inlier ⇔ `|d − d_GT| ≤ max(absTol, relTol·d_GT)` (defaults relTol=1 %, absTol per dataset:
   ETH3D 2 cm, BlendedMVS scene-scale-relative); pixels without GT ignored. Same metric report as today
   (ROC/PR-AUC, P@0.1/R@0.1, Spearman, Brier/ECE). The existing fusion-faithful label harness stays as a
   secondary diagnostic (different question: "would fusion keep it").
2. **Fusion vs GT geometry** — ETH3D: official tool (F1/accuracy/completeness at 1/2/5/10 cm).
   BlendedMVS: Open3D — sample GT mesh to a dense cloud; completeness = fraction of GT samples with a fused
   point within tol; accuracy = fraction of fused points within tol of mesh; gross-outlier % at 10×tol.
   Report per scene/resolution for w0 vs w3 (and later variants).
3. **Timing table** — per-phase wall + per-map compute for estimation, adjust, fusion at all 3 resolutions
   (extends `mvs_bench/TIMING.md` format; adjust must be ≥2× (target ≥5×) faster than fusion wall on every row).
4. **MapAnything calibration study** — run the existing pseudo-GT pipeline (`MapAnyInferMV.py` + gated
   `MapAnyVoxelFuse.py`) on 2–3 benchmark scenes and score it against real GT (depth-error distribution,
   pseudo-GT cloud precision/completeness). Output: a short verdict on whether/how to keep using it for
   GT-less scenes.

Deliverables: download/convert scripts, `EvalConfidence.py` GT mode, `gt_bench` runner + aggregator producing
`AGGREGATE_GT.md`, MapAnything verdict note. Baseline numbers for current master vs this branch (w0/w3, raw/adjusted conf).

## WS2 — Confidence speed

Target: ≥10× per-map compute on the standalone path (2 s → ≤0.2 s at 2560×1918, single core per view);
adjust-phase wall ≥5× faster than fusion on the full timing table. **No multi-threading inside per-view code**
(views are already processed in parallel across the thread pool); no change to across-map scheduling.

Phase A — standalone CPU rewrite (same semantics, `--postprocess-dmaps 4`):
1. Precompute per (reference, neighbor) pair single-precision fused transforms:
   `A = K_n·R_n·R_rᵀ·K_r⁻¹`, `b = K_n·R_n·(C_r − C_n)` → projection is `x_n = A·(u·d, v·d, d) + b`
   (one 3×3 float mad per pixel per neighbor, replacing three double-precision camera-object calls);
   analogous inverse pair for the forward-backward gate; constant `R_n·R_rᵀ` for the normal gate.
2. Single-precision throughout the pixel loop; gate short-circuit order depth → FB → normal → conf;
   rotate the reference normal only when the normal gate is reached.
3. Kill redundant I/O (batching across views): use `DMapCache` (as fusion does) so each neighbor dmap is
   read from disk once per phase instead of ~9×; keep the adjusted confidence in memory instead of the
   `adjusted.cmap` write→reload→delete round-trip.
4. Compute `ComputeIntraMapPrior` once per dmap and reuse it in the fusion rescue (cache in `DepthData`
   or a sidecar) instead of recomputing per phase. Remove the ineffective nested-OpenMP pragma inside it.
Acceptance: GT ROC-AUC within ±0.002 of the current implementation on the benchmark (float vs double may
cause tiny diffs; byte-identity not required), per-map compute speedup measured on all timing-table rows.

Phase B — integrated geometric-stage confidence (user idea; after Phase A):
- New optional mode: during the **last** geometric-consistency iteration of `EstimateDepthMaps`
  (`OPTDENSE::nEstimationGeometricIters`, SceneDensify.cpp:2570–2596) each view's worker already holds all
  neighbor depth-maps (CPU) or has them resident on GPU (`pmCUDAPool`/`pmMetalPool`, SceneDensify.cpp:2563–2567).
  Run the confirmation sweep + posterior as an epilogue of that view's estimation, writing the adjusted
  confidence directly — zero extra I/O; on the CUDA path a trivial gather kernel over already-uploaded maps.
- Semantics difference to measure: neighbors are the gc-input maps (one iteration stale, pre-final-filter).
  A/B on the GT benchmark vs Phase-A standalone; if ROC-AUC drop < 0.01, document integrated mode as the
  recommended default for pipelines that want confidence; standalone stays for reuse-existing-dmaps workflows.
- Knob: new `OPTDENSE` flag (e.g. `Estimate Confidence = 1`) independent of `--postprocess-dmaps`.
- Bonus experiment: expose the estimator's own per-pixel geometric-consistency score (DepthMap.cpp:602)
  as an additional confidence feature (free evidence, evaluated on GT before adoption).

Phase C — standalone CUDA kernel: deferred; only if Phase B doesn't cover GPU users' needs.

## WS3 — Accuracy (hand-crafted, gated on the WS1 benchmark)

Confidence:
1. **Free-space-violation (FSV) negative evidence**: in the neighbor sweep, classify non-confirming
   neighbors: violation if the neighbor sees well *behind* the projected point (`d_n > d_proj·(1+ε)` —
   its ray passes through our surface), neutral if occluded (`d_n < d_proj·(1−ε)`). Violations enter the
   posterior as failures: `conf = (s·pGeo + Pconf)/(s + Pconf + λ·V) · gate · photoFactor`, λ tuned on GT.
   Nearly free — the depth comparison already exists.
2. **Soft gates + bilinear sampling**: bilinear neighbor-depth lookup (replacing `ROUND2INT` nearest) and
   continuous agreement weights (Gaussian in depth and reprojection residual, cosine in normal angle)
   accumulated as soft K/Pconf. Behind an A/B knob; adopted only if GT ROC improves.
3. **GT-driven recalibration**: sweep τ, floor, photo-factor w0, prior strength s, λ, gate σ with
   `SweepConfParams.py` pointed at GT labels (add V / soft-K to `--export-conf-features`);
   per-resolution defaults if the sweep justifies them. Formula stays analytic.

Fusion completeness vs outliers:
4. **FSV guard on rescued points**: a point kept only thanks to virtual support (`w·pGeo`) must additionally
   have zero (or ≤ vMax) free-space violations among observing views — targets exactly the introduced-outlier
   population while keeping the completeness gain.
5. **Second-chance pass** for the *discarded-recoverable* bucket: after the main fusion loop, revisit
   discarded seed pixels with high prior AND FSV=0 under a relaxed pixel-count minimum; measured on GT
   (completeness↑, gross-outliers not ↑) before enabling by default.
6. **Retune `fFusePriorWeight` on real GT** (w=3 was chosen against the noisy pseudo-GT; re-derive the knee).

Gate for every WS3 item: GT completeness strictly improves or cost strictly drops, with gross-outlier %
not increasing beyond +0.05 pp and confidence ROC-AUC not regressing on any benchmark scene.

## Order & risks

Order: WS1 → WS2-A → WS2-B → WS3 (items 1–3 conf, then 4–6 fusion). Each step lands with benchmark numbers.

Risks / mitigations:
- BlendedMVS meshes are MVS-reconstruction-grade (small holes): use the rendered GT depth's validity as an
  ignore-mask; completeness measured against mesh samples only where GT exists.
- ETH3D laser shadows: handled by the official evaluation tool (unobserved-space aware).
- Disk/GPU contention on the shared machine: all data under `/home/ubuntu/virginia`; GPU only needed for
  the MapAnything calibration study and depth-map (re-)estimation.
- Integrated-mode staleness (WS2-B) may cost accuracy: measured explicitly; standalone path remains.
