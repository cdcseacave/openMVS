# GT quality assessment — D2 regeneration (2026-08-09)

Full regeneration of the GT benchmark at the current branch head (develop merge `1ccc1fa` with the
**D2 quantized dmap codec**, fused GPU confidence `a7c2a04`, Mesh::Clean fix `147f450`), answering
two questions: **(1)** how good is the adjust-confidence recalibration and fusion completeness on the
GT datasets, and **(2)** did the D2 codec (conf unorm8, depth float16) or any of the branch changes
move the previously recorded numbers.

**Setup.** 28 scene-levels — ETH3D 6 scenes × L{3,2,1} (courtyard, facade, meadow, office,
delivery_area, pipes), BlendedMVS 5 scenes × L{1,0} — via `run_scene.sh` (paired eval: estimate raw →
snapshot → standalone adjust → fuse w0/w2/w3). The paired confidence eval isolates the conf channel
(same depths, only confidence differs); it uses the standalone CPU adjust — GPU-fused parity is
established separately (§ CUDA_CONFIDENCE.md: |ΔROC| = 0.000026), so these results cover both
backends. Artifacts: `virginia/gt_bench/runs/<scene>/L<r>_d2`, `results_d2/`, `AGGREGATE_D2.md`,
`AGGREGATE_D2_CONFOP.txt`, `AGGREGATE_D2_FUSE.txt`.

## 1. Adjust-confidence quality (raw → adjusted)

| pool | ROC-AUC | compl@≤0.5% contam | @≤1% | @≤2% | @≤5% | contam@≥90% compl | @≥95% |
|---|---|---|---|---|---|---|---|
| **ALL (28)** | 0.844 → **0.926** | 18.7 → **44.9**% | 31.5 → **57.9**% | 48.3 → **70.9**% | 66.3 → **86.3**% | 10.4 → **7.1**% | 12.8 → 12.2% |
| ETH3D (18) | 0.816 → 0.910 | 5.5 → 32.9% | 17.9 → 49.0% | 33.1 → 66.0% | 59.0 → 84.7% | 11.3 → 8.7% | 14.1 → 13.1% |
| BlendedMVS (10) | 0.895 → 0.956 | 45.3 → 66.4% | 58.6 → 73.9% | 78.6 → 79.7% | 79.5 → 89.2% | 8.7 → 4.1% | 10.5 → 10.5% |

- ROC-AUC improves on **28/28** scene-levels (mean +0.082; largest +0.20 facade L3, +0.18 courtyard L3).
- Completeness kept at a ≤1% contamination budget improves on **27/27** comparable scene-levels,
  mean **+28.5pp** — the headline user-facing effect: at a fixed quality budget the adjusted
  confidence keeps roughly twice the surface.
- At raw confidence, 11 of 28 scene-levels have **no usable ≤1% operating point at all** (0.0–0.2%
  completeness); adjusted gives 8 of those a real one (e.g. facade L2 0→87.5%, courtyard L3 0.1→63.2%,
  pipes L1 0→53.3%).
- Limits: at extreme completeness targets (≥99%) the frontier is unchanged (adj ≤ raw on only 11/28)
  — recalibration reorders the mid-ranking, not the deepest tail; and a few hard indoor/low-texture
  levels regress on contam@≥95% (office L2 −11pp, office L1 −7.6pp, meadow L2 −3.7pp, pipes L1 −3.5pp)
  while still improving ROC and the ≤1–2% budget points.

## 2. Regression vs the recorded baseline (D2 codec effect)

Same 28 scene-levels vs `results_final` (pre-merge codec, full-float conf/depth):

| metric | baseline | D2 regeneration | Δ |
|---|---|---|---|
| mean ROC raw | 0.8444 | 0.8444 | ±0.0000 |
| mean ROC adjusted | 0.9276 | 0.9261 | −0.0015 |
| fusion w2 compl. gain vs w0 (mean) | +2.42pp | +2.55pp | +0.13pp |
| fusion w3 compl. gain vs w0 (mean) | +5.79pp | +6.47pp | +0.68pp |
| gross added vs w0, w2 / w3 (mean) | +0.07 / +0.18pp | +0.07 / +0.17pp | ≈0 |
| scenes over +0.05pp gross budget, w2 / w3 | 2/28, 12/28 | 2/28, 11/28 | ≈0 |

**Verdict: no measurable regression.** The unorm8 confidence + float16 depth quantization and the
branch changes reproduce the recorded quality within estimation noise (CUDA PatchMatch run-to-run
spread is larger than every Δ above).

## 3. Fusion completeness (w0 = rescue off, w2/w3 = `fFusePriorWeight` 2/3)

| pool | w2 compl. gain (mean/med/max) | w3 compl. gain | w2 gross added (mean) | w3 gross added | over-budget w2 | w3 |
|---|---|---|---|---|---|---|
| ALL (28) | +2.6 / +2.4 / +6.4pp | +6.5 / +6.9 / +13.8pp | +0.07pp | +0.17pp | 2/28 | 11/28 |
| ETH3D (18) | +3.3 / +3.0 / +6.4pp | +8.0 / +7.4 / +13.8pp | +0.01pp | +0.04pp | 0/18 | 6/18 |
| BlendedMVS (10) | +1.3 / +0.8 / +4.0pp | +3.8 / +2.6 / +10.0pp | +0.16pp | +0.40pp | 2/10 | 5/10 |

The confidence-guided rescue buys real completeness on every pool; w3 buys ~2.5× more than w2 but
pushes 11/28 scene-levels past the +0.05pp gross-outlier budget (worst +1.8pp, BlendedMVS 5ba19a8a).
w2 stays within budget on 26/28 with the two exceptions marginal.

**Decision (2026-08-10): the default stays w3** — the pipeline is normally followed by mesh
reconstruction, which cleans the few extra gross outliers and benefits from the extra true points.
**Use w2 (`--fusion-prior-weight 2`) when the dense point cloud itself is the final output** (fewer
outliers at slightly lower completeness); documented in the `DensifyPointCloud` CLI help and the
`fFusePriorWeight` option description.

## 4. Operational notes

- `ETH3DMultiViewEvaluation` links PCL 1.14 dynamically; a system update removed it mid-benchmark
  (every ETH3D fusion eval failed with exit 127). Restore with
  `apt install libpcl-{common,io,search,kdtree,octree}1.14`. The sweep script is marker-idempotent,
  so re-running after the fix only redid the failed eval stage.
- Reproduce: `virginia/gt_bench/run_sweep_d2.sh` (sequential, ~7h on one A100) then
  `virginia/gt_bench/aggregate_d2.sh`.
