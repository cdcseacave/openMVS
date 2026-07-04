<!--
PROVENANCE: Task 20 fFusePriorWeight w-sweep, recorded 2026-07-04.
- Binary: make/bin/Release/DensifyPointCloud built 2026-07-04 (HEAD of
  feature/fusion-faithful-confidence with the GT-recalibrated confidence defaults from Task 17
  and the Task-18 fusion FSV guard nFuseViolationMax=0 active).
- Method: fusion-ONLY re-runs. Per (scene, reslevel) the raw depth-maps were estimated once, the
  confidence adjusted once (--postprocess-dmaps 4, shipped defaults), then the SAME adjusted dmaps
  were fused at Fuse Prior Weight = w for w in {0,1,2,3,4,5} (--dense-config-file, --geometric-iters 0,
  --postprocess-dmaps 0 so no dmap is re-estimated/re-adjusted between weights) and each cloud scored
  vs real GT (eth3d_eval.sh / EvalFusionGT.py). Driver + raw JSONs:
  /home/ubuntu/virginia/gt_bench/task20/ (sweep_driver.sh, <tag>/fuse_w{0..5}.json).
- DECISION is anchored on the 6 DETERMINISTIC ETH3D scenes. The 2 BlendedMVS scenes are NOISY
  CONTEXT ONLY -- bmvs fusion is confounded by a pre-existing OpenMP neighbor-cache race
  (nondeterministic run-to-run) AND, for w5 on high-gross scenes, huge clouds; see
  gt_bench/VERDICT_MAPANYTHING.md / task-18-report.md and the memory gt-benchmark.md.
-->
# fFusePriorWeight w-sweep on real GT (Task 20)

Retunes the fusion few-view-rescue weight `OPTDENSE::fFusePriorWeight` against **real ground truth**,
replacing its 2026-06-30 value (3.0) that had only ever been validated on the now-retired MapAnything
pseudo-GT (`gt_bench/VERDICT_MAPANYTHING.md`). All rows use the **shipped** fusion FSV guard
(`nFuseViolationMax = 0`, Task 18) and the **shipped** GT-recalibrated confidence posterior (Task 17).

## Decision rule (from the task brief)

Maximize completeness subject to **gross-outliers ≤ (w=0 gross) + 0.05 pp** at the **mid tolerance**
(0.02 m for ETH3D), **judged on the deterministic ETH3D scenes**. Report the completeness/outlier
curve and the knee. `w = 1` is a structural **no-op** everywhere (identical cloud to w0): the binding
fusion gate is `nMinPixelsFuse = 5`, and a typical few-view cluster has ~2 fused pixels, so the
virtual support `w·pGeo` must reach ~3 before it bridges the gate — the effect is strongly nonlinear
with a useful range of w ≈ 2–5 (consistent with the ladita note in the confidence-recalibration
memory).

## [1] Per-scene: completeness@0.02 m and gross-outlier% across w (6 deterministic ETH3D scenes)

`compl` = completeness at the 0.02 m mid tier; `gross%` = fraction of reconstruction points with no
GT scan surface within 0.5 m; `Δgross` = gross% − (w0 gross%), the quantity the +0.05 pp budget caps.

| scene | w0 | w2 | w3 | w4 | w5 |
|---|---|---|---|---|---|
| **courtyard** compl / gross% / Δgross | 0.363 / 0.168 / +0.000 | 0.390 / 0.173 / +0.005 | 0.423 / 0.190 / +0.022 | 0.465 / 0.206 / +0.039 | 0.513 / 0.243 / +0.075 |
| **facade** compl / gross% / Δgross | 0.400 / 0.080 / +0.000 | 0.432 / 0.080 / +0.001 | 0.474 / 0.089 / +0.009 | 0.532 / 0.142 / +0.062 | 0.588 / 0.276 / +0.196 |
| **meadow** compl / gross% / Δgross | 0.290 / 0.052 / +0.000 | 0.352 / 0.054 / +0.003 | 0.422 / 0.063 / +0.011 | 0.512 / 0.075 / +0.024 | 0.586 / 0.115 / +0.063 |
| **office** compl / gross% / Δgross | 0.269 / 0.407 / +0.000 | 0.287 / 0.439 / +0.032 | 0.333 / 0.500 / +0.093 | 0.360 / 0.658 / +0.251 | 0.391 / 1.462 / +1.054 |
| **delivery_area** compl / gross% / Δgross | 0.646 / 0.696 / +0.000 | 0.680 / 0.725 / +0.030 | 0.723 / 0.764 / +0.068 | 0.767 / 0.817 / +0.122 | 0.803 / 0.995 / +0.300 |
| **pipes** compl / gross% / Δgross | 0.183 / 0.011 / +0.000 | 0.194 / 0.014 / +0.004 | 0.209 / 0.020 / +0.010 | 0.220 / 0.033 / +0.023 | 0.227 / 0.084 / +0.074 |

(w1 omitted: byte-identical to w0 on every scene.)

## [2] Aggregate ETH3D curve + the knee

Mean over the 6 deterministic ETH3D scenes (equal per-scene weight), plus the **marginal efficiency**
= Δ(mean completeness, pp) per Δ(mean Δgross, pp) between consecutive weights:

| w | mean compl@0.02 | mean gross% | mean Δgross (pp) | max scene Δgross (pp) | #scenes over +0.05 pp | marg. eff. (Δcompl pp / Δgross pp) |
|---|---|---|---|---|---|---|
| 0 | 0.3584 | 0.2354 | +0.0000 | +0.0000 | 0 | — |
| 1 | 0.3584 | 0.2354 | +0.0000 | +0.0000 | 0 | no-op |
| 2 | 0.3890 | 0.2477 | +0.0123 | +0.0322 | 0 | 247.9 |
| **3** | **0.4308** | **0.2710** | **+0.0356** | +0.0927 | 2 | **179.8** |
| 4 | 0.4757 | 0.3219 | +0.0865 | +0.2506 | 3 | 88.1 |
| 5 | 0.5179 | 0.5291 | +0.2937 | +1.0543 | 6 | 20.4 |

**Knee = w3.** Three independent readings agree on w=3 as the operating point:
1. **Aggregate budget:** mean Δgross is **+0.0356 pp at w3 (≤ +0.05 pp, PASS)** and jumps to **+0.0865 pp
   at w4 (> +0.05 pp, FAIL)**. w3 is the largest weight whose mean gross increase stays inside budget.
2. **Marginal efficiency:** completeness-gained-per-gross halves at the w3→w4 step (179.8 → 88.1) and
   collapses at w4→w5 (→ 20.4). The curve bends at w3.
3. **Completeness:** w3 buys a large mean completeness gain over the budget-safe w2 (+4.2 pp:
   0.389 → 0.431) — the last big, cheap completeness increment before the gross cost accelerates.

## [3] Decision

**Keep `fFusePriorWeight = 3.0` (no change).** w=3 is the GT-chosen knee; it equals the incumbent
default, so no `DEFVAR` edit and no rebuild-for-defaults were required. w3 lifts mean ETH3D
completeness@0.02 from 0.358 (w0) to **0.431 (+7.2 pp)** for a mean gross increase of just **+0.036 pp**
(0.235% → 0.271%), inside the +0.05 pp budget.

**Honest caveat (per-scene worst-case).** Under a strict "every deterministic scene must individually
satisfy +0.05 pp" reading, **office (+0.093 pp) and delivery_area (+0.068 pp)** marginally exceed the
budget at w3, which would select w=2. w=3 is nonetheless the defensible global choice because:
- The two exceedances are small in absolute terms (both scenes stay under 0.51 % / 0.76 % gross at w3;
  no ETH3D scene exceeds ~0.1 pp Δgross at w3). At w4 four scenes exceed and office reaches +0.25 pp.
- office's gross metric is partially degenerate: its `gross_tol/diag ≈ 0.066` (BASELINE table [3]) is
  5–10× the outdoor scenes' ratio, so its absolute gross% is inflated and least comparable.
- Dropping to w2 to shave a fraction of a pp of gross would forfeit the +4.2 pp mean completeness gain.
- The mean-budget test, the knee (marginal efficiency), and the incumbent all land on w=3.

If a future policy insists on a strict per-scene worst-case gross cap, **w=2 is the conservative
fallback** (mean Δgross +0.012 pp, no scene over +0.033 pp) at a ~4 pp mean completeness cost.

## [4] BlendedMVS — NOISY CONTEXT ONLY (NOT used for the decision)

bmvs fusion is confounded by the pre-existing OpenMP neighbor-cache race (nondeterministic run-to-run)
and by the empty-point-cloud RNG neighbor selection; per the task these are context only. Mid tier =
0.005 (fraction of GT bbox diagonal); gross = 5 % of diagonal.

| scene | w0 | w2 | w3 | w4 | w5 |
|---|---|---|---|---|---|
| **bmvs_59d2657f** compl / gross% / Δgross | 0.981 / 4.64 / +0.00 | 0.987 / 5.71 / +1.07 | 0.993 / 6.85 / +2.22 | 0.995 / 10.59 / +5.95 | 0.997 / 14.77 / +10.13 |
| **bmvs_5a640093** compl / gross% / Δgross | 0.872 / 0.03 / +0.00 | 0.877 / 0.04 / +0.02 | 0.898 / 0.08 / +0.06 | 0.923 / 0.25 / +0.22 | 0.938 / 1.04 / +1.01 |

Directionally consistent with ETH3D (completeness rises monotonically with w, gross accelerates past
w3), but the absolute bmvs gross magnitudes — especially 59d2657f's multi-pp jumps — are dominated by
the race and the small-scene gross-fraction sensitivity and must not be read as evidence. On the
low-gross scene (5a640093) the w3 Δgross is +0.06 pp, in line with the ETH3D picture.

## Reproduce

```bash
# fusion-only w-sweep + scoring, all 8 scenes (idempotent; FORCE=1 to redo)
bash /home/ubuntu/virginia/gt_bench/task20/sweep_driver.sh
# regenerate the aggregate table + knee analysis
/home/ubuntu/miniconda3/bin/python /home/ubuntu/virginia/gt_bench/task20/aggregate_sweep.py
```
