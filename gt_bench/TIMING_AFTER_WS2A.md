# Task 11 (WS2A) — intra-map prior reuse + nested-OpenMP flag: full timing re-run + gate verdicts

**Binary:** `make/bin/Release/DensifyPointCloud` rebuilt at HEAD of this branch
(Task 9 fused-SP AdjustConfidence + Task 10 phase-lifetime DMapCache + **Task 11** shared/cached
`ComputeIntraMapPrior` via `GetIntraMapPrior(bParallel)`).
**Protocol:** CPU-only, dmaps REUSED (no estimation). Each scene/level was seeded from its pristine
`raw_dmaps/` snapshot into a fresh sibling workdir `runs/<scene>/L<r>_ws2a` (so the committed
baseline `runs/<scene>/L<r>` and `results/` are untouched), then stage 2 (adjust, `--postprocess-dmaps 4`)
+ stage 3a/3b (fuse w0/w3) + conf_adj eval were run with `--geometric-iters 0`. New result JSONs live
in `/home/ubuntu/virginia/gt_bench/results_ws2a/`; companion aggregate `AGGREGATE_WS2A.md`.
The **"adjust ms/map OLD"** column is the committed pre-Task-9 `BASELINE_2026-07.md` number, so the
**"compute speedup"** column is the CUMULATIVE Task 9+10+11 effect vs that baseline — NOT
Task-11-attributable (Task 11's own A/B delta is ~nil; see report §ablation).

## Equivalence (pure caching/threading-flag change — values must not move)

- **eth3d_courtyard L2**: OLD (pre-Task-11) vs NEW binary, both run `--postprocess-dmaps 4` on
  byte-identical raw dmaps → all **38/38 `.dmap` MD5-identical**. Re-checked on an independent
  seed copy (REP) and on **L1** (38/38) — all identical. Byte-equivalence PASS.
- **conf_adj ROC** vs the committed baseline, all 28 rows: **max|dROC| = 0.00021** (gate c, below).

## Old → New timing table + per-row gate verdicts

Gate legend — **A**: per-map adjust compute ≥10× faster than baseline (judged on largest-resolution
rows only: bmvs L0, eth3d L1; "-" = not a largest-res row). **B**: adjust wall ≥5× below fusion wall
(fusMin = min(fuse w0, fuse w3); PASS needs fusMin/adjWall ≥ 5). **C**: conf_adj ROC within ±0.002 of
baseline.

| Scene | L | n | adjust ms/map OLD | adjust ms/map NEW | compute speedup | adjust wall NEW (s) | fuse w0 NEW (s) | fuse w3 NEW (s) | fusMin/adjWall | conf_adj ROC OLD | conf_adj ROC NEW | dROC | A | B | C |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bmvs_59817e4a | 0 | 91 | 226.5 | 118.6 | 1.91x | 13.1 | 27.1 | 25.6 | 1.96x | 0.9285 | 0.9287 | +0.0001 | FAIL | FAIL | PASS |
| bmvs_59817e4a | 1 | 91 | 213.8 | 121.8 | 1.75x | 11.5 | 26.4 | 25.1 | 2.18x | 0.9294 | 0.9293 | -0.0002 | - | FAIL | PASS |
| bmvs_59d2657f | 0 | 77 | 287.8 | 116.7 | 2.47x | 11.9 | 21.0 | 19.1 | 1.60x | 0.9783 | 0.9783 | +0.0000 | FAIL | FAIL | PASS |
| bmvs_59d2657f | 1 | 77 | 220.4 | 118.1 | 1.87x | 13.2 | 21.0 | 19.2 | 1.45x | 0.9766 | 0.9768 | +0.0001 | - | FAIL | PASS |
| bmvs_5a640093 | 0 | 110 | 215.8 | 119.9 | 1.80x | 16.9 | 33.2 | 31.8 | 1.88x | 0.9539 | 0.9537 | -0.0002 | FAIL | FAIL | PASS |
| bmvs_5a640093 | 1 | 110 | 220.6 | 121.4 | 1.82x | 15.8 | 33.8 | 30.6 | 1.94x | 0.9539 | 0.9541 | +0.0002 | - | FAIL | PASS |
| bmvs_5b7a3890 | 0 | 375 | 130.4 | 98.7 | 1.32x | 62.9 | 71.4 | 56.5 | 0.90x | 0.8991 | 0.8991 | +0.0000 | FAIL | FAIL | PASS |
| bmvs_5b7a3890 | 1 | 375 | 147.7 | 98.0 | 1.51x | 58.9 | 73.2 | 56.8 | 0.96x | 0.8999 | 0.9000 | +0.0001 | - | FAIL | PASS |
| bmvs_5ba19a8a | 0 | 100 | 201.2 | 114.2 | 1.76x | 14.2 | 30.5 | 29.1 | 2.05x | 0.9544 | 0.9543 | -0.0001 | FAIL | FAIL | PASS |
| bmvs_5ba19a8a | 1 | 100 | 203.6 | 115.7 | 1.76x | 13.3 | 30.5 | 28.6 | 2.16x | 0.9544 | 0.9544 | +0.0000 | - | FAIL | PASS |
| eth3d_courtyard | 1 | 38 | 2364.8 | 1447.4 | 1.63x | 35.4 | 106.5 | 99.7 | 2.82x | 0.9333 | 0.9333 | -0.0000 | FAIL | FAIL | PASS |
| eth3d_courtyard | 2 | 38 | 638.5 | 364.0 | 1.75x | 13.0 | 29.8 | 28.2 | 2.17x | 0.9361 | 0.9361 | +0.0000 | - | FAIL | PASS |
| eth3d_courtyard | 3 | 38 | 193.7 | 108.4 | 1.79x | 7.2 | 9.4 | 9.0 | 1.25x | 0.8676 | 0.8676 | -0.0000 | - | FAIL | PASS |
| eth3d_delivery_area | 1 | 44 | 2441.1 | 1461.7 | 1.67x | 36.9 | 127.5 | 122.4 | 3.32x | 0.9302 | 0.9302 | -0.0000 | FAIL | FAIL | PASS |
| eth3d_delivery_area | 2 | 44 | 623.4 | 370.7 | 1.68x | 12.8 | 36.5 | 34.7 | 2.72x | 0.9163 | 0.9163 | -0.0000 | - | FAIL | PASS |
| eth3d_delivery_area | 3 | 44 | 176.1 | 100.7 | 1.75x | 7.4 | 11.2 | 10.8 | 1.46x | 0.8775 | 0.8775 | -0.0000 | - | FAIL | PASS |
| eth3d_facade | 1 | 76 | 2526.6 | 1511.8 | 1.67x | 70.0 | 246.1 | 225.1 | 3.22x | 0.9080 | 0.9080 | +0.0000 | FAIL | FAIL | PASS |
| eth3d_facade | 2 | 76 | 656.9 | 389.5 | 1.69x | 21.8 | 68.4 | 63.0 | 2.89x | 0.9169 | 0.9169 | -0.0000 | - | FAIL | PASS |
| eth3d_facade | 3 | 76 | 175.2 | 104.0 | 1.69x | 14.2 | 21.5 | 19.2 | 1.35x | 0.8668 | 0.8668 | +0.0000 | - | FAIL | PASS |
| eth3d_meadow | 1 | 15 | 1764.5 | 1164.1 | 1.52x | 14.2 | 19.2 | 15.2 | 1.07x | 0.7911 | 0.7911 | -0.0000 | FAIL | FAIL | PASS |
| eth3d_meadow | 2 | 15 | 447.0 | 312.0 | 1.43x | 5.3 | 6.7 | 6.1 | 1.15x | 0.8196 | 0.8196 | -0.0000 | - | FAIL | PASS |
| eth3d_meadow | 3 | 15 | 170.7 | 89.1 | 1.92x | 3.6 | 3.1 | 3.1 | 0.86x | 0.8069 | 0.8069 | +0.0000 | - | FAIL | PASS |
| eth3d_office | 1 | 26 | 1345.5 | 902.2 | 1.49x | 19.5 | 32.1 | 28.6 | 1.47x | 0.8828 | 0.8828 | +0.0000 | FAIL | FAIL | PASS |
| eth3d_office | 2 | 26 | 457.4 | 282.3 | 1.62x | 8.5 | 12.4 | 10.9 | 1.28x | 0.9027 | 0.9027 | -0.0000 | - | FAIL | PASS |
| eth3d_office | 3 | 26 | 172.3 | 75.3 | 2.29x | 4.6 | 4.6 | 4.2 | 0.93x | 0.8925 | 0.8925 | +0.0000 | - | FAIL | PASS |
| eth3d_pipes | 1 | 14 | 1704.6 | 1207.1 | 1.41x | 14.3 | 25.5 | 22.4 | 1.57x | 0.8843 | 0.8843 | +0.0000 | FAIL | FAIL | PASS |
| eth3d_pipes | 2 | 14 | 504.3 | 342.0 | 1.47x | 5.7 | 8.6 | 7.7 | 1.35x | 0.8811 | 0.8811 | +0.0000 | - | FAIL | PASS |
| eth3d_pipes | 3 | 14 | 203.2 | 101.5 | 2.00x | 3.1 | 3.1 | 3.0 | 1.00x | 0.8525 | 0.8525 | -0.0000 | - | FAIL | PASS |

## Gate summary (HARD gates — reported honestly)

- **(a) ≥10× per-map adjust compute @ largest-res rows: 0/11 PASS.** Range 1.32×–2.47× (best
  bmvs_59d2657f L0, worst bmvs_5b7a3890 L0). This is the *cumulative* Task 9+10+11 speedup over the
  pre-Task-9 baseline; even so it is far under 10×. **Structural, not a run artifact:** Task 9's own
  ablation already proved the pooled per-map speedup caps at ~2.6× (≈4.5× with an infinitely fast
  sweep) because the confirmation sweep (~75-80%) and fixed map-alloc overhead dominate. Task 11 only
  touches the prior (~20-25% share) and only its threading flag — it cannot move this gate.
- **(b) adjust wall ≥5× below fusion wall, EVERY row: 0/28 PASS.** fusMin/adjWall ranges 0.86×–3.32×.
  The **baseline itself never satisfied this** (Task-7 report: adjust/fusion wall ratio min 0.32,
  median 0.65, max 1.29 → fusion/adjust ≤ ~3.1×). It is an aspirational end-state for the whole
  optimization arc; Task 11 changes neither fusion cost nor (materially) adjust wall, so it does not
  and cannot move this gate. Worst rows are the smallest scenes (meadow/pipes/office L3, and the
  375-view bmvs_5b7a3890 where adjust is per-map CPU work over up to 8 neighbors and rivals fusion).
- **(c) conf_adj ROC within ±0.002, EVERY row: 28/28 PASS.** max|dROC| = 0.00021. This is the gate
  Task 11 actually governs (correctness of the pure caching/threading refactor) and it passes cleanly,
  corroborated by the byte-identical courtyard L2/L1 dmaps.

## Verdict

**DONE_WITH_CONCERNS.** Gate (c) — the equivalence/correctness gate this task owns — passes on all 28
rows (byte-identical + ROC within 0.0002). Gates (a) and (b) fail on all rows, but both are
structurally out of Task 11's reach (proven by Task 9's ablation for (a); true of the baseline itself
for (b)); the controller decides whether to accept them as scoped-out. Task 11's measurable win on the
*staged* benchmark is ~nil (adjust wall flat; cross-phase prior reuse only manifests in a single-process
estimate→adjust→fuse run, not in separate-process stages); its value is the correctness-preserving
single shared prior code path + the explicit no-per-view-threading flag (adjust=false / fusion=true).
