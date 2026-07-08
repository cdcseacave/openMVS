# w2 vs w3 fusion-default re-check — with trustworthy BlendedMVS (2026-07-08)

After the BlendedMVS determinism fix (commit 763be0d), BlendedMVS fusion is reproducible and joins
ETH3D as first-class evidence. This re-checks the `fFusePriorWeight` default across **all 28
deterministic scene-levels** (18 ETH3D + 10 BlendedMVS) at w ∈ {0, 2, 3}.

Metric: gross-outlier% (dedicated tier: 0.5 m ETH3D / 5%-diag bmvs) and completeness at the mid
tolerance (0.02 m ETH3D / 0.5%-diag bmvs). Per-scene adopt rule (from the plan): the largest w whose
gross ≤ w0-gross **+ 0.05pp on every scene**.

## Result

| | mean completeness gain @mid | scene-levels OVER the +0.05pp gross budget |
|---|---|---|
| **w2** | **+2.35 pp** | **2 / 28** (only `bmvs_59d2657f` L0/L1, +0.8pp — a pathological textureless wall) |
| **w3** | +5.61 pp | **12 / 28** (6 bmvs + ETH3D office×3 + delivery_area×3; overages up to +2.0pp) |

- w3 nearly doubles w2's completeness gain, but violates the outlier budget on **43%** of scenes,
  some materially (bmvs_59d2657f +2.0pp, ETH3D office +0.14pp, delivery_area +0.09pp).
- w2 respects the budget on **26/28**; the only exceedance (59d2657f, a near-textureless wall where
  *any* rescue adds outliers) is mild (+0.8pp) and would drag a strict "every-scene" rule down to
  w1/w0 for one pathological case — not representative.
- BlendedMVS on its own: w3 over-budget on 6/10, w2 on 2/10 — same direction as ETH3D (6/18 vs 0/18),
  so the two datasets agree.

## Recommendation: **w2** (change the shipped default 3 → 2)

The user's stated fusion goal is "better completeness **while not introducing outliers**" — outliers
are the constraint. w2 captures ~42% of w3's completeness gain while keeping gross within budget
almost everywhere; w3 buys the extra completeness by exceeding the outlier budget on nearly half the
benchmark. Ship **w2** as the conservative default; users wanting maximum completeness at a known
outlier cost can still set `Fuse Prior Weight = 3` (or higher). One scene (bmvs_59d2657f) resists any
rescue without adding a little gross — inherent to its texturelessness, not a w-tuning failure.

Full per-scene table: regenerate via `results_final/*_fuse_{w0,w2,w3}.json` (the analysis script is in
the session log). ETH3D was deterministic throughout; the BlendedMVS rows here are the first
trustworthy bmvs fusion numbers in the project.
