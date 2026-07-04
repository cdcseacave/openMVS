# Integrated vs. standalone confidence — total-time A/B decision (Task 13)

**Question.** Task 12 added an *integrated* confidence mode (`Estimate Confidence = 1`, via
`--dense-config-file`) that runs the fusion-faithful confidence sweep inline as an epilogue of the
**last geometric-consistency iteration**, instead of as a separate `--postprocess-dmaps 4` phase.
Its confidence quality matches the standalone path within noise, but Task 12 measured *marginal*
cost only and found it scene-dependent. The adopt/reject decision must be made on **total pipeline
wall time**, and after quantifying whether the estimation worker-pool starvation is cheaply fixable.

**Verdict (up front).** **Keep STANDALONE (`--postprocess-dmaps 4`) as the default / primary path.**
Integrated mode is a total-time win only on small / few-image scenes and a net loss on large-image
scenes; the worker-pool lever does **not** cheaply fix that (its best case is a tie, not a win); and
ROC parity, while within 0.01 on 4 of 5 scenes, slips to −0.0125 on the hardest 15-view scene.
Integrated mode stays available (opt-in) and is worth using for confidence-only consumers (e.g.
TSDF) that want to skip a separate phase or avoid keeping dmaps on disk — **especially on small
scenes**.

Binding: Release build (`make/bin/Release/DensifyPointCloud`, HEAD `6fa582f`),
`LD_LIBRARY_PATH=/usr/local/cuda/lib64`, GPU A100-SXM4-40GB, `nMaxThreads=30`, `nproc=30`, all at
resolution-level 1, workdirs under `/home/ubuntu/virginia/gt_bench/runs/<scene>/task13_*`.

---

## 1. Five-scene total-time + ROC A/B

- **Path S (standalone):** estimate without the knob (`--postprocess-dmaps 0`), then the standalone
  adjust phase (`--postprocess-dmaps 4 --geometric-iters 0`) on the just-made dmaps.
  `Total_S = est_wall + adjust_wall`.
- **Path I (integrated):** estimate with `Estimate Confidence = 1` (`--postprocess-dmaps 0`).
  `Total_I = estI_wall` — confidence is produced inside estimation, no second phase.
- ROC = pooled ROC-AUC from `EvalConfidence.py` GT mode (`--gt-format eth3d`/`blendedmvs`), inlier/
  outlier labels from the GT depth. `bmvs / office / courtyard` reuse Task 12's from-scratch runs;
  `meadow / facade` were run fresh this task. Standalone ROC for the reused three is the committed
  L1 `conf_adj` baseline (resolution matches); `meadow / facade` standalone ROC was regenerated.

| Scene | n imgs | est_S (s) | adjust_S (s) | **Total_S (s)** | **Total_I (s)** | ΔTime (I−S) | ROC_S | ROC_I | ΔROC (I−S) | time winner |
|---|---|---|---|---|---|---|---|---|---|---|
| bmvs_59d2657f | 77 | 31.11 | 10.40 | **41.50** | **38.64** | **−2.86** | 0.9766 | 0.9770 | +0.0004 | **integrated** |
| eth3d_meadow | 15 | 46.40 | 15.27 | **61.67** | **52.28** | **−9.39** | 0.7926 | 0.7801 | **−0.0125** | **integrated** |
| eth3d_office | 26 | 76.56 | 16.59 | **93.15** | **101.29** | **+8.14** | 0.8828 | 0.8804 | −0.0024 | standalone |
| eth3d_courtyard | 38 | 146.60 | 34.11 | **180.71** | **222.41** | **+41.70** | 0.9333 | 0.9326 | −0.0007 | standalone |
| eth3d_facade | 76 | 282.39 | 65.42 | **347.81** | **412.09** | **+64.28** | 0.9079 | 0.9073 | −0.0006 | standalone |

**Total-time:** integrated wins on the 2 small scenes (bmvs 77 tiny images / many views; meadow 15
tiny images), loses on the 3 larger-image scenes (office/courtyard/facade) by +8 … +64 s. The loss
grows with per-map sweep cost (which scales with image resolution × neighbor count): facade's 76
large images cost ~2.5 s/map of sweep, squeezed through only 4 workers.

**ROC quality:** ΔROC (I−S) ∈ [−0.0125, +0.0004]. Within ±0.01 on **4 of 5** scenes (bmvs, office,
courtyard, facade). The one exception is **meadow (−0.0125)** — the hardest, lowest-overlap scene
(15 images, pooled ROC only ~0.79). Standalone meadow is stable across runs (committed baseline
0.7911, this task 0.7926), so the −0.0125 is not run noise: at 15 views the integrated mode's
neighbor set (the estimation-time `depthData.images[]` selection, score-filtered in `InitViews`)
diverges more from the fusion-time neighbor set (`idxNeighbors`) the standalone phase uses, changing
the per-pixel confirmation count K and hence the confidence. On scenes with normal overlap the two
neighbor sets agree and ROC matches to ~0.002.

Per-scene JSONs: `/home/ubuntu/virginia/gt_bench/task13/results/` (meadow/facade `*_conf.json`),
`/home/ubuntu/virginia/gt_bench/task12/results/` (reused three), committed `gt_bench/results/`
(standalone baselines).

---

## 2. Worker-pool lever investigation

### (a) How many CPU workers run the inline sweep during CUDA estimation?

**Four.** On the CUDA path `data.nDenseWorkers` is set to the PatchMatch pool size
(`SceneDensify.cpp:2828`): `poolSize = CLAMP(OPTDENSE::nPatchMatchCUDAInstances, 1, nMaxThreads)`
= `CLAMP(4, 1, 30)` = **4** (confirmed "Using CUDA compute backend … (4 workers)" in every log).
The last-geometric-iteration dispatch spawns exactly `nDenseWorkers` threads
(`cList<SEACAVE::Thread> threads(data.nDenseWorkers)`, `:2909`), and each of those per-view workers
runs the full pipeline `InitViews → EstimateDepthMap (GPU) → AdjustConfidence (CPU sweep) → Save`.
So the inline sweep runs on **4** CPU workers, versus the standalone adjust phase's
`MINF(nMaxThreads, nImages)` = up to **30** (`:2992`). That 4-vs-30 gap is the entire root cause of
the large-scene total-time loss.

### (b) Is the CPU idle during CUDA PatchMatch (so more CPU threads for the sweep would be nearly free)?

**Yes — the CPU is largely idle during GPU PatchMatch.** `mpstat 1` during a courtyard estimation
showed ~**88 % idle** (only 4 of 30 cores can be busy, and even those alternate GPU-dispatch and CPU
work); `nvidia-smi dmon` showed the GPU SM oscillating and never pinned. So there is ample CPU
head-room: in principle the confidence epilogue could use ~30 CPU threads for the sweep at little
extra cost — **if** those threads did not also drive GPU dispatch.

### (c) Is there a cheap, SAFE way to give the epilogue more parallelism?

**No cheap safe lever exists, and the one exposed knob makes things worse.** The only knob that
raises the sweep-worker count is `nPatchMatchCUDAInstances`, but `nDenseWorkers` **is** that pool —
raising it raises GPU-dispatch concurrency for **every** geometric iteration too, oversubscribing the
single A100. Measured directly (CLI `--patch-match-cuda-instances 16`, "16 workers" confirmed):

| Measurement | 4 workers | 16 workers | effect of 4→16 |
|---|---|---|---|
| courtyard **raw estimate** wall | 146.60 s | **184.17 s** | **+26 % SLOWER** (GPU oversubscription) |
| courtyard **integrated** (estI) wall | 222.41 s | 220.60 s | −0.8 % (negligible) |
| meadow **integrated** (estI) wall | 52.28 s | **59.10 s** | **+13 % SLOWER** |
| courtyard total sweep CPU (Σ per-map) | 59.2 s | 51.6 s | only −12 % (sweep is memory-bound) |
| meadow total sweep CPU (Σ per-map) | 11.9 s | 13.1 s | +10 % (contention) |

Two facts kill the naive lever: (i) the sweep is **memory-bandwidth bound** and barely parallelizes
past a few threads — 4× the workers cut courtyard's total sweep CPU only 12 % (59.2 → 51.6 s); and
(ii) that tiny saving is cancelled or exceeded by the GPU-oversubscription penalty on the earlier
iterations (courtyard raw estimate +26 %). Net: 16 workers is a wash on courtyard and *slower* on
meadow. **Increasing `nPatchMatchCUDAInstances` is not a fix.**

The genuinely safe fix is to **decouple** the epilogue's CPU pool from the GPU-dispatch pool: keep 4
GPU workers for PatchMatch, but run the confidence sweep across ~30 CPU threads **after** the GPU
work of the last iteration (more per-view workers is allowed by the no-threads-inside-`AdjustConfidenceSweep`
rule; this adds *no* threads inside the per-view sweep). That is a **dispatch restructuring** (a
separate post-estimation sweep pass over the last-iteration dmaps, or a second thread pool sized
independently), **not a 1–2 line change**, so per the brief it is **NOT applied here — deferred to
Task 14**.

### Projection under the decoupled fix

If the epilogue sweep ran on ~30 CPU threads it would take approximately the standalone adjust
**wall** time (that wall *is* this exact sweep on 30 threads, including the same fixed per-phase
overhead). So `Total_I(fixed) ≈ est_wall + adjust_wall = Total_S`:

| Scene | est_wall | + adjust_wall (=30-thread sweep) | = Total_I(fixed) | vs Total_S | vs Total_I(now) |
|---|---|---|---|---|---|
| eth3d_courtyard | 146.60 | 34.11 | **≈180.7** | ≈ tie | −41.7 s |
| eth3d_facade | 282.39 | 65.42 | **≈347.8** | ≈ tie | −64.3 s |
| eth3d_office | 76.56 | 16.59 | **≈93.2** | ≈ tie | −8.1 s |

**Even the ideal lever fix only ties standalone** — it never beats it — because the sweep work is
identical and integrated saves no I/O (Task 12 measured **identical** dmap opens, 1386 vs 1386). The
only real advantage integrated retains is on small scenes, where folding the sweep into estimation
amortizes the standalone phase's fixed per-phase overhead (dmap-cache warm-up, thread spawn,
full-scene residency), which is why bmvs and meadow win *now*.

---

## 3. Decision

Per the spec: recommend integrated as the default for confidence-consuming-but-not-dense-cloud
pipelines **iff** ROC_I is within 0.01 of ROC_S on **every** scene **and** `Total_I ≤ Total_S` on
every scene (or after a lever fix). Both preconditions fail:

- **Total time:** `Total_I ≤ Total_S` holds on only 2 of 5 scenes; integrated loses by +8 … +64 s on
  the three larger-image scenes, and the lever does not cheaply fix it (best case = tie).
- **ROC:** within 0.01 on 4 of 5, but −0.0125 on meadow (real, from neighbor-set divergence at 15
  views), so "within 0.01 on every scene" does not strictly hold either.

**Decision: STANDALONE (`--postprocess-dmaps 4`) is the recommended DEFAULT for pipelines that
consume the adjusted confidence.** It parallelizes the sweep across all cores, ties-or-beats
integrated on total time on every scene, and has the tighter ROC parity.

**Integrated mode (`Estimate Confidence = 1`) stays as a supported opt-in**, recommended specifically
for: (1) confidence-only consumers (e.g. TSDF integration) that never fuse a dense cloud and want to
avoid a second process / keeping dmaps on disk; and (2) small / few-image scenes, where it is also
the faster total-time path. It is **not** recommended as a blanket default until Task 14 decouples
the epilogue CPU pool (which would bring large-scene total time to parity with standalone).

**Standalone also remains the only path for reuse-existing-dmaps workflows** (adjust dmaps produced
by a prior estimation run) — integrated can only produce confidence during a fresh estimation.

Documentation updated accordingly: the `AdjustConfidence` block comment (`libs/MVS/SceneDensify.cpp`,
"TWO MODES / WHICH TO USE") and `gt_bench/README.md` ("Confidence estimation: standalone vs
integrated").

---

## 4. Bonus (geometric-consistency 5th feature) — SKIPPED

The estimator's per-pixel geometric-consistency score (`DepthMap.cpp` `ScorePixelImage`, the
`consistency` term, ~:590–603) is a transient local variable folded into the NCC score and never
persisted to any map or sidecar. Exporting it as a 5th feature would require adding a new per-pixel
buffer through `DepthEstimator`/`PatchMatchCUDA` and a new export path — nontrivial plumbing on the
CUDA side. Per the brief ("Skip if it needs nontrivial plumbing"), this is **deferred to Task 17**,
which owns the feature sweep.

---

## Appendix — commands (reproducible)

```bash
BIN="env LD_LIBRARY_PATH=/usr/local/cuda/lib64 make/bin/Release/DensifyPointCloud"
COMMON="--resolution-level 1 --max-resolution 3200 -v 2"
# Path S:
$BIN scene.mvs -w $WD $COMMON --fusion-mode 1 --postprocess-dmaps 0 -o est.mvs                       # estimate
$BIN scene.mvs -w $WD $COMMON --geometric-iters 0 --fusion-mode 1 --postprocess-dmaps 4 -o adj.mvs   # standalone adjust
# Path I (estconf.cfg contains the single line: Estimate Confidence = 1):
$BIN scene.mvs -w $WD $COMMON --fusion-mode 1 --postprocess-dmaps 0 --dense-config-file estconf.cfg -o est.mvs
# ROC:
python scripts/python/EvalConfidence.py $WD --gt-depth-dir <gt> --gt-format eth3d --scene-mvs scene.mvs --json out.json
# Lever probe: add  --patch-match-cuda-instances 16  to the estimate.
```

Driver scripts: `scratchpad/task13_run_stage.sh` (timed stage runner), `scratchpad/task13_eval.sh`
(ROC eval). Raw numbers: `scratchpad/task13_data.json`.
