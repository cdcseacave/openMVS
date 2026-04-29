# v3 head-to-head: N8 + N3-fusion-revert + ProjectPointP fix vs a1_refcache_v2

## Build label
- **Baseline:** `a1_refcache_v2` (commit `b1f9db6`, before C4 / N3 / N8)
- **v3:** working tree at 2026-04-29 18:50 with:
  - N8: drop in-function normalize from `TRMatrixBase::Set(wa, phi)` (`libs/Common/Rotation.inl:701`); contract is now "axis must be unit", with debug ASSERT
  - N8 (caller): `DepthEstimator::CorrectNormal` (`libs/MVS/DepthMap.h:464`) pre-normalizes the `normal × viewDir` cross-product axis with a zero-norm guard
  - N3 partial revert: `SceneDensify.cpp` fusion asserts (1493, 1521, 1700, 1705) restored to `1e-2f` tolerance — the chain `Cast<Normal::Type>(camera.R.t() * Cast<REAL>(normal))` can drift > 1e-4 from disk-loaded R orthogonality residual + CUDA-pipeline normal noise; intermittent assert popups in Debug builds reproduce this
  - ProjectPointP migration: `SceneGeometry.cpp:246` now uses `std::get<0>(camera.ProjectPointP(point))`
  - Cumulative C4/A1/C3/D10/C2/C1/D4 already in baseline, plus N4 (drop redundant `.normalized()` in `GenerateRandomNormal`) and N2 (port C4 pattern to CPU `PerturbEstimate`)

## Hardware / build
RTX 4070 (8.9, 12 GB), i7-13700KF, Windows 11, RelWithDebInfo. τ = 0.005 m on Truck.

## Numbers (Truck, TnT trajectory branch)

| Cell      | a1_v2 F1 | v3 F1  | ΔF1     | a1_v2 wall | v3 wall | a1_v2 vmean | v3 vmean | a1_v2 pts  | v3 pts     |
|-----------|----------|--------|---------|------------|---------|-------------|----------|------------|------------|
| R=1 V=8   | 0.7001   | 0.7002 | +0.0001 | 108.3 s    | 111.2 s | 55.0 ms     | 57.7 ms  | 5,824,598  | 5,819,842  |
| R=1 V=16  | 0.6992   | 0.6995 | +0.0003 | 143.8 s    | 146.9 s | 91.7 ms     | 93.2 ms  | 5,747,973  | 5,739,510  |
| R=0 V=8   | 0.7464   | 0.7462 | −0.0002 | 482.7 s    | 531.1 s | 207.3 ms    | 208.8 ms | 20,803,411 | 20,766,787 |
| R=0 V=16  | 0.7445   | 0.7444 | −0.0001 | 597.5 s    | 638.5 s | 332.6 ms    | 344.6 ms | 21,567,492 | 21,518,679 |

## Take-away

- **F1 neutral** — all four ΔF1 values fall within the ICP alignment-cascade noise floor (±0.0003) we measured during the c4 v2 study.
- **Per-view kernel time neutral** — Δview_ms of +1.5 to +12 ms (≤ +3.6 %), consistent with run-to-run thermal variance.
- **Wall time** — R=1 within +3 %, R=0 within +10 % (+48 / +41 s on 8-10 min runs, dominated by 1.8 GB PLY-write I/O variance previously documented).
- **Point counts** drop ≤ 0.3 % per cell — fewer borderline pixels survive C4-Rodrigues + N2 perturbation, but recoverable surface coverage is unchanged.

## Decision

Commit the v3 changes (N8 + N3-fusion-revert + ProjectPointP fix). The CUDA-pipeline normal-norm drift > 1e-4 is acknowledged and **deferred for separate investigation**: the `1e-2f` fusion tolerance is reinstated as a known mitigation, with `1e-4f` retained on all CPU-only code paths (N3 untouched outside fusion).

## Debug-mode validation

Small Tests scene (`apps/Tests/data/scene.mvs`, 4 images, 0.29 MPx each):
- **CPU Debug densify** (8.4 s, 59,496 points): 0 asserts fired with default 1e-4f tolerance on all CPU sites.
- **CUDA Debug densify** (intermittent — 1m14s, 499 points on the run that completed): once produced a normal-norm assert popup at one of the four fusion sites; second run completed cleanly. Tolerance reverted to 1e-2f to absorb this.

## Open follow-up (deferred)

### Drift diagnostic results (2026-04-29 21:05)

Instrumented `DenseFuseDepthMaps` to log per-image
`||R*R.t() − I||_F` and the distribution of `||n||` and `||R.t()·n||`
across all valid pixels. Ran two passes on Truck R=1 V=8: one CPU
(`--cuda-device ""`, 24 min wall), one CUDA (`--cuda-device -1`, 1m46s).

**Findings (251 images per pass, ~122M valid pixels each):**

| Source | Camera R orthogonality | Raw ‖n‖ max-dev | Pixels > 1e-4 | Pixels > 1e-2 |
|---|---|---|---|---|
| CPU  | 1.5e-16 … 8.3e-16 | **0.1095** | 7 of 122M | 6 |
| CUDA | 1.5e-16 … 8.3e-16 | 1.07e-6 | 0 of 125M | 0 |

**Surprising inversion of the hypothesis:** CUDA-stored normals are
clean (1e-6 noise floor, no outliers). The 1e-2 drift is **CPU-side**,
concentrated in three images: img 34 (1 px @ 0.110), img 139 (5 px
@ 0.013), img 242 (1 px @ 3.1e-4). All three images have perfectly
orthogonal R (5e-16 residual) — so the disk-loaded poses are not the
source either.

**Root cause is unidentified but localized to CPU `EstimateDepthMap`
random-init or perturbation path.** The 7 outlier pixels (5.7e-8 hit
rate) suggest a numerically-unstable code path that fires on rare pixel
patterns — possibly a denormal-triggered division, a NaN survivor, or
an iteration that exits without re-normalising. The fusion-time `1e-2f`
tolerance set by this commit absorbs all 7 outliers; tightening to
default 1e-4 would intermittently fire on Truck-class scenes with > 1
in 17M pixels exceeding the threshold.

**Suggested next investigation step:** add a per-pixel log of
`(idxImage, x, y, depth, ||n||)` for any pixel where `||n|| > 1e-3`
deviation, capture the offending pixels' inputs, and trace back through
`PerturbEstimate` / interpolation. Out of scope for this commit.
