<!--
PROVENANCE: Task 8 (MapAnything-vs-GT calibration study), 2026-07-03.
Produced by scripts/python/MapAnyVsGT.py against the pseudo-GT pipeline unchanged since
commit 7f8ca1c (MapAnyInferMV.py MA_MASK_EDGES=0/MA_RES=910, MapAnyVoxelFuse.py MINVIEWS=2).
Scenes: eth3d_courtyard L2, eth3d_office L2 (real SfM sparse-depth conditioning), bmvs_5a640093 L1
(scene.mvs has ZERO SfM sparse vertices -- InterfaceMVSNet import -- so its MapAnything witness was
generated WITHOUT sparse-depth conditioning, images+intrinsics+poses only, via the standalone
mapany/MapAnyInferMV_nosparse.py helper; labelled `nosparse_witness=true` in its JSON).
Full numbers: /home/ubuntu/virginia/gt_bench/mapany/<scene>/mapanyvsgt.json.
Full task report: /home/ubuntu/openMVS/.superpowers/sdd/task-8-report.md.
-->
# Verdict: was the MapAnything pseudo-GT trustworthy?

**Short answer: no, not for quantitative tuning decisions.** The MapAnything pseudo-GT clouds that
guided all pre-benchmark confidence/fusion-rescue tuning have a positional error floor (tens of cm to
several tens of percent of scene depth) comparable to or larger than the effect sizes (single-digit
percentage-point completeness/gross-outlier deltas) that tuning was trying to resolve. The clouds
built from it (`magt.ply`) score 3-51% accuracy and 17-62% gross-outlier rate against REAL ground
truth, on scenes where the actual OpenMVS reconstruction it was supposed to grade scores 87-99%
accuracy and <1% gross-outlier rate on the SAME real GT.

## (a) How noisy is MapAnything depth vs real GT?

Per-view depth compared to real GT depth (GtUtils: ETH3D distorted-grid remap / BlendedMVS PFM),
**after** the exact per-view MVS-confident-pixel scale alignment `MapAnyVoxelFuse.py` itself applies
before voxel-fusing (so this scores precisely the depth quantity that fed the pseudo-GT cloud, not
the model's raw unanchored output):

| Scene | sparse cond.? | n views scored | median rel. depth err | frac >1% | frac >3% | frac >10% |
|---|---|---|---|---|---|---|
| eth3d_courtyard L2 | yes (33.5k SfM pts) | 38/38 | **3.3%** | 82.7% | 53.5% | 11.0% |
| eth3d_office L2 | yes (3.5k SfM pts) | 22/22 | **6.3%** | 88.7% | 69.6% | 37.3% |
| bmvs_5a640093 L1 | **no** (0 SfM pts) | 110/110 | **3.8%** | 84.0% | 57.7% | 19.0% |

At the scenes' median depths (courtyard 8.9m, office 2.0m, bmvs 1.2m) a 3-6% median relative error is
already a ~5-30cm typical displacement -- at or above even the LOOSEST tier (10cm ETH3D / 4.5cm
BlendedMVS) of the real-GT benchmark's own tolerance ladder, and 3-25x the finest tier (1cm). Coverage
was complete (0 skipped views on all 3 scenes: every dmap had a matching MapAnything witness, a valid
MVS-confident scale anchor, and real GT). Notably office (WITH real sparse conditioning) is the
*worst* of the three by this metric -- sparse-depth conditioning alone does not guarantee low error;
indoor low-texture scenes remain hard for the model regardless.

## (b) Per-cloud accuracy vs real GT (and against `cloud_w3.ply`, for context)

`magt.ply` (the multi-view-gated, `MINVIEWS>=2`, voxel-fused pseudo-GT cloud) scored with the SAME
tools/tolerances `gt_bench/run_scene.sh` uses for the real fusion eval (`eth3d_eval.sh` /
`EvalFusionGT.score_cloud`), alongside `cloud_w3.ply` (the actual OpenMVS reconstruction, rescue ON)
for context:

| Scene | cloud | n pts | completeness (loosest tier) | accuracy (loosest tier) | gross-outlier frac |
|---|---|---|---|---|---|
| eth3d_courtyard | magt.ply (pseudo-GT) | 747,793 | 51.2% @10cm | **23.5%** @10cm | **34.8%** @50cm |
| eth3d_courtyard | cloud_w3.ply (real recon) | 3,014,151 | 54.3% @10cm | 99.4% @10cm | 0.26% @50cm |
| eth3d_office | magt.ply (pseudo-GT) | 420,883 | 66.9% @10cm | **50.8%** @10cm | **17.4%** @50cm |
| eth3d_office | cloud_w3.ply (real recon) | 904,388 | 47.3% @10cm | 98.6% @10cm | 0.70% @50cm |
| bmvs_5a640093 | magt.ply (pseudo-GT, no-sparse) | 1,033,123 | 91.7% @1%diag(4.5cm) | **13.9%** @4.5cm | **61.7%** @gross(22.5cm) |
| bmvs_5a640093 | cloud_w3.ply (real recon) | 3,697,099 | 92.9% @4.5cm | 99.4% @4.5cm | 0.14% @gross(22.5cm) |

(Full tolerance ladders and completeness/accuracy at every tier are in the per-scene
`mapanyvsgt.json`; ETH3D tolerances are fixed absolute meters, BlendedMVS tolerances are fractions of
the GT sample cloud's bbox diagonal -- not cross-dataset comparable, same caveat as
`gt_bench/BASELINE_2026-07.md` table [3].)

**Reading the asymmetry**: completeness (does *some* pseudo-GT point sit near each real-GT sample) is
often reasonable-to-good (51-92%) -- the mono witness genuinely covers most of the true surface
region. But accuracy (does each pseudo-GT *point* sit near real GT) is uniformly bad (2.7-51%,
usually well under half) and gross-outlier rate is high (17-62%). In other words: **the pseudo-GT
cloud finds roughly the right surface but places its points imprecisely and, on a large fraction of
points, outright wrong** -- exactly the asymmetry you'd expect from noisy per-pixel depth
(median 3-6% relative error, long tail to 20-40%+) fused into a point cloud with no ground-truth
correction. `cloud_w3.ply`, by contrast, is 87-99% accurate and <1% gross-outlier against the SAME
real GT on every scene -- the actual OpenMVS reconstruction the pseudo-GT was supposed to be grading
is **1-2 orders of magnitude more reliable than its own former grader.**

The no-sparse-conditioning bmvs witness is not qualitatively different in per-pixel depth error (3.8%
median, similar to the two ETH3D scenes) but is markedly worse in cloud-level gross-outlier rate
(61.7% vs 17-35%) -- per-view scale correction (which this study applies uniformly to isolate the
"is the depth-*shape* usable" question from "did the model get the right absolute scale") cannot fix
cross-view SHAPE disagreement, and shape consistency is what the multi-view voxel gate and the
gross-outlier test actually depend on. Sparse-depth conditioning helps, but even WITH it (eth3d
scenes) gross-outlier rate is still 17-35% -- high enough that no scene here would be called
trustworthy pseudo-GT by the accuracy/gross-outlier axis.

## (c) How far do the OLD pseudo-GT-based conclusions hold under real GT?

**No scene overlap exists** between `/home/ubuntu/virginia/mvs_bench/AGGREGATE.md` (ladita,
SceauxCastle, Meetingroom, Truck, HIRESDEMO_CROP1 -- Tanks&Temples-style scenes with no real GT
available) and `gt_bench/BASELINE_2026-07.md` (ETH3D + BlendedMVS scenes, which DO have real GT) --
they are disjoint scene sets by construction (real GT didn't exist for the old scenes). The comparison
below is therefore of the *qualitative claims themselves*, replicated independently under real GT on a
different scene set, not a paired before/after on the same scenes.

| Old pseudo-GT-based claim (AGGREGATE.md) | Real-GT BASELINE (28 scene-levels, 11 scenes) | Holds? |
|---|---|---|
| "w3 >= w0 on EVERY scene and EVERY tolerance" (completeness) | Checked all completeness cells in BASELINE table [2]: **w3 >= w0 in all 28 scene-levels, every tolerance, zero exceptions.** | **YES, exactly.** |
| "adds ~no gross outliers" (dense-GT floater test: w3 delta 0.0005-0.20pp) | BASELINE table [3]: gross-outlier% w0->w3 delta is **never negative** (28/28), but ranges 0.0 to **+2.6pp** (bmvs_59d2657f 4.7%->7.3%/5.4%->7.8%; ETH3D scenes mostly +0.0 to +0.3pp, e.g. eth3d_office L3 0.6%->0.9%). Several deltas are a ~40-50% *relative* increase on their own base rate. | **Directionally yes** (rescue doesn't blow up outliers; absolute rates stay under 8% everywhere) **but the old "~no outliers added" MAGNITUDE claim understated it** -- real GT shows small-but-consistently-positive, occasionally non-trivial (bmvs_59d2657f) increases the pseudo-GT floater test didn't surface at that magnitude. |
| Confidence recalibration net positive (ROC/PR gains) | BASELINE table [1]: dROC +0.02 to +0.15 in 27/28 scene-levels (one exception: eth3d_meadow L1, -0.019, a 15-image low-structure grass field -- already flagged as the sole negative in BASELINE's own provenance note). | **YES, robustly** -- and this claim was never MapAnything-dependent: `LabelFusionInliers`/`EvalConfidence.py`'s GT mode grades confidence against fusion-geometry labels or real GT depth directly, not against the MapAnything witness. This axis of the work is unaffected by today's verdict. |
| mvs-missing identical w0/w3 (rescue only revives discarded depths, never fabricates) | Not re-measured here (BASELINE has no root-cause decomposition table); this is a mechanistic/code-level invariant (`fFusePriorWeight` only grants virtual support to pixels the reference dmap already estimated -- see `DenseFuseDepthMaps`/`ComputeIntraMapPrior`), not a GT-quality-dependent claim, so it is unaffected either way. | N/A (untested here, but architecturally guaranteed) |

**Where MapAnything pseudo-GT actually mattered**: it was NOT used for confidence-recalibration
scoring (that used fusion-geometry labels, and now real GT). It WAS used (a) as the completeness/
gross-outlier witness behind the `mvs_bench/AGGREGATE.md` table [3]-[5] numbers, and (b) to pick the
`fFusePriorWeight=3` default and validate the plane+normal prior redesign (variant A) via
`fuse_rescore_mono.py`'s per-added-point majority-support precision estimates. (a)'s DIRECTIONAL
conclusion (w3>=w0 everywhere) independently replicated under real GT; its precise MAGNITUDES (the
"~no outliers added" claim, and any completeness percentage read at face value) should not be trusted
to better than the pseudo-GT's own ~3-6%-depth/17-62%-gross-outlier error floor. (b)'s precision
numbers (e.g. "added points are 92-95% as mono-consistent as gold-standard fusion points") were always
RELATIVE (rescued vs. baseline points scored against the SAME noisy witness, differencing out most of
the witness's own bias) rather than absolute accuracy claims, so they are less exposed by this finding
than a bare "X% complete" or "Y% gross outliers" statement would be -- but they still inherit whatever
correlation the witness's noise has with the rescue mechanism itself, which this study cannot rule out.

## (d) Decision

**Retire MapAnything pseudo-GT for tuning.** Do not use `magt.ply` completeness/accuracy/gross-outlier
numbers, or per-pixel MapAnything depth comparisons, to justify a knob change, a threshold pick, or a
magnitude claim ("+X% completeness", "adds ~no outliers") going forward -- `gt_bench`'s real GT
(ETH3D laser scans, BlendedMVS textured meshes) now covers exactly the scene types (indoor/outdoor,
BlendedMVS/ETH3D) the old pseudo-GT work targeted, at 1-2 orders of magnitude better precision
(87-99% accuracy / <1% gross-outlier vs the pseudo-GT's 3-51% / 17-62%), and should be used instead.

**Conditionally keep it for GT-less visual/directional QA only**, with these error bars stated
explicitly every time it's cited: per-pixel MapAnything depth error is ~3-6% median relative (worse,
19-37%, beyond the 10% tail) even after MVS-anchor scale correction; the resulting voxel-fused cloud
is 3-51% accurate and 17-62% gross-outlier against real structure. It remains usable for: (i) a rough,
DIRECTIONAL sanity check ("did completeness go up or down") on brand-new scenes with no real GT yet --
the one directional claim tested here (w3>=w0) replicated exactly under real GT -- and (ii) qualitative
visual inspection (the floater/off-surface pathologies it previously caught, e.g. the pre-gate
SceauxCastle bug, are still visible by eye). It must NOT be used for: magnitude/percentage claims,
gross-outlier-rate claims, or any A/B decision where the effect size under test is within an order of
magnitude of the witness's own ~3-6%/17-62% error floor -- which describes essentially every tuning
decision made so far (single-digit-percentage-point completeness deltas, sub-1%-point gross-outlier
deltas).
