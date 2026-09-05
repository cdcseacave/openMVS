# View-graph disambiguation: what the triplet filter keeps, what threshold it picks, and a second cue

**Date:** 2026-09-05
**Branch:** `feature/roma2-onnx`
**Supersedes nothing.** Extends the filter shipped by `e2a376e` / `9f97c62` / `f01b872` / `9c56eae`
and documented in `docs/design/TripletDisambiguation.md`.

## 1. Why

The camera-triplet filter (Manam & Govindu, CVPR 2024) ships behind `--filter-triplets`, default
off, because the pre-registered promotion rule failed: at `m = 0.6` every capture loses registered
images (−5.3 % to −76.4 %) and three of five worsen their median rotation error. It is a large win
in exactly one measured place — the dense repetitive graph, 368 of 377 images in one component
against 131.

Three separate measurements say the default is failing for reasons that are fixable, not because
the score is uninformative:

1. **The score ranks well; the removal rule is crude.** AUC up to 0.90, precision ≈ 1.0 among the
   kept pairs. What loses the images is not the ranking.
2. **Most of the removed pairs are removed for having no evidence, not bad evidence.** Algorithm 1
   step 1 discards every edge outside the largest triplet-graph component, which includes every
   edge in no triangle at all. On the labelled reference those are overwhelmingly *true* pairs:
   426 of 490 unscored labelled pairs on `3b43828e`, 415 of 441 on `32265651`.
3. **`tau` is not calibrated for video keyframes.** Eqn. 3 derives it from `m` and `d_max/|V|`;
   nothing in that derivation knows the graph is about to fragment.

This spec fixes 1-3 in that order, then adds a second, independent cue, because the one fusion
measurement available says independent cues beat any single one.

## 2. Scope

In scope: the removal rule, an automatic threshold, a second per-edge cue, and the measurements
that decide whether any of it may flip a default.

Out of scope, deliberately:

- **Flipping `--filter-triplets` to default true.** No measurement in this spec authorises that.
  The promotion rule stays the conjunction already recorded in `TripletDisambiguation.md`, and it
  is re-run, not relaxed.
- **Rule C, thresholding the score distribution.** Ruled out on evidence: every capture's score
  histogram is smooth and broad with its peak in the top bin and no valley, so an Otsu cut, a
  mixture fit and a knee search all have nothing to lock onto. The threshold has to come from a
  graph-level objective, which is what §3.2 is.
- **Doppelgangers++ as a cue.** Permanently out. Its labels remain usable as a *reference* for
  scoring cues offline; the model itself never enters the pipeline.

## 3. Design

### 3.1 An edge is removed for bad evidence, never for absent evidence

`FilterPairsByTriplets` currently removes a pair when `score < tau`, and an unscored pair carries
`-1`, so absence of evidence removes the pair through the same comparison as weak evidence. Those
are different claims and only one of them is supported.

**Rule:** a pair with no score is kept. Only a scored pair below `tau` is removed.

No configuration flag. The filter is already opt-in; adding a second switch to choose between a
rule we have evidence for and one we have evidence against is a route to nowhere. The paper's exact
variant stays reproducible offline: `pairs.csv` exports `TripletScore`, and an unscored pair is
exactly an empty cell.

Consequences that are part of the change, not follow-ups:

- The header comment on `ComputeTripletScores` and `FilterPairsByTriplets`, the `--filter-triplets`
  CLI help ("and the pairs in no triangle at all"), and `docs/design/TripletDisambiguation.md` all
  currently *promise* the old behaviour. Each is wrong the moment this lands.
- The log line reports unscored pairs as kept, not as removed.
- `TripletFilterTest`'s path-graph case asserts that a graph with no triplet at all loses every
  pair. Under the new rule it must keep every pair. That assertion flip is the test that proves the
  change bites; a test that passes both before and after would prove nothing.

Predicted survivor-graph effect at the paper's own tau, from `AUTO-ENABLE-ANALYSIS.md`:

| arm | keptLCC | components | images below degree 2 |
|---|---|---|---|
| `32265651`/sift | 305 → 373 | 72 → 3 | 75 → 7 |
| `3b43828e`/sift | 198 → 289 | 177 → 15 | 193 → 39 |
| `32265651`/`roma2gate` (the win case) | 376 → 377 | 2 → 1 | 3 → 1 |

The win case is essentially untouched, which is the property that matters: a less aggressive filter
must not give back the one place the filter earns its keep.

### 3.2 The threshold is the strictest one that keeps the graph together

Score once. The paper's Eqn. 3, `tau(m) = m (1 - r) + r` with `r = d_max/|V|` of `G_LCT`, is the
**ceiling**: the filter is never stricter than the `m` it was given. Below it, the threshold is the
**strictest** `tau` whose survivor graph — every unscored pair, every pair scoring at or above
`tau` — keeps at least 99 % of the unfiltered graph's largest connected component in one component.
There is no other bar. If the ceiling itself keeps the graph together it is applied as given; if
nothing above the lowest score does, the lowest score is chosen and nothing is removed.

**Why the strictest, and why connectivity alone.** This was decided against the standard
ambiguous-scene datasets (`~/virginia/datasets/Disambiguation`, the sets of Yan et al. 2017 and
Heinly et al. 2014 that the disambiguation literature is measured on), by replaying candidate rules
on each set's exported view graph and looking at the images. On those sets the shape of the problem
is the opposite of what the previous rule assumed:

- **Every pair verifies.** Two identical facades give every image pair 50-1000 epipolar inliers, so
  exhaustive matching on a 19-image street pan yields the complete graph, `r = 18/19` and a ceiling
  of 0.98 at `m = 0.6`. The true pairs are the band of frames within a few steps of each other — a
  *minority* of the graph (35 of 171 on street). The correct filter removes most of the edges.
- **The score ranks them correctly.** Against a frame-gap truth (pairs within two frames true,
  beyond it false) the triplet score reaches AUC 0.99 on street; the strong chain scores 0.95-1.0,
  the doppelganger pairs 0.67-0.75, and the true low-overlap pairs *below* the doppelgangers.
- **So the right answer is the backbone.** Keep the strongest edges down to the point where they
  hold the graph together, and no further: every edge below that is either a weak true pair the
  chain does not need or a doppelganger, and nothing in the counts tells the two apart. On street
  that is `tau = 0.961`: 20 of 171 pairs, the chain intact, no far pair kept.

The rule this replaces — relax from the ceiling and accept the first candidate that fragments
nothing, strands nobody and removes under 20 % of the pairs — was fitted on video-keyframe captures
and stands down on **all six** small sets, because the correct answer there removes 66-96 % of the
pairs and leaves the chain's two endpoints at degree 1. Its "doppelgangers are a minority by
construction" premise is false exactly where the filter is needed. The paper's own fixed threshold
errs the other way: at `m = 0.3` it fragments five of the six chains (largest component 10/19,
7/21, 9/23, 7/25, 20/31, 34/64), which is the oversplit the paper's Table 3 reports on the same sets
(9/21, 7/25, 12/31, 9/23 cameras). The connectivity-driven threshold keeps every chain whole:

| set | images | pairs | r | ceiling (m=0.6) | chosen tau | kept | far pairs kept |
|---|---|---|---|---|---|---|---|
| street | 19 | 171 | 0.947 | 0.979 | 0.961 | 20 | 0 |
| books | 21 | 209 | 0.952 | 0.981 | 0.905 | 47 | 0 |
| oats | 23 | 253 | 0.957 | 0.983 | 0.893 | 44 | 1 |
| cereal | 25 | 296 | 0.960 | 0.984 | 0.804 | 69 | 4 |
| desk | 31 | 439 | 0.968 | 0.987 | 0.978 | 37 | 3 (loop closure) |
| cup | 64 | 2013 | 0.984 | 0.994 | 0.991 | 75 | 1 |

**The known frontier, stated rather than hidden.** When the true junction between two parts of a
scene is weaker than a doppelganger pair between them, the rule reconnects through the
doppelganger. On oats the two identical canisters (frames 0-7 and 13-22) are joined by the pair
(6,21) with 880 inliers at score 0.893 — the chosen threshold *is* that pair's score — because the
true transition through the featureless Wheat Thins frames scores lower. Cereal has the same shape
with its two boxes. No statistic of counts and triangles separates that pair from a true junction:
a doppelganger's triangles are mutually consistent, and its inlier count matches a true adjacent
pair's. The paper's fixed threshold avoids it only by fragmenting the chain. Whether one such pair
among forty true ones folds the *reconstruction* — the pipeline's own robust averaging sees the
loop the filter cannot — is measured (§5.7), not assumed; the tools for the next step (a second
graph pass, an ordering cue) are chosen from that measurement.

Sparse graphs are the other regime: on this branch's video captures `r` is 0.21-0.33, the ceiling
0.7 or below, and connectivity decides how far below it. The ceiling keeps the paper's generic
behaviour on a dense healthy orbit (Truck: `m = 0.6` removes about two thirds of the pairs, which
the paper reports as harmless sparsification and this branch has not measured; §5.3).

This is cheap. The scores do not depend on `m`, so one `ComputeTripletScores` call serves the whole
search; the largest component only grows as `tau` falls, so the strictest passing threshold is a
binary search over the distinct scores below the ceiling — `O(log |E|)` union-find passes.

`minScore` is the paper's `m`, and with the search on it is the ceiling. `--triplet-auto-tau`,
default **true**; when false, `tau(m)` is applied as given, the paper's behaviour.

### 3.3 A second cue, measured before it is trusted

The one fusion measurement available is unambiguous: on 4837 shared pairs, two independent cues
scored 0.8751 and 0.8469 alone, correlated at Pearson r = 0.353, and **their mean scored 0.9161** —
better than either. Independence is what pays, so a second cue is worth more than a better single
cue.

**Cue 1 — bipartite local clustering coefficient** (Wilson & Snavely, ICCV 2013). For an edge
`(i,j)`, take `A = N(i) \ {j}` and `B = N(j) \ {i}` in the view graph, and score the edge by how
densely `A` and `B` are cross-connected: the fraction of pairs `(a,b) in A x B`, `a != b`, that are
themselves edges. A true edge sits inside a well-connected local neighbourhood; a doppelganger edge
joins two neighbourhoods that share nothing but the false edge. Graph-only: no matches, no tracks,
no descriptors, no model. This is why it lands first.

**An edge whose neighbourhoods admit no cross pair is unscored, not zero.** Those are different
claims, and conflating them is the same mistake §3.1 removes from the triplet filter. On a barbell —
two triangles joined by one bridge — the bridge scores 0.0 across four cross pairs, none of which is
an edge, and that is damning. But a triangle edge `(0,1)` has `A = B = {2}`, whose only ordered pair
is the excluded self-pair, so there is nothing to measure at all; scoring it 0.0 would make a good
edge indistinguishable from the doppelganger. It carries `-1`, the sentinel `ComputeTripletScores`
already uses and `pairs.csv` already writes as an empty cell. This also keeps the offline fusion
honest: both cues mark "no evidence" the same way, so an unmeasurable edge is dropped from an
average rather than counted as maximally suspicious.

**Cue 2 — ambiguity-aware track cue** (Kataria et al.). This one needs tracks, and at filter time
there are none: `FilterPairsByTriplets` runs at `Scene.cpp:697`, inside `Reconstruct`, before any
triangulation. Computing it means building a provisional track set by union-find over the stored
matches — affordable, but its cost and its correctness both deserve their own measurement rather
than riding in on the back of a graph-only cue. It is therefore a second, separate step, delivered
after Cue 1, not dropped.

**Neither cue gates an edge in this round.** Both are exported as columns of `pairs.csv` beside
`TripletScore`, and evaluated offline: AUC alone, AUC fused with `TripletScore` (mean of the two,
the fusion that is already measured to work), on the labelled scenes. A cue enters the filter only
after that measurement, in a later round. This generalises the constraint the reference campaign
already established for a single cue — no cue rejects an edge on its own — into the rule that a cue
rejects nothing until its independence has been measured.

### 3.4 Where the filter runs

The filter runs at `Scene.cpp:697`, in `Reconstruct`. `ViewGraphCalibrator` runs at
`Scene.cpp:639`, inside `MatchPairs`. So the calibrator solves focal lengths over the **unfiltered**
graph, doppelganger edges included, and the filter only sees the graph afterwards.

Removing those edges first should hand the calibrator a cleaner graph. This is untested, and it is a
*measurement* (§5.4) rather than a feature: the arm runs the filter in both positions and reports
the calibrated focals and the registration that follows.

Running that A/B does need the call moved, so the "before" arm is built from a **throwaway local
edit, never a configuration switch**. Where the filter runs is a property of the pipeline, not a
user's choice, and shipping both positions behind a flag would be exactly the kind of parallel route
this branch does not take. Whichever position the measurement supports becomes the only one.

## 4. What changes

| File | Change |
|---|---|
| `libs/SFM/ViewGraphTriplets.h` | `TripletFilterConfig::autoTau`; `minScore` documented as the strictness the sweep starts from; the two header comments corrected for §3.1; declaration of the survivor-graph evaluator and of the bipartite cue |
| `libs/SFM/ViewGraphTriplets.cpp` | §3.1 removal rule; §3.2 sweep; §3.3 Cue 1; corrected log line |
| `libs/SFM/PairsMatcher.cpp` | `ExportPairsCSV` gains the Cue 1 column beside `TripletScore` |
| `apps/CreateStructure/CreateStructure.cpp` | `--triplet-auto-tau`; corrected `--filter-triplets` and `--triplet-min-score` help |
| `libs/SFM/PythonWrapper.cpp` | `auto_tau` on the config; the cue exposed beside `compute_triplet_scores` |
| `apps/Tests/TestsSFM.cpp` | `TripletFilterTest`'s path-graph assertion flipped; sweep tests; cue test |
| `docs/design/TripletDisambiguation.md` | the removal rule, the threshold, the second cue, and which follow-ups this closes |

## 5. Measurement

Every number below is controller work: it runs the pipeline, which no implementer does.

**5.1 §3.1 offline, before it ships.** Simulate both removal rules on every `pairs.csv` this branch
has recorded. **Done**, with `triplet_replay.py`, and it reproduces on this branch's own graphs —
largest component / images below degree 2 at `m = 0.6`, paper's rule against the shipped one:

| arm | unfiltered | paper's rule | keep unscored |
|---|---|---|---|
| 8d2f4877 sift | 203/3 | 187/21 | 190/17 |
| 8d2f4877 onepass | 209/5 | 205/11 | **209/5** |
| 8d2f4877 capdensity | 209/5 | 205/12 | **209/5** |
| 38004114 sift | 247/12 | **122**/113 | **220**/23 |
| 38004114 onepass | 305/1 | 258/53 | **305/1** |

On the one-pass graphs the shipped rule restores the unfiltered component exactly while still
removing weak scored edges. On the 38004114 sift graph the paper's rule costs 125 of 247 images;
keeping the unscored edges recovers 98 of them.

**5.2 The threshold rule, replayed.** **Done twice.** The first replay, on this branch's own video
captures, produced a three-bar rule (fragment nothing, strand nobody, remove under 20 %). The
second, on the ambiguous-scene datasets the filter exists for, showed that rule standing down on
every one of them and replaced it with the connectivity-driven threshold of §3.2; the table there is
its measured behaviour. The lesson is recorded so it is not relearned: a rule fitted on graphs
without doppelgangers cannot be validated on graphs without doppelgangers.

**5.7 The ambiguous-scene campaign.** `~/virginia/datasets/Disambiguation/run_disambig_campaign.sh`
runs the base and triplet arms over the yan2017 sets (exhaustive) and the sparse ToH and
heinly2014/indoor sets (vocabulary matching, 50 pairs per image); `rule_replay.py` replays threshold
rules on an arm's `pairs.csv`, `collapse_eval.py` scores a sequence's poses for folding, and
`plot_cameras.py` draws them. The reference numbers are the paper's Table 3 (cameras in the
disambiguated reconstruction: books 9/21, cereal 7/25, cup failed, desk 12/31, oats 9/23, street
19/19, ToH 338/338, indoor 42/152) and, for the heinly2014 sets, Doppelgangers++ Table 2. The bar
is a reconstruction that does not fold with at least as many cameras.

**5.3 The outdoor check.** **Done, and it is what found the missing third bar.** Scene type
dominates difficulty: outdoor object orbits essentially never fire (fraction of pairs below 0.5:
Truck 0.005, Barn 0.040) while indoor captures fire on 41-77 %, so a rule fitted indoors is untested
exactly where it is most likely to misfire — and it misfired. On Truck both removal rules and every
threshold leave the graph at 251/0, so connectivity could never object, and the rule happily removed
first 92 % and then 67 % of it. `Courthouse` remains unrun and is the next outdoor arm.

What still needs a *reconstruction* rather than a replay: whether the 17 % the corrected rule removes
from Truck's one-pass graph costs anything. Connectivity says no; only pose error can say.

**5.4 Filter placement.** §3.4, both positions, reporting calibrated focals and registration.

**5.5 The overlap-gate re-test.** The confident-overlap gate is a clear win on `f7dbf861` (308 vs
309 registered, rotation mean 0.899 vs 0.988, 157 vs 257 Ceres failures, matching 389 s vs 588 s)
but registers 124 of 377 on `32265651` against the ungated 146. The filter later fixed the real
cause on that capture, so the standalone verdict is stale. Re-test on top of the filter.

**Caveat that is part of the test:** the one-pass dense verdict may subsume the overlap gate
entirely — both reject pairs on warp-derived evidence. The re-test must state whether the overlap
clause still adds anything now the dense verdict exists, and if it does not, **recommend removing
it** rather than keeping two gates that do one job.

**5.6 Promotion.** No default flips in this round. The recorded rule stands: no sift capture loses
more than 2 % of its registered images; none worsens its median rotation error by more than 5 %;
and the dense repetitive arm keeps its component win. It is a conjunction and it is re-run, not
relaxed.

## 6. Risks

- **The connectivity-driven threshold reconnects through a doppelganger when the true junction is
  weaker.** Oats and cereal, §3.2. The filter then leaves one or a few false pairs among tens of
  true ones, where the unfiltered graph had a hundred; whether the reconstruction survives that is
  the first thing §5.7 measures.
- **On a dense healthy graph the ceiling removes most of the pairs.** That is the paper's own
  sparsification and it is opt-in, but this branch has not measured its cost in pose accuracy.
  The promotion conjunction is the gate, and it is unchanged.
- **§3.1 could give back the win.** A less aggressive filter removes fewer doppelgangers. The
  measured prediction is that the win case moves 376 → 377, but it is a prediction from one
  reference; §5.1 checks it on this branch's own graphs before the code ships.
- **Cue 1 is O(|N(i)| x |N(j)|) per edge.** On a dense video graph the neighbourhoods are large.
  The implementation must bound this — sorted adjacency intersection, and the cue reported as a
  per-edge cost in the first measurement — or a cue meant to be cheap becomes the expensive part.
- **A provisional track build for Cue 2 is not free and not obviously correct.** Splitting it into
  its own step is the mitigation; it does not eliminate the risk.
