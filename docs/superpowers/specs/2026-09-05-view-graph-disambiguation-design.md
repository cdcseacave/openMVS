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

### 3.2 The threshold comes from the survivor graph, not from a constant

**Rule A.** Score once. Sweep `m` downward over a fixed ladder. At each candidate, evaluate the
survivor graph the filter *would* produce and accept the candidate only if both hold:

- `keptLCC >= 0.99 x LCC(unfiltered)` — the filter may not fragment the reconstruction;
- `lowDegree(survivor) <= lowDegree(unfiltered) + 0.01 x |V|` — it may not strand images.

**Both bars are relative to the unfiltered graph, and the second one has to be.** Written as an
absolute "under 1 % of nodes below degree 2", the rule is unreachable on real captures: the
unfiltered lidar one-pass graph already has 5 images below degree 2 out of 209, against an absolute
bar of 2, so every candidate fails — *including* the one that removes nothing. Replaying the ladder
on this branch's own recorded graphs showed exactly that, with the survivor graph at `m = 0.6`
identical to the unfiltered graph and the rule still refusing it. What the test exists to prevent is
the *filter* stranding images, so it bounds the increase.

Take the **strictest** (largest `m`) accepted candidate. If none is accepted, the filter removes
nothing and says so: a graph where no threshold is safe is a graph this filter has no business
touching.

This is cheap. The scores do not depend on `m` — only `tau` does, through Eqn. 3 on `G_LCT` — so
one `ComputeTripletScores` call serves the whole sweep, and each candidate costs one union-find
pass over the edges plus a degree count. The ladder is `m = 0.95` down to the configured
`minScore` in steps of `0.05`.

`minScore` therefore changes meaning: it stops being *the* threshold and becomes the **floor of the
sweep**. That is a rename of intent, not of the field, and the CLI help says so.

`--triplet-auto-tau`, default **true**. When false, the single configured `minScore` is used, which
is the paper's behaviour and what every recorded measurement used. Auto is the default because the
manual value is the part all three measurements say is unreliable.

**Honesty about the fit:** Rule A is 6/6 correct on the measured arms, firing only on
`32265651`/`roma2gate` at `tau 0.70` and standing down on the five arms where the filter loses. But
it is fitted on those same arms with a single positive, and it was fitted *before* §3.1 existed.
With triangle-less edges kept, more captures satisfy the connectivity test, so the sweep will start
filtering where it currently stands down. Rule A must therefore be re-swept on top of §3.1
(§5.2) and checked outdoors (§5.3) before it is trusted, and it may not flip any default on its own.

### 3.3 A second cue, measured before it is trusted

The one fusion measurement available is unambiguous: on 4837 shared pairs, two independent cues
scored 0.8751 and 0.8469 alone, correlated at Pearson r = 0.353, and **their mean scored 0.9161** —
better than either. Independence is what pays, so a second cue is worth more than a better single
cue.

**Cue 1 — bipartite local clustering coefficient** (Wilson & Snavely, ICCV 2013). For an edge
`(i,j)`, take `A = N(i) \ {j}` and `B = N(j) \ {i}` in the view graph, and score the edge by how
densely `A` and `B` are cross-connected: the fraction of pairs `(a,b) in A x B` that are themselves
edges. A true edge sits inside a well-connected local neighbourhood; a doppelganger edge joins two
neighbourhoods that share nothing but the false edge. Graph-only: no matches, no tracks, no
descriptors, no model. This is why it lands first.

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

Removing those edges first should hand the calibrator a cleaner graph. This is untested, and it is
a *measurement* in this spec (§5.4), not a code change: the arm runs the filter in both positions
and reports the calibrated focals and the registration that follows. A move lands only if the
measurement supports it.

## 4. What changes

| File | Change |
|---|---|
| `libs/SFM/ViewGraphTriplets.h` | `TripletFilterConfig::autoTau`; `minScore` documented as the sweep floor; the two header comments corrected for §3.1; declaration of the survivor-graph evaluator and of the bipartite cue |
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

**5.2 Rule A re-swept on top of §3.1.** **Done**, and it corrected the rule (the relative bar
above). With that correction the sweep fires on four of the five arms — `m = 0.90`, `0.65`, `0.15`
and `0.65` — and stands down on 8d2f4877 sift, whose graph loses 13 images of its largest component
at every threshold. Standing down there is the rule working, not failing.

**5.3 The outdoor check.** Scene type dominates difficulty: outdoor object orbits essentially never
fire (fraction of pairs below 0.5: Truck 0.005, Barn 0.040) while indoor captures fire on 41-77 %.
A rule fitted indoors is untested exactly where it is most likely to misfire, so `Truck` and
`Courthouse` are in the arm list and a regression there is a reportable result, not something to
tune away.

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

- **Rule A is fitted on seven arms with one positive.** §5.2 and §5.3 are the detectors; if the
  sweep starts firing where it used to stand down and loses images there, Rule A is wrong and the
  manual threshold stays.
- **§3.1 could give back the win.** A less aggressive filter removes fewer doppelgangers. The
  measured prediction is that the win case moves 376 → 377, but it is a prediction from one
  reference; §5.1 checks it on this branch's own graphs before the code ships.
- **Cue 1 is O(|N(i)| x |N(j)|) per edge.** On a dense video graph the neighbourhoods are large.
  The implementation must bound this — sorted adjacency intersection, and the cue reported as a
  per-edge cost in the first measurement — or a cue meant to be cheap becomes the expensive part.
- **A provisional track build for Cue 2 is not free and not obviously correct.** Splitting it into
  its own step is the mitigation; it does not eliminate the risk.
