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
`tau` — joins every *piece* the ceiling leaves: a component of the survivor graph at the ceiling
that lies inside the unfiltered graph's largest component and holds at least 1 % of it (§3.7; on a
set of fewer than 101 images every component inside it is a piece). There is no other bar. If the
ceiling itself joins every piece, or its largest piece already holds a majority of the images the
pieces hold together (§3.8), it is applied as given; if nothing above the lowest score does, the
lowest score is chosen and nothing is removed.

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

### 3.5 The strength of an edge is its inlier count discounted by the inliers' coverage

The paper weighs an edge by `n_ij`, its epipolar inlier count. Measured on the small ambiguous
sets (§5.7), that number ranks the wrong edge first exactly where it matters: the junction between
the two halves of oats is `(6,7)` with 627 inliers, and the doppelganger `(6,21)` has 880; on cup
the chain link `(11,12)` has 358 and the half-turn doppelganger `(1,38)` has 443. §3.2's descent
then reconnects the graph through the doppelganger, and one such edge among ~40 true ones is
enough to fold the reconstruction — oats, cereal and cup folded on the first campaign; street,
books and desk did not. Nothing in the counts or the triangle structure separates the two cases:
replaying the descent as a sequence of merge events shows the bad merges indistinguishable from
the good ones in score, count and component size, and the cross-edge matrix between the two halves
shows why — a doppelganger band between two runs of a pan (forty 400-1000-inlier edges between
frames 3..6 and 13..22 on oats) is also what a genuine revisit produces.

What differs is *where* the inliers sit. A doppelganger's matches lie on the duplicated object and
nowhere else; a true adjacent pair's matches spread over the whole overlap. `(6,21)`: 880 inliers
on the oats can alone. `(6,7)`: 627 over the can, the box beside it and the table.

**Rule:** the strength of an edge is `s_ij = n_ij * c_ij`, where `c_ij` is the fraction of a
`gridSize x gridSize` grid over the image that the pair's track-forming matches occupy, taken as
the smaller of the two images' fractions (pinhole images bin on a uniform pixel grid, spherical
ones on equal-solid-angle cells). It is the same grid, binning and match set `ComputeIntrinsicWeight`
already uses for `weightSpatial`, computed by one shared function. The triplet score is
`q_ij = mean over triplets of s_ij / max s_kl` and two scene pairs on one image pair collapse onto
the stronger `s`. Nothing else changes: the threshold rule of §3.2 and the unscored rule of §3.1
apply to the discounted scores as they did to the counts.

Not the angle term. `weightSpatial` multiplies the coverage by a ray-angle weight that goes to
zero below 1.5 degrees of baseline. That measures triangulation conditioning, not overlap, and
replayed on ToH it drops adjacent pairs with 6000-9000 inliers because consecutive frames of a
walk have almost no baseline. Coverage alone keeps them.

Evidence, from replaying the exported graphs of the first campaign (`triplet_replay.py`; its
count mode reproduces the run's `TripletScore` column to four decimals and the same unscored set):

| set | count: doppelganger kept | coverage: doppelganger kept | true junction under coverage |
|---|---|---|---|
| street | none | none | kept |
| books | none | none | kept |
| desk | none — the revisit `21..24 x 30` is kept, correctly | none | kept |
| oats | `(6,21)` 880 | **none** | `(6,7)` kept |
| cup | `(1,38)` 443 | **none** | ring closed through gap-2 edges |
| cereal | `(7,16)` 862, `(7,15)` 917, `(5,13)` 1390 | the same | `(13,14)` 320 dropped |

The outcome is the same at coverage powers 0.5, 1 and 2; power 1 ships and there is no parameter.
On ToH's sparse graph (`r` = 0.22) the ceiling applies as given in both modes; coverage keeps 2354
of 8449 pairs against 2659, with fewer far edges (361 against 545 at a frame gap of 30 or more) and
three adjacent pairs dropped that the gap-2 edges bridge.

**The frontier after this cue is cereal.** Its true junction `(13,14)` is weak in count (320) and
in coverage (a mug close-up against a view of the box), while the doppelganger `(7,16)` is a box
filling both frames. No per-pair statistic of the matches separates them, and the paper oversplits
cereal too (7 cameras).

`pairs.csv` gains a `Coverage` column, so the discount can be replayed offline against the raw
count; `TripletScore` is the discounted score from now on.

### 3.6 A triangle of three inlier-deficient pairs is no evidence

Coverage (§3.5) tells a doppelganger from a true junction when the duplicated object is part of
the frame. On ToH the whole frame is the duplicated object: the Temple of Heaven is round and
three-fold symmetric, and every frame of the 338-image orbit has look-alikes a third and two
thirds of a turn away. Both arms of the first campaign fold the orbit the same way (`fold_map.py`:
frames 130-170 land on 240-280 and 200-220 on 310-330, a frame gap of 110 = one third of the
orbit), and the look-alike pairs have the coverage of the true ones (0.47 against 0.49; raising the
coverage power shrinks both bands alike). Nothing in the triangles separates them either, because
the look-alike pairs form triangles among *themselves* — three copies of one facade, each pair as
strong as the other two — and such a triangle scores each of its edges at 1. On the vocab-50 graph
that gives the gap-110 band scores of 0.55-0.80 against a ceiling of 0.689, and the reconstruction
folds through it.

What separates them is a deficit. Two-view geometry reads a look-alike pair as a near-duplicate
viewpoint: its median ray angle is 2-3 degrees, the angle of a consecutive pair of the walk. A
consecutive pair at that angle carries 7500 inliers; the look-alike carries 800-1500, because
only the repeated structure matches and nothing else does — the details that differ between the
copies, the background, the ground. That is the "missing correspondences" cue of the
disambiguation literature (Jiang et al. 2012, Heinly et al. 2014), measured on the pair's own
matches instead of on a reconstruction. A genuine wide-baseline pair also has few inliers, but at
a wide angle. On ToH (`gap_stats.py`, vocab-50 graph):

| pairs | median ray angle | median inliers | yield p10 / median / p90 |
|---|---|---|---|
| frame gap 1 | 1.5° | 7494 | 0.97 / 1.00 / 1.00 |
| gap 5 | 7.5° | 3563 | 0.87 / 0.96 / 1.00 |
| gap 10 | 15.8° | 1704 | 0.81 / 0.90 / 1.00 |
| look-alike bands (gaps 25, 30-59, 60-89, 90-119, 150-179, 210-239) | 1.9-3.4° | 800-1500 | 0.07-0.18 / 0.13-0.24 / 0.15-0.32 |
| gap 330 and more (the orbit's closure) | 3.7° | 3614 | 0.60 / 0.65 / 0.75 |

**Yield.** For a pair, `u_ij = n_ij / min(K_i, K_j)`, with `K_i` the inlier count of image i's
strongest pair: the fraction of what these two images can deliver that the pair delivered. The
graph's own envelope `H(θ)` is the 90th percentile of `u` over the pairs in each 1-degree bin of
median ray angle (bins holding at least five pairs), made non-increasing in θ by a suffix maximum;
a pair whose bin holds no envelope takes the nearest populated bin above it, and above the highest
populated bin the envelope stays at that bin's value. The yield is `y_ij = min(1, u_ij / H(θ_ij))`:
how much of what a pair at this angle normally delivers in this graph the pair delivered. It is
scale-free and carries no constant of the scene or the matcher — the envelope is what this graph's
own pairs do — and a graph with fewer than five pairs in every bin has no envelope and every yield
is 1. The three constants (1-degree bins, five pairs, the 90th percentile) live in the scorer, not
in the configuration; 75, 90 and 95 replay identically, and so does an `exp(-θ/10°)` envelope in
place of the measured one.

**Rule.** A triangle whose three edges all yield less than `minYield` = 0.4 is a doppelganger
triangle — look-alike copies vouching for one another — and contributes zero to the score sum of
each of its edges while still counting in the divisor. Everything else is as it was: the strength
`s_ij = n_ij c_ij`, the mean over triangles, G_LCT, the threshold of §3.2, the unscored rule of §3.1.
The yield never scales a strength and never removes a pair on its own; `minYield` = 0 switches the
rule off.

Why not the two obvious forms. Multiplying the yield into the strength loses oats — its
doppelganger `(6,21)` yields 0.79 against the junction's 0.54, because on the small sets the
doppelganger *is* the strong pair, which is what coverage handles — and cannot touch ToH, where
every edge of a look-alike triangle is discounted alike and the triangle still scores 1; it also
drops ToH's closure along with the look-alikes. Removing every pair below a yield gate fixes ToH
but halves cup's complete graph, lowers its ceiling from 0.994 to 0.887 and lets cup's half-turn
band `(0-1, 36-38)` in. The triangle rule touches neither: every small set replays identically to
§3.5, and ToH keeps zero edges in every look-alike band, 50 of the 153 closure edges (gap 300 and
more) and the consecutive pairs exactly as before, with the ceiling applied as given.

Sensitivity: `minYield` 0.35 and 0.45 replay identically to 0.4 on all seven sets; 0.6 lets oats'
`(6,21)` back in, because the true triangles of the small sets start yielding below the bar.

Cost if wrong: a scene whose true pairs all yield poorly at small angles — a day/night mix, a set
whose only near-duplicate viewpoints are look-alikes — has fewer triangles to score with and falls
back to the descent of §3.2, which is where it was before this rule.

**The frontier after this cue is still cereal**, and now also the first real measurement on the
heinly2014 collections, whose images are not a walk: the envelope there is set by the tourists'
near-duplicate photos, which every popular viewpoint has.

**A ray angle of zero is "never measured".** `ImagePair::meanRayAngle` is 0 on a pair whose
relative pose was never decomposed, not a measurement of a zero baseline; such a pair carries no
evidence of a deficit and takes yield 1 — outside the envelope, never a doppelganger triangle's
member on its own account. Binning it at zero degrees would put it in the bin whose envelope is
highest and read every such pair as inlier-deficient (Radcliffe matched exhaustively: 311 of
20,306 scored pairs).

### 3.7 The descent does not chase stragglers

§3.2's descent was written against the small sets, where the ceiling shatters a complete graph
into pieces of two to nine frames and reassembling them *is* the job. On the internet collections
the ceiling leaves something else (`triplet_replay.py --bridges` on the base graphs, §3.6's rules):

| set | images | ceiling | components at the ceiling | descent under the 99 % rule |
|---|---|---|---|---|
| church | 277 | 0.723 | 257, 4, 2, 2, 1, 1, ... (16) | to 0.426, 3273 pairs kept against ~2450 at the ceiling, for 20 images in pieces of 1-4 |
| big_ben | 402 | 0.682 | 392, 2, 1, 1, 1, 1, ... (10) | to 0.517 for 10 images in pieces of 1-2 |
| radcliffe | 282 | 0.734 | 218, 47, 2, 2, 2, 1, ... (16) | to 0.293 — the 47-piece joins at 0.559 through one 63-inlier pair, the rest of the way is for twelve stragglers |
| indoor | 152 | 0.808 | 106, 46 | to 0.774, one bridging pair of 471 inliers, coverage 0.68, yield 0.97 |
| street ... cup | 19-64 | 0.98-0.99 | 5, 4, 3, 3, 2, 2 / 17, 9, 8, 7, 7, 5, ... | as §3.2 |

A one- or two-image component the ceiling leaves behind is not an over-split: it is an image the
graph vouches for through a single weak pair (23-63 inliers on radcliffe's bridges). Descending
to fetch it admits *every* edge between the ceiling and that pair's score — hundreds of pairs on
church, doppelgangers among them — to gain one image, which the paper simply drops and which
resection can still register if its unscored pairs carry it.

**Rule.** A *piece* is a component of the survivor graph at the ceiling that lies inside the
unfiltered graph's largest component and holds at least 1 % of it (`ceil(n0 / 100)` images, so
every component inside it counts on a set of fewer than 101 images); anything smaller is a
straggler, and so is anything outside that component. The threshold is the strictest one at or
below the ceiling whose survivor graph joins every piece into one component — its largest component
holds at least the images the pieces hold together. There is no percentage any more: the 99 % of
§3.2 was a straggler allowance that fitted the small sets (where it rounds to 100 %) and is too
small on the large ones. Stragglers are neither chased nor removed: they keep whatever unscored
pairs they have, and a scored bridge that happens to sit above the chosen threshold keeps them
attached.

The restriction to the unfiltered graph's largest component is not a nicety. A component of the
unfiltered graph that shares no pair with the rest — a verified pair sitting in no triangle with
anything else — cannot be joined to anything by any threshold, since no score admits an edge that
does not exist. Counting it as a piece sets a target no candidate can reach and the search falls
through to the loosest one, keeping every scored pair; in the paper's terms such an island is
simply another model, and the filter treats it as it treats a straggler.

On the small sets this changes nothing: `ceil(n0 / 100)` = 1 there, every component is a piece and
"every piece joined" is what 99 % rounded to. On church and big_ben the ceiling now applies as
given (church joins its 4-piece at 0.652); on radcliffe the descent stops at 0.559, where the
47-image piece joins; on indoor the two halves still join through their one strong bridge — whether
that pair is a true junction or a look-alike is the reconstruction's answer (§5.7), not the filter's.

Cost if wrong: a genuine sub-scene of fewer than 1 % of the images (a detail cluster of a large
collection) that only connects below the ceiling is left as its own component instead of being
joined — the paper's behaviour, and a separate model rather than a wrong merge.

**The bar is the pieces, not a count.** "Joins every piece" means the pieces share one
component of the survivor graph, tested on one representative image per piece; it is not the
largest component reaching the number of images the pieces hold. The two differ when stragglers
accrete: as the threshold falls, stragglers attach to a piece before the last piece joins, and a
count of nodes is satisfied while a piece is still apart (two chains of a hundred images and a
hundred and ten stragglers hung on the first: the count is met at the stragglers' score with the
second chain apart; the pieces share a root only at the bridge's). The piece test is monotone in
the threshold, so the search below the ceiling is unchanged.

**A ceiling that leaves no piece.** When every component the ceiling leaves is smaller than the
floor — a large collection shattered into pairs — there is nothing to join and nothing to seed
in; that is the graph that most needs repair, not the one to leave alone. Then every component of
the unfiltered largest component is a piece (the floor becomes one image), the descent joins them
as on a small set, and the seed views are the largest of them.

### 3.8 The descent only repairs a shattered ceiling

§3.2's descent was written for graphs the ceiling shatters. On the small sets, and on any
collection matched exhaustively, `d_max/|V|` is close to 1, the ceiling sits at 0.94-0.995, and
it leaves pieces of two to forty images out of twenty to a hundred and fifty — fragments of one
camera path or one building, which reassembling is the whole job. On the church matched
exhaustively the same kind of ceiling (0.942) leaves something else: two pieces of 140 and 85
images, the south facade with the canal views and the north facade, separated the way
Doppelgangers++ separates them (157+106), and joined at 0.892 by one pair of 253 inliers. The
descent joined them; only the reconstruction's failure to cross that pair kept the model
unfolded. A ceiling whose largest piece already holds most of the images has done the paper's
job: what hangs below it is a straggler or the other face of a symmetric building, and no
threshold can tell which.

| graph | ceiling | largest piece / images in pieces |
|---|---|---|
| books, cereal, cup, desk, oats, street (exhaustive) | 0.979-0.994 | 7/21, 5/25, 17/64, 8/31, 9/23, 5/19 |
| indoor (exhaustive) | 0.995 | 33/151 |
| church (exhaustive) | 0.942 | 140/244 |
| radcliffe (exhaustive) | 0.943 | 120/254 — no majority: the descent joins every piece (§3.9) |
| brandenburg (exhaustive) | 0.973 | 124/144 |
| ToH, big_ben, the vocab-50 heinly graphs | 0.68-0.81 | one piece |

The pieces are those of the runs' own scores (the `TripletScore` column of their pair export). The
offline replay approximates the strength from the match count rather than the filtered inlier
count, and on these near-complete graphs its pieces differ from the run's by tens of images
(Radcliffe: 176 against 120+52); every number a rule is argued from is the run's.

**Rule.** The descent below the ceiling happens only when the ceiling shattered the graph: when
no piece holds a strict majority of the images the pieces hold together. If the largest piece
holds more than half of them, the ceiling is applied as given, and the smaller pieces stay apart
as the paper leaves them, like §3.7's stragglers. Otherwise the threshold is the strictest one
joining every piece, as §3.2 and §3.7 say. The bar is a strict majority so that a graph the
ceiling cuts in two equal halves (the pan of §5, three and three) is still repaired.

Cost if wrong: a collection whose ceiling leaves a majority piece and a genuine second sub-scene
joined only below the ceiling gets that sub-scene as a separate model — the paper's own
behaviour — and never a merge through a doppelganger.

### 3.9 The reconstruction seeds in the largest piece the ceiling leaves

The filter's output only counts once it is reconstructed, and the reconstruction begins with a
star around one reference view: `StarInitializer::SelectReferenceView` takes the image whose
valid pairs carry the most weighted inliers, and the star, then resection, grow from there. That
measure favours the densest cluster of look-alike views: on Radcliffe matched exhaustively the
images of the 45-image piece carry a median of 21,000 weighted inliers against 11,500 in the
120-image piece, so the seed lands in the 45-piece. The resection then does the paper's step 11
of its own accord — it refuses the doppelganger bridges the descent let through: the church run
grew from its 85-piece to 93 images and never crossed the 253-inlier pair into the 140-piece,
Radcliffe grew from the 45-piece to 45+19+4 and never crossed into the 120-piece — but it can
only refuse, not choose: the seed's side becomes the model, and that side is whichever holds the
most near-duplicate views, the worst criterion there is for a symmetric building. The Radcliffe
run with the ceiling applied as given, in whose graph the heaviest image happened to sit in the
121-piece, registered exactly that piece.

Stopping the descent at a majority instead was tried on the runs' own scores and rejected: it
gives Radcliffe 174 (the paper's 177) and the church 140, and halves every small set (books 14/21,
cereal 13/25, cup 34/64, desk 19/31, oats 12/23, street 12/19) and the indoor loop (106/152),
because the fragments of one camera path join at the same scores the doppelganger pieces do
(Radcliffe's 45-piece at 0.876, books' last fragment at 0.875). The scores cannot tell them apart;
the resection can, given the right start.

**Rule.** The filter reports the images of the largest piece the ceiling leaves — the pieces of
§3.7, at the ceiling and before any descent; between equally large pieces, the one holding the
lowest image index — and the reconstruction chooses its reference view among them, the heaviest by
the same measure as before. When the filter is off, or none of the reported images has a valid
pair, every image is a candidate, as before. With clustering, a sub-scene's candidates are the
reported images it holds, and a sub-scene holding none chooses among all its own. The descent is
unchanged: the pieces are still joined, and the resection still decides which joins it crosses.

**The star must be able to grow.** The reference view is the heaviest candidate with at least
`minViews − 1` valid pairs — the smallest star the initializer accepts, its centre excluded; three
by default. On Street matched exhaustively the descent keeps 20 of 171 pairs, a chain; the largest
ceiling piece holds five images and its heaviest image has two pairs, a star of two arms, which the
initializer refuses — and the run reconstructed nothing where the heaviest image overall, with
three, had given all 19. A candidate with too few pairs is skipped; when no seed view qualifies the
choice falls to every image, and when no image qualifies the heaviest image is chosen and the
initializer reports the shortfall as it always has. The bar is the initializer's own minimum, not
a new parameter.

Expected on the exhaustive runs: Radcliffe seeds in the 120-piece, crosses the 0.928 bridges into
the 52-piece (172 images; the paper 177, Doppelgangers++ 186) and refuses the 45-piece's; the
church seeds in the 140-piece the ceiling already cut it to (the paper 136, Doppelgangers++ 157);
the small sets and the indoor loop seed in one fragment of their single path and grow through it
as before.

Cost if wrong: on a collection whose largest ceiling piece is the wrong face of the building —
more views of the doppelganger than of the rest — the model is that face, the paper's own
outcome, and no worse than today's arbitrary side.

### 3.10 The minimum score is 0.75

The church matched exhaustively four times gives four graphs whose facades split at the ceiling
in two and merge in the other two: the north-south pairs the matcher happens to verify score
between 0.89 and 0.96, and `tau(0.6)` on these graphs is 0.942, inside that band. Whether the
model is one facade (135 images, the paper's 136) or both folded (223) is decided by the
matcher's run-to-run variation, not by the rules. On the same four graphs, with the shipped
majority rule:

| m | ceiling | church, largest piece / second (four graphs) | radcliffe | brandenburg | big_ben (retrieval graph) |
|---|---|---|---|---|---|
| 0.6 | 0.942 | 139/85, 140/85, 227 merged, 225 merged | 119 + 54 + 44 + 19 + 15, descent to 0.842 | 125, folded | 391 at the ceiling |
| 0.7 | 0.957 | 134-137 / 80-83, all four split | 113 + 53 + 44 + 19 + 13, descent to 0.842 | 113 | 382 |
| 0.75 | 0.964 | 131-134 / 80-83, all four split | 109 + 51 + 43 + 18 + 13, descent to 0.842 | 102 | 374 |
| 0.8 | 0.971 | 127-131 / 56-58 + 23-24, the south facade fragmenting | 101 + 51 + 42 + 17 + 12 | 69 + 13 + ..., descent | 366 |
| 0.9 | 0.986 | 96-99 / 49-50 / 19, no majority: the descent merges them | 84 + 46 + ... | 40 + ..., descent | 181 + 168 |

The small sets, ToH and the indoor loop are untouched by `m` in this range: their ceilings shatter
the graph whatever `m` is and the descent, which joins every piece, reaches the same threshold.
Radcliffe's ceiling never leaves a majority piece and the descent reaches the same threshold; only
its seed piece shrinks a little.

The sweep argued for 0.75 as the default, and it was shipped and run: the church split in both
further matchings (130 and 129 images, one-sided). Then Big Ben matched exhaustively (r 0.664)
answered the other way: at 0.75 the ceiling 0.916 keeps 2,299 scored pairs, the near-duplicate
cliques thinly joined, resection registers 295 images and the reconstruction invalidates 244 of
them for want of well-conditioned tracks, 147 remain; at 0.6 the ceiling 0.866 keeps 3,281 and
371 of 403 register (the paper's 379). No single `m` serves both a two-faced building and a tower
whose graph is one face: §3.11 chooses between the two ceilings from what they leave.

**Rule.** The default minimum score `m` stays at the paper's generic 0.6. §3.11 says when the
ceiling at 0.75 is used instead.

### 3.11 The ceiling looks for the second face

Two ceilings are evaluated: `tau(m)` at the default `m = 0.6`, and `tau(m₂)` at `m₂ = 0.75`. If the
graph the higher ceiling leaves is *two-faced* — its largest piece holds a strict majority of the
images in pieces (§3.8) and its second-largest piece holds at least a third of the largest — the
higher ceiling is the ceiling; otherwise the lower one is, as the paper has it. Everything after
(§3.7's descent, §3.8's majority, §3.9's seed) applies to the chosen ceiling unchanged.

| graph | at 0.6 | at 0.75 | two-faced? | ceiling used | result |
|---|---|---|---|---|---|
| church (seven matchings) | 139/85 or 227 merged | 130-135 / 80-83 | yes (second 60 % of the largest) | 0.75 | 129-135, one-sided |
| radcliffe | 119 + 54 + 44 + 19 + 15, no majority | 105-117 + 47-53 + ..., no majority | no | 0.6 | descent to 0.84, 175-181 |
| brandenburg | 125 | 102 + 7 + 6 + ... | no (second 7) | 0.6 | 125, folded |
| big_ben exhaustive | 387 | 376 | no (one piece) | 0.6 | 371 |
| big_ben retrieval | 391 | 374 | no | 0.6 | 375 |
| arc exhaustive | 363 + 29 | 305 + 42 + 9 + 6 + 6 | no (second 42 of 305) | 0.6 | the 363-piece |
| indoor, ToH, the small sets | shattered | shattered, no majority | no | 0.6 | the descent, unchanged |

The third is a bar, like the 1 % floor and the majority: the other face of a two-faced building
holds a substantial share of the views (the church's north facade 60 % of the south, Radcliffe's
look-alike pieces about half of the largest), and a night cluster or a detail cluster hanging off
the largest piece holds a few percent.

Cost if wrong: a collection with a genuine second sub-scene holding a third of the largest piece,
joined to it only between the two ceilings, gets that sub-scene as its own model — the paper's own
behaviour, and a separate model rather than a merge; and one more constant beside the floor and the
majority.

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
