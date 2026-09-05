# Camera-Triplet View-Graph Disambiguation

## Overview

An optional, dependency-free pre-reconstruction filter that removes wrong image pairs — repeated
structure ("doppelgangers"), retrieval false positives — from the matched view graph using nothing
but the graph itself and one strength per edge: the epipolar inlier count discounted by the
fraction of the frame the inliers cover. A doppelganger's matches sit on the duplicated object
alone while a true adjacent pair's spread over the whole overlap, which is what tells apart a
doppelganger with more inliers than the true junction beside it. A triangle whose three pairs all
*yield* poorly — each reads as a near-duplicate viewpoint by its ray angle yet delivers a fraction
of the inliers such pairs deliver between these images — is look-alike copies vouching for one
another and carries no evidence. It reimplements
S. M. Manam and V. M. Govindu, *Leveraging Camera Triplets for Efficient and Accurate
Structure-from-Motion*, CVPR 2024, pp. 4959–4968 (Algorithm 1, Eqn. 3), from the paper alone — no
code from the MATLAB release or any port. `ViewGraphTriplets.{h,cpp}`; **off by default**.

## The algorithm

View graph `G = (V,E)`: nodes = images, edges = pairs with a stored two-view geometry and at least
one inlier, carrying the strength `s_ij = n_ij * c_ij`, with `n_ij = ImagePair::GetNumWeightedInliers()`
and `c_ij = ComputePairCoverage(...)` the fraction of the frame the inliers cover.

1. **Triplet graph.** The triangles of `G` are the nodes of the *triplet graph* `G_T`, two adjacent
   iff they share an edge of `G`. The edges taking part in its largest connected component form
   `G_LCT`; everything else — every edge in no triangle included — is **unscored**.
2. **Score.** `q^t_ij = s_ij / max_{(k,l) in t} s_kl` per triplet `t`; `q_ij` is its mean over the
   triplets of `G_LCT` containing `(i,j)`. A triplet whose three edges all yield less than
   `minYield` (`TripletFilterConfig::minYield`, 0.4, a configuration field without a command-line
   flag) contributes 0 to that mean: the yield of an edge is `n_ij / min(K_i, K_j)` (each `K` the
   image's strongest pair) against the graph's own 90th-percentile envelope of that ratio per
   degree of median ray angle, capped at 1.
3. **Threshold.** `tau = m·(1 − d_max/|V|) + d_max/|V|`, with `|V|` and `d_max` the node count and
   maximum degree **of `G_LCT`** (the paper says "of the graph"; the graph whose edges carry a score
   is `G_LCT`). `m` is the one user parameter — the default, 0.6, per the paper's generic/large-scale
   guidance; 0.9 highly ambiguous, 0.3 medium/small ambiguous.
4. **Selection.** A scored pair is removed iff its score is below the threshold; an **unscored pair
   is kept** — the paper's step 1 discards every edge outside `G_LCT`, but on the two labelled
   references those pairs are overwhelmingly true (426 of 490 on one capture, 415 of 441 on the
   other), so absence of evidence keeps a pair.
5. **The threshold below the ceiling** (`--triplet-auto-tau`, the default). `tau` above is Eqn. 3's
   value at `m`, but it is only a **ceiling**: the filter is never stricter than that value. A
   *piece* is a component of the ceiling's survivor graph that lies inside the unfiltered graph's
   largest component and holds at least 1% of it (so on a set under 101 images every such component
   is a piece); when the largest piece holds a strict majority of the images the pieces hold
   together, the ceiling is applied as given and the smaller pieces stay apart — a ceiling that
   keeps most of the graph together has done its job, and what hangs below it may be the other
   face of a symmetric building; otherwise the threshold actually used is the strictest one whose
   survivor graph joins every piece. Stragglers — smaller than a piece, or in a different component
   of the unfiltered graph — are neither chased nor removed: fetching one would admit every edge
   between the ceiling and the single weak pair that attaches it, for one image. The paper's step
   11, extracting the largest component of the filtered graph, is **not** applied — `SceneCluster`
   already selects components.
6. **The second face** (`--triplet-second-face-score`, 0.75, part of `--triplet-auto-tau`). Before
   any of the above, the ceiling at `m` 0.75 is tried first, and it is the ceiling when the graph it
   leaves is *two-faced* — its largest piece holds a strict majority of the images in pieces and its
   second-largest piece holds at least a third of the largest; otherwise the paper's ceiling at 0.6
   stands. Why: on a two-faced building the paper's ceiling sits among the scores of the pairs
   bridging the facades (the church: 0.942 at 0.6, bridges scoring 0.89-0.96, merged in two matchings
   of four; at 0.75 every one of seven graphs splits into the facades, 130-135 and 80-83 images),
   while on a building whose graph is one face the stricter ceiling cuts the graph so thin that the
   reconstruction discards most of what it registers (Big Ben: 147 of 403 images at 0.75, 371 at
   0.6, one piece either way); a second piece of a third is the bar because the other face of a
   two-faced building holds a substantial share of the views and a night or detail cluster hanging
   off the largest piece holds a few percent (Brandenburg: 7 of 102 at 0.75).
7. **Seeding.** The filter reports the images of the largest piece the ceiling leaves (ties to the
   piece holding the lowest image index), and `StarInitializer::SelectReferenceView` chooses the
   reconstruction's reference view among them: the heaviest by weighted inliers with at least
   `minViews - 1` valid pairs (three by default) among the seed views, else the heaviest so-qualified
   image among every image, else — when nothing qualifies anywhere — the heaviest image overall, so
   the caller can report the shortfall. Why: the resection refuses the doppelganger bridges the
   descent lets through but cannot choose the side it starts on, and the heaviest image overall sits
   in the densest cluster of look-alike views (Radcliffe: the 45-image piece, so the 120-image piece
   never registered; Street: the largest piece's heaviest image had two pairs, the star needs three,
   and the run reconstructed nothing).

*Why it catches what the existing cycle test cannot.* `ImagePair::weightTriplet`
(`PairsWeighting.cpp`) scores a pair by how many of its triangles close rotationally, and a
doppelganger's false edges are **mutually consistent** — the two near-identical façades form a block
whose cycles do close. The *inlier* asymmetry is what does not. Both scores are kept.

*Implementation.* Triangle enumeration by sorted-adjacency intersection over `i < j < k`, streamed
twice — once to union the three edges of every triplet (the components of `G_T`, carried on the
edges) and once to accumulate the scores — so the state is O(|E|), never O(#triplets). No external
solver, no Boost graph, and **serial**: a few ms on the densest capture here (377 images, 6241
pairs, 44 889 triplets). Duplicate pairs collapse onto one edge weighted by the strongest.

## Where it runs, and the flags

`Scene::Reconstruct` applies it **right after** the diagnostics export (`--export-pairs-csv` /
`--export-retrieval-csv`) and before `matchImagesOnly` returns, from both call sites (after
`MatchPairs`, and on the already-matched-`.sfm` early return). The CSV therefore always lists the
*whole* matched graph, each pair's score in a trailing `TripletScore` column (empty = unscored) of
the export's header
`ImageA,ImageB,NumMatches,Coverage,Weight,WeightSpatial,WeightConnectivity,WeightTriplet,MeanRayAngle,TripletScore`,
so a run can be re-scored offline from its own export; `ComputePairsWeights` re-runs afterwards, when
something was removed. Four lines from a Radcliffe run report everything:

```
Triplet filter: the ceiling at the second-face score 0.75 (0.967) leaves no second face (largest piece 110, second 52): the paper's ceiling stands
Triplet filter: tau 0.842, the strictest threshold that joins every piece (ceiling 0.948 at m 0.60, d_max/|V| 0.869); the ceiling leaves 5 pieces (components of at least 3 images) holding 254 images between them, and 28 stragglers; survivor graph keeps 268/282 images in its largest component, 18 below degree 2 (0 before), and 2843/20298 distinct image pairs
Triplet filter: kept 5567/23022 scene pairs (tau 0.842; 282 nodes, max degree 245; 796087 triplets in 1 components, 254099 doppelganger triplets gave no evidence; 17455 below tau removed, 2724 unscored kept); the reconstruction seeds in the largest piece the ceiling leaves (121 images)
Selected reference view 130 with 45757 connections over 42 pairs among 121 seed views
```

and, for contrast, the church's verdict on its second ceiling, where it is used:

```
Triplet filter: the ceiling at the second-face score 0.75 (0.964) leaves two faces, pieces of 131 and 82 images: used
Triplet filter: tau 0.964, the ceiling applied as given, its largest piece holding a majority (ceiling 0.964 at m 0.75, the second face's, d_max/|V| 0.856); the ceiling leaves 7 pieces (components of at least 3 images) holding 237 images between them, and 40 stragglers; survivor graph keeps 131/277 images in its largest component, 62 below degree 2 (0 before), and 812/20004 distinct image pairs
```

Two caveats. It is **not idempotent**: a saved filtered scene re-fed with the flag still set is
filtered *again*, with a fresh ceiling and, with `--triplet-auto-tau`, a fresh connectivity search
over the already shrunken graph. And with geometric verification disabled (`maxEpipolarError = 0`) no pair carries a two-view geometry, so nothing is an
edge, nothing is scored, and the filter removes the **whole** graph — the log line says so.

| Flag | Default | Effect |
|---|---|---|
| `--filter-triplets B` | **`false`** | apply the filter to the matched view graph |
| `--triplet-auto-tau B` | **`true`** | treat `tau(m)` as a ceiling: below it, the strictest threshold that joins every piece the ceiling leaves, unless the largest piece already holds a majority of the images in pieces, in which case the ceiling is applied as given; off applies `tau(m)` as given, the second face included |
| `--triplet-min-score F` | `0.6` | the paper's minimum edge score *m*, in (0,1); with `--triplet-auto-tau` the ceiling the threshold is derived from, otherwise applied as given |
| `--triplet-second-face-score F` | `0.75` | with `--triplet-auto-tau`, a stricter minimum score whose ceiling replaces the default's when the graph it leaves has two faces: a majority piece and a second piece of at least a third of it |

`TripletFilterConfig::minYield` (0.4) has no flag.

Python: `TripletFilterConfig(enabled, auto_tau, min_score, second_face_score, min_yield)`, `ReconstructionConfig.triplet_filter_cfg`, and `compute_triplet_scores(scene, min_score, min_yield, grid_size)` → the scores, `tau` and the graph statistics.

## Harness

`scripts/python/tests/triplet_disambiguation.py` (numpy only) reimplements the shipped rule from a
pairs CSV's own `NumMatches`, `Coverage` and `MeanRayAngle` columns: `score` writes its own score
and the kept flag for a given m; `parity` compares those scores against the same export's
`TripletScore` column (tolerance 1e-5); `roc` joins with `pair_gt_labels.py --mode coverage` output,
where **plausible** = true edge, **implausible** = false edge, **ambiguous** excluded.

## Measurements

Run folders live under the captures on the shared volume (never in the repo), `<capture>/openmvs-triplet-20260830-*`, driven by `.../polycam/normal/openmvs-triplet-20260830-tools/`.
**Parity:** 7 graphs (5 SIFT + 2 dense), 2078–6241 pairs, identical unscored sets, maximum absolute C++/Python difference **5.3e-7**.
**Discrimination** vs the depth-derived labels (AUC over all labelled pairs with unscored ranked
last / over the scored pairs alone):

| graph | pairs | triplets / comps | scored | AUC (all / scored) | true kept @ m=0.6 | false kept @ m=0.6 |
|---|---|---|---|---|---|---|
| `c191135f` sift | 2078 | 8287 / 4 | 1873 | 0.848 / 0.874 | 37.2 % | 0.0 % |
| `b192978b` sift | 5537 | 29725 / 6 | 4990 | 0.829 / 0.841 | 42.1 % | 5.9 % |
| `f7dbf861` sift | 3106 | 12898 / 3 | 2850 | 0.885 / 0.896 | 37.8 % | 0.0 % |
| `3b43828e` sift | 2211 | 5917 / 20 | 1691 | 0.615 / 0.766 | 31.0 % | 5.2 % |
| `32265651` sift | 3786 | 16807 / 5 | 3223 | 0.748 / 0.802 | 32.5 % | 2.5 % |
| `32265651` dense, gate-only | 6241 | 44889 / 1 | 6061 | 0.842 / 0.847 | 51.0 % | 2.3 % |
| `32265651` dense, gate+cross-check | 4480 | 23483 / 1 | 4318 | 0.892 / 0.896 | 51.1 % | 0.8 % |

Precision among the kept pairs is 0.96–1.00 everywhere; the problem is recall. `tau` was calibrated
on internet photo collections, where a true edge's inlier count varies far less than across a
hand-held capture, so `m = 0.6` discards **56–76 %** of the verified pairs and `m = 0.3` still
31–59 %.

**Reconstruction effect.** Every arm reconstructs the *same* saved matched scene, so control and filtered differ only by `--filter-triplets`.
Cells: *registered images (= the largest component here) [pairs kept] / components / median rotation error*.

| capture (input pairs) | control | m = 0.6 | m = 0.3 |
|---|---|---|---|
| `c191135f` sift (2078) | **199** / 53 / 0.954° | 47 [599] / 205 / 0.963° | 44 [964] / 208 / 0.817° |
| `b192978b` sift (5537) | **392** / 179 / 0.903° | 303 [1860] / 268 / 0.749° | 384 [3282] / 187 / 0.727° |
| `f7dbf861` sift (3106) | **282** / 64 / 0.311° | 267 [902] / 79 / 0.333° | 275 [1505] / 71 / 0.339° |
| `3b43828e` sift (2211) | **199** / 254 / 1.873° | 138 [536] / 315 / 4.269° | 181 [911] / 272 / 1.985° |
| `32265651` sift (3786) | **309** / 69 / 0.535° | 288 [1031] / 90 / 0.645° | 295 [1904] / 83 / 0.615° |
| `32265651` dense (6241) | 131 / 23 / 0.970° | **368** [2409] / 10 / 0.706° | 340 [4054] / 38 / 0.800° |

The one large **win** is the repetitive capture's *dense* graph, whose extra doppelganger edges are
what fragments it: 368 of 377 images in one component against 131, and a better median rotation.

**Doppelgangers** (`.../datasets/doppelgangers/openmvs-triplet-20260830/`). `test_pairs.npy`: 4660
labelled pairs, 16 scenes, 2330/2330. `reconstructions.tar.gz` (3 GB) holds **only** COLMAP
reconstructions — no database with `two_view_geometries` — so there are no full view graphs to score
and `n_sift_matches` is the edge weight; only 4 of 16 scenes reach 50 triangles from the labelled
pairs alone (the other 12 — Sofia 26, Charlottenburg 19, Brno 12, the rest ≤ 7 — are **unscorable**,
0–15 pairs scored). The paper validated on *COLMAP* view graphs: a weak proxy.

| scene | pairs | triplets | scored | AUC (all / scored) |
|---|---|---|---|---|
| Saint Alexander Nevsky, Łódź | 378 | 343 | 313 | 0.589 / 0.657 |
| Alexander Nevsky Cathedral, Tallinn | 382 | 250 | 313 | 0.611 / 0.700 |
| Cathedral of St Alexander Nevsky, Prešov | 128 | 116 | 104 | 0.528 / 0.638 |
| Washington Square Arch | 238 | 73 | 112 | 0.401 / 0.956 |

### Ambiguous-scene datasets

The datasets are the ones the disambiguation literature is written about: the video sets of Yan et al.
2017 (`books` 21 images, `cereal` 25, `cup` 64, `desk` 31, `oats` 23, `street` 19, and the Temple of
Heaven `ToH` 338) and the internet collections of Heinly et al. 2014 (`indoor` 153, `brandenburg_gate`
176, `church_on_spilled_blood` 278, `radcliffe_camera` 283, `big_ben` 403, `arc_de_triomphe` 435,
`alexander_nevsky_cathedral` 449). Reference counts are registered images: the paper's own filter
`G_F` (Manam and Govindu 2024, Table 1, `m = 0.3` on these sets) and Doppelgangers++ (Xiangli et al.,
Table 2, `a+b` = two models). Matching is exhaustive, as in the paper's reference implementation; the
filter's minimum score is its default 0.6 and its second-face score 0.75. On the sets whose ceiling
shatters the graph `m` only sets a ceiling the descent replaces, and the paper's 0.6 or 0.3 give the
same thresholds; on the church the second ceiling decides: at 0.6 the ceiling (0.942) sits among the
scores of the pairs bridging the facades and two matchings of four merged them, at 0.75 (0.964) every
graph seen leaves the two facades as a majority piece and a second piece of two thirds of it, so the
stricter ceiling is used; on Big Ben (one piece at 0.75), Brandenburg (a second piece of 7 images) and
Radcliffe (no majority) the paper's ceiling stands.

**Video sets** (all images register with and without the filter, so the verdict is whether the camera
path folds; "fold pairs" are camera pairs within one median step of each other at least five frames
apart, "spread" the extent of the camera path relative to its step):

| set | images | without the filter | with the filter | paper's `G_F` | verdict |
|---|---|---|---|---|---|
| books | 21 | 21, folded | 21; 2 fold pairs, the hover at frames 1-6, genuine | 9 | unfolded |
| cereal | 25 | 25, folded (spread 0.237) | 25, still folded (spread 0.277; frames 8-14): the true junction (13,14) carries 320 inliers at score 0.556, the doppelganger (7,16) 862 at 0.893, weaker in every cue | 7 | folded |
| cup | 64 | 64, folded | 64; 0 fold pairs, an open ring | 40 (their one failure) | unfolded |
| desk | 31 | 31, folded | 31; 23 fold pairs, all the start hover and the genuine revisit | 12 | unfolded |
| oats | 23 | 23, folded (spread 0.293) | 23; 0 fold pairs (spread 0.707) | 9 | unfolded |
| street | 19 | 19, folded | 19; 0 fold pairs | 19 | unfolded |
| ToH | 338 | 338, folded on the temple's one-third turn (804 fold pairs at gaps of 20 frames or more) | 338; 4 fold pairs, all the genuine closure of frames 0-9 onto 330-339 | — | unfolded |

**Internet collections** (the "without the filter" column is the branch's default matching, a
vocabulary tree at 50 pairs per image, which folds every two-faced building; the filter column is
exhaustive matching; a model is "one-sided" when every registered camera lies on one side of the
facade plane, checked with photos of known side):

| set | images | without the filter | with the filter | paper's `G_F` | Doppelgangers++ | verdict |
|---|---|---|---|---|---|---|
| indoor | 153 | 152 | 152, one loop | 42 | 152 | the loop is real; the paper over-splits it |
| brandenburg_gate | 176 | 173, folded | 127, folded: at 0.75 the second piece (7 images) is a cluster, not a face, so the paper's ceiling (0.973) stands and leaves one piece of 127 holding both faces and a majority, applied as given; the stricter ceiling on its own leaves 94, still folded | 129 | 151 | folded |
| church_on_spilled_blood | 278 | 270, folded | 126, one-sided: the stricter ceiling (0.964) leaves the south facade with the canal views (131 images) and the north facade (82) as two faces, so it is used; its largest piece holds a majority, the reconstruction seeds in it and the north facade stays unregistered (129-135 in earlier matchings of the same set) | 136 | 157+106 | unfolded |
| radcliffe_camera | 283 | 277, folded | 181: at 0.75 the largest piece (110 images) holds no majority, so the paper's ceiling (0.948) stands; it leaves pieces of 121, 55, 44, 19 and 15 images, none a majority, the descent to 0.842 joins them, the reconstruction seeds in the 121-piece, crosses into the 55-piece through a 312-inlier pair scoring 0.932 and refuses the 95-inlier bridge at 0.842 into the other three | 177 | 186+94 | unfolded |
| big_ben | 403 | 391 | 377: at 0.75 the graph is one piece (374 images), so the paper's ceiling (0.866) stands and leaves one piece of 391, applied as given; the stricter ceiling on its own keeps so few pairs that the reconstruction discards most of what it registers (147) | 379 | 394 | one face |
| arc_de_triomphe | 435 | 405 | 363: at 0.75 the second piece (43 images) is under a third of the largest (320), so the paper's ceiling (0.816) stands and leaves one piece of 395, applied as given; the 39 stragglers and 32 images of the piece do not register (the stricter ceiling on its own: 302) | 394 | 392 | side not checked |
| alexander_nevsky_cathedral | 449 | 442 | 418: at 0.75 the graph is one piece (411 images), so the paper's ceiling (0.954) stands and leaves one piece of 420, applied as given (the stricter ceiling on its own: 413) | 429 | 445 | one face |

On the two-faced buildings the filter matches the paper (church 126 against 136, Radcliffe 181
against 177) and, like the paper and Doppelgangers++, produces one face per model; on the one-faced
buildings the second ceiling is not used and the paper's ceiling keeps the model whole (Big Ben 377
against 379, Nevsky 418 against 429, the Arc 363 against 394); Brandenburg and cereal are the two
misses, and in both the doppelganger pairs outscore the true junction in every cue the filter has; on
the video sets every path but cereal's unfolds where the paper over-splits (books 9 of 21, oats 9 of
23, desk 12 of 31) or fails (cup).

## The default, and why

The pre-registered rule made `--filter-triplets` default to true only if *all* of: no sift capture
loses more than 2 % of its registered images; none worsens its median rotation error by more than
5 %; and the dense `32265651` run registers more images in its largest component than the dense
gate-only run alone (124). At `m = 0.6` the first fails on **every** capture (−5.3 % to −76.4 %) and the
second on three of five (+7.1 %, +20.6 %, +128 %); only the third passes, decisively (368 > 124).
The rule is a conjunction: **the default stays `false`**, enable it on a dense repetitive graph.

The filter is a tool for scenes with repeated structure, matched exhaustively; on the retrieval-matched
graphs the branch builds by default (50 pairs per image) the ceiling leaves one piece holding both
faces of every two-faced building here, because retrieval prefers the look-alike pairs and the
triangles a doppelganger sits in hold few of the strong true pairs that would score it down.
`--filter-triplets` stays an explicit flag, to be set with exhaustive matching on such a scene.

## Limitations and follow-ups

* **Auto-enabling is the obvious next step.** `--triplet-auto-tau` already picks the threshold;
  `--filter-triplets` itself is still a manual flag, and the filter is a large win exactly where the
  matched graph is dense *and* fragments (368 of 377 images in one component against 131 on the dense
  `32265651` graph) and a loss everywhere else — a rule reading the graph's own statistics would beat
  that manual flag too.
* **Filtering before view-graph calibration is untested.** It runs after `MatchPairs`, so
  `ViewGraphCalibrator` has already solved focal lengths over the *unfiltered* graph. Removing the
  doppelganger edges first should hand it a cleaner graph; it belongs with the experiment above.
* **Brandenburg Gate**: both faces sit inside one ceiling piece (127 images at the paper's 0.6, 94 at
  0.75; the second piece at 0.75 holds 7 images, so the stricter ceiling is not used) because the
  night photos of the two faces match each other as strongly as neighbours do; no inlier-count cue
  separates them, and the filter has no other.
* **cereal**: the true junction is weaker than the doppelganger in every cue (320 inliers at 0.556
  against 862 at 0.893); the paper over-splits it instead.
* **One model per run**: the pieces the ceiling leaves apart stay unregistered (the church's north
  facade, Radcliffe's three look-alike pieces), where Doppelgangers++ reports two models.
  Reconstructing the remaining pieces as further models is a pipeline question, not the filter's.
* **Retrieval-matched graphs**: the method needs the exhaustive graph; see the default.
* **The offline replay** (`triplet_replay.py` beside the datasets) approximates the run's strength
  from the match count; its pieces at the ceiling differ from the run's by tens of images on these
  near-complete graphs, and only the run's own `TripletScore` column is evidence.
* **Registered-image counts on `32265651` are not a stable ranking**: four matcher configurations register 146/124/137/192 images there, with *disjoint* sets.
* **Not built here**, recorded as follow-ups: Kataria et al.'s ambiguity-aware track-length cue
  (AAM) and Wilson & Snavely's bipartite local clustering coefficient — both complementary to the
  inlier-ratio cue, both needing more than one integer per edge.
