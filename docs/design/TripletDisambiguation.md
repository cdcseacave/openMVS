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
   is `G_LCT`). `m` is the one user parameter — the default, 0.3, the paper's value for the medium and
   small ambiguous sets (0.6 its generic/large-scale value, 0.9 its highly ambiguous one).
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
   any of the above, the ceiling at `m` 0.75 is tried first, not as a threshold but to **name** the
   faces: when the graph it leaves is *two-faced* — its largest piece holds a strict majority of the
   images in pieces and its second-largest piece holds at least a third of the largest — the paper's
   ceiling at `m` applies **inside the larger face**, and every pair joining the other face to an
   image outside it is cut whatever its score, and so is every pair of an *ambiguous* image: one
   outside both faces whose pairs kept at the paper's ceiling reach both. The descent does not run
   once the faces are named — every pair between them is cut at every threshold, so there is nothing
   left for it to join. When the graph at the stricter ceiling is not two-faced, the paper's ceiling
   stands untouched. Why: on a two-faced building the paper's ceiling sits among the scores of the
   pairs bridging the facades (the church: 0.942 at 0.6, bridges scoring 0.89-0.96, merged in two
   matchings of four; at 0.75 every one of seven graphs splits into the facades, 130-135 and 80-83
   images) — applied inside the south facade, the paper's ceiling at `tau(0.3)` keeps 148 of its
   images in the piece where the higher ceiling by itself kept 131, with no north-facade image in it;
   a second piece of a third is the bar because the other face of a
   two-faced building holds a substantial share of the views and a night or detail cluster hanging
   off the largest piece holds a few percent (Brandenburg: 7 of 102 at 0.75).
7. **Seeding.** The filter reports the images of the largest piece the ceiling leaves (ties to the
   piece holding the lowest image index), and `StarInitializer::SelectReferenceView` chooses the
   reconstruction's reference view among them: the heaviest by weighted inliers with at least
   `minViews - 1` valid pairs (three by default) among the seed views, else the heaviest so-qualified
   image among every image, else — when nothing qualifies anywhere — the heaviest image overall, so
   the caller can report the shortfall. Why: the resection refuses the doppelganger bridges the
   descent lets through but cannot choose the side it starts on, and the heaviest image overall sits
   in the densest cluster of look-alike views (in the matching that showed this, Radcliffe's
   heaviest image sat in a 45-image piece and the 120-image piece never registered; Street: the
   largest piece's heaviest image had two pairs, the star needs three, and the run reconstructed
   nothing).

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
*whole* matched graph, each pair's score in a `TripletScore` column (empty = unscored), followed by
seven relative-pose columns (empty when the pair has none), of the export's header
`ImageA,ImageB,NumMatches,Coverage,Weight,WeightSpatial,WeightConnectivity,WeightTriplet,MeanRayAngle,TripletScore,RelQw,RelQx,RelQy,RelQz,RelTx,RelTy,RelTz`,
so a run can be re-scored offline from its own export; `ComputePairsWeights` re-runs afterwards, when
something was removed. Four lines from a Radcliffe run report everything:

```
Triplet filter: the ceiling at the second-face score 0.75 (0.967) leaves no second face (largest piece 111, second 52): the paper's ceiling stands
Triplet filter: tau 0.908, the ceiling applied as given, its largest piece holding a majority (ceiling 0.908 at m 0.30, d_max/|V| 0.869); the ceiling leaves 4 pieces (components of at least 3 images) holding 261 images between them, and 21 stragglers; survivor graph keeps 181/282 images in its largest component, 28 below degree 2 (0 before), and 1946/20361 distinct image pairs
Triplet filter: kept 4668/23083 scene pairs (tau 0.908; 282 nodes, max degree 245; 800921 triplets in 1 components, 242673 doppelganger triplets gave no evidence; 18415 below tau and 0 cut by the face rule removed, 2722 unscored kept); the reconstruction seeds in the largest piece the ceiling leaves (181 images)
Selected reference view 130 with 38832 connections over 30 pairs among 181 seed views
```

and, for contrast, the church's, where the second ceiling names the faces and the paper's ceiling applies inside the larger one:

```
Triplet filter: the ceiling at the second-face score 0.75 (0.964) leaves two faces, pieces of 133 and 83 images: the paper's ceiling 0.899 applies inside the larger face; 8909 pairs joining the other face and 0 pairs of 0 ambiguous images cut
Triplet filter: tau 0.899, the paper's ceiling applied inside the larger face, the other face cut off (ceiling 0.899 at m 0.30, d_max/|V| 0.856); the ceiling leaves 7 pieces (components of at least 3 images) holding 257 images between them, and 20 stragglers; survivor graph keeps 148/277 images in its largest component, 33 below degree 2 (0 before), and 1918/20238 distinct image pairs
Triplet filter: kept 2917/23016 scene pairs (tau 0.899; 277 nodes, max degree 237; 819680 triplets in 1 components, 213544 doppelganger triplets gave no evidence; 11190 below tau and 8909 cut by the face rule removed, 1000 unscored kept); the reconstruction seeds in the largest piece the ceiling leaves (148 images)
```

Two caveats. It is **not idempotent**: a saved filtered scene re-fed with the flag still set is
filtered *again*, with a fresh ceiling and, with `--triplet-auto-tau`, a fresh connectivity search
over the already shrunken graph. And with geometric verification disabled (`maxEpipolarError = 0`)
no pair carries a two-view geometry, so there are no inlier counts, no pair is scored, and the
filter keeps every pair and does nothing — the log line reports it as `0 below tau removed, N
unscored kept`.

| Flag | Default | Effect |
|---|---|---|
| `--filter-triplets B` | **`false`** | apply the filter to the matched view graph |
| `--triplet-auto-tau B` | **`true`** | treat `tau(m)` as a ceiling: below it, the strictest threshold that joins every piece the ceiling leaves, unless the largest piece already holds a majority of the images in pieces, in which case the ceiling is applied as given; off applies `tau(m)` as given, the second face included |
| `--triplet-min-score F` | `0.3` | the paper's minimum edge score *m*, in [0,1]; with `--triplet-auto-tau` the ceiling the threshold is derived from, otherwise applied as given |
| `--triplet-second-face-score F` | `0.75` | with `--triplet-auto-tau`, a stricter minimum score whose ceiling replaces the default's when the graph it leaves has two faces: a majority piece and a second piece of at least a third of it |

`TripletFilterConfig::minYield` (0.4) has no flag.

Python: `TripletFilterConfig(enabled, auto_tau, min_score, second_face_score, min_yield)`, `ReconstructionConfig.triplet_filter_cfg`, and `compute_triplet_scores(scene, min_score, min_yield, grid_size)` → the scores, `tau` and the graph statistics.

## Harness

`scripts/python/tests/triplet_disambiguation.py` (numpy only) reimplements the scoring -- strength,
the yield envelope, the triangles of `G_LCT` -- from a pairs CSV's own `NumMatches`, `Coverage` and
`MeanRayAngle` columns and applies `tau(m)` as a threshold; it does not replay the second-face
choice, the descent or the seeding, which stay C++-side only. `score` writes its own score and the
kept flag for a given m (`-m`, default the shipped 0.3); `parity` compares those scores against the
same export's `TripletScore` column (tolerance 1e-5); `roc` joins with `pair_gt_labels.py --mode
coverage` output, where **plausible** = true edge, **implausible** = false edge, **ambiguous**
excluded.

## Measurements

Run folders live under the captures on the shared volume (never in the repo), `<capture>/openmvs-triplet-20260830-*`, driven by `.../polycam/normal/openmvs-triplet-20260830-tools/`.
**Parity:** checked against two exhaustive exports, identical unscored sets both, maximum absolute C++/Python difference **5.140e-07** on `street` (171 pairs) and **5.287e-07** on `radcliffe_camera` (22197 pairs, 19383 of them scored).
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
`alexander_nevsky_cathedral` 449). Reference counts are registered images, both columns from Manam and
Govindu 2024, Table 3 (`m = 0.3` on these sets): their own filter `G_F`, and Doppelgangers (Cai et
al. 2023) as they ran it. Doppelgangers++ (Xiangli et al.) evaluates five of these collections and
reports the church as 157+106 and Radcliffe as 94+186 (two models each), Big Ben 394, the Arc 423 and
Nevsky 447; its Brandenburg Gate is a different, 2137-image collection, and it does not cover Indoor
or the video sets. Matching is exhaustive, as in the paper's reference implementation; the filter's minimum score is
its default 0.3, the paper's value on these sets, and its second-face score 0.75; the runs are the
`openmvs-disambig-20260905l-triplet` folders under each set beside the datasets. On the sets whose
ceiling shatters the graph `m` only sets a ceiling the descent replaces, and 0.3 and 0.6 give the
same thresholds. On the church the second ceiling (0.964) leaves the two facades as a majority piece
and a second piece of two thirds of it, so it names the faces, and the paper's ceiling (0.899)
applies inside the larger one; on Radcliffe (no majority at 0.75), Brandenburg (a second piece of 7
images), Big Ben, the Arc and Nevsky (one piece) the paper's ceiling stands, and at 0.3 it leaves a
majority piece on every one of them, applied as given. A collection's verdict is its comparison with
the Doppelgangers authors' verified COLMAP model of the same collection (`model_compare.py` beside
the datasets): after a similarity alignment on the images the two have in common, an image is
misplaced when its position is off by half the model's radius or more, or its viewing direction by
30 degrees or more. A folded model has its misplaced images' directions off by 60 degrees and more;
a scattered one has them off by position alone, with the directions within a few degrees.

**Video sets** (all images register with and without the filter, so the verdict is whether the camera
path folds; "fold pairs" are camera pairs within one median step of each other at least five frames
apart, "spread" the extent of the camera path relative to its step):

| set | images | without the filter | with the filter | paper's `G_F` | verdict |
|---|---|---|---|---|---|
| books | 21 | 21, folded | 21; 2 fold pairs, the hover at frames 1-6, genuine | 9 | unfolded |
| cereal | 25 | 25, folded (spread 0.237) | 25; the descent to 0.782; the path doubles back (spread 0.277, frames 8-14) exactly as the verified reference model has it: 24 of its 24 images in common, none misplaced | 7 | unfolded |
| cup | 64 | 64, folded | 52; at 0.3 the ceiling (0.989) leaves a majority piece of 52, applied as given, and the other 12 images stay out (at 0.6 the descent joined all 64); 0 fold pairs, an open ring, 51 of the reference's 63 in common and none misplaced | 40 (their one failure) | unfolded |
| desk | 31 | 31, folded | 31; 23 fold pairs, all the start hover and the genuine revisit | 12 | unfolded |
| oats | 23 | 23, folded (spread 0.293) | 23; 0 fold pairs (spread 0.707) | 9 | unfolded |
| street | 19 | 19, folded | 19; 0 fold pairs | 19 | unfolded |
| ToH | 338 | 338, folded on the temple's one-third turn (804 fold pairs at gaps of 20 frames or more) | 338 (vocabulary tree at 50 pairs per image, tau 0.455 applied as given); 3 fold pairs at gaps of 20 frames or more, all the genuine closure of frame 0 onto 329 | — | unfolded |

**Internet collections** (the "without the filter" column is the branch's default matching, a
vocabulary tree at 50 pairs per image, which folds every two-faced building; the filter column is
exhaustive matching except on `indoor`, whose loop the vocabulary tree keeps; a model is "one-sided"
when every registered camera lies on one side of the facade plane, and the verdict column is the
reference-model comparison described above):

| set | images | without the filter | with the filter | paper's `G_F` | Doppelgangers | verdict |
|---|---|---|---|---|---|---|
| indoor | 153 | 152 | 152, one loop (vocabulary tree at 50 pairs per image, tau 0.664 applied as given) | 42 | 152 | the loop is real; the paper over-splits it |
| brandenburg_gate | 176 | 173, folded | 145: at 0.75 the second piece (7 images) is a cluster, not a face, so the paper's ceiling (0.952) stands; it leaves a majority piece and is applied as given | 129 | 151 | unfolded: 144 of the reference's 151 in common, directions within 6 degrees at the 90th percentile |
| church_on_spilled_blood | 278 | 270, folded | 143, one-sided: the second ceiling (0.964) leaves the south facade with the canal views (133 images) and the north facade (83) as two faces and names them; the paper's ceiling (0.899) applies inside the south face, whose piece grows to 148, and the north face is cut off and stays unregistered | 136 | 258 | one-sided, unfolded: 126 of the south reference's 137 in common, directions within 8 degrees at the 90th percentile |
| radcliffe_camera | 283 | 277, folded | 181: at 0.75 the largest piece (111 images) holds no majority, so the paper's ceiling (0.908) stands; it leaves a majority piece of 181, applied as given | 177 | 94 | one-sided, unfolded: 181 of the side reference's 185 in common, 11 misplaced by position |
| big_ben | 403 | 391 | 385: one piece at 0.75, so the paper's ceiling (0.763) stands, applied as given | 379 | 394 | folded: 209 of 377 common images misplaced, the reference's two sides on one another (the limit below) |
| arc_de_triomphe | 435 | 405 | 403: at 0.75 the second piece is under a third of the largest, so the paper's ceiling (0.679) stands, applied as given | 394 | 392 | unfolded: 370 of the reference's 395 in common, 26 misplaced by position, directions within 5 degrees at the 90th percentile |
| alexander_nevsky_cathedral | 449 | 442 | 434: one piece at 0.75, so the paper's ceiling (0.919) stands, applied as given | 429 | 445 | unfolded: 433 of the reference's 446 in common, directions within 11 degrees at the 90th percentile |

Against the paper's own filter the branch registers more on every collection and keeps every model
but one unfolded: the church 143 against 136 (one-sided, as the paper's model and both learned
methods' models are), Radcliffe 181 against 177, Brandenburg 145 against 129, the Arc 403 against 394,
Nevsky 434 against 429; on the video sets every path unfolds where the paper over-splits (books 9 of
21, oats 9 of 23, desk 12 of 31, cereal 7 of 25) or fails (cup: 52 whole against 40). Big Ben is the
one miss: 385 registered against 379, but folded, as every threshold of the pairwise geometry leaves
it (the limit below). Against Doppelgangers++, which reports two models on the church (157+106) and
Radcliffe (94+186), the branch's one model is within 5 images of the larger on Radcliffe and 14
short of it on the church; on the Arc (423) and Nevsky (447) it is 20 and 13 short, and on Big Ben
(394) short and folded.

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
* **Big Ben**: the tower's two long sides are near-identical and the matcher verifies more pairs
  between them than between the true corners: at every threshold of the triplet score the bridges
  between the sides outnumber the true corner links (at 0.3, 13 against 6), rotation cycles close
  through the symmetry as often as through true pairs, and the thinnest cut of the kept graph parts
  the sides from each other, not from their doppelgangers. Nothing in pairwise geometry tells the
  two apart; the reference model's authors used appearance. The filter registers 385 of 403 images,
  more than the paper's 379, on a folded model.
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
