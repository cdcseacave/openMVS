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
code from the MATLAB release or any port. `ViewGraphTriplets.{h,cpp}`; **on by default**, in a
mode that removes only the pairs joining pieces the paper's threshold keeps apart, and of those
only what the graph can spare (below); the cutting rule that unfolds a symmetric building is
`--triplet-cut`.

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

## Two modes

The algorithm above is the **cutting rule**: the threshold applied as the paper means it, the
pieces it leaves apart left apart, the faces named and cut, the descent below a shattered ceiling.
It is what unfolds a symmetric building, and it is behind `--triplet-cut` because on a scene with
no repeated structure it is a loss: on eight Polycam interiors it halves the registrations of
every capture it touches (2678a364 95 -> 50, e00da096 92 -> 14, 5992d620 191 -> 116, 16d09ada
233 -> 154), removing pairs the reference trajectories say are true, and the second ceiling names
a room a face (5828945d: a 39-image room cut off as "the other face"); on a dense orbit it
removes the wide-baseline pairs bundle adjustment needs most and costs 7-14 % of the rotation
accuracy (see the default). Nothing in a pair tells a doppelganger from a weak true pair -- on
Big Ben the pairs scoring below 0.2 are 7,394 true and 5,451 false, and on a video every pair but
the consecutive ones is the weak side of a triangle a consecutive pair tops -- but the graph
around them differs, and that is what the default reads.

The default is the **keep mode**. The ceiling `tau(m)` is applied to the verified graph and the
pairs it keeps -- the unscored ones and those scoring at or above it -- are joined into
*pieces*, the connected components of the ceiling's survivor graph. The candidates are the scored
pairs below the threshold whose two images lie in different pieces: what the ceiling sets apart.
A weak pair inside one piece is kept, whatever its score, and counts toward the floor. Of the
candidates, only what the graph can spare goes:

1. **The floor.** Every image keeps at least `keepPairs` (3) of its pairs and enough of them to
   hold `keepMatches` (2,000) weighted inliers, counting only pairs whose ray angle
   (`ImagePair::meanRayAngle`, the median angle between the viewing rays of the track-forming
   matches) reaches `keepMinAngle` (3 degrees; an unmeasured angle counts). Pairs that are not
   candidates count first; images are served in ascending order of what they keep, each retaining
   its best-scoring counting candidates, ties to the stronger, until both bounds hold or such
   candidates run out; a retained pair counts for both its images. Why the angle: a near-duplicate
   pair yields no 3D point, so a burst of near-duplicate frames would otherwise keep only its own
   pairs, lose every link to the rest of the capture and be invalidated for a median triangulation
   angle below the reconstruction's 1.5 degrees (5992d620 lost ten such frames before the angle
   counted).
2. **The threshold is the strictest one the graph fits.** An image falls short at a threshold when
   its counting pairs that are not candidates at that threshold already fall short of the floor.
   The keep mode's threshold is the strictest value at or below the ceiling at which at most
   `keepMaxShort` (a half) of the images fall short: the ceiling when it fits, else the largest
   candidate score that does, found by a binary search over the distinct scores below the ceiling
   (the count is monotone). A graph that fits no threshold -- every scored pair kept and still more
   than half the images short -- loses nothing; the scores are still computed and exported and the
   seeds still named. Every interior of the campaign is such a graph: most of its images hold fewer
   than 2,000 matches at 3 degrees or more in the whole graph.
3. **The repair.** Every connected component of the unfiltered graph stays one component: the
   candidates still unretained, best-scoring first, are retained whenever they join two components
   of the survivor graph.
4. Distinct image pairs decide, through their highest-scoring scene pair; duplicates follow.
   Seeding is as in the cutting rule (the largest piece the ceiling leaves).

A graph the ceiling leaves in one piece has no candidates and loses nothing, and the log says so:

```
Triplet filter: the ceiling 0.520 (m 0.30, d_max/|V| 0.315) leaves every component of the graph in one piece (1 pieces of 251 nodes, the largest 251): no pair below it joins two pieces, nothing to set apart, nothing removed
```

That is every Tanks and Temples orbit (Truck 251, Ignatius 263, Meetingroom 371, Caterpillar 383,
Barn 410 images, all one piece), the Temple of Heaven video (338), Heinly's indoor set (152) and
the interiors that are not one piece fit no threshold. The paper's evidence for the threshold is a
*collection*: the survivor graph of a two-faced building falls into pieces at the ceiling
(measured on the campaign's graphs, the largest piece holds 240 of the church's 277 images, 145
of Brandenburg's 175, 244 of Radcliffe's 281, 414 of the Arc's 433, 434 of Nevsky's 447, 396 of
Big Ben's 402), and it is the pairs joining those pieces the paper's method removes. On an orbit
the same threshold names sixty per cent of the pairs -- the wide-baseline ones, which lose to a
consecutive pair in every triangle -- and the graph stays one piece; an earlier keep rule that
took every scored pair below the threshold as a candidate removed them (Truck kept 2,327 of 6,054
pairs, Meetingroom 3,868 of 8,735) and paid 7-14 % of the rotation accuracy for it. The
labelled exports say why no threshold can do better: the ceiling sits below 61 % of Truck's true
pairs, 63 % of Meetingroom's, 73 % of the Arc's, 86 % of Nevsky's and 86 % of the church's, and
below 98-100 % of the false pairs everywhere; a collection at 62-152 verified pairs per image
survives losing three quarters of its true pairs by redundancy, an orbit at 24 pairs per image
does not.

When the keep mode acts, the log names the pieces, the candidates and what the floor kept:

```
Triplet filter: tau 0.963, the ceiling (ceiling 0.963 at m 0.30, d_max/|V| 0.947) leaves 3 pieces (the largest 9 of 19 nodes); 111 pairs below the threshold join two of them and are the candidates, 4 of 19 images short of the floor (3 pairs holding 2000 matches at 3 degrees or more) without them: 8 candidates retained for the floor, 0 to keep every component whole, 0 images whose candidates ran out before the floor held
Triplet filter: kept 68/171 scene pairs (tau 0.963; 19 nodes, max degree 18; 969 triplets in 1 components, 235 doppelganger triplets gave no evidence; 103 below tau and 0 cut by the face rule removed, 0 unscored kept); the reconstruction seeds in the largest piece the ceiling leaves (9 images)
```

Why not a larger floor instead of the fit: measured on the interiors with the floor counting every
pair, the incremental reconstruction of these captures flips under any change of the pair set, in
both directions -- a floor of 1,000 matches breaks 2678a364 outright (median rotation error 28
degrees against 0.4), a floor of 4,000 breaks 8d2f4877 after removing 189 of its 2,628 pairs where
2,000, removing 790, improves it, and 2,000 registers 468 of 17ac94cc's 554 images against 274 at
twice the error. No floor short of keeping everything guarantees the base on such a graph; the fit
does, by construction. A floor set as a share of an image's matches was simulated and dropped (it
does nothing for a dense orbit and floods false pairs back on cup and Radcliffe); a plain stand-down
at the ceiling without the descent was measured and dropped (it left the small sets and Brandenburg
untouched); and normalising the strength by the graph's yield envelope at the pair's ray angle, so
a wide-baseline pair stops losing to the consecutive pair in every triangle, was replayed on the
labelled exports and dropped: it moves Truck's share of true pairs below the ceiling from 61 % to
50 % and merges the church's two faces into one piece.

## Where it runs, and the flags

`Scene::Reconstruct` applies it **right after** the diagnostics export (`--export-pairs-csv` /
`--export-retrieval-csv`) and before `matchImagesOnly` returns, from both call sites (after
`MatchPairs`, and on the already-matched-`.sfm` early return). The CSV therefore always lists the
*whole* matched graph, each pair's score in a `TripletScore` column (empty = unscored), followed by
seven relative-pose columns (empty when the pair has none), of the export's header
`ImageA,ImageB,NumMatches,Coverage,Weight,WeightSpatial,WeightConnectivity,WeightTriplet,MeanRayAngle,TripletScore,RelQw,RelQx,RelQy,RelQz,RelTx,RelTy,RelTz`,
where the rotation maps a point of image A into image B (x_B = R x_A + t, the quaternion scalar first) and the translation is a unit vector, so a run can be re-scored offline from its own export; `ComputePairsWeights` re-runs afterwards, when
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
| `--filter-triplets B` | **`true`** | apply the filter to the matched view graph, in the keep mode unless `--triplet-cut` |
| `--triplet-cut B` | `false` | the cutting rule: the ceiling applied as given when its largest piece holds a majority, the smaller pieces left apart, the faces named and the other face cut, the descent below a shattered ceiling, no floor -- unfolds a symmetric building, halves the registrations of an interior and costs an orbit 7-14 % of its rotation accuracy |
| `--triplet-keep-pairs N` | `3` | keep mode: the pairs every image keeps at least |
| `--triplet-keep-matches N` | `2000` | keep mode: the weighted inliers every image keeps at least, over its counting pairs |
| `--triplet-keep-min-angle F` | `3` | keep mode: only pairs whose ray angle reaches this many degrees count towards the floor (0 counts every pair) |
| `--triplet-keep-max-short F` | `0.5` | keep mode: the largest share of the images that may fall short of the floor without the candidates; the threshold descends to the strictest one at or below the paper's where that holds, and a graph fitting none loses nothing (1 fits every threshold) |
| `--triplet-auto-tau B` | `true` | with `--triplet-cut`: treat `tau(m)` as a ceiling and descend below it to the strictest threshold joining every piece the ceiling leaves, unless the largest piece already holds a majority; off applies `tau(m)` as given, the second face included |
| `--triplet-min-score F` | `0.3` | the paper's minimum edge score *m*, in [0,1], from which the ceiling `tau(m)` is derived (both modes) |
| `--triplet-second-face-score F` | `0.75` | with `--triplet-cut` and `--triplet-auto-tau`: a stricter minimum score whose ceiling names the faces when the graph it leaves has two (a majority piece and a second piece of at least a third of it); the default's ceiling then applies inside the larger face and every pair joining the other face is removed |

`TripletFilterConfig::minYield` (0.4) has no flag.

Python: `TripletFilterConfig(enabled, auto_tau, cut, keep_pairs, keep_matches, keep_min_angle, keep_max_short, min_score, second_face_score, min_yield)`, `ReconstructionConfig.triplet_filter_cfg`, and `compute_triplet_scores(scene, min_score, min_yield, grid_size)` → the scores, `tau` and the graph statistics.

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
`openmvs-disambig-20260905l-triplet` folders under each set beside the datasets (`indoor` and `ToH`,
matched with the vocabulary tree: `openmvs-disambig-20260905l-vocab50-triplet`). On the sets whose
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
| ToH | 338 | 338, folded on the temple's one-third turn (804 fold pairs at gaps of 20 frames or more) | 338 (vocabulary tree at 50 pairs per image, tau 0.455 applied as given); 3 fold pairs at gaps of 20 frames or more, all the genuine closure of frame 0 onto 329 | 338 | unfolded |

**Internet collections** (the "without the filter" column is the branch's default matching, a
vocabulary tree at 50 pairs per image, which folds every two-faced building; the filter column is
exhaustive matching except on `indoor`, whose loop the vocabulary tree keeps; a model is "one-sided"
when every registered camera lies on one side of the facade plane, and the verdict column is the
reference-model comparison described above):

| set | images | without the filter | with the filter | paper's `G_F` | Doppelgangers | verdict |
|---|---|---|---|---|---|---|
| indoor | 153 | 152 | 152, one loop (vocabulary tree at 50 pairs per image, tau 0.664 applied as given) | 42 | 152 | the loop is real; the paper over-splits it |
| brandenburg_gate | 176 | 173, folded | 145: at 0.75 the second piece (7 images) is a cluster, not a face, so the paper's ceiling (0.952) stands; it leaves a majority piece and is applied as given | 129 | 151 | unfolded: 144 of the reference's 151 in common, 21 misplaced by position, directions within 6 degrees at the 90th percentile |
| church_on_spilled_blood | 278 | 270, folded | 143, one-sided: the second ceiling (0.964) leaves the south facade with the canal views (133 images) and the north facade (83) as two faces and names them; the paper's ceiling (0.899) applies inside the south face, whose piece grows to 148, and the north face is cut off and stays unregistered | 136 | 258 | one-sided, unfolded but scattered: 126 of the south reference's 137 in common, 46 of them misplaced by position (up to 3.9 model radii; the run at the stricter ceiling alone had 40 of 116), directions within 8 degrees at the 90th percentile |
| radcliffe_camera | 283 | 277, folded | 181: at 0.75 the largest piece (111 images) holds no majority, so the paper's ceiling (0.908) stands; it leaves a majority piece of 181, applied as given | 177 | 94 | one-sided, unfolded: 181 of the side reference's 185 in common, 11 misplaced by position |
| big_ben | 403 | 391 | 385: one piece at 0.75, so the paper's ceiling (0.763) stands, applied as given | 379 | 394 | folded: 209 of 377 common images misplaced, the reference's two sides on one another (the limit below) |
| arc_de_triomphe | 435 | 405 | 403: at 0.75 the second piece is under a third of the largest, so the paper's ceiling (0.679) stands, applied as given | 394 | 392 | unfolded: 370 of the reference's 395 in common, 26 misplaced by position, directions within 5 degrees at the 90th percentile |
| alexander_nevsky_cathedral | 449 | 442 | 434: one piece at 0.75, so the paper's ceiling (0.919) stands, applied as given | 429 | 445 | unfolded: 433 of the reference's 446 in common, 32 misplaced by position, directions within 11 degrees at the 90th percentile |

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

The filter is on by default because the keep mode leaves a scene with no repeated structure as it
was -- a graph the ceiling keeps in one piece has nothing to set apart, and a graph that fits no
threshold loses nothing -- and removes, from a collection the ceiling splits, the pairs joining
its pieces as far as the graph can spare them. The campaign of 2026-09-07 (runs
`openmvs-triplet-default-20260907-<arm>` under each normal scene and `openmvs-disambig-20260907-<arm>`
under each ambiguous set; every arm reconstructs the base arm's saved matched scene, so the arms
differ only in the filter; `final-nofilter` is that scene reconstructed with the filter off, the
baseline every arm is read against, since a saved scene reconstructs a few images apart from the
in-process run and two reconstructions of the same scene differ by the bundle adjustment's
run-to-run variance):

| scene | images | what the filter did | registered: no filter / default | rotation median / p90 (deg): no filter / default |
|---|---|---|---|---|
| Truck (T&T) | 251 | one piece: nothing removed | 251 / 251 | 0.141 / 0.159 both |
| Meetingroom (T&T) | 371 | one piece: nothing removed | 371 / 371 | 0.138 / 0.151 both |
| 2678a364 (Polycam) | 165 | 8 pieces, 11 joining pairs, 134 of 148 images short: fits no threshold | 95 / 95 | 0.402 / 0.704 both |
| e00da096 | 189 | 7 pieces, 11 joins, 159 of 168 short: fits none | 92 / 92 | 1.033 / 1.369 both |
| 5992d620 | 223 | 7 pieces, 420 joins, 144 of 200 short: fits none | 192 / 191 | 0.616 / 0.742 against 0.610 / 0.711 |
| 8d2f4877 | 225 | 5 pieces, 29 joins, 138 of 205 short: fits none | 199 / 199 | 0.307 / 0.745 against 0.302 / 0.742 |
| 5828945d | 238 | 6 pieces, 21 joins, 195 of 231 short: fits none | 33 / 33 | 1.258 / 1.492 both |
| 16d09ada | 242 | one piece: nothing removed | 233 / 233 | 0.321 / 0.598 against 0.311 / 0.600 |
| 5ada248e | 540 | 13 pieces, 19 joins, 426 of 501 short: fits none | 207 / 207 | 0.492 / 0.719 both |
| 17ac94cc | 554 | 10 pieces, 8 joins, 401 of 534 short: fits none | 274 / 274 | 0.690 / 1.043 both |

Ignatius, Caterpillar and Barn are one piece at the ceiling like Truck and Meetingroom and take
the same early return. The pre-registered rule for the default (no capture loses more than 2 % of
its registrations or 5 % of its median rotation accuracy) is met on every scene above because the
keep mode removes nothing there; the differences in the table are the run-to-run variance (two
reconstructions of the same unfiltered Truck scene differ by more than the default and the
no-filter arm do). An earlier keep rule that took every scored pair below the threshold as a
candidate cost Truck 0.157 against 0.141, Meetingroom 0.148 against 0.138, Caterpillar 0.127
against 0.111 and Barn 0.124 against 0.113 (+7 to +14 %) by removing the wide-baseline pairs;
that measurement is why the candidates are only what the ceiling sets apart.

On the ambiguous sets (the same three arms; the two-face collections' faces never co-observe, so
a model holding cameras of both reference faces joined them through false pairs -- "joined"; a
video set's fold is counted in fold pairs, frames within one median step of each other and five
or more apart; "misplaced" is the reference comparison's count, view 30 degrees or half a radius
off):

| set | images | no filter | default (keep mode) | cutting rule (`--triplet-cut`) |
|---|---|---|---|---|
| church_on_spilled_blood | 278 | 272, both faces: joined | 1441 of 22996 pairs removed; 275, both faces: joined | 142, one-sided |
| brandenburg_gate | 176 | 173, both sides: joined | 1119 of 11880 removed; 171, both sides: joined | 145, one-sided |
| radcliffe_camera | 283 | 277, both sides: joined | 2413 of 22517 removed; 278, both sides: joined | 65, one-sided, on a matching the ceiling leaves in one piece (181 on the earlier matching) |
| arc_de_triomphe | 435 | 423, folded (view p90 150 deg) | 68 of 27108 removed; 418, folded | 400, unfolded (view p90 4.9) |
| big_ben | 403 | 395, folded | 18 of 35922 removed; 395, folded | 379, folded (the limitation below) |
| alexander_nevsky_cathedral | 449 | 440, folded (view p50 78) | 945 of 68071 removed; 444, folded | 432, unfolded (view p90 9.1) |
| street | 19 | 19, 10 misplaced | 103 of 171 removed; 19, 4 misplaced | 19, 0 misplaced |
| cereal | 25 | 25, 10 misplaced, folded | 236 of 296 removed; 25, 4 misplaced | 25, 0 misplaced |
| cup | 64 | 64, 28 misplaced, folded | 623 of 2013 removed; 64, 14 misplaced, still folded | 52, 0 misplaced, 12 images out |
| books | 21 | 21, 7 fold pairs | 147 of 209 removed; 21, 2 fold pairs | 21, 2 fold pairs |
| desk | 31 | 31, 31 fold pairs | 308 of 439 removed; 31, 31 fold pairs | 31, 23 fold pairs |
| oats | 23 | 23, 0 fold pairs | 170 of 248 removed; 23, 0 | 22, 0 |
| ToH | 338 | 338, 804 fold pairs (folded) | one piece: nothing removed; 338, 804 fold pairs | 338, 3 fold pairs (unfolded) |
| indoor | 153 | 152 | one piece: nothing removed; 152 | 152 |

The keep mode helps the small video sets and leaves the collections as they were: its repair keeps
every component of the unfiltered graph whole, so a two-faced building keeps its best-scoring
joins and reconstructs with both faces in one model, and the Arc and Nevsky, nearly one piece at
the ceiling with their false pairs inside it, fold as without the filter. Those are the scenes for
`--triplet-cut`, whose numbers reproduce the earlier campaign on this base within a few images
(church 142 against 143, Arc 400 against 403, Nevsky 432 against 434) except on Radcliffe, whose
re-matched graph the ceiling no longer splits.

## Limitations and follow-ups

* **The keep mode does not separate two faces.** Its repair keeps every component of the
  unfiltered graph whole, so a two-faced collection whose faces the ceiling sets apart keeps its
  best-scoring joins and reconstructs as one model with both faces in it (church, Brandenburg and
  Radcliffe register both reference faces under the default; the cutting rule registers one), and
  a collection that is nearly one piece at the ceiling (the Arc, Nevsky) keeps the false pairs
  inside that piece and folds as without the filter. Such a collection is run with `--triplet-cut`.
* **A symmetric object orbited by a video is one piece at the ceiling.** The Temple of Heaven (338
  frames) keeps every pair under the default and folds (804 fold pairs, 3 under `--triplet-cut`):
  its false pairs join symmetric sides of a graph the consecutive frames already hold together, and
  no structure of the graph tells them from an orbit's wide-baseline true pairs, which score as
  low. The cutting rule unfolds it at the price the orbits pay.
* **The wide-baseline bias.** Every non-consecutive pair of a video is the weak side of a triangle
  a consecutive pair tops, so the pairs bundle adjustment needs most score lowest; the keep mode
  sidesteps the bias by never removing a pair inside a piece. Normalising the strength by the
  yield envelope at the pair's ray angle was replayed on the labelled exports and does not remove
  it (Truck's share of true pairs below the ceiling 61 % to 50 %, and the church's faces merge).
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
* **One model per run, under the cutting rule**: the pieces the ceiling leaves apart stay
  unregistered (the church's north facade, Radcliffe's three look-alike pieces), where
  Doppelgangers++ reports two models. Reconstructing the remaining pieces as further models is a
  pipeline question, not the filter's.
* **Retrieval-matched graphs**: the method needs the exhaustive graph; see the default.
* **The offline replay** (`triplet_replay.py` beside the datasets) approximates the run's strength
  from the match count; its pieces at the ceiling differ from the run's by tens of images on these
  near-complete graphs, and only the run's own `TripletScore` column is evidence.
* **Registered-image counts on `32265651` are not a stable ranking**: four matcher configurations register 146/124/137/192 images there, with *disjoint* sets.
* **Not built here**, recorded as follow-ups: Kataria et al.'s ambiguity-aware track-length cue
  (AAM) and Wilson & Snavely's bipartite local clustering coefficient — both complementary to the
  inlier-ratio cue, both needing more than one integer per edge.
