# Camera-Triplet View-Graph Disambiguation

## Overview

An optional, dependency-free pre-reconstruction filter that removes wrong image pairs — repeated
structure ("doppelgangers"), retrieval false positives — from the matched view graph using nothing
but the graph itself and **one integer per edge**: the epipolar inlier count. It reimplements
S. M. Manam and V. M. Govindu, *Leveraging Camera Triplets for Efficient and Accurate
Structure-from-Motion*, CVPR 2024, pp. 4959–4968 (Algorithm 1, Eqn. 3), from the paper alone — no
code from the authors' MATLAB release or any third-party port. See `ViewGraphTriplets.{h,cpp}`,
`triplet_disambiguation.py`, `TripletFilterTest`. **Off by default** — see *The default, and why*.

## The algorithm

View graph `G = (V,E)`: nodes = images, edges = pairs with a stored two-view geometry and at least
one inlier `n_ij = ImagePair::GetNumFilteredInliers()`.

1. **Triplet graph.** The triangles of `G` are the nodes of the *triplet graph* `G_T`, two adjacent
   iff they share an edge of `G`. The edges of `G` taking part in the largest connected component of
   `G_T` form `G_LCT`; everything else — every edge in no triangle included — is **unscored**.
2. **Score.** `q^t_ij = n_ij / max_{(k,l) in t} n_kl` per triplet `t`; `q_ij` is the mean of
   `q^t_ij` over the triplets of `G_LCT` containing `(i,j)`.
3. **Threshold.** `tau = m·(1 − d_max/|V|) + d_max/|V|`, with `|V|` and `d_max` the node count and
   maximum degree **of `G_LCT`** (the paper says "of the graph"; the graph whose edges carry a score
   is `G_LCT`). `m` is the one user parameter — per the paper 0.6 generic/large-scale, 0.9 highly
   ambiguous, 0.3 medium/small ambiguous.
4. **Selection.** Keep `(i,j)` iff `q_ij >= tau` (Theorem 1: this solves the paper's regularised
   edge-selection problem); unscored pairs are removed. The paper's step 11, extracting the largest
   component of the filtered graph, is deliberately **not** applied — openMVS selects components
   itself (`SceneCluster`).

*Why it catches what the existing cycle test cannot.* `ImagePair::weightTriplet`
(`PairsWeighting.cpp`) scores a pair by how many of its triangles close rotationally. A
doppelganger's false edges are **mutually consistent** — the two near-identical façades form a
self-consistent block whose cycles do close — so a rotation-cycle test is blind to them; the
*inlier* asymmetry is what does not survive. Both scores are kept.

*Implementation.* Triangle enumeration by sorted-adjacency intersection over `i < j < k`; union-find
over triplets keyed by the shared edge; one pass for the per-triplet maximum and the per-edge
running mean. No external solver, no Boost graph, and **serial** — on the densest capture here
(377 images, 6241 pairs, 44 889 triplets) it costs a few milliseconds. Duplicate pairs collapse onto
one edge weighted by the strongest and share its score; a triangle-free graph scores nothing and the
filter empties it.

## Where it runs, and the flags

`Scene::Reconstruct` applies it **right after** the diagnostics export (`--export-pairs-csv` /
`--export-retrieval-csv`) and before `matchImagesOnly` returns, from both call sites (after
`MatchPairs`, and on the already-matched-`.sfm` early return). The CSV therefore always lists the
*whole* matched graph, each pair's score in a trailing `TripletScore` column (empty = unscored), so
a run can be re-scored offline from its own export; `ComputePairsWeights` then re-runs so the
weights describe the filtered graph. One `VERBOSE` line reports everything: `Triplet filter: kept
599/2078 pairs (tau 0.664 from m 0.60; 218 nodes, max degree 35; 8287 triplets in 4 components; 205
pairs unscored, 1274 below tau)`.

| Flag | Default | Effect |
|---|---|---|
| `--filter-triplets B` | **`false`** | apply the filter to the matched view graph |
| `--triplet-min-score F` | `0.6` | the paper's minimum edge score *m*, in (0,1); lower removes fewer edges |

Python: `TripletFilterConfig(enabled, min_score)`, `ReconstructionConfig.triplet_filter_cfg`, and `compute_triplet_scores(scene, m)` → the scores, `tau` and the graph statistics.

## Harness

`scripts/python/tests/triplet_disambiguation.py` (numpy only): `score --pairs pairs.csv -o out.csv
[-m 0.6]` writes its own score and the kept flag; `parity --pairs pairs.csv` re-implements the
scoring in NumPy and compares it with the CSV's `TripletScore` column (tolerance 1e-5); `roc --pairs
pairs.csv --labels pair_labels.csv -o roc_curve.csv` joins with `pair_gt_labels.py --mode coverage`
output, where **plausible** = true edge, **implausible** = false edge, **ambiguous** excluded.

## Measurements

Run folders live under the captures on the shared volume (never in the repo),
`<capture>/openmvs-triplet-20260830-*`, driven by `.../polycam/normal/openmvs-triplet-20260830-tools/`.
**Parity:** 7 graphs (5 SIFT + 2 dense), 2078–6241 pairs, identical unscored sets, maximum absolute
C++/Python difference **5.3e-7**. **Discrimination** vs the depth-derived labels (AUC over all
labelled pairs with unscored ranked last / over the scored pairs alone):

| graph | pairs | triplets / comps | scored | AUC (all / scored) | true kept @ m=0.6 | false kept @ m=0.6 |
|---|---|---|---|---|---|---|
| `c191135f` sift | 2078 | 8287 / 4 | 1873 | 0.848 / 0.874 | 37.2 % | 0.0 % |
| `b192978b` sift | 5537 | 29725 / 6 | 4990 | 0.829 / 0.841 | 42.1 % | 5.9 % |
| `f7dbf861` sift | 3106 | 12898 / 3 | 2850 | 0.885 / 0.896 | 37.8 % | 0.0 % |
| `3b43828e` sift | 2211 | 5917 / 20 | 1691 | 0.615 / 0.766 | 31.0 % | 5.2 % |
| `32265651` sift | 3786 | 16807 / 5 | 3223 | 0.748 / 0.802 | 32.5 % | 2.5 % |
| `32265651` dense, gate-only | 6241 | 44889 / 1 | 6061 | 0.842 / 0.847 | 51.0 % | 2.3 % |
| `32265651` dense, gate+cross-check | 4480 | 23483 / 1 | 4318 | 0.892 / 0.896 | 51.1 % | 0.8 % |

Precision among the kept pairs is 0.96–1.00 everywhere: what survives is almost purely true. The
problem is recall — `tau` was calibrated on internet photo collections, where a true edge's inlier
count varies far less than across a hand-held video capture's near/far, forward/sideways geometry —
so `m = 0.6` discards **55–74 %** of the verified pairs, `m = 0.3` still 42–65 %.

**Reconstruction effect.** Every arm reconstructs the *same* saved matched scene, so control and
filtered differ only by `--filter-triplets`. Cells: *registered images (= the largest component
here) [pairs kept] / components / `--compare-mvs` median rotation error*.

| capture (input pairs) | control | m = 0.6 | m = 0.3 |
|---|---|---|---|
| `c191135f` sift (2078) | **199** / 53 / 0.954° | 47 [599] / 205 / 0.963° | 44 [964] / 208 / 0.817° |
| `b192978b` sift (5537) | **392** / 179 / 0.903° | 303 [1860] / 268 / 0.749° | 384 [3282] / 187 / 0.727° |
| `f7dbf861` sift (3106) | **282** / 64 / 0.311° | 267 [902] / 79 / 0.333° | 275 [1505] / 71 / 0.339° |
| `3b43828e` sift (2211) | **199** / 254 / 1.873° | 138 [536] / 315 / 4.269° | 181 [911] / 272 / 1.985° |
| `32265651` sift (3786) | **309** / 69 / 0.535° | 288 [1031] / 90 / 0.645° | 295 [1904] / 83 / 0.615° |
| `32265651` dense (6241) | 131 / 23 / 0.970° | **368** [2409] / 10 / 0.706° | 340 [4054] / 38 / 0.800° |

The one place the filter is a large **win** is the repetitive capture's *dense* graph, where the
matcher's extra doppelganger edges are exactly what fragments the reconstruction: 368 of 377 images
in one component against the control's 131 (124 for the Task-4 gate arm alone), and a better median
rotation error. On the SIFT graphs it only ever costs images.

**Doppelgangers** (`.../datasets/doppelgangers/openmvs-triplet-20260830/`). `test_pairs.npy`: 4660
labelled pairs, 16 scenes, 2330/2330. `reconstructions.tar.gz` (3 GB) was downloaded and holds
**only** COLMAP reconstructions — no database with `two_view_geometries` — so there are no full view
graphs to score and `n_sift_matches` is the edge weight; only 4 of 16 scenes reach 50 triangles from
the labelled pairs alone (the other 12 — Sofia 26, Charlottenburg 19, Brno 12, the rest ≤ 7 — are
**unscorable**: 0–15 pairs scored). The paper validated on *COLMAP* view graphs: a weak proxy.

| scene | pairs | triplets | scored | AUC (all / scored) |
|---|---|---|---|---|
| Saint Alexander Nevsky, Łódź | 378 | 343 | 313 | 0.589 / 0.657 |
| Alexander Nevsky Cathedral, Tallinn | 382 | 250 | 313 | 0.611 / 0.700 |
| Cathedral of St Alexander Nevsky, Prešov | 128 | 116 | 104 | 0.528 / 0.638 |
| Washington Square Arch | 238 | 73 | 112 | 0.401 / 0.956 |

## The default, and why

The pre-registered rule made `--filter-triplets` default to true only if *all* of: no sift capture
loses more than 2 % of its registered images; none worsens its median rotation error by more than
5 %; and the dense `32265651` run registers more images in its largest component than the Task-4
gate run alone (124). At `m = 0.6` the first clause fails on **every** capture (−5.3 % to −76.4 %)
and the second on three of five (+7.1 %, +20.6 %, +128 %); only the third passes, decisively
(368 > 124). The rule is a conjunction, so **the default stays `false`** — enable it today only on a
dense-matched graph of a repetitive scene.

## Limitations and follow-ups

* **`tau` is the weak part, not the score.** The ranking is informative (AUC up to 0.90, precision
  ≈ 1.0 among the kept pairs); Eqn. 3's threshold is not calibrated for video keyframes. A
  capture-adaptive `m`, or thresholding on the score distribution instead of on `d_max/|V|`, is the
  next experiment; the exported `TripletScore` column makes it purely offline.
* **Discarding the unscored pairs is what costs the images.** Step 1 drops every edge outside the
  largest triplet-graph component, and here those are mostly *true*: 426 of the 490 unscored
  labelled pairs on `3b43828e`, 415 of 441 on `32265651`. Keeping them, or scoring each component
  separately, is worth measuring.
* **Registered-image counts on `32265651` are not a stable ranking**: Task 4 found four matcher
  configurations registering 146/124/137/192 images there with *disjoint* sets.
* **Not built here**, recorded as follow-ups: Kataria et al.'s ambiguity-aware track-length cue
  (AAM) and Wilson & Snavely's bipartite local clustering coefficient — both complementary to the
  inlier-ratio cue, both needing more than one integer per edge.
