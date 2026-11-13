# Triplet Scores

A dependency-free scoring of the matched view graph, on by default, that runs right after pair
matching and before reconstruction (`FilterPairsByTriplets`, `ComputeTripletScores`,
`ViewGraphTriplets.h/cpp`). It implements S. M. Manam and V. M. Govindu, "Leveraging Camera
Triplets for Efficient and Accurate Structure-from-Motion", CVPR 2024 (Algorithm 1, Eqn. 3), and
scores every verified pair so that repeated structure and retrieval false positives score low,
using only the graph itself and one strength per edge.

By default the scores remove nothing. They have two consumers: the contradiction cut that follows
(`ContradictionCut.md`) weighs each pair by its score, and the reconstruction seeds in the images
of the largest piece the score threshold leaves (the seed views). `--triplet-cut` applies the
paper's own rule, in which every pair below the threshold is removed.

The per-pair triangle-consistency term of the pair weight (`ImagePair::weightTriplet`,
`PairWeight.md`) is a different quantity: it counts triangles whose rotations close, where this
score compares the pair's inlier strength with the other two pairs of each triangle.

## The score

`BuildViewGraph` builds the graph: the images are nodes and the geometrically verified pairs that
hold at least one weighted inlier are edges, one edge per distinct image pair (two scene pairs of
the same images share the edge, described by the stronger). An edge's strength is its weighted
inlier count (`ImagePair::GetNumWeightedInliers`) times its coverage (`ComputePairCoverage` on the
`PairsWeightingConfig::gridSize` grid, 10 x 10; the share of the frame the track-forming matches
occupy, `PairWeight.md`). An edge without stored matches has strength 0 and is never scored.

`ScoreEdges` (Algorithm 1, steps 1-9):

1. The triangles of the graph are enumerated by intersecting sorted adjacency lists; they are
   streamed, never stored, so memory is O(edges).
2. Two triangles are adjacent when they share an edge; the connected components of this triplet
   graph are found with a union-find on the edges. The largest component by triangle count is the
   scored graph; ties go to the smallest root.
3. Every edge of the scored graph is scored by the mean, over the triangles it belongs to, of
   `s_ij / max(s_kl)` over the triangle's three strengths: a true edge is comparable to the
   strongest edge of every triangle it belongs to, a false edge is the weak side of triangles built
   around true edges. The score is in [0,1].
4. The threshold is `tau(m) = m (1 - r) + r`, with `r = d_max / |V|` of the scored graph (its
   maximum degree over its node count) and `m` the one user parameter. Without a scored graph `tau`
   is `m`.

Pairs that carry no evidence stay unscored (`TripletScores::unscored`, -1) and are kept at every
threshold: a pair in no triangle, in a smaller triplet component, or without stored matches. The
scores do not depend on `m`; only `tau` does.

### The yield rule

A triangle of three look-alike pairs vouches for itself without being evidence. `ComputeYields`
measures, for every edge, what it delivers against what pairs at its ray angle deliver in this
graph:

- `u_e = n_e / min(K_i, K_j)`, `n_e` the edge's weighted inliers and `K_i` image i's yardstick, the
  count of its strongest pair that is not a near-duplicate (`Yardstick`, `PairsWeighting.h`; a
  near-duplicate only when the image has no other pair).
- The envelope `H` is the 90th percentile of `u` over the edges of each 1-degree bin of the edge's
  median ray angle (90 degrees and beyond share the last bin), over the bins holding at least five
  edges, made non-increasing in the angle by a suffix maximum: a narrower viewpoint never promises
  less than a wider one. An empty bin takes the value of the nearest populated bin above it, and the
  bins above the highest populated one keep its value. With no populated bin every edge yields 1.
- The yield is `min(1, u_e / H)`. An edge without a measured ray angle takes no part in the
  envelope and yields 1.

A triangle whose three edges all yield below `minYield` (0.4) contributes nothing to its edges'
sums but still counts in the mean's denominator, so a pair held only by such triangles scores low.
`minYield` 0 turns the rule off.

## The seed views

The ceiling `tau(m)` cuts the graph into pieces: the connected components of the graph kept at the
ceiling (an unscored edge is kept, a scored edge is kept at or above the ceiling), restricted to
the images of the largest connected component of the unfiltered graph and to components of at
least 1% of that component's nodes (every component when the ceiling leaves none). The images of
the largest piece are the seed views, returned in every mode. `Scene::ReconstructHierarchical`
hands each sub-scene its share of them and the star initializer chooses its reference view among
them (`StarInitialization.md`): after a cutting rule the seed's side of a symmetric building is the
model, and the heaviest image overall sits in the densest cluster of look-alike views.

## The cutting rule (`--triplet-cut`)

Every scored pair below the final threshold is removed; unscored pairs stay. The final threshold
is found in two parts.

1. Second face. When the second-face score `secondFaceScore` (0.75) is above `m`, a stricter
   ceiling `secondFaceScore (1 - r) + r` is tried first. When the pieces it leaves are two faces,
   that is a majority piece (more than half of the pieced nodes) and a second piece of at least a
   third of it, the smaller face is cut off: every pair joining it to an image outside it is
   removed, and so is every pair of an ambiguous image, one outside both faces whose kept pairs at
   the paper's ceiling reach both. The paper's ceiling then applies inside the larger face alone,
   and the descent below does not run.
2. Descent. Otherwise, a ceiling `tau(m)` whose largest piece already holds a strict majority of
   the pieced nodes applies as given. A ceiling that shatters the graph (small sets and every
   exhaustively matched collection, where `d_max / |V|` is near 1) descends to the strictest
   threshold, among the distinct scores below the ceiling, at which the graph kept joins every
   piece again (found by bisection, the test being monotone); components too small to be pieces
   are neither chased nor removed.

The removed pairs leave the scene, the pair weights are recomputed, and the scores handed to the
contradiction cut are those of the pairs left (recomputed on the thinned graph, since a lost
triangle changes the score of the pairs beside it).

## Parameters

| Parameter (`TripletFilterConfig`) | CLI flag | Default | Effect |
|---|---|---|---|
| `enabled` | `--filter-triplets B` | true | score the matched view graph (and, with `cut`, filter it); off: every pair is unscored and the contradiction cut runs unweighted |
| `cut` | `--triplet-cut B` | false | the cutting rule |
| `minScore` | `--triplet-min-score F` | 0.3 | the paper's `m`, in [0,1]: 0.3 for medium and small ambiguous sets, 0.6 generic, 0.9 highly ambiguous |
| `secondFaceScore` | `--triplet-second-face-score F` | 0.75 | cutting rule: the stricter minimum score whose ceiling names the two faces; at or below `minScore` the faces are not looked for |
| `minYield` | none | 0.4 | the yield rule's bar; 0 = off |

`minScore`, `minYield` and `secondFaceScore` are clamped into [0,1] with a warning in
`FilterPairsByTriplets`; the command line refuses `--triplet-min-score` and
`--triplet-second-face-score` outside [0,1].

Python: `TripletFilterConfig` with `enabled`, `cut`, `min_score`, `min_yield` and
`second_face_score`, set on `ReconstructionConfig.triplet_filter_cfg`; and
`compute_triplet_scores(scene, min_score, min_yield, grid_size)` returns the scores of a matched
scene (one per pair, -1 unscored) with `tau` and the statistics of the scored graph, without
filtering it. Its `min_score` defaults to 0.

`Scene::Reconstruct` applies the filter right after the diagnostics export (`--export-pairs-csv`,
whose `TripletScore` column carries every pair's score, so the CSV lists the whole matched graph),
then the contradiction cut, both before `--match-images-only` returns. A matched scene that is
loaded again passes through both again. With geometric verification off no pair carries a
two-view geometry, so no pair is scored.

Use the default for a scene without repeated structure (orbits, interiors). Use `--triplet-cut` to
unfold a symmetric building, or a symmetric object orbited by a video, at the price of halving the
registrations of an interior whose rooms it cuts apart.

## Invariants

- `ComputeTripletScores` scores only the edges of the largest triplet component; every other pair
  is unscored.
- `scores` has one entry per scene pair, in [0,1] or `unscored`; a pair removed by the face rule is
  marked internally and never reaches the consumers.
- The default mode removes no pair.
- The seed views are ascending image indices, empty when the filter is off or the graph has no edge.

## Rejected alternatives

- A minimum score of 0.75 as the default instead of the paper's 0.3: tried and reverted.
- A threshold sweep that relaxes the threshold until the graph holds together: replaced by the
  strictest threshold at which the graph kept joins every piece.
- Removing pairs by score as the default: only the cutting rule does, and it halves the
  registrations of an interior; the default hands the scores to the contradiction cut instead.

## Open items and limitations

- The scores alone cannot remove a pair: a collection whose false pairs outnumber and outweigh its
  true ones even after the scores can still be averaged into the wrong arrangement by the
  contradiction cut, and a two-faced collection whose cross-face pairs are mutually consistent
  stays folded and is run with `--triplet-cut`.
- Under the cutting rule the pieces the ceiling leaves apart stay unregistered: one model per run,
  not one per piece.
- A building whose two long sides are near-identical and matched more densely than its true
  corners is beyond any pairwise-geometry method: nothing in the graph's structure tells the sides
  from the corners.
- Wide-baseline bias: on a video every non-consecutive pair is the weak side of a triangle a
  consecutive pair tops, which is why only the cutting rule removes pairs by score and the default
  mode never does.
