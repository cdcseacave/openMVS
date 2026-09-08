# Camera-Triplet View-Graph Disambiguation

A dependency-free filter of the matched view graph, on by default, that runs right after pair
matching and before reconstruction. It reimplements S. M. Manam and V. M. Govindu, "Leveraging
Camera Triplets for Efficient and Accurate Structure-from-Motion", CVPR 2024 (Algorithm 1, Eqn. 3),
removing wrong pairs -- repeated structure, retrieval false positives -- using nothing but the
graph itself and one strength per edge. `ViewGraphTriplets.h/cpp`.

## The algorithm

The view graph has the images as nodes and the geometrically verified pairs as edges, each carrying
a strength: its weighted inlier count discounted by the fraction of the frame the inliers cover, on
the pair weighting's grid. The triangles of the view graph form the triplet graph, two triangles
adjacent when they share an edge; the edges taking part in the largest connected component of the
triplet graph are scored by the mean, over the edge's triangles, of `s_ij / max s_kl` -- a true edge
is comparable to the strongest edge of every triangle it belongs to, a false edge is the weak side
of the triangles built around true edges. The threshold is `tau(m) = m(1 - r) + r`, with
`r = d_max / |V|` of the scored graph and `m` the one user parameter (0.3 by default, the paper's
value for medium and small ambiguous sets; 0.6 generic, 0.9 highly ambiguous). Unscored pairs -- no
triangle, a smaller component, no stored matches -- carry no evidence and are kept at every
threshold.

A triangle whose three pairs all deliver below `minYield` (0.4) of what pairs at their ray angle
deliver elsewhere in this graph gives no evidence either, per the graph's own 90th-percentile
envelope over 1-degree bins of ray angle: three look-alikes vouching for one another is not
evidence that any of them is true.

## Two modes

**Keep mode**, the default. The ceiling `tau(m)` only names the pieces: the connected components
the matched graph falls into once every scored pair below it is provisionally removed. The
candidates are the scored pairs below the ceiling that join two of those pieces -- a weak pair
inside one piece is kept whatever its score. Of the candidates only what the graph can spare goes:
every image keeps a floor of `keepPairs` pairs holding `keepMatches` weighted inliers, counting
only pairs whose ray angle reaches `keepMinAngle` degrees (an unmeasured angle counts; a
near-duplicate pair yields no 3D point); the threshold is the strictest one at or below the ceiling
at which at most `keepMaxShort` of the images fall short of that floor from their non-candidate
pairs, and a graph fitting none loses nothing. Last, the repair: every component of the matched
graph stays one component, joined back by its best-scoring candidates. A graph already in one piece
at the ceiling loses nothing either.

**Cutting rule** (`--triplet-cut`), the paper's own rule. Every scored pair below the threshold
goes. A second, stricter ceiling at the second-face score (0.75) is tried first: when the graph it
leaves has two faces -- a majority piece and a second one of at least a third of it -- the smaller
face is cut off, together with the pairs of any image outside both faces whose kept pairs reach
both, and the paper's ceiling then applies inside the larger face alone. Otherwise the descent
decides: a ceiling whose largest piece already holds a strict majority applies as given, and a
ceiling that shatters the graph descends to the strictest threshold joining every piece, where a
piece is a component of at least 1% of the largest component.

In both modes the reconstruction seeds in the largest piece the ceiling leaves. Use the default: it
never removes a pair inside a piece, so it never hurts a scene without repeated structure (orbits,
interiors). Use `--triplet-cut` to unfold a symmetric building or a symmetric object orbited by a
video, at the price of halving the registrations of an interior.

## Where it runs, and the flags

`Scene::Reconstruct` applies the filter right after the diagnostics export (`--export-pairs-csv`,
whose `TripletScore` column carries every pair's score, so the CSV lists the whole matched graph)
and before `--match-images-only` returns. The pair weights are recomputed afterward when anything
was removed.

| Flag | Default | Effect |
|---|---|---|
| `--filter-triplets B` | `true` | run the filter |
| `--triplet-cut B` | `false` | the cutting rule instead of the keep mode |
| `--triplet-min-score F` | `0.3` | the paper's minimum edge score `m` |
| `--triplet-keep-pairs N` | `3` | keep mode: pairs every image keeps at least |
| `--triplet-keep-matches N` | `2000` | keep mode: weighted inliers every image keeps at least |
| `--triplet-keep-min-angle F` | `3` | keep mode: minimum ray angle, in degrees, for a pair to count towards the floor |
| `--triplet-keep-max-short F` | `0.5` | keep mode: largest share of images the threshold may leave short of the floor |
| `--triplet-second-face-score F` | `0.75` | cutting rule: the stricter ceiling that names the two faces |

`minYield` (0.4) has no command-line flag. Python: `TripletFilterConfig` with the attributes
`enabled`, `cut`, `min_score`, `min_yield`, `keep_pairs`, `keep_matches`, `keep_min_angle`,
`keep_max_short` and `second_face_score`,
`ReconstructionConfig.triplet_filter_cfg`, and `compute_triplet_scores(scene, min_score, min_yield,
grid_size)` to score a matched scene without filtering it.

Two caveats. The filter is not idempotent: a saved, already-filtered scene re-fed with
`--filter-triplets true` is filtered again, against its own already-thinned graph. And with
geometric verification off, no pair carries a two-view geometry, so no pair is scored and nothing is
removed.

## Limitations

The keep mode never separates two faces of a symmetric collection and never removes a false pair
inside a piece, so a two-faced collection, or one that is nearly one piece at the ceiling, stays
folded under the default and is run with `--triplet-cut`. Under the cutting rule, the pieces the
ceiling leaves apart stay unregistered: one model per run, not one per piece. A building whose two
long sides are near-identical and matched more densely than its true corners is beyond any
pairwise-geometry method -- nothing in the graph's structure tells the sides from the corners. And
the wide-baseline bias: on a video, every non-consecutive pair is the weak side of a triangle a
consecutive pair tops, which is why the keep mode is built to never remove a pair inside a piece.
