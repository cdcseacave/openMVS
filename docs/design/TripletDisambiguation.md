# Camera-Triplet View-Graph Disambiguation

A dependency-free scoring of the matched view graph, on by default, that runs right after pair
matching and before reconstruction. It reimplements S. M. Manam and V. M. Govindu, "Leveraging
Camera Triplets for Efficient and Accurate Structure-from-Motion", CVPR 2024 (Algorithm 1, Eqn. 3),
scoring every pair -- repeated structure and retrieval false positives score low -- using nothing but
the graph itself and one strength per edge. `ViewGraphTriplets.h/cpp`. By default the scores remove
nothing themselves: they weigh the contradiction cut that follows (below), which removes the wrong
pairs; `--triplet-cut` applies the paper's own rule, every pair below the threshold goes.

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

## The default mode, the cutting rule and the contradiction cut

**Default mode.** The filter removes nothing. It scores every pair, names the seed views -- the
images of the largest piece the ceiling `tau(m)` leaves, the pieces being the connected components
the matched graph falls into once every scored pair below it is provisionally removed -- and hands
the scores to the contradiction cut. The reconstruction seeds in that largest piece.

**Cutting rule** (`--triplet-cut`), the paper's own rule. Every scored pair below the threshold
goes. A second, stricter ceiling at the second-face score (0.75) is tried first: when the graph it
leaves has two faces -- a majority piece and a second one of at least a third of it -- the smaller
face is cut off, together with the pairs of any image outside both faces whose kept pairs reach
both, and the paper's ceiling then applies inside the larger face alone. Otherwise the descent
decides: a ceiling whose largest piece already holds a strict majority applies as given, and a
ceiling that shatters the graph descends to the strictest threshold joining every piece, where a
piece is a component of at least 1% of the largest component. The scores handed on are those of
the pairs it leaves.

**The contradiction cut** (`FilterPairsByContradiction`, `GlobalRotationAveraging.h/cpp`) runs after
the triplet filter in either mode, on the pairs holding a relative pose, and removes the pairs the
rest of the graph contradicts, in two stages.

- The rotation stage averages the relative rotations of the usable pairs -- a pose and a composite
  weight above `minPairWeight` -- each weighted by its composite weight times its triplet score (an
  unscored pair weighs 1, a pair scored 0 not at all), by `GlobalRotationEstimator` from the maximum
  spanning tree of those weights, one connected component at a time, the largest first. Every pair
  with a pose whose two images one solve reached, usable or not, is removed when its relative
  rotation is more than `GlobalRotationEstimatorOptions::maxRelativeRotationAngle` (12 degrees)
  from the averaged rotations. A pair the graph holds alone, a bridge, is fitted exactly and never
  removed. The scores are what lets the averaging land in the true basin of a symmetric collection:
  look-alike pairs between opposite images outweigh the true neighbours in composite weight, and
  no longer do once the scores scale them.
- The translation stage runs on the pairs the rotation stage kept, when its bar
  `ReconstructionConfig::maxPairDirectionResidual` (`--max-pair-direction-residual`, 45 degrees) is
  above 0. Every pair with a ray angle past `kNearDuplicateRayAngle` (2 degrees; under it a pair
  has no baseline direction of its own) has its baseline direction taken in the world frame of the
  rotation stage. The scale of every usable pair is solved from the pair-scale equations of
  `ComputePairScales` (StarInitializer.h), per component of the pair-scale graph, robustly
  (`RobustAverage`, Geman-McClure) around the sparse route of `GlobalScaleEstimator`; the camera
  centres follow from `C_2 - C_1 = scale * baseline` per component of at least 10 pairs, solved by
  `GlobalTranslationEstimator` reweighted towards the sum of norms (40 rounds). A pair of a
  component is judged by its own solve, any other pair by the largest solve that places both its
  images, and removed when its baseline direction is more than the bar from the centres. 0 skips
  the stage: the rotation stage alone.

Use the default for a scene without repeated structure (orbits, interiors). Use `--triplet-cut` to
unfold a symmetric building or a symmetric object orbited by a video, at the price of halving the
registrations of an interior.

## Where it runs, and the flags

`Scene::Reconstruct` applies the filter right after the diagnostics export (`--export-pairs-csv`,
whose `TripletScore` column carries every pair's score, so the CSV lists the whole matched graph),
then the contradiction cut, and both before `--match-images-only` returns. The pair weights are
recomputed after either when it removed anything. With `--filter-triplets false` every pair is
unscored and the contradiction cut runs unweighted.

| Flag | Default | Effect |
|---|---|---|
| `--filter-triplets B` | `true` | score the view graph (and, with `--triplet-cut`, filter it) |
| `--triplet-cut B` | `false` | the cutting rule: every pair below the threshold goes |
| `--triplet-min-score F` | `0.3` | the paper's minimum edge score `m` |
| `--triplet-second-face-score F` | `0.75` | cutting rule: the stricter ceiling that names the two faces |
| `--max-pair-direction-residual F` | `45` | the contradiction cut's translation stage: pairs whose baseline direction is more than this many degrees from the camera centres go; 0 = the rotation stage alone |

`minYield` (0.4) has no command-line flag. Python: `TripletFilterConfig` with the attributes
`enabled`, `cut`, `min_score`, `min_yield` and `second_face_score`,
`ReconstructionConfig.triplet_filter_cfg` and `ReconstructionConfig.max_pair_direction_residual`,
and `compute_triplet_scores(scene, min_score, min_yield, grid_size)` to score a matched scene
without filtering it.

Two caveats. The filter is not idempotent: a saved, already-filtered scene re-fed with
`--filter-triplets true` is filtered again, against its own already-thinned graph. And with
geometric verification off, no pair carries a two-view geometry, so no pair is scored and nothing is
removed.

## Limitations

The default mode removes nothing by score, and the contradiction cut judges a pair only against
the rotations and camera centres the rest of the graph gives, so a collection whose false pairs
outnumber and outweigh its true ones even after the scores can still be averaged into the wrong
arrangement; a two-faced collection whose cross-face pairs are mutually consistent stays folded
and is run with `--triplet-cut`. Under the cutting rule, the pieces the ceiling leaves apart stay
unregistered: one model per run, not one per piece. A building whose two long sides are
near-identical and matched more densely than its true corners is beyond any pairwise-geometry
method -- nothing in the graph's structure tells the sides from the corners. And the wide-baseline
bias: on a video, every non-consecutive pair is the weak side of a triangle a consecutive pair
tops, which is why only the cutting rule removes pairs by score and the default mode never does.
