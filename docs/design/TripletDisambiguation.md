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
  The estimator (`GlobalRotationEstimator::EstimateRotations`: the spanning-tree start, 5 rounds
  of an L1 fit by 10 ADMM iterations each at a fixed penalty of 1, then IRLS with Geman-McClure at
  5 degrees) reads the weights as relative confidences only: `SetupLinearSystem` scales them so
  their median is `GlobalRotationEstimatorOptions::weightMedian` (10, no flag). The L1 step needs
  it -- its ADMM shrinks the weighted residuals by a fixed amount within a fixed number of
  iterations, so unscaled it would sit nearer a weighted least-squares fit for heavy weights and
  nearer an L1 fit for light ones, and the cut would change with the absolute size of the pair
  weight; the spanning tree reads only their order and the IRLS cancels a common factor, so the
  whole solve is invariant to multiplying every weight by one number. The merge's block averaging
  (`AverageBlockPoses`, GlobalAlignment.cpp) runs the same estimator on its seam weights and is
  scaled the same way.
- The translation stage runs on the pairs the rotation stage kept, when its bar
  `ReconstructionConfig::maxPairDirectionResidual` (`--max-pair-direction-residual`, 45 degrees) is
  above 0. Every pair with a ray angle past `kNearDuplicateRayAngle` (2 degrees; under it a pair
  has no baseline direction of its own) has its baseline direction taken in the world frame of the
  rotation stage. The scale of every usable pair is solved from the pair-scale equations of
  `ComputePairScales` (StarInitializer.h), per component of the pair-scale graph, robustly
  (`RobustAverage`, Geman-McClure) around `GlobalScaleEstimator`; the camera centres follow from
  `C_2 - C_1 = scale * baseline` per component of at least 10 pairs, solved by
  `GlobalTranslationEstimator` reweighted towards the sum of norms (40 rounds). A pair of a
  component is judged by its own solve, any other pair by the largest solve that places both its
  images, and removed when its baseline direction is more than the bar from the centres. 0 skips
  the stage: the rotation stage alone.
- **The repair** (`ReconstructionConfig::repairContradictedPairs`, `--repair-contradicted-pairs`,
  default on; part of the translation stage, so with a bar of 0 it does not act)
  brings back a removed pair instead of losing it (`RepairContradictedPairs`, run once after both
  stages on the pairs they removed, in parallel over those pairs). The pair keeps the averaged
  rotation `R_2 R_1^T`, and `FitBaselineDirection` fits its baseline direction to its track-forming
  matches at that rotation: the epipolar constraint is linear in the direction, RANSAC over two
  matches (200 rounds) counts the matches within 1 degree of the epipolar plane, then a least
  squares of the agreeing ones and the sign that puts most of them in front of both cameras. The
  pair is repaired when at least 80% of its track-forming matches (and 8) agree, the largest
  centres solve placing both its images has their offset within 20 degrees of the fitted direction
  (no such solve: not repaired), and its two images are the same spot (joined by pairs under
  `kNearDuplicateRayAngle`) or an image of a third spot holds a kept pair with each (a block of
  look-alikes between two spots vouches only for itself). The bars do not depend on the pair's
  parallax angle. A repaired pair carries the new relative pose (E, and F between pinholes), its
  agreeing matches as its sparse and dense segments (the other sparse matches become outliers, the
  other dense ones are dropped), a ray angle measured again, and counts as kept; the weights are
  recomputed with everything else. One log line: pairs repaired of those removed and the seconds.

Cost: the stage takes 1 to 78 ms per capture at 8 threads over the 33 captures of the validation
(4 to 28 ms on the six below), at most 0.0025% of a reconstruction's wall, because only a pair that
is vouched for and placed by a centres solve is fitted.

Validation of the repair (pre-cut matched scenes reloaded at 8 threads, one run per row, the repair
off -> on): the pairs the cut removed and the pairs repaired, with the stage's own time, and the
reconstruction that follows. 6DCA1E8E ends in one of several outcomes from run to run (images 617
to 871 registered in part or not), so its row is not a comparison of the repair.

| capture | removed | repaired | stage | registered | right | wrong | pose score |
|---|---|---|---|---|---|---|---|
| 4e5c6f6d | 257 | 41 | 6 ms | 293 -> 291 | 250 -> 250 | 33 -> 32 | 0.9132 -> 0.9141 |
| 1c65aab4 | 160 | 30 | 4 ms | 513 -> 513 | 513 -> 513 | 0 -> 0 | 0.9852 -> 0.9854 |
| Palace | 1826 | 10 | 27 ms | 495 -> 495 | 494 -> 494 | 1 -> 1 | 0.8113 -> 0.8119 |
| 5ada248e | 513 | 142 | 14 ms | 534 -> 530 | 528 -> 524 | 5 -> 5 | 0.9572 -> 0.9557 |
| 17ac94cc | 496 | 147 | 11 ms | 553 -> 553 | 521 -> 521 | 8 -> 8 | 0.6415 -> 0.6404 |
| 6DCA1E8E | 1172 | 42 | 13 ms | 785 -> 725 | 464 -> 467 | 8 -> 5 | 0.7541 -> 0.7678 |

5ada248e registers 534 or 530 images from run to run without the repair too (the same four
images), and no repaired pair touches them. Truth of the repaired pairs against the reference, over
the 33 captures (1563 repaired of 15389 removed; right = rotation within 5 and direction within 15
degrees), by the pair's parallax angle:

| parallax angle (deg) | repaired | right | not right (wrong pose / not co-visible / unjudged) |
|---|---|---|---|
| under 8 | 47 | 42 | 5 |
| 8 to 20 | 280 | 262 | 18 |
| 20 and over | 1236 | 1145 | 91 |

Rejected alternatives, each replayed on the 33 captures' removed pairs: a share bar of 0.5 under 8
degrees of parallax (+315 right pairs at 92% precision, no change in any registration); the rotation
refined within a 1 or 2 degree ball before the fit (+76 right at 88%, +104 at 80%); a bar on the
epipolar residual that carries the parallax (the unnormalised triple product, 0.2 to 0.5 degrees:
precision 82 to 85%, loses 225 to 1009 right pairs); the centre-angle bar scaled by the parallax
(`DirectionTolerance`) or tightened at wide baseline (more bad pairs, or right pairs lost for few
bad ones); matches under 1 to 3 degrees of parallax not voting (+17 to +35 right pairs); a ray-angle
ceiling (-44 right pairs for 13 fewer bad ones).

Open: a pair the cut keeps at under 2 degrees of parallax is outside the repair's reach (the
translation stage skips such a pair, and the repair reconsiders only removed ones), yet on
Outdoor_004 54% of the kept pairs under 1 degree and 21% of those from 1 to 2 degrees have a baseline
direction more than 30 degrees off the reference (about 5 to 10% on 5ada248e, 6DCA1E8E and
Indoor_044).

Validation of the weight scaling (T/records-2026-11-11-weight). Invariance, the rotation stage
replayed in Python from each capture's pre-cut graph (`replay_all.py`, verdicts identical to the C++
on every capture) at the weights times 0.25, 1, 4 and 10, today's pair weight: pairs whose verdict
changes between the four scales without the scaling / with it -- Palace 0 / 0, 4e5c6f6d 0 / 0,
17ac94cc 8 / 0, 5ada248e 26 / 0, alameda 0 / 0. Against the unscaled cut at the weights as they
are, the median of 10 changes 0 verdicts on four captures and 5 (all without a reference label) on
5ada248e; between medians of 3 and 10 the verdicts move by at most 10 pairs on any of the five captures, under
either pair weight (`replay_c.md`). The
reconstruction, pre-cut matched scenes reloaded at 8 threads, one run each, unscaled -> scaled:

| capture | registered | right | wrong | pose score | wrong block starts | wrong registrations (block / pool / post-merge) |
|---|---|---|---|---|---|---|
| Palace | 495 -> 495 | 494 -> 494 | 1 -> 1 | 0.8120 -> 0.8103 | 0 -> 0 | 8 / 7 / 0 -> 8 / 7 / 0 |
| 4e5c6f6d | 291 -> 291 | 250 -> 250 | 32 -> 32 | 0.9139 -> 0.9140 | 0 -> 0 | 71 / 0 / 26 -> 71 / 0 / 26 |
| 17ac94cc | 553 -> 553 | 521 -> 521 | 8 -> 8 | 0.6398 -> 0.6443 | 0 -> 0 | 57 / 1 / 0 -> 57 / 1 / 0 |
| 5ada248e | 534 -> 530 | 528 -> 524 | 5 -> 5 | 0.9583 -> 0.9562 | 0 -> 0 | 30 / 0 / 42 -> 31 / 0 / 29 |
| Outdoor_004 | 767 -> 767 | 767 -> 767 | 0 -> 0 | 0.7190 -> 0.7196 | 0 -> 0 | 44 / 0 / 1 -> 44 / 0 / 1 |
| 6DCA1E8E | 785 -> 841 | 464 -> 464 | 8 -> 7 | 0.7541 -> 0.7527 | - -> 0 | - -> 167 / 1 / 99 |

The pose-score differences are under the 0.008 a reload of the same scene varies by; the 5ada248e
registration difference is one run (other unscaled reloads of that scene also registered 530, 524
right). 6DCA1E8E ends one of two ways from run to run (its images 617-871, which no reference judges,
registered or not): the unscaled run (a reload of another stream, without the monitor's dump) left
them out, the scaled one registered them; the images the references judge are the same 464 right. Rejected: the L1 step on unit weights (GLOMAP's default) folds Palace (998 pairs removed,
203 of them right, against 1825 / 0); no L1 step, the IRLS straight from the spanning tree, removes
45 right pairs on 5ada248e; the ADMM run to convergence folds Palace and costs up to 100 times the
iterations; weights replaced by their ranks discard how much heavier a pair is.

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
| `--repair-contradicted-pairs B` | `true` | repair instead of remove: a removed pair comes back with a baseline direction refitted at the averaged rotation when its matches, the centres and a neighbouring spot agree (does not act with the bar at 0) |

`minYield` (0.4) has no command-line flag. Python: `TripletFilterConfig` with the attributes
`enabled`, `cut`, `min_score`, `min_yield` and `second_face_score`,
`ReconstructionConfig.triplet_filter_cfg` and `ReconstructionConfig.max_pair_direction_residual`
(set outside [0,180], NaN included, it raises `ValueError`, as the command line refuses it),
and `compute_triplet_scores(scene, min_score, min_yield, grid_size)` to score a matched scene
without filtering it.

Two caveats. The contradiction cut is not idempotent: it re-applies on every reload of a matched
scene, whatever `--filter-triplets` says (unweighted when the triplet filter is off), against the
graph its earlier run already thinned, and a `--match-images-only` scene is saved after the cut, so
the pairs it removed are not in the saved scene. And with geometric verification off, no pair
carries a two-view geometry, so no pair is scored and nothing is removed.

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
