# Contradiction Cut

The contradiction cut removes the verified pairs that the rest of the view graph contradicts: a
pair whose relative rotation is not the one a robust global averaging of the pairs gives, or whose
baseline direction is not the one the camera centres those pairs place give. It exists because
repeated structure and look-alike views fabricate pairs that pass two-view verification yet cannot
be placed consistently with the rest of the graph. A repair step brings back a removed pair whose
own matches support a corrected relative pose. With `--image-sequence` the chain of the verified
neighbour pairs is trusted: a pair that contradicts it is vetoed, and none of its links is removed.
`FilterPairsByContradiction`, `RepairContradictedPairs`, `VetoPairsByChain`
(`GlobalRotationAveraging.h/cpp`).

It runs in `Scene::Reconstruct` right after the triplet scores (`TripletScores.md`), on the pairs
that hold a relative pose, in every mode: with the triplet filter off the pairs are simply unscored
and weigh 1 times their composite weight. The pair weights (`PairWeight.md`) are recomputed after
it when it removed or repaired anything. It runs once on a scene's pairs: the scene then reaches
`PAIRS_FILTERED` (`Scene::Status::STATE`), and a scene saved after it (`--match-images-only`) carries
the kept pairs, each with its repair mark, and is not cut again when a reconstruction loads it; the
cut is not idempotent (a second run would judge the graph the first already thinned). A matched
scene saved before it (`MATCHED` without `PAIRS_FILTERED`) is cut once when it is loaded. With
geometric verification off no pair carries a pose and nothing is removed.

## The weights it reads

A pair is usable when it holds a relative pose and its composite weight is above
`ReconstructionConfig::minPairWeight` (1.22, the track bar of `PairWeight.md`). Each usable pair is
weighted by its composite weight times its triplet score; an unscored pair (negative score) counts
as 1, a pair scored 0 not at all. The scores are what lets the averaging land in the true basin of
a symmetric collection: look-alike pairs between opposite images outweigh the true neighbours in
composite weight, and no longer do once the scores scale them.

## The rotation stage

1. The usable pairs are averaged one connected component at a time, the largest first. Each
   component is solved by `GlobalRotationEstimator::EstimateRotations`; the images it reaches get
   its rotations and the component's number, and the next solve takes the pairs left.
2. Every pair with a pose whose two images one solve reached, usable or not, is judged: it is
   removed when its relative rotation is more than
   `GlobalRotationEstimatorOptions::maxRelativeRotationAngle` (12 degrees) from the one the
   averaged rotations give. A pair the graph holds alone, a bridge, is fitted exactly and is never
   removed.

### The rotation averaging

`GlobalRotationEstimator::EstimateRotations` (a port of the rotation averaging of GLOMAP):

1. Start from the maximum spanning tree of the pair weights over the largest connected component of
   images (`RotationsFromMaximumSpanningTree`); the root, the node with most tree edges, is fixed at
   the identity. The tree reads only the order of the weights.
2. `SetupLinearSystem` builds the first-order system `dR_ij = dR_j - dR_i` in the angle-axis
   tangent space and scales the weights so that their median is
   `GlobalRotationEstimatorOptions::weightMedian` (10, no flag). The weights are relative
   confidences only; the scaling makes the result independent of their absolute size. The L1 step
   needs it: its ADMM shrinks the weighted residuals by a fixed amount within a fixed number of
   iterations, so unscaled it would sit nearer a weighted least-squares fit for heavy weights and
   nearer an L1 fit for light ones, and the cut would change with the absolute size of the pair
   weight. The IRLS that follows cancels a common factor, and the spanning tree reads only the
   order, so the whole solve is invariant to multiplying every weight by one number.
3. `SolveL1Regression`: up to `maxNumL1Iterations` (5) rounds. Each round fits the current weighted
   residuals with `LeastAbsoluteDeviationSolver` (ADMM, penalty `rho` 1, over-relaxation 1, at most
   10 iterations, stopping earlier when its primal and dual residuals fall under their tolerances),
   updates the rotations and stops when the average step is under 0.001.
4. `SolveIRLS`: up to `maxNumIrlsIterations` (100) iteratively reweighted least squares with the
   Geman-McClure weight `sigma^2 / (sigma^2 + e^2)^2`, `sigma` = `irlsLossParameterSigma` (5
   degrees), stopping when the average step is under 0.001.

The merge's block averaging (`AverageBlockPoses`, `HierarchicalSFM.md`) runs the same estimator
with the same options on its seam weights.

## The translation stage

It runs on the pairs the rotation stage kept when
`ReconstructionConfig::maxPairDirectionResidual` (`--max-pair-direction-residual`, 45 degrees) is
above 0; 0 skips it (`RemoveTranslationContradictions`).

1. Judged pairs: pairs not yet removed, with a pose, a mean ray angle of at least
   `kNearDuplicateRayAngle` (2 degrees; under it a pair has no baseline direction of its own), both
   images in one rotation component, and a non-zero baseline. The baseline direction is taken in the
   world frame of the rotation stage.
2. Solved pairs: the judged pairs that are usable and have a score other than 0.
3. The scale of every solved pair comes from the pair-scale equations of `ComputePairScales`
   (`IncrementalReconstruction.md`, "The star initializer"), solved per connected component of the pair-scale graph, robustly
   (`RobustAverage`, Geman-McClure at a scale bar of 1.05 in the log of the scale, `kRobustRounds`
   10 rounds) around `GlobalScaleEstimator`. Only a component of at least 10 pairs is solved; the
   components are visited largest first.
4. The camera centres follow from `C_2 - C_1 = scale * baseline` per component, solved by
   `GlobalTranslationEstimator` reweighted towards the sum of norms (40 rounds; each equation's
   weight is the inverse of its residual floored at 0.001 of the component's median pair length;
   convex, so no start decides), the equations weighted by composite weight times score.
5. A pair of a component is judged by its own solve; any other judged pair by the largest solve that
   places both its images. It is removed when the angle between its baseline direction and the
   offset of those two centres exceeds the bar.

Steps 3 and 4 are one function, `SolveScaleComponents` (`GlobalRotationAveraging.cpp`), which the
chain of an image sequence (below) runs on its own pairs and weights. With `--image-sequence` the
veto runs before step 1 and the verdict of step 5 spares the chain links (below).

### The chain of an image sequence

With `--image-sequence` (`PairSelection.md`) the images are frames of a possibly interrupted
sequence, and the neighbour pairs alone (at most the sequence overlap apart, no name break between,
`SequenceNeighbours`) place the camera centres along it: `SolveSequenceChain` runs the scale and
centres solve of steps 3 and 4 on the chain links, weighted by composite weight alone. It runs
between the two stages, when the translation stage runs (a bar of 0: no chain, the option does
nothing in the cut, and `FilterViewGraph` warns, as it does at `--match-sequence-overlap 0`). A
link is a scene pair with a relative pose whose images are neighbours, both rotated, not removed,
with a positive composite weight (the centres solve skips a weightless pair, which would leave its
images placed at the origin), a world baseline of some length (as the stage's judged pairs), a mean
ray angle of at least `kNearDuplicateRayAngle` (a frame standing still has no baseline direction),
and whose covisibility, when its matcher gave one, is at least `--sequence-min-covisibility` (0,
the test off, by default): under a bar, a neighbour pair that does not show the same place is no
link, so it splits the chain as a name break does. The covisibility is serialized with the pair, so
the bar can be changed on a reload of a scene matched with a dense matcher; a pair matched without
one (or saved before the score existed) carries none. In the cut a link also joins two images of
one rotation component (the rotations of two components share no frame), so a segment lies inside
one.

The bar is off by default because the dense matcher's own covisibility gate (`--dense-min-covisibility`,
0.5) already does its work, and the true neighbour pairs score from that gate up. Measured on 352
frames of a shopping-mall capture matched with FlexGlue: the gate refuses the look-alike pairs, the
forced neighbour pairs among them (no pair is stored across a cut between two look-alike places, and
4 far pairs are stored where SIFT verified 400), while the covisibility of the true neighbour pairs
spreads over 0.50 to 1.0 (median 0.83, 32% under 0.75). At a bar of 0.75 the chain placed 205 of
the 352 frames by 341 of 401 links; at 0 it places 328 by 633 of 638. A bar above the matcher's gate
is for a capture whose look-alike neighbour pairs pass the gate.

Each solved component of the links' pair-scale graph (at least 10 links) is a segment; a smaller
one places nothing. An image two components place belongs to the larger, a component whose every
image a larger one holds is no segment, and the segments are numbered in the order of their first
image. The chain gives per image its segment and its centre in
the segment's frame (the orientation of the rotations, a scale of its own) and the path length
along the segment through its placed images in order, per segment the median step between adjacent
placed images, and per pair the solved scale of a link: a link's offset in its segment's frame is its
world baseline times that scale. A link keeps its scale only when the component that solved it owns
both its images, so a positive scale means both images lie in one segment and the scale is in that
segment's frame; a neighbour pair joining two segments, or solved by a component other than its
images' segment, has 0, as every pair that is no link. A neighbour pair whose pose triangulates none
of its matches has no pair-scale equation, so no scale: it is no link either.

A frame standing still is placed where its neighbour stands. Where the camera stops (the end of a
capture, for one) the neighbour pairs of a frame are all under the near-duplicate angle, so no link
places it, and none of its pairs could be judged against the chain. A *still pair* is a pair that is
no link only for having no baseline direction: its mean ray angle is under `kNearDuplicateRayAngle`
or its world baseline has no length, and every other condition of a link holds (neighbours, a pose,
both images rotated, not removed, a positive weight, the covisibility bar). After the segments, the
paths and the local steps are computed, an image no segment placed that a still pair joins to a
placed image takes that image's segment, centre and path length. This is repeated pass by pass until
a pass places no image, each pass reading only the images placed before it; of several placed images
an image takes the nearest in position (image index), the lower one on a tie, so the order of the
scene's pairs decides nothing. A still pair stays no link: it has no scale, is not protected and is
judged by the veto like any pair. The local step, the path and the numbering of the segments are
those of the links alone, an image placed by standing still starts no segment (no segment solved:
nothing placed), and `SequenceChain::numStill` counts those images.

On a walk of 40 cameras 1 unit apart with exact relative poses (`SequenceChainTest`) the chain is
one segment whose centres, after a scale and translation fit, are within 1e-7 of the truth; a name
break, or a covisibility of 0.3 across the middle at a bar of 0.75, splits it into two segments of
20; a neighbour
pair of weight 0 is no link. When the walk ends with its last four cameras at one place, the last
one, whose three neighbour pairs stand still, is placed at exactly the centre and path length of
the one before it, the 108 links, the centres and the local step being those of the scene without
its pairs; with seven at one place the last is placed by the second pass.

### The veto of the chain

In a repetitive place (a corridor whose bays look alike) a pair of frames a period apart verifies
with the right rotation and a short baseline along the corridor. Its direction agrees with the
camera centres, so step 5 does not see it; enough of them out-vote the true pairs in the centres
solve, which then contradicts true pairs. The chain knows how far apart the two frames are, so the
pairs that contradict it are removed before the translation stage reads them (`VetoPairsByChain`).

A pair is judged when it is not removed by the rotation stage and is no chain link, has a pose and a
non-zero world baseline, both its images lie in one segment (placed by its links or by standing
still), and it shares at least one pair-scale equation (`ComputePairScales`, over the links and
those pairs) with a link of that segment. A neighbour pair that is no link (a covisibility under the
bar or a still pair, for two) is judged like any other pair. Every other pair is left to the two
stages.

A pair under `kNearDuplicateRayAngle` is judged like any other, though the translation stage leaves
it out because its baseline direction is noise. The strongest look-alikes are such pairs: two frames
a period apart see the same picture and verify with nearly no baseline. The rule below needs no
direction for them: a pair of length L far under the chain distance D is off the chain by at least
D - L whatever its direction, and a true near-duplicate, whose two frames the chain puts at one
place (D near 0), is off by at most L + D and is not short against D. The length comes from the
scale equations, which read the matches with a ray angle over 0.5 degrees: a pair with fewer than
15 of them in common with a link (two pictures that are the same exactly) has no equation and is
not judged.

Each equation with a link carries the link's solved scale to the pair (`s_pair = s_link * ratio`
for `ScalePair(link, pair, ratio)`, `s_link / ratio` for `ScalePair(pair, link, ratio)`); the pair's
scale is the median over them. In the segment's frame the pair's length is that scale times the
length of its pose's baseline, and its offset the scale times its world baseline. With the chain
offset the difference of the chain centres of its two images, the chain distance its length, the
path the chain's path length between the two images and the local step the segment's median step,
the pair is vetoed (`ChainVetoes`) when

- the distance between its offset and the chain offset exceeds both 8 local steps and 0.2 of the
  path (the chain drifts with its length, so a loop closure off by a share of the path is no
  contradiction), and
- its length is under a third of the chain distance (a look-alike claims that two distant frames
  stand close; a pair as long as the chain says differs from it in direction, which is step 5's to
  judge).

A vetoed pair counts as removed and is never offered to the repair: its matches agree on a
direction, and along a corridor it is the centres', so the repair would bring it back.

### The protected links

The translation stage solves with the chain links among its pairs as without the option; only its
verdict changes: a chain link (a pair of positive `SequenceChain::linkScale`) whose direction is
past the bar is kept and counted as protected. Where the centres contradict a link of the chain,
the centres are the ones in doubt.

One log line reports the chain in the cut: `Sequence chain: S segment(s) over K links; veto removed
V of J judged pairs; P chain pairs protected (veto <time>)`; `SequenceFilterStats` returns the same
five numbers. The solve itself logs one line starting `Sequence chain solve:` (the segments, the
images the links place, the links and the scale equations, the time, and at its end `, N still
image(s)`, the images placed by standing still; or that no neighbour pair is a link). At `-v 3`
the cut also writes one line per judged pair, `Chain pair: <image> <image> length <L> chain
<distance> path <path> discrepancy <d> veto <0|1> equations <N>` (N the pair-scale equations with
links its scale is the median of), and one per placed image, one placed by standing still included,
`Chain centre: <image> <segment> <x> <y> <z>`, an image named by its file name without directory
and extension (a white space replaced by `_`, its index when it has no name).

## The repair

`ReconstructionConfig::repairContradictedPairs` (`--repair-contradicted-pairs`, default on) brings
back a removed pair instead of losing it. It is part of the translation stage, because it confirms a
refitted direction against that stage's camera centres: with the bar at 0 it does not act
(`FilterPairsByContradiction` asserts `repair` only with a bar above 0, and the scene passes
`repair` only then). `RepairContradictedPairs` runs once, after both stages, on the pairs they
removed, in parallel over those pairs. A pair the chain of an image sequence vetoed is not among
them, vouches for nothing and, when it is under the near-duplicate angle, makes no spot of its two
images (step 4): it is vetoed for claiming that two distant frames stand close.

For a removed pair the repair

1. keeps the averaged rotation `R_2 R_1^T` and fits the baseline direction to the pair's
   track-forming matches at that rotation (`FitBaselineDirection`): the epipolar constraint is
   linear in the direction; RANSAC over two matches (200 rounds) counts the matches whose epipolar
   plane the direction lies within 1 degree of, then a least squares of the agreeing ones gives the
   direction and the sign that puts most of them in front of both cameras;
2. accepts the fit only when at least 80% of the track-forming matches, and at least 8, agree with
   the RANSAC's best direction;
3. requires that the largest centres solve placing both of its images has their offset within 20
   degrees of the fitted direction (no such solve: not repaired);
4. requires that its two images are the same spot (joined by pairs under `kNearDuplicateRayAngle`
   of ray angle) or that an image of a third spot holds a kept pair with each: a block of look-alikes
   between two spots vouches only for itself.

The bars do not depend on the pair's parallax angle. A repaired pair carries the new relative pose
(E, and F between pinholes), its agreeing matches as its sparse and dense segments (the other sparse
matches become outliers, the other dense ones are dropped), a ray angle measured again, and counts
as kept (`ImagePair::returnedByRepair`, serialized with the pair). Its weighted inliers are recomputed
from its new counts.

In the pair weights a repaired pair forms no triangle (`ComputePairsWeights`): it carries the
averaged rotation, so every triangle it closes with kept pairs closes by construction and says
nothing. Its own triangle share is 0, it is an edge of no other pair's triangles, and it is no
image's yardstick of the connectivity term unless it is the image's only pair. Every kept pair
therefore weighs what it weighs in the graph without the repaired pairs.

One log line reports the pairs repaired of those removed and the seconds.

## Parameters

| Parameter | CLI flag | Default | Effect |
|---|---|---|---|
| `ReconstructionConfig::maxPairDirectionResidual` | `--max-pair-direction-residual F` | 45 | translation stage bar, degrees in [0,180]; 0 = rotation stage alone |
| `ReconstructionConfig::repairContradictedPairs` | `--repair-contradicted-pairs B` | true | repair instead of remove; no effect with the bar at 0 |
| `ReconstructionConfig::minPairWeight` | none | 1.22 | the usable pairs' weight bar |
| `GlobalRotationEstimatorOptions::maxRelativeRotationAngle` | none | 12 | rotation stage verdict, degrees |
| `GlobalRotationEstimatorOptions::weightMedian` | none | 10 | median the weights are scaled to |
| `GlobalRotationEstimatorOptions::irlsLossParameterSigma` | none | 5 | Geman-McClure scale, degrees |
| `GlobalRotationEstimatorOptions::maxNumL1Iterations`, `maxNumIrlsIterations` | none | 5, 100 | solver rounds |
| repair constants (`GlobalRotationAveraging.cpp`) | none | 1 degree, 200 rounds, 8 matches, 80%, 20 degrees | epipolar band, RANSAC rounds, minimum matches, minimum agreeing share, maximum angle to the centres |
| translation constants | none | 10 pairs, 40 rounds, 0.001, 1.05 | smallest solved component, reweighting rounds, L1 floor, scale bar |
| `MatchConfig::imageSequence` | `--image-sequence B` | false | the chain of the neighbour pairs is solved, the pairs that contradict it vetoed, its links protected (needs the bar above 0) |
| `MatchConfig::matchSequenceOverlap`, `sequenceMaxGap` | `--match-sequence-overlap N`, `--sequence-max-gap N` | 3, 10 | the neighbours (`PairSelection.md`) |
| `MatchConfig::sequenceMinCovisibility` | `--sequence-min-covisibility F` | 0 | a neighbour pair carrying a covisibility below it is no link; 0 = no bar (the dense matcher's gate, 0.5, already refuses the pairs that do not show one place) |
| veto constants (`GlobalRotationAveraging.cpp`) | none | 8 steps, 0.2, 3 | tolerance in local steps, tolerance as a share of the path, the chain distance over this is the length a vetoed pair stays under |

Python: `ReconstructionConfig.max_pair_direction_residual` (set outside [0,180], NaN included, it
raises `ValueError`, as the command line refuses it). The repair flag has no Python attribute.

## Invariants

- `scores` has one entry per scene pair; the translation bar is in [0,180]; `repair` implies a bar
  above 0.
- Pairs without a pose are never judged. A pair is judged only when one rotation solve reached both
  its images.
- A bridge of the usable graph is never removed by the rotation stage.
- The pair order of the scene is kept; removal compacts the pair array in one pass and the weights
  are recomputed after it.
- The repair modifies only pairs the stages removed, and unmarks them as removed.
- Without the neighbours (`--image-sequence` off) no chain is solved and the cut is the two stages and
  the repair alone. With them and a bar of 0, or a chain too small to solve (under 10 links in every
  component), nothing is judged and nothing protected.
- A vetoed pair is never repaired, vouches for no removed pair and joins no spot of the repair. A
  chain link is never removed by the translation stage (the rotation stage runs before the chain and
  may remove a neighbour pair).
- A segment of the chain lies inside one rotation component, so a judged pair's rotations and its
  segment's centres share a frame.
- An image placed by standing still has the segment, centre and path length of an image the links
  of that segment place; the links' scales, the local step and the segments are the same with and
  without the still pairs. A still pair has no scale and is not protected.
- The translation stage reads nothing of the frames standing still: with the chain as without, it
  leaves every pair under the near-duplicate angle out of its solve and its verdict.

## Validation

Truth of the repaired pairs against the reference poses, over 33 captures (1563 pairs repaired of
15389 removed; right = rotation within 5 degrees and baseline direction within 15 degrees), by the
pair's parallax angle:

| parallax angle (deg) | repaired | right | not right (wrong pose / not co-visible / unjudged) |
|---|---|---|---|
| under 8 | 47 | 42 | 5 |
| 8 to 20 | 280 | 262 | 18 |
| 20 and over | 1236 | 1145 | 91 |

Over the 18 captures of the evaluation 679 pairs come back: 647 right, 20 wrong, 12 unjudged; 12 of
the 16 badly wrong ones (rotation over 5 or direction over 30 degrees) are vouched for only by
pairs that are themselves wrong. The repair stage takes 1 to 78 ms per capture at 8 threads over
the 33 captures, because only a pair that is vouched for and placed by a centres solve is fitted.

With the rotation weights scaled to a median of 10 the verdicts do not change with the absolute
size of the weights: replaying the rotation stage at the weights times 0.25, 1, 4 and 10, no pair
changes verdict on five captures. The end-to-end reconstructions of the pairs the cut leaves are in
`PairWeight.md`.

The chain's veto and protection, on synthetic scenes with exact poses (`SequenceVetoTest`). A
corridor whose point pattern repeats every 10 units, 60 cameras walking 30 steps out and 30 back 2
units to the side (the revisits), every pair up to 6 apart and every revisit true, and the planted
look-alikes (i, i+11) and (i, i+21), each with the true pose of (i, i+1) on matches one and two
periods on: the chain is one segment of 174 links, and of the 220 judged pairs the 28 look-alikes
are vetoed (1 step long against a chain distance of 11 and 21 steps, 10 and 20 steps off) and no
true pair is (the pairs 4 to 6 apart and the 27 revisits come out at their true length, under 1e-6
steps off the chain); without the option the cut keeps all 28, their direction being the centres'.
Through the view-graph filter, with the look-alikes of both ways (56) and a name break at the turn:
without the option the 56 stay and the translation stage removes 48 true pairs; with it the 56 are
vetoed and no true pair is removed. On the 16-camera arc a neighbour pair whose baseline is flipped,
its matches supporting the flipped pose, is removed without the option and kept with it, the one
protected link.

Pairs under the near-duplicate angle, on the same corridor. Near-duplicate look-alikes (i, i+10) of
the way out, the second frame a quarter of a step past the period or before it (a ray angle of 1.08
to 1.11 degrees): all 20 are judged and vetoed, 0.25 steps long against a chain distance of 10
steps, 9.75 and 10.25 off, with the 28 look-alikes and nothing else; without the option all 20 stay.
65 true pairs, 4 to 6 apart or revisits, with the ray angle set to 1.5 degrees and their geometry
untouched, are judged at their true length and kept. With the second frame at the period exactly no
match has a ray angle, the 20 pairs have no scale equation, are not judged and stay. With the repair
on, the vetoed near-duplicate look-alikes stay removed, and a look-alike two periods on that the
rotation stage removed, which the repair would return if the vetoed (5,15) and (15,25) made its two
images one spot, stays removed too. The corridor ending with its last six cameras at one place: 54
to 56 are placed by their links and 57 to 59 by standing still, at the centre of 56; the 56
look-alikes are vetoed, the 6 of the images 57 to 59 among them, the 9 true pairs of those images
are judged at their true length, the 15 pairs among the six have no equation and are not judged,
and no true pair is removed.

The measurement these two rules answer, by the veto before them on a capture of 1086 panoramas of a
shopping mall: it removed 1,933 of the 1,940 look-alike pairs it judged and none of 9,805 true
pairs, and left unjudged 28 look-alikes under the near-duplicate angle (ray angles of 1.3 to 1.7
degrees, 165 to 202 matches each) and 5 pairs of the last frame, where the capture ends standing
still (its three neighbour pairs at 0.6 to 0.8 degrees).

## Rejected alternatives

Repair:

- A minimum number of vouching images (3): gives up 38 right pairs for 8 wrong ones removed.
- A share bar of 0.5 under 8 degrees of parallax: +315 right pairs at 92% precision, no change in
  any registration.
- The rotation refined within a 1 or 2 degree ball before the fit: +76 right at 88%, +104 at 80%.
- A bar on the epipolar residual that carries the parallax (the unnormalised triple product, 0.2 to
  0.5 degrees): precision 82 to 85%, loses 225 to 1009 right pairs.
- The centre-angle bar scaled by the parallax, or tightened at wide baseline: more bad pairs, or
  right pairs lost for few bad ones.
- Matches under 1 to 3 degrees of parallax not voting: +17 to +35 right pairs.
- A ray-angle ceiling: -44 right pairs for 13 fewer bad ones.
- Counting a repaired pair in the triangles of the other pairs: 250 right / 28 wrong images on
  4e5c6f6d against 268 / 3 with the rule above.

Rotation averaging:

- The L1 step on unit weights (GLOMAP's default): folds Palace (998 pairs removed, 203 of them
  right, against 1825 removed and none right).
- No L1 step, the IRLS straight from the spanning tree: removes 45 right pairs on 5ada248e.
- The ADMM run to convergence: folds Palace and costs up to 100 times the iterations.
- Weights replaced by their ranks: discards how much heavier a pair is.

A spanning-tree rotation check of the pairs (a residual bar on the rotations of their maximum spanning tree): the rotation stage judges the same pairs against a robust averaging of all the usable pairs instead of their tree.

Weighting:

- The cut weighted without the triplet score: loses 78 right junction pairs of Palace (`PairWeight.md`).
- The triplet score raised to a power in the cut: no more false pairs removed.

## Open items

- The veto and the protection of the chain are validated on synthetic scenes only (above), the
  judging of the pairs under the near-duplicate angle and the frames standing still among them.
- A pair without a pair-scale equation with a link of its segment is not judged: fewer than 15
  matches over 0.5 degrees of ray angle in common with a link (two pictures the same exactly).
- A frame standing still is put exactly where its neighbour stands: a stretch of frames creeping
  under the near-duplicate angle is placed at one point, or at its two ends when links place the
  frames on both sides, and its pairs are judged against that.
- A wrong neighbour link is trusted: it is protected, and the true pairs that disagree with a chain
  it bent can be vetoed; a vetoed pair is final (never repaired).
- Protection does not depend on the link agreeing with the chain's own centres.
- Look-alike pairs between two segments are not judged (both images must lie in one segment).
- The rotation stage runs before the chain and may remove a neighbour pair, which then is no link.
- A pair the cut keeps at under 2 degrees of parallax is outside the repair's reach (the
  translation stage skips such a pair, and the repair reconsiders only removed ones). On a panorama
  capture 54% of the kept pairs under 1 degree and 21% of those from 1 to 2 degrees have a baseline
  direction more than 30 degrees off the reference; on three other captures about 5 to 10%.
- The cut judges a pair only against the rotations and camera centres the rest of the graph gives,
  so a collection whose false pairs outnumber and outweigh its true ones even after the scores can
  still be averaged into the wrong arrangement (`TripletScores.md`, the cutting rule).
