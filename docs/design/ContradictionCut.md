# Contradiction Cut

The contradiction cut removes the verified pairs that the rest of the view graph contradicts: a
pair whose relative rotation is not the one a robust global averaging of the pairs gives, or whose
baseline direction is not the one the camera centres those pairs place give. It exists because
repeated structure and look-alike views fabricate pairs that pass two-view verification yet cannot
be placed consistently with the rest of the graph. A repair step brings back a removed pair whose
own matches support a corrected relative pose. When the images are frames of a sequence, the camera
path their neighbour pairs place vetoes the pairs that contradict it, between the two stages ("The
chain of an image sequence"). `FilterPairsByContradiction`, `RepairContradictedPairs`
(`GlobalRotationAveraging.h/cpp`); the chain is `ApplySequenceChain` (`SequenceChain.h/cpp`). The
stages share, through the internal header `ContradictionCut.h`, one state per pair (`PairState`: `KEPT`, `LINK`, `CUT` by a stage, `VETOED` by the
chain) and one world baseline per pair (`ComputeWorldBaselines`: none for a pair without a pose or
whose two images do not lie in one rotation component).

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

Steps 3 and 4 are one function, `SolveScaleComponents` (`ContradictionCut.h`), given the
equations of step 2; the chain of an image sequence (below) runs it on its own pairs and weights.

## The chain of an image sequence

In a repetitive place (a corridor whose bays look alike) two frames a period apart verify with the
right rotation and a short baseline whose direction the camera centres agree with, so neither stage
removes them, and enough of them out-vote the true pairs in the centres solve. When the images are
frames of a sequence, the sequence knows better: its neighbour pairs place the frames along the
capture, and a pair that contradicts that placement is vetoed (`ApplySequenceChain`,
`SequenceChain.h/cpp`). The chain runs between the two stages, on the pairs the rotation stage kept,
in a sequence run (`MatchConfig::IsImageSequence()`; `FilterViewGraph` passes
`SequenceNeighbours::FromConfig`) and only when the translation stage runs (a bar above 0).

### Terms

- *Neighbours*: two images at most `--match-sequence-overlap` positions apart with no name break
  between them (`SequenceNeighbours`; `PairSelection.md`, "Sequence neighbours").
- *Candidate link*: a pair of neighbours the rotation stage kept (`PairState::KEPT`), with a world
  baseline, a positive composite weight (the centres solve skips a weightless pair, which would
  leave its images at the origin) and a baseline direction (`HasBaselineDirection`: a baseline of
  some length and a mean ray angle of at least `kNearDuplicateRayAngle`, 2 degrees).
- *Still pair*: a pair that meets every condition of a candidate link but the baseline direction:
  the camera stands still. It still has a pose and both images in one rotation component.
- *Scale component*: a connected component of the pair-scale graph of the candidate links (the
  equations of `ComputePairScales`), as distinct from a rotation component of the rotation stage.
- *Segment*: a scale component the solve places (at least 10 candidate links, `minScaleComponent`),
  with a frame of its own: the orientation of the rotations and a scale of its own. A world baseline
  joins two images of one rotation component, so a segment lies inside one.
- *Link*: a candidate link the chain solved: it has a scale in the segment both its images lie in
  (`SequenceChain::linkScale` above 0, `PairState::LINK`). A candidate link the chain did not solve
  is an ordinary kept pair.
- *Still image*: an image no segment places that a still pair joins to an image a segment places.
- For two images of one segment: the *chain offset* is the difference of their centres in the
  segment's frame and the *chain distance* its length; the *path* is the difference of their path
  lengths, an image's path length being the distance from the segment's first image through its
  placed images in index order; the *local step* is the segment's median distance between
  consecutive placed images.

### The solve

`SolveSequenceChain` runs `SolveScaleComponents` on the candidate links alone, weighted by
composite weight (no triplet score). Each solved scale component, the largest first, places its
images: an image two components place belongs to the larger, a component whose every image a larger
one holds is no segment, and the segments are numbered in the order of their first image. A
candidate link keeps its solved scale only when both its images belong to the segment that solved
it, and is then a link; its offset in that frame is its world baseline times that scale. A
candidate link between two segments, in a scale component under 10 candidate links, or without a
pair-scale equation stays `KEPT`.

The paths and the local steps are those of the images the links place. Then each still image takes
the segment, centre and path length of the placed image a still pair joins it to, the nearest in
position and the lower on a tie, pass by pass until a pass places none, each pass reading only the
images placed before it; the order of the scene's pairs decides nothing. A still pair has no scale,
and the segments, the scales of the links and the local steps are those of the links alone
(`SequenceChain::numStill` counts the still images).

### The veto

A pair is judged when it is `KEPT` (not removed by the rotation stage, and no link), has a world
baseline of some length, both its images lie in one segment (placed by its links or as still
images), and it shares at least one pair-scale equation with a link of that segment
(`ComputePairScales` given the links as `anchors`: only the equations between a link and a pair
that is none). A candidate link the chain did not solve and a still pair are judged like any other
pair; every other pair is left to the two stages.

Each equation carries the link's scale to the pair (`s_pair = s_link * ratio` for
`ScalePair(link, pair, ratio)`, `s_link / ratio` for `ScalePair(pair, link, ratio)`), and the
pair's scale is the median over them. In the segment's frame the pair's offset is that scale times
its world baseline and its length L that scale times the baseline's length. With d the distance
between the pair's offset and the chain offset, D the chain distance, P the path and s the local
step, `ChainVetoes` vetoes the pair when

    d > max(8 s, 0.2 P)  and  L < D / 3

(`kVetoLocalSteps`, `kVetoPathShare`, `kVetoShortDivisor` in `SequenceChain.cpp`). The path term
tolerates the chain's drift; a pair about as long as D that differs from it in direction is the
translation stage's to judge.

A pair under `kNearDuplicateRayAngle` is judged too, though the translation stage leaves it out:
the strongest look-alikes are such pairs, two frames a period apart that see the same picture. The
rule needs no direction for them: a pair of length L far under D is off the chain by at least
D - L whatever its direction, while a true near-duplicate, whose images the chain puts at one place,
is not short against D. Its length comes from the scale equations, which read the matches over 0.5
degrees of ray angle; a pair with fewer than 15 of them in common with a link has no equation.

A vetoed pair (`PairState::VETOED`) counts as removed and is never offered to the repair: its
matches agree on a direction, and along a corridor it is the centres', so the repair would bring it
back. It vouches for no removed pair and makes no spot of its two images (see "The repair").

### The protection

The translation stage solves with the links among its pairs as without the chain; only its verdict
changes: a link whose direction is past the bar is kept. A candidate link the chain did not solve is
not protected. Where the centres contradict a link, the centres are the ones in doubt.

### Log

Each step logs its own work. The solve: `Sequence chain solve: S segment(s) placing N images by K
links of M candidates (C scale components, E equations), T still image(s) (<time>)`, N the images
the links place, or `Sequence chain solve: no link among the neighbour pairs (<time>)`. The veto:
`Sequence chain veto: removed V of J judged pairs (<time>)`. The translation stage, when it kept
any: `Translation cut: kept P link(s) of the sequence chain it would have removed`.
`SequenceFilterStats` returns the counts. At `-v 4` each vetoed pair adds `Vetoed pair: <image>
<image> length <L> chain <D> path <P> discrepancy <d> equations <n>`, the lengths in the segment's
frame and n the equations its scale is the median of, an image named by its file name without
directory and extension (a white space replaced by `_`, its index when it has no name). A pair
judged and kept has no line: a capture has thousands of them.

## The repair

`ReconstructionConfig::repairContradictedPairs` (`--repair-contradicted-pairs`, default on) brings
back a removed pair instead of losing it. It is part of the translation stage, because it confirms a
refitted direction against that stage's camera centres: with the bar at 0 it does not act
(`FilterPairsByContradiction` asserts `repair` only with a bar above 0, and the scene passes
`repair` only then). `RepairContradictedPairs` runs once, after both stages, on the pairs they
removed, in parallel over those pairs. A pair the chain of an image sequence vetoed is not among
them, vouches for nothing and makes no spot of its two images (step 4): it is vetoed for claiming
that two distant frames stand close.

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
| `MatchConfig::imageSequence` | `--image-sequence B` | false (on with `--match-mode 2`) | the chain of an image sequence, with the options of its neighbours (`PairSelection.md`, "Parameters and defaults"); needs the bar above 0 |
| veto constants (`SequenceChain.cpp`) | none | 8, 0.2, 3 | `kVetoLocalSteps`, `kVetoPathShare`, `kVetoShortDivisor` of `ChainVetoes` |

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
- Without the neighbours, with a bar of 0, or with no segment, the chain judges and protects
  nothing: the cut is the two stages and the repair alone.
- A vetoed pair is never repaired, vouches for no removed pair and joins no spot of the repair; a
  link is never removed by the translation stage.
- A segment lies inside one rotation component. A still image has the segment, centre and path
  length of an image the links place; the segments, the scales of the links and the local steps are
  the same with and without the still pairs.

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

The chain, on synthetic scenes with exact poses. On a walk of 40 cameras 1 unit apart
(`SequenceChainTest`) it is one segment whose centres, after a scale and translation fit, are within
2% of the walk's length of the truth; a name break splits it into two segments of 20, and a walk
ending with its last cameras at one place has its last image placed exactly where the image before
it stands (seven at one place: by a second pass). On a corridor whose point pattern repeats every
10 units, 60 cameras walking out and back (`SequenceVetoTest`), the chain is one segment of 174
links and of its 220 judged pairs exactly the 28 planted look-alikes (one step long, 11 and 21 steps
apart on the chain) are vetoed; without the option the cut keeps all 28, their direction being the
centres'. The same holds with a name break at the turn (two segments) and through the view-graph
filter, the SEQUENTIAL mode included. The 20 near-duplicate look-alikes, a quarter of a step long
10 steps apart, are vetoed as well, while 65 true pairs set under the near-duplicate angle are
judged and kept; with the repair on the vetoed pairs stay removed and make no spot. A corridor
ending with six cameras at one place places three as still images, vetoes their look-alikes and
removes no true pair. On a closed loop of 80 cameras in the SEQUENTIAL mode the six pairs that
close it are judged and kept, also when the chain drifts off the loop by 12 steps. On the 16-camera
arc a neighbour pair with a flipped baseline its matches support is removed without the option and
kept with it, the one protected link.

On a capture of 1,086 panoramas of a shopping mall with repeating corridors (SIFT matching) the veto
judged 14,512 pairs and removed 2,244. Against the ground truth it removed 2,194 of the 2,201
look-alike pairs (a pair under a third as long as the true distance of its images) and none of the
10,791 true pairs, and the fold of the last 38 frames onto a look-alike corridor 102 m away was
gone. Three other captures (1,110 to 1,198 panoramas) reconstructed the same with and without the
option.

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

- A wrong neighbour pair without a name break that the chain solves is trusted as a link:
  protected, it can bend the chain into vetoing true pairs.
- A pair between two segments is not judged.
- A pair with no pair-scale equation to a link is not judged.
- A stretch of still images creeping under the near-duplicate angle is placed at one point.
- A pair the cut keeps at under 2 degrees of parallax is outside the repair's reach (the
  translation stage skips such a pair, and the repair reconsiders only removed ones). On a panorama
  capture 54% of the kept pairs under 1 degree and 21% of those from 1 to 2 degrees have a baseline
  direction more than 30 degrees off the reference; on three other captures about 5 to 10%.
- The cut judges a pair only against the rotations and camera centres the rest of the graph gives,
  so a collection whose false pairs outnumber and outweigh its true ones even after the scores can
  still be averaged into the wrong arrangement (`TripletScores.md`, the cutting rule).
