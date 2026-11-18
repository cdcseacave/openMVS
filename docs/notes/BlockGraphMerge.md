# The block merge as one decision over the block graph

A design note, not a description of the code. This merge was designed, built as a development patch and
measured against the simple merge that ships (one robust averaging of the block graph and one cut, see
[../design/HierarchicalSFM.md](../design/HierarchicalSFM.md)); the simple merge gave the same models and was
chosen. The note is kept so that this design can be rebuilt.

Designed 2026-10-20 (record `final-merge-filter-design-20261020.md` in the folder T named at the end),
measured until 2026-10-26 and kept as an alternative: on 2026-10-26 the user chose the simple merge
for production (one robust averaging of the block graph and one cut,
`simple-block-merge-20261021.md`) because it gave the same models, and asked that this design be
written down so that it can be rebuilt. A *block* is a sub-scene of at most 200 images that the
clustering cut from the capture and the hierarchical reconstruction solved alone, in its own frame;
the merge puts the blocks in one frame. This design replaces the camera-vote merge (sequential
placement, a dozen bars and fall-backs; the production merge before the simple one) by one decision
over the whole block graph. The view-graph filter and star evidence designed beside it shipped
(43444cf) and are not part of it.

**The readings.** A *reading* is one similarity (rotation, scale, translation) carrying one block's
frame into a neighbour's. Every block pair joined by cross pairs of at least `minCommonTracks` 25
weighted inliers is read in both directions: one block's cameras as a generalized camera against the
other's inlier tracks (`EstimateRigAgainstPoints`, a PnP with scale), refined over its inliers at
the 4 px bar (`maxReprojError`). The scale counts as observed when the rig's centres spread over at
least `minRigSpreadRatio` 0.03 of the median depth; a reading without one is judged on rotation
alone. A direction whose rig is one camera is a *door*: that camera's own pose against the other
block's points (P3P RANSAC on its bearings), its pose in its own block giving rotation and
translation, the scale the median ratio of the depths its points have in the two blocks, from at
least `kDoorMinDepthRatios` 10 correspondences that are also tracks of its own block (fewer:
unobserved). Readings of the two directions within the graph bars of each other are one reading,
refined over both (`GroupModes`). Without doors the merge places fewer blocks: alameda's block 5,
whose only door is one camera, and 1ea1bc0e's 45-image block on the older blocks.

**Extra modes and the camera trigger.** One direction can hold several readings: a copy of a
repeated structure (a facade period along) or the other end of a bent block. After a mode is found
its inliers at the bar are removed, and another is searched only while the correspondences left hold
at least `minSeamWeight` 2 rig cameras, each with at least `minCameraInliers` 10 of them, none
within the loose bar of the mode just found; at most `kMaxSeamModes` 3, and not once a mode has
under 25 inliers. The *loose bar* is `looseSeamFactor` 3 x 4 px = 12 px, what a block bent within
the graph bars is off by at its far end: a bend or a badly triangulated point leaves a camera's
correspondences inside it, a copy leaves them far from the true mode. The trigger replaced the first
design's (search again while the mode left over `kModeUnexplainedShare` 0.3 of its cameras'
correspondences unexplained after re-triangulating the points): alameda's merge fell from 906 s to
270 s, about 150 lines went, and the one cost was 5936fac3 -0.0051 [-0.0076, -0.0030] F1 (merge
alone, three runs).

**The evidence number.** Each reading gets e = w x (1 - f) x p, and nothing is refused but e = 0. w
is the number of cameras supporting it on its weaker side (a camera supports with at least
`minCameraInliers` inlier correspondences at the bar; a side held by one camera weighs 1). f is the
free-space conflict, below. p is the share of the inlier weight of the verified cross pairs that the
reading reproduces: a pair of at least `minPairInliers` 30 weighted inliers agrees when the relative
rotation its two images get under the reading is within `maxPairRotationResidual` 5 deg of what it
measured, and so is its baseline direction (within atan(tan 5 deg / ray angle)); p is 1 when no pair
is read. As an entry gate (the earlier design) the pairs refused right readings (alameda's (3,11),
share 0.48); dropped, they placed 5992d620's block 1 wrong at every thread count (F1 -0.155 [-0.217,
-0.099]); as a factor of e they removed alameda's wrong block 3 (1.66 deg off, 0.05 after), +0.0076
[+0.0048, +0.0107].

**Free space.** f is the share of the two blocks' own structure that a reading carries in front of
the surfaces the other block's cameras observe, past what each block shows in front of its own. A
surface is built once per block (`FreeSpaceSurfaceOf`): up to `kFreeSpaceCameras` 150 cameras spread
over the block, a 16 x 12 grid of image cells each, a cell's depth the median depth of the block's
inlier tracks seen there when at least 3 are, and up to `kFreeSpacePoints` 20000 inlier points. The
reading carries each block's points into the other's cameras, both ways; a point nearer than 0.8 of
its cell's depth is in front, one between 0.8 and 1.25 is at the surface, and f = max(0, front /
(front + at) - own) / (1 - own), own the larger of the two blocks' own front shares; f is 0 when no
point lands where a surface is known. A copy puts the other block's points in front of what the
cameras saw (62-66% on alameda's facade copies, 15-17% under the right mode). The cells are read
through each camera's own projection (`freespace=projection`): a pinhole through its K without the
lens polynomial, a spherical camera through its equirectangular grid with the ray length as depth
(FreeSpaceSphericalTest fails without it; on the eleven captures the result is unchanged to four
digits).

**Selection by consensus over spanning forests** (`SelectSeamGraph`, `select=consensus`). The edges
are the block pairs with a reading of e > 0. A forest is drawn: the first takes every edge's
heaviest reading, the others one reading per edge at random in proportion to its e, grown heaviest
first under e times a uniform random factor (Kruskal). The blocks are posed along the forest; every
other edge takes its heaviest reading within the graph bars of those poses
(`maxGraphRotationResidual` 5 deg, `maxGraphScaleResidual` 1.05, `maxGraphTranslationResidual` 0.05
of the camera footprint the two blocks share, the smaller extent floored at `kMinExtentShare` 0.2 of
the larger), the bars widened by the square root of the length of the cycle the edge closes with the
forest (Enqvist et al. 2011); two ends of different metric chains are judged on rotation alone. The
consensus is the sum of the e of the readings taken. Draws go on to the number RANSAC's formula asks
at the resection's confidence 0.999 (1000 to 100000, seeded, so a run is deterministic). The best
forest's readings are averaged (`AverageBlockPoses`: rotations, log-scales, then translations of the
blocks' camera centroids, each by Geman-McClure reweighting, `kRobustRounds` 10) and every edge is
read again at the plain bars of the averaged poses until none changes; an edge with no reading in
the bars is off. Right loops close within 3-5 deg and 2.5-5%; 1 deg breaks Playground's loop.

**Bridges and the exclusive-evidence rule.** An edge on no cycle of the edges left (a *bridge*)
cannot be tested by the others. It keeps its heaviest reading only when that reading's weaker side
holds `minSeamWeight` 2 cameras and its e is at least `bridgeWeightMargin` 1.5 times the evidence of
every rival counted only over the correspondences the rival explains at the bar that the heaviest
leaves past its own loose bar (`JudgeBridges`, `EvidenceBeyond`); otherwise the edge is off and its
blocks go to the resection. A bent second reading of the same seam lies inside the heaviest one's
loose bar and is no rival; a copy a period away keeps its whole weight. The camera trigger needs the
rule: with plain rival weights b68e320e's 55-image block is lost at every thread count (F1 -0.119),
with it the block is placed in 3 of 3 runs and 5992d620's 16-thread bridge is saved.

**The merged model and the tail.** Each connected component of the edges left is a model; the one
holding the most images is the scene, and every other block is merged without poses for the
resection to recover. The closure joins every correspondence of every placed block pair that lies
within the loose bar under the final block poses into its tracks by construction (`closure=all`;
closing at the plain bar costs 0.0126 of accuracy). `AdjustMergedScene` filters the tracks at the
loose bar, triangulates, filters again and runs one bundle adjustment; the reconstruction's own
final adjustment follows. The resection's repeated full adjustment as the tail (`tail=repeated`)
tied and was not adopted.

**Why it was not shipped.** On the 17 captures that split into blocks the simple merge gave this
design's model: blocks placed 102 and 102 of 103 (the camera-vote merge 90), the same blocks on 17
of 17 and the same registered and wrong images on 16 (1c65aab4: 504 against 503 images), F1 within
the noise (below). It drops the extra-mode search and mode grouping, free space, the evidence
product, the consensus and the widened bars, the exclusive-evidence rule and the rotation tree
sweep: roughly 750 lines out and 150 in, `GlobalAlignment.cpp` estimated at about 2350 lines against this
design's 2950 and the camera-vote merge's 4495 (the shipped file holds 2004), with no new mechanism for the
user to approve. The
consensus showed no effect on the current blocks (the design without it, `c8-front5`, against the
full one: 1 better, 1 worse, 12 ties, median +0.0000). The new resection (next-image order, stall
retry, closemax) also registers the images of blocks the camera-vote merge leaves out, so placement
no longer becomes F1: the simple merge beat the camera-vote merge on 8 captures to 1 on 2026-10-21
(median +0.0019) and on 2 to 2 on 2026-10-25.

**What it handles that the simple merge does not.** A block holding several bodies at independent
scales (non-rigid) makes its neighbours' readings inconsistent through it; the simple merge then
places one rigid block and the non-rigid ones wrong, where here the extra modes give each body a
reading and the consensus lets each edge take the one its body explains. On the older blocks
(graph7's, 77 blocks made at dc2b28d with the relative-pose fallback and without star evidence)
Outdoor_004's sub-scenes 0, 4 and 6 are non-rigid: this design places 7 of 7 blocks, F1 0.7334 (101
images in minor bodies stay off), the simple merge 7 of 7 with blocks 2, 4 and 6 wrong, F1 0.5005
(merge alone, thread-noisy). On the current blocks the difference is gone: all 83 blocks of the
2026-10-20 control are rigid, and on the 2026-10-25 blocks the two non-rigid ones, 26ccbd51's
sub-scene 1 (30 images, bodies of 24 and 4) and Outdoor_004's sub-scene 4 (181 images, bodies of
140, 24 and 17), are placed right by this design and the simple merge. Star evidence and the removal
of the relative-pose fallback make rigid blocks upstream, which is why the user accepted the limit.

Three planted scenes (`apps/Tests`, in the dev patches) fail under the simple merge; no capture of
the 18 produces one, and the user accepted them as known limits. `HeavyFalseChordTest` (a 12-block
ring and a 180 deg chord carried by every camera of one block, so it weighs what a true seam does,
with no verified pairs): the simple merge keeps the chord, its averaging settling on the chord
rather than on two light ring seams; this design passes it, which the simple-merge record attributes to the free-space factor.
`CopyOnDoorTriangleTest` (a true seam of 10 cameras, a one-camera door, and on the third edge a copy
one facade period along, shifted 10.7% of the footprint as alameda's (3,11) copy is, weighing 2.5
against the true reading's 2): the simple merge keeps the copy by weight; the consensus poses the
blocks by the reading that closes the triangle and is the only piece among those tried that passes
it. `BridgeEqualReadingsTest` (a triangle of true seams and a block reached by one pair holding two
readings 15 deg apart, 5 and 4 cameras): the simple merge places the block on one of the two
readings, where here the bridge margin takes the bridge off when the rival's evidence lies past the
loose bar. The dev test plants the readings without correspondences, so that rule reads nothing and
the test fails under the final switches; production would plant them (`BentRivalBridgeTest` does), a
form that was not run.

**Measured, 2026-10-20/21: the confirmation on 14 captures** (base ac8abbf, default clustering, 8
threads, one run each; the control is the camera-vote merge on identical blocks;
`final-design-confirm-20261020-report.txt`). F1 against the control: 8 captures better, 1 worse
(Outdoor_004 -0.0325, thread-noisy, where the control placed 5 of 8 blocks), 5 ties, median +0.0017;
blocks placed 82 against 73 of 83 on the 13 that split, none wrong; merge time alameda 1256 s to 269
s, Playground 315 to 77, dd20ce7c 352 to 93, Indoor_044 280 to 77. Every gate was met (no wrong
block, at least the control's blocks, no non-noisy capture worse by over 0.005 F1, merge time within
1.1 x + 30 s of the control's). On the older blocks, merge alone (2026-10-18), the camera-vote merge
placed 15 of alameda's 16 blocks at F1 .9593 in 1842 s and this design 16 of 16 at .9970 in 264 s;
Outdoor_004 went from 3-4 of 7 blocks under the selection without the consensus to 7 of 7 with it
(+0.1761 [+0.1544, +0.1980]), the only capture the consensus moved. The simple merge against it
(`simplemerge-20261021-report.txt`): on the 13 split captures 1 better, 3 worse, 9 ties, median
+0.0000; on the older blocks within 0.0062 on every capture but Outdoor_004 and 5992d620 (0.9576
against 0.8563, thread-noisy, rejected reference).

**Measured, 2026-10-25/26: the retest on the 18 test captures** (the new resection; arms on
identical saved blocks, 8 threads, one run each: `ctl` the camera-vote merge, `simple`, `simple-pts`
the simple merge on the 3D-3D estimator, `final` this design; 2678a364 is one block and has no
merge. F1 is read on the approved references, a tie being |dF1| of at most 0.0005; Outdoor_004,
5992d620, 2678a364 and Palace are thread-noisy, so a single-run move there is not evidence;
5992d620, 8d2f4877, 1ea1bc0e, dd20ce7c and 26ccbd51 have rejected references and are read on
counts.) Over the 17 captures that split: blocks placed 90 / 102 / 99 / 102 of 103 (ctl / simple /
simple-pts / final), no wrong block except simple-pts' alameda block 5 (its one-camera-door block),
registered images 9340 / 9348 / 9349 / 9349 (simple-pts summed from the cells), merge plus loose-bar
adjustment seconds 2786 / 1374 / 839 / 1383 (1386 summing final's rounded cells); on the ten
captures whose camera-vote merge takes under a minute the dev arms take 1 to 37 s longer, ctl
running no such adjustment; CPU of the whole run over the first 14, simple 7597 s, final 7496 s.
Paired F1 over the 12 approved-reference captures: final against simple 0 better, 2 worse (Palace
-0.0010, 1c65aab4 -0.0008), 10 ties; simple against ctl 2 better (5936fac3 +0.0007, Indoor_044
+0.0016), 2 worse (Palace -0.0025, Outdoor_004 -0.0013, both noisy), 8 ties; final against ctl 3
better, 3 worse, 6 ties. The captures where the arms differ (ctl / simple / final):

| capture (blocks) | blocks placed | registered | F1 | merge + adjust s |
|---|---|---|---|---|
| alameda (17) | 16 / 17 / 17 | 1731 all | .9971 / .9972 / .9972 | 1284 / 417 / 418 |
| Playground (3) | 2 / 3 / 3 | 307 all | .9991 / .9990 / .9990 | 310 / 98 / 97 |
| 5936fac3 (8) | 7 / 8 / 8 | 457 all | .9815 / .9822 / .9824 | 23 / 54 / 50 |
| Palace* (4) | 2 / 4 / 4 | 484 / 491 / 491 | .8054 / .8029 / .8038 | 11 / 48 / 41 |
| Outdoor_004* (8) | 5 / 8 / 8 | 802 all | .7697 / .7684 / .7688 | 40 / 68 / 75 |
| Indoor_044 (9) | 9 all | 1087 all | .7332 / .7349 / .7350 | 284 / 126 / 132 |
| 5e65b613 (6) | 5 / 6 / 6 | 571 all | .9969 / .9965 / .9963 | 79 / 78 / 75 |
| 1c65aab4 (4) | 4 all | 503 / 503 / 504 | .9784 / .9785 / .9793 | 11 / 37 / 36 |
| 26ccbd51 (11), counts | 10 / 11 / 11 | 703 / 704 / 704 | - | 50 / 63 / 80 |
| 5992d620* (3), counts | 1 / 3 / 3 | 195 all | - | 3 / 18 / 17 |

The other seven captures (Truck, b68e320e at 3 of 4, 935c0f53, 6685ec71, 8d2f4877, 1ea1bc0e,
dd20ce7c) place the same blocks and register the same images in all three arms, F1 within 0.0002 on
the approved ones; dd20ce7c's merge is 403 / 129 / 127 s and 6685ec71's 157 / 70 / 69 s. No arm has
a wrong image but Palace's one. The 3D-3D form places one block fewer on b68e320e (2 of 4), two
fewer on 1c65aab4 (2 of 4), and alameda's block 5 wrong.

**Where the code is.** T is the measurement tools folder `openmvs-release-20260908-tools`, T6
`openmvs-roma2-20260901-task6-tools`, both beside the datasets. The merge is
`T/dev-mergeretest-09d3cfc-20261025.patch` (4682 lines, applies unchanged to 09d3cfc and d69e811):
`libs/SFM/GlobalAlignmentDev.h` (482 lines) and `.cpp` (4121), namespace `SFM::MergeDev`, the final
design's rewrite of `GlobalAlignment` with every variant behind `DevMergeVariant`, and its
`AdjustMergedScene`, plus a 38-line change to `Scene::ReconstructHierarchical` that runs it when
`OPENMVS_DEV_MERGE` is set and saves the blocks to, or reads them from, the folder
`OPENMVS_DEV_SUBSCENES` names (how every arm of the retest got identical blocks). This design is
`OPENMVS_DEV_MERGE=score=product+modes=cameras+rivals=exclusive+freespace=projection+closure=all+select=consensus`;
the simple merge is `select=simple+modes=none+freespace=off+closure=all+bars=5/1.20/0.20`;
`--cluster-alignment 0` selects the 3D-3D estimator in both. The snapshot is
`T6/bin-mergeretest-2-d69e811` (`provenance.txt` holds the diff). To rebuild, export the commit with
`T/dev_tree_setup.sh`, `git apply` the patch, `git add -N` its two new files, re-run cmake (sources
are globbed) and build. The patch carries no tests: the scenario tests (`HeavyWrongChordTest`,
`CopyOnDoorTriangleTest`, `BridgeEqualReadingsTest`, `BentRingKeepsSeamsTest`, `FoldedBlock45Test`,
the free-space, product, camera-mode and bent-rival tests) are in
`T/finaldesign-ac8abbf-20261020.patch` (12091 lines, with the filter and star) and
`T/simplemerge-full-ac8abbf-20261021.patch` (the same plus the simple variant); under the final
switches `FoldedBlockTest`, `HierarchicalCameraAlignmentTest` and `SeamGraphConsensusTest` assert a
removed mechanism and fail, to be rewritten to the outcome. Run folders:
`<capture>/mergeretest-20261025-{ctl,simple,simple-pts,final}` (reports
`T/merge-retest-20261025-report.txt`, `-report-b2.txt`, `-report-b3.txt`),
`<capture>/finaldesign-20261020-c8-*`, `<capture>/simplemerge-20261021-*`.

**Limits and what was not measured.** A block is moved by one similarity, so a non-rigid one is
placed by a single body; a bridge with two coherent readings of near-equal evidence goes off and its
blocks go to the resection; a one-camera door stands only where a cycle corroborates it; a scene
whose weighted majority is itself wrong (Big Ben) is cut toward the wrong answer; no appearance cue
is read. Not measured: the merge stage's own CPU (there is no per-stage counter), thread noise on
the per-capture differences (one run per arm), the production form of `BridgeEqualReadingsTest`, and
a non-rigid block whose bodies pull the readings apart on the current pipeline (none among the 18
captures).
