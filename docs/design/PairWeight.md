# Pair Weight

The composite pair weight `W` (`ImagePair::GetCompositeWeight`, ImagePair.h) ranks the verified
image pairs by how much the reconstruction can trust them, and three bars on it decide which pairs
the reconstruction reads at all. It is computed for the whole scene by `ComputePairsWeights`
(PairsWeighting.cpp) after matching, again after the triplet filter and after the contradiction
cut, and then frozen (the blocks of a hierarchical run keep the scene's weights).

## The formula

    W = min(WI, 1000) * coverage * cbrt(angle * connectivity) * (0.25 + triangles)

- `WI`, the weighted inliers (`ImagePair::GetNumWeightedInliers`, written by `ComputePairsWeights`
  into `weightedInliers`): the sparse inliers plus the dense inliers at
  `PairsWeightingConfig::DenseScale()` (`denseFrameInliers` / `denseMatchesPerFrame`, 25 / 2000), so a
  whole dense frame counts as a modest descriptor pair. Capped at 1000 in the weight.
- `coverage` (`ImagePair::overlapArea`, `ComputePairCoverage`): the share of a 10 x 10 grid the pair's
  track-forming matches occupy, the smaller of the two images'. Pinhole images bin on the pixel grid;
  spherical images on equal-solid-angle cells over the whole sphere (azimuth, sine of latitude), so a
  panorama pair's coverage is a share of the full sphere.
- `angle` (`ImagePair::ComputeAngleBaselineWeight` on `meanRayAngle`): 1 at 15 degrees of mean ray
  angle, a gentle quadratic loss down to 0.7 at 24 degrees, exponential decay past it, a steep
  quadratic below 1.5 degrees; 1 for a pair without a measured angle.
- `weightSpatial` (`ComputeIntrinsicWeight`) stores coverage x angle; it is 0 for a pair with fewer
  than `minInliers` (15) track-forming matches or whose weighted inliers round to 0, and such a pair
  has no weight at all. The product in `GetCompositeWeight` is written as
  `cbrt(weightSpatial * overlapArea^2 * connectivity)`, which equals coverage x cbrt(angle x connectivity).
- `connectivity` (`weightConnectivity`, step 4 of `ComputePairsWeights`): `WI` over the geometric mean
  of the two images' yardsticks (each image's strongest pair, a near-duplicate or repaired pair left out
  unless the image has nothing else), capped at 1. The pair's evidence against its images' best; nothing else.
- `triangles` (`weightTriplet`, step 3): valid / (valid + invalid + 5) over the triangles the pair
  closes, a triangle valid when its rotations compose to within 5 degrees. A pair the contradiction
  cut's repair brought back forms no triangle: its share is 0 and it is an edge of no other pair's
  triangles (its rotation is the averaged one, so its triangles close by construction).

The coverage and the triangle consistency count in full: they are the two terms that rank how well a
pair is matched against the reference reconstructions. The angle and the connectivity keep their
geometric mean, so neither weak term alone annihilates a pair of hundreds of inliers.

## The three bars

| bar | default | gates |
|---|---|---|
| `ReconstructionConfig::minPairWeight` | 1.22 | the pairs that form tracks (`BuildTracks`, strict >, in the scene, in every block and in a block's rebuild after the pool inserts images) and the pairs the contradiction cut averages (its rotation and translation stages, `FilterPairsByContradiction`) |
| `ClusterConfig::minPairWeight` | 0.9 | the edges of the clustering graph and its interface sums (`SceneCluster`, >=), and the tracks the clustering measures its seams on when the scene arrives without tracks |
| `ViewGraphCalibratorConfig::minPairWeight` | 0.9 | the pairs that give a focal residual to the view-graph calibration (>=; it runs during matching, for pinhole cameras left untrusted after the lens fit) |

The track/cut bar is 1.22 and not 0.9: on Outdoor_004 the pairs a bar of 0.9 adds are the ones
that put a block wrong, and on the normal-camera captures they buy nothing (see Validation). The
rotation averaging that reads `W` (the cut) scales the weights to a fixed median inside the
estimator (`GlobalRotationEstimatorOptions::weightMedian`), so it reads only their ratios.

Every other reader of `W` reads only its order or ratios: the next-image order and the pool's block
choice (sums of `W` over an image's links), the lens fit (the heaviest pairs per camera), the track
conflict settlement and a block's track rebuild (pair order), the frames-convention vote (the ten
heaviest pairs), the clustering's METIS edge weights (`W` x 10 rounded, above the clustering bar). The
bundle adjustment, the star initializer, the resection's pose checks and the merge read the weighted
inliers `WI`, not `W`. `PairsMatcher::ExportPairsCSV` takes the track bar but does not use it.

## Invariants

`W` >= 0, and `W` > 0 exactly when the pair has at least `minInliers` track-forming matches, weighted
inliers that do not round to 0 and a non-zero connectivity (`HasValidWeight`; a link is a pair with a
relative pose and a valid weight, `IsPoseLinkPair`). `ComputePairsWeights` asserts the intrinsic weight
positive wherever it computes triangles and connectivity, and the evidence positive in the yardsticks.

## Validation

Pre-cut matched scenes reloaded at 8 threads, one run each, the weight-scaled rotation averaging in
both columns (T/records-2026-11-11-weight). Per cell: registered / right / wrong / pose score
[wrong block starts; wrong registrations block / pool / post-merge].

| capture | the previous weight (inlier-ratio boost, cbrt of all terms, bar 3) | this weight |
|---|---|---|
| Palace | 495 / 494 / 1 / 0.8103 [0; 8 / 7 / 0] | 495 / 494 / 1 / 0.8191 [0; 11 / 5 / 0] |
| 4e5c6f6d | 291 / 250 / 32 / 0.9140 [0; 71 / 0 / 26] | 281 / 269 / 3 / 0.9489 [0; 34 / 17 / 28] |
| 17ac94cc | 553 / 521 / 8 / 0.6443 [0; 57 / 1 / 0] | 551 / 520 / 6 / 0.6234 [0; 56 / 2 / 3] |
| 5ada248e | 530 / 524 / 5 / 0.9562 [0; 31 / 0 / 29] | 531 / 525 / 4 / 0.9562 [0; 13 / 1 / 21] |
| Outdoor_004 | 767 / 767 / 0 / 0.7196 [0; 44 / 0 / 1] | 774 / 767 / 7 / 0.7310 [0; 13 / 0 / 0] |
| Indoor_044 (track bar 0.9) | 1087 / 1087 / 0 / 0.7138 (a reload without the monitor) | 1087 / 1087 / 0 / 0.7141 [0; 13 / 0 / 0] |
| 6DCA1E8E (track bar 0.9) | 841 / 464 / 7 / 0.7527 [0; 167 / 1 / 99] | 667 / 465 / 7 / 0.7510 [0; 146 / 0 / 82] |

6DCA1E8E ends one of two ways from run to run (its images 617-871, which no reference judges,
registered or not); its two runs ended in different ways, and the images the references judge are
the same. 4e5c6f6d's gain is the formula's: the same 269 right / 3 wrong at a track bar of 0.9.

The track/cut bar on Outdoor_004 (this formula; right / wrong / pose score): 0.9 at every use
660 / 107 / 0.6507; the clustering bar alone at 1.22 666 / 101 / 0.6503; the track/cut bar alone at
1.22 767 / 7 / 0.7310; every bar at 1.22 767 / 7 / 0.7295. At 0.9 the image that starts the failure
registers in its block on 37 inliers 5.3 degrees off its links (32 inliers, 0.5 degrees at 1.22), the
block's error spreads to 28 neighbours of a revisited spot, and three blocks attached only through
that spot follow it (78 images right in their own blocks, wrong in the model). The 342 pairs between
0.9 and 1.22 are 15% wrong-pose against 1% above 1.22; they are panorama pairs, whose coverage
(median 0.18, against 0.33 to 0.42 on the normal-camera captures) enters the weight in full.

## Rejected alternatives

- The triangle term squared, (0.1 + triangles)^2, bars 0.75: folds Palace (286 right, 209 wrong), with
  the median-scaled rotation averaging too.
- That squared weight for the tracks while the cut keeps the previous one: two weights for one
  quantity, no gain (Palace 0.8009).
- The cut weighted without the triplet score: loses Palace's 78 right junction pairs.
- A track/cut bar of 0.9: about 100 wrong images on Outdoor_004, no gain elsewhere.
- The inlier-ratio boost on the connectivity: rank correlation 0.05 with the references' ranking.
- The triplet score raised to a power in the cut: no more false pairs removed.

## Open items

- 17ac94cc's pose score falls with this formula (0.6443 -> 0.6234, the same right images; mean score
  over them 0.4823 -> 0.4592); the track bar does not carry it (0.6292 at 0.9).
- Outdoor_004's images 508-514 register wrong in their block under both weights; the final weak-image
  filter removes the three the previous weight registers and keeps them under this one.
- The resection accepts an image on 37 inliers 5.3 degrees off every one of its links (the link
  quorum's bar is 15 degrees): the Outdoor_004 failure at a track bar of 0.9 starts there.
