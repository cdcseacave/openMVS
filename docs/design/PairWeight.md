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
  `PairsWeightingConfig::DenseScale()` (`denseFrameInliers` / `denseMatchesPerFrame`, 50 / 2000), so a
  whole dense frame counts as a modest descriptor pair and a pair with fewer dense inliers counts
  proportionally fewer. Capped at 1000 in the weight.
- `coverage` (`ImagePair::overlapArea`, `ComputePairCoverage`): the share of a 10 x 10 grid the pair's
  track-forming matches occupy, the smaller of the two images'. Pinhole images bin on the pixel grid;
  spherical images on equal-solid-angle cells over the whole sphere (azimuth, sine of latitude), so a
  panorama pair's coverage is a share of the full sphere.
- `angle` (`ImagePair::ComputeAngleBaselineWeight` on `meanRayAngle`): 1 at 15 degrees of mean ray
  angle, falling quadratically (1 - 0.3 (d / 9)^2, d the distance from 15 degrees) to 0.7 at 24
  degrees and, on the narrow side, at 6 degrees; above 24 degrees an exponential decay
  (0.7 exp(-(angle - 24) / 10.5)); below 6 degrees the same quadratic continues to 0.325 at 1.5
  degrees, under which it falls as 0.325 (angle / 1.5)^2; 1 when the mean ray angle is not positive
  (no angle measured).
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

Every bar is one comparison, `ImagePair::ExceedsWeightBar`: a pair is used only when its composite weight is strictly above the bar, so a bar of 0 drops exactly the pairs without weight and a negative bar drops none.

| bar | default | gates |
|---|---|---|
| `ReconstructionConfig::minPairWeight` | 1.22 | the pairs that form tracks (`BuildTracks`, in the scene, in every block and in a block's rebuild after the pool inserts images) and the pairs the contradiction cut averages (its rotation and translation stages, `FilterPairsByContradiction`) |
| `ClusterConfig::minPairWeight` | 0.9 | the edges of the clustering graph and its interface sums (`SceneCluster`), and the tracks the clustering measures its seams on when the scene arrives without tracks |
| `ViewGraphCalibratorConfig::minPairWeight` | 0.9 | the pairs that give a focal residual to the view-graph calibration (it runs during matching, for pinhole cameras left untrusted after the lens fit) |

The track/cut bar is 1.22. A bar of 0.9, the value of the other two, is the more aggressive
setting: it lets more pairs form tracks and enter the cut. On the normal-camera captures the two
values give the same results; on the panorama capture Outdoor_004 the pairs 0.9 adds are the ones
that put a block wrong (see Validation). The
rotation averaging that reads `W` (the cut) scales the weights to a fixed median inside the
estimator (`GlobalRotationEstimatorOptions::weightMedian`), so it reads only their ratios.

Every other reader of `W` reads only its order or ratios: the next-image order and the pool's block
choice (sums of `W` over an image's links), the lens fit (the heaviest pairs per camera), the track
conflict settlement and a block's track rebuild (pair order), the frames-convention vote (the ten
heaviest pairs), the clustering's METIS edge weights (`W` x 10 rounded, above the clustering bar). The
bundle adjustment, the star initializer, the resection's pose checks and the merge read the weighted
inliers `WI`, not `W`.

## Invariants

`W` >= 0, and `W` > 0 exactly when the pair has at least `minInliers` track-forming matches, weighted
inliers that do not round to 0 and a non-zero connectivity (`HasValidWeight`; a link is a pair with a
relative pose and a valid weight, `IsPoseLinkPair`). `ComputePairsWeights` asserts the intrinsic weight
positive wherever it computes triangles and connectivity, and the evidence positive in the yardsticks.

## Validation

Pre-cut matched scenes reloaded at 8 threads, one run each, with the default bars. Per cell:
registered / right / wrong / pose score [wrong block starts; wrong registrations at block / pool /
post-merge stage]. Right and wrong are judged against reference poses.

| capture | registered / right / wrong / pose score | wrong block starts; wrong registrations |
|---|---|---|
| Palace | 495 / 494 / 1 / 0.8191 | 0; 11 / 5 / 0 |
| 4e5c6f6d | 281 / 269 / 3 / 0.9489 | 0; 34 / 17 / 28 |
| 17ac94cc | 551 / 520 / 6 / 0.6234 | 0; 56 / 2 / 3 |
| 5ada248e | 531 / 525 / 4 / 0.9562 | 0; 13 / 1 / 21 |
| Outdoor_004 (panorama) | 774 / 767 / 7 / 0.7310 | 0; 13 / 0 / 0 |

The track/cut bar on Outdoor_004 (right / wrong / pose score): 0.9 at every use 660 / 107 / 0.6507;
the clustering bar alone at 1.22 666 / 101 / 0.6503; the track/cut bar alone at 1.22 767 / 7 /
0.7310; every bar at 1.22 767 / 7 / 0.7295. At 0.9 the image that starts the failure registers in
its block on 37 inliers 5.3 degrees off its links (32 inliers, 0.5 degrees at 1.22), the block's
error spreads to 28 neighbours of a revisited spot, and three blocks attached only through that spot
follow it (78 images right in their own blocks, wrong in the model). The 342 pairs between 0.9 and
1.22 are 15% wrong-pose against 1% above 1.22; they are panorama pairs, whose coverage (median
0.18, against 0.33 to 0.42 on the normal-camera captures) enters the weight in full.

## Rejected alternatives

- The triangle term squared, (0.1 + triangles)^2, with bars at 0.75: folds Palace (286 right, 209 wrong).
- That squared weight for the tracks while the cut keeps another: two weights for one quantity, no gain.
- The cut weighted without the triplet score: loses 78 right junction pairs of Palace.
- A track/cut bar of 0.9 as the default (it stays the more aggressive setting): about 100 wrong images on Outdoor_004, no gain elsewhere.
- An inlier-ratio boost on the connectivity: rank correlation 0.05 with the references' ranking.
- The triplet score raised to a power in the cut: no more false pairs removed.
- A coverage term computed on the sparse matches alone: no gain on any capture, and right images lost on a lidar one.
- Gating the dense-only pairs by triangle closure: hurts a long indoor capture at every bar tried (0.2 to 0.4: 144 to 184 wrong images against 56 to 66).
- A static cap on the dense inliers (a quarter each, 300 at most) instead of the frame-anchored
  scale: it flattens every link of an image whose pairs all exceed it, so on a textureless interior
  a near and a far link weigh the same through the connectivity term.
- A dense frame target of 100 (1/20 of the draw): 77 to 80 wrong images against 50 to 61 at 50; 150 is refused too.
- A finer coverage grid (20 cells per side, or the side taken from the number of matches): closer to the true overlap (mean error +0.10 against +0.17 on
  12,000 pairs with depth), but it scales every weight by 0.7 to 0.86 and, with the bars scaled alone, folds a symmetric building in the contradiction
  cut; it needs everything that reads the weight calibrated again, so the 10 x 10 grid stays.

## Open items

- The resection accepts an image on a few dozen inliers several degrees off every one of its links
  (the link quorum's bar is 15 degrees); at a track bar of 0.9 a failure starts there.
- A panorama pair's coverage is low and enters the weight in full, so the bar separates wrong
  pairs less sharply on panoramas than on normal cameras.
- The weak-image filter keeps some images that sit wrong in their block under this weight.
