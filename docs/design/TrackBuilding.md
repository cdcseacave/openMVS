# Track Building

A track is one 3D point seen by several images: a set of observations, each an (image, keypoint)
pair, with a triangulated position. The tracks are built from the matches of the verified pairs
(`BuildTracks`), given a position by a linear triangulation (`TriangulateTracks`), and held to the
model by the track filter (`FilterTracks`). `Track.h/cpp`, `Triangulation.h/cpp`.

```
Track { Point3 position; ObservationArr observations; uint8_t numInliers; }
```

The first `numInliers` observations are the inliers (`Track::begin()..end()` iterates them); a track
is an inlier track when it has at least `Track::MIN_VIEWS` (2) of them (`IsInlier`). A track is
valid when it has at least two observations.

## Building the tracks

`BuildTracks(scene, minPairWeight, conflict)` replaces `scene.tracks`.

1. Pairs. Every pair with stored matches whose composite weight (`PairWeight.md`) is strictly above
   `minPairWeight` takes part; a negative `minPairWeight` disables the bar (a bar of 0 still drops
   the pairs of weight 0). The pairs arrive in decreasing composite weight, the order
   `ComputePairsWeights` leaves `scene.pairs` in.
2. Links. Only the track-forming matches of a pair link keypoints: its verified sparse inliers plus
   its dense supplement (`ImagePair::GetNumTrackFormingMatches`), nothing past them. Each match links
   the global feature IDs of its two keypoints (an image's offset plus the keypoint index).
3. Union. All links are united plainly with a disjoint-set (union-find) structure; a component is
   every keypoint the matches connect, right or wrong.
4. Conflicts. A component that holds two keypoints of one image is not a track: one of its links is
   wrong, or the two keypoints are one feature detected twice. Such a component is settled on its
   own links (`ConflictedComponent`), below.
5. Tracks. The keypoints are grouped by component; a group of fewer than two observations is
   dropped. Observations are sorted by (image, keypoint). A new track has the position
   `Track::UNTRIANGULATED_COORD` and no inliers.

### Settling a conflicted component (`TrackConflictConfig`)

The wrong link is found by what the other links say about it, not by which pair arrived first.
`TrackConflictConfig` (`ReconstructionConfig::trackConflictCfg`, mirrored in
`ClusterConfig::trackConflictCfg`, which has its own copy with the same defaults; no command-line
flag):

| Parameter | Default | Effect |
|---|---|---|
| `cut` | true | cut links until every piece is a track; false: the union veto |
| `mergeDistance` | 3 px | two keypoints of one image this close are one feature (0 disables) |
| `maxComponentSize` | 5000 | a component of more keypoints than this takes the veto whatever `cut` says |

The cut (`ConflictedComponent::Cut`):

- The support of a link is the number of triangles through it, the common neighbours of its two
  keypoints, counted once on the component as it arrived: a triangle is evidence whether or not one
  of its links has since been cut.
- While some piece holds two keypoints of one image that are farther apart than `mergeDistance`,
  take the two keypoints of the first such image farthest apart, find the shortest path between them
  over the standing links, and cut the least-supported link on it: the fewest triangles, then the
  lighter pair (composite weight), then the link that came first in pair order. The side the
  triangles corroborate stays.
- The pieces left are the tracks. A piece that still holds one image twice holds two keypoints of
  one feature (within `mergeDistance`): the keypoint with more standing links stands for the image
  (the lower index at a tie) and the others get no track.

The veto (`ConflictedComponent::Veto`; `cut` false, or a component above `maxComponentSize`): the
links are united in pair order and a link is refused when it would bring an image in twice, unless
every keypoint of that image in one set is within `mergeDistance` of every one in the other. The
link that arrived first stands.

## Triangulation

`TriangulateTracks(scene, outliersOnly, reprojThreshold, minAngleThreshold)`
is the method the reconstruction uses; it triangulates each track with `TriangulateSkewLLS` in
parallel.

- A track seen by fewer than two posed images has no position that two posed views determine: it is
  left out and its inliers are reset to 0, so its inlier observations are always posed.
- `outliersOnly` skips the tracks that are already inliers.
- `TriangulateSkewLLS` works on the unit bearing `d` of each posed observation
  (`Camera::UnprojectNormalized`, so it is the same for pinhole and spherical cameras) and solves
  `[d]_x (R P + t) = 0` for the point `P`: the three rows of `[d]_x R` per observation, an
  overdetermined linear least squares solved by SVD. The block has rank 2 for every unit `d` and
  the squared residual is `|d x (R P + t)|^2`, the same weight in every direction of the ray. Two of
  the three rows would not do: the pair `(0, -dz, dy)`, `(dz, 0, -dx)` becomes parallel as `dz` goes
  to 0 (a ray 90 degrees off the camera's forward axis, as common for a panorama as any other ray),
  where the observation would fix the point only along the optical axis. It then keeps the
  observations that project in front of the camera within `reprojThreshold` pixels of their keypoint,
  solves again over those inliers alone, and moves them
  to the front of `observations`.
- It fails, returning 0 and leaving the track with no inliers, when fewer than `minInliers` (2)
  observations are inliers or when the widest angle between two inlier rays
  (`Track::ComputeMaxAngleBetweenRays`, the angle the pair of cameras furthest apart sees the point
  under) is below `minAngleThreshold`. The position written stays, and a later adjustment does not
  take the track in because it has no inliers.

### The untriangulated marker

A track without a position is made at `Track::UNTRIANGULATED_COORD` (1e18) in all three
coordinates, not at the origin, which a posed camera can sit on (the star initializer puts its
reference camera there). A track of two posed views that a caller filters before it is triangulated
(the resection's full adjustment, the merged model's adjustment) is then judged at a point far from
every keypoint, with rays that meet at no angle, so the filter's own tests reject it. The value is
finite, so nothing computed from it turns to INF or NaN, and its square still fits a float. Nothing
reads the position as a flag: `numInliers` says whether a track is triangulated.

## The track filter

`FilterTracks(scene, maxReprojErrorPixels, minAngleDegrees, multDepthNear, multDepthFar)` re-judges every track against the current poses and cameras and sets
`numInliers`; it returns the mean reprojection error in pixels and degrees of the inliers.

- A track with fewer than two posed observations is not judged and ends with no inliers.
- Each observation of a posed image is an inlier when its ray makes an angle with the ray to the
  track's position that is within the pixel bar converted to an angle per camera
  (`Camera::PixelErrorToAngular`), so the same check works for pinhole and spherical cameras; a
  position behind the camera fails it. Described and dense observations answer to the same bar.
- A track needs at least two inlier observations and a widest ray angle of at least
  `minAngleDegrees` between its inlier cameras, else it ends with no inliers.
- When more than 1000 tracks remain, a track whose mean distance to its inlier cameras is below
  `multDepthNear` (0.05) times the median of those distances, or above `multDepthFar` (20) times it,
  is removed (0 disables a side).

## Parameters and where the bars are used

| Parameter | Default | Where |
|---|---|---|
| `ReconstructionConfig::minPairWeight` | 1.22 | the pair bar of `BuildTracks` in the scene, in every block and in a block's rebuild after the pool |
| `ClusterConfig::minPairWeight` | 0.9 | the pair bar when the clustering builds the tracks of a scene that arrives without them |
| `ReconstructionConfig::maxReprojError` | 4 px | the reconstruction's triangulation and filter bar |
| `ReconstructionConfig::maxFineReprojError` | 2 px | the filter after the final bundle adjustment |
| `ReconstructionConfig::minAngleThreshold` | 1.5 degrees | the triangulation and filter angle bar |
| `ResectionConfig::maxReprojError`, `minAngleThreshold` | 4 px, 1.0 degrees | the resection's own triangulations and filters, and the rebuild of a block's tracks |
| `StarInitConfig::maxReprojError`, `minAngleThreshold` | 6 px, 1.0 degrees | the star initializer's first triangulation; its later passes tighten to `max(maxReprojError - 1, 1)` and `max(maxReprojError - 2, 1)` px, and raise the angle by 0.5 and 1.0 degrees, capped at 3 |
| `multDepthNear`, `multDepthFar` | 0.05, 20 | one value for the reconstruction and the resection |

The reconstruction's own passes: the final model is filtered and its outlier tracks triangulated at
`maxReprojError`, adjusted, and filtered at `maxFineReprojError`; the merged scene is filtered and
triangulated at the loose bar of 3 x `maxReprojError` (`GlobalAlignmentConfig::looseSeamFactor`);
the known-poses path first triangulates at 4 x `maxReprojError` because its imported poses are
approximate. The bundle adjustment fits the inlier tracks (`BundleAdjustment.md`).

## Invariants

- `BuildTracks` produces a track with each image at most once, at least two observations and
  `numInliers` 0; a debug build verifies it (match indices inside the keypoint counts, no image
  twice in a track).
- A track's inlier observations are all posed images; `TriangulateTracks` and `FilterTracks` reset
  `numInliers` of a track that lost a pose.
- After a failed triangulation the track holds no inliers.
- `TriangulateTracks` asserts a non-empty track array.

## Validation

The conflict policy on alameda (1734 images, 12.7M links): the share of wrong keypoints kept is
simulated offline against the poses of a SIFT reconstruction of the same images, the rest is
reconstructed from the same matches:

| policy | wrong keypoint kept in the conflicts a triangulation can judge | registered | median rotation error | centre error (of the scene diagonal) | builder time (60.7M observations) |
|---|---|---|---|---|---|
| veto in pair order | 22% | 1734 | 0.048 degrees | 0.0103% | 34.0 s |
| cut with merge within 3 px | 17% | 1734 | 0.048 degrees | 0.0098% | 29.5 s |

The cut reconstructs in 8.5% less time and with 4.9% fewer points. Track lengths are identical
under the two policies (median 2, 90th percentile 8, 99th percentile 31 to 33, mean 4.2).

## Rejected alternatives

- The veto in pair order as the policy: keeps the wrong keypoint in 22% of the judgeable conflicts
  and drops a million links, half of them matches the SIFT matching has too; it remains for
  components above `maxComponentSize` and behind `cut = false`.
- Ordering the pairs by weight for the veto: changes nothing, the pairs already arrive sorted by
  composite weight.
- No merge of close keypoints: the merge within 3 px halves the conflicts that end with neither
  keypoint kept.
- A looser reprojection bar for dense observations: a factor on the bar that nothing ever set; one
  bar serves described and dense observations.

## Open items

- The minimum triangulation angle is 1.5 degrees in `ReconstructionConfig` (the reconstruction's
  final filters) and 1.0 degrees in `ResectionConfig` and `StarInitConfig` (triangulation and
  filtering inside the resection, the star and the sub-scenes' block rebuild); the resection's value
  is never overridden by the reconstruction.
- `TriangulateDLT`, a pinhole-only DLT triangulation, exists and is used only by the tests; the
  reconstruction uses `TriangulateSkewLLS`.
- `TriangulateSkewLLS` keeps the first two rows of `[d]_x R`; they are linearly dependent for a
  bearing with a zero z component in the camera frame (only the third coordinate is then
  constrained by that observation), which a spherical camera produces at 90 degrees of longitude.
- The `ReconstructionConfig::minPairWeight` comment says 0 disables the bar; `BuildTracks` disables
  it with a negative value and a bar of 0 still drops the pairs without a weight.
