# Incremental Reconstruction

The default reconstruction path (`Scene::Reconstruct`, per cluster inside
`Scene::ReconstructHierarchical`): registers images one at a time, interleaving pose estimation with
bundle adjustment and dropping images whose evidence falls apart.

## Overview

Pair selection (`PairsMatcher.h/cpp`) proposes which pairs to attempt and matches and verifies them
(`MatchGeometric.h/cpp`); the triplet filter (`ViewGraphTriplets.h/cpp`, see
`TripletDisambiguation.md`) removes the pairs a look-alike or repeated structure fabricated. The star
initializer (`StarInitializer.h/cpp`) picks a well-connected reference view and registers a handful of
its strongest neighbours at once, seeding the model the resection (`Resection.h/cpp`, `PoseLink.h`)
grows one image at a time, interleaved with local and full bundle adjustments
(`BundleAdjustment.h/cpp`). Once every reachable image is registered, the image filter
(`FilterWeaklyConnectedImages`, `Track.h/cpp`) drops what does not hold up, and a final bundle
adjustment (`Scene::Reconstruct`) closes the pass.

## Pair selection

The retrieval-based match modes (VOCABULARY, RETRIEVAL) rank candidates by visual similarity with no
notion of capture order, and on a video or hand-held walkthrough that leaves out a share of the truly
overlapping consecutive pairs -- exactly the ones that hold an interior's matched graph together end to
end. `--match-sequence-overlap` (default 3, the whole candidate set of SEQUENTIAL mode) is now also
added, unconditionally, on top of whatever VOCABULARY or RETRIEVAL already picked
(`PairsMatcher::AddSequentialPairs`): every image gets its next `overlap` neighbours in import order
proposed alongside its ranked candidates. Images import in capture order, so a consecutive pair
verifies almost every time; 0 turns the prior off.

## The star initializer

The star initializer refines the focal length alone, in its own small bundle adjustment
(`StarInitConfig::refineFocalLength`, default true), leaving every distortion term fixed: a star of a
handful of views cannot constrain a radial polynomial, and a solve free to move both together can
converge a focal that looks plausible but is wrong, having absorbed the error into distortion instead
-- the resection's own bundle adjustments refine distortion later, once the model is large enough to
hold it. A focal forced with `--focal-length` is left untouched here too: a star cannot improve an
approximate value, only bend it, while the global bundle adjustments, holding far more views, refine it
safely.

## The resection

Candidates are the unregistered images with the most 2D-3D correspondences to the model, plus every
image within `ratioCorrespondences` (0.3) of the best of them (`SelectNextImages`), solved by
bearing-vector PnP (PoseLib RANSAC). A pose is accepted only when credible: `minInliers` (12) inliers
at least, and `minInlierRatio` (0.25) of the correspondences, unless the inlier count alone reaches
`minInliersAbsolute` (100) -- a small consensus inside a large set can agree on a pose the image never
had. A weakly supported pose (inlier share under half) is cross-checked against the rotation its
verified pairs to registered images predict -- not the single strongest pair, misleading if that
neighbour is misplaced, but the quorum: the largest group, among the strongest few links, agreeing
within `maxRelativeRotationError` (15 degrees) of one another (`PoseLink.h`). A pose further than that
from the quorum's rotation is rejected; contradictory links (no two agree) are left to the other rules.

When no image reaches `minCorrespondences` (15), or an iteration's candidates all fail to register,
the resection falls back to relative poses (`relativePoseFallback`, on by default): an image still
joined to the model by a verified pair -- its tracks two-view, not yet triangulable -- is registered
from that pair's relative pose instead of PnP, trying up to three ranked candidates and skipping one
only when it has links to spare and no two of them agree (a candidate with just two contradicting
links is registered from the heavier one instead). Rotation and center both come from the quorum: two
or more agreeing rays are intersected by least squares (rejecting a near-degenerate or
behind-the-neighbour solution); one usable ray falls
back to a distance drawn from the median baseline its neighbour already has to its own registered
neighbours -- what a steadily moving capture suggests -- after which the two-view tracks are
triangulated, handing the next round the correspondences it was missing.

Local bundle adjustment (`localBAEvery`, every 10 registrations, a fixed window of `maxLocalWindow`
neighbours, never refining intrinsics) avoids a global solve every step. Full bundle adjustment runs
every `fullBAEvery` registrations (25, then 50, then 100 and every 100 after), or sooner when the
average inlier ratio drops below `avgInliersRatioForceBA` (0.6, only once `minImagesForceBA` (3) images
have registered since the last one). A registration's ratio is normalized by its own share of described
versus dense correspondences before it enters that average (`denseInlierRatioFactor`), since a dense
correspondence reaches a lower inlier share than a described one even on a healthy model -- refining
the main-set intrinsics (focal, k1, k2, as far as `--refine-intrinsics` allows them) until
`minRefineExtIntrs` (100) images are registered, and after that whatever the intrinsics level allows
beyond the main set.

## Bundle adjustment

Reprojection residuals fit each track's observations against the camera model, robustified by a Huber
loss (2 px in the resection's own solves); a dense (warp-sampled) keypoint's residual is discounted
relative to a described one's (measured, or pinned with `--ba-dense-weight`; see `ROMA2InProcess.md`).
What a solve costs is the number of observations it fits, and one image can bring thousands of them
-- warp samples where it was matched densely, detections where it was not -- so each image
contributes at most `maxObservationsPerImage` (1500, `--ba-max-obs`) of any kind. The budget goes to
the described observations first -- a detected position is the precise measurement, so an image keeps
all of its own unless they alone exceed the budget -- and the warp samples fill what is left of it,
so that an image with few detections spends most of the budget on warp samples and one with many
spends little. Each kind is taken round-robin across a grid over the image, so that what survives
covers the frame instead of clustering where the matcher was densest; inside a cell the longest track
comes first, and the remaining ties are ordered by a key drawn for that solve alone, so successive
adjustments take different subsets and, over a reconstruction, most observations take part in some
solve. A track left with a single view is given a dropped observation back, a described one first; a
track left with none stays out of the solve. The budget applies only
once the scene's observations reach `minObservationsForCap` (1000000,
`--ba-cap-min-obs`): a scene small enough to be solved whole is solved whole, and once it is not,
every solve of it is budgeted, including a local window that alone holds only a few images.

Every verified pair whose two images are both in the solve also adds a relative-pose residual
(`RelativePoseError`): the model's relative rotation against the pair's, and its baseline direction
against the pair's, sharing one Huber loss at 3 standard deviations. Their sigmas are `--ba-pair-sigma`
(default 1 degree for the rotation, twice that for the baseline direction, the weaker of a pair's two
measurements), divided by the square root of the pair's weighted inlier count (capped at 500)
against a reference of 100. A verified pair's evidence is otherwise invisible to the reprojection
residuals, and this is what holds a joint straight where the tracks are few or two-view only. Either
sigma at 0 switches that half off; both at 0 leaves the solve fitting reprojections alone.

`--refine-intrinsics` (default 1) picks what the bundle adjustments may touch: 0 fixes every
intrinsic, 1 focal length + k1, k2, 2 adds k3, the principal point and the tangential terms. Local BA
never refines intrinsics, full BA stays at the main set regardless, and only the extended BA -- from
`minRefineExtIntrs` images on, or the final solve -- opens everything allowed.

## The image filter

`FilterWeaklyConnectedImages` runs after the shared tail's final bundle adjustment; the images it
removes are withheld from the resection offered the rest, so a removed image is not just handed back
and removed again, and the filter runs a second time only when that resection registers something.
Two tier verdicts drop an image outright:
spatial distribution (triangulated points occupy too little of the image, `minObservationArea` 0.15)
and geometric degeneracy (median triangulation angle too small, `minTriangulationAngle` 1.5 degrees).
The rest form a covisibility graph -- an edge per pair sharing `minCovisibilityCount` (5) inlier tracks
-- keeping only the largest connected component; a k-core peel then drops images whose independent
covisibility degree falls under `minCovisDegree` (2), repeating on their neighbours, and the largest
component is retaken. An optional pose-consistency cut (`maxPoseInconsistencyAngle`, off by default)
can additionally drop an edge whose global rotation disagrees with the pair's own stored one, for a
scene the tier verdicts and the peel do not separate.

Every stage above judges an image by triangulated structure, which one the resection registered from a
relative pose alone does not have -- its tracks are two-view, with no covisibility edge at all.
Corroboration (`maxCorroborationAngle`, default 5 degrees) rescues such an image when at least two
verified pairs, each carrying 15 or more weighted inliers, join it to images already settled -- settled
starting as whatever the tier verdicts and the largest component keep on their own merits -- and each
pair's model rotation and baseline direction agree with what it measured, within that angle. A
near-duplicate pair, whose rays triangulate under 2 degrees, is judged on rotation alone. Settled grows
a round at a time as newly corroborated images become witnesses for the next round, rescuing a chain of
relative-pose registrations as far as it reaches back -- but never from an image its own round is still
deciding, so a chain cannot lift itself in.

## Flags

| Flag | Default | Effect |
|---|---|---|
| `--match-sequence-overlap N` | `3` | consecutive images every image is matched with; the whole candidate set of SEQUENTIAL mode, a prior added to VOCABULARY/RETRIEVAL (0 = off) |
| `--refine-intrinsics 0\|1\|2` | `1` | bundle-adjustment intrinsics: 0 none, 1 focal length + k1, k2; 2 adds k3, the principal point and the tangential terms |
| `--ba-pair-sigma F` | `1` | relative-pose residual rotation sigma, in degrees (the baseline direction at twice that; 0 disables the residuals) |

The resection's acceptance parameters (`minInlierRatio`, `minInliersAbsolute`,
`maxRelativeRotationError`, `relativePoseFallback`) and the image filter's corroboration bar
(`maxCorroborationAngle`, its inlier floor) have no flags; their defaults live in `ResectionConfig` and
the `FilterWeaklyConnectedImages` call in `Scene.cpp`.

## Limitations

A chain of images registered from relative poses alone carries an unobservable scale until a third
view triangulates their shared points -- the one-ray fallback's median-baseline prior is a guess, not a
measurement. A long single trajectory with no loop closure drifts like any incremental method: the
relative-pose residuals restrain a joint the tracks do not hold, not a bend spread evenly across a
trajectory that never re-observes itself. A look-alike block whose false pairs outnumber its true ones
can still join the model at the wrong place -- the triplet filter's cutting rule and the
pose-consistency cut are the tools for that, not this document's own defaults, which assume the matched
graph is largely correct. And the pair residuals mostly restate evidence the reprojection residuals
already carry where a joint is well tracked; pushing `--ba-pair-sigma` much below a degree only fights
the tracks for control of a joint neither side has reason to distrust.
