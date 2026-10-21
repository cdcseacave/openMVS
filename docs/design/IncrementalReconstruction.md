# Incremental Reconstruction

The default reconstruction path (`Scene::Reconstruct`, per cluster inside
`Scene::ReconstructHierarchical`): registers images one at a time, interleaving pose estimation with
bundle adjustment and dropping images whose evidence falls apart.

## Overview

Pair selection (`PairsMatcher.h/cpp`) proposes which pairs to attempt and matches and verifies them
(`MatchGeometric.h/cpp`); the triplet filter (`ViewGraphTriplets.h/cpp`, see
`TripletDisambiguation.md`) scores them and the contradiction cut that follows
(`FilterPairsByContradiction`) removes the pairs a look-alike or repeated structure fabricated. The star
initializer (`StarInitializer.h/cpp`) picks a well-connected reference view and registers a handful of
its best-evidenced neighbours at once, seeding the model the resection (`Resection.h/cpp`, `PoseLink.h`)
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

The reference view is the image with the most weighted inliers (among the seed views the triplet filter
names, see `TripletDisambiguation.md`); it is a neighbour's evidence, not its inlier count, that lists
the neighbour in its star. A pair carries a relative pose of unit baseline, so a neighbour's distance to
the reference is known only through the matches that triangulate: the star's evidence of a pair is its
weighted inliers times the share of its track-forming matches whose ray angle at the pair's pose reaches
`StarInitConfig::evidenceMinAngle` (`ImagePair::ComputeShareAboveAngle`; `Scene::ReconstructHierarchical`
sets it to the reconstruction's `minAngleThreshold`, 1.5 degrees by default, the bar the track filter
applies later). A pair without evidence is no arm of the star, and the neighbours are sorted by evidence
and kept above `ratioInliersFilter` (0.2) of the mean raw inlier count of the three heaviest, up to
`maxViews`. A view that shares only far points with the reference (a skyline, a building across the street)
has inliers by the hundreds and a unit baseline that guesses its distance at the scale of the
near views, and the star it enters fixes a distance no match backs: it is left to the resection, which
registers it from the model's own points once it sees some.

The star initializer refines the focal length alone, in its own small bundle adjustment
(`StarInitConfig::refineFocalLength`, default true), leaving every distortion term fixed: a star of a
handful of views cannot constrain a radial polynomial, and a solve free to move both together can
converge a focal that looks plausible but is wrong, having absorbed the error into distortion instead
-- the resection's own bundle adjustments refine distortion later, once the model is large enough to
hold it. A focal forced with `--focal-length` is left untouched here too: a star cannot improve an
approximate value, only bend it, while the global bundle adjustments, holding far more views, refine it
safely.

## The resection

Candidates are the unregistered images with the most 2D-3D correspondences to the model, plus every image
within `ratioCorrespondences` (0.3) of the best of them (`SelectNextImages`), solved by bearing-vector
PnP (PoseLib RANSAC) with an inlier bar of `ransac.threshold` (8 px, twice `maxReprojError`). The bar is
set for the dense (warp-sampled) correspondences, which land several pixels off on a full-resolution
image (a median of 6 px, against 1 px for a detected keypoint) and make up most of the correspondences of
an image seen through dense matches: at 4 px a third of them counted as inliers on a healthy model, so
the average inlier ratio below stayed under its bar and forced a full bundle adjustment after nearly
every registration (154 over 514 images, against 10 at 8 px, the poses coming out the same), and a
minority of them could vote in a pose degrees off. The adjustments that follow are robust to the outliers
the wider bar admits and the track filter removes them, so the filter's own bar stays at 4 px; 6 px is
the bar to try once the dense matching gets more precise. A pose is accepted only when credible:
`minInliers` (12) inliers at least, and `minInlierRatio` (0.25) of the correspondences, unless the inlier
count alone reaches `minInliersAbsolute` (100) -- a small consensus inside a large set can agree on a
pose the image never had. Every pose, however well supported, is cross-checked against the rotation its
verified pairs to registered images predict -- not the single strongest pair, misleading if that
neighbour is misplaced, but the quorum: the largest group,
among the strongest few links, agreeing within `maxRelativeRotationError` (15 degrees) of one another
(`PoseLink.h`). A pose further than that from the quorum's rotation is rejected; contradictory links (no
two agree) are left to the other rules. On alameda a pose registered with a 54% inlier share (87 of 162
correspondences) was 1.9 degrees off, and the 15 images registered from its structure landed 1.9 degrees
off with it, while every right registration of the scene passes the quorum by a wide margin (never more
than 7.7 degrees against the 15-degree bar). The same quorum's links are also weighed on the baseline
direction they predict (`maxLinkDirectionError`, 3 degrees): each link's own tolerance grows with its ray
angle (`atan(tan(3°)/rayAngle)`), so a wide-baseline pair forgives little disagreement and a
near-duplicate one (rays under 2 degrees, its own baseline unreliable) fixes no direction and is left out
of the count; the pose is refused only when the links whose direction disagrees outweigh, by weighted
inliers, the ones that agree, so one misplaced neighbour cannot refuse it alone.

Before either link check runs, every pose first has to be registered where its
evidence actually is: `minRegisteredEvidence` (0.1) requires the composite weight of the image's pairs
into already-registered images to reach a tenth of the registered share of the scene's images, out of the
weight of all its pairs -- a bar that rises as the model fills in, so an image is not hung off the model
by the few pairs it happens to have when most of its evidence still lies beyond the model's edge. An
image that misses the bar is not refused outright, only deferred -- retried once more of its neighbours
have registered, or picked up by the tail pass, which sees every pair. A round in which every
candidate is deferred ends this sub-scene's resection there:
only the whole-scene pass that follows the merge sees those images again, with their neighbours
registered, and a community that stays unregistered as a whole is left out.

The resection stops when no image reaches `minCorrespondences` (15), or when an iteration's candidates
all fail to register.

Local bundle adjustment (`localBAEvery`, every 10 registrations, a fixed window of `maxLocalWindow`
neighbours, never refining intrinsics) avoids a global solve every step. Full bundle adjustment runs
every `fullBAEvery` registrations (25, then 50, then 100 and every 100 after), or sooner when the
average inlier ratio drops below `avgInliersRatioForceBA` (0.6, only once `minImagesForceBA` (3) images
have registered since the last one). The average runs over the registrations alone -- an image whose
attempt failed or was refused is not in the model and adds nothing to it -- and a registration's ratio is
normalized by its own share of described versus dense correspondences before it enters (`denseInlierRatioFactor`),
since a dense correspondence reaches a lower inlier share than a described one even on a healthy model -- refining
the main-set intrinsics (focal, k1, k2, as far as `--refine-intrinsics` allows them) until
`minRefineExtIntrs` (100) images are registered, and after that whatever the intrinsics level allows
beyond the main set.

Between the adjustments, the tracks are triangulated and filtered against `maxReprojError` (4 px) and
`minAngleThreshold`. Both kinds of observation answer to that bar. `FilterTracks` and `TriangulateTracks`
can hold a dense (warp-sampled) observation to a multiple of it (`denseReprojErrorFactor`, 1 by default and
in every call the reconstruction makes): the warp is computed at a fixed resolution, so a dense keypoint's
pixel error grows with the image size where a detected keypoint's does not.

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

`FilterWeaklyConnectedImages` runs after the shared tail's final bundle adjustment. Of the images it
removes, the ones cut for contradicting the model -- a cut component whose verified pairs to the kept
images put it elsewhere than the model did, by more inlier weight than agrees -- are withheld from the
resection offered the rest, so such an image is not just handed back and removed again; the ones cut
for want of evidence (no verified pair of theirs puts them elsewhere than the model) are offered again,
since the grown model may hold what they lacked. The filter runs a second time only when that resection
registers something.
Two tier verdicts drop an image outright:
spatial distribution (triangulated points occupy too little of the image, `minObservationArea` 0.15)
and geometric degeneracy (median triangulation angle too small, `minTriangulationAngle` 1.5 degrees).
The rest form a covisibility graph -- an edge per pair sharing `minCovisibilityCount` (5) inlier tracks
-- keeping only the largest connected component; a k-core peel then drops images whose independent
covisibility degree falls under `minCovisDegree` (2), repeating on their neighbours, and the largest
component is retaken. An optional pose-consistency cut (`maxPoseInconsistencyAngle`, off by default)
can additionally drop an edge whose global rotation disagrees with the pair's own stored one, for a
scene the tier verdicts and the peel do not separate.

## Flags

| Flag | Default | Effect |
|---|---|---|
| `--match-sequence-overlap N` | `3` | consecutive images every image is matched with; the whole candidate set of SEQUENTIAL mode, a prior added to VOCABULARY/RETRIEVAL (0 = off) |
| `--refine-intrinsics 0\|1\|2` | `1` | bundle-adjustment intrinsics: 0 none, 1 focal length + k1, k2; 2 adds k3, the principal point and the tangential terms |
| `--ba-pair-sigma F` | `1` | relative-pose residual rotation sigma, in degrees (the baseline direction at twice that; 0 disables the residuals) |

The resection's acceptance parameters (`minInlierRatio`, `minInliersAbsolute`,
`maxRelativeRotationError`, `maxLinkDirectionError`,
`minRegisteredEvidence`) have no flags; their defaults live in `ResectionConfig`.

## Limitations

A long single trajectory with no loop closure drifts like any incremental method: the
relative-pose residuals restrain a joint the tracks do not hold, not a bend spread evenly across a
trajectory that never re-observes itself. A look-alike block whose false pairs outnumber its true ones
can still join the model at the wrong place -- the triplet filter's cutting rule and the
pose-consistency cut are the tools for that, not this document's own defaults, which assume the matched
graph is largely correct. And the pair residuals mostly restate evidence the reprojection residuals
already carry where a joint is well tracked; pushing `--ba-pair-sigma` much below a degree only fights
the tracks for control of a joint neither side has reason to distrust.

A pose whose evidence is wrong but agrees with itself passes every check above: on alameda, image 1733
registers from four pairs of 28-124 sparse matches whose relative rotations agree with each other within
2.9 degrees, all 2.6-4.6 degrees off the reference -- biased in the same direction, not scattered, so no
quorum or link-majority rule can tell it from right evidence. The dense fill made those same pairs'
geometry better, not worse: the same pairs matched with SIFT alone land 7-29 degrees off the reference,
and SIFT could not keep the image registered at all. Documented, not fixed.
