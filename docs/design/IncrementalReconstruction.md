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
names, see `TripletDisambiguation.md`), a near-duplicate pair's inliers left out unless the image has no
other pair; it is a neighbour's evidence, not its inlier count, that lists
the neighbour in its star. A pair carries a relative pose of unit baseline, so a neighbour's distance to
the reference is known only through the matches that triangulate: the star's evidence of a pair is its
weighted inliers times the share of its track-forming matches whose ray angle at the pair's pose reaches
`StarInitConfig::evidenceMinAngle` (`ImagePair::ComputeShareAboveAngle`; `Scene::ReconstructHierarchical`
sets it to the reconstruction's `minAngleThreshold`, 1.5 degrees by default, the bar the track filter
applies later). A pair without evidence is no arm of the star, and the neighbours are sorted by evidence
and kept when their evidence reaches `ratioInliersFilter` (0.2) of the mean evidence of the three
strongest candidates that are not near-duplicates (all candidates when every one is), up to `maxViews`:
the floor is in the units the neighbours are sorted and compared by, so the strongest candidate always
passes it. A star with fewer than `minViews - 1` neighbours that have evidence is refused ("insufficient
initial views") before the floor is applied; one that keeps fewer goes on with the neighbours it keeps,
however few. A near-duplicate pair (`ImagePair::IsNearDuplicate`: its 10x10 overlap above
`PairsWeightingConfig::maxDuplicateOverlap`, 0.75, and its measured ray angle under
`maxDuplicateRayAngle`, 1.5 degrees; `StarInitConfig::pairsWeighting`, set by the scene) sets neither the
floor, nor the reference view, nor the pair the star's scale is fixed on (the reference's strongest pair
otherwise). Tuned against the reference poses of 33 captures (a pair fixes no distance when the true
parallax of its shared points is under 1.5 degrees; records-2026-11-05-duplicates/rule-075-15.md): 85% of
the pairs the rule flags fix no distance, and the yardstick moves off a pair that fixes no distance for
2045 images and off one that does for 296. The overlap condition keeps out the frames turned on the spot
by a large rotation, whose ray angle is small too.

| Floor of a neighbour's evidence (11 captures, 8 threads, reloads, without the near-duplicate rule; records-2026-11-05-duplicates/report-final.md) | final wrong | F1 summed |
|---|---|---|
| 0.2 x mean inlier count of the three heaviest | 22.5 | 9.728 |
| 0.2 x mean evidence of the three strongest (shipped) | 20.5 | 9.763 |

On f7dbf861 block 1 the inlier floor was 381 and no neighbour of 23 passed it (near-duplicate frames
with many inliers and a dozen that fix a distance), the evidence floor is 57 and 18 pass. The floor in
evidence admits more low-evidence views: Outdoor_004 block 3's star took one wrong camera there (0 wrong
after the reconstruction).

A view that shares only far points with the reference (a skyline, a building across the street) has
inliers by the hundreds and a unit baseline that guesses its distance at the scale of the near views, and
the star it enters fixes a distance no match backs: it is left to the resection, which registers it from
the model's own points once it sees some.

The star initializer refines the focal length alone, in its own small bundle adjustment
(`StarInitConfig::refineFocalLength`, default true), leaving every distortion term fixed: a star of a
handful of views cannot constrain a radial polynomial, and a solve free to move both together can
converge a focal that looks plausible but is wrong, having absorbed the error into distortion instead
-- the resection's own bundle adjustments refine distortion later, once the model is large enough to
hold it. A focal forced with `--focal-length` is left untouched here too: a star cannot improve an
approximate value, only bend it, while the global bundle adjustments, holding far more views, refine it
safely.

## The resection

Candidates are every unregistered image holding at least `minCorrespondences` (15) 2D-3D correspondences
to the model and at least one link -- a verified pair with a relative pose to a registered image
(`ForEachLink`, `PoseLink.h`, the same links the pose checks below read through `CollectPoseLinks`). An
image can reach the correspondences through long tracks alone before any image it is paired with is
registered; its pose would then have no link for the checks to weigh it against (on 6DCA1E8E image 326,
registered so in 2 of 4 runs, came out with its centre 20% off, and registered right, with 5 links,
when it waited), so it waits until one of its paired images is registered. The rule sits in the one
`RegisterImages` every resection uses (block, pool, whole-scene); the star start is not a resection.
Candidates are ordered by the weight of the evidence the model already holds for it: the composite weight
of its verified pairs to registered images (`SelectNextImages`; ties by the correspondence count). Among
the candidates of one moment the one with the most correspondences is often the one that fails while a
less counted one beside it, whose pairs lie in the model, registers: over the attempts judged at the event
on the fourteen test captures, of two attempts of the same round with different outcomes this weight puts
the right one first in 68% of the cases, the correspondence count in 32%. A round tries the candidates
down the list until one registers, and the next round orders them again on the model that registration
left (COLMAP's loop); no candidate is cut for holding few correspondences against the best one. Each is
solved by bearing-vector
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
direction they predict (`maxLinkDirectionError`, 5 degrees, `--max-link-direction-error`; 0 = no direction
check at all), judged on what the pair measured and on what the model sees: the model's parallax of a link
is the baseline the pose makes with the neighbour over the median distance of the pose's inlier points. A
link that the pair (its ray angle) and the model both see under `kNearDuplicateRayAngle` (2 degrees)
agrees; one that only one of them sees that close abstains (a near-duplicate pair's own baseline is
unreliable, and a baseline the model sees that short fixes no direction worth checking); any other is
held to `atan(tan(5°)/a)`, `a` the larger of the pair's ray angle and the model's parallax, so a wide
baseline forgives little disagreement and the pose under test can tighten its own tolerance but never
widen it. The pose is refused only when the links whose direction disagrees outweigh, by weighted
inliers, the ones that agree, so one misplaced neighbour cannot refuse it alone. Against the pair's ray
angle alone (one run per arm at 8 threads on the 14 test captures) the model's parallax cut the refusals
of right poses from 517 attempts on 21 images, 10 of them never registered, to 22 attempts on 7 images,
4 never registered, refused 178 wrong poses instead of 97 and accepted 41 instead of 45. Its known weak
spot is a quorum of pairs whose translation is wrong at a ray angle of a few degrees (Outdoor_004's
panorama pairs, 50-110 degrees off at 2-16 degrees): it cannot tell a wrong pair from a wrong pose and
refuses the right one.

A pose that passes the rotation and direction checks is then weighed on the rotation EVERY link predicts,
not only the quorum's: the mean angle between the pose's rotation and the rotations its links to
registered images predict from their neighbours' current poses, each link weighted by its pair weight
(the weighted inliers, the dense ones at `PairsWeightingConfig::DenseScale()`), must not exceed
`maxLinkRotationDisagreement` (8 degrees; 0 = off; no command-line flag). It runs also when the quorum
is contested, and an image with no such link is not judged. The quorum hears only its agreeing group, so
a pose 13 degrees from a strong link and in line with weak ones passes it; the weighted mean hears the
strong link. Its refusals are logged as "warning: rejected the pose of image I: ... and D degrees away
from the rotations its N links predict". Against no such check (12 runs on 10 test captures, 8 threads)
it cut the wrong registrations at acceptance from 629.5 to 593 (block 417 -> 403, pool 28 -> 24,
post-merge 184.5 -> 166) and the wrong images of the final models from 70.5 to 59.5, refusing 160 poses
of 49 images, 8 of them right ones (at 5 degrees: 252 right of 932). Most wrong acceptances it avoids are
transient ones the block's own adjustments would have repaired; where it matters it keeps one wrong
image out of a block (6DCA1E8E's block 2: wrong images at the block stage's end 45 -> 11) or changes the
registration order so a wrong chain never starts (Outdoor_004: final wrong 12 -> 0).

A pose that passes these checks is accepted however little of the image's evidence the model holds; the
order above already tries such an image after the candidates whose pairs lie in the model. (A deferral
that threw away a pose whose registered share of the pair weight was under a tenth of the registered
share of the scene's images only cost images: the under-coupled sub-scene it kept out of Outdoor_004 is
refused by the merge, and its images come in right in the whole-scene pass.)

A stall -- no image holds `minCorrespondences` (15) correspondences and a link, or every candidate failed -- with
the model changed since the last full bundle adjustment runs that adjustment and tries again: it
re-triangulates the tracks the change made triangulable or left without a position and moves the
model, which can give the remaining images the correspondences or the fit they lacked. Two things
change the model: an image registered, and images a filter removed right before the resection
(`ImagesRemoved`, the weak-image filter ahead of the whole-scene pass: their tracks lost views). A
stall with nothing registered or removed since the last full adjustment ends the resection. (Without
the removal counted, Outdoor_004 lost the 200 images the filter cut after the merge whenever the
whole-scene pass stalled before registering anything; it got them back only when a few leftover
registrations happened to trigger the adjustment.) Every resection closes with a full adjustment of its own
(every track triangulated again, every intrinsic the flags allow, 100 iterations), also when it
registered nothing since the last one and also when it had no image to try: the whole-scene pass that
follows the merge receives the model filtered at the final bundle adjustment's tighter bar, and the
closing re-admits the observations within the resection's bar that filter dropped (8d2f4877 keeps 389k
of its 413k tracks valid with the closing, 146k without it). So the whole-scene pass always runs, also when the merge registered every image, and a pass
whose only unregistered images are the ones the weak-image filter withheld for contradicting the model
(`ImagesRemoved`) skips its loop, logs "warning: no unregistered images" and closes all the same. It
returns false like any pass that registered nothing, so the caller does not filter the weakly connected
images a second time. A sub-scene whose star initialization registered every image closes the same way.
Over the fourteen test captures the order, the loop and the stall retry together leave 112 images
unregistered where the count order, the 0.3 cut of the count, the deferral and the stop at the first
stall left 470 (b68e320e 201 -> 269 of 269, 5936fac3 393 -> 456, alameda 1690 -> 1731), add no wrong
image to any capture, and take 32% more time (a refused candidate is attempted again after every
registration, and every retry is a full adjustment).

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

A track seen by fewer than two posed images has no position that two posed views determine, so the triangulation
leaves it out and the filter does not judge it: it either was never triangulated and still holds the default
position, or it lost a view's pose after it was triangulated. A triangulation that fails, for want of two inliers or because
its rays meet below `minAngleThreshold`, leaves its track with no inliers, so an adjustment that runs before the
next filter does not take the track in.

A track that has no position yet is made at an improbable one, `Track::UNTRIANGULATED_COORD` (1e18) in all three
coordinates, and not at the origin, which a posed camera can sit on (the star puts its reference camera there): a
track of two posed views that a caller filters before it is triangulated (the resection's full adjustment, the merged
model's adjustment) is then judged at a point far from every keypoint, with rays that meet at no angle, so the
filter's own tests reject it. The value is finite, so nothing computed from it turns to INF or NaN. Nothing reads the
position as a flag of "not triangulated": `numInliers` says that. A scene saved before this default stores (0,0,0)
for such a track, and the rule above covers what that reaches.

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
| `--max-link-direction-error F` | `5` | the resection's direction check, in degrees, in [0,90] (`maxLinkDirectionError`, read by every resection: the sub-scenes' and the whole-scene pass); 0 = no direction check at all, the inlier-share and rotation checks stay |

The resection's other acceptance parameters (`minInlierRatio`, `minInliersAbsolute`,
`maxRelativeRotationError`, `maxLinkRotationDisagreement`) have no flags; their defaults live in `ResectionConfig`.

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
