# What a dense observation costs: its density, its weight in resection, and the captures that break

**Date:** 2026-09-05
**Branch:** `feature/roma2-onnx`
**Builds on:** `docs/superpowers/specs/2026-09-04-dense-budget-and-weight-design.md`, which made the
dense fill a density over uncovered overlap and made bundle adjustment estimate the dense
observation weight from residuals.

## 1. Why

The dense fill now has a principled *budget* (a density, capped by the verdict's smaller inlier
area) and a principled *weight in bundle adjustment* (estimated per solve from the two populations'
robust reprojection sigmas). Two things it still does not have:

**The density itself is unmeasured.** `PAIR-CORRECTNESS-REPORT.md` says so in as many words: 2000
correspondences per full frame of overlap turns into roughly two million tracks on every capture
regardless of its size, and nothing measured says 2000 is where the accuracy curve flattens. The
cost is not subtle:

| capture | matching | bundle adjustment | peak RSS | tracks |
|---|---|---|---|---|
| 8d2f4877 | 269 s | 9 105 s | 18.23 GB | 2 012 171 |
| 38004114 | 343 s | 16 952 s | 16.47 GB | 2 164 672 |
| Truck | 338 s | 5 700 s | 25.10 GB | 2 153 582 |

Matching is not the cost. Bundle adjustment is, and the dial that drives it has never been swept.

**Resection does not know a dense observation from a described one.** Bundle adjustment
down-weights dense observations and the view graph discounts them, but `Resection::RegisterImage`
(`libs/SFM/Resection.cpp:68`) builds one PnP problem over every inlier-track observation and scores
all of them against a single angular threshold derived from `config.ransac.threshold x
GetFeatureNoiseScale()`. That threshold is tuned for descriptor keypoints. Measured median
reprojection error of the two populations is not comparable — described keypoints at 0.67-1.01 px
against a dense segment whose positions are warp-cell accurate — so dense observations
systematically fail an inlier test calibrated for SIFT and drag the ratio down with them. This is
the shape of the 0.40/0.48 resection inlier ratio that opened the campaign.

Weighting dense observations in resection was deferred (G5) until the p10 resection inlier ratio
could be read from real arms. Those arms have run. This spec closes it.

## 2. Scope

In scope: the resection change, the density sweep, and the hard captures that have never been run.

Out of scope: the dense fill's budget rule and the bundle adjustment weight, both settled by the
2026-09-04 spec; `--roma2-match`'s default, which stays **off** and is not what any measurement
here is arguing about.

## 3. Design

### 3.1 Resection fits on the observations calibrated for its threshold

`poselib::estimate_absolute_pose_bearings` takes a single `opt.max_error`. There is no per-point
threshold and no per-point weight, so "down-weight the dense observations" is not expressible
against this estimator without changing it. The available design is which observations enter the
problem.

**Rule:** the PnP problem is built from **described** observations only. `Image::IsDenseKeypoint`
already answers this per observation, and the observation loop already has the image and the
feature index in hand.

**Fallback, which is part of the rule and not an afterthought:** if the described observations of
an image number fewer than `config.minInliers`, the problem is rebuilt over **all** observations.
On a textureless capture the dense segment is not a supplement, it is the entire evidence — the
capture where SIFT registers 0 images and the one pass registers 248 of 311 — and an image that
would have registered from dense observations must not stop registering because a rule written for
well-textured scenes excluded them. The fallback is reported in the log so an arm can tell how
often it fired.

Two properties this deliberately keeps:

- Dense observations still triangulate, still enter bundle adjustment at their estimated weight,
  and still count in the view graph at its own discount. Only the PnP inlier test stops seeing
  them.
- The reported resection inlier ratio becomes a statistic about the population its threshold was
  calibrated for, which is what makes it comparable to the pre-dense numbers at all.

### 3.2 The density is chosen by measurement, not inherited

`--roma2-dense-matches` keeps its meaning (correspondences per full frame of overlap, capped by
`DenseFillCeiling`) and its default of 2000 until §4.1 says otherwise. This spec does not change
the code; it changes what the default is *based on*.

## 4. Measurement

Controller work. §4.1 is already running.

**4.1 The density sweep.** `--roma2-dense-matches` at 500 / 1000 / 2000 / 4000 on 38004114, the
textureless interior where the dense pass is decisive (SIFT registers 0 images; the one pass
registers 248 of 311). One binary for every point of the sweep — a frozen snapshot of the branch
tip, `bin-tip-56f898f` — with the subset, focal and slot budget identical to the recorded
baselines, so the density is the only thing that moves. 2000 is included as the anchor that ties
the curve back to the recorded `onepass` and `capdensity` numbers. Cheapest first, so the low end
reads early. Run folders `<capture>/openmvs-roma2-20260905-density-<N>/`.

Report per point: registered images, track count and mean length, dense track-length histogram,
median and p10 resection inlier ratio, median described and dense reprojection error, the estimated
bundle adjustment dense weight, relative-rotation and centre error against GT, matching and total
wall time, peak RSS.

The question is where accuracy flattens against a bundle adjustment cost that runs 5 700-16 952 s.
A density that buys nothing above 1000 is worth roughly half the campaign's wall clock.

**4.2 Truck, as the control.** The chosen density plus 2000 on a well-textured outdoor capture,
where the dense fill is already known to add mostly redundant correspondences of lower quality. A
density tuned on a textureless interior must not be promoted without this: scene type dominates
difficulty on every other measurement this branch has made.

**4.3 The hard captures.** The one-pass arm has never run on them. Subsets of about 500 images
exist for `OfficeBadLoop`, `HouseBadDrift` and `House4Levels`, built and never used; every one-pass
number on record comes from three easy-to-medium captures of 225-311 images.

These run **after** §4.1, at the density it chooses, and that ordering is the point: at 2000 a
500-image capture extrapolates to well over ten hours of bundle adjustment each, and running three
of them before knowing whether the density buys anything would spend days measuring the wrong
configuration. `sift` and `onepass` arms, the same instruments, the same report.

**4.4 §3.1's effect.** Median and p10 resection inlier ratio, registered images, and how often the
fallback fired — on both a textured and a textureless capture. The fallback firing on every image
of a textureless capture is the expected result there, not a failure.

## 5. What changes

| File | Change |
|---|---|
| `libs/SFM/Resection.cpp` | §3.1: the PnP problem is built from described observations, with the all-observation fallback and its log line |
| `libs/SFM/Resection.h` | the rule and its fallback documented where the config lives |
| `apps/Tests/TestsSFM.cpp` | a test that the described-only selection holds, and a test that the fallback fires below `minInliers` |
| `docs/design/ROMA2InProcess.md` | what a dense observation now costs in each of the three consumers: resection, bundle adjustment, view graph |

## 6. Risks

- **Described-only resection could lose images on mixed captures** — a capture textured enough to
  have described keypoints but not enough to reach `minInliers` from them alone sits exactly on the
  fallback boundary, and the fallback is all-or-nothing per image. §4.4 measures registration, not
  just the ratio, for this reason.
- **The sweep's cheapest point may be indistinguishable from its most expensive.** That is a
  result, not a failure, and it argues for a much lower default.
- **The hard captures may not reconstruct at any density.** They were selected as hard: bad loops,
  bad drift, four levels. A capture that fails everywhere is still a reportable result, and it is
  the reason they were built.
