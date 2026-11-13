# Dense Matching (RoMa v2 and FlexGlue)

## Purpose and scope

A dense matcher is a neural model run in process through ONNX Runtime (`libs/SFM/OnnxRuntime.h/cpp`). It
offers two independent services to the classical SfM front end:

- **Retrieval descriptors**: one global descriptor per image, from which `--match-mode 4` (RETRIEVAL, see
  `PairSelection.md`) ranks candidate pairs.
- **One-pass pair matching** (`--roma2-match`): the matcher's *warp* of a candidate pair (for every cell of a coarse
  grid on image A, where that cell lands in B and how confident the model is, in both directions) decides whether the
  pair is real (the verdict), guides the descriptor matching of the pair's SIFT/AKAZE keypoints, and fills the rest of
  the overlap with dense correspondences. A pair the verdict rejects is dropped; there is no descriptor-matching
  fallback and no union with a descriptor-matched pair set.

Two models implement the interface `DenseMatcher` (`libs/SFM/DenseMatcher.h`): RoMa v2 (default) and FlexGlue
(`--dense-matcher flexglue`). Everything downstream of the warp (`JudgePairROMA2`, `TrackKeypointsByWarp`,
`MatchFeaturesGuided`, `AssemblePairROMA2`, `StorePairROMA2`, the planar twin switch, the lens fit, the dense
pseudo-count in the pair weight, `PairWeight.md`) never asks which model produced the warp.

Out of scope: building and provisioning ONNX Runtime and the model files (`../RoMa2Model.md`); the bundle
adjustment's weight of a dense observation (`BundleAdjustment.md`: it measures the precision of the dense keypoints
per solve, independently of the view graph's fixed dense pseudo-count described here under the options).

Source files: `OnnxRuntime.h/cpp` (session and tensor wrapper), `DenseMatcher.h`, `RoMa2Matcher.h/cpp`,
`FlexGlueMatcher.h/cpp`, `MatchROMA2.h/cpp` (describe pass, slot plan, verdict, assembly, storage, planar twin switch,
`ROMA2Config`), `ROMA2Warp.h/cpp` (warp conventions, keypoint tracking, sampling, dense append), `MatchGeometric.h/cpp`
(`MatchFeaturesGuided`), `GlobalDescriptors.h/cpp` (cosine retrieval index), `Scene.cpp` (`Scene::MatchPairs`).

## Options and where they act

`--roma2` is the master switch for the model; it does nothing alone. `CreateStructure` rejects `--roma2` unless
`--roma2-match true` or `--match-mode 4` asks for something the model produces, rejects `--roma2-match` without
`--roma2`, rejects `--dense-matcher flexglue` without `--roma2`, and rejects `--flexglue-model` with another matcher.
The two services are independent: `--match-mode 4` alone ranks pairs by the descriptors and matches them with the
ordinary SIFT/AKAZE/ORB flow; `--roma2-match` composes with any match mode.

| option | `ROMA2Config` field | default | effect |
|---|---|---|---|
| `--roma2 B` | `enabled` | false | load the in-process model |
| `--roma2-model DIR` | `modelPath` | `$OPENMVS_ROMA2_MODEL_PATH`, else the install prefix copy if present | folder of the RoMa2 graphs and manifest |
| `--roma2-setting turbo\|fast\|base` | `setting` | `base` | preset: the image side is 320, 512 or 640 px and the warp grid is a quarter of it (160 cells at `base`) |
| `--roma2-provider auto\|cuda\|coreml\|dml\|cpu` | `provider` | `auto` | ONNX Runtime execution provider (see Providers) |
| `--roma2-match B` | `useMatching` | false | one-pass pair matching |
| `--dense-matcher roma2\|flexglue` | `matcher` | `roma2` | the model behind `--roma2 true`, for both services |
| `--flexglue-model DIR` | `flexglueModelPath` | `$OPENMVS_FLEXGLUE_MODEL_PATH` | folder with the three FlexGlue graphs |
| `--dense-min-covisibility F` | `minCovisibility` | 0.2 | covisibility gate before the verdict, in [0,1]; 0 = off; a matcher that gives no score (RoMa2) is never gated |
| `--roma2-slots N` | `slotBudget` | 64 | image descriptor tensors resident on the device during pair matching; at least 2 |
| `--roma2-min-confidence F` | `minConfidence` | 0.1 | confidence at which a warp cell takes part in the verdict, the keypoint tracking and the fill |
| `--roma2-min-overlap F` | `minOverlap` | 0.10 | the verdict's bar on the smaller inlier area (see Verdict); 0 admits every pair |
| `--roma2-dense-matches N` | `denseMatchesPerFrame` | 2000 | dense correspondences per full frame of overlap, a density and not a per-pair count; also the frame target of the view graph's dense pseudo-count, so the same value reaches `PairsWeightingConfig::denseMatchesPerFrame` |

`ROMA2Config` fields with no command-line option, all used by the one pass:

| field | default | meaning |
|---|---|---|
| `useGPU` | true | false (`--gpu-device` asking for the CPU) forces the CPU provider |
| `denseEpipolarErrorFactor` | 1 | multiple of the matcher's epipolar bar (`MatchConfig::maxEpipolarError`) a dense correspondence is held to when the pair's geometry classifies it |
| `denseFitWeight` | 0.003 | weight of a dense correspondence against a sparse one's 1 in the refinement that closes the pair's relative pose; 0 drops the dense matches from the polish |
| `guidedBandLengthCells` | 2 | guided search: warp cells either way along the epipolar line from the warp's prediction |
| `guidedBandHalfWidthCells` | 0.5 | guided search: warp cells either way across the line; 0 searches the disc of the length's radius |
| `guidedSameFeatureDistance` | 3 px | a rival this close to the winner is the same feature described twice and is no rival |
| `twinMinPlaneShare` | 0.75 | planar twin switch: share of a pair's track-forming matches a homography must explain |
| `twinMaxLoopAngle` | 3 deg | planar twin switch: rotation of a triangle's loop that counts as closed |
| `twinMinLoopGain` | 3 | planar twin switch: closed triangles a switch must gain; 0 turns the switch off |

The view graph's dense pseudo-count is `PairsWeightingConfig::denseFrameInliers` (50) of `denseMatchesPerFrame`
inliers: a whole frame of warp samples weighs as 50 descriptor inliers, one function used wherever a count decides.
It is a fixed constant, deliberately independent of the bundle adjustment's measured weight: the warp localizes a
point several times less precisely than a descriptor, which the bundle adjustment charges per solve, but it says
nearly as much about whether two images overlap, which is all the view graph asks (`PairWeight.md`).

## The DenseMatcher interface

`Scene::MatchPairs` creates one `DenseMatcher` per call (declared before the `PairsMatcher` that holds it, so it is
destroyed after it) and drives it from one thread; the pool only runs the consumers of its output.

| member | meaning |
|---|---|
| `Load(dir, setting, provider)`, `IsLoaded()`, `ProviderName()` | load the model's graphs on the execution provider |
| `ImageSize()`, `Preprocess(bgr, planarRgb)` | side of the square an image is squashed to, and the planar RGB floats made of it (each model keeps its own resize; the prefetch ring that runs it on the pool is shared) |
| `GridSize()` | cells per side of the warp |
| `RetrievalSize()`, `MakeCode()`, `DescribeImage(planar, code, retrieval)` | one image's descriptor tensor (on the device when the provider keeps tensors there) and its unit global retrieval descriptor, stored in `Image::globalDescriptor` |
| `MatchPair(codeA, codeB, PairInput, PairOutput)` | both warps of the pair plus the optional outputs below |
| `NeedsIntrinsics()` | whether `MatchPair` reads the cameras, so that every image must have a pinhole camera |

`PairInput` holds the pixel sizes of the two images and their pinhole intrinsics as the pipeline holds them at the
matching stage (`MakePairInput`, with the principal point moved by half a pixel to the corner-origin convention of
`PinholeIntrinsics`): the focal that import, `--focal-length` or the EXIF gave, before the lens fit and the focal
check (`TwoViewVerificationAndLens.md`).

`PairOutput` is the `PairWarps` plus two optional outputs, absent where a model gives none (RoMa2 gives neither):

- `covisibility` (0..1): "do the two images show the same place".
- `pose` (`PosePrior`): a coarse relative rotation `R` and unit baseline direction `t` in the pair's own convention
  (`x2 = R*x1 + s*t`, `s > 0` unknown; the convention of `ImagePair::relativePose`).

The pair callback of the warp pass tests presence, never the model's name. A pair with a covisibility below
`minCovisibility` is dropped before the verdict (logged as `ROMA2 covisibility gate: N of M warped pairs below B`).
The two outputs of a stored pair go into `ImagePair::covisibility` and `ImagePair::posePrior`; neither is serialized
and neither is geometry (no `Reset` clears them). `--export-pairs-csv` appends the columns `Covisibility`,
`PriorQw..PriorQz`, `PriorTx..PriorTz` only when some pair has a covisibility.

## The models

### RoMa v2

A DINOv3 backbone and a coarse matcher head, exported as two static-shape, batch-1, fp32 ONNX graphs plus a JSON
manifest (`roma_<setting>.json`, `format_version` 1; `RoMa2Manifest::Load` rejects any other version and any graph
I/O that disagrees with the shapes derived from `image_size` and `warp_size`; `OnnxModel::Load` rejects dynamic
dimensions).

| graph | inputs | outputs |
|---|---|---|
| descriptor (`roma_<setting>_descriptor_fp32.onnx`) | `image` [1,3,S,S] RGB planar in [0,1] (mean/std applied in graph) | `layers` [1,2,G,G,1024] (the image's descriptor tensor, G = S/16), `value_facets` [1,2,G,G,1024] (scratch for the pooling), `retrieval` [1,2048] |
| coarse match (`roma_<setting>_match_coarse_fp32.onnx`) | `descriptors_A`, `descriptors_B` (two `layers` tensors) | `warp` [1,C,C,2] and `confidence` [1,C,C,1] for A to B, and `warp_BA`, `confidence_BA` for B to A, from the same forward pass (C = S/4); confidence is a raw logit, the sigmoid is applied on the host |

The refiner stages are not exported: they buy match precision, which the guided sparse matches already supply wherever
the scene has texture, and no depth maps are produced from the warps.

Conventions (`ROMA2Warp.h`), all with `align_corners=false` in OpenMVS working-orientation pixels (integer = pixel
centre; `Image::LoadPixels` rotates portrait images to landscape, the same pixels the keypoints were extracted from, so
no map is ever rotated): warp cell (i,j) describes the source image at the pixel `(i+0.5)*W/C - 0.5`
(`CellToPixel`, inverse `PixelToCell`); a warp value `n` is the target pixel `0.5*(n+1)*W - 0.5` (`DenormCoord`,
inverse `NormCoord`).

**Retrieval descriptor.** 2048 floats pooled on the device inside the descriptor graph and read back as `retrieval`:
per-slice generalized-mean pooling (p = 3) of `value_facets`, L2 per slice, concatenation, L2, signed power
normalization, L2. It is the only path: a manifest whose descriptor graph has no `retrieval` output is refused at
load. The pooled output is also what makes the describe pass cheap, since only 2048 floats per image cross the bus.

**Session lifecycle.** `RoMa2Onnx::Load` loads the descriptor graph only; the coarse-match graph loads on the first
`MatchCoarse` call on the same provider, so a retrieval-only run never opens it. The `layers` of an image are not
cached across the describe pass and the pair pass: the pair pass describes again on a slot load.

### FlexGlue

FlexGlue (a DINOv3 ConvNeXt-B backbone, a stride-8 feature head and a 9-layer transformer matching two feature maps)
runs from three graphs written by the export script `export_flexglue.py`:

| graph | inputs | outputs |
|---|---|---|
| `flexglue_descriptor.onnx` | `image` [1,3,512,512] RGB in [0,1] (squashed bilinear, no antialiasing) | `features` [1,256,64,64], `retrieval` [1,256] |
| `flexglue_dense.onnx` | `features0/1`, `image_size0/1` | `warp0/1` [1,128,128,2], `confidence0/1` [1,128,128], `covisibility` [1], `pooled0/1` [1,512,16,16] |
| `flexglue_pose_h0.onnx` | `pooled0/1`, `intrinsics0/1` [1,4] (fx, fy, cx, cy in the 512 frame) | `R10` [1,3,3], `t10` [1,3] |

- **Warps**: FlexGlue's dense mode at coarse stride 4, both directions on a 128 x 128 grid, targets in normalized
  [-1,1] coordinates of the original frame (RoMa2's convention). A cell's confidence is 0 where it fails FlexGlue's own
  checks (confidence under 0.1, forward-backward cycle error over 3 descriptor pixels), so `minConfidence` (0.1)
  keeps exactly the cells FlexGlue keeps.
- **Covisibility**: read from the same transformer pass as the warps, no extra call.
- **Pose head**: FlexGluePose's H0 head on the final pair-conditioned features, average-pooled to 16 x 16 inside the
  dense graph, with the two cameras' rays; one small extra graph call per pair. It needs a pinhole camera for every
  image (a spherical scene is refused by name at the start of the pass). Its output follows `x1 = R10 x0 + s t10`.
- **Retrieval descriptor**: the `features` map GeM-pooled (p = 3, clamped at 1e-6) and L2-normalized inside the
  descriptor graph, 256 floats, stored where RoMa2's 2048 go. No RoMa2 model is loaded in a FlexGlue run.
- **Descriptor tensors** (256 x 64 x 64 floats per image) live on the device and are bound into the dense graph.

Licences: the code of FlexGlue and FlexGluePose is MIT. The weights file contains the DINOv3 ConvNeXt-B backbone,
distributed under the DINOv3 License (Meta; use-restricting, not MIT), which the descriptor graph and the dense graph's
input features carry. The model files are read from a folder the user names and are never built in.

### Providers

`OnnxModel::Load` tries providers in order CUDA, CoreML, DirectML, CPU, each only if the ONNX Runtime build lists it,
and keeps the first that constructs a session; a failed candidate logs a warning and the next is tried. A provider
named explicitly by `--roma2-provider` is a requirement: if unavailable or failing, the load fails and the run stops
instead of finishing on the CPU. With `auto`, `--gpu-device` selecting the CPU (`-2`) forces CPU, and the CUDA provider
uses the device `SEACAVE::CUDA` selected. Each session logs its provider when loaded.

## Pair graph and slot plan

The candidate list holds each unordered pair once (`MakePairIdx` orders the indices; `MatchPairsROMA2` sorts, removes
duplicates, and drops pairs the scene already stores or whose images lack a camera or descriptors, counting the two
causes apart). One graph call states both directions of a pair, so only the pair (A, B) with A < B is warped.

`MakeSlotPlan` schedules which image descriptor tensors stay resident while the pairs are warped one at a time: Belady's
optimal replacement (evict the resident image whose next use is furthest away, or never needed again) over the pairs
sorted by `(ID1, ID2)`, at most `--roma2-slots` slots. Consecutive pairs share their first image, so every use of an
image falls in a short window and a modest budget rarely reloads. The plan is computed once and replayed: the
describe-and-load of a slot runs on the calling thread (a session is driven by one thread; the image load and
preprocessing are prefetched on the pool in a ring keyed by submission order), while each pair's verdict, guided
match and assembly run on the pool, bounded by a semaphore on live warps. Assembled pairs are stored serially in
`(ID1, ID2)` order, because the keypoint indices `StorePairROMA2` hands out depend on what the two images already
carry. The budget is a pure cost knob: the same pairs are stored whatever it is. The plan and the loads and reloads it
costs are logged with the pass summary.

## One-pass matching

`Scene::MatchPairs` runs the pass as the whole matching round (`PairsMatcher::Match` calls `MatchPairsROMA2` instead of
the descriptor round; the verification-feedback round, `PairSelection.md`, runs the same pass on its proposals). Per
candidate pair:

1. **Warp** (`MatchPair`): both directions in one call. The covisibility gate acts here.
2. **Verdict** (`JudgePairROMA2`). One geometry is fitted through `PairsMatcher::GeometricFilter` on a
   coverage-uniform sample of A's eligible cells (`SampleWarpByCoverage`, target 4000; fewer than 8 rejects the pair).
   A cell is eligible when its confidence reaches `minConfidence` and the point it warps to lies inside the other image
   (`IsWarpCellEligible`, the one predicate every scan of a warp uses). The tolerance of the fit and of every test on
   warp cells is half a warp cell in pixels (`WarpTolerance`, a function of the image sizes and the grid, not a
   setting). The inlier area of A is the share of the whole warp grid whose eligible cells the geometry explains
   (Sampson distance within the tolerance); the inlier area of B is measured against the same geometry on B's
   eligible cells through the B-to-A warp. The pair is admitted iff `min(inlierAreaA, inlierAreaB) >= minOverlap`.
3. **Guided sparse matching** (`TrackKeypointsByWarp`, then `MatchFeaturesGuided` with a `GuidedSearch`). A's described
   keypoints are tracked into B through the warp. For each, the candidates are B's described keypoints in a band along
   the keypoint's epipolar line under the verdict's geometry, centred on the warp's prediction: `guidedBandLengthCells`
   warp cells either way along the line and `guidedBandHalfWidthCells` either way across it (the disc of the length's
   radius without a geometry, with a half-width of 0, or within two lengths of the epipole). The winner is the
   smallest descriptor distance. It is accepted iff it beats its closest rival in the band by the matcher's ratio
   (`MatchConfig::matchRatio`; a rival is any candidate farther than `guidedSameFeatureDistance` from the winner), and
   also beats the closest described keypoint of B outside the band by the same ratio (the reference outside, asked of
   every winner, from the thread's k-NN query over all of B's descriptors). Only described keypoints take part. This is
   where appearance enters, and the only place.
4. **Dense fill** (`SampleWarpComplementary`). `DenseFillGridSide` turns the density into a fixed bucket pitch,
   `ceil(sqrt(denseMatchesPerFrame))` clamped to the warp side, over the whole warp grid: the same pitch for every
   pair, so a pair's draw has the same density per area whatever its overlap, and two pairs sharing image A put their
   samples on the same lattice, which is what lets dense matches chain into tracks longer than two views. A bucket
   holding a guided match draws nothing, so the draw is the density over the uncovered overlap. `DenseFillCeiling`
   bounds it at that density over the smaller of the two inlier areas, which is what bounds the keypoint density in
   B; where it binds, `ThinSampleByLatticePriority` keeps a prefix of the same pair-independent order the bucket
   winners were picked on, so tighter and looser ceilings keep nested samples of A.
5. **One geometry for the pair** (`AssemblePairROMA2`, through `FitAndClassifyPairROMA2`). `GeometricFilter` runs once
   more on guided plus dense at the matcher's `maxEpipolarError`, then `RefineRelativePose` polishes the pose with
   each dense correspondence at `denseFitWeight`. The classification keeps the guided matches within the matcher's
   epipolar tolerance as the pair's sparse (descriptor) segment and the dense correspondences within
   `denseEpipolarErrorFactor` times that tolerance as its dense segment (through E on the bearings when a camera has a
   lens and both trust their intrinsics, else the pixel F). With the verification off (`maxEpipolarError` 0) the warp's
   half cell is the dense bar. If the union fit fails, the verdict's own geometry stands, so an admitted pair is always
   stored; a pair with zero sparse inliers is dense-only, its dense segment its whole evidence.
6. **Store** (`StorePairROMA2`): created, or replacing a same-key pair, then the dense segment is appended
   (`AppendDenseMatches`: new dense keypoints past each image's described prefix, matches placed inside the
   track-forming prefix of `ImagePair::matches` and outside the descriptor-evidence count).

The pair's `matches` is partitioned `[sparse inliers | dense | RANSAC inliers the strict filter rejected]`
(`ImagePair`); the sparse count is the descriptor evidence, the first two segments form tracks, and the view graph
ranks on the sparse plus the dense count scaled by the dense pseudo-count.

At verbosity 3 each judged pair logs one line (`REJECT`, or `ADMIT` with the draw's ceiling, pitch, band half-width in
pixels and the guided, sparse and dense counts), and the pass logs a summary: candidates, judged, admitted, stored,
dense-only, slots, loads, reloads, pairs already stored, unprepared, failed loads, failed matches, dense matches. A
pair that was already stored is the ordinary outcome of a re-run or of the feedback round; an unprepared candidate is
an earlier stage's failure shrinking the view graph.

### Why the min-side inlier area

A hallucinated warp is a smooth field, locally a homography, and every homography is explained exactly by a family
of fundamental matrices, so a RANSAC inlier count or ratio on the warp's own cells cannot tell a hallucination from a
true pair, and a lower confidence floor only makes it worse. The inlier area of one geometry can, and the B side
supplies the precision: it rejects the pairs that put the whole of A on a few pixels of B. The measured min-side area
is 0.55-0.75 of the true overlap, so 0.10 asks for roughly 0.15-0.17 of true overlap and 0.15 for about a quarter of
it.

### Why the band

The band takes only its direction from the geometry and its position from the warp, so it survives an imprecise focal
or principal point and unmodelled distortion (the prediction is in B's own pixels; the direction is off by a few
degrees at most). Rivals along the line, which the epipolar test cannot see, are refused by the ratio inside the band;
an impostor off the line cannot take the search from the true candidate; and the reference outside the band for every
winner refuses the random winner of a handful of candidates where the true keypoint was never detected. What still
gets through, a component holding one image twice, is the track builder's to resolve (`TrackBuilding.md`).

## Re-verification and the planar twin switch

When a camera changes after matching (a focal moved by the focal check, a lens fitted, a focal calibrated),
`PairsMatcher::ComputeRelativePoses` verifies again every pair of that camera. A pair the one pass matched (it carries a
dense segment, or an image of it carries dense keypoints) goes through `ReverifyPairROMA2` instead of
`GeometricFilter`: its sparse candidates (the sparse segment, the RANSAC inliers a strict filter rejected, the outliers)
and its dense segment are classified again, kept apart, by `FitAndClassifyPairROMA2` on a geometry fitted afresh. A
match with a dense end is a dense candidate wherever the pair holds it. A pair whose re-fit fails is invalidated, as
`GeometricFilter` leaves a pair it cannot verify. `GeometricFilter` over all of a pair's matches would hold the dense
samples to the descriptor estimator at full weight, which is why the pair is not passed through it.

**Planar twin switch** (`SwitchPlanarTwinsROMA2`, called by `Scene::MatchPairs` right after the focal and lens stage,
on a fresh run and on a reload alike, before anything reads the poses). The matches of a pair that lie mostly on one
plane admit two relative poses that explain them equally well, the stored one and its planar twin. For every posed
pair of the one pass, `PlanarTwinROMA2` fits a homography (OpenCV RANSAC, 2000 iterations, on the cameras' normalized
planes at the angular equivalent of `maxEpipolarError`) on its track-forming matches; when it explains at least
`twinMinPlaneShare` of them, its physically valid decompositions (90% of the homography's inliers in front of both
cameras) give the stored pose (the nearest) and the twin (the other), polished with the union fit's polish; a twin
within 0.5 deg of rotation and 10 deg of direction of the stored pose is the same pose and no candidate. The rotation
loops decide: a triangle is a candidate (a, b) and an image c with (a, c) and (b, c) posed, closed when its loop
rotation is within `twinMaxLoopAngle`. A candidate's gain is its triangles closed after switching minus before, every
other pair at its current state, so twins that depend on each other are judged with each other's current choice. The
largest gain switches when it is at least `twinMinLoopGain` (ties: more sparse inliers, then the larger `(ID1, ID2)`),
the gains of the candidates sharing a triangle with it are counted again, and this repeats until no gain reaches the
margin; a pair may switch back, and each switch closes at least the margin more triangles, so the loop ends. A pair
that ends at its twin takes the twin's pose, E and F and re-classifies its candidates against them
(`ReverifyPairROMA2` without the refit); its weight is recomputed afterwards.

## Invariants

- `DescribeImage` returns exactly `RetrievalSize()` floats of unit norm; a warp is `GridSize()` square with
  confidence in [0,1]; a covisibility is in [0,1] and a pose prior is a rotation and a unit direction (asserted in
  `FlexGlueMatcher::MatchPair`).
- Global descriptors of the other matcher (RoMa2 2048-D, FlexGlue 256-D) are never used: `Scene::MatchPairs` compares
  the stored size with the selected matcher's and describes the images again on a mismatch. A scene that already carries
  matching descriptors skips the describe pass and loads no model for RETRIEVAL alone.
- A requested model that is unavailable is an error naming it, never a silent fallback to another ranking or matching
  path. `CreateStructure` checks what it can before feature extraction runs; `Scene::MatchPairs` checks the rest,
  because only it knows whether the scene already carries descriptors.
- A pair the gate drops, like one the verdict rejects, is not stored; a later round that proposes it warps it again.
- A matcher with `NeedsIntrinsics()` runs only on pinhole cameras.
- Verdicts, guided matches and assemblies run in parallel but are stored serially in `(ID1, ID2)` order; RANSAC is
  seeded, so a pair's verdict and guided match set are a pure function of its inputs (the warp itself comes from the
  model's arithmetic on the chosen provider).
- The classification of a re-verified pair assumes a dense match is recognized by a dense keypoint at an end. The
  redundant-keypoint filter can collapse both ends of a dense match onto described keypoints, after which the strict
  filter's re-derivation reads that match as sparse; the debug check of the partition (`CheckSparseSegmentIsDescribed`)
  accepts this one-directional gap and runs only while the strict filter is on. With the strict filter off, re-verified
  dense matches may sit inside the sparse count.

## Validation

Both matchers from images at 8 threads (right / wrong = images registered right or wrong against the capture's
reference; F1 = the pose score; describe and matching passes in seconds; GPU peak = the process's memory).

| capture | matcher | right / wrong | F1 | describe pass | matching passes | GPU peak MiB |
|---|---|---|---|---|---|---|
| 4e5c6f6d | RoMa2 | 249 / 30 | .9132 | 64 | 860 | 9718 |
| 4e5c6f6d | FlexGlue + gate 0.2 | 258 / 21 to 278 / 0 (two runs) | .9231 to .9690 | 4 to 6 | 353 to 398 | 4160 |
| 1c65aab4 | RoMa2 | 513 / 0 | .9857 | 22 | 634 | 10152 |
| 1c65aab4 | FlexGlue + gate 0.2 | 514 / 0 | .9865 | 6 | 613 | 4448 |
| 5ada248e | RoMa2 | 523 / 5 | .9488 | 23 | 664 | 10698 |
| 5ada248e | FlexGlue + gate 0.2 | 529 / 4 | .9564 | 12 | 782 | 4492 |
| Palace | RoMa2 | 496 / 1 | .8116 | 46 | 755 | 9962 |
| Palace | FlexGlue + gate 0.2 | 492 / 1 | .7954 | 7 | 815 | 4296 |

The FlexGlue export matches PyTorch to 2.6e-5 in descriptor features and 0.9999999 in retrieval cosine; the pose graph
differs from FlexGluePose's own head by 0.20 deg in rotation and 0.43 deg in direction. The retrieval descriptor
choice (feature map, GeM p = 3, 256-D) reaches recall 0.165 at K = 10 against RoMa2's 0.171 (mean over 4e5c6f6d,
1c65aab4, 5ada248e and Palace).

## Rejected alternatives

- Guided search in a disc around the prediction: its rivals sit along the epipolar line where the epipolar test cannot
  see them, and an impostor off the line takes the disc from the true candidate; the epipolar band replaced it.
- Asking the outside reference of lone winners only, or of none: the band then admits matches SIFT does not have in
  floods, which glue most keypoints into one component; every winner answers.
- A band half-width adapted to the verdict's epipolar residual: cleaner but loses a fifth of the matches SIFT also has,
  costs time, and floods like the fixed width without the full reference.
- Holding a dense correspondence to half a warp cell instead of the matcher's epipolar bar: it pollutes every track it
  enters.
- Gate-and-supplement (a warp validating a pair before descriptor matching and supplementing a weak one afterwards): the
  inlier ratio is the wrong variable; the one pass judges, matches and fills a pair from one warp.
- An inlier count or ratio as the verdict, and cycle-consistency tests: the min-side inlier area separates hallucinated
  warps without them.
- A stride through each pair's own sample to enforce the ceiling: its survivors depend on the pair, which cuts the
  chaining of dense matches into tracks.
- The RoMa2 refiner stages and FlexGlue's 512 x 512 refiner: not exported.
- FlexGlue's sparse matcher graph for the covisibility alone: the dense graph returns the score with the warps.
- A pose prior as a filter or corrector of the pair's pose, and a covisibility filter after the contradiction cut: not
  done; the gate before the verdict is the only use of the score.
- Pooling the backbone's last stage for the FlexGlue retrieval descriptor: lower recall than the feature map.

## Open items

- The pose prior (`ImagePair::posePrior`, FlexGlue only) is stored and exported to the pairs CSV but read by nothing.
- The pose head was trained on 32 x 32 token maps and is fed the 64 x 64 maps average-pooled to 16 x 16; the input
  resolution is not re-confirmed.
- The verdict does not tell apart two images of different instances of a repeated structure (both give a consistent
  correspondence set); the triplet scores and the contradiction cut downstream do (`TripletScores.md`,
  `ContradictionCut.md`).
- A planar, textureless, small-baseline pair degenerates the essential matrix (the warp is right, the pose is not);
  `PairsMatcher` has no homography branch. The planar twin switch resolves the pairs whose loops decide.
- CoreML and DirectML latency is unmeasured.
- FlexGlue's coarser warp grid (128 cells against 160) is the candidate cause of its lower F1 on Palace.
- `--roma2` help text names DINOv3/GeM retrieval; with `--dense-matcher flexglue` the retrieval is FlexGlue's pooled
  256-D descriptor.
