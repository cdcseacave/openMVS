# Semi-Global Matching Densification

## 1. Purpose and scope

`STEREO::SemiGlobalMatcher` (`libs/MVS/SemiGlobalMatcher.h/.cpp`) is the CPU alternative to
PatchMatch for dense depth estimation, selected with `DensifyPointCloud --fusion-mode -2`. Each
image is matched against all its selected neighbor views at once with a hierarchical Semi-Global
Matching over inverse-depth samples (`MatchMultiView`); the resulting depth-map and confidence-map
enter the standard depth-map pipeline (optional optimization, then the depth-map fusion of
`DepthMapFusion.md`) exactly as a PatchMatch depth-map would. `--fusion-mode -1` instead exports
the disparity-maps of the rectified image pairs (`Match`, `.dimap` files), a standalone stereo
product nothing in the pipeline reads back.

The implementation follows two papers: the cost aggregation is the memory-efficient variant of
*Semi-Global Matching* (H. Hirschmüller, PAMI 2008; *Memory Efficient Semi-Global Matching*,
Hirschmüller, Buder, Ernst, ISPRS 2012) and the coarse-to-fine scheme is tSGM from *SURE:
Photogrammetric surface reconstruction from imagery* (M. Rothermel, K. Wenzel, D. Fritsch,
N. Haala, 2012). The multi-view cost is a plane sweep in the reference view in the spirit of
Collins (CVPR 1996) and Gallup et al. (CVPR 2007), with the plane slanted per pixel. The driver is
`Scene::DenseReconstruction` in `libs/MVS/SceneDensify.cpp` (`nFusionMode < 0`), which creates the
matcher's worker threads and calls `MatchMultiView` (or `Match`) per image.

Scope: everything from the posed images to the per-image depth-map and confidence-map. The view
selection (`Scene::SelectNeighborViews`), the pair rectification (`Image::StereoRectifyImages`)
and the downstream depth-map fusion are shared with PatchMatch and documented elsewhere.

---

## 2. Algorithm as implemented

### 2.1 Classic SGM and the hierarchical scheme

Classic SGM matches one rectified pair over a global disparity range `[dmin, dmax)`: a pixelwise
matching cost `C(p,d)` for every pixel and disparity, aggregated along 8 image paths with the
recursion

    L_r(p,d) = C(p,d) + min( L_r(p-r,d), L_r(p-r,d±1)+P1, min_k L_r(p-r,k)+P2 ) - min_k L_r(p-r,k)

summed over the paths into `S(p,d)`, winner-take-all per pixel, sub-pixel interpolation of the
minimum, a left/right consistency check and a speckle filter. Its cost is proportional to the
number of pixels times the number of disparities, and its memory too when the aggregated volume is
kept for the sub-pixel step.

tSGM keeps the algorithm but runs it on an image pyramid: the coarsest level searches the whole
disparity range, every finer level searches per pixel only a small range around the disparities
the previous level found in the pixel's neighborhood. The range is narrow (a few disparities) where
the previous level is valid, wider where it is invalid, so the work at full resolution is a small
constant per pixel instead of the whole range, and the aggregation at each level is regularized by
the level below. The per-pixel ranges make the cost and aggregation buffers variable-length: every
pixel owns a slice `[idx, idx+numDisp)` of a flat cost array (`PixelData`), and the path recursion
intersects the ranges of consecutive pixels.

The multi-view matcher reuses this machinery unchanged by making its "disparities" inverse-depth
samples: along the viewing ray, the projection of a point into another view moves almost linearly
with inverse depth, as the disparity of a rectified pair does.

### 2.2 Multi-view depth estimation (`MatchMultiView`, `--fusion-mode -2`)

For the reference image:

1. **Depth range.** The sparse points seen by the image give `[0.9·dMin, 1.1·dMax]`
   (`SparseDepthRange`); an image without points gets an empty depth-map.
2. **Neighbor views.** The same subset PatchMatch uses (`nNumViews` at most, score at least
   `max(fViewMinScoreRatio·best, fViewMinScore)`), capped at 32. For each, the relative pose gives,
   at every pyramid level, `x_k ~ A·x + invz·b` with `A = K_k·R·K_ref⁻¹` and `b = K_k·t`: the
   projection into view `k` of the reference pixel `x` at inverse depth `invz`.
3. **Inverse-depth samples.** Uniform in inverse depth from `1/dMax`, with the step at full
   resolution set so that one step moves the projection by one pixel in the neighbor where it moves
   the most (measured at the image center and mid depth); every other neighbor moves less. At a
   pyramid level of scale `s` the step is `step0/s`, so index `d` at one level is index `2d` at
   the next, exactly as a disparity. The count is capped so that the quarter-pixel sub-pixel
   indices fit in `int16`.
4. **Pyramid.** The coarsest level is the largest power-of-two reduction that keeps the image at
   least `minResolution` (320) pixels wide, and at least a halving. Each level resizes the
   reference color and gray images and every neighbor's gray image (`INTER_AREA`) and rescales the
   intrinsics.
5. **Level loop** (§2.3), then the sub-pixel refinement (§2.6) and the conversion of every valid
   index to a depth and a confidence.

### 2.3 Level loop

- **Coarsest level.** Every pixel of the valid region (the level minus the 7x7 border) searches the
  whole sample range (`Range2RangeMap` without the per-column clamp of the rectified case).
- **Finer levels.** Before the new level overwrites it, the previous level's map gives each pixel a
  slope (`FitSlopes`: the least-squares plane through the valid indices of its 7x7 neighborhood,
  at least 6 of them, slope clamped to ±4 indices per pixel, zero otherwise). Its ranges come from
  `Disparity2RangeMap` exactly as for a pair: the valid indices of a 7x7 window (41x41 if the pixel
  itself is invalid) give a median and a min/max; the range at twice the scale spans them around
  twice the median, at least 5 samples wide, capped at 32 (64 for invalid pixels); fewer than 3
  valid indices leave the pixel unsearched.
- **Filtering.** There is no second disparity-map to cross-check against. Only the coarsest level
  is speckle-filtered (`cv::filterSpeckles`, `nSpeckleSize` pixels, 5 samples), so a wrong
  isolated region does not seed the ranges of the finer levels; the depth-map fusion later discards
  the depths no other view confirms.

### 2.4 Matching cost

For a pixel and sample, each neighbor view contributes a WZNCC cost, and the sample's cost is the
mean of the two lowest neighbor costs, so a view that is occluded at the pixel, or does not see it,
does not veto the right depth.

- **Window.** 7x7 with bilateral weights from the reference color image,
  `exp(-|ΔBGR|²/(2·(0.3·255)²) - (Δx²+Δy²)/(2·(0.4·7)²))`, sampled at every other texel (the
  16 texels at offsets ±1, ±3), with the weighted, mean-removed reference patch precomputed once per
  pixel (`InitWeightedPatch`).
- **Slanted warp.** Over a plane, inverse depth is affine in the pixel coordinates, so the
  neighbor position of texel `δ` is `h0 + A·δ + b·(g·δ)·step`, with `h0 = A·x + invz·b` and `g`
  the pixel's slope from the previous level (zero at the coarsest level). The slant costs three
  multiply-adds per texel.
- **Sampling.** Bilinear on the neighbor's gray image; a neighbor whose warped patch corners are
  not all inside its image, or behind it, is skipped for that sample (the patch is convex under
  the homography, so its corners bound it).
- **Score.** `ncc = Σw·(I_k-mean)·(I_ref-mean) / sqrt(var_ref·var_k)`, cost
  `round((1-min(ncc,1))·255)`, 255 for `ncc ≤ 0`. The variance product has no regularization
  term: the gray images are in [0,1], so the product is of order 1e-5 for textured patches and any
  epsilon large enough to matter drowns the NCC of every patch. A product at or below 1e-16 is a
  textureless patch and costs 255, as does a sample no neighbor sees.

### 2.5 Aggregation and winner-take-all (`Aggregate`)

Shared with the pair matcher. The aggregated costs are `uint16`, one slice per pixel, zeroed
before the sweep. The 8 paths are 4 direction pairs: each path family is a set of independent
lines dispatched to the thread pool, each line walking its pixels with two rolling buffers (the
previous and current pixel's `L` slice and range). The single-threaded path runs the classic two
raster passes with 4 directions each.

Per pixel step, with `Lp` the previous pixel's slice over range `Rp` and `Ls` the current one over
`Rs`: `P2` is adaptive, `P2·(1 + 14·exp(-ΔI²/(2·38²)))` with `ΔI` the gray difference along the
path in 0..255 (256-entry table), so the large-jump penalty is 15x stronger inside flat regions
than across edges; `P1` is constant. If the ranges do not intersect, `L(d) = C(d) + P2`; otherwise
`L(d) = C(d) + min(Lp(d), Lp(d±1)+P1, minLp+P2) - minLp` with `minLp` over the intersection. The
sum of the 8 paths is the aggregated cost; winner-take-all takes its minimum, whose value is the
pixel's cost (`AccumCostMap`).

### 2.6 Sub-pixel refinement and depth

After the finest level only, in two steps.

1. **On the aggregated costs** (`RefineDisparityMap`), of the winner and its two neighbors: the
   smaller cost difference over the larger is mapped by the selected fit (`SUBPIXEL_LC_BLEND`, a
   blend of the linear and cosine fits) to an offset in (-0.5, 0.5), with a two-value estimate at
   the range ends, and stored in quarter-sample units (`subpixelSteps = 4`).
2. **On the matching cost.** The aggregated costs are integer, quantized to a quarter sample and
   smoothed along the paths, which limits the accuracy at tight tolerances. From that estimate `t`,
   the cost of the whole 7x7 window (all 49 texels, float, unrounded) against the two views that
   match best at `t` is evaluated at `t` and `t ± δ`; `t` moves to the minimum of the parabola
   through the three, clamped to `±δ`, if its cost there is not higher. Two iterations, `δ` 0.5
   then 0.25 samples; the search stops where the three costs are not convex or a view does not see
   the window. The same two views are kept for every evaluation so that the curve is of one cost.

The depth is `1/(invzMin + t·step0)`, unquantized. The confidence (`PeakRatioConfidence`) measures how
unique the winner is: with `best` its aggregated cost and `second` the lowest one among the
samples not adjacent to it, it is `sqrt(1 - best/second)`, zero if there is no such sample (a
finer level searches at least 5) or both are zero. The square root puts the ratio on the scale of
PatchMatch's NCC-based confidence, so the fusion gate `1 - fNCCThresholdKeep` applies unchanged
and drops 2% of the depths. The driver then estimates the normal-map when `nEstimateNormals == 2`
(the default) and resets the image's depth bounds.

### 2.7 Pair disparity export (`Match`, `--fusion-mode -1`)

For each neighbor in the same subset, unless the pair's `.dimap` exists in either direction: the
pair is stereo-rectified around the projections of the sparse points it shares, and matched coarse
to fine in both directions with the same cost (dense 7x7 window, fronto-parallel), aggregation and
sub-pixel refinement. The coarsest level searches the disparity span of the image's depth range
over the valid pixels (`DepthRange2Disparity`), clamped per column to keep the patch inside the
other image; the finer levels use `Disparity2RangeMap` on the flipped (`FlipDirection`)
cross-checked map. Every level cross-checks the left map against the right one
(`|dL + dR(c+dL)| ≤ 1`); the coarsest level also cross-checks the right one, speckle-filters both
and trims the rows' border regions with fewer than 3 valid disparities from the masks
(`ExtractMask`). The left disparity-map, its cost, `H`, `Q` and the sub-pixel step are written to
`<L>_<R>.dimap`; `ImportPointCloud` turns such a file into a point cloud.

### 2.8 Threading and memory

The matcher owns a static `EventThreadPool` of `nMaxThreads` workers (created by
`DenseDepthMapData` for the SGM fusion modes) and a semaphore; every phase queues one job per
worker pulling line or pixel indices from a shared atomic counter, and waits for them. The
densification loop estimates one image at a time, so the pool is the only parallelism. The per-level
buffers are `imagePixels` (16 bytes per pixel), `imageCosts` (1 byte per pixel and sample) and
`imageAccumCosts` (2 bytes per pixel and sample); the coarsest level, searching the full range,
dominates both memory and time. The multi-view matcher also holds the gray images of the
neighbors at full and current resolution.

---

## 3. Parameters and defaults

| parameter | where | default | meaning |
|---|---|---|---|
| `--fusion-mode` | app | `0` | `-2` SGM densification, `-1` export the pair `.dimap` only |
| `nNumViews`, `fViewMinScore(Ratio)` | `OPTDENSE` | shared with PatchMatch | neighbor views matched |
| `minResolution` | `MatchMultiView`, `Match` | 320 | minimum width of the coarsest level |
| window | header | 7x7 (`halfWindowSize` 3) | WZNCC patch; Census uses 7x9 |
| `texelStep` | `MatchMultiView` | 2 | multi-view window sampled at 16 of 49 texels |
| `numBestViews` | `MatchMultiView` | 2 | neighbor costs averaged per sample |
| slope fit | `FitSlopes` | 7x7, at least 6 valid, ±4 | slant of the multi-view window |
| `P1`, `P2` | ctor | 9, 12 | smoothness penalties on the 0-255 cost scale |
| `P2alpha`, `P2beta` | ctor | 14, 38 | `P2·(1+alpha·exp(-ΔI²/(2·beta²)))` |
| `subpixelMode`, `subpixelSteps` | ctor | `LC_BLEND`, 4 | sub-pixel fit and quantization |
| cost refinement | `MatchMultiView` | 2 iterations, `δ` 0.5, 49 texels, best 2 views | sub-pixel search on the matching cost |
| `nSpeckleSize` | `OPTDENSE` | 100 | speckle filter at the coarsest level |
| neighborhood, caps, floor | `Disparity2RangeMap` | 7x7 / 41x41, 32 / 64, 5 | per-pixel range from the previous level |
| depth range | `SparseDepthRange` | 0.9x / 1.1x of the sparse depths | samples of the coarsest level |
| `thCross`, `thValid` | pair export | 1, 3 | cross-check tolerance, border-trim threshold |

The constructor defaults are the only ones; `DenseDepthMapData` constructs the matcher without
arguments, so the pair export uses the same penalties.

---

## 4. Invariants and constraints

- Every disparity-map, cost-map and mask of a level is the level size minus the 7x7 border; the
  multi-view depth-map is written at the full image size with that border left empty, and the
  exported `.dimap` restores the border (`ExportDisparityDataRawFull`).
- Range maps: `pixel.idx` slices are contiguous in raster order and `maxNumDisp` bounds every
  slice, which sizes the rolling line buffers of the aggregation.
- `P2 ≥ P1` for every `ΔI`, required by the constant-time recursion.
- The inverse-depth index doubles from one level to the next because its origin is fixed at
  `1/dMax` and its step halves; `Disparity2RangeMap` and `FitSlopes` rely on it (a slope in indices
  per pixel is the same number at both levels).
- The multi-view neighbor list is reserved before it is filled: its images hold pointers into
  themselves and must not be relocated.
- The gray image type follows the similarity: float [0,1] for WZNCC (the variance threshold 1e-16
  and the `ΔI·255` lookup depend on it), 8-bit for Census; the multi-view matcher requires WZNCC.

---

## 5. Validation of the shipped defaults

Three EPFL ground-truth scenes, `--resolution-level 1`, F-score of the dense point-cloud against
the laser-scanned ground truth at the scene's tolerance (visibility-restricted completeness, the
`bench/eval_mesh2mesh.py` metric); walls on a 24-thread workstation, PatchMatch with the default
geometric iterations. The scenes have cameras only: their virtual point-cloud, and hence the depth
ranges, is seeded, so a run is reproducible to the byte and every difference between arms is real.

| scene | pair SGM + pair fusion | multi-view SGM | PatchMatch CPU | PatchMatch CUDA |
|---|---|---|---|---|
| Herz-Jesu-P8 (8 views, τ 1 cm) | F 0.332, 33 s | **F 0.372, 31 s** | F 0.402, 85 s | F 0.403, 6 s |
| fountain-P11 (11 views, τ 0.5 cm) | F 0.253, 65 s | **F 0.260, 46 s** | F 0.268, 150 s | F 0.252, 9 s |
| Herz-Jesu-P25 (25 views, τ 1 cm) | F 0.466, 145 s | **F 0.503, 130 s** | | F 0.609, 21 s |

Multi-view SGM matches PatchMatch's precision on Herz-Jesu-P8 (0.699 vs 0.697) and gains recall
on every scene over the pair version (0.254 vs 0.223, 0.173 vs 0.167, 0.389 vs 0.353); on
fountain-P11 it leads it at every tolerance, most at 2τ and 4τ (F 0.540 vs 0.497, 0.741 vs 0.711).
The sub-pixel search on the matching cost (§2.6) accounts for 0.007 of the F-score on
fountain-P11 and Herz-Jesu-P25 (0.253 and 0.496 without it) and for 20% of the time; it raises the
precision on all three scenes (by 0.005, 0.020, 0.017) and the F-score at 2τ (0.588 vs 0.580,
0.540 vs 0.532, 0.710 vs 0.701), but costs 0.003 at τ on Herz-Jesu-P8 (0.376 without it), where
the fusion merges the now closer depths into 2% fewer points. At the depth-map level, before fusion, it fills 96% of the pixels, and its
confidence separates good depths from bad ones: on Herz-Jesu-P8 the lowest two deciles are 6-11%
precise at τ, the others 23-62% (ROC-AUC of the confidence predicting a depth within τ: 0.71, and
0.68 on fountain-P11). The remaining gap to PatchMatch is recall and widens with the number of
views (Herz-Jesu-P25).

The pair matcher's poor recall before this design had a separate cause in the cost: a
regularization epsilon of 1e-3 under the square root of the variance product, three orders of
magnitude above the product itself for [0,1] intensities, pushed nearly every NCC toward zero; its
removal took the pair version on Herz-Jesu-P8 from F 0.13 to 0.31, the penalties and the pair
fusion's trust range to 0.33.

---

## 6. Rejected alternatives

All numbers are F on Herz-Jesu-P8 unless stated. Most were measured before the virtual point-cloud
was seeded and carry about ±0.002 of run-to-run noise.

**Architecture**
- **Pair matching and pair fusion** (the previous `-2`): every neighbor rectified and matched
  alone, cross-checked, and the pair depth-maps clustered per pixel by overlapping trust ranges.
  Beaten or tied on all three scenes by the multi-view matcher (§5), and slower, since each pair
  is matched in both directions. The per-pair filters were measured before it was dropped, all
  within noise: cross-check tolerance 2 at any level (0.330-0.331), keeping cross-check failures
  with a clear minimum (0.331), a uniqueness filter on the best-to-second cost ratio at 0.05 and
  0.1 (0.330), the ratio as confidence (0.331), searching the pixels with nothing known around them
  over the whole previous range (0.331), no border trimming of the masks (0.333). The pair
  matcher's recall is not lost in its filters.
- **Seeding the coarsest level from the sparse cloud** (per-pixel ranges and a mask from the
  sparse points' disparities): with the random virtual cloud of a cameras-only scene it loses a
  third of the F-score (its mask discards 70% of the image); with a real SfM cloud it equals the
  full-range search for 10% less time. The full-range coarsest level is robust to the cloud.

**Multi-view cost and filtering**
- **Neighbor costs averaged:** best 1 0.335, best 2 0.354, best 3 0.350, best 5 0.328 (all
  fronto-parallel); best 3 on fountain-P11 0.249 vs 0.252.
- **Uniqueness filter** (best-to-second aggregated cost ratio): 0.03 0.348, 0.08 0.345 vs 0.350.
- **Confidence gate** before fusion: 0.4 0.374, 0.6 0.366 vs 0.375; fountain-P11 0.249, 0.248 vs
  0.252. Precision rises, recall falls more.
- **Confidence from the cost:** `1 - cost/(8·255)`, one minus the mean cost per path, ranks the
  depths as well as the peak ratio (ROC-AUC 0.718 vs 0.714 on Herz-Jesu-P8, 0.676 vs 0.679 on
  fountain-P11) and gives the same F-score (0.375/0.253/0.499 vs 0.376/0.253/0.496 on the three
  scenes), but measures texture, not uniqueness: its top decile is less precise than the middle
  ones. The peak ratio without the square root puts 15-18% of the depths under the fusion gate
  (0.368/0.247/0.488), four times the ratio 0.374.
- **Slanted aggregation** (shifting the previous pixel's costs along each path by the index change
  the slope predicts): 0.371 vs 0.375; fountain-P11 0.248 vs 0.252.
- **Dense window at the finest level, parabola or linear sub-pixel fit:** 0.376, and 0.255, 0.254,
  0.253 on fountain-P11: within noise, and the dense window is 40% slower.
- **Slope fit window** 5x5 vs 7x7: 0.375 vs 0.376.
- **Per-pixel view selection** (ACMM-like: after each level but the finest, every view's cost at
  the pixel's winner, averaged over the valid pixels of a 5x5 window, selects the views the next
  level matches the pixel against): the best 2 views fixed per pixel 0.369, the best 3 0.366; the
  mean of every view under cost 128 0.350 (under 80, 0.357); the best-two mean per sample among
  the views under 128 0.374 (under 100, 0.373; 1x1 window 0.374, 9x9 0.376), vs 0.374-0.376. The
  best-two mean per sample already drops the views that are occluded at a sample, and a set fixed
  per pixel from a coarser winner is wrong wherever that winner is.
- **Photometric refinement variants:** on the 16 matched texels instead of the whole window 0.369,
  and 0.257 on fountain-P11, vs 0.372 and 0.260; one iteration 0.373 and 0.259; three 0.372 and
  0.260; stepping `δ` toward the lower side where the costs are not convex, instead of stopping,
  the same as stopping.
- **Penalties:** x0.25 0.354, x0.5 0.357, x1 (18/24) 0.354, x2 0.339 (fronto-parallel); with the
  slant 18/24 0.369 vs 9/12 0.375.

**Pair matcher (still the `-1` export)**
- **Texture-scaled epsilon** in the NCC (1.6e-7): 0.289 vs 0.31.
- **Coarser top level** (160 px): -0.011 F for -20% time.
- **Census cost**: 0.295 vs 0.331, 12% faster; kept as a compile-time option.
- **Sub-pixel steps 8 or 16**, a **speckle filter after the finest level**: within noise.

---

## 7. Future work

Ordered by expected gain on recall, the gap to PatchMatch.

1. **Depth-map refinement seeded by SGM.** A few PatchMatch iterations starting from the SGM depth
   and normals would add per-pixel slanted planes and view selection, and a geometric-consistency
   pass against the neighbors' depth-maps, which is where PatchMatch's recall advantage grows with
   the number of views. It also makes SGM a fast initializer for PatchMatch.
2. **Speed.** SIMD over the disparity slice in the aggregation and over the texels in the cost; a
   CUDA port of both, which would put the SGM mode next to PatchMatch CUDA in wall time.
