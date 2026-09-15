# Semi-Global Matching Densification

## 1. Purpose and scope

`STEREO::SemiGlobalMatcher` (`libs/MVS/SemiGlobalMatcher.h/.cpp`) is the CPU alternative to
PatchMatch for dense depth estimation, selected with `DensifyPointCloud --fusion-mode -2`
(`-1` only estimates and exports the pair disparity-maps). Per image, every selected neighbor view
is stereo-rectified with it and the pair is matched coarse to fine with Semi-Global Matching; the
pair disparity-maps are then fused into the image's depth-map, which enters the standard
depth-map pipeline (optional optimization, then the depth-map fusion of `DepthMapFusion.md`)
exactly as a PatchMatch depth-map would.

The implementation follows two papers: the cost aggregation is the memory-efficient variant of
*Semi-Global Matching* (H. Hirschmüller, PAMI 2008; *Memory Efficient Semi-Global Matching*,
Hirschmüller, Buder, Ernst, ISPRS 2012) and the coarse-to-fine scheme is tSGM from *SURE:
Photogrammetric surface reconstruction from imagery* (M. Rothermel, K. Wenzel, D. Fritsch,
N. Haala, 2012). The driver is `Scene::DenseReconstruction` in `libs/MVS/SceneDensify.cpp`
(`nFusionMode < 0`), which creates the matcher's worker threads, calls `Match` then `Fuse` per
image, and hands the depth-map on.

Scope: everything from the rectified pair to the per-image depth-map and confidence-map. The
view selection (`Scene::SelectNeighborViews`), the rectification (`Image::StereoRectifyImages`)
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
the previous level is valid and consistent, wider where it is invalid, so the work at full
resolution is a small constant per pixel instead of the whole range, and the aggregation at each
level is regularized by the level below. The per-pixel ranges are what makes the cost and
aggregation buffers variable-length: every pixel owns a slice `[idx, idx+numDisp)` of a flat cost
array (`PixelData`), and the path recursion intersects the ranges of consecutive pixels.

### 2.2 Per-image driver (`SemiGlobalMatcher::Match(scene, ...)`)

For image `L`:

1. The sparse points seen by `L` give its depth range `[0.9·dMin, 1.1·dMax]` and the list of
   points shared with each neighbor. An image without points is skipped.
2. Neighbors are visited in score order, the same subset PatchMatch uses (`nNumViews` at most,
   score at least `max(fViewMinScoreRatio·best, fViewMinScore)`). A pair whose `.dimap` exists in
   either direction (from a previous run or from the neighbor having been processed earlier) is
   skipped: a pair is matched once and used by both images.
3. The pair is stereo-rectified around the projections of its shared points
   (`Image::StereoRectifyImages` returns the rectified color images, the validity masks and the
   rectification homography `H` and reprojection matrix `Q`). The gray images are float in [0,1].
4. The pyramid depth is chosen so that the coarsest level is at least `minResolution` (320) pixels
   wide and at most half the rectified size.
5. The level loop of §2.3 runs, then the sub-pixel refinement (§2.6), then the left disparity-map,
   its aggregated cost, `H`, `Q` and the sub-pixel step are written to `<L>_<R>.dimap`.

### 2.3 Level loop

At every level both directions are matched, right-to-left first, then left-to-right, each with its
own per-pixel range map. What differs is how the ranges are set:

- **Coarsest level.** The masks are resized to the level and cropped by the half window.
  `DepthRange2Disparity` projects every 4th valid pixel back through the level's `H⁻¹` and
  converts the two depth bounds to disparities through `Q⁻¹`; the span of all samples, widened by
  the sampling step and clamped to ±width, is the global range. `Range2RangeMap` gives every valid
  pixel that range clamped per column to the disparities keeping the patch inside the other image;
  the right image searches the negated range.
- **Finer levels.** The masks are upscaled 2x (`UpscaleMask`, offset by the half window so the
  valid areas of the two levels coincide). The cross-checked left map of the previous level is
  flipped into a right map (`FlipDirection`: each valid `d` writes `-d` into the three right pixels
  around `c+d`) and each direction's range map is built by `Disparity2RangeMap`: for every
  previous-level pixel, the valid disparities in a 7x7 window (41x41 if the pixel itself is
  invalid) give a median and a min/max; the range at twice the scale is `[2·min, 2·max]` around
  `2·median`, at least 5 disparities wide and capped at 32 (64 for invalid pixels) by keeping the
  part around the median. Fewer than 3 valid disparities in the window, or an invalid mask, leave
  the pixel unsearched. Pixel `(r,c)` of the previous level covers the 2x2 block at
  `(2r+halfWindowSizeY, 2c+halfWindowSizeX)` of the new one.

After both directions are matched, the left map is cross-checked against the right one
(`ConsistencyCrossCheck`: `|dL + dR(c+dL)| ≤ 1`, else invalid). The coarsest level is filtered
harder because it fixes the validity masks of every finer level: the right map is cross-checked
too, both maps are speckle-filtered (`cv::filterSpeckles`, `nSpeckleSize` pixels, 5 disparities)
and `ExtractMask` removes from each mask the border regions of every row that hold fewer than 3
valid disparities. A level without a single searchable disparity in either direction abandons the
pair.

### 2.4 Matching cost (`Match(left, right, ...)`, first block)

Default `SGM_SIMILARITY_WZNCC`: weighted zero-mean NCC over a 7x7 window. For each left pixel the
window weights are bilateral, `exp(-|ΔBGR|²/(2·(0.3·255)²) - (Δx²+Δy²)/(2·(0.4·7)²))` from the
left color image, and the weighted, mean-removed left patch is precomputed once. For every
disparity of the pixel's range the right patch at `(c+d, r)` is read row by row (the patch rows are
always inside the image; a patch whose columns fall outside costs 255) and

    ncc  = Σ w·(I_R - mean_w(I_R))·(I_L - mean_w(I_L)) / sqrt(var_w(I_L)·var_w(I_R))
    cost = ncc ≤ 0 ? 255 : round((1 - min(ncc,1))·255)

The variance product has no regularization term: the gray images are in [0,1], so the product is
of order 1e-5 for textured patches, and any epsilon large enough to matter drowns the NCC of every
patch. A patch with a variance product at or below 1e-16 is textureless and costs 255.

`SGM_SIMILARITY_CENSUS` (compile-time alternative): 7x9 Census transform on 8-bit gray, cost is
4x the Hamming distance.

### 2.5 Aggregation and winner-take-all

The aggregated costs are `uint16`, one slice per pixel like the costs, zeroed before the sweep.
The 8 paths are 4 direction pairs: every path family is a set of independent lines (columns for
the vertical paths, rows for the horizontal, columns plus rows for the diagonals) dispatched to
the thread pool; each line walks its pixels with two rolling `LineData` buffers (previous and
current pixel's `L` slice and range). The single-threaded path (`nMaxThreads == 1`) runs the
classic two raster passes, forward with 4 directions then backward with the 4 opposite ones,
keeping one line of state per direction.

Per pixel step (`pixelAccum`), with `Lp` the previous pixel's slice over its range `Rp` and `Ls`
the current slice over `Rs`:

- `P2` is adaptive: `P2·(1 + 14·exp(-ΔI²/(2·38²)))`, `ΔI` the gray difference along the path in
  0..255 (a table of 256 values, `GenerateP2s`), so the large-jump penalty is 15x stronger inside
  flat regions than across edges. `P1` is constant.
- If `Rp` and `Rs` do not intersect, `L(d) = C(d) + P2` over `Rs` (no previous information).
- Otherwise `minLp` is the minimum of `Lp` over the intersection and
  `L(d) = C(d) + min(Lp(d), Lp(d-1)+P1, Lp(d+1)+P1, minLp+P2) - minLp`, each neighbor term only
  where `d`, `d±1` fall in the intersection.

The sum of the 8 paths is the pixel's aggregated cost; winner-take-all picks the minimum over the
range, and its value is kept as the pixel's cost (`AccumCostMap`).

### 2.6 Sub-pixel refinement (`RefineDisparityMap`)

Only after the finest level, on the aggregated cost of the winning disparity and its two
neighbors: the smaller of the two cost differences, normalized by the larger, is mapped by the
selected fit (`SUBPIXEL_LC_BLEND` by default, a blend of the linear and cosine fits) to an offset in
(-0.5, 0.5); at the range ends a two-value estimate is used. The result is stored as an `int16`
disparity in quarter-pixel units (`subpixelSteps = 4`).

### 2.7 Pair fusion into the image depth-map (`Fuse`)

For image `L`, every neighbor of the same subset as `Match` is loaded from its `.dimap`. A pair
matched from the neighbor's side is reused by remapping `Q` with the relative pose,
`Q' = P·K_R⁻¹·Q`, so the same disparity projects into `L`.

`ProjectDisparity2DepthMap` turns a pair disparity-map into a depth-map in the un-rectified
image: each disparity gives a depth and an image position `u` through `Q`; the depth is trusted
in the depth interval of `d∓1` disparity (`DepthRange`); the sample is scattered to its up to 4
nearest pixels, kept per quadrant when closer than the previous sample (overlap border 0.75 px).
Per pixel, the closest of the 4 samples (within 0.75 px) is the center and every sample within 2%
of its depth is averaged with its range and confidence, weighted by distance. The confidence is
`1 - cost/(8·255)`: one minus the mean cost per path, in [0,1] like PatchMatch's NCC-based
confidence, so the downstream gates (`1 - fNCCThresholdKeep`) apply unchanged.

The pair depth-maps are then clustered per pixel, rows in parallel: a depth joins every cluster
whose current range contains it (the cluster range shrinks to the intersection), else starts a
new one; the largest cluster's depths and confidences are averaged. A single pair suffices
(`minViews = 1`): a depth seen by one pair is still cross-checked across images by the depth-map
fusion. The driver then estimates the normal-map from the depth-map when `nEstimateNormals == 2`
and resets the image's depth bounds.

### 2.8 Threading and memory

The matcher owns a static `EventThreadPool` of `nMaxThreads` workers (created by
`DenseDepthMapData` for the SGM fusion modes) and a semaphore; every phase queues one job per
worker that pulls line or pixel indices from a shared atomic counter, and waits for all of them.
The densification loop estimates one image at a time, so the pool is the only parallelism. The
per-level buffers are `imagePixels` (16 bytes per pixel), `imageCosts` (1 byte per pixel and
disparity) and `imageAccumCosts` (2 bytes per pixel and disparity); the coarsest level, searching
the full range, dominates memory but at a quarter of the pixels or less.

---

## 3. Parameters and defaults

| parameter | where | default | meaning |
|---|---|---|---|
| `--fusion-mode` | app | `0` | `-2` SGM densification, `-1` export the pair `.dimap` only |
| `nNumViews`, `fViewMinScore(Ratio)` | `OPTDENSE` | shared with PatchMatch | neighbor subset matched and fused |
| `minResolution` | `Match` | 320 | minimum width of the coarsest level; the top level is never the full resolution |
| window | header | 7x7 (`halfWindowSize` 3) | WZNCC patch; Census uses 7x9 |
| `P1`, `P2` | ctor | 18, 24 | smoothness penalties on the 0-255 cost scale |
| `P2alpha`, `P2beta` | ctor | 14, 38 | `P2·(1+alpha·exp(-ΔI²/(2·beta²)))` |
| `subpixelMode`, `subpixelSteps` | ctor | `LC_BLEND`, 4 | sub-pixel fit and quantization |
| `thCross` | `ConsistencyCrossCheck` | 1 | left/right consistency tolerance in disparities |
| `nSpeckleSize` | `OPTDENSE` | 100 | speckle filter at the coarsest level only |
| `thValid` | `ExtractMask` | 3 | valid disparities that end a masked border region |
| neighborhood, caps, floor | `Disparity2RangeMap` | 7x7 / 41x41, 32 / 64, 5 | per-pixel range from the previous level |
| depth range | `Match` | 0.9x / 1.1x of the sparse depths | disparity range of the coarsest level |
| trust range | `ProjectDisparity2DepthMap` | ±1 disparity | interval two pair depths must share to cluster |
| `minViews` | `Fuse` call | 1 | pairs a cluster needs |

The constructor defaults are the only ones; `DenseDepthMapData` constructs the matcher without
arguments.

---

## 4. Invariants and constraints

- A `.dimap` is written once per unordered pair and read by both images; `Fuse` visits the same
  neighbor subset as `Match`, so a missing file is a warning, not an expected state.
- Every disparity-map, cost-map and mask of a level is the level size minus the 7x7 border; the
  exported `.dimap` restores the border (`ExportDisparityDataRawFull`) so its size is the rectified
  image's.
- Range maps: `pixel.idx` slices are contiguous in raster order and `maxNumDisp` bounds every
  slice, which sizes the rolling line buffers of the aggregation.
- `P2 ≥ P1` for every `ΔI`, required by the constant-time recursion.
- The right-to-left direction is always matched first so the left match can use the cost buffers
  sized for it; both directions resize the buffers to their own total.
- The gray image type follows the similarity: float [0,1] for WZNCC (the variance threshold
  1e-16 and the `ΔI·255` lookup depend on it), 8-bit for Census.

---

## 5. Validation of the shipped defaults

Two EPFL ground-truth scenes, `--resolution-level 1`, F-score of the dense point-cloud against
the laser-scanned ground truth at the scene's tolerance (visibility-restricted completeness, the
`bench/eval_mesh2mesh.py` metric); walls on a 24-thread workstation.

| scene | SGM before the rework | SGM now | PatchMatch CUDA | PatchMatch CPU |
|---|---|---|---|---|
| Herz-Jesu-P8, τ 1 cm | F 0.13, 64 s | F 0.33, 33 s | F 0.40, 6 s | F 0.40, 85 s |
| fountain-P11, τ 0.5 cm | F 0.11, 111 s | F 0.26, 63 s | F 0.25, 9 s | F 0.27, 150 s |

SGM's precision matches PatchMatch's; the gap is recall (0.22 vs 0.28 on Herz-Jesu-P8). The
rework found the cause of the earlier recall in the cost, not in the range limitation: a
regularization epsilon of 1e-3 under the square root of the variance product, three orders of
magnitude above the product itself for [0,1] intensities, pushed nearly every NCC toward zero,
made the winner-take-all ambiguous and let the cross-check discard two thirds of the pixels at
every level. Removing it alone took Herz-Jesu-P8 from 0.13 to 0.31; the penalties (3/4 to 18/24)
and the symmetric ±1 trust range gave the rest. The numbers in §6 are F on Herz-Jesu-P8 unless
stated, run-to-run noise ±0.002.

---

## 6. Rejected alternatives

- **Seeding the coarsest level from the sparse cloud** (the original design: the sparse points'
  disparities define per-pixel ranges and a mask at the top level). With the random virtual cloud
  of a cameras-only scene it loses a third of the F-score (0.21 vs 0.31: its mask discards 70% of
  the image). With a real SfM cloud from the ground-truth poses it equals the full-range search
  (0.1247 vs 0.1258, both offset by the 1.6 cm the bundle adjustment moved the poses) for 10% less
  time. Removed: the full-range top level is robust to the sparse cloud's density.
- **Texture-scaled epsilon** in the NCC (1.6e-7): 0.289, worse than none.
- **Coarser top level** (160 px): -0.011 F for -20% time.
- **Census cost**: 0.295 vs 0.331, 12% faster; kept as a compile-time option.
- **Sub-pixel steps 8 or 16** and a **speckle filter after the finest level**: within noise.
- **Penalty scale**: x1 0.308, x4 0.317, x6 0.319, x8 0.321, x12 0.330 on Herz-Jesu-P8, but
  x12 loses on fountain-P11 (0.247 vs 0.256 at x4-x6); x6 is the compromise.
- **Trust range**: the earlier asymmetric `[floor(d)-1, floor(d)+1]` 0.308, ±0.5 0.315, ±1
  0.320, ±1.5 0.328 on Herz-Jesu-P8 but flat to negative on fountain-P11; ±1 shipped.

---

## 7. Future work

Ordered by expected gain; the first item on recall is an architecture change, the ones after it
are cheap experiments on the current code. Measure on both EPFL scenes first, then a Tanks and
Temples scene.

**Recall**

1. **Aggregate the neighbors before the winner-take-all.** Today every pair is matched alone,
   cross-checked alone and only then fused, so a pixel occluded or textureless in one pair is
   dropped by that pair and survives only if another pair sees it clearly; the per-pair
   winner-take-all also has no multi-view evidence to resolve an ambiguous minimum. Building one
   cost volume in the reference view instead, over per-pixel depth (or inverse-depth) samples,
   summing the WZNCC of every neighbor at each sample (with the per-view minimum or a robust mean
   so an occluded view does not veto), and running the 8-path aggregation and the winner-take-all
   once, gives every pixel the support of all its views, removes the rectification resampling, and
   yields one depth with a true multi-view confidence. The coarse-to-fine range logic carries over
   unchanged in depth samples. This is the change most likely to close the recall gap to
   PatchMatch, which already scores its candidates against all views.
2. **Confidence from the cost curve, not the cost.** `1 - cost/(8·255)` separates textured from
   textureless pixels, not correct from wrong ones, so the downstream gate keeps bad textured
   estimates and drops good flat ones. A peak-ratio (second-best over best aggregated cost) or
   the left/right disparity residual as confidence would let the fusion keep more of the valid
   pixels and reject more outliers; it also improves precision through the same gate.
3. **Recover instead of discard at the finest level.** The 1-disparity cross-check at every level
   removes slanted-surface pixels whose right-to-left match lands one pixel off; a cross-check
   tolerance scaled to the level (coarse pixels span 4-8 image pixels) and, at the finest level, a
   sub-pixel cross-check after the refinement would keep them. Pixels the check still removes but
   whose aggregated cost has a clear minimum could be kept at low confidence for the fusion to
   decide.
4. **Masks that only shrink.** `ExtractMask` and the speckle filter at the coarsest level fix the
   valid region for good, and a pixel with fewer than 3 valid neighbors in its window is never
   searched again. Letting invalid interior pixels keep the wide range at every level (masking
   only the rectification border) is a one-line experiment.
5. **More pairs per image.** The pair count is PatchMatch's `nNumViews`; SGM's cost is linear in
   it and recall should grow with the union of the pairs. Sweep it.

**Precision**

6. **Plane-fit refinement of the fused depth-map.** The quarter-pixel quantization is not the
   limit (finer steps measured as noise); the fronto-parallel 7x7 window is. A few PatchMatch
   iterations initialized from the SGM depth (the estimator already exists; it needs a prior
   depth-map input) would add normals and sub-quantization depth at a fraction of a full
   PatchMatch run, and would also make SGM a fast initialization for PatchMatch.
7. **Confidence-weighted pair fusion.** The largest cluster is a plain average; weighting each
   pair depth by its confidence and by the width of its trust range (a wide baseline pair is more
   precise in depth) should tighten the fused depth.
8. **Normals for the depth-map fusion.** The SGM depth-maps carry no normals unless
   `nEstimateNormals == 2`; estimating them by default lets the fusion's normal checks apply.

**Speed**

9. **Vectorize the aggregation** over the disparity slice (`uint16` lanes, as OpenCV SGBM and
   libSGM do); the ranges vary per pixel but the recursion is over the intersection, which is a
   contiguous slice. The aggregation is about half of `Match`; 3-5x on it is realistic.
10. **Vectorize the cost inner loop** (7 floats per row, one masked lane) and skip the per-disparity
    right-patch mean by carrying row sums along the disparity sweep.
11. **CUDA port** of cost and aggregation, the classic GPU SGM, which would put the SGM mode next
    to PatchMatch CUDA in wall time.
12. **Parallel `ProjectDisparity2DepthMap`**: the scatter into the 4 quadrant buffers is
    single-threaded (~0.6 s per image); row bands with private buffers remove it from the critical
    path.
