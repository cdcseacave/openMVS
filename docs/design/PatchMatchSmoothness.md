# GPU PatchMatch Smoothness Coupling

## 1. Purpose and scope

The GPU PatchMatch (CUDA `libs/MVS/PatchMatchCUDA.cu`, Metal `libs/MVS/PatchMatchMetal.metal`) scores every
plane hypothesis of a pixel by its photometric (and, in the geometric passes, geometric) cost alone, so on
weak texture neighbouring pixels settle on unrelated planes: the depth-maps are noisy and their normals,
which fusion compares between views, are noisier still. The smoothness coupling adds a pairwise prior to
the per-pixel decision: a hypothesis that agrees with a smoothed plane of the pixel's current neighbourhood
costs up to 20 % less. It is the kernels' behaviour, with no option to turn it off.

Scope: every CUDA and Metal estimation pass (photometric and geometric, every pyramid level, every camera
model). Out of scope: the CPU estimator (`DepthEstimatorT`, `libs/MVS/DepthMap.cpp`), which keeps its own
term (`DENSE_SMOOTHNESS_PLANE`, `fRandomSmoothBonus`: a bonus per agreeing already-estimated neighbour), and
the HIP backend.

## 2. Algorithm as implemented

The coupling is a quadratic relaxation of a pairwise smoothness energy (PM-Huber style): instead of
comparing a hypothesis with each neighbour, every hypothesis pays one robust distance to a per-pixel
smoothed plane that a separate kernel rebuilds from the current estimates.

### Smoothed plane (`SmoothPlanes<MODEL>`)

Launched by `PatchMatch::RunCUDA` over the full grid before every `BlackPixelProcess` and every
`RedPixelProcess`, so each pass sees the smoothed planes of the estimates the previous pass left. Per pixel
`p` it walks the 3x3 neighbourhood `q` (the pixel included) with an estimate (`depth > 0`):

```
w(q)   = exp(-(I(q) - I(p))^2 / (2*(25/255)^2)  -  cost(q)^2 / (2*0.3^2))
n      = normalize(sum w(q)*n(q))
X      = sum w(q)*X(q) / sum w(q)            X(q) = Ray(q)*depth(q), reference camera frame
depthS = n.X / n.Ray(p)                       the plane (n, X) met along the pixel ray
```

`I` is the reference image texel (`FetchRefTexel`), `cost` the stored cost of the neighbour (its
confidence: an unconverged or occluded neighbour stops pulling), the colour term the patch's bilateral
weight (no smoothing across image edges). It stores `(n, depthS)` in `cudaSmoothPlanes` (one `Point4` per
pixel, allocated with the estimates in `AllocatePatchMatchCUDA`), or zero when the neighbourhood has no
estimate, its normals cancel out (`|sum w n| <= 1e-3 * sum w`), the plane is seen edge-on or from behind
(`n.Ray(p) >= -0.01`) or meets the ray behind the camera. An equirectangular reference (`MODEL_SPHERICAL_REF`)
wraps the columns across the seam as the propagation does (`SEACAVE::WrapColumn`, rays from the
`refTrig` tables through `RefPixel::Ray`); every other reference runs the pinhole instance (`MODEL_PINHOLE`,
also for the `MODEL_PINHOLE_REF` passes: the kernel only reads the reference).

### Cost factor (`SmoothnessFactor`, `ProcessPixel`)

With the smoothed plane of the pixel as its point `Xs = Ray(p)*depthS` and normal `ns`, a hypothesis
`(n, depth)` has the relative distance `d = ns.(Ray(p)*depth - Xs) / depth` to it, and its aggregated cost
is multiplied by

```
factor = 1 - 0.2 * exp(-d^2 / (2*0.02^2)  -  (1 - ns.n) / (13 deg)^2)
```

(`1 - cos` stands for `acos^2 / 2`, one `exp` and no `acos` per hypothesis). `ProcessPixel` applies it to
the 8 propagated neighbour planes (`finalCosts`), to the pixel's own plane re-scored with the new view
weights, and to the 3 or 4 refinement candidates; the stored cost is the scaled one. A pixel without a
smoothed plane keeps its plain costs. The per-view costs (`costArray`) that drive the view selection stay
photometric.

The scaled stored cost then also feeds the neighbour choice of the propagation (the lowest cost along each
direction), the weights of the next `SmoothPlanes`, `FilterPlanes` (`fThresholdKeepCost`) and the
photometric confidence, from which the confidence recalibration starts (`DepthMapConfidence.md`).

### Metal

`PatchMatchMetal.metal` mirrors the CUDA code line by line: the constants and `SmoothnessFactor` before
`ProcessPixel`, the kernel `SmoothPlanes` (function constant `MODEL`, `MODEL_PINHOLE` or
`MODEL_SPHERICAL_REF`) reading the planes, costs and reference texture through the common bindings and
writing buffer 10, which `BlackPixelProcess`/`RedPixelProcess` read. `PatchMatchMetal.mm` builds the two
pipelines (`Impl::psSmooth`), allocates the buffer per scale and encodes a full-grid `SmoothPlanes` pass
before every black and red pass of the command buffer.

## 3. Parameters

Compile-time constants next to the kernel (`PatchMatchCUDA.cu`, mirrored in `PatchMatchMetal.metal`); one
jointly validated operating point, not user options.

| constant | value | meaning |
|---|---|---|
| `smoothBonus` | 0.2 | cost reduction of a hypothesis on the smoothed plane |
| `smoothSigmaDepth` | sigma 0.02 | distance to the smoothed plane, relative to the depth (the CPU's `fRandomSmoothDepth`) |
| `smoothSigmaNormal` | sigma 13 deg | normal agreement, on `1 - cos` (the CPU's `fRandomSmoothNormal`) |
| `smoothSigmaColor` | sigma 25/255 | neighbour weight: gray difference to the pixel (the patch bilateral sigma) |
| `smoothSigmaConf` | sigma 0.3 | neighbour weight: its stored cost (the view-selection sigma) |

## 4. Invariants

- `SmoothPlanes` runs before every black and red pass on the planes and costs of the same stream, so a pass
  never reads a smoothed plane older than the previous pass, and never one of another depth-map.
- A stored smoothed plane is either zero (none) or a unit normal facing the pixel ray with a positive depth.
- Every hypothesis scaled has a positive depth (`ASSERT` in `SmoothnessFactor`).
- The factor lies in `[0.8, 1]`: the coupling never raises a cost, and a hypothesis whose photometric cost
  is more than 25 % above another's never wins by it.
- The red/black parity is untouched: `SmoothPlanes` is a separate launch, `ProcessPixel` reads only the
  smoothed plane of its own pixel.

## 5. Validation

Smoothness coupling against the kernel without it, CUDA (RTX 4070), each measured from its own release
binary. ETH3D: 8 high-res training scenes at resolution level 1 (3024x2016), `bench/eth3d/ab_eth3d.py`
(F/P/R at 2 and 5 cm after the camera Sim3 alignment; the ICP-refined alignment moves F by at most 0.004
and leaves every difference within 0.0005). Normal noise: the angle between the normals of adjacent pixels
of the depth-maps, p50/p90.

| ETH3D scene | F@2cm without / with (dF) | P / R @2cm with | F@5cm without / with (dF) | P / R @5cm with | normal p50/p90 without / with |
|---|---|---|---|---|---|
| pipes | 0.7323 / 0.7452 (+0.0129) | 0.885 / 0.644 | 0.8022 / 0.8061 (+0.0039) | 0.931 / 0.711 | 14.6/77.4 / 3.6/39.9 deg |
| office | 0.5681 / 0.5873 (+0.0192) | 0.927 / 0.430 | 0.6985 / 0.7154 (+0.0169) | 0.973 / 0.566 | 25.4/93.6 / 3.5/43.4 deg |
| terrace | 0.8620 / 0.8646 (+0.0026) | 0.956 / 0.789 | 0.8829 / 0.8823 (-0.0006) | 0.978 / 0.804 | 8.7/66.4 / 2.8/22.4 deg |
| courtyard | 0.8816 / 0.8852 (+0.0036) | 0.913 / 0.859 | 0.9345 / 0.9363 (+0.0018) | 0.959 / 0.915 | 11.1/60.0 / 3.0/24.6 deg |
| delivery_area | 0.8532 / 0.8551 (+0.0019) | 0.916 / 0.802 | 0.8971 / 0.8993 (+0.0022) | 0.957 / 0.849 | 8.9/61.4 / 2.4/15.9 deg |
| kicker | 0.6733 / 0.7176 (+0.0443) | 0.937 / 0.581 | 0.7808 / 0.8143 (+0.0335) | 0.970 / 0.702 | 13.5/78.4 / 3.0/26.0 deg |
| relief | 0.7872 / 0.7883 (+0.0011) | 0.974 / 0.662 | 0.8032 / 0.8038 (+0.0006) | 0.989 / 0.677 | 6.8/41.2 / 2.4/11.4 deg |
| electro | 0.8901 / 0.9048 (+0.0147) | 0.969 / 0.849 | 0.9312 / 0.9366 (+0.0054) | 0.986 / 0.892 | 10.3/79.3 / 2.5/32.4 deg |
| mean dF | +0.0125 | | +0.0080 | | |

Metal (Apple Silicon, the same scenes, inputs and scorer): F@2cm pipes 0.7311 / 0.7452, office
0.5687 / 0.5871; F@5cm pipes 0.8022 / 0.8064, office 0.6980 / 0.7143, within 0.0012 of CUDA in every
cell. `MetalModelMathTest`, `SyntheticSphericalGPUTest` (held to the CUDA bounds), `SyntheticMixedGPUTest`
and `GPUConfidenceTest` pass on Metal.

Tanks and Temples Barn at r1 (official evaluator, both arms scored in one process). Holo360D Outdoor_004:
40 panoramas at 1440 px, the fused cloud against the GT range maps (`Tests evalcloud`). Confidence: the
recalibrated confidence scored against local ground truth.

| measure | without | with |
|---|---:|---:|
| T&T Barn F / P / R | 0.6559 / 0.601 / 0.722 | 0.6560 / 0.560 / 0.793 |
| Holo360D fused F@2% / F@5% | 0.452 / 0.647 | 0.511 / 0.699 |
| Holo360D median relative error, pixels both estimate | 0.0385 | 0.0355 |
| confidence ROC-AUC, all / ETH3D / BlendedMVS | 0.892 / 0.889 / 0.918 | 0.900 / 0.898 / 0.917 |
| inlier share kept at 1 % contamination | 37.6 % | 36.7 % |
| contamination at 90 % completeness | 8.9 % | 6.6 % |
| GPU time of the black + red passes incl. `SmoothPlanes`, photometric + geometric, pipes / office | 3.14 / 5.68 s | 3.24 / 5.79 s (+3.1 % / +2.0 %) |
| the same per MP and iteration, photometric passes, pipes / office | 4.82 / 4.96 ms | 4.99 / 5.11 ms (+3.5 % / +3.0 %) |
| the same, geometric passes, pipes / office | 5.77 / 6.39 ms | 5.89 / 6.39 ms (+2.1 % / -0.1 %) |

GPU times: CUDA events around the launches (a local timing build of both kernels), mean of 3 interleaved
runs per scene; the run-to-run spread of the photometric total is 1-4 %.

## 6. Rejected alternatives

- The CPU term ported as is (4-neighbour bonus, the PR proposal): kernel +10 %, pipes F@5cm -0.011.
- The factor only comparing the hypotheses, the stored cost left photometric: F slightly negative.
- A colour x confidence weighted per-neighbour bonus, also variance-gated and ramped over the iterations:
  the per-candidate neighbour loop costs too much kernel time.
- Bonus 0.4: T&T Barn -0.0079, precision -0.05.
- The stored cost photometric with the coupling to the smoothed plane: Barn -0.0064, office -0.0065.
- Fusion NCC keep threshold 1.2 alone: +0.0049, but it disables the recalibration gate.
- `SmoothPlanes` over the pixels of the next pass only (half the grid): no measurable kernel time gain.
- The distance in the algebraically identical form `(ns.Ray)*(1 - depthS/depth)`, precomputed per pixel:
  measured, no gain (the kernels stay at the 128-register cap, the pinhole ones with the same stack;
  photometric passes +2.8 % on pipes, +0.0 % on office).

## 7. Open items

- HIP: the backend port takes the CUDA kernel as it is now, coupling included.
- The constants were tuned on ETH3D at level 1; full-resolution and aerial scenes are untested.
- The photometric passes cost up to 3.5 % more; the cost is in `ProcessPixel` (not in `SmoothPlanes`).
