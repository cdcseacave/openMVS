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

Smoothness coupling against the kernel without it, CUDA (RTX 4070). ETH3D: 8 high-res training scenes at
resolution level 1 (3024x2016), F at 2 cm unless noted, `bench/eth3d/ab_eth3d.py`. Tanks and Temples Barn
at r1. Holo360D Outdoor_004: 40 panoramas at 1440 px. Normal noise: median angle between the normals of
adjacent pixels. Confidence: the recalibrated confidence scored against local ground truth.

| measure | without | with |
|---|---:|---:|
| ETH3D pipes / office / terrace / courtyard, dF@2cm | | +0.0127 / +0.0192 / +0.0026 / +0.0037 |
| ETH3D delivery_area / kicker / relief / electro, dF@2cm | | +0.0019 / +0.0440 / +0.0012 / +0.0148 |
| ETH3D mean dF@2cm / dF@5cm | | +0.0125 / +0.0080 |
| T&T Barn F | 0.6559 | 0.6562 |
| Holo360D fused F@5% | 0.647 | 0.699 |
| Holo360D median relative error, pixels both estimate | 0.0354 | 0.0327 |
| adjacent-normal angle p50 (ETH3D scenes) | 7-25 deg | 2.4-3.6 deg |
| confidence ROC-AUC, all / ETH3D / BlendedMVS | 0.892 / 0.889 / 0.918 | 0.900 / 0.898 / 0.917 |
| inlier share kept at 1 % contamination | 37.6 % | 36.7 % |
| contamination at 90 % completeness | 8.9 % | 6.6 % |
| GPU time of the black + red passes incl. `SmoothPlanes`, photometric + geometric, pipes / office | 3.12 / 5.69 s | 3.26 / 5.81 s (+4.4 % / +2.2 %) |
| the same per MP and iteration, photometric passes, pipes / office | 4.77 / 4.97 ms | 5.02 / 5.13 ms (+5.3 % / +3.2 %) |
| the same, geometric passes, pipes / office | 5.75 / 6.39 ms | 5.90 / 6.39 ms (+2.5 % / +0.0 %) |

GPU times: CUDA events around the launches (a local timing build of both kernels), mean of 3 interleaved
runs per scene.

## 6. Rejected alternatives

- The CPU term ported as is (4-neighbour bonus, the PR proposal): kernel +10 %, pipes F@5cm -0.011.
- The factor only comparing the hypotheses, the stored cost left photometric: F slightly negative.
- A colour x confidence weighted per-neighbour bonus, also variance-gated and ramped over the iterations:
  the per-candidate neighbour loop costs too much kernel time.
- Bonus 0.4: T&T Barn -0.0079, precision -0.05.
- The stored cost photometric with the coupling to the smoothed plane: Barn -0.0064, office -0.0065.
- Fusion NCC keep threshold 1.2 alone: +0.0049, but it disables the recalibration gate.
- `SmoothPlanes` over the pixels of the next pass only (half the grid): no measurable kernel time gain.

## 7. Open items

- Metal: mirrored, not yet compiled nor run on Apple hardware (`MetalModelMathTest`, `SyntheticSphericalGPUTest`,
  `SyntheticMixedGPUTest` and an ETH3D A/B against CUDA to do on the Mac).
- HIP: the backend port takes the CUDA kernel as it is now, coupling included.
- The constants were tuned on ETH3D at level 1; full-resolution and aerial scenes are untested.
- The photometric passes cost up to 5 % more; the cost is in `ProcessPixel` (not in `SmoothPlanes`), a
  cheaper form of the distance (`(ns.Ray)*(1 - depthS/depth)`, fewer live registers) is untested.
