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
smoothed plane built from the current estimates of its neighbours.

### Smoothed plane (`ProcessPixel`)

During a black (red) pass the 4 adjacent pixels `q` of `p` (up, down, left, right) are red (black): the pass
does not rewrite them, so `ProcessPixel` reads their current estimates directly, with no snapshot and no
extra launch, and each pass sees the estimates the previous pass left. Over those of them with an estimate
(`depth > 0`):

```
w(q)   = exp(-(I(q) - I(p))^2 / (2*(25/255)^2)  -  cost(q)^2 / (2*0.3^2))
n      = normalize(sum w(q)*n(q))
X      = sum w(q)*X(q) / sum w(q)            X(q) = Ray(q)*depth(q), reference camera frame
depthS = n.X / n.Ray(p)                       the plane (n, X) met along the pixel ray
```

`I` is the reference image texel (`FetchRefTexel`), `cost` the stored cost of the neighbour (its
confidence: an unconverged or occluded neighbour stops pulling), the colour term the patch's bilateral
weight (no smoothing across image edges). The pixel has no smoothed plane when no neighbour has an
estimate, their normals cancel out (`|sum w n| <= 1e-3 * sum w`), the plane is seen edge-on or from behind
(`n.Ray(p) >= -0.01`) or meets the ray behind the camera. The neighbours are the propagation's
`neighborPositions` (`valid[0..3]`): an equirectangular reference (`MODEL_SPHERICAL_REF`) wraps the columns
across the seam (`SEACAVE::WrapColumn`, rays from the `refTrig` tables through `RefPixel::Ray`).

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
direction), the neighbour weights of the next pass's smoothed planes, `FilterPlanes` (`fThresholdKeepCost`) and the
photometric confidence, from which the confidence recalibration starts (`DepthMapConfidence.md`).

### Metal

`PatchMatchMetal.metal` mirrors the CUDA code line by line: the constants and `SmoothnessFactor` before
`ProcessPixel`, the smoothed plane built inside it from the same 4 neighbours (columns wrapped for
`MODEL_SPHERICAL_REF`, rays through `RefTexelRay`). There is no extra kernel, buffer or pipeline.

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

- The smoothed plane reads only pixels of the other colour, which the running pass does not write: it is
  the same whatever the order in which the threads run, and never mixes estimates of two depth-maps.
- A smoothed plane, when the pixel has one, is a unit normal facing the pixel ray with a positive depth.
- Every hypothesis scaled has a positive depth (`ASSERT` in `SmoothnessFactor`).
- The factor lies in `[0.8, 1]`: the coupling never raises a cost, and a hypothesis whose photometric cost
  is more than 25 % above another's never wins by it.

## 5. Validation

Smoothness coupling against the kernel without it, CUDA (RTX 4070), each measured from its own release
binary. ETH3D: 8 high-res training scenes at resolution level 1 (3024x2016), `bench/eth3d/ab_eth3d.py`
(F/P/R at 2 and 5 cm after the camera Sim3 alignment; the ICP-refined alignment moves F by at most 0.004
and leaves every difference within 0.0005). Normal noise: the angle between the normals of adjacent pixels
of the depth-maps, p50/p90.

| ETH3D scene | F@2cm without / with (dF) | P / R @2cm with | F@5cm without / with (dF) | P / R @5cm with | normal p50/p90 without / with |
|---|---|---|---|---|---|
| pipes | 0.7323 / 0.7458 (+0.0135) | 0.884 / 0.645 | 0.8022 / 0.8063 (+0.0041) | 0.930 / 0.712 | 14.6/77.4 / 3.4/40.8 deg |
| office | 0.5681 / 0.5893 (+0.0212) | 0.927 / 0.432 | 0.6985 / 0.7187 (+0.0202) | 0.973 / 0.570 | 25.4/93.6 / 3.4/44.8 deg |
| terrace | 0.8620 / 0.8649 (+0.0029) | 0.956 / 0.790 | 0.8829 / 0.8826 (-0.0003) | 0.978 / 0.804 | 8.7/66.4 / 2.8/23.9 deg |
| courtyard | 0.8816 / 0.8855 (+0.0039) | 0.913 / 0.860 | 0.9345 / 0.9366 (+0.0021) | 0.959 / 0.915 | 11.1/60.0 / 2.9/25.4 deg |
| delivery_area | 0.8532 / 0.8553 (+0.0021) | 0.916 / 0.802 | 0.8971 / 0.8993 (+0.0022) | 0.956 / 0.849 | 8.9/61.4 / 2.4/16.4 deg |
| kicker | 0.6733 / 0.7223 (+0.0490) | 0.937 / 0.588 | 0.7808 / 0.8169 (+0.0361) | 0.970 / 0.706 | 13.5/78.4 / 2.9/27.3 deg |
| relief | 0.7872 / 0.7884 (+0.0012) | 0.974 / 0.662 | 0.8032 / 0.8038 (+0.0006) | 0.989 / 0.677 | 6.8/41.2 / 2.4/11.5 deg |
| electro | 0.8901 / 0.9048 (+0.0147) | 0.969 / 0.849 | 0.9312 / 0.9366 (+0.0054) | 0.986 / 0.892 | 10.3/79.3 / 2.5/33.8 deg |
| mean dF | +0.0136 | | +0.0088 | | |

Metal (Apple Silicon, the same scenes, inputs and scorer): METAL_PLACEHOLDER

Tanks and Temples Barn at r1 (official evaluator, both arms scored in one process). Holo360D Outdoor_004:
40 panoramas at 1440 px, the fused cloud against the GT range maps (`Tests evalcloud`).

| measure | without | with |
|---|---:|---:|
| T&T Barn F / P / R | 0.6559 / 0.601 / 0.722 | 0.6575 / 0.561 / 0.793 |
| Holo360D fused F@1% / F@2% / F@5% | 0.272 / 0.452 / 0.647 | 0.324 / 0.511 / 0.700 |
| Holo360D median relative error, pixels both estimate | 0.0347 | 0.0322 |
| GPU time of the black + red passes, photometric, pipes / office | 2.18 / 3.79 s | 2.20 / 3.84 s (+0.8 % / +1.1 %) |
| the same, geometric passes, pipes / office | 0.99 / 1.88 s | 0.98 / 1.83 s (-0.9 % / -2.5 %) |
| the same, photometric + geometric, pipes / office | 3.17 / 5.67 s | 3.18 / 5.67 s (+0.3 % / -0.1 %) |

GPU times: CUDA events around the launches (a local timing build of both kernels), mean of 3 interleaved
runs per scene; the run-to-run spread of the total is 0.5-3 %.

## 6. Rejected alternatives

- The CPU term ported as is (4-neighbour bonus, the PR proposal): kernel +10 %, pipes F@5cm -0.011.
- The factor only comparing the hypotheses, the stored cost left photometric: F slightly negative.
- A colour x confidence weighted per-neighbour bonus, also variance-gated and ramped over the iterations:
  the per-candidate neighbour loop costs too much kernel time.
- Bonus 0.4: T&T Barn -0.0079, precision -0.05.
- The stored cost photometric with the coupling to the smoothed plane: Barn -0.0064, office -0.0065.
- Fusion NCC keep threshold 1.2 alone: +0.0049, but it disables the recalibration gate.
- The smoothed plane of the full 3x3 neighbourhood (both colours, the pixel included), built by a separate
  full-grid kernel into a snapshot buffer before every pass: the same F (ETH3D mean dF@2cm -0.0010 against
  the 4 neighbours), the passes 2-3 % slower.
- The 4 neighbours fitted unweighted (`DepthPlaneFit` + `NormalFromDepthGradient`, the plane through their
  mean depth): pipes / office F@2cm -0.014 / -0.011 (the fit bends across depth edges), the passes 2-3 %
  slower; also reusing its normal as the refinement's surface-normal candidate: the same.
- The distance in the algebraically identical form `(ns.Ray)*(1 - depthS/depth)`, precomputed per pixel:
  no gain (the kernels stay at the 128-register cap).

## 7. Open items

- HIP: the backend port takes the CUDA kernel as it is now, coupling included.
- The constants were tuned on ETH3D at level 1; full-resolution and aerial scenes are untested.
- The recalibrated confidence (`DepthMapConfidence.md`) is not yet scored against local ground truth with
  the coupling.
