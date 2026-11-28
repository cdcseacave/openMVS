# GPU PatchMatch Smoothness Coupling

## 1. Purpose and scope

The GPU PatchMatch (CUDA `libs/MVS/PatchMatchCUDA.cu`, Metal `libs/MVS/PatchMatchMetal.metal`) scores every
plane hypothesis of a pixel by its photometric (and, in the geometric passes, geometric) cost alone, so on
weak texture neighbouring pixels settle on unrelated planes: the depth-maps are noisy and their normals,
which fusion compares between views, are noisier still. The smoothness coupling adds a pairwise prior to
the per-pixel decision: a hypothesis that agrees with a smoothed plane of the pixel's current neighbourhood
(in depth and, where the patch is weakly textured, in normal) costs up to 20 % less. It is the kernels'
behaviour, with no option to turn it off.

Scope: every CUDA and Metal estimation pass (photometric and geometric, every pyramid level, every camera
model), and so every HIP pass, which runs the CUDA kernel (`docs/design/HIPBackend.md`). Out of scope: the
CPU estimator (`DepthEstimatorT`, `libs/MVS/DepthMap.cpp`), which keeps its own term
(`DENSE_SMOOTHNESS_PLANE`, `fRandomSmoothBonus`: a bonus per agreeing already-estimated neighbour).

## 2. Algorithm as implemented

The coupling is a quadratic relaxation of a pairwise smoothness energy (PM-Huber style): instead of
comparing a hypothesis with each neighbour, every hypothesis pays one robust distance to a per-pixel
smoothed plane built from the current estimates of its neighbours.

### Smoothed plane (`ProcessPixel`)

During a black (red) pass the 4 adjacent pixels `q` of `p` (up, down, left, right) are red (black): the pass
does not rewrite them, so `ProcessPixel` reads their current estimates directly, with no snapshot and no
extra launch, and each pass sees the estimates and the stored costs the previous pass left (the first black
pass of a run sees the plain costs `InitializeScore` stores). Over those of them in the image with an
estimate (`depth > 0`):

```
w(q)   = exp(-(I(q) - I(p))^2 / (2*(25/255)^2)  -  cost(q)^2 / (2*0.3^2))
n      = normalize(sum w(q)*n(q))
X      = sum w(q)*X(q) / sum w(q)            X(q) = Ray(q)*depth(q), reference camera frame
depthS = n.X / n.Ray(p)                       the plane (n, X) met along the pixel ray
```

`I` is the reference image texel (`FetchRefTexel`), `cost` the stored cost of the neighbour (its
confidence: an unconverged or occluded neighbour stops pulling), the colour term the patch's bilateral
weight (no smoothing across image edges). The pixel has no smoothed plane when no neighbour has an
estimate or all their weights vanish (`sum w <= 1e-4`: dissimilar in colour or unconfident), their normals
cancel out (`|sum w n| <= 1e-3 * sum w`), the plane is seen edge-on or from behind
(`n.Ray(p) >= -0.01`) or meets the ray behind the camera. The neighbours are the propagation's
`neighborPositions`, taken when in the image (`valid[0..3]`: every sample of the near up, down, left and
right patterns lies on that side of the pixel); across the seam of an equirectangular reference
(`MODEL_SPHERICAL_REF`) the neighbour column is taken as it is: the texel fetch repeats the columns and
`RefPixel::Ray` (rays from the `refTrig` tables) wraps them.

### Cost factor (`SmoothnessFactor`, `ProcessPixel`)

With the smoothed plane of the pixel as its point `Xs = Ray(p)*depthS` and normal `ns`, a hypothesis
`(n, depth)` has the relative distance `d = ns.(Ray(p)*depth - Xs) / depth` to it, and its aggregated cost
is multiplied by

```
t      = exp(-varRef / 0.0025)                texture weight of the reference patch, once per pixel
factor = 1 - 0.2 * exp(-d^2 / (2*0.02^2)  -  t * (1 - ns.n) / (13 deg)^2)
```

(`1 - cos` stands for `acos^2 / 2`, one `exp` and no `acos` per hypothesis). `varRef` is the bilaterally
weighted variance of the reference patch the score already computes (`RefPatchCache::varRef`; its median
is 0.001 on ETH3D office and pipes, 0.006 on terrace, 0.07 on T&T Barn, 0.15 on Ignatius). In a weakly
textured patch (`t` near 1) the bonus asks for the agreement in depth and in normal; in a textured one
(`t` near 0) for the agreement in depth only. There the texture constrains the normal by itself, and a
normal pulled to its neighbours whatever the images say is no evidence for the consumers of the normal:
the fusion (the normal test of a join, the contradiction guard) and the intra-map prior of the confidence
compare the normals between views and with the depth-map, and let more wrong points through when the
normals agree by construction (section 6).

`ProcessPixel` applies the factor to
the 8 propagated neighbour planes (`finalCosts`), to the pixel's own plane re-scored with the new view
weights, and to the 3 or 4 refinement candidates; the stored cost is the scaled one. A pixel without a
smoothed plane keeps its plain costs. The per-view costs (`costArray`) that drive the view selection stay
unscaled.

The scaled stored cost then also feeds the neighbour choice of the propagation (the lowest cost along each
direction), the neighbour weights of the next pass's smoothed planes, `FilterPlanes`
(`fThresholdKeepCost`) and the photometric confidence, from which the confidence recalibration starts (`DepthMapConfidence.md`).

### Metal

`PatchMatchMetal.metal` mirrors the CUDA code line by line: the constants and `SmoothnessFactor` before
`ProcessPixel`, the smoothed plane and the normal sigma of the pixel built inside it from the same 4
neighbours and `RefCache::varRef` (the sampler repeats the columns of an equirectangular reference,
`RefTexelRay` wraps them). There is no extra kernel, buffer or pipeline.

## 3. Parameters

Compile-time constants next to the kernel (`PatchMatchCUDA.cu`, mirrored in `PatchMatchMetal.metal`); one
jointly validated operating point, not user options.

| constant | value | meaning |
|---|---|---|
| `smoothBonus` | 0.2 | cost reduction of a hypothesis on the smoothed plane |
| `smoothSigmaDepth` | sigma 0.02 | distance to the smoothed plane, relative to the depth (the CPU's `fRandomSmoothDepth`) |
| `smoothSigmaNormal` | sigma 13 deg | normal agreement, on `1 - cos` (the CPU's `fRandomSmoothNormal`) |
| `smoothSigmaTexture` | scale 0.0025 | weight of the normal agreement, `exp(-varRef/0.0025)` (the texture-less bound of `ScoreNCC`) |
| `smoothSigmaColor` | sigma 25/255 | neighbour weight: gray difference to the pixel (the patch bilateral sigma) |
| `smoothSigmaConf` | sigma 0.3 | neighbour weight: its stored cost (the view-selection sigma) |

## 4. Invariants

- The smoothed plane reads only pixels of the other colour, which the running pass does not write: it is
  the same whatever the order in which the threads run, and never mixes estimates of two depth-maps.
- A smoothed plane, when the pixel has one, is a unit normal facing the pixel ray with a positive depth.
- Every hypothesis scaled has a positive depth (`ASSERT` in `SmoothnessFactor`).
- The normal sigma of a pixel is `smoothSigmaNormal` times a weight in (0, 1], never positive (`ASSERT` in
  `SmoothnessFactor`): the normal term only ever lowers the bonus.
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
| pipes | 0.7323 / 0.7476 (+0.0153) | 0.890 / 0.644 | 0.8022 / 0.8068 (+0.0046) | 0.934 / 0.710 | 14.6/77.4 / 6.6/50.6 deg |
| office | 0.5681 / 0.5951 (+0.0270) | 0.931 / 0.437 | 0.6985 / 0.7222 (+0.0237) | 0.975 / 0.574 | 25.4/93.6 / 8.5/58.3 deg |
| terrace | 0.8620 / 0.8671 (+0.0051) | 0.962 / 0.789 | 0.8829 / 0.8841 (+0.0012) | 0.982 / 0.804 | 8.7/66.4 / 6.2/52.3 deg |
| courtyard | 0.8816 / 0.8834 (+0.0018) | 0.914 / 0.855 | 0.9345 / 0.9357 (+0.0012) | 0.959 / 0.913 | 11.1/60.0 / 9.0/52.3 deg |
| delivery_area | 0.8532 / 0.8591 (+0.0059) | 0.925 / 0.802 | 0.8971 / 0.9018 (+0.0047) | 0.962 / 0.849 | 8.9/61.4 / 5.5/40.7 deg |
| kicker | 0.6733 / 0.7178 (+0.0445) | 0.943 / 0.580 | 0.7808 / 0.8179 (+0.0371) | 0.974 / 0.705 | 13.5/78.4 / 11.8/66.8 deg |
| relief | 0.7872 / 0.7889 (+0.0017) | 0.975 / 0.662 | 0.8032 / 0.8039 (+0.0007) | 0.989 / 0.677 | 6.8/41.2 / 4.6/20.5 deg |
| electro | 0.8901 / 0.9051 (+0.0150) | 0.971 / 0.848 | 0.9312 / 0.9373 (+0.0061) | 0.987 / 0.892 | 10.3/79.3 / 6.2/63.5 deg |
| mean dF | +0.0145 | | +0.0099 | | |

Over the 8 scenes the precision at 2 cm changes by +0.0006 and the recall by +0.0193 on average.

Metal (Apple Silicon, the same scenes, inputs and scorer): F@2cm pipes 0.7311 / 0.7479, office
0.5687 / 0.5944; F@5cm pipes 0.8022 / 0.8079, office 0.6980 / 0.7201, within 0.0021 of CUDA in every
cell. `MetalModelMathTest`, `SyntheticSphericalGPUTest` (held to the CUDA bounds), `SyntheticMixedGPUTest`
and `GPUConfidenceTest` pass on Metal.

Tanks and Temples Barn at r1 and Ignatius at its 3 mm tolerance (official evaluator, the arms scored in
one process). Barn is given under the alignment the evaluator refines on the cloud without the coupling:
its refinement on the coupled cloud lands 2-3 mm off and costs either cloud 0.005 (0.6542 with, 0.6508
without). Holo360D Outdoor_004: 40 panoramas at 1440 px, the fused cloud against the GT range maps
(`Tests evalcloud`).

| measure | without | with |
|---|---:|---:|
| T&T Barn F / P / R | 0.6559 / 0.601 / 0.722 | 0.6592 / 0.596 / 0.738 |
| T&T Ignatius (default options) F / P / R | 0.788 / 0.712 / 0.882 | 0.788 / 0.711 / 0.883 |
| Holo360D fused F@1% / F@2% / F@5% | 0.272 / 0.452 / 0.647 | 0.286 / 0.474 / 0.670 |
| Holo360D median relative error, pixels both estimate | 0.0363 | 0.0344 |
| GPU time of the black + red passes, photometric, pipes / office | 2.16 / 3.82 s | 2.17 / 3.83 s (+0.5 % / +0.5 %) |
| the same, geometric passes, pipes / office | 0.98 / 1.88 s | 0.98 / 1.83 s (-0.3 % / -2.3 %) |
| the same, photometric + geometric, pipes / office | 3.14 / 5.69 s | 3.15 / 5.67 s (+0.3 % / -0.4 %) |

GPU times: CUDA events around the launches (a local timing build of both kernels), mean of 3 interleaved
runs per scene; the run-to-run spread of the total is 0.4-3 %.

## 6. Rejected alternatives

- The CPU term ported as is (4-neighbour bonus, the PR proposal): kernel +10 %, pipes F@5cm -0.011.
- The factor only comparing the hypotheses, the stored cost left photometric: F slightly negative.
- A colour x confidence weighted per-neighbour bonus, also variance-gated and ramped over the iterations:
  the per-candidate neighbour loop costs too much kernel time.
- Bonus 0.4: T&T Barn -0.0079, precision -0.05.
- The normal agreement counted in every patch (no texture weight): ETH3D mean dF@2cm +0.0136 with the
  precision down 0.004, 16-55 % more points, T&T Ignatius F -0.018 (P 0.712 -> 0.679), Barn P -0.04. The
  normals of adjacent pixels then agree (p50 2.4-3.4 deg) but are hardly closer to the surface (pipes
  15.4 -> 11.4 deg), and the fusion's normal rejections on Ignatius halve (17.9 M -> 8.3 M).
- No normal term (agreement in depth only): no precision loss and T&T unchanged, but ETH3D dF@2cm +0.0155
  against +0.0228 over pipes, office, terrace and kicker, and normals as noisy as without the coupling.
- The texture weight at a larger scale (0.005 to 0.05): Ignatius F -0.001 to -0.007.
- A wider normal sigma (25, 40, 60 deg): alone (40 deg) Ignatius F -0.007; with the texture weight the same
  F as 13 deg and noisier normals (pipes p50 8.4 deg at 25 deg, 10.3 at 40 deg, against 6.6).
- Bonus 0.1 with the unweighted normal term: Ignatius F -0.010, 60 % of the ETH3D gain.
- The fusion's normal threshold tightened instead (20 / 15 deg): Ignatius F -0.016 at 20 deg; 15 deg undoes
  the ETH3D gain (office +0.0007).
- A gate on the spread of the neighbour planes: no effect.
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

- HIP: compiled with the coupling (the CUDA kernel), not yet run on AMD hardware.
- Scenes mixing weak and strong texture keep only part of what the unweighted normal term gave them
  (Holo360D fused F@5% 0.670 against 0.700, kicker F@2cm 0.7178 against 0.7223, courtyard 0.8834 against
  0.8855): the texture weight is per patch, and tuned for the precision of the textured scenes.
- The constants were tuned on ETH3D at level 1 and T&T Barn / Ignatius; full-resolution and aerial scenes
  are untested.
- The recalibrated confidence (`DepthMapConfidence.md`) is not yet scored against local ground truth with
  the coupling.
