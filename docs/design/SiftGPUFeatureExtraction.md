# SiftGPU Feature Extraction

## 1. Purpose and scope

`FeaturesExtractor` with `detectorType = SIFTGPU` (CreateStructure `--detector-type SIFTGPU`) extracts
SIFT keypoints and RootSIFT descriptors on the GPU for the sparse reconstruction. This record covers
the host side in `libs/SFM/FeaturesExtractor.cpp` (image decoding, SiftGPU parameters, per-image
feature selection) and the detector itself in the local vcpkg port `ports/siftgpu/source/src`
(CUDA: `ProgramCU.cu`, `PyramidCU.cpp`, `CuTexImage.cpp`; OpenGL: `ProgramGLSL.cpp`, `PyramidGL.cpp`,
`ShaderMan.cpp`). The CUDA path is used when available (`FeatureExtractionConfig::useCUDA`); the
OpenGL path implements the same detector, orientation and options and gives the same features up to
floating-point differences. Matching, the other detectors and the SfM are documented elsewhere.

The port is built by vcpkg in manifest mode with `VCPKG_OVERLAY_PORTS=ports`; vcpkg hashes the port
sources, so a CMake configure after a change rebuilds only `siftgpu`, as long as the vcpkg toolchain
(and with it the ABI hash of every package) is unchanged. A changed toolchain makes vcpkg remove and
rebuild all dependencies first.

---

## 2. Algorithm as implemented

### 2.1 Input pipeline (`ExtractFeaturesSiftGPU`)

One thread owns the SiftGPU context. Images are decoded ahead of it on `scene.threadPool`: one task
and one `std::future<bool>` per image, at most one task in flight per pool thread and at most 768 MB
of decoded images. A task loads the pixels, converts them to SiftGPU's float grey input and
optionally releases the image pixels; the GPU thread waits on the image's future, uploads, extracts,
and never loads an image itself. Decode failures are reported through the future's value, never by
throwing, so the GPU thread cannot leave a running task behind.

### 2.2 Gaussian and DoG pyramid

SiftGPU's pyramid: first octave `-fo -1` (input upsampled 2x), 3 levels per octave (`-d 3`),
`-maxd = maxImageSize * 2`. Gaussian levels, DoG and gradient magnitude/angle levels are pitch-linear
`CuTexImage` buffers. Each buffer caches the texture objects bound to it (one per texture
description); they are created on first use and destroyed when the buffer is reallocated or
destroyed, never by a copy (`CuTexImage` is not copyable).

### 2.3 Keypoint detection (`ComputeKEY_Kernel`, GLSL keypoint shaders)

For each DoG level, a pixel is a keypoint when, in order:
1. `|D| > 0.8 t` with `t = -t 0.001` (SiftGPU's default is 0.02/3; D on a 0..1 grey scale);
2. it is a 3x3x3 extremum;
3. edge test on the 3x3 Hessian of D: `det > 0` and `trace² / det < (r+1)²/r` with `r = -e 12.5`;
4. curvature floor (`-cmin 0.001`): the smaller principal curvature
   `|λmin| = (|fxx+fyy| - sqrt((fxx-fyy)² + 4fxy²)) / 2` must reach `cmin / σ²`, σ being the level's
   Gaussian sigma, i.e. the scale-normalized curvature `σ²|λmin| ≥ cmin`;
5. SiftGPU's sub-pixel and sub-scale refinement (one 3x3 solve, offset test), after which the
   interpolated `|D|` must exceed `t`.

The floor is what makes a very low DoG threshold usable: low-contrast extrema are kept only when they
are curved in both directions (blobs and corners on walls and floors), while flat and edge-like
extrema, which a low threshold otherwise admits, are rejected. It is the absolute cornerness test of
the RSa1 detector (Acute3D ImageMatching: `2|λmin| > 200` on its scale-normalized DoG, about
`σ²|λmin| ≥ 7e-4` in SiftGPU's DoG units); its DoG contrast test is negligible. The edge ratio 12.5 is
also RSa1's.

Darkness adaptation (`-da`, `FeatureExtractionConfig::siftgpuDarknessAdaptivity`, CreateStructure
`--siftgpu-darkness-adaptivity`, off by default) multiplies `t`, the pre-test threshold and the floor
by `min(2G + 0.1, 1)`, G being the Gaussian level at the pixel (0..1). In a gamma-encoded image the
contrast of a structure falls with brightness while sensor noise does not, so the factor recovers
dark structure on well exposed images and admits noise on underexposed ones (§5).

List generation: each warp of `ComputeKEY_Kernel` (blocks of 32x4 pixels) writes the ballot bit
mask of its 32 pixels and adds their count to the row count (`atomicAdd`). One download of the row
counts per image gives every level's keypoint count and row offsets (host prefix sums);
`ListKeys_Kernel` then writes each level's keypoint list from the masks.

### 2.4 Orientation (`ComputeOrientation_Kernel`, GLSL orientation shaders)

The gradients in SiftGPU's window (Gaussian weight σ_w = 1.5·σ_kp, radius 3σ_w) are summed as
circular moments `M_k = Σ w·|g|·exp(i·k·θ)`, k = 0..5, by repeated complex multiplication with the
gradient direction (no binning). The orientation density smoothed by a wrapped Gaussian of
0.35 rad, `D(φ) = Σ_k g_k·Re(M_k·exp(-i·k·φ))` with `g_0 = 1`, `g_k = 2·exp(-k²·0.35²/2)`, is sampled
at the 36 former bin centres `(t + 0.5)·10°`; the peak search, the parabolic refinement and the second
orientation (a local maximum above 0.8 of the highest, `-mo 2`) are SiftGPU's. 0.35 rad matches the
six `[1 1 1]/3` smoothing passes of SiftGPU's 36-bin histogram, which this replaces; the angle is no
longer quantized before smoothing. The density is the same in CUDA and GLSL (`OrientationMomentMacros`,
`OrientationGain`).

Features with two orientations are split on the GPU: `CountOrientations_Kernel` counts per block,
the host scans the block counts of all levels after one download, and `ExpandOrientations_Kernel`
scatters.

### 2.5 Descriptor and download

SiftGPU's 4x4x8 descriptor (`ComputeDescriptor_Kernel`), its sample loops bounded to the rows and
columns inside the rotated window. All levels write into one device buffer that is normalized in
place and downloaded once per image. `FeaturesExtractor` converts to RootSIFT ×512 bytes.

### 2.6 Per-image selection (`SelectFeatures`)

SiftGPU returns every keypoint (`-tc2` off). At most `GetMaxNumFeatures()` (27,000 with the default
3000 per 3x3 cell) are kept on a grid of about 256 px cells: the largest per-cell quota that fits the
budget is found (water filling), cells with fewer keypoints keep all of theirs, the remainder is given
one per cell in cell order, and inside a cell the largest scales win (ties by index). An image under
budget keeps every keypoint.

---

## 3. Parameters and defaults

| Parameter | Value | Where |
|---|---|---|
| DoG peak threshold `-t` | 0.001 | `peakThreshold`, constant |
| Curvature floor `-cmin` (σ²·\|λmin\|) | 0.001 | `minCurvature`, constant |
| Edge threshold `-e` | 12.5 | `edgeThreshold`, constant |
| Darkness adaptation `-da` | off | `siftgpuDarknessAdaptivity`, `--siftgpu-darkness-adaptivity` |
| First octave, levels per octave | -1, 3 | constants |
| Orientations per keypoint `-mo` | 2 | constant |
| Orientation harmonics, smoothing | 5, 0.35 rad | `ORIENTATION_HARMONICS`, `ORIENTATION_SMOOTHING` (CUDA), GLSL constants |
| Selection budget | `GetMaxNumFeatures()` (27,000) | `maxFeaturesPerCell` x 9 |
| Selection cell | about 256 px | `gridCellSize` |
| Decode look-ahead | pool threads, 768 MB | `ExtractFeaturesSiftGPU` |

SiftGPU's own defaults for `-cmin` (0 = off) and `-da` (off) leave other users of the library on
the original detector; the orientation estimator has no switch.

## 4. Invariants and constraints

- `-cmin` is compared with the curvature of the level's DoG divided by σ², so it is independent of
  the octave; changing `-d` or the pyramid sigmas changes its meaning.
- A texture object belongs to the buffer it is bound to and lives until that buffer is reallocated;
  kernels may run asynchronously after the host function that bound it returns.
- The CUDA and GLSL detectors must stay equivalent: the same thresholds, floor, darkness factor and
  orientation density (feature counts within about 0.3%, orientation error within 0.01°).
- With darkness adaptation off and the same settings, the optimized CUDA pipeline reproduced the
  previous pipeline byte for byte (1,164 images); keep that check for refactors that should not
  change results.

## 5. Validation of the shipped defaults

Benchmark: Acute3D `tools/feature_benchmark` (Metashape references of 7 TnT and 13 ETH3D scenes,
1,294 image pairs, plus a synthetic suite with exact homographies); SfM: Acute3D ImageMatching on
imported SiftGPU features (`sfm_compare.py`, 9 scenes, 3-5 multithreaded runs, median camera-centre
error relative to the camera extent; Courtroom excluded where noted because about 10% of its
reference cameras are flipped). Run-to-run SfM noise is a few hundredths of a percent per scene
average (office alone spans 0.5-0.9%).

| Change | Measure | Before | Shipped |
|---|---|---|---|
| Decoding on the pool | ETH3D 24 MP, images/s (facade, office) | 5.0, 7.4-8.0 | 12.8, 16.4 |
| CUDA list generation, cached textures, GPU orientation split, one download | GPU ms per 1920x1080 image | 28 | 21 |
| (same) | CreateStructure Ignatius, 263 images: extraction | 8.8 s | 5.5 s |
| Threshold 0.005 + 3x3 cap → 0.0025 + 256 px grid | weak-cell coverage, weak scenes, native | 0.31 | 0.60 |
| (same) | SfM centre error, 9 scenes | 0.380% | 0.366% |
| + floor 0.001, threshold 0.001, edge 12.5 | ratio-test precision, 20 scenes, native / 8192 | 0.648 / 0.576 | 0.664 / 0.607 |
| (same) | weak-cell coverage, 20 scenes, native | 0.617 | 0.678 |
| (same) | correct mutual matches per pair, native | 2193 | 2028 |
| (same) | pose AUC@10, native / 2048 | 0.892 / 0.773 | 0.891 / 0.781 |
| (same) | SfM centre error, 9 scenes (RSa1: 0.342%) | 0.366% | 0.338% |
| Moment orientation | median orientation error, synthetic rotation/scale | 0.90° | 0.73° |
| (same) | top-1 / correct matches, synthetic | 0.944 / 11,381 | 0.946 / 11,410 |
| (same) | pose AUC@10, 5 dev scenes, weak / textured, native | 0.884 / 0.881 | 0.897 / 0.896 |
| (same) | SfM centre error, 8 scenes | 0.297% | 0.295% |
| GLSL parity | orientation error CUDA / GLSL | | 0.733° / 0.727° |
| (same) | features per image, 20 Meetingroom images, CUDA / GLSL | | 10,257 / 10,233 |

Darkness adaptation, all features kept (no SiftGPU or ImageMatching cap), SfM centre error / share of
cameras within 1° and 1%: normal scenes (8) 0.343% / 89.7% off, 0.277% / 91.7% on (RSa1 0.270% /
92.3%); low-light copies (5 scenes, about -3 EV with photon and read noise) 0.419% / 84.2% off,
0.434% / 71.8% on. Capped at 8192 or 2048 features per image it lowers match precision on weak scenes
(0.29 → 0.26 at 8192). Hence off by default: it helps uncapped extraction of well exposed images.

CreateStructure on Ignatius registers 262 of 263 images with the shipped orientation (two runs) and
263 with the histogram; the dropped image was borderline before (19 2D-3D correspondences for a
minimum of 15, 12 now). ImageMatching registers all 263 with the same features.

## 6. Rejected alternatives

- Fixed 256 px cell quota without redistribution: 14% fewer correct matches, pose AUC@10 0.888 vs
  0.892 (20 scenes), no SfM gain (0.368% vs 0.366%).
- Threshold 0.0015 or 0.001 without the curvature floor: more features, no more correct matches.
- CLAHE before upload: doubled decode time without a matching gain.
- Floor 0.0007 with threshold 0.001 (edge 20): highest weak-cell coverage (0.757), SfM 0.365% vs
  0.338%.
- Curvature from a smoothed 5x5 fit instead of 3x3 differences: no better.
- RSa1's stability recheck (offset agreement with the preceding DoG level): removed 69-78% of the
  keypoints at 0.2 px², worse on every metric at 1 or 4 px².
- RSa1-style local contrast margin: weak-cell coverage collapsed to 0.28.
- 4 levels per octave: +5% correct matches at native, lower top-1, mixed pose AUC; first octave 0:
  3x fewer features, worse.
- Darkness adaptation of the DoG threshold only: no effect, the floor rejects the added extrema.
- Square-root darkness factor: better match precision, worse SfM (0.325% on normal scenes, 0.459% on
  low-light ones); a size-dependent noise floor on the factor rarely applied.
- 6 or 7 harmonics, smoothing 0.30 rad: within noise of 5 harmonics at 0.35 rad.
- Shared-memory orientation histogram: no measurable speed difference.
- Ranking the selection by DoG contrast, or a coarse 6-cell share followed by contrast × scale: more
  repeatable across views (rank correlation 0.83-0.90 vs 0.50-0.73 for scale) and better at an 8192
  budget (SfM 0.359% vs 0.397%), but worse at the default 27,000 (0.341% vs 0.318%); it would also
  need SiftGPU to return the DoG value of each keypoint.

## 7. Open items

- A per-image budget well below 27,000 would favour the two-stage contrast selection; SiftGPU would
  have to export the refined DoG value it computes in `ComputeKEY_Kernel`.
- Darkness adaptation needs a noise-aware threshold to help on underexposed images.
