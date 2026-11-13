# SiftGPU Feature Extraction

## 1. Purpose and scope

`FeaturesExtractor` with `detectorType = SIFTGPU` (CreateStructure `--detector-type SIFTGPU`, the command line's default)
extracts SIFT keypoints and RootSIFT descriptors on the GPU for the sparse reconstruction. This document covers the host
side in `libs/SFM/FeaturesExtractor.cpp` (image decoding, SiftGPU parameters, per-image feature selection) and the detector
itself in the local vcpkg port `ports/siftgpu/source/src` (CUDA: `ProgramCU.cu`, `PyramidCU.cpp`, `CuTexImage.cpp`;
OpenGL: `ProgramGLSL.cpp`, `PyramidGL.cpp`, `ShaderMan.cpp`). The backend is selected by
`FeatureExtractionConfig::useCUDA` (CreateStructure `--siftgpu-cuda`, 1 by default; 0 or a CPU-requesting `--gpu-device`
selects OpenGL/GLSL, also for the SiftGPU matcher); the OpenGL path implements the same detector, orientation and options
and gives the same features up to floating-point differences. The other detectors, the grid extraction they share and
RootSIFT are in `FeatureExtraction.md`.

---

## 2. Algorithm as implemented

### 2.1 Input pipeline (`SiftGPUFeatureCoordinator::ProcessImages`)

One thread owns the SiftGPU context. Images are decoded ahead of it on `scene.threadPool`: one task
and one `std::future<bool>` per image, at most the current image plus one task per pool thread in
flight (a one-thread pool still decodes the next image during the GPU work) and at most 960 MB of
decoded images (their 8-bit pixels and float input); the image after the current one is decoded even
when the two exceed the budget. A task loads the pixels, converts them to
SiftGPU's float grey input and optionally releases the image pixels; the GPU thread waits on the
image's future, uploads, extracts, and never loads an image itself. An image that fails to load is
reported through the future's value; an exception travels through the future to the GPU thread, and
on every exit from the pipeline a guard waits for the decode and post-processing tasks still using
its buffers.

### 2.2 Gaussian and DoG pyramid

SiftGPU's pyramid: first octave `-fo -1` (input upsampled 2x), 3 levels per octave (`-d 3`),
`-maxd = maxImageSize * 2`. Gaussian levels, DoG and gradient magnitude/angle levels are pitch-linear
`CuTexImage` buffers whose rows are padded to a multiple of 4 pixels; the padding holds no image data,
so keypoints are searched only inside the octave's image width (`PyramidCU::_octaveWidth`), which
keeps every keypoint inside the input image. Each buffer caches the texture objects bound to it (one per channel format
and texture descriptor, and for the 2D ones per view size); they are created on first use and
destroyed when the buffer is reallocated or destroyed, the 2D ones also when the view size changes
(after a device synchronization), never by a copy (`CuTexImage` is not copyable).

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
extrema, which a low threshold otherwise admits, are rejected: the floor, not the DoG value, is the
main contrast test.

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
at 36 sample angles `(t + 0.5)·10°`; the peak search, the parabolic refinement and the second
orientation (a local maximum above 0.8 of the highest, `-mo 2`, threshold `-ot`, read by CUDA and
GLSL) are SiftGPU's. Two local maxima are at least two samples (20°) apart, and the 0.35 rad
smoothing merges directions closer than about 40° into one peak. 0.35 rad equals the smoothing of
six `[1 1 1]/3` passes over a 36-bin histogram, and the angle is not quantized before smoothing. The density is the
same in CUDA and GLSL (`OrientationMomentMacros`, `OrientationGain`).

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
| Backend `-cuda` / `-glsl` | CUDA | `useCUDA`, `--siftgpu-cuda` |
| Darkness adaptation `-da 0/1` | off | `siftgpuDarknessAdaptivity`, `--siftgpu-darkness-adaptivity` |
| First octave, levels per octave | -1, 3 | constants |
| Orientations per keypoint `-mo` | 2 | constant |
| Second-orientation threshold `-ot` | 0.8 of the highest peak | SiftGPU default |
| Orientation harmonics, smoothing | 5, 0.35 rad | `ORIENTATION_HARMONICS`, `ORIENTATION_SMOOTHING` (CUDA), GLSL constants |
| Selection budget | `GetMaxNumFeatures()` (27,000) | `maxFeaturesPerCell` x 9 |
| Selection cell | about 256 px | `gridCellSize` |
| Decode look-ahead | pool threads + 1, 960 MB | `ProcessImages` |

SiftGPU's own defaults for `-cmin` (0 = off) and `-da` (off) leave other users of the library on
the original detector; the orientation estimator has no switch.

## 4. Invariants and constraints

- `-cmin` is compared with the curvature of the level's DoG divided by sigma^2, so it is independent of the octave;
  changing `-d` or the pyramid sigmas changes its meaning.
- A texture object belongs to the buffer it is bound to and lives until that buffer is reallocated (a 2D one until the
  view size changes); kernels may run asynchronously after the host function that bound it returns, so it is destroyed
  only once they are done.
- The CUDA and GLSL detectors stay equivalent: the same thresholds, floor, darkness factor and orientation density
  (feature counts within about 0.3%, orientation error within 0.01 deg).
- SiftGPU keeps its options and backend in process-global state, so `FeaturesExtractor` passes the backend and every
  option explicitly on each initialization.
- The SiftGPU context is bound to one thread; decoding and the post-processing of the features run on the pool.

---

## 5. Validation of the shipped defaults

Benchmark: feature matching against reference poses of 7 TnT and 13 ETH3D scenes (1,294 image pairs), a synthetic suite
with exact homographies, and SfM on the extracted features (3-5 multithreaded runs, median camera-centre error relative
to the camera extent). Run-to-run SfM noise is a few hundredths of a percent per scene average.

| Measure | Shipped value |
|---|---|
| ETH3D 24 MP images per second with decoding on the pool (facade, office) | 12.8, 16.4 |
| GPU time per 1920x1080 image (CUDA list generation, cached textures, GPU orientation split, one download) | 21 ms |
| CreateStructure feature extraction, Ignatius, 263 images | 5.5 s |
| Ratio-test precision, 20 scenes, native size / 8192 features | 0.664 / 0.607 |
| Weak-cell coverage, 20 scenes, native size | 0.678 |
| Correct mutual matches per pair, native size | 2028 |
| Pose AUC@10, native / 2048 features | 0.891 / 0.781 |
| Median orientation error, synthetic rotation and scale (moment estimator) | 0.73 deg |
| Top-1 ratio / correct matches, synthetic | 0.946 / 11,410 |
| Pose AUC@10, 5 dev scenes, weak / textured, native | 0.897 / 0.896 |
| SfM centre error, 8 scenes | 0.295% |
| Orientation error CUDA / GLSL | 0.733 deg / 0.727 deg |
| Features per image, 20 Meetingroom images, CUDA / GLSL | 10,257 / 10,233 |
| Darkness adaptation off / on (default off), SfM centre error / share of cameras within 1 deg and 1%, all features kept: 8 normal scenes | 0.343% / 89.7% / 0.277% / 91.7% |
| Darkness adaptation off / on, 5 low-light scenes (about -3 EV with photon and read noise) | 0.419% / 84.2% / 0.434% / 71.8% |

Darkness adaptation helps uncapped extraction of well exposed images and, with the per-image cap at 8192 or 2048,
lowers match precision on weak scenes (0.29 to 0.26 at 8192); hence it is off by default.

---

## 6. Rejected alternatives

- A fixed 256 px cell quota without redistribution: 14% fewer correct matches, no SfM gain.
- A lower DoG threshold (0.0015 or 0.001) without the curvature floor: more features, no more correct matches.
- A stability recheck against the preceding DoG level: removed 69-78% of the keypoints at 0.2 px^2 and was worse on every
  metric at 1 or 4 px^2.
- One orientation per keypoint instead of up to two: loses weak-region supports in 13 of 20 scenes at the default budget;
  a second-orientation threshold of 0.9 instead of 0.8 costs 4-5% of the correct matches on weak and low-light scenes.
- 4 levels per octave (+5% correct matches but lower top-1, mixed pose AUC) and first octave 0 (3x fewer features).
- Darkness adaptation of the DoG threshold only (no effect; the floor rejects the added extrema) and a square-root
  darkness factor (better match precision, worse SfM).
- Ranking by DoG contrast for the per-image selection: better at an 8192 budget, worse at the default 27,000, and it
  needs SiftGPU to return the DoG value of each keypoint.
- Adaptive non-maximal suppression, SSC/KD/range suppression, quadtrees, weighted farthest-point or Poisson sampling,
  DPP selection, overlapping cells with protected floors, descriptor-uniqueness rankings: no consistent gain in correct
  matches, weak coverage or reconstruction.
- CLAHE before upload: doubled the decode time without a matching gain.
- An own-image stability prepass (four transformed extractions and self-matching): 4.6-10.4 s per image against 0.4-0.8 s,
  for no better poses.

---

## 7. Open items

- A cheap image-local ranker that retains weak-region support and useful alternate orientations is unresolved; the cost of
  exporting the DoG value of each keypoint and any quality gain require measurement.
- Darkness adaptation needs a noise-aware threshold to help on underexposed images.
