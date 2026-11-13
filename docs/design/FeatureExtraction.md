# Feature Extraction

## Purpose and scope

`FeaturesExtractor` (`libs/SFM/FeaturesExtractor.h/cpp`, called by `Scene::ExtractFeatures`) detects keypoints and computes
descriptors on every image of the scene, spreading them over the frame so that textured regions do not use up the budget
of weakly textured ones. The result is `Image::keypoints`, `Image::descriptors` (the *described* keypoints; dense
keypoints appended later by the dense matcher come after them and have no descriptor row, `DenseMatching.md`) and the
scene state `FEATURES_EXTRACTED` with the detector type. The SiftGPU detector has its own document,
`SiftGPUFeatureExtraction.md`. Spherical images are extracted face by face (`SphericalCameras.md`).

## Detectors

| `--detector-type` | detector | descriptor | notes |
|---|---|---|---|
| `AKAZE` | OpenCV AKAZE, CPU | binary, matched by Hamming distance | matching defaults: ratio 0.9, absolute bound 100, 50 matches minimum |
| `ORB` | OpenCV ORB, CPU, `nfeatures` = `maxFeaturesPerCell` | binary, Hamming | ratio 0.9, absolute bound 64, 50 matches minimum |
| `SIFT` | OpenCV SIFT, CPU | RootSIFT, 128 bytes | ratio 0.8, no absolute bound, 15 matches minimum |
| `SIFTGPU` | SiftGPU on CUDA or OpenGL | RootSIFT, 128 bytes | same matching defaults as SIFT; see `SiftGPUFeatureExtraction.md` |

The matching defaults of the chosen detector (`MatchConfig::DefaultsForFeatureType`: binary or quantized descriptors, the
number of descriptors per image given to the vocabulary tree, the absolute distance bound, the ratio, `minMatches`) are
applied by `CreateStructure` right after the detector is parsed; see `PairSelection.md` and
`TwoViewVerificationAndLens.md`.

**Default detector.** The command line's default is `SIFTGPU` (`FeatureType::DEFAULT`), and the matching defaults follow
the detector parsed. The library's `FeatureExtractionConfig::detectorType` default is `AKAZE`; a library or Python caller
that does not set it gets AKAZE (see open items).

**RootSIFT.** For SIFT and SIFTGPU the 128-dimensional float descriptor is L1-normalized, square-rooted, and quantized to
`uint8` by scaling by 512 (most values are below 0.4, so the scale keeps more precision than 255 would)
(`FeaturesExtractor::ConvertToRootSIFT`). Descriptors are stored as bytes for every detector; whether Hamming or L2 is
used is `MatchConfig::descriptorsAreBinary`.

## Grid extraction (CPU detectors)

`ExtractImage` splits the image into a 3 x 3 grid and extracts per cell, so features cannot all concentrate in
textured areas and featureless regions are not ignored.

1. Each cell is detected on its region extended by a border of `min(64, min(cellWidth, cellHeight) / 2)` pixels on the
   sides that touch another cell, so structures cut by a cell boundary are detected whole. Keypoint coordinates are
   moved to the image frame, and only keypoints inside the core cell (without the border) are kept, so no area is
   extracted twice.
2. If a cell yields fewer than `minFeaturesPerCell` (500) keypoints, the detector is run again up to five times with
   progressively more sensitive settings, and the detector is reset to its default afterwards:

   | detector | setting by retry | default restored |
   |---|---|---|
   | AKAZE | threshold 0.0005, 0.0001, 5e-5, 1e-5, 1e-6 | 0.001 |
   | ORB | FAST threshold 15, 10, 7, 5, 3 | 20 |
   | SIFT | contrast threshold 0.03, 0.02, 0.015, 0.01, 0.005 | 0.04 |

3. A cell keeps at most `maxFeaturesPerCell` (3000) keypoints, the best by `Image::ComputeKeypointWeight`
   (`response / (response + 0.03)` times a size factor ramping from 0.5 at 2 px to 1.5 at 20 px). The whole image holds at
   most `GetMaxNumFeatures() = 9 * maxFeaturesPerCell` (27,000) features. For SiftGPU the same budget is spent on a finer
   adaptive grid (`SiftGPUFeatureExtraction.md`).
4. AKAZE and ORB responses are quadratic in contrast; they are square-rooted so that every detector's response scales
   linearly with contrast, which the keypoint weight relies on.

CPU extraction runs one detector per pool thread with OpenCV's own threading off. Image pixels are released after extraction
(`releaseImagePixels`). An image that already has features (a loaded scene) is skipped; extraction on an image with dense
keypoints is a contract violation (asserted), since it would discard keypoints that stored matches still reference.

OpenMVG interchange: `--import-openmvg-dir` reads `<stem>.feat` (x y scale orientation per line) and `<stem>.desc`
(binary descriptor rows) instead of extracting an image that has them; `--export-openmvg-dir` writes the described
keypoints of every extracted image in the same layout.

## Options and defaults

| option | default | meaning |
|---|---|---|
| `--detector-type`, `-t` | `SIFTGPU` | AKAZE, ORB, SIFT or SIFTGPU |
| `--max-features-per-cell` | 3000 | features per cell of the 3 x 3 grid; times 9 is the per-image budget |
| `--min-features-per-cell` | 500 | below this a cell is detected again with a more sensitive setting |
| `--siftgpu-cuda` | true | SiftGPU backend: CUDA (if available) or OpenGL/GLSL; also the backend of the SiftGPU matcher |
| `--siftgpu-darkness-adaptivity` | false | SiftGPU: lower the DoG threshold and curvature floor in dark areas |
| `--gpu-device` | -1 | (CUDA builds) GPU selection; -2, `cpu` or empty selects the CPU path (and OpenGL/GLSL for SiftGPU) |
| `--release-descriptors` | true | release descriptors after matching |
| `--spherical-bottom-band` | 0.2 | spherical images only (`SphericalCameras.md`) |

## Invariants

- Extraction produces described keypoints only: `keypoints.size() == descriptors.rows` on return.
- A SiftGPU keypoint carries the fixed response 0.1; its per-image selection ranks by scale, not by response.

## Open items

- The detector default differs between the command line (`SIFTGPU`) and `FeatureExtractionConfig` (`AKAZE`). A caller
  that builds a `ReconstructionConfig` without the command line gets AKAZE unless it sets the detector, and its
  `MatchConfig` defaults must be set for the detector with `DefaultsForFeatureType`.
- `--gpu-device` does not choose the SiftGPU CUDA device: SiftGPU is initialized on CUDA device 0 whenever CUDA is wanted.
  Only the ONNX Runtime CUDA provider follows the device `SEACAVE::CUDA` selected.
