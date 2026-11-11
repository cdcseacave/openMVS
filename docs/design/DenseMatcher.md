# Dense Matchers (RoMa v2, FlexGlue) behind one interface

The in-process dense matching of `ROMA2InProcess.md` (the describe pass, the retrieval descriptors, the one pass of
verdict / guided matching / dense fill / store) reads one thing from the model: a **warp** per pair (for every cell
of a coarse grid on image A, where that cell lands in B and how confident the model is, both directions). Two models
produce it, selected by `--dense-matcher roma2|flexglue` (`ROMA2Config::matcher`); RoMa2 is the default and its output
is unchanged by the interface. Everything downstream of the warp (`JudgePairROMA2`, `TrackKeypointsByWarp`,
`MatchFeaturesGuided`, `AssemblePairROMA2`, `StorePairROMA2`, the planar twin switch, the lens fit, the dense
pseudo-count in the pair weight) is shared and never asks which model made the warp; the verdict's tolerance is half a
warp cell, taken from the grid.

## The interface (`libs/SFM/DenseMatcher.h`)

`DenseMatcher` is what `Scene::MatchPairs` creates (one per call, destroyed after `PairsMatcher`), what the describe pass
(`ComputeGlobalDescriptorsROMA2`) and the warp pass (`ForEachWarpROMA2`, `MatchPairsROMA2`) drive, and what
`PairsMatcher::SetROMA2` holds. One thread drives it.

| member | meaning |
|---|---|
| `Load(dir, setting, provider)` | the graphs of the model folder on the execution provider (`auto/cuda/coreml/dml/cpu`) |
| `ImageSize()`, `Preprocess(bgr, planarRgb)` | the side of the square an image is squashed to and the planar RGB floats made of it (each model keeps its own resize; the prefetch ring that runs it on the pool is shared) |
| `GridSize()` | cells per side of the warp the model returns |
| `MakeCode()`, `DescribeImage(planar, code, retrieval)` | one image's descriptor tensor (on the device where the provider keeps tensors) and its unit global retrieval descriptor (`RetrievalSize()` floats, stored in `Image::globalDescriptor`) |
| `MatchPair(codeA, codeB, PairInput, PairOutput)` | both warps of a pair, plus the optional outputs below |
| `NeedsIntrinsics()` | whether `MatchPair` reads the cameras in `PairInput`, so that every image must have a pinhole camera |

`PairInput` carries the pixel sizes of the two images and their pinhole intrinsics as the pipeline holds them at the matching
stage (`MakePairInput` in `MatchROMA2.cpp`, from `Image::GetK()`): the focal that import, `--focal-length` or the EXIF gave,
**before** the lens fit and the focal refinement (`CalibrateFocalsAndFitLenses`) that run after the matching.

`PairOutput` is the `PairWarps` plus two optional outputs, absent where a model gives none (RoMa2 gives neither):

- `covisibility` (`std::optional<float>`, 0..1): "do the two images show the same place".
- `pose` (`std::optional<PosePrior>`): a coarse relative rotation `R` and unit baseline direction `t` in the pair's own
  convention (`x2 = R*x1 + s*t`, `s > 0` unknown; the convention of `ImagePair::relativePose`, whose `GetT()` is `t`
  up to scale). A prior to check or guide a pose by, never the pose: nothing reads it yet.

The pair callback of `MatchPairsROMA2` tests presence, never the model's name: a pair with a covisibility below
`ROMA2Config::minCovisibility` is dropped before the verdict (the verdict, the guided matching and the fill are not paid
for it; logged as `ROMA2 covisibility gate: N of M warped pairs below B`), and the two outputs of a stored pair go
into `ImagePair::covisibility` and `ImagePair::posePrior`. They are not serialized (the archive layout is read by tools
built before them) and not geometry (no `Reset` clears them). `--export-pairs-csv` appends the columns
`Covisibility,PriorQw,PriorQx,PriorQy,PriorQz,PriorTx,PriorTy,PriorTz` (the prior in the layout of the `Rel*` columns) only when
some pair has a covisibility, so a RoMa2 or SIFT run writes the file it always wrote.

## RoMa2 behind the interface

`RoMa2Onnx : DenseMatcher`, thin adapters over the calls the rest of `ROMA2InProcess.md` describes: `Preprocess` =
`PreprocessImageRoMa2` at the manifest's size, `MakeCode` = `MakeLayers`, `DescribeImage` = `Describe` (the graph's own
2048-D retrieval pooling), `MatchPair` = `MatchCoarse`. No logic moved.

## FlexGlue (`libs/SFM/FlexGlueMatcher.h/cpp`)

FlexGlue (cvg/FlexGlue; a DINOv3 ConvNeXt-B backbone, a stride-8 feature head and a 9-layer transformer that matches two
feature maps) runs from three ONNX graphs exported by `export_flexglue.py` (records folder
`T/records-2026-11-11-flexglue/`, traced from the PyTorch modules with the fused-attention conventions of FlexGlue's own
`onnx/export.py`; model folder `flexglue-onnx-20261111`):

| graph | inputs -> outputs |
|---|---|
| `flexglue_descriptor.onnx` | `image` [1,3,512,512] RGB in [0,1] (squashed bilinear, no antialiasing, as FlexGlue does) -> `features` [1,256,64,64] and `retrieval` [1,256] |
| `flexglue_dense.onnx` | `features0/1`, `image_size0/1` (w,h of the original images) -> `warp0/1` [1,128,128,2], `confidence0/1` [1,128,128], `covisibility` [1], `pooled0/1` [1,512,16,16] |
| `flexglue_pose_h0.onnx` | `pooled0/1`, `intrinsics0/1` [1,4] (fx, fy, cx, cy in the 512 frame) -> `R10` [1,3,3], `t10` [1,3] |

- **Warps.** FlexGlue's dense mode at coarse stride 4: both directions on a 128 x 128 grid of cell centres, the target in
  normalised [-1,1] coordinates of the original frame (RoMa2's warp convention, so the shared code reads them as they
  are). A cell's confidence is 0 where it fails FlexGlue's own checks (confidence under 0.1, forward-backward cycle
  error over 3 descriptor pixels), so `ROMA2Config::minConfidence` (0.1) keeps exactly the cells FlexGlue keeps.
- **Covisibility.** FlexGlue's covisibility score of the pair, read from the same transformer pass as the warps (no
  extra call). The gate is on at 0.2 (`--dense-min-covisibility`, 0 = off). A RoMa2 run is never gated: it gives none.
- **Pose head.** FlexGluePose's H0 head (216 k parameters) on the final pair-conditioned features of the transformer,
  average-pooled to 16 x 16 inside the dense graph, with the two cameras' rays; one small extra graph call per pair
  (about 8 ms on 104 ms of the pair's graph call, below the 15% bar at which it would have become an option); it needs a pinhole camera
  for every image and a spherical scene is refused by name. The
  intrinsics are the ones of the matching stage, scaled to the 512 frame. Its output is `R10` (camera-0 to camera-1
  rotation) and `t10` with `x1 = R10 x0 + s t10`, the convention of the pair's `relativePose`.
- **Retrieval descriptor.** The `features` map GeM-pooled (generalised mean, p = 3, clamped at 1e-6) and L2-normalised
  inside the descriptor graph: 256 floats per image, stored in `Image::globalDescriptor` exactly where RoMa2's 2048 go, so
  `--match-mode 4` ranks candidate pairs from it unchanged (`CollectRetrievalPairs`). No RoMa2 model is loaded in a FlexGlue
  run. The choice among poolings is measured in the validation table.
- **Descriptor tensors** (256 x 64 x 64 floats, 4 MB per image against RoMa2's 13 MB) live on the device (`MakeDeviceTensor`)
  and are bound into the dense graph; the warps and scores come back to the host.

### Licences

Code of FlexGlue and FlexGluePose: MIT. The weights file `flexglue.pth` contains the DINOv3 ConvNeXt-B backbone
(`extractor.backbone.*`), distributed under the **DINOv3 License** (Meta; a use-restricting licence, not MIT): the descriptor
graph and the dense graph's input features carry it. The matcher and descriptor-head weights are MIT. The H0 pose head
weights (`checkpoints/h0.pt`) ship in the FlexGluePose repository (MIT code; the checkpoint has no separate licence file).
Nothing in openMVS decides on this: the model files are read from a folder the user names and are never built in.

## Options

| option | default | effect |
|---|---|---|
| `--dense-matcher roma2\|flexglue` | `roma2` | the model behind `--roma2 true` |
| `--flexglue-model DIR` | `$OPENMVS_FLEXGLUE_MODEL_PATH` | folder with the three graphs |
| `--dense-min-covisibility F` | `0.2` | gate before the verdict, 0 = off; ignored by a model that gives no score |

The other `--roma2-*` options (`--roma2-slots`, `--roma2-min-confidence`, `--roma2-min-overlap`, `--roma2-dense-matches`,
`--roma2-provider`) apply to FlexGlue's warps as they do to RoMa2's; `--roma2-model` and `--roma2-setting` are RoMa2's.

## Invariants

- `DescribeImage` returns exactly `RetrievalSize()` floats of unit norm; a `MatchPair` warp is `GridSize()` square with
  confidence in [0,1]; a covisibility is in [0,1] (asserted).
- A matcher with `NeedsIntrinsics()` runs only on pinhole cameras (checked once, at the start of `MatchPairsROMA2`).
- The interface adds no state to the shared passes: slot planning, prefetching and the semaphore bounding live warps are
  unchanged.

## Validation

Records: `T/records-2026-11-11-flexglue/` (`export_flexglue.py`, `parity_flexglue.py`, `retr_*.py`, `queue.py`, `ident.sh`).

**Export parity against PyTorch** (`parity_4e5c6f6d.md`, 10 pairs, ONNX Runtime CPU): descriptor features max 2.6e-5, retrieval cosine
0.9999999, dense warps over the cells valid in both max 4.7e-2 normalised units (median 1.8e-5; 1 unit is half the frame),
covisibility 2.1e-4 against the traced module and 5.8e-4 against upstream float32 FlexGlue, the pose graph against FlexGlue-Pose's own
head on the same pooled input 0.20 deg in rotation and 0.43 deg in direction (the head's own error against references is 3 to 12 deg).

**Retrieval descriptor** (`retr_eval.md`; candidates = pairs in the top-K of both images, K = 10; recall = share of the truly overlapping
pairs, reference ties >= 3, among the candidates; mean over 4e5c6f6d, 1c65aab4, 5ada248e and Palace, whose truth is a pose proxy):

| descriptor | dim | recall K=10 | recall K=20 | recall at RoMa2's candidate count, K=10 |
|---|---|---|---|---|
| RoMa2 (GeM of DINOv3 ViT facets) | 2048 | 0.171 | 0.261 | 0.155 |
| FlexGlue feature map, GeM p=3 (shipped) | 256 | 0.165 | 0.249 | 0.144 |
| backbone last stage, average | 1024 | 0.158 | 0.246 | 0.134 |
| backbone last stage, GeM p=3 | 1024 | 0.144 | 0.221 | 0.126 |

**Pose prior convention** (`FlexGlueReconstructTest`, 4 images, CPU): rotation within 0.7 to 2.7 deg and baseline direction within 2.2 to
16.6 deg of the pose the pair's own matches give.

## Rejected alternatives

- FlexGlue's 512 x 512 refiner (RoMa v2's): not exported; the coarse stride-4 grid is 128 cells per side (RoMa2: 160).
- A pose prior as a filter or a corrector of the pair's pose: dropped; it is stored and exported only.
- FlexGlue's sparse matcher graph for the covisibility alone (one extra graph call per candidate): the dense graph returns the score
  with the warps.
- A covisibility filter after the contradiction cut: dropped; the gate before the verdict is the only use of the score.

## Open items

- Re-confirm the head's input resolution: it was trained on 32 x 32 token maps and is fed the 64 x 64 maps average-pooled to 16 x 16.
- The pose prior is stored and exported but read by nothing.
