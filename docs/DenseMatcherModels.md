# Dense Matcher Models (RoMa v2 and FlexGlue) — Obtaining and Publishing

## Overview

`--dense-matcher` (RETRIEVAL pair selection and one-pass dense matching, `design/DenseMatching.md`) needs one exported model bundle on
disk: RoMa v2 (`--dense-matcher roma2`) or FlexGlue (`--dense-matcher flexglue`). A bundle is a folder holding a JSON manifest that ties its graphs
together and the ONNX graphs it names. OpenMVS carries none of it in its own repository: each bundle is fetched on demand from a
published copy and verified against a checksums file the repository does carry (`models/<model>/checksums.txt`). With FlexGlue
selected, only the FlexGlue bundle is loaded, required or looked for.

| | RoMa v2 | FlexGlue |
|---|---|---|
| manifest | `roma_<setting>.json` | `flexglue_<setting>.json` |
| graphs | descriptor (DINOv3 ViT; `layers`, `value_facets`, 2048-D `retrieval`), coarse match (both warps) | descriptor (DINOv3 ConvNeXt-B + FlexGlue head; `features`, 256-D `retrieval`), match (both warps, confidences, covisibility, H1 relative pose) |
| published preset | `base-fp32` (~1.43 GB, weights in `.onnx.data` sidecars) | `base-fp16` (~0.46 GB: descriptor fp32, match graph with float16 transformer layers; weights inside the `.onnx`) |
| option / environment | `--roma2-model DIR` / `$OPENMVS_ROMA2_MODEL_PATH` | `--flexglue-model DIR` / `$OPENMVS_FLEXGLUE_MODEL_PATH` |
| install copy | `<prefix>/share/openMVS/roma2` | `<prefix>/share/openMVS/flexglue` |
| CMake target | `roma2-model` | `flexglue-model` |
| Hugging Face (canonical, fetched at the commit the script pins) | `cDcSeacave/openmvs-roma2-onnx` at `ea2b04648ba0` | `cDcSeacave/openmvs-flexglue-onnx` at `f635c84532db` |
| GitHub mirror (the same files, flat) | release `roma2-model` of `cdcseacave/openMVS` | release `flexglue-model` of `cdcseacave/openMVS` |

Model discovery follows the same order for both matchers: the explicit command-line option, else the
environment variable, else the installed `<prefix>/share/openMVS/<matcher>` directory (verified to exist on
disk, so an unfetched prefix still reports a missing model).

## Obtaining a model

**`cmake --build <build-dir> --target roma2-model`** or **`--target flexglue-model`** — opt-in targets, never part of `all`
and never run at configure time. They exist only with `OpenMVS_USE_ONNXRUNTIME=ON` and a Python3 interpreter found at configure
time (otherwise CMake logs a status message pointing at the script). They run

```
scripts/fetch_model.py --model roma2    --setting base --precision fp32 --dest <install-prefix>/share/openMVS/roma2
scripts/fetch_model.py --model flexglue --setting base --precision fp16 --dest <install-prefix>/share/openMVS/flexglue
```

into the install prefix, so they usually need the privileges of `make install`; an unprivileged user runs the script by hand into
a folder they own and points the environment variable at it.

**`scripts/fetch_model.py`**:

```
scripts/fetch_model.py --model {roma2,flexglue} [--setting base] [--precision P] [--dest DIR]
                       [--repo ID] [--revision REV] [--mirror URL]
```

- `--model` — the bundle (required).
- `--setting`, `--precision` — the preset (default: the published one, `base`/`fp32` for RoMa v2, `base`/`fp16` for FlexGlue).
- `--dest` — the model folder itself (default: the model's environment variable, else `~/.cache/openMVS/<model>`; never the
  current directory, which is usually a checkout that must not receive the DINOv3 weights). The files land flat, as the loader reads
  them, though both hosts nest them under `<setting>-<precision>/`.
- `--repo`, `--revision`, `--mirror` — the Hugging Face repo, the Hub commit, the mirror base URL (defaults: the model's own).

Two sources, tried in order: the Hugging Face repo (via `huggingface_hub.snapshot_download` when importable — resumable,
revision-pinned), then the GitHub-release mirror (streamed with `urllib.request`), reached both when `huggingface_hub` is missing
and when the Hub fetch fails. The destination is checked for write access first. Every file is verified against
`models/<model>/checksums.txt` (SHA256, read relative to the script): a file already present with the right digest is left alone,
one with a wrong digest is deleted before a fresh attempt, and a mismatch after fetching deletes the file and fails naming both
digests. A preset the checksums file has no entries for fails by name. Nothing of this runs in CI.

## What's published

RoMa v2 `base-fp32` and FlexGlue `base-fp16`. Other presets would be sibling folders on the same hosts with their own checksum
entries; two precisions of one setting would first need distinct manifest names.

## The FlexGlue bundle

Exported from FlexGlue with the H1 pose head, traced with the
`torch.onnx` dynamo exporter at opset 18 (ONNX Runtime's contrib MultiHeadAttention and Gelu in the match graph), the
exporter's stack-trace metadata stripped. Contents of `base-fp16/`:
`flexglue_base.json` (manifest: sizes, normalisation, retrieval recipe, gates, heads and their conventions, files, SHA256 of each
graph, versions of the exporter and the source), `flexglue_base_descriptor_fp32.onnx`, `flexglue_base_match_fp16.onnx`, their
`.export.json` sidecars, and `export.log`. Conventions: the pose head H1 reads no camera; `x1 = R10 x0 + s t10` (`s > 0` unknown),
`t10` a unit direction in camera 1. Parity with PyTorch: `design/DenseMatching.md` (Validation).

## Licence

Both descriptor graphs embed DINOv3 (Meta) backbone weights, which are **not** covered by OpenMVS's own AGPL licence: they are
distributed under Meta's DINOv3 License, which

- **§1(a)** grants a royalty-free, worldwide right to use, reproduce, distribute, copy, and create derivative works of and modify
  the materials — no revenue threshold, no acceptable-use annex;
- **§1(b)(i)** permits redistributing the materials or derivatives only under that same Agreement, with a copy of it supplied;
- **§1(b)(ii)** asks that published research using them acknowledge DINOv3;
- **§1(b)(iii)** and **(v)** require trade-control compliance and forbid ITAR, military, weapons, nuclear and espionage use;
- **§8** lets Meta amend the Agreement.

The other weights: RoMa v2's are MIT (Johan Edstedt); FlexGlue's feature head and matcher are MIT (Computer Vision and Geometry
Lab, ETH Zurich), and so is the H1 pose head (the licence of its repository; it was trained on features of the frozen FlexGlue
model, DINOv3 backbone included).

| bundle | graph | weights | licence |
|---|---|---|---|
| RoMa v2 | descriptor | DINOv3 ViT backbone | DINOv3 License |
| RoMa v2 | coarse match | RoMa v2 matcher | MIT |
| FlexGlue | descriptor | DINOv3 ConvNeXt-B backbone (87.6 M parameters) | DINOv3 License |
| FlexGlue | descriptor | FlexGlue stride-8 head (2.0 M) | MIT |
| FlexGlue | match | FlexGlue matcher (about 46 M) | MIT |
| FlexGlue | match | H1 pose head (0.46 M) | MIT |

**Running `--dense-matcher roma2` or `flexglue` with either model means accepting the DINOv3 License.** `scripts/fetch_model.py` prints this notice on every
fetch, pointing here for a summary and at `LICENSE-DINOv3.md` in each published repository for the pinned text (it is not one of
the files fetched into `--dest`). Read the Agreement itself for the exact terms; this section summarizes it.

## Publishing a new bundle (maintainer only)

Each host needs the nested `<setting>-<precision>/` layout of the checksums file. Stage it once with the model card and the
licence files, then upload the staged copy:

```
# RoMa v2: manifest + 2 graphs + 2 .onnx.data + 2 .export.json (+ the export logs) under base-fp32/, README.md, LICENSE-DINOv3.md, LICENSE-RoMaV2-MIT.txt
hf upload cDcSeacave/openmvs-roma2-onnx publish . --repo-type model
gh release upload roma2-model publish/base-fp32/* --repo cdcseacave/openMVS
# FlexGlue: manifest + 2 graphs + 2 .export.json (+ export.log) under base-fp16/, README.md, LICENSE-DINOv3.md, LICENSE-FlexGlue-MIT.txt
hf upload cDcSeacave/openmvs-flexglue-onnx publish . --repo-type model
gh release upload flexglue-model publish/base-fp16/* --repo cdcseacave/openMVS
```

GitHub release assets cannot carry a subdirectory, so the mirror upload is flat, and the fetch script maps the mirror's bare names
to its flat local names. The mirror tags (`roma2-model`, `flexglue-model`) are what the script's default mirror URLs name. A new
export's checksums can be pinned as soon as the export exists (they hash file content), with the `<setting>-<precision>/` prefix
added; the script's default `--revision` waits for the upload: it moves to the commit SHA the upload prints (for both bundles), so a
fetch pins exactly the bytes the checksums describe.
