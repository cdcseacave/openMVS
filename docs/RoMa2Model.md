# RoMa v2 Model — Obtaining and Publishing

## Overview

`--roma2` (RETRIEVAL pair selection and one-pass dense matching — `docs/design/ROMA2InProcess.md`)
needs the exported RoMa v2 ONNX graphs on disk: a DINOv3 descriptor graph, a bidirectional
coarse-match graph, and the `roma_<setting>.json` manifest tying them together. OpenMVS does not
carry a byte of that model in its own repository — it is fetched on demand from a published bundle
and verified against a checksums file the repository does carry (`models/roma2/checksums.txt`).

## Obtaining the model

**`cmake --build <build-dir> --target roma2-model`** — an opt-in target, never part of `all` and
never run at configure time (fetching ~1.4 GB at configure time would break offline and CI builds).
It exists only when the build has `OpenMVS_USE_ONNXRUNTIME=ON` and a Python3 interpreter was found at
configure time; otherwise CMake logs a status message pointing at running the script by hand instead.
It runs

```
scripts/fetch_roma2_model.py --setting base --precision fp32 --dest <install-prefix>/share/openMVS/roma2
```

— the same path `ROMA2Config::ResolveModelPath()` falls back to when neither `--roma2-model` nor
`$OPENMVS_ROMA2_MODEL_PATH` is given (`OPENMVS_ROMA2_MODEL_INSTALL_DIR`, checked with
`File::isFolder` so an unfetched/uninstalled prefix still produces the ordinary "needs a model"
error rather than a path that does not exist). Run the target once after building with ONNX Runtime
enabled, and `--roma2` needs no further configuration. The target writes into the CMake install
prefix, so it usually needs the same privileges as `make install` — an unprivileged user should
expect to run it with `sudo`, or reconfigure `CMAKE_INSTALL_PREFIX`, or fetch by hand (below) into a
directory they own and point `$OPENMVS_ROMA2_MODEL_PATH` at it instead.

**`scripts/fetch_roma2_model.py`** can also be run directly, with more control:

```
scripts/fetch_roma2_model.py [--setting base] [--precision fp32] [--dest DIR]
                              [--repo ID] [--revision REV] [--mirror URL]
```

- `--setting` — RoMa v2 preset: `turbo`, `fast` or `base` (default `base`).
- `--precision` — exported precision (default `fp32`; `fp16` is not published yet).
- `--dest` — the model directory itself (default: `$OPENMVS_ROMA2_MODEL_PATH`, else
  `~/.cache/openMVS/roma2` — the CMake target always passes `--dest` explicitly, so this default
  only ever serves a hand-run; it is never the current directory, since the obvious hand-run's cwd
  is a repository checkout that does not gitignore the model, and the DINOv3 weights must never
  enter it, see Licence below).
  **`--dest` *is* what `--roma2-model DIR` / `$OPENMVS_ROMA2_MODEL_PATH` mean** —
  `RoMa2Onnx::Load` reads `roma_<setting>.json` and every graph file directly under it, with no
  further nesting. The script therefore writes flat files (`roma_<setting>.json`,
  `roma_<setting>_descriptor_<precision>.onnx[.data]`,
  `roma_<setting>_match_coarse_<precision>.onnx[.data]`, and their `.export.json` sidecars), even
  though the published layout on both hosts nests them under a `<setting>-<precision>/` prefix.
  Multiple presets can share one `--dest` (the filenames already carry `setting` and precision); two
  *precisions of the same setting* cannot yet, since both would want the same manifest name.
- `--repo` — the Hugging Face model repo id (default `cDcSeacave/openmvs-roma2-onnx`).
- `--revision` — the Hugging Face commit to fetch (default: the commit that published the bundle
  `models/roma2/checksums.txt` pins; a re-export moves both together — see Publishing, below).
- `--mirror` — base URL of the GitHub-release mirror, used when `huggingface_hub` is not importable
  or a Hugging Face fetch fails (default `https://github.com/cdcseacave/openMVS/releases/download/roma2-model`).

Two sources, tried in order: the Hugging Face repo (canonical, via
`huggingface_hub.snapshot_download` when that package is importable — resumable, deduplicating,
revision-pinned), then the GitHub-release mirror (streamed with `urllib.request`) — reached both when
`huggingface_hub` is not installed and when it is installed but the fetch through it failed (offline,
blocked, an outage, an unknown repo/revision), so installing it can only ever add a first attempt, never
take a working path away. A warning names which of the two happened before the mirror is tried, so a
genuinely broken `--repo`/`--revision` still reads differently from a machine that simply has no
`huggingface_hub`. GitHub flattens release assets, so the mirror's bare file names are already the
script's flat local names — nothing to reconstruct on that side, unlike the Hub fetch, which moves the
needed files up out of its own `<setting>-<precision>/` nesting into `--dest`.

Before fetching anything, the destination is checked for write access — the failure that hits a
`roma2-model` CMake target run without the needed privileges (Obtaining the model, above) — and reported
by name rather than surfacing as a raw traceback or being blamed on Hugging Face.

Every fetched file is verified against `models/roma2/checksums.txt` (SHA256, read relative to the
script itself, not the caller's working directory) — that file is what pins the published artefact's
exact bytes without the repository carrying any of it, and what makes the mirror as trustworthy as
the canonical host. `base`/`fp32`'s seven digests are pinned there already (they are hashes of file
contents, so they were valid before any upload). A file already present with the right digest costs
one hash and is left untouched ("cached"); one with the wrong digest is deleted before a fresh
attempt; a mismatch after fetching deletes the file, prints both digests and exits with an error —
never a partially-verified result reported as success. Requesting a `--setting`/`--precision`
combination the file has no entries for (today, anything but `base`/`fp32`) fails the same way,
naming the missing prefix rather than fetching files nothing can verify.

Nothing about this runs in CI: the model is large and needs network access, and the fetch is never
triggered by a plain build.

## What's published today

`base` preset, fp32, ~1.43 GB. `turbo` and `fast` follow later from the same repo layout, and fp16
lands as a sibling directory once it has been measured — `--setting`/`--precision` exist precisely
so neither addition changes anything already published.

## Licence

The descriptor graph embeds DINOv3 (Meta) backbone weights; RoMa v2's own code is MIT
(Johan Edstedt). The DINOv3 weights are **not** covered by OpenMVS's own AGPL licence — they are
distributed under Meta's DINOv3 License, which:

- **§1(a)** grants a royalty-free, worldwide right to use, reproduce, distribute, copy, and create
  derivative works of and modify the materials — no revenue threshold, no acceptable-use annex.
- **§1(b)(i)** permits redistributing the materials or derivatives only under that same Agreement,
  with a copy of it supplied.
- **§1(b)(ii)** asks that published research using them acknowledge DINOv3.
- **§1(b)(iii)** and **(v)** require trade-control compliance and forbid ITAR, military, weapons,
  nuclear and espionage use.
- **§8** lets Meta amend the Agreement.

**Running `--roma2` means accepting the DINOv3 License.** `scripts/fetch_roma2_model.py` prints this
notice every time it fetches the model, pointing back at this section for a summary and at
`LICENSE-DINOv3.md` in the published repo itself (`cDcSeacave/openmvs-roma2-onnx` on Hugging Face, or
the GitHub-release mirror — it is not one of the files this script fetches into `--dest`) for the
pinned text. The grant in §1(a) and the restrictions in §1(b)(iii)/(v) both apply in full; neither
narrows the other — read the Agreement itself for the exact terms, this section summarizes it, it
does not replace it.

## Publishing a new bundle (maintainer only)

The export bundle is flat (`roma_base.json`, the two graphs, their `.onnx.data` and `.export.json`
sidecars); both hosts need the nested `<setting>-<precision>/` layout the fetch script expects
(`models/roma2/checksums.txt`'s own paths, Obtaining the model above). Stage that layout once, then
upload the staged copy to both hosts, excluding the `*.reference/` parity fixtures and the
`export*.log` files the export run also leaves behind:

```
mkdir -p publish/base-fp32 && cp <bundle-dir>/roma_base*.{json,onnx,onnx.data} publish/base-fp32/
#   (manifest + 2 graphs + 2 .onnx.data + 2 .export.json -- no *.reference/, no *.log)
hf upload cDcSeacave/openmvs-roma2-onnx publish . --repo-type model           # canonical
gh release upload roma2-model publish/base-fp32/* --repo cdcseacave/openMVS   # mirror, flat
```

GitHub release assets cannot carry a subdirectory, which is why the mirror upload is flat and why the
fetch script's mirror path reconstructs the local flat names itself rather than mapping a nested one
(Obtaining the model, above). The mirror tag is not a placeholder: `fetch_roma2_model.py`'s
`DEFAULT_MIRROR_URL` hardcodes `roma2-model`, so a release published under any other tag breaks the
default mirror URL. A new export's checksums can be pinned into `models/roma2/checksums.txt` as soon
as the export exists — they are hashes of file content, valid before any upload (as already done for
`base`/`fp32` above), with the `<setting>-<precision>/` prefix added by hand (the checksums file's own
header says so). The one thing that must wait for the upload itself is this script's default
`--revision`: it moves onto the commit SHA the upload prints, so a fetch pins the exact bytes the
checksums describe rather than whatever `main` holds later.
