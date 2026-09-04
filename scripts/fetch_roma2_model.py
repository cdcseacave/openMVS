#!/usr/bin/env python3
"""Fetch the published RoMa v2 ONNX model (a DINOv3 descriptor graph + a bidirectional
coarse-match graph) that `--roma2` needs at runtime, without the OpenMVS repository ever
carrying a byte of it.

    fetch_roma2_model.py [--setting base] [--precision fp32]
                          [--dest DIR] [--repo ID] [--revision REV] [--mirror URL]

RULING R147: `--dest` *is* the model directory the C++ side consumes -- the same thing
`--roma2-model DIR` and `OPENMVS_ROMA2_MODEL_PATH` have always meant (`RoMa2Onnx::Load` reads
`modelDir + "roma_" + setting + ".json"` and every graph file directly under it, no further
nesting). So the files this script writes to `--dest` are flat: `roma_<setting>.json`,
`roma_<setting>_descriptor_<precision>.onnx[.data]`,
`roma_<setting>_match_coarse_<precision>.onnx[.data]`, and their `.export.json` sidecars.

`models/roma2/checksums.txt` still names each file under a `<setting>-<precision>/` prefix --
that is the *published* path on both hosts (the Hub repo really does nest by preset, and so does
the reconstructed tree on the mirror side), not the local one. Mapping published path -> local
path is one line (`Path(published).name`), done in exactly one place below.

Multiple presets can coexist in one `--dest`: `roma_<setting>.json` and the graph filenames
already carry `setting` and `precision`, so `--roma2-setting` picks between presets already
fetched into the same directory. The one real collision would be two *precisions of the same
setting* (e.g. base/fp32 and a future base/fp16): both manifests would want the name
`roma_base.json`, so a base/fp16 bundle -- not published today -- will need a distinct manifest
name of its own before it can share a `--dest` with base/fp32.

Two sources:

  * Canonical: the Hugging Face model repo (`--repo`), via `huggingface_hub.snapshot_download`
    when that package is importable -- resumable, deduplicating, revision-pinned. It fetches the
    Hub's own `<setting>-<precision>/` layout into a scratch directory, then this script moves
    the needed files up into the flat `--dest` and discards the scratch copy.
  * Mirror: GitHub release assets of the OpenMVS repo (`--mirror`), streamed with
    `urllib.request` when huggingface_hub is not importable. GitHub flattens release assets (no
    subdirectories), so the mirror's bare file names are already this script's local names --
    nothing to reconstruct on that side.

Every fetched file is verified against `models/roma2/checksums.txt` (read relative to this
script, not the current directory) -- that file is what pins the published artefact's exact
bytes without the repository carrying them, and what makes the mirror as trustworthy as the
canonical host. A file already present with the right digest is left alone and counted as
"cached"; one with the wrong digest is deleted before a fresh attempt. A mismatch after
fetching deletes the file, prints the expected and actual digests, and exits 1.

This is never run at configure time -- see the opt-in `roma2-model` CMake target -- and nothing
about it is exercised automatically in CI: the model is ~1.4 GB and requires network access.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys
import tempfile
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

# --- Publishing targets: the only place these are named -------------------------------------
# Canonical host: the Hugging Face model repo carrying the exported RoMa v2 graphs.
HF_REPO = "cDcSeacave/openmvs-roma2-onnx"
# Mirror: a release of the OpenMVS repo itself, holding the same files as flat release assets
# (GitHub does not let a release asset carry a subdirectory).
GITHUB_MIRROR_REPO = "cdcseacave/openMVS"
GITHUB_MIRROR_TAG = "roma2-model"
DEFAULT_MIRROR_URL = f"https://github.com/{GITHUB_MIRROR_REPO}/releases/download/{GITHUB_MIRROR_TAG}"
# Nothing has been published yet -- this stays "main" until the real commit SHA is pinned here
# after the upload.
DEFAULT_REVISION = "main"

# models/roma2/checksums.txt lives in the OpenMVS repo, next to this script's own parent
# directory -- resolved from __file__ so a fetch works regardless of the caller's cwd.
CHECKSUMS_PATH = Path(__file__).resolve().parent.parent / "models" / "roma2" / "checksums.txt"

DINOV3_NOTICE = (
    "-----------------------------------------------------------------------------\n"
    "The RoMa v2 descriptor graph embeds DINOv3 (Meta) weights, distributed under\n"
    "Meta's DINOv3 License -- NOT MIT/BSD. RoMa v2's own code is MIT (Johan Edstedt).\n"
    "By fetching this model you accept the DINOv3 License terms; see LICENSE-DINOv3.md\n"
    "alongside the fetched files for the pinned text.\n"
    "-----------------------------------------------------------------------------"
)


class ModelFetchError(RuntimeError):
    """Raised when the model cannot be fetched, or a file fails to verify against
    models/roma2/checksums.txt."""


@dataclass
class FetchResult:
    """What one `fetch()` call actually did, per file -- so a caller (or a test) can tell a
    no-op run from a real transfer without relying on filesystem timestamps."""

    dest: Path
    # keyed by the *published* path from checksums.txt (e.g. "base-fp32/roma_base.json"); the
    # file itself lives flat at dest / Path(that path).name -- see _local_path().
    status: Dict[str, str] = field(default_factory=dict)  # published path -> "cached"|"downloaded"

    @property
    def downloaded(self) -> list:
        return sorted(p for p, s in self.status.items() if s == "downloaded")

    @property
    def cached(self) -> list:
        return sorted(p for p, s in self.status.items() if s == "cached")


def _local_path(dest: Path, published_relpath: str) -> Path:
    """Map a checksums.txt entry ("<setting>-<precision>/<name>", the published path on both
    hosts) to where it actually lives on disk: flat, directly under `dest` (RULING R147)."""
    return dest / Path(published_relpath).name


def sha256_file(path: Path) -> str:
    """Stream a file's SHA256 -- the descriptor's .onnx.data can be gigabytes."""
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_digest(path: Path, expected_sha256: str) -> bool:
    """True iff `path` exists and its SHA256 equals `expected_sha256` (case-insensitive).
    False for a missing file -- callers use that to decide whether a fetch is needed."""
    path = Path(path)
    if not path.is_file():
        return False
    return sha256_file(path).lower() == expected_sha256.strip().lower()


def read_checksums(checksums_path: Path) -> Dict[str, str]:
    """Parse `models/roma2/checksums.txt`: lines of `<sha256>  <relative/path>`, '#' comments
    and blank lines ignored. Raises ModelFetchError naming `checksums_path` when it has no
    entries at all -- the placeholder the repo ships until the controller regenerates it from
    the export run and pastes the digests in."""
    checksums_path = Path(checksums_path)
    try:
        lines = checksums_path.read_text().splitlines()
    except OSError as e:
        raise ModelFetchError(f"cannot read checksums file '{checksums_path}': {e}") from e
    entries: Dict[str, str] = {}
    for lineno, raw in enumerate(lines, 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            raise ModelFetchError(f"{checksums_path}:{lineno}: malformed checksum line: {raw!r}")
        digest, relpath = parts
        entries[relpath.strip()] = digest.strip()
    if not entries:
        raise ModelFetchError(
            f"'{checksums_path}' has no checksum entries yet -- the RoMa v2 model has not been "
            "published, or this checkout predates the publish; refusing to fetch with nothing to "
            "verify against"
        )
    return entries


def _entries_for(checksums: Dict[str, str], setting: str, precision: str, checksums_path: Path) -> Dict[str, str]:
    prefix = f"{setting}-{precision}/"
    subset = {relpath: digest for relpath, digest in checksums.items() if relpath.startswith(prefix)}
    if not subset:
        raise ModelFetchError(
            f"no checksum entries for '{prefix}' in '{checksums_path}' -- unknown --setting/"
            "--precision, or that combination has not been published yet"
        )
    return subset


def _fetch_via_huggingface(dest: Path, repo: str, revision: str, setting: str, precision: str,
                            pending: Dict[str, str]) -> bool:
    """Try `huggingface_hub.snapshot_download`. Returns False (without touching the network) if
    huggingface_hub is not importable, so the caller falls back to the mirror; returns True once
    the needed files have been moved into place (verification against checksums.txt happens
    afterward either way)."""
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        return False
    dest.mkdir(parents=True, exist_ok=True)
    prefix = f"{setting}-{precision}"
    with tempfile.TemporaryDirectory() as scratch:
        snapshot_download(
            repo_id=repo,
            revision=revision,
            allow_patterns=[f"{prefix}/*"],
            local_dir=scratch,
        )
        # The Hub repo nests by preset ("<setting>-<precision>/"); the local model directory is
        # flat (RULING R147), so move only the files this call still needs up by basename and
        # let the scratch directory -- shell and all -- disappear with the `with` block.
        nested_dir = Path(scratch) / prefix
        for published_relpath in pending:
            fetched = nested_dir / Path(published_relpath).name
            if fetched.is_file():
                target = _local_path(dest, published_relpath)
                if target.is_file():
                    target.unlink()
                shutil.move(str(fetched), str(target))
    return True


def _fetch_via_mirror(dest: Path, mirror: str, pending: Dict[str, str]) -> None:
    """Stream each pending file from the flat GitHub mirror release, writing through a `.part`
    file so an interrupted fetch never leaves a truncated graph in place. GitHub's flattened
    asset name is already this script's local flat name -- nothing to reconstruct here."""
    dest.mkdir(parents=True, exist_ok=True)
    for published_relpath in pending:
        basename = Path(published_relpath).name
        url = f"{mirror.rstrip('/')}/{basename}"
        target = dest / basename
        part = target.with_name(target.name + ".part")
        try:
            with urllib.request.urlopen(url) as response, open(part, "wb") as out:
                shutil.copyfileobj(response, out)
        except (urllib.error.URLError, OSError) as e:
            if part.is_file():
                part.unlink()
            raise ModelFetchError(f"failed to fetch '{url}': {e}") from e
        part.replace(target)


def fetch(
    dest,
    setting: str = "base",
    precision: str = "fp32",
    repo: str = HF_REPO,
    revision: str = DEFAULT_REVISION,
    mirror: str = DEFAULT_MIRROR_URL,
    checksums_path=CHECKSUMS_PATH,
) -> FetchResult:
    """Fetch (or verify already-present copies of) every file `models/roma2/checksums.txt`
    publishes under `<setting>-<precision>/`, writing them flat into `dest` -- the model
    directory `RoMa2Onnx::Load` / `--roma2-model` / `OPENMVS_ROMA2_MODEL_PATH` reads directly.

    Idempotent: a file already present with the right digest costs one hash and is left
    untouched. A file present with the wrong digest is deleted before a fresh attempt. Raises
    ModelFetchError on any checksum problem, missing source, or a still-wrong digest after
    fetching -- callers never see a partially-verified result silently reported as success.
    """
    dest = Path(dest)
    checksums_path = Path(checksums_path)
    entries = _entries_for(read_checksums(checksums_path), setting, precision, checksums_path)
    result = FetchResult(dest=dest)

    pending: Dict[str, str] = {}
    for published_relpath, digest in entries.items():
        local_path = _local_path(dest, published_relpath)
        if verify_digest(local_path, digest):
            result.status[published_relpath] = "cached"
        else:
            if local_path.is_file():  # wrong digest: never leave a stale/corrupt file in place
                local_path.unlink()
            pending[published_relpath] = digest

    if not pending:
        return result

    if not _fetch_via_huggingface(dest, repo, revision, setting, precision, pending):
        _fetch_via_mirror(dest, mirror, pending)

    for published_relpath, digest in pending.items():
        local_path = _local_path(dest, published_relpath)
        if not verify_digest(local_path, digest):
            actual = sha256_file(local_path) if local_path.is_file() else None
            if local_path.is_file():
                local_path.unlink()
            raise ModelFetchError(
                f"digest mismatch for '{published_relpath}': expected {digest}, got {actual or '<missing>'}"
            )
        result.status[published_relpath] = "downloaded"

    return result


def resolve_dest(dest_arg: Optional[str]) -> Path:
    """`--dest` if given; else $OPENMVS_ROMA2_MODEL_PATH; else ./roma2-model. The CMake target
    always passes --dest explicitly, so this default only ever serves a hand-run."""
    if dest_arg:
        return Path(dest_arg).expanduser()
    env = os.environ.get("OPENMVS_ROMA2_MODEL_PATH")
    if env:
        return Path(env).expanduser()
    return Path("./roma2-model")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fetch the published RoMa v2 ONNX model (DINOv3 descriptor + coarse-match graphs).")
    parser.add_argument("--setting", default="base", help="RoMa v2 preset: turbo|fast|base (default: base)")
    parser.add_argument("--precision", default="fp32", help="exported precision (default: fp32)")
    parser.add_argument("--dest", default=None,
                         help="destination directory -- the model directory itself, flat "
                              "(default: $OPENMVS_ROMA2_MODEL_PATH, else ./roma2-model)")
    parser.add_argument("--repo", default=HF_REPO, help=f"Hugging Face model repo id (default: {HF_REPO})")
    parser.add_argument("--revision", default=DEFAULT_REVISION,
                         help=f"Hugging Face revision/commit to fetch (default: {DEFAULT_REVISION})")
    parser.add_argument("--mirror", default=DEFAULT_MIRROR_URL,
                         help="mirror base URL, used only when huggingface_hub is not importable "
                              f"(default: {DEFAULT_MIRROR_URL})")
    return parser


def main(argv=None) -> int:
    args = build_argument_parser().parse_args(argv)
    dest = resolve_dest(args.dest)
    # RULING R145: --dest has no way to see the CMake install prefix (a Python script can't read
    # a compile definition), so a hand-run relying on the default must never be surprised by where
    # things landed.
    print(f"destination: {dest.resolve()}")

    try:
        result = fetch(dest, setting=args.setting, precision=args.precision,
                        repo=args.repo, revision=args.revision, mirror=args.mirror)
    except ModelFetchError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    if result.downloaded:
        print(f"fetched {len(result.downloaded)} file(s), {len(result.cached)} already present and verified")
    else:
        print("already present: every file verified against checksums.txt, nothing downloaded")
    print()
    print(DINOV3_NOTICE)
    print()
    # RULING R147: --dest *is* the model directory -- RoMa2Onnx::Load reads its files directly,
    # with no "<setting>-<precision>" segment to append.
    print(f"model ready at: {dest.resolve()}")
    print(f"set OPENMVS_ROMA2_MODEL_PATH={dest.resolve()} (or pass --roma2-model {dest.resolve()} "
          "to CreateStructure) so --roma2 finds it. If --dest was the CMake install prefix's own "
          "share/openMVS/roma2, ROMA2Config::ResolveModelPath()'s install-prefix fallback already "
          "finds it with no environment variable needed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
