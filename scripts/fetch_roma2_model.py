#!/usr/bin/env python3
"""Fetch the published RoMa v2 ONNX model (a DINOv3 descriptor graph + a bidirectional
coarse-match graph) that `--roma2` needs at runtime, without the OpenMVS repository ever
carrying a byte of it.

    fetch_roma2_model.py [--setting base] [--precision fp32]
                          [--dest DIR] [--repo ID] [--revision REV] [--mirror URL]

Two sources, always producing the same on-disk tree under `<dest>/<setting>-<precision>/`:

  * Canonical: the Hugging Face model repo (`--repo`), via `huggingface_hub.snapshot_download`
    when that package is importable -- resumable, deduplicating, revision-pinned.
  * Mirror: GitHub release assets of the OpenMVS repo (`--mirror`), streamed with
    `urllib.request` when huggingface_hub is not importable. GitHub flattens release assets (no
    subdirectories), so files are uploaded there under their bare name and this script
    reconstructs the "<setting>-<precision>/" layout locally on download so both sources leave
    an identical tree.

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
    status: Dict[str, str] = field(default_factory=dict)  # relative path -> "cached"|"downloaded"

    @property
    def downloaded(self) -> list:
        return sorted(p for p, s in self.status.items() if s == "downloaded")

    @property
    def cached(self) -> list:
        return sorted(p for p, s in self.status.items() if s == "cached")


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


def _entries_for(checksums: Dict[str, str], setting: str, precision: str) -> Dict[str, str]:
    prefix = f"{setting}-{precision}/"
    subset = {relpath: digest for relpath, digest in checksums.items() if relpath.startswith(prefix)}
    if not subset:
        raise ModelFetchError(
            f"no checksum entries for '{prefix}' in '{CHECKSUMS_PATH}' -- unknown --setting/"
            "--precision, or that combination has not been published yet"
        )
    return subset


def _fetch_via_huggingface(dest: Path, repo: str, revision: str, setting: str, precision: str) -> bool:
    """Try `huggingface_hub.snapshot_download`. Returns False (without touching the network) if
    huggingface_hub is not importable, so the caller falls back to the mirror; returns True once
    the bulk download has run (verification against checksums.txt happens afterward either way)."""
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        return False
    dest.mkdir(parents=True, exist_ok=True)
    snapshot_download(
        repo_id=repo,
        revision=revision,
        allow_patterns=[f"{setting}-{precision}/*"],
        local_dir=str(dest),
    )
    return True


def _fetch_via_mirror(dest: Path, mirror: str, pending: Dict[str, str]) -> None:
    """Stream each pending file from the flat GitHub mirror, writing through a `.part` file so
    an interrupted fetch never leaves a truncated graph in place, and reconstructing the
    "<setting>-<precision>/" layout locally (GitHub itself only stores the bare filename)."""
    for relpath in pending:
        basename = relpath.rsplit("/", 1)[-1]
        url = f"{mirror.rstrip('/')}/{basename}"
        target = dest / relpath
        target.parent.mkdir(parents=True, exist_ok=True)
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
    """Fetch (or verify already-present copies of) every file
    `models/roma2/checksums.txt` lists under `<setting>-<precision>/`, into `dest`.

    Idempotent: a file already present with the right digest costs one hash and is left
    untouched. A file present with the wrong digest is deleted before a fresh attempt. Raises
    ModelFetchError on any checksum problem, missing source, or a still-wrong digest after
    fetching -- callers never see a partially-verified result silently reported as success.
    """
    dest = Path(dest)
    entries = _entries_for(read_checksums(checksums_path), setting, precision)
    result = FetchResult(dest=dest)

    pending: Dict[str, str] = {}
    for relpath, digest in entries.items():
        path = dest / relpath
        if verify_digest(path, digest):
            result.status[relpath] = "cached"
        else:
            if path.is_file():  # wrong digest: never leave a stale/corrupt file in place
                path.unlink()
            pending[relpath] = digest

    if not pending:
        return result

    if not _fetch_via_huggingface(dest, repo, revision, setting, precision):
        _fetch_via_mirror(dest, mirror, pending)

    for relpath, digest in pending.items():
        path = dest / relpath
        if not verify_digest(path, digest):
            actual = sha256_file(path) if path.is_file() else None
            if path.is_file():
                path.unlink()
            raise ModelFetchError(
                f"digest mismatch for '{relpath}': expected {digest}, got {actual or '<missing>'}"
            )
        result.status[relpath] = "downloaded"

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
                         help="destination directory (default: $OPENMVS_ROMA2_MODEL_PATH, else ./roma2-model)")
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

    model_dir = dest / f"{args.setting}-{args.precision}"
    if result.downloaded:
        print(f"fetched {len(result.downloaded)} file(s), {len(result.cached)} already present and verified")
    else:
        print("already present: every file verified against checksums.txt, nothing downloaded")
    print()
    print(DINOV3_NOTICE)
    print()
    print(f"model ready at: {model_dir}")
    print(f"set OPENMVS_ROMA2_MODEL_PATH={model_dir} (or pass --roma2-model {model_dir} to "
          "CreateStructure) so --roma2 finds it. The install-prefix fallback in "
          "ROMA2Config::ResolveModelPath() only looks directly under the install directory "
          "itself (no <setting>-<precision> segment), so unless OPENMVS_ROMA2_MODEL_PATH is set "
          "you still need --roma2-model to point at the path above.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
