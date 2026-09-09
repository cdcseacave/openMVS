#!/usr/bin/env python3
"""Fetch the published RoMa v2 ONNX model (a DINOv3 descriptor graph + a bidirectional
coarse-match graph) that `--roma2` needs at runtime, without the OpenMVS repository ever
carrying a byte of it.

    fetch_roma2_model.py [--setting base] [--precision fp32]
                          [--dest DIR] [--repo ID] [--revision REV] [--mirror URL]

`--dest` *is* the model directory the C++ side consumes -- the same thing
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

Two sources, the Hub tried first:

  * Canonical: the Hugging Face model repo (`--repo`), via `huggingface_hub.snapshot_download`
    when that package is importable -- resumable, deduplicating, revision-pinned. It fetches the
    Hub's own `<setting>-<precision>/` layout into a staging directory nested under `--dest`
    itself (not a temporary one thrown away on every call, which would discard the resume state
    an interrupted multi-gigabyte download depends on), then this script moves the needed files up
    into the flat `--dest` and, once nothing is left to move, removes the now-empty staging
    directory.
  * Mirror: GitHub release assets of the OpenMVS repo (`--mirror`), streamed with
    `urllib.request`. Used whenever the Hub does not deliver the files: huggingface_hub is not
    importable, or a Hub fetch was attempted and failed (offline, blocked, an outage, an unknown
    repo/revision) -- installing huggingface_hub never makes this script less able to reach the
    model, only ever adds a first, canonical attempt before the same mirror it would otherwise go
    to directly. GitHub flattens release assets (no subdirectories), so the mirror's bare file
    names are already this script's local names -- nothing to reconstruct on that side.

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
# The Hub commit that published the bundle models/roma2/checksums.txt pins; a re-export moves
# both together.
DEFAULT_REVISION = "ea2b04648ba0fb1c33d1dac2bdc65bfb17b0ef5f"

# models/roma2/checksums.txt lives in the OpenMVS repo, next to this script's own parent
# directory -- resolved from __file__ so a fetch works regardless of the caller's cwd.
CHECKSUMS_PATH = Path(__file__).resolve().parent.parent / "models" / "roma2" / "checksums.txt"

DINOV3_NOTICE = (
    "-----------------------------------------------------------------------------\n"
    "The RoMa v2 descriptor graph embeds DINOv3 (Meta) weights, distributed under\n"
    "Meta's DINOv3 License -- NOT MIT/BSD. RoMa v2's own code is MIT (Johan Edstedt).\n"
    "By fetching this model you accept the DINOv3 License terms; see docs/RoMa2Model.md\n"
    "(Licence section) for a summary, and LICENSE-DINOv3.md in the published repo itself\n"
    f"({HF_REPO} on Hugging Face, or the GitHub-release mirror) for the pinned text --\n"
    "it is not one of the files this script fetches into --dest.\n"
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
    hosts) to where it actually lives on disk: flat, directly under `dest`."""
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
    huggingface_hub is not importable, so the caller falls back to the mirror. Raises
    ModelFetchError for any other failure (network, auth, an unknown repo/revision, a filesystem
    error moving the result into place) -- the caller falls back to the mirror for that too, it
    just needs to know the two cases apart to say which one happened. Returns True once the
    needed files have been moved into place (verification against checksums.txt happens
    afterward either way). The caller has already made sure `dest` exists and is writable."""
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        return False
    # Everything below talks to the network (snapshot_download) or can hit a filesystem error
    # (shutil.move across a mountpoint, a full disk): neither is a correctness problem -- a
    # partial/failed move here can never pass digest verification, on this run or the next -- but
    # left unguarded it would surface as a raw traceback instead of the clean
    # ModelFetchError/exit(1) every other failure path gives. Wrap it so a network hiccup, the
    # most likely first failure, reads like a message.
    try:
        prefix = f"{setting}-{precision}"
        # `local_dir=dest`, not a fresh temporary directory: huggingface_hub's local_dir mode
        # keeps its own resume/etag metadata under `<local_dir>/.cache/huggingface/`, reused only
        # when the same local_dir is handed back on a later call. A directory thrown away at the
        # end of every call -- even one ended by an exception -- can never reuse that state, so a
        # download interrupted partway through a multi-gigabyte file used to restart from zero
        # every time. `dest` is stable across runs (it is `--roma2-model`/
        # `$OPENMVS_ROMA2_MODEL_PATH` itself), so it is what makes resuming possible.
        nested_dir = dest / prefix
        if nested_dir.is_dir() and any(nested_dir.iterdir()):
            print(f"resuming a Hugging Face download already in progress under '{nested_dir}'",
                  file=sys.stderr)
        snapshot_download(
            repo_id=repo,
            revision=revision,
            allow_patterns=[f"{prefix}/*"],
            local_dir=dest,
        )
        # The Hub repo nests by preset ("<setting>-<precision>/"); the local model directory is
        # flat, so move only the files this call still needs up by basename.
        for published_relpath in pending:
            fetched = nested_dir / Path(published_relpath).name
            if fetched.is_file():
                target = _local_path(dest, published_relpath)
                if target.is_file():
                    target.unlink()
                shutil.move(str(fetched), str(target))
        # A successful run leaves no staging shell behind: every file the nested directory held
        # had a name in checksums.txt and was moved up, so it should now be empty. It is left in
        # place, unremoved, only if something is still sitting in it (e.g. a file the published
        # repo carries that checksums.txt never named) -- this never forces a directory away, since
        # doing so could delete a file this call did not itself decide was safe to move. The
        # hidden `.cache/huggingface/` metadata directory is left alone here regardless: whether it
        # still serves a purpose depends on whether the digests the caller checks afterward
        # actually verify, which this function cannot see -- see fetch()'s own cleanup below.
        try:
            nested_dir.rmdir()
        except OSError:
            pass
    except Exception as e:
        raise ModelFetchError(f"failed to fetch '{repo}' (revision '{revision}') from Hugging Face: {e}") from e
    return True


def _fetch_via_mirror(dest: Path, mirror: str, pending: Dict[str, str]) -> None:
    """Stream each pending file from the flat GitHub mirror release, writing through a `.part`
    file so an interrupted fetch never leaves a truncated graph in place. GitHub's flattened
    asset name is already this script's local flat name -- nothing to reconstruct here. The
    caller has already made sure `dest` exists and is writable."""
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
        except BaseException:
            # any other interruption (e.g. KeyboardInterrupt) must not leave a `.part` file
            # behind either -- see also the sweep in fetch() that clears litter from a run that
            # was interrupted even more abruptly than an exception allows for
            if part.is_file():
                part.unlink()
            raise
        part.replace(target)


def _cleanup_huggingface_resume_metadata(dest: Path, prefix: str) -> None:
    """Remove the Hugging Face local_dir resume/dedup bookkeeping for `prefix` under `dest`.
    Meant to be called only once every file `prefix` names has been fetched AND independently
    verified against checksums.txt: at that point there is nothing left to resume, and a later
    re-run of this preset would skip straight to the cached path on digests anyway, so a hidden
    cache left inside the user's model directory no longer serves a purpose. An interrupted fetch
    must never reach this call -- that bookkeeping is what makes the *next* run able to resume.

    Scoped to `prefix` alone: a different --setting/--precision sharing this --dest may still be
    genuinely mid-fetch, so only the shared `.cache/huggingface/` tree itself is removed, and only
    once nothing else is using it. Best-effort and silent on failure -- this on-disk layout is
    huggingface_hub's own private bookkeeping, not a public API, and a leftover file changes
    nothing about correctness."""
    huggingface_dir = dest / ".cache" / "huggingface"
    download_dir = huggingface_dir / "download"
    try:
        prefix_download_dir = download_dir / prefix
        if prefix_download_dir.is_dir():
            shutil.rmtree(prefix_download_dir)
        if download_dir.is_dir() and not any(download_dir.iterdir()):
            shutil.rmtree(huggingface_dir)
    except OSError:
        pass


def _ensure_dest_writable(dest: Path) -> None:
    """Raise a ModelFetchError naming `dest` if it cannot be created or written to. Meant to be
    called once, before either fetch branch runs: `roma2-model`'s CMake target defaults `--dest`
    to the CMake install prefix, which an unprivileged user cannot write, and that must be
    reported plainly rather than escaping as a raw traceback or being blamed on Hugging Face."""
    try:
        dest.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        raise ModelFetchError(
            f"cannot write to '{dest}': {e} -- re-run with sudo, or pass --dest to a directory "
            "you own (then set OPENMVS_ROMA2_MODEL_PATH to it)"
        ) from e
    if not os.access(dest, os.W_OK):
        raise ModelFetchError(
            f"cannot write to '{dest}': permission denied -- re-run with sudo, or pass --dest to "
            "a directory you own (then set OPENMVS_ROMA2_MODEL_PATH to it)"
        )


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
    ModelFetchError on any checksum problem, missing source, a destination that cannot be
    written to, or a still-wrong digest after fetching -- callers never see a partially-verified
    result silently reported as success. The Hub is tried first; a failure there (not merely
    huggingface_hub being absent) falls back to the mirror instead of giving up, since the two
    are pinned to the same digests and are meant to be interchangeable.
    """
    dest = Path(dest)
    checksums_path = Path(checksums_path)
    entries = _entries_for(read_checksums(checksums_path), setting, precision, checksums_path)
    result = FetchResult(dest=dest)

    # Litter from an earlier run interrupted more abruptly than `_fetch_via_mirror`'s own
    # exception handling can catch (a killed process, a lost connection): nothing else ever
    # revisits a file once it stops being pending, so a `.part` next to an already-cached,
    # already-verified file would otherwise sit there forever, looking like a broken model.
    for published_relpath in entries:
        local_path = _local_path(dest, published_relpath)
        part = local_path.with_name(local_path.name + ".part")
        if part.is_file():
            part.unlink()

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

    # Checked before any network attempt, and before either fetch branch creates `dest` itself,
    # so a destination the caller cannot write to (e.g. the roma2-model CMake target's default
    # install prefix) is always reported by name instead of surfacing as a raw traceback (the
    # mirror branch used to create `dest` outside any try) or being blamed on Hugging Face (the
    # Hub branch used to create `dest` as the first statement inside its own try).
    _ensure_dest_writable(dest)

    hub_error: Optional[ModelFetchError] = None
    try:
        fetched_via_hub = _fetch_via_huggingface(dest, repo, revision, setting, precision, pending)
    except ModelFetchError as e:
        fetched_via_hub = False
        hub_error = e

    if not fetched_via_hub:
        if hub_error is not None:
            print(f"warning: {hub_error}", file=sys.stderr)
            print("falling back to the GitHub mirror", file=sys.stderr)
        else:
            print("huggingface_hub not installed; using the GitHub mirror", file=sys.stderr)
        try:
            _fetch_via_mirror(dest, mirror, pending)
        except ModelFetchError as mirror_error:
            if hub_error is not None:
                raise ModelFetchError(
                    f"Hugging Face fetch failed ({hub_error}); mirror fetch also failed: {mirror_error}"
                ) from mirror_error
            raise

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

    # Every file this run needed has now been independently verified against checksums.txt:
    # nothing is left to resume, and a later re-run would skip straight to the cached path on
    # digests anyway, so any Hugging Face resume/dedup bookkeeping this run left under `dest` no
    # longer serves a purpose. A run that raised above never reaches this line, so an interrupted
    # or unverified fetch keeps whatever bookkeeping the *next* run needs to resume from it.
    _cleanup_huggingface_resume_metadata(dest, f"{setting}-{precision}")

    return result


def resolve_dest(dest_arg: Optional[str]) -> Path:
    """`--dest` if given; else $OPENMVS_ROMA2_MODEL_PATH; else `~/.cache/openMVS/roma2`. The
    CMake target always passes --dest explicitly, so this default only ever serves a hand-run --
    and it must not be a path under the current directory: the obvious hand-run has a checkout
    for its cwd, which does not gitignore it, and the DINOv3 weights this fetches must never enter
    the OpenMVS repository (see docs/RoMa2Model.md's Licence section)."""
    if dest_arg:
        return Path(dest_arg).expanduser()
    env = os.environ.get("OPENMVS_ROMA2_MODEL_PATH")
    if env:
        return Path(env).expanduser()
    return Path("~/.cache/openMVS/roma2").expanduser()


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fetch the published RoMa v2 ONNX model (DINOv3 descriptor + coarse-match graphs).")
    parser.add_argument("--setting", default="base", help="RoMa v2 preset: turbo|fast|base (default: base)")
    parser.add_argument("--precision", default="fp32", help="exported precision (default: fp32)")
    parser.add_argument("--dest", default=None,
                         help="destination directory -- the model directory itself, flat "
                              "(default: $OPENMVS_ROMA2_MODEL_PATH, else ~/.cache/openMVS/roma2)")
    parser.add_argument("--repo", default=HF_REPO, help=f"Hugging Face model repo id (default: {HF_REPO})")
    parser.add_argument("--revision", default=DEFAULT_REVISION,
                         help=f"Hugging Face revision/commit to fetch (default: {DEFAULT_REVISION})")
    parser.add_argument("--mirror", default=DEFAULT_MIRROR_URL,
                         help="mirror base URL, used when huggingface_hub is not importable or "
                              f"the Hugging Face fetch fails (default: {DEFAULT_MIRROR_URL})")
    return parser


def main(argv=None) -> int:
    args = build_argument_parser().parse_args(argv)
    dest = resolve_dest(args.dest)
    # --dest has no way to see the CMake install prefix (a Python script can't read
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
    # --dest *is* the model directory -- RoMa2Onnx::Load reads its files directly,
    # with no "<setting>-<precision>" segment to append.
    print(f"model ready at: {dest.resolve()}")
    print(f"set OPENMVS_ROMA2_MODEL_PATH={dest.resolve()} (or pass --roma2-model {dest.resolve()} "
          "to CreateStructure) so --roma2 finds it. If --dest was the CMake install prefix's own "
          "share/openMVS/roma2, ROMA2Config::ResolveModelPath()'s install-prefix fallback already "
          "finds it with no environment variable needed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
