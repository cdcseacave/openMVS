#!/usr/bin/env python3
"""Unit tests for scripts/fetch_roma2_model.py.

Runs with plain `python3 -m unittest` and touches no network: huggingface_hub is never imported
here (it need not even be installed -- the whole point of the ImportError fallback is that it
works without it), and the mirror is a local `file://` directory standing in for a GitHub
release's flat asset list.
"""
import contextlib
import hashlib
import io
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path

# fetch_roma2_model.py lives at scripts/fetch_roma2_model.py; this test file sits three levels
# below (scripts/python/tests/), so add scripts/ to sys.path rather than relying on cwd.
_SCRIPTS_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SCRIPTS_DIR))

import fetch_roma2_model as fm  # noqa: E402


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


class VerifyDigestTests(unittest.TestCase):
    def test_matching_digest_returns_true(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "f.bin"
            path.write_bytes(b"hello world")
            self.assertTrue(fm.verify_digest(path, _sha256(b"hello world")))

    def test_mismatched_digest_returns_false(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "f.bin"
            path.write_bytes(b"hello world")
            self.assertFalse(fm.verify_digest(path, _sha256(b"goodbye world")))

    def test_missing_file_returns_false(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertFalse(fm.verify_digest(Path(tmp) / "missing.bin", _sha256(b"x")))


class FetchTests(unittest.TestCase):
    """Exercises `fetch()` entirely over a local file:// mirror. huggingface_hub is not
    installed in this environment, so the ImportError fallback to the mirror is the only path
    that ever runs -- exactly what a no-network test needs.

    RULING R147: `--dest` is the flat model directory RoMa2Onnx::Load reads directly
    (`modelDir + "roma_<setting>.json"`, no "<setting>-<precision>" segment). checksums.txt
    still names the *published* path under that prefix -- these fixtures use the real manifest
    basename (roma_base.json) so a regression back to the nested layout would be caught by
    test_manifest_lands_flat_directly_under_dest below, not just inferred from a generic name.
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        root = Path(self._tmp.name)
        self.dest = root / "dest"
        self.mirror_dir = root / "mirror"
        self.mirror_dir.mkdir()
        self.mirror_url = self.mirror_dir.resolve().as_uri()

        self.setting = "base"
        self.precision = "fp32"
        self.basename = "roma_base.json"  # the manifest RoMa2Onnx::Load looks for, verbatim
        self.published_relpath = f"{self.setting}-{self.precision}/{self.basename}"
        self.content = b'{"setting": "base", "fixture": true}' + b" " * 200
        self.digest = _sha256(self.content)

        # The mirror is flat -- exactly what a GitHub release gives back.
        _write(self.mirror_dir / self.basename, self.content)

        self.checksums_path = root / "checksums.txt"
        self.checksums_path.write_text(
            f"# test fixture\n{self.digest}  {self.published_relpath}\n")

    def _fetch(self, **overrides):
        params = dict(
            setting=self.setting,
            precision=self.precision,
            mirror=self.mirror_url,
            checksums_path=self.checksums_path,
        )
        params.update(overrides)
        return fm.fetch(self.dest, **params)

    def _local_target(self) -> Path:
        return self.dest / self.basename

    def _install_fake_huggingface_hub(self, snapshot_download) -> None:
        """Stand in for huggingface_hub (not installed in this environment, and this suite must
        not install it) so the Hugging Face branch's own logic -- not just the ImportError
        fallback to the mirror -- gets exercised too, still with no real network access."""
        fake_module = types.ModuleType("huggingface_hub")
        fake_module.snapshot_download = snapshot_download
        sys.modules["huggingface_hub"] = fake_module
        self.addCleanup(sys.modules.pop, "huggingface_hub", None)

    def test_downloads_missing_file_from_mirror_flat(self):
        result = self._fetch()
        target = self._local_target()
        self.assertTrue(target.is_file())
        self.assertEqual(target.read_bytes(), self.content)
        self.assertFalse(target.with_name(target.name + ".part").exists())
        self.assertEqual(result.downloaded, [self.published_relpath])
        self.assertEqual(result.cached, [])

    def test_manifest_lands_flat_directly_under_dest(self):
        """The one assertion that would have caught RULING R147's bug: this is exactly the path
        ROMA2Config::ResolveModelPath() hands to RoMa2Onnx::Load, which reads
        modelDir + "roma_" + setting + ".json" with no further nesting."""
        self._fetch()
        self.assertTrue((self.dest / f"roma_{self.setting}.json").is_file())
        # and definitely not tucked under a reconstructed "<setting>-<precision>/" shell
        self.assertFalse((self.dest / f"{self.setting}-{self.precision}").exists())

    def test_huggingface_branch_moves_only_files_named_in_checksums(self):
        """Fix round 2, finding #3: a file the published source carries but
        models/roma2/checksums.txt never named must not be silently accepted into --dest.

        Chosen on the Hugging Face branch, not the mirror one: the HF branch lists a whole
        "<setting>-<precision>/" folder via allow_patterns and then must filter what it moves up
        by `pending` -- that filter is exactly the kind of guarantee a future change (say, moving
        the whole nested folder instead of iterating `pending`) could quietly break. The mirror
        branch never lists a folder at all -- it only ever requests exact basenames drawn from
        `pending` -- so a stray file sitting in the mirror directory could never be fetched by
        construction; a test for it there would pass by tautology, not by exercising a filter.
        huggingface_hub is not installed here (nor should this suite install it), so the branch's
        own move-up logic is exercised through a fake module standing in for it -- still no
        network.
        """
        stray_name = "roma_base_a_file_checksums_txt_never_named.bin"

        def fake_snapshot_download(repo_id, revision, allow_patterns, local_dir):
            prefix = allow_patterns[0].split("/", 1)[0]
            nested = Path(local_dir) / prefix
            _write(nested / self.basename, self.content)
            _write(nested / stray_name, b"present upstream, never pinned in checksums.txt")

        self._install_fake_huggingface_hub(fake_snapshot_download)
        # Remove the mirror's copy too: if the Hugging Face branch were skipped (e.g. the
        # ImportError check broke and never picked up the fake module above), a fall-through to
        # the mirror must fail loudly instead of masking that with an unrelated success.
        (self.mirror_dir / self.basename).unlink()

        result = self._fetch()

        self.assertTrue((self.dest / self.basename).is_file())
        self.assertFalse((self.dest / stray_name).exists())
        self.assertEqual(result.downloaded, [self.published_relpath])

    def test_huggingface_branch_wraps_failures_as_model_fetch_error(self):
        """Fix round 2, finding #1: a network hiccup (the most likely first failure a user
        hits) -- or any other failure inside the Hugging Face branch, e.g. a cross-filesystem
        shutil.move error -- must read as this tool's own clean error, not a raw traceback."""
        def fake_snapshot_download(repo_id, revision, allow_patterns, local_dir):
            raise ConnectionError("simulated network failure")

        self._install_fake_huggingface_hub(fake_snapshot_download)
        # If the Hugging Face branch were skipped, a fall-through to the mirror could still
        # succeed and hide the missing wrap -- remove its copy so that path fails loudly too.
        (self.mirror_dir / self.basename).unlink()

        with self.assertRaises(fm.ModelFetchError) as ctx:
            self._fetch()

        self.assertIn("simulated network failure", str(ctx.exception))

    def test_falls_back_to_mirror_when_huggingface_fetch_fails(self):
        """A Hugging Face attempt that raises (not merely ImportError) must still fall back to
        the mirror -- otherwise having huggingface_hub installed makes this script strictly less
        able to reach the model than not having it installed at all."""
        def fake_snapshot_download(repo_id, revision, allow_patterns, local_dir):
            raise ConnectionError("simulated network failure")

        self._install_fake_huggingface_hub(fake_snapshot_download)
        # leave the mirror's copy in place this time -- the fallback must actually use it

        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            result = self._fetch()

        target = self._local_target()
        self.assertTrue(target.is_file())
        self.assertEqual(target.read_bytes(), self.content)
        self.assertEqual(result.downloaded, [self.published_relpath])
        # a Hugging Face failure and a merely-absent huggingface_hub must read differently, so a
        # user can tell "the Hub was not available" from "the Hub said no"
        self.assertIn("simulated network failure", stderr.getvalue())

    def test_huggingface_and_mirror_both_failing_names_both_reasons(self):
        def fake_snapshot_download(repo_id, revision, allow_patterns, local_dir):
            raise ConnectionError("simulated network failure")

        self._install_fake_huggingface_hub(fake_snapshot_download)
        (self.mirror_dir / self.basename).unlink()

        with self.assertRaises(fm.ModelFetchError) as ctx:
            self._fetch()

        message = str(ctx.exception)
        self.assertIn("simulated network failure", message)
        self.assertIn("mirror", message.lower())

    def test_missing_huggingface_hub_falls_back_to_mirror_without_reporting_a_hub_failure(self):
        """The absence of huggingface_hub is not a failed Hub attempt -- nothing claiming the Hub
        "failed" should appear, only that the mirror is being used, keeping the two cases the user
        can hit visibly distinct."""
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self._fetch()

        self.assertNotIn("failed", stderr.getvalue().lower())

    def test_stale_part_file_next_to_an_already_cached_file_is_removed(self):
        """A `.part` left by an earlier interrupted fetch of a file that is now already present
        and verified is never revisited by the pending-file loop, so nothing else would ever
        clean it up."""
        target = self._local_target()
        _write(target, self.content)
        part = target.with_name(target.name + ".part")
        _write(part, b"leftover from an interrupted run")
        (self.mirror_dir / self.basename).unlink()  # nothing should need to be fetched

        result = self._fetch()

        self.assertFalse(part.exists())
        self.assertEqual(result.cached, [self.published_relpath])
        self.assertEqual(result.downloaded, [])

    def test_file_already_present_with_right_digest_is_left_untouched_and_cached(self):
        target = self._local_target()
        _write(target, self.content)
        # Remove the mirror's copy so a redownload attempt -- if one wrongly happened -- would
        # fail loudly instead of silently re-writing identical bytes.
        (self.mirror_dir / self.basename).unlink()

        result = self._fetch()

        # RULING R139: assert on the actually-transferred-file counter, not mtime (whose
        # one-second granularity on many filesystems would make a fast run flaky).
        self.assertEqual(result.downloaded, [])
        self.assertEqual(result.cached, [self.published_relpath])
        self.assertEqual(target.read_bytes(), self.content)

    def test_file_present_with_wrong_digest_is_deleted_and_fetch_fails(self):
        target = self._local_target()
        _write(target, b"corrupted, not the published bytes")
        # The mirror has no fix available either, so the re-fetch attempt itself fails too --
        # the file must not survive the wrong-digest check regardless.
        (self.mirror_dir / self.basename).unlink()

        with self.assertRaises(fm.ModelFetchError):
            self._fetch()

        self.assertFalse(target.is_file())

    def test_empty_checksums_file_fails_naming_the_file(self):
        empty = Path(self._tmp.name) / "empty_checksums.txt"
        empty.write_text("# header only, no entries yet\n")

        with self.assertRaises(fm.ModelFetchError) as ctx:
            self._fetch(checksums_path=empty)

        self.assertIn(str(empty), str(ctx.exception))


class DestWritabilityTests(unittest.TestCase):
    """The `roma2-model` CMake target's default destination (the CMake install prefix) is not
    writable by an unprivileged user. That must be reported by name, before any network attempt is
    made -- not as a raw PermissionError traceback (the old mirror-branch behaviour) and not
    blamed on Hugging Face (the old Hub-branch behaviour, since its dest.mkdir() ran inside its own
    try)."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        root = Path(self._tmp.name)

        self.setting = "base"
        self.precision = "fp32"
        self.basename = "roma_base.json"
        self.published_relpath = f"{self.setting}-{self.precision}/{self.basename}"
        self.content = b'{"setting": "base", "fixture": true}' + b" " * 200
        self.digest = _sha256(self.content)

        self.checksums_path = root / "checksums.txt"
        self.checksums_path.write_text(f"# test fixture\n{self.digest}  {self.published_relpath}\n")

        self.mirror_dir = root / "mirror"
        self.mirror_dir.mkdir()
        _write(self.mirror_dir / self.basename, self.content)
        self.mirror_url = self.mirror_dir.resolve().as_uri()

        readonly_parent = root / "readonly_parent"
        readonly_parent.mkdir()
        readonly_parent.chmod(0o500)  # read + execute, no write: cannot create anything inside
        self.addCleanup(readonly_parent.chmod, 0o700)  # let TemporaryDirectory clean up after us
        self.dest = readonly_parent / "roma2"

    def _fetch(self, **overrides):
        params = dict(setting=self.setting, precision=self.precision, mirror=self.mirror_url,
                      checksums_path=self.checksums_path)
        params.update(overrides)
        return fm.fetch(self.dest, **params)

    def test_reports_the_destination_by_name_before_touching_the_network(self):
        with self.assertRaises(fm.ModelFetchError) as ctx:
            self._fetch()

        message = str(ctx.exception)
        self.assertIn(str(self.dest), message)
        self.assertFalse(self.dest.exists())

    def test_reported_even_with_huggingface_hub_installed(self):
        """The writability check must run before `_fetch_via_huggingface` is even tried, so a
        non-writable destination is never misreported as a Hugging Face failure."""
        def fake_snapshot_download(repo_id, revision, allow_patterns, local_dir):
            self.fail("must not touch the network before the destination is known writable")

        fake_module = types.ModuleType("huggingface_hub")
        fake_module.snapshot_download = fake_snapshot_download
        sys.modules["huggingface_hub"] = fake_module
        self.addCleanup(sys.modules.pop, "huggingface_hub", None)

        with self.assertRaises(fm.ModelFetchError) as ctx:
            self._fetch()

        self.assertIn(str(self.dest), str(ctx.exception))


class ResolveDestTests(unittest.TestCase):
    """The no-`--dest`, no-environment-variable default must never be a path under the current
    directory -- the obvious hand-run's cwd is a repository checkout that does not gitignore it,
    and the DINOv3 weights it would fetch must never enter the OpenMVS repository."""

    def test_default_is_the_user_cache_directory_not_the_cwd(self):
        had_env = "OPENMVS_ROMA2_MODEL_PATH" in os.environ
        env_backup = os.environ.pop("OPENMVS_ROMA2_MODEL_PATH", None)
        try:
            dest = fm.resolve_dest(None)
        finally:
            if had_env:
                os.environ["OPENMVS_ROMA2_MODEL_PATH"] = env_backup

        self.assertEqual(dest, Path("~/.cache/openMVS/roma2").expanduser())
        self.assertNotEqual(dest, Path("./roma2-model"))
        self.assertNotIn(Path.cwd(), dest.parents)


if __name__ == "__main__":
    unittest.main()
