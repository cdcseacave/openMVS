#!/usr/bin/env python3
"""Unit tests for scripts/fetch_roma2_model.py.

Runs with plain `python3 -m unittest` and touches no network: huggingface_hub is never imported
here (it need not even be installed -- the whole point of the ImportError fallback is that it
works without it), and the mirror is a local `file://` directory standing in for a GitHub
release's flat asset list.
"""
import hashlib
import sys
import tempfile
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


if __name__ == "__main__":
    unittest.main()
