"""OpenMVS Python bindings.

This package wraps the native ``pyOpenMVS`` extension module and resolves the
DLL search path so the ``import openmvs`` works regardless of how the user
launches Python on Windows. Specifically it adds:

* the directory containing ``pyOpenMVS.dll/.pyd`` itself (so co-located
  vcpkg DLLs like ``boost_python*``, ``opencv_*``, ``ceres``, ``glog``, ...
  resolve)
* the CUDA ``bin/x64`` directory containing ``cublas64_*.dll``,
  ``cusolver64_*.dll``, ``cusparse64_*.dll`` (transitively required by Ceres)

On non-Windows platforms this resolution is unnecessary and the file simply
re-exports the extension module's public API.

Typical usage::

    import openmvs as ovs
    sfm = ovs.SfMScene(max_threads=8)
    sfm.reconstruct("path/to/images", ovs.ReconstructionConfig())
    sfm.export_to_mvs("scene.mvs")

    mvs = ovs.Scene(max_threads=8)
    mvs.load("scene.mvs")
    mvs.dense_reconstruction()
"""
from __future__ import annotations

import os
import sys
from pathlib import Path


def _add_dll_dir(path: str) -> None:
    """Best-effort `os.add_dll_directory(path)` (Windows only, Python >= 3.8)."""
    if not os.path.isdir(path):
        return
    add = getattr(os, "add_dll_directory", None)
    if add is not None:
        try:
            add(path)
        except (OSError, FileNotFoundError):
            pass


def _resolve_cuda_bin() -> str | None:
    """Locate the most recent CUDA ``bin/x64`` directory by env var or filesystem.

    Order of preference: ``CUDA_PATH`` env var, then the highest-versioned
    install under ``%ProgramFiles%/NVIDIA GPU Computing Toolkit/CUDA``.
    """
    env = os.environ.get("CUDA_PATH")
    if env:
        for sub in ("bin/x64", "bin"):
            p = os.path.join(env, sub)
            if os.path.isdir(p):
                return p

    pf = os.environ.get("ProgramFiles", r"C:\Program Files")
    root = os.path.join(pf, "NVIDIA GPU Computing Toolkit", "CUDA")
    if not os.path.isdir(root):
        return None

    candidates = [
        d for d in os.listdir(root)
        if d.startswith("v") and os.path.isdir(os.path.join(root, d))
    ]

    def _ver_key(name: str) -> tuple[int, ...]:
        try:
            return tuple(int(x) for x in name.lstrip("v").split("."))
        except ValueError:
            return (0,)

    for name in sorted(candidates, key=_ver_key, reverse=True):
        for sub in ("bin/x64", "bin"):
            p = os.path.join(root, name, sub)
            if os.path.isdir(p):
                return p
    return None


if sys.platform == "win32":
    _here = Path(__file__).resolve().parent
    _add_dll_dir(str(_here))

    cuda_bin = _resolve_cuda_bin()
    if cuda_bin is not None:
        _add_dll_dir(cuda_bin)


from .pyOpenMVS import *  # noqa: F401, F403, E402
from .pyOpenMVS import (  # noqa: E402
    ExportMVSConfig,
    FeatureExtractionConfig,
    ImportConfig,
    MatchConfig,
    ROMA2Config,
    ReconstructionConfig,
    Scene,
    SfMScene,
    ViewGraphCalibratorConfig,
    export_sfm_to_mvs,
    set_working_folder,
)

__all__ = [
    "ExportMVSConfig",
    "FeatureExtractionConfig",
    "ImportConfig",
    "MatchConfig",
    "ROMA2Config",
    "ReconstructionConfig",
    "Scene",
    "SfMScene",
    "ViewGraphCalibratorConfig",
    "export_sfm_to_mvs",
    "set_working_folder",
]