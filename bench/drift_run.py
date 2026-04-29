"""Run two targeted diagnostic densify passes on Truck (R=1, V=8): one CPU,
one CUDA. Each cleans the runMetashape work dir, runs DPC, and copies the
resulting OpenMVS log file into bench/ with a clear suffix.

Diagnostic: SceneDensify.cpp emits per-image '[NDRIFT] ...' lines logging
||R*R.t() - I||_F + ||n||/||R.t()*n|| histograms. Comparing CPU vs CUDA
isolates kernel-side normal drift from camera.R orthogonality residual.
"""
from __future__ import annotations

import glob
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

BIN = r"C:\Users\danco\Pro\openMVS\make\bin\vc17\x64\RelWithDebInfo\DensifyPointCloud.exe"
SCENE_DIR = Path(r"C:\Pro\TanksAndTemples\data\training\Truck")
WORK = SCENE_DIR / "runMetashape"
BENCH = Path(r"C:\Users\danco\Pro\openMVS\bench")


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def clean() -> None:
    for pat in ("*.dmap", "*.dmap.png", "scene_dense.ply", "scene_dense.mvs"):
        for f in glob.glob(str(WORK / pat)):
            try:
                os.remove(f)
            except OSError:
                pass


def latest_log(before: set[str]) -> Path | None:
    after = set(glob.glob(str(WORK / "DensifyPointCloud-*.log")))
    new = after - before
    if not new:
        return None
    return Path(max(new, key=os.path.getmtime))


def run(cuda_device: str, label: str) -> Path:
    log(f"=== run {label} (cuda-device='{cuda_device}') ===")
    log("cleaning dmaps")
    clean()
    before = set(glob.glob(str(WORK / "DensifyPointCloud-*.log")))
    cmd = [BIN, "scene.mvs",
           "--cuda-device", cuda_device,
           "--resolution-level", "1",
           "--number-views", "8",
           "--estimate-roi", "0",
           "--crop-to-roi", "0",
           "--tower-mode", "0"]
    log(f"cmd: {' '.join(cmd)}")
    t0 = time.time()
    proc = subprocess.run(cmd, cwd=str(WORK), capture_output=True, text=True)
    wall = time.time() - t0
    log(f"exit={proc.returncode} wall={wall:.1f}s")
    lpath = latest_log(before)
    if not lpath:
        log("no log produced!")
        sys.exit(1)
    dst = BENCH / f"drift_{label}.log"
    shutil.copy(lpath, dst)
    log(f"log copied to {dst}")
    return dst


def main() -> None:
    BENCH.mkdir(parents=True, exist_ok=True)
    cpu_log = run("", "cpu")
    cuda_log = run("-1", "cuda")
    log(f"DONE  cpu={cpu_log}  cuda={cuda_log}")


if __name__ == "__main__":
    main()
