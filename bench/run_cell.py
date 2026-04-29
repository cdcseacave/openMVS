"""Run a single benchmark cell: clean dmaps, densify, evaluate, append CSV.

Usage:
    python run_cell.py --R 1 --V 8 --build baseline
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

BIN = r"C:\Users\danco\Pro\openMVS\make\bin\vc17\x64\RelWithDebInfo\DensifyPointCloud.exe"
SCENE_DIR = Path(r"C:\Pro\TanksAndTemples\data\training\Truck")
WORK = SCENE_DIR / "runMetashape"
TNT = Path(r"C:\Pro\TanksAndTemples\python_toolbox\evaluation\run.py")
PY_EVAL = Path(r"C:\Users\danco\.venv\tnt\Scripts\python.exe")
CSV_PATH = Path(r"C:\Users\danco\Pro\openMVS\bench\truck_results.csv")


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def clean_dmaps() -> None:
    for pat in ("*.dmap", "*.dmap.png", "scene_dense.ply", "scene_dense.mvs"):
        for f in glob.glob(str(WORK / pat)):
            try:
                os.remove(f)
            except OSError as e:
                log(f"  could not remove {f}: {e}")
    # Also clear cached alignment so each run is independent.
    for ext in (".npy", ".npy.bak"):
        f = SCENE_DIR / f"Truck_final_transform{ext}"
        if f.exists():
            f.unlink()


def latest_log(before_paths: set[str]) -> Path | None:
    after = set(glob.glob(str(WORK / "DensifyPointCloud-*.log")))
    new = after - before_paths
    if not new:
        return None
    return Path(max(new, key=os.path.getmtime))


def parse_densify_log(log_path: Path) -> dict:
    """Pull total wall, per-view stats, point count from a DensifyPointCloud log."""
    total_ms = None
    point_count = None
    per_view_ms: list[int] = []
    pattern_total = re.compile(r"Densifying point-cloud completed:\s*(\d+)\s*points\s*\(([^)]+)\)")
    pattern_view = re.compile(r"estimated using\s+\d+ images:\s*\d+x\d+\s*\((\d+)ms\)")
    txt = log_path.read_text(encoding="utf-8", errors="replace")
    m = pattern_total.search(txt)
    if m:
        point_count = int(m.group(1))
        total_ms = duration_to_seconds(m.group(2))
    for m in pattern_view.finditer(txt):
        per_view_ms.append(int(m.group(1)))
    return {
        "total_s": total_ms,
        "points": point_count,
        "n_views": len(per_view_ms),
        "view_ms_mean": (sum(per_view_ms) / len(per_view_ms)) if per_view_ms else None,
        "view_ms_max": max(per_view_ms) if per_view_ms else None,
    }


def duration_to_seconds(s: str) -> float:
    """Parse OpenMVS '9m27s978ms' / '4m3s80ms' / '212ms' style strings."""
    h = m_ = sec = ms = 0
    rest = s
    mh = re.match(r"(\d+)h", rest)
    if mh:
        h = int(mh.group(1)); rest = rest[mh.end():]
    mm = re.match(r"(\d+)m(?!s)", rest)
    if mm:
        m_ = int(mm.group(1)); rest = rest[mm.end():]
    ms2 = re.match(r"(\d+)s", rest)
    if ms2:
        sec = int(ms2.group(1)); rest = rest[ms2.end():]
    mms = re.match(r"(\d+)ms", rest)
    if mms:
        ms = int(mms.group(1))
    return h * 3600 + m_ * 60 + sec + ms / 1000.0


def parse_eval_stdout(stdout: str) -> dict:
    out = {"precision": None, "recall": None, "fscore": None, "tau": None}
    for k, key in (("precision", "precision"), ("recall", "recall"), ("f-score", "fscore"), ("distance tau", "tau")):
        m = re.search(rf"{re.escape(k)}\s*:\s*([0-9.]+)", stdout)
        if m:
            out[key] = float(m.group(1))
    return out


def run_densify(R: int, V: int) -> tuple[Path, dict, float]:
    before = set(glob.glob(str(WORK / "DensifyPointCloud-*.log")))
    cmd = [BIN, "scene.mvs",
           "--resolution-level", str(R),
           "--number-views", str(V),
           "--estimate-roi", "0",   # workaround: v2.4.0 ROIPointWeights stack-overflows
           "--crop-to-roi", "0",
           "--tower-mode", "0"]
    log(f"densify: {' '.join(cmd)}")
    t0 = time.time()
    proc = subprocess.run(cmd, cwd=str(WORK), capture_output=True, text=True)
    wall = time.time() - t0
    if proc.returncode != 0:
        log(f"densify FAILED rc={proc.returncode}")
        log("stdout tail:\n" + proc.stdout[-800:])
        log("stderr tail:\n" + proc.stderr[-800:])
        sys.exit(1)
    lpath = latest_log(before)
    stats = parse_densify_log(lpath) if lpath else {}
    log(f"densify done in {wall:.1f}s wall, log={lpath.name if lpath else 'NONE'}, "
        f"openmvs_total={stats.get('total_s')}, points={stats.get('points')}, "
        f"views={stats.get('n_views')}, view_ms_mean={stats.get('view_ms_mean')}")
    return lpath, stats, wall


def run_eval(R: int, V: int, build: str) -> dict:
    out_dir = f"evaluation_R{R}_V{V}_{build}"
    cmd = [str(PY_EVAL), str(TNT),
           "--dataset-dir", str(SCENE_DIR),
           "--traj-path", "runMetashape/poses.log",
           "--ply-path", "runMetashape/scene_dense.ply",
           "--out-dir", out_dir]
    log(f"eval: {' '.join(cmd)}")
    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True)
    wall = time.time() - t0
    out = parse_eval_stdout(proc.stdout + "\n" + proc.stderr)
    out["eval_wall_s"] = wall
    out["eval_dir"] = str(SCENE_DIR / out_dir)
    if proc.returncode != 0 or out["fscore"] is None:
        log(f"eval rc={proc.returncode}, parsed={out}")
        log("stdout tail:\n" + proc.stdout[-1500:])
        log("stderr tail:\n" + proc.stderr[-800:])
    else:
        log(f"eval done in {wall:.1f}s: P={out['precision']} R={out['recall']} F1={out['fscore']} tau={out['tau']}")
    return out


def append_csv(row: dict) -> None:
    cols = ["build", "R", "V", "precision", "recall", "fscore", "tau",
            "openmvs_total_s", "wall_s", "eval_wall_s",
            "points", "view_ms_mean", "view_ms_max", "n_views",
            "log", "eval_dir"]
    new = not CSV_PATH.exists()
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    with CSV_PATH.open("a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerow(row)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--R", type=int, required=True)
    p.add_argument("--V", type=int, required=True)
    p.add_argument("--build", type=str, default="baseline")
    args = p.parse_args()
    log(f"=== cell R={args.R} V={args.V} build={args.build} ===")
    log("cleaning dmaps + alignment cache")
    clean_dmaps()
    log_path, dstats, wall = run_densify(args.R, args.V)
    estats = run_eval(args.R, args.V, args.build)
    row = {
        "build": args.build, "R": args.R, "V": args.V,
        "precision": estats.get("precision"), "recall": estats.get("recall"), "fscore": estats.get("fscore"),
        "tau": estats.get("tau"),
        "openmvs_total_s": dstats.get("total_s"),
        "wall_s": round(wall, 2), "eval_wall_s": round(estats.get("eval_wall_s", 0), 2),
        "points": dstats.get("points"),
        "view_ms_mean": round(dstats.get("view_ms_mean") or 0, 1),
        "view_ms_max": dstats.get("view_ms_max"),
        "n_views": dstats.get("n_views"),
        "log": log_path.name if log_path else "",
        "eval_dir": estats.get("eval_dir", ""),
    }
    append_csv(row)
    log(f"row appended to {CSV_PATH}")
    log(f"DONE cell R={args.R} V={args.V}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
