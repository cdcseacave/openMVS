#!/usr/bin/env python3
"""Convert a COLMAP model to a Tanks-and-Temples trajectory .log file.

Self-contained replacement for the toolbox's convert_to_logfile.py (which
depends on a missing read_model.py and predates the COLMAP 4.x binary format).
We export the model to TXT with `colmap model_converter` first, so this script
only parses the stable images.txt text format.

TnT .log format: one entry per INPUT image, ordered by sorted filename, as
  <k> <k> 0
  <4x4 camera-to-world matrix, one row per line>
Unregistered images get the identity matrix (per the TnT convention), so the
entry count and index order match the reference <Scene>_COLMAP_SfM.log exactly,
which is what the evaluator's index-wise trajectory correspondence requires.

Usage: colmap_to_tnt_log.py <images_txt> <images_dir> <out.log> [ext=jpg]
"""
import glob
import os
import sys

import numpy as np


def quat_to_R(qw, qx, qy, qz):
    n = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def main():
    images_txt, images_dir, out_log = sys.argv[1], sys.argv[2], sys.argv[3]
    ext = sys.argv[4] if len(sys.argv) > 4 else "jpg"

    # name -> camera-to-world 4x4
    pose = {}
    with open(images_txt) as f:
        lines = [ln for ln in f if not ln.startswith("#")]
    # images.txt: 2 lines per image; first line has the pose, second the 2D pts
    for i in range(0, len(lines), 2):
        parts = lines[i].split()
        if len(parts) < 10:
            continue
        qw, qx, qy, qz = map(float, parts[1:5])
        tx, ty, tz = map(float, parts[5:8])
        name = parts[9]
        R = quat_to_R(qw, qx, qy, qz)
        t = np.array([tx, ty, tz])
        c2w = np.eye(4)
        c2w[:3, :3] = R.T
        c2w[:3, 3] = -R.T @ t
        pose[os.path.basename(name)] = c2w

    imgs = sorted(os.path.basename(p) for p in glob.glob(f"{images_dir}/*.{ext}"))
    if not imgs:
        sys.exit(f"no *.{ext} images in {images_dir}")

    nreg = 0
    with open(out_log, "w") as f:
        for k, name in enumerate(imgs):
            M = pose.get(name)
            if M is None:
                M = np.eye(4)
            else:
                nreg += 1
            f.write(f"{k} {k} 0\n")
            for r in range(4):
                f.write(" ".join("{0:.12f}".format(M[r, c]) for c in range(4)) + "\n")
    print(f"{out_log}: {len(imgs)} entries, {nreg} registered", file=sys.stderr)


if __name__ == "__main__":
    main()
