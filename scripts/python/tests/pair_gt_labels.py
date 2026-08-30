#!/usr/bin/env python3
"""Label image pairs of a Polycam capture as ground-truth plausible, implausible or ambiguous.

Two modes, both driven by the capture's own ARKit ground truth:

* ``coverage`` (the default whenever ``<capture>/keyframes/depth/`` exists) measures how much of
  one frame the other actually sees. Every valid depth pixel of A is unprojected with A's
  intrinsics scaled to the depth grid, carried into the world with A's GT pose, projected into B,
  and counted when it lands inside B's image with positive depth *and* survives an occlusion check
  against B's own depth map (``|z_proj - depth_b| <= max(--occlusion-abs, --occlusion-rel *
  depth_b)``). ``cov_ab`` is that fraction, ``cov_ba`` the same the other way, and
  ``min_cov = min(cov_ab, cov_ba)``.

    - **plausible** iff the optical axes differ by at most ``--max-axis-deg`` *and*
      ``min_cov >= --min-cov-plausible``
    - **implausible** iff the axes differ by more than ``--max-axis-deg`` *or*
      ``min_cov < --max-cov-implausible``
    - **ambiguous** otherwise (neither clearly overlapping nor clearly not)

* ``distance`` is the older, purely pose-based fallback: implausible when the optical axes differ by
  more than ``--max-axis-deg`` or the camera centres are farther apart than ``--max-baseline-mult``
  times the median consecutive-keyframe baseline; plausible otherwise, never ambiguous. It is kept
  because it needs no depth, but it is miscalibrated on room-scale captures - genuinely overlapping
  pairs there reach several times the median baseline.

Every raw quantity behind the verdict is written out (``cov_ab``, ``cov_ba``, ``min_cov``,
``axis_angle_deg``, ``center_dist``, ``baseline_ratio``), so a caller can re-threshold without
recomputing anything, and every threshold is a command-line argument.

**Pose convention.** ``keyframes/cameras/<stem>.json``'s ``t_00..t_23`` is the 3x4 **camera-to-world**
matrix of the ARKit camera: ``apps/InterfacePolycam/InterfacePolycam.cpp:204-220`` builds
``T_session_arkitcam = T(t_03,t_13,t_23) * R(t_ij)`` and *inverts* it to obtain the world->camera pose
openMVS stores. The camera centre in world coordinates is therefore ``(t_03, t_13, t_23)`` read
straight off, and - ARKit cameras looking down their own -Z, which the same code turns into openMVS's
+Z with a 180 deg rotation about X - the world-space optical axis is ``-R[:,2]`` and the world->camera
rotation is ``diag(1,-1,-1) @ R.T``. ``corrected_cameras/`` is preferred over ``cameras/`` when the
capture ships it, exactly as InterfacePolycam does.

**Image indexing.** Scene image index = position in the sorted keyframe file order, which is the order
openMVS assigns image IDs in (verify against the ``poses.csv`` of a matched run).

Usage:
    pair_gt_labels.py CAPTURE_DIR --pairs pair_diag.csv -o pair_labels.csv
    pair_gt_labels.py CAPTURE_DIR --pairs-list "12,340 7,9" -o pair_labels.csv
    pair_gt_labels.py CAPTURE_DIR --pairs pairs.csv --pairs-valid pairs_valid.csv -o out.csv
    pair_gt_labels.py CAPTURE_DIR --mode distance --pairs pairs.csv -o out.csv

``--pairs`` accepts any CSV whose header carries one of the index column pairs ``idA/idB``, ``a/b``
or ``ImageA/ImageB`` (the last one holds image paths, resolved back to indices through their stem).
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

try:
    from PIL import Image
except ImportError:  # only the coverage mode needs it
    Image = None

FLIP = np.diag([1.0, -1.0, -1.0])  # AngleAxis(pi, X): ARKit camera axes -> openMVS/CV camera axes


def keyframe_stems(capture_dir):
    """The capture's keyframes in the order openMVS indexes them (sorted file order)."""
    images_dir = os.path.join(capture_dir, "keyframes", "images")
    if not os.path.isdir(images_dir):
        sys.exit("error: '%s' is not a Polycam capture (no keyframes/images)" % capture_dir)
    names = sorted(n for n in os.listdir(images_dir) if not n.startswith("."))
    stems = [os.path.splitext(n)[0] for n in names]
    # openMVS sorts the file names, so lexicographic order must agree with the numeric order the
    # timestamps suggest; uniform stem length is what guarantees that
    if all(s.isdigit() for s in stems) and len(set(len(s) for s in stems)) != 1:
        if stems != sorted(stems, key=int):
            sys.exit("error: keyframe stems have mixed lengths and sort differently as text and as "
                     "numbers, so the scene's image order cannot be reproduced from the file names")
    return stems


def cameras_dir(capture_dir):
    # InterfacePolycam prefers corrected_cameras when the capture ships them; read what it reads
    corrected = os.path.join(capture_dir, "keyframes", "corrected_cameras")
    if os.path.isdir(corrected):
        return corrected
    plain = os.path.join(capture_dir, "keyframes", "cameras")
    if not os.path.isdir(plain):
        sys.exit("error: '%s' has no keyframes/cameras" % capture_dir)
    return plain


def load_cameras(capture_dir, stems):
    """Per keyframe: world centre, world optical axis, world->camera rotation and the intrinsics."""
    folder = cameras_dir(capture_dir)
    centres, axes, rotations, intrinsics = [], [], [], []
    for stem in stems:
        path = os.path.join(folder, stem + ".json")
        if not os.path.isfile(path):
            sys.exit("error: no camera for keyframe '%s' (%s)" % (stem, path))
        with open(path) as f:
            data = json.load(f)
        rot = np.array([[float(data["t_%d%d" % (r, c)]) for c in range(3)] for r in range(3)])
        centre = np.array([float(data["t_%d3" % r]) for r in range(3)])
        axis = -rot[:, 2]  # ARKit cameras look down their own -Z
        norm = np.linalg.norm(axis)
        if norm <= 0:
            sys.exit("error: degenerate rotation for keyframe '%s'" % stem)
        centres.append(centre)
        axes.append(axis / norm)
        rotations.append(FLIP @ rot.T)  # world -> openMVS camera
        intrinsics.append(tuple(float(data[k]) for k in ("fx", "fy", "cx", "cy")) +
                          (int(data["width"]), int(data["height"])))
    return np.array(centres), np.array(axes), np.array(rotations), intrinsics


def median_baseline(centres):
    steps = np.linalg.norm(np.diff(centres, axis=0), axis=1)
    return float(np.median(steps)) if len(steps) else 0.0


def load_depth(capture_dir, stem, minConfidence):
    """Depth in metres (0 = invalid) on its own grid, low-confidence pixels dropped."""
    path = os.path.join(capture_dir, "keyframes", "depth", stem + ".png")
    depth = np.asarray(Image.open(path)).astype(np.float32) / 1000.0  # uint16 millimetres
    conf_path = os.path.join(capture_dir, "keyframes", "confidence", stem + ".png")
    if minConfidence > 0 and os.path.isfile(conf_path):
        conf = np.asarray(Image.open(conf_path))
        # ARKit ships three confidence levels; this capture family stores them as 0 / 54 / 255,
        # so anything above 0 is "medium or high" and 0 is the low level worth dropping
        if conf.shape == depth.shape:
            depth = np.where(conf >= minConfidence, depth, 0.0)
    return depth


def scaled_intrinsics(intr, shape):
    """Full-resolution intrinsics rescaled to a depth grid, pixel-centre convention preserved."""
    fx, fy, cx, cy, width, height = intr
    sx, sy = shape[1] / float(width), shape[0] / float(height)
    return fx * sx, fy * sy, (cx + 0.5) * sx - 0.5, (cy + 0.5) * sy - 0.5


def world_points(capture_dir, stems, rotations, centres, intrinsics, minConfidence, needed):
    """{index: (world XYZ of its valid depth pixels, number of grid cells, depth map, shape)}"""
    out = {}
    for i in sorted(needed):
        depth = load_depth(capture_dir, stems[i], minConfidence)
        fx, fy, cx, cy = scaled_intrinsics(intrinsics[i], depth.shape)
        rows, cols = depth.shape
        v, u = np.nonzero(depth > 0)
        z = depth[v, u]
        cam = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)  # openMVS camera frame
        world = cam @ rotations[i] + centres[i]  # (R_w2c^T x) written as x @ R_w2c
        out[i] = (world.astype(np.float32), int(v.size), depth, (rows, cols))
    return out


def coverage(src, dst, rotations, centres, intrinsics, occlusionAbs, occlusionRel):
    """Fraction of src's valid depth pixels that dst also sees, occlusion-checked against dst's depth."""
    world, numValid, _, _ = src
    depthDst, shape = dst[2], dst[3]
    if numValid == 0:
        return 0.0
    fx, fy, cx, cy = scaled_intrinsics(intrinsics, shape)
    cam = (world - centres) @ rotations.T  # world -> dst camera
    z = cam[:, 2]
    ok = z > 0
    if not ok.any():
        return 0.0
    # keep the pixel test in floating point: a point almost exactly on the camera plane projects to
    # a coordinate too large for int32, and casting that is undefined
    uf = cam[ok, 0] * fx / z[ok] + cx
    vf = cam[ok, 1] * fy / z[ok] + cy
    inside = (np.isfinite(uf) & np.isfinite(vf) &
              (uf > -0.5) & (uf < shape[1] - 0.5) & (vf > -0.5) & (vf < shape[0] - 0.5))
    if not inside.any():
        return 0.0
    u = np.rint(uf[inside]).astype(np.int32)
    v = np.rint(vf[inside]).astype(np.int32)
    zp = z[ok][inside]
    zd = depthDst[v, u]
    tolerance = np.maximum(occlusionAbs, occlusionRel * zd)
    visible = (zd > 0) & (np.abs(zp - zd) <= tolerance)
    return float(np.count_nonzero(visible)) / float(numValid)


def read_pairs(path, stemToIndex):
    """Read (idA, idB) out of a CSV, skipping the '#' comment lines."""
    with open(path) as f:
        rows = [line for line in f if not line.startswith("#")]
    reader = csv.DictReader(rows)
    fields = reader.fieldnames or []
    for keyA, keyB in (("idA", "idB"), ("a", "b"), ("ImageA", "ImageB")):
        if keyA in fields and keyB in fields:
            break
    else:
        sys.exit("error: '%s' has no idA/idB, a/b or ImageA/ImageB columns" % path)
    pairs = []
    for row in reader:
        if keyA == "ImageA":
            idA = stemToIndex.get(os.path.splitext(os.path.basename(row[keyA]))[0])
            idB = stemToIndex.get(os.path.splitext(os.path.basename(row[keyB]))[0])
            if idA is None or idB is None:
                sys.exit("error: '%s' names an image that is not a keyframe of this capture" % path)
        else:
            idA, idB = int(row[keyA]), int(row[keyB])
        pairs.append((idA, idB))
    return pairs


def read_campaign_min_cov(path):
    """Join column of the campaign's pseudo-GT pair list: (min,max) -> its own min_cov.

    The key is normalised to (min,max) because the lookup is, and a pair list is under no
    obligation to order its two columns - an ``a > b`` row keyed as written would silently never
    be found.
    """
    out = {}
    with open(path) as f:
        for row in csv.DictReader(line for line in f if not line.startswith("#")):
            if "min_cov" not in row:
                sys.exit("error: '%s' has no min_cov column" % path)
            a, b = int(row["a"]), int(row["b"])
            out[(min(a, b), max(a, b))] = row["min_cov"]
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("capture", help="the Polycam capture folder (holding keyframes/images and keyframes/cameras)")
    parser.add_argument("--pairs", help="CSV of pairs to label (idA/idB, a/b or ImageA/ImageB columns)")
    parser.add_argument("--pairs-list", help="pairs as 'a,b' items separated by spaces, instead of --pairs")
    parser.add_argument("--pairs-valid", help="optional CSV with a,b,min_cov columns to join (the campaign's pseudo-GT list)")
    parser.add_argument("-o", "--output", required=True, help="output CSV")
    parser.add_argument("--mode", choices=("auto", "coverage", "distance"), default="auto",
                        help="auto (coverage when keyframes/depth exists, else distance)")
    parser.add_argument("--max-axis-deg", type=float, default=90.0, help="implausible above this optical-axis difference (default 90)")
    parser.add_argument("--min-cov-plausible", type=float, default=0.15, help="coverage mode: plausible at or above this min_cov (default 0.15)")
    parser.add_argument("--max-cov-implausible", type=float, default=0.03, help="coverage mode: implausible below this min_cov (default 0.03)")
    parser.add_argument("--occlusion-abs", type=float, default=0.10, help="coverage mode: absolute depth tolerance in metres (default 0.10)")
    parser.add_argument("--occlusion-rel", type=float, default=0.10, help="coverage mode: relative depth tolerance (default 0.10)")
    parser.add_argument("--min-depth-confidence", type=int, default=1,
                        help="coverage mode: drop depth pixels below this confidence byte (default 1, i.e. ARKit's low level; 0 = keep all)")
    parser.add_argument("--max-baseline-mult", type=float, default=3.0, help="distance mode: implausible above this multiple of the median consecutive baseline (default 3)")
    args = parser.parse_args()

    stems = keyframe_stems(args.capture)
    stemToIndex = {stem: i for i, stem in enumerate(stems)}
    centres, axes, rotations, intrinsics = load_cameras(args.capture, stems)
    baseline = median_baseline(centres)
    if baseline <= 0:
        sys.exit("error: the capture's median consecutive-keyframe baseline is zero")

    if args.pairs:
        pairs = read_pairs(args.pairs, stemToIndex)
    elif args.pairs_list:
        pairs = [tuple(int(v) for v in item.split(",")) for item in args.pairs_list.split()]
    else:
        sys.exit("error: pass either --pairs or --pairs-list")
    for idA, idB in pairs:
        if not (0 <= idA < len(stems) and 0 <= idB < len(stems)):
            sys.exit("error: pair (%d,%d) is outside the capture's %d keyframes" % (idA, idB, len(stems)))
    campaign = read_campaign_min_cov(args.pairs_valid) if args.pairs_valid else {}

    mode = args.mode
    hasDepth = os.path.isdir(os.path.join(args.capture, "keyframes", "depth"))
    if mode == "auto":
        mode = "coverage" if hasDepth else "distance"
    if mode == "coverage":
        if not hasDepth:
            sys.exit("error: --mode coverage needs keyframes/depth/")
        if Image is None:
            sys.exit("error: --mode coverage needs Pillow (pip install pillow)")
        clouds = world_points(args.capture, stems, rotations, centres, intrinsics,
                              args.min_depth_confidence, {i for pair in pairs for i in pair})

    numByLabel = {"plausible": 0, "implausible": 0, "ambiguous": 0}
    with open(args.output, "w", newline="") as f:
        f.write("# pair_gt_labels.py mode=%s capture=%s\n" % (mode, os.path.basename(os.path.normpath(args.capture))))
        f.write("# %d keyframes, median consecutive-keyframe baseline %.6f GT units\n" % (len(stems), baseline))
        if mode == "coverage":
            f.write("# plausible: axis <= %g deg and min_cov >= %g | implausible: axis > %g deg or min_cov < %g | ambiguous: otherwise\n"
                    % (args.max_axis_deg, args.min_cov_plausible, args.max_axis_deg, args.max_cov_implausible))
            f.write("# coverage: valid depth pixels of one frame the other also sees, occlusion tolerance max(%g m, %g x depth), depth confidence >= %d\n"
                    % (args.occlusion_abs, args.occlusion_rel, args.min_depth_confidence))
        else:
            f.write("# implausible: axis > %g deg or centres > %g x the median baseline; plausible otherwise (no ambiguous class)\n"
                    % (args.max_axis_deg, args.max_baseline_mult))
        writer = csv.writer(f)
        writer.writerow(["idA", "idB", "stem_a", "stem_b", "cov_ab", "cov_ba", "min_cov",
                         "axis_angle_deg", "center_dist", "baseline_ratio", "campaign_min_cov", "label"])
        for idA, idB in pairs:
            angle = math.degrees(math.acos(max(-1.0, min(1.0, float(axes[idA] @ axes[idB])))))
            dist = float(np.linalg.norm(centres[idA] - centres[idB]))
            ratio = dist / baseline
            if mode == "coverage":
                covAB = coverage(clouds[idA], clouds[idB], rotations[idB], centres[idB],
                                 intrinsics[idB], args.occlusion_abs, args.occlusion_rel)
                covBA = coverage(clouds[idB], clouds[idA], rotations[idA], centres[idA],
                                 intrinsics[idA], args.occlusion_abs, args.occlusion_rel)
                minCov = min(covAB, covBA)
                if angle > args.max_axis_deg or minCov < args.max_cov_implausible:
                    label = "implausible"
                elif minCov >= args.min_cov_plausible:
                    label = "plausible"
                else:
                    label = "ambiguous"
                covCells = ["%.6f" % covAB, "%.6f" % covBA, "%.6f" % minCov]
            else:
                label = "implausible" if (angle > args.max_axis_deg or ratio > args.max_baseline_mult) else "plausible"
                covCells = ["", "", ""]
            numByLabel[label] += 1
            writer.writerow([idA, idB, stems[idA], stems[idB]] + covCells +
                            ["%.4f" % angle, "%.6f" % dist, "%.4f" % ratio,
                             campaign.get((min(idA, idB), max(idA, idB)), ""), label])

    if campaign:
        joined = sum(1 for idA, idB in pairs if (min(idA, idB), max(idA, idB)) in campaign)
        print("joined %d/%d pairs against the %d rows of '%s'"
              % (joined, len(pairs), len(campaign), args.pairs_valid))
    print("%d keyframes, median consecutive baseline %.6f, mode %s" % (len(stems), baseline, mode))
    print("%d pairs labelled: %d plausible, %d implausible, %d ambiguous"
          % (len(pairs), numByLabel["plausible"], numByLabel["implausible"], numByLabel["ambiguous"]))


if __name__ == "__main__":
    main()
