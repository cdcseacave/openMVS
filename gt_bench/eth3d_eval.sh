#!/bin/bash
# eth3d_eval.sh <fused.ply> <scene_dir> <out.json>
#
# Evaluates a reconstructed point cloud against an ETH3D scene's laser-scan ground truth via the
# official ETH3DMultiViewEvaluation tool, then parses its stdout (Tolerances/Completenesses/
# Accuracies/F1-scores lines) into a JSON with the same key shape as EvalFusionGT.py's BlendedMVS
# path (+ f1), for Task 7's aggregator.
#
# <scene_dir> = a gt_bench eth3d scene root, e.g. /home/ubuntu/virginia/gt_bench/eth3d/courtyard
#   (must contain dslr_scan_eval/scan_alignment.mlp).
#
# IMPORTANT (Task 2 finding, kept here so it isn't "fixed" by mistake): the tool applies each
# scan's global_T_mesh transform (from scan_alignment.mlp) ONLY to the ground-truth point clouds
# it loads -- NOT to --reconstruction_ply_path, which is read as-is. This is correct for real
# reconstructions produced from dslr_calibration_undistorted camera poses: they are already in the
# same global/aligned frame scan_alignment.mlp registers the raw laser scans into. Do NOT
# transform <fused.ply> before calling the tool.
#
# Self-test (see below / gt_bench/README.md): pass scan1.ply as <fused.ply> together with a
# single-scan identity .mlp as ground truth -> completeness=accuracy=f1=1.0 exactly (verified).
#
# Tolerances are fixed absolute meters (ETH3D scenes are already metric), not derived from a bbox
# diagonal like EvalFusionGT.py's BlendedMVS fractional tolerances -- override via env var.
set -euo pipefail

FUSED_PLY="${1:?usage: eth3d_eval.sh <fused.ply> <scene_dir> <out.json>}"
SCENE_DIR="${2:?usage: eth3d_eval.sh <fused.ply> <scene_dir> <out.json>}"
OUT_JSON="${3:?usage: eth3d_eval.sh <fused.ply> <scene_dir> <out.json>}"

TOOL="${ETH3D_TOOL:-/home/ubuntu/virginia/gt_bench/tools/multi-view-evaluation/build/ETH3DMultiViewEvaluation}"
TOLERANCES="${ETH3D_TOLERANCES:-0.01,0.02,0.05,0.1}"
PY=/home/ubuntu/miniconda3/bin/python
REPO=/home/ubuntu/openMVS
# Never write temp files into the repo or /tmp root -- see gt_bench/README.md data-root policy.
TMPDIR_ETH3D="${TMPDIR_ETH3D:-/home/ubuntu/virginia/gt_bench/tmp}"

MLP="$SCENE_DIR/dslr_scan_eval/scan_alignment.mlp"

[ -x "$TOOL" ] || { echo "error: ETH3D tool not found/executable: $TOOL" >&2; exit 1; }
[ -f "$MLP" ] || { echo "error: ground-truth mlp not found: $MLP" >&2; exit 1; }
[ -f "$FUSED_PLY" ] || { echo "error: reconstruction ply not found: $FUSED_PLY" >&2; exit 1; }

mkdir -p "$TMPDIR_ETH3D"
LOG=$(mktemp "$TMPDIR_ETH3D/eth3d_eval.XXXXXX.log")
trap 'rm -f "$LOG"' EXIT

echo "Running ETH3DMultiViewEvaluation: reconstruction=$FUSED_PLY  gt_mlp=$MLP  tolerances=$TOLERANCES" >&2
"$TOOL" --tolerances "$TOLERANCES" \
    --reconstruction_ply_path "$FUSED_PLY" \
    --ground_truth_mlp_path "$MLP" 2>&1 | tee "$LOG" >&2

mkdir -p "$(dirname "$OUT_JSON")"

"$PY" - "$LOG" "$FUSED_PLY" "$MLP" "$OUT_JSON" "$TOLERANCES" <<'PYEOF'
import sys, os, re, json

log_path, fused_ply, mlp_path, out_json, tol_csv = sys.argv[1:6]
sys.path.insert(0, '/home/ubuntu/openMVS/scripts/python')
import numpy as np
import GtUtils

text = open(log_path).read()


def parse_line(label):
    m = re.search(r'^%s:\s*(.+)$' % re.escape(label), text, re.MULTILINE)
    if not m:
        raise ValueError('%r line not found in ETH3DMultiViewEvaluation output:\n%s' % (label, text))
    return [float(x) for x in m.group(1).split()]


tol = parse_line('Tolerances')
comp = parse_line('Completenesses')
acc = parse_line('Accuracies')
f1 = parse_line('F1-scores')

keys = [k.strip() for k in tol_csv.split(',')]
assert len(keys) == len(tol) == len(comp) == len(acc) == len(f1), (
    'tolerance count mismatch: --tolerances=%r vs parsed %r/%r/%r/%r' % (keys, tol, comp, acc, f1))
for k, t in zip(keys, tol):
    assert abs(float(k) - t) < 1e-9, 'tolerance label %r does not match parsed value %r' % (k, t)


def ply_vertex_count(path):
    with open(path, 'rb') as f:
        n = 0
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
            elif line == 'end_header' or line == '':
                break
    return n


rec_xyz = GtUtils.load_ply_xyz(fused_ply)
n_rec = len(rec_xyz)
diag = float(np.linalg.norm(rec_xyz.max(0) - rec_xyz.min(0))) if n_rec else 0.0

# n_gt: sum vertex counts of every scan .ply referenced by the .mlp (some scenes, e.g. courtyard,
# register more than one scan simultaneously).
import xml.etree.ElementTree as ET
mlp_dir = os.path.dirname(os.path.abspath(mlp_path))
tree = ET.parse(mlp_path)
n_gt = 0
for mesh in tree.getroot().iter('MLMesh'):
    fn = mesh.get('filename')
    p = fn if os.path.isabs(fn) else os.path.join(mlp_dir, fn)
    if os.path.isfile(p):
        n_gt += ply_vertex_count(p)
    else:
        print('warning: GT scan referenced by %s not found, excluded from n_gt: %s' % (mlp_path, p), file=sys.stderr)

out = {
    'completeness': dict(zip(keys, comp)),
    'accuracy': dict(zip(keys, acc)),
    # ADAPTATION: ETH3DMultiViewEvaluation has no native "gross outlier" concept (unlike
    # EvalFusionGT.py's separate 5%-of-diag gross_tol). Defined here as 1 - accuracy at the
    # LARGEST requested tolerance (last entry of --tolerances, default 0.1m): the exact fraction
    # of reconstruction points with NO ground-truth scan surface within that tolerance.
    'gross_outlier_frac': 1.0 - acc[-1],
    # ETH3D tolerances are fixed absolute meters, not derived from a bbox diagonal (unlike
    # EvalFusionGT.py's BlendedMVS path) -- tol_abs duplicates the same values under the same
    # string keys for JSON-shape parity with EvalFusionGT.py.
    'tol_abs': dict(zip(keys, tol)),
    # informational only (reconstruction bbox diagonal); NOT used to derive tol_abs here.
    'diag': diag,
    'n_rec': n_rec,
    'n_gt': n_gt,
    'f1': dict(zip(keys, f1)),
}
os.makedirs(os.path.dirname(os.path.abspath(out_json)) or '.', exist_ok=True)
with open(out_json, 'w') as fh:
    json.dump(out, fh, indent=2)
print('JSON written to %s' % out_json)
print(json.dumps(out, indent=2))
PYEOF
