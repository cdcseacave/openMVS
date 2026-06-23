#!/usr/bin/env bash
# Compute precision / recall / F1 of a fused point cloud against the official
# Tanks-and-Temples ground truth, using the isl-org/TanksAndTemples toolbox.
#
# Usage: ./evaluate_f1.sh <Scene> <path/to/fused.ply> <backend-label>
#   writes $TNT_ROOT/<Scene>/eval_<label>/evaluation.json
#
# Ground-truth layout expected under $TNT_ROOT/<Scene>/gt/ :
#   <Scene>.ply                ground-truth point cloud (public, training set)
#   <Scene>.json               crop volume
#   <Scene>_trans.txt          initial alignment (GT <- reconstruction)
#   <Scene>_COLMAP_SfM.log     reference camera trajectory
# (all provided by the TnT training download).
#
# NOTE: because the volunteer CUDA run returns only its fused PLY, YOU run this
# step on every backend's PLY with the SAME GT and toolbox version -> evaluation
# is held perfectly constant; only the reconstruction differs.
source "$(cd "$(dirname "$0")" && pwd)/lib.sh"

SCENE="${1:?usage: evaluate_f1.sh <Scene> <fused.ply> <label>}"
PLY="${2:?usage: evaluate_f1.sh <Scene> <fused.ply> <label>}"
LABEL="${3:?usage: evaluate_f1.sh <Scene> <fused.ply> <label>}"
GT="$TNT_ROOT/$SCENE/gt"
EVAL="$TNT_ROOT/$SCENE/eval_$LABEL"
SPARSE="$TNT_ROOT/$SCENE/colmap/sparse/0"
IMG="$TNT_ROOT/$SCENE/images"
TRAJ="$TNT_ROOT/$SCENE/traj_tnt.log"
HERE="$(cd "$(dirname "$0")" && pwd)"

[ -f "$PLY" ] || die "missing reconstruction $PLY"
[ -d "$TNT_TOOLBOX" ] || die "TnT toolbox missing at $TNT_TOOLBOX (see setup.sh)"
[ -f "$GT/$SCENE.ply" ] || die "missing GT $GT/$SCENE.ply (download TnT training GT)"
mkdir -p "$EVAL"

# Idempotent: skip if already evaluated with a real F1. FORCE=1 redoes.
if [ -z "${FORCE:-}" ] && grep -q '"f1": [0-9]' "$EVAL/evaluation.json" 2>/dev/null; then
  log "[$SCENE/$LABEL] already evaluated -> $EVAL/evaluation.json (FORCE=1 to redo)"
  exit 0
fi

# 1) Our reconstruction's trajectory in TnT .log format (one entry per input
#    image, identity for unregistered) — required as --traj-path so the toolbox
#    can align our cloud's frame to the GT. Built once per scene, shared by all
#    backends (same SfM -> same scene.mvs -> same poses).
if [ ! -f "$TRAJ" ]; then
  [ -d "$SPARSE" ] || die "missing COLMAP model $SPARSE (run prepare_scene.sh)"
  log "[$SCENE] generating TnT trajectory log from COLMAP model"
  mkdir -p "$SPARSE/../sparse_txt"
  colmap model_converter --input_path "$SPARSE" \
    --output_path "$SPARSE/../sparse_txt" --output_type TXT >/dev/null 2>&1 || \
    die "colmap model_converter failed"
  "$TNT_PYTHON" "$HERE/colmap_to_tnt_log.py" \
    "$SPARSE/../sparse_txt/images.txt" "$IMG" "$TRAJ" jpg || die "traj conversion failed"
fi

# 2) The toolbox derives the scene name from basename(--dataset-dir) and expects
#    the GT files directly inside it. Our GT lives in gt/, so expose a dir whose
#    basename IS the scene, with symlinks to the GT files.
DSDIR="$GT/$SCENE"
mkdir -p "$DSDIR"
for suf in .ply .json _trans.txt _COLMAP_SfM.log _mapping_reference.txt; do
  [ -e "$GT/$SCENE$suf" ] && ln -sf "../$SCENE$suf" "$DSDIR/$SCENE$suf"
done

log "[$SCENE/$LABEL] running TnT evaluation (precision/recall/F1)"
"$TNT_PYTHON" "$TNT_TOOLBOX/python_toolbox/evaluation/run.py" \
  --dataset-dir "$DSDIR" \
  --traj-path   "$TRAJ" \
  --ply-path    "$PLY" \
  --out-dir     "$EVAL" \
  >"$EVAL/eval.log" 2>&1 || die "TnT eval failed, see $EVAL/eval.log"

# The toolbox writes <Scene>.precision/recall/fscore; normalise to one JSON.
P=$(grep -hoE 'precision : [0-9.]+'  "$EVAL"/*.txt "$EVAL/eval.log" 2>/dev/null | tail -1 | awk '{print $3}')
R=$(grep -hoE 'recall : [0-9.]+'     "$EVAL"/*.txt "$EVAL/eval.log" 2>/dev/null | tail -1 | awk '{print $3}')
F=$(grep -hoE 'f-score : [0-9.]+'    "$EVAL"/*.txt "$EVAL/eval.log" 2>/dev/null | tail -1 | awk '{print $3}')

cat > "$EVAL/evaluation.json" <<JSON
{ "scene": "$SCENE", "label": "$LABEL",
  "precision": ${P:-null}, "recall": ${R:-null}, "f1": ${F:-null} }
JSON
log "[$SCENE/$LABEL] F1=${F} (P=${P} R=${R}) -> $EVAL/evaluation.json"
