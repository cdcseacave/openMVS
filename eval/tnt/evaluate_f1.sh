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
#    can align our cloud's frame to the GT.
#
#    Derive it from the shared scene.mvs ITSELF (export to a COLMAP text model
#    with InterfaceCOLMAP, then colmap_to_tnt_log.py). This keeps the eval
#    self-contained from just the scene.mvs + its images — no separate
#    colmap/sparse/0 needed — which is exactly what a CUDA volunteer or anyone
#    reproducing from the published tarball has.
#
#    IMPORTANT: the scene.mvs lives in its OWN COLMAP frame. Do NOT substitute
#    the GT's <Scene>_COLMAP_SfM.log as the trajectory — it has the same entry
#    count but different poses, so the alignment silently breaks and F1 collapses
#    (~0.01). The poses must come from our reconstruction, i.e. the scene.mvs.
if [ ! -f "$TRAJ" ]; then
  MVS="$TNT_ROOT/$SCENE/mvs/scene.mvs"
  [ -f "$MVS" ] || die "missing $MVS"
  [ -x "$INTERFACE_COLMAP" ] || die "InterfaceCOLMAP not built (see setup.sh)"
  log "[$SCENE] deriving TnT trajectory from scene.mvs (InterfaceCOLMAP export)"
  EXPORT="$TNT_ROOT/$SCENE/colmap_export"; rm -rf "$EXPORT"; mkdir -p "$EXPORT"
  # scene.mvs references its images by relative path, so export from its folder.
  ( cd "$(dirname "$MVS")" && "$INTERFACE_COLMAP" -i "$MVS" -o "$EXPORT" --binary 0 ) \
    >/dev/null 2>&1 || die "InterfaceCOLMAP export failed"
  IMAGES_TXT="$(find "$EXPORT" -name images.txt | head -1)"
  [ -n "$IMAGES_TXT" ] || die "no images.txt from InterfaceCOLMAP export"
  # filename list for the per-image entries: the (undistorted) images scene.mvs
  # references, falling back to the raw image set if the dense dir is absent.
  IMGDIR="$TNT_ROOT/$SCENE/colmap/dense/images"; [ -d "$IMGDIR" ] || IMGDIR="$IMG"
  "$TNT_PYTHON" "$HERE/colmap_to_tnt_log.py" \
    "$IMAGES_TXT" "$IMGDIR" "$TRAJ" jpg || die "traj conversion failed"
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
