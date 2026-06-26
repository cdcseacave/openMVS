#!/usr/bin/env bash
# Orchestrate the LOCAL columns (Metal + CPU) over the scene list and emit the
# markdown table to paste into the PR discussion.
#
# Usage: ./run_local.sh                # SCENES from lib.sh
#        SCENES="Barn Truck" ./run_local.sh
source "$(cd "$(dirname "$0")" && pwd)/lib.sh"
HERE="$(cd "$(dirname "$0")" && pwd)"

for SCENE in $SCENES; do
  [ -f "$TNT_ROOT/$SCENE/mvs/scene.mvs" ] || "$HERE/prepare_scene.sh" "$SCENE"
  for BACKEND in metal cpu; do
    "$HERE/densify.sh" "$SCENE" "$BACKEND"
    PLY=$(python3 -c "import json;print(json.load(open('$TNT_ROOT/$SCENE/out_$BACKEND/metrics.json'))['ply'])")
    if [ -f "$TNT_ROOT/$SCENE/gt/$SCENE.ply" ]; then
      "$HERE/evaluate_f1.sh" "$SCENE" "$PLY" "$BACKEND"
    else
      warn "[$SCENE] no ground truth -> densify+timing only, F1 skipped (drop GT in $TNT_ROOT/$SCENE/gt and re-run evaluate_f1.sh)"
    fi
  done
done

log "Building markdown table"
python3 "$HERE/make_table.py" "$TNT_ROOT" $SCENES | tee "$TNT_ROOT/RESULTS.md"
log "Table written to $TNT_ROOT/RESULTS.md"
