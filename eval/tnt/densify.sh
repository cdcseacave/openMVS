#!/usr/bin/env bash
# Densify + fuse ONE scene with ONE backend, starting from the shared scene.mvs.
# Records wall time, CPU-user time and fused point count to a metrics JSON.
#
# Usage: ./densify.sh <Scene> <metal|cpu>
#   reads  $TNT_ROOT/<Scene>/mvs/scene.mvs
#   writes $TNT_ROOT/<Scene>/out_<backend>/scene_dense.ply
#          $TNT_ROOT/<Scene>/out_<backend>/metrics.json
#
# Backend selection (this OpenMVS lineage): Metal is the default on Apple;
# OPENMVS_DISABLE_METAL=1 forces the pure-CPU DepthEstimator path.
source "$(cd "$(dirname "$0")" && pwd)/lib.sh"

SCENE="${1:?usage: densify.sh <Scene> <metal|cpu>}"
BACKEND="${2:?usage: densify.sh <Scene> <metal|cpu>}"
MVS="$TNT_ROOT/$SCENE/mvs/scene.mvs"
OUT="$TNT_ROOT/$SCENE/out_$BACKEND"

[ -f "$MVS" ] || die "missing $MVS (run prepare_scene.sh $SCENE)"
[ -x "$DPC" ] || die "DensifyPointCloud not found at $DPC"
mkdir -p "$OUT"

# Idempotent: skip if already densified (lets the sweep resume). FORCE=1 redoes.
if [ -z "${FORCE:-}" ] && [ -f "$OUT/metrics.json" ] && \
   [ -f "$(python3 -c "import json;print(json.load(open('$OUT/metrics.json')).get('ply',''))" 2>/dev/null)" ]; then
  log "[$SCENE/$BACKEND] already done -> $OUT/metrics.json (FORCE=1 to redo)"
  exit 0
fi

case "$BACKEND" in
  metal) unset OPENMVS_DISABLE_METAL ;;
  cpu)   export OPENMVS_DISABLE_METAL=1 ;;
  *)     die "backend must be 'metal' or 'cpu' (CUDA volunteers use contribute_cuda.sh)" ;;
esac

log "[$SCENE/$BACKEND] densifying (params: ${DENSIFY_PARAMS[*]})"
# /usr/bin/time -l gives wall + user + sys + peak RSS on macOS.
TIMELOG="$OUT/time.txt"
/usr/bin/time -l "$DPC" "$MVS" \
    "${DENSIFY_PARAMS[@]}" \
    -w "$OUT" -o "$OUT/scene_dense.mvs" \
    >"$OUT/densify.log" 2>"$TIMELOG" || die "densify failed, see $OUT/densify.log"

PLY="$OUT/scene_dense.ply"
[ -f "$PLY" ] || PLY="$OUT/scene_dense.mvs.ply"
[ -f "$PLY" ] || die "no fused ply produced in $OUT"

WALL=$(awk '/real/{print $1}' "$TIMELOG" | tail -1)
USER=$(awk '/user/{print $1}' "$TIMELOG" | tail -1)
RSS=$( awk '/maximum resident/{print $1}' "$TIMELOG" | tail -1)
NPTS=$(ply_vertex_count "$PLY")

cat > "$OUT/metrics.json" <<JSON
{
  "scene": "$SCENE",
  "backend": "$BACKEND",
  "wall_s": ${WALL:-null},
  "cpu_user_s": ${USER:-null},
  "peak_rss_bytes": ${RSS:-null},
  "fused_points": ${NPTS:-0},
  "ply": "$PLY"
}
JSON
log "[$SCENE/$BACKEND] wall=${WALL}s pts=${NPTS} -> $OUT/metrics.json"
