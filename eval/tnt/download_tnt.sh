#!/usr/bin/env bash
# Fetch the Tanks-and-Temples TRAINING data and lay it out where the harness
# expects it. The official images + ground truth are distributed from
# https://www.tanksandtemples.org/download/ (training set, with GT) — there is
# no stable public CLI/mirror, so this script VERIFIES the layout and tells you
# exactly what to drop where rather than silently downloading the wrong thing.
#
# Expected per-scene layout under $TNT_ROOT/<Scene>/ :
#   images/                      the JPG frames (training image set)
#   gt/<Scene>.ply               ground-truth point cloud
#   gt/<Scene>.json              crop volume
#   gt/<Scene>_trans.txt         initial alignment
#   gt/<Scene>_COLMAP_SfM.log    reference trajectory
source "$(cd "$(dirname "$0")" && pwd)/lib.sh"

ok=1
for SCENE in $SCENES; do
  d="$TNT_ROOT/$SCENE"
  miss=()
  [ -d "$d/images" ] && [ -n "$(ls -A "$d/images" 2>/dev/null)" ] || miss+=("images/")
  for f in "$SCENE.ply" "$SCENE.json" "${SCENE}_COLMAP_SfM.log"; do
    [ -f "$d/gt/$f" ] || miss+=("gt/$f")
  done
  if [ ${#miss[@]} -eq 0 ]; then
    log "[$SCENE] OK"
  else
    ok=0
    warn "[$SCENE] missing: ${miss[*]}"
  fi
done

if [ "$ok" -ne 1 ]; then
  cat >&2 <<EOF

To obtain the missing data:
  1. Go to https://www.tanksandtemples.org/download/  (Training set).
  2. Download, per scene, the image set AND the ground-truth package
     (.ply, .json crop, _COLMAP_SfM.log, _trans.txt).
  3. Unpack into:  $TNT_ROOT/<Scene>/images  and  $TNT_ROOT/<Scene>/gt
  4. Re-run this script to verify.
EOF
  exit 1
fi
log "All scenes present."
