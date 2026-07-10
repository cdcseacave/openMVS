#!/usr/bin/env bash
# Confidence GT operating-frontier sweep: for every benchmark scene-level, grade BOTH the raw
# (pre-adjust, raw_dmaps/) and the adjusted (in-place) confidence maps against real GT depth with
# EvalConfidence.py GT mode, whose pooled JSON now carries the contamination-vs-completeness
# operating frontier (operating_points). Output -> results_confop/<TAG>_confop_{raw,adj}.json.
# Idempotent: skips any variant whose JSON already exists and is non-empty. bmvs first (cheap,
# fast directional signal), then eth3d (first raw run per scene-level builds the GT-remap .npy
# cache that the adj run reuses).
set -u
ROOT=/home/ubuntu/virginia/gt_bench
REPO=/home/ubuntu/openMVS
PY=/home/ubuntu/miniconda3/bin/python
OUT="$ROOT/results_confop"
mkdir -p "$OUT"

# bmvs before eth3d
scenes="$(awk '!/^#/{print "bmvs_" substr($1,1,8)}' "$REPO/gt_bench/scenes_blendedmvs.txt")
$(awk '!/^#/{print "eth3d_" $1}' "$REPO/gt_bench/scenes_eth3d.txt")"

for SCENE in $scenes; do
  case "$SCENE" in
    bmvs_*)
      LEVELS="1 0"; SHORT="${SCENE#bmvs_}"
      FULLID="$(awk -v s="$SHORT" '!/^#/ && $1 ~ ("^" s) {print $1; exit}' "$REPO/gt_bench/scenes_blendedmvs.txt")"
      GT_FORMAT=blendedmvs; GT_DEPTH_DIR="$ROOT/blendedmvs/$FULLID" ;;
    eth3d_*)
      LEVELS="3 2 1"; NAME="${SCENE#eth3d_}"
      GT_FORMAT=eth3d; GT_DEPTH_DIR="$ROOT/eth3d/$NAME" ;;
    *) echo "skip unknown $SCENE"; continue ;;
  esac
  SCENE_MVS="$ROOT/runs/$SCENE/scene.mvs"
  GTC="$ROOT/runs/$SCENE/gt_cache"
  for L in $LEVELS; do
    WD="$ROOT/runs/$SCENE/L$L"; TAG="${SCENE}_L${L}"
    [ -d "$WD/raw_dmaps" ] || { echo "MISSING raw_dmaps for $TAG -- skip"; continue; }
    for variant in raw adj; do
      [ "$variant" = raw ] && DM="$WD/raw_dmaps" || DM="$WD"
      J="$OUT/${TAG}_confop_${variant}.json"
      if [ -s "$J" ]; then echo "skip (exists) $J"; continue; fi
      echo "=== $(date +%H:%M:%S) $TAG $variant ==="
      "$PY" "$REPO/scripts/python/EvalConfidence.py" "$DM" \
          --gt-depth-dir "$GT_DEPTH_DIR" --gt-format "$GT_FORMAT" --scene-mvs "$SCENE_MVS" \
          --gt-cache-dir "$GTC" --quiet --json "$J" \
          && echo "OK $TAG $variant" || echo "FAIL $TAG $variant"
    done
  done
done
echo "ALL_DONE $(date +%H:%M:%S)"
