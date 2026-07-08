#!/bin/bash
# gt_bench/run_scene.sh <scene> <reslevel>
#
# Runs the full per-scene/per-resolution GT-bench pipeline for one OpenMVS scene:
#   1) estimate depth-maps (raw confidence)         -> snapshot to raw_dmaps/
#   2) adjust confidence in place (postprocess=4)    -> overwrites dmaps in WD
#   3) fuse adjusted dmaps: w0 (rescue off) + w3 (rescue on, default fFusePriorWeight=3.0)
#   4) evaluate: conf_raw, conf_adj (EvalConfidence.py GT mode),
#                fuse_w0, fuse_w3 (EvalFusionGT.py for bmvs, eth3d_eval.sh for eth3d)
#   5) write a timing.json summarizing wall/compute times for this scene/res
#
# Writes results/<scene>_L<r>_{conf_raw,conf_adj,fuse_w0,fuse_w3,timing}.json
#
# Idempotent: each stage is skipped if its output/marker already exists, unless FORCE=1
# (FORCE=1 re-runs every stage for this scene/res and regenerates every JSON).
#
# ---- Workdir layout / path-resolution design (see task-7-report.md for full derivation) ----
# scene.mvs (written by Task 3's import_scenes.sh) stores image paths RELATIVE TO THE
# WORKING FOLDER used at import time, which was runs/<scene>/ (2 levels above the data
# root, e.g. "../../blendedmvs/<id>/images/00000000.jpg" or "../../eth3d/<name>/images/...").
# DensifyPointCloud resolves BOTH the scene's image paths AND its own depth-map output
# filenames against the SAME single -w/--working-folder value (libs/Common/Common.h's
# MAKE_PATH / MAKE_PATH_FULL macros), so per-resolution isolation (this script uses
# runs/<scene>/L<r>/ as the working folder, so L1/L0/L2/L3 dmaps for the same scene never
# collide -- dmap filenames are only keyed by view ID, not by resolution) would normally
# break image-path resolution by exactly one directory level ("../../blendedmvs/..." from
# runs/<scene>/L<r>/ normalizes to runs/blendedmvs/... instead of gt_bench/blendedmvs/...).
# Fixed with two ONE-TIME, repo-root-relative symlinks created below (idempotent):
#   runs/blendedmvs -> ../blendedmvs   and   runs/eth3d -> ../eth3d
# which make the lexical ".." normalization land in the right place regardless of which
# scene/resolution subdirectory -w points at. Verified empirically (see task-7-report.md).
set -euo pipefail

usage() { echo "usage: run_scene.sh <scene> <reslevel>" >&2; exit 1; }
[ $# -ge 2 ] || usage
SCENE="$1"; L="$2"

ROOT=/home/ubuntu/virginia/gt_bench
REPO=/home/ubuntu/openMVS
# WD/RES are overridable (Task 11 re-run: point WD at a fresh workdir seeded from the
# original raw_dmaps/ snapshot -- kept as a SIBLING of runs/<scene>/L<r> under runs/<scene>/
# so the existing runs/{blendedmvs,eth3d} path-resolution symlinks still apply unchanged --
# and RES at a separate results dir so the committed baseline JSONs are never touched).
# Both default to the original hardcoded locations, so every existing caller is unaffected.
WD="${WD:-$ROOT/runs/$SCENE/L$L}"
RES="${RES:-$ROOT/results}"
SCENE_MVS="$ROOT/runs/$SCENE/scene.mvs"
BINRAW="$REPO/make/bin/Release/DensifyPointCloud"
BIN=(env LD_LIBRARY_PATH=/usr/local/cuda/lib64 "$BINRAW")
PY=/home/ubuntu/miniconda3/bin/python
MAXRES=3200
FORCE="${FORCE:-0}"
# Task 11 re-run knobs: the acceptance gates only need adjust/fuse WALL TIMES + conf_adj ROC,
# not conf_raw (identical to baseline, code doesn't touch raw confidence) or fusion GT
# completeness (fusion's own math is untouched by Task 11; only the prior's cache/threading
# changed, and courtyard L2 byte-equivalence already proves adjusted dmaps are unchanged, so
# fused clouds are provably unchanged too). Default 0 preserves the full original pipeline.
SKIP_CONF_RAW="${SKIP_CONF_RAW:-0}"
SKIP_FUSE_EVAL="${SKIP_FUSE_EVAL:-0}"

[ -x "$BINRAW" ] || { echo "error: $BINRAW not found/executable (build it first)" >&2; exit 1; }
[ -f "$SCENE_MVS" ] || { echo "error: $SCENE_MVS not found" >&2; exit 1; }

mkdir -p "$WD" "$RES"

# one-time path-resolution symlinks (see header comment); safe/no-op if already present
[ -e "$ROOT/runs/blendedmvs" ] || ln -s ../blendedmvs "$ROOT/runs/blendedmvs"
[ -e "$ROOT/runs/eth3d" ]      || ln -s ../eth3d      "$ROOT/runs/eth3d"

# ---- resolve GT source + format from scene-name convention (bmvs_<first8>, eth3d_<name>) ----
case "$SCENE" in
  bmvs_*)
    SHORT="${SCENE#bmvs_}"
    FULLID="$(awk -v s="$SHORT" '!/^#/ && $1 ~ ("^" s) {print $1; exit}' "$REPO/gt_bench/scenes_blendedmvs.txt")"
    [ -n "$FULLID" ] || { echo "error: cannot map $SCENE -> full BlendedMVS scene id via scenes_blendedmvs.txt" >&2; exit 1; }
    GT_FORMAT=blendedmvs
    GT_DEPTH_DIR="$ROOT/blendedmvs/$FULLID"
    GT_MESH="$ROOT/blendedmvs/$FULLID/textured_mesh"
    ;;
  eth3d_*)
    NAME="${SCENE#eth3d_}"
    GT_FORMAT=eth3d
    GT_DEPTH_DIR="$ROOT/eth3d/$NAME"
    GT_SCENE_DIR="$ROOT/eth3d/$NAME"
    ;;
  *)
    echo "error: unrecognized scene name '$SCENE' (expected bmvs_* or eth3d_*)" >&2; exit 1
    ;;
esac
[ -d "$GT_DEPTH_DIR" ] || { echo "error: GT dir not found: $GT_DEPTH_DIR" >&2; exit 1; }

TAG="${SCENE}_L${L}"
log(){ echo "[run_scene $TAG] $*"; }

# ---- GPU contention check (shared box) ----
wait_for_gpu() {
  local waited=0 cap=1800 used
  while :; do
    used=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | sort -n | tail -1)
    [ -z "$used" ] && return 0   # no nvidia-smi / no GPU visible -- don't block
    if [ "$used" -le 30000 ]; then return 0; fi
    if [ "$waited" -ge "$cap" ]; then
      log "warning: GPU still busy (${used} MiB used) after ${cap}s wait, proceeding anyway"
      return 0
    fi
    log "GPU busy (${used} MiB used > 30000 MiB threshold), waiting 30s..."
    sleep 30; waited=$((waited + 30))
  done
}

newest_log() { ls -t "$WD"/DensifyPointCloud-*.log 2>/dev/null | head -1; }
elapsed_s() { python3 -c "print(f'{$2-$1:.3f}')"; }

json_exists() { [ -s "$RES/${TAG}_$1.json" ]; }

COMMON_OPTS=(--resolution-level "$L" --max-resolution "$MAXRES" -v 2)

# NOTE: the DensifyPointCloud invocations below are NOT wrapped in a command
# substitution (that would silently swallow -- and mix stage timing/log-path
# tracking with -- the binary's own console output, which is copious at -v 2).
# Wall time is measured with plain date(1) calls in the current shell instead.
#
# CRITICAL (review fix): stages 2/3a/3b MUST pass --geometric-iters 0. With the default
# (2), SceneDensify.cpp's geometric-consistency loop (:2570) unconditionally re-estimates
# EVERY image regardless of the on-disk dmap cache (the cache-reuse check at :2701 requires
# nEstimationGeometricIter < 0, which is false inside the loop) and then REPLACES each
# cached dmap with the freshly re-estimated geo.dmap (:2601-2603). Without this flag,
# stage 2 adjusted freshly RE-ESTIMATED dmaps (breaking the paired raw-vs-adjusted
# comparison against stage 1's snapshot) and stages 3a/3b re-estimated yet again, so
# fusion never saw the adjusted confidence at all. Stage 1 keeps the default so dmaps are
# estimated WITH geometric consistency exactly once; every later stage reuses them from
# cache untouched (verified via dmap checksums across stages -- see task-7-report.md).

# ================= Stage 1: estimate raw depth-maps =================
MARK_EST="$WD/.time_estimate"
if [ "$FORCE" = 1 ] || [ ! -f "$MARK_EST" ] || [ ! -d "$WD/raw_dmaps" ]; then
  log "stage 1/4: estimating depth-maps (raw confidence)"
  wait_for_gpu
  t0=$(date +%s.%N)
  "${BIN[@]}" "$SCENE_MVS" -w "$WD" "${COMMON_OPTS[@]}" --fusion-mode 1 --postprocess-dmaps 0 -o est.mvs
  t1=$(date +%s.%N)
  WALL=$(elapsed_s "$t0" "$t1")
  # atomic snapshot (review fix): populate a .tmp dir first, then a single rename -- a crash
  # mid-copy can never leave a half-populated raw_dmaps/ that the idempotency check (which
  # keys on the FINAL name) would mistake for a complete snapshot.
  rm -rf "$WD/raw_dmaps.tmp" "$WD/raw_dmaps"
  mkdir -p "$WD/raw_dmaps.tmp"
  cp "$WD"/depth*.dmap "$WD/raw_dmaps.tmp/"
  mv "$WD/raw_dmaps.tmp" "$WD/raw_dmaps"
  echo "$WALL" > "$MARK_EST"
  log "stage 1/4 done: ${WALL}s, snapshot -> raw_dmaps/"
else
  log "stage 1/4: skip (already done, $(cat "$MARK_EST")s)"
fi

# ================= Stage 2: adjust confidence in place =================
MARK_ADJ="$WD/.time_adjust"
if [ "$FORCE" = 1 ] || [ ! -f "$MARK_ADJ" ]; then
  log "stage 2/4: adjusting confidence in place"
  wait_for_gpu
  t0=$(date +%s.%N)
  "${BIN[@]}" "$SCENE_MVS" -w "$WD" "${COMMON_OPTS[@]}" --geometric-iters 0 --fusion-mode 1 --postprocess-dmaps 4 -o adj.mvs
  t1=$(date +%s.%N)
  WALL=$(elapsed_s "$t0" "$t1")
  LAST_LOG=$(newest_log)
  ADJLINE=$(grep -a "Confidence-maps adjusted:" "$LAST_LOG" | tail -1 || true)
  [ -n "$ADJLINE" ] || { echo "error: 'Confidence-maps adjusted:' line not found in $LAST_LOG" >&2; exit 1; }
  ADJVALS=$(python3 -c "
import re, sys
line = '''$ADJLINE'''
# the optional trailing '; <X>s prior / <Y>s confirmation' split (added with the Task-9
# confirmation-sweep rewrite) is tolerated but not required, so the parser works with both
# pre- and post-split binaries
m = re.search(r'Confidence-maps adjusted:\s*(\d+)\s*depth-maps\s*\(([^;]*);\s*([0-9.eE+-]+)s prior\+confirmation compute,\s*([0-9.eE+-]+)ms/map avg(?:;[^)]*)?\)', line)
if not m:
    print('error: could not parse adjust line: %r' % line, file=sys.stderr); sys.exit(1)
print(m.group(1), m.group(3), m.group(4))
")
  read -r N_DMAPS COMPUTE_S MS_PER_MAP <<< "$ADJVALS"
  echo "$WALL $COMPUTE_S $MS_PER_MAP $N_DMAPS" > "$MARK_ADJ"
  log "stage 2/4 done: wall=${WALL}s compute=${COMPUTE_S}s ms/map=${MS_PER_MAP} n=${N_DMAPS}"
else
  log "stage 2/4: skip (already done)"
fi

# ================= Stage 3: fuse adjusted dmaps (w0, w3) =================
MARK_W0="$WD/.time_fuse_w0"
if [ "$FORCE" = 1 ] || [ ! -f "$MARK_W0" ] || [ ! -s "$WD/cloud_w0.ply" ]; then
  log "stage 3a/4: fusing (rescue OFF, Fuse Prior Weight = 0)"
  printf 'Fuse Prior Weight = 0\n' > "$WD/fuse_w0.cfg"
  t0=$(date +%s.%N)
  "${BIN[@]}" "$SCENE_MVS" -w "$WD" "${COMMON_OPTS[@]}" --geometric-iters 0 --postprocess-dmaps 0 \
      --dense-config-file fuse_w0.cfg -o cloud_w0.mvs
  t1=$(date +%s.%N)
  WALL=$(elapsed_s "$t0" "$t1")
  [ -s "$WD/cloud_w0.ply" ] || { echo "error: expected $WD/cloud_w0.ply not produced" >&2; exit 1; }
  echo "$WALL" > "$MARK_W0"
  log "stage 3a/4 done: ${WALL}s"
else
  log "stage 3a/4: skip (already done, $(cat "$MARK_W0")s)"
fi

MARK_W3="$WD/.time_fuse_w3"
if [ "$FORCE" = 1 ] || [ ! -f "$MARK_W3" ] || [ ! -s "$WD/cloud_w3.ply" ]; then
  log "stage 3b/4: fusing (rescue ON, default Fuse Prior Weight = 3.0)"
  t0=$(date +%s.%N)
  "${BIN[@]}" "$SCENE_MVS" -w "$WD" "${COMMON_OPTS[@]}" --geometric-iters 0 --postprocess-dmaps 0 -o cloud_w3.mvs
  t1=$(date +%s.%N)
  WALL=$(elapsed_s "$t0" "$t1")
  [ -s "$WD/cloud_w3.ply" ] || { echo "error: expected $WD/cloud_w3.ply not produced" >&2; exit 1; }
  echo "$WALL" > "$MARK_W3"
  log "stage 3b/4 done: ${WALL}s"
else
  log "stage 3b/4: skip (already done, $(cat "$MARK_W3")s)"
fi

# ================= Stage 4: evaluations =================
GT_CACHE_ARGS=()
if [ "$GT_FORMAT" = eth3d ]; then
  GT_CACHE_ARGS=(--gt-cache-dir "$ROOT/runs/$SCENE/gt_cache")
fi

if [ "$SKIP_CONF_RAW" = 1 ]; then
  log "stage 4/4: skip conf_raw (SKIP_CONF_RAW=1)"
elif [ "$FORCE" = 1 ] || ! json_exists conf_raw; then
  log "stage 4/4: EvalConfidence (raw)"
  "$PY" "$REPO/scripts/python/EvalConfidence.py" "$WD/raw_dmaps" \
      --gt-depth-dir "$GT_DEPTH_DIR" --gt-format "$GT_FORMAT" --scene-mvs "$SCENE_MVS" \
      "${GT_CACHE_ARGS[@]}" --json "$RES/${TAG}_conf_raw.json"
else
  log "stage 4/4: skip conf_raw (exists)"
fi

if [ "$FORCE" = 1 ] || ! json_exists conf_adj; then
  log "stage 4/4: EvalConfidence (adjusted)"
  "$PY" "$REPO/scripts/python/EvalConfidence.py" "$WD" \
      --gt-depth-dir "$GT_DEPTH_DIR" --gt-format "$GT_FORMAT" --scene-mvs "$SCENE_MVS" \
      "${GT_CACHE_ARGS[@]}" --json "$RES/${TAG}_conf_adj.json"
else
  log "stage 4/4: skip conf_adj (exists)"
fi

eval_fusion() {
  local wtag="$1" ply="$2"
  if [ "$SKIP_FUSE_EVAL" = 1 ]; then
    log "stage 4/4: skip fuse_$wtag GT eval (SKIP_FUSE_EVAL=1)"
  elif [ "$FORCE" = 1 ] || ! json_exists "fuse_$wtag"; then
    log "stage 4/4: fusion eval ($wtag)"
    if [ "$GT_FORMAT" = blendedmvs ]; then
      "$PY" "$REPO/scripts/python/EvalFusionGT.py" "$ply" --gt-mesh "$GT_MESH" \
          --n-samples 2000000 --json "$RES/${TAG}_fuse_${wtag}.json"
    else
      bash "$REPO/gt_bench/eth3d_eval.sh" "$ply" "$GT_SCENE_DIR" "$RES/${TAG}_fuse_${wtag}.json"
    fi
  else
    log "stage 4/4: skip fuse_$wtag (exists)"
  fi
}
eval_fusion w0 "$WD/cloud_w0.ply"
eval_fusion w3 "$WD/cloud_w3.ply"

# optional extra fusion weight for the w2-vs-w3 default re-check (DO_W2=1); default off = no change
if [ "${DO_W2:-0}" = 1 ]; then
  if [ "$FORCE" = 1 ] || [ ! -s "$WD/cloud_w2.ply" ]; then
    printf 'Fuse Prior Weight = 2\n' > "$WD/fuse_w2.cfg"
    "${BIN[@]}" "$SCENE_MVS" -w "$WD" "${COMMON_OPTS[@]}" --geometric-iters 0 --postprocess-dmaps 0 \
        --dense-config-file fuse_w2.cfg -o cloud_w2.mvs
  fi
  eval_fusion w2 "$WD/cloud_w2.ply"
fi

# ================= timing.json =================
if [ "$FORCE" = 1 ] || ! json_exists timing; then
  EST_WALL=$(cat "$MARK_EST")
  read -r ADJ_WALL ADJ_COMPUTE ADJ_MS_PER_MAP N_DMAPS < "$MARK_ADJ"
  W0_WALL=$(cat "$MARK_W0")
  W3_WALL=$(cat "$MARK_W3")
  python3 -c "
import json
out = {
    'estimation_wall_s': float('$EST_WALL'),
    'adjust_wall_s': float('$ADJ_WALL'),
    'adjust_compute_s': float('$ADJ_COMPUTE'),
    'adjust_ms_per_map': float('$ADJ_MS_PER_MAP'),
    'fusion_w0_wall_s': float('$W0_WALL'),
    'fusion_w3_wall_s': float('$W3_WALL'),
    'n_dmaps': int('$N_DMAPS'),
}
with open('$RES/${TAG}_timing.json', 'w') as f:
    json.dump(out, f, indent=2)
print(json.dumps(out, indent=2))
"
  log "stage 4/4: timing.json written"
else
  log "stage 4/4: skip timing (exists)"
fi

log "done."
