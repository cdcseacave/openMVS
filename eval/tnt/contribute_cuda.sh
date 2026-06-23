#!/usr/bin/env bash
# ============================================================================
# CALL FOR HELP: fill the CUDA column of the Metal-PatchMatch benchmark.
# ============================================================================
# You need: an NVIDIA GPU + an OpenMVS build with CUDA PatchMatch enabled
# (develop branch, -DOpenMVS_USE_CUDA=ON).
#
# You DON'T need the Tanks-and-Temples ground truth or the evaluation toolbox:
# you densify the *shared* scene.mvs we provide and send back the fused PLY +
# timing. We run the F1 evaluation on our side with the exact same GT/toolbox
# used for the Metal and CPU columns, so your result is directly comparable.
#
# ---------------------------------------------------------------------------
# Usage:
#   ./contribute_cuda.sh <path/to/DensifyPointCloud_CUDA> <Scene> <scene.mvs> [gpu_id]
#
# Example:
#   ./contribute_cuda.sh /opt/openMVS/bin/DensifyPointCloud Truck ./Truck/scene.mvs 0
#
# Then send us  <Scene>_cuda_result.tar.gz  (contains the fused PLY + metrics).
# ---------------------------------------------------------------------------
set -euo pipefail

DPC="${1:?usage: contribute_cuda.sh <DensifyPointCloud> <Scene> <scene.mvs> [gpu_id]}"
SCENE="${2:?scene name, e.g. Truck}"
MVS="${3:?path to the shared scene.mvs}"
GPU="${4:-0}"
OUT="./${SCENE}_out_cuda"

[ -x "$DPC" ] || { echo "ERROR: $DPC is not executable"; exit 1; }
[ -f "$MVS" ] || { echo "ERROR: scene.mvs not found at $MVS"; exit 1; }
mkdir -p "$OUT"

# IMPORTANT: keep these parameters byte-for-byte identical to lib.sh so the
# only difference vs the Metal/CPU columns is the backend. Do not change them.
DENSIFY_PARAMS=(
  --resolution-level 1
  --min-resolution 640
  --max-resolution 3200
  --number-views 5
  --fusion-mode 0
  --cuda-device "$GPU"
)

echo "[cuda] densifying $SCENE on GPU $GPU ..."
/usr/bin/env bash -c '
  start=$(date +%s.%N)
  "$1" "$2" "${@:4}" -w "$3" -o "$3/scene_dense.mvs" >"$3/densify.log" 2>&1
  end=$(date +%s.%N)
  echo "$end - $start" | bc
' _ "$DPC" "$MVS" "$OUT" "${DENSIFY_PARAMS[@]}" > "$OUT/wall.txt" || {
  echo "ERROR: densify failed, see $OUT/densify.log"; exit 1; }

PLY="$OUT/scene_dense.ply"; [ -f "$PLY" ] || PLY="$OUT/scene_dense.mvs.ply"
[ -f "$PLY" ] || { echo "ERROR: no fused PLY produced"; exit 1; }

WALL=$(cat "$OUT/wall.txt")
NPTS=$(awk '/^element vertex/{print $3; exit}' "$PLY")
GPUNAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo unknown)

cat > "$OUT/metrics.json" <<JSON
{ "scene": "$SCENE", "backend": "cuda", "wall_s": ${WALL:-null},
  "fused_points": ${NPTS:-0}, "gpu": "$GPUNAME", "ply": "scene_dense.ply" }
JSON

cp "$PLY" "$OUT/scene_dense.ply" 2>/dev/null || true
tar -czf "${SCENE}_cuda_result.tar.gz" -C "$OUT" scene_dense.ply metrics.json densify.log
echo
echo "=========================================================================="
echo " DONE: $SCENE on $GPUNAME -> ${NPTS} points in ${WALL}s"
echo " Please attach  ${SCENE}_cuda_result.tar.gz  to the PR thread:"
echo "   https://github.com/cdcseacave/openMVS/pull/1279"
echo "=========================================================================="
