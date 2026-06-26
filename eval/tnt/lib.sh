# shellcheck shell=bash
# Shared configuration for the Tanks-and-Temples Metal/CPU/CUDA benchmark.
# Source this from the other scripts: `source "$(dirname "$0")/lib.sh"`.
set -euo pipefail

# --- Paths -------------------------------------------------------------------
TNT_ROOT="${TNT_ROOT:-$HOME/dev/tnt-bench}"          # working root for the bench
OPENMVS_BIN="${OPENMVS_BIN:-$HOME/dev/openMVS-metal/make/bin}"
DPC="${DPC:-$OPENMVS_BIN/DensifyPointCloud}"
INTERFACE_COLMAP="${INTERFACE_COLMAP:-$OPENMVS_BIN/InterfaceCOLMAP}"
export DYLD_LIBRARY_PATH="$OPENMVS_BIN:${DYLD_LIBRARY_PATH:-}"

# Official Tanks-and-Temples evaluation toolbox (isl-org/TanksAndTemples).
TNT_TOOLBOX="${TNT_TOOLBOX:-$TNT_ROOT/TanksAndTemples}"
# Dedicated Python 3.12 venv (open3d has no wheel for Homebrew's python@3.14).
TNT_PYTHON="${TNT_PYTHON:-$TNT_ROOT/venv/bin/python}"
[ -x "$TNT_PYTHON" ] || TNT_PYTHON="$(command -v python3)"

# --- Scenes ------------------------------------------------------------------
# Training subset = the scenes with PUBLIC ground truth (F1 computable locally).
# Override with: SCENES="Barn Truck" ./run_local.sh
SCENES="${SCENES:-Barn Ignatius Meetingroom Truck}"
# Full training set if you want the exhaustive table:
#   SCENES="Barn Caterpillar Church Courthouse Ignatius Meetingroom Truck"

# --- Fixed densification parameters (IDENTICAL across all backends) ----------
# These are the only knobs that affect quality; they MUST match for every
# backend so the table compares the backend, not the configuration.
DENSIFY_PARAMS=(
  --resolution-level 1
  --min-resolution 640
  --max-resolution 3200
  --number-views 5
  --fusion-mode 0
  --process-priority 0
)

log()  { printf '\033[1;34m[tnt]\033[0m %s\n' "$*" >&2; }
warn() { printf '\033[1;33m[tnt][warn]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[tnt][err]\033[0m %s\n' "$*" >&2; exit 1; }

# Count vertices in a binary/ascii PLY without loading it fully.
ply_vertex_count() {
  awk '
    /^element vertex/ { print $3; exit }
    /^end_header/     { exit }
  ' "$1" 2>/dev/null || echo 0
}
