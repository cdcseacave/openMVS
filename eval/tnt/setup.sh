#!/usr/bin/env bash
# One-time setup for the local (Metal + CPU) benchmark on Apple Silicon.
#   1. checks/builds the OpenMVS apps we need (DensifyPointCloud, InterfaceCOLMAP)
#   2. installs colmap (CPU SfM) via Homebrew
#   3. clones the official TnT evaluation toolbox + python deps
# It does NOT download the datasets (large) — see download_tnt.sh.
source "$(cd "$(dirname "$0")" && pwd)/lib.sh"
mkdir -p "$TNT_ROOT"

log "[1/3] OpenMVS apps"
[ -x "$DPC" ] || warn "DensifyPointCloud missing — build with ~/dev/build_openmvs_cpu.sh"
if [ ! -x "$INTERFACE_COLMAP" ]; then
  warn "InterfaceCOLMAP not built. Building it now..."
  cmake --build "$HOME/dev/openMVS-metal/make" --target InterfaceCOLMAP -j || \
    die "InterfaceCOLMAP build failed"
fi

log "[2/3] colmap"
command -v colmap >/dev/null || brew install colmap

log "[3/3] TnT evaluation toolbox + python venv"
if [ ! -d "$TNT_TOOLBOX" ]; then
  git clone --depth 1 https://github.com/isl-org/TanksAndTemples "$TNT_TOOLBOX"
fi
# open3d has no wheel for Homebrew's python@3.14; use a dedicated 3.12 venv.
VENV="$TNT_ROOT/venv"
if [ ! -x "$VENV/bin/python" ]; then
  PY312="$(command -v python3.12 || true)"
  [ -n "$PY312" ] || die "python3.12 needed for open3d (brew install python@3.12)"
  "$PY312" -m venv "$VENV"
  "$VENV/bin/pip" install --quiet --upgrade pip
fi
"$VENV/bin/pip" install --quiet open3d numpy matplotlib || \
  warn "pip install open3d failed in venv $VENV"

# The toolbox targets open3d 0.9 (no wheel for py3.12); patch it to the modern
# open3d 0.1x API: o3d.registration -> o3d.pipelines.registration, drop the
# removed RANSACConvergenceCriteria.max_validation, add the new `checkers` arg.
REG="$TNT_TOOLBOX/python_toolbox/evaluation/registration.py"
if [ -f "$REG" ] && grep -q 'o3d\.registration\.' "$REG"; then
  log "patching TnT toolbox for open3d 0.1x API"
  sed -i '' 's/o3d\.registration\./o3d.pipelines.registration./g' "$REG"
  sed -i '' '/rr\.max_validation = 100000/d' "$REG"
  # insert empty checkers list before the criteria arg of the ransac call
  perl -0pi -e 's/(TransformationEstimationPointToPoint\(True\),\n\s*6,\n)(\s*rr,)/$1        [],\n$2/' "$REG"
fi

log "Setup done. Next: ./download_tnt.sh, then ./run_local.sh"
