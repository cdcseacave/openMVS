#!/usr/bin/env bash
# Package the shared densification inputs (scene.mvs + the undistorted images it
# references) into one self-contained tarball per scene, for CUDA volunteers.
#
# scene.mvs references images by RELATIVE path (../colmap/dense/images/...), so
# the archive preserves that layout. A volunteer just unpacks and runs:
#
#   tar -xzf <Scene>_mvs.tar.gz
#   ./contribute_cuda.sh /path/to/DensifyPointCloud_CUDA <Scene> <Scene>/mvs/scene.mvs
#
# Usage: ./pack_scene_mvs.sh            # SCENES from lib.sh
#        SCENES="Truck" ./pack_scene_mvs.sh
source "$(cd "$(dirname "$0")" && pwd)/lib.sh"

DIST="$TNT_ROOT/dist"
mkdir -p "$DIST"

for SCENE in $SCENES; do
  MVS="$SCENE/mvs/scene.mvs"
  IMG="$SCENE/colmap/dense/images"
  [ -f "$TNT_ROOT/$MVS" ] || { warn "[$SCENE] no scene.mvs, skipping"; continue; }
  [ -d "$TNT_ROOT/$IMG" ] || { warn "[$SCENE] no undistorted images, skipping"; continue; }
  OUT="$DIST/${SCENE}_mvs.tar.gz"
  log "[$SCENE] packing scene.mvs + $(ls "$TNT_ROOT/$IMG" | wc -l | tr -d ' ') images"
  tar -czf "$OUT" -C "$TNT_ROOT" "$MVS" "$IMG"
  log "[$SCENE] -> $OUT ($(du -h "$OUT" | cut -f1))"
done

echo
log "Done. Upload $DIST/*.tar.gz and put the links in PR_COMMENT_DRAFT.md."
ls -lh "$DIST"/*.tar.gz 2>/dev/null
