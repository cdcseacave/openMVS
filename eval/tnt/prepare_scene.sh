#!/usr/bin/env bash
# Produce the SHARED densification input `scene.mvs` for one TnT scene.
#
# This is the methodological keystone: SfM is run ONCE here, and the resulting
# scene.mvs (+ undistorted images) is the artifact every backend densifies.
# Metal, CPU and the community CUDA run all start from THIS file, so the table
# compares the densification backend and nothing else.
#
# Usage: ./prepare_scene.sh <Scene>
#   expects images in  $TNT_ROOT/<Scene>/images/
#   writes             $TNT_ROOT/<Scene>/mvs/scene.mvs  (+ undistorted images)
#
# Requires: colmap (CPU is fine on Apple Silicon) and InterfaceCOLMAP.
source "$(cd "$(dirname "$0")" && pwd)/lib.sh"

SCENE="${1:?usage: prepare_scene.sh <Scene>}"
SDIR="$TNT_ROOT/$SCENE"
IMG="$SDIR/images"
WORK="$SDIR/colmap"
MVS="$SDIR/mvs"

[ -d "$IMG" ] || die "no images at $IMG (run download_tnt.sh first)"
command -v colmap >/dev/null || die "colmap not found (brew install colmap)"
[ -x "$INTERFACE_COLMAP" ] || die "InterfaceCOLMAP not built at $INTERFACE_COLMAP"

mkdir -p "$WORK" "$MVS"

if [ ! -f "$WORK/sparse/0/cameras.bin" ]; then
  log "[$SCENE] COLMAP feature extraction"
  colmap feature_extractor --database_path "$WORK/db.db" --image_path "$IMG" \
    --ImageReader.single_camera 0 --FeatureExtraction.use_gpu 0
  log "[$SCENE] COLMAP matching (sequential — TnT frames are ordered video)"
  colmap sequential_matcher --database_path "$WORK/db.db" \
    --FeatureMatching.use_gpu 0 --SequentialMatching.overlap 10
  log "[$SCENE] COLMAP mapper (incremental SfM)"
  mkdir -p "$WORK/sparse"
  colmap mapper --database_path "$WORK/db.db" --image_path "$IMG" --output_path "$WORK/sparse"
fi

log "[$SCENE] COLMAP image undistortion"
colmap image_undistorter --image_path "$IMG" --input_path "$WORK/sparse/0" \
  --output_path "$WORK/dense" --output_type COLMAP

log "[$SCENE] InterfaceCOLMAP -> scene.mvs"
"$INTERFACE_COLMAP" -i "$WORK/dense" -o "$MVS/scene.mvs" --image-folder "$WORK/dense/images"

log "[$SCENE] DONE -> $MVS/scene.mvs"
log "Share this file (+ $MVS/*.dmap none yet, + undistorted images dir) with CUDA volunteers."
