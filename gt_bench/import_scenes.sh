#!/bin/bash
# Imports all GT-benchmark scenes (BlendedMVS + ETH3D) into OpenMVS projects under
# /home/ubuntu/virginia/gt_bench/runs/<scene>/scene.mvs
#
# Idempotent: symlinks are only created if missing; InterfaceMVSNet/InterfaceCOLMAP
# overwrite scene.mvs unconditionally on every run, so re-running is safe.
#
# Naming convention (used by every later task): bmvs_<first8ofSceneID>, eth3d_<name>
#
# Path-resolution notes (verified by reading source + inspecting the produced scene.mvs):
#  - InterfaceMVSNet has NO --image-folder flag; it hardcodes an "images/" subfolder
#    name under -i. BlendedMVS ships that folder as "blended_images/", so we symlink
#    <scene>/images -> blended_images before importing.
#  - InterfaceCOLMAP wants "<-i>/sparse/{cameras,images,points3D}.{txt,bin}". ETH3D's
#    dslr_calibration_undistorted/ already has exactly cameras.txt/images.txt/points3D.txt,
#    so a single directory symlink <scene>/colmap/sparse -> ../dslr_calibration_undistorted
#    is enough (no per-file symlinks needed). --image-folder is pointed directly (absolute)
#    at <scene>/images/, which already contains the dslr_images_undistorted/ subfolder that
#    ETH3D's images.txt names are prefixed with.
#  - Regardless of importer, MVS::Scene::Save (libs/MVS/Scene.cpp) unconditionally
#    rewrites every image path as MAKE_PATH_REL(WORKING_FOLDER_FULL, ...) before writing
#    scene.mvs -- so BOTH importers end up storing image paths RELATIVE TO THE WORKING
#    FOLDER (-w), which we always set equal to the output directory. Confirmed empirically:
#    a BlendedMVS scene.mvs image name is "../../blendedmvs/<id>/images/00000000.jpg" (goes
#    through the per-scene "images" symlink above, so that symlink must stay in place), an
#    ETH3D one is "../../eth3d/<name>/images/dslr_images_undistorted/DSC_0323.JPG". Later
#    tools (Task 7's DensifyPointCloud runner) MUST be run with -w <this workdir> (or cwd =
#    this workdir) for these relative paths to resolve.
#
# In addition, a runs/<scene>/images symlink to the scene's source image folder is created
# in every workdir for uniformity/convenience, even though scene.mvs itself does not depend
# on it for path resolution.
set -euo pipefail

ROOT=/home/ubuntu/virginia/gt_bench
REPO=/home/ubuntu/openMVS
BIN=$REPO/make/bin/Release
export LD_LIBRARY_PATH=/usr/local/cuda/lib64

# --- BlendedMVS (InterfaceMVSNet) ---
grep -v '^#' "$REPO/gt_bench/scenes_blendedmvs.txt" | awk '{print $1}' | while read -r id; do
  s=$ROOT/blendedmvs/$id
  out=$ROOT/runs/bmvs_${id:0:8}
  mkdir -p "$out"

  [ -e "$s/images" ] || ln -s blended_images "$s/images"
  [ -e "$out/images" ] || ln -s "$s/blended_images" "$out/images"

  "$BIN/InterfaceMVSNet" -i "$s" -o "$out/scene.mvs" -w "$out"
done

# --- ETH3D (InterfaceCOLMAP) ---
grep -v '^#' "$REPO/gt_bench/scenes_eth3d.txt" | awk '{print $1}' | while read -r name; do
  s=$ROOT/eth3d/$name
  out=$ROOT/runs/eth3d_$name
  mkdir -p "$out"

  mkdir -p "$s/colmap"
  [ -e "$s/colmap/sparse" ] || ln -s ../dslr_calibration_undistorted "$s/colmap/sparse"
  [ -e "$out/images" ] || ln -s "$s/images" "$out/images"

  "$BIN/InterfaceCOLMAP" -i "$s/colmap" -o "$out/scene.mvs" -w "$out" --image-folder "$s/images/"
done

echo "Done. Imported scenes:"
ls -d "$ROOT"/runs/bmvs_* "$ROOT"/runs/eth3d_* 2>/dev/null
