#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
Dense reconstruction of a scene of any size: depth-maps, point-cloud, mesh, refined mesh and texture,
with the scene split in sub-scenes that each fit this machine's memory.

  MvsScalablePipeline.py scene.mvs

runs, in the folder of scene.mvs:
  1. DensifyPointCloud --fusion-mode 1: the depth-maps of the whole scene;
  2. DensifyPointCloud --sub-scene-memory: the scene cut into boxes whose depth-map fusion and mesh
     reconstruction each fit the memory budget (by default what this machine has available, less a
     reserve), written as subscenes/scene_XXXX.mvs;
  3. per sub-scene: DensifyPointCloud (fusion of the existing depth-maps), ReconstructMesh, RefineMesh;
  4. MvsMergeChunks.py: the sub-scene point-clouds and meshes merged, each keeping what lies in its box;
  5. TextureMesh on the merged mesh, with all the images.
A scene that fits in one box is reconstructed whole, exactly as by calling the modules one by one.
The results are scene_dense.ply, scene_dense_mesh.ply, scene_dense_mesh_refined.ply and
scene_dense_mesh_refined_texture.mvs/.ply (+ textures).

  --memory auto|off|<GB>   the budget per sub-scene (default auto: fit this machine; off: no split)
  --steps                  which of densify,mesh,refine,texture to run (default all)
  --densify/--mesh/--refine/--texture "<options>"   extra options passed to that module
  --resume                 skip a step whose output exists already
  --bin <folder>           the OpenMVS binaries (default: searched in PATH)
"""

import argparse
import glob
import os
import shlex
import shutil
import subprocess
import sys

STEPS = ('densify', 'mesh', 'refine', 'texture')
SUBSCENES = 'subscenes'


def find_bin(folder, name):
  exe = shutil.which(name, path=folder) if folder else shutil.which(name)
  if not exe:
    sys.exit('error: %s not found%s' % (name, ' in ' + folder if folder else ' in PATH (use --bin)'))
  return exe


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('scene', help='the input scene (.mvs), with its images')
  parser.add_argument('--memory', default='auto', help='memory budget per sub-scene: auto, off or GB (default auto)')
  parser.add_argument('--steps', default=','.join(STEPS), help='steps to run, among %s' % ','.join(STEPS))
  for step in STEPS:
    parser.add_argument('--' + step, default='', metavar='OPTIONS', help='extra options of the %s module' % step)
  parser.add_argument('--resume', action='store_true', help='skip a step whose output exists already')
  parser.add_argument('--bin', default=None, help='folder of the OpenMVS binaries (default: PATH)')
  parser.add_argument('--dry-run', action='store_true', help='print the commands without running them')
  args = parser.parse_args()

  steps = [s for s in args.steps.split(',') if s]
  for s in steps:
    if s not in STEPS:
      sys.exit('error: unknown step %s' % s)
  if args.memory == 'auto':
    memory = -1.0
  elif args.memory == 'off':
    memory = 0.0
  else:
    try:
      memory = float(args.memory)
    except ValueError:
      sys.exit('error: --memory must be auto, off or a number of GB')
    if memory <= 0:
      sys.exit('error: --memory must be positive')
  scene = os.path.abspath(args.scene)
  if not os.path.isfile(scene):
    sys.exit('error: %s does not exist' % scene)
  work = os.path.dirname(scene)
  stem = os.path.splitext(os.path.basename(scene))[0]
  extra = {s: shlex.split(getattr(args, s)) for s in STEPS}
  densify, reconstruct, refine, texture = (find_bin(args.bin, m) for m in
                                            ('DensifyPointCloud', 'ReconstructMesh', 'RefineMesh', 'TextureMesh'))

  def run(cmd, output=None):
    """run one module in the scene folder (OpenMVS resolves every path relative to -w)"""
    if args.resume and output and os.path.exists(os.path.join(work, output)):
      print('# skip (exists): %s' % output, flush=True)
      return
    print('# ' + ' '.join(cmd), flush=True)
    if args.dry_run:
      return
    if subprocess.run(cmd, cwd=work).returncode != 0:
      sys.exit('error: %s failed' % os.path.basename(cmd[0]))

  def module(exe, step, inp, out, *opts):
    run([exe, '-w', work, '-i', inp, '-o', out, *opts, *extra[step]], os.path.splitext(out)[0] + '.ply')

  def reconstruct_scene(base):
    """mesh and refine one (sub-)scene whose dense point-cloud is <base>_dense.mvs; returns the final mesh"""
    mesh = base + '_dense_mesh'
    if 'mesh' in steps:
      module(reconstruct, 'mesh', base + '_dense.mvs', mesh + '.mvs')
    if 'refine' in steps:
      module(refine, 'refine', base + '_dense.mvs', mesh + '_refined.mvs', '-m', mesh + '.ply')
      mesh += '_refined'
    return mesh

  chunks = []
  if memory != 0 and 'densify' in steps:
    if not (args.resume and glob.glob(os.path.join(work, SUBSCENES, 'scene_[0-9][0-9][0-9][0-9].mvs'))):
      # the depth-maps of the whole scene, then the split they predict the memory of
      run([densify, '-w', work, '-i', stem + '.mvs', '--fusion-mode', '1', *extra['densify']])
      os.makedirs(os.path.join(work, SUBSCENES), exist_ok=True)
      for old in glob.glob(os.path.join(work, SUBSCENES, 'scene_[0-9][0-9][0-9][0-9].mvs')):
        os.remove(old)  # a previous split's boxes: their number can differ
      run([densify, '-w', work, '-i', stem + '.mvs', '-o', os.path.join(SUBSCENES, 'scene.mvs'),
           '--sub-scene-memory', str(memory)])
    chunks = sorted(glob.glob(os.path.join(work, SUBSCENES, 'scene_[0-9][0-9][0-9][0-9].mvs')))
    if args.dry_run:
      chunks = [os.path.join(work, SUBSCENES, 'scene_0000.mvs'), os.path.join(work, SUBSCENES, 'scene_0001.mvs')]
    if len(chunks) > 1:
      print('# %d sub-scenes' % len(chunks), flush=True)
    else:
      print('# the scene fits the memory budget: not split', flush=True)
      if not args.dry_run and not os.listdir(os.path.join(work, SUBSCENES)):
        os.rmdir(os.path.join(work, SUBSCENES))
  # the existing depth-maps are only fused (no geometric pass over them again)
  fuse = ['--geometric-iters', '0'] if memory != 0 else []

  if len(chunks) <= 1:
    if 'densify' in steps:
      module(densify, 'densify', stem + '.mvs', stem + '_dense.mvs', *fuse)
    mesh = reconstruct_scene(stem)
  else:
    meshes = None
    for chunk in chunks:
      base = os.path.join(SUBSCENES, os.path.splitext(os.path.basename(chunk))[0])
      module(densify, 'densify', base + '.mvs', base + '_dense.mvs', *fuse)
      meshes = reconstruct_scene(base)
    merge = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'MvsMergeChunks.py'),
             os.path.join(SUBSCENES, 'scene_XXXX.mvs')]
    run([*merge, os.path.join(SUBSCENES, 'scene_XXXX_dense.ply'), stem + '_dense.ply'], stem + '_dense.ply')
    suffix = os.path.basename(meshes)[len('scene_0000'):]
    for name in sorted({'_dense_mesh', suffix}) if 'mesh' in steps else ():
      run([*merge, os.path.join(SUBSCENES, 'scene_XXXX' + name + '.ply'), stem + name + '.ply'], stem + name + '.ply')
    mesh = stem + suffix
  if 'texture' in steps:
    # the merged mesh is textured from all the images
    module(texture, 'texture', stem + '.mvs' if len(chunks) > 1 else stem + '_dense.mvs', mesh + '_texture.mvs',
           '-m', mesh + '.ply')


if __name__ == '__main__':
  main()
