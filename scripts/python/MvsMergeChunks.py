#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
Merge the meshes of the sub-scenes a scene was split into (DensifyPointCloud --sub-scene-memory) into one
mesh, the way a tiled reconstruction is assembled: each sub-scene mesh keeps only the faces whose
centroid lies in its own region of interest (the box the split gave it, stored as the sub-scene's ROI),
half-open on the upper side so a face on a shared boundary goes to exactly one sub-scene; the kept faces
are concatenated. Reconstruct each sub-scene with a margin around its box (ReconstructMesh --roi-border)
so its surface near the boundary is built with the context of both sides, then merge:

MvsMergeChunks.py scene_XXXX.mvs scene_XXXX_mesh.ply merged_mesh.ply

The two patterns name the sub-scenes (they carry the regions of interest) and their meshes, with XXXX in
place of the sub-scene number. Geometry only: texture coordinates are not carried over.
"""

import argparse
import glob
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from MvsUtils import loadMVSInterface  # noqa: E402

PLY_TYPES = {'float': '<f4', 'float32': '<f4', 'double': '<f8', 'float64': '<f8', 'uchar': 'u1', 'uint8': 'u1',
             'char': 'i1', 'int8': 'i1', 'ushort': '<u2', 'uint16': '<u2', 'short': '<i2', 'int16': '<i2',
             'uint': '<u4', 'uint32': '<u4', 'int': '<i4', 'int32': '<i4'}


def read_mesh(path):
  """Binary little-endian PLY triangle mesh: vertex x,y,z (+ fixed-size properties), face vertex_indices."""
  with open(path, 'rb') as f:
    header = []
    while True:
      line = f.readline().decode('ascii').strip()
      header.append(line)
      if line == 'end_header':
        break
    body = f.read()
  if 'format binary_little_endian 1.0' not in header:
    raise ValueError('%s: binary little-endian PLY expected' % path)
  elems, cur = [], None
  for h in header:
    t = h.split()
    if t[:1] == ['element']:
      cur = [t[1], int(t[2]), []]
      elems.append(cur)
    elif t[:1] == ['property']:
      if t[1] == 'list':
        if t[4] != 'vertex_indices':
          raise ValueError('%s: list property %s not supported' % (path, t[4]))
        cur[2] += [('n', PLY_TYPES[t[2]]), ('vertex_indices', PLY_TYPES[t[3]], (3,))]
      else:
        cur[2].append((t[2], PLY_TYPES[t[1]]))
  data, offset = {}, 0
  for name, count, fields in elems:
    dt = np.dtype(fields)
    data[name] = np.frombuffer(body, dt, count, offset)
    offset += dt.itemsize * count
  v, fc = data['vertex'], data['face']
  if (fc['n'] != 3).any():
    raise ValueError('%s: triangles expected' % path)
  return np.stack([v['x'], v['y'], v['z']], 1).astype(np.float32), fc['vertex_indices'].astype(np.uint32)


def write_mesh(path, vertices, faces):
  header = ('ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\n'
            'property float z\nelement face %d\nproperty list uchar uint vertex_indices\nend_header\n'
            % (len(vertices), len(faces)))
  rec = np.zeros(len(faces), dtype=[('n', 'u1'), ('i', '<u4', (3,))])
  rec['n'] = 3
  rec['i'] = faces
  with open(path, 'wb') as f:
    f.write(header.encode('ascii'))
    f.write(vertices.astype('<f4').tobytes())
    f.write(rec.tobytes())


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('input_scenes', help='sub-scene pattern, XXXX in place of the sub-scene number')
  parser.add_argument('input_meshes', help='sub-scene mesh pattern, XXXX in place of the sub-scene number')
  parser.add_argument('output_mesh')
  args = parser.parse_args()
  for pat in (args.input_scenes, args.input_meshes):
    if 'XXXX' not in os.path.basename(pat):
      sys.exit('%s: the pattern must contain XXXX' % pat)
  pattern = os.path.abspath(args.input_meshes)
  files = sorted(glob.glob(pattern.replace('XXXX', '[0-9][0-9][0-9][0-9]')))
  if not files:
    sys.exit('no sub-scene mesh matches %s' % pattern)
  name_re = re.compile(re.escape(os.path.basename(pattern)).replace('XXXX', r'(\d{4})') + '$')
  all_vertices, all_faces, offset = [], [], 0
  for path in files:
    match = name_re.match(os.path.basename(path))
    if not match:
      continue
    scene = os.path.abspath(args.input_scenes).replace('XXXX', match.group(1))
    obb = loadMVSInterface(scene).get('obb')
    if obb is None:
      sys.exit('%s: the sub-scene has no region of interest' % scene)
    rot = np.asarray(obb['rot'], np.float64).reshape(3, 3)
    lo, hi = np.asarray(obb['pt_min'], np.float64), np.asarray(obb['pt_max'], np.float64)
    vertices, faces = read_mesh(path)
    centroids = vertices[faces].astype(np.float64).mean(axis=1) @ rot.T
    keep = np.all((centroids >= lo) & (centroids < hi), axis=1)
    faces = faces[keep]
    used = np.unique(faces)
    remap = np.full(len(vertices), -1, np.int64)
    remap[used] = np.arange(len(used))
    all_vertices.append(vertices[used])
    all_faces.append(remap[faces] + offset)
    offset += len(used)
    print('%s: %d of %d faces kept' % (os.path.basename(path), int(keep.sum()), len(keep)))
  vertices, faces = np.concatenate(all_vertices), np.concatenate(all_faces)
  write_mesh(args.output_mesh, vertices, faces.astype(np.uint32))
  print('merged %d sub-scenes: %d vertices, %d faces -> %s' % (len(files), len(vertices), len(faces), args.output_mesh))


if __name__ == '__main__':
  main()
