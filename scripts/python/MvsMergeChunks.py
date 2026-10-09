#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
Merge the meshes or the point-clouds of the sub-scenes a scene was split into (DensifyPointCloud
--sub-scene-memory) into one, the way a tiled reconstruction is assembled: each sub-scene mesh keeps only
the faces whose centroid lies in its own region of interest (the box the split gave it, stored as the
sub-scene's ROI), each point-cloud the points inside it, half-open on the upper side so an element on a
shared boundary goes to exactly one sub-scene; the kept elements are concatenated with all their vertex
properties (colors, normals, ...). The boundaries are not stitched; a margin around each box
(ReconstructMesh --roi-border) builds the surface near them with the context of both sides, at 1.5-2x the
mesh cost for no measurable gain on T&T Church.

MvsMergeChunks.py scene_XXXX.mvs scene_XXXX_dense_mesh.ply merged_mesh.ply
MvsMergeChunks.py scene_XXXX.mvs scene_XXXX_dense.ply merged_dense.ply

The two patterns name the sub-scenes (they carry the regions of interest) and their meshes or point-clouds,
with XXXX in place of the sub-scene number. Texture coordinates are not carried over (texture the merged
mesh). MvsScalablePipeline.py runs the whole split pipeline, this merge included.
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


class Vertices:
  """The vertex records of a binary little-endian PLY, kept as raw bytes so every property survives the
  merge, the variable-length lists of a dense point-cloud (view_indices, view_weights) included."""

  def __init__(self, props, body, count):
    self.props = props  # the header 'property ...' lines
    fixed, lists = [], []
    for line in props:
      t = line.split()
      if t[1] == 'list':
        lists.append((np.dtype(PLY_TYPES[t[2]]), np.dtype(PLY_TYPES[t[3]])))
      elif lists:
        raise ValueError('fixed-size vertex property %s after a list' % t[2])
      else:
        fixed.append((t[2], PLY_TYPES[t[1]]))
    self.fixed = np.dtype(fixed)
    record = self.fixed.itemsize
    if not lists:
      lengths = np.full(count, record, np.int64)
    else:
      # a record's length depends on its list counts: walk the records once
      lengths = np.empty(count, np.int64)
      pos = 0
      for i in range(count):
        size = record
        for cnt, item in lists:
          n = int.from_bytes(body[pos + size:pos + size + cnt.itemsize], 'little')
          size += cnt.itemsize + n * item.itemsize
        lengths[i] = size
        pos += size
    self.lengths = lengths
    self.starts = np.concatenate(([0], np.cumsum(lengths)[:-1])).astype(np.int64)
    self.data = np.frombuffer(body, np.uint8, int(lengths.sum()))
    self.count = count

  def xyz(self):
    rows = self.data[self.starts[:, None] + np.arange(self.fixed.itemsize)]
    v = rows.reshape(-1).view(self.fixed)
    return np.stack([v['x'], v['y'], v['z']], 1).astype(np.float64)

  def select(self, idx):
    """the records idx (sorted indices or a boolean mask), as bytes"""
    keep = np.zeros(self.count, bool)
    keep[idx] = True
    return self.data[np.repeat(keep, self.lengths)].tobytes(), int(keep.sum())


def read_ply(path):
  """Binary little-endian PLY: the vertices and the triangles (None for a point-cloud)."""
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
      cur[2].append(h)
  if [e[0] for e in elems] not in (['vertex'], ['vertex', 'face']):
    raise ValueError('%s: a vertex element and optionally a face element expected' % path)
  vertices = Vertices(elems[0][2], body, elems[0][1])
  if len(elems) == 1 or elems[1][1] == 0:
    return vertices, None
  t = elems[1][2][0].split()
  if len(elems[1][2]) != 1 or t[1] != 'list' or t[4] != 'vertex_indices':
    raise ValueError('%s: faces with only vertex_indices expected' % path)
  fc = np.frombuffer(body, np.dtype([('n', PLY_TYPES[t[2]]), ('i', PLY_TYPES[t[3]], (3,))]), elems[1][1],
                     len(vertices.data))
  if (fc['n'] != 3).any():
    raise ValueError('%s: triangles expected' % path)
  return vertices, fc['i'].astype(np.uint32)


def write_ply(path, props, count, vertex_bytes, faces):
  header = 'ply\nformat binary_little_endian 1.0\nelement vertex %d\n' % count + ''.join(p + '\n' for p in props)
  if faces is not None:
    header += 'element face %d\nproperty list uchar uint vertex_indices\n' % len(faces)
  header += 'end_header\n'
  with open(path, 'wb') as f:
    f.write(header.encode('ascii'))
    for chunk in vertex_bytes:
      f.write(chunk)
    if faces is not None:
      rec = np.zeros(len(faces), dtype=[('n', 'u1'), ('i', '<u4', (3,))])
      rec['n'] = 3
      rec['i'] = faces
      f.write(rec.tobytes())


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('input_scenes', help='sub-scene pattern, XXXX in place of the sub-scene number')
  parser.add_argument('input_plys', help='sub-scene mesh or point-cloud pattern, XXXX in place of the sub-scene number')
  parser.add_argument('output_ply')
  args = parser.parse_args()
  for pat in (args.input_scenes, args.input_plys):
    if 'XXXX' not in os.path.basename(pat):
      sys.exit('%s: the pattern must contain XXXX' % pat)
  pattern = os.path.abspath(args.input_plys)
  files = sorted(glob.glob(pattern.replace('XXXX', '[0-9][0-9][0-9][0-9]')))
  if not files:
    sys.exit('no sub-scene file matches %s' % pattern)
  name_re = re.compile(re.escape(os.path.basename(pattern)).replace('XXXX', r'(\d{4})') + '$')
  props, vertex_bytes, all_faces, offset, is_mesh = None, [], [], 0, None
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
    vertices, faces = read_ply(path)
    if is_mesh is None:
      is_mesh, props = faces is not None, vertices.props
    elif is_mesh != (faces is not None):
      sys.exit('%s: meshes and point-clouds can not be merged together' % path)
    elif vertices.props != props:
      sys.exit('%s: the vertex properties differ from the other sub-scenes' % path)
    xyz = vertices.xyz()
    if faces is None:
      # a point belongs to the box it lies in
      local = xyz @ rot.T
      keep = np.all((local >= lo) & (local < hi), axis=1)
      data, num = vertices.select(keep)
      vertex_bytes.append(data)
      offset += num
      print('%s: %d of %d points kept' % (os.path.basename(path), num, len(keep)))
      continue
    # a face belongs to the box its centroid lies in
    centroids = xyz[faces].mean(axis=1) @ rot.T
    keep = np.all((centroids >= lo) & (centroids < hi), axis=1)
    faces = faces[keep]
    used = np.unique(faces)
    remap = np.full(vertices.count, -1, np.int64)
    remap[used] = np.arange(len(used))
    vertex_bytes.append(vertices.select(used)[0])
    all_faces.append(remap[faces] + offset)
    offset += len(used)
    print('%s: %d of %d faces kept' % (os.path.basename(path), int(keep.sum()), len(keep)))
  faces = np.concatenate(all_faces).astype(np.uint32) if is_mesh else None
  write_ply(args.output_ply, props, offset, vertex_bytes, faces)
  print('merged %d sub-scenes: %d vertices%s -> %s' % (len(files), offset,
        ', %d faces' % len(faces) if is_mesh else '', args.output_ply))


if __name__ == '__main__':
  main()
