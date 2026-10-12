#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
Merge the meshes or the point-clouds of the sub-scenes a scene was split into (DensifyPointCloud
--sub-scene-memory) into one, the way a tiled reconstruction is assembled: each sub-scene mesh keeps only
the faces whose centroid lies in its own region of interest (the box the split gave it, stored as the
sub-scene's ROI), each point-cloud the points inside it, half-open on the upper side so an element on a
shared boundary goes to exactly one sub-scene; the kept elements are concatenated with all their vertex
properties (colors, normals, views, ...). The boundaries are not stitched; a margin around each box
(ReconstructMesh --roi-border) builds the surface near them with the context of both sides, at 1.5-2x the
mesh cost for no measurable gain on T&T Church.

MvsMergeChunks.py scene_XXXX.mvs scene_XXXX_dense_mesh.ply merged_mesh.ply
MvsMergeChunks.py --scene scene.mvs scene_XXXX.mvs scene_XXXX_dense.ply merged_dense.ply

The two patterns name the sub-scenes (they carry the regions of interest) and their meshes or point-clouds,
with XXXX in place of the sub-scene number; every sub-scene matching the first pattern needs its mesh or
point-cloud. The views of a point-cloud (view_indices) number the images of their sub-scene: --scene gives
the scene the sub-scenes were split from, whose numbering they take in the merge. The sub-scenes are read
one at a time and the result is streamed to disk, so the memory needed follows the largest sub-scene, not
the scene. Texture coordinates are not carried over (texture the merged mesh). MvsScalablePipeline.py runs
the whole split pipeline, this merge included.
"""

import argparse
import array
import glob
import os
import re
import shutil
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from MvsUtils import loadMVSInterface  # noqa: E402

PLY_TYPES = {'float': '<f4', 'float32': '<f4', 'double': '<f8', 'float64': '<f8', 'uchar': 'u1', 'uint8': 'u1',
             'char': 'i1', 'int8': 'i1', 'ushort': '<u2', 'uint16': '<u2', 'short': '<i2', 'int16': '<i2',
             'uint': '<u4', 'uint32': '<u4', 'int': '<i4', 'int32': '<i4'}
NO_ID = 0xFFFFFFFF
BLOCK = 1 << 20  # records processed at once


def gather(data, offsets, dtype):
  """the dtype values stored at the byte offsets of data"""
  dtype = np.dtype(dtype)
  out = np.empty(len(offsets), dtype)
  span = np.arange(dtype.itemsize)
  for b in range(0, len(offsets), BLOCK):
    out[b:b + BLOCK] = data[offsets[b:b + BLOCK, None] + span].reshape(-1).view(dtype)
  return out


def scatter(data, offsets, values):
  """store the values at the byte offsets of data"""
  span = np.arange(values.dtype.itemsize)
  raw = values.view(np.uint8).reshape(-1, values.dtype.itemsize)
  for b in range(0, len(offsets), BLOCK):
    data[offsets[b:b + BLOCK, None] + span] = raw[b:b + BLOCK]


class Vertices:
  """The vertex records of a binary little-endian PLY, kept as raw bytes so every property survives the
  merge: the fixed-size ones and the variable-length lists of a dense point-cloud (view_indices,
  view_weights), in any order."""

  def __init__(self, props, data, count):
    self.props = props  # the header 'property ...' lines
    self.data = data  # the PLY body, writable, starting at the first record
    self.count = count
    # a record is runs of fixed-size properties, each run but the last followed by a list
    runs, self.lists = [[]], []
    for line in props:
      t = line.split()
      if t[1] == 'list':
        self.lists.append((t[4], np.dtype(PLY_TYPES[t[2]]), np.dtype(PLY_TYPES[t[3]])))
        runs.append([])
      else:
        runs[-1].append((t[2], PLY_TYPES[t[1]]))
    self.runs = [np.dtype(r) for r in runs]
    if not self.lists:
      self.starts = np.arange(count, dtype=np.int64) * self.runs[0].itemsize
      self.end = count * self.runs[0].itemsize
    else:
      # a record's length depends on its list counts: walk the records once
      buf, starts = memoryview(data), array.array('q')
      add = starts.append
      steps = [(run.itemsize, cnt.itemsize, item.itemsize) for run, (_, cnt, item) in zip(self.runs, self.lists)]
      tail = self.runs[-1].itemsize
      pos = 0
      for _ in range(count):
        add(pos)
        for size, cnt, item in steps:
          pos += size
          n = buf[pos] if cnt == 1 else int.from_bytes(buf[pos:pos + cnt], 'little')
          pos += cnt + n * item
        pos += tail
      self.starts = np.frombuffer(starts, np.int64)
      self.end = pos
    if self.end > len(data):
      raise ValueError('the vertex records end past the file')

  def run_starts(self, k):
    """where the run k of fixed-size properties starts in each record (after the list k-1)"""
    starts = self.starts
    for run, (_, cnt, item) in zip(self.runs[:k], self.lists):
      head = starts + run.itemsize
      starts = head + cnt.itemsize + gather(self.data, head, cnt).astype(np.int64) * item.itemsize
    return starts

  def field(self, name):
    """the values of the fixed-size property name of every record"""
    for k, run in enumerate(self.runs):
      if name in run.names:
        dtype, offset = run.fields[name][:2]
        return gather(self.data, self.run_starts(k) + offset, dtype)
    raise ValueError('no vertex property %s' % name)

  def xyz(self):
    return np.stack([self.field(c) for c in 'xyz'], 1).astype(np.float64)

  def renumber(self, name, lut):
    """replace each item i of the list property name by lut[i]"""
    k = [lst[0] for lst in self.lists].index(name)
    _, cnt, item = self.lists[k]
    heads = self.run_starts(k) + self.runs[k].itemsize  # each record's list: its count, then its items
    counts = gather(self.data, heads, cnt).astype(np.int64)
    for b in range(0, self.count, BLOCK):
      n = counts[b:b + BLOCK]
      # item j of record r is at heads[r] + cnt + j*item, j counted from the record's first item
      first = np.cumsum(n) - n
      offsets = np.repeat(heads[b:b + BLOCK] + cnt.itemsize - first * item.itemsize, n) + \
          np.arange(n.sum(), dtype=np.int64) * item.itemsize
      values = gather(self.data, offsets, item)
      if values.size and values.max() >= len(lut):
        raise ValueError('a %s item is not below %d' % (name, len(lut)))
      scatter(self.data, offsets, lut[values].astype(item))

  def write(self, f, keep):
    """write the records keep (a boolean mask) to f"""
    lengths = np.diff(self.starts, append=self.end)
    for b in range(0, self.count, BLOCK):
      e = min(b + BLOCK, self.count)
      self.data[self.starts[b]:self.starts[e - 1] + lengths[e - 1]][np.repeat(keep[b:e], lengths[b:e])].tofile(f)


def read_ply(path):
  """Binary little-endian PLY: the vertices and the triangles (None for a point-cloud)."""
  with open(path, 'rb') as f:
    header = []
    while True:
      line = f.readline().decode('ascii').strip()
      header.append(line)
      if line == 'end_header':
        break
    data = np.fromfile(f, np.uint8)
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
  vertices = Vertices(elems[0][2], data, elems[0][1])
  if len(elems) == 1:
    return vertices, None  # no face element: a point-cloud (a mesh with 0 faces is still a mesh)
  if elems[1][1] == 0:
    return vertices, np.empty((0, 3), np.uint32)
  t = elems[1][2][0].split()
  if len(elems[1][2]) != 1 or t[1] != 'list' or t[4] != 'vertex_indices':
    raise ValueError('%s: faces with only vertex_indices expected' % path)
  fc = np.frombuffer(data, np.dtype([('n', PLY_TYPES[t[2]]), ('i', PLY_TYPES[t[3]], (3,))]), elems[1][1],
                     vertices.end)
  if (fc['n'] != 3).any():
    raise ValueError('%s: triangles expected' % path)
  return vertices, fc['i'].astype(np.uint32)


def image_ids(scene):
  """the ID of each image of a scene (its index when it has none)"""
  return [img['id'] if img.get('id', NO_ID) != NO_ID else i for i, img in enumerate(scene['images'])]


def load_scene(path):
  data = loadMVSInterface(path)
  if data is None:
    sys.exit('%s: can not load the scene' % path)
  return data


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('--scene', help='the scene the sub-scenes were split from (needed to merge point views)')
  parser.add_argument('input_scenes', help='sub-scene pattern, XXXX in place of the sub-scene number')
  parser.add_argument('input_plys', help='sub-scene mesh or point-cloud pattern, XXXX in place of the sub-scene number')
  parser.add_argument('output_ply')
  args = parser.parse_args()
  for pat in (args.input_scenes, args.input_plys):
    if 'XXXX' not in os.path.basename(pat):
      sys.exit('%s: the pattern must contain XXXX' % pat)
  # the sub-scenes there are now, each with its box, its images and its mesh or point-cloud
  pattern = os.path.abspath(args.input_scenes)
  name_re = re.compile(re.escape(os.path.basename(pattern)).replace('XXXX', r'(\d{4})') + '$')
  boxes = []
  for scene in sorted(glob.glob(pattern.replace('XXXX', '[0-9][0-9][0-9][0-9]'))):
    match = name_re.match(os.path.basename(scene))
    if not match:
      continue
    path = os.path.abspath(args.input_plys).replace('XXXX', match.group(1))
    if not os.path.isfile(path):
      sys.exit('%s: missing, the sub-scene %s has no such product' % (path, scene))
    data = load_scene(scene)
    obb = data.get('obb')
    if obb is None:
      sys.exit('%s: the sub-scene has no region of interest' % scene)
    boxes.append((path, np.asarray(obb['rot'], np.float64).reshape(3, 3), np.asarray(obb['pt_min'], np.float64),
                  np.asarray(obb['pt_max'], np.float64), image_ids(data)))
  if not boxes:
    sys.exit('no sub-scene matches %s' % pattern)
  # the boxes tile the scene's box: a shared face goes to the box above it (half-open upper side),
  # and the scene box's own upper faces, shared with no box, stay closed
  top = np.max([box[3] for box in boxes], axis=0)

  def inside(local, lo, hi):
    return np.all((local >= lo) & np.where(hi >= top, local <= hi, local < hi), axis=1)

  # the kept records and faces go to temporary files, joined under the header once their counts are known
  tmp_vertices, tmp_faces = args.output_ply + '.vertices.tmp', args.output_ply + '.faces.tmp'
  props, scene_index, num_vertices, num_faces, is_mesh = None, None, 0, 0, None
  try:
    with open(tmp_vertices, 'wb') as fv, open(tmp_faces, 'wb') as ff:
      for path, rot, lo, hi, ids in boxes:
        vertices, faces = read_ply(path)
        if is_mesh is None:
          is_mesh, props = faces is not None, vertices.props
        elif is_mesh != (faces is not None):
          sys.exit('%s: meshes and point-clouds can not be merged together' % path)
        elif vertices.props != props:
          sys.exit('%s: the vertex properties differ from the other sub-scenes' % path)
        if any(lst[0] == 'view_indices' for lst in vertices.lists):
          # the views number the sub-scene images: renumber them to the scene's
          if scene_index is None:
            if not args.scene:
              sys.exit('%s: the points carry their views, --scene must give the scene the sub-scenes were '
                       'split from' % path)
            scene_index = {i: idx for idx, i in enumerate(image_ids(load_scene(args.scene)))}
          missing = [i for i in ids if i not in scene_index]
          if missing:
            sys.exit('%s: the sub-scene image ID %d is not in %s' % (path, missing[0], args.scene))
          try:
            vertices.renumber('view_indices', np.array([scene_index[i] for i in ids], np.uint32))
          except ValueError as e:
            sys.exit('%s: %s' % (path, e))
        xyz = vertices.xyz()
        if faces is None:
          # a point belongs to the box it lies in
          keep = inside(xyz @ rot.T, lo, hi)
          vertices.write(fv, keep)
          num_vertices += int(keep.sum())
          print('%s: %d of %d points kept' % (os.path.basename(path), int(keep.sum()), len(keep)), flush=True)
          continue
        # a face belongs to the box its centroid lies in
        keep = inside(xyz[faces].mean(axis=1) @ rot.T, lo, hi)
        print('%s: %d of %d faces kept' % (os.path.basename(path), int(keep.sum()), len(keep)), flush=True)
        faces = faces[keep]
        used = np.zeros(vertices.count, bool)
        used[faces] = True
        remap = np.cumsum(used, dtype=np.int64) - 1 + num_vertices
        vertices.write(fv, used)
        rec = np.empty(len(faces), dtype=[('n', 'u1'), ('i', '<u4', (3,))])
        rec['n'] = 3
        rec['i'] = remap[faces]
        rec.tofile(ff)
        num_vertices += int(used.sum())
        num_faces += len(faces)
    header = 'ply\nformat binary_little_endian 1.0\nelement vertex %d\n' % num_vertices + \
        ''.join(p + '\n' for p in props)
    if is_mesh:
      header += 'element face %d\nproperty list uchar uint vertex_indices\n' % num_faces
    header += 'end_header\n'
    with open(args.output_ply, 'wb') as f:
      f.write(header.encode('ascii'))
      for tmp in (tmp_vertices, tmp_faces):
        with open(tmp, 'rb') as src:
          shutil.copyfileobj(src, f, 64 << 20)
  finally:
    for tmp in (tmp_vertices, tmp_faces):
      if os.path.exists(tmp):
        os.remove(tmp)
  print('merged %d sub-scenes: %d vertices%s -> %s' % (len(boxes), num_vertices,
        ', %d faces' % num_faces if is_mesh else '', args.output_ply))


if __name__ == '__main__':
  main()
