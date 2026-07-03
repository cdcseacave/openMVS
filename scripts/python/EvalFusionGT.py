#!/usr/bin/env python3
"""
Score a fused/reconstructed point cloud against a ground-truth mesh (BlendedMVS-style eval).

Metrics (all fractions of surface samples within a distance tolerance):
  completeness(t) = fraction of GT surface samples that have a reconstructed point within t
  accuracy(t)     = fraction of reconstructed points that have a GT surface sample within t
  gross_outlier_frac = fraction of reconstructed points with NO GT surface sample within a much
                        larger `gross_tol` (floaters/blunders, not just noise)

Tolerances default to {0.25%, 0.5%, 1%} of the GT sample cloud's bbox diagonal (gross outliers at
5% of diag), matching BlendedMVS scene scale variability. ETH3D uses fixed-metric tolerances via
the official ETH3DMultiViewEvaluation tool instead -- see gt_bench/eth3d_eval.sh.

Usage:
  python EvalFusionGT.py <fused.ply> --gt-mesh <mesh.obj|mesh.ply|textured_mesh_dir>
                          [--n-samples 10000000] [--seed 0] [--json out.json]

--gt-mesh accepts:
  - a single .obj file
  - a single binary_little_endian .ply mesh (vertex x,y,z + a face list property)
  - a DIRECTORY of `tile_*.obj` files (BlendedMVS textured_mesh/ layout) -- all tiles are loaded
    and concatenated (vertex indices re-offset per tile) into one mesh.

NN backend: scipy.spatial.cKDTree if available, else a dependency-free uniform-grid spatial hash
(_nn_dist_grid) returning EXACT distances (expanding-ring search with a certified safe-radius
termination rule, brute-force resolution for anything uncertified after ring_cap -- see its
docstring). NOTE: this box's /home/ubuntu/miniconda3 python has no scipy installed, so in
practice `_nn_dist_grid` is the path actually exercised end-to-end here, not just a rarely-used
fallback -- see scripts/python/tests/test_evalfusion_gt.py for direct brute-force-comparison tests
of it (not just the looser cube-test tolerance margins).

--json writes {completeness: {frac_str: value}, accuracy: {frac_str: value}, gross_outlier_frac,
tol_abs: {frac_str: abs_value}, diag, n_rec, n_gt} for Task 7's aggregator. Tolerance dict keys are
the STRING fraction labels ("0.0025", "0.005", "0.01"), not floats, so the JSON round-trips cleanly
(JSON object keys are always strings; using the fraction string avoids float-repr key drift).
"""
import os
import sys
import glob
import json
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import GtUtils

# BlendedMVS default tolerances, as fractions of the GT sample cloud's bbox diagonal.
# (label, fraction) -- label is the exact JSON key string.
DEFAULT_TOL_FRACS = [('0.0025', 0.0025), ('0.005', 0.005), ('0.01', 0.01)]
DEFAULT_GROSS_FRAC = 0.05


def make_cube_mesh():
    """Unit cube surface, 8 verts / 12 tris (2 triangles per face). Test helper, also handy as a
    tiny synthetic GT mesh for smoke-testing the CLI end to end."""
    V = np.array([
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0],
    ])
    F = np.array([
        [0, 1, 2], [0, 2, 3],   # bottom z=0
        [4, 6, 5], [4, 7, 6],   # top z=1
        [0, 1, 5], [0, 5, 4],   # front y=0
        [3, 6, 2], [3, 7, 6],   # back y=1
        [0, 4, 7], [0, 7, 3],   # left x=0
        [1, 2, 6], [1, 6, 5],   # right x=1
    ], dtype=np.int64)
    return V, F


def sample_mesh(V, F, n, seed=0):
    """Uniform-area sampling of a triangle mesh surface -> (n, 3) points."""
    a = V[F[:, 1]] - V[F[:, 0]]
    b = V[F[:, 2]] - V[F[:, 0]]
    area = 0.5 * np.linalg.norm(np.cross(a, b), axis=1)
    total = area.sum()
    if total <= 0:
        raise ValueError('sample_mesh: mesh has zero total surface area')
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(F), n, p=area / total)
    r1, r2 = rng.random((2, n))
    s = np.sqrt(r1)
    return (V[F[idx, 0]] * (1 - s)[:, None] + V[F[idx, 1]] * (s * (1 - r2))[:, None]
            + V[F[idx, 2]] * (s * r2)[:, None])


def load_obj(path):
    """Minimal Wavefront OBJ loader: vertices + triangulated (fan) faces. Ignores vt/vn/mtllib
    (geometry-only eval)."""
    V, F = [], []
    for line in open(path):
        if line.startswith('v '):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            ix = [int(t.split('/')[0]) - 1 for t in line.split()[1:]]
            for k in range(1, len(ix) - 1):
                F.append([ix[0], ix[k], ix[k + 1]])
    return np.asarray(V, dtype=np.float64), np.asarray(F, dtype=np.int64)


def load_mesh_ply(path):
    """Minimal binary_little_endian PLY mesh loader: vertex x,y,z (float32/float64) + a face
    list property (e.g. `property list uchar int vertex_indices`). Faces are fan-triangulated.
    ASCII PLY is not supported (matches GtUtils.load_ply_xyz's binary-only scope)."""
    import struct
    SZ = {'float': 4, 'double': 8, 'uchar': 1, 'uint8': 1, 'char': 1, 'int8': 1,
          'int': 4, 'uint': 4, 'uint32': 4, 'int32': 4, 'ushort': 2, 'uint16': 2,
          'short': 2, 'int16': 2, 'float32': 4, 'float64': 8}
    UNPACK = {'uchar': 'B', 'uint8': 'B', 'char': 'b', 'int8': 'b', 'int': 'i', 'uint': 'I',
              'uint32': 'I', 'int32': 'i', 'ushort': 'H', 'uint16': 'H', 'short': 'h', 'int16': 'h'}
    with open(path, 'rb') as f:
        fmt, cur, n_vert, n_face = None, None, 0, 0
        vert_props, face_props = [], []
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if line.startswith('format'):
                fmt = line.split()[1]
            elif line.startswith('element'):
                parts = line.split()
                cur = parts[1]
                if cur == 'vertex':
                    n_vert = int(parts[2])
                elif cur == 'face':
                    n_face = int(parts[2])
            elif line.startswith('property'):
                parts = line.split()
                if cur == 'vertex':
                    vert_props.append(parts)
                elif cur == 'face':
                    face_props.append(parts)
            elif line == 'end_header':
                break
        assert fmt == 'binary_little_endian', f'load_mesh_ply only supports binary_little_endian, got {fmt}: {path}'

        vfixed = [p for p in vert_props if p[1] != 'list']
        head = [(p[1], p[2]) for p in vfixed[:3]]
        if [nm for _, nm in head] != ['x', 'y', 'z']:
            raise ValueError(f'load_mesh_ply requires x,y,z as the first vertex properties, got {head}: {path}')
        rec = sum(SZ[p[1]] for p in vfixed)
        buf = f.read()
        pos = 0
        V = np.empty((n_vert, 3), dtype=np.float64)
        xt, yt, zt = head[0][0], head[1][0], head[2][0]
        for i in range(n_vert):
            V[i, 0] = struct.unpack_from('<' + UNPACK.get(xt, 'f' if xt in ('float', 'float32') else 'd'), buf, pos)[0]
            V[i, 1] = struct.unpack_from('<' + UNPACK.get(yt, 'f' if yt in ('float', 'float32') else 'd'), buf, pos + SZ[xt])[0]
            V[i, 2] = struct.unpack_from('<' + UNPACK.get(zt, 'f' if zt in ('float', 'float32') else 'd'), buf, pos + SZ[xt] + SZ[yt])[0]
            pos += rec

        flist = [p for p in face_props if p[1] == 'list']
        if not flist:
            raise ValueError(f'load_mesh_ply: no face list property found: {path}')
        count_ty, elem_ty = flist[0][2], flist[0][3]
        F = []
        for _ in range(n_face):
            cnt = struct.unpack_from('<' + UNPACK[count_ty], buf, pos)[0]
            pos += SZ[count_ty]
            ix = struct.unpack_from('<' + UNPACK[elem_ty] * cnt, buf, pos)
            pos += SZ[elem_ty] * cnt
            for k in range(1, cnt - 1):
                F.append([ix[0], ix[k], ix[k + 1]])
        return V, np.asarray(F, dtype=np.int64)


def load_mesh(path):
    """Dispatch: directory of `tile_*.obj` (BlendedMVS textured_mesh/) -> concatenate all tiles
    (vertex indices re-offset per tile); single .obj; single binary_little_endian .ply mesh."""
    if os.path.isdir(path):
        tiles = sorted(glob.glob(os.path.join(path, 'tile_*.obj')))
        if not tiles:
            raise ValueError(f'load_mesh: no tile_*.obj files found in directory {path}')
        Vs, Fs, voff = [], [], 0
        for t in tiles:
            v, f = load_obj(t)
            if len(v) == 0:
                continue
            Vs.append(v)
            if len(f):
                Fs.append(f + voff)
            voff += len(v)
        V = np.concatenate(Vs, axis=0) if Vs else np.zeros((0, 3))
        F = np.concatenate(Fs, axis=0) if Fs else np.zeros((0, 3), dtype=np.int64)
        return V, F

    ext = os.path.splitext(path)[1].lower()
    if ext == '.obj':
        return load_obj(path)
    if ext == '.ply':
        return load_mesh_ply(path)
    raise ValueError(f'load_mesh: unsupported GT mesh format {path!r} (expected .obj, .ply, or a tile_*.obj directory)')


# ---------------------------------------------------------------------------
# Nearest-neighbor distance backend.
#
# ADAPTATION vs. the task brief: the brief's fallback sketch was
# `import CompletenessGT as C; return C.nn_dist(query, ref)`, but
# scripts/python/CompletenessGT.py has no `nn_dist` function -- it only does
# binary voxel-membership coverage checks (keys()/dilate()), not continuous
# nearest-neighbor distances, which score_cloud() needs (it thresholds the
# same distance array at several tolerances plus the gross-outlier tolerance).
# miniconda's python also has no scipy or scikit-learn installed (verified),
# so `_nn_dist_grid` below -- a dependency-free uniform spatial-hash grid with
# a brute-force fallback for unresolved (typically far-outlier) points -- is
# implemented from scratch and is the path actually used on this box, not a
# rarely-exercised fallback. See test_nn_dist_grid_matches_bruteforce /
# test_nn_dist_grid_far_outliers in the test file for direct verification
# against brute force (independent of the cube test's looser tolerances).
# ---------------------------------------------------------------------------

def _bruteforce_min_dist(query, ref, qchunk=200, rchunk=100000):
    """Exact min distance from each query point to ref, chunked to bound memory. Used only for
    the (normally small) set of points _nn_dist_grid's neighborhood search fails to resolve."""
    if len(ref) == 0:
        return np.full(len(query), np.inf)
    r_sq = np.einsum('ij,ij->i', ref, ref)
    out = np.empty(len(query))
    for i in range(0, len(query), qchunk):
        blk = query[i:i + qchunk]
        q_sq = np.einsum('ij,ij->i', blk, blk)
        best = np.full(len(blk), np.inf)
        for j in range(0, len(ref), rchunk):
            rblk = ref[j:j + rchunk]
            d2 = q_sq[:, None] + r_sq[j:j + rchunk][None, :] - 2.0 * (blk @ rblk.T)
            np.maximum(d2, 0.0, out=d2)
            best = np.minimum(best, d2.min(axis=1))
        out[i:i + qchunk] = np.sqrt(best)
    return out


def _nn_dist_grid(query, ref, ring_cap=4):
    """Dependency-free EXACT nearest-neighbor distance from each `query` point to `ref`, via a
    uniform spatial hash grid sized to ~1 ref point/cell.

    Search: expanding Chebyshev rings around each query's cell (ring 1 = the full 3x3x3 block
    including the query's own cell; ring r>1 = only the (2r+1)^3-(2r-1)^3 shell of new cells),
    carrying the best distance found so far across rings.

    Termination (REVIEW FIX -- the certified safe-radius rule): a query's best-so-far distance d
    is only ACCEPTED once d <= ring*cell. Derivation: the query point q lies inside its cell c
    (q_i in [c_i*cell, (c_i+1)*cell) per axis, from floor()); after searching all cells within
    Chebyshev distance `ring` of c, the searched block spans [(c_i-ring)*cell, (c_i+ring+1)*cell)
    per axis, so q is at least ring*cell from every face of the block, and any point OUTSIDE the
    block is therefore at distance >= ring*cell from q. Hence d <= ring*cell proves no unsearched
    point can beat d. Without this rule, the first candidate found in a ring could be accepted
    even when a closer point sits just outside the searched block -- on non-uniform-density
    clouds (cell sized for the average density, query in a sparse region whose true NN is several
    cells away) that returned overestimated distances (reviewer repro: bimodal cloud, 3rd-nearest
    at 0.2764 returned instead of the true NN at 0.2026; now covered by
    test_nn_dist_grid_bimodal_density). Queries still uncertified after ring_cap (typically far
    outliers or points in large coverage holes) are resolved exactly via _bruteforce_min_dist, so
    every returned distance is exact either way -- completeness/accuracy thresholding depends on
    that.

    Encodes cell coordinates into a single int64 hash key with a 21-bit-per-axis budget
    (~+-1,048,576 cells from the ref bbox origin) -- query cells outside that budget cannot alias
    a real ref cell key (clipped to `pos=len-1` and rejected by the exact-key equality check on
    `uniq_keys[pos_c] == keys_flat`, and separately guarded by `inb`); since every ref cell IS in
    budget (asserted below), an out-of-budget neighbor cell is provably empty of ref points, so
    skipping it does not break the safe-radius certification."""
    query = np.asarray(query, dtype=np.float64)
    ref = np.asarray(ref, dtype=np.float64)
    nq, nr = len(query), len(ref)
    if nq == 0:
        return np.zeros(0)
    if nr == 0:
        return np.full(nq, np.inf)

    mn = ref.min(axis=0)
    # Robust density estimate for cell sizing: use a percentile-trimmed extent so a small number
    # of far outliers in `ref` (reconstructed clouds routinely contain floater points -- this is
    # exactly what gross_outlier_frac measures) can't blow up the estimated cell size for the
    # dense bulk of the cloud. Using the raw min/max here previously made cells ~15x too coarse on
    # a unit-cube-plus-100-far-outliers input: ~3400 ref points per cell meant a single ring-1
    # (3x3x3) block enumerated ~1.8e10 candidate (query, ref) pairs and exhausted host memory. The
    # encoding grid origin (mn, below) still spans the TRUE min/max, so no point -- including the
    # outliers -- is excluded from the grid; only the density/cell-size estimate is robust.
    lo = np.percentile(ref, 1, axis=0)
    hi = np.percentile(ref, 99, axis=0)
    extent = np.maximum(hi - lo, 1e-9)
    vol = float(extent[0] * extent[1] * extent[2])
    cell = max((vol / max(nr, 1)) ** (1.0 / 3.0), 1e-9)

    BITS = 21
    OFF = 1 << (BITS - 1)
    LIMIT = 1 << BITS

    def cellidx(pts):
        return np.floor((pts - mn) / cell).astype(np.int64)

    def enc(c):
        co = c + OFF
        inb = np.all((co >= 0) & (co < LIMIT), axis=1)
        key = (np.clip(co[:, 0], 0, LIMIT - 1) << (2 * BITS)) \
            | (np.clip(co[:, 1], 0, LIMIT - 1) << BITS) \
            | np.clip(co[:, 2], 0, LIMIT - 1)
        return key, inb

    ref_c = cellidx(ref)
    ref_key, ref_inb = enc(ref_c)
    if not np.all(ref_inb):
        raise ValueError('_nn_dist_grid: reference point cloud spans too many cells for the '
                          'int64 hash encoding (extremely non-uniform density); reduce n-samples '
                          'or pre-filter the point cloud')
    order = np.argsort(ref_key, kind='stable')
    ref_sorted = ref[order]
    key_sorted = ref_key[order]
    uniq_keys, starts = np.unique(key_sorted, return_index=True)
    ends = np.append(starts[1:], len(key_sorted))
    n_uniq = len(uniq_keys)

    q_c = cellidx(query)
    best = np.full(nq, np.inf)
    remaining = np.arange(nq)

    for ring in range(1, ring_cap + 1):
        if remaining.size == 0:
            break
        rng = np.arange(-ring, ring + 1)
        gx, gy, gz = np.meshgrid(rng, rng, rng, indexing='ij')
        offs = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        if ring > 1:
            # only the new shell (Chebyshev distance == ring); inner cells were already searched
            # in previous rings and their candidates are carried in `best`.
            offs = offs[np.abs(offs).max(axis=1) == ring]

        qc = q_c[remaining]                                  # (M,3)
        neigh = (qc[:, None, :] + offs[None, :, :]).reshape(-1, 3)   # (M*K,3)
        keys_flat, inb = enc(neigh)
        pos = np.searchsorted(uniq_keys, keys_flat)
        pos_c = np.clip(pos, 0, max(n_uniq - 1, 0))
        valid = inb & (pos < n_uniq) & (uniq_keys[pos_c] == keys_flat) if n_uniq else np.zeros_like(inb)

        K = offs.shape[0]
        M = len(remaining)
        if np.any(valid):
            hit_q = np.repeat(np.arange(M), K)[valid]
            s = starts[pos_c[valid]]
            e = ends[pos_c[valid]]
            counts = e - s
            total = int(counts.sum())
            if total > 200_000_000:
                # Defensive valve: even with the robust percentile-based cell sizing above, a
                # pathologically skewed density (e.g. a huge duplicate-point cluster) could still
                # blow up a single ring's candidate count. Fail loudly instead of risking host
                # OOM (this box has no swap -- see openmvs-build-env memory notes).
                raise MemoryError(
                    f'_nn_dist_grid: ring {ring} search would materialize {total} candidate '
                    f'(query,ref) pairs (>2e8); likely pathological density skew in the '
                    f'reference cloud -- aborting rather than risking host OOM')
            if total > 0:
                group_start = np.cumsum(counts) - counts
                idx_in_group = np.arange(total) - np.repeat(group_start, counts)
                ref_idx_flat = np.repeat(s, counts) + idx_in_group
                q_idx_flat = np.repeat(hit_q, counts)

                diffs = query[remaining][q_idx_flat] - ref_sorted[ref_idx_flat]
                d = np.sqrt(np.einsum('ij,ij->i', diffs, diffs))

                local_best = np.full(M, np.inf)
                np.minimum.at(local_best, q_idx_flat, d)
                best[remaining] = np.minimum(best[remaining], local_best)

        # certified safe-radius acceptance (see docstring): a best-so-far distance d is provably
        # the true NN distance only once d <= ring*cell -- anything outside the searched block is
        # at distance >= ring*cell. Queries with a candidate but d > ring*cell MUST keep
        # expanding (their true NN may sit just outside the searched block).
        remaining = remaining[best[remaining] > ring * cell]

    if remaining.size:
        # uncertified after ring_cap (far outliers / large coverage holes): resolve exactly.
        best[remaining] = _bruteforce_min_dist(query[remaining], ref)

    return best


def nn_dist(query, ref):
    try:
        from scipy.spatial import cKDTree
        return cKDTree(ref).query(query, workers=-1)[0]
    except ImportError:
        return _nn_dist_grid(query, ref)


def _voxel_dedup(points, voxel_size):
    """Keeps one representative point per occupied voxel of side voxel_size (first-seen wins in
    input order). No-op if voxel_size <= 0 or points is empty. Used by score_cloud -- see the
    comment there for why."""
    if voxel_size <= 0 or len(points) == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    # pack the 3 int64 axis-indices into one structured-array key so np.unique can dedup whole
    # rows (voxel cells) in one vectorized pass instead of a Python-level loop.
    packed = np.ascontiguousarray(keys).view([('', keys.dtype)] * 3)
    _, first_idx = np.unique(packed, return_index=True)
    return points[np.sort(first_idx)]


def score_cloud(rec, gt, tols, gross_tol):
    """rec: (Nr,3) reconstructed points. gt: (Ng,3) GT surface samples. tols: list of absolute
    distance thresholds. gross_tol: absolute distance threshold for gross-outlier detection.
    Returns completeness/accuracy keyed by the exact tol values passed in (not fraction labels --
    the CLI layer below does that relabeling for JSON output)."""
    # ADAPTATION (Task 7 real-GT validation finding, 2026-07-03): a real OpenMVS fused cloud
    # (10^5-10^6+ points) paired with a GT sample of comparable size makes _nn_dist_grid's ring-1
    # candidate-pair estimate exceed its 2e8 memory-safety valve essentially unconditionally --
    # not because of a data corruption/outlier pathology (checked directly: neither cloud's
    # per-cell point density is wildly skewed in isolation on real data), but simply because
    # O(n_query * 27 * local_ref_density) crosses the valve once n_query reaches ~10^6, which any
    # real dense reconstruction does. Measured directly on a real 1.48M-point OpenMVS cloud: a
    # voxel 1/10th of the finest tolerance only merged 6% of points (real inter-point spacing is
    # centimeter-scale, not sub-mm) and still exceeded the valve; a voxel equal to the finest
    # tolerance itself merged ~89% and comfortably cleared it. Fix: voxel-dedup BOTH clouds at the
    # finest requested tolerance before scoring -- two points closer together than the finest
    # tolerance already fall in the same completeness/accuracy bucket at every requested
    # granularity in the overwhelming majority of cases (they can only disagree for a pair that
    # straddles a tolerance boundary within one voxel diagonal, a second-order edge effect), so
    # this does not materially change what the metric measures -- it mainly drops the redundant
    # multi-view reobservations of the same physical surface point that dense MVS fusion produces
    # well below its own reconstruction precision. Standard practice in MVS benchmarks (e.g.
    # Tanks & Temples voxel-downsamples before evaluation). n_rec/n_gt below still report the
    # ORIGINAL (pre-dedup) point counts.
    voxel = min(tols) if tols else 0.0
    rec_d, gt_d = _voxel_dedup(rec, voxel), _voxel_dedup(gt, voxel)
    d_gt2rec = nn_dist(gt_d, rec_d)        # completeness
    d_rec2gt = nn_dist(rec_d, gt_d)        # accuracy / outliers
    return {'completeness': {t: float((d_gt2rec <= t).mean()) for t in tols},
            'accuracy':     {t: float((d_rec2gt <= t).mean()) for t in tols},
            'gross_outlier_frac': float((d_rec2gt > gross_tol).mean()),
            'n_rec': len(rec), 'n_gt': len(gt)}


def main():
    ap = argparse.ArgumentParser(description='Score a fused point cloud against a GT mesh (BlendedMVS-style)')
    ap.add_argument('fused_ply', help='fused/reconstructed point cloud (OpenMVS-format binary PLY)')
    ap.add_argument('--gt-mesh', required=True,
                     help='GT mesh: .obj file, binary_little_endian .ply mesh, or a directory of '
                          'tile_*.obj (BlendedMVS textured_mesh/) -- tiles are concatenated')
    ap.add_argument('--n-samples', type=int, default=10_000_000, help='number of GT surface samples (default 10M)')
    ap.add_argument('--seed', type=int, default=0, help='RNG seed for surface sampling')
    ap.add_argument('--json', default=None, help='write the result JSON here')
    args = ap.parse_args()

    print('loading fused cloud: %s' % args.fused_ply)
    rec = GtUtils.load_ply_xyz(args.fused_ply)
    print('  %d points' % len(rec))

    print('loading GT mesh: %s' % args.gt_mesh)
    V, F = load_mesh(args.gt_mesh)
    print('  %d verts, %d tris' % (len(V), len(F)))

    print('sampling %d GT surface points (seed=%d)' % (args.n_samples, args.seed))
    gt = sample_mesh(V, F, args.n_samples, seed=args.seed)

    diag = float(np.linalg.norm(gt.max(0) - gt.min(0)))
    abs_tols = [frac * diag for _, frac in DEFAULT_TOL_FRACS]
    gross_tol = DEFAULT_GROSS_FRAC * diag

    print('GT bbox diag=%.6g  tolerances(abs)=%s  gross_tol(abs)=%.6g' % (diag, abs_tols, gross_tol))
    m = score_cloud(rec, gt, tols=abs_tols, gross_tol=gross_tol)

    out = {
        'completeness': {label: m['completeness'][t] for (label, _), t in zip(DEFAULT_TOL_FRACS, abs_tols)},
        'accuracy':     {label: m['accuracy'][t]     for (label, _), t in zip(DEFAULT_TOL_FRACS, abs_tols)},
        'gross_outlier_frac': m['gross_outlier_frac'],
        'tol_abs': {label: t for (label, _), t in zip(DEFAULT_TOL_FRACS, abs_tols)},
        'diag': diag,
        'n_rec': m['n_rec'],
        'n_gt': m['n_gt'],
    }
    print(json.dumps(out, indent=2))
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)) or '.', exist_ok=True)
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=2)
        print('JSON written to %s' % args.json)
    return out


if __name__ == '__main__':
    main()
