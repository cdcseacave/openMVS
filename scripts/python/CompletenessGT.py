#!/usr/bin/env python3
# CompletenessGT.py — completeness + root-cause + gross-outlier assessment of OpenMVS fused clouds
# against the MapAnything voxel-fused pseudo-GT (magt.ply). All clouds are OpenMVS-format binary PLY
# (xyz, rgb, normal, per-point view_indices+view_weights).
#
# Metrics per OpenMVS cloud:
#  [completeness] fraction of GT voxels covered (an OpenMVS point within ~1 tolerance voxel), overall and
#                 broken down by the GT point's view-count (1,2,3-4,5+).
#  [root-cause]   (needs --dmaps) of the GT NOT covered by the cloud: how much is "discarded" (a raw MVS
#                 depth estimate DOES exist there, fusion dropped it) vs "mvs-missing" (no MVS depth at
#                 all). Built by back-projecting every valid raw depth-map pixel (pre-fusion).
#  [gross-outlier] fraction of OpenMVS points with NO MA-GT surface within a GROSS tolerance (default 10%
#                 of depth ~ 10 GT voxels): MapAnything is dense+smooth, so these are floaters. The test:
#                 w3 outlier-rate ~ w0 (added completeness must not add gross outliers).
#
# Usage: python CompletenessGT.py <magt_ply> <omvs_ply...> [--voxel V] [--dmaps DIR] [--gross-mult M] [--stride S]
import sys, os, glob
sys.path.insert(0, '/home/ubuntu/openMVS/scripts/python')
import numpy as np


def read_ply(path):
    """Header-aware streaming reader -> (xyz f64 Nx3, view_count i32 N, voxel_size or None)."""
    with open(path, 'rb') as f:
        fixed = 0; xyz_off = {}; lists = []; vsize = None; n = 0
        TS = {'char': 1, 'uchar': 1, 'int8': 1, 'uint8': 1, 'short': 2, 'ushort': 2, 'int16': 2,
              'uint16': 2, 'int': 4, 'uint': 4, 'int32': 4, 'uint32': 4, 'float': 4, 'float32': 4,
              'double': 8, 'float64': 8}
        while True:
            ln = f.readline().decode('latin1').split()
            if not ln:
                continue
            if ln[0] == 'comment' and len(ln) >= 3 and ln[1] == 'voxel_size':
                vsize = float(ln[2])
            elif ln[0] == 'element' and ln[1] == 'vertex':
                n = int(ln[2])
            elif ln[0] == 'property':
                if ln[1] == 'list':
                    lists.append((TS[ln[2]], TS[ln[3]]))
                else:
                    if ln[2] in ('x', 'y', 'z'):
                        xyz_off[ln[2]] = fixed
                    fixed += TS[ln[1]]
            elif ln[0] == 'end_header':
                break
        buf = f.read()
    import struct
    xyz = np.empty((n, 3), np.float64); vc = np.zeros(n, np.int32)
    ox, oy, oz = xyz_off['x'], xyz_off['y'], xyz_off['z']; upf = struct.Struct('<f').unpack_from
    off = 0
    for i in range(n):
        xyz[i, 0] = upf(buf, off + ox)[0]; xyz[i, 1] = upf(buf, off + oy)[0]; xyz[i, 2] = upf(buf, off + oz)[0]
        off += fixed
        for li, (cs, es) in enumerate(lists):
            cnt = buf[off] if cs == 1 else int.from_bytes(buf[off:off + cs], 'little')
            if li == 0:
                vc[i] = cnt
            off += cs + cnt * es
    return xyz, vc, vsize


def keys(xyz, voxel, mins, M):
    """Pack voxel coords to int64 within frame [mins, mins+M); out-of-frame -> -1."""
    q = np.floor(xyz / voxel).astype(np.int64) - mins
    inb = np.all((q >= 0) & (q < M), axis=1)
    k = np.where(inb, q[:, 0] + q[:, 1] * M[0] + q[:, 2] * (M[0] * M[1]), -1)
    return k


def dilate(k, M):
    """27-neighborhood dilation of in-frame keys (tolerance ~1 voxel)."""
    Mi, Mj = int(M[0]), int(M[1])
    out = [k]
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            for dk in (-1, 0, 1):
                if di or dj or dk:
                    out.append(k + di + dj * Mi + dk * Mi * Mj)
    return np.unique(np.concatenate(out))


def dmap_keys(dmap_dir, voxel, mins, M, stride):
    """Voxel keys (in-frame) of every valid RAW depth-map pixel (pre-fusion, all estimates)."""
    from MvsUtils import loadDMAP
    ks = []
    for fdm in sorted(glob.glob(os.path.join(dmap_dir, 'depth*.dmap'))):
        d = loadDMAP(fdm)
        dep = np.asarray(d['depth_map'], np.float64)
        K = np.asarray(d['depth_K'], np.float64); R = np.asarray(d['R'], np.float64)
        C = np.asarray(d['C'], np.float64).reshape(3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        sel = dep > 0
        if stride > 1:
            s2 = np.zeros_like(sel); s2[::stride, ::stride] = True; sel &= s2
        rr, cc = np.nonzero(sel)
        if rr.size == 0:
            continue
        z = dep[rr, cc]
        Xc = np.stack([(cc - cx) / fx * z, (rr - cy) / fy * z, z], 1)
        X = C + Xc @ R
        k = keys(X, voxel, mins, M)
        ks.append(np.unique(k[k >= 0]))
    return np.unique(np.concatenate(ks)) if ks else np.array([], np.int64)


def popopt(args, flag, default=None, cast=str):
    if flag in args:
        i = args.index(flag); v = cast(args[i + 1]); del args[i:i + 2]; return v
    return default


def main(argv):
    args = list(argv)
    voxel = popopt(args, '--voxel', None, float)
    dmaps = popopt(args, '--dmaps', None, str)
    gross_mult = popopt(args, '--gross-mult', 10.0, float)
    stride = popopt(args, '--stride', 2, int)
    magt, omvs_list = args[0], args[1:]

    gt_xyz, gt_vc, gt_vsize = read_ply(magt)
    if voxel is None:
        voxel = gt_vsize
    print('completeness vs %s : %d GT pts, voxel=%.5g (gross=%.5g = %gx)'
          % (magt.split('/')[-1], len(gt_xyz), voxel, voxel * gross_mult, gross_mult), flush=True)
    clouds = {p.split('/')[-1]: read_ply(p)[0] for p in omvs_list}

    # common frame at the fine voxel (GT + OMVS clouds bound it; raw-MVS/outlier points clip to it)
    allpts = np.concatenate([gt_xyz] + list(clouds.values()))
    mins = np.floor(allpts.min(0) / voxel).astype(np.int64)
    M = (np.floor(allpts.max(0) / voxel).astype(np.int64) - mins + 1)
    assert M[0] * M[1] * M[2] < 2**62, 'frame too large; increase --voxel'
    gt_keys = keys(gt_xyz, voxel, mins, M)
    buckets = [('1', gt_vc == 1), ('2', gt_vc == 2), ('3-4', (gt_vc >= 3) & (gt_vc <= 4)), ('5+', gt_vc >= 5)]

    raw_dil = None
    if dmaps:
        raw_dil = dilate(dmap_keys(dmaps, voxel, mins, M, stride), M)
        print('raw-MVS voxel set: %d voxels (stride=%d)' % (len(raw_dil), stride), flush=True)

    # coarse frame for gross-outlier test
    gv = voxel * gross_mult
    gmins = np.floor(allpts.min(0) / gv).astype(np.int64)
    gM = (np.floor(allpts.max(0) / gv).astype(np.int64) - gmins + 1)
    gt_gross = dilate(np.unique(keys(gt_xyz, gv, gmins, gM)), gM)

    for name, oxyz in clouds.items():
        ok = keys(oxyz, voxel, mins, M)
        omvs_dil = dilate(np.unique(ok[ok >= 0]), M)
        covered = np.isin(gt_keys, omvs_dil)
        print('\n=== %s ===' % name)
        print('  COMPLETENESS = %.4f   (%d/%d GT voxels)   [OMVS pts=%d]'
              % (covered.mean(), int(covered.sum()), len(gt_keys), len(oxyz)))
        for bn, bs in buckets:
            if bs.sum():
                print('    views=%-3s : completeness=%.4f  (GT pts=%d)' % (bn, covered[bs].mean(), int(bs.sum())))
        if raw_dil is not None:
            raw_cov = np.isin(gt_keys, raw_dil)
            print('  ROOT-CAUSE of GT coverage: reconstructed=%.4f  discarded(MVS-had-it)=%.4f  mvs-missing=%.4f'
                  % (covered.mean(), (raw_cov & ~covered).mean(), (~raw_cov).mean()))
        # gross outliers: OMVS points with no MA-GT surface within ~gross_mult fine-voxels
        gk = keys(oxyz, gv, gmins, gM)
        out = ~np.isin(gk, gt_gross)
        print('  GROSS-OUTLIERS (>~%.4g world / %gx voxel from any MA surface): %.4f  (%d/%d pts)'
              % (gv, gross_mult, out.mean(), int(out.sum()), len(oxyz)))


if __name__ == '__main__':
    main(sys.argv[1:])
