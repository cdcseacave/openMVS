#!/usr/bin/env python3
# MapAnyVoxelFuse.py — build a dense pseudo-GT point cloud from MapAnything per-view mono depth-maps.
#
# For each view: scale-align the MapAnything depth to the MVS depth (median ratio on confident MVS
# pixels — the same anchor used by fuse_rescore_mono.py, independent of any rescued points), back-project
# every valid pixel to world space, then voxel-fuse all views: split the scene into a regular grid of
# cubic voxels sized to the scene's smallest resolvable detail (median GSD = depth/focal) and fuse the
# points in each voxel into one point, KEEPING the set of views that saw it. The result is written in
# OpenMVS's own binary PLY format (per-point view_indices + view_weights lists) so it loads in the
# OpenMVS viewer and in the existing scorers.
#
# Run in the sam3 env (needs cv2 + numpy):  /home/ubuntu/miniconda3/envs/sam3/bin/python
# Usage: python MapAnyVoxelFuse.py <dmap_dir> <mono_npz_dir> <img_dir> <out_ply> [voxel=auto] [stride=1]
import sys, os, glob
sys.path.insert(0, '/home/ubuntu/openMVS/scripts/python')
import numpy as np, cv2
from MvsUtils import loadDMAP

dmap_dir, mono_dir, img_dir, out_ply = sys.argv[1:5]
voxel_arg = sys.argv[5] if len(sys.argv) > 5 else 'auto'
STRIDE = int(sys.argv[6]) if len(sys.argv) > 6 else 1
MINCONF = float(os.environ.get('MINCONF', '0.1'))


def reduce_by_voxel(vox, *arrays):
    """Group rows by integer voxel coord (N,3); return (uniq_vox, count, *summed_arrays)."""
    uniq, inv = np.unique(vox, axis=0, return_inverse=True)
    inv = np.asarray(inv).ravel()
    M = len(uniq)
    cnt = np.bincount(inv, minlength=M).astype(np.int64)
    out = []
    for a in arrays:
        s = np.empty((M, a.shape[1]), np.float64)
        for k in range(a.shape[1]):
            s[:, k] = np.bincount(inv, weights=a[:, k].astype(np.float64), minlength=M)
        out.append(s)
    return (uniq, cnt) + tuple(out)


# ---- load each view, scale-align, collect GSD samples to pick the voxel size ----
views = []
gsd = []
depths = []
for fdm in sorted(glob.glob(os.path.join(dmap_dir, 'depth*.dmap'))):
    stem = os.path.splitext(os.path.basename(fdm))[0]
    fn = os.path.join(mono_dir, stem + '.npz')
    if not os.path.exists(fn):
        continue
    d = loadDMAP(fdm)
    m = np.load(fn)
    ma = m['depth'].astype(np.float64)
    mask = m['mask'].astype(bool)
    mvs = np.asarray(d['depth_map'], np.float64)
    conf = np.asarray(d['confidence_map'], np.float64)
    K = np.asarray(d['depth_K'], np.float64)
    ok = mask & (mvs > 0) & (ma > 0) & (conf >= MINCONF)
    if ok.sum() < 200:
        continue
    s = float(np.median(mvs[ok] / ma[ok]))               # MA -> MVS metric scale (per view)
    fx = K[0, 0]
    dvalid = (s * ma)[mask & (ma > 0)]
    md = float(np.median(dvalid))
    gsd.append(md / fx)                                  # ground sampling distance (world size / pixel)
    depths.append(md)
    views.append(dict(d=d, ma=ma * s, normal=m['normal'].astype(np.float64), mask=mask & (ma > 0),
                      K=K, R=np.asarray(d['R'], np.float64), C=np.asarray(d['C'], np.float64).reshape(3),
                      fname=d['file_name']))
if not views:
    print('ERROR: no matching dmap/npz pairs'); sys.exit(1)

gsd_med = float(np.median(gsd)); depth_med = float(np.median(depths))
if voxel_arg == 'auto':
    # Size the fusion voxel to the mono-consistency scale (a fraction of scene depth), NOT the raw GSD:
    # mono depth noise across views (~1% of depth) far exceeds the GSD, so GSD-sized voxels never fuse
    # cross-view observations (-> ~1 view/voxel, a huge per-view union). DEPTHFRAC*median_depth is where
    # same-surface multi-view points coincide, yielding genuine multi-view voxels.
    DEPTHFRAC = float(os.environ.get('DEPTHFRAC', '0.01'))
    voxel = DEPTHFRAC * depth_med
else:
    voxel = float(voxel_arg)
print('voxel-fuse %d views: voxel=%.5g (= %.4f%% of median depth %.4g = %.1f x GSD %.4g), stride=%d'
      % (len(views), voxel, 100 * voxel / depth_med, depth_med, voxel / gsd_med, gsd_med, STRIDE), flush=True)

# ---- back-project each view, pre-reduce to its own voxels ----
parts = []
for vid, v in enumerate(views):
    K, R, C = v['K'], v['R'], v['C']
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    H, W = v['ma'].shape
    sel = v['mask'].copy()
    if STRIDE > 1:
        s2 = np.zeros_like(sel); s2[::STRIDE, ::STRIDE] = True; sel &= s2
    rr, cc = np.nonzero(sel)
    if rr.size == 0:
        continue
    z = v['ma'][rr, cc]
    Xc = np.stack([(cc - cx) / fx * z, (rr - cy) / fy * z, z], axis=1)   # camera frame
    X = C + Xc @ R                                                       # world: (X-C)=Xc@R
    ncam = v['normal'][rr, cc]
    nw = ncam @ R
    nw /= np.maximum(np.linalg.norm(nw, axis=1, keepdims=True), 1e-9)
    # color from the source image, resized to the dmap grid (same as MapAnyInferMV)
    img = cv2.imread(os.path.join(img_dir, v['fname']))
    if img is not None:
        img = cv2.cvtColor(cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        col = img[rr, cc].astype(np.float64)
    else:
        col = np.full((rr.size, 3), 180.0)
    vox = np.floor(X / voxel).astype(np.int32)
    uniq, cnt, sumX, sumC, sumN = reduce_by_voxel(vox, X, col, nw)
    parts.append(dict(vox=uniq, cnt=cnt, sumX=sumX, sumC=sumC, sumN=sumN,
                      vid=np.full(len(uniq), vid, np.int32)))

# ---- global merge across views ----
allvox = np.concatenate([p['vox'] for p in parts])
allcnt = np.concatenate([p['cnt'] for p in parts])
allX = np.concatenate([p['sumX'] for p in parts])
allC = np.concatenate([p['sumC'] for p in parts])
allN = np.concatenate([p['sumN'] for p in parts])
allV = np.concatenate([p['vid'] for p in parts])
guniq, ginv = np.unique(allvox, axis=0, return_inverse=True)
ginv = np.asarray(ginv).ravel()
G = len(guniq)
gcnt = np.bincount(ginv, weights=allcnt, minlength=G)
xyz = np.stack([np.bincount(ginv, weights=allX[:, k], minlength=G) for k in range(3)], 1) / gcnt[:, None]
rgb = (np.stack([np.bincount(ginv, weights=allC[:, k], minlength=G) for k in range(3)], 1) / gcnt[:, None])
nrm = np.stack([np.bincount(ginv, weights=allN[:, k], minlength=G) for k in range(3)], 1)
nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)
# distinct views per voxel (sorted by voxel then view = CSR layout)
pairs = np.unique(np.stack([ginv, allV], 1), axis=0)
vcount = np.bincount(pairs[:, 0], minlength=G).astype(np.int64)
view_flat = pairs[:, 1].astype(np.uint32)               # already grouped by voxel, ascending view
offs = np.zeros(G + 1, np.int64); offs[1:] = np.cumsum(vcount)
print('GT cloud: %d voxels, views/point mean=%.2f max=%d' % (G, vcount.mean(), vcount.max()), flush=True)

# ---- write OpenMVS binary PLY (per-k vectorized: x,y,z f32 | rgb u8 | nx,ny,nz f32 | u8+k*u32 | u8+k*f32) ----
xyz = xyz.astype('<f4'); rgb = np.clip(rgb, 0, 255).astype('u1'); nrm = nrm.astype('<f4')
hdr = ("ply\nformat binary_little_endian 1.0\ncomment voxel_size %.8g\nelement vertex %d\n"
       "property float x\nproperty float y\nproperty float z\n"
       "property uchar red\nproperty uchar green\nproperty uchar blue\n"
       "property float nx\nproperty float ny\nproperty float nz\n"
       "property list uchar uint view_indices\n"
       "property list uchar float view_weights\nend_header\n") % (voxel, G)
with open(out_ply, 'wb') as f:
    f.write(hdr.encode())
    order = np.argsort(vcount, kind='stable')
    for k in np.unique(vcount):
        if k == 0:
            continue
        idx = order[vcount[order] == k]
        dt = np.dtype([('xyz', '<f4', (3,)), ('rgb', 'u1', (3,)), ('n', '<f4', (3,)),
                       ('vc', 'u1'), ('vi', '<u4', (k,)), ('wc', 'u1'), ('vw', '<f4', (k,))])
        assert dt.itemsize == 29 + 8 * k, (dt.itemsize, k)
        rec = np.empty(len(idx), dt)
        rec['xyz'] = xyz[idx]; rec['rgb'] = rgb[idx]; rec['n'] = nrm[idx]
        rec['vc'] = k; rec['wc'] = k
        gather = offs[idx][:, None] + np.arange(k)[None, :]
        rec['vi'] = view_flat[gather]
        rec['vw'] = 1.0
        f.write(rec.tobytes())
print('wrote %s (%d points)' % (out_ply, G), flush=True)
