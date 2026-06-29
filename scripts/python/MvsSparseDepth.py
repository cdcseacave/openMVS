#!/usr/bin/env python3
"""
Build per-view SPARSE metric depth maps from an OpenMVS scene.mvs (SfM sparse points),
to guide MapAnything toward metric, scene-scale mono depth (for the completeness witness ONLY).

The sparse SfM points are independent of the dense MVS depth-maps being evaluated, so using them
to anchor the pseudo-GT is not circular with the filtering (which uses only OpenMVS dense data).
"""
import sys, os, glob
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from MvsUtils import loadDMAP, loadMVSInterface


def load_scene_points(scene_mvs):
    """Return (Xs (N,3) world points, view_ids (list of sets) per point)."""
    d = loadMVSInterface(scene_mvs)
    Xs = np.array([v['X'] for v in d['vertices']], dtype=np.float64)
    vids = [set(int(w['image_id']) for w in v['views']) for v in d['vertices']]
    # map image id -> indices of points visible in it
    by_view = {}
    for i, s in enumerate(vids):
        for vid in s:
            by_view.setdefault(vid, []).append(i)
    by_view = {k: np.asarray(v, np.int64) for k, v in by_view.items()}
    return Xs, by_view


def sparse_depth_for_view(Xs, idx, dmap):
    """Project the SfM points visible in this view into its (depth-resolution) image -> sparse depth map."""
    d = loadDMAP(dmap)
    W, H = int(d['depth_width']), int(d['depth_height'])
    K = np.asarray(d['depth_K'], np.float64); R = np.asarray(d['R'], np.float64); C = np.asarray(d['C'], np.float64).reshape(3)
    if idx.size == 0:
        return np.zeros((H, W), np.float32), 0, d
    X = Xs[idx]                                   # (n,3)
    cam = (R @ (X.T - C[:, None]))                # (3,n)
    z = cam[2]
    pix = K @ cam
    with np.errstate(invalid='ignore', divide='ignore'):
        u = np.round(pix[0] / z).astype(int); v = np.round(pix[1] / z).astype(int)
    ok = (z > 0) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
    sd = np.zeros((H, W), np.float32)
    uu, vv, zz = u[ok], v[ok], z[ok]
    # nearest-point wins per pixel (keep smallest depth = closest surface)
    order = np.argsort(-zz)                        # write far first, near overwrites
    sd[vv[order], uu[order]] = zz[order].astype(np.float32)
    return sd, int(ok.sum()), d


def main():
    scene_mvs, dmap_dir = sys.argv[1], sys.argv[2]
    Xs, by_view = load_scene_points(scene_mvs)
    print('scene points=%d, views with points=%d' % (Xs.shape[0], len(by_view)))
    dmaps = sorted(glob.glob(os.path.join(dmap_dir, 'depth*.dmap')))
    tot_anchor, tot_resid, nv = 0, [], 0
    for f in dmaps[:30]:
        d0 = loadDMAP(f); vid = int(d0['reference_view_id'])
        idx = by_view.get(vid, np.array([], np.int64))
        sd, na, d = sparse_depth_for_view(Xs, idx, f)
        # sanity: sparse SfM depth vs dense MVS depth at the anchor pixels (should roughly agree)
        dv = np.asarray(d['depth_map'], np.float64)
        m = (sd > 0) & (dv > 0)
        if m.sum() > 10:
            rel = np.abs(sd[m] - dv[m]) / dv[m]
            tot_resid.append(np.median(rel)); nv += 1
        tot_anchor += na
    print('avg anchors/view=%.0f  median |sparse-MVS|/MVS over %d views = %.4f (should be small => ids+poses match)' % (
        tot_anchor / max(len(dmaps[:30]), 1), nv, float(np.median(tot_resid)) if tot_resid else float('nan')))


if __name__ == '__main__':
    main()
