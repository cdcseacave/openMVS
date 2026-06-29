#!/usr/bin/env python3
# MULTI-VIEW MapAnything: feed posed neighbour groups (img+intrinsics+pose+sparse depth) jointly ->
# sharper, view-consistent METRIC depth. Save per-view depth+normal+mask on the dmap grid (moge npz).
# sam3 env. Usage: python mapany_infer_mv.py <scene.mvs> <dmap_dir> <scene_img_dir> <out_dir> [chunk] [overlap] [limit]
import sys, os, glob, time
sys.path.insert(0, '/home/ubuntu/openMVS/scripts/python')
import numpy as np, cv2, torch
from MvsUtils import loadDMAP
from MvsSparseDepth import load_scene_points, sparse_depth_for_view
from mapanything.models import MapAnything
from mapanything.utils.image import preprocess_inputs

scene_mvs, dmap_dir, img_dir, out_dir = sys.argv[1:5]
CHUNK = int(sys.argv[5]) if len(sys.argv) > 5 else 16
OV = int(sys.argv[6]) if len(sys.argv) > 6 else 6
limit = int(sys.argv[7]) if len(sys.argv) > 7 else 0
probe = os.environ.get('PROBE')
os.makedirs(out_dir, exist_ok=True)
model = MapAnything.from_pretrained(os.environ.get('MA_MODEL', 'facebook/map-anything')).to('cuda').eval()
Xs, by_view = load_scene_points(scene_mvs)


def normals_from_depth(depth, K):
    H, W = depth.shape; Kinv = np.linalg.inv(K)
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    P = (np.stack([uu, vv, np.ones_like(uu)], -1).astype(np.float64) @ Kinv.T) * depth[..., None]
    dx = np.zeros_like(P); dy = np.zeros_like(P)
    dx[:, 1:-1] = P[:, 2:] - P[:, :-2]; dy[1:-1] = P[2:] - P[:-2]
    n = np.cross(dx, dy); n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-9)
    f = n[..., 2] > 0; n[f] = -n[f]
    return n.astype(np.float32)


# build sorted view records (vid correlates with spatial order for sequential capture)
recs = []
for f in sorted(glob.glob(os.path.join(dmap_dir, 'depth*.dmap'))):
    d = loadDMAP(f); recs.append((int(d['reference_view_id']), f, d['file_name'], int(d['depth_width']), int(d['depth_height'])))
recs.sort()
if limit:
    recs = recs[:limit]
N = len(recs)
# overlapping windows; each view kept from the window where it is most central
best_central = {}
windows = []
i = 0
while i < N:
    w = list(range(i, min(i + CHUNK, N)))
    windows.append(w)
    if i + CHUNK >= N:
        break
    i += CHUNK - OV
print('%d views, %d windows of <=%d (overlap %d)' % (N, len(windows), CHUNK, OV), flush=True)

t0 = time.time()
for wi, w in enumerate(windows):
    views, metas = [], []
    for j in w:
        vid, f, fname, W, H = recs[j]
        d = loadDMAP(f)
        K = np.asarray(d['depth_K'], np.float64); R = np.asarray(d['R'], np.float64); C = np.asarray(d['C'], np.float64).reshape(3)
        img = cv2.resize(cv2.cvtColor(cv2.imread(os.path.join(img_dir, fname)), cv2.COLOR_BGR2RGB), (W, H), interpolation=cv2.INTER_AREA)
        sd, na, _ = sparse_depth_for_view(Xs, by_view.get(vid, np.array([], np.int64)), f)
        c2w = np.eye(4); c2w[:3, :3] = R.T; c2w[:3, 3] = C
        views.append({'img': torch.from_numpy(img.copy()), 'intrinsics': torch.from_numpy(K).float(),
                      'camera_poses': torch.from_numpy(c2w).float(), 'depth_z': torch.from_numpy(sd.astype(np.float32)),
                      'is_metric_scale': torch.tensor([True])})
        metas.append((vid, f, W, H, K, d))
    pv = preprocess_inputs(views)
    with torch.no_grad():
        preds = model.infer(pv, memory_efficient_inference=True)
    center = (w[0] + w[-1]) / 2.0
    for k, (vid, f, W, H, K, d) in enumerate(metas):
        cen = -abs(w[k] - center)
        if vid in best_central and best_central[vid] >= cen:
            continue
        best_central[vid] = cen
        dz = preds[k]['depth_z'].squeeze().detach().cpu().numpy().astype(np.float32)
        depth = cv2.resize(dz, (W, H), interpolation=cv2.INTER_LINEAR)
        mask = np.isfinite(depth) & (depth > 0); depth = np.where(mask, depth, 0.0)
        normal = normals_from_depth(np.maximum(depth, 1e-6), K)
        np.savez_compressed(os.path.join(out_dir, 'depth%04d.npz' % vid).replace('depth%04d' % vid, os.path.splitext(os.path.basename(f))[0]),
                            depth=depth, normal=normal, mask=mask)
        if probe and wi == 0 and k == 0:
            dv = np.asarray(d['depth_map'], np.float64); m = (dv > 0) & mask
            print('PRED keys', list(preds[0].keys())[:6], '... SCALE med(MA/MVS)=%.4f' % float(np.median(depth[m] / dv[m])), flush=True)
    if (wi + 1) % 5 == 0:
        print('window %d/%d (%.0fs)' % (wi + 1, len(windows), time.time() - t0), flush=True)
print('DONE %d views in %.1fs' % (N, time.time() - t0), flush=True)
