#!/usr/bin/env python3
# Run MapAnything per view with SfM sparse-depth + intrinsics + pose priors -> METRIC depth at scene scale.
# Save depth+normal+mask on the dmap grid (moge-compatible npz), for the completeness witness ONLY.
# Run in the sam3 conda env. Usage: python mapany_infer.py <scene.mvs> <dmap_dir> <scene_img_dir> <out_dir> [limit]
import sys, os, glob, time
sys.path.insert(0, '/home/ubuntu/openMVS/scripts/python')
import numpy as np, cv2, torch
from MvsUtils import loadDMAP
from MvsSparseDepth import load_scene_points, sparse_depth_for_view
from mapanything.models import MapAnything
from mapanything.utils.image import preprocess_inputs

scene_mvs, dmap_dir, img_dir, out_dir = sys.argv[1:5]
limit = int(sys.argv[5]) if len(sys.argv) > 5 else 0
probe = os.environ.get('PROBE')
os.makedirs(out_dir, exist_ok=True)
dev = 'cuda'
model = MapAnything.from_pretrained(os.environ.get('MA_MODEL', 'facebook/map-anything')).to(dev).eval()
Xs, by_view = load_scene_points(scene_mvs)
dmaps = sorted(glob.glob(os.path.join(dmap_dir, 'depth*.dmap')))
if limit: dmaps = dmaps[:limit]


def normals_from_depth(depth, K):
    H, W = depth.shape
    Kinv = np.linalg.inv(K)
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    pix = np.stack([uu, vv, np.ones_like(uu)], -1).astype(np.float64)
    P = (pix @ Kinv.T) * depth[..., None]              # (H,W,3) camera coords
    dy = np.zeros_like(P); dx = np.zeros_like(P)
    dx[:, 1:-1] = P[:, 2:] - P[:, :-2]; dy[1:-1] = P[2:] - P[:-2]
    n = np.cross(dx, dy)
    n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-9)
    flip = n[..., 2] > 0; n[flip] = -n[flip]           # camera-facing
    return n.astype(np.float32)


t0 = time.time()
for i, f in enumerate(dmaps):
    stem = os.path.splitext(os.path.basename(f))[0]
    d = loadDMAP(f); vid = int(d['reference_view_id'])
    W, H = int(d['depth_width']), int(d['depth_height'])
    K = np.asarray(d['depth_K'], np.float64); R = np.asarray(d['R'], np.float64); C = np.asarray(d['C'], np.float64).reshape(3)
    img = cv2.cvtColor(cv2.imread(os.path.join(img_dir, d['file_name'])), cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
    sd, na, _ = sparse_depth_for_view(Xs, by_view.get(vid, np.array([], np.int64)), f)
    c2w = np.eye(4); c2w[:3, :3] = R.T; c2w[:3, 3] = C
    view = {
        'img': torch.from_numpy(img.copy()),                                  # (H,W,3) [0,255]
        'intrinsics': torch.from_numpy(K).float(),
        'camera_poses': torch.from_numpy(c2w).float(),
        'depth_z': torch.from_numpy(sd.astype(np.float32)),
        'is_metric_scale': torch.tensor([True]),
    }
    pv = preprocess_inputs([view])
    with torch.no_grad():
        pred = model.infer(pv, memory_efficient_inference=True)
    p = pred[0]
    if probe and i == 0:
        print('PRED keys:', list(p.keys()))
        for k, v in p.items():
            if torch.is_tensor(v): print(' ', k, tuple(v.shape), v.dtype)
    dz = p['depth_z'].squeeze().detach().cpu().numpy().astype(np.float32)      # model-res metric depth
    Kp = p['intrinsics'].squeeze().detach().cpu().numpy() if 'intrinsics' in p else K
    mh, mw = dz.shape
    depth = cv2.resize(dz, (W, H), interpolation=cv2.INTER_LINEAR)
    mask = np.isfinite(depth) & (depth > 0)
    depth = np.where(mask, depth, 0.0)
    normal = normals_from_depth(np.maximum(depth, 1e-6), K)
    np.savez_compressed(os.path.join(out_dir, stem + '.npz'), depth=depth, normal=normal, mask=mask)
    if probe and i == 0:
        dv = np.asarray(d['depth_map'], np.float64); m = (dv > 0) & mask
        print('SCALE check: median(MapAny/MVS) over valid = %.4f (==~1 means metric)  anchors=%d' % (
            float(np.median(depth[m] / dv[m])), na))
    if (i + 1) % 20 == 0: print('%d/%d (%.0fs)' % (i + 1, len(dmaps), time.time() - t0), flush=True)
print('DONE', len(dmaps), 'in', round(time.time() - t0, 1), 's', flush=True)
