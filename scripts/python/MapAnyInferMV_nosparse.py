#!/usr/bin/env python3
# Multi-view MapAnything inference WITHOUT sparse-depth conditioning (Task-8 calibration study).
#
# USE WHEN: a scene's scene.mvs has ZERO SfM sparse vertices, so MapAnyInferMV.py's sparse-depth
# anchoring has nothing to project (BlendedMVS scenes imported via InterfaceMVSNet -- confirmed on
# bmvs_5a640093: loadMVSInterface(...)['vertices'] has len 0). Check the vertex count first and prefer
# MapAnyInferMV.py whenever real SfM sparse points exist.
#
# NEVER substitute the missing sparse guidance with GT-derived or dense-MVS-derived depth: the whole
# point of the pseudo-GT witness is independence from the dense reconstruction being evaluated (and
# from any GT it is later scored against) -- feeding either in would contaminate every downstream
# comparison. If there are no SfM points, the ONLY honest inputs are images+intrinsics+poses (this
# script); label the outputs as unconditioned (e.g. MapAnyVsGT.py --nosparse-witness).
#
# This is a stripped copy of MapAnyInferMV.py with the sparse-depth
# anchor removed entirely: each view dict carries ONLY img + intrinsics + camera_poses (no depth_z, no
# is_metric_scale override) -- i.e. exactly the "images+intrinsics+poses only" mode the MapAnything repo's
# own scripts/demo_images_only_inference.py exercises (depth_z/is_metric_scale are both documented-Optional
# keys in mapanything.utils.inference.ALLOWED_VIEW_KEYS; omitting them is the model's normal
# no-depth-conditioning input, not a hack). is_metric_scale is NOT set here -- the model defaults it to
# True (mapanything/utils/inference.py preprocess_input_views_for_inference step 4) with no actual sparse
# anchor behind it, so MapAnyVoxelFuse.py's own per-view scale-alignment to confident MVS pixels is what
# actually fixes scale for the downstream pseudo-GT cloud (same as it does for every other scene).
#
# Deliberately NOT feeding an all-zero sparse-depth map here: MvsSparseDepth.sparse_depth_for_view with
# an empty point set returns an all-zero array, and if that were passed as 'depth_z' it would still be
# converted to a (degenerate, all-zero) 'depth_along_ray' input and fed to the model's depth-conditioning
# branch (mapanything/models/mapanything/model.py:_encode_and_fuse_depths keys off in-view key PRESENCE,
# not per-pixel positivity) -- i.e. it would silently lie to the model ("trust this all-zero depth as
# metric") rather than cleanly signal "no depth provided". Omitting the key entirely is the correct/honest
# no-sparse-conditioning mode.
#
# Usage: python MapAnyInferMV_nosparse.py <scene.mvs> <dmap_dir> <scene_img_dir> <out_dir> [chunk] [overlap] [limit]
# Same env knobs as MapAnyInferMV.py (MA_APPLY_MASK, MA_MASK_EDGES, MA_CONF_MASK, MA_CONF_PCT, MA_MV_CONF, MA_RES).
import sys, os, glob, time
sys.path.insert(0, '/home/ubuntu/openMVS/scripts/python')
import numpy as np, cv2, torch
from MvsUtils import loadDMAP
from mapanything.models import MapAnything
from mapanything.utils.image import preprocess_inputs

scene_mvs, dmap_dir, img_dir, out_dir = sys.argv[1:5]
CHUNK = int(sys.argv[5]) if len(sys.argv) > 5 else 16
OV = int(sys.argv[6]) if len(sys.argv) > 6 else 6
limit = int(sys.argv[7]) if len(sys.argv) > 7 else 0
probe = os.environ.get('PROBE')


def envflag(name, default):
    return os.environ.get(name, default) not in ('0', '', 'false', 'False')


APPLY_MASK = envflag('MA_APPLY_MASK', '1')
MASK_EDGES = envflag('MA_MASK_EDGES', '0')
CONF_MASK = envflag('MA_CONF_MASK', '0')
CONF_PCT = float(os.environ.get('MA_CONF_PCT', '10'))
MV_CONF = envflag('MA_MV_CONF', '0')
MA_RES = int(os.environ.get('MA_RES', '0'))
INFER_KW = dict(memory_efficient_inference=True, apply_mask=APPLY_MASK, mask_edges=MASK_EDGES,
                apply_confidence_mask=CONF_MASK, confidence_percentile=CONF_PCT, use_multiview_confidence=MV_CONF)
PRE_KW = {} if MA_RES <= 0 else dict(resize_mode='longest_side', size=MA_RES)
print('CONFIG [NO-SPARSE: img+intrinsics+poses only] apply_mask=%d mask_edges=%d conf_mask=%d(pct=%g) mv_conf=%d res=%s'
      % (APPLY_MASK, MASK_EDGES, CONF_MASK, CONF_PCT, MV_CONF, MA_RES or 'native518'), flush=True)

os.makedirs(out_dir, exist_ok=True)
model = MapAnything.from_pretrained(os.environ.get('MA_MODEL', 'facebook/map-anything')).to('cuda').eval()
# NOTE: scene_mvs is accepted for CLI-signature parity with MapAnyInferMV.py / regen scripts, but is NOT
# loaded here (no sparse points to project -- that's the whole point of this variant).


def normals_from_depth(depth, K):
    H, W = depth.shape; Kinv = np.linalg.inv(K)
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    P = (np.stack([uu, vv, np.ones_like(uu)], -1).astype(np.float64) @ Kinv.T) * depth[..., None]
    dx = np.zeros_like(P); dy = np.zeros_like(P)
    dx[:, 1:-1] = P[:, 2:] - P[:, :-2]; dy[1:-1] = P[2:] - P[:-2]
    n = np.cross(dx, dy); n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-9)
    f = n[..., 2] > 0; n[f] = -n[f]
    return n.astype(np.float32)


def rs(a, W, H, interp):
    return cv2.resize(np.asarray(a, np.float32), (W, H), interpolation=interp)


def getarr(p, key):
    v = p.get(key, None)
    return None if v is None else v.squeeze().detach().cpu().numpy()


# build sorted view records (vid correlates with spatial order for sequential capture)
recs = []
for f in sorted(glob.glob(os.path.join(dmap_dir, 'depth*.dmap'))):
    d = loadDMAP(f); recs.append((int(d['reference_view_id']), f, d['file_name'], int(d['depth_width']), int(d['depth_height'])))
recs.sort()
if limit:
    recs = recs[:limit]
N = len(recs)
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
        c2w = np.eye(4); c2w[:3, :3] = R.T; c2w[:3, 3] = C
        # NO depth_z, NO is_metric_scale -- images+intrinsics+poses only (see module docstring).
        views.append({'img': torch.from_numpy(img.copy()), 'intrinsics': torch.from_numpy(K).float(),
                      'camera_poses': torch.from_numpy(c2w).float()})
        metas.append((vid, f, W, H, K, d))
    pv = preprocess_inputs(views, **PRE_KW)
    with torch.no_grad():
        preds = model.infer(pv, **INFER_KW)
    center = (w[0] + w[-1]) / 2.0
    for k, (vid, f, W, H, K, d) in enumerate(metas):
        cen = -abs(w[k] - center)
        if vid in best_central and best_central[vid] >= cen:
            continue
        best_central[vid] = cen
        p = preds[k]
        dz = getarr(p, 'depth_z').astype(np.float32)
        depth = rs(dz, W, H, cv2.INTER_LINEAR)
        mask = np.isfinite(depth) & (depth > 0)
        depth = np.where(mask, depth, 0.0)
        normal = normals_from_depth(np.maximum(depth, 1e-6), K)
        out = dict(depth=depth, normal=normal, mask=mask)
        conf = getarr(p, 'conf')
        if conf is not None:
            out['conf'] = rs(conf, W, H, cv2.INTER_LINEAR).astype(np.float32)
        na_mask = getarr(p, 'non_ambiguous_mask')
        if na_mask is not None:
            out['na_mask'] = rs(na_mask.astype(np.float32), W, H, cv2.INTER_NEAREST) > 0.5
        np.savez_compressed(os.path.join(out_dir, os.path.splitext(os.path.basename(f))[0] + '.npz'), **out)
        if probe and wi == 0 and k == 0:
            dv = np.asarray(d['depth_map'], np.float64); m = (dv > 0) & mask
            print('PRED keys', list(p.keys()), 'valid=%.3f SCALE med(MA/MVS)=%.4f'
                  % (mask.mean(), float(np.median(depth[m] / dv[m])) if m.any() else -1), flush=True)
    if (wi + 1) % 5 == 0:
        print('window %d/%d (%.0fs)' % (wi + 1, len(windows), time.time() - t0), flush=True)
print('DONE %d views in %.1fs' % (N, time.time() - t0), flush=True)
