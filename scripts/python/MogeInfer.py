#!/usr/bin/env python3
# Run MoGe-2 per view, resized to the MVS dmap grid; save depth+normal+mask per view as npz.
# Run in the amb3r conda env (torch+cuda+cv2): PYTHONPATH=.../moge python moge_infer.py <dmap_dir> <scene_dir> <out_dir>
import sys, os, glob, time
sys.path.insert(0, '/home/ubuntu/openMVS/scripts/python')
import numpy as np, cv2, torch
from MvsUtils import loadDMAP
from moge.model.v2 import MoGeModel

dmap_dir, scene_dir, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(out_dir, exist_ok=True)
model = MoGeModel.from_pretrained('Ruicheng/moge-2-vitl-normal').cuda().eval()
dmaps = sorted(glob.glob(os.path.join(dmap_dir, 'depth*.dmap')))
print(f'{len(dmaps)} views', flush=True)
t0 = time.time()
for i, f in enumerate(dmaps):
    stem = os.path.splitext(os.path.basename(f))[0]
    d = loadDMAP(f)
    W, H = int(d['depth_width']), int(d['depth_height'])
    bgr = cv2.imread(os.path.join(scene_dir, d['file_name']))
    if bgr is None:
        print('skip (no image)', stem, flush=True); continue
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    # cap input long side to limit GPU memory (output is resized to the dmap grid anyway)
    MAXSIDE = int(os.environ.get('MOGE_MAXSIDE', '1600'))
    h0, w0 = rgb.shape[:2]
    if max(h0, w0) > MAXSIDE:
        sc = MAXSIDE / max(h0, w0)
        rgb = cv2.resize(rgb, (int(round(w0 * sc)), int(round(h0 * sc))), interpolation=cv2.INTER_AREA)
    ten = torch.tensor(rgb / 255.0, dtype=torch.float32).permute(2, 0, 1).cuda()
    with torch.no_grad():
        out = model.infer(ten)
    depth = out['depth'].cpu().numpy().astype(np.float32)
    normal = out['normal'].cpu().numpy().astype(np.float32)
    mask = out['mask'].cpu().numpy().astype(bool)
    valid = mask & np.isfinite(depth)
    depth = np.where(valid, depth, 0.0)
    depth_r = cv2.resize(depth, (W, H), interpolation=cv2.INTER_LINEAR)
    mask_r = cv2.resize(valid.astype(np.uint8), (W, H), interpolation=cv2.INTER_NEAREST).astype(bool)
    normal_r = cv2.resize(normal, (W, H), interpolation=cv2.INTER_LINEAR)
    normal_r /= np.maximum(np.linalg.norm(normal_r, axis=2, keepdims=True), 1e-6)
    np.savez_compressed(os.path.join(out_dir, stem + '.npz'), depth=depth_r, normal=normal_r.astype(np.float32), mask=mask_r)
    if (i + 1) % 20 == 0:
        print(f'{i+1}/{len(dmaps)} ({round(time.time()-t0,1)}s)', flush=True)
print('DONE', len(dmaps), 'in', round(time.time() - t0, 1), 's', flush=True)
