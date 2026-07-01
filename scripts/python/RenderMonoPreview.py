#!/usr/bin/env python3
# RenderMonoPreview.py — dump colormapped PNG previews of MapAnything mono depth-maps for quick viewing.
# Run in sam3 env (cv2). Usage: python RenderMonoPreview.py <mono_npz_dir>
import sys, os, glob
import numpy as np, cv2

mono_dir = sys.argv[1]
for fn in sorted(glob.glob(os.path.join(mono_dir, 'depth*.npz'))):
    m = np.load(fn)
    depth = m['depth'].astype(np.float64); mask = m['mask'].astype(bool)
    valid = mask & (depth > 0)
    if valid.sum() == 0:
        continue
    lo, hi = np.percentile(depth[valid], [2, 98])
    norm = np.clip((depth - lo) / max(hi - lo, 1e-9), 0, 1)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~valid] = 0
    out = fn[:-4] + '_preview.png'
    cv2.imwrite(out, img)
print('wrote %d previews to %s' % (len(glob.glob(os.path.join(mono_dir, '*_preview.png'))), mono_dir), flush=True)
