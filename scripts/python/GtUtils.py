"""GT depth utilities for the gt_bench evaluation (BlendedMVS PFM, ETH3D raw depth)."""
import numpy as np, struct, re, os

def read_pfm(path):
    with open(path, 'rb') as f:
        header = f.readline().decode().rstrip()
        if header not in ('Pf', 'PF'): raise ValueError('not PFM: ' + path)
        w, h = map(int, f.readline().decode().split())
        scale = float(f.readline().decode().rstrip())
        data = np.fromfile(f, '<f4' if scale < 0 else '>f4', w * h * (3 if header == 'PF' else 1))
    img = data.reshape((h, w, 3) if header == 'PF' else (h, w))
    return np.flipud(img).copy()  # PFM stores rows bottom-up

def read_eth3d_depth(path, width, height):
    d = np.fromfile(path, '<f4')
    assert d.size == width * height, f'{path}: {d.size} != {width}x{height}'
    d = d.reshape(height, width).copy()
    d[~np.isfinite(d)] = np.nan
    d[d <= 0] = np.nan
    return d

def resize_depth_nearest(d, shape):
    h2, w2 = shape; h, w = d.shape
    yi = (np.arange(h2) * (h / h2) + 0.5 * h / h2).astype(int).clip(0, h - 1)
    xi = (np.arange(w2) * (w / w2) + 0.5 * w / w2).astype(int).clip(0, w - 1)
    return d[yi][:, xi]

def gt_labels(d_est, d_gt, rel_tol=0.01, abs_tol=0.0):
    valid = np.isfinite(d_gt) & (d_gt > 0) & (d_est > 0)
    tol = np.maximum(rel_tol * np.where(np.isfinite(d_gt), d_gt, 0), abs_tol)
    err = np.abs(np.where(valid, d_est - d_gt, 0))
    inlier = valid & (err <= tol)
    return inlier, valid & ~inlier, valid

def view_image_names(scene_mvs):
    # NOTE: brief referenced `MvsUtils.loadMVS`; the actual loader in this repo's
    # MvsUtils.py is `loadMVSInterface` (verified: MvsReadMVS.py's loader has an
    # unrelated bug and is not used here). Adapted accordingly.
    from MvsUtils import loadMVSInterface
    mvs = loadMVSInterface(scene_mvs)
    return [os.path.basename(im['name']) for im in mvs['images']]

def load_ply_xyz(path):
    # Vertex-only reader tolerating variable-length list properties (OpenMVS clouds).
    with open(path, 'rb') as f:
        n, props, fmt, cur_element = 0, [], None, None
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if line.startswith('format'): fmt = line.split()[1]
            elif line.startswith('element'):
                parts = line.split()
                cur_element = parts[1]
                if cur_element == 'vertex':
                    n = int(parts[2]); props = []
            elif line.startswith('property') and cur_element == 'vertex':
                # NOTE: adapted vs. the brief. The brief guarded property capture
                # with `if n:` (any element seen so far), which keeps accumulating
                # properties from LATER elements too (e.g. ETH3D scan1.ply has a
                # trailing `element camera 1` with 24 float/int properties after
                # `element vertex`). That inflated the computed per-vertex record
                # size and broke `np.fromfile(...).reshape(n, rec)` on real ETH3D
                # scan files. Scoping capture to `cur_element == 'vertex'` fixes it
                # while still tolerating trailing non-vertex elements (their bytes
                # are simply never read, since we stop after n*rec vertex bytes).
                props.append(line.split())
            elif line == 'end_header': break
        assert fmt == 'binary_little_endian', fmt
        SZ = {'float': 4, 'double': 8, 'uchar': 1, 'uint8': 1, 'int': 4, 'uint': 4,
              'uint32': 4, 'ushort': 2, 'uint16': 2, 'float32': 4, 'float64': 8}
        fixed = [p for p in props if p[1] != 'list']
        lists = [p for p in props if p[1] == 'list']
        if not lists:  # fast path: one big fromfile
            rec = sum(SZ[p[1]] for p in fixed)
            off = 0; offs = {}
            for p in fixed: offs[p[2]] = (off, p[1]); off += SZ[p[1]]
            raw = np.fromfile(f, np.uint8, n * rec).reshape(n, rec)
            xyz = np.empty((n, 3))
            for i, name in enumerate(('x', 'y', 'z')):
                o, t = offs[name]
                xyz[:, i] = raw[:, o:o + SZ[t]].copy().view('<f4' if SZ[t] == 4 else '<f8')[:, 0]
            return xyz
        # slow path: sequential parse (lists have per-vertex length)
        # The '<3f' unpack below is only correct when the first three fixed
        # properties are exactly float32 x, y, z -- guard against any other
        # layout (e.g. color-first or double-typed coords) instead of silently
        # returning garbage coordinates.
        head = [(p[1], p[2]) for p in fixed[:3]]
        if [nm for _, nm in head] != ['x', 'y', 'z'] or \
           any(ty not in ('float', 'float32') for ty, _ in head):
            raise ValueError(
                f'load_ply_xyz slow path requires float32 x,y,z as the first '
                f'three fixed vertex properties, got {head}: {path}')
        pre = sum(SZ[p[1]] for p in fixed)
        xyz = np.empty((n, 3), np.float64)
        buf = f.read()
        pos = 0
        for i in range(n):
            xyz[i] = struct.unpack_from('<3f', buf, pos)[:3]
            pos += pre
            for p in lists:
                cnt = struct.unpack_from('<' + {'uchar': 'B', 'uint8': 'B', 'int': 'i', 'uint': 'I', 'uint32': 'I'}[p[2]], buf, pos)[0]
                pos += SZ[p[2]] + cnt * SZ[p[3]]
        return xyz


# ---------------------------------------------------------------------------
# Step 5: ETH3D GT-depth convention.
#
# Empirically resolved (see scripts/python/tests/test_gtutils.py
# test_eth3d_convention_regression and .superpowers/sdd/task-4-report.md for the
# full measurement): ETH3D `ground_truth_depth` files are on the DISTORTED
# image grid (matches Task 2's finding: file byte size implies the distorted
# 6048x4032 resolution, not any of the per-camera undistorted resolutions) and
# store PLAIN Z-DEPTH (camera-frame Z, not along-ray/Euclidean distance).
# Measured median relative error vs. the laser scan (mlp-registered, z-buffered
# per pixel) projected through the distorted camera at those same distorted
# pixels: ~0.0000% (n=143699px) on courtyard (outdoor) and ~0.0000%
# (n=110241px) on office (indoor) -- see report for exact numbers --
# comfortably under the 1% gate, while ray-distance under the same distorted
# grid is off by 15.8%/17.1% median, and both undistorted-grid hypotheses are
# structurally impossible (GT file byte count only matches the distorted
# resolution, never the per-camera undistorted resolution).
#
# To bring distorted-grid GT depth onto the undistorted pixel grid used by
# OpenMVS's dmaps, `remap_eth3d_depth_to_undistorted` walks each undistorted
# pixel, forms its normalized ray via the undistorted PINHOLE K, forward-
# distorts that ray with the distorted camera's THIN_PRISM_FISHEYE model
# (COLMAP convention), and nearest-samples the distorted-grid GT depth at the
# resulting distorted pixel location. Because both hypotheses share the same
# camera center and the depth convention is Z (not ray distance), the Z value
# transfers unchanged under this remap (no ray-angle correction needed).
# ---------------------------------------------------------------------------

def _thin_prism_fisheye_distort(u, v, params):
    """Forward-distort normalized camera coords (u, v) with COLMAP's
    THIN_PRISM_FISHEYE model. params = (fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1).
    Returns pixel coordinates (px, py) in the DISTORTED image.

    NOTE: adapted vs. the brief's formula. The brief described this as a plain
    Brown-Conrady-style radial/tangential/thin-prism distortion applied
    directly to the pinhole ray (u, v) = (X/Z, Y/Z). Verified against COLMAP
    source (src/colmap/sensor/models.h, ThinPrismFisheyeCameraModel::ImgFromCam):
    the "Fisheye" part of the name is not decorative -- COLMAP first maps the
    pinhole ray through an EQUIDISTANT FISHEYE transform (uu,vv) =
    theta/r * (u,v) with theta = atan(r), r = |(u,v)|, and only THEN applies
    the k1..k4/p1,p2/sx1,sy1 polynomial to (uu, vv), not to (u, v) directly.
    Confirmed empirically: reprojecting COLMAP's own SfM points (points3D.txt)
    through the naive (no fisheye step) formula gave a median 340px error
    against images.txt's observed 2D keypoints; adding the fisheye step
    dropped that to sub-pixel (0.64px median). Automated as
    test_thin_prism_fisheye_reprojection in tests/test_gtutils.py (real-data
    guarded, asserts median < 2px)."""
    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1 = params
    r = np.sqrt(u * u + v * v)
    theta = np.arctan(r)
    scale = np.where(r > 1e-12, theta / np.where(r > 1e-12, r, 1.0), 1.0)
    uu = u * scale
    vv = v * scale
    r2 = uu * uu + vv * vv
    radial = 1.0 + k1 * r2 + k2 * r2**2 + k3 * r2**3 + k4 * r2**4
    du = uu * radial + 2 * p1 * uu * vv + p2 * (r2 + 2 * uu * uu) + sx1 * r2
    dv = vv * radial + p1 * (r2 + 2 * vv * vv) + 2 * p2 * uu * vv + sy1 * r2
    px = fx * du + cx
    py = fy * dv + cy
    return px, py


def load_colmap_camera_params(cameras_txt, camera_id=None):
    """Parse a COLMAP cameras.txt, return dict {camera_id: (model, width, height, params[])}
    or the single entry for `camera_id` if given."""
    cams = {}
    with open(cameras_txt) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            cid = int(parts[0])
            model = parts[1]
            width, height = int(parts[2]), int(parts[3])
            params = [float(x) for x in parts[4:]]
            cams[cid] = (model, width, height, params)
    if camera_id is not None:
        return cams[camera_id]
    return cams


def remap_eth3d_depth_to_undistorted(depth_distorted, cam_distorted, cam_pinhole):
    """Remap a distorted-grid ETH3D GT z-depth map onto the undistorted pixel grid.

    cam_distorted: (width, height, params) for the THIN_PRISM_FISHEYE distorted camera
                   (params = fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,sx1,sy1).
    cam_pinhole:   (width, height, params) for the undistorted PINHOLE camera
                   (params = fx,fy,cx,cy).
    Returns an (height, width) float32 array on the undistorted grid, NaN where
    the corresponding distorted-grid sample is invalid or falls outside the
    distorted image bounds.
    """
    w_u, h_u, p_u = cam_pinhole
    w_d, h_d, p_d = cam_distorted
    fx_u, fy_u, cx_u, cy_u = p_u[:4]

    yy, xx = np.meshgrid(np.arange(h_u), np.arange(w_u), indexing='ij')
    u = (xx.astype(np.float64) + 0.5 - cx_u) / fx_u
    v = (yy.astype(np.float64) + 0.5 - cy_u) / fy_u

    px, py = _thin_prism_fisheye_distort(u, v, p_d)
    xi = np.floor(px).astype(np.int64)
    yi = np.floor(py).astype(np.int64)
    inb = (xi >= 0) & (xi < w_d) & (yi >= 0) & (yi < h_d)

    out = np.full((h_u, w_u), np.nan, dtype=np.float32)
    xi_c = np.clip(xi, 0, w_d - 1)
    yi_c = np.clip(yi, 0, h_d - 1)
    sampled = depth_distorted[yi_c, xi_c]
    out[inb] = sampled[inb]
    return out
