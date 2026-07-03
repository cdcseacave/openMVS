import sys, os, struct, io, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import GtUtils

def test_pfm_roundtrip(tmp='/tmp/claude_test.pfm'):
    d = np.random.rand(7, 5).astype(np.float32)
    with open(tmp, 'wb') as f:  # write little-endian bottom-up PFM
        f.write(b'Pf\n5 7\n-1.0\n'); np.flipud(d).astype('<f4').tofile(f)
    r = GtUtils.read_pfm(tmp)
    assert r.shape == (7, 5) and np.allclose(r, d)

def test_gt_labels():
    d_gt  = np.array([[10.0, 10.0, np.nan, 10.0]])
    d_est = np.array([[10.05, 11.0, 10.0, 0.0]])   # 0 = no estimate
    inl, outl, valid = GtUtils.gt_labels(d_est, d_gt, rel_tol=0.01)
    assert inl.tolist()  == [[True, False, False, False]]
    assert outl.tolist() == [[False, True, False, False]]
    assert valid.tolist()== [[True, True, False, False]]

def test_resize_nearest():
    d = np.arange(16, dtype=np.float32).reshape(4, 4)
    r = GtUtils.resize_depth_nearest(d, (2, 2))
    assert r.shape == (2, 2)

def _write_ply_header(f, n, props, extra_elements=b''):
    f.write(b'ply\nformat binary_little_endian 1.0\n')
    f.write(f'element vertex {n}\n'.encode())
    for p in props:
        f.write((p + '\n').encode())
    f.write(extra_elements)
    f.write(b'end_header\n')

def test_load_ply_xyz_fixed_props(tmp='/tmp/claude_test_fixed.ply'):
    n = 5
    xyz = np.random.rand(n, 3).astype(np.float32)
    rgb = np.random.randint(0, 255, (n, 3)).astype(np.uint8)
    with open(tmp, 'wb') as f:
        _write_ply_header(f, n, [
            'property float x', 'property float y', 'property float z',
            'property uchar red', 'property uchar green', 'property uchar blue'])
        for i in range(n):
            f.write(xyz[i].tobytes())
            f.write(rgb[i].tobytes())
    r = GtUtils.load_ply_xyz(tmp)
    assert r.shape == (n, 3)
    assert np.allclose(r, xyz, atol=1e-6)

def test_load_ply_xyz_with_list_property(tmp='/tmp/claude_test_list.ply'):
    # Mimics OpenMVS dense point cloud: per-vertex variable-length `views` list.
    n = 4
    xyz = np.random.rand(n, 3).astype(np.float32)
    view_counts = [1, 3, 0, 2]
    with open(tmp, 'wb') as f:
        _write_ply_header(f, n, [
            'property float x', 'property float y', 'property float z',
            'property list uchar int views'])
        for i in range(n):
            f.write(xyz[i].tobytes())
            cnt = view_counts[i]
            f.write(struct.pack('<B', cnt))
            for j in range(cnt):
                f.write(struct.pack('<i', j))
    r = GtUtils.load_ply_xyz(tmp)
    assert r.shape == (n, 3)
    assert np.allclose(r, xyz, atol=1e-6)

def test_load_ply_xyz_slow_path_guards_xyz_layout():
    # The slow (list-property) path reads x,y,z with a hardcoded '<3f' unpack,
    # which is only correct when the first three fixed properties are exactly
    # float32 x, y, z. Any other layout must raise ValueError, not silently
    # return garbage coordinates.
    def write_ply(tmp, props, record_bytes):
        with open(tmp, 'wb') as f:
            _write_ply_header(f, 1, props)
            f.write(record_bytes)
            f.write(struct.pack('<B', 0))  # empty views list for the single vertex
        return tmp

    # color-first layout: x,y,z are not the first fixed properties
    p = write_ply('/tmp/claude_test_colorfirst.ply',
                  ['property uchar red', 'property uchar green', 'property uchar blue',
                   'property float x', 'property float y', 'property float z',
                   'property list uchar int views'],
                  struct.pack('<BBBfff', 1, 2, 3, 1.0, 2.0, 3.0))
    try:
        GtUtils.load_ply_xyz(p)
        raise SystemExit('FAIL: color-first slow-path PLY did not raise ValueError')
    except ValueError:
        pass

    # double-typed coordinates: right names, wrong byte width for '<3f'
    p = write_ply('/tmp/claude_test_doublexyz.ply',
                  ['property double x', 'property double y', 'property double z',
                   'property list uchar int views'],
                  struct.pack('<ddd', 1.0, 2.0, 3.0))
    try:
        GtUtils.load_ply_xyz(p)
        raise SystemExit('FAIL: double-xyz slow-path PLY did not raise ValueError')
    except ValueError:
        pass

    # conforming layout still works
    p = write_ply('/tmp/claude_test_conform.ply',
                  ['property float x', 'property float y', 'property float z',
                   'property list uchar int views'],
                  struct.pack('<fff', 1.0, 2.0, 3.0))
    r = GtUtils.load_ply_xyz(p)
    assert r.shape == (1, 3) and np.allclose(r, [[1.0, 2.0, 3.0]])

def test_load_ply_xyz_trailing_element(tmp='/tmp/claude_test_trailing.ply'):
    # Mimics ETH3D scan1.ply: `element vertex` (x,y,z only) followed by a
    # trailing `element camera` with many more properties. A parser that
    # doesn't scope `property` lines to the current element will inflate the
    # per-vertex record size and misread the vertex block.
    n = 6
    xyz = np.random.rand(n, 3).astype(np.float32)
    with open(tmp, 'wb') as f:
        extra = (b'element camera 1\n'
                 b'property float view_px\nproperty float view_py\nproperty float view_pz\n'
                 b'property int viewportx\nproperty int viewporty\n')
        _write_ply_header(f, n, ['property float x', 'property float y', 'property float z'], extra)
        for i in range(n):
            f.write(xyz[i].tobytes())
        f.write(struct.pack('<fffii', 1.0, 2.0, 3.0, 100, 200))  # camera record
    r = GtUtils.load_ply_xyz(tmp)
    assert r.shape == (n, 3)
    assert np.allclose(r, xyz, atol=1e-6)


# ---------------------------------------------------------------------------
# Step 5 regression test: empirically resolved ETH3D GT-depth convention.
#
# Method: project the (mlp-aligned) laser scan into each camera via the
# DISTORTED THIN_PRISM_FISHEYE model, z-buffer per distorted pixel (nearest
# point wins -- emulates the depth-map render ETH3D itself used to produce
# ground_truth_depth), and compare the winning point's camera-frame Z against
# the GT depth file sampled at that same distorted pixel. Real-data guarded:
# skips cleanly if /home/ubuntu/virginia/gt_bench/eth3d/<scene> is absent.
#
# Measured (see task-4-report.md for the full derivation, including the two
# false starts -- missing the scan_alignment.mlp registration, and missing
# COLMAP's equidistant-fisheye pre-step in the THIN_PRISM_FISHEYE model):
#   courtyard (outdoor): median rel err = ~0.0000% z-depth  vs 15.81% ray-dist  (n=143699 px)
#   office    (indoor):  median rel err = ~0.0000% z-depth  vs 17.09% ray-dist  (n=110241 px)
# Winning hypothesis: DISTORTED grid, Z-DEPTH (not ray distance). The
# "undistorted grid" hypotheses are structurally rejected even before scoring:
# read_eth3d_depth(path, w_undistorted, h_undistorted) raises (file byte count
# only matches the distorted resolution -- consistent with Task 2's finding).
# ---------------------------------------------------------------------------

_ETH3D_ROOT = '/home/ubuntu/virginia/gt_bench/eth3d'

def _qvec_to_rotmat(qw, qx, qy, qz):
    return np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2],
    ])

def _load_colmap_pose(images_txt, image_name):
    with open(images_txt) as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 10 and parts[-1].endswith(image_name):
                qw, qx, qy, qz = map(float, parts[1:5])
                tx, ty, tz = map(float, parts[5:8])
                cam_id = int(parts[8])
                return _qvec_to_rotmat(qw, qx, qy, qz), np.array([tx, ty, tz]), cam_id
    raise KeyError(image_name)

def _load_mlp_transform(mlp_path, mesh_label):
    import xml.etree.ElementTree as ET
    tree = ET.parse(mlp_path)
    for mesh in tree.getroot().iter('MLMesh'):
        if mesh.get('label') == mesh_label or mesh.get('filename') == mesh_label:
            vals = [float(x) for x in mesh.find('MLMatrix44').text.split()]
            return np.array(vals).reshape(4, 4)
    raise KeyError(mesh_label)

def _eth3d_convention_median_errors(scene, image_name, n_subsample=1_000_000, seed=0):
    scene_dir = os.path.join(_ETH3D_ROOT, scene)
    jpg_dir = os.path.join(scene_dir, 'dslr_calibration_jpg')

    R, t, cam_id = _load_colmap_pose(os.path.join(jpg_dir, 'images.txt'), image_name)
    cams_d = GtUtils.load_colmap_camera_params(os.path.join(jpg_dir, 'cameras.txt'))
    model_d, w_d, h_d, params_d = cams_d[cam_id]
    assert model_d == 'THIN_PRISM_FISHEYE', model_d

    depth_path = os.path.join(scene_dir, 'ground_truth_depth/dslr_images', image_name)
    depth_distorted = GtUtils.read_eth3d_depth(depth_path, w_d, h_d)

    # Structural rejection of the "undistorted grid" hypothesis.
    cams_u = GtUtils.load_colmap_camera_params(os.path.join(scene_dir, 'dslr_calibration_undistorted/cameras.txt'))
    _, w_u, h_u, _ = cams_u[cam_id]
    rejected = False
    try:
        GtUtils.read_eth3d_depth(depth_path, w_u, h_u)
    except AssertionError as e:
        rejected = '!= ' in str(e)  # confirms rejection is via the size check
    assert rejected, 'expected undistorted-grid read to fail (file size mismatch)'

    xyz = GtUtils.load_ply_xyz(os.path.join(scene_dir, 'dslr_scan_eval/scan1.ply'))
    rng = np.random.default_rng(seed)
    n = min(n_subsample, xyz.shape[0])
    xyz = xyz[rng.choice(xyz.shape[0], n, replace=False)]

    M = _load_mlp_transform(os.path.join(scene_dir, 'dslr_scan_eval/scan_alignment.mlp'), 'scan1.ply')
    xyz = (M[:3, :3] @ xyz.T).T + M[:3, 3]

    Xc = (R @ xyz.T).T + t
    z = Xc[:, 2]
    front = z > 0.05
    Xc, z = Xc[front], z[front]
    raydist = np.linalg.norm(Xc, axis=1)

    px, py = GtUtils._thin_prism_fisheye_distort(Xc[:, 0] / z, Xc[:, 1] / z, params_d)
    xi, yi = np.floor(px), np.floor(py)
    inb = np.isfinite(xi) & np.isfinite(yi) & (xi >= 0) & (xi < w_d) & (yi >= 0) & (yi < h_d)
    xi, yi, z, raydist = xi[inb].astype(np.int64), yi[inb].astype(np.int64), z[inb], raydist[inb]

    # z-buffer: keep the nearest-Z point per distorted pixel (emulates a depth render).
    flat = yi * w_d + xi
    order = np.argsort(z)
    flat, z, raydist = flat[order], z[order], raydist[order]
    _, first = np.unique(flat, return_index=True)
    win_flat, win_z, win_r = flat[first], z[first], raydist[first]
    win_y, win_x = win_flat // w_d, win_flat % w_d

    gt = depth_distorted[win_y, win_x]
    valid = np.isfinite(gt)
    gt, win_z, win_r = gt[valid], win_z[valid], win_r[valid]

    rel_err_z = np.abs(win_z - gt) / gt
    rel_err_r = np.abs(win_r - gt) / gt
    return float(np.median(rel_err_z)), float(np.median(rel_err_r)), int(gt.size)

_RUNS_ROOT = '/home/ubuntu/virginia/gt_bench/runs'

def test_view_image_names_real_data():
    # Real-data-guarded: dmap index -> image basename mapping via MvsUtils.
    scene_mvs = os.path.join(_RUNS_ROOT, 'eth3d_courtyard/scene.mvs')
    if not os.path.isfile(scene_mvs):
        print('SKIP test_view_image_names_real_data: scene.mvs not present')
        return
    names = GtUtils.view_image_names(scene_mvs)
    assert len(names) == 38, f'expected 38 courtyard views, got {len(names)}'
    assert 'DSC_0286.JPG' in names, 'known image basename missing'
    assert all(('/' not in n and '\\' not in n) for n in names), 'expected basenames only'
    print(f'view_image_names: {len(names)} names, first={names[0]}')

def _load_colmap_observations(images_txt, image_name, max_n=2000):
    # COLMAP images.txt: pose line, then a POINTS2D line of (x, y, point3d_id) triples.
    with open(images_txt) as f:
        lines = f.readlines()
    for i, line in enumerate(lines):
        if line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) >= 10 and parts[-1].endswith(image_name):
            obs = []
            pts = lines[i + 1].split()
            for j in range(0, len(pts), 3):
                pid = int(pts[j + 2])
                if pid != -1:
                    obs.append((float(pts[j]), float(pts[j + 1]), pid))
            return obs[:max_n]
    raise KeyError(image_name)

def test_thin_prism_fisheye_reprojection():
    # Automates the fisheye pre-step verification (previously only prose in the
    # GtUtils docstring/report): reproject COLMAP's own SfM points
    # (points3D.txt) into a distorted image via _thin_prism_fisheye_distort and
    # compare against the observed 2D keypoints recorded in images.txt.
    # Measured median error 0.64px with the equidistant-fisheye pre-step
    # (vs. 340px median without it -- the naive Brown-Conrady-only formula).
    scene_dir = os.path.join(_ETH3D_ROOT, 'courtyard')
    jpg_dir = os.path.join(scene_dir, 'dslr_calibration_jpg')
    if not os.path.isdir(jpg_dir):
        print('SKIP test_thin_prism_fisheye_reprojection: dslr_calibration_jpg not present')
        return
    image_name = 'DSC_0286.JPG'
    R, t, cam_id = _load_colmap_pose(os.path.join(jpg_dir, 'images.txt'), image_name)
    model, w, h, params = GtUtils.load_colmap_camera_params(os.path.join(jpg_dir, 'cameras.txt'))[cam_id]
    assert model == 'THIN_PRISM_FISHEYE', model

    obs = _load_colmap_observations(os.path.join(jpg_dir, 'images.txt'), image_name, max_n=2000)
    wanted = {pid for _, _, pid in obs}
    pts3d = {}
    with open(os.path.join(jpg_dir, 'points3D.txt')) as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.split()
            pid = int(parts[0])
            if pid in wanted:
                pts3d[pid] = np.array([float(parts[1]), float(parts[2]), float(parts[3])])

    errs = []
    for x_obs, y_obs, pid in obs:
        X = pts3d.get(pid)
        if X is None:
            continue
        Xc = R @ X + t
        if Xc[2] <= 0:
            continue
        px, py = GtUtils._thin_prism_fisheye_distort(Xc[0] / Xc[2], Xc[1] / Xc[2], params)
        errs.append(np.hypot(px - x_obs, py - y_obs))
    errs = np.asarray(errs)
    assert errs.size >= 500, f'only {errs.size} observations matched (need >=500)'
    med = float(np.median(errs))
    assert med < 2.0, f'median reprojection error {med:.2f}px >= 2px'
    print(f'fisheye reprojection: median err={med:.3f}px n={errs.size}')

def test_eth3d_convention_regression():
    scenes = [('courtyard', 'DSC_0286.JPG'), ('office', 'DSC_0219.JPG')]
    for scene, _ in scenes:
        if not os.path.isdir(os.path.join(_ETH3D_ROOT, scene, 'dslr_calibration_jpg')):
            print(f'SKIP test_eth3d_convention_regression: {scene} dslr_calibration_jpg not present')
            return
    for scene, image_name in scenes:
        med_z, med_ray, n_px = _eth3d_convention_median_errors(scene, image_name)
        assert n_px >= 100_000, f'{scene}: only {n_px} pixels compared (need >=100k)'
        assert med_z < 0.01, f'{scene}: z-depth median rel err {med_z:.4%} >= 1% threshold'
        assert med_ray > med_z, f'{scene}: ray-distance hypothesis unexpectedly beat z-depth'
        print(f'{scene}: median rel err z-depth={med_z:.4%} ray-dist={med_ray:.4%} n={n_px}')

def test_remap_eth3d_depth_to_undistorted_accuracy():
    # Shape/dtype smoke check plus an accuracy check: remap courtyard's
    # distorted-grid GT depth onto the undistorted grid, then independently
    # verify it against the scan projected with the PLAIN UNDISTORTED PINHOLE
    # camera (no distortion model at all) -- a second, independent path from
    # the one used in test_eth3d_convention_regression (which never leaves
    # the distorted grid). Measured: median rel err ~0.0000% on 84251 px
    # (courtyard) -- see task-4-report.md.
    scene, image_name, cam_id = 'courtyard', 'DSC_0286.JPG', 1
    scene_dir = os.path.join(_ETH3D_ROOT, scene)
    jpg_cams_txt = os.path.join(scene_dir, 'dslr_calibration_jpg/cameras.txt')
    if not os.path.isfile(jpg_cams_txt):
        print('SKIP test_remap_eth3d_depth_to_undistorted_accuracy: dslr_calibration_jpg not present')
        return

    cams_d = GtUtils.load_colmap_camera_params(jpg_cams_txt)
    cams_u = GtUtils.load_colmap_camera_params(os.path.join(scene_dir, 'dslr_calibration_undistorted/cameras.txt'))
    model_d, w_d, h_d, params_d = cams_d[cam_id]
    model_u, w_u, h_u, params_u = cams_u[cam_id]
    fx_u, fy_u, cx_u, cy_u = params_u

    depth_path = os.path.join(scene_dir, 'ground_truth_depth/dslr_images', image_name)
    depth_distorted = GtUtils.read_eth3d_depth(depth_path, w_d, h_d)
    depth_undist = GtUtils.remap_eth3d_depth_to_undistorted(depth_distorted, (w_d, h_d, params_d), (w_u, h_u, params_u))
    assert depth_undist.shape == (h_u, w_u)
    assert depth_undist.dtype == np.float32

    R, t, _ = _load_colmap_pose(os.path.join(scene_dir, 'dslr_calibration_undistorted/images.txt'), image_name)
    xyz = GtUtils.load_ply_xyz(os.path.join(scene_dir, 'dslr_scan_eval/scan1.ply'))
    rng = np.random.default_rng(1)
    n = min(1_000_000, xyz.shape[0])
    xyz = xyz[rng.choice(xyz.shape[0], n, replace=False)]
    M = _load_mlp_transform(os.path.join(scene_dir, 'dslr_scan_eval/scan_alignment.mlp'), 'scan1.ply')
    xyz = (M[:3, :3] @ xyz.T).T + M[:3, 3]

    Xc = (R @ xyz.T).T + t
    z = Xc[:, 2]
    front = z > 0.05
    Xc, z = Xc[front], z[front]

    # plain undistorted PINHOLE projection (no distortion model at all)
    px = fx_u * Xc[:, 0] / z + cx_u
    py = fy_u * Xc[:, 1] / z + cy_u
    xi, yi = np.floor(px).astype(np.int64), np.floor(py).astype(np.int64)
    inb = (xi >= 0) & (xi < w_u) & (yi >= 0) & (yi < h_u)
    xi, yi, z = xi[inb], yi[inb], z[inb]

    flat = yi * w_u + xi
    order = np.argsort(z)
    flat, z = flat[order], z[order]
    _, first = np.unique(flat, return_index=True)
    win_flat, win_z = flat[first], z[first]
    win_y, win_x = win_flat // w_u, win_flat % w_u

    gt = depth_undist[win_y, win_x]
    valid = np.isfinite(gt)
    gt, win_z = gt[valid], win_z[valid]
    rel_err = np.abs(win_z - gt) / gt
    n_px, med = rel_err.size, float(np.median(rel_err))
    assert n_px >= 50_000, f'only {n_px} pixels compared (need >=50k)'
    assert med < 0.01, f'remap median rel err {med:.4%} >= 1% threshold'
    print(f'remap accuracy: median rel err={med:.4%} n={n_px}')


if __name__ == '__main__':
    test_pfm_roundtrip(); test_gt_labels(); test_resize_nearest()
    test_load_ply_xyz_fixed_props(); test_load_ply_xyz_with_list_property()
    test_load_ply_xyz_slow_path_guards_xyz_layout(); test_load_ply_xyz_trailing_element()
    test_view_image_names_real_data(); test_thin_prism_fisheye_reprojection()
    test_eth3d_convention_regression(); test_remap_eth3d_depth_to_undistorted_accuracy()
    print('OK')
