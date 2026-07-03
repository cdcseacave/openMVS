import sys, os, json, struct, subprocess, tempfile, shutil
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import EvalFusionGT as E

_BMVS_ROOT = '/home/ubuntu/virginia/gt_bench/blendedmvs'
# Test scratch dir: never the repo or /tmp root -- see gt_bench/README.md data-root policy.
_SCRATCH_ROOT = '/home/ubuntu/virginia/gt_bench/tmp'
os.makedirs(_SCRATCH_ROOT, exist_ok=True)


def test_cube():
    # GT = unit cube surface; recon = same but one face missing + 100 far outliers
    v, f = E.make_cube_mesh()                       # helper returned for tests
    assert v.shape == (8, 3) and f.shape == (12, 3)
    gt = E.sample_mesh(v, f, 200000)
    rec = gt[gt[:, 2] < 0.999]                      # drop top face samples (~1/6)
    out = np.random.RandomState(0).rand(100, 3) * 10 + 5
    m = E.score_cloud(np.vstack([rec, out]), gt, tols=[0.01], gross_tol=0.5)
    assert 0.79 < m['completeness'][0.01] < 0.88    # ~5/6 of faces present
    assert m['gross_outlier_frac'] > 0.0003         # the 100 planted floaters


def test_nn_dist_grid_matches_bruteforce():
    # Directly validate the no-scipy grid-hash fallback against a naive
    # O(n*m) brute-force reference on a small random point set (scipy is not
    # installed in /home/ubuntu/miniconda3, so this is the path actually used
    # end-to-end by the cube test above and by real runs on this box).
    rng = np.random.default_rng(1)
    ref = rng.random((3000, 3))
    query = rng.random((800, 3))
    got = E._nn_dist_grid(query, ref)
    # brute force
    d2 = ((query[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
    want = np.sqrt(d2.min(axis=1))
    assert np.allclose(got, want, atol=1e-9), \
        f'max abs err={np.max(np.abs(got - want))}'


def test_nn_dist_grid_bimodal_density():
    # Reviewer's adversarial case (Critical review finding): on a bimodal-density cloud the grid
    # cell is sized for the AVERAGE density, so a query in the sparse region has its true NN
    # several cells away. The pre-fix code accepted the best candidate of the first ring that
    # contained ANY candidate -- returning e.g. the 3rd-nearest point (0.2764) instead of the
    # true NN (0.2026) when a closer point sat just outside the searched block. The certified
    # safe-radius termination rule (accept only once best <= ring*cell) must make ALL distances
    # exact, verified here against brute force for every query, sparse- and dense-region alike.
    rng = np.random.default_rng(42)
    ref = np.vstack([
        rng.random((50, 3)),                     # sparse cluster in [0,1]^3
        rng.random((2000, 3)) * 0.05 + 2.0,      # dense cluster in [2,2.05]^3
    ])
    query = np.vstack([
        rng.random((300, 3)) * 1.4 - 0.2,        # in/around the sparse cluster
        rng.random((100, 3)) * 0.25 + 1.9,       # in/around the dense cluster
        rng.random((100, 3)) * 3.0 - 0.5,        # spanning both + the empty gap between them
    ])
    got = E._nn_dist_grid(query, ref)
    d2 = ((query[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
    want = np.sqrt(d2.min(axis=1))
    assert np.allclose(got, want, atol=1e-9), \
        f'max abs err={np.max(np.abs(got - want))} at q={np.argmax(np.abs(got - want))}'


def test_nn_dist_grid_far_outliers():
    # Query points far outside the reference cloud's bounding box must still
    # resolve to their true (large) nearest-neighbor distance via the
    # brute-force straggler fallback, not silently return inf or a wrong hash
    # collision.
    rng = np.random.default_rng(2)
    ref = rng.random((500, 3))          # unit cube
    query = rng.random((20, 3)) * 2 + 20  # far away, in [20, 22]^3
    got = E._nn_dist_grid(query, ref)
    d2 = ((query[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
    want = np.sqrt(d2.min(axis=1))
    assert np.allclose(got, want, atol=1e-9)
    assert np.all(got > 15)  # sanity: genuinely far


def _write_obj(path, V, F):
    with open(path, 'w') as f:
        for x, y, z in V:
            f.write('v %.10f %.10f %.10f\n' % (x, y, z))
        for a, b, c in F:
            f.write('f %d %d %d\n' % (a + 1, b + 1, c + 1))


def test_load_mesh_dir_concatenation_synthetic():
    # Two tiny synthetic tiles with disjoint vertex sets; verify load_mesh()
    # on the directory concatenates vertices/faces with correctly offset
    # face indices (not just summed counts -- actual geometry round-trips).
    tmp = tempfile.mkdtemp(dir=_SCRATCH_ROOT)
    try:
        v0, f0 = E.make_cube_mesh()
        v1 = v0 + np.array([10.0, 0.0, 0.0])  # shifted copy -> distinguishable tile
        _write_obj(os.path.join(tmp, 'tile_0_0.obj'), v0, f0)
        _write_obj(os.path.join(tmp, 'tile_0_1.obj'), v1, f0)
        V, F = E.load_mesh(tmp)
        assert V.shape == (16, 3) and F.shape == (24, 3)
        # face indices must be valid and each tile's faces must reference
        # only that tile's vertex block (correct offsetting, not overlap)
        assert F.min() >= 0 and F.max() < len(V)
        tri_pts_first = V[F[0]]
        tri_pts_last = V[F[-1]]
        assert np.all(tri_pts_first < 5)      # first tile's triangle, unshifted cube
        assert np.all(tri_pts_last[:, 0] >= 10)  # last tile's triangle, shifted +10 in x
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_load_obj_real_tile():
    # Data-guarded: load exactly ONE real BlendedMVS tile .obj file.
    if not os.path.isdir(_BMVS_ROOT):
        print('SKIP test_load_obj_real_tile: BlendedMVS data not present at %s' % _BMVS_ROOT)
        return
    scenes = sorted(d for d in os.listdir(_BMVS_ROOT) if os.path.isdir(os.path.join(_BMVS_ROOT, d)) and not d.startswith('_'))
    if not scenes:
        print('SKIP test_load_obj_real_tile: no scene dirs under %s' % _BMVS_ROOT)
        return
    import glob
    tile = None
    for sc in scenes:
        cand = sorted(glob.glob(os.path.join(_BMVS_ROOT, sc, 'textured_mesh', 'tile_*.obj')))
        if cand:
            tile = cand[0]
            break
    if tile is None:
        print('SKIP test_load_obj_real_tile: no tile_*.obj found under any scene')
        return
    V, F = E.load_obj(tile)
    assert V.shape[0] > 0 and F.shape[0] > 0, f'{tile}: empty mesh'
    assert np.all(np.isfinite(V)), f'{tile}: non-finite vertex coords'
    assert F.min() >= 0 and F.max() < len(V), f'{tile}: face index out of range'
    print(f'test_load_obj_real_tile: {tile} -> {len(V)} verts, {len(F)} tris')


def test_load_mesh_dir_real_scene():
    # Data-guarded: concatenate an ENTIRE real BlendedMVS scene's tiled OBJ
    # mesh via the directory path and cross-check against independently
    # counted per-tile totals (validates the vertex-index-offset logic on
    # real multi-tile data, not just the synthetic 2-tile test above).
    if not os.path.isdir(_BMVS_ROOT):
        print('SKIP test_load_mesh_dir_real_scene: BlendedMVS data not present at %s' % _BMVS_ROOT)
        return
    import glob
    scenes = sorted(d for d in os.listdir(_BMVS_ROOT) if os.path.isdir(os.path.join(_BMVS_ROOT, d)) and not d.startswith('_'))
    scene_dir = None
    for sc in scenes:
        d = os.path.join(_BMVS_ROOT, sc, 'textured_mesh')
        if glob.glob(os.path.join(d, 'tile_*.obj')):
            scene_dir = d
            break
    if scene_dir is None:
        print('SKIP test_load_mesh_dir_real_scene: no textured_mesh with tiles found')
        return

    tiles = sorted(glob.glob(os.path.join(scene_dir, 'tile_*.obj')))
    want_nv = want_nf = 0
    for t in tiles:
        with open(t) as fh:
            for line in fh:
                if line.startswith('v '):
                    want_nv += 1
                elif line.startswith('f '):
                    want_nf += 1

    V, F = E.load_mesh(scene_dir)
    assert V.shape[0] == want_nv, f'{scene_dir}: vertex count {V.shape[0]} != {want_nv}'
    assert F.shape[0] == want_nf, f'{scene_dir}: face count {F.shape[0]} != {want_nf}'
    assert np.all(np.isfinite(V))
    assert F.min() >= 0 and F.max() < len(V)
    print(f'test_load_mesh_dir_real_scene: {scene_dir} ({len(tiles)} tiles) -> {V.shape[0]} verts, {F.shape[0]} tris')


def _write_ply_xyz(path, xyz):
    n = len(xyz)
    with open(path, 'wb') as f:
        f.write(b'ply\nformat binary_little_endian 1.0\n')
        f.write(f'element vertex {n}\n'.encode())
        f.write(b'property float x\nproperty float y\nproperty float z\n')
        f.write(b'end_header\n')
        xyz.astype('<f4').tofile(f)


def test_score_cloud_dedup_bias():
    # REVIEW FIX (round 3): score_cloud voxel-dedups the REFERENCE cloud of each direction at
    # finest_tol/4 before scoring (ref-side only: query-side dedup reweights the metric
    # population on non-uniformly dense clouds -- measured up to 0.04 absolute bias on this very
    # test before the fix -- see score_cloud's comment). This test bounds the metric bias dedup
    # introduces, against the exact no-dedup path, on NON-UNIFORM clouds dense enough to trigger
    # heavy dedup -- the committed assertion is the one score_cloud's comment promises:
    # |metric(dedup) - metric(exact)| < 0.005 absolute for completeness and accuracy at the
    # finest tolerance. The clouds are built so both metrics are genuinely threshold-sensitive
    # (mid-range values with real probability mass near the tolerance boundary), not saturated
    # at 0/1 where dedup bias would be invisible.
    rng = np.random.default_rng(11)
    tol = 0.05
    v = tol / 4.0

    # Geometry note: the GT is a gently CURVED surface (z = 0.2*sin(x)*sin(y)), not an exact
    # z=0 plane -- an exactly-planar cloud has (near-)zero extent on one axis, which degenerates
    # _nn_dist_grid's volume-based cell sizing (cell -> microscopic, certification radius
    # ring_cap*cell -> ~0) and pushes EVERY query to the O(n*m) brute-force fallback: still
    # exact, but hours-slow at these sizes. Real MVS scenes always have 3D extent; the curved
    # surface keeps the test in the realistic regime.
    def surf(x, y):
        return 0.2 * np.sin(x) * np.sin(y)

    # GT: dense half x<2.5 (lateral spacing ~0.0043 << v -> heavy dedup), sparse half x>2.5
    # (spacing ~0.0125 ~= v -> mild dedup): strongly non-uniform, per the review requirement.
    n_gd, n_gs = 280000, 80000
    gx = np.concatenate([rng.random(n_gd) * 2.5, 2.5 + rng.random(n_gs) * 2.5])
    gy = rng.random(n_gd + n_gs) * 5.0
    gt = np.column_stack([gx, gy, surf(gx, gy)])

    # rec: the same surface plus a one-sided height error |N(0, tol/2)| -- most points well
    # inside tol, ~5% beyond it, i.e. real probability mass right at the tolerance boundary,
    # where dedup bias would show in ACCURACY. Laterally non-uniform: dense x<2.5, THIN
    # (spacing ~ tol) for 2.5<x<4.85 -- the thin band spreads gt->rec NN distances across the
    # tolerance so COMPLETENESS is threshold-sensitive too -- and empty for x>4.85 (a narrow
    # uncovered stripe; kept narrow so its far-field queries stay cheap for the exact path).
    n_rd, n_rt = 200000, 3000
    rx = np.concatenate([rng.random(n_rd) * 2.5, 2.5 + rng.random(n_rt) * 2.35])
    ry = rng.random(n_rd + n_rt) * 5.0
    rec = np.column_stack([rx, ry, surf(rx, ry) + np.abs(rng.normal(0.0, tol / 2, n_rd + n_rt))])
    # dedup must actually trigger, on both clouds, or this test proves nothing
    n_gt_d, n_rec_d = len(E._voxel_dedup(gt, v)), len(E._voxel_dedup(rec, v))
    assert n_gt_d < 0.5 * len(gt), (n_gt_d, len(gt))
    assert n_rec_d < 0.8 * len(rec), (n_rec_d, len(rec))
    m = E.score_cloud(rec, gt, tols=[tol], gross_tol=10 * tol)
    # exact reference: identical metrics with NO dedup (nn_dist itself is exact -- proven by the
    # test_nn_dist_grid_* brute-force-match tests above)
    comp_exact = float((E.nn_dist(gt, rec) <= tol).mean())
    acc_exact = float((E.nn_dist(rec, gt) <= tol).mean())
    d_comp = abs(m['completeness'][tol] - comp_exact)
    d_acc = abs(m['accuracy'][tol] - acc_exact)
    print('test_score_cloud_dedup_bias: gt %d->%d rec %d->%d | comp %.4f vs exact %.4f (|d|=%.5f)'
          ' | acc %.4f vs exact %.4f (|d|=%.5f)'
          % (len(gt), n_gt_d, len(rec), n_rec_d,
             m['completeness'][tol], comp_exact, d_comp, m['accuracy'][tol], acc_exact, d_acc))
    assert d_comp < 0.005, d_comp
    assert d_acc < 0.005, d_acc
    # sanity: both metrics are mid-range (threshold-sensitive), not saturated at 0/1 where
    # dedup bias would be invisible
    assert 0.80 < m['completeness'][tol] < 0.995, m['completeness'][tol]
    assert 0.80 < m['accuracy'][tol] < 0.995, m['accuracy'][tol]


def test_cli_end_to_end_json_shape():
    # Synthetic end-to-end smoke of the actual CLI: writes a fused.ply and a
    # single-file GT .obj (unit cube), runs EvalFusionGT.py as a subprocess,
    # and checks the JSON shape/keys Task 7's aggregator depends on.
    tmp = tempfile.mkdtemp(dir=_SCRATCH_ROOT)
    try:
        v, f = E.make_cube_mesh()
        obj_path = os.path.join(tmp, 'gt_cube.obj')
        _write_obj(obj_path, v, f)

        gt = E.sample_mesh(v, f, 20000)
        rec = gt[gt[:, 2] < 0.999]
        rng = np.random.RandomState(0)
        outl = rng.rand(50, 3) * 10 + 5
        fused_path = os.path.join(tmp, 'fused.ply')
        _write_ply_xyz(fused_path, np.vstack([rec, outl]).astype(np.float32))

        json_path = os.path.join(tmp, 'out.json')
        script = os.path.join(os.path.dirname(__file__), '..', 'EvalFusionGT.py')
        r = subprocess.run(
            [sys.executable, script, fused_path, '--gt-mesh', obj_path,
             '--n-samples', '20000', '--json', json_path],
            capture_output=True, text=True)
        assert r.returncode == 0, f'stdout={r.stdout}\nstderr={r.stderr}'

        with open(json_path) as fh:
            out = json.load(fh)
        for key in ('completeness', 'accuracy', 'gross_outlier_frac', 'tol_abs', 'diag', 'n_rec', 'n_gt'):
            assert key in out, f'missing key {key} in {out}'
        assert set(out['completeness'].keys()) == {'0.0025', '0.005', '0.01'}
        assert set(out['accuracy'].keys()) == {'0.0025', '0.005', '0.01'}
        assert set(out['tol_abs'].keys()) == {'0.0025', '0.005', '0.01'}
        for k, frac in (('0.0025', 0.0025), ('0.005', 0.005), ('0.01', 0.01)):
            assert abs(out['tol_abs'][k] - frac * out['diag']) < 1e-9
        assert out['n_rec'] == len(rec) + 50
        assert out['diag'] > 0
        assert 0.0 <= out['completeness']['0.01'] <= 1.0
        assert out['gross_outlier_frac'] > 0.0
        print('test_cli_end_to_end_json_shape:', json.dumps(out, indent=2))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    test_cube()
    test_nn_dist_grid_matches_bruteforce()
    test_nn_dist_grid_bimodal_density()
    test_nn_dist_grid_far_outliers()
    test_load_mesh_dir_concatenation_synthetic()
    test_load_obj_real_tile()
    test_load_mesh_dir_real_scene()
    test_score_cloud_dedup_bias()
    test_cli_end_to_end_json_shape()
    print('OK')
