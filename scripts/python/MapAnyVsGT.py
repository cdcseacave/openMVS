#!/usr/bin/env python3
"""
Task 8: MapAnything-vs-GT calibration study.

All the pre-benchmark tuning of the confidence-recalibration + few-view fusion rescue relied on
MapAnything mono-depth voxel-fused pseudo-GT clouds (`magt.ply`, built by MapAnyInferMV.py +
MapAnyVoxelFuse.py) as a stand-in for ground truth, because no real GT was available at the time.
Now that gt_bench has real GT (ETH3D laser scans, BlendedMVS textured meshes), this script measures
how trustworthy that pseudo-GT actually was, by scoring it against the real thing.

Two independent scorings, both against REAL ground truth (never against each other):

  (1) PER-VIEW DEPTH ACCURACY. The MapAnything per-view depth is compared to real GT depth AFTER the
      SAME per-view scale alignment MapAnyVoxelFuse.py itself applies before voxel-fusing (median
      ratio MVS/MA over MVS-confident pixels, `scale_align_view` below === MapAnyVoxelFuse.py's own
      scale-alignment code) -- i.e. this scores the EXACT depth quantity that got fused into
      magt.ply, not the model's raw un-anchored output. Reports median |d_MA-d_GT|/d_GT and the
      fraction of jointly-valid pixels with relative error >1%/3%/10%, per view and pooled per scene.
      GT loading mirrors EvalConfidence.py's GT mode / EvalFusionGT.py's run_gt_mode (GtUtils:
      read_pfm for BlendedMVS, remap_eth3d_depth_to_undistorted + gt_cache for ETH3D).

  (2) PER-CLOUD ACCURACY. GtUtils.load_ply_xyz + EvalFusionGT.score_cloud (or, for ETH3D, the
      official ETH3DMultiViewEvaluation tool via gt_bench/eth3d_eval.sh, since ETH3D GT is a
      registered laser scan, not a mesh -- score_cloud has no mesh to sample against there) scores
      magt.ply against real GT at the exact tolerances gt_bench/run_scene.sh uses for cloud_w0/w3,
      alongside the SAME numbers for that scene's OpenMVS `cloud_w3.ply` for context (reused from an
      existing gt_bench/results/<scene>_L<r>_fuse_w3.json when --cloud-w3-json is given and exists,
      to stay byte-identical with the recorded BASELINE numbers instead of a stochastically-resampled
      recomputation; otherwise computed fresh the same way).

Usage:
  python MapAnyVsGT.py <scene> --gt-format {eth3d,blendedmvs} \\
      --dmap-dir <run dir with depth*.dmap> --mono-dir <MapAnyInferMV.py npz output dir> \\
      --magt-ply <MapAnyVoxelFuse.py output ply> \\
      --gt-depth-dir <GT root: ETH3D scene dir | BlendedMVS scene dir> \\
      [--gt-scene-dir <ETH3D scene dir with dslr_scan_eval/, default = --gt-depth-dir>] \\
      [--gt-mesh <BlendedMVS textured_mesh dir, default = <gt-depth-dir>/textured_mesh>] \\
      [--gt-cache-dir <dir, ETH3D remap cache -- point at the SAME cache run_scene.sh/EvalConfidence.py \\
                        already populated for this scene/resolution to skip the expensive remap>] \\
      [--cloud-w3-ply <path>] [--cloud-w3-json <existing gt_bench/results/..._fuse_w3.json>] \\
      [--minconf 0.1] [--n-samples 2000000] [--nosparse-witness] [--json out.json] [--quiet]

--nosparse-witness only affects the JSON/report label (marks this scene's MapAnything witness as
having been generated WITHOUT sparse-depth conditioning, e.g. because scene.mvs has zero SfM sparse
vertices -- see MapAnyInferMV_nosparse.py); it does not change any computation here.
"""
import os
import sys
import glob
import json
import argparse
import subprocess
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from MvsUtils import loadDMAP
import GtUtils
import EvalFusionGT

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ETH3D_EVAL_SH = os.path.join(REPO_ROOT, 'gt_bench', 'eth3d_eval.sh')

RELERR_THRESHOLDS = (0.01, 0.03, 0.10)  # >1% / >3% / >10% relative depth error


def fmt(x):
    return 'n/a' if x != x else '%.4f' % x  # x != x detects nan


# ---------------------------------------------------------------------------
# (1) Per-view depth accuracy
# ---------------------------------------------------------------------------

def scale_align_view(d_dmap, npz, minconf):
    """Reproduces MapAnyVoxelFuse.py's per-view scale alignment EXACTLY (same MINCONF-gated
    median(mvs/ma) ratio over MVS-confident pixels), so the depth scored here is the identical
    quantity that got voxel-fused into magt.ply -- not the model's raw un-anchored output.
    Returns (ma_scaled or None, n_anchor_pixels, scale)."""
    mvs = np.asarray(d_dmap['depth_map'], np.float64)
    conf = np.asarray(d_dmap['confidence_map'], np.float64)
    ma = np.asarray(npz['depth'], np.float64)
    mask = np.asarray(npz['mask'], bool)
    ok = mask & (mvs > 0) & (ma > 0) & (conf >= minconf)
    n_anchor = int(ok.sum())
    if n_anchor < 200:
        return None, n_anchor, float('nan')
    s = float(np.median(mvs[ok] / ma[ok]))
    return ma * s, n_anchor, s


def _colmap_image_camera_id(images_txt, image_name):
    """COLMAP images.txt field 9 (camera_id) for `image_name`, matched as a path suffix (same
    approach as EvalFusionGT.py's run_gt_mode / EvalConfidence.py's GT mode)."""
    with open(images_txt) as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 10 and parts[-1].endswith(image_name):
                return int(parts[8])
    return None


def load_gt_depth_view(image_name, d_shape, gt_format, gt_depth_dir, eth3d_ctx, gt_cache_dir):
    """Load+grid-align real GT depth for one view onto d_shape=(h,w). Returns None if unavailable.
    Mirrors EvalConfidence.py's `_load_gt_depth_view` / EvalFusionGT.py's equivalent."""
    if gt_format == 'blendedmvs':
        stem = os.path.splitext(image_name)[0]
        pfm_path = os.path.join(gt_depth_dir, 'rendered_depth_maps', stem + '.pfm')
        if not os.path.isfile(pfm_path):
            return None
        return GtUtils.resize_depth_nearest(GtUtils.read_pfm(pfm_path), d_shape)

    # eth3d
    cams_d, cams_u, images_txt_d = eth3d_ctx
    h, w = d_shape
    cache_path = os.path.join(
        gt_cache_dir, '%s_%dx%d.npy' % (image_name.replace(os.sep, '_'), h, w)) if gt_cache_dir else None
    if cache_path and os.path.isfile(cache_path):
        return np.load(cache_path)
    cam_id = _colmap_image_camera_id(images_txt_d, image_name)
    if cam_id is None or cam_id not in cams_d or cam_id not in cams_u:
        return None
    _, w_d, h_d, params_d = cams_d[cam_id]
    _, w_u, h_u, params_u = cams_u[cam_id]
    depth_path = os.path.join(gt_depth_dir, 'ground_truth_depth', 'dslr_images', image_name)
    if not os.path.isfile(depth_path):
        return None
    depth_distorted = GtUtils.read_eth3d_depth(depth_path, w_d, h_d)
    depth_undist = GtUtils.remap_eth3d_depth_to_undistorted(
        depth_distorted, (w_d, h_d, params_d), (w_u, h_u, params_u))
    d_gt = GtUtils.resize_depth_nearest(depth_undist, d_shape)
    if cache_path:
        os.makedirs(gt_cache_dir, exist_ok=True)
        np.save(cache_path, d_gt)
    return d_gt


def run_depth_eval(args):
    dmaps = sorted(glob.glob(os.path.join(args.dmap_dir, 'depth*.dmap')))
    if not dmaps:
        print('error: no depth*.dmap files in %s' % args.dmap_dir); sys.exit(1)

    eth3d_ctx = None
    if args.gt_format == 'eth3d':
        jpg_dir = os.path.join(args.gt_depth_dir, 'dslr_calibration_jpg')
        cams_d = GtUtils.load_colmap_camera_params(os.path.join(jpg_dir, 'cameras.txt'))
        cams_u = GtUtils.load_colmap_camera_params(
            os.path.join(args.gt_depth_dir, 'dslr_calibration_undistorted', 'cameras.txt'))
        eth3d_ctx = (cams_d, cams_u, os.path.join(jpg_dir, 'images.txt'))

    per_view = []
    pool_rel = []
    n_no_npz = n_no_scale = n_no_gt = n_too_few = 0
    for dm in dmaps:
        stem = os.path.splitext(os.path.basename(dm))[0]
        npz_path = os.path.join(args.mono_dir, stem + '.npz')
        if not os.path.isfile(npz_path):
            n_no_npz += 1
            continue
        d = loadDMAP(dm)
        npz = np.load(npz_path)
        ma_scaled, n_anchor, scale = scale_align_view(d, npz, args.minconf)
        if ma_scaled is None:
            n_no_scale += 1
            continue
        image_name = os.path.basename(d['file_name'])
        d_gt = load_gt_depth_view(image_name, ma_scaled.shape, args.gt_format, args.gt_depth_dir,
                                   eth3d_ctx, args.gt_cache_dir)
        if d_gt is None:
            n_no_gt += 1
            continue
        mask = np.asarray(npz['mask'], bool)
        valid = mask & (ma_scaled > 0) & np.isfinite(d_gt) & (d_gt > 0)
        n_valid = int(valid.sum())
        if n_valid < 50:
            n_too_few += 1
            continue
        rel = np.abs(ma_scaled[valid] - d_gt[valid]) / d_gt[valid]
        pool_rel.append(rel)
        row = dict(image=image_name, n_valid=n_valid, n_anchor=n_anchor, scale=scale,
                   median_rel_err=float(np.median(rel)))
        for t in RELERR_THRESHOLDS:
            row['frac_gt_%dpct' % round(t * 100)] = float(np.mean(rel > t))
        per_view.append(row)
        if not args.quiet:
            print('%-40s n_valid=%7d scale=%.4f  median_rel_err=%s  >1%%=%s >3%%=%s >10%%=%s' % (
                image_name, n_valid, scale, fmt(row['median_rel_err']),
                fmt(row['frac_gt_1pct']), fmt(row['frac_gt_3pct']), fmt(row['frac_gt_10pct'])))

    if pool_rel:
        pooled_rel = np.concatenate(pool_rel)
        pooled = dict(n_pixels=int(pooled_rel.size), median_rel_err=float(np.median(pooled_rel)))
        for t in RELERR_THRESHOLDS:
            pooled['frac_gt_%dpct' % round(t * 100)] = float(np.mean(pooled_rel > t))
    else:
        pooled = dict(n_pixels=0, median_rel_err=float('nan'))
        for t in RELERR_THRESHOLDS:
            pooled['frac_gt_%dpct' % round(t * 100)] = float('nan')

    coverage = dict(n_dmaps=len(dmaps), n_scored=len(per_view), n_no_npz=n_no_npz,
                     n_no_scale_anchor=n_no_scale, n_no_gt=n_no_gt, n_too_few_valid_px=n_too_few)
    return per_view, pooled, coverage


# ---------------------------------------------------------------------------
# (2) Per-cloud accuracy
# ---------------------------------------------------------------------------

def score_cloud_eth3d(ply_path, gt_scene_dir, out_json):
    subprocess.run(['bash', ETH3D_EVAL_SH, ply_path, gt_scene_dir, out_json], check=True)
    with open(out_json) as f:
        return json.load(f)


def score_cloud_blendedmvs(ply_path, gt_mesh_dir, n_samples, out_json=None, seed=0):
    rec = GtUtils.load_ply_xyz(ply_path)
    V, F = EvalFusionGT.load_mesh(gt_mesh_dir)
    gt = EvalFusionGT.sample_mesh(V, F, n_samples, seed=seed)
    diag = float(np.linalg.norm(gt.max(0) - gt.min(0)))
    abs_tols = [frac * diag for _, frac in EvalFusionGT.DEFAULT_TOL_FRACS]
    gross_tol = EvalFusionGT.DEFAULT_GROSS_FRAC * diag
    m = EvalFusionGT.score_cloud(rec, gt, tols=abs_tols, gross_tol=gross_tol)
    out = {
        'completeness': {label: m['completeness'][t] for (label, _), t in zip(EvalFusionGT.DEFAULT_TOL_FRACS, abs_tols)},
        'accuracy': {label: m['accuracy'][t] for (label, _), t in zip(EvalFusionGT.DEFAULT_TOL_FRACS, abs_tols)},
        'gross_outlier_frac': m['gross_outlier_frac'],
        'tol_abs': {label: t for (label, _), t in zip(EvalFusionGT.DEFAULT_TOL_FRACS, abs_tols)},
        'diag': diag, 'n_rec': m['n_rec'], 'n_gt': m['n_gt'],
    }
    if out_json:
        with open(out_json, 'w') as f:
            json.dump(out, f, indent=2)
    return out


def score_cloud(args, ply_path, tag):
    out_json = os.path.join(os.path.dirname(os.path.abspath(args.json)) if args.json else '/tmp',
                             '%s_%s_cloudscore.json' % (args.scene, tag))
    if args.gt_format == 'eth3d':
        gt_scene_dir = args.gt_scene_dir or args.gt_depth_dir
        return score_cloud_eth3d(ply_path, gt_scene_dir, out_json)
    gt_mesh = args.gt_mesh or os.path.join(args.gt_depth_dir, 'textured_mesh')
    return score_cloud_blendedmvs(ply_path, gt_mesh, args.n_samples, out_json)


def print_cloud_table(label, res):
    tol = res.get('tol_abs', {})
    comp = res.get('completeness', {})
    acc = res.get('accuracy', {})
    keys = sorted(tol.keys(), key=lambda k: tol[k])
    cells = ' '.join('%s=%.4f' % (k, comp.get(k, float('nan'))) for k in keys)
    cells_a = ' '.join('%s=%.4f' % (k, acc.get(k, float('nan'))) for k in keys)
    print('%-14s n_rec=%-9d n_gt=%-11d completeness: %s' % (label, res.get('n_rec', 0), res.get('n_gt', 0), cells))
    print('%-14s %-9s %-11s accuracy:     %s' % ('', '', '', cells_a))
    print('%-14s gross_outlier_frac=%.4f' % ('', res.get('gross_outlier_frac', float('nan'))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scene')
    ap.add_argument('--gt-format', required=True, choices=('eth3d', 'blendedmvs'))
    ap.add_argument('--dmap-dir', required=True, help='dir with depth*.dmap (the MVS run whose confident '
                     'pixels anchor the MapAnything scale, and whose file_name fields map view->image)')
    ap.add_argument('--mono-dir', required=True, help='MapAnyInferMV(.py|_nosparse.py) npz output dir')
    ap.add_argument('--magt-ply', required=True, help='MapAnyVoxelFuse.py pseudo-GT cloud to score')
    ap.add_argument('--gt-depth-dir', required=True,
                     help='GT root: ETH3D scene dir (has ground_truth_depth/, dslr_calibration_*/) or '
                          'BlendedMVS scene dir (has rendered_depth_maps/, textured_mesh/)')
    ap.add_argument('--gt-scene-dir', default=None,
                     help='ETH3D only: dir with dslr_scan_eval/scan_alignment.mlp (default = --gt-depth-dir)')
    ap.add_argument('--gt-mesh', default=None,
                     help='BlendedMVS only: textured_mesh dir (default = <gt-depth-dir>/textured_mesh)')
    ap.add_argument('--gt-cache-dir', default=None,
                     help='ETH3D only: remap cache dir -- point at the SAME cache run_scene.sh/'
                          'EvalConfidence.py already populated for this scene/resolution to reuse it')
    ap.add_argument('--cloud-w3-ply', default=None, help='OpenMVS cloud_w3.ply for context scoring')
    ap.add_argument('--cloud-w3-json', default=None,
                     help='existing gt_bench/results/<scene>_L<r>_fuse_w3.json -- if present, reused '
                          'verbatim instead of recomputing (stays byte-identical to the recorded BASELINE)')
    ap.add_argument('--minconf', type=float, default=0.1, help='MVS confidence gate for the scale anchor '
                     '(must match MapAnyVoxelFuse.py MINCONF, default 0.1)')
    ap.add_argument('--n-samples', type=int, default=2_000_000, help='BlendedMVS GT mesh surface samples')
    ap.add_argument('--nosparse-witness', action='store_true',
                     help='label only: this scene\'s MapAnything witness was generated WITHOUT '
                          'sparse-depth conditioning (scene.mvs had zero SfM vertices)')
    ap.add_argument('--json', default=None, help='write the combined result JSON here')
    ap.add_argument('--quiet', action='store_true', help='suppress per-view depth lines')
    args = ap.parse_args()

    print('=' * 100)
    print('MapAnyVsGT: scene=%s  gt_format=%s  nosparse_witness=%s' % (
        args.scene, args.gt_format, args.nosparse_witness))
    print('=' * 100)

    print('\n---- (1) per-view depth accuracy (MapAnything, MVS-anchor-scaled, vs real GT) ----')
    per_view, pooled_depth, coverage = run_depth_eval(args)
    print('\ncoverage: n_dmaps=%d n_scored=%d n_no_npz=%d n_no_scale_anchor=%d n_no_gt=%d n_too_few_valid_px=%d' % (
        coverage['n_dmaps'], coverage['n_scored'], coverage['n_no_npz'], coverage['n_no_scale_anchor'],
        coverage['n_no_gt'], coverage['n_too_few_valid_px']))
    print('POOLED (%d views, %d px): median_rel_err=%s  >1%%=%s  >3%%=%s  >10%%=%s' % (
        coverage['n_scored'], pooled_depth['n_pixels'], fmt(pooled_depth['median_rel_err']),
        fmt(pooled_depth['frac_gt_1pct']), fmt(pooled_depth['frac_gt_3pct']), fmt(pooled_depth['frac_gt_10pct'])))

    print('\n---- (2) per-cloud accuracy vs real GT ----')
    print('scoring magt.ply (%s) ...' % args.magt_ply)
    magt_score = score_cloud(args, args.magt_ply, 'magt')
    print_cloud_table('magt.ply', magt_score)

    w3_score = None
    if args.cloud_w3_json and os.path.isfile(args.cloud_w3_json):
        print('\nreusing existing cloud_w3 score: %s' % args.cloud_w3_json)
        with open(args.cloud_w3_json) as f:
            w3_score = json.load(f)
    elif args.cloud_w3_ply:
        print('\nscoring cloud_w3.ply (%s) ...' % args.cloud_w3_ply)
        w3_score = score_cloud(args, args.cloud_w3_ply, 'cloud_w3')
    if w3_score is not None:
        print_cloud_table('cloud_w3.ply', w3_score)
    else:
        print('\n(no cloud_w3 context score requested)')

    out = dict(scene=args.scene, gt_format=args.gt_format, nosparse_witness=bool(args.nosparse_witness),
               depth_per_view=per_view, depth_pooled=pooled_depth, depth_coverage=coverage,
               cloud_magt=magt_score, cloud_w3_context=w3_score)
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)) or '.', exist_ok=True)
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=2)
        print('\nJSON written to %s' % args.json)
    return out


if __name__ == '__main__':
    main()
