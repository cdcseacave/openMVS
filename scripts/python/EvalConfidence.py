#!/usr/bin/env python3
"""
Evaluate how well a depth-map confidence-map predicts which depths survive fusion.

It scores the confidence-map stored in each .dmap (loaded via MvsUtils.loadDMAP) against the
per-pixel inlier/outlier labels exported by OpenMVS (DepthMapsData::LabelFusionInliers, run with
DensifyPointCloud --export-fusion-labels), which writes a .flabel (uint8) and .fsupport (uint16)
map beside every .dmap.

Labels (kept in sync with SceneDensify.cpp):
  0 INVALID, 1 OUTLIER, 2 AMBIGUOUS, 3 WEAK_INLIER, 4 CONFIDENT_INLIER

Two regimes:
  STRICT  : positives = CONFIDENT_INLIER,            negatives = OUTLIER   (would-survive vs contradicted)
  LENIENT : positives = CONFIDENT_INLIER|WEAK_INLIER, negatives = OUTLIER   (rewards recovering fusion's
            few-view false-negatives)

Metrics: ROC-AUC, PR-AUC (average precision), precision/recall at the fusion gate t (default 0.1),
false-negative-recall (fraction of WEAK_INLIER kept >= t), Spearman(conf, support), Brier, ECE.

Usage:
  python EvalConfidence.py <dmap_dir> [--labels-dir DIR] [--threshold 0.1]
                           [--max-pixels-per-image N] [--csv out.csv] [--quiet]

Compare methods by pointing --dmap-dir at runs produced with different --postprocess-dmaps while
keeping --labels-dir fixed (labels are geometry-only and identical across confidence variants).
No dependency beyond numpy (and MvsUtils for .dmap parsing).

GT-depth mode (additive; keeps the .flabel/.fsupport path above fully working):
  python EvalConfidence.py <dmap_dir> --gt-depth-dir <dir> --gt-format {blendedmvs,eth3d}
                           --scene-mvs <scene.mvs> [--rel-tol 0.01] [--abs-tol 0.0] [--json out.json]

Instead of consuming the fusion-exported .flabel/.fsupport maps, this mode derives per-pixel
inlier/outlier labels directly from ground-truth depth (GtUtils.gt_labels), matching each dmap
to its scene image via the dmap's own embedded 'file_name' field (NOT a positional lookup into
--scene-mvs's images array -- view.GetID() (the dmap filename's numeric ID) is not guaranteed
to equal that array position; confirmed reversed on a COLMAP-imported scene, see run_gt_mode).
It reuses the SAME roc_auc/pr_auc/prec_recall_at/ece metric machinery as the STRICT flabel
regime above (LENIENT has no GT counterpart: GT labels are binary, there is no WEAK_INLIER).
  blendedmvs : GT file = <gt-depth-dir>/rendered_depth_maps/<image-stem>.pfm (GtUtils.read_pfm)
  eth3d      : GT file = <gt-depth-dir>/ground_truth_depth/dslr_images/<image-name>, on the
               DISTORTED grid as z-depth (GtUtils.read_eth3d_depth); remapped onto the undistorted
               dmap grid with GtUtils.remap_eth3d_depth_to_undistorted (needs the COLMAP camera
               models under <gt-depth-dir>/dslr_calibration_{jpg,undistorted}/). The remap is
               moderately expensive, so results are cached per view as .npy under
               <gt-depth-dir>/../gt_cache/ (override with --gt-cache-dir), keyed by image name +
               dmap shape -- NOT written into the repo.
--json writes {scene, mode, per_view: [...], pooled: {roc_auc, pr_auc, p_at_01, r_at_01,
spearman, brier, ece, n_labeled}} for Task 7's aggregator.
"""
import os
import sys
import re
import glob
import json
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from MvsUtils import loadDMAP
import GtUtils

INVALID, OUTLIER, AMBIGUOUS, WEAK_INLIER, CONFIDENT_INLIER = 0, 1, 2, 3, 4
LMAP_MAGIC = 0x50414D4C  # 'LMAP'


def load_raw_map(path):
    """Load a .flabel/.fsupport raw map written by SaveRawMap (SceneDensify.cpp)."""
    with open(path, 'rb') as f:
        hdr = np.frombuffer(f.read(16), dtype=np.int32)
        if hdr.size != 4 or int(hdr[0]) != LMAP_MAGIC:
            raise ValueError('bad raw-map header in %s' % path)
        w, h, elem = int(hdr[1]), int(hdr[2]), int(hdr[3])
        dt = {1: np.uint8, 2: np.uint16, 4: np.float32}[elem]
        data = np.frombuffer(f.read(w * h * np.dtype(dt).itemsize), dtype=dt)
        return data.reshape(h, w)


def roc_auc(scores, labels):
    """ROC-AUC via the Mann-Whitney U statistic (rank-based, handles ties)."""
    n = labels.size
    npos = int(labels.sum())
    nneg = n - npos
    if npos == 0 or nneg == 0:
        return float('nan')
    order = np.argsort(scores, kind='mergesort')
    s = scores[order]
    ranks = np.empty(n, dtype=np.float64)
    i = 0
    while i < n:  # average ranks within tie groups
        j = i
        while j + 1 < n and s[j + 1] == s[i]:
            j += 1
        ranks[i:j + 1] = 0.5 * (i + j) + 1.0
        i = j + 1
    rank_pos = ranks[labels[order] == 1].sum()
    return (rank_pos - npos * (npos + 1) / 2.0) / (npos * nneg)


def pr_auc(scores, labels):
    """Average precision (area under precision-recall, step interpolation)."""
    npos = int(labels.sum())
    if npos == 0:
        return float('nan')
    order = np.argsort(-scores, kind='mergesort')
    y = labels[order].astype(np.float64)
    tp = np.cumsum(y)
    fp = np.cumsum(1.0 - y)
    precision = tp / np.maximum(tp + fp, 1e-12)
    recall = tp / npos
    ap = 0.0
    prev_r = 0.0
    for k in range(y.size):
        if y[k] == 1:  # recall increases only at a true positive
            ap += (recall[k] - prev_r) * precision[k]
            prev_r = recall[k]
    return ap


def spearman(a, b):
    if a.size < 2:
        return float('nan')
    ra = np.argsort(np.argsort(a, kind='mergesort')).astype(np.float64)
    rb = np.argsort(np.argsort(b, kind='mergesort')).astype(np.float64)
    ra -= ra.mean(); rb -= rb.mean()
    den = np.sqrt((ra * ra).sum() * (rb * rb).sum())
    return float((ra * rb).sum() / den) if den > 0 else float('nan')


def prec_recall_at(scores, labels, t):
    pred = scores >= t
    tp = int(np.sum(pred & (labels == 1)))
    fp = int(np.sum(pred & (labels == 0)))
    fn = int(np.sum(~pred & (labels == 1)))
    prec = tp / (tp + fp) if (tp + fp) else float('nan')
    rec = tp / (tp + fn) if (tp + fn) else float('nan')
    return prec, rec


def ece(scores, labels, nbins=10):
    edges = np.linspace(0.0, 1.0, nbins + 1)
    n = labels.size
    e = 0.0
    for k in range(nbins):
        lo, hi = edges[k], edges[k + 1]
        m = (scores >= lo) & (scores < hi if k < nbins - 1 else scores <= hi)
        if not np.any(m):
            continue
        e += (m.sum() / n) * abs(scores[m].mean() - labels[m].mean())
    return e


def eval_image(conf, label, support, t, max_pixels):
    """Return (strict_dict, lenient_dict, accumulators) for one image, or None if no usable pixels."""
    valid = label != INVALID
    if not np.any(valid):
        return None
    conf = conf[valid].astype(np.float64)
    label = label[valid]
    support = support[valid].astype(np.float64)

    out = {}
    for name, pos_mask in (('strict', label == CONFIDENT_INLIER),
                           ('lenient', (label == CONFIDENT_INLIER) | (label == WEAK_INLIER))):
        sel = pos_mask | (label == OUTLIER)
        c, y = conf[sel], pos_mask[sel].astype(np.int32)
        if max_pixels and c.size > max_pixels:
            idx = np.linspace(0, c.size - 1, max_pixels).astype(np.int64)
            c, y = c[idx], y[idx]
        if y.sum() == 0 or y.sum() == y.size:
            out[name] = None
            continue
        prec, rec = prec_recall_at(c, y, t)
        out[name] = dict(roc=roc_auc(c, y), pr=pr_auc(c, y), prec=prec, rec=rec,
                         brier=float(np.mean((c - y) ** 2)), ece=ece(c, y), n=int(c.size),
                         scores=c, ys=y)
    # false-negative recall: fraction of WEAK_INLIER pixels kept >= t (recovered few-view inliers)
    weak = label == WEAK_INLIER
    fnr = float(np.mean(conf[weak] >= t)) if np.any(weak) else float('nan')
    # rank-correlation of confidence with view-support over all inliers
    inl = (label == CONFIDENT_INLIER) | (label == WEAK_INLIER)
    sp = spearman(conf[inl], support[inl]) if int(inl.sum()) >= 2 else float('nan')
    return out, dict(fnr=fnr, spearman=sp, n_weak=int(weak.sum()))


def metrics_from_gt(d_est, conf, d_gt, rel_tol=0.01, abs_tol=0.0, t=0.1, max_pixels=0):
    """GT-depth counterpart of eval_image's STRICT regime: labels come from GtUtils.gt_labels
    (d_est vs. ground-truth depth) instead of the exported .flabel/.fsupport maps. d_gt is resized
    (nearest) onto d_est's grid when shapes differ. Reuses the same roc_auc/pr_auc/prec_recall_at/
    ece machinery as eval_image; returns a flat dict (not a (regimes, extra) pair) keyed with the
    JSON-schema names (roc_auc, pr_auc, p_at_01, r_at_01, spearman, brier, ece, n_labeled) plus the
    raw (scores, ys) arrays for pooling across views.

    'spearman' has no direct GT-mode analogue of flabel's spearman(conf, view-support) -- there is
    no separate support signal here -- so it is the rank correlation of confidence against the
    binary GT inlier/outlier label itself (rank-biserial), over the same labeled-pixel selection
    used for the other metrics."""
    if d_gt.shape != d_est.shape:
        d_gt = GtUtils.resize_depth_nearest(d_gt, d_est.shape)
    inlier, outlier, _ = GtUtils.gt_labels(d_est, d_gt, rel_tol, abs_tol)
    sel = inlier | outlier  # every GT-valid pixel is labeled inlier xor outlier (no AMBIGUOUS class)
    c = np.asarray(conf)[sel].astype(np.float64)
    y = inlier[sel].astype(np.int32)
    if max_pixels and c.size > max_pixels:
        idx = np.linspace(0, c.size - 1, max_pixels).astype(np.int64)
        c, y = c[idx], y[idx]
    if c.size == 0:
        prec = rec = brier = ece_v = float('nan')
    else:
        prec, rec = prec_recall_at(c, y, t)
        brier = float(np.mean((c - y) ** 2))
        ece_v = ece(c, y)
    return dict(roc_auc=roc_auc(c, y), pr_auc=pr_auc(c, y), p_at_01=prec, r_at_01=rec,
                spearman=spearman(c, y.astype(np.float64)), brier=brier, ece=ece_v,
                n_labeled=int(c.size), scores=c, ys=y)


def _colmap_image_camera_id(images_txt, image_name):
    """Look up the camera_id (COLMAP images.txt field 9) for `image_name` (matched as a path
    suffix, since images.txt entries are relative paths like 'images/dslr_images/DSC_0286.JPG')."""
    with open(images_txt) as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 10 and parts[-1].endswith(image_name):
                return int(parts[8])
    return None


def _load_gt_depth_view(image_name, d_shape, args, eth3d_ctx):
    """Load+grid-align the ground-truth depth for one view onto d_shape=(h,w). Returns None if no
    GT is available for this image (skipped by the caller)."""
    if args.gt_format == 'blendedmvs':
        stem = os.path.splitext(image_name)[0]
        pfm_path = os.path.join(args.gt_depth_dir, 'rendered_depth_maps', stem + '.pfm')
        if not os.path.isfile(pfm_path):
            return None
        return GtUtils.resize_depth_nearest(GtUtils.read_pfm(pfm_path), d_shape)

    # eth3d
    cams_d, cams_u, images_txt_d, cache_dir = eth3d_ctx
    h, w = d_shape
    cache_path = os.path.join(cache_dir, '%s_%dx%d.npy' % (image_name.replace(os.sep, '_'), h, w)) if cache_dir else None
    if cache_path and os.path.isfile(cache_path):
        return np.load(cache_path)
    cam_id = _colmap_image_camera_id(images_txt_d, image_name)
    if cam_id is None or cam_id not in cams_d or cam_id not in cams_u:
        return None
    model_d, w_d, h_d, params_d = cams_d[cam_id]
    model_u, w_u, h_u, params_u = cams_u[cam_id]
    depth_path = os.path.join(args.gt_depth_dir, 'ground_truth_depth', 'dslr_images', image_name)
    if not os.path.isfile(depth_path):
        return None
    depth_distorted = GtUtils.read_eth3d_depth(depth_path, w_d, h_d)
    depth_undist = GtUtils.remap_eth3d_depth_to_undistorted(
        depth_distorted, (w_d, h_d, params_d), (w_u, h_u, params_u))
    d_gt = GtUtils.resize_depth_nearest(depth_undist, d_shape)
    if cache_path:
        os.makedirs(cache_dir, exist_ok=True)
        np.save(cache_path, d_gt)
    return d_gt


def run_gt_mode(args):
    """GT-depth label mode: same metric machinery as eval_image's STRICT regime (via
    metrics_from_gt), labels sourced from ground-truth depth instead of fusion .flabel exports."""
    dmaps = sorted(glob.glob(os.path.join(args.dmap_dir, 'depth*.dmap')))
    if not dmaps:
        print('error: no depth*.dmap files in %s' % args.dmap_dir); sys.exit(1)

    eth3d_ctx = None
    if args.gt_format == 'eth3d':
        jpg_dir = os.path.join(args.gt_depth_dir, 'dslr_calibration_jpg')
        cams_d = GtUtils.load_colmap_camera_params(os.path.join(jpg_dir, 'cameras.txt'))
        cams_u = GtUtils.load_colmap_camera_params(
            os.path.join(args.gt_depth_dir, 'dslr_calibration_undistorted', 'cameras.txt'))
        cache_dir = args.gt_cache_dir or os.path.join(
            os.path.dirname(os.path.abspath(args.gt_depth_dir.rstrip('/'))), 'gt_cache')
        eth3d_ctx = (cams_d, cams_u, os.path.join(jpg_dir, 'images.txt'), cache_dir)

    per_view, pool_c, pool_y = [], [], []
    for dm in dmaps:
        # BUGFIX (found during Task 7 real-GT validation, 2026-07-03): the dmap filename's
        # numeric index is view.GetID() (libs/MVS/DepthMap.h ComposeDepthFilePath), which is
        # NOT guaranteed to equal the 0-based position of that image in --scene-mvs's images
        # array -- confirmed on a COLMAP-imported (ETH3D) scene where the two orderings are
        # fully reversed (dmap0001's embedded file_name was the LAST entry of
        # GtUtils.view_image_names, not the first). Using positional lookup silently scored
        # every view's confidence against a DIFFERENT image's ground-truth depth (ROC-AUC
        # collapsed to ~0.45, i.e. worse than chance). Fixed by trusting the dmap's own
        # embedded 'file_name' field instead of any external index/position mapping -- it is
        # written by OpenMVS at estimation time and is unambiguously the image that produced
        # this exact depth/confidence map.
        data = loadDMAP(dm)
        if data is None or not data.get('has_conf'):
            if not args.quiet:
                print('skip %s (no confidence in dmap)' % dm)
            continue
        image_name = os.path.basename(data['file_name'])
        d_est = np.asarray(data['depth_map'], dtype=np.float64)
        conf = np.asarray(data['confidence_map'], dtype=np.float32)
        d_gt = _load_gt_depth_view(image_name, d_est.shape, args, eth3d_ctx)
        if d_gt is None:
            if not args.quiet:
                print('skip %s (no GT depth found)' % image_name)
            continue
        r = metrics_from_gt(d_est, conf, d_gt, args.rel_tol, args.abs_tol, args.threshold, args.max_pixels_per_image)
        per_view.append({k: r[k] for k in ('roc_auc', 'pr_auc', 'p_at_01', 'r_at_01', 'spearman',
                                            'brier', 'ece', 'n_labeled')} | {'image': image_name})
        if r['n_labeled']:
            pool_c.append(r['scores']); pool_y.append(r['ys'])
        if not args.quiet:
            print('%-24s roc=%s pr=%s P@%.2f=%s R@%.2f=%s brier=%s ece=%s n=%d' % (
                image_name, fmt(r['roc_auc']), fmt(r['pr_auc']), args.threshold, fmt(r['p_at_01']),
                args.threshold, fmt(r['r_at_01']), fmt(r['brier']), fmt(r['ece']), r['n_labeled']))

    if pool_c:
        c, y = np.concatenate(pool_c), np.concatenate(pool_y)
        prec, rec = prec_recall_at(c, y, args.threshold)
        pooled = dict(roc_auc=roc_auc(c, y), pr_auc=pr_auc(c, y), p_at_01=prec, r_at_01=rec,
                      spearman=spearman(c, y.astype(np.float64)),
                      brier=float(np.mean((c - y) ** 2)), ece=ece(c, y), n_labeled=int(c.size))
    else:
        pooled = dict(roc_auc=float('nan'), pr_auc=float('nan'), p_at_01=float('nan'), r_at_01=float('nan'),
                      spearman=float('nan'), brier=float('nan'), ece=float('nan'), n_labeled=0)

    print('\n================ GT AGGREGATE (%d/%d views, format=%s, t=%.2f) ================' % (
        len(per_view), len(dmaps), args.gt_format, args.threshold))
    print('ROC-AUC=%s  PR-AUC=%s  P@%.2f=%s  R@%.2f=%s  Spearman=%s  Brier=%s  ECE=%s  n_labeled=%d' % (
        fmt(pooled['roc_auc']), fmt(pooled['pr_auc']), args.threshold, fmt(pooled['p_at_01']),
        args.threshold, fmt(pooled['r_at_01']), fmt(pooled['spearman']), fmt(pooled['brier']),
        fmt(pooled['ece']), pooled['n_labeled']))

    out = dict(scene=os.path.basename(os.path.realpath(args.gt_depth_dir)), mode=args.gt_format,
               per_view=per_view, pooled=pooled)
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=2)
        print('JSON written to %s' % args.json)
    return out


def fmt(x):
    return 'n/a' if x != x else '%.4f' % x  # x!=x detects nan


def main():
    ap = argparse.ArgumentParser(description='Evaluate confidence vs fusion inlier/outlier labels')
    ap.add_argument('dmap_dir', help='directory containing depth*.dmap files (their confMap is graded)')
    ap.add_argument('--labels-dir', default=None, help='directory with .flabel/.fsupport (default: dmap_dir)')
    ap.add_argument('--threshold', type=float, default=0.1, help='fusion confidence gate (default 0.1 = 1-fNCCThresholdKeep)')
    ap.add_argument('--max-pixels-per-image', type=int, default=400000, help='subsample cap per image per regime (0 = all)')
    ap.add_argument('--csv', default=None, help='optional per-image CSV output path')
    ap.add_argument('--quiet', action='store_true', help='suppress per-image lines')
    ap.add_argument('--gt-depth-dir', default=None,
                    help='GT mode: root dir with rendered_depth_maps/ (blendedmvs) or '
                         'ground_truth_depth/+dslr_calibration_*/ (eth3d); switches to GT-depth label mode')
    ap.add_argument('--gt-format', choices=('blendedmvs', 'eth3d'), default=None,
                    help='GT mode: ground-truth depth convention (required with --gt-depth-dir)')
    ap.add_argument('--scene-mvs', default=None,
                    help='GT mode: scene.mvs used to map dmap view-index -> image name (required with --gt-depth-dir)')
    ap.add_argument('--rel-tol', type=float, default=0.01, help='GT mode: relative depth tolerance for inlier/outlier labeling')
    ap.add_argument('--abs-tol', type=float, default=0.0, help='GT mode: absolute depth tolerance floor')
    ap.add_argument('--gt-cache-dir', default=None,
                    help='GT mode (eth3d): override the remapped-GT .npy cache dir '
                         '(default: a gt_cache/ dir alongside --gt-depth-dir)')
    ap.add_argument('--json', default=None, help='GT mode: write {scene,mode,per_view,pooled} JSON here')
    args = ap.parse_args()

    if args.gt_depth_dir:
        if not args.gt_format or not args.scene_mvs:
            print('error: --gt-depth-dir requires --gt-format and --scene-mvs'); sys.exit(1)
        run_gt_mode(args)
        return

    labels_dir = args.labels_dir or args.dmap_dir
    dmaps = sorted(glob.glob(os.path.join(args.dmap_dir, '*.dmap')))
    if not dmaps:
        print('error: no .dmap files in %s' % args.dmap_dir); sys.exit(1)

    agg = {'strict': {'scores': [], 'ys': []}, 'lenient': {'scores': [], 'ys': []}}
    per_image, fnr_all, sp_all, nweak_all = [], [], [], 0
    csv_rows = []
    for dm in dmaps:
        stem = os.path.splitext(os.path.basename(dm))[0]
        flabel = os.path.join(labels_dir, stem + '.flabel')
        fsupport = os.path.join(labels_dir, stem + '.fsupport')
        if not (os.path.exists(flabel) and os.path.exists(fsupport)):
            if not args.quiet:
                print('skip %s (no labels)' % stem)
            continue
        data = loadDMAP(dm)
        if data is None or not data.get('has_conf'):
            if not args.quiet:
                print('skip %s (no confidence in dmap)' % stem)
            continue
        conf = np.asarray(data['confidence_map'], dtype=np.float32)
        label = load_raw_map(flabel)
        support = load_raw_map(fsupport)
        if conf.shape != label.shape:
            print('warn %s: conf %s vs label %s shape mismatch' % (stem, conf.shape, label.shape)); continue
        res = eval_image(conf, label, support, args.threshold, args.max_pixels_per_image)
        if res is None:
            continue
        regimes, extra = res
        for name in ('strict', 'lenient'):
            r = regimes[name]
            if r is not None:
                agg[name]['scores'].append(r['scores']); agg[name]['ys'].append(r['ys'])
        fnr_all.append(extra['fnr']); sp_all.append(extra['spearman']); nweak_all += extra['n_weak']
        per_image.append((stem, regimes, extra))
        s = regimes['strict']; l = regimes['lenient']
        csv_rows.append([stem,
                         fmt(s['roc']) if s else 'n/a', fmt(s['pr']) if s else 'n/a',
                         fmt(l['roc']) if l else 'n/a', fmt(l['pr']) if l else 'n/a',
                         fmt(extra['fnr']), fmt(extra['spearman'])])
        if not args.quiet:
            print('%-16s STRICT[roc=%s pr=%s] LENIENT[roc=%s pr=%s] fnr=%s spearman=%s' % (
                stem,
                fmt(s['roc']) if s else 'n/a', fmt(s['pr']) if s else 'n/a',
                fmt(l['roc']) if l else 'n/a', fmt(l['pr']) if l else 'n/a',
                fmt(extra['fnr']), fmt(extra['spearman'])))

    print('\n================ AGGREGATE (%d images, dmaps=%s, labels=%s, t=%.2f) ================' % (
        len(per_image), args.dmap_dir, labels_dir, args.threshold))
    for name in ('strict', 'lenient'):
        if not agg[name]['scores']:
            print('%-8s: no usable pixels' % name.upper()); continue
        c = np.concatenate(agg[name]['scores']); y = np.concatenate(agg[name]['ys'])
        prec, rec = prec_recall_at(c, y, args.threshold)
        print('%-8s: ROC-AUC=%s  PR-AUC=%s  P@%.2f=%s  R@%.2f=%s  Brier=%s  ECE=%s  (pos=%d neg=%d)' % (
            name.upper(), fmt(roc_auc(c, y)), fmt(pr_auc(c, y)),
            args.threshold, fmt(prec), args.threshold, fmt(rec),
            fmt(float(np.mean((c - y) ** 2))), fmt(ece(c, y)), int(y.sum()), int(y.size - y.sum())))
    fnr_all = np.array([x for x in fnr_all if x == x])
    sp_all = np.array([x for x in sp_all if x == x])
    print('false-neg-recall (WEAK_INLIER kept >= %.2f): mean=%s over %d weak-pixels' % (
        args.threshold, fmt(float(fnr_all.mean()) if fnr_all.size else float('nan')), nweak_all))
    print('Spearman(conf, support): mean=%s' % fmt(float(sp_all.mean()) if sp_all.size else float('nan')))

    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('image,strict_roc,strict_pr,lenient_roc,lenient_pr,fnr,spearman\n')
            for row in csv_rows:
                f.write(','.join(row) + '\n')
        print('per-image CSV written to %s' % args.csv)


if __name__ == '__main__':
    main()
