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
"""
import os
import sys
import glob
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from MvsUtils import loadDMAP

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
    args = ap.parse_args()

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
