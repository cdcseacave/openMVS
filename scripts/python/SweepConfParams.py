#!/usr/bin/env python3
"""
Offline sweep of the AdjustConfidence parameters against fusion inlier/outlier labels.

Reads the per-pixel method features exported by DensifyPointCloud --export-conf-features
(.cfeatK/.cfeatPconf/.cfeatNfsv/.cfeatPrior/.cfeatPhoto) and the geometry-only labels
(.flabel/.fsupport, from --export-fusion-labels). Because the final confidence is a closed
form of those features, any parameter set can be scored instantly without re-running C++:

  gate        = 1 - exp(-(K + kPrior*pGeo)/tau)
  posterior   = (s*pGeo + Pconf)/(s + Pconf + wFSV*Nfsv)
  photoFactor = w0 + (1-w0)*photo
  conf        = clip(posterior*gate*photoFactor, 0, 1); if K>=1: conf = max(conf, floor*photo)

Pools labeled pixels across one or more scene directories, sweeps a parameter grid, and
reports the best sets by STRICT/LENIENT ROC-AUC, PR-AUC and the operating-point metrics at the
fusion gate t=0.1. The 'photo' feature alone is the raw-photometric baseline.

Usage:
  python SweepConfParams.py <feat_dir1> [<feat_dir2> ...] [--per-image-cap 250000] [--gate 0.1]
"""
import os, sys, glob, argparse, itertools
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from EvalConfidence import load_raw_map, roc_auc, pr_auc, prec_recall_at, spearman, ece

INVALID, OUTLIER, AMBIGUOUS, WEAK, CONF = 0, 1, 2, 3, 4


def load_scene(d, cap, max_images=0):
    """Load+subsample labeled pixels from one feature dir. Returns dict of flat arrays."""
    cols = {k: [] for k in ('K', 'Pconf', 'Nfsv', 'pGeo', 'photo', 'label', 'support')}
    fls = sorted(glob.glob(os.path.join(d, '*.flabel')))
    if max_images and len(fls) > max_images:  # evenly sample a subset of images
        fls = [fls[i] for i in np.linspace(0, len(fls) - 1, max_images).astype(np.int64)]
    for n, fl in enumerate(fls):
        stem = os.path.splitext(os.path.basename(fl))[0]
        p = lambda ext: os.path.join(d, stem + ext)
        if not all(os.path.exists(p(e)) for e in ('.cfeatK', '.cfeatPconf', '.cfeatNfsv', '.cfeatPrior', '.cfeatPhoto', '.fsupport')):
            continue
        label = load_raw_map(fl).ravel()
        sel = (label == OUTLIER) | (label == WEAK) | (label == CONF)
        if not np.any(sel):
            continue
        idx = np.flatnonzero(sel)
        if cap and idx.size > cap:
            idx = idx[np.linspace(0, idx.size - 1, cap).astype(np.int64)]
        cols['K'].append(load_raw_map(p('.cfeatK')).ravel()[idx].astype(np.float64))
        cols['Pconf'].append(load_raw_map(p('.cfeatPconf')).ravel()[idx].astype(np.float64))
        cols['Nfsv'].append(load_raw_map(p('.cfeatNfsv')).ravel()[idx].astype(np.float64))
        cols['pGeo'].append(load_raw_map(p('.cfeatPrior')).ravel()[idx].astype(np.float64))
        cols['photo'].append(load_raw_map(p('.cfeatPhoto')).ravel()[idx].astype(np.float64))
        cols['label'].append(label[idx].astype(np.int32))
        cols['support'].append(load_raw_map(p('.fsupport')).ravel()[idx].astype(np.float64))
        if (n + 1) % 40 == 0:
            print('  ...loaded %d/%d images of %s' % (n + 1, len(fls), os.path.basename(d.rstrip('/'))), flush=True)
    if not cols['K']:
        return None
    return {k: np.concatenate(v) for k, v in cols.items()}


def conf_from(F, p):
    s, tau, kPrior, w0, floor, wFSV = p
    gate = 1.0 - np.exp(-(F['K'] + kPrior * F['pGeo']) / tau)
    posterior = (s * F['pGeo'] + F['Pconf']) / (s + F['Pconf'] + wFSV * F['Nfsv'])
    conf = np.clip(posterior * gate * (w0 + (1.0 - w0) * F['photo']), 0.0, 1.0)
    return np.where(F['K'] >= 1, np.maximum(conf, floor * F['photo']), conf)


def metrics(conf, F, t):
    label = F['label']
    out = {}
    for name, pos in (('S', label == CONF), ('L', (label == CONF) | (label == WEAK))):
        sel = pos | (label == OUTLIER)
        c, y = conf[sel], pos[sel].astype(np.int32)
        prec, rec = prec_recall_at(c, y, t)
        out[name] = dict(roc=roc_auc(c, y), pr=pr_auc(c, y), prec=prec, rec=rec)
    weak = label == WEAK
    out['weakR'] = float(np.mean(conf[weak] >= t)) if np.any(weak) else float('nan')
    inl = (label == CONF) | (label == WEAK)
    out['sp'] = spearman(conf[inl], F['support'][inl]) if int(inl.sum()) >= 2 else float('nan')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('feat_dirs', nargs='+')
    ap.add_argument('--per-image-cap', type=int, default=8000, help='max labeled pixels sampled per image')
    ap.add_argument('--max-images', type=int, default=0, help='evenly sample at most this many images per scene (0=all)')
    ap.add_argument('--sweep-sample', type=int, default=1200000, help='pooled pixels used to rank the grid (winner re-scored on full pool)')
    ap.add_argument('--gate', type=float, default=0.1)
    ap.add_argument('--top', type=int, default=12)
    args = ap.parse_args()

    scenes = {}
    for d in args.feat_dirs:
        F = load_scene(d, args.per_image_cap, args.max_images)
        if F is None:
            print('skip %s (no paired features/labels)' % d); continue
        scenes[os.path.basename(d.rstrip('/'))] = F
        print('loaded %-20s %d labeled px (conf=%d weak=%d out=%d)' % (
            os.path.basename(d.rstrip('/')), F['K'].size,
            int((F['label'] == CONF).sum()), int((F['label'] == WEAK).sum()), int((F['label'] == OUTLIER).sum())))
    if not scenes:
        print('no scenes'); sys.exit(1)
    pool = {k: np.concatenate([F[k] for F in scenes.values()]) for k in scenes[next(iter(scenes))]}
    print('POOLED %d labeled pixels across %d scenes\n' % (pool['label'].size, len(scenes)))
    # subsample for fast grid ranking; winner/baseline/default re-scored on the full pool
    if args.sweep_sample and pool['label'].size > args.sweep_sample:
        idx = np.linspace(0, pool['label'].size - 1, args.sweep_sample).astype(np.int64)
        sweep = {k: v[idx] for k, v in pool.items()}
    else:
        sweep = pool

    # baseline: raw photometric confidence is the 'photo' feature itself
    base = metrics(pool['photo'], pool, args.gate)
    DEFAULT = (2.0, 1.0, 0.3, 0.3, 0.5, 0.0)
    dflt = metrics(conf_from(pool, DEFAULT), pool, args.gate)

    grid = dict(s=[1, 2, 4, 8], tau=[0.5, 1.0, 1.5, 2.0], kPrior=[0.0, 0.3, 0.6],
                w0=[0.1, 0.3, 0.5], floor=[0.3, 0.5, 0.7], wFSV=[0.0, 0.5])
    combos = list(itertools.product(*[grid[k] for k in ('s', 'tau', 'kPrior', 'w0', 'floor', 'wFSV')]))
    print('sweeping %d combos on %d-pixel sample...' % (len(combos), sweep['label'].size))
    results = []
    for p in combos:
        m = metrics(conf_from(sweep, p), sweep, args.gate)
        # objective: overall ranking (LENIENT PR-AUC + STRICT ROC-AUC), require few-view recovery
        obj = 0.5 * m['L']['pr'] + 0.5 * m['S']['roc']
        if m['weakR'] < 0.85 or m['L']['rec'] < 0.95:
            obj -= 0.5  # penalize losing true inliers at the fusion gate
        results.append((obj, p, m))
    results.sort(key=lambda r: -r[0])

    def row(tag, m, p=None):
        ps = ('s=%g tau=%g kP=%g w0=%g fl=%g fsv=%g' % p) if p else ''
        return ('%-10s Sroc=%.3f Spr=%.3f | Lroc=%.3f Lpr=%.3f | P@g=%.3f R@g=%.3f weakR=%.3f sp=%.3f  %s' % (
            tag, m['S']['roc'], m['S']['pr'], m['L']['roc'], m['L']['pr'],
            m['L']['prec'], m['L']['rec'], m['weakR'], m['sp'], ps))

    print('\n===== BASELINE / DEFAULT (pooled, gate=%.2f) =====' % args.gate)
    print(row('baseline', base)); print(row('default', dflt, DEFAULT))
    print('\n===== TOP %d PARAM SETS (by 0.5*Lpr + 0.5*Sroc, weakR>=0.85) =====' % args.top)
    for obj, p, m in results[:args.top]:
        print(row('obj=%.3f' % obj, m, p))

    best = results[0]
    print('\n===== WINNER (re-scored on FULL pool) =====')
    print('winner params: s=%g tau=%g kPrior=%g w0=%g floor=%g wFSV=%g' % best[1])
    print(row('winner', metrics(conf_from(pool, best[1]), pool, args.gate), best[1]))
    print('\n===== WINNER per-scene breakdown =====')
    for name, F in scenes.items():
        m = metrics(conf_from(F, best[1]), F, args.gate)
        b = metrics(F['photo'], F, args.gate)
        print('  %-16s NEW %s' % (name, row('', m).strip()))
        print('  %-16s RAW %s' % ('', row('', b).strip()))


if __name__ == '__main__':
    main()
