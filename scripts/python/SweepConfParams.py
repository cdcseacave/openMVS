#!/usr/bin/env python3
"""
Offline sweep of the AdjustConfidence parameters against fusion inlier/outlier labels.

Reads the per-pixel method features exported by DensifyPointCloud --export-conf-features
(.cfeatK/.cfeatPconf/.cfeatPrior/.cfeatPhoto) and the geometry-only labels
(.flabel/.fsupport, from --export-fusion-labels). Because the final confidence is a closed
form of those features, any parameter set can be scored instantly without re-running C++:

  gate        = 1 - exp(-(K + kPrior*pGeo)/tau)
  posterior   = (s*pGeo + Pconf)/(s + Pconf)
  photoFactor = w0 + (1-w0)*photo
  conf        = clip(posterior*gate*photoFactor, 0, 1); if K>=1: conf = max(conf, floor*photo)

Pools labeled pixels across one or more scene directories, sweeps a parameter grid, and
reports the best sets by STRICT/LENIENT ROC-AUC, PR-AUC and the operating-point metrics at the
fusion gate t=0.1. The 'photo' feature alone is the raw-photometric baseline.

Usage:
  python SweepConfParams.py <feat_dir1> [<feat_dir2> ...] [--per-image-cap 250000] [--gate 0.1]
"""
import os, sys, glob, json, argparse, itertools, types, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from EvalConfidence import load_raw_map, roc_auc, pr_auc, prec_recall_at, spearman, ece
import EvalConfidence
from MvsUtils import loadDMAP
import GtUtils

INVALID, OUTLIER, AMBIGUOUS, WEAK, CONF = 0, 1, 2, 3, 4


def load_scene(d, cap, max_images=0):
    """Load+subsample labeled pixels from one feature dir. Returns dict of flat arrays."""
    cols = {k: [] for k in ('K', 'Pconf', 'pGeo', 'photo', 'label', 'support')}
    fls = sorted(glob.glob(os.path.join(d, '*.flabel')))
    if max_images and len(fls) > max_images:  # evenly sample a subset of images
        fls = [fls[i] for i in np.linspace(0, len(fls) - 1, max_images).astype(np.int64)]
    for n, fl in enumerate(fls):
        stem = os.path.splitext(os.path.basename(fl))[0]
        p = lambda ext: os.path.join(d, stem + ext)
        if not all(os.path.exists(p(e)) for e in ('.cfeatK', '.cfeatPconf', '.cfeatPrior', '.cfeatPhoto', '.fsupport')):
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
    s, tau, kPrior, w0, floor = p
    gate = 1.0 - np.exp(-(F['K'] + kPrior * F['pGeo']) / tau)
    posterior = (s * F['pGeo'] + F['Pconf']) / (s + F['Pconf'])
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


# ============================================================================
# Task 17: GT-driven recalibration sweep (extends the Task-9-era .flabel/.fsupport
# sweep above with a GT-depth-label mode). Reuses:
#   - GtUtils.gt_labels (Task 4) for the per-pixel inlier/outlier decision
#   - EvalConfidence._load_gt_depth_view / _colmap_image_camera_id (Task 5) for
#     loading+grid-aligning per-view ground-truth depth
#   - the cfeatK/cfeatV/cfeatPconf/cfeatPrior/cfeatPhoto sidecars exported by
#     --export-conf-features (Task 9/15/16), read via load_raw_map (same 'LMAP'
#     raw-map format as .flabel/.fsupport)
#
# Directory contract (produced by gt_bench/task17/export_all.sh): under a
# --gt-task17-root, one subdirectory per (scene, resolution level), e.g.
# "eth3d_courtyard_L2" or "bmvs_59d2657f_L0", each containing:
#   raw/                  pristine copy of raw_dmaps/*.dmap (untouched: export-conf-
#                         features returns false before the dmap re-save)
#   hard_m{2,3,5}/        cfeatK(raw int)/cfeatV/cfeatPconf/cfeatPrior/cfeatPhoto,
#                         bConfSoftGates=0, Conf Violation Margin={2,3,5}
#   soft_m{2,3,5}/        same, bConfSoftGates=1 (cfeatK = ROUND2INT(Ksoft*1000))
#
# Because margin only changes how cfeatV is CLASSIFIED (SceneDensify.cpp
# ~1668-1679/~1725-1728) -- K/Ksoft, Pconf, pGeo, photo are margin-independent for
# a given mode -- K/Pconf/Prior/Photo are read once per mode from the margin=3
# sidecar set (verified byte-identical across margins as a sanity check, see
# verify_margin_invariance below) and only cfeatV is reloaded per margin.
#
# OFFLINE POSTERIOR (exact copy of the C++ formula, SceneDensify.cpp:1801-1808 --
# keep in sync if that block changes):
#   gate      = 1 - exp(-(Kf + kPrior*pGeo)/tau)
#   posterior = (s*pGeo + Pconf) / (s + Pconf + lambda*V)
#   photoF    = w0 + (1-w0)*photo
#   conf      = clip(posterior*gate*photoF, 0, 1)
#   if Kf>=1: conf = max(conf, floor*photo)
# kPrior (fConfPriorGate) is held FIXED at the current shipped default (0.3) --
# it is a "known knob" per the brief but NOT one of the 7 DEFVARs the brief lists
# as sweep/adopt targets (fConfPriorStrength/ConfirmTau/PhotoFloor/Floor/
# ViolationWeight/ViolationMargin/bConfSoftGates).
# ============================================================================

KPRIOR_FIXED = 0.3  # fConfPriorGate current default; not swept (see note above)
GT_MODES = ('hard', 'soft')
GT_MARGINS = (2, 3, 5)
# shipped defaults BEFORE this task's recalibration (mode=hard i.e. bConfSoftGates=0;
# margin irrelevant at lam=0)
GT_DEFAULT = dict(s=1.0, tau=2.0, w0=0.5, floor=0.5, lam=0.0, margin=3, mode='hard')
GT_GRID = dict(
    s=[0.5, 1.0, 2.0],               # fConfPriorStrength
    tau=[1.5, 2.0, 3.0],             # fConfConfirmTau
    w0=[0.3, 0.5, 0.7],              # fConfPhotoFloor
    floor=[0.03, 0.05, 0.1],         # fConfFloor
    lam=[0.0, 0.25, 0.5, 1.0, 2.0],  # fConfViolationWeight
)


def resolve_gt_ctx(scene, gt_root, repo_root):
    """Mirror gt_bench/run_scene.sh's scene-name -> GT-source resolution."""
    if scene.startswith('bmvs_'):
        short = scene[len('bmvs_'):]
        full = None
        with open(os.path.join(repo_root, 'gt_bench', 'scenes_blendedmvs.txt')) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                sid = line.split()[0]
                if sid.startswith(short):
                    full = sid
                    break
        if not full:
            raise ValueError('cannot resolve bmvs full id for %s' % scene)
        return dict(gt_format='blendedmvs',
                     gt_depth_dir=os.path.join(gt_root, 'blendedmvs', full),
                     scene_mvs=os.path.join(gt_root, 'runs', scene, 'scene.mvs'),
                     gt_cache_dir=None)
    if scene.startswith('eth3d_'):
        name = scene[len('eth3d_'):]
        return dict(gt_format='eth3d',
                     gt_depth_dir=os.path.join(gt_root, 'eth3d', name),
                     scene_mvs=os.path.join(gt_root, 'runs', scene, 'scene.mvs'),
                     gt_cache_dir=os.path.join(gt_root, 'runs', scene, 'gt_cache'))
    raise ValueError('unrecognized scene name %r (expected bmvs_*/eth3d_*)' % scene)


def load_scene_gt(scene_dir_path, scene_name, gt_root, repo_root, cap, rel_tol=0.01,
                   abs_tol=0.0, max_images=0):
    """Load one <scene>_L<L> task17 directory into per-mode pooled feature arrays,
    labeled against ground-truth depth (GtUtils.gt_labels), for every image that has
    both cfeat sidecars AND resolvable GT depth. Returns (dict[mode] -> features-or-
    None, n_views_used)."""
    ctx = resolve_gt_ctx(scene_name, gt_root, repo_root)
    raw_dir = os.path.join(scene_dir_path, 'raw')
    dmaps = sorted(glob.glob(os.path.join(raw_dir, 'depth*.dmap')))
    if max_images and len(dmaps) > max_images:
        dmaps = [dmaps[i] for i in np.linspace(0, len(dmaps) - 1, max_images).astype(np.int64)]

    eth3d_ctx = None
    if ctx['gt_format'] == 'eth3d':
        jpg_dir = os.path.join(ctx['gt_depth_dir'], 'dslr_calibration_jpg')
        cams_d = GtUtils.load_colmap_camera_params(os.path.join(jpg_dir, 'cameras.txt'))
        cams_u = GtUtils.load_colmap_camera_params(
            os.path.join(ctx['gt_depth_dir'], 'dslr_calibration_undistorted', 'cameras.txt'))
        eth3d_ctx = (cams_d, cams_u, os.path.join(jpg_dir, 'images.txt'), ctx['gt_cache_dir'])
    args_ns = types.SimpleNamespace(gt_format=ctx['gt_format'], gt_depth_dir=ctx['gt_depth_dir'])

    acc = {m: {'K': [], 'Pconf': [], 'pGeo': [], 'photo': [], 'label': [],
               'V': {mg: [] for mg in GT_MARGINS}} for m in GT_MODES}
    n_views_used = 0
    for dm in dmaps:
        data = loadDMAP(dm)
        if data is None or not data.get('has_conf'):
            continue
        stem = os.path.splitext(os.path.basename(dm))[0]
        image_name = os.path.basename(data['file_name'])
        d_est = np.asarray(data['depth_map'], dtype=np.float64)
        d_gt = EvalConfidence._load_gt_depth_view(image_name, d_est.shape, args_ns, eth3d_ctx)
        if d_gt is None:
            continue
        inlier, outlier, _ = GtUtils.gt_labels(d_est, d_gt, rel_tol, abs_tol)
        sel = (inlier | outlier).ravel()
        if not np.any(sel):
            continue
        idx = np.flatnonzero(sel)
        if cap and idx.size > cap:
            idx = idx[np.linspace(0, idx.size - 1, cap).astype(np.int64)]
        label = inlier.ravel()[idx].astype(np.int32)
        base_dirs = {m: os.path.join(scene_dir_path, '%s_m3' % m) for m in GT_MODES}
        if not all(os.path.exists(os.path.join(base_dirs[m], stem + '.cfeatK')) for m in GT_MODES):
            continue
        n_views_used += 1
        for mode in GT_MODES:
            base_dir = base_dirs[mode]
            Kraw = load_raw_map(os.path.join(base_dir, stem + '.cfeatK')).ravel()[idx].astype(np.float64)
            if mode == 'soft':  # Task 16: soft cfeatK is fixed-point Ksoft*1000
                Kraw = Kraw / 1000.0
            acc[mode]['K'].append(Kraw)
            acc[mode]['Pconf'].append(load_raw_map(os.path.join(base_dir, stem + '.cfeatPconf')).ravel()[idx].astype(np.float64))
            acc[mode]['pGeo'].append(load_raw_map(os.path.join(base_dir, stem + '.cfeatPrior')).ravel()[idx].astype(np.float64))
            acc[mode]['photo'].append(load_raw_map(os.path.join(base_dir, stem + '.cfeatPhoto')).ravel()[idx].astype(np.float64))
            acc[mode]['label'].append(label)
            for mg in GT_MARGINS:
                vpath = os.path.join(scene_dir_path, '%s_m%d' % (mode, mg), stem + '.cfeatV')
                V = (load_raw_map(vpath).ravel()[idx].astype(np.float64) if os.path.exists(vpath)
                     else np.zeros(idx.size))
                acc[mode]['V'][mg].append(V)

    out = {}
    for mode in GT_MODES:
        if not acc[mode]['K']:
            out[mode] = None
            continue
        d = {k: np.concatenate(v) for k, v in acc[mode].items() if k != 'V'}
        d['V'] = {mg: np.concatenate(acc[mode]['V'][mg]) for mg in GT_MARGINS}
        out[mode] = d
    return out, n_views_used


def verify_margin_invariance(scenes, scene_name):
    """Sanity check backing the 'K/Pconf/Prior/Photo are margin-independent' claim
    the loader relies on (only cfeatV differs across margin=2/3/5 exports): re-load
    one scene's hard_m2/hard_m5 cfeatK/Pconf/Prior/Photo directly and diff against
    hard_m3 (already loaded). Returns max abs diff found (0.0 if truly identical)."""
    maxdiff = 0.0
    scene_dir = scenes[scene_name]['_dir']
    F3 = scenes[scene_name]['hard']
    if F3 is None:
        return None
    for mg in (2, 5):
        d = os.path.join(scene_dir, 'hard_m%d' % mg)
        if not os.path.isdir(d):
            continue
        for ext in ('.cfeatK', '.cfeatPconf', '.cfeatPrior', '.cfeatPhoto'):
            for f in sorted(glob.glob(os.path.join(d, '*' + ext))):
                stem = os.path.basename(f)[:-len(ext)]
                ref = os.path.join(scene_dir, 'hard_m3', stem + ext)
                if not os.path.exists(ref):
                    continue
                a = load_raw_map(f).astype(np.float64)
                b = load_raw_map(ref).astype(np.float64)
                if a.shape == b.shape:
                    maxdiff = max(maxdiff, float(np.max(np.abs(a - b))))
    return maxdiff


def conf_from_gt(F, s, tau, w0, floor, lam, margin):
    V = F['V'][margin]
    gate = 1.0 - np.exp(-(F['K'] + KPRIOR_FIXED * F['pGeo']) / max(tau, 0.01))
    posterior = (s * F['pGeo'] + F['Pconf']) / (s + F['Pconf'] + lam * V)
    conf = np.clip(posterior * gate * (w0 + (1.0 - w0) * F['photo']), 0.0, 1.0)
    return np.where(F['K'] >= 1.0, np.maximum(conf, floor * F['photo']), conf)


# Fully-vectorized ROC-AUC + average-precision for the hot grid loop. EvalConfidence's
# roc_auc/pr_auc are byte-for-byte correct but tie-average / accumulate AP in a pure-Python
# per-element loop (O(n) Python) -- fine for a handful of eval calls, but 2430 combos x pooled
# 2M px = billions of Python iterations. These reproduce the SAME numbers with numpy ops only
# (verified equal to the reference within 1e-12 in verify_metric_equivalence below).
def roc_auc_fast(scores, labels):
    n = labels.size
    npos = int(labels.sum()); nneg = n - npos
    if npos == 0 or nneg == 0:
        return float('nan')
    order = np.argsort(scores, kind='mergesort')
    s = scores[order]
    y = labels[order]
    # average ranks within equal-value tie groups (vectorized)
    change = np.empty(n, dtype=bool); change[0] = True; change[1:] = s[1:] != s[:-1]
    grp = np.cumsum(change) - 1
    pos = np.arange(1, n + 1, dtype=np.float64)  # 1-based rank of each sorted element
    counts = np.bincount(grp)
    sums = np.bincount(grp, weights=pos)
    ranks = (sums / counts)[grp]
    rank_pos = ranks[y == 1].sum()
    return float((rank_pos - npos * (npos + 1) / 2.0) / (npos * nneg))


def pr_auc_fast(scores, labels):
    npos = int(labels.sum())
    if npos == 0:
        return float('nan')
    order = np.argsort(-scores, kind='mergesort')
    y = labels[order].astype(np.float64)
    tp = np.cumsum(y); fp = np.cumsum(1.0 - y)
    precision = tp / np.maximum(tp + fp, 1e-12)
    recall = tp / npos
    dr = np.empty(recall.size); dr[0] = recall[0]; dr[1:] = recall[1:] - recall[:-1]
    return float(np.sum(dr * precision))  # dr!=0 only at a true positive == the reference's step sum


def metrics_gt(conf, label, t=0.1, with_pr=False):
    prec, rec = prec_recall_at(conf, label, t)
    out = dict(roc=roc_auc_fast(conf, label), prec=prec, rec=rec)
    out['pr'] = pr_auc_fast(conf, label) if with_pr else float('nan')
    return out


def pool_mode(scenes, mode, names=None):
    parts = [scenes[n][mode] for n in (names if names is not None else scenes.keys())
             if scenes.get(n, {}).get(mode) is not None]
    if not parts:
        return None
    out = {k: np.concatenate([p[k] for p in parts]) for k in ('K', 'Pconf', 'pGeo', 'photo', 'label')}
    out['V'] = {mg: np.concatenate([p['V'][mg] for p in parts]) for mg in GT_MARGINS}
    return out


def subsample_F(F, k):
    """Evenly-spaced deterministic subsample of a feature dict to at most k rows (keeps
    the pooled/per-scene grid ranking cheap -- an argsort over the FULL pool 2430x is the
    dominant cost). Winner + baseline are always re-scored on the FULL arrays afterward, so
    this only affects RANKING, never the reported/adopted numbers. k=0 -> no subsampling."""
    if F is None:
        return None
    n = F['K'].size
    if not k or n <= k:
        return F
    idx = np.linspace(0, n - 1, k).astype(np.int64)
    out = {key: F[key][idx] for key in ('K', 'Pconf', 'pGeo', 'photo', 'label')}
    out['V'] = {mg: F['V'][mg][idx] for mg in GT_MARGINS}
    return out


def combo_row(tag, s, tau, w0, floor, lam, margin, mode, m):
    return ('%-9s mode=%-4s margin=%d s=%-4g tau=%-4g w0=%-4g floor=%-5g lam=%-4g | '
            'roc=%.4f pr=%.4f P@.1=%.4f R@.1=%.4f' % (
                tag, mode, margin, s, tau, w0, floor, lam, m['roc'], m['pr'], m['prec'], m['rec']))


def run_gt_sweep(scenes, gate=0.1, roc_guard=0.005, p01_eps=0.002, top=15,
                  k_scene=50000, k_pool=300000):
    """Full-grid (not coordinate-descent -- 2*3*3*3*3*5=2430 combos was cheap enough
    to brute-force, logged below) sweep of s/tau/w0/floor/lam x margin x hard-vs-soft,
    scored on the pooled GT labels, subject to the brief's selection rule:
      (a) no scene-level regresses ROC-AUC below its current-default ROC - roc_guard
      (b) pooled P@0.1 not worse than current-default pooled P@0.1 (- p01_eps slack)
    picks the max pooled ROC-AUC survivor."""
    n_combos = (len(GT_GRID['s']) * len(GT_GRID['tau']) * len(GT_GRID['w0']) * len(GT_GRID['floor'])
                * len(GT_GRID['lam']))
    print('search method: FULL GRID (not coordinate descent) -- %d (s x tau x w0 x floor x lam) '
          'x %d margins x %d modes = %d total evals; cheap enough to brute-force in full (no '
          'local-optimum risk from a greedy coordinate-descent path).' % (
              n_combos, len(GT_MARGINS), len(GT_MODES), n_combos * len(GT_MARGINS) * len(GT_MODES)))

    scene_names = list(scenes.keys())
    # FULL per-scene (hard) baseline -- reported/adopted numbers use full arrays
    baseline_scene = {}
    for name in scene_names:
        F = scenes[name].get('hard')
        if F is None:
            continue
        c = conf_from_gt(F, GT_DEFAULT['s'], GT_DEFAULT['tau'], GT_DEFAULT['w0'], GT_DEFAULT['floor'],
                          GT_DEFAULT['lam'], GT_DEFAULT['margin'])
        baseline_scene[name] = metrics_gt(c, F['label'], gate)

    pools = {mode: pool_mode(scenes, mode, scene_names) for mode in GT_MODES}
    baseline_pool = pool_mode(scenes, 'hard', scene_names)
    c_b = conf_from_gt(baseline_pool, GT_DEFAULT['s'], GT_DEFAULT['tau'], GT_DEFAULT['w0'],
                        GT_DEFAULT['floor'], GT_DEFAULT['lam'], GT_DEFAULT['margin'])
    baseline_pooled = metrics_gt(c_b, baseline_pool['label'], gate, with_pr=True)
    print('\n===== CURRENT DEFAULT (pooled over %d scene-levels, FULL %d px) =====' % (
        len(scene_names), baseline_pool['label'].size))
    print(combo_row('default', GT_DEFAULT['s'], GT_DEFAULT['tau'], GT_DEFAULT['w0'], GT_DEFAULT['floor'],
                     GT_DEFAULT['lam'], GT_DEFAULT['margin'], GT_DEFAULT['mode'], baseline_pooled))

    # SUBSAMPLED grid inputs (ranking only): per-scene (both modes) + pooled per mode. The guard
    # compares combo per-scene ROC vs a SUBSAMPLED-hard baseline (apples-to-apples on the same
    # rows); the winner is re-scored on FULL arrays below.
    print('grid ranking on subsampled arrays: k_scene=%d px/scene, k_pool=%d px/pool '
          '(winner+baseline re-scored on FULL arrays)' % (k_scene, k_pool))
    scenes_grid = {name: {m: subsample_F(scenes[name].get(m), k_scene) for m in GT_MODES}
                   for name in scene_names}
    pools_grid = {mode: subsample_F(pools[mode], k_pool) for mode in GT_MODES}
    baseline_scene_grid = {}
    for name in scene_names:
        F = scenes_grid[name].get('hard')
        if F is None:
            continue
        c = conf_from_gt(F, GT_DEFAULT['s'], GT_DEFAULT['tau'], GT_DEFAULT['w0'], GT_DEFAULT['floor'],
                          GT_DEFAULT['lam'], GT_DEFAULT['margin'])
        baseline_scene_grid[name] = metrics_gt(c, F['label'], gate)

    t0 = time.time()
    results = []
    combos = list(itertools.product(GT_MODES, GT_MARGINS, GT_GRID['s'], GT_GRID['tau'], GT_GRID['w0'],
                                     GT_GRID['floor'], GT_GRID['lam']))
    for i, (mode, margin, s, tau, w0, floor, lam) in enumerate(combos):
        pool = pools_grid[mode]
        if pool is None:
            continue
        conf = conf_from_gt(pool, s, tau, w0, floor, lam, margin)
        pooled_m = metrics_gt(conf, pool['label'], gate)
        min_delta = float('inf')
        per_scene = {}
        for name in scene_names:
            F = scenes_grid[name].get(mode)
            base = baseline_scene_grid.get(name)
            if F is None or base is None:
                continue
            c = conf_from_gt(F, s, tau, w0, floor, lam, margin)
            m = metrics_gt(c, F['label'], gate)
            per_scene[name] = m
            min_delta = min(min_delta, m['roc'] - base['roc'])
        results.append(dict(mode=mode, margin=margin, s=s, tau=tau, w0=w0, floor=floor, lam=lam,
                             pooled=pooled_m, min_scene_delta=min_delta, per_scene=per_scene))
        if (i + 1) % 500 == 0:
            print('  ...swept %d/%d combos (%.1fs)' % (i + 1, len(combos), time.time() - t0), flush=True)
    print('sweep done: %d combos scored in %.1fs' % (len(results), time.time() - t0))

    survivors = [r for r in results
                 if r['min_scene_delta'] >= -roc_guard
                 and r['pooled']['prec'] >= baseline_pooled['prec'] - p01_eps]
    print('\nguard: min per-scene-level dROC >= -%.3f AND pooled P@%.2f >= %.4f - %.3f -> '
          '%d/%d combos survive' % (roc_guard, gate, baseline_pooled['prec'], p01_eps, len(survivors), len(results)))

    results_sorted = sorted(results, key=lambda r: -r['pooled']['roc'])
    survivor_ids = {id(r) for r in survivors}
    print('\n===== TOP %d BY POOLED ROC (guard-violators marked *) =====' % top)
    for r in results_sorted[:top]:
        marker = '' if id(r) in survivor_ids else '  *guard-fail*'
        print(combo_row('combo', r['s'], r['tau'], r['w0'], r['floor'], r['lam'], r['margin'], r['mode'],
                         r['pooled']) + '  minDscene=%+.4f%s' % (r['min_scene_delta'], marker))

    if not survivors:
        print('\nNO combo satisfies the guard -- KEEPING current defaults.')
        return None, baseline_pooled, baseline_scene, results

    # Re-score subsample-survivors on FULL arrays (pooled + per-scene) and re-apply the guard on
    # FULL data, walking survivors by descending subsample ROC; adopt the first that STILL passes.
    # This guarantees the adopted winner satisfies the guard on the full pool, not just the ranking
    # subsample. (Subsample ROC has ~0.001-0.003 SE, so a borderline subsample-survivor could fail
    # the full-data guard; this closes that gap.)
    def full_rescore(r):
        pool = pools[r['mode']]
        conf = conf_from_gt(pool, r['s'], r['tau'], r['w0'], r['floor'], r['lam'], r['margin'])
        pooled = metrics_gt(conf, pool['label'], gate, with_pr=True)
        per_scene = {}
        min_delta = float('inf')
        for name in scene_names:
            F = scenes[name].get(r['mode'])
            base = baseline_scene.get(name)
            if F is None or base is None:
                continue
            c = conf_from_gt(F, r['s'], r['tau'], r['w0'], r['floor'], r['lam'], r['margin'])
            m = metrics_gt(c, F['label'], gate)
            per_scene[name] = m
            min_delta = min(min_delta, m['roc'] - base['roc'])
        return pooled, per_scene, min_delta

    survivors_sorted = sorted(survivors, key=lambda r: -r['pooled']['roc'])
    winner = None
    for cand in survivors_sorted:
        pooled_full, per_scene_full, min_delta_full = full_rescore(cand)
        passes = (min_delta_full >= -roc_guard and pooled_full['prec'] >= baseline_pooled['prec'] - p01_eps)
        if passes:
            cand['pooled'] = pooled_full
            cand['per_scene'] = per_scene_full
            cand['min_scene_delta'] = min_delta_full
            winner = cand
            break
        else:
            print('  (subsample-survivor s=%g tau=%g w0=%g floor=%g lam=%g margin=%d mode=%s FAILS '
                  'full-data guard: minDscene=%+.4f pooledP@.1=%.4f -- skipping)' % (
                      cand['s'], cand['tau'], cand['w0'], cand['floor'], cand['lam'], cand['margin'],
                      cand['mode'], min_delta_full, pooled_full['prec']))
    if winner is None:
        print('\nNO subsample-survivor passes the FULL-data guard -- KEEPING current defaults.')
        return None, baseline_pooled, baseline_scene, results
    print('\n===== WINNER (max pooled ROC among guard survivors, re-scored on FULL data) =====')
    print(combo_row('winner', winner['s'], winner['tau'], winner['w0'], winner['floor'], winner['lam'],
                     winner['margin'], winner['mode'], winner['pooled']))
    print('pooled dROC = %+.4f vs current default (%.4f -> %.4f)' % (
        winner['pooled']['roc'] - baseline_pooled['roc'], baseline_pooled['roc'], winner['pooled']['roc']))
    print('\n===== WINNER per-scene-level breakdown (NEW vs DEFAULT) =====')
    for name in scene_names:
        base = baseline_scene.get(name)
        m = winner['per_scene'].get(name)
        if base is None or m is None:
            continue
        print('  %-24s DEFAULT roc=%.4f P@.1=%.4f  ->  NEW roc=%.4f P@.1=%.4f  dROC=%+.4f' % (
            name, base['roc'], base['prec'], m['roc'], m['prec'], m['roc'] - base['roc']))
    return winner, baseline_pooled, baseline_scene, results


def resolution_check(scenes, winner, gate=0.1, k_pool=300000):
    """Task-17 brief: check whether the global winner differs materially between
    resolution regimes. We have two: eth3d (all at L2) and bmvs (L0 native + L1).
    For each regime, independently find its OWN best combo (max pooled ROC, same
    grid, no cross-scene guard -- diagnostic only) and compare its ROC to the
    GLOBAL winner's ROC evaluated on that same regime's pool. >0.01 gap => flag."""
    def regime_names(pred):
        return [n for n in scenes.keys() if pred(n)]

    regimes = {
        'eth3d_L2': regime_names(lambda n: n.startswith('eth3d_')),
        'bmvs_L0': regime_names(lambda n: n.startswith('bmvs_') and n.endswith('_L0')),
        'bmvs_L1': regime_names(lambda n: n.startswith('bmvs_') and n.endswith('_L1')),
    }
    print('\n===== PER-RESOLUTION CHECK =====')
    combos = list(itertools.product(GT_MODES, GT_MARGINS, GT_GRID['s'], GT_GRID['tau'], GT_GRID['w0'],
                                     GT_GRID['floor'], GT_GRID['lam']))
    any_flag = False
    for rname, names in regimes.items():
        if not names:
            continue
        # subsample each regime pool for the grid scan (diagnostic ROC only; same rationale as
        # run_gt_sweep -- keeps the argsort-per-combo cost bounded)
        pool_by_mode = {mode: subsample_F(pool_mode(scenes, mode, names), k_pool) for mode in GT_MODES}
        best = None
        for mode, margin, s, tau, w0, floor, lam in combos:
            pool = pool_by_mode[mode]
            if pool is None:
                continue
            c = conf_from_gt(pool, s, tau, w0, floor, lam, margin)
            m = metrics_gt(c, pool['label'], gate)
            if best is None or m['roc'] > best[0]['roc']:
                best = (m, dict(mode=mode, margin=margin, s=s, tau=tau, w0=w0, floor=floor, lam=lam))
        if best is None:
            continue
        pool_g = pool_by_mode.get(winner['mode'])
        if pool_g is None:
            continue
        c_g = conf_from_gt(pool_g, winner['s'], winner['tau'], winner['w0'], winner['floor'], winner['lam'], winner['margin'])
        m_g = metrics_gt(c_g, pool_g['label'], gate)
        gap = best[0]['roc'] - m_g['roc']
        flag = gap > 0.01
        any_flag = any_flag or flag
        print('  %-10s n_scenes=%d  own-best roc=%.4f (%s)  global-winner-applied roc=%.4f  gap=%.4f%s' % (
            rname, len(names), best[0]['roc'], best[1], m_g['roc'], gap, '  <-- FLAG (>0.01)' if flag else ''))
    if any_flag:
        print('=> at least one resolution regime diverges >0.01 from the global winner; '
              'per-resolution defaults COULD be justified (see report for the final call).')
    else:
        print('=> no regime diverges >0.01 from the global winner; ONE GLOBAL default is fine.')
    return any_flag


def main_gt(args):
    scenes = {}
    scene_dirs = sorted(d for d in glob.glob(os.path.join(args.gt_task17_root, '*'))
                         if os.path.isdir(d) and os.path.isdir(os.path.join(d, 'raw'))
                         and (os.path.basename(d).startswith('eth3d_') or os.path.basename(d).startswith('bmvs_')))
    if not scene_dirs:
        print('no <scene>_L<L> directories with a raw/ subdir found under %s' % args.gt_task17_root)
        sys.exit(1)
    for d in scene_dirs:
        name = os.path.basename(d)
        t0 = time.time()
        modes_f, n_views = load_scene_gt(d, name.rsplit('_L', 1)[0], args.gt_root, args.repo,
                                          args.per_image_cap, args.rel_tol, args.abs_tol, args.max_images)
        scenes[name] = modes_f
        scenes[name]['_dir'] = d
        nk = {m: (modes_f[m]['K'].size if modes_f[m] is not None else 0) for m in GT_MODES}
        print('loaded %-24s %d views used, hard=%d px soft=%d px (%.1fs)' % (
            name, n_views, nk['hard'], nk['soft'], time.time() - t0))
    if args.verify_margin_invariance:
        print('\n===== VERIFYING margin-invariance of K/Pconf/Prior/Photo (sanity check) =====')
        for name in scenes:
            md = verify_margin_invariance(scenes, name)
            if md is not None:
                print('  %-24s max|diff| hard_m{2,5} vs hard_m3 (K,Pconf,pGeo,photo) = %g%s' % (
                    name, md, '  OK (exact)' if md == 0.0 else '  <-- NONZERO, formula assumption violated!'))

    winner, baseline_pooled, baseline_scene, all_results = run_gt_sweep(
        scenes, gate=args.gate, roc_guard=args.roc_guard, p01_eps=args.p01_eps, top=args.top)

    if winner is not None:
        resolution_check(scenes, winner, gate=args.gate)

    if args.json_out:
        def strip(m):
            return {k: (float(v) if v == v else None) for k, v in m.items()}
        out = dict(
            baseline=dict(params=GT_DEFAULT, pooled=strip(baseline_pooled),
                          per_scene={n: strip(m) for n, m in baseline_scene.items()}),
            winner=(None if winner is None else dict(
                params={k: winner[k] for k in ('mode', 'margin', 's', 'tau', 'w0', 'floor', 'lam')},
                pooled=strip(winner['pooled']),
                per_scene={n: strip(m) for n, m in winner['per_scene'].items()},
                min_scene_delta=winner['min_scene_delta'])),
            top=[dict(params={k: r[k] for k in ('mode', 'margin', 's', 'tau', 'w0', 'floor', 'lam')},
                      pooled=strip(r['pooled']), min_scene_delta=r['min_scene_delta'])
                 for r in sorted(all_results, key=lambda r: -r['pooled']['roc'])[:50]],
        )
        with open(args.json_out, 'w') as f:
            json.dump(out, f, indent=2)
        print('\nJSON written to %s' % args.json_out)


def main_legacy():
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
    DEFAULT = (1.0, 2.0, 0.3, 0.5, 0.5)  # shipped OPTDENSE defaults (fConfPriorStrength/ConfirmTau/PriorGate/PhotoFloor/Floor)
    dflt = metrics(conf_from(pool, DEFAULT), pool, args.gate)

    grid = dict(s=[1, 2, 4, 8], tau=[0.5, 1.0, 1.5, 2.0], kPrior=[0.0, 0.3, 0.6],
                w0=[0.1, 0.3, 0.5], floor=[0.3, 0.5, 0.7])
    combos = list(itertools.product(*[grid[k] for k in ('s', 'tau', 'kPrior', 'w0', 'floor')]))
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
        ps = ('s=%g tau=%g kP=%g w0=%g fl=%g' % p) if p else ''
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
    print('winner params: s=%g tau=%g kPrior=%g w0=%g floor=%g' % best[1])
    print(row('winner', metrics(conf_from(pool, best[1]), pool, args.gate), best[1]))
    print('\n===== WINNER per-scene breakdown =====')
    for name, F in scenes.items():
        m = metrics(conf_from(F, best[1]), F, args.gate)
        b = metrics(F['photo'], F, args.gate)
        print('  %-16s NEW %s' % (name, row('', m).strip()))
        print('  %-16s RAW %s' % ('', row('', b).strip()))


def main():
    """Dispatch: --gt-task17-root switches to the Task-17 GT-driven recalibration
    sweep (main_gt); otherwise the original Task-9-era .flabel/.fsupport sweep
    (main_legacy) runs unchanged."""
    if '--gt-task17-root' in sys.argv:
        ap = argparse.ArgumentParser(description='Task 17: GT-driven confidence-parameter recalibration sweep')
        ap.add_argument('--gt-task17-root', required=True,
                        help='root dir with <scene>_L<L>/{raw,hard_m*,soft_m*}/ (see gt_bench/task17/export_all.sh)')
        ap.add_argument('--gt-root', default='/home/ubuntu/virginia/gt_bench',
                        help='gt_bench data root (eth3d/, blendedmvs/, runs/<scene>/gt_cache)')
        ap.add_argument('--repo', default=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                        help='openMVS repo root (for gt_bench/scenes_blendedmvs.txt)')
        ap.add_argument('--per-image-cap', type=int, default=6000, help='max labeled pixels sampled per view')
        ap.add_argument('--max-images', type=int, default=0, help='evenly sample at most this many views per scene (0=all)')
        ap.add_argument('--rel-tol', type=float, default=0.01, help='GT relative depth tolerance (GtUtils.gt_labels)')
        ap.add_argument('--abs-tol', type=float, default=0.0, help='GT absolute depth tolerance floor')
        ap.add_argument('--gate', type=float, default=0.1, help='fusion confidence gate for P@/R@')
        ap.add_argument('--roc-guard', type=float, default=0.005,
                        help='max allowed per-scene-level ROC regression vs current default')
        ap.add_argument('--p01-eps', type=float, default=0.002,
                        help='slack allowed on pooled P@gate vs current default ("not worse")')
        ap.add_argument('--top', type=int, default=15, help='how many top combos to print')
        ap.add_argument('--verify-margin-invariance', action='store_true',
                        help='sanity-check that K/Pconf/Prior/Photo are identical across margin=2/3/5 exports')
        ap.add_argument('--json-out', default=None, help='write full sweep results (baseline/winner/top50) as JSON')
        args = ap.parse_args()
        main_gt(args)
    else:
        main_legacy()


if __name__ == '__main__':
    main()
