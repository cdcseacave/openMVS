#!/usr/bin/env python3
"""
Indirect COMPLETENESS judge for the AdjustConfidence filter, using MoGe-2 as an independent
single-view geometric witness (no ground-truth mesh required).

The fusion oracle (LabelFusionInliers) measures precision but cannot certify the population the
filter is meant to rescue: correct surface seen in too few views that strict fusion discards
(WEAK_INLIER / AMBIGUOUS). MoGe-2 gives a per-view depth+normal opinion that is independent of
multi-view confirmation, so it can certify whether MVS depth lies on the real local surface there.

Methodology (RSC-PC, designed via multi-agent review):
  - per view: robust GLOBAL affine alignment of MoGe inverse-depth to MVS inverse-depth, fit ONLY on
    CONFIDENT_INLIER pixels (independent of the confidence under test); validity-gated.
  - same-surface witness = relative-depth agreement AND camera-space normal agreement (convention
    auto-resolved); thresholds self-calibrated on held-out CONFIDENT_INLIER so 90% of true surface
    passes (calibrated on CI vs OUTLIER, measured on WEAK/AMBIGUOUS -> non-circular).
  - rescue population = {WEAK_INLIER, AMBIGUOUS} that MoGe certifies ON_SURFACE; completeness =
    fraction kept (conf>=theta). Compared NEW vs RAW vs pGeo-PLACEBO at EQUAL precision (two oracles)
    to defeat keep-everything. PRIMARY claim = AMBIGUOUS/K=0 channel (NEW is independent of fusion's K).

Usage:
  python MogeCompleteness.py <dmap_dir> <moge_npz_dir> [--cap-per-view 200000] [--tag NAME]
"""
import sys, os, glob, argparse
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from MvsUtils import loadDMAP
from EvalConfidence import load_raw_map, roc_auc

INVALID, OUTLIER, AMBIGUOUS, WEAK, CONF = 0, 1, 2, 3, 4
EPS = 1e-9
RELMAD = float(os.environ.get('WITNESS_RELMAD', '0.05'))  # per-view affine residual MAD guard (relax for metric depth)


def new_conf(K, Pconf, pGeo, photo):  # shipped defaults s=1,tau=2,kPrior=0.3,w0=0.5,floor=0.5
    gate = 1.0 - np.exp(-(K + 0.3 * pGeo) / 2.0)
    post = (pGeo + Pconf) / (1.0 + Pconf)
    c = np.clip(post * gate * (0.5 + 0.5 * photo), 0.0, 1.0)
    return np.where(K >= 1, np.maximum(c, 0.5 * photo), c)


def robust_affine(gm, gv, iters=400, band=0.05):
    """Fit gv ~= a*gm + b robustly (RANSAC + Huber IRLS). gm,gv are 1D disparities on trusted pixels."""
    n = gm.size
    if n < 50:
        return None
    rng = np.arange(n)
    best_in, best_ab = -1, None
    scale = np.median(np.abs(gv - np.median(gv))) + EPS
    tol = band * (np.median(gv) + EPS)
    # deterministic-ish sampling: evenly spaced pairs across a shuffled-by-stride order
    idx = (np.arange(iters * 2) * 2654435761 % n)
    for k in range(iters):
        i, j = idx[2 * k], idx[2 * k + 1]
        if gm[i] == gm[j]:
            continue
        a = (gv[i] - gv[j]) / (gm[i] - gm[j])
        b = gv[i] - a * gm[i]
        if a <= 0:
            continue
        res = np.abs(a * gm + b - gv)
        nin = int(np.sum(res < tol))
        if nin > best_in:
            best_in, best_ab = nin, (a, b)
    if best_ab is None:
        return None
    a, b = best_ab
    for _ in range(8):  # Huber IRLS
        res = a * gm + b - gv
        s = 1.4826 * np.median(np.abs(res - np.median(res))) + EPS
        c = 1.345 * s
        w = np.where(np.abs(res) <= c, 1.0, c / (np.abs(res) + EPS))
        W = w.sum()
        ma, mg = (w * gm).sum() / W, (w * gv).sum() / W
        cov = (w * (gm - ma) * (gv - mg)).sum()
        var = (w * (gm - ma) ** 2).sum() + EPS
        a = cov / var
        b = mg - a * ma
        if a <= 0:
            return None
    res = np.abs(a * gm + b - gv)
    mad = 1.4826 * np.median(np.abs(res - np.median(res)))
    inlier_frac = float(np.mean(res < tol))
    return a, b, inlier_frac, mad / (np.median(gv) + EPS)


def orient_cam_facing(n):  # OpenCV camera looks +z; visible-surface normal has n_z<0
    flip = n[..., 2] > 0
    return np.where(flip[..., None], -n, n)


def process_view(dmap, moge, fl, fs, fK, fPc, fPr, fPh, cap, vid):
    """Return dict of flat per-pixel arrays for the relevant (CI/OUTLIER/WEAK/AMBIG & mono-reliable) pixels, or None."""
    d = loadDMAP(dmap)
    if d is None or not d.get('has_normal') or not d.get('has_conf'):
        return None
    dv = np.asarray(d['depth_map'], np.float64)              # (H,W)
    nv = np.asarray(d['normal_map'], np.float64)             # camera-space
    H, W = dv.shape
    M = np.load(moge)
    dm, nm, mk = M['depth'].astype(np.float64), M['normal'].astype(np.float64), M['mask']
    if dm.shape != (H, W):
        return None
    label = load_raw_map(fl); support = load_raw_map(fs)
    K = load_raw_map(fK).astype(np.float64); Pc = load_raw_map(fPc).astype(np.float64)
    Pr = load_raw_map(fPr).astype(np.float64); Ph = load_raw_map(fPh).astype(np.float64)

    valid = (dv > 0) & mk & np.isfinite(dm) & (dm > 0)
    # MoGe depth-discontinuity edge band -> abstain
    gy, gx = np.gradient(np.log(np.maximum(dm, EPS)))
    grad = np.abs(gx) + np.abs(gy)
    edge = grad > np.percentile(grad[valid], 95) if valid.any() else np.zeros_like(valid)
    edge = (np.abs(np.gradient(edge.astype(float))[0]) + np.abs(np.gradient(edge.astype(float))[1]) + edge) > 0  # 1px dilate
    reliable = valid & ~edge
    if reliable.sum() < 300:
        return None
    # per-view depth percentile band
    dlo, dhi = np.percentile(dv[reliable], [2, 98])
    ci = reliable & (label == CONF) & (dv >= dlo) & (dv <= dhi)
    if ci.sum() < 200:
        return None
    gm, gv = 1.0 / dm, 1.0 / dv
    if os.environ.get('WITNESS_METRIC'):
        # metric depth (e.g. sparse-guided MapAnything): scale-only correction, no affine/inlier guard
        a = float(np.median(gv[ci] / np.maximum(gm[ci], EPS))); b = 0.0
        inl, relmad = 1.0, 0.0
    else:
        fit = robust_affine(gm[ci].ravel(), gv[ci].ravel())
        if fit is None:
            return None
        a, b, inl, relmad = fit
    sp_m = np.percentile(gm[ci], 95) - np.percentile(gm[ci], 5)
    sp_v = np.percentile(gv[ci], 95) - np.percentile(gv[ci], 5)
    if inl < 0.5 or relmad > RELMAD or sp_m < 0.3 * sp_v:   # validity guards
        return {'unreliable_view': True, 'vid': vid}
    dhat = 1.0 / np.maximum(a * gm + b, EPS)
    r_depth = np.abs(dv - dhat) / dv

    # normals: orient both camera-facing; resolve MoGe axis-sign convention on CI
    nv_o = orient_cam_facing(nv)
    nrm = np.linalg.norm(nm, axis=2, keepdims=True); nm = nm / np.maximum(nrm, EPS)
    best, bestC = -2, None
    for C in ([1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]):
        cand = orient_cam_facing(nm * np.array(C))
        md = np.median(np.sum(nv_o[ci] * cand[ci], axis=1))
        if md > best:
            best, bestC = md, C
    nm_o = orient_cam_facing(nm * np.array(bestC))
    normal_ok_view = best >= 0.8
    dot = np.clip(np.sum(nv_o * nm_o, axis=2), -1, 1)
    r_n = np.degrees(np.arccos(dot))
    # local normal variance -> unreliable normal
    nstd = np.zeros((H, W))
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            nstd += np.linalg.norm(nm_o - np.roll(np.roll(nm_o, dy, 0), dx, 1), axis=2)
    normal_reliable = reliable & normal_ok_view & (nstd < 1.5) & ~edge

    sel = reliable & np.isin(label, [CONF, OUTLIER, WEAK, AMBIGUOUS])
    ij = np.flatnonzero(sel.ravel())
    if cap and ij.size > cap:
        ij = ij[np.linspace(0, ij.size - 1, cap).astype(np.int64)]
    g = lambda A: A.ravel()[ij]
    return {'unreliable_view': False, 'vid': vid, 'best_dot': best,
            'r_depth': g(r_depth), 'r_n': g(r_n), 'normal_reliable': g(normal_reliable).astype(bool),
            'label': g(label).astype(np.int32), 'K': g(K), 'Pconf': g(Pc), 'pGeo': g(Pr), 'photo': g(Ph),
            'support': g(support).astype(np.float64)}


def main():
    np.seterr(divide='ignore', invalid='ignore')  # masked-out pixels produce inf/nan, handled by masks
    ap = argparse.ArgumentParser()
    ap.add_argument('dmap_dir'); ap.add_argument('moge_dir')
    ap.add_argument('--cap-per-view', type=int, default=200000)
    ap.add_argument('--tag', default='scene')
    args = ap.parse_args()
    dmaps = sorted(glob.glob(os.path.join(args.dmap_dir, 'depth*.dmap')))
    cols, nviews, ndrop = [], 0, 0
    for vid, dm in enumerate(dmaps):
        stem = os.path.splitext(os.path.basename(dm))[0]
        npz = os.path.join(args.moge_dir, stem + '.npz')
        paths = [os.path.join(args.dmap_dir, stem + e) for e in ['.flabel', '.fsupport', '.cfeatK', '.cfeatPconf', '.cfeatPrior', '.cfeatPhoto']]
        if not os.path.exists(npz) or not all(os.path.exists(p) for p in paths):
            continue
        r = process_view(dm, npz, *paths, cap=args.cap_per_view, vid=vid)
        if r is None:
            continue
        if r.get('unreliable_view'):
            ndrop += 1; continue
        cols.append(r); nviews += 1
    if not cols:
        print('no usable views'); sys.exit(1)
    A = {k: np.concatenate([c[k] for c in cols]) for k in ('r_depth', 'r_n', 'normal_reliable', 'label', 'K', 'Pconf', 'pGeo', 'photo', 'support')}
    vid = np.concatenate([np.full(c['r_depth'].size, c['vid']) for c in cols])
    print('=== %s : %d views used, %d dropped MONO_UNRELIABLE, %d relevant px (avg best-CI-dot %.3f) ===' % (
        args.tag, nviews, ndrop, A['label'].size, float(np.mean([c['best_dot'] for c in cols]))))

    # ---- calibrate same-surface thresholds on a held-out HALF of CI so CI ON-rate = 0.90 ----
    ci = A['label'] == CONF
    half = (vid % 2 == 0)
    cal = ci & half
    NORMCAP = 45.0  # hard cap so the normal leg actually bites (orientation, not just scale)
    def on_surface(tau_d, tau_n):
        depth_ok = A['r_depth'] <= tau_d
        norm_ok = (A['r_n'] <= tau_n) | ~A['normal_reliable']
        return depth_ok & norm_ok
    def tn_at(q):
        m = cal & A['normal_reliable']
        return min(np.percentile(A['r_n'][m], q * 100), NORMCAP) if m.any() else NORMCAP
    # joint percentile q s.t. held-out CI ON = 0.90 (normal threshold capped at NORMCAP)
    best_q = 0.95
    for q in np.linspace(0.80, 0.995, 40):
        td = np.percentile(A['r_depth'][cal], q * 100)
        if on_surface(td, tn_at(q))[ci & ~half].mean() >= 0.90:
            best_q = q; break
    tau_d = np.percentile(A['r_depth'][cal], best_q * 100)
    tau_n = tn_at(best_q)
    ON = on_surface(tau_d, tau_n)
    # OFF = disagree on depth OR (where the normal is reliable) on orientation -> catches smooth-but-wrong floaters
    OFF = (A['r_depth'] > tau_d) | (A['normal_reliable'] & (A['r_n'] > tau_n))
    print('calibrated tau_depth=%.4f tau_normal=%.1fdeg (q=%.3f)' % (tau_d, tau_n, best_q))

    # ---- SANITY: CI ON-rate, OUTLIER OFF-rate, CI-vs-OUTLIER same-surface AUC ----
    ci_test = ci & ~half
    out = A['label'] == OUTLIER
    ci_on = ON[ci_test].mean(); out_off = OFF[out].mean()
    score = -A['r_depth']  # continuous same-surface score (no confidence used)
    auc = roc_auc(score[ci_test | out], (A['label'][ci_test | out] == CONF).astype(int))
    print('SANITY  CI_ON=%.3f (>=0.85?) OUTLIER_OFF=%.3f (>=0.6?) CIvsOUT_AUC=%.3f (>=0.80?) CI_ON-OUT_ON=%.3f (>=0.5?)' % (
        ci_on, out_off, auc, ci_on - ON[out].mean()))
    valid_metric = (ci_on >= 0.85) and (out_off >= 0.6) and (auc >= 0.80)
    print('  -> MoGe witness %s for this scene' % ('VALID' if valid_metric else 'INVALID (treat completeness as low-confidence)'))

    # ---- confidences ----
    RAW = A['photo']; PLAC = A['pGeo']
    NEW = new_conf(A['K'], A['Pconf'], A['pGeo'], A['photo'])
    confs = {'NEW': NEW, 'RAW': RAW, 'PLACEBO_pGeo': PLAC}

    # ---- rescue populations ----
    resc = np.isin(A['label'], [WEAK, AMBIGUOUS])
    POS = resc & ON; NEG = resc & OFF
    ci_or_out = ci | out
    thetas = np.linspace(0.0, 1.0, 501)
    i01 = int(np.searchsorted(thetas, 0.1))

    def counts_ge(c, m):  # vectorized: for each theta, #pixels in subset m with c>=theta
        s = np.sort(c[m]); return s.size - np.searchsorted(s, thetas, side='left')

    def channel(mask_pos, mask_neg, title):
        npos, nneg = int(mask_pos.sum()), int(mask_neg.sum())
        if npos < 50:
            print('  [%s] too few POS (%d) - skip' % (title, npos)); return None
        cur = {}
        for cn, c in confs.items():
            kci, kout = counts_ge(c, ci), counts_ge(c, out)
            kpos, kneg = counts_ge(c, mask_pos), counts_ge(c, mask_neg)
            fp = kci / np.maximum(kci + kout, 1)               # fusion precision (CI vs OUTLIER, all pixels)
            rp = kpos / np.maximum(kpos + kneg, 1)             # within-rescue precision (MoGe oracle)
            cur[cn] = (kpos / npos, fp, rp)                    # completeness, fusion-prec, rescue-prec vs theta
        def comp_at(axis, P):
            o = {}
            for cn, (comp, fp, rp) in cur.items():
                ax = fp if axis == 1 else rp; order = np.argsort(ax)
                o[cn] = float(np.interp(P, ax[order], comp[order]))
            return o
        Pf = float(cur['NEW'][1][i01]); cf = comp_at(1, Pf)   # NEW operational fusion-precision
        Pr = float(cur['NEW'][2][i01]); cr = comp_at(2, Pr)
        print('  [%-10s] |POS|=%-7d |NEG|=%-6d  @equal FUSION-prec %.3f: C_NEW=%.3f C_RAW=%.3f C_PLAC=%.3f | dC(NEW-RAW)=%+.3f dC(NEW-PLAC)=%+.3f' % (
            title, npos, nneg, Pf, cf['NEW'], cf['RAW'], cf['PLACEBO_pGeo'], cf['NEW'] - cf['RAW'], cf['NEW'] - cf['PLACEBO_pGeo']))
        print('  [%-10s] %30s @equal RESCUE-prec %.3f: C_NEW=%.3f C_RAW=%.3f C_PLAC=%.3f | dC(NEW-RAW)=%+.3f' % (
            title, '', Pr, cr['NEW'], cr['RAW'], cr['PLACEBO_pGeo'], cr['NEW'] - cr['RAW']))
        return cf['NEW'] - cf['RAW']

    print('\n--- POS_rescue=%d NEG_rescue=%d (WEAK %d / AMBIG %d) ---' % (
        int(POS.sum()), int(NEG.sum()), int((resc & (A['label'] == WEAK)).sum()), int((resc & (A['label'] == AMBIGUOUS)).sum())))
    K0 = A['K'] == 0; Kw = A['K'] >= 1
    print('=== PRIMARY (clean channel, K=0, NEW independent of fusion): AMBIGUOUS/K=0 ===')
    d_amb = channel(POS & (A['label'] == AMBIGUOUS) & K0, NEG & (A['label'] == AMBIGUOUS) & K0, 'AMBIG K=0')
    print('=== few-view RESCUE target: WEAK_INLIER (K>=1, what the prior+floor is meant to keep) ===')
    d_weak = channel(POS & (A['label'] == WEAK) & Kw, NEG & (A['label'] == WEAK) & Kw, 'WEAK K>=1')
    print('=== pooled rescue (context) ===')
    channel(POS, NEG, 'POOLED')
    for cn, c in (('NEW', NEW), ('RAW', RAW)):
        kept = c >= 0.1
        print('back-compat fusion-precision@0.1 %s=%.3f' % (cn, (kept & ci).sum() / max((kept & ci_or_out).sum(), 1)))
    print('\nVERDICT(%s): witness=%s | WEAK(K>=1) dC@equalFusionPrec=%s | AMBIG/K0 dC=%s' % (
        args.tag, 'VALID' if valid_metric else 'INVALID(low-confidence)',
        ('%+.3f' % d_weak) if d_weak is not None else 'n/a', ('%+.3f' % d_amb) if d_amb is not None else 'n/a'))


if __name__ == '__main__':
    main()
