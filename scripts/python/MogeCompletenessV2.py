#!/usr/bin/env python3
"""
v2 of the MoGe completeness witness: tighten v1 by aggregating MoGe across NEIGHBOUR views.

For each reference MVS 3D point X, project it into geometry-selected neighbour views, sample each
neighbour's GLOBALLY-aligned MoGe surface, and take a robust point-to-plane consensus residual
rho_cross (median over neighbours). Independent per-view MoGe scale/distortion noise averages out
(~1/sqrt(N)), while a wrong-depth slab's bias is SHARED across neighbours and survives -> the witness
gets tighter (lower CI scatter) without explaining away outliers. The MoGe confirmation count Km is
reported but never enters the ON/OFF decision (avoids the Km<->filter-K confound).

Controls (per the design): tightening proof (std and AUC of rho_cross vs single-view rho), an
INVARIANCE check (completeness dC computed under BOTH the v1 single-view and v2 cross-view witness),
and a SCRAMBLE control (random neighbours -> AUC ~0.5, dC ~0).

Usage: python MogeCompletenessV2.py <dmap_dir> <moge_npz_dir> [--tag NAME] [--cap-per-view 40000] [--nneigh 8]
"""
import sys, os, glob, argparse
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from MvsUtils import loadDMAP
from EvalConfidence import load_raw_map, roc_auc
from MogeCompleteness import robust_affine, orient_cam_facing, new_conf
from MogeCompleteness import INVALID, OUTLIER, AMBIGUOUS, WEAK, CONF, EPS, RELMAD

CACHE = {}; CACHE_ORDER = []; CACHE_MAX = 26


def prep_view(vid, paths):
    if vid in CACHE:
        return CACHE[vid]
    dmap, npz, fl, fK, fPc, fPr, fPh = paths
    d = loadDMAP(dmap)
    if d is None or not d.get('has_normal'):
        CACHE[vid] = None; return None
    dv = np.asarray(d['depth_map'], np.float64)
    nv = np.asarray(d['normal_map'], np.float64)            # MVS camera-space normal
    H, W = dv.shape
    M = np.load(npz)
    dm, nm, mk = M['depth'].astype(np.float64), M['normal'].astype(np.float64), M['mask']
    if dm.shape != (H, W):
        CACHE[vid] = None; return None
    K = np.asarray(d['depth_K'], np.float64); R = np.asarray(d['R'], np.float64); C = np.asarray(d['C'], np.float64).reshape(3)
    valid = (dv > 0) & mk & np.isfinite(dm) & (dm > 0)
    gy, gx = np.gradient(np.log(np.maximum(dm, EPS)))
    grad = np.abs(gx) + np.abs(gy)
    edge = grad > (np.percentile(grad[valid], 95) if valid.any() else 1e9)
    edge = (np.abs(np.gradient(edge.astype(float))[0]) + np.abs(np.gradient(edge.astype(float))[1]) + edge) > 0
    reliable = valid & ~edge
    label = load_raw_map(fl)
    feats = (load_raw_map(fK).astype(np.float64), load_raw_map(fPc).astype(np.float64),
             load_raw_map(fPr).astype(np.float64), load_raw_map(fPh).astype(np.float64))
    # per-view global affine on CI (g_mvs ~ a*g_moge + b)
    res = None
    if reliable.sum() >= 300:
        dlo, dhi = np.percentile(dv[reliable], [2, 98])
        ci = reliable & (label == CONF) & (dv >= dlo) & (dv <= dhi)
        if ci.sum() >= 200:
            fit = robust_affine((1.0 / dm)[ci].ravel(), (1.0 / dv)[ci].ravel())
            if fit is not None:
                a, b, inl, relmad = fit
                spm = np.percentile((1.0/dm)[ci], 95) - np.percentile((1.0/dm)[ci], 5)
                spv = np.percentile((1.0/dv)[ci], 95) - np.percentile((1.0/dv)[ci], 5)
                if inl >= 0.5 and relmad <= RELMAD and spm >= 0.3 * spv:
                    res = (a, b, ci)
    if res is None:
        CACHE[vid] = None; return None
    a, b, ci = res
    g_al = a * (1.0 / dm) + b                                # aligned inverse-depth (MVS scale)
    # resolve MoGe normal sign convention on CI, world-frame normals for both
    nv_w = R.T @ orient_cam_facing(nv).reshape(-1, 3).T; nv_w = nv_w.T.reshape(H, W, 3)
    nmn = nm / np.maximum(np.linalg.norm(nm, axis=2, keepdims=True), EPS)
    best, bestC = -2, [1, 1, 1]
    nv_o = orient_cam_facing(nv)
    for Csgn in ([1,1,1],[1,-1,-1],[-1,1,-1],[-1,-1,1]):
        cand = orient_cam_facing(nmn * np.array(Csgn))
        md = np.median(np.sum(nv_o[ci] * cand[ci], axis=1))
        if md > best: best, bestC = md, Csgn
    nm_w = R.T @ orient_cam_facing(nmn * np.array(bestC)).reshape(-1, 3).T; nm_w = nm_w.T.reshape(H, W, 3)
    Kinv = np.linalg.inv(K)
    viewdir = R.T @ np.array([0.0, 0.0, 1.0])                 # camera +z in world
    out = dict(K=K, Kinv=Kinv, R=R, C=C, H=H, W=W, dv=dv, dm=dm, g_al=g_al, mask=reliable,
               nm_w=nm_w, nv_w=nv_w, label=label, feats=feats, ci=ci, viewdir=viewdir, bestdot=best)
    CACHE[vid] = out; CACHE_ORDER.append(vid)
    if len(CACHE_ORDER) > CACHE_MAX:
        old = CACHE_ORDER.pop(0)
        if old in CACHE and old != vid: CACHE.pop(old, None)
    return out


def main():
    np.seterr(divide='ignore', invalid='ignore')
    ap = argparse.ArgumentParser()
    ap.add_argument('dmap_dir'); ap.add_argument('moge_dir')
    ap.add_argument('--tag', default='scene'); ap.add_argument('--cap-per-view', type=int, default=40000)
    ap.add_argument('--nneigh', type=int, default=8); ap.add_argument('--scramble', action='store_true')
    args = ap.parse_args()
    dmaps = sorted(glob.glob(os.path.join(args.dmap_dir, 'depth*.dmap')))
    PATHS = {}
    for f in dmaps:
        stem = os.path.splitext(os.path.basename(f))[0]
        npz = os.path.join(args.moge_dir, stem + '.npz')
        fl = os.path.join(args.dmap_dir, stem + '.flabel')
        ff = [os.path.join(args.dmap_dir, stem + e) for e in ('.cfeatK', '.cfeatPconf', '.cfeatPrior', '.cfeatPhoto')]
        if os.path.exists(npz) and os.path.exists(fl) and all(os.path.exists(x) for x in ff):
            try:
                vid = int(loadDMAP(f)['reference_view_id'])
            except Exception:
                continue
            PATHS[vid] = (f, npz, fl, ff[0], ff[1], ff[2], ff[3])
    vids = sorted(PATHS)
    # light meta pass (poses + reliability) for neighbour selection
    meta = {}
    for vid in vids:
        v = prep_view(vid, PATHS[vid])
        if v is not None:
            meta[vid] = (v['C'].copy(), v['viewdir'].copy(), v['bestdot'])
    rvids = list(meta)
    print('=== %s v2 : %d/%d reliable views ===' % (args.tag, len(rvids), len(vids)))
    rng = np.random.default_rng(0)

    def neighbours(ref):
        c0, d0, _ = meta[ref]
        cand = []
        for n in rvids:
            if n == ref: continue
            cn, dn, _ = meta[n]
            ang = np.degrees(np.arccos(np.clip(d0.dot(dn), -1, 1)))
            if 3.0 <= ang <= 55.0:
                cand.append((np.linalg.norm(cn - c0), n))
        cand.sort()
        sel = [n for _, n in cand[:args.nneigh]]
        if args.scramble:                                    # control: random reliable views
            pool = [n for n in rvids if n != ref]
            sel = list(rng.choice(pool, size=min(args.nneigh, len(pool)), replace=False))
        return sel

    COLS = {k: [] for k in ('rho_cross', 'rho_single', 'r_n', 'M', 'Km', 'label', 'K', 'Pconf', 'pGeo', 'photo', 'vid')}
    for ref in rvids:
        v = prep_view(ref, PATHS[ref])
        if v is None: continue
        sel = neighbours(ref)
        if not sel: continue
        H, W = v['H'], v['W']
        relevant = v['mask'] & np.isin(v['label'], [CONF, OUTLIER, WEAK, AMBIGUOUS])
        ij = np.flatnonzero(relevant.ravel())
        if ij.size == 0: continue
        if ij.size > args.cap_per_view:
            ij = ij[np.linspace(0, ij.size - 1, args.cap_per_view).astype(np.int64)]
        r, c = np.divmod(ij, W)
        dvp = v['dv'].ravel()[ij]
        uv1 = np.stack([c, r, np.ones_like(c)], 0).astype(float)
        Xc = (v['Kinv'] @ uv1) * dvp                          # ref camera coords
        Xw = v['R'].T @ Xc + v['C'][:, None]                  # world (3, P)
        nrefw = v['nv_w'].reshape(-1, 3)[ij]                  # ref MVS world normal
        # single-view residual (v1-style: ref's own aligned MoGe point-to-plane)
        dhat_ref = 1.0 / np.maximum(v['g_al'].ravel()[ij], EPS)
        rho_single = np.abs(dvp - dhat_ref) / dvp
        # cross-view point-to-plane against neighbours' aligned MoGe surfaces
        P = ij.size
        RHO = np.full((P, len(sel)), np.nan); CONF_N = np.zeros((P, len(sel)), bool)
        for k, n in enumerate(sel):
            vn = prep_view(n, PATHS[n])
            if vn is None: continue
            cam = vn['R'] @ (Xw - vn['C'][:, None])           # (3,P)
            z = cam[2]; ok = z > 0
            pix = vn['K'] @ cam
            u = np.full(P, -1); vv = np.full(P, -1)
            with np.errstate(invalid='ignore'):
                u[ok] = np.round(pix[0, ok] / z[ok]).astype(int); vv[ok] = np.round(pix[1, ok] / z[ok]).astype(int)
            ins = ok & (u >= 0) & (u < vn['W']) & (vv >= 0) & (vv < vn['H'])
            if not ins.any(): continue
            uu, vu = u[ins], vv[ins]
            mrel = vn['mask'][vu, uu]
            g = vn['g_al'][vu, uu]
            good = mrel & (g > EPS)
            idxg = np.flatnonzero(ins)[good]
            if idxg.size == 0: continue
            d_al = 1.0 / g[good]
            uvn = np.stack([uu[good], vu[good], np.ones(idxg.size)], 0).astype(float)
            Xsurf = vn['R'].T @ ((vn['Kinv'] @ uvn) * d_al) + vn['C'][:, None]
            nw = vn['nm_w'][vu[good], uu[good]]               # (g,3)
            s = np.sum((Xw[:, idxg] - Xsurf) * nw.T, axis=0)
            rho = np.abs(s) / dvp[idxg]
            RHO[idxg, k] = rho
        valid_n = ~np.isnan(RHO)
        M = valid_n.sum(1)
        with np.errstate(invalid='ignore', divide='ignore'):
            rho_cross = np.nanmedian(np.where(valid_n, RHO, np.nan), axis=1)
        Km = (np.where(valid_n, RHO, np.inf) <= 0.05).sum(1)
        for arr, key in ((rho_cross, 'rho_cross'), (rho_single, 'rho_single'), (M, 'M'), (Km, 'Km'),
                         (v['label'].ravel()[ij], 'label'), (dvp * 0 + ref, 'vid')):
            COLS[key].append(arr)
        Kf, Pc, Pr, Ph = v['feats']
        COLS['K'].append(Kf.ravel()[ij]); COLS['Pconf'].append(Pc.ravel()[ij])
        COLS['pGeo'].append(Pr.ravel()[ij]); COLS['photo'].append(Ph.ravel()[ij])
        COLS['r_n'].append(np.full(P, 0.0))                   # normal residual folded into point-to-plane here
    A = {k: np.concatenate(v) for k, v in COLS.items() if v}
    A['label'] = A['label'].astype(np.int32)
    print('relevant px=%d  median M(neighbours)=%.1f' % (A['label'].size, float(np.median(A['M']))))

    # ---- choose residual: cross where M>=2 else single (single-view = thin-evidence fallback) ----
    A['rho'] = np.where(A['M'] >= 2, A['rho_cross'], A['rho_single'])
    fin = np.isfinite(A['rho']) & np.isfinite(A['rho_single'])
    for k in list(A): A[k] = A[k][fin]
    ci = A['label'] == CONF; out = A['label'] == OUTLIER
    half = (A['vid'].astype(int) % 2 == 0)
    mc = A['M'] >= 2

    def gates(rho, mask, name):
        cal = ci & half & mask
        if cal.sum() < 50:
            print('  [%-9s] too few CI in subset' % name); return None
        tau = np.percentile(rho[cal], 90)
        ON = (rho <= tau) & mask; OFF = (rho > 2 * tau) & mask
        cit = ci & ~half & mask; o = out & mask
        ci_on = ON[cit].mean(); out_off = OFF[o].mean() if o.any() else float('nan')
        sub = cit | o; auc = roc_auc(-rho[sub], (A['label'][sub] == CONF).astype(int))
        valid = (ci_on >= 0.85) and (out_off >= 0.6) and (auc >= 0.85)
        print('  [%-9s] n=%-7d tau=%.4f CI_ON=%.3f OUTLIER_OFF=%.3f AUC=%.3f std_CI=%.4f -> %s' % (
            name, int(mask.sum()), tau, ci_on, out_off, auc, np.std(rho[cit]), 'VALID' if valid else 'borderline'))
        return dict(ON=ON, OFF=OFF, valid=valid, std=np.std(rho[cit]), auc=auc)

    allm = np.ones_like(ci)
    print('--- witness gates (CI-calibrated; held-out CI / full OUTLIER; conservative) ---')
    gs_all = gates(A['rho_single'], allm, 'single@all')
    gs_mc = gates(A['rho_single'], mc, 'single@M>=2')      # same pixels as cross, for fair tightening compare
    gx = gates(A['rho_cross'], mc, 'cross@M>=2') if mc.any() else None
    gcomb = gates(A['rho'], allm, 'combined')
    if gs_mc and gx:
        print('  TIGHTENING (M>=2 CI): std %.4f -> %.4f (%.0f%% drop) | AUC %.3f -> %.3f (d=%+.3f)' % (
            gs_mc['std'], gx['std'], 100 * (1 - gx['std'] / max(gs_mc['std'], 1e-9)),
            gs_mc['auc'], gx['auc'], gx['auc'] - gs_mc['auc']))
    ON, OFF = gcomb['ON'], gcomb['OFF']
    ON_s, OFF_s = gs_all['ON'], gs_all['OFF']

    # ---- completeness dC under the COMBINED witness, plus invariance vs single-view ----
    RAW = A['photo']; PLAC = A['pGeo']; NEW = new_conf(A['K'], A['Pconf'], A['pGeo'], A['photo'])
    confs = {'NEW': NEW, 'RAW': RAW, 'PLAC': PLAC}
    thetas = np.linspace(0.0, 1.0, 401); i01 = int(np.searchsorted(thetas, 0.1))

    def cge(c, m):
        s = np.sort(c[m]); return s.size - np.searchsorted(s, thetas, 'left')

    def dC(ON_, OFF_, posmask, label):
        POS = posmask & ON_; NEG = posmask & OFF_
        np_, nn = int(POS.sum()), int(NEG.sum())
        if np_ < 50: return None
        cur = {}
        for cn, c in confs.items():
            kci, ko = cge(c, ci), cge(c, out); comp = cge(c, POS) / np_
            cur[cn] = (comp, kci / np.maximum(kci + ko, 1))
        Pf = float(cur['NEW'][1][i01])
        def at(P, cn):
            comp, fp = cur[cn]; o = np.argsort(fp); return float(np.interp(P, fp[o], comp[o]))
        return np_, nn, Pf, at(Pf, 'NEW'), at(Pf, 'RAW'), at(Pf, 'PLAC')

    print('--- rescue completeness @ equal fusion-precision (NEW@0.1), COMBINED witness ---')
    K0 = A['K'] == 0; Kw = A['K'] >= 1
    for title, pm in (('AMBIG/K0', np.isin(A['label'], [AMBIGUOUS]) & K0),
                      ('WEAK/K>=1', (A['label'] == WEAK) & Kw),
                      ('POOLED', np.isin(A['label'], [WEAK, AMBIGUOUS]))):
        r = dC(ON, OFF, pm, title)
        if r: print('  [%-9s] |POS|=%-7d |NEG|=%-6d P*=%.3f C_NEW=%.3f C_RAW=%.3f C_PLAC=%.3f dC(NEW-RAW)=%+.3f dC(NEW-PLAC)=%+.3f' % (
            title, r[0], r[1], r[2], r[3], r[4], r[5], r[3] - r[4], r[3] - r[5]))
    print('--- INVARIANCE: WEAK/K>=1 dC under single-view vs cross witness ---')
    for nm_, (ONx, OFFx) in (('single', (ON_s, OFF_s)), ('combined', (ON, OFF))):
        r = dC(ONx, OFFx, (A['label'] == WEAK) & Kw, 'w')
        if r: print('  [%-8s] dC(NEW-RAW)=%+.3f (POS=%d)' % (nm_, r[3] - r[4], r[0]))


if __name__ == '__main__':
    main()
