#!/usr/bin/env python3
"""Aggregate the fusion GT-eval JSONs (results_final/<TAG>_fuse_{w0,w2,w3}.json, from run_scene.sh's
EvalFusionGT.py / eth3d_eval.sh paths) into one per-scene-level table + budget check, so the fusion
completeness-vs-outlier decision is presented in one place alongside the confidence one.

Metric per scene-level (weights w0=rescue off, w2, w3):
  completeness @ MID tolerance : bmvs 0.005 (0.5% of GT-mesh bbox diag) / eth3d 0.02 (2 cm laser)
  gross-outlier fraction       : bmvs = pts >5% of diag from GT / eth3d = 1 - accuracy@0.5 m
Adopt rule (from the plan): largest w whose gross <= w0-gross + 0.05pp on EVERY scene.

Usage: python aggregate_fuse.py [results_dir]   (default: /home/ubuntu/virginia/gt_bench/results_final)
"""
import os, sys, json, glob, statistics as st

RES = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/virginia/gt_bench/results_final'
MID = {'bmvs': '0.005', 'eth3d': '0.02'}          # mid completeness tolerance key per dataset
BUDGET = 0.0005                                    # +0.05pp per-scene gross budget


def load(tag, w):
    p = os.path.join(RES, '%s_fuse_%s.json' % (tag, w))
    return json.load(open(p)) if os.path.isfile(p) else None


def main():
    tags = sorted(os.path.basename(p).rsplit('_fuse_w0.json', 1)[0]
                  for p in glob.glob(os.path.join(RES, '*_fuse_w0.json')))
    print('%-22s  %-5s | comp@mid  w0     w2     w3   |  gross%%   w0     w2     w3   | dgross w2/w3  budget'
          % ('scene-level', 'set'))
    over = {'w2': [], 'w3': []}
    rows = []
    for tag in tags:
        ds = 'eth3d' if tag.startswith('eth3d') else 'bmvs'
        mid = MID[ds]
        j = {w: load(tag, w) for w in ('w0', 'w2', 'w3')}
        if any(j[w] is None for w in j):
            print('%-22s  MISSING (%s)' % (tag, [w for w in j if j[w] is None])); continue
        comp = {w: j[w]['completeness'][mid] for w in j}
        gross = {w: j[w]['gross_outlier_frac'] for w in j}
        dg2, dg3 = gross['w2'] - gross['w0'], gross['w3'] - gross['w0']
        b2 = 'OK' if dg2 <= BUDGET else 'OVER'
        b3 = 'OK' if dg3 <= BUDGET else 'OVER'
        if dg2 > BUDGET: over['w2'].append(tag)
        if dg3 > BUDGET: over['w3'].append(tag)
        rows.append((tag, ds, comp, gross))
        print('%-22s  %-5s | %6.3f %6.3f %6.3f | %6.3f %6.3f %6.3f | %+5.2f/%+5.2fpp  %s/%s' % (
            tag, ds, comp['w0'], comp['w2'], comp['w3'],
            100 * gross['w0'], 100 * gross['w2'], 100 * gross['w3'],
            100 * dg2, 100 * dg3, b2, b3))

    def ms(v):
        return 'mean %5.2f  median %5.2f  max %5.2f' % (st.mean(v), st.median(v), max(v))

    def block(rs, name):
        if not rs:
            return
        cg2 = [100 * (r[2]['w2'] - r[2]['w0']) for r in rs]   # completeness gain vs w0 (pp)
        cg3 = [100 * (r[2]['w3'] - r[2]['w0']) for r in rs]
        ga0 = [100 * r[3]['w0'] for r in rs]                  # absolute gross-outlier (%)
        ga2 = [100 * r[3]['w2'] for r in rs]
        ga3 = [100 * r[3]['w3'] for r in rs]
        dg2 = [100 * (r[3]['w2'] - r[3]['w0']) for r in rs]   # gross ADDED vs w0 (pp)
        dg3 = [100 * (r[3]['w3'] - r[3]['w0']) for r in rs]
        print('\n%s (%d scene-levels)' % (name, len(rs)))
        print('  completeness gain vs w0 (pp):  w2 [%s]   w3 [%s]' % (ms(cg2), ms(cg3)))
        print('  absolute gross-outlier (%%):    w0 [%s]   w2 [%s]   w3 [%s]' % (ms(ga0), ms(ga2), ms(ga3)))
        print('  gross ADDED vs w0 (pp):        w2 [%s]   w3 [%s]' % (ms(dg2), ms(dg3)))

    print('\n=== aggregates: mean / median / max across scene-levels ===')
    block(rows, 'ALL')
    block([r for r in rows if r[1] == 'eth3d'], 'ETH3D')
    block([r for r in rows if r[1] == 'bmvs'], 'BlendedMVS')
    print('\n(for reference, per-scene +0.05pp gross budget: over on w2 = %d/%d, w3 = %d/%d)'
          % (len(over['w2']), len(rows), len(over['w3']), len(rows)))


if __name__ == '__main__':
    main()
