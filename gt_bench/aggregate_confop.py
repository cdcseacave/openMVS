#!/usr/bin/env python3
"""Aggregate the confidence GT operating-frontier sweep (results_confop/<TAG>_confop_{raw,adj}.json,
produced by run_confop.sh) into a raw-vs-adjusted comparison on the two fusion-comparable GT metrics:
outlier CONTAMINATION and COMPLETENESS.

For each scene-level it reads the pooled operating frontier of the raw (pre-adjust) and adjusted
confidence and reports, at matched operating points:
  * completeness kept at a fixed outlier-contamination budget  (higher = better; adj should win)
  * outlier contamination at a fixed completeness target       (lower  = better; adj should win)
plus ROC-AUC for continuity with the earlier headline. Prints a per-scene-level table and dataset /
overall aggregates (mean deltas + win counts). No deps beyond the stdlib.

Usage: python aggregate_confop.py [results_confop_dir]
"""
import os, sys, json, glob

RESDIR = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/virginia/gt_bench/results_confop'
CB = ['0.005', '0.010', '0.020', '0.050']   # contamination budgets -> completeness kept
CT = ['0.90', '0.95', '0.99']               # completeness targets   -> contamination admitted


def load(tag, variant):
    p = os.path.join(RESDIR, '%s_confop_%s.json' % (tag, variant))
    if not os.path.isfile(p):
        return None
    with open(p) as f:
        return json.load(f)['pooled']


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def fnum(x, pct=False):
    if x is None:
        return '  n/a'
    return ('%5.1f%%' % (100 * x)) if pct else ('%.4f' % x)


def main():
    tags = sorted(os.path.splitext(os.path.basename(p))[0].rsplit('_confop_', 1)[0]
                  for p in glob.glob(os.path.join(RESDIR, '*_confop_adj.json')))
    if not tags:
        print('no *_confop_adj.json in %s' % RESDIR); return

    rows = []
    for tag in tags:
        r, a = load(tag, 'raw'), load(tag, 'adj')
        if r is None or a is None:
            print('!! missing pair for %s (raw=%s adj=%s)' % (tag, r is not None, a is not None)); continue
        ro, ao = r['operating'], a['operating']
        rows.append(dict(
            tag=tag, dataset='eth3d' if tag.startswith('eth3d') else 'bmvs',
            roc_r=r['roc_auc'], roc_a=a['roc_auc'],
            compl_r={b: ro['completeness_at_contam'][b] for b in CB},
            compl_a={b: ao['completeness_at_contam'][b] for b in CB},
            contam_r={t: ro['contam_at_completeness'][t] for t in CT},
            contam_a={t: ao['contam_at_completeness'][t] for t in CT}))

    # ---- per-scene-level table: completeness @ 1% contamination, contamination @ 95% completeness ----
    print('\n=== per scene-level (headline operating points) ===')
    print('%-22s  ROC raw->adj    compl@1%%contam raw->adj (d)     contam@95%%compl raw->adj (d)' % 'scene-level')
    for x in rows:
        cr, ca = x['compl_r']['0.010'], x['compl_a']['0.010']
        kr, ka = x['contam_r']['0.95'], x['contam_a']['0.95']
        dc = None if (cr is None or ca is None) else ca - cr
        dk = None if (kr is None or ka is None) else kr - ka   # positive = adj cleaner
        print('%-22s  %.4f->%.4f   %s->%s (%+5.1fpp)   %s->%s (%+5.2fpp)' % (
            x['tag'], x['roc_r'], x['roc_a'], fnum(cr, 1), fnum(ca, 1),
            0 if dc is None else 100 * dc, fnum(kr, 1), fnum(ka, 1),
            0 if dk is None else 100 * dk))

    # ---- aggregates ----
    def agg(subset, label):
        if not subset:
            return
        print('\n=== %s (%d scene-levels) ===' % (label, len(subset)))
        print('ROC-AUC pooled: %.4f -> %.4f  (mean d %+.4f)' % (
            mean([x['roc_r'] for x in subset]), mean([x['roc_a'] for x in subset]),
            mean([x['roc_a'] - x['roc_r'] for x in subset])))
        print('COMPLETENESS kept at contamination budget (higher=better):')
        for b in CB:
            dr = mean([x['compl_r'][b] for x in subset]); da = mean([x['compl_a'][b] for x in subset])
            deltas = [x['compl_a'][b] - x['compl_r'][b] for x in subset
                      if x['compl_a'][b] is not None and x['compl_r'][b] is not None]
            win = sum(1 for d in deltas if d > 1e-9); tot = len(deltas)
            print('  <= %-5s contam: raw %s  adj %s  mean d %+6.1fpp  adj>=raw %d/%d' % (
                b, fnum(dr, 1), fnum(da, 1), 100 * mean(deltas) if deltas else 0, win, tot))
        print('CONTAMINATION admitted at completeness target (lower=better):')
        for t in CT:
            dr = mean([x['contam_r'][t] for x in subset]); da = mean([x['contam_a'][t] for x in subset])
            deltas = [x['contam_r'][t] - x['contam_a'][t] for x in subset
                      if x['contam_a'][t] is not None and x['contam_r'][t] is not None]
            win = sum(1 for d in deltas if d > 1e-9); tot = len(deltas)
            print('  >= %-4s compl: raw %s  adj %s  mean d %+6.2fpp  adj<=raw %d/%d' % (
                t, fnum(dr, 1), fnum(da, 1), 100 * mean(deltas) if deltas else 0, win, tot))

    agg([x for x in rows if x['dataset'] == 'eth3d'], 'ETH3D')
    agg([x for x in rows if x['dataset'] == 'bmvs'], 'BlendedMVS')
    agg(rows, 'ALL')


if __name__ == '__main__':
    main()
