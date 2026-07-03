#!/usr/bin/env python3
"""gt_bench/aggregate_gt.py -- scan results/*.json and emit AGGREGATE_GT.md.

Usage: /home/ubuntu/miniconda3/bin/python gt_bench/aggregate_gt.py [results_dir] [out.md]
Defaults: results_dir=/home/ubuntu/virginia/gt_bench/results
          out.md=/home/ubuntu/virginia/gt_bench/AGGREGATE_GT.md

Stdlib only (json, os, re, glob, sys) -- no third-party deps, per the brief.

Reads run_scene.sh's output convention: results/<scene>_L<r>_<tag>.json, tag in
{conf_raw, conf_adj, fuse_w0, fuse_w3, timing}. Scene names are recovered from the
FILENAME (bmvs_<id>/eth3d_<name>_L<r>_<tag>.json), NOT from any 'scene' field embedded in
the JSON bodies -- EvalConfidence.py's GT-mode JSON derives its own 'scene' field from
realpath(--gt-depth-dir)'s basename, which does not reproduce the run_scene.sh scene token
(see task-5-report.md / task-7-report.md). This script is deliberately tolerant of a
partially-populated results/ directory (the baseline arrives incrementally, scene by scene,
resolution by resolution): any missing tag for a given scene/res is rendered as '-' rather
than raising.
"""
import glob
import json
import os
import re
import sys

FNAME_RE = re.compile(r'^(?P<scene>.+)_L(?P<res>\d+)_(?P<tag>conf_raw|conf_adj|fuse_w0|fuse_w3|timing)\.json$')

TAGS = ('conf_raw', 'conf_adj', 'fuse_w0', 'fuse_w3', 'timing')


def load_results(results_dir):
    """Returns {(scene, res:int): {tag: parsed_json_or_None}}."""
    out = {}
    for path in sorted(glob.glob(os.path.join(results_dir, '*.json'))):
        m = FNAME_RE.match(os.path.basename(path))
        if not m:
            print('warning: skipping unrecognized results file: %s' % path, file=sys.stderr)
            continue
        key = (m.group('scene'), int(m.group('res')))
        try:
            with open(path) as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            print('warning: could not read/parse %s (%s), treating as missing' % (path, e), file=sys.stderr)
            data = None
        out.setdefault(key, {})[m.group('tag')] = data
    return out


def sorted_keys(results):
    # sort by dataset family (bmvs before eth3d is arbitrary; alphabetical scene name, then
    # resolution level ascending) for stable, readable table ordering.
    return sorted(results.keys(), key=lambda k: (k[0], k[1]))


def fmt(x, nd=4):
    if x is None:
        return '-'
    if isinstance(x, float):
        if x != x:  # NaN
            return 'nan'
        return ('%.' + str(nd) + 'f') % x
    return str(x)


def fmt_pct(x, nd=1):
    if x is None:
        return '-'
    if isinstance(x, float) and x != x:
        return 'nan'
    return ('%.' + str(nd) + 'f%%') % (100.0 * x)


def get(d, *path):
    """Safe nested dict getter; returns None on any missing key/None along the path."""
    cur = d
    for p in path:
        if cur is None:
            return None
        cur = cur.get(p) if isinstance(cur, dict) else None
    return cur


def table1_confidence(results, keys):
    """[1] conf ROC/PR/P@0.1 raw vs adjusted, per scene x res."""
    lines = []
    lines.append('| Scene | L | ROC-AUC raw | ROC-AUC adj | dROC | PR-AUC raw | PR-AUC adj | '
                  'P@0.1 raw | P@0.1 adj | n_labeled (adj) |')
    lines.append('|---|---|---|---|---|---|---|---|---|---|')
    any_row = False
    for scene, res in keys:
        entry = results[(scene, res)]
        raw = get(entry, 'conf_raw', 'pooled')
        adj = get(entry, 'conf_adj', 'pooled')
        if raw is None and adj is None:
            continue
        any_row = True
        roc_raw, roc_adj = get(raw, 'roc_auc'), get(adj, 'roc_auc')
        droc = (roc_adj - roc_raw) if isinstance(roc_raw, float) and isinstance(roc_adj, float) else None
        lines.append('| %s | %d | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            scene, res,
            fmt(roc_raw), fmt(roc_adj), fmt(droc, 4),
            fmt(get(raw, 'pr_auc')), fmt(get(adj, 'pr_auc')),
            fmt(get(raw, 'p_at_01')), fmt(get(adj, 'p_at_01')),
            fmt(get(adj, 'n_labeled'), 0)))
    if not any_row:
        lines.append('| (no conf_raw/conf_adj results yet) | | | | | | | | | |')
    return '\n'.join(lines)


def _tol_cell(d, section, wanted_frac_labels=('0.01', '0.02', '0.05', '0.1')):
    """Renders a tol-keyed sub-dict (completeness/accuracy/f1) as 'k1=v1 k2=v2 ...',
    tolerant of missing keys and of BlendedMVS's different (fraction-of-diag) label set vs
    ETH3D's (absolute-meters) label set -- see task-6-report.md; we don't assume a shared
    vocabulary, we just print whatever tol keys are present, sorted numerically."""
    sub = get(d, section)
    if not sub:
        return '-'
    try:
        items = sorted(sub.items(), key=lambda kv: float(kv[0]))
    except ValueError:
        items = sorted(sub.items())
    return ' '.join('%s=%.3f' % (k, v) for k, v in items)


def table2_fusion(results, keys):
    """[2] fusion completeness/accuracy/F1 per tol, w0 vs w3."""
    lines = []
    lines.append('| Scene | L | n_rec w0 | n_rec w3 | dn_rec | completeness w0 | completeness w3 | '
                  'accuracy w0 | accuracy w3 | f1 w0 | f1 w3 |')
    lines.append('|---|---|---|---|---|---|---|---|---|---|---|')
    any_row = False
    for scene, res in keys:
        entry = results[(scene, res)]
        w0, w3 = entry.get('fuse_w0'), entry.get('fuse_w3')
        if w0 is None and w3 is None:
            continue
        any_row = True
        n0, n3 = get(w0, 'n_rec'), get(w3, 'n_rec')
        dn = None
        if isinstance(n0, int) and isinstance(n3, int) and n0:
            dn = (n3 - n0) / float(n0)
        lines.append('| %s | %d | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            scene, res, fmt(n0, 0), fmt(n3, 0), fmt_pct(dn),
            _tol_cell(w0, 'completeness'), _tol_cell(w3, 'completeness'),
            _tol_cell(w0, 'accuracy'), _tol_cell(w3, 'accuracy'),
            _tol_cell(w0, 'f1'), _tol_cell(w3, 'f1')))
    if not any_row:
        lines.append('| (no fuse_w0/fuse_w3 results yet) | | | | | | | | | | |')
    return '\n'.join(lines)


def table3_gross_outliers(results, keys):
    """[3] gross-outlier % (w0 vs w3)."""
    lines = []
    lines.append('| Scene | L | gross_outlier% w0 | gross_outlier% w3 | gross_tol/diag (w0) |')
    lines.append('|---|---|---|---|---|')
    any_row = False
    for scene, res in keys:
        entry = results[(scene, res)]
        w0, w3 = entry.get('fuse_w0'), entry.get('fuse_w3')
        if w0 is None and w3 is None:
            continue
        any_row = True
        gtol = get(w0, 'gross_tol_abs')
        diag = get(w0, 'diag')
        gtol_diag = ('%.4f' % (gtol / diag) if (gtol and diag) else '-')
        lines.append('| %s | %d | %s | %s | %s |' % (
            scene, res, fmt_pct(get(w0, 'gross_outlier_frac')), fmt_pct(get(w3, 'gross_outlier_frac')),
            gtol_diag))
    if not any_row:
        lines.append('| (no fuse_w0/fuse_w3 results yet) | | | | |')
    return '\n'.join(lines)


def table4_timing(results, keys):
    """[4] timing: per-map adjust ms, adjust wall, fusion wall (w0/w3), ratio."""
    lines = []
    lines.append('| Scene | L | n_dmaps | estimate (s) | adjust wall (s) | adjust compute (s) | '
                  'adjust ms/map | fuse w0 (s) | fuse w3 (s) | w3/w0 ratio |')
    lines.append('|---|---|---|---|---|---|---|---|---|---|')
    any_row = False
    for scene, res in keys:
        t = results[(scene, res)].get('timing')
        if t is None:
            continue
        any_row = True
        w0, w3 = get(t, 'fusion_w0_wall_s'), get(t, 'fusion_w3_wall_s')
        ratio = (w3 / w0) if (isinstance(w0, (int, float)) and w0) else None
        lines.append('| %s | %d | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            scene, res, fmt(get(t, 'n_dmaps'), 0), fmt(get(t, 'estimation_wall_s'), 2),
            fmt(get(t, 'adjust_wall_s'), 2), fmt(get(t, 'adjust_compute_s'), 2),
            fmt(get(t, 'adjust_ms_per_map'), 1), fmt(w0, 2), fmt(w3, 2), fmt(ratio, 3)))
    if not any_row:
        lines.append('| (no timing results yet) | | | | | | | | | |')
    return '\n'.join(lines)


def coverage_summary(results, keys):
    """A quick per-scene/res tag-presence matrix, so it's obvious at a glance what the
    (possibly still-incomplete) baseline run has produced so far."""
    lines = ['| Scene | L | ' + ' | '.join(TAGS) + ' |', '|---|---|' + '---|' * len(TAGS)]
    for scene, res in keys:
        entry = results[(scene, res)]
        cells = ['OK' if entry.get(tag) is not None else '-' for tag in TAGS]
        lines.append('| %s | %d | %s |' % (scene, res, ' | '.join(cells)))
    return '\n'.join(lines)


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/virginia/gt_bench/results'
    out_path = sys.argv[2] if len(sys.argv) > 2 else '/home/ubuntu/virginia/gt_bench/AGGREGATE_GT.md'

    results = load_results(results_dir)
    keys = sorted_keys(results)

    n_scenes = len(set(k[0] for k in keys))
    n_rows = len(keys)

    parts = []
    parts.append('# GT Bench Aggregate\n')
    parts.append('Generated by `gt_bench/aggregate_gt.py` from `%s`.\n' % results_dir)
    parts.append('%d scene/resolution combination(s) found across %d scene(s).\n' % (n_rows, n_scenes))

    parts.append('## Coverage (which tags are present per scene x resolution)\n')
    parts.append(coverage_summary(results, keys) + '\n')

    parts.append('## [1] Confidence: raw vs adjusted (GT-labeled ROC/PR/P@0.1)\n')
    parts.append(table1_confidence(results, keys) + '\n')

    parts.append('## [2] Fusion: completeness/accuracy/F1 per tolerance, w0 (rescue off) vs w3 (rescue on)\n')
    parts.append('Tol-keyed cells show every tolerance level reported by that scene\'s eval script -- '
                  'BlendedMVS keys are fractions of the reconstruction bbox diagonal, ETH3D keys are '
                  'absolute meters (see task-6-report.md); the two label sets are not directly comparable '
                  'across datasets, only within a scene/row.\n')
    parts.append(table2_fusion(results, keys) + '\n')

    parts.append('## [3] Gross-outlier fraction, w0 vs w3\n')
    parts.append('Caveat (same spirit as table 2\'s note): gross-outlier values are NOT comparable '
                  'across datasets -- the gross tolerance and the `diag` it is related to have different '
                  'semantics per dataset (BlendedMVS: gross tol fixed at 5% of the GT sample cloud\'s '
                  'bbox diagonal, no `gross_tol_abs` in its JSON so the ratio column shows `-`; ETH3D: '
                  'gross tol is a fixed absolute 0.5 m while `diag` is merely the reconstruction\'s own '
                  'bbox diagonal, so the ratio varies per scene and measures nothing dataset-comparable). '
                  'Compare w0 vs w3 within a row, or across scenes of the SAME dataset only.\n')
    parts.append(table3_gross_outliers(results, keys) + '\n')

    parts.append('## [4] Timing\n')
    parts.append(table4_timing(results, keys) + '\n')

    md = '\n'.join(parts)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or '.', exist_ok=True)
    with open(out_path, 'w') as f:
        f.write(md)
    print('AGGREGATE_GT.md written to %s (%d scene/res rows)' % (out_path, n_rows))
    print(md)


if __name__ == '__main__':
    main()
