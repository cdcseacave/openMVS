#!/usr/bin/env python3
"""
Task 9 equivalence check: compare AdjustConfidence's per-pixel exported features
(.cfeatK/.cfeatPconf/.cfeatPrior/.cfeatPhoto, written by --export-conf-features 1) between a
REFERENCE run (old double-precision confirmation loop) and a NEW run (fused single-precision
rewrite, see NeighborProj in libs/MVS/SceneDensify.cpp), both computed from the SAME raw
depth/normal/confidence inputs.

For each (ref_dir, new_dir) scene pair, matches sidecars by dmap stem ("depth0001" etc, same
convention as SweepConfParams.py), restricts the comparison to pixels with a valid reference
depth (loaded from the .dmap beside the ref sidecars via MvsUtils.loadDMAP -- the depth-map is
untouched by --export-conf-features, so ref and new share the identical validity mask), and
reports:
  - % of valid pixels with IDENTICAL confirmation count K (uint16 cfeatK)
  - max |Pconf_new - Pconf_ref| restricted to pixels where K matched
  - a histogram of (K_new - K_ref) over all valid pixels (diagnostic: how far mismatches drift)
  - a sanity check on cfeatPrior/cfeatPhoto (untouched code paths -- ComputeIntraMapPrior and the
    raw confMap read -- so these should match ~exactly; a mismatch here would indicate the rewrite
    disturbed something outside the confirmation loop)

Small float-vs-double boundary flips in K are EXPECTED (see task-9-brief.md's acceptance
thresholds): the new loop accumulates the ref->neighbor projection entirely in float32 instead of
float64-then-truncate, so a neighbor whose gate margin sits within float rounding error of a
threshold can flip in/out. Acceptance (Task 9 brief): >=99.5% identical K, max|dPconf|<=1e-3
where K matches. The end-to-end GT ROC-AUC delta (<=0.002) is checked separately in Phase 2 via
EvalConfidence.py on the two full re-adjusted scenes -- this script only isolates the per-pixel
math of the confirmation loop itself.

Usage:
  python compare_features.py --pair REF_DIR NEW_DIR [--pair REF_DIR2 NEW_DIR2 ...] [--json out.json]

Where REF_DIR/NEW_DIR are directories containing depthNNNN.dmap + depthNNNN.cfeat{K,Pconf,Prior,Photo}
(e.g. gt_bench/task9/ref_features/eth3d_courtyard/L2 vs gt_bench/task9/new_features/eth3d_courtyard/L2).
No dependency beyond numpy (and MvsUtils for .dmap parsing, EvalConfidence for load_raw_map).
"""
import os
import sys
import glob
import json
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'python'))
from MvsUtils import loadDMAP
from EvalConfidence import load_raw_map

FEATS = ('cfeatK', 'cfeatPconf', 'cfeatPrior', 'cfeatPhoto')


def load_stem(d, stem):
    """Load {K, Pconf, Prior, Photo, valid} flat arrays for one dmap stem, or None if incomplete."""
    p = lambda ext: os.path.join(d, stem + '.' + ext)
    if not all(os.path.exists(p(e)) for e in FEATS):
        return None
    dmap_path = os.path.join(d, stem + '.dmap')
    if not os.path.exists(dmap_path):
        return None
    depth = loadDMAP(dmap_path)['depth_map']
    valid = (depth.ravel() > 0)
    return {
        'K': load_raw_map(p('cfeatK')).ravel().astype(np.int64),
        'Pconf': load_raw_map(p('cfeatPconf')).ravel().astype(np.float64),
        'Prior': load_raw_map(p('cfeatPrior')).ravel().astype(np.float64),
        'Photo': load_raw_map(p('cfeatPhoto')).ravel().astype(np.float64),
        'valid': valid,
    }


def compare_scene(ref_dir, new_dir, khist_range=5):
    """Compare every matched stem in ref_dir/new_dir. Returns per-scene aggregate dict."""
    stems = sorted(os.path.splitext(os.path.basename(p))[0]
                    for p in glob.glob(os.path.join(ref_dir, '*.cfeatK')))
    n_valid_total = 0
    n_kmatch_total = 0
    max_dpconf = 0.0
    max_dprior = 0.0
    max_dphoto = 0.0
    khist = {}  # dK -> count, dK clamped to [-khist_range-1(overflow bucket), +khist_range+1]
    per_image = []
    n_images_compared = 0
    for stem in stems:
        ref = load_stem(ref_dir, stem)
        new = load_stem(new_dir, stem)
        if ref is None or new is None:
            continue
        if ref['K'].size != new['K'].size:
            print('warning: size mismatch for %s (ref=%d new=%d), skipping' %
                  (stem, ref['K'].size, new['K'].size), file=sys.stderr)
            continue
        n_images_compared += 1
        valid = ref['valid']
        n_valid = int(valid.sum())
        n_valid_total += n_valid
        if n_valid == 0:
            continue

        dK = (new['K'] - ref['K'])[valid]
        kmatch = (dK == 0)
        n_kmatch = int(kmatch.sum())
        n_kmatch_total += n_kmatch

        if n_kmatch > 0:
            dpconf = np.abs(new['Pconf'][valid][kmatch] - ref['Pconf'][valid][kmatch])
            max_dpconf = max(max_dpconf, float(dpconf.max()))

        # sanity: Prior (ComputeIntraMapPrior, untouched) and Photo (raw confMap read, untouched)
        dprior = np.abs(new['Prior'][valid] - ref['Prior'][valid])
        dphoto = np.abs(new['Photo'][valid] - ref['Photo'][valid])
        max_dprior = max(max_dprior, float(dprior.max()) if dprior.size else 0.0)
        max_dphoto = max(max_dphoto, float(dphoto.max()) if dphoto.size else 0.0)

        clamped = np.clip(dK, -khist_range - 1, khist_range + 1)
        vals, counts = np.unique(clamped, return_counts=True)
        for v, c in zip(vals.tolist(), counts.tolist()):
            khist[v] = khist.get(v, 0) + c

        per_image.append({
            'stem': stem,
            'n_valid': n_valid,
            'frac_k_match': n_kmatch / n_valid,
            'max_dpconf_where_k_match': float(dpconf.max()) if n_kmatch > 0 and dpconf.size else 0.0,
        })

    frac_k_match = (n_kmatch_total / n_valid_total) if n_valid_total else float('nan')
    return {
        'ref_dir': ref_dir,
        'new_dir': new_dir,
        'n_images_compared': n_images_compared,
        'n_images_total_ref': len(stems),
        'n_valid_pixels': n_valid_total,
        'frac_k_match': frac_k_match,
        'max_dpconf_where_k_match': max_dpconf,
        'max_dprior_sanity': max_dprior,
        'max_dphoto_sanity': max_dphoto,
        'k_diff_histogram': {str(k): v for k, v in sorted(khist.items())},
        'per_image': per_image,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pair', nargs=2, action='append', metavar=('REF_DIR', 'NEW_DIR'), required=True,
                     help='one ref/new feature-dir pair per scene; repeat for multiple scenes')
    ap.add_argument('--json', default=None, help='write full results (incl. per-image) to this path')
    ap.add_argument('--k-hist-range', type=int, default=5,
                     help='|dK| beyond this is pooled into a single overflow bucket (default 5)')
    args = ap.parse_args()

    scenes = []
    for ref_dir, new_dir in args.pair:
        print('=== %s  vs  %s ===' % (ref_dir, new_dir))
        res = compare_scene(ref_dir, new_dir, args.k_hist_range)
        scenes.append(res)
        print('  images compared: %d/%d' % (res['n_images_compared'], res['n_images_total_ref']))
        print('  valid pixels:    %d' % res['n_valid_pixels'])
        print('  K identical:     %.4f%%  (accept >= 99.5%%)' % (100.0 * res['frac_k_match']))
        print('  max|dPconf| where K matches: %.6g  (accept <= 1e-3)' % res['max_dpconf_where_k_match'])
        print('  sanity max|dPrior|=%.3g max|dPhoto|=%.3g (expect ~0, code path untouched)' %
              (res['max_dprior_sanity'], res['max_dphoto_sanity']))
        print('  dK histogram (K_new - K_ref, +-overflow clamped): %s' % res['k_diff_histogram'])

    n_valid_pooled = sum(s['n_valid_pixels'] for s in scenes)
    n_match_pooled = sum(round(s['frac_k_match'] * s['n_valid_pixels']) for s in scenes if s['n_valid_pixels'])
    pooled_frac = (n_match_pooled / n_valid_pooled) if n_valid_pooled else float('nan')
    pooled_max_dpconf = max((s['max_dpconf_where_k_match'] for s in scenes), default=0.0)
    print('\n=== POOLED across %d scene(s) ===' % len(scenes))
    print('  K identical: %.4f%%  (accept >= 99.5%%)' % (100.0 * pooled_frac))
    print('  max|dPconf| where K matches: %.6g  (accept <= 1e-3)' % pooled_max_dpconf)
    verdict = pooled_frac >= 0.995 and pooled_max_dpconf <= 1e-3
    print('  VERDICT: %s' % ('PASS' if verdict else 'FAIL -- see per-scene numbers above'))

    if args.json:
        out = {
            'scenes': scenes,
            'pooled': {
                'n_valid_pixels': n_valid_pooled,
                'frac_k_match': pooled_frac,
                'max_dpconf_where_k_match': pooled_max_dpconf,
                'pass': bool(verdict),
            },
        }
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=2)
        print('wrote %s' % args.json)

    return 0 if verdict else 1


if __name__ == '__main__':
    sys.exit(main())
