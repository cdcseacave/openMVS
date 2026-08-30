#!/usr/bin/env python3
"""Does `CreateStructure --export-retrieval-csv` reproduce the shipped engine's retrieval ranking?

The C++ retrieval pass (`GlobalDescriptors` + `PoolRetrievalDescriptor`) and polycpp's TensorRT
describe engine rank the same images by cosine over a pooled RoMa v2 descriptor. This script joins
the two `retrieval_rankings.csv` files on image index -- checking that index against the file stems
both sides carry, so a scene-order difference is caught instead of silently scoring noise -- and
reports, per capture:

  same set / same order   the strict list-identity views. NOT a gate (controller ruling R14): the
                          engine emits bf16 and any fp32 consumer is bounded well below 100% here;
                          `CROSSCHECK.md` Step 3b measures that ceiling at 64.80% with the campaign's
                          own torch fp32 tensor, so a low number here says "bf16", not "wrong".
  mean overlap            |top-K intersection| / K averaged over images -- the R14 gate (>= 95%).
  top-10 Jaccard          the brief's Step-2 view of the same quantity at K = 10.
  Spearman                rank correlation over the pairs both top-50 lists contain, which sees the
                          ORDER of the tail the overlap percentages ignore.
  |dsim|                  absolute similarity difference on the pairs both lists contain, meaningful
                          only when the two sides pool the same recipe (the LAYERS arm).

The top-K parsing and its tie-break are `roma2_onnx_crosscheck.engine_rankings`, so this script and
the export's own cross-check read the engine's file identically.

    python scripts/python/tests/compare_retrieval_rankings.py ~/megaloc-vs-dinov3-2026-08-28/captures.txt \
        --arm openmvs-roma2-20260830-roma2layers --out /tmp/compare.csv
"""
import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from roma2_onnx_crosscheck import engine_rankings  # noqa: E402  (same directory)


def read_rankings(path):
    """{idxA: [(similarity, idxB)] descending} plus {idx: stem} if the file names its endpoints."""
    ranked = defaultdict(list)
    stems = {}
    with Path(path).open() as handle:
        for row in csv.DictReader(handle):
            a, b = int(row["idxA"]), int(row["idxB"])
            ranked[a].append((float(row["similarity"]), b))
            if "imageA" in row:
                stems[a] = Path(row["imageA"]).stem
                stems[b] = Path(row["imageB"]).stem
    for values in ranked.values():
        values.sort(key=lambda t: (-t[0], t[1]))
    return ranked, stems


def read_engine_stems(path):
    """{idx: stem} from the engine dump's images.csv, the scene order its rankings index into."""
    with Path(path).open() as handle:
        return {int(row["idx"]): row["stem"] for row in csv.DictReader(handle)}


def spearman(a, b):
    """Rank correlation of two equal-length sequences; nan when either side is constant."""
    if len(a) < 3:
        return float("nan")
    ra = np.argsort(np.argsort(np.asarray(a, dtype=float)))
    rb = np.argsort(np.argsort(np.asarray(b, dtype=float)))
    if ra.std() == 0 or rb.std() == 0:
        return float("nan")
    return float(np.corrcoef(ra, rb)[0, 1])


def compare(engine_csv, mine_csv, images_csv, top_k, jaccard_k):
    engine_top = engine_rankings(engine_csv)          # {idxA: [idxB] * 16}, the crosscheck's tie-break
    engine_full, _ = read_rankings(engine_csv)
    mine_full, mine_stems = read_rankings(mine_csv)

    # the two index spaces must be the same scene order or every number below is noise
    mismatched = []
    if images_csv and mine_stems:
        engine_stems = read_engine_stems(images_csv)
        for idx, stem in sorted(mine_stems.items()):
            if idx in engine_stems and engine_stems[idx] != stem:
                mismatched.append((idx, engine_stems[idx], stem))

    same_set = same_order = 0
    overlap, jaccard, rho, dsim = [], [], [], []
    scored = 0
    for a, expected in engine_top.items():
        if a not in mine_full:
            continue
        scored += 1
        got = [b for _, b in mine_full[a][:len(expected)]]
        same_set += set(got) == set(expected)
        same_order += got == expected
        overlap.append(len(set(got) & set(expected)) / len(expected))
        ex10, got10 = set(expected[:jaccard_k]), set(got[:jaccard_k])
        jaccard.append(len(ex10 & got10) / max(1, len(ex10 | got10)))
        # order and value over the tail: every pair both top-50 lists carry
        theirs = {b: s for s, b in engine_full[a]}
        mine = {b: s for s, b in mine_full[a]}
        shared = sorted(set(theirs) & set(mine))
        if len(shared) >= 3:
            rho.append(spearman([theirs[b] for b in shared], [mine[b] for b in shared]))
            dsim.extend(abs(theirs[b] - mine[b]) for b in shared)

    finite = [r for r in rho if np.isfinite(r)]
    return dict(
        n_images=scored, n_engine=len(engine_top), n_mine=len(mine_full),
        stem_mismatches=len(mismatched),
        set_percent=100.0 * same_set / max(1, scored),
        order_percent=100.0 * same_order / max(1, scored),
        overlap_percent=100.0 * float(np.mean(overlap or [0.0])),
        jaccard_percent=100.0 * float(np.mean(jaccard or [0.0])),
        spearman_mean=float(np.mean(finite)) if finite else float("nan"),
        spearman_min=float(np.min(finite)) if finite else float("nan"),
        dsim_mean=float(np.mean(dsim)) if dsim else float("nan"),
        dsim_max=float(np.max(dsim)) if dsim else float("nan"),
    ), mismatched


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("captures", type=Path, help="file listing capture directories")
    ap.add_argument("--arm", required=True,
                    help="run folder under each capture holding the C++ retrieval_rankings.csv")
    ap.add_argument("--csv-name", default="retrieval_rankings.csv")
    ap.add_argument("--engine-run", default="roma2_power_2026-08-28")
    ap.add_argument("--engine-dump", default="selection_dump/round1",
                    help="dump directory under the engine run (round1 and oneround are identical)")
    ap.add_argument("--top-k", type=int, default=16, help="the R14 gate's K")
    ap.add_argument("--jaccard-k", type=int, default=10, help="the brief's Step-2 K")
    ap.add_argument("--min-overlap", type=float, default=95.0, help="the R14 gate, in percent")
    ap.add_argument("--out", type=Path, help="write the per-capture rows to this CSV (and .json)")
    args = ap.parse_args()

    rows, failures = [], 0
    for line in args.captures.expanduser().read_text().split():
        capture = Path(line).expanduser()
        mine_csv = capture / args.arm / args.csv_name
        dump = capture / args.engine_run / args.engine_dump
        if not mine_csv.is_file():
            print(f"skip {capture.name[:8]}: no {mine_csv}")
            continue
        if not (dump / "retrieval_rankings.csv").is_file():
            print(f"skip {capture.name[:8]}: no {dump}/retrieval_rankings.csv")
            continue
        row, mismatched = compare(dump / "retrieval_rankings.csv", mine_csv,
                                  dump / "images.csv", args.top_k, args.jaccard_k)
        row = dict(capture=capture.name[:8], arm=args.arm, **row)
        rows.append(row)
        if mismatched:
            print(f"  ERROR {row['capture']}: {len(mismatched)} index/stem mismatches, "
                  f"first {mismatched[0]}")
            failures += 1
        verdict = "PASS" if row["overlap_percent"] >= args.min_overlap else "FAIL"
        failures += verdict == "FAIL"
        print(f"{row['capture']}  n {row['n_images']:>4}  same-set {row['set_percent']:6.2f}%  "
              f"same-order {row['order_percent']:6.2f}%  top-{args.top_k} overlap "
              f"{row['overlap_percent']:6.2f}% [{verdict}]  top-{args.jaccard_k} Jaccard "
              f"{row['jaccard_percent']:6.2f}%  rho {row['spearman_mean']:.4f}  "
              f"|dsim| {row['dsim_mean']:.2e} (max {row['dsim_max']:.2e})")
    if not rows:
        return 1
    mean = float(np.mean([r["overlap_percent"] for r in rows]))
    worst = float(np.min([r["overlap_percent"] for r in rows]))
    print(f"mean top-{args.top_k} overlap {mean:.2f}%, worst capture {worst:.2f}% "
          f"(gate >= {args.min_overlap:.1f}%)")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        args.out.with_suffix(".json").write_text(json.dumps(rows, indent=1))
        print(f"wrote {args.out}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
