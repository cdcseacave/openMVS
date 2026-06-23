#!/usr/bin/env python3
"""Collate metrics.json + evaluation.json into a markdown table for the PR.

Usage: make_table.py <TNT_ROOT> <Scene> [<Scene> ...]

Picks up whatever backends are present (metal, cpu, and cuda_* dropped in by
community contributors via contribute_cuda.sh), so the same script grows the
table as the CUDA column gets filled.
"""
import json
import os
import sys


def load(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def fmt(v, nd=3):
    return "—" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


def main():
    root, scenes = sys.argv[1], sys.argv[2:]
    # discover backend labels from eval_* dirs across scenes
    labels = []
    for s in scenes:
        sd = os.path.join(root, s)
        if not os.path.isdir(sd):
            continue
        for d in sorted(os.listdir(sd)):
            if d.startswith("eval_"):
                lab = d[len("eval_"):]
                if lab not in labels:
                    labels.append(lab)
    order = {"metal": 0, "cpu": 1}
    labels.sort(key=lambda x: (order.get(x, 2), x))

    print("## Tanks-and-Temples — Metal vs CPU"
          + (" vs CUDA" if any(l.startswith("cuda") for l in labels) else "")
          + " (training set, F-score @ default τ)\n")
    print("All backends densify the **identical shared `scene.mvs`**; only the "
          "PatchMatch backend differs. Evaluation (GT, crop, toolbox) is held "
          "constant. Timings are wall-clock on each contributor's machine and "
          "are **not** cross-architecture comparable — see hardware notes.\n")

    head = "| Scene | Backend | F1 ↑ | Precision | Recall | Points | Wall (s) |"
    sep = "|---|---|---|---|---|---|---|"
    print(head)
    print(sep)
    f1_sum = {l: [] for l in labels}
    for s in scenes:
        for lab in labels:
            ev = load(os.path.join(root, s, f"eval_{lab}", "evaluation.json"))
            # metrics come from out_<backend>; cuda contributors ship metrics too
            mt = load(os.path.join(root, s, f"out_{lab}", "metrics.json"))
            f1 = ev.get("f1")
            if isinstance(f1, (int, float)):
                f1_sum[lab].append(f1)
            print(f"| {s} | {lab} | {fmt(f1)} | {fmt(ev.get('precision'))} "
                  f"| {fmt(ev.get('recall'))} | {fmt(mt.get('fused_points'),0)} "
                  f"| {fmt(mt.get('wall_s'),2)} |")
    for lab in labels:
        vals = f1_sum[lab]
        mean = sum(vals) / len(vals) if vals else None
        print(f"| **mean** | **{lab}** | **{fmt(mean)}** | | | | |")


if __name__ == "__main__":
    main()
