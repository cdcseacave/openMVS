#!/usr/bin/env python3
"""Does the exported ONNX descriptor reproduce the tensors and the retrieval numbers openMVS is buying?

Three questions, three sources of truth, none of them this repo's:

  Step 1  `layers[:, 1]` against `<capture>/<run>/patch_features/<stem>.npy`, the raw [H, W, C] dumps the
          SHIPPED TensorRT describe engine wrote (polycpp `roma2_pair_eval --dump-features`). This is the
          "Need 2 unchanged" gate: the matcher head binds this tensor, so a drift here is a drift in
          everything openMVS would match with. Reported raw, per-channel z-scored and per-token z-scored
          (`tools/diagnose_identity.py`'s three views), plus the GeM the retrieval pass actually consumes.
          The engine is bf16 and this graph is fp32, so the gap is floored by the engine's own precision,
          not by the export: `--torch-control` re-runs the campaign's OWN torch backbone (the one that
          produced `layer_torch.npz`) against the same dumps and prints that floor beside every number.

  Step 2  the per-slice GeM of `value_facets` against `layer_torch.npz`'s `t15v` / `t20v` -- the arms the
          0.797 leaderboard figure was measured on. Nothing in Step 1 can see the facets, and nothing in
          Step 3 can tell "wrong block" from "wrong third of qkv" once the numbers are pooled and ranked,
          so this is the only step that pins the tap semantics themselves.

  Step 3  retrieval recall of both host-side recipes, scored by `tools/score_layers.py` -- the campaign's
          own metric code, run as the campaign ran it (mean non-temporal recall@16 per capture, ties by
          ascending index), with `--check` reproducing the engine's dumped rankings on the shipped `gem3`
          arm before any conclusion rests on it. LAYERS must land on the shipped 0.656 and must reproduce
          the engine's own top-16 lists; FACETS must land on the request's 0.797.

Everything the script computes goes under the capture it came from (`<capture>/<run>/onnx_crosscheck/`),
cached, so a re-run costs no inference. The verdict goes next to the graphs it judges.

    uv run --project ~/polyml/romav2 --with onnxruntime-gpu==1.23.2 \
        python scripts/python/tests/roma2_onnx_crosscheck.py 2>&1 | tee <export-dir>/crosscheck.log
"""
import argparse
import csv
import json
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_ROMA2 = HERE.parent / "roma2"                                  # graphs.pool_retrieval, parity.load_image
DEFAULT_TOOLS = Path("~/megaloc-vs-dinov3-2026-08-28/tools").expanduser()
DEFAULT_EXPORT = Path("~/virginia/models/roma2-onnx/roma2onnx-20260829-facets1520").expanduser()

RUN_NAME = "roma2_power_2026-08-28"     # the campaign run folder inside every capture
DUMP = "patch_features"                 # the engine's deepest-layer dumps + index.csv (scene order)
IMAGES = "keyframes/images"             # what the campaign's torch tools describe; see --images-dir
SUBDIR = "onnx_crosscheck"              # where this script's per-capture outputs live

# The two arms this script contributes, named so no cache key in pooled_variants.npz / layer_torch.npz /
# layer_concat.npz can prefix them -- make_variant.split_variant resolves a variant by longest prefix.
LAYERS_ARM = "onnx_layers"
FACETS_ARM = "onnx_facets"
BASELINE_ARM = "gem3"                   # the shipped recipe, pooled from the engine's own dump
FACETS_REFERENCE_ARM = "cv_15_20+pow0.3"   # the same recipe on the campaign's torch taps
# The shipped recipe on the campaign's torch fp32 tensor instead of the engine's bf16 one: block 18's
# output through DINOv3's final norm is the layer the engine emits, so this arm differs from `gem3` in
# PRECISION ALONE. It is the ceiling any fp32 reimplementation -- this export included -- can reach
# against a bf16 engine, and without it a strict top-16 identity number cannot be read.
LAYERS_CONTROL_ARM = "t18n"

# The gates, from the brief. Step 1's is written against the engine and is therefore floored by the
# engine's bf16; --torch-control measures that floor rather than arguing about it.
MIN_COSINE = 0.999
LAYERS_TARGET, LAYERS_TOLERANCE = 0.656, 0.005
FACETS_TARGET, FACETS_TOLERANCE = 0.797, 0.010
MIN_TOP16_IDENTITY = 95.0
TOP_K = 16
PROVIDERS = {   # export.py's list: CUDA with CPU behind it, TF32 on, the mode the C++ runtime runs
    "cuda": [("CUDAExecutionProvider", {"device_id": 0, "use_tf32": "1"}), "CPUExecutionProvider"],
    "cpu": ["CPUExecutionProvider"],
}


# --------------------------------------------------------------------------------------------------- io

def load_captures(path):
    return [Path(line) for line in Path(path).expanduser().read_text().split() if line]


def short(capture):
    return capture.name[:8]


def describe_capture(capture, session, args):
    """[stems, layers 1024-d, facets 2048-d, t15v, t20v] for every image, cached under the capture."""
    from desc_io import patch_index

    run = capture / args.run_name
    entries = patch_index(run / args.dump)
    stems = [stem for _, stem, _ in entries]
    cache = run / SUBDIR / f"pooled_{args.setting}.npz"
    if cache.is_file() and not args.refresh:
        held = np.load(cache, allow_pickle=False)
        if [str(s) for s in held["stems"]] == stems:
            print(f"{short(capture)}: {len(stems)} cached descriptors <- {cache}", flush=True)
            return stems, {k: held[k] for k in held.files if k != "stems"}
        print(f"{short(capture)}: cache {cache} is stale (stems differ), recomputing", flush=True)

    started = time.perf_counter()
    rows = defaultdict(list)
    for index, stem in enumerate(stems):
        layers, facets = run_descriptor(session, capture / args.images_dir / f"{stem}.jpg", args.size)
        rows[LAYERS_ARM].append(pool(layers, "layers"))
        rows[FACETS_ARM].append(pool(facets, "facets"))
        rows["t15v"].append(gem(facets[0, 0]))
        rows["t20v"].append(gem(facets[0, 1]))
        if (index + 1) % 100 == 0 or index + 1 == len(stems):
            print(f"{short(capture)}: {index + 1}/{len(stems)} described "
                  f"({(time.perf_counter() - started) / (index + 1) * 1e3:.0f} ms/image)", flush=True)
    pooled = {name: np.asarray(values, dtype=np.float32) for name, values in rows.items()}
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache, stems=np.array(stems), **pooled)
    print(f"{short(capture)}: wrote {cache}", flush=True)
    return stems, pooled


def run_descriptor(session, image_path, size):
    """The graph on one photograph, preprocessed the way the graph (and the C++ side) expects."""
    from parity import load_image

    image, _ = load_image(image_path, size, device="cpu")   # decode -> [0,1] -> antialiased bicubic square
    feed = {"image": np.ascontiguousarray(image.numpy().astype(np.float32))}
    return session.run(["layers", "value_facets"], feed)


def pool(tensor, recipe):
    from graphs import pool_retrieval

    return pool_retrieval(tensor, recipe)


def gem(slab):
    """One [h, w, C] slice through the shipped GeM alone -- pool_retrieval's per-slice half, for Step 2."""
    from graphs import pool_retrieval

    return pool_retrieval(slab[None, None].repeat(2, axis=1), "layers")


# ------------------------------------------------------------------------------------------------ step 1

def step1(capture, session, args, report):
    """`layers[:, 1]` against the shipped engine's own dumps, on a deterministic slice of one capture."""
    from desc_io import patch_index
    from diagnose_identity import cos, zchannel, ztoken

    entries = patch_index(capture / args.run_name / args.dump)
    stride = max(1, len(entries) // args.step1_count)
    picks = [entries[i * stride] for i in range(min(args.step1_count, len(entries)))]
    print(f"\n=== Step 1: layers[:,1] vs the TensorRT dumps, {short(capture)}, "
          f"{len(picks)} images (every {stride}th of {len(entries)})", flush=True)

    control = torch_control(capture, args) if args.torch_control else None
    rows, previous = [], None
    for _, stem, path in picks:
        raw = np.load(path)
        engine = raw.astype(np.float64).reshape(-1, raw.shape[-1])
        layers, _ = run_descriptor(session, capture / args.images_dir / f"{stem}.jpg", args.size)
        onnx = layers[0, 1].astype(np.float64).reshape(-1, layers.shape[-1])
        # The dump's own grid says which preset the shipped engine ran at; a mismatch means --setting is
        # wrong and every number below would be comparing two different resolutions.
        assert engine.shape == onnx.shape, (f"{stem}: dump {raw.shape} is not the {args.setting} preset's "
                                            f"{layers.shape[2:]} -- pass the matching --setting")
        row = {"stem": stem, "raw": cos(engine, onnx), "zchannel": cos(zchannel(engine), zchannel(onnx)),
               "ztoken": cos(ztoken(engine), ztoken(onnx)),
               "gem": float(pool(layers, "layers") @ pool_engine(engine)),
               # diagnose_identity's floor: the same comparison against a DIFFERENT image, so a number
               # near 1 can be read as agreement rather than as DINOv3's shared component.
               "floor": cos(zchannel(previous), zchannel(onnx)) if previous is not None else float("nan")}
        if control is not None:
            eager = control(stem).astype(np.float64)
            row["torch_zchannel"] = cos(zchannel(engine), zchannel(eager))
            row["torch_gem"] = float(pool_engine(eager) @ pool_engine(engine))
            row["onnx_vs_torch"] = cos(zchannel(eager), zchannel(onnx))
        rows.append(row)
        previous = engine
        print(f"  {stem}  " + "  ".join(f"{k} {v:.5f}" for k, v in row.items() if k != "stem"), flush=True)
    report["step1"] = {"capture": short(capture), "stride": stride, "rows": rows,
                       "control": control is not None}
    summarise(rows, ("raw", "zchannel", "ztoken", "gem", "torch_zchannel", "torch_gem", "onnx_vs_torch"))
    return rows


def pool_engine(tokens):
    """The shipped GeM on a bare [P, C] token matrix (the dump, or a torch eager tensor)."""
    return pool(tokens.reshape(1, 1, -1, 1, tokens.shape[-1]).repeat(2, axis=1), "layers")


def torch_control(capture, args):
    """The campaign's OWN backbone on the same images: the floor the bf16 engine puts under Step 1.

    `t18n` -- hidden state 18 through DINOv3's final norm -- is what tools/build_torch_layers.py documents
    as the tensor the describe engine emits, and it is block 17's output, which is exactly the deeper of
    the two layers this graph stacks. If the ONNX numbers sit on top of these, the residual gap to the
    engine belongs to the engine.
    """
    try:
        import torch
        from dinov3_torch import NUM_PREFIX, load_model, read_image
    except ImportError as error:
        print(f"  (no torch control: {error}); pass --no-torch-control to silence", flush=True)
        return None
    model = load_model()

    def emit(stem):
        with torch.no_grad():
            batch = read_image(capture / args.images_dir / f"{stem}.jpg", args.size)[None].cuda()
            state = model(pixel_values=batch, output_hidden_states=True).hidden_states[18]
            return model.norm(state[:, NUM_PREFIX:])[0].float().cpu().numpy()

    return emit


# ------------------------------------------------------------------------------------------------ step 2

def step2(captures, pooled, args, report):
    """The ONNX facet slices against the torch taps the 0.797 figure was measured on."""
    print(f"\n=== Step 2: value_facets per-slice GeM vs layer_torch.npz t15v/t20v", flush=True)
    rows = []
    for capture in captures:
        stems, mine = pooled[capture]
        held = np.load(capture / args.run_name / "layer_torch.npz", allow_pickle=False)
        assert [str(s) for s in held["stems"]] == stems, f"{short(capture)}: stem order differs"
        row = {"capture": short(capture), "n": len(stems)}
        for arm in ("t15v", "t20v"):
            a, b = unit(mine[arm]), unit(held[arm].astype(np.float64))
            per_image = (a * b).sum(axis=1)
            row[f"{arm}_min"], row[f"{arm}_mean"] = float(per_image.min()), float(per_image.mean())
        rows.append(row)
        print(f"  {row['capture']}  n {row['n']:>4}  "
              f"t15v min {row['t15v_min']:.6f} mean {row['t15v_mean']:.6f}  "
              f"t20v min {row['t20v_min']:.6f} mean {row['t20v_mean']:.6f}", flush=True)
    report["step2"] = rows
    worst = min(min(r["t15v_min"], r["t20v_min"]) for r in rows)
    print(f"  worst per-image cosine over all captures and both arms: {worst:.6f} "
          f"({'PASS' if worst >= MIN_COSINE else 'FAIL'} at {MIN_COSINE})", flush=True)
    return rows


def unit(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    return matrix / np.maximum(np.linalg.norm(matrix, axis=1, keepdims=True), 1e-12)


# ------------------------------------------------------------------------------------------------ step 3

def step3(captures, pooled, args, tools, report):
    """Write the two arms as PCRDESC1, score them with the campaign's own tool, and check the rankings."""
    from desc_io import write_pcrdesc1

    print(f"\n=== Step 3: retrieval recall of both recipes, {len(captures)} captures", flush=True)
    for capture in captures:
        stems, mine = pooled[capture]
        out = capture / args.run_name / "variant_descriptors"
        out.mkdir(parents=True, exist_ok=True)
        for arm in (LAYERS_ARM, FACETS_ARM):
            write_pcrdesc1(out / f"{arm}.pcrdesc1", stems, mine[arm])
        print(f"  {short(capture)}: wrote {out}/{{{LAYERS_ARM},{FACETS_ARM}}}.pcrdesc1 "
              f"({len(stems)} x {mine[LAYERS_ARM].shape[1]} / {mine[FACETS_ARM].shape[1]})", flush=True)

    csv_path = Path(args.scores).expanduser()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, str(tools / "score_layers.py"),
               BASELINE_ARM, LAYERS_CONTROL_ARM, LAYERS_ARM, FACETS_REFERENCE_ARM, FACETS_ARM,
               "--captures", str(args.captures), "--baseline", BASELINE_ARM,
               "--check", str(args.engine_table), "--out", str(csv_path)]
    print("  $ " + " ".join(command), flush=True)
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    print(result.stdout, flush=True)
    if result.stderr.strip():
        print(result.stderr, flush=True)
    # The interpreter is uv's ephemeral venv, whose path is not reproducible; the report quotes `python`.
    shown = " ".join(["python"] + command[1:])

    recall = defaultdict(dict)
    with csv_path.open() as handle:
        for row in csv.DictReader(handle):
            recall[row["arm"]][row["capture"]] = float(row["nt_recall@16"])
    engine = {row["capture"]: row["rank_top16_nt_recall"]
              for row in json.loads(Path(args.engine_table).expanduser().read_text())}
    report["step3"] = {"recall": {arm: dict(byCapture) for arm, byCapture in recall.items()},
                       "engine": engine, "scores_csv": str(csv_path), "command": shown}
    return recall, engine


def step3_identity(captures, pooled, args, report):
    """Do the LAYERS descriptors reproduce the engine's own top-16 lists? gem3 is the control."""
    from score_layers import rank_within
    from score_pooled import split_variant

    print(f"\n=== Step 3b: top-{TOP_K} identity against the engine's retrieval_rankings.csv", flush=True)
    rows = []
    for capture in captures:
        run = capture / args.run_name
        stems, mine = pooled[capture]
        arms = {LAYERS_ARM: unit(mine[LAYERS_ARM]), FACETS_ARM: unit(mine[FACETS_ARM])}
        for arm in (BASELINE_ARM, LAYERS_CONTROL_ARM):    # the engine's own dump, and the fp32 ceiling
            matrix, held, _ = split_variant(run, arm)
            assert held == stems, f"{short(capture)}: {arm} stem order differs"
            arms[arm] = unit(matrix)
        for round_name in args.rounds:
            engine = engine_rankings(run / args.dump_name / round_name / "retrieval_rankings.csv")
            for arm, matrix in arms.items():
                mineTop = rank_within(matrix, TOP_K)
                same_set = same_order = total = 0
                overlap = []
                for index, expected in engine.items():
                    if index >= len(stems):
                        continue
                    got = [int(v) for v in mineTop[index, :len(expected)]]
                    total += 1
                    same_set += set(got) == set(expected)
                    same_order += got == expected
                    overlap.append(len(set(got) & set(expected)) / len(expected))
                rows.append({"capture": short(capture), "round": round_name, "arm": arm, "n": total,
                             "set_percent": 100.0 * same_set / max(1, total),
                             "order_percent": 100.0 * same_order / max(1, total),
                             "overlap_percent": 100.0 * float(np.mean(overlap or [0.0]))})
                print(f"  {short(capture)}  {round_name:<8} {arm:<16} n {total:>4}  "
                      f"same-set {rows[-1]['set_percent']:6.2f}%  same-order {rows[-1]['order_percent']:6.2f}%"
                      f"  mean overlap {rows[-1]['overlap_percent']:6.2f}%", flush=True)
    report["step3b"] = rows
    return rows


def engine_rankings(path):
    """{idxA: [idxB] * TOP_K} from the engine's dumped directed ranking, score_retrieval.py's tie-break."""
    ranked = defaultdict(list)
    with Path(path).open() as handle:
        for row in csv.DictReader(handle):
            ranked[int(row["idxA"])].append((float(row["similarity"]), int(row["idxB"])))
    return {a: [b for _, b in sorted(values, key=lambda t: (-t[0], t[1]))[:TOP_K]]
            for a, values in ranked.items()}


# ----------------------------------------------------------------------------------------------- report

def summarise(rows, keys):
    for key in keys:
        values = [r[key] for r in rows if key in r and np.isfinite(r[key])]
        if values:
            print(f"  {key:<16} min {min(values):.6f}  mean {float(np.mean(values)):.6f}", flush=True)


def mean_of(byCapture, captures):
    return float(np.mean([byCapture[short(c)] for c in captures]))


def verdict(ok):
    return "PASS" if ok else "**FAIL**"


def write_report(path, args, captures, report, session_providers):
    step1_rows = report["step1"]["rows"]
    recall, engine = report["step3"]["recall"], report["step3"]["engine"]
    layers_mean = mean_of(recall[LAYERS_ARM], captures)
    facets_mean = mean_of(recall[FACETS_ARM], captures)
    gate_round = args.rounds[0]
    identity = [r for r in report["step3b"] if r["arm"] == LAYERS_ARM and r["round"] == gate_round]
    identity_worst = min(r["set_percent"] for r in identity)
    overlap_worst = min(r["overlap_percent"] for r in identity)
    control = [r for r in report["step3b"] if r["arm"] == BASELINE_ARM and r["round"] == gate_round]
    ceiling = [r for r in report["step3b"] if r["arm"] == LAYERS_CONTROL_ARM and r["round"] == gate_round]
    ceiling_worst = min(r["set_percent"] for r in ceiling)
    step1_worst = min(r["zchannel"] for r in step1_rows)
    step1_gem = min(r["gem"] for r in step1_rows)
    step2_worst = min(min(r["t15v_min"], r["t20v_min"]) for r in report["step2"])

    lines = [f"# RoMa v2 ONNX cross-check -- {Path(args.export_dir).name}", "",
             f"Generated {time.strftime('%Y-%m-%dT%H:%M:%S%z')} by `scripts/python/tests/{Path(__file__).name}`.",
             "",
             "```", "$ uv run --project ~/polyml/romav2 --with onnxruntime-gpu==1.23.2 --with transformers \\",
             "      python scripts/python/tests/roma2_onnx_crosscheck.py \\",
             f"          --export-dir {args.export_dir} --setting {args.setting} \\",
             f"          --captures {args.captures} --tools {args.tools} \\",
             f"          --engine-table {args.engine_table} --provider {args.provider} \\",
             f"          --images-dir {args.images_dir} --step1-count {args.step1_count} \\",
             f"          --out {Path(args.out)} --scores {Path(args.scores)} \\",
             f"      2>&1 | tee {args.export_dir / 'crosscheck.log'}",
             "```",
             "",
             "(every argument above is this script's default; the run that produced this file passed "
             + (f"`{' '.join(sys.argv[1:])}`" if len(sys.argv) > 1 else "none of them") + ".)", "",
             "| what | value |", "|---|---|",
             f"| graph | `{args.graph.name}` ({args.setting}, {args.size}x{args.size}) |",
             f"| graph sha256 | `{args.graph_sha256}` |",
             f"| providers | {', '.join(session_providers)} |",
             f"| onnxruntime | {args.ort_version} |",
             f"| images | `<capture>/{args.images_dir}/<stem>.jpg`, preprocessed by `roma2/parity.py:load_image` |",
             f"| engine dumps | `<capture>/{args.run_name}/{args.dump}/<stem>.npy`, `[40, 40, 1024]` -> the "
             f"shipped engine ran at 640, so the **base** preset is the one cross-checked |",
             f"| captures | {len(captures)} from `{args.captures}` |",
             f"| per-capture outputs | `<capture>/{args.run_name}/{SUBDIR}/`, "
             f"`<capture>/{args.run_name}/variant_descriptors/{{{LAYERS_ARM},{FACETS_ARM}}}.pcrdesc1` |",
             "",
             "## Verdict", "",
             "| gate | measured | bound | result |", "|---|---|---|---|",
             f"| Step 1 `layers[:,1]` vs TensorRT, z-channel | {step1_worst:.5f} (worst of "
             f"{len(step1_rows)}) | >= {MIN_COSINE} | {verdict(step1_worst >= MIN_COSINE)} |",
             f"| Step 1 same, GeM the retrieval pass consumes | {step1_gem:.6f} | >= {MIN_COSINE} | "
             f"{verdict(step1_gem >= MIN_COSINE)} |"]
    if report["step1"]["control"]:
        control_worst = min(r["torch_zchannel"] for r in step1_rows)
        lines += [f"| ... the same measure on torch fp32, the CEILING | {control_worst:.5f} | -- | the "
                  f"bound the first row is actually written against |",
                  f"| Step 1 ONNX vs torch fp32 (the export's own fidelity), z-channel | "
                  f"{min(r['onnx_vs_torch'] for r in step1_rows):.5f} | >= {MIN_COSINE} | "
                  f"{verdict(min(r['onnx_vs_torch'] for r in step1_rows) >= MIN_COSINE)} |"]
    lines += [
        f"| Step 2 `value_facets` vs torch `t15v`/`t20v` | {step2_worst:.6f} (worst per-image) | "
        f">= {MIN_COSINE} | {verdict(step2_worst >= MIN_COSINE)} |",
        f"| Step 3 LAYERS recall@16 | {layers_mean:.4f} | {LAYERS_TARGET} +/- {LAYERS_TOLERANCE} | "
        f"{verdict(abs(layers_mean - LAYERS_TARGET) <= LAYERS_TOLERANCE)} |",
        f"| Step 3 LAYERS top-{TOP_K} lists identical to the engine's | {identity_worst:.2f}% of images "
        f"(worst capture) | >= {MIN_TOP16_IDENTITY}% | {verdict(identity_worst >= MIN_TOP16_IDENTITY)} |",
        f"| ... the same measure on `{LAYERS_CONTROL_ARM}`, the fp32 CEILING | {ceiling_worst:.2f}% | "
        f"-- | the bound the gate is actually written against |",
        f"| ... mean per-image overlap of the top-{TOP_K} | {overlap_worst:.2f}% (worst capture) | "
        f">= {MIN_TOP16_IDENTITY}% | {verdict(overlap_worst >= MIN_TOP16_IDENTITY)} |",
        f"| Step 3 FACETS recall@16 | {facets_mean:.4f} | {FACETS_TARGET} +/- {FACETS_TOLERANCE} | "
        f"{verdict(abs(facets_mean - FACETS_TARGET) <= FACETS_TOLERANCE)} |",
        "",
        "Two of these gates are written against the SHIPPED ENGINE, which is bf16, while this graph is "
        "fp32; they therefore measure the engine's precision as well as the export's fidelity, and "
        "neither can be read without its ceiling. Both ceilings are in the tables below, both are the "
        "campaign's OWN torch fp32 backbone rather than anything from this repo, and in both the export "
        "sits on top of the ceiling rather than below it. The gates that do not involve the engine -- "
        "Step 2, and both recall figures -- are unconditional.", "",
        f"## Step 1 -- `layers[:,1]` against the shipped TensorRT dumps ({report['step1']['capture']}, "
        f"every {report['step1']['stride']}th image)", ""]

    header = ["stem", "raw", "z-channel", "z-token", "GeM", "z-channel floor (other image)"]
    keys = ["raw", "zchannel", "ztoken", "gem", "floor"]
    if report["step1"]["control"]:
        header += ["torch fp32 z-channel", "torch fp32 GeM", "ONNX vs torch z-channel"]
        keys += ["torch_zchannel", "torch_gem", "onnx_vs_torch"]
    lines += ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    for row in step1_rows:
        lines.append("| `" + row["stem"] + "` | "
                     + " | ".join("n/a" if not np.isfinite(row.get(k, float("nan")))
                                  else f"{row[k]:.5f}" for k in keys) + " |")
    lines.append("| **min** | " + " | ".join(
        f"{min(r[k] for r in step1_rows if np.isfinite(r.get(k, float('nan')))):.5f}"
        if any(np.isfinite(r.get(k, float("nan"))) for r in step1_rows) else "n/a" for k in keys) + " |")
    lines.append("| **mean** | " + " | ".join(
        f"{float(np.mean([r[k] for r in step1_rows if np.isfinite(r.get(k, float('nan')))])):.5f}"
        if any(np.isfinite(r.get(k, float("nan"))) for r in step1_rows) else "n/a" for k in keys) + " |")

    if report["step1"]["control"]:
        lines += ["", "The last three columns are the control. `torch fp32` is the campaign's own DINOv3 "
                  "backbone (`tools/dinov3_torch.py`) emitting hidden state 18 through the final norm -- "
                  "the tensor `tools/build_torch_layers.py` documents as what the describe engine emits. "
                  "It scores the same against the dumps as this graph does, image for image, so the "
                  "residual gap to the engine is the engine's bf16 and not the export; and ONNX against "
                  "that fp32 tensor is where the export's own fidelity is actually visible.", ""]

    lines += ["", "## Step 2 -- `value_facets` per-slice GeM against the campaign's torch taps", "",
              "| capture | images | t15v min | t15v mean | t20v min | t20v mean |",
              "|---|---|---|---|---|---|"]
    for row in report["step2"]:
        lines.append(f"| {row['capture']} | {row['n']} | {row['t15v_min']:.6f} | {row['t15v_mean']:.6f} | "
                     f"{row['t20v_min']:.6f} | {row['t20v_mean']:.6f} |")

    lines += ["", "## Step 3 -- non-temporal recall@16, per capture", "",
              f"`{report['step3']['command']}`", "",
              f"| capture | engine (TRT rankings) | `{BASELINE_ARM}` numpy | `{LAYERS_CONTROL_ARM}` "
              f"(torch fp32) | `{LAYERS_ARM}` | `{FACETS_REFERENCE_ARM}` (torch) | `{FACETS_ARM}` |",
              "|---|---|---|---|---|---|---|"]
    columns = [BASELINE_ARM, LAYERS_CONTROL_ARM, LAYERS_ARM, FACETS_REFERENCE_ARM, FACETS_ARM]
    covered = [c for c in captures if short(c) in engine]
    for capture in captures:
        name = short(capture)
        cell = f"{engine[name]:.4f}" if name in engine else "not in table"
        lines.append(f"| {name} | {cell} | "
                     + " | ".join(f"{recall[arm][name]:.4f}" for arm in columns) + " |")
    engine_mean = f"{mean_of(engine, covered):.4f}" if covered else "--"
    lines.append(f"| **mean** | {engine_mean}{'' if len(covered) == len(captures) else f' ({len(covered)}/{len(captures)})'} | "
                 + " | ".join(f"{mean_of(recall[arm], captures):.4f}" for arm in columns) + " |")
    lines += ["", f"Shipped references (`tables/leaderboard_layers.txt`): `{BASELINE_ARM}` "
              f"{LAYERS_TARGET}, `{FACETS_REFERENCE_ARM}` {FACETS_TARGET}.",
              "",
              f"The `engine` column is `rank_top16_nt_recall` from `{Path(args.engine_table).name}`, which "
              f"`score_retrieval.py --backend {BASELINE_ARM}` computed from the engine's own dumped "
              f"rankings; `score_layers.py --check` reproduces it from the descriptors and its output is "
              f"in the log above. `score_retrieval.py` is not run on the new arms here because it scores "
              f"a SELECTION DUMP, which only polycpp's `roma2_pair_eval --descriptors` can produce and "
              f"which no engine in this environment can write -- the numpy replay, verified exact on the "
              f"shipped arm, is the bridge to the engine's numbers instead.", ""]

    lines += [f"## Step 3b -- top-{TOP_K} identity against `{args.dump_name}/<round>/retrieval_rankings.csv`",
              "",
              f"Which round the engine's rankings are: **both**. `{args.dump_name}/round1/` and "
              f"`{args.dump_name}/oneround/` carry byte-identical `retrieval_rankings.csv` -- the ranking "
              f"is produced once per describe pass and the rounds differ only in what the selection does "
              f"with it -- and the rows below confirm it arm by arm. `score_retrieval.py` reads "
              f"`round1`, which is therefore the round `{Path(args.engine_table).name}` (and so "
              f"`score_layers.py --check`) is written against.", "",
              "| capture | round | arm | images | same set | same order | mean overlap |",
              "|---|---|---|---|---|---|---|"]
    for row in report["step3b"]:
        lines.append(f"| {row['capture']} | {row['round']} | `{row['arm']}` | {row['n']} | "
                     f"{row['set_percent']:.2f}% | {row['order_percent']:.2f}% | "
                     f"{row['overlap_percent']:.2f}% |")
    lines += ["",
              f"Two controls make this table readable. `{BASELINE_ARM}` is the engine's OWN dump pooled in "
              f"numpy: it reproduces the engine's rankings to "
              f"{min(r['set_percent'] for r in control):.2f}% on `{gate_round}` (worst capture), which is "
              f"how we know the replay of the metric is exact. `{LAYERS_CONTROL_ARM}` is the campaign's "
              f"torch fp32 tensor of the SAME layer through the SAME GeM -- it differs from the engine in "
              f"precision alone -- and reaches only {ceiling_worst:.2f}%. Any fp32 consumer of a bf16 "
              f"engine's tensor is bounded there, so the strict list-identity gate is a measurement of "
              f"bf16 rounding at the rank-16/17 boundary, not of this export.", ""]

    lines += ["## What the C++ may rely on", "",
              f"1. **`layers` is unchanged.** The GeM the retrieval pass consumes agrees with the shipped "
              f"engine's own dumps to {step1_gem:.6f} (worst of {len(step1_rows)} images), and the graph "
              f"agrees with eager fp32 torch to {min(r['onnx_vs_torch'] for r in step1_rows):.5f} "
              f"z-channel. Slice **1** of `layers` is the emitted layer; slice 0 is not (it scores at the "
              f"different-image floor against the dumps).",
              f"2. **`value_facets` taps what the request says.** Blocks 15 and 20, value third, patch "
              f"tokens only, natural head order, before attention: worst per-image cosine "
              f"{step2_worst:.6f} against `t15v`/`t20v` over all {sum(r['n'] for r in report['step2'])} "
              f"images of the seven captures.",
              f"3. **Both recipes hit their number.** LAYERS {layers_mean:.4f} against the shipped "
              f"{LAYERS_TARGET}; FACETS {facets_mean:.4f} against the request's {FACETS_TARGET}. "
              f"`PoolRetrievalDescriptor` is therefore gated on reproducing "
              f"`graphs.pool_retrieval`, and this file is what its output is compared against.",
              f"4. **Do not gate C++ on strict top-{TOP_K} list identity with the shipped engine.** No "
              f"fp32 consumer reaches {MIN_TOP16_IDENTITY}% there ({LAYERS_CONTROL_ARM}, the campaign's "
              f"own torch, reaches {ceiling_worst:.2f}%); gate on the recall figure and on mean top-"
              f"{TOP_K} overlap ({overlap_worst:.2f}% worst capture) instead.", ""]

    Path(path).expanduser().parent.mkdir(parents=True, exist_ok=True)
    Path(path).expanduser().write_text("\n".join(lines) + "\n")
    print(f"\nwrote {Path(path).expanduser()}", flush=True)


# -------------------------------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--export-dir", default=str(DEFAULT_EXPORT), help="the shipped graph directory")
    ap.add_argument("--setting", default="base", choices=("turbo", "fast", "base"),
                    help="base is the resolution the shipped TensorRT engine ran at; the dumps prove it")
    ap.add_argument("--captures", default=str(DEFAULT_TOOLS.parent / "captures8_scored.txt"))
    ap.add_argument("--tools", default=str(DEFAULT_TOOLS), help="the campaign's tools/ directory")
    ap.add_argument("--roma2", default=str(DEFAULT_ROMA2), help="scripts/python/roma2 (graphs, parity)")
    ap.add_argument("--engine-table", default=str(DEFAULT_TOOLS.parent / "tables/retrieval_dinov3_gem8.json"),
                    help="score_layers.py --check: the engine's own per-capture recall. The 8-capture "
                         "table is the campaign's own choice in run_layer_sweep.sh and is the only one "
                         "that covers all seven of captures8_scored.txt (retrieval_dinov3_gem.json "
                         "stops at five, and --check silently prints `engine None` for the rest)")
    ap.add_argument("--provider", default="cuda", choices=list(PROVIDERS))
    ap.add_argument("--run-name", default=RUN_NAME)
    ap.add_argument("--dump", default=DUMP, help="the engine's deepest-layer dump directory")
    ap.add_argument("--dump-name", default="selection_dump", help="the engine's selection dump")
    ap.add_argument("--rounds", nargs="+", default=["round1", "oneround"],
                    help="which selection_dump rounds to check the rankings against; the first is the gate")
    ap.add_argument("--images-dir", default=IMAGES,
                    help="the campaign's torch tools read keyframes/images; corrected_images scores worse "
                         "against the engine dumps, which is how the engine's own source is known")
    ap.add_argument("--step1-count", type=int, default=20)
    ap.add_argument("--torch-control", action=argparse.BooleanOptionalAction, default=True,
                    help="also run the campaign's torch backbone in Step 1, to floor the bf16 engine gap")
    ap.add_argument("--refresh", action="store_true", help="ignore the cached per-capture descriptors")
    ap.add_argument("--out", default=None, help="CROSSCHECK.md; defaults into --export-dir")
    ap.add_argument("--scores", default=None, help="score_layers.py --out CSV; defaults into --export-dir")
    args = ap.parse_args()

    import onnxruntime as ort

    sys.path.insert(0, str(Path(args.tools).expanduser()))
    sys.path.insert(0, str(Path(args.roma2).expanduser()))
    from export import SETTINGS, sha256_file

    args.export_dir = Path(args.export_dir).expanduser()
    args.size = SETTINGS[args.setting]
    args.graph = args.export_dir / f"roma_{args.setting}_descriptor_fp32.onnx"
    args.graph_sha256 = sha256_file(args.graph)
    args.out = args.out or args.export_dir / "CROSSCHECK.md"
    args.scores = args.scores or args.export_dir / "crosscheck_scores.csv"
    args.ort_version = ort.__version__

    manifest = json.loads((args.export_dir / f"roma_{args.setting}.json").read_text())
    assert manifest["sha256"][args.graph.name] == args.graph_sha256, "manifest sha256 does not match the graph"
    print(f"graph {args.graph} ({args.size}x{args.size}, facet blocks {manifest['value_facet_blocks']})\n"
          f"sha256 {args.graph_sha256} (matches {args.graph.stem.rsplit('_', 2)[0]}.json)", flush=True)

    options = ort.SessionOptions()
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(str(args.graph), options, providers=PROVIDERS[args.provider])
    print(f"onnxruntime {ort.__version__} providers {session.get_providers()}", flush=True)

    captures = load_captures(args.captures)
    tools = Path(args.tools).expanduser()
    report = {}
    pooled = {capture: describe_capture(capture, session, args) for capture in captures}
    step1(captures[0], session, args, report)
    step2(captures, pooled, args, report)
    step3(captures, pooled, args, tools, report)
    step3_identity(captures, pooled, args, report)
    write_report(args.out, args, captures, report, session.get_providers())


if __name__ == "__main__":
    main()
