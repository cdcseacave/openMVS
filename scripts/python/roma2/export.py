#!/usr/bin/env python3
"""Export RoMa v2 (https://github.com/Parskatt/RoMaV2) to ONNX for OpenMVS's in-process ROMA2 matcher.

Two static fp32 graphs per preset (spec: megaloc-vs-dinov3-2026-08-28/EXPORT_REQUEST.md; layout: polyml
romav2/{graphs,export}.py):
  roma_<setting>_descriptor_fp32.onnx    image[1,3,S,S] (RGB planar, [0,1]) -> layers[1,2,S/16,S/16,1024]
                                         (norm(block 11), norm(block 17)),
                                         value_facets[1,2,S/16,S/16,1024] (v_proj of blocks 15 and 20,
                                         patch tokens, before attention)
  roma_<setting>_match_coarse_fp32.onnx  (descriptors_A, descriptors_B, img_A, img_B)
                                         -> warp[1,S/4,S/4,2], confidence[1,S/4,S/4,1]

`S` is the square input resolution --setting traces for: turbo 320, fast 512, base 640. The graphs are fp32,
static shape, batch 1; precision is a property of the graph, and the runtime decides the rest (ORT runs the
fp32 graph with TF32 on CUDA).

Two stages, because the DINOv3 descriptor is per image while matching is per pair: a scene matches each
image against several others, so exporting the descriptor separately lets the caller run it once per image
and reuse the result.

Three steps, and none of them optional. `onnx` traces a stage; `check` runs the traced graph under
onnxruntime and judges it against the eager fp32 model — on cosine for the descriptor, on where the pair
graph claims a correspondence for the matcher (see check_correspondences); `manifest` writes the
roma_<setting>.json the C++ loader binds against, with a sha256 per file. Those checksums are
provenance -- `export.py check` and whoever copies a model directory around verify them; the C++
loader (`RoMa2Manifest::Load`) never reads them, so a truncated .onnx.data still surfaces at scene
time as an opaque ONNX Runtime load failure, and `export.py check` is what attributes it.

Run inside polyml's export project env:
  cd ~/polyml/romav2 && uv run --with 'onnxruntime-gpu==1.23.2' \\
      python ~/openMVS/scripts/python/roma2/export.py onnx --stage descriptor --setting base --out-dir <dir>
  ... --with 'onnxruntime-gpu==1.23.2' python .../export.py check --onnx <dir>/roma_base_descriptor_fp32.onnx
  ... python .../export.py manifest --out-dir <dir> --setting base

export.sh runs the whole sequence for the three presets, which is what a shipped model directory is.
"""
import argparse
import hashlib
import json
import re
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

SETTINGS = {"turbo": 320, "fast": 512, "base": 640}   # model.H_lr after apply_setting

DEFAULT_CHECKPOINT = "~/.cache/torch/hub/checkpoints/romav2.0.1.pt"   # == Polycam/romav2 mirror's romav2.0.1.pt
DEFAULT_ROMA2_REPO = "~/polyml/romav2"                # the vendored RoMaV2; ~/RoMaV2 (upstream) also works

STAGE_IO = {
    "descriptor": (["image"], ["layers", "value_facets"]),
    "match": (["descriptors_A", "descriptors_B", "img_A", "img_B"], ["warp", "confidence"]),
}

FORMAT_VERSION = 1    # roma_<setting>.json's schema version, read by RoMa2Manifest::Load

WARMUP_RUNS = 10      # discarded before timing: the first executions carry allocation and clock ramp

# What the manifest publishes about the two retrieval recipes graphs.pool_retrieval implements and the C++
# PoolRetrievalDescriptor reproduces, so that neither side carries a constant the other could change under
# it. The dimensions are not here: they are read off the graph, which cannot be wrong about its own width.
GEM_P = 3             # the GeM exponent, pool_retrieval's .pow(3) ... .pow(1 / 3)
FACETS_POWER = 0.3    # pool_retrieval's default power, the request's sign(d)|d|^0.3
LAYERS_SLICE = 1      # the layer the shipped GeM recipe pools: slice 1 of layers, i.e. block 17

# CUDA first with CPU behind it, which is what the C++ runtime does; TF32 on, which is ORT's own default and
# the mode the shipped graphs are measured in. A graph that only agrees with eager in fp32 would be a graph
# openMVS cannot ship, so the check is run in the mode the consumer runs.
PROVIDERS = {
    "cuda": [("CUDAExecutionProvider", {"device_id": 0, "use_tf32": "1"}), "CPUExecutionProvider"],
    "cpu": ["CPUExecutionProvider"],
}


def onnx_path(out_dir, setting, stage, coarse, precision):
    """roma_<setting>_<stage>[_coarse]_<precision>.onnx — no architecture: the graph is portable input."""
    parts = ["roma", setting, stage] + (["coarse"] if coarse else []) + [precision]
    return Path(out_dir).expanduser().resolve() / ("_".join(parts) + ".onnx")


def reference_dir_for_onnx(onnx_file):
    return Path(onnx_file).with_suffix(".reference")


def sidecar_for_onnx(onnx_file):
    """<graph>.export.json — how this graph was traced, written next to it by `onnx`.

    A traced graph does not record the flags that produced it: which blocks the facets were tapped from,
    whether the RoPE cache was on, which checkpoint and which checkout. `manifest` publishes all of that,
    and without a sidecar it would be publishing its own argv — a manifest that says 15,20 because that is
    the default, describing a graph traced with 12,18. So the tracer writes what it did, and the manifest
    reads it back."""
    return Path(onnx_file).with_suffix(".export.json")


def setting_from_onnx(onnx_file):
    """The preset a graph was traced for, read back from its own name."""
    setting = Path(onnx_file).stem.split("_")[1]
    if setting not in SETTINGS:
        raise SystemExit(f"cannot tell the setting from {Path(onnx_file).name}: expected roma_<setting>_...")
    return setting


def graph_size(path):
    """An ONNX plus the external data file its weights live in."""
    path = Path(path)
    return sum(sibling.stat().st_size for sibling in path.parent.glob(path.name + "*"))


def block_list(spec):
    """argparse type for --value-facet-blocks: "15,20" -> [15, 20]."""
    return [int(block) for block in spec.split(",")]


def sha256_file(path):
    """The digest the manifest publishes per file; streamed, because .onnx.data is gigabytes."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def package_version(name):
    """The installed version of a build dependency, without importing it."""
    from importlib import metadata
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return None


def export_onnx(args):
    from graphs import FACET_BLOCKS, REFINER_NOT_EXPORTED, trace_to_onnx   # torch only enters here

    if args.with_refiner or (args.stage == "match" and not args.coarse):
        raise NotImplementedError(REFINER_NOT_EXPORTED)
    inputs, outputs = STAGE_IO[args.stage]
    out = onnx_path(args.out_dir, args.setting, args.stage, args.coarse, "fp32")
    out.parent.mkdir(parents=True, exist_ok=True)
    reference = reference_dir_for_onnx(out)
    produced = trace_to_onnx(out, reference, args.stage, args.coarse, args.setting, args.checkpoint,
                             args.roma2_repo, inputs, outputs, args.exporter,
                             facet_blocks=args.value_facet_blocks or FACET_BLOCKS,
                             rope_cache=args.rope_cache)
    sidecar, recorded = sidecar_for_onnx(out), {
        "format_version": FORMAT_VERSION,
        "stage": args.stage,
        "coarse": bool(args.coarse),
        "setting": args.setting,
        "precision": "fp32",
        "exporter": args.exporter,
        # meaningless for the pair stage, which taps nothing: recorded as null rather than as a default
        "value_facet_blocks": (list(args.value_facet_blocks or FACET_BLOCKS)
                               if args.stage == "descriptor" else None),
        "rope_cache": bool(args.rope_cache),
        "checkpoint_sha256": sha256_file(Path(args.checkpoint).expanduser()),
        "romav2_commit": git_commit(Path(args.roma2_repo).expanduser()),
        "dinov3_hub_commit": dinov3_hub_commit(args.roma2_repo),
        "exported_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    sidecar.write_text(json.dumps(recorded, indent=2) + "\n")
    print(f"wrote {out} ({graph_size(out) / 1048576:.1f} MB including weights)", flush=True)
    print(f"wrote {reference}/ ({len(inputs)} inputs, {produced} eager fp32 outputs)", flush=True)
    print(f"wrote {sidecar.name} (exporter {recorded['exporter']}, facet blocks "
          f"{recorded['value_facet_blocks']}, rope_cache {recorded['rope_cache']}, checkpoint "
          f"{recorded['checkpoint_sha256'][:12]}, romav2 {recorded['romav2_commit']})", flush=True)


def check_onnx(args):
    """Run the graph under onnxruntime on its reference inputs and compare against the eager fp32 outputs.

    polyml export.py:274-357 with an ORT session in place of the TensorRT execution context, and the same
    judging: a runtime is not shippable until its numbers have been looked at, because a graph that loads
    and computes nothing looks exactly like one that works until something reads its output.

    Two references exist for every graph. The one `onnx` writes beside it is from the random tensors it
    traced with — enough to catch a graph computing nothing, and all the descriptor needs. The pair stage
    is also checked against a photographic reference (parity.py), because a warp has structure to get wrong
    that random inputs do not contain.
    """
    import onnxruntime as ort

    reference = Path(args.reference) if args.reference else reference_dir_for_onnx(args.onnx)
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(args.onnx, so, providers=PROVIDERS[args.provider])
    print(f"providers: {session.get_providers()}", flush=True)
    feeds = {i.name: np.ascontiguousarray(np.load(reference / f"in_{i.name}.npy").astype(np.float32))
             for i in session.get_inputs()}
    names = [o.name for o in session.get_outputs()]
    values = session.run(names, feeds)

    if args.repeat:
        elapsed = []
        for iteration in range(args.repeat + WARMUP_RUNS):
            started = time.perf_counter()
            session.run(names, feeds)
            if iteration >= WARMUP_RUNS:
                elapsed.append((time.perf_counter() - started) * 1000)
        elapsed.sort()
        print(f"latency over {args.repeat} runs: median {elapsed[len(elapsed) // 2]:.2f} ms  "
              f"min {elapsed[0]:.2f} ms  p99 {elapsed[int(0.99 * (len(elapsed) - 1))]:.2f} ms", flush=True)

    produced, worst_cosine = {}, 1.0
    for name, host in zip(names, values):
        produced[name] = host
        expected = np.load(reference / f"out_{name}.npy").astype(np.float64)
        actual = host.astype(np.float64)
        cosine = float(expected.ravel() @ actual.ravel()
                       / (np.linalg.norm(expected) * np.linalg.norm(actual)))
        error = np.abs(expected - actual)
        finite = float(np.isfinite(host).mean())
        print(f"{name}: cosine {cosine:.6f}  max abs err {error.max():.4f}  "
              f"p99 {np.percentile(error, 99):.4f}  finite {100 * finite:.2f}%", flush=True)
        worst_cosine = min(worst_cosine, cosine if finite == 1.0 else 0.0)

    if args.noise_floor and "image" in feeds:
        from graphs import bf16_noise_floor   # torch, and the weights: only on demand

        cosine, max_abs = bf16_noise_floor(setting_from_onnx(args.onnx), args.checkpoint, args.roma2_repo,
                                           feeds["image"], np.load(reference / "out_layers.npy"))
        print(f"noise floor (the model's own bf16 autocast vs eager fp32) layers: cosine {cosine:.6f}  "
              f"max abs {max_abs:.4f}", flush=True)

    if set(STAGE_IO["match"][1]) <= set(produced):
        check_correspondences(reference, produced, args)
    elif worst_cosine < args.min_cosine:
        raise SystemExit(f"FAILED: worst cosine {worst_cosine:.6f} is below --min-cosine {args.min_cosine}")
    else:
        # The descriptor stage is judged a second time on the pooled retrieval descriptors, because those
        # are what openMVS consumes: GeM cubes its input and then takes a mean over the whole grid, so a
        # per-element cosine of 0.9999 says less about the 2048-d vector than measuring it does.
        from graphs import pool_retrieval

        for recipe, output in (("facets", "value_facets"), ("layers", "layers")):
            if output not in produced:
                # A layers-only descriptor graph (Polycam's) is a legitimate file, but not one openMVS's
                # retrieval pass can be judged on, and the caller has to be told which half is missing.
                raise SystemExit(f"FAILED: this graph has no '{output}' output, so the pooled {recipe} "
                                 f"descriptor cannot be judged; it emits {sorted(produced)}")
            got = pool_retrieval(produced[output], recipe)
            want = pool_retrieval(np.load(reference / f"out_{output}.npy"), recipe)
            cosine = float(got @ want)
            print(f"pooled {recipe}: cosine {cosine:.6f}  max abs err {np.abs(got - want).max():.2e}",
                  flush=True)
            if cosine < args.min_cosine:
                raise SystemExit(f"FAILED: pooled {recipe} descriptor cosine {cosine:.6f} is below "
                                 f"--min-cosine {args.min_cosine}")
        print(f"OK: worst cosine {worst_cosine:.6f}", flush=True)


def check_correspondences(reference, produced, args):
    """Judge a pair graph on where it claims a correspondence, not on a cosine over everything it emits.

    A cosine over the whole output weights a cell in a non-overlapping region like a confident match, and
    roughly a quarter of the grid is the former, where the warp is unconstrained and diverges by tens of
    pixels in any implementation. It also mixes the overlap logit with the precision entries beside it,
    which are squares of network outputs and swamp it. Neither says anything about matching quality, and
    ranking runtimes by it inverts the order that pixel agreement gives.

    So: how far the warp moved where the model says there is a match, and how often the two disagree that
    there is one at all.
    """
    # Before anything is measured. A NaN anywhere makes the error NaN, every percentile of it NaN, and
    # every comparison against a bound False — so a graph computing nothing would pass the check that
    # exists to catch exactly that, and print OK doing it.
    for name, values in produced.items():
        finite = float(np.isfinite(values).mean())
        if finite != 1.0:
            raise SystemExit(f"FAILED: {name} is {100 * (1 - finite):.2f}% non-finite")

    warp_expected = np.load(reference / "out_warp.npy").astype(np.float64)[0]
    confidence_expected = np.load(reference / "out_confidence.npy").astype(np.float64)[0]
    side = warp_expected.shape[0]
    overlap = 1.0 / (1.0 + np.exp(-confidence_expected[..., 0]))
    matched = overlap >= 0.5
    if not matched.any():
        raise SystemExit("FAILED: the reference has no matched cells to judge against")

    # Normalized coordinates span the image, so a coordinate delta is (side / 2) pixels.
    error = np.abs(warp_expected - produced["warp"].astype(np.float64)[0]).max(axis=-1)[matched] * (side / 2)
    agreement = 100.0 * ((confidence_expected[..., 0] > 0)
                         == (produced["confidence"].astype(np.float64)[0][..., 0] > 0)).mean()
    p50, p99 = np.percentile(error, 50), np.percentile(error, 99)
    print(f"matched cells {100 * matched.mean():.1f}%  warp px p50 {p50:.4f}  p99 {p99:.3f}  "
          f"max {error.max():.2f}  decision agreement {agreement:.4f}%", flush=True)

    if p99 > args.max_warp_error:
        raise SystemExit(f"FAILED: warp p99 {p99:.3f} px exceeds --max-warp-error {args.max_warp_error}")
    if agreement < args.min_agreement:
        raise SystemExit(f"FAILED: decision agreement {agreement:.4f}% is below "
                         f"--min-agreement {args.min_agreement}")
    print(f"OK: warp p99 {p99:.3f} px, decision agreement {agreement:.4f}%", flush=True)


def graph_io(graph):
    """({input: dims}, {output: dims}) read off a traced graph. -1 for a dim the trace left dynamic."""
    def shape(value):
        return [d.dim_value if d.HasField("dim_value") else -1 for d in value.type.tensor_type.shape.dim]
    return ({v.name: shape(v) for v in graph.graph.input},
            {v.name: shape(v) for v in graph.graph.output})


def git_commit(repo):
    """The short HEAD of the checkout that provided the romav2 package, or None if it is not a checkout."""
    try:
        return subprocess.run(["git", "-C", str(repo), "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def dinov3_hub_commit(repo):
    """The DINOv3 hub revision romav2 pins, read from the source that pins it (features.py:104)."""
    source = (Path(repo).expanduser() / "src" / "romav2" / "features.py").read_text()
    found = re.search(r"facebookresearch/dinov3:([0-9a-f]{7,40})", source)
    return found.group(1) if found else None


def read_sidecar(onnx_file):
    """The <graph>.export.json beside a graph, or a hard failure naming what to do about it."""
    sidecar = sidecar_for_onnx(onnx_file)
    if not sidecar.is_file():
        raise SystemExit(f"missing {sidecar}: this graph carries no record of how it was traced, and a "
                         f"manifest may not invent one. Re-run `export.py onnx` for it, or write the "
                         f"sidecar from the export log if the graph predates it.")
    recorded = json.loads(sidecar.read_text())
    missing = [field for field in ("exporter", "precision", "value_facet_blocks", "rope_cache",
                                   "checkpoint_sha256", "romav2_commit", "dinov3_hub_commit",
                                   "exported_at") if field not in recorded]
    if missing:
        raise SystemExit(f"FAILED: {sidecar.name} is missing {missing}")
    return recorded


def agree_on(sidecars, fields):
    """The fields the two stages of one model directory must have been traced with identically.

    Two graphs beside each other are one model only if they came from the same weights and the same
    checkout. Silently publishing the descriptor's provenance for a pair graph built a week earlier from
    another checkpoint is exactly the mix a manifest exists to make impossible.
    """
    agreed = {}
    for field in fields:
        values = {key: sidecar[field] for key, sidecar in sidecars.items()}
        if len(set(map(repr, values.values()))) != 1:
            raise SystemExit(f"FAILED: the graphs in this directory disagree about {field}: {values}. "
                             f"They are not one export; re-trace them together.")
        agreed[field] = next(iter(values.values()))
    return agreed


def write_manifest(args):
    """Write roma_<setting>.json: what the C++ loader needs to bind these graphs, and to refuse others.

    Every shape in it is read back from the graphs themselves rather than recomputed from the setting, so
    the manifest cannot describe a file that is not there, and a graph traced at the wrong resolution is
    caught while the exporter still knows what it did. Every listed file is hashed for the same reason one
    layer down: an .onnx.data truncated in transit surfaces inside ORT as an opaque failure with nothing in
    it that points back at the copy.

    Provenance — exporter, facet blocks, RoPE cache, checkpoint, commits — comes from each graph's
    .export.json sidecar and from nowhere else, because this command's own defaults would otherwise
    describe whatever the last caller happened to type. The flags of the same name are assertions: they
    fail on a mismatch instead of overwriting one.
    """
    import onnx
    from graphs import LAYER_IDX, PATCH

    out_dir = Path(args.out_dir).expanduser().resolve()
    size = SETTINGS[args.setting]
    grid, cells = size // PATCH, size // 4

    files, io, sha, traced, sidecars = {}, {}, {}, {}, {}
    for key, stage, coarse in (("descriptor", "descriptor", False), ("match_coarse", "match", True)):
        path = onnx_path(out_dir, args.setting, stage, coarse, args.precision)
        data = path.with_name(path.name + ".data")
        for needed in (path, data):
            if not needed.is_file():
                raise SystemExit(f"missing {needed}: trace the {key} stage before writing the manifest")
        sidecars[key] = read_sidecar(path)
        traced[key] = onnx.load(str(path), load_external_data=False)
        inputs, outputs = graph_io(traced[key])
        io[key] = {"inputs": inputs, "outputs": outputs}
        files[key], files[f"{key}_data"] = path.name, data.name
        sha[path.name], sha[data.name] = sha256_file(path), sha256_file(data)

    width = io["descriptor"]["outputs"]["layers"][-1]
    expected = {
        "descriptor": ({"image": [1, 3, size, size]},
                       {"layers": [1, 2, grid, grid, width], "value_facets": [1, 2, grid, grid, width]}),
        "match_coarse": ({"descriptors_A": [1, 2, grid, grid, width],
                          "descriptors_B": [1, 2, grid, grid, width],
                          "img_A": [1, 3, size, size], "img_B": [1, 3, size, size]},
                         {"warp": [1, cells, cells, 2], "confidence": [1, cells, cells, 1]}),
    }
    for key, (inputs, outputs) in expected.items():
        for role, want in (("inputs", inputs), ("outputs", outputs)):
            if io[key][role] != want:
                raise SystemExit(f"FAILED: {files[key]} {role} {io[key][role]} are not the contract's {want}")

    # What the graphs say about their own tracing, cross-checked with each other and with anything the
    # caller asserted on the command line. A manifest is provenance: it may not invent any of this.
    provenance = agree_on(sidecars, ("exporter", "precision", "checkpoint_sha256", "romav2_commit",
                                     "dinov3_hub_commit", "rope_cache"))
    facet_blocks = sidecars["descriptor"]["value_facet_blocks"]
    asserted = {"exporter": args.exporter, "precision": args.precision}
    if args.value_facet_blocks:
        asserted["value_facet_blocks"] = args.value_facet_blocks
    if args.checkpoint:
        asserted["checkpoint_sha256"] = sha256_file(Path(args.checkpoint).expanduser())
    if args.roma2_repo:
        asserted["romav2_commit"] = git_commit(Path(args.roma2_repo).expanduser())
        asserted["dinov3_hub_commit"] = dinov3_hub_commit(args.roma2_repo)
    recorded = dict(provenance, value_facet_blocks=facet_blocks)
    for field, claimed in asserted.items():
        if claimed is not None and recorded[field] != claimed:
            raise SystemExit(f"FAILED: {files['descriptor']} was traced with {field} "
                             f"{recorded[field]!r}, but this manifest run was told {claimed!r}. The "
                             f"sidecars are what the graphs were traced with; drop the flag or re-export.")

    descriptor = traced["descriptor"]
    manifest = {
        "format_version": FORMAT_VERSION,
        "model": "roma2",
        "setting": args.setting,
        "image_size": size,
        "patch": PATCH,
        "layers": list(LAYER_IDX),
        "descriptor_layers_shape": io["descriptor"]["outputs"]["layers"],
        "warp_size": cells,
        "confidence_channels": io["match_coarse"]["outputs"]["confidence"][-1],
        "value_facet_blocks": list(facet_blocks),
        "value_facets_shape": io["descriptor"]["outputs"]["value_facets"],
        "retrieval_recipes": {
            "facets": {"dim": 2 * width, "gem_p": GEM_P, "power": FACETS_POWER},
            "layers": {"dim": width, "gem_p": GEM_P, "slice": LAYERS_SLICE},
        },
        "files": files,
        "io": io,
        "opset": next(o.version for o in descriptor.opset_import if o.domain in ("", "ai.onnx")),
        "exporter": provenance["exporter"],
        "precision": provenance["precision"],
        "rope_cache": provenance["rope_cache"],
        "torch": descriptor.producer_version,
        "onnx": onnx.__version__,
        "onnxscript": package_version("onnxscript"),
        "checkpoint_sha256": provenance["checkpoint_sha256"],
        "romav2_commit": provenance["romav2_commit"],
        "dinov3_hub_commit": provenance["dinov3_hub_commit"],
        "sha256": sha,
        "traced_at": {key: sidecar["exported_at"] for key, sidecar in sidecars.items()},
        "exported_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    out = out_dir / f"roma_{args.setting}.json"
    out.write_text(json.dumps(manifest, indent=2) + "\n")
    hashed = sum((out_dir / name).stat().st_size for name in files.values())
    print(f"wrote {out} ({len(files)} files, {hashed / 1048576:.1f} MB hashed)", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    po = sub.add_parser("onnx", help="trace a stage to fp32 ONNX (architecture independent)")
    po.add_argument("--stage", default="descriptor", choices=list(STAGE_IO),
                    help="descriptor: the per-image DINOv3 layers and value facets; match: the per-pair warp")
    po.add_argument("--coarse", action="store_true",
                    help="match only: stop at the matcher head's quarter-resolution grid instead of the refiners")
    po.add_argument("--with-refiner", action="store_true",
                    help="not exported: the refiner cascade needs romav2's local_corr CUDA extension, the "
                         "VGG19-BN fine-feature weights and grid_sample. The flag exists so asking for it "
                         "says why, and so --stage match can grow the branch without renaming anything")
    # The remaining romav2 settings add bidirectional matching and a second high-resolution stage, neither of
    # which the exported graphs reproduce — they would trace a quietly wrong model.
    po.add_argument("--setting", default="base", choices=list(SETTINGS),
                    help="square input resolution romav2 traces for: "
                         + ", ".join(f"{name} {size}" for name, size in SETTINGS.items()))
    po.add_argument("--out-dir", default=".",
                    help="directory the .onnx, its .onnx.data and its .reference/ are written to")
    po.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT,
                    help="the RoMa v2 checkpoint to trace; a missing file is an error, never a pull")
    po.add_argument("--roma2-repo", default=DEFAULT_ROMA2_REPO,
                    help="checkout whose src/ provides the romav2 package")
    po.add_argument("--value-facet-blocks", type=block_list,
                    help="the two comma-separated 0-indexed backbone blocks the value_facets output taps "
                         "(default 15,20 — only a re-measurement should move these)")
    po.add_argument("--exporter", default="dynamo", choices=["dynamo", "torchscript"],
                    help="torchscript is the fallback if the dynamo capture drops the hook-captured facets")
    po.add_argument("--rope-cache", action="store_true",
                    help="memoise the RoPE sin/cos per (H, W) instead of recomputing them per block, which "
                         "lets the exporter fold them into one shared constant pair. Off by default: it "
                         "saves 0.9 MB of 978 MB at turbo and moves value_facets by ~3e-6 relative")

    pk = sub.add_parser("check", help="compare a graph under onnxruntime against the eager fp32 reference")
    pk.add_argument("--onnx", required=True)
    pk.add_argument("--reference", help="defaults to the .reference directory beside the graph")
    pk.add_argument("--provider", default="cuda", choices=list(PROVIDERS),
                    help="cuda falls back to CPU per node, as the runtime does; cpu is the tolerance floor "
                         "every non-CUDA platform sees")
    # The descriptor has no confidence to condition on, so it is judged by cosine. polyml's floor on a real
    # pair is 0.9991 at turbo for a bf16 engine, and a bound at 0.999 would sit inside the spread between
    # builds; an fp32 ONNX sits far inside it.
    pk.add_argument("--min-cosine", type=float, default=0.998,
                    help="descriptor only: fail below this, on the raw outputs and on the pooled "
                         "descriptors, or on any non-finite output")
    # polyml's bounds, which carry roughly 4x margin over what its bf16 engines measure: warp p99 reaches
    # 0.35 px on the coarse stage there, and agreement stays above 99.88%.
    pk.add_argument("--max-warp-error", type=float, default=2.0,
                    help="pair stages: fail if the warp's p99 error in matched cells exceeds this, in pixels")
    pk.add_argument("--min-agreement", type=float, default=99.5,
                    help="pair stages: fail if the graph and eager disagree about whether a cell is "
                         "matched more often than this allows, as a percentage")
    pk.add_argument("--repeat", type=int, default=100,
                    help="time this many executions after the parity comparison; 0 to skip")
    pk.add_argument("--noise-floor", action="store_true",
                    help="descriptor only: also print how far the model's own bf16 autocast lands from the "
                         "same eager fp32 reference, which is the floor any runtime is measured against")
    pk.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT, help="--noise-floor only")
    pk.add_argument("--roma2-repo", default=DEFAULT_ROMA2_REPO, help="--noise-floor only")

    pm = sub.add_parser("manifest", help="write roma_<setting>.json for the graphs in a directory")
    pm.add_argument("--out-dir", default=".", help="the model directory holding both stages of --setting")
    pm.add_argument("--setting", default="base", choices=list(SETTINGS))
    pm.add_argument("--precision", default="fp32", help="the precision in the graphs' file names")
    # Provenance is read from each graph's .export.json sidecar, never from these. They exist only so a
    # caller can assert what it believes it is describing: a mismatch is an error, not an overwrite.
    pm.add_argument("--exporter", choices=["dynamo", "torchscript"],
                    help="assert the exporter the sidecars record; fail if it differs")
    pm.add_argument("--value-facet-blocks", type=block_list,
                    help="assert the blocks the descriptor sidecar records; fail if they differ")
    pm.add_argument("--checkpoint", help="assert the checkpoint the sidecars were traced from, by sha256")
    pm.add_argument("--roma2-repo", help="assert the checkout the sidecars record, by commit")

    args = ap.parse_args()
    if args.command == "onnx" and args.coarse and args.stage != "match":
        ap.error("--coarse only applies to --stage match")
    if getattr(args, "value_facet_blocks", None) and len(args.value_facet_blocks) != 2:
        ap.error("--value-facet-blocks needs exactly two blocks: value_facets is [1, 2, G, G, C]")
    {"onnx": export_onnx, "check": check_onnx, "manifest": write_manifest}[args.command](args)


if __name__ == "__main__":
    main()
