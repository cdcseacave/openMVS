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
and reuse the result. The pair stage is not traced yet.

Run inside polyml's export project env:
  cd ~/polyml/romav2 && uv run --with 'onnxruntime-gpu==1.23.2' \\
      python ~/openMVS/scripts/python/roma2/export.py onnx --stage descriptor --setting base --out-dir <dir>
"""
import argparse
from pathlib import Path

SETTINGS = {"turbo": 320, "fast": 512, "base": 640}   # model.H_lr after apply_setting

DEFAULT_CHECKPOINT = "~/.cache/torch/hub/checkpoints/romav2.0.1.pt"   # == Polycam/romav2 mirror's romav2.0.1.pt
DEFAULT_ROMA2_REPO = "~/polyml/romav2"                # the vendored RoMaV2; ~/RoMaV2 (upstream) also works

STAGE_IO = {
    "descriptor": (["image"], ["layers", "value_facets"]),
    "match": (["descriptors_A", "descriptors_B", "img_A", "img_B"], ["warp", "confidence"]),
}


def onnx_path(out_dir, setting, stage, coarse, precision):
    """roma_<setting>_<stage>[_coarse]_<precision>.onnx — no architecture: the graph is portable input."""
    parts = ["roma", setting, stage] + (["coarse"] if coarse else []) + [precision]
    return Path(out_dir).expanduser().resolve() / ("_".join(parts) + ".onnx")


def reference_dir_for_onnx(onnx_file):
    return Path(onnx_file).with_suffix(".reference")


def graph_size(path):
    """An ONNX plus the external data file its weights live in."""
    path = Path(path)
    return sum(sibling.stat().st_size for sibling in path.parent.glob(path.name + "*"))


def block_list(spec):
    """argparse type for --value-facet-blocks: "15,20" -> [15, 20]."""
    return [int(block) for block in spec.split(",")]


def export_onnx(args):
    from graphs import FACET_BLOCKS, trace_to_onnx   # torch only enters here

    inputs, outputs = STAGE_IO[args.stage]
    out = onnx_path(args.out_dir, args.setting, args.stage, args.coarse, "fp32")
    out.parent.mkdir(parents=True, exist_ok=True)
    reference = reference_dir_for_onnx(out)
    produced = trace_to_onnx(out, reference, args.stage, args.coarse, args.setting, args.checkpoint,
                             args.roma2_repo, inputs, outputs, args.exporter,
                             facet_blocks=args.value_facet_blocks or FACET_BLOCKS,
                             rope_cache=args.rope_cache)
    print(f"wrote {out} ({graph_size(out) / 1048576:.1f} MB including weights)", flush=True)
    print(f"wrote {reference}/ ({len(inputs)} inputs, {produced} eager fp32 outputs)", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    po = sub.add_parser("onnx", help="trace a stage to fp32 ONNX (architecture independent)")
    po.add_argument("--stage", default="descriptor", choices=list(STAGE_IO),
                    help="descriptor: the per-image DINOv3 layers and value facets; match: the per-pair warp")
    po.add_argument("--coarse", action="store_true",
                    help="match only: stop at the matcher head's quarter-resolution grid instead of the refiners")
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

    args = ap.parse_args()
    if args.command == "onnx":
        if args.coarse and args.stage != "match":
            ap.error("--coarse only applies to --stage match")
        if args.stage != "descriptor":
            ap.error(f"--stage {args.stage} is not traced yet: the pair graph and its MatchWrap land with "
                     f"the matching stage")
        if args.value_facet_blocks and len(args.value_facet_blocks) != 2:
            ap.error("--value-facet-blocks needs exactly two blocks: value_facets is [1, 2, G, G, C]")
    {"onnx": export_onnx}[args.command](args)


if __name__ == "__main__":
    main()
