#!/usr/bin/env python3
"""Build a reference directory from real images, for checking a graph against something photographic.

    python parity.py A.jpg B.jpg --stage match --coarse --setting base --out-dir real_base_match_coarse.reference
    python export.py check --onnx roma_base_match_coarse_fp32.onnx --reference real_base_match_coarse.reference

`export.py onnx` already saves a reference beside each graph, but from the random tensors it traced with.
Random input exercises the same kernels and is enough to catch a graph that computes nothing, while a
photograph is what catches precision that only misbehaves on real structure — edges, texture, sky. The
matcher head is the stage that needs it: its warp is an argmax over a correlation volume, which on random
descriptors has no peak to move.

Preprocesses exactly as romav2.match() does (load -> [0,1] -> bicubic antialias to the setting's square
size), then runs the eager model in fp32 — the same ground truth `export.py onnx` records — and writes the
npy layout `check` reads. Unlike that traced reference, the pair stage here is fed the descriptor stage's
real output rather than random tensors of the right shape.

What openMVS adds to polyml's parity.py, all of it for the C++ side (Task 7's RoMa2OnnxParityTest reads
this directory, and its always-on tests read --fixtures):
  source_A.png / source_B.png   the decoded RGB8 sources, losslessly, so the C++ preprocessing test starts
                                from the same pixels rather than from its own JPEG decoder's
  pooled_facets_{A,B}.npy       [2 * C] and [C]: the retrieval descriptors PoolRetrievalDescriptor has to
  pooled_layers_{A,B}.npy       reproduce, for both images of the pair
  parity.json                   what the directory holds and the bounds it is judged under
  --fixtures DIR                the two model-free fixtures the always-on C++ tests use, as raw fp32
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

from export import DEFAULT_CHECKPOINT, DEFAULT_ROMA2_REPO, SETTINGS, STAGE_IO

FIXTURE_CROP = (97, 61)     # deliberately odd, so a C++ stride or row-padding bug cannot hide in it
FIXTURE_SIZE = 64           # the square the crop is resized to: 3 * 64 * 64 * 4 = 48 KB
FIXTURE_POOL_SHAPE = (1, 2, 3, 3, 8)


def load_image(path, size, device="cuda"):
    """(the [0,1] square tensor the model takes, the decoded RGB8 the C++ side must start from)."""
    from PIL import Image

    decoded = np.array(Image.open(path).convert("RGB"), dtype=np.uint8)
    image = torch.from_numpy(decoded.astype(np.float32)).permute(2, 0, 1)[None].to(device) / 255.0
    return resize(image, size), decoded


def resize(image, size):
    """romav2.match()'s preprocessing, and the one the C++ RoMa2Onnx::Preprocess reproduces."""
    return torch.nn.functional.interpolate(image, size=(size, size), mode="bicubic", align_corners=False,
                                           antialias=True)


def write_fixtures(directory, decoded_A):
    """The two always-on C++ fixtures: they pin the host-side halves of the contract without a model.

    Raw little-endian fp32, not npy, because the tests that read them run on every build and should not
    need a header parser to do it. The pooling fixture is a seeded random tensor rather than a slice of a
    real descriptor so that it stays 1 KB and stays reproducible from the seed alone.

    Computed on the CPU, deliberately, unlike everything else here. These bytes are committed and are then
    the expected value for every build on every machine, so they must be a property of the recipe and not
    of whichever GPU happened to regenerate them: torch's own CUDA and CPU bicubic differ by ~2e-7 on this
    crop, which is small but is a hardware fingerprint, and a fixture nobody without an A100 can reproduce
    is not a fixture.
    """
    from PIL import Image
    from graphs import pool_retrieval

    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    width, height = FIXTURE_CROP
    x0, y0 = (decoded_A.shape[1] - width) // 2, (decoded_A.shape[0] - height) // 2
    crop = decoded_A[y0:y0 + height, x0:x0 + width]
    Image.fromarray(crop).save(directory / "preprocess_source.png")
    square = resize(torch.from_numpy(crop.astype(np.float32)).permute(2, 0, 1)[None] / 255.0, FIXTURE_SIZE)
    square.numpy().astype("<f4").tofile(directory / f"preprocess_{FIXTURE_SIZE}.bin")

    pooled = torch.rand(FIXTURE_POOL_SHAPE, generator=torch.Generator().manual_seed(0))
    shape = "x".join(str(dimension) for dimension in FIXTURE_POOL_SHAPE[1:])   # the batch dim is implied
    pooled.numpy().astype("<f4").tofile(directory / f"pool_input_{shape}.bin")
    facets, layers = pool_retrieval(pooled, "facets"), pool_retrieval(pooled, "layers")
    facets.astype("<f4").tofile(directory / f"pool_facets_{facets.size}.bin")
    layers.astype("<f4").tofile(directory / f"pool_layers_{layers.size}.bin")
    print(f"wrote {directory}/ fixtures (cpu): preprocess_source.png {width}x{height}, "
          f"preprocess_{FIXTURE_SIZE}.bin {tuple(square.shape)}, pool_input_{shape}.bin "
          f"{FIXTURE_POOL_SHAPE} -> facets[{facets.size}] layers[{layers.size}]", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("img_a")
    ap.add_argument("img_b")
    ap.add_argument("--stage", default="descriptor", choices=list(STAGE_IO))
    ap.add_argument("--coarse", action="store_true", help="match only: the head's own grid, no refiners")
    ap.add_argument("--setting", default="base", choices=list(SETTINGS))
    ap.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT,
                    help="a reference built on substituted weights would validate nothing, so a missing "
                         "checkpoint is an error rather than a pull")
    ap.add_argument("--roma2-repo", default=DEFAULT_ROMA2_REPO)
    ap.add_argument("--value-facet-blocks", type=int, nargs=2, metavar=("B0", "B1"),
                    help="the blocks value_facets taps; must match what the graph was traced with")
    ap.add_argument("--out-dir", default="real.reference")
    ap.add_argument("--fixtures", help="also write the always-on C++ fixtures into this directory")
    args = ap.parse_args()
    if args.coarse and args.stage != "match":
        ap.error("--coarse only applies to --stage match")

    from graphs import (FACET_BLOCKS, REFINER_NOT_EXPORTED, DescriptorWrap, MatchWrap, build_model,
                        pool_retrieval, save_reference)

    if args.stage == "match" and not args.coarse:
        raise NotImplementedError(REFINER_NOT_EXPORTED)

    model = build_model(args.setting, args.checkpoint, args.roma2_repo)
    img_A, decoded_A = load_image(args.img_a, model.H_lr)
    img_B, decoded_B = load_image(args.img_b, model.H_lr)

    input_names, output_names = STAGE_IO[args.stage]
    with DescriptorWrap(model, args.value_facet_blocks or FACET_BLOCKS).eval() as descriptor, torch.no_grad():
        descriptors_A, descriptors_B = descriptor(img_A), descriptor(img_B)   # (layers, value_facets) each
        if args.stage == "descriptor":
            inputs, eager = (img_A,), descriptors_A
        else:
            # The real descriptors, not tensors of the right shape: this is the reference that exists
            # because random inputs give the matcher head nothing to find.
            inputs = (descriptors_A[0], descriptors_B[0], img_A, img_B)
            eager = MatchWrap(model, args.coarse).eval()(*inputs)
    eager_tensors = (eager,) if isinstance(eager, torch.Tensor) else tuple(eager)

    directory = Path(save_reference(args.out_dir, input_names, inputs, output_names, eager_tensors))
    for name, tensor in zip(output_names, eager_tensors):
        print(f"{name}: {tuple(tensor.shape)} absmax {tensor.abs().max().item():.3f}", flush=True)

    # The additions the C++ parity test reads. The pooled descriptors come from the same two forward passes
    # the stage above ran, so nothing here can disagree with what was written beside it.
    from PIL import Image

    pooled = {}
    for side, decoded, (layers, facets) in (("A", decoded_A, descriptors_A), ("B", decoded_B, descriptors_B)):
        Image.fromarray(decoded).save(directory / f"source_{side}.png")
        for recipe, tensor in (("facets", facets), ("layers", layers)):
            vector = pool_retrieval(tensor, recipe)
            np.save(directory / f"pooled_{recipe}_{side}.npy", vector)
            pooled[f"{recipe}_{side}"] = vector
            print(f"pooled_{recipe}_{side}: [{vector.size}] norm {np.linalg.norm(vector):.6f}", flush=True)

    (directory / "parity.json").write_text(json.dumps({
        "format_version": 1,
        "setting": args.setting,
        "image_size": model.H_lr,
        "stage": args.stage + ("_coarse" if args.coarse else ""),
        "sources": {"A": str(Path(args.img_a).resolve()), "B": str(Path(args.img_b).resolve())},
        "source_shape": {side: list(decoded.shape) for side, decoded in (("A", decoded_A), ("B", decoded_B))},
        "value_facet_blocks": list(args.value_facet_blocks or FACET_BLOCKS),
        "shapes": {f"in_{name}": list(tensor.shape) for name, tensor in zip(input_names, inputs)}
                  | {f"out_{name}": list(tensor.shape) for name, tensor in zip(output_names, eager_tensors)}
                  | {f"pooled_{name}": [vector.size] for name, vector in pooled.items()},
        # polyml's, the bounds export.py check defaults to and the ones the C++ test judges under.
        "bounds": {"min_cosine": 0.998, "max_warp_error_px": 2.0, "min_agreement_percent": 99.5},
    }, indent=2) + "\n")

    if args.fixtures:
        write_fixtures(args.fixtures, decoded_A)
    print(f"wrote {directory.resolve()}/ from {args.img_a} + {args.img_b}", flush=True)


if __name__ == "__main__":
    main()
