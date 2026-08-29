"""The torch side of the RoMa v2 export: the stage graphs, their tracing and the retrieval pooling.

Kept apart from export.py (layout: polyml romav2/graphs.py): everything that loads weights or builds a
module lives here, so running a shipped graph and reading its numbers stays a job for onnxruntime and numpy.
Two things in export.py do reach in and bring torch with them — the pooled-descriptor half of `check` and
the model constants the manifest publishes — because a second copy of the retrieval recipe or of the layer
indices is exactly the divergence the C++ side would inherit.
"""
import contextlib
import inspect
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

PATCH = 16
LAYER_IDX = [11, 17]            # features.py:87-89, 0-based blocks; final norm applied; CLS+4 registers dropped
FACET_BLOCKS = [15, 20]         # EXPORT_REQUEST.md: v_proj of blocks 15 and 20 (0-indexed), in that order


class DescriptorWrap(torch.nn.Module):
    """img -> (layers, value_facets), the two tensors the descriptor stage of the contract emits.

    `layers` is the DINOv3 layers the matcher consumes (model.f, i.e. features.py's wrapped_forward with
    autocast stripped by to_fp32), stacked along dim 1. The layers come out of the backbone as token slices
    into one shared buffer, so they start at an offset into storage they do not own; stacking makes the
    stage emit a single tensor that owns its storage from element zero, which is what a runtime can bind a
    plain device pointer to. Matching depends on this tensor exactly as it is.

    `value_facets` is the V third of the fused qkv Linear of the facet blocks, captured by a forward hook
    inside the same forward. The hub's SelfAttention runs one fused qkv Linear on norm1(x) and only then
    reshapes to (B, N, 3, heads, head_dim) (dinov3/layers/attention.py:88,111), so the last third is exactly
    v_proj(norm1(x)) with heads concatenated in their natural order, taken before attention weighting and
    before o_proj. The projections are computed anyway; the hook only keeps them alive to an output. CLS and
    the 4 register tokens are dropped and the result is laid out [1, 2, h, w, 1024], like layers.

    Reading a hook's capture back out of self.taps makes torch.onnx.export warn that "the tensor attributes
    self.taps[...] were assigned during export" and suggest register_buffer. It is benign for a graph traced
    once at a fixed shape — the captures are graph values, not state carried between calls — and the audit
    that both outputs reach the graph plus _check_descriptor cover what the warning is pointing at.
    """

    def __init__(self, model, facet_blocks=FACET_BLOCKS):
        super().__init__()
        self.m = model
        self.facet_blocks = list(facet_blocks)
        backbone = _dino_backbone(model.f)              # the DinoVisionTransformer under features.py's wrapper
        assert len(backbone.blocks) == 24 and backbone.n_storage_tokens == 4
        assert backbone.patch_size == PATCH, f"the contract's G = S/{PATCH} grid needs patch {PATCH}"
        assert len(self.facet_blocks) == 2, \
            f"value_facets is [1, 2, G, G, C], so it needs exactly two blocks, not {self.facet_blocks}"
        assert all(0 <= b < len(backbone.blocks) for b in self.facet_blocks), \
            f"facet blocks {self.facet_blocks} outside the backbone's 0..{len(backbone.blocks) - 1}"
        self.n_prefix = 1 + backbone.n_storage_tokens   # CLS + 4 registers (vision_transformer.py:308)
        self.taps = {}
        self.handles = [backbone.blocks[b].attn.qkv.register_forward_hook(self._tap(b))
                        for b in self.facet_blocks]     # fused qkv output [B, N, 3C] (hub attention.py:88)

    def _tap(self, b):
        def hook(_module, _inputs, output):
            self.taps[b] = output
        return hook

    def close(self):
        """Unregister the qkv hooks. The model outlives the wrap — the match stage runs against the same
        instance in the same process — so leaving them registered would keep taping tensors nothing reads."""
        for handle in self.handles:
            handle.remove()
        self.handles.clear()
        self.taps.clear()

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        self.close()

    def forward(self, img):
        self.taps.clear()
        f0, f1 = self.m.f(img)                          # [1, h, w, 1024] each: final norm applied, patch tokens only
        B, h, w, C = f0.shape
        facets = [self.taps[b].reshape(B, -1, 3 * C)[:, self.n_prefix:, 2 * C:].reshape(B, h, w, C)
                  for b in self.facet_blocks]
        self.taps.clear()                               # sliced already; do not pin the [1, N, 3C] captures
        return torch.stack((f0, f1), dim=1), torch.stack(facets, dim=1)


def _dino_backbone(f):
    """features.py's partial_wrap monkeypatches the hub model's forward; find the module that owns .blocks."""
    for module in f.modules():
        if hasattr(module, "blocks") and hasattr(module, "n_storage_tokens"):
            return module
    raise RuntimeError("DINOv3 backbone not found under model.f")


class MatchWrap(torch.nn.Module):
    """(descriptors, images) -> (warp_AB, confidence_AB).

    Reproduces RoMaV2.forward from the matcher onwards, for the single low-resolution stage the exported
    settings use (no hr images, so the precision channels are never zeroed out and the stage loop runs
    once) and without bidirectional matching. With coarse=True it returns the matcher head's own
    prediction (a quarter of the input resolution, one confidence channel) instead of running the refiner
    cascade that walks it up to the image resolution and adds the precision parameters.

    The refined confidence carries the overlap logit and three entries of a precision matrix. Those three
    are squares of network outputs accumulated across stages, so a small perturbation upstream moves them
    a long way: in bf16 they hold a cosine of 0.86 against eager fp32 where the logit beside them holds
    0.999. They are exported because they cost nothing to carry and a consumer may still want them, but
    they are the reason an engine is judged on warp agreement rather than on a cosine over every output.
    """

    def __init__(self, model, coarse):
        super().__init__()
        self.m = model
        self.coarse = coarse

    def forward(self, descriptors_A, descriptors_B, img_A, img_B):
        matcher_output = self.m.matcher(
            [descriptors_A[:, 0], descriptors_A[:, 1]],
            [descriptors_B[:, 0], descriptors_B[:, 1]],
            img_A=img_A,
            img_B=img_B,
            bidirectional=False,
        )
        warp, confidence = matcher_output["warp_AB"], matcher_output["confidence_AB"]
        if self.coarse:
            return warp.clone(), confidence.clone()

        from romav2.romav2 import _interpolate_warp_and_confidence

        B, C, H, W = img_A.shape
        scale_factor = torch.tensor((W / self.m.anchor_width, H / self.m.anchor_height), device=img_A.device)
        refiner_features_A = self.m.refiner_features(img_A)
        refiner_features_B = self.m.refiner_features(img_B)
        for patch_size_str, refiner in self.m.refiners.items():
            patch_size = int(patch_size_str)
            warp, confidence = _interpolate_warp_and_confidence(
                warp=warp, confidence=confidence, H=H, W=W, patch_size=patch_size, zero_out_precision=False
            )
            refined = refiner(
                f_A=refiner_features_A[patch_size],
                f_B=refiner_features_B[patch_size],
                prev_warp=warp,
                prev_confidence=confidence,
                scale_factor=scale_factor,
            )
            warp, confidence = refined["warp"], refined["confidence"]
        return warp.clone(), confidence.clone()


# MatchWrap's refiner branch above is polyml's, kept so the two copies stay diffable, but openMVS exports
# the coarse head alone and stage_graph refuses the rest with this. Three reasons, none of them a graph
# detail: the refiners correlate locally through romav2's local_corr CUDA extension (the pure-torch
# fallback traces, but as a different computation than the one that runs), their fine features come from a
# VGG19-BN whose weights are a second checkpoint this export does not carry, and they resample with
# grid_sample, which lands on ORT as an op whose accelerated coverage is uneven across providers. The
# coarse head needs none of the three (verified: no GridSample node in the traced coarse graph), and its
# 4 px/cell grid at base is what openMVS's warp-guided re-matching consumes.
REFINER_NOT_EXPORTED = (
    "the refiner cascade is out of scope for the openMVS export: it needs romav2's local_corr CUDA "
    "extension (or a batched rewrite), the VGG19-BN fine-feature weights this export does not ship, and "
    "grid_sample. Export the coarse head instead: --stage match --coarse")


def to_fp32(model):
    """Strip the model's autocast so it traces as a plain fp32 graph (polyml graphs.py:112-131).

    RoMa is written to run under autocast: it holds bf16 weights, feeds them fp32 activations, and lets
    autocast reconcile the two at each op. Tracing under autocast gives a graph whose dtypes are an artifact
    of wherever autocast happened to insert its casts, which is not a description of the model to hand to a
    runtime — so the trace is fp32 throughout. Disabling autocast is not enough on its own: the stored
    reduced-precision dtype attributes (RoPE, for one) have to be rewritten too, or the traced graph keeps
    casting back down.
    """
    torch.autocast = lambda *args, **kwargs: contextlib.nullcontext()
    model.float()
    for module in model.modules():
        for name, value in list(vars(module).items()):
            if value is torch.bfloat16 or value is torch.float16:
                setattr(module, name, torch.float32)
            elif name == "enable_amp" and value is True:
                setattr(module, name, False)
    return model


def _assert_masked_bias(model):
    """hub dinov3/layers/attention.py:36 initialises bias_mask with NaN; the checkpoint must have overwritten
    it, or every attention block would emit NaN through a bias the trace bakes in as a constant.

    In romav2.0.1 the mask is all zeros and so is the qkv bias behind it — this backbone's attention runs
    weight-only — rather than the 1/0/1 (q keeps its bias, k loses it) that mask_k_bias exists for. Either is
    a mask; what must hold is that it is finite and binary, which is what the trace bakes in.
    """
    n = 0
    for name, m in model.named_modules():
        mask = getattr(m, "bias_mask", None)
        if mask is None:
            continue
        assert torch.isfinite(mask).all(), f"{name}.bias_mask is not finite"
        assert ((mask == 0) | (mask == 1)).all(), f"{name}.bias_mask is not binary"
        n += 1
    assert n == 24, f"expected 24 LinearKMaskedBias modules in the backbone, found {n}"


def _cache_rope(model):
    """Memoise the RoPE sin/cos per (H, W). Opt-in, off by default — see build_model.

    vision_transformer.py:276 and vit/__init__.py:207 recompute them inside the per-block loop, so an
    otherwise static graph carries one identical copy of the same constant per block. Memoising lets the
    exporter fold the whole family into one shared sin/cos pair: on turbo the descriptor graph goes from 332
    initializers and 1249 nodes to 312 and 1169, at the same accuracy against the eager reference (rms
    1.8025e-04 against 1.8024e-04 on layers). Each cache holds one small sin/cos pair (410 KB each at base)
    for the life of the model, which is why this is not worth doing by default.
    """
    for name, rope in (("f", model.f.rope_embed), ("matcher.mv_vit", model.matcher.mv_vit.rope_embed)):
        assert rope is not None, f"model.{name}.rope_embed is None (use_rope=False): nothing to memoise"
        cache, orig = {}, rope.forward

        def fwd(*, H, W, _orig=orig, _cache=cache):
            if (H, W) not in _cache:
                _cache[(H, W)] = _orig(H=H, W=W)
            return _cache[(H, W)]

        rope.forward = fwd


def build_model(setting, checkpoint, roma2_repo, device="cuda", rope_cache=False, fp32=True):
    """The fp32, eval, traceable RoMa v2 for `setting` (polyml graphs.py:134-139, local weights).

    The vendored RoMaV2 takes weights=<path> (src/romav2/romav2.py:92-119); the upstream fork (~/RoMaV2) has
    no such argument, so there it is redirected through torch.hub.load_state_dict_from_url at the same file.
    A pull is never a fallback: everything downstream — the eager reference the graph is checked against
    included — is built from whatever weights land here, so a substitution would go unnoticed.

    rope_cache is off by default: it saves 0.9 MB of 978 MB at turbo and the two traced graphs agree only to
    ~3e-6 relative on value_facets, short of the 1e-6 the optimisation was gated on, so the shipped graph
    keeps the structure polyml traces.

    fp32=False leaves the model exactly as the checkpoint ships it — bf16 weights under its own autocast —
    which is not traceable but is what bf16_noise_floor measures the graphs' error against.
    """
    repo = Path(roma2_repo).expanduser()
    source = repo / "src"
    assert source.is_dir(), f"--roma2-repo {repo} has no src/: expected a RoMaV2 checkout"
    sys.path.insert(0, str(source))
    torch.set_float32_matmul_precision("highest")   # romav2.forward() asserts this
    ckpt = Path(checkpoint).expanduser()
    assert ckpt.is_file(), f"checkpoint not found: {ckpt}"
    if device == "cpu":
        torch.cuda.is_available = lambda: False     # romav2/device.py picks the device at import time
    from romav2 import RoMaV2
    if "weights" in inspect.signature(RoMaV2.__init__).parameters:
        model = RoMaV2(weights=str(ckpt))
    else:
        torch.hub.load_state_dict_from_url = lambda url, map_location=None, **kw: torch.load(
            ckpt, map_location=map_location, weights_only=True)
        model = RoMaV2()
    assert list(model.cfg.descriptor.layer_idx) == LAYER_IDX, \
        f"the matcher head was trained on blocks {LAYER_IDX}, model asks for {model.cfg.descriptor.layer_idx}"
    model.apply_setting(setting)
    model = model.to(device).eval()
    if fp32:
        model = to_fp32(model)
    _assert_masked_bias(model)
    if rope_cache:
        _cache_rope(model)
    return model


def bf16_noise_floor(setting, checkpoint, roma2_repo, image, layers_fp32):
    """How far the model's own bf16 autocast lands from the eager fp32 reference the graphs are judged on.

    A graph is measured against eager fp32, but nobody runs this model in eager fp32 — RoMa's own inference
    path is bf16 under autocast. So the interesting question about a cosine of 0.99999 is not whether it is
    close to 1 but how it compares with the distance the model already moves when run the way it was
    trained to run. That distance is this, and it is the floor: no runtime is more faithful to `layers` than
    the model itself is.
    """
    model = build_model(setting, checkpoint, roma2_repo, fp32=False)
    device = next(model.parameters()).device
    with torch.no_grad():
        layers = torch.stack(model.f(torch.as_tensor(np.asarray(image)).to(device)), dim=1)
    actual = layers.float().cpu().numpy().astype(np.float64)
    expected = np.asarray(layers_fp32, dtype=np.float64)
    cosine = float(expected.ravel() @ actual.ravel() / (np.linalg.norm(expected) * np.linalg.norm(actual)))
    return cosine, float(np.abs(expected - actual).max())


def pool_retrieval(tensor, recipe="facets", power=0.3):
    """The Python reference of the C++ PoolRetrievalDescriptor (EXPORT_REQUEST.md's recipe).

    tensor is a [1, 2, h, w, C] numpy array or torch tensor. facets: per-slice GeM p=3 -> L2 -> concat -> L2
    -> sign|d|^power -> L2, 2048-d. layers: GeM p=3 on slice 1 -> L2, 1024-d — the shipped recipe, kept as
    the parity gate. Accumulated in float64 because the cube inside GeM costs precision.
    """
    t = torch.as_tensor(tensor).detach().to("cpu", torch.float64)[0]                    # [2, h, w, C]
    gem = lambda s: F.normalize(s.reshape(-1, s.shape[-1]).clamp(min=1e-6).pow(3).mean(0).pow(1 / 3), dim=0)
    if recipe == "layers":
        return gem(t[1]).float().numpy()
    d = F.normalize(torch.cat([gem(t[0]), gem(t[1])]), dim=0)
    return F.normalize(torch.sign(d) * d.abs().pow(power), dim=0).float().numpy()


def stage_graph(model, stage, coarse, facet_blocks=FACET_BLOCKS):
    """The module to trace for `stage`, plus example inputs of the shapes it will be called with."""
    size = model.H_lr
    device = next(model.parameters()).device
    img_A = torch.rand(1, 3, size, size, device=device)
    if stage == "descriptor":
        return DescriptorWrap(model, facet_blocks).eval(), (img_A,)
    if not coarse:
        raise NotImplementedError(REFINER_NOT_EXPORTED)
    # Descriptor-shaped inputs rather than the descriptor's actual output: the graph only needs shapes and
    # dtypes to trace, and these same tensors become the graph's own parity inputs, so running the backbone
    # first would only make the check depend on a second stage. parity.py feeds it real descriptors.
    grid = size // PATCH
    width = _dino_backbone(model.f).embed_dim      # 1024 for dinov3_vitl16, the contract's descriptor width
    img_B = torch.rand(1, 3, size, size, device=device)
    descriptors_A = torch.rand(1, 2, grid, grid, width, device=device)
    descriptors_B = torch.rand(1, 2, grid, grid, width, device=device)
    return MatchWrap(model, coarse).eval(), (descriptors_A, descriptors_B, img_A, img_B)


def save_reference(directory, input_names, inputs, output_names, outputs):
    """Write the tensors a graph is checked against: its own inputs and its eager fp32 outputs."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for name, tensor in zip(input_names, inputs):
        np.save(directory / f"in_{name}.npy", tensor.detach().float().cpu().numpy())
    for name, tensor in zip(output_names, outputs):
        np.save(directory / f"out_{name}.npy", tensor.detach().float().cpu().numpy())
    return directory


def _check_descriptor(graph, img, layers, value_facets, tol=1e-4):
    """Prove, before tracing, that the two descriptor outputs are the ones the contract names.

    `layers` has to be what polyml's DescriptorWrap already emits, because the matcher head binds it. The
    facets are recomputed from an independently captured block input rather than trusted to the hook, so a
    tap on the wrong block, the wrong third of qkv or the wrong token offset fails here instead of shipping.
    """
    backbone = _dino_backbone(graph.m.f)
    block_input = {}
    handles = [backbone.blocks[b].register_forward_pre_hook(
        lambda _module, args, _b=b: block_input.__setitem__(_b, args[0])) for b in graph.facet_blocks]
    try:
        expected_layers = torch.stack(graph.m.f(img), dim=1)
    finally:
        for handle in handles:
            handle.remove()

    error = (layers - expected_layers).abs().max().item()
    assert error <= tol, f"layers != stack(model.f(img), 1): max abs {error:.3e}"
    print(f"  layers == stack(model.f(img), 1): max abs {error:.2e}", flush=True)
    for i, b in enumerate(graph.facet_blocks):
        blk = backbone.blocks[b]
        C = blk.attn.qkv.in_features
        v = blk.attn.qkv(blk.norm1(block_input[b]))[..., 2 * C:][:, graph.n_prefix:]
        expected = value_facets[:, i]
        error = (v.reshape(expected.shape) - expected).abs().max().item()
        assert error <= tol, f"value_facets[:, {i}] != v_proj(norm1(x)) of block {b}: max abs {error:.3e}"
        print(f"  value_facets[:, {i}] == block {b} qkv(norm1(x))[..., 2C:]: max abs {error:.2e}", flush=True)


def _check_coarse_match(model, warp, confidence, descriptors_A, descriptors_B, img_A, img_B,
                        roma2_repo=None, warp_tol=1e-4, confidence_tol=1e-3):
    """Prove, before tracing, that MatchWrap's coarse output is the model's own coarse prediction.

    MatchWrap calls model.matcher directly, so nothing in the wrap would notice if RoMaV2's coarse entry
    point did something else on the way in or out — a sigmoid, a clone of a different key, a transpose. The
    fork's coarse_cached_match is that entry point (romav2.py:358-383): it takes the cached per-image
    features and returns exactly what openMVS wants a graph for, which makes it the reference this wrap has
    to reproduce. It mutates frame["features"] in place, so it is given clones.

    Only the ~/RoMaV2 fork carries the method; polyml's vendored copy predates it, and there the proof is
    skipped rather than faked. Run it once per change with --roma2-repo ~/RoMaV2: the matcher, the head and
    the DPT under them are byte-identical between the two checkouts.
    """
    reference = getattr(model, "coarse_cached_match", None)
    if reference is None:
        # Column 0, upper case, on stderr: a skipped gate has to be greppable in a log that is mostly
        # torch's export chatter, or "the proof ran" and "the proof was not there" read the same.
        print(f"WARNING: coarse pre-trace proof SKIPPED - RoMaV2.coarse_cached_match not in "
              f"{roma2_repo or 'this romav2 checkout'}; the traced graph is UNPROVEN against the model's "
              f"own coarse path. Re-run with --roma2-repo ~/RoMaV2 to exercise it.",
              file=sys.stderr, flush=True)
        return
    frame = lambda L, img: {"features": [L[:, 0].clone(), L[:, 1].clone()], "rescaled": img}
    expected = reference(frame(descriptors_A, img_A), frame(descriptors_B, img_B))
    warp_error = (warp - expected["warp_AB"]).abs().max().item()
    confidence_error = (confidence - expected["confidence_AB"]).abs().max().item()
    assert warp_error <= warp_tol, f"warp != coarse_cached_match's warp_AB: max abs {warp_error:.3e}"
    assert confidence_error <= confidence_tol, \
        f"confidence != coarse_cached_match's confidence_AB: max abs {confidence_error:.3e}"
    print(f"  (warp, confidence) == model.coarse_cached_match: max abs {warp_error:.2e} / "
          f"{confidence_error:.2e}", flush=True)


def trace_to_onnx(out_path, reference_dir, stage, coarse, setting, checkpoint, roma2_repo, input_names,
                  output_names, exporter="dynamo", facet_blocks=FACET_BLOCKS, rope_cache=False):
    """Trace `stage` to fp32 ONNX and save the eager fp32 reference beside it. Returns the output count.

    The reference is eager fp32 whatever a consumer later runs the graph in, so a check measures the runtime
    against the model rather than against another copy of its own approximation. Two audits run on the
    written graph: that no reduced-precision initializer survived to_fp32, and that the graph's outputs are
    the contract's — both are silent failures otherwise, one at accuracy and one at bind time.
    """
    import onnx

    model = build_model(setting, checkpoint, roma2_repo, rope_cache=rope_cache)
    graph, example_inputs = stage_graph(model, stage, coarse, facet_blocks)

    print(f"trace fp32 ONNX ({stage}{' coarse' if coarse else ''}) ...", flush=True)
    try:
        with torch.no_grad():
            eager = graph(*example_inputs)
            if stage == "descriptor":
                _check_descriptor(graph, example_inputs[0], *eager)
            elif coarse:
                _check_coarse_match(model, *eager, *example_inputs, roma2_repo=roma2_repo)
            if exporter == "dynamo":
                torch.onnx.export(graph, tuple(example_inputs), str(out_path), input_names=input_names,
                                  output_names=output_names, opset_version=18, dynamo=True,
                                  external_data=True)
            else:   # fallback if the dynamo capture drops the hook-captured value_facets
                torch.onnx.export(graph, tuple(example_inputs), str(out_path), input_names=input_names,
                                  output_names=output_names, opset_version=18, dynamo=False,
                                  do_constant_folding=True)
                inline = onnx.load(str(out_path))   # the legacy exporter writes one file; split the weights
                external = Path(out_path).with_name(Path(out_path).name + ".data")
                external.unlink(missing_ok=True)    # onnx.save appends to an external data file that exists
                onnx.save(inline, str(out_path), save_as_external_data=True, all_tensors_to_one_file=True,
                          location=external.name, size_threshold=1024)
    finally:
        # the descriptor wrap's hooks, before the model is handed on or the process moves to a check;
        # MatchWrap is polyml's verbatim and holds nothing to release, so it has no close()
        close = getattr(graph, "close", None)
        if close is not None:
            close()

    onnx.checker.check_model(str(out_path), full_check=True)
    written = onnx.load(str(out_path), load_external_data=False)
    bad = [i.name for i in written.graph.initializer
           if i.data_type in (onnx.TensorProto.BFLOAT16, onnx.TensorProto.FLOAT16)]
    assert not bad, f"non-fp32 initializers survived to_fp32: {bad[:5]}"
    produced = [o.name for o in written.graph.output]
    assert set(produced) == set(output_names), f"graph outputs {produced} are not the contract's {output_names}"
    print(f"  checker OK ({exporter}, opset 18); {len(written.graph.initializer)} initializers, "
          f"none bf16/fp16; outputs {produced}", flush=True)

    eager_tensors = (eager,) if isinstance(eager, torch.Tensor) else tuple(eager)
    save_reference(reference_dir, input_names, example_inputs, output_names, eager_tensors)
    return len(eager_tensors)
