"""Export the DeFM ViT-S/14 checkpoint used by VNTPA to ONNX (opset 17)."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys

import torch


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default=str(root / "conf/models/defm_vit_s14.pth"))
    parser.add_argument("--out", default=str(root / "conf/models/defm_vit_s14.onnx"))
    parser.add_argument(
        "--defm-source",
        default=str(root / "python"),
        help="Path containing vendored thirdparty/defm or the upstream defm package",
    )
    parser.add_argument("--validate", action="store_true", help="Compare three CPU inputs with ONNX Runtime")
    parser.add_argument("--reference-dir", type=Path, help="Save float32 input/output fixtures for the C++ Engine")
    parser.add_argument("--report", type=Path, help="Write export and numerical validation evidence as JSON")
    parser.add_argument("--update-manifest", action="store_true", help="Update the manifest hash after numerical validation")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--pytorch-atol", type=float, default=5e-4,
                        help="Cross-framework float32 tolerance; C++/ORT uses a separate 1e-4 check")
    args = parser.parse_args()
    if args.update_manifest and not args.validate:
        parser.error("--update-manifest requires --validate")
    source = Path(args.defm_source).resolve()
    if source.name == "thirdparty":
        source = source.parent
    sys.path.insert(0, str(source))
    os.environ.setdefault("XFORMERS_DISABLED", "1")
    if (source / "thirdparty/defm").is_dir():
        from thirdparty.defm.models.vision_transformer import vit_small  # noqa: PLC0415
    else:
        from defm.models.vision_transformer import vit_small  # noqa: PLC0415

    torch.set_num_threads(args.threads)
    torch.manual_seed(0)

    model = vit_small(
        img_size=224,
        patch_size=14,
        in_chans=3,
        init_values=1e-5,
        ffn_layer="swiglufused",
        block_chunks=4,
        qkv_bias=True,
        proj_bias=True,
        ffn_bias=True,
        num_register_tokens=0,
        interpolate_offset=0.1,
        interpolate_antialias=False,
    )
    state = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    model.load_state_dict(state.get("model", state))
    model.eval()

    class Export(torch.nn.Module):
        def __init__(self, encoder: torch.nn.Module) -> None:
            super().__init__()
            self.encoder = encoder

        def forward(self, depth: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
            tokens = self.encoder.get_intermediate_layers(depth, n=1, reshape=False)[0]
            return tokens, tokens.mean(dim=1)

    wrapper = Export(model).eval()
    output = Path(args.out).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".onnx.tmp")
    torch.onnx.export(
        wrapper,
        torch.randn(1, 3, 224, 224),
        str(temporary),
        opset_version=17,
        input_names=["depth"],
        output_names=["tokens", "pooled"],
        dynamo=False,
    )
    import onnx  # noqa: PLC0415

    graph = onnx.load(str(temporary))
    onnx.checker.check_model(graph)
    evidence = {"torch": torch.__version__, "opset": 17, "cases": {}}
    if args.validate:
        import numpy as np  # noqa: PLC0415
        import onnxruntime as ort  # noqa: PLC0415

        options = ort.SessionOptions()
        options.intra_op_num_threads = args.threads
        session = ort.InferenceSession(str(temporary), sess_options=options,
                                       providers=["CPUExecutionProvider"])
        evidence["onnxruntime"] = ort.__version__
        samples = {
            "zeros": torch.zeros(1, 3, 224, 224),
            "random": torch.randn(1, 3, 224, 224),
            "ramp": torch.linspace(-2, 2, 3 * 224 * 224).reshape(1, 3, 224, 224),
        }
        if args.reference_dir:
            args.reference_dir.mkdir(parents=True, exist_ok=True)
        for name, sample in samples.items():
            with torch.inference_mode():
                expected = [value.numpy() for value in wrapper(sample)]
            actual = session.run(["tokens", "pooled"], {"depth": sample.numpy()})
            errors = {}
            for tensor_name, shape, reference, result in zip(
                ["tokens", "pooled"], [(1, 256, 384), (1, 384)], expected, actual
            ):
                if result.shape != shape or not np.isfinite(result).all():
                    raise RuntimeError(f"{name}/{tensor_name}: invalid output shape or values")
                error = float(np.max(np.abs(reference - result)))
                if error >= args.pytorch_atol:
                    raise RuntimeError(f"{name}/{tensor_name}: max error {error} exceeds {args.pytorch_atol}")
                errors[tensor_name] = error
                if args.reference_dir:
                    result.astype("<f4").tofile(args.reference_dir / f"{name}.{tensor_name}.f32")
            if args.reference_dir:
                sample.numpy().astype("<f4").tofile(args.reference_dir / f"{name}.depth.f32")
            evidence["cases"][name] = errors
        # Runtime checks above establish the shapes of this static export.
        for tensor, shape in zip(graph.graph.output, [(1, 256, 384), (1, 384)]):
            for dimension, size in zip(tensor.type.tensor_type.shape.dim, shape):
                dimension.dim_value = size
        onnx.checker.check_model(graph)
        onnx.save(graph, str(temporary))
        evidence["pytorch_atol"] = args.pytorch_atol
    temporary.replace(output)
    evidence["path"] = str(output)
    evidence["bytes"] = output.stat().st_size
    evidence["sha256"] = hashlib.sha256(output.read_bytes()).hexdigest()
    if args.update_manifest:
        manifest = root / "conf/models/defm.pb.txt"
        text, count = re.subn(r'^sha256: "[0-9a-f]+"$',
                             f'sha256: "{evidence["sha256"]}"', manifest.read_text(),
                             flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError("expected exactly one manifest sha256 field")
        manifest.write_text(text)
    report = json.dumps(evidence, indent=2)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(report + "\n")
    print(report)


if __name__ == "__main__":
    main()
