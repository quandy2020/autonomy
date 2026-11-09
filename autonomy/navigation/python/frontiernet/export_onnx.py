# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Export a FrontierNet variant to ONNX for the C++ ``Engine``.

The exported graph is static::

    depth [1, C, 224, 224] -> df [1, 1, 224, 224], masks [1, 11, 224, 224]

``C`` is 3 for ``--variant defm`` (DeFM-normalized depth, the runtime path) and
4 for ``--variant official`` (RGB-D, needs ``rgbd_11cls.pth``).

Both outputs keep the official semantics: ``df`` is ``-log(d / normalizer)`` and
``masks`` are the 11 gain-bin logits, so the C++ side only needs the same
threshold/argmax decoding as :mod:`frontiernet.postprocess`.

Usage (inside the SpaceHero container)::

    /opt/venv/bin/python -m frontiernet.export_onnx \
        --variant defm --defm-weights conf/models/defm_vit_s14.pth \
        --out conf/models/frontiernet_defm.onnx --validate \
        --manifest conf/models/frontiernet_defm.pb.txt --report /tmp/frontiernet-export.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Optional, Sequence

import torch

from frontiernet.model import FrontierNetDeFM
from frontiernet.unet import TwoHeadUnet

__all__ = ["build_model", "export", "main"]

DEFM_MEAN = (0.248880, 0.495620, 0.492858)
DEFM_STD = (0.139357, 0.271314, 0.297177)


def _official_in_channels(weights: str) -> int:
    from frontiernet.checkpoint import read_state_dict

    state = read_state_dict(weights)
    weight = state.get("encoder.conv1.weight")
    if weight is None:
        raise KeyError(f"{weights} has no encoder.conv1.weight")
    return int(weight.shape[1])


def build_model(args: argparse.Namespace) -> torch.nn.Module:
    """Return the model described by the command line, in eval mode."""
    if args.variant == "defm":
        from frontiernet.checkpoint import resolve_weights

        model = FrontierNetDeFM(
            classes=args.classes,
            defm_weights=resolve_weights(args.defm_weights) if args.defm_weights else None,
            input_size=args.input_size,
        )
        if args.weights:
            from frontiernet.checkpoint import (
                load_frontiernet_heads,
                read_state_dict,
                resolve_weights,
            )

            resolved = resolve_weights(args.weights)
            if args.load == "full":
                state = read_state_dict(resolved)
                model.load_state_dict(state, strict=True)
                print(f"loaded full state dict ({len(state)} tensors)", flush=True)
            else:
                report = load_frontiernet_heads(model, resolved)
                report.raise_for_errors()
                print(report.summary(), flush=True)
        return model.eval()
    if not args.weights:
        raise ValueError("--variant official needs --weights (rgbd_11cls.pth)")
    state = None
    from frontiernet.checkpoint import read_state_dict

    state = read_state_dict(args.weights)
    model = TwoHeadUnet(classes=args.classes, in_channels=_official_in_channels(args.weights))
    model.load_state_dict(state, strict=True)
    return model.eval()


def _manifest_text(name: str, path: str, digest: str, in_channels: int,
                   input_size: int, classes: int) -> str:
    mean = DEFM_MEAN if in_channels == 3 else (0.0, 0.0, 0.0, 0.0)
    std = DEFM_STD if in_channels == 3 else (1.0, 1.0, 1.0, 1.0)
    lines = [
        f'# FrontierNet {"DeFM" if in_channels == 3 else "official RGB-D"} encoder, '
        "exported by python/frontiernet/export_onnx.py",
        f'name: "{name}"',
        f'path: "{path}"',
        f'sha256: "{digest}"',
        'opset: "17"',
        'backend: "onnx"',
        'precision: "fp32"',
        "inputs {",
        '  name: "depth"',
        f"  shape: [1, {in_channels}, {input_size}, {input_size}]",
        '  dtype: "float32"',
        "}",
        "outputs {",
        '  name: "df"',
        f"  shape: [1, 1, {input_size}, {input_size}]",
        '  dtype: "float32"',
        "}",
        "outputs {",
        '  name: "masks"',
        f"  shape: [1, {classes}, {input_size}, {input_size}]",
        '  dtype: "float32"',
        "}",
        "mean: [" + ", ".join(f"{value:.6f}" for value in mean) + "]",
        "std: [" + ", ".join(f"{value:.6f}" for value in std) + "]",
        "",
    ]
    return "\n".join(lines)


def export(args: argparse.Namespace) -> dict:
    """Export and optionally validate; returns the evidence report."""
    torch.set_num_threads(args.threads)
    torch.manual_seed(0)
    model = build_model(args)
    in_channels = int(getattr(model, "n_channels", 4))
    output = Path(args.out).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".onnx.tmp")
    example = torch.zeros(1, in_channels, args.input_size, args.input_size)
    torch.onnx.export(
        model,
        example,
        str(temporary),
        opset_version=17,
        input_names=["depth"],
        output_names=["df", "masks"],
        do_constant_folding=True,
        dynamo=False,
    )
    evidence: dict = {
        "variant": args.variant,
        "torch": torch.__version__,
        "opset": 17,
        "input_shape": list(example.shape),
        "cases": {},
    }
    if args.validate:
        import numpy as np
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = args.threads
        session = ort.InferenceSession(
            str(temporary), sess_options=options, providers=["CPUExecutionProvider"]
        )
        evidence["onnxruntime"] = ort.__version__
        evidence["inputs"] = [
            {"name": item.name, "shape": list(item.shape)} for item in session.get_inputs()
        ]
        evidence["outputs"] = [
            {"name": item.name, "shape": list(item.shape)} for item in session.get_outputs()
        ]
        samples = {
            "zeros": torch.zeros(1, in_channels, args.input_size, args.input_size),
            "random": torch.randn(1, in_channels, args.input_size, args.input_size),
            "ramp": torch.linspace(
                -2, 2, in_channels * args.input_size * args.input_size
            ).reshape(1, in_channels, args.input_size, args.input_size),
        }
        expected_shapes = {
            "df": (1, 1, args.input_size, args.input_size),
            "masks": (1, args.classes, args.input_size, args.input_size),
        }
        for name, sample in samples.items():
            with torch.inference_mode():
                expected = [value.numpy() for value in model(sample)]
            actual = session.run(["df", "masks"], {"depth": sample.numpy()})
            errors = {}
            for tensor_name, reference, result in zip(["df", "masks"], expected, actual):
                if tuple(result.shape) != expected_shapes[tensor_name]:
                    raise RuntimeError(
                        f"{name}/{tensor_name}: shape {result.shape} != "
                        f"{expected_shapes[tensor_name]}"
                    )
                if not np.isfinite(result).all():
                    raise RuntimeError(f"{name}/{tensor_name}: non-finite output")
                error = float(np.max(np.abs(reference - result)))
                if error >= args.pytorch_atol:
                    raise RuntimeError(
                        f"{name}/{tensor_name}: max error {error} >= {args.pytorch_atol}"
                    )
                errors[tensor_name] = error
            evidence["cases"][name] = errors
        evidence["pytorch_atol"] = args.pytorch_atol
    temporary.replace(output)
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    evidence["path"] = str(output)
    evidence["bytes"] = output.stat().st_size
    evidence["sha256"] = digest
    if args.manifest:
        manifest = Path(args.manifest)
        manifest.parent.mkdir(parents=True, exist_ok=True)
        manifest.write_text(
            _manifest_text(
                manifest.stem,
                args.manifest_model_path or str(output),
                digest,
                in_channels,
                args.input_size,
                args.classes,
            ),
            encoding="utf-8",
        )
        evidence["manifest"] = str(manifest)
    return evidence


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="frontiernet.export_onnx", description=__doc__)
    parser.add_argument("--variant", choices=("defm", "official"), default="defm")
    parser.add_argument("--classes", type=int, default=11)
    parser.add_argument("--input-size", type=int, default=224)
    parser.add_argument("--weights", default=None, help="rgbd_11cls.pth (official heads)")
    parser.add_argument(
        "--load",
        choices=("heads", "full"),
        default="heads",
        help="heads: transplant decoder+heads only; full: strict load of a "
        "checkpoint written by frontiernet.train",
    )
    parser.add_argument("--defm-weights", default=None, help="defm_vit_s14.pth")
    parser.add_argument("--out", required=True)
    parser.add_argument("--manifest", default=None)
    parser.add_argument(
        "--manifest-model-path",
        default=None,
        help="path recorded in the manifest; defaults to --out",
    )
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--validate", action="store_true")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--pytorch-atol", type=float, default=5e-4)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    evidence = export(args)
    report = json.dumps(evidence, indent=2, sort_keys=True)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(report + "\n", encoding="utf-8")
    print(report)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
