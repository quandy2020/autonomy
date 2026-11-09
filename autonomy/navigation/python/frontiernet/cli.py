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

"""Command line entry points for the FrontierNet / DeFM work.

``smoke``    build both models and run a random frame (no weights needed)
``baseline`` run the official checkpoint and store its output as an npz
``align``    run the DeFM variant with the official decoder/heads and diff it
             against the baseline
``export``   write the ONNX graph the C++ ``Engine`` runs (see export_onnx.py)
``collect``  capture depth/RGB frames from autolink for retraining
``label``    run the official checkpoint once to produce teacher labels
``train``    retrain the adapter + decoder + heads over a frozen DeFM encoder
``verify``   run checkpoint + ONNX + teacher on one frame and compare
"""

from __future__ import annotations

import argparse
import json
import os
from typing import Optional

import numpy as np

from frontiernet.postprocess import region_iou
from frontiernet.version import __version__


def _load_depth(path: str) -> np.ndarray:
    if path.endswith(".npy"):
        return np.load(path).astype(np.float32)
    from PIL import Image

    array = np.asarray(Image.open(path))
    if array.dtype == np.uint16:
        return array.astype(np.float32) * 0.001
    return array.astype(np.float32)


def _load_rgb(path: Optional[str]) -> Optional[np.ndarray]:
    if not path:
        return None
    from PIL import Image

    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"), dtype=np.float32) / 255.0


def _infer_in_channels(state) -> int:
    weight = state.get("encoder.conv1.weight")
    if weight is None:
        raise KeyError("checkpoint has no encoder.conv1.weight")
    return int(weight.shape[1])


def _official_model(weights: str, classes: int, rgb_available: bool):
    """Build the official model with the channel count the checkpoint expects."""
    from frontiernet.checkpoint import read_state_dict
    from frontiernet.unet import TwoHeadUnet

    state = read_state_dict(weights)
    in_channels = _infer_in_channels(state)
    if in_channels == 4 and not rgb_available:
        raise ValueError(f"{weights} expects RGB-D input (4 channels); pass --rgb")
    model = TwoHeadUnet(classes=classes, in_channels=in_channels)
    model.load_state_dict(state, strict=True)
    return model


def _predictor(model, args):
    from frontiernet.infer import FrontierPredictor

    return FrontierPredictor(
        model,
        device=args.device,
        input_size=args.input_size,
        classes=args.classes,
        df_normalizer=args.df_normalizer,
        df_threshold=args.df_threshold,
        scale_factor=args.scale,
    )


def _write(out_dir: str, name: str, result, extra: Optional[dict] = None) -> dict:
    os.makedirs(out_dir, exist_ok=True)
    result.save(os.path.join(out_dir, f"{name}.npz"))
    summary = result.summary()
    if extra:
        summary.update(extra)
    with open(os.path.join(out_dir, f"{name}.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
    return summary


def _synthetic_backbone():
    """Shape-only stand-in so ``smoke`` runs without the DeFM dependencies."""
    import torch.nn as nn
    import torch.nn.functional as F

    class SyntheticBackbone(nn.Module):
        embed_dim = 384

        def __init__(self) -> None:
            super().__init__()
            self.stem = nn.Conv2d(3, self.embed_dim, kernel_size=1)

        def forward(self, x):
            base = F.adaptive_avg_pool2d(self.stem(x), 16)
            return [base * (index + 1) for index in range(5)]

    return SyntheticBackbone()


def cmd_smoke(args: argparse.Namespace) -> int:
    import torch

    from frontiernet.model import FrontierNetDeFM
    from frontiernet.unet import TwoHeadUnet

    official = TwoHeadUnet(classes=args.classes, in_channels=4)
    backbone = _synthetic_backbone() if args.backbone == "synthetic" else None
    defm = FrontierNetDeFM(
        classes=args.classes, defm_weights=args.defm_weights, backbone=backbone
    )
    size = args.input_size
    with torch.no_grad():
        df_rgbd, masks_rgbd = official(torch.zeros(1, 4, size, size))
        df_defm, masks_defm = defm(torch.zeros(1, 3, size, size))
    report = {
        "version": __version__,
        "official": {
            "df": list(df_rgbd.shape),
            "masks": list(masks_rgbd.shape),
            "params": sum(p.numel() for p in official.parameters()),
        },
        "defm": {
            "df": list(df_defm.shape),
            "masks": list(masks_defm.shape),
            "params": defm.describe(),
        },
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


def cmd_baseline(args: argparse.Namespace) -> int:
    depth = _load_depth(args.depth)
    rgb = _load_rgb(args.rgb)
    model = _official_model(args.weights, args.classes, rgb is not None)
    result = _predictor(model, args).predict(rgb=rgb, depth=depth)
    summary = _write(args.out, args.name, result, {"weights": args.weights})
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def cmd_align(args: argparse.Namespace) -> int:
    depth = _load_depth(args.depth)
    rgb = _load_rgb(args.rgb)
    official = _official_model(args.weights, args.classes, rgb is not None)

    from frontiernet.checkpoint import load_frontiernet_heads
    from frontiernet.model import FrontierNetDeFM

    defm = FrontierNetDeFM(
        classes=args.classes,
        defm_weights=args.defm_weights,
        input_size=args.input_size,
    )
    report = load_frontiernet_heads(defm, args.weights)

    baseline = _predictor(official, args).predict(rgb=rgb, depth=depth)
    candidate = _predictor(defm, args).predict(depth=depth)
    _write(args.out, f"{args.name}_official", baseline, {"weights": args.weights})
    _write(
        args.out,
        f"{args.name}_defm",
        candidate,
        {"checkpoint": report.summary(), "defm_weights": args.defm_weights},
    )
    comparison = {
        "checkpoint": report.summary(),
        "region_iou": region_iou(baseline.region, candidate.region),
        "label_agreement": float(
            (baseline.labels == candidate.labels).mean() if baseline.labels.size else 0.0
        ),
        "df_mean_abs_delta": float(np.abs(baseline.df - candidate.df).mean()),
        "official_frontier_pixels": int(baseline.region.sum()),
        "defm_frontier_pixels": int(candidate.region.sum()),
        "note": "encoder differs by design; expect low agreement before head retraining",
    }
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, f"{args.name}_align.json"), "w", encoding="utf-8") as handle:
        json.dump(comparison, handle, indent=2, sort_keys=True)
    print(json.dumps(comparison, indent=2, sort_keys=True))
    return 0


def cmd_export(args: argparse.Namespace) -> int:
    """Forward to :mod:`frontiernet.export_onnx` so the flags stay in one place."""
    from frontiernet.export_onnx import main as export_main

    return int(export_main(args.export_args))


def cmd_collect(args: argparse.Namespace) -> int:
    from frontiernet.collect import main as collect_main

    return int(collect_main(args.collect_args))


def cmd_label(args: argparse.Namespace) -> int:
    from frontiernet.label import label_frames

    summary = label_frames(
        args.frames,
        args.out,
        args.weights,
        classes=args.classes,
        device=args.device,
        input_size=args.input_size,
        scale=args.scale,
        batch_size=args.batch_size,
        limit=args.limit,
        overwrite=args.overwrite,
        df_threshold_m=args.df_threshold,
        normalizer=args.df_normalizer,
        geometry=args.geometry,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def cmd_train(args: argparse.Namespace) -> int:
    from frontiernet.train import main as train_main

    return int(train_main(args.train_args))


def cmd_verify(args: argparse.Namespace) -> int:
    from frontiernet.verify import verify_frame

    report = verify_frame(
        args.frame,
        args.checkpoint,
        defm_weights=args.defm_weights,
        onnx_path=args.onnx,
        teacher_weights=args.teacher_weights,
        device=args.device,
        input_size=args.input_size,
        classes=args.classes,
        df_normalizer=args.df_normalizer,
        df_threshold=args.df_threshold,
        onnx_atol=args.onnx_atol,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="frontiernet", description=__doc__)
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="command", required=True)

    def common(target: argparse.ArgumentParser) -> None:
        target.add_argument("--classes", type=int, default=11)
        target.add_argument("--input-size", type=int, default=224)
        target.add_argument("--device", default="cpu")
        target.add_argument("--df-normalizer", type=float, default=10.0)
        target.add_argument("--df-threshold", type=float, default=0.5)
        target.add_argument("--scale", type=float, default=0.5)

    smoke = sub.add_parser("smoke", help="run both models on a random frame")
    common(smoke)
    smoke.add_argument("--defm-weights", default=None)
    smoke.add_argument(
        "--backbone",
        choices=("synthetic", "defm"),
        default="synthetic",
        help="synthetic skips the DeFM deps; defm builds the real ViT",
    )
    smoke.set_defaults(func=cmd_smoke)

    baseline = sub.add_parser("baseline", help="run the official checkpoint")
    common(baseline)
    baseline.add_argument("--weights", required=True)
    baseline.add_argument("--depth", required=True, help=".npy or 16-bit PNG depth")
    baseline.add_argument("--rgb", default=None)
    baseline.add_argument("--out", default="frontiernet_out")
    baseline.add_argument("--name", default="official")
    baseline.set_defaults(func=cmd_baseline)

    align = sub.add_parser("align", help="compare DeFM heads against the baseline")
    common(align)
    align.add_argument("--weights", required=True)
    align.add_argument("--defm-weights", default=None)
    align.add_argument("--depth", required=True)
    align.add_argument("--rgb", default=None)
    align.add_argument("--out", default="frontiernet_out")
    align.add_argument("--name", default="frame")
    align.set_defaults(func=cmd_align)

    export = sub.add_parser("export", help="export the ONNX graph for the C++ engine")
    export.add_argument(
        "export_args",
        nargs=argparse.REMAINDER,
        help="flags forwarded to frontiernet.export_onnx",
    )
    export.set_defaults(func=cmd_export)

    collect = sub.add_parser("collect", help="capture frames from autolink")
    collect.add_argument(
        "collect_args",
        nargs=argparse.REMAINDER,
        help="flags forwarded to frontiernet.collect",
    )
    collect.set_defaults(func=cmd_collect)

    label = sub.add_parser("label", help="generate teacher labels for captured frames")
    common(label)
    label.add_argument("--frames", required=True)
    label.add_argument("--out", required=True)
    label.add_argument("--weights", required=True)
    label.add_argument("--batch-size", type=int, default=8)
    label.add_argument("--limit", type=int, default=None)
    label.add_argument("--overwrite", action="store_true")
    label.add_argument("--geometry", choices=("aligned", "official"), default="aligned")
    label.set_defaults(func=cmd_label)

    train = sub.add_parser("train", help="retrain the DeFM adapter + heads")
    train.add_argument(
        "train_args",
        nargs=argparse.REMAINDER,
        help="flags forwarded to frontiernet.train",
    )
    train.set_defaults(func=cmd_train)

    verify = sub.add_parser(
        "verify", help="compare checkpoint, ONNX graph and teacher on one frame"
    )
    common(verify)
    verify.add_argument("--frame", required=True, help="frames/<idx>.npz")
    verify.add_argument("--checkpoint", required=True, help="trained best.pth")
    verify.add_argument("--defm-weights", default=None)
    verify.add_argument("--onnx", default=None, help="exported frontiernet_defm.onnx")
    verify.add_argument("--teacher-weights", default=None, help="rgbd_11cls.pth")
    verify.add_argument("--onnx-atol", type=float, default=1e-3)
    verify.set_defaults(func=cmd_verify)
    return parser


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
