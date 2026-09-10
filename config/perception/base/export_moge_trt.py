#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
"""Export MoGe-2 static ONNX (docs/onnx.md) then build TensorRT engine."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pretrained",
        default="Ruicheng/moge-2-vits-normal",
        help="HF id or local .pt path",
    )
    parser.add_argument("--height", type=int, default=518)
    parser.add_argument("--width", type=int, default=518)
    parser.add_argument("--num-tokens", type=int, default=1800)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(
            "/home/quandy/workspace/github/autonomy/src/autonomy/"
            "config/perception/base"
        ),
    )
    parser.add_argument(
        "--trtexec",
        default="/opt/TensorRT/bin/trtexec",
    )
    parser.add_argument("--skip-trt", action="store_true")
    args = parser.parse_args()

    os.environ["XFORMERS_DISABLED"] = "1"

    import torch
    import torch.nn.functional as F
    from moge.model.v2 import MoGeModel
    from moge.utils.geometry_torch import normalized_view_plane_uv

    num_tokens = args.num_tokens
    # Square static grid: √1800 ≈ 42 for the default 518×518 export.
    token_side = int(round(num_tokens**0.5))

    class MoGeStatic(MoGeModel):
        """Static MoGe forward without squeeze()-induced ONNX If nodes (TRT)."""

        def forward(self, image: torch.Tensor):  # type: ignore[override]
            batch_size, _, img_h, img_w = image.shape
            device, dtype = image.device, image.dtype
            base_h = base_w = token_side
            aspect_ratio = 1.0

            features, cls_token = self.encoder(
                image, base_h, base_w, return_class_token=True
            )
            features = [features, None, None, None, None]
            for level in range(5):
                uv = normalized_view_plane_uv(
                    width=base_w * 2**level,
                    height=base_h * 2**level,
                    aspect_ratio=aspect_ratio,
                    dtype=dtype,
                    device=device,
                )
                uv = uv.permute(2, 0, 1).unsqueeze(0).expand(
                    batch_size, -1, -1, -1
                )
                if features[level] is None:
                    features[level] = uv
                else:
                    features[level] = torch.concat(
                        [features[level], uv], dim=1
                    )

            features = self.neck(features)
            points = self.points_head(features)[-1]
            normal = self.normal_head(features)[-1]
            mask = self.mask_head(features)[-1]
            metric_scale = self.scale_head(cls_token)

            points, normal, mask = (
                F.interpolate(
                    v,
                    (img_h, img_w),
                    mode="bilinear",
                    align_corners=False,
                    antialias=False,
                )
                for v in (points, normal, mask)
            )
            points = self._remap_points(points.permute(0, 2, 3, 1))
            normal = F.normalize(normal.permute(0, 2, 3, 1), dim=-1)
            # Indexing (not squeeze) keeps a fixed [N,H,W] / [N] shape for TRT.
            mask = mask[:, 0].sigmoid()
            metric_scale = metric_scale[:, 0].exp()
            return points, normal, mask, metric_scale

    args.out_dir.mkdir(parents=True, exist_ok=True)
    onnx_path = args.out_dir / "moge-2-vits-normal_518x518.onnx"
    engine_path = args.out_dir / "moge-2-vits-normal_518x518_fp16.engine"

    print(f"Loading {args.pretrained} …", flush=True)
    model = MoGeStatic.from_pretrained(args.pretrained)
    model.onnx_compatible_mode = True
    model.eval()

    dummy = torch.rand(1, 3, args.height, args.width)
    print(f"Exporting static ONNX → {onnx_path}", flush=True)
    # Torch>=2.x defaults to dynamo exporter (needs onnxscript). Prefer legacy
    # TorchScript path matching MoGe docs/onnx.md.
    torch.onnx.export(
        model,
        (dummy,),
        str(onnx_path),
        input_names=["image"],
        output_names=["points", "normal", "mask", "metric_scale"],
        dynamic_axes=None,
        opset_version=14,
        dynamo=False,
    )
    print(f"ONNX size: {onnx_path.stat().st_size / 1e6:.1f} MB", flush=True)

    if args.skip_trt:
        return 0
    if not Path(args.trtexec).is_file():
        print(f"trtexec not found: {args.trtexec}", file=sys.stderr)
        return 1

    cmd = [
        args.trtexec,
        f"--onnx={onnx_path}",
        f"--saveEngine={engine_path}",
        "--fp16",
        "--memPoolSize=workspace:4096",
    ]
    print(" ".join(cmd), flush=True)
    subprocess.check_call(cmd)
    print(f"Engine → {engine_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
