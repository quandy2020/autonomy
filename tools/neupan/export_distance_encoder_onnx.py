#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Export a NeuPAN ObsPointNet `.pth` checkpoint to ONNX for
# autonomy/control/controller/neupan_controller (DistanceEncoder).
#
# Usage (with NeuPAN + PyTorch installed):
#   python3 tools/neupan/export_distance_encoder_onnx.py \
#       --checkpoint /path/to/model_5000.pth \
#       --edge-count 4 \
#       --output /path/to/distance_encoder.onnx
#
# edge_count must match the robot footprint edge count used during training
# (rectangle → 4; custom polygon → number of sides).

"""Export NeuPAN ObsPointNet weights to ONNX for the distance encoder."""

from __future__ import annotations

import argparse
import sys


def build_model(edge_count: int):
    try:
        from neupan.blocks.obs_point_net import ObsPointNet
    except ImportError:
        import torch.nn as nn

        class ObsPointNet(nn.Module):
            def __init__(self, input_dim: int = 2, output_dim: int = 4) -> None:
                super().__init__()
                hidden_dim = 32
                self.MLP = nn.Sequential(
                    nn.Linear(input_dim, hidden_dim),
                    nn.LayerNorm(hidden_dim),
                    nn.Tanh(),
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.LayerNorm(hidden_dim),
                    nn.Tanh(),
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.LayerNorm(hidden_dim),
                    nn.Tanh(),
                    nn.Linear(hidden_dim, output_dim),
                    nn.ReLU(),
                )

            def forward(self, x):
                return self.MLP(x)

    return ObsPointNet(2, edge_count)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, help="Path to model_*.pth")
    parser.add_argument(
        "--edge-count",
        type=int,
        required=True,
        help="Output channels (= robot polygon edge count, usually 4)",
    )
    parser.add_argument("--output", required=True, help="Output .onnx path")
    parser.add_argument(
        "--opset", type=int, default=17, help="ONNX opset (default 17)"
    )
    args = parser.parse_args()

    import torch

    model = build_model(args.edge_count)
    state = torch.load(args.checkpoint, map_location="cpu")
    model.load_state_dict(state)
    model.eval()

    dummy = torch.zeros(1, 2, dtype=torch.float32)
    torch.onnx.export(
        model,
        dummy,
        args.output,
        input_names=["points"],
        output_names=["mu"],
        dynamic_axes={
            "points": {0: "num_points"},
            "mu": {0: "num_points"},
        },
        opset_version=args.opset,
    )
    print(f"Wrote {args.output} (edge_count={args.edge_count})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
