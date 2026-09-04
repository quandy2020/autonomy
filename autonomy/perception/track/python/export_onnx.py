#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
"""Export the track network to ONNX (simple or planar MINCO)."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
import torch.nn as nn

from config import load_config
from models.network import TrackNetwork


class ExportWrapper(nn.Module):
    """Single output tensor `prediction` with layout [B,C,V,H], C=6 or 12."""

    def __init__(self, network: TrackNetwork):
        super().__init__()
        self.network = network

    def forward(
        self, depth: torch.Tensor, observation: torch.Tensor
    ) -> torch.Tensor:
        params_raw, score_logits, objectness_logits = self.network(
            depth, observation
        )
        return torch.cat(
            [params_raw, score_logits, objectness_logits], dim=1
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--checkpoint", type=str, default=None)
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument(
        "--trajectory-mode", type=str, default=None, choices=["simple", "minco"]
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_config(args.config)
    if args.trajectory_mode is not None:
        config["trajectory_mode"] = args.trajectory_mode
    trajectory_mode = str(config.get("trajectory_mode", "simple"))

    default_ckpt = (
        "checkpoints/yopo_track_minco_last.pt"
        if trajectory_mode == "minco"
        else "checkpoints/yopo_track_last.pt"
    )
    default_onnx = (
        "checkpoints/yopo_track_minco.onnx"
        if trajectory_mode == "minco"
        else "checkpoints/yopo_track.onnx"
    )
    checkpoint_path = Path(args.checkpoint or default_ckpt)
    output_path = Path(args.output or config.get("onnx_path", default_onnx))
    output_path.parent.mkdir(parents=True, exist_ok=True)

    network = TrackNetwork(
        hidden=int(config["hidden_dim"]),
        trajectory_mode=trajectory_mode,
    )
    network.set_grid(int(config["vertical_num"]), int(config["horizon_num"]))
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    network.load_state_dict(checkpoint["model"])
    network.eval()
    wrapped = ExportWrapper(network)

    vertical_bins = int(config["vertical_num"])
    horizontal_bins = int(config["horizon_num"])
    depth = torch.zeros(
        1, 1, int(config["image_height"]), int(config["image_width"])
    )
    observation = torch.zeros(1, 4, vertical_bins, horizontal_bins)

    torch.onnx.export(
        wrapped,
        (depth, observation),
        str(output_path),
        input_names=["depth", "observation"],
        output_names=["prediction"],
        opset_version=17,
        dynamic_axes={
            "depth": {0: "batch"},
            "observation": {0: "batch"},
            "prediction": {0: "batch"},
        },
    )
    channels = 12 if trajectory_mode == "minco" else 6
    print(
        f"exported ONNX → {output_path} (mode={trajectory_mode}, C={channels})"
    )


if __name__ == "__main__":
    main()
