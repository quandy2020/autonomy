#!/usr/bin/env python3
# Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Export a NeuPAN DUNE ObsPointNet checkpoint (.pth) to ONNX.

Default output: autonomy/control/conf/models/neupan_distance_encoder.onnx
(weights are gitignored; do not force-add).
"""

from __future__ import annotations

import argparse
import os

import torch
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

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.MLP(x)


def _state_dict(checkpoint: object) -> dict:
    if isinstance(checkpoint, dict):
        for key in ("state_dict", "model", "net"):
            value = checkpoint.get(key)
            if isinstance(value, dict):
                return value
        if all(isinstance(v, torch.Tensor) for v in checkpoint.values()):
            return checkpoint
    raise TypeError("unsupported checkpoint format")


def _normalize_keys(state: dict) -> dict:
    stripped = {
        (key[len("module.") :] if key.startswith("module.") else key): value
        for key, value in state.items()
    }
    if any(key.startswith("MLP.") for key in stripped):
        return stripped
    return {f"MLP.{key}": value for key, value in stripped.items()}


def _edge_count(state: dict) -> int:
    last = None
    for key, value in state.items():
        if key.endswith("weight") and value.ndim == 2:
            last = value
    if last is None:
        raise ValueError("no Linear weight in checkpoint")
    return int(last.shape[0])


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    models = os.path.normpath(
        os.path.join(here, "..", "..", "..", "conf", "models")
    )
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--checkpoint",
        default=os.path.join(models, "neupan_model_5000.pth"),
    )
    parser.add_argument("--edge-count", type=int, default=0)
    parser.add_argument(
        "--output",
        default=os.path.join(models, "neupan_distance_encoder.onnx"),
    )
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()

    try:
        raw = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    except TypeError:
        raw = torch.load(args.checkpoint, map_location="cpu")
    state = _normalize_keys(_state_dict(raw))
    edges = args.edge_count if args.edge_count > 0 else _edge_count(state)
    model = ObsPointNet(2, edges)
    model.load_state_dict(state)
    model.eval()

    dummy = torch.zeros(8, 2)
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    export_kw = dict(
        input_names=["points"],
        output_names=["mu"],
        dynamic_axes={"points": {0: "N"}, "mu": {0: "N"}},
        opset_version=args.opset,
    )
    try:
        torch.onnx.export(model, dummy, args.output, dynamo=False, **export_kw)
    except TypeError:
        torch.onnx.export(model, dummy, args.output, **export_kw)
    print(f"wrote {args.output} (edge-count={edges})")


if __name__ == "__main__":
    main()
