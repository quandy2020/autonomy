#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Export a small Twist policy (toy or custom torch.nn.Module) to ONNX and
# optionally emit a matching PolicyConfig textproto.
#
# Usage (requires torch):
#   python3 tools/policy/export_policy_onnx.py --toy \
#       --output autonomy/control/conf/models/policy_toy.onnx \
#       --sidecar autonomy/control/conf/policy_config_toy_example.pb.txt
#
# Custom module:
#   python3 tools/policy/export_policy_onnx.py \
#       --checkpoint policy.pt --module-path my_pkg.policy:PolicyNet \
#       --output /path/to/policy.onnx

"""Export Twist policy graphs for policy_controller."""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path


def build_toy(kinematic_dim: int = 6, path_dim: int = 40, action_dim: int = 2):
    import torch
    import torch.nn as nn

    class ToyTwistPolicy(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(kinematic_dim + path_dim, 64),
                nn.ReLU(),
                nn.Linear(64, action_dim),
                nn.Tanh(),
            )
            self._k = kinematic_dim
            self._p = path_dim

        def forward(self, observation_kinematic, observation_path):
            # Accept NCHW-ish leading batch dims flattened by view.
            k = observation_kinematic.reshape(observation_kinematic.shape[0], -1)
            p = observation_path.reshape(observation_path.shape[0], -1)
            out = self.net(torch.cat([k, p], dim=-1))
            # Scale toy output into mild cmd range.
            scale = out.new_tensor([0.5, 1.0][: out.shape[-1]])
            return out * scale

    return ToyTwistPolicy()


def load_module(spec: str, checkpoint: str | None):
    import torch

    if ":" not in spec:
        raise SystemExit("--module-path must be package.module:ClassName")
    mod_name, cls_name = spec.split(":", 1)
    mod = importlib.import_module(mod_name)
    cls = getattr(mod, cls_name)
    model = cls()
    if checkpoint:
        state = torch.load(checkpoint, map_location="cpu")
        if isinstance(state, dict) and "state_dict" in state:
            state = state["state_dict"]
        model.load_state_dict(state)
    return model


def write_sidecar(path: Path, onnx_path: Path, path_points: int) -> None:
    text = f"""# Auto-generated toy PolicyConfig for policy_controller smoke tests.
# Regenerate: python3 tools/policy/export_policy_onnx.py --toy \\
#   --output {onnx_path} --sidecar {path}
name: "toy_twist_policy"
version: "0.1"

observation {{
  kinematic: true
  path: true
}}

action {{
  limits {{
    max_vx: 0.5
    max_wz: 1.0
    max_ax: 1.0
    max_alpha_z: 2.0
  }}
}}

engine {{
  model_path: "{onnx_path.as_posix()}"
}}
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print(f"Wrote sidecar: {path} (path_points expect {path_points})")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toy", action="store_true", help="Export built-in MLP")
    parser.add_argument("--checkpoint", default="", help="Optional .pt weights")
    parser.add_argument(
        "--module-path",
        default="",
        help="package.module:ClassName for a custom torch.nn.Module",
    )
    parser.add_argument("--output", required=True, help="Output .onnx path")
    parser.add_argument("--sidecar", default="", help="Optional PolicyConfig .pb.txt")
    parser.add_argument("--path-points", type=int, default=20)
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()

    try:
        import torch
    except ImportError as exc:
        print("torch is required: pip install torch", file=sys.stderr)
        raise SystemExit(2) from exc

    path_dim = max(1, args.path_points) * 2
    if args.toy:
        model = build_toy(path_dim=path_dim)
    elif args.module_path:
        model = load_module(args.module_path, args.checkpoint or None)
    else:
        print("Specify --toy or --module-path", file=sys.stderr)
        return 2

    model.eval()
    kinematic = torch.zeros(1, 6, dtype=torch.float32)
    path = torch.zeros(1, path_dim, dtype=torch.float32)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    torch.onnx.export(
        model,
        (kinematic, path),
        str(out_path),
        input_names=["observation_kinematic", "observation_path"],
        output_names=["action"],
        dynamic_axes=None,
        opset_version=args.opset,
    )
    print(f"Wrote ONNX: {out_path}")

    if args.sidecar:
        write_sidecar(Path(args.sidecar), out_path.resolve(), args.path_points)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
