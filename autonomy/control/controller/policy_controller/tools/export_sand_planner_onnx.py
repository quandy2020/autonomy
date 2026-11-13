#!/usr/bin/env python3
# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
"""Export SanD-Planner .pth as a single-step ONNX denoiser and compare runtimes."""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Tuple

import numpy as np
import torch
import torch.nn as nn


def _add_sandplanner_path(root: str) -> None:
    if not os.path.isdir(root):
        raise FileNotFoundError(
            f"SanD-Planner source not found: {root}. Clone "
            "https://github.com/WangJinCheng1998/sandplanner first."
        )
    if root not in sys.path:
        sys.path.insert(0, root)


def _axis_mid_half(stats: dict) -> Tuple[torch.Tensor, torch.Tensor]:
    mids = []
    halves = []
    for axis in ("x", "y", "z"):
        lo = float(stats["axis"][axis]["p1"])
        hi = float(stats["axis"][axis]["p99"])
        if hi <= lo:
            hi = lo + 1e-6
        mids.append((lo + hi) * 0.5)
        halves.append((hi - lo) * 0.5)
    return (
        torch.tensor(mids, dtype=torch.float32),
        torch.tensor(halves, dtype=torch.float32),
    )


class SandPlannerDenoiserStep(nn.Module):
    """Single denoising step matching policy_controller ONNX I/O names."""

    def __init__(
        self,
        encoder: nn.Module,
        unet: nn.Module,
        num_control_points: int,
        goal_mid: torch.Tensor,
        goal_half: torch.Tensor,
        train_timesteps: int,
    ) -> None:
        super().__init__()
        self.encoder = encoder
        self.unet = unet
        self.num_control_points = int(num_control_points)
        self.train_timesteps = int(max(1, train_timesteps))
        self.register_buffer("goal_mid", goal_mid.view(1, 3))
        self.register_buffer("goal_half", goal_half.view(1, 3))

    def forward(
        self,
        observation_depth: torch.Tensor,
        observation_goal: torch.Tensor,
        observation_previous_velocity: torch.Tensor,
        sample: torch.Tensor,
        timestep: torch.Tensor,
    ) -> torch.Tensor:
        batch = observation_depth.shape[0]
        depth = observation_depth.unsqueeze(2)
        goal_norm = torch.clamp(
            (observation_goal - self.goal_mid) / self.goal_half, -1.0, 1.0
        )
        turn = observation_previous_velocity[:, 1]
        has_turn = observation_previous_velocity.abs().sum(dim=1) > 1.0e-6
        hidden = self.encoder(
            depth,
            goal_norm,
            initial_turn=turn,
            has_initial_turn=has_turn,
        )
        control = sample.reshape(batch, self.num_control_points, 3).transpose(1, 2)
        time_float = timestep.reshape(-1).to(dtype=observation_depth.dtype)
        scaled = torch.where(
            time_float <= 1.0,
            time_float * float(self.train_timesteps - 1),
            time_float,
        )
        time_index = scaled.round().clamp(0, self.train_timesteps - 1).to(
            dtype=torch.long
        )
        unet_out = self.unet(
            sample=control,
            timestep=time_index,
            encoder_hidden_states=hidden.to(dtype=control.dtype),
        )
        velocity = unet_out.sample
        return velocity.transpose(1, 2).reshape(batch, self.num_control_points * 3)


def load_official_model(source_root: str, checkpoint_path: str, stats_path: str, device: str):
    _add_sandplanner_path(source_root)
    from sand_planner.core.model_manager import ModelManager
    from sand_planner.config import InferenceConfig

    config = InferenceConfig(
        checkpoint_path=checkpoint_path,
        stats_path=stats_path,
        device=device,
        base_dir=source_root,
    )
    # InferenceConfig joins relative paths onto base_dir; restore absolute paths.
    config.checkpoint_path = os.path.abspath(checkpoint_path)
    config.stats_path = os.path.abspath(stats_path)
    manager = ModelManager(config)
    model, _normalizer = manager.load_model()
    model.eval()
    if hasattr(model.unet, "_orig_mod"):
        model.unet = model.unet._orig_mod
    if hasattr(model.condition_encoder, "_orig_mod"):
        model.condition_encoder = model.condition_encoder._orig_mod
    _replace_adaptive_pool(model)
    return model, int(getattr(model, "num_control_points", 8))


class _FixedSizeAvgPool(nn.Module):
    """Bilinear resize, because AdaptiveAvgPool2d 11x14 -> 8x12 does not export."""

    def forward(self, hidden: torch.Tensor) -> torch.Tensor:
        return torch.nn.functional.interpolate(
            hidden, size=(8, 12), mode="bilinear", align_corners=False
        )


def _replace_adaptive_pool(model: nn.Module) -> None:
    encoder = model.condition_encoder.depth_encoder
    encoder.adaptive_pool = _FixedSizeAvgPool()


def build_step_module(model, stats_path: str) -> SandPlannerDenoiserStep:
    with open(stats_path, "r", encoding="utf-8") as handle:
        stats = json.load(handle)
    mid, half = _axis_mid_half(stats)
    train_timesteps = int(getattr(model, "num_train_timesteps", 1000))
    step = SandPlannerDenoiserStep(
        encoder=model.condition_encoder,
        unet=model.unet,
        num_control_points=int(model.num_control_points),
        goal_mid=mid,
        goal_half=half,
        train_timesteps=train_timesteps,
    )
    step.eval()
    return step


def dummy_inputs(num_control_points: int, device: torch.device):
    depth = torch.rand(1, 4, 168, 224, device=device) * 8.0
    goal = torch.tensor([[4.0, 1.0, 0.0]], device=device)
    velocity = torch.zeros(1, 3, device=device)
    sample = torch.randn(1, num_control_points * 3, device=device)
    sample[:, 0:3] = 0.0
    timestep = torch.tensor([500.0], device=device)
    return depth, goal, velocity, sample, timestep


def export_onnx(step: nn.Module, onnx_path: str, inputs) -> None:
    os.makedirs(os.path.dirname(onnx_path) or ".", exist_ok=True)
    names = (
        "observation_depth",
        "observation_goal",
        "observation_previous_velocity",
        "sample",
        "timestep",
    )
    kwargs = dict(
        input_names=list(names),
        output_names=["action"],
        opset_version=17,
        do_constant_folding=True,
    )
    try:
        torch.onnx.export(step, inputs, onnx_path, dynamo=False, **kwargs)
        return
    except TypeError:
        pass
    except Exception as error:
        print(f"legacy onnx export failed: {error}")
    torch.onnx.export(step, inputs, onnx_path, dynamo=True, **kwargs)


def verify(step: nn.Module, onnx_path: str, inputs, atol: float) -> None:
    import onnxruntime as ort

    with torch.no_grad():
        pytorch = step(*inputs).detach().cpu().numpy()
    session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    feed = {
        "observation_depth": inputs[0].detach().cpu().numpy().astype(np.float32),
        "observation_goal": inputs[1].detach().cpu().numpy().astype(np.float32),
        "observation_previous_velocity": inputs[2]
        .detach()
        .cpu()
        .numpy()
        .astype(np.float32),
        "sample": inputs[3].detach().cpu().numpy().astype(np.float32),
        "timestep": inputs[4].detach().cpu().numpy().astype(np.float32),
    }
    onnx_out = session.run(["action"], feed)[0]
    max_abs = float(np.max(np.abs(pytorch - onnx_out)))
    mean_abs = float(np.mean(np.abs(pytorch - onnx_out)))
    print(f"pytorch_shape={pytorch.shape} onnx_shape={onnx_out.shape}")
    print(f"max_abs_diff={max_abs:.6e} mean_abs_diff={mean_abs:.6e}")
    if not np.isfinite(onnx_out).all():
        raise RuntimeError("ONNX output contains non-finite values")
    if max_abs > atol:
        raise RuntimeError(f"ONNX mismatch exceeds atol={atol}: max_abs={max_abs}")
    print("VERIFY_OK")


def parse_args() -> argparse.Namespace:
    default_root = os.environ.get(
        "SAND_PLANNER_SRC", "/tmp/sandplanner_src/sandplanner"
    )
    default_models = os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "conf",
        "models",
    )
    default_models = os.path.abspath(default_models)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default=default_root)
    parser.add_argument(
        "--checkpoint",
        default=os.path.join(default_models, "NoMax.pth"),
    )
    parser.add_argument(
        "--stats",
        default=os.path.join(default_models, "sand_planner_trajectory_stats.json"),
    )
    parser.add_argument(
        "--onnx",
        default=os.path.join(default_models, "sand_planner_nomax_denoiser_step.onnx"),
    )
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--atol", type=float, default=2.0e-4)
    parser.add_argument("--skip-export", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not os.path.isfile(args.checkpoint):
        raise FileNotFoundError(args.checkpoint)
    device = args.device
    if device.startswith("cuda") and not torch.cuda.is_available():
        device = "cpu"
    print(f"loading checkpoint={args.checkpoint} device={device}")
    model, num_cp = load_official_model(
        args.source_root, args.checkpoint, args.stats, device
    )
    step = build_step_module(model, args.stats).to(device)
    inputs = dummy_inputs(num_cp, torch.device(device))
    if not args.skip_export:
        print(f"exporting {args.onnx}")
        cpu_step = step.to("cpu")
        cpu_inputs = tuple(tensor.detach().cpu() for tensor in inputs)
        export_onnx(cpu_step, args.onnx, cpu_inputs)
        step = cpu_step
        inputs = cpu_inputs
    print(f"verifying {args.onnx}")
    verify(step, args.onnx, inputs, args.atol)
    print(f"onnx={args.onnx}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
