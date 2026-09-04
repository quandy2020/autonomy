# Copyright 2026 The Openbot Authors
"""Track network supporting simple terminal-state and planar MINCO heads."""

from __future__ import annotations

import torch
from torch import nn


class DepthBackbone(nn.Module):
    def __init__(self, out_channels: int = 64):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Conv2d(1, 32, 5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 128, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, out_channels, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
        )

    def forward(self, depth: torch.Tensor) -> torch.Tensor:
        return self.layers(depth)


class DualHead(nn.Module):
    def __init__(
        self,
        in_channels: int,
        trajectory_channel_count: int,
        objectness_channel_count: int = 1,
    ):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Conv2d(in_channels, 128, 1),
            nn.ReLU(inplace=True),
        )
        self.trajectory_head = nn.Conv2d(128, trajectory_channel_count, 1)
        self.objectness_head = nn.Conv2d(128, objectness_channel_count, 1)

    def forward(
        self, features: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        shared_features = self.shared(features)
        return (
            self.trajectory_head(shared_features),
            self.objectness_head(shared_features),
        )


class TrackNetwork(nn.Module):
    """
    trajectory_mode:
      simple: traj_channels=5 (4 state + score), objectness separate
      minco:  traj_channels=11 (10 MINCO params + score), objectness separate
    Exported ONNX concatenates traj + objectness → 6 or 12 channels.
    """

    def __init__(
        self,
        hidden: int = 64,
        observation_dim: int = 4,
        trajectory_mode: str = "simple",
    ):
        super().__init__()
        self.trajectory_mode = trajectory_mode
        self.traj_param_channels = 10 if trajectory_mode == "minco" else 4
        self.backbone = DepthBackbone(hidden)
        self.head = DualHead(
            hidden + observation_dim,
            trajectory_channel_count=self.traj_param_channels + 1,
        )
        self.pool = nn.AdaptiveAvgPool2d((1, 5))

    def set_grid(
        self, vertical_bin_count: int, horizontal_bin_count: int
    ) -> None:
        self.pool = nn.AdaptiveAvgPool2d(
            (vertical_bin_count, horizontal_bin_count)
        )

    def forward(
        self, depth: torch.Tensor, observation: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        features = self.backbone(depth)
        features = self.pool(features)
        if observation.shape[-2:] != features.shape[-2:]:
            observation = torch.nn.functional.interpolate(
                observation, size=features.shape[-2:], mode="nearest"
            )
        fused = torch.cat([features, observation], dim=1)
        trajectory_head, objectness_logits = self.head(fused)
        params_raw = trajectory_head[:, : self.traj_param_channels]
        score_logits = trajectory_head[:, self.traj_param_channels :]
        return params_raw, score_logits, objectness_logits
