# Copyright 2026 The Openbot Authors
"""Planar yaw×range lattice for ground-robot tracking (mirrors C++ Lattice)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch


@dataclass
class LatticeConfig:
    horizontal_bin_count: int = 5
    vertical_bin_count: int = 1
    camera_horizontal_field_of_view_deg: float = 90.0
    planning_horizon_m: float = 3.0
    max_linear_velocity_mps: float = 0.5
    max_yaw_rate_rps: float = 1.0


class Lattice:
    def __init__(
        self, config: LatticeConfig, device: torch.device | None = None
    ):
        self.config = config
        self.device = device or torch.device("cpu")
        self.lattice_size = (
            config.horizontal_bin_count * config.vertical_bin_count
        )
        field_of_view_rad = math.radians(
            config.camera_horizontal_field_of_view_deg
        )
        yaw_step = (
            field_of_view_rad / config.horizontal_bin_count
            if config.horizontal_bin_count > 1
            else 0.0
        )
        self.yaw_bin_half_width_rad = (
            0.5 * yaw_step
            if config.horizontal_bin_count > 1
            else 0.5 * field_of_view_rad
        )

        yaw_values = []
        for horizontal_index in range(config.horizontal_bin_count):
            yaw = (
                -0.5 * yaw_step * (config.horizontal_bin_count - 1)
                + horizontal_index * yaw_step
            )
            yaw_values.append(yaw)
        self.yaw = torch.tensor(
            yaw_values, dtype=torch.float32, device=self.device
        )
        self.anchor_position = torch.stack(
            [
                torch.cos(self.yaw) * config.planning_horizon_m,
                torch.sin(self.yaw) * config.planning_horizon_m,
            ],
            dim=-1,
        )

    def decode_terminal_state(self, prediction: torch.Tensor) -> torch.Tensor:
        """
        prediction: [B, 4, V, H] → tanh-normalized
          (yaw_offset, range_offset, linear_velocity, yaw_rate)
        returns terminal state [B, 4, V, H] as
          (x, y, linear_velocity, yaw_rate) in body frame.
        """
        batch_size, _, vertical_bins, horizontal_bins = prediction.shape
        flat = prediction.permute(0, 2, 3, 1).reshape(
            batch_size, vertical_bins * horizontal_bins, 4
        )
        yaw_offset = flat[..., 0] * self.yaw_bin_half_width_rad
        range_offset = flat[..., 1]
        linear_velocity = (
            flat[..., 2] * self.config.max_linear_velocity_mps
        )
        yaw_rate = flat[..., 3] * self.config.max_yaw_rate_rps

        yaw = self.yaw.view(1, -1).expand(batch_size, -1)
        # Match C++ Lattice::RebuildAnchors / DecodeTerminalState order:
        # horizontal index 0 is the left-most (most negative) yaw bin.
        if vertical_bins * horizontal_bins != yaw.shape[1]:
            yaw = self.yaw.view(1, -1).expand(batch_size, -1)

        range_m = (range_offset + 1.0) * self.config.planning_horizon_m
        terminal_x = torch.cos(yaw + yaw_offset) * range_m
        terminal_y = torch.sin(yaw + yaw_offset) * range_m
        stacked = torch.stack(
            [terminal_x, terminal_y, linear_velocity, yaw_rate], dim=-1
        )
        return stacked.permute(0, 2, 1).reshape(
            batch_size, 4, vertical_bins, horizontal_bins
        )
