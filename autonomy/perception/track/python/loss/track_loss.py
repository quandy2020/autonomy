# Copyright 2026 The Openbot Authors
"""Guidance-style losses for simple and planar-MINCO track modes."""

from __future__ import annotations

import math

import torch
from torch import nn

from minco import MincoS3NUPlanar
from primitive import Lattice


class TrackLoss(nn.Module):
    def __init__(self, config: dict, lattice: Lattice):
        super().__init__()
        self.guidance_weight = float(config["wg"])
        self.smoothness_weight = float(config["ws"])
        self.collision_weight = float(config["wc"])
        self.objectness_weight = float(config["wt"])
        self.score_weight = float(config["w_score"])
        self.desired_standoff_m = float(config["follow_distance"])
        self.planning_horizon_m = float(config["radio_range"])
        self.objectness_angle_rad = (
            float(config.get("nms_angle_deg", 15.0)) * math.pi / 180.0
        )
        self.trajectory_mode = str(config.get("trajectory_mode", "simple"))
        self.piece_duration_s = float(config.get("minco_piece_duration_s", 1.0))
        self.vel_max = float(config["vel_max"])
        self.acc_max = float(config.get("acc_max", 1.0))
        self.lattice = lattice
        self.minco = MincoS3NUPlanar(piece_num=2, device=lattice.device)

    def forward(
        self,
        params_raw: torch.Tensor,
        score_logits: torch.Tensor,
        objectness_logits: torch.Tensor,
        target_xy_m: torch.Tensor,
        occupancy_map: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        if self.trajectory_mode == "minco":
            return self._forward_minco(
                params_raw,
                score_logits,
                objectness_logits,
                target_xy_m,
                occupancy_map,
            )
        return self._forward_simple(
            params_raw,
            score_logits,
            objectness_logits,
            target_xy_m,
            occupancy_map,
        )

    def _objectness_and_score(
        self,
        terminal_xy: torch.Tensor,  # (B,N,2)
        score_logits: torch.Tensor,
        objectness_logits: torch.Tensor,
        target_xy_m: torch.Tensor,
        trajectory_cost: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        batch_size, lattice_size, _ = terminal_xy.shape
        score = torch.nn.functional.softplus(score_logits).reshape(
            batch_size, lattice_size
        )
        score_loss = torch.nn.functional.smooth_l1_loss(
            score, trajectory_cost.detach()
        )
        flat_xy = terminal_xy.reshape(batch_size * lattice_size, 2)
        cell_yaw = torch.atan2(flat_xy[:, 1], flat_xy[:, 0]).reshape(
            batch_size, lattice_size
        )
        target_yaw = torch.atan2(
            target_xy_m[:, 1], target_xy_m[:, 0]
        ).unsqueeze(1)
        angle_error = (cell_yaw - target_yaw).abs()
        angle_error = torch.minimum(angle_error, 2 * math.pi - angle_error)
        labels = (angle_error < self.objectness_angle_rad).float()
        target_range = torch.linalg.norm(
            target_xy_m, dim=-1, keepdim=True
        ).clamp_min(1e-3)
        in_range = (
            (target_range.squeeze(-1) < 2.0 * self.planning_horizon_m)
            .float()
            .unsqueeze(1)
        )
        labels = labels * in_range
        objectness_loss = torch.nn.functional.binary_cross_entropy_with_logits(
            objectness_logits.reshape(batch_size, lattice_size), labels
        )
        return score_loss, objectness_loss, labels

    def _forward_simple(
        self,
        params_raw: torch.Tensor,
        score_logits: torch.Tensor,
        objectness_logits: torch.Tensor,
        target_xy_m: torch.Tensor,
        occupancy_map: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        terminal_state = self.lattice.decode_terminal_state(
            torch.tanh(params_raw)
        )
        batch_size, _, vertical_bins, horizontal_bins = terminal_state.shape
        lattice_size = vertical_bins * horizontal_bins
        xy = (
            terminal_state[:, 0:2]
            .permute(0, 2, 3, 1)
            .reshape(batch_size, lattice_size, 2)
        )
        linear_velocity = terminal_state[:, 2].reshape(batch_size, lattice_size)
        yaw_rate = terminal_state[:, 3].reshape(batch_size, lattice_size)
        smoothness = linear_velocity.square() + yaw_rate.square()

        target_range_m = torch.linalg.norm(
            target_xy_m, dim=-1, keepdim=True
        ).clamp_min(1e-3)
        desired_xy = target_xy_m * (self.desired_standoff_m / target_range_m)
        desired = desired_xy.unsqueeze(1).expand(batch_size, lattice_size, 2)
        guidance = torch.linalg.norm(xy - desired, dim=-1)

        flat = xy.reshape(batch_size * lattice_size, 2)
        u = (flat[:, 0] / (2.0 * self.planning_horizon_m) + 0.5).clamp(0, 1)
        v = (flat[:, 1] / (2.0 * self.planning_horizon_m) + 0.5).clamp(0, 1)
        grid = torch.stack([u * 2 - 1, v * 2 - 1], dim=-1).view(
            batch_size, lattice_size, 1, 2
        )
        safety = torch.nn.functional.grid_sample(
            occupancy_map, grid, align_corners=True, padding_mode="border"
        ).reshape(batch_size, lattice_size)

        trajectory_cost = (
            self.smoothness_weight * smoothness
            + self.collision_weight * safety
            + self.guidance_weight * guidance
        )
        score_loss, objectness_loss, labels = self._objectness_and_score(
            xy, score_logits, objectness_logits, target_xy_m, trajectory_cost
        )
        positive_mask = labels > 0.5
        trajectory_loss = (
            trajectory_cost[positive_mask].mean()
            if positive_mask.any()
            else trajectory_cost.mean()
        )
        total = (
            trajectory_loss
            + self.score_weight * score_loss
            + self.objectness_weight * objectness_loss
        )
        return {
            "total": total,
            "trajectory": trajectory_loss.detach(),
            "score": score_loss.detach(),
            "objectness": objectness_loss.detach(),
            "cost": trajectory_cost.detach(),
        }

    def _forward_minco(
        self,
        params_raw: torch.Tensor,
        score_logits: torch.Tensor,
        objectness_logits: torch.Tensor,
        target_xy_m: torch.Tensor,
        occupancy_map: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        params = torch.tanh(params_raw)
        batch_size, _, vertical_bins, horizontal_bins = params.shape
        lattice_size = vertical_bins * horizontal_bins
        flat = params.permute(0, 2, 3, 1).reshape(batch_size * lattice_size, 10)

        # Explicit per-cell decode using lattice math on tensors.
        yaw = self.lattice.yaw.view(1, -1).expand(batch_size, -1)
        if yaw.shape[1] != lattice_size:
            yaw = self.lattice.yaw.view(1, -1).repeat(
                1, lattice_size // max(self.lattice.yaw.numel(), 1)
            )[:, :lattice_size]
            yaw = yaw.expand(batch_size, -1)

        half = self.lattice.yaw_bin_half_width_rad
        horizon = self.lattice.config.planning_horizon_m
        inner_yaw = yaw + flat[:, 0].view(batch_size, lattice_size) * half
        inner_range = (flat[:, 1].view(batch_size, lattice_size) + 1.0) * horizon
        tail_yaw = yaw + flat[:, 2].view(batch_size, lattice_size) * half
        tail_range = (flat[:, 3].view(batch_size, lattice_size) + 1.0) * horizon
        inner = torch.stack(
            [torch.cos(inner_yaw) * inner_range, torch.sin(inner_yaw) * inner_range],
            dim=-1,
        )
        tail = torch.stack(
            [torch.cos(tail_yaw) * tail_range, torch.sin(tail_yaw) * tail_range],
            dim=-1,
        )
        tail_vel = flat[:, 4:6].view(batch_size, lattice_size, 2) * self.vel_max
        tail_acc = flat[:, 6:8].view(batch_size, lattice_size, 2) * self.acc_max
        durations = (
            flat[:, 8:10].view(batch_size, lattice_size, 2) + 1.0
        ) * self.piece_duration_s
        durations = durations.clamp_min(0.1)

        head = torch.zeros(
            batch_size * lattice_size, 3, 2, device=params.device, dtype=params.dtype
        )
        head[:, 1, 0] = 0.2 * self.vel_max  # mild forward prior
        tail_pva = torch.zeros(
            batch_size * lattice_size, 3, 2, device=params.device, dtype=params.dtype
        )
        tail_pva[:, 0] = tail.reshape(batch_size * lattice_size, 2)
        tail_pva[:, 1] = tail_vel.reshape(batch_size * lattice_size, 2)
        tail_pva[:, 2] = tail_acc.reshape(batch_size * lattice_size, 2)
        inner_pts = inner.reshape(batch_size * lattice_size, 1, 2)
        self.minco.set_parameters(
            head,
            tail_pva,
            inner_pts,
            durations.reshape(batch_size * lattice_size, 2),
        )
        samples = self.minco.sample_positions(8)  # (BN, S, 2)
        sample_count = samples.shape[1]
        flat_samples = samples.reshape(batch_size * lattice_size * sample_count, 2)
        u = (
            flat_samples[:, 0] / (2.0 * self.planning_horizon_m) + 0.5
        ).clamp(0, 1)
        v = (
            flat_samples[:, 1] / (2.0 * self.planning_horizon_m) + 0.5
        ).clamp(0, 1)
        grid = torch.stack([u * 2 - 1, v * 2 - 1], dim=-1).view(
            batch_size * lattice_size, sample_count, 1, 2
        )
        occ = occupancy_map.repeat_interleave(lattice_size, dim=0)
        safety = (
            torch.nn.functional.grid_sample(
                occ, grid, align_corners=True, padding_mode="border"
            )
            .reshape(batch_size, lattice_size, sample_count)
            .mean(dim=-1)
        )

        target_range_m = torch.linalg.norm(
            target_xy_m, dim=-1, keepdim=True
        ).clamp_min(1e-3)
        desired_xy = target_xy_m * (self.desired_standoff_m / target_range_m)
        guidance = torch.linalg.norm(tail - desired_xy.unsqueeze(1), dim=-1)
        smooth = self.minco.jerk_energy().view(batch_size, lattice_size)
        time_cost = durations.sum(dim=-1)

        trajectory_cost = (
            self.smoothness_weight * smooth
            + self.collision_weight * safety
            + self.guidance_weight * guidance
            + 0.05 * time_cost
        )
        score_loss, objectness_loss, labels = self._objectness_and_score(
            tail, score_logits, objectness_logits, target_xy_m, trajectory_cost
        )
        positive_mask = labels > 0.5
        trajectory_loss = (
            trajectory_cost[positive_mask].mean()
            if positive_mask.any()
            else trajectory_cost.mean()
        )
        total = (
            trajectory_loss
            + self.score_weight * score_loss
            + self.objectness_weight * objectness_loss
        )
        return {
            "total": total,
            "trajectory": trajectory_loss.detach(),
            "score": score_loss.detach(),
            "objectness": objectness_loss.detach(),
            "cost": trajectory_cost.detach(),
        }
