# Copyright 2026 The Openbot Authors
"""Planar 2-piece MINCO_S3NU (YOPO-MINCO adapted to ground robots)."""

from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn


class MincoS3NUPlanar(nn.Module):
    """Differentiable 2-piece planar MINCO for training (spatial dim = 2)."""

    def __init__(self, piece_num: int = 2, device=None):
        super().__init__()
        assert piece_num == 2, "Ground-robot track currently uses piece_num=2"
        self.device = device or torch.device("cpu")
        self.dtype = torch.float32
        self.piece_num = piece_num
        self.spatial_dim = 2
        self.batch_size = None
        self.durations = None
        self.coeffs = None

    def set_parameters(
        self,
        head_pva: torch.Tensor,  # (B,3,2)
        tail_pva: torch.Tensor,  # (B,3,2)
        inner_pts: torch.Tensor,  # (B,1,2)
        durations: torch.Tensor,  # (B,2)
    ) -> None:
        durations = durations.to(self.device, dtype=self.dtype)
        head_pva = head_pva.to(self.device, dtype=self.dtype)
        tail_pva = tail_pva.to(self.device, dtype=self.dtype)
        inner_pts = inner_pts.to(self.device, dtype=self.dtype)
        self.batch_size = durations.shape[0]
        self.durations = durations.clamp_min(0.1)

        t0 = self.durations[:, 0]
        t1 = self.durations[:, 1]
        A = self._build_A(t0, t1)
        zeros5 = torch.zeros(
            self.batch_size, 5, self.spatial_dim, device=self.device, dtype=self.dtype
        )
        b = torch.cat(
            [head_pva, inner_pts[:, 0:1], zeros5, tail_pva], dim=1
        )  # (B,12,2)
        self.coeffs = torch.linalg.solve(A, b)

    def _build_A(self, t0: torch.Tensor, t1: torch.Tensor) -> torch.Tensor:
        batch = t0.shape[0]
        A = torch.zeros(batch, 12, 12, device=self.device, dtype=self.dtype)
        one = torch.ones(batch, device=self.device, dtype=self.dtype)
        A[:, 0, 0] = 1.0
        A[:, 1, 1] = 1.0
        A[:, 2, 2] = 2.0
        t0_2, t0_3, t0_4, t0_5 = t0**2, t0**3, t0**4, t0**5
        A[:, 3, 0:6] = torch.stack([one, t0, t0_2, t0_3, t0_4, t0_5], dim=-1)
        A[:, 4, 0:6] = A[:, 3, 0:6]
        A[:, 4, 6] = -1.0
        A[:, 5, 1:6] = torch.stack(
            [one, 2 * t0, 3 * t0_2, 4 * t0_3, 5 * t0_4], dim=-1
        )
        A[:, 5, 7] = -1.0
        A[:, 6, 2:6] = torch.stack(
            [2 * one, 6 * t0, 12 * t0_2, 20 * t0_3], dim=-1
        )
        A[:, 6, 8] = -2.0
        A[:, 7, 3:6] = torch.stack([6 * one, 24 * t0, 60 * t0_2], dim=-1)
        A[:, 7, 9] = -6.0
        A[:, 8, 4:6] = torch.stack([24 * one, 120 * t0], dim=-1)
        A[:, 8, 10] = -24.0
        t1_2, t1_3, t1_4, t1_5 = t1**2, t1**3, t1**4, t1**5
        A[:, 9, 6:12] = torch.stack([one, t1, t1_2, t1_3, t1_4, t1_5], dim=-1)
        A[:, 10, 7:12] = torch.stack(
            [one, 2 * t1, 3 * t1_2, 4 * t1_3, 5 * t1_4], dim=-1
        )
        A[:, 11, 8:12] = torch.stack(
            [2 * one, 6 * t1, 12 * t1_2, 20 * t1_3], dim=-1
        )
        return A

    def sample_positions(self, samples_per_piece: int = 10) -> torch.Tensor:
        """Return (B, 2*S, 2) positions along the solved trajectory."""
        assert self.coeffs is not None
        samples = []
        for piece in range(self.piece_num):
            duration = self.durations[:, piece]
            local_t = (
                torch.linspace(
                    1.0 / samples_per_piece,
                    1.0,
                    samples_per_piece,
                    device=self.device,
                    dtype=self.dtype,
                ).unsqueeze(0)
                * duration.unsqueeze(1)
            )
            powers = torch.stack(
                [
                    torch.ones_like(local_t),
                    local_t,
                    local_t**2,
                    local_t**3,
                    local_t**4,
                    local_t**5,
                ],
                dim=-1,
            )
            coeff = self.coeffs[:, 6 * piece : 6 * piece + 6, :]
            pos = torch.einsum("bki,bid->bkd", powers, coeff)
            samples.append(pos)
        return torch.cat(samples, dim=1)

    def jerk_energy(self) -> torch.Tensor:
        assert self.coeffs is not None
        energy = torch.zeros(self.batch_size, device=self.device, dtype=self.dtype)
        for piece in range(self.piece_num):
            c = self.coeffs[:, 6 * piece : 6 * piece + 6, :]
            c3, c4, c5 = c[:, 3], c[:, 4], c[:, 5]
            t = self.durations[:, piece]
            t2, t3, t4, t5 = t**2, t**3, t**4, t**5
            energy = energy + (
                36.0 * (c3**2).sum(-1) * t
                + 144.0 * (c3 * c4).sum(-1) * t2
                + 192.0 * (c4**2).sum(-1) * t3
                + 240.0 * (c3 * c5).sum(-1) * t3
                + 720.0 * (c4 * c5).sum(-1) * t4
                + 720.0 * (c5**2).sum(-1) * t5
            )
        return energy


class MincoTrajPlanar:
    """Numpy inference solver mirroring C++ MincoTrajectory (dim=2)."""

    def __init__(self):
        self.coeffs = None
        self.durations = None
        self.total_time = 0.0

    def solve(
        self,
        head_pva: np.ndarray,
        tail_pva: np.ndarray,
        inner_pos: np.ndarray,
        durations,
    ) -> "MincoTrajPlanar":
        head_pva = np.asarray(head_pva, dtype=np.float64).reshape(3, 2)
        tail_pva = np.asarray(tail_pva, dtype=np.float64).reshape(3, 2)
        inner_pos = np.asarray(inner_pos, dtype=np.float64).reshape(2)
        durations = np.asarray(durations, dtype=np.float64).reshape(2)
        durations = np.maximum(durations, 0.1)
        self.durations = durations
        self.total_time = float(durations.sum())

        A = np.zeros((12, 12), dtype=np.float64)
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 2.0
        t0 = durations[0]
        powers0 = np.array([1, t0, t0**2, t0**3, t0**4, t0**5])
        A[3, 0:6] = powers0
        A[4, 0:6] = powers0
        A[4, 6] = -1.0
        A[5, 1:6] = [1, 2 * t0, 3 * t0**2, 4 * t0**3, 5 * t0**4]
        A[5, 7] = -1.0
        A[6, 2:6] = [2, 6 * t0, 12 * t0**2, 20 * t0**3]
        A[6, 8] = -2.0
        A[7, 3:6] = [6, 24 * t0, 60 * t0**2]
        A[7, 9] = -6.0
        A[8, 4:6] = [24, 120 * t0]
        A[8, 10] = -24.0
        t1 = durations[1]
        A[9, 6:12] = [1, t1, t1**2, t1**3, t1**4, t1**5]
        A[10, 7:12] = [1, 2 * t1, 3 * t1**2, 4 * t1**3, 5 * t1**4]
        A[11, 8:12] = [2, 6 * t1, 12 * t1**2, 20 * t1**3]

        b = np.zeros((12, 2), dtype=np.float64)
        b[0:3] = head_pva
        b[3] = inner_pos
        b[9:12] = tail_pva
        self.coeffs = np.linalg.solve(A, b)
        return self

    def velocity(self, time_s: float) -> np.ndarray:
        return self._eval(time_s, 1)

    def position(self, time_s: float) -> np.ndarray:
        return self._eval(time_s, 0)

    def _eval(self, time_s: float, derivative: int) -> np.ndarray:
        assert self.coeffs is not None
        t = float(np.clip(time_s, 0.0, max(0.0, self.total_time - 1e-9)))
        piece = 1 if t >= self.durations[0] else 0
        local = t - (0.0 if piece == 0 else self.durations[0])
        if derivative == 0:
            powers = np.array(
                [1, local, local**2, local**3, local**4, local**5]
            )
        else:
            powers = np.array(
                [0, 1, 2 * local, 3 * local**2, 4 * local**3, 5 * local**4]
            )
        return powers @ self.coeffs[piece * 6 : (piece + 1) * 6]
