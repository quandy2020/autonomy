"""Pose tokenisation matching C++ PoseAttention."""

from __future__ import annotations

import numpy as np


def fourier_pose(x_m: float, y_m: float, theta_rad: float, dim: int = 64) -> np.ndarray:
    if dim < 4:
        raise ValueError("dim must be >= 4")
    values = np.array([x_m, y_m, np.sin(theta_rad), np.cos(theta_rad)], dtype=np.float32)
    out = np.zeros(dim, dtype=np.float32)
    for i in range(dim):
        base = i % 4
        band = i // 4
        frequency = 2.0 ** (band / 4.0)
        scaled = float(values[base]) * frequency
        out[i] = np.tanh(scaled) if base < 2 else np.sin(scaled)
    return out


def relative_pose(current: np.ndarray, goal: np.ndarray) -> np.ndarray:
    """Return goal expressed in the current frame: [dx, dy, dtheta]."""
    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    c, s = np.cos(current[2]), np.sin(current[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy,
                     np.arctan2(np.sin(goal[2] - current[2]),
                                np.cos(goal[2] - current[2]))], dtype=np.float32)


def normalize_actions(actions: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return (actions - mean) / np.maximum(std, 1e-6)


def denormalize_actions(actions: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return actions * np.maximum(std, 1e-6) + mean
