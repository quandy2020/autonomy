"""Planar geometry helpers."""

from __future__ import annotations

import numpy as np


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def relative_position(pose: np.ndarray, goal: np.ndarray) -> np.ndarray:
    return np.asarray(goal, dtype=np.float32)[:2] - np.asarray(pose, dtype=np.float32)[:2]
