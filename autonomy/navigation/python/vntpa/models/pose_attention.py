"""Pose utilities kept independent of the ONNX graph."""

from __future__ import annotations

import numpy as np


def relative_pose(pose: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Return planar relative pose ``(dx, dy, dyaw)``."""
    pose = np.asarray(pose, dtype=np.float32)
    reference = np.asarray(reference, dtype=np.float32)
    delta = pose[:2] - reference[:2]
    yaw = float(pose[2] - reference[2])
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    return np.array([
        cos_yaw * delta[0] + sin_yaw * delta[1],
        -sin_yaw * delta[0] + cos_yaw * delta[1],
        yaw,
    ], dtype=np.float32)
