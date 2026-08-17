"""Fake Habitat-free simulator for unit / smoke tests."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


class FakeSimulator:
    """Deterministic stand-in implementing the Simulator sensing API."""

    def __init__(self, width: int = 64, height: int = 48) -> None:
        self.width = width
        self.height = height
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.urdf = None
        self.session = object()

    def reset(self, x: float, y: float, yaw: float) -> None:
        self.set_pose(x, y, yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)

    def step(self) -> None:
        return None

    def close(self) -> None:
        return None

    def eye_height(self) -> float:
        return 0.5

    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def linspace_angles(self, angle_min: float, angle_max: float, count: int) -> np.ndarray:
        if count <= 1:
            return np.array([float(angle_min)], dtype=np.float64)
        return np.linspace(float(angle_min), float(angle_max), num=int(count), dtype=np.float64)

    def laser_ranges(self, angle_min, angle_max, num_beams, range_max) -> np.ndarray:
        return np.full((int(num_beams),), 0.5 * float(range_max), dtype=np.float32)

    def lidar_points(self, h_min, h_max, h_beams, v_min, v_max, v_rings, range_max) -> np.ndarray:
        h = self.linspace_angles(h_min, h_max, h_beams)
        v = self.linspace_angles(v_min, v_max, v_rings)
        radius = 0.5 * float(range_max)
        points = []
        for pitch in v:
            for yaw in h:
                points.append(
                    (
                        radius * math.cos(yaw) * math.cos(pitch),
                        radius * math.sin(yaw) * math.cos(pitch),
                        radius * math.sin(pitch),
                    )
                )
        return np.asarray(points, dtype=np.float32)

    def color_depth(self):
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        color[..., 1] = 128
        depth = np.full((self.height, self.width), 2.0, dtype=np.float32)
        return color, depth

    def world_cloud(self, planar_x, planar_y, eye_y, h_angles, v_angles, range_max):
        radius = 0.5 * float(range_max)
        points = []
        for pitch in v_angles:
            for yaw in h_angles:
                hx = planar_x + radius * math.cos(yaw) * math.cos(pitch)
                hy = eye_y + radius * math.sin(pitch)
                hz = planar_y + radius * math.sin(yaw) * math.cos(pitch)
                points.append((hx, hz, hy))
        return np.asarray(points, dtype=np.float32)
