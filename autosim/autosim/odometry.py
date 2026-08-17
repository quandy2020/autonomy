from __future__ import annotations

from typing import Tuple

import numpy as np


class WheelOdometry:
    """Wheel odometry pose; optional isotropic XY noise on updates."""

    def __init__(self, noise_std: float = 0.0, seed: int = 0) -> None:
        self.noise_std = float(noise_std)
        self._rng = np.random.default_rng(seed)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def update(self, gt_x: float, gt_y: float, gt_yaw: float) -> Tuple[float, float, float]:
        self._x = float(gt_x)
        self._y = float(gt_y)
        self._yaw = float(gt_yaw)
        if self.noise_std > 0.0:
            self._x += float(self._rng.normal(0.0, self.noise_std))
            self._y += float(self._rng.normal(0.0, self.noise_std))
        return self._x, self._y, self._yaw

    def pose(self) -> Tuple[float, float, float]:
        return self._x, self._y, self._yaw
