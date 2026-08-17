from __future__ import annotations

import math
from typing import Tuple


def _wrap_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


class Drive:
    """Differential-drive command filter and planar integrator."""

    def __init__(
        self,
        max_linear: float,
        max_angular: float,
        watchdog_sec: float,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.watchdog_sec = float(watchdog_sec)
        self._x = float(x)
        self._y = float(y)
        self._yaw = float(yaw)
        self._v = 0.0
        self._w = 0.0
        self._last_cmd_t = 0.0

    def set_twist(self, linear_x: float, angular_z: float, t: float) -> None:
        self._v = max(-self.max_linear, min(self.max_linear, float(linear_x)))
        self._w = max(-self.max_angular, min(self.max_angular, float(angular_z)))
        self._last_cmd_t = float(t)

    def step(self, dt: float, t: float) -> Tuple[float, float, float]:
        if t - self._last_cmd_t > self.watchdog_sec:
            self._v = 0.0
            self._w = 0.0
        self._x += self._v * math.cos(self._yaw) * dt
        self._y += self._v * math.sin(self._yaw) * dt
        self._yaw = _wrap_yaw(self._yaw + self._w * dt)
        return self.pose()

    def pose(self) -> Tuple[float, float, float]:
        return self._x, self._y, self._yaw

    def velocity(self) -> Tuple[float, float]:
        return self._v, self._w
