from __future__ import annotations

from typing import Optional, Tuple


class ImuEstimator:
    """Finite-difference yaw rate and body accel from planar motion."""

    def __init__(self) -> None:
        self._yaw: Optional[float] = None
        self._v: Optional[float] = None
        self._t: Optional[float] = None

    def reset(self, yaw: float, t: float, v: float = 0.0) -> None:
        self._yaw = float(yaw)
        self._v = float(v)
        self._t = float(t)

    def update(self, yaw: float, v: float, t: float) -> Tuple[float, float, float]:
        if self._t is None or self._yaw is None or self._v is None:
            self.reset(yaw, t, v)
            return 0.0, 0.0, 0.0
        dt = t - self._t
        if dt <= 0.0:
            return 0.0, 0.0, 0.0
        gyro_z = (yaw - self._yaw) / dt
        ax = (v - self._v) / dt
        ay = 0.0
        self._yaw = float(yaw)
        self._v = float(v)
        self._t = float(t)
        return float(gyro_z), float(ax), float(ay)
