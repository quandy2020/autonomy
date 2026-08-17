from __future__ import annotations

from typing import Tuple


class GroundTruth:
    """Pass-through ground-truth pose sampler."""

    def sample(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        return float(x), float(y), float(yaw)
