# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Timed Elastic Band–inspired local follower (simplified)."""

from __future__ import annotations

import math
from typing import Any

from benchmark.common.types import Pose2D, Twist2D
from benchmark.tasks.planning.base import Planner
from benchmark.tasks.planning.pure_pursuit import PurePursuitPlanner


class TebPlanner(Planner):
    """Time-optimalish pursuit with angular acceleration limiting."""

    name = "teb"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        self.inner = PurePursuitPlanner(config)
        self.max_w_acc = float(self.config.get("max_angular_acc", 2.0))
        self.dt = float(self.config.get("dt", 0.1))
        self._last_w = 0.0

    def compute(self, pose: Pose2D, goal: Pose2D) -> Twist2D:
        raw = self.inner.compute(pose, goal)
        # Limit angular jerk/acc for smoother bands
        dw = raw.angular - self._last_w
        max_dw = self.max_w_acc * self.dt
        if abs(dw) > max_dw:
            raw = Twist2D(
                linear=raw.linear,
                angular=self._last_w + math.copysign(max_dw, dw),
            )
        self._last_w = raw.angular
        return raw
