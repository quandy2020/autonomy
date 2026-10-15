# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Local pure-pursuit style twist (benchmark-side fallback)."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from benchmark.core.types import Pose2D, Twist2D
from benchmark.planning.base import Planner


class PurePursuitPlanner(Planner):
    name = "pure_pursuit"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.desired_v = float(self.config.get("desired_linear_vel", 0.4))
        self.max_w = float(self.config.get("max_angular_vel", 1.0))

    def compute(self, pose: Pose2D, goal: Pose2D) -> Twist2D:
        dx, dy = goal.x - pose.x, goal.y - pose.y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        yaw_err = (desired - pose.yaw + math.pi) % (2 * math.pi) - math.pi
        return Twist2D(
            linear=min(self.desired_v, 0.8 * dist),
            angular=max(-self.max_w, min(self.max_w, 1.5 * yaw_err)),
        )
