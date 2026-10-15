# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Dynamic Window Approach (classic local planner, Arena-compatible)."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

from benchmark.core.types import Pose2D, Twist2D
from benchmark.planning.base import Planner


class DwaPlanner(Planner):
    name = "dwa"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.max_v = float(self.config.get("max_linear_vel", 0.5))
        self.max_w = float(self.config.get("max_angular_vel", 1.5))
        self.v_samples = int(self.config.get("v_samples", 5))
        self.w_samples = int(self.config.get("w_samples", 9))
        self.predict_time = float(self.config.get("predict_time", 1.0))
        self.dt = float(self.config.get("dt", 0.1))
        self.heading_weight = float(self.config.get("heading_weight", 0.8))
        self.velocity_weight = float(self.config.get("velocity_weight", 0.2))
        self.dist_weight = float(self.config.get("dist_weight", 1.0))

    def _simulate(self, pose: Pose2D, v: float, w: float) -> Pose2D:
        x, y, yaw = pose.x, pose.y, pose.yaw
        t = 0.0
        while t < self.predict_time:
            yaw += w * self.dt
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            t += self.dt
        return Pose2D(x=x, y=y, yaw=yaw)

    def _score(self, end: Pose2D, goal: Pose2D, v: float) -> float:
        heading = math.atan2(goal.y - end.y, goal.x - end.x)
        heading_err = abs((heading - end.yaw + math.pi) % (2 * math.pi) - math.pi)
        dist = math.hypot(goal.x - end.x, goal.y - end.y)
        return (
            -self.heading_weight * heading_err
            - self.dist_weight * dist
            + self.velocity_weight * v
        )

    def compute(self, pose: Pose2D, goal: Pose2D) -> Twist2D:
        best: Tuple[float, Twist2D] = (-1e9, Twist2D())
        for i in range(self.v_samples):
            v = self.max_v * i / max(1, self.v_samples - 1)
            for j in range(self.w_samples):
                w = -self.max_w + 2 * self.max_w * j / max(1, self.w_samples - 1)
                end = self._simulate(pose, v, w)
                score = self._score(end, goal, v)
                if score > best[0]:
                    best = (score, Twist2D(linear=v, angular=w))
        return best[1]
