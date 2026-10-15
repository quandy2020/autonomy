# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Short-horizon MPC-style local planner (sampling + cost)."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

from benchmark.core.types import Pose2D, Twist2D
from benchmark.planning.base import Planner


class MpcPlanner(Planner):
    """Roll out candidate (v,w) sequences and pick lowest cost-to-go."""

    name = "mpc"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.horizon = int(self.config.get("horizon", 8))
        self.dt = float(self.config.get("dt", 0.1))
        self.max_v = float(self.config.get("max_linear_vel", 0.5))
        self.max_w = float(self.config.get("max_angular_vel", 1.2))
        self.v_samples = int(self.config.get("v_samples", 4))
        self.w_samples = int(self.config.get("w_samples", 7))

    def _rollout(self, pose: Pose2D, v: float, w: float) -> Tuple[Pose2D, float]:
        x, y, yaw = pose.x, pose.y, pose.yaw
        cost = 0.0
        for _ in range(self.horizon):
            yaw += w * self.dt
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            cost += 0.01 * abs(w)  # prefer smoother angular
        return Pose2D(x=x, y=y, yaw=yaw), cost

    def compute(self, pose: Pose2D, goal: Pose2D) -> Twist2D:
        best_score = -1e18
        best = Twist2D()
        for i in range(self.v_samples):
            v = self.max_v * i / max(1, self.v_samples - 1)
            for j in range(self.w_samples):
                w = -self.max_w + 2 * self.max_w * j / max(1, self.w_samples - 1)
                end, smooth = self._rollout(pose, v, w)
                dist = math.hypot(goal.x - end.x, goal.y - end.y)
                heading = math.atan2(goal.y - end.y, goal.x - end.x)
                herr = abs((heading - end.yaw + math.pi) % (2 * math.pi) - math.pi)
                score = -1.2 * dist - 0.6 * herr - smooth + 0.15 * v
                if score > best_score:
                    best_score = score
                    best = Twist2D(linear=v, angular=w)
        return best
