# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Composable navigation rewards (Arena / ROSNavRL style)."""

from __future__ import annotations

import math
from typing import Dict

from benchmark.core.types import Pose2D


def navigation_reward(
    *,
    pose: Pose2D,
    goal: Pose2D,
    prev_dist: float,
    collision: bool,
    reached: bool,
    step_penalty: float = -0.01,
) -> tuple[float, Dict[str, float]]:
    dist = math.hypot(goal.x - pose.x, goal.y - pose.y)
    progress = prev_dist - dist
    reward = step_penalty + 1.0 * progress
    parts = {"step": step_penalty, "progress": 1.0 * progress}
    if collision:
        reward -= 5.0
        parts["collision"] = -5.0
    if reached:
        reward += 10.0
        parts["goal"] = 10.0
    return reward, parts
