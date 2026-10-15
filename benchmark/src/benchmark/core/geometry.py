# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Geometry helpers."""

from __future__ import annotations

import math

from benchmark.core.types import Pose2D


def wrap_angle(yaw: float) -> float:
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def heading_error(pose: Pose2D, goal: Pose2D) -> float:
    desired = math.atan2(goal.y - pose.y, goal.x - pose.x)
    return wrap_angle(desired - pose.yaw)


def reached(pose: Pose2D, goal: Pose2D, tol: float) -> bool:
    return distance(pose, goal) <= tol
