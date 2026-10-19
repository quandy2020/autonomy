# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.common.types import Pose2D, Twist2D
from benchmark.evaluation.control_curves import build_control_curves
from benchmark.tasks.episode_runner import TrajectoryPoint


def test_build_control_curves_velocity_accel_jerk() -> None:
    pts = [
        TrajectoryPoint(t=0.0, pose=Pose2D(), twist=Twist2D(0.0, 0.0)),
        TrajectoryPoint(t=0.1, pose=Pose2D(0.05), twist=Twist2D(0.5, 0.1)),
        TrajectoryPoint(t=0.2, pose=Pose2D(0.12), twist=Twist2D(1.0, 0.2)),
        TrajectoryPoint(t=0.3, pose=Pose2D(0.22), twist=Twist2D(1.0, 0.0)),
    ]
    curves = build_control_curves(pts)
    assert len(curves["t"]) == 4
    assert curves["stats"]["v_max"] == 1.0
    assert curves["stats"]["a_max"] > 0.0
    assert "jerk" in curves
