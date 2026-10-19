# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.common.types import Pose2D
from benchmark.tasks.planning.factory import create_planner, list_planners


def test_list_planners() -> None:
    names = set(list_planners())
    assert {"pure_pursuit", "dwa", "mpc", "teb", "policy", "hybrid"} <= names


def test_planners_compute() -> None:
    pose = Pose2D()
    goal = Pose2D(x=3.0, y=1.0)
    for name in ("pure_pursuit", "dwa", "mpc", "teb", "policy", "hybrid"):
        planner = create_planner({"name": name})
        if hasattr(planner, "set_observation"):
            planner.set_observation({"lidar": [8.0] * 36, "goal": [3.0, 1.0, 0.0]})
        twist = planner.compute(pose, goal)
        assert twist.linear >= 0.0
