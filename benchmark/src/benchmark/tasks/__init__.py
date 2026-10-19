# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Benchmark tasks: missions + slam / perception / collect / replay / hil."""

from __future__ import annotations

from benchmark.common.registry import get_registry
from benchmark.tasks.base import Task
from benchmark.tasks.collect import CollectTask
from benchmark.tasks.hil import HilTask
from benchmark.tasks.missions import (
    MISSION_CATALOG,
    DockTask,
    ExploreMappingTask,
    PatrolTask,
    TeleopTask,
)
from benchmark.tasks.multi_robot import run_multi_robot
from benchmark.tasks.navigate import NavigateTask
from benchmark.tasks.perception import PerceptionTask
from benchmark.tasks.replay import ReplayTask
from benchmark.tasks.robot_profile import RobotProfile
from benchmark.tasks.slam import SlamTask

_hub = get_registry()
_hub.tasks.register("navigate", NavigateTask)
_hub.tasks.register("dock", DockTask)
_hub.tasks.register("explore_mapping", ExploreMappingTask)
_hub.tasks.register("patrol", PatrolTask)
_hub.tasks.register("teleop", TeleopTask)
_hub.tasks.register("slam", SlamTask)
_hub.tasks.register("perception", PerceptionTask)
_hub.tasks.register("collect", CollectTask)
_hub.tasks.register("replay", ReplayTask)
_hub.tasks.register("hil", HilTask)

__all__ = [
    "MISSION_CATALOG",
    "CollectTask",
    "DockTask",
    "ExploreMappingTask",
    "HilTask",
    "NavigateTask",
    "PatrolTask",
    "PerceptionTask",
    "ReplayTask",
    "SlamTask",
    "Task",
    "TeleopTask",
    "RobotProfile",
    "run_multi_robot",
]
