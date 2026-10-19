# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Load Arena-style scenario JSON/YAML into EpisodeSetup."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml

from benchmark.common.types import Pose2D
from benchmark.tasks.modes.base import EpisodeSetup


def load_scenario(path: str | Path) -> EpisodeSetup:
    p = Path(path).expanduser()
    text = p.read_text(encoding="utf-8")
    if p.suffix.lower() in {".yaml", ".yml"}:
        data = yaml.safe_load(text) or {}
    else:
        data = json.loads(text)
    return scenario_dict_to_setup(data)


def scenario_dict_to_setup(data: dict[str, Any]) -> EpisodeSetup:
    robot = data.get("robot") or data.get("robots") or {}
    if isinstance(robot, list):
        robot = robot[0] if robot else {}
    start = robot.get("start") or data.get("start") or {}
    goal = robot.get("goal") or data.get("goal") or {}
    waypoints_raw = robot.get("waypoints") or data.get("waypoints") or []
    waypoints: list[Pose2D] = []
    for wp in waypoints_raw:
        if isinstance(wp, dict):
            waypoints.append(Pose2D(x=float(wp.get("x", 0)), y=float(wp.get("y", 0))))
        elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
            waypoints.append(Pose2D(x=float(wp[0]), y=float(wp[1])))

    obstacles = data.get("obstacles")
    if isinstance(obstacles, list):
        n_obs = len(obstacles)
    else:
        n_obs = int(obstacles or data.get("n_obstacles", 8))

    peds = data.get("pedestrians")
    if isinstance(peds, list):
        n_ped = len(peds)
    else:
        n_ped = int(peds or data.get("n_pedestrians", 3))

    return EpisodeSetup(
        start=Pose2D(
            x=float(start.get("x", data.get("start_x", 0.0))),
            y=float(start.get("y", data.get("start_y", 0.0))),
            yaw=float(start.get("yaw", data.get("start_yaw", 0.0))),
        ),
        goal=Pose2D(
            x=float(goal.get("x", data.get("goal_x", 5.0))),
            y=float(goal.get("y", data.get("goal_y", 0.0))),
        ),
        obstacles=n_obs,
        pedestrians=n_ped,
        moving_obstacles=int(data.get("moving_obstacles", 0)),
        waypoints=waypoints,
        info={"mode": "scenario", "scenario": data.get("name", "loaded")},
    )
