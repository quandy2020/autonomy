# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Procedural world presets → WorldState / YAML."""

from __future__ import annotations

from typing import Any, Dict

from benchmark.worlds.generators.maze import MazeSpec, maze_to_yaml


PRESETS = ("posts", "office", "warehouse", "maze", "hospital", "cafeteria", "rescue")


def generate_world(name: str, *, seed: int = 0, **kwargs: Any) -> Dict[str, Any]:
    key = name.strip().lower()
    if key == "maze":
        return maze_to_yaml(MazeSpec(seed=seed, **{k: v for k, v in kwargs.items() if k in MazeSpec.__dataclass_fields__}))
    if key == "posts":
        return {
            "name": "posts",
            "scenario_type": "posts",
            "seed": seed,
            "obstacle_number": int(kwargs.get("obstacle_number", 100)),
            "x_length": float(kwargs.get("x_length", 20)),
            "y_length": float(kwargs.get("y_length", 20)),
        }
    if key in ("office", "hospital", "cafeteria"):
        return {
            "name": key,
            "scenario_type": "random_room",
            "seed": seed,
            "room_number": int(kwargs.get("room_number", 8)),
            "x_length": float(kwargs.get("x_length", 30)),
            "y_length": float(kwargs.get("y_length", 20)),
        }
    if key in ("warehouse", "rescue"):
        return {
            "name": key,
            "scenario_type": "posts",
            "seed": seed,
            "obstacle_number": int(kwargs.get("obstacle_number", 200)),
            "x_length": float(kwargs.get("x_length", 40)),
            "y_length": float(kwargs.get("y_length", 30)),
        }
    raise ValueError(f"unknown world preset {name!r}; known: {', '.join(PRESETS)}")
