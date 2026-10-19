# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Named world presets (Arena Flatland / Gazebo / Unity style)."""

from __future__ import annotations

from typing import Any

from benchmark.simulators.worlds.generators.dynamic_map import generate_dynamic_layout

PRESETS: dict[str, dict[str, Any]] = {
    "corridor": {
        "obstacles": 4,
        "pedestrians": 2,
        "moving_obstacles": 0,
        "goal_x": 3.0,
        "goal_y": 0.0,
    },
    "open": {
        "obstacles": 0,
        "pedestrians": 0,
        "moving_obstacles": 0,
        "goal_x": 4.0,
        "goal_y": 0.0,
    },
    "cluttered": {
        "obstacles": 12,
        "pedestrians": 6,
        "moving_obstacles": 2,
        "goal_x": 3.5,
        "goal_y": 1.0,
    },
    "hospital": {"map": "hospital", "pedestrians": 8, "goal_x": 5.0},
    "canteen": {"map": "canteen", "pedestrians": 10, "goal_x": 4.0},
    "warehouse": {"map": "industrial", "pedestrians": 4, "goal_x": 6.0},
    "office": {"map": "office", "pedestrians": 6, "goal_x": 4.5},
    "barn": {"map": "barn", "pedestrians": 2, "goal_x": 5.0},
}


def get_preset(name: str) -> dict[str, Any]:
    if name not in PRESETS:
        raise KeyError(f"unknown world preset: {name}; choose from {sorted(PRESETS)}")
    return dict(PRESETS[name])


def spawn_preset_obstacles(world: Any, name: str, *, seed: int = 0) -> int:
    """Install Arena layout disks onto a WorldState; return count."""
    preset = get_preset(name)
    kind = str(preset.get("map") or name)
    disks = generate_dynamic_layout(seed=seed, kind=kind, count=int(preset.get("obstacles", 16)))
    world.obstacles.clear()
    world.obstacles.extend(disks)
    return len(disks)
