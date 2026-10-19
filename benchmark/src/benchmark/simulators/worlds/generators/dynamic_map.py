# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Procedural world presets → WorldState / YAML."""

from __future__ import annotations

from typing import Any

import numpy as np

from benchmark.simulators.world_state import ObstacleDisk
from benchmark.simulators.worlds.generators.maze import MazeSpec, maze_to_yaml

PRESETS = (
    "posts",
    "office",
    "warehouse",
    "maze",
    "hospital",
    "cafeteria",
    "canteen",
    "rescue",
    "barn",
    "industrial",
)


def generate_dynamic_layout(
    *, seed: int = 0, kind: str = "barn", count: int = 20
) -> list[ObstacleDisk]:
    """Arena map generators approximated as obstacle disks for Fake/Flatland."""
    rng = np.random.default_rng(seed)
    key = kind.strip().lower()
    disks: list[ObstacleDisk] = []
    if key in ("maze",):
        return generate_maze_obstacles_safe(seed)
    if key in ("barn", "outdoor"):
        for _ in range(count):
            disks.append(
                ObstacleDisk(
                    x=float(rng.uniform(-8, 8)),
                    y=float(rng.uniform(-8, 8)),
                    radius=float(rng.uniform(0.3, 0.9)),
                )
            )
        return disks
    if key in ("canteen", "cafeteria"):
        # Tables + chairs clusters
        for _ in range(max(3, count // 3)):
            tx, ty = float(rng.uniform(-7, 7)), float(rng.uniform(-7, 7))
            disks.append(ObstacleDisk(x=tx, y=ty, radius=0.55))
            for ang in (0.0, 1.57, 3.14, -1.57):
                if rng.random() < 0.55:
                    disks.append(
                        ObstacleDisk(
                            x=tx + 0.9 * float(np.cos(ang)),
                            y=ty + 0.9 * float(np.sin(ang)),
                            radius=0.25,
                        )
                    )
        return disks
    if key in ("industrial", "warehouse"):
        for i in range(count):
            horizontal = rng.random() < 0.5
            x = float(rng.uniform(-8, 8))
            y = float(rng.uniform(-8, 8))
            for k in range(3):
                disks.append(
                    ObstacleDisk(
                        x=x + (0.7 * k if horizontal else 0.0),
                        y=y + (0.0 if horizontal else 0.7 * k),
                        radius=0.35,
                    )
                )
        return disks
    if key in ("office", "hospital"):
        for _ in range(count):
            x, y = float(rng.uniform(-8, 8)), float(rng.uniform(-8, 8))
            disks.append(ObstacleDisk(x=x, y=y, radius=0.45))  # desk
            disks.append(ObstacleDisk(x=x + 0.6, y=y, radius=0.22))  # chair
            disks.append(ObstacleDisk(x=x - 0.7, y=y, radius=0.15))  # separator
        return disks
    # default posts
    for _ in range(count):
        disks.append(
            ObstacleDisk(
                x=float(rng.uniform(-8, 8)),
                y=float(rng.uniform(-8, 8)),
                radius=float(rng.uniform(0.25, 0.7)),
            )
        )
    return disks


def generate_maze_obstacles_safe(seed: int) -> list[ObstacleDisk]:
    from benchmark.simulators.worlds.generators.maze import generate_maze_obstacles

    return generate_maze_obstacles(MazeSpec(seed=seed))


def generate_world(name: str, *, seed: int = 0, **kwargs: Any) -> dict[str, Any]:
    key = name.strip().lower()
    if key == "maze":
        return maze_to_yaml(
            MazeSpec(
                seed=seed, **{k: v for k, v in kwargs.items() if k in MazeSpec.__dataclass_fields__}
            )
        )
    if key == "posts":
        return {
            "name": "posts",
            "scenario_type": "posts",
            "seed": seed,
            "obstacle_number": int(kwargs.get("obstacle_number", 100)),
            "x_length": float(kwargs.get("x_length", 20)),
            "y_length": float(kwargs.get("y_length", 20)),
        }
    if key in ("office", "hospital", "cafeteria", "canteen"):
        return {
            "name": key,
            "scenario_type": "random_room",
            "seed": seed,
            "room_number": int(kwargs.get("room_number", 8)),
            "x_length": float(kwargs.get("x_length", 30)),
            "y_length": float(kwargs.get("y_length", 20)),
            "layout_disks": len(generate_dynamic_layout(seed=seed, kind=key)),
        }
    if key in ("warehouse", "rescue", "industrial", "barn"):
        return {
            "name": key,
            "scenario_type": "posts",
            "seed": seed,
            "obstacle_number": int(kwargs.get("obstacle_number", 200)),
            "x_length": float(kwargs.get("x_length", 40)),
            "y_length": float(kwargs.get("y_length", 30)),
            "layout_disks": len(generate_dynamic_layout(seed=seed, kind=key)),
        }
    raise ValueError(f"unknown world preset {name!r}; known: {', '.join(PRESETS)}")
