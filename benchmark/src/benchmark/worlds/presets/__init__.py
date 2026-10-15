# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Named world presets (corridor / open / cluttered)."""

from __future__ import annotations

from typing import Any, Dict


PRESETS: Dict[str, Dict[str, Any]] = {
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
}


def get_preset(name: str) -> Dict[str, Any]:
    if name not in PRESETS:
        raise KeyError(f"unknown world preset: {name}; choose from {sorted(PRESETS)}")
    return dict(PRESETS[name])
