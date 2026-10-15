# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Walking behavior presets for pedestrians."""

from __future__ import annotations

from typing import Dict


PRESETS: Dict[str, dict] = {
    "leisure": {"preferred_speed": 0.8, "radius": 0.25},
    "commuter": {"preferred_speed": 1.4, "radius": 0.22},
    "rush": {"preferred_speed": 1.8, "radius": 0.2},
}


def resolve_behavior(name: str) -> dict:
    return dict(PRESETS.get(str(name).lower(), PRESETS["leisure"]))
