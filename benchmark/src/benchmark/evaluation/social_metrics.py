# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Per-step social metric helpers (no dependency on episode_runner)."""

from __future__ import annotations

import math
from collections.abc import Sequence


def update_social_step(
    *,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    pedestrians: Sequence[object],
    dt: float,
    private_radius: float = 0.5,
    facing_fov: float = math.pi / 3,
) -> dict[str, float]:
    private = 0.0
    facing = 0.0
    seen = 0.0
    for ped in pedestrians:
        px = float(getattr(ped, "x", 0.0))
        py = float(getattr(ped, "y", 0.0))
        pvx = float(getattr(ped, "vx", 0.0))
        pvy = float(getattr(ped, "vy", 0.0))
        dist = math.hypot(px - robot_x, py - robot_y)
        if dist < private_radius:
            private += dt
        to_ped = math.atan2(py - robot_y, px - robot_x)
        dyaw = (to_ped - robot_yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(dyaw) < facing_fov and dist < 4.0:
            facing += dt
        ped_yaw = math.atan2(pvy, pvx) if (abs(pvx) + abs(pvy)) > 1e-3 else 0.0
        to_robot = math.atan2(robot_y - py, robot_x - px)
        dped = (to_robot - ped_yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(dped) < facing_fov and dist < 4.0:
            seen += dt
    return {"private": private, "facing": facing, "seen": seen}
