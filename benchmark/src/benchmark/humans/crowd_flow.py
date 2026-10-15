# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Directional crowd flow (Arena crowd_flow)."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from benchmark.humans.base import HumanModel, apply_wall_clamp
from benchmark.simulators.world_state import PedestrianAgent, WorldState


class CrowdFlowHumans(HumanModel):
    name = "crowd_flow"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        direction = self.config.get("flow_direction") or [1.0, 0.0]
        self.flow: Tuple[float, float] = (float(direction[0]), float(direction[1]))
        norm = math.hypot(*self.flow) or 1.0
        self.flow = (self.flow[0] / norm, self.flow[1] / norm)
        self.preferred_speed = float(self.config.get("preferred_speed", 1.0))
        self.radius = float(self.config.get("radius", 0.25))
        self.lane_spread = float(self.config.get("lane_spread", 3.0))

    def spawn(self, world: WorldState, count: int) -> None:
        world.pedestrians.clear()
        half = world.width / 2 - 1.0
        for i in range(max(0, count)):
            # Spawn along a line perpendicular to flow
            lat = ((i / max(1, count - 1)) - 0.5) * 2 * self.lane_spread if count > 1 else 0.0
            fx, fy = self.flow
            px, py = -fy * lat, fx * lat
            # Place near upstream wall
            x = -half * fx + px
            y = -half * fy + py
            gx = half * fx + px
            gy = half * fy + py
            world.pedestrians.append(
                PedestrianAgent(
                    x=float(x),
                    y=float(y),
                    goal_x=float(gx),
                    goal_y=float(gy),
                    radius=self.radius,
                    preferred_speed=self.preferred_speed,
                    vx=fx * self.preferred_speed * 0.5,
                    vy=fy * self.preferred_speed * 0.5,
                )
            )

    def step(self, world: WorldState) -> None:
        fx, fy = self.flow
        for ped in world.pedestrians:
            # Keep drifting along flow; recycle when past downstream
            ped.vx = 0.85 * ped.vx + 0.15 * fx * ped.preferred_speed
            ped.vy = 0.85 * ped.vy + 0.15 * fy * ped.preferred_speed
            # Soft separation
            for other in world.pedestrians:
                if other is ped:
                    continue
                ox, oy = ped.x - other.x, ped.y - other.y
                od = math.hypot(ox, oy) + 1e-6
                if od < 1.0:
                    ped.vx += 0.5 * ox / od * world.dt
                    ped.vy += 0.5 * oy / od * world.dt
            ped.x += ped.vx * world.dt
            ped.y += ped.vy * world.dt
            # Recycle through opposite boundary
            half_w, half_h = world.width / 2 - ped.radius, world.height / 2 - ped.radius
            if abs(ped.x) > half_w or abs(ped.y) > half_h:
                ped.x = -math.copysign(half_w * 0.95, ped.x if abs(ped.x) > half_w else fx)
                ped.y = -math.copysign(half_h * 0.95, ped.y if abs(ped.y) > half_h else fy)
                if abs(fx) >= abs(fy):
                    ped.x = -math.copysign(half_w * 0.9, fx)
                else:
                    ped.y = -math.copysign(half_h * 0.9, fy)
            apply_wall_clamp(ped, world)
