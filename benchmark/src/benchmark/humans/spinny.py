# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Spinny demo pedestrians moving on a circle (Arena Spinny)."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from benchmark.humans.base import HumanModel, apply_wall_clamp
from benchmark.simulators.world_state import PedestrianAgent, WorldState


class SpinnyHumans(HumanModel):
    name = "spinny"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.radius_orbit = float(self.config.get("orbit_radius", 3.0))
        self.omega = float(self.config.get("omega", 0.4))
        self.ped_radius = float(self.config.get("radius", 0.25))
        self._angles: list[float] = []

    def spawn(self, world: WorldState, count: int) -> None:
        world.pedestrians.clear()
        self._angles = []
        n = max(0, count)
        for i in range(n):
            ang = 2 * math.pi * i / max(n, 1)
            self._angles.append(ang)
            x = self.radius_orbit * math.cos(ang)
            y = self.radius_orbit * math.sin(ang)
            world.pedestrians.append(
                PedestrianAgent(
                    x=x,
                    y=y,
                    goal_x=x,
                    goal_y=y,
                    radius=self.ped_radius,
                    preferred_speed=abs(self.omega * self.radius_orbit),
                )
            )

    def step(self, world: WorldState) -> None:
        if not world.pedestrians:
            return
        if len(self._angles) != len(world.pedestrians):
            self._angles = [2 * math.pi * i / len(world.pedestrians) for i in range(len(world.pedestrians))]
        for i, ped in enumerate(world.pedestrians):
            self._angles[i] += self.omega * world.dt
            ang = self._angles[i]
            nx = self.radius_orbit * math.cos(ang)
            ny = self.radius_orbit * math.sin(ang)
            ped.vx = (nx - ped.x) / max(world.dt, 1e-6)
            ped.vy = (ny - ped.y) / max(world.dt, 1e-6)
            ped.x, ped.y = nx, ny
            ped.goal_x, ped.goal_y = nx, ny
            apply_wall_clamp(ped, world)
