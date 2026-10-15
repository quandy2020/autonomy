# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simplified ORCA / RVO-style reciprocal velocity obstacle model."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

from benchmark.humans.base import HumanModel, apply_wall_clamp, robot_xy
from benchmark.simulators.world_state import PedestrianAgent, WorldState


def _norm(x: float, y: float) -> Tuple[float, float, float]:
    d = math.hypot(x, y) + 1e-9
    return x / d, y / d, d


class OrcaHumans(HumanModel):
    name = "orca"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.preferred_speed = float(self.config.get("preferred_speed", 1.1))
        self.radius = float(self.config.get("radius", 0.25))
        self.time_horizon = float(self.config.get("time_horizon", 1.5))
        self.neighbor_dist = float(self.config.get("neighbor_dist", 3.0))

    def spawn(self, world: WorldState, count: int) -> None:
        world.pedestrians.clear()
        for _ in range(max(0, count)):
            x, y = world._sample_free(0.6)
            gx, gy = world._sample_free(0.6)
            world.pedestrians.append(
                PedestrianAgent(
                    x=x,
                    y=y,
                    goal_x=gx,
                    goal_y=gy,
                    radius=self.radius,
                    preferred_speed=self.preferred_speed,
                )
            )

    def step(self, world: WorldState) -> None:
        if not world.pedestrians:
            return
        rx, ry = robot_xy(world)
        new_vel: list[Tuple[float, float]] = []
        for i, p in enumerate(world.pedestrians):
            if (p.goal_x - p.x) ** 2 + (p.goal_y - p.y) ** 2 < 0.16:
                p.goal_x, p.goal_y = world._sample_free(0.5)
            dx, dy, _ = _norm(p.goal_x - p.x, p.goal_y - p.y)
            pref_vx = dx * p.preferred_speed
            pref_vy = dy * p.preferred_speed
            vx, vy = pref_vx, pref_vy
            # Reciprocal avoidance vs peers + robot
            others = [(o.x, o.y, o.vx, o.vy, o.radius) for j, o in enumerate(world.pedestrians) if j != i]
            others.append((rx, ry, 0.0, 0.0, world.robot_radius))
            for ox, oy, ovx, ovy, orad in others:
                relx, rely, dist = _norm(ox - p.x, oy - p.y)
                if dist > self.neighbor_dist:
                    continue
                combined = p.radius + orad + 0.05
                if dist < combined:
                    # Push apart
                    push = (combined - dist) / self.time_horizon
                    vx -= relx * push * 2.0
                    vy -= rely * push * 2.0
                    continue
                # Time to collision under relative velocity
                rvx = vx - ovx
                rvy = vy - ovy
                a = rvx * rvx + rvy * rvy
                b = 2 * ((p.x - ox) * rvx + (p.y - oy) * rvy)
                c = dist * dist - combined * combined
                disc = b * b - 4 * a * c
                if a < 1e-6 or disc < 0:
                    continue
                t = (-b - math.sqrt(disc)) / (2 * a)
                if 0 < t < self.time_horizon:
                    w = 1.0 / max(t, 0.05)
                    vx -= relx * w * 0.4
                    vy -= rely * w * 0.4
            speed = math.hypot(vx, vy)
            if speed > p.preferred_speed * 1.2:
                vx *= p.preferred_speed * 1.2 / speed
                vy *= p.preferred_speed * 1.2 / speed
            new_vel.append((vx, vy))
        for p, (vx, vy) in zip(world.pedestrians, new_vel):
            p.vx, p.vy = vx, vy
            p.x += vx * world.dt
            p.y += vy * world.dt
            apply_wall_clamp(p, world)


class Rvo2Humans(OrcaHumans):
    name = "rvo2"
