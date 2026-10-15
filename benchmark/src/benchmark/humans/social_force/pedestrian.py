# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Social-force human model driving WorldState pedestrians."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.humans.base import HumanModel, apply_wall_clamp, robot_xy
from benchmark.humans.social_force.model import AgentState, SocialForceModel
from benchmark.simulators.world_state import PedestrianAgent, WorldState


class SocialForceHumans(HumanModel):
    name = "social_force"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.model = SocialForceModel(
            A=float(self.config.get("A", 2.0)),
            B=float(self.config.get("B", 0.3)),
            tau=float(self.config.get("tau", 0.5)),
            robot_force=float(self.config.get("robot_force", 2.5)),
        )
        self.preferred_speed = float(self.config.get("preferred_speed", 1.2))
        self.radius = float(self.config.get("radius", 0.25))

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
        agents = [
            AgentState(
                x=p.x,
                y=p.y,
                vx=p.vx,
                vy=p.vy,
                goal_x=p.goal_x,
                goal_y=p.goal_y,
                radius=p.radius,
                preferred_speed=p.preferred_speed,
            )
            for p in world.pedestrians
        ]
        # Refresh goals when near target
        for a in agents:
            if (a.goal_x - a.x) ** 2 + (a.goal_y - a.y) ** 2 < 0.16:
                gx, gy = world._sample_free(0.5)
                a.goal_x, a.goal_y = gx, gy
        self.model.step(agents, robot_xy(world), dt=world.dt, robot_radius=world.robot_radius)
        for ped, a in zip(world.pedestrians, agents):
            ped.x, ped.y, ped.vx, ped.vy = a.x, a.y, a.vx, a.vy
            ped.goal_x, ped.goal_y = a.goal_x, a.goal_y
            apply_wall_clamp(ped, world)
