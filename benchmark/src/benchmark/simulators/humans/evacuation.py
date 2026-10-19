# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Evacuation SFM: shared exit + strong peer repulsion (Arena Evacuation)."""

from __future__ import annotations

from typing import Any

from benchmark.simulators.humans.base import HumanModel, apply_wall_clamp, robot_xy
from benchmark.simulators.humans.social_force.model import AgentState, SocialForceModel
from benchmark.simulators.world_state import PedestrianAgent, WorldState


class EvacuationHumans(HumanModel):
    name = "evacuation"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        self.exit_x = float(self.config.get("exit_x", 8.0))
        self.exit_y = float(self.config.get("exit_y", 0.0))
        self.preferred_speed = float(self.config.get("preferred_speed", 1.6))
        self.radius = float(self.config.get("radius", 0.25))
        self.model = SocialForceModel(
            A=float(self.config.get("A", 5.0)),
            B=float(self.config.get("B", 0.2)),
            tau=float(self.config.get("tau", 0.3)),
            robot_force=float(self.config.get("robot_force", 3.0)),
        )

    def spawn(self, world: WorldState, count: int) -> None:
        world.pedestrians.clear()
        for _ in range(max(0, count)):
            x, y = world._sample_free(0.5)
            world.pedestrians.append(
                PedestrianAgent(
                    x=x,
                    y=y,
                    goal_x=self.exit_x,
                    goal_y=self.exit_y,
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
                goal_x=self.exit_x,
                goal_y=self.exit_y,
                radius=p.radius,
                preferred_speed=p.preferred_speed,
            )
            for p in world.pedestrians
        ]
        self.model.step(agents, robot_xy(world), dt=world.dt, robot_radius=world.robot_radius)
        for ped, a in zip(world.pedestrians, agents):
            ped.x, ped.y, ped.vx, ped.vy = a.x, a.y, a.vx, a.vy
            ped.goal_x, ped.goal_y = self.exit_x, self.exit_y
            apply_wall_clamp(ped, world)
