# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""HuNavSim-style social state machine over social force (Arena HuNav)."""

from __future__ import annotations

import math
from enum import Enum
from typing import Any

from benchmark.simulators.humans.base import HumanModel, apply_wall_clamp, robot_xy
from benchmark.simulators.humans.social_force.model import AgentState, SocialForceModel
from benchmark.simulators.world_state import PedestrianAgent, WorldState


class SocialState(str, Enum):
    WALKING = "walking"
    STOPPED = "stopped"
    TALKING = "talking"
    RUNNING = "running"
    SCARED = "scared"


class HuNavHumans(HumanModel):
    """Pedestrians with HHI/HRI social states on top of Helbing SFM."""

    name = "hunav"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        self.preferred_speed = float(self.config.get("preferred_speed", 1.2))
        self.radius = float(self.config.get("radius", 0.25))
        self.scare_distance = float(self.config.get("scare_distance", 1.2))
        self.talk_distance = float(self.config.get("talk_distance", 1.5))
        self.model = SocialForceModel(
            A=float(self.config.get("A", 2.0)),
            B=float(self.config.get("B", 0.3)),
            tau=float(self.config.get("tau", 0.5)),
            robot_force=float(self.config.get("robot_force", 2.5)),
        )
        self.states: list[SocialState] = []

    def spawn(self, world: WorldState, count: int) -> None:
        world.pedestrians.clear()
        self.states = []
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
            self.states.append(SocialState.WALKING)

    def _update_states(self, world: WorldState) -> None:
        rx, ry = robot_xy(world)
        for i, p in enumerate(world.pedestrians):
            dist_r = math.hypot(p.x - rx, p.y - ry)
            if dist_r < self.scare_distance:
                self.states[i] = SocialState.SCARED
                continue
            # Talk if another ped nearby and both slow
            talking = False
            for j, q in enumerate(world.pedestrians):
                if i == j:
                    continue
                if math.hypot(p.x - q.x, p.y - q.y) < self.talk_distance:
                    if world.rng.random() < 0.02:
                        talking = True
                        break
            if talking:
                self.states[i] = SocialState.TALKING
            elif world.rng.random() < 0.01:
                self.states[i] = SocialState.STOPPED
            elif world.rng.random() < 0.02:
                self.states[i] = SocialState.RUNNING
            else:
                self.states[i] = SocialState.WALKING

    def step(self, world: WorldState) -> None:
        if not world.pedestrians:
            return
        if len(self.states) != len(world.pedestrians):
            self.states = [SocialState.WALKING] * len(world.pedestrians)
        self._update_states(world)
        agents: list[AgentState] = []
        for i, p in enumerate(world.pedestrians):
            speed = p.preferred_speed
            st = self.states[i]
            if st == SocialState.STOPPED or st == SocialState.TALKING:
                speed = 0.05
            elif st == SocialState.RUNNING or st == SocialState.SCARED:
                speed = p.preferred_speed * 1.6
            if (p.goal_x - p.x) ** 2 + (p.goal_y - p.y) ** 2 < 0.16:
                p.goal_x, p.goal_y = world._sample_free(0.5)
            agents.append(
                AgentState(
                    x=p.x,
                    y=p.y,
                    vx=p.vx,
                    vy=p.vy,
                    goal_x=p.goal_x,
                    goal_y=p.goal_y,
                    radius=p.radius,
                    preferred_speed=speed,
                )
            )
        self.model.step(agents, robot_xy(world), dt=world.dt, robot_radius=world.robot_radius)
        for ped, a, st in zip(world.pedestrians, agents, self.states):
            ped.x, ped.y, ped.vx, ped.vy = a.x, a.y, a.vx, a.vy
            ped.goal_x, ped.goal_y = a.goal_x, a.goal_y
            apply_wall_clamp(ped, world)
            # Expose state for social metrics / viz
            setattr(ped, "social_state", st.value)
