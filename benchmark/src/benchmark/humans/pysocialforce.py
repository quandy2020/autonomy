# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Moussaid-style extended SFM with group forces (Arena PySocialForce default)."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from benchmark.humans.base import HumanModel, apply_wall_clamp, robot_xy
from benchmark.humans.social_force.model import AgentState, SocialForceModel
from benchmark.simulators.world_state import PedestrianAgent, WorldState


class PySocialForceHumans(HumanModel):
    name = "pysocialforce"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.preferred_speed = float(self.config.get("preferred_speed", 1.2))
        self.radius = float(self.config.get("radius", 0.25))
        self.group_size = int(self.config.get("group_size", 3))
        self.group_attract = float(self.config.get("group_attract", 0.8))
        self.group_repulse = float(self.config.get("group_repulse", 1.5))
        self.model = SocialForceModel(
            A=float(self.config.get("A", 2.0)),
            B=float(self.config.get("B", 0.3)),
            tau=float(self.config.get("tau", 0.5)),
            robot_force=float(self.config.get("robot_force", 2.5)),
        )
        self.groups: List[List[int]] = []

    def spawn(self, world: WorldState, count: int) -> None:
        world.pedestrians.clear()
        self.groups = []
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
        # Partition into groups; leader shares waypoint with members
        ids = list(range(len(world.pedestrians)))
        gs = max(1, self.group_size)
        self.groups = [ids[i : i + gs] for i in range(0, len(ids), gs)]
        for group in self.groups:
            if not group:
                continue
            leader = world.pedestrians[group[0]]
            for idx in group[1:]:
                world.pedestrians[idx].goal_x = leader.goal_x
                world.pedestrians[idx].goal_y = leader.goal_y

    def _group_forces(self, world: WorldState) -> List[Tuple[float, float]]:
        forces = [(0.0, 0.0) for _ in world.pedestrians]
        for group in self.groups:
            if len(group) < 2:
                continue
            members = [world.pedestrians[i] for i in group]
            cx = sum(m.x for m in members) / len(members)
            cy = sum(m.y for m in members) / len(members)
            for idx, m in zip(group, members):
                # Attract to COM
                dx, dy = cx - m.x, cy - m.y
                dist = math.hypot(dx, dy) + 1e-6
                fx = self.group_attract * dx / dist
                fy = self.group_attract * dy / dist
                # Soft repulsion inside group
                for other in members:
                    if other is m:
                        continue
                    ox, oy = m.x - other.x, m.y - other.y
                    od = math.hypot(ox, oy) + 1e-6
                    if od < m.radius + other.radius + 0.3:
                        mag = self.group_repulse * math.exp(-(od - m.radius - other.radius) / 0.3)
                        fx += mag * ox / od
                        fy += mag * oy / od
                forces[idx] = (fx, fy)
        return forces

    def step(self, world: WorldState) -> None:
        if not world.pedestrians:
            return
        # Sync member goals to leaders
        for group in self.groups:
            if not group:
                continue
            leader = world.pedestrians[group[0]]
            if (leader.goal_x - leader.x) ** 2 + (leader.goal_y - leader.y) ** 2 < 0.16:
                leader.goal_x, leader.goal_y = world._sample_free(0.5)
            for idx in group[1:]:
                world.pedestrians[idx].goal_x = leader.goal_x
                world.pedestrians[idx].goal_y = leader.goal_y
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
        base = self.model.forces(agents, robot_xy(world), robot_radius=world.robot_radius)
        group_f = self._group_forces(world)
        for a, (fx, fy), (gx, gy) in zip(agents, base, group_f):
            a.vx += (fx + gx) * world.dt
            a.vy += (fy + gy) * world.dt
            speed = math.hypot(a.vx, a.vy)
            if speed > a.preferred_speed * 1.3:
                a.vx *= a.preferred_speed * 1.3 / speed
                a.vy *= a.preferred_speed * 1.3 / speed
            a.x += a.vx * world.dt
            a.y += a.vy * world.dt
        for ped, a in zip(world.pedestrians, agents):
            ped.x, ped.y, ped.vx, ped.vy = a.x, a.y, a.vx, a.vy
            ped.goal_x, ped.goal_y = a.goal_x, a.goal_y
            apply_wall_clamp(ped, world)


class DeepSocialHumans(PySocialForceHumans):
    """Placeholder for video-trained Deep Social Force (slower reaction)."""

    name = "deep_social"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("preferred_speed", 0.7)
        cfg.setdefault("tau", 1.2)  # slower reaction
        cfg.setdefault("robot_force", 1.5)
        super().__init__(cfg)


class LightSfmHumans(PySocialForceHumans):
    name = "lightsfm"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("group_size", 1)  # individual agents
        super().__init__(cfg)


class PedsimHumans(PySocialForceHumans):
    """Pedsim / Helbing alias."""

    name = "pedsim"
