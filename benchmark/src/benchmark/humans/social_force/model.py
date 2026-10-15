# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Helbing-style social force model (Arena pedestrian stack)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple


@dataclass
class AgentState:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    goal_x: float = 0.0
    goal_y: float = 0.0
    radius: float = 0.25
    preferred_speed: float = 1.2
    mass: float = 80.0


class SocialForceModel:
    """Compute acceleration for pedestrians given neighbors + robot disk."""

    def __init__(
        self,
        *,
        A: float = 2.0,
        B: float = 0.3,
        tau: float = 0.5,
        robot_force: float = 2.5,
    ) -> None:
        self.A = A
        self.B = B
        self.tau = tau
        self.robot_force = robot_force

    def forces(
        self,
        agents: Sequence[AgentState],
        robot_xy: Tuple[float, float],
        robot_radius: float = 0.2,
    ) -> List[Tuple[float, float]]:
        out: List[Tuple[float, float]] = []
        rx, ry = robot_xy
        for i, a in enumerate(agents):
            dx, dy = a.goal_x - a.x, a.goal_y - a.y
            dist = math.hypot(dx, dy) + 1e-6
            des_vx = a.preferred_speed * dx / dist
            des_vy = a.preferred_speed * dy / dist
            fx = (des_vx - a.vx) / self.tau
            fy = (des_vy - a.vy) / self.tau
            # Robot repulsion
            ox, oy = a.x - rx, a.y - ry
            od = math.hypot(ox, oy) + 1e-6
            gap = od - a.radius - robot_radius
            if gap < 2.0:
                mag = self.robot_force * math.exp(-gap / self.B)
                fx += mag * ox / od
                fy += mag * oy / od
            # Peer repulsion
            for j, b in enumerate(agents):
                if i == j:
                    continue
                ox, oy = a.x - b.x, a.y - b.y
                od = math.hypot(ox, oy) + 1e-6
                gap = od - a.radius - b.radius
                if gap < 1.5:
                    mag = self.A * math.exp(-gap / self.B)
                    fx += mag * ox / od
                    fy += mag * oy / od
            out.append((fx, fy))
        return out

    def step(
        self,
        agents: List[AgentState],
        robot_xy: Tuple[float, float],
        dt: float = 0.1,
        robot_radius: float = 0.2,
    ) -> None:
        forces = self.forces(agents, robot_xy, robot_radius)
        for a, (fx, fy) in zip(agents, forces):
            a.vx += fx * dt
            a.vy += fy * dt
            speed = math.hypot(a.vx, a.vy)
            if speed > a.preferred_speed * 1.3:
                a.vx *= a.preferred_speed * 1.3 / speed
                a.vy *= a.preferred_speed * 1.3 / speed
            a.x += a.vx * dt
            a.y += a.vy * dt
