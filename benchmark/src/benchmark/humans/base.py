# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Human / crowd model interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Tuple

from benchmark.simulators.world_state import PedestrianAgent, WorldState


class HumanModel(ABC):
    name: str = "base"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = dict(config or {})

    @abstractmethod
    def spawn(self, world: WorldState, count: int) -> None:
        ...

    @abstractmethod
    def step(self, world: WorldState) -> None:
        ...


def apply_wall_clamp(agent: PedestrianAgent, world: WorldState) -> None:
    half_w, half_h = world.width / 2, world.height / 2
    r = agent.radius
    if abs(agent.x) > half_w - r:
        agent.x = max(-half_w + r, min(half_w - r, agent.x))
        agent.vx *= -0.5
    if abs(agent.y) > half_h - r:
        agent.y = max(-half_h + r, min(half_h - r, agent.y))
        agent.vy *= -0.5


def robot_xy(world: WorldState) -> Tuple[float, float]:
    return world.pose.x, world.pose.y
