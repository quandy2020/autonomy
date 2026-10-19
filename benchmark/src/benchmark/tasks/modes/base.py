# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena-style task mode: how start/goal/obstacles are sampled each episode."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

from benchmark.common.types import Pose2D
from benchmark.simulators.base import SimulatorAdapter


@dataclass
class EpisodeSetup:
    start: Pose2D = field(default_factory=Pose2D)
    goal: Pose2D = field(default_factory=lambda: Pose2D(x=5.0, y=0.0))
    obstacles: int = 10
    pedestrians: int = 5
    moving_obstacles: int = 2
    waypoints: list = field(default_factory=list)
    info: dict[str, Any] = field(default_factory=dict)


class TaskMode(ABC):
    name: str = "base"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = dict(config or {})

    @abstractmethod
    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup: ...
