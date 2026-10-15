# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Classic ↔ learning planner switch (Arena All-in-One style)."""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from benchmark.core.types import Pose2D, Twist2D
from benchmark.planning.base import Planner
from benchmark.planning.classic.dwa import DwaPlanner
from benchmark.planning.learning.policy import PolicyPlanner


class SwitchingPlanner(Planner):
    """Use learning policy in open space; fall back to DWA near obstacles."""

    name = "hybrid"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.clearance_switch = float(self.config.get("clearance_switch", 1.0))
        self.classic = DwaPlanner(config)
        self.learning = PolicyPlanner(config)
        self._last_obs: Optional[Dict[str, Any]] = None
        self.last_mode = "learning"

    def set_observation(self, obs: Dict[str, Any]) -> None:
        self._last_obs = obs
        self.learning.set_observation(obs)

    def compute(self, pose: Pose2D, goal: Pose2D) -> Twist2D:
        min_range = float("inf")
        if self._last_obs and "lidar" in self._last_obs:
            lidar = np.asarray(self._last_obs["lidar"], dtype=np.float32)
            if lidar.size:
                min_range = float(np.min(lidar))
        if min_range < self.clearance_switch:
            self.last_mode = "classic"
            return self.classic.compute(pose, goal)
        self.last_mode = "learning"
        return self.learning.compute(pose, goal)
