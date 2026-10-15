# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Robot profile helpers (footprint / velocity limits)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class RobotProfile:
    name: str
    footprint: str = "differential"
    wheelbase: float = 0.2
    max_linear_vel: float = 0.5
    max_angular_vel: float = 1.5
    robot_radius: float = 0.2

    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]] = None) -> "RobotProfile":
        cfg = dict(config or {})
        return cls(
            name=str(cfg.get("name", "robot")),
            footprint=str(cfg.get("footprint", "differential")),
            wheelbase=float(cfg.get("wheelbase", 0.2)),
            max_linear_vel=float(cfg.get("max_linear_vel", 0.5)),
            max_angular_vel=float(cfg.get("max_angular_vel", 1.5)),
            robot_radius=float(cfg.get("robot_radius", 0.2)),
        )

    def clamp_twist(self, linear: float, angular: float) -> tuple[float, float]:
        v = max(-self.max_linear_vel, min(self.max_linear_vel, linear))
        w = max(-self.max_angular_vel, min(self.max_angular_vel, angular))
        return v, w
