# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena planner name aliases → PolicyPlanner variants (ONNX or reactive).

These wrappers mirror Arena-Rosnav planner names so challenge YAMLs and papers
can drop in ``planner.name: cadrl|crowdnav|…`` without code changes. When an
ONNX checkpoint is present they use it; otherwise a lidar-reactive fallback.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.planning.learning.policy import PolicyPlanner


class CadrlPlanner(PolicyPlanner):
    name = "cadrl"


class CrowdNavPlanner(PolicyPlanner):
    name = "crowdnav"


class RlcaPlanner(PolicyPlanner):
    name = "rlca"


class SarlPlanner(PolicyPlanner):
    name = "sarl"


class SarlStarPlanner(PolicyPlanner):
    name = "sarl_star"


class ApplrPlanner(PolicyPlanner):
    """Hybrid Applr-style: more conservative frontal gating."""

    name = "applr"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("desired_linear_vel", 0.3)
        cfg.setdefault("max_angular_vel", 1.0)
        super().__init__(cfg)


class DragonPlanner(PolicyPlanner):
    name = "dragon"


class TrailPlanner(PolicyPlanner):
    name = "trail"


class DrlvoPlanner(PolicyPlanner):
    name = "drlvo"
