# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena Gazebo ↔ autosim sidecar + kinematic twin."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.simulators.sidecar import SidecarSimulator


class GazeboSimulator(SidecarSimulator):
    name = "gazebo"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("bridge_node", "benchmark_gazebo")
        cfg.setdefault("backend", "gazebo")
        # Prefer autosim fake/habitat YAML when Gazebo binary is absent.
        cfg.setdefault("launch_autosim", False)
        cfg.setdefault("autolink", True)
        super().__init__(cfg)
