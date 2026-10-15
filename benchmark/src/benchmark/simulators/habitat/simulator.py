# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Habitat-Sim adapter (autosim sidecar + autolink cmd_vel + twin).

Maps to Arena Unity/photorealistic indoor role: RGB-D scenes via Habitat when
``launch_autosim=true``; otherwise kinematic twin for offline Arena Evaluation.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

from benchmark.core.types import StepResult, Twist2D
from benchmark.simulators.sidecar import SidecarSimulator
from benchmark.simulators.process import resolve_autosim_root


class HabitatSimulator(SidecarSimulator):
    name = "habitat"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("bridge_node", "benchmark_habitat")
        cfg.setdefault("subscribe_scan", True)
        cfg.setdefault("sync_odom", True)
        cfg.setdefault("autolink", True)
        if not cfg.get("autosim_config"):
            candidate = resolve_autosim_root() / "config" / "default.yaml"
            if candidate.is_file():
                cfg["autosim_config"] = str(candidate)
        self.scene = str(cfg.get("scene") or cfg.get("glb") or "")
        self.sensors = list(cfg.get("sensors") or ["lidar2d", "rgbd"])
        super().__init__(cfg)

    def step(self, twist: Twist2D) -> StepResult:
        result = super().step(twist)
        result.info["backend"] = "habitat"
        result.info["scene"] = self.scene or Path(str(self.config.get("autosim_config", ""))).name
        result.info["sensors"] = self.sensors
        if self._bridge_ok:
            scan = self._bridge.latest_scan()
            if scan is not None:
                result.info["scan_live"] = True
        return result

    def status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "started": self._started,
            "sidecar": self._external,
            "autolink": self._bridge_ok,
            "scene": self.scene,
            "sensors": self.sensors,
            "autosim_config": self.config.get("autosim_config"),
        }
