# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Isaac Sim adapter (autosim sidecar + autolink cmd_vel + twin).

Arena 5.0 Isaac Gym / Isaac Sim role: photorealistic + multi-sensor when
``launch_autosim=true`` and Isaac Lab is available.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from benchmark.common.types import StepResult, Twist2D
from benchmark.simulators.process import resolve_autosim_root
from benchmark.simulators.sidecar import SidecarSimulator


class IsaacSimulator(SidecarSimulator):
    name = "isaac"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("bridge_node", "benchmark_isaac")
        cfg.setdefault("subscribe_scan", True)
        cfg.setdefault("sync_odom", True)
        cfg.setdefault("autolink", True)
        cfg.setdefault("startup_wait_s", 2.0)
        if not cfg.get("autosim_config"):
            candidate = resolve_autosim_root() / "config" / "isaac.yaml"
            if candidate.is_file():
                cfg["autosim_config"] = str(candidate)
        if not cfg.get("isaaclab_path"):
            cfg["isaaclab_path"] = os.environ.get("ISAACLAB_PATH", "/workspace/isaaclab")
        self.sensors = list(
            cfg.get("sensors")
            or ["lidar2d", "lidar3d", "rgb", "rgbd", "imu", "sonar", "foot_contact"]
        )
        super().__init__(cfg)

    def step(self, twist: Twist2D) -> StepResult:
        result = super().step(twist)
        result.info["backend"] = "isaac"
        result.info["sensors"] = self.sensors
        result.info["isaaclab"] = self.config.get("isaaclab_path")
        if self._bridge_ok:
            scan = self._bridge.latest_scan()
            if scan is not None:
                result.info["scan_live"] = True
        return result

    def status(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "started": self._started,
            "sidecar": self._external,
            "autolink": self._bridge_ok,
            "sensors": self.sensors,
            "isaaclab_path": self.config.get("isaaclab_path"),
            "autosim_config": self.config.get("autosim_config"),
            "config_exists": Path(str(self.config.get("autosim_config", ""))).is_file(),
        }
