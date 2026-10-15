# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Shared autosim sidecar + kinematic twin + autolink cmd_vel publisher."""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

import numpy as np

from benchmark.bridge.autolink_bridge import AutolinkBridge
from benchmark.core.types import Pose2D, StepResult, Twist2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.simulators.fake.simulator import FakeSimulator
from benchmark.simulators.process import AutosimProcess

_LOG = logging.getLogger(__name__)


def _ranges_from_scan(scan: Any, n_beams: int = 36, max_range: float = 8.0) -> Optional[np.ndarray]:
    """Extract planar ranges from LaserScan-like message or list."""
    if scan is None:
        return None
    raw: Optional[List[float]] = None
    if hasattr(scan, "ranges"):
        try:
            raw = [float(x) for x in scan.ranges]
        except Exception:  # noqa: BLE001
            raw = None
    elif isinstance(scan, (list, tuple, np.ndarray)):
        raw = [float(x) for x in scan]
    if not raw:
        return None
    arr = np.asarray(raw, dtype=np.float32)
    arr = np.nan_to_num(arr, nan=max_range, posinf=max_range, neginf=0.0)
    arr = np.clip(arr, 0.0, max_range)
    if arr.size == n_beams:
        return arr
    # Resample to twin lidar resolution
    idx = np.linspace(0, arr.size - 1, n_beams)
    return np.interp(idx, np.arange(arr.size), arr).astype(np.float32)


class SidecarSimulator(SimulatorAdapter):
    """Base for habitat / isaac / gazebo / unity: optional autosim + local twin."""

    name = "sidecar"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self._twin = FakeSimulator(config)
        self._proc = AutosimProcess(config)
        self._bridge = AutolinkBridge(
            node_name=str(self.config.get("bridge_node", f"benchmark_{self.name}")),
            cmd_vel=str(self.config.get("cmd_vel", "/cmd_vel")),
            odom=str(self.config.get("odom", "/odom")),
            scan=str(self.config.get("scan", "/scan")),
        )
        self._external = False
        self._bridge_ok = False
        self._sync_odom = bool(self.config.get("sync_odom", True))
        self._inject_scan = bool(self.config.get("inject_scan", True))
        self._subscribe_scan = bool(self.config.get("subscribe_scan", True))

    @property
    def world(self) -> Any:
        return self._twin.world

    def start(self) -> None:
        path = self.config.get("autosim_config")
        launch = bool(self.config.get("launch_autosim", False))
        use_bridge = bool(self.config.get("autolink", True))
        _LOG.info(
            "%s start (autosim=%s launch=%s autolink=%s scan=%s)",
            self.name,
            path,
            launch,
            use_bridge,
            self._subscribe_scan,
        )
        if launch:
            self._external = self._proc.start()
            if not self._external:
                _LOG.warning("%s sidecar failed — twin only", self.name)
        if use_bridge and (self._external or bool(self.config.get("force_autolink", False))):
            self._bridge_ok = self._bridge.connect(
                subscribe_odom=self._sync_odom,
                subscribe_scan=self._subscribe_scan,
            )
            if self._bridge_ok:
                self._wait_first_odom()
        self._twin.start()
        self._started = True

    def _wait_first_odom(self, timeout_s: float = 1.0) -> None:
        """Spin briefly for first odom if sidecar is live."""
        import time

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._bridge.spin_once()
            if self._bridge.latest_pose() is not None:
                _LOG.info("%s received first /odom", self.name)
                return
            time.sleep(0.05)

    def configure_world(self, **kwargs: Any) -> Pose2D:
        return self._twin.configure_world(**kwargs)

    def reset(self, *, seed: Optional[int] = None) -> Pose2D:
        return self._twin.reset(seed=seed)

    def set_goal(self, goal: Pose2D) -> None:
        self._twin.set_goal(goal)

    def get_goal(self) -> Pose2D:
        return self._twin.get_goal()

    def step(self, twist: Twist2D) -> StepResult:
        if self._bridge_ok:
            self._bridge.publish_twist(twist)
            self._bridge.spin_once()

        result = self._twin.step(twist)

        if self._bridge_ok and self._sync_odom:
            live = self._bridge.latest_pose()
            if live is not None:
                self._twin.world.pose = live
                result.pose = live
                result.info["odom_synced"] = True

        # Prefer live LaserScan in observation for learning / hybrid planners.
        if self._bridge_ok and self._inject_scan:
            ranges = _ranges_from_scan(self._bridge.latest_scan())
            if ranges is not None:
                obs = dict(result.info.get("observation") or self._twin.world.observation())
                obs["lidar"] = ranges
                obs["lidar_source"] = "autolink_scan"
                result.info["observation"] = obs
                result.info["scan_injected"] = True

        result.info["sidecar"] = self._external
        result.info["autolink"] = self._bridge_ok
        result.info["autosim_alive"] = self._proc.alive
        return result

    def get_pose(self) -> Pose2D:
        if self._bridge_ok and self._sync_odom:
            live = self._bridge.latest_pose()
            if live is not None:
                return live
        return self._twin.get_pose()

    def status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "started": self._started,
            "sidecar": self._external,
            "autolink": self._bridge_ok,
            "autosim": self._proc.status(),
            "sync_odom": self._sync_odom,
            "inject_scan": self._inject_scan,
        }

    def stop(self) -> None:
        self._bridge.shutdown()
        self._proc.stop()
        self._twin.stop()
        self._started = False
        self._external = False
        self._bridge_ok = False
