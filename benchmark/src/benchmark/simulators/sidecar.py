# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Shared autosim sidecar + kinematic twin + autolink cmd_vel publisher."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from benchmark.bridge.autolink_bridge import AutolinkBridge
from benchmark.core.types import Pose2D, StepResult, Twist2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.simulators.fake.simulator import FakeSimulator
from benchmark.simulators.process import AutosimProcess

_LOG = logging.getLogger(__name__)


class SidecarSimulator(SimulatorAdapter):
    """Base for habitat / isaac: optional external autosim + local twin."""

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

    def start(self) -> None:
        path = self.config.get("autosim_config")
        launch = bool(self.config.get("launch_autosim", False))
        use_bridge = bool(self.config.get("autolink", True))
        _LOG.info(
            "%s start (autosim=%s launch=%s autolink=%s)",
            self.name,
            path,
            launch,
            use_bridge,
        )
        if launch:
            self._external = self._proc.start()
            if not self._external:
                _LOG.warning("%s sidecar failed — twin only", self.name)
        if use_bridge and (self._external or bool(self.config.get("force_autolink", False))):
            self._bridge_ok = self._bridge.connect(
                subscribe_odom=self._sync_odom,
                subscribe_scan=bool(self.config.get("subscribe_scan", False)),
            )
        self._twin.start()
        self._started = True

    def configure_world(self, **kwargs: Any) -> Pose2D:
        return self._twin.configure_world(**kwargs)

    def reset(self, *, seed: Optional[int] = None) -> Pose2D:
        return self._twin.reset(seed=seed)

    def set_goal(self, goal: Pose2D) -> None:
        self._twin.set_goal(goal)

    def get_goal(self) -> Pose2D:
        return self._twin.get_goal()

    def step(self, twist: Twist2D) -> StepResult:
        # Always publish to sidecar when bridge is live.
        if self._bridge_ok:
            self._bridge.publish_twist(twist)
            self._bridge.spin_once()

        result = self._twin.step(twist)

        # Optionally replace twin pose with live odom for metric honesty.
        if self._bridge_ok and self._sync_odom:
            live = self._bridge.latest_pose()
            if live is not None:
                self._twin.world.pose = live
                result.pose = live
                result.info["odom_synced"] = True

        result.info["sidecar"] = self._external
        result.info["autolink"] = self._bridge_ok
        return result

    def get_pose(self) -> Pose2D:
        if self._bridge_ok and self._sync_odom:
            live = self._bridge.latest_pose()
            if live is not None:
                return live
        return self._twin.get_pose()

    def stop(self) -> None:
        self._bridge.shutdown()
        self._proc.stop()
        self._twin.stop()
        self._started = False
        self._external = False
        self._bridge_ok = False
