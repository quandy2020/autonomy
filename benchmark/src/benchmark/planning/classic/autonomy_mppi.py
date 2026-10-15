# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Planner that prefers a live autonomy.control process, else local MPC."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from benchmark.bridge.autolink_bridge import AutolinkBridge
from benchmark.bridge.autonomy_control import AutonomyControlBridge
from benchmark.core.types import Pose2D, Twist2D
from benchmark.planning.base import Planner
from benchmark.planning.classic.mpc import MpcPlanner

_LOG = logging.getLogger(__name__)


class AutonomyMppiPlanner(Planner):
    """Attach ``autonomy.control`` when binary/DAG exist; publish goals via autolink.

    Without a live controller, falls back to the local sampling MPC (same API).
    """

    name = "autonomy_mppi"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self._local = MpcPlanner(config)
        self._control = AutonomyControlBridge(config)
        self._bridge = AutolinkBridge(
            node_name=str(self.config.get("bridge_node", "benchmark_mppi")),
            cmd_vel=str(self.config.get("cmd_vel", "/cmd_vel")),
            odom=str(self.config.get("odom", "/odom")),
        )
        self._live = False
        self._started = False

    def ensure_started(self) -> None:
        if self._started:
            return
        self._started = True
        launched = self._control.start()
        bridged = self._bridge.connect(subscribe_odom=True)
        self._live = bool(launched and bridged)
        if self._live:
            _LOG.info("AutonomyMppiPlanner live (control+autolink)")
        else:
            _LOG.info("AutonomyMppiPlanner using local MPC fallback")

    def compute(self, pose: Pose2D, goal: Pose2D) -> Twist2D:
        self.ensure_started()
        # Local compute always available for Arena Evaluation loop.
        twist = self._local.compute(pose, goal)
        if self._live:
            # Mirror command onto /cmd_vel so ControllerServer / plant can track.
            self._bridge.publish_twist(twist)
            self._bridge.spin_once()
        return twist

    def shutdown(self) -> None:
        self._bridge.shutdown()
        self._control.stop()
        self._live = False
        self._started = False
