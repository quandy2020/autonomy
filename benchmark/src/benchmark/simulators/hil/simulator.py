# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Hardware HAL-in-the-loop adapter via autodriver (+ optional twin)."""

from __future__ import annotations

import logging
from typing import Any

from benchmark.common.types import Pose2D, StepResult, Twist2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.simulators.fake.simulator import FakeSimulator

_LOG = logging.getLogger(__name__)


class HilSimulator(SimulatorAdapter):
    name = "hil"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        self._twin: FakeSimulator | None = None
        if self.config.get("use_sim_twin", True):
            self._twin = FakeSimulator(config)

    def start(self) -> None:
        device = self.config.get("device") or ""
        serial = self.config.get("serial") or ""
        _LOG.info(
            "HilSimulator start (device=%r serial=%r twin=%s)",
            device,
            serial,
            bool(self._twin),
        )
        if self._twin:
            self._twin.start()
        self._started = True

    def configure_world(self, **kwargs: Any) -> Pose2D:
        if self._twin:
            return self._twin.configure_world(**kwargs)
        return Pose2D()

    def reset(self, *, seed: int | None = None) -> Pose2D:
        if self._twin:
            return self._twin.reset(seed=seed)
        return Pose2D()

    def set_goal(self, goal: Pose2D) -> None:
        if self._twin:
            self._twin.set_goal(goal)

    def get_goal(self) -> Pose2D:
        if self._twin:
            return self._twin.get_goal()
        return Pose2D()

    @property
    def world(self) -> Any:
        return self._twin.world if self._twin else None

    def step(self, twist: Twist2D) -> StepResult:
        # Production: publish to autodriver chassis via autolink (HilTask).
        if self._twin:
            return self._twin.step(twist)
        return StepResult(step=0, pose=Pose2D(), twist=twist)

    def get_pose(self) -> Pose2D:
        if self._twin:
            return self._twin.get_pose()
        return Pose2D()

    def stop(self) -> None:
        if self._twin:
            self._twin.stop()
        self._started = False
