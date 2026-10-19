# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simulator adapter abstract base."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from benchmark.common.types import Pose2D, StepResult, Twist2D


class SimulatorAdapter(ABC):
    """Thin facade over autosim / autodriver backends."""

    name: str = "base"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = dict(config or {})
        self._started = False

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def reset(self, *, seed: int | None = None) -> Pose2D: ...

    @abstractmethod
    def step(self, twist: Twist2D) -> StepResult: ...

    @abstractmethod
    def stop(self) -> None: ...

    def get_pose(self) -> Pose2D:
        return Pose2D()

    def __enter__(self) -> SimulatorAdapter:
        self.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.stop()
