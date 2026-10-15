# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simulator adapter abstract base."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from benchmark.core.types import Pose2D, StepResult, Twist2D


class SimulatorAdapter(ABC):
    """Thin facade over autosim / autodriver backends."""

    name: str = "base"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = dict(config or {})
        self._started = False

    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def reset(self, *, seed: Optional[int] = None) -> Pose2D:
        ...

    @abstractmethod
    def step(self, twist: Twist2D) -> StepResult:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...

    def get_pose(self) -> Pose2D:
        return Pose2D()

    def __enter__(self) -> "SimulatorAdapter":
        self.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.stop()
