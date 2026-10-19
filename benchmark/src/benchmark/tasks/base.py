# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Task abstract base and runner protocol."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from benchmark.common.config import BenchmarkConfig
from benchmark.common.types import EpisodeResult
from benchmark.simulators.base import SimulatorAdapter


class Task(ABC):
    name: str = "base"

    def __init__(
        self,
        config: BenchmarkConfig,
        simulator: SimulatorAdapter | None = None,
        **kwargs: Any,
    ) -> None:
        self.config = config
        self.task_cfg: dict[str, Any] = config.section("task")
        self.simulator = simulator
        self.extra = kwargs

    @abstractmethod
    def run(self) -> list[EpisodeResult]: ...
