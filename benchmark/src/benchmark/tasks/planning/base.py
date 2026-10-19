# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Planner abstraction (wraps autonomy.control / planning)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from benchmark.common.types import Pose2D, Twist2D


class Planner(ABC):
    name: str = "base"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = dict(config or {})

    @abstractmethod
    def compute(self, pose: Pose2D, goal: Pose2D) -> Twist2D: ...
