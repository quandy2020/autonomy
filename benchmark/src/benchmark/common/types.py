# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Shared geometry and episode result types."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class EpisodeStatus(str, Enum):
    RUNNING = "running"
    GOAL_REACHED = "goal_reached"
    COLLISION = "collision"
    TIMEOUT = "timeout"
    ABORTED = "aborted"
    SUCCESS = "success"
    FAILED = "failed"


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Twist2D:
    linear: float = 0.0
    angular: float = 0.0


@dataclass
class StepResult:
    step: int
    pose: Pose2D
    twist: Twist2D = field(default_factory=Twist2D)
    reward: float = 0.0
    done: bool = False
    info: dict[str, Any] = field(default_factory=dict)


@dataclass
class EpisodeResult:
    episode_id: int
    status: EpisodeStatus
    steps: int = 0
    path_length: float = 0.0
    time_s: float = 0.0
    collisions: int = 0
    metrics: dict[str, float] = field(default_factory=dict)
    info: dict[str, Any] = field(default_factory=dict)
    error: str | None = None
