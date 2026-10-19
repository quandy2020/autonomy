# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Observation / action space descriptors (RL-ready)."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class BoxSpace:
    low: float
    high: float
    shape: tuple[int, ...]
    dtype: str = "float32"


@dataclass(frozen=True)
class DictSpace:
    spaces: dict


@dataclass(frozen=True)
class ObservationSpec:
    lidar: BoxSpace | None = None
    image: BoxSpace | None = None
    pose: BoxSpace | None = None
    extras: Sequence[str] = ()


@dataclass(frozen=True)
class ActionSpec:
    """Differential-drive twist action."""

    twist: BoxSpace = BoxSpace(low=-1.0, high=1.0, shape=(2,))
