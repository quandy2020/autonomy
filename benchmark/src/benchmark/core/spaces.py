# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Observation / action space descriptors (RL-ready)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class BoxSpace:
    low: float
    high: float
    shape: Tuple[int, ...]
    dtype: str = "float32"


@dataclass(frozen=True)
class DictSpace:
    spaces: dict


@dataclass(frozen=True)
class ObservationSpec:
    lidar: Optional[BoxSpace] = None
    image: Optional[BoxSpace] = None
    pose: Optional[BoxSpace] = None
    extras: Sequence[str] = ()


@dataclass(frozen=True)
class ActionSpec:
    """Differential-drive twist action."""

    twist: BoxSpace = BoxSpace(low=-1.0, high=1.0, shape=(2,))
