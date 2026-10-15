# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Planner-side metric helpers (path deviation, control effort)."""

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple

import numpy as np


def path_length(points: Sequence[Tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    arr = np.asarray(points, dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))


def mean_curvature(yaws: Iterable[float], dt: float = 0.1) -> float:
    ys = list(yaws)
    if len(ys) < 2 or dt <= 0:
        return 0.0
    d = np.diff(np.unwrap(np.asarray(ys, dtype=float))) / dt
    return float(np.mean(np.abs(d)))


def control_effort(twists: Sequence[Tuple[float, float]]) -> float:
    if not twists:
        return 0.0
    arr = np.asarray(twists, dtype=float)
    return float(np.mean(np.linalg.norm(arr, axis=1)))
