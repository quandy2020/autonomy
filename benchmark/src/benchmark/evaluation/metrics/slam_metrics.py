# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""SLAM trajectory metrics: ATE / RPE / coverage."""

from __future__ import annotations

import math
from typing import Dict, Sequence, Tuple

from benchmark.core.types import Pose2D


def ate(gt: Sequence[Pose2D], est: Sequence[Pose2D]) -> float:
    """Absolute Trajectory Error (mean translational RMSE)."""
    n = min(len(gt), len(est))
    if n == 0:
        return 0.0
    err = 0.0
    for i in range(n):
        dx = gt[i].x - est[i].x
        dy = gt[i].y - est[i].y
        err += dx * dx + dy * dy
    return math.sqrt(err / n)


def rpe(gt: Sequence[Pose2D], est: Sequence[Pose2D], delta: int = 1) -> float:
    """Relative Pose Error over ``delta`` steps (translational)."""
    n = min(len(gt), len(est))
    if n <= delta:
        return 0.0
    err = 0.0
    count = 0
    for i in range(n - delta):
        gdx = gt[i + delta].x - gt[i].x
        gdy = gt[i + delta].y - gt[i].y
        edx = est[i + delta].x - est[i].x
        edy = est[i + delta].y - est[i].y
        err += (gdx - edx) ** 2 + (gdy - edy) ** 2
        count += 1
    return math.sqrt(err / max(1, count))


def occupancy_coverage(
    hits: Sequence[Tuple[float, float]],
    *,
    cell: float = 0.5,
    map_half: float = 10.0,
) -> float:
    if not hits:
        return 0.0
    visited = {(int(x / cell), int(y / cell)) for x, y in hits}
    dim = max(1, int(2 * map_half / cell))
    return len(visited) / float(dim * dim)


def slam_summary(gt: Sequence[Pose2D], est: Sequence[Pose2D], hits: Sequence[Tuple[float, float]]) -> Dict[str, float]:
    return {
        "ate": ate(gt, est),
        "rpe": rpe(gt, est),
        "map_coverage": occupancy_coverage(hits),
    }
