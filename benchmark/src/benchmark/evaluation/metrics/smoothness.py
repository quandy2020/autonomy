# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Smoothness metrics: curvature / acceleration / jerk proxies."""

from __future__ import annotations

import math
from typing import Dict, List, Sequence

from benchmark.tasks.episode_runner import EpisodeTrace


def _angular_series(trace: EpisodeTrace) -> List[float]:
    return [p.twist.angular for p in trace.points]


def mean_abs_curvature(trace: EpisodeTrace) -> float:
    vals = [abs(w) for w in _angular_series(trace)]
    return float(sum(vals) / len(vals)) if vals else 0.0


def mean_abs_jerk(trace: EpisodeTrace) -> float:
    ws = _angular_series(trace)
    if len(ws) < 3:
        return 0.0
    jerks = [abs(ws[i + 1] - 2 * ws[i] + ws[i - 1]) for i in range(1, len(ws) - 1)]
    return float(sum(jerks) / len(jerks))


def mean_abs_acceleration(trace: EpisodeTrace) -> float:
    vs = [p.twist.linear for p in trace.points]
    if len(vs) < 2:
        return 0.0
    acc = [abs(vs[i] - vs[i - 1]) for i in range(1, len(vs))]
    return float(sum(acc) / len(acc))


def roughness(trace: EpisodeTrace) -> float:
    """Path roughness ≈ sum of heading changes / path length."""
    if len(trace.points) < 2 or trace.path_length < 1e-6:
        return 0.0
    yaw_changes = 0.0
    for a, b in zip(trace.points, trace.points[1:]):
        dyaw = (b.pose.yaw - a.pose.yaw + math.pi) % (2 * math.pi) - math.pi
        yaw_changes += abs(dyaw)
    return yaw_changes / trace.path_length


def smoothness_summary(traces: Sequence[EpisodeTrace]) -> Dict[str, float]:
    if not traces:
        return {
            "mean_curvature": 0.0,
            "mean_jerk": 0.0,
            "mean_acceleration": 0.0,
            "mean_roughness": 0.0,
        }
    return {
        "mean_curvature": float(sum(mean_abs_curvature(t) for t in traces) / len(traces)),
        "mean_jerk": float(sum(mean_abs_jerk(t) for t in traces) / len(traces)),
        "mean_acceleration": float(sum(mean_abs_acceleration(t) for t in traces) / len(traces)),
        "mean_roughness": float(sum(roughness(t) for t in traces) / len(traces)),
    }
