# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Controller time-series: velocity, acceleration, jerk (linear & angular)."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from benchmark.tasks.episode_runner import EpisodeTrace, TrajectoryPoint


def _finite_diff(values: Sequence[float], times: Sequence[float]) -> list[float]:
    if len(values) < 2:
        return [0.0] * len(values)
    out = [0.0]
    for i in range(1, len(values)):
        dt = max(1e-4, times[i] - times[i - 1])
        out.append((values[i] - values[i - 1]) / dt)
    return out


def downsample(series: dict[str, list[float]], max_points: int = 400) -> dict[str, list[float]]:
    n = len(series.get("t") or [])
    if n <= max_points or n == 0:
        return series
    step = max(1, n // max_points)
    idx = list(range(0, n, step))
    if idx[-1] != n - 1:
        idx.append(n - 1)
    return {k: [v[i] for i in idx] for k, v in series.items()}


def build_control_curves(
    points: Sequence[TrajectoryPoint],
    *,
    max_points: int = 400,
) -> dict[str, Any]:
    """Build controller curves from commanded / measured twist samples."""
    if not points:
        empty: list[float] = []
        return {
            "t": empty,
            "v": empty,
            "w": empty,
            "a": empty,
            "alpha": empty,
            "jerk": empty,
            "jerk_angular": empty,
            "stats": {},
        }

    t = [float(p.t) for p in points]
    v = [float(p.twist.linear) for p in points]
    w = [float(p.twist.angular) for p in points]
    a = _finite_diff(v, t)
    alpha = _finite_diff(w, t)
    jerk = _finite_diff(a, t)
    jerk_w = _finite_diff(alpha, t)

    def _peak(xs: Sequence[float]) -> float:
        return float(max((abs(x) for x in xs), default=0.0))

    def _mean_abs(xs: Sequence[float]) -> float:
        return float(sum(abs(x) for x in xs) / len(xs)) if xs else 0.0

    stats = {
        "v_max": _peak(v),
        "w_max": _peak(w),
        "a_max": _peak(a),
        "a_mean_abs": _mean_abs(a),
        "jerk_max": _peak(jerk),
        "jerk_mean_abs": _mean_abs(jerk),
        "alpha_max": _peak(alpha),
        "jerk_angular_max": _peak(jerk_w),
        "samples": len(t),
        "duration_s": t[-1] - t[0] if len(t) > 1 else 0.0,
    }
    series = downsample(
        {
            "t": t,
            "v": v,
            "w": w,
            "a": a,
            "alpha": alpha,
            "jerk": jerk,
            "jerk_angular": jerk_w,
        },
        max_points=max_points,
    )
    return {**series, "stats": stats}


def curves_from_trace(trace: EpisodeTrace, *, max_points: int = 400) -> dict[str, Any]:
    return build_control_curves(trace.points, max_points=max_points)


def merge_episode_curves(curves_list: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Prefer last non-empty episode curves; keep per-episode stats list."""
    stats = [c.get("stats") or {} for c in curves_list if c]
    last = next((c for c in reversed(curves_list) if c and c.get("t")), {})
    return {
        "latest": last,
        "episodes": list(curves_list),
        "stats_all": stats,
    }
