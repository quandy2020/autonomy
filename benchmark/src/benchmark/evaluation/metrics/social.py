# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Social navigation metrics (Arena Evaluation Table V)."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from benchmark.tasks.episode_runner import EpisodeTrace


def _mean(vals: Sequence[float]) -> float:
    return float(sum(vals) / len(vals)) if vals else 0.0


def social_summary(traces: Sequence[EpisodeTrace]) -> dict[str, float]:
    """Aggregate private-zone / facing / seen-by times across episodes."""
    return {
        "time_in_private_zone_s": _mean([getattr(t, "time_in_private_zone", 0.0) for t in traces]),
        "time_facing_pedestrians_s": _mean(
            [getattr(t, "time_facing_pedestrians", 0.0) for t in traces]
        ),
        "time_seen_by_pedestrians_s": _mean(
            [getattr(t, "time_seen_by_pedestrians", 0.0) for t in traces]
        ),
        "mean_velocity": _mean([_mean_velocity(t) for t in traces]),
    }


def _mean_velocity(trace: EpisodeTrace) -> float:
    if len(trace.points) < 2:
        return 0.0
    vs = [abs(p.twist.linear) for p in trace.points]
    return float(sum(vs) / len(vs))


# Re-export for callers that import from metrics.social
from benchmark.evaluation.social_metrics import update_social_step  # noqa: E402

__all__ = ["social_summary", "update_social_step"]
