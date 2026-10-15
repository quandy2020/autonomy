# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Efficiency metrics (Arena Evaluation)."""

from __future__ import annotations

from typing import Dict, Sequence

from benchmark.core.types import EpisodeStatus
from benchmark.tasks.episode_runner import EpisodeTrace


def success_rate(traces: Sequence[EpisodeTrace]) -> float:
    if not traces:
        return 0.0
    ok = sum(1 for t in traces if t.status == EpisodeStatus.GOAL_REACHED)
    return ok / len(traces)


def mean_path_length(traces: Sequence[EpisodeTrace]) -> float:
    if not traces:
        return 0.0
    return float(sum(t.path_length for t in traces) / len(traces))


def mean_time_to_goal(traces: Sequence[EpisodeTrace]) -> float:
    reached = [t for t in traces if t.status == EpisodeStatus.GOAL_REACHED]
    if not reached:
        return 0.0
    return float(sum(t.time_s for t in reached) / len(reached))


def efficiency_summary(traces: Sequence[EpisodeTrace]) -> Dict[str, float]:
    return {
        "success_rate": success_rate(traces),
        "mean_path_length": mean_path_length(traces),
        "mean_time_to_goal": mean_time_to_goal(traces),
    }
