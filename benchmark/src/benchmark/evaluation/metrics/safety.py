# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Safety metrics (Arena Evaluation)."""

from __future__ import annotations

from collections.abc import Sequence

from benchmark.tasks.episode_runner import EpisodeTrace


def collision_amount(traces: Sequence[EpisodeTrace]) -> float:
    return float(sum(t.collisions for t in traces))


def mean_min_clearance(traces: Sequence[EpisodeTrace]) -> float:
    vals = [t.min_clearance for t in traces if t.min_clearance < 1e8]
    if not vals:
        return 0.0
    return float(sum(vals) / len(vals))


def safety_summary(traces: Sequence[EpisodeTrace]) -> dict[str, float]:
    return {
        "collision_amount": collision_amount(traces),
        "mean_min_clearance": mean_min_clearance(traces),
        "collision_episodes": float(sum(1 for t in traces if t.collisions > 0)),
    }
