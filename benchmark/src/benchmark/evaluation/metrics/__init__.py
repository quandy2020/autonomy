# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

# Back-compat helper used by unit tests on EpisodeResult lists
from collections.abc import Sequence

from benchmark.common.types import EpisodeResult, EpisodeStatus
from benchmark.evaluation.metrics.efficiency import efficiency_summary
from benchmark.evaluation.metrics.safety import safety_summary
from benchmark.evaluation.metrics.smoothness import smoothness_summary
from benchmark.evaluation.metrics.social import social_summary
from benchmark.evaluation.metrics.status import status_counts


def success_rate(episodes: Sequence[EpisodeResult]) -> float:
    """Success rate over ``EpisodeResult`` lists (unit-test helper)."""
    if not episodes:
        return 0.0
    ok = sum(1 for e in episodes if e.status in (EpisodeStatus.GOAL_REACHED, EpisodeStatus.SUCCESS))
    return ok / len(episodes)


__all__ = [
    "efficiency_summary",
    "safety_summary",
    "smoothness_summary",
    "social_summary",
    "status_counts",
    "success_rate",
]
