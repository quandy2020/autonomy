# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from benchmark.evaluation.metrics.efficiency import efficiency_summary, success_rate
from benchmark.evaluation.metrics.safety import safety_summary
from benchmark.evaluation.metrics.smoothness import smoothness_summary
from benchmark.evaluation.metrics.status import status_counts

# Back-compat helper used by unit tests on EpisodeResult lists
from typing import Sequence

from benchmark.core.types import EpisodeResult, EpisodeStatus


def success_rate_results(episodes: Sequence[EpisodeResult]) -> float:
    if not episodes:
        return 0.0
    ok = sum(
        1
        for e in episodes
        if e.status in (EpisodeStatus.GOAL_REACHED, EpisodeStatus.SUCCESS)
    )
    return ok / len(episodes)


# Alias historical name
success_rate = success_rate_results  # type: ignore[assignment]

__all__ = [
    "efficiency_summary",
    "safety_summary",
    "smoothness_summary",
    "status_counts",
    "success_rate",
]
