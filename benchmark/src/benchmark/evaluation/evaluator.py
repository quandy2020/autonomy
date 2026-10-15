# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Aggregate episode metrics."""

from __future__ import annotations

from typing import Any, Dict, List, Sequence

from benchmark.core.types import EpisodeResult, EpisodeStatus


class Evaluator:
    def summarize(self, episodes: Sequence[EpisodeResult]) -> Dict[str, Any]:
        if not episodes:
            return {"count": 0}
        successes = sum(
            1
            for e in episodes
            if e.status in (EpisodeStatus.GOAL_REACHED, EpisodeStatus.SUCCESS)
        )
        path = [e.path_length for e in episodes]
        times = [e.time_s for e in episodes]
        return {
            "count": len(episodes),
            "success_rate": successes / len(episodes),
            "mean_path_length": sum(path) / len(path),
            "mean_time_s": sum(times) / len(times),
            "collisions": sum(e.collisions for e in episodes),
            "statuses": {s.value: sum(1 for e in episodes if e.status == s) for s in EpisodeStatus},
        }
