# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Episode terminal status counts (Arena Evaluation)."""

from __future__ import annotations

from typing import Dict, Sequence

from benchmark.core.types import EpisodeStatus
from benchmark.tasks.episode_runner import EpisodeTrace


def status_counts(traces: Sequence[EpisodeTrace]) -> Dict[str, int]:
    counts = {s.value: 0 for s in EpisodeStatus}
    for t in traces:
        counts[t.status.value] = counts.get(t.status.value, 0) + 1
    return counts
