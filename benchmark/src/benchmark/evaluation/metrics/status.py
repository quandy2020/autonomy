# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Episode terminal status counts (Arena Evaluation)."""

from __future__ import annotations

from collections.abc import Sequence

from benchmark.common.types import EpisodeStatus
from benchmark.tasks.episode_runner import EpisodeTrace


def status_counts(traces: Sequence[EpisodeTrace]) -> dict[str, int]:
    counts = {s.value: 0 for s in EpisodeStatus}
    for t in traces:
        counts[t.status.value] = counts.get(t.status.value, 0) + 1
    return counts
