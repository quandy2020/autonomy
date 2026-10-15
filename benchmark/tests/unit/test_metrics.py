# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.core.types import EpisodeResult, EpisodeStatus
from benchmark.evaluation.evaluator import Evaluator
from benchmark.evaluation.metrics import success_rate


def test_success_rate() -> None:
    eps = [
        EpisodeResult(0, EpisodeStatus.GOAL_REACHED),
        EpisodeResult(1, EpisodeStatus.TIMEOUT),
    ]
    assert success_rate(eps) == 0.5
    summary = Evaluator().summarize(eps)
    assert summary["count"] == 2
    assert summary["success_rate"] == 0.5
