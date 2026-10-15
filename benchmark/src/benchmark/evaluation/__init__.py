# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Evaluation package (lazy exports to avoid import cycles with episode_runner)."""

from __future__ import annotations

from typing import Any

__all__ = ["ArenaEvaluation", "Evaluator", "save_traces", "write_report"]


def __getattr__(name: str) -> Any:
    if name == "ArenaEvaluation":
        from benchmark.evaluation.arena_evaluation import ArenaEvaluation

        return ArenaEvaluation
    if name == "Evaluator":
        from benchmark.evaluation.evaluator import Evaluator

        return Evaluator
    if name == "save_traces":
        from benchmark.evaluation.recorder import save_traces

        return save_traces
    if name == "write_report":
        from benchmark.evaluation.report import write_report

        return write_report
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
