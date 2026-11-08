"""Habitat evaluator adapter; requires the optional habitat extra."""

from __future__ import annotations

from .evaluator import EvalResult


class HabitatEvaluator:
    def __init__(self, config_path: str) -> None:
        self.config_path = config_path

    def run(self) -> EvalResult:
        raise NotImplementedError("install the habitat extra to run closed-loop evaluation")
