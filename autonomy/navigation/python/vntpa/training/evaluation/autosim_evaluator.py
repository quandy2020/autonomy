"""autosim evaluator adapter."""

from __future__ import annotations

from .evaluator import EvalResult


class AutosimEvaluator:
    def __init__(self, config_path: str) -> None:
        self.config_path = config_path

    def run(self) -> EvalResult:
        raise NotImplementedError("launch vntpa.cli.autosim to run autosim evaluation")
