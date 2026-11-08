"""Aggregate evaluation result."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class EvalResult:
    episodes: int = 0
    successes: int = 0
    spl_values: list[float] = field(default_factory=list)

    @property
    def success_rate(self) -> float:
        return self.successes / self.episodes if self.episodes else 0.0

    @property
    def mean_spl(self) -> float:
        return sum(self.spl_values) / len(self.spl_values) if self.spl_values else 0.0
