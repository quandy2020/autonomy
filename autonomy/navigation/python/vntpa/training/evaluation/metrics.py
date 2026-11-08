"""Navigation metrics."""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass
from typing import Any, Dict, List

import numpy as np


def success(final_position: np.ndarray, goal: np.ndarray, radius: float = 0.2) -> bool:
    return float(np.linalg.norm(np.asarray(final_position)[:2] - np.asarray(goal)[:2])) <= radius


def spl(success: bool, shortest_path: float, path_length: float) -> float:
    if not success or shortest_path <= 0 or path_length <= 0:
        return 0.0
    return float(shortest_path / max(shortest_path, path_length))

@dataclass
class BenchmarkReport:
    runs: int
    p50_ms: float
    p95_ms: float
    p99_ms: float
    mean_ms: float
    actions: List[int]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "runs": self.runs,
            "p50_ms": self.p50_ms,
            "p95_ms": self.p95_ms,
            "p99_ms": self.p99_ms,
            "mean_ms": self.mean_ms,
            "actions": self.actions,
        }


def percentile(values: List[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((q / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


class Benchmark:
    """Run a predictor repeatedly; warmup is excluded from stats."""

    def __init__(self, predictor, depth: np.ndarray, pose: np.ndarray, goal: np.ndarray,
                 *, warmup: int = 3, runs: int = 20) -> None:
        self.predictor = predictor
        self.depth, self.pose, self.goal = depth, pose, goal
        self.warmup, self.runs = warmup, max(1, runs)

    def run(self) -> BenchmarkReport:
        for _ in range(max(0, self.warmup)):
            self.predictor.predict(self.depth, self.pose, self.goal)
        samples: List[float] = []
        actions: List[int] = []
        for _ in range(self.runs):
            start = time.perf_counter()
            result = self.predictor.predict(self.depth, self.pose, self.goal)
            samples.append((time.perf_counter() - start) * 1000.0)
            actions = [int(action) for action in result.actions]
        return BenchmarkReport(
            runs=self.runs,
            p50_ms=percentile(samples, 50.0),
            p95_ms=percentile(samples, 95.0),
            p99_ms=percentile(samples, 99.0),
            mean_ms=statistics.fmean(samples),
            actions=actions,
        )
