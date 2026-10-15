# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena Evaluation-style pipeline: record → metrics → report."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Union

from benchmark.evaluation.metrics.efficiency import efficiency_summary
from benchmark.evaluation.metrics.safety import safety_summary
from benchmark.evaluation.metrics.smoothness import smoothness_summary
from benchmark.evaluation.metrics.social import social_summary
from benchmark.evaluation.metrics.status import status_counts
from benchmark.evaluation.report import write_report
from benchmark.tasks.episode_runner import EpisodeTrace, make_planner, run_episode, trace_to_result
from benchmark.tasks.modes import create_mode
from benchmark.core.config import BenchmarkConfig
from benchmark.simulators.factory import create_simulator


class ArenaEvaluation:
    """Run N episodes and compute Arena-compatible metric suites."""

    def __init__(self, config: BenchmarkConfig) -> None:
        self.config = config
        self.traces: List[EpisodeTrace] = []

    def run(self, episodes: Optional[int] = None) -> Dict[str, Any]:
        task_cfg = self.config.section("task")
        n = int(episodes if episodes is not None else task_cfg.get("episodes", 5))
        timeout_s = float(task_cfg.get("timeout_s", 60))
        tol = float(task_cfg.get("goal_tolerance_m", 0.35))
        mode = create_mode(str(task_cfg.get("mode", "random")), dict(task_cfg.get("mode_config") or {}))
        planner = make_planner(self.config.section("planner"))
        sim = create_simulator(self.config)
        self.traces = []
        with sim:
            for ep in range(n):
                self.traces.append(
                    run_episode(
                        sim,
                        mode,
                        planner,
                        episode_id=ep,
                        timeout_s=timeout_s,
                        goal_tolerance=tol,
                    )
                )
        return self.summarize()

    def summarize(self, traces: Optional[Sequence[EpisodeTrace]] = None) -> Dict[str, Any]:
        traces = list(traces if traces is not None else self.traces)
        summary: Dict[str, Any] = {
            "count": len(traces),
            "status": status_counts(traces),
            "safety": safety_summary(traces),
            "efficiency": efficiency_summary(traces),
            "smoothness": smoothness_summary(traces),
            "social": social_summary(traces),
            "episodes": [trace_to_result(t).metrics for t in traces],
        }
        return summary

    def write(self, output_dir: Union[str, Path], name: str = "arena_eval") -> Path:
        summary = self.summarize()
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        write_report(summary, out, name=name)
        # Also dump raw status-friendly JSON
        (out / f"{name}_full.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return out / f"{name}.json"
