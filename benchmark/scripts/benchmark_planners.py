#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Compare classic / learning / hybrid planners on the same Arena scenarios."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from benchmark.common.config import load_config  # noqa: E402
from benchmark.evaluation.arena_evaluation import ArenaEvaluation  # noqa: E402
from benchmark.evaluation.visualizer import traces_to_svg  # noqa: E402


PLANNERS = ("pure_pursuit", "dwa", "mpc", "teb", "policy", "hybrid")


def main() -> None:
    cfg_path = ROOT / "configs" / "default.yaml"
    out_dir = ROOT / "data" / "eval_results" / "planner_compare"
    out_dir.mkdir(parents=True, exist_ok=True)
    table = []
    for name in PLANNERS:
        cfg = load_config(
            cfg_path,
            overrides=[
                f"planner.name={name}",
                "task.mode=scenario",
                "task.episodes=5",
                "task.timeout_s=12",
                "task.obstacles=8",
                "task.pedestrians=2",
                "task.moving_obstacles=0",
                "task.goal_x=5.0",
            ],
        )
        pipe = ArenaEvaluation(cfg)
        summary = pipe.run()
        summary["planner"] = name
        table.append(
            {
                "planner": name,
                "success_rate": summary["efficiency"]["success_rate"],
                "mean_path_length": summary["efficiency"]["mean_path_length"],
                "mean_time_to_goal": summary["efficiency"]["mean_time_to_goal"],
                "collisions": summary["safety"]["collision_amount"],
                "mean_jerk": summary["smoothness"]["mean_jerk"],
            }
        )
        traces_to_svg(pipe.traces, out_dir / f"{name}.svg")
        (out_dir / f"{name}.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"{name:14s} success={table[-1]['success_rate']:.2f} "
              f"path={table[-1]['mean_path_length']:.2f} "
              f"collisions={table[-1]['collisions']:.0f}")
    (out_dir / "summary.json").write_text(json.dumps(table, indent=2), encoding="utf-8")
    print(f"wrote {out_dir}")


if __name__ == "__main__":
    main()
