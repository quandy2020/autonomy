# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Record episode traces to disk for offline Arena Evaluation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from benchmark.tasks.episode_runner import EpisodeTrace


def trace_to_dict(trace: EpisodeTrace) -> dict[str, Any]:
    return {
        "episode_id": trace.episode_id,
        "status": trace.status.value,
        "collisions": trace.collisions,
        "coverage": trace.coverage,
        "path_length": trace.path_length,
        "time_s": trace.time_s,
        "min_clearance": trace.min_clearance if trace.min_clearance < 1e8 else None,
        "setup": {
            "start": {
                "x": trace.setup.start.x,
                "y": trace.setup.start.y,
                "yaw": trace.setup.start.yaw,
            },
            "goal": {"x": trace.setup.goal.x, "y": trace.setup.goal.y},
            "obstacles": trace.setup.obstacles,
            "pedestrians": trace.setup.pedestrians,
            "info": trace.setup.info,
        },
        "trajectory": [
            {
                "t": p.t,
                "x": p.pose.x,
                "y": p.pose.y,
                "yaw": p.pose.yaw,
                "v": p.twist.linear,
                "w": p.twist.angular,
                "clearance": p.clearance if p.clearance < 1e8 else None,
            }
            for p in trace.points
        ],
    }


def save_traces(traces: list[EpisodeTrace], path: str | Path) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    payload = [trace_to_dict(t) for t in traces]
    p.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return p
