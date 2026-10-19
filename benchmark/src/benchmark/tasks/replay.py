# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Replay recorded JSONL episodes onto autolink ``cmd_vel`` (or dry-run)."""

from __future__ import annotations

import logging
import time
from pathlib import Path

from benchmark.bridge.autolink_bridge import AutolinkBridge
from benchmark.common.types import EpisodeResult, EpisodeStatus, Twist2D
from benchmark.evaluation.dataset import load_jsonl_episode
from benchmark.tasks.base import Task

_LOG = logging.getLogger(__name__)


class ReplayTask(Task):
    name = "replay"

    def run(self) -> list[EpisodeResult]:
        source = Path(str(self.task_cfg.get("source", "data/datasets/collect")))
        rate = float(self.task_cfg.get("rate", 1.0))
        loop = bool(self.task_cfg.get("loop", False))
        dry_run = bool(self.task_cfg.get("dry_run", False))
        files = (
            sorted(source.glob("episode_*.jsonl"))
            if source.is_dir()
            else ([source] if source.is_file() else [])
        )
        if not files:
            _LOG.warning("ReplayTask: no episodes under %s", source)
            return [
                EpisodeResult(
                    episode_id=0,
                    status=EpisodeStatus.FAILED,
                    error=f"no episodes in {source}",
                )
            ]

        bridge = AutolinkBridge(node_name="benchmark_replay")
        connected = False if dry_run else bridge.connect(subscribe_odom=False)
        _LOG.info(
            "ReplayTask: %s files @ %.2fx (autolink=%s dry_run=%s)",
            len(files),
            rate,
            connected,
            dry_run,
        )

        results: list[EpisodeResult] = []
        for ep, path in enumerate(files):
            frames = load_jsonl_episode(path)
            published = 0
            t0 = time.monotonic()
            for frame in frames:
                action = frame.get("action") or [0.0, 0.0]
                twist = Twist2D(linear=float(action[0]), angular=float(action[1]))
                bridge.publish_twist(twist)
                published += 1
                # Pace approximately at collect fps * rate
                time.sleep(max(0.0, 0.05 / max(rate, 1e-3)))
            elapsed = time.monotonic() - t0
            results.append(
                EpisodeResult(
                    episode_id=ep,
                    status=EpisodeStatus.SUCCESS,
                    steps=published,
                    time_s=elapsed,
                    metrics={"frames": float(published)},
                    info={"source": str(path), "autolink": connected, "stub": False},
                )
            )
            if not loop:
                break
        bridge.shutdown()
        return results
