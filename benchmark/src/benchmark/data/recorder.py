# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Local episode recorder (JSONL) with optional autosim LeRobot export."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from benchmark.core.types import Pose2D, Twist2D

_LOG = logging.getLogger(__name__)


class EpisodeRecorder:
    """Buffer frames and flush to ``episode_XXXX.jsonl`` (+ optional LeRobot)."""

    def __init__(self, root: Path, *, fps: float = 10.0, use_lerobot: bool = False) -> None:
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.period = 1.0 / max(fps, 1e-3)
        self.elapsed = 0.0
        self.frames: List[Dict[str, Any]] = []
        self.episode_id = 0
        self._collector: Any = None
        if use_lerobot:
            self._try_lerobot()

    def _try_lerobot(self) -> None:
        try:
            from autosim.dataset.collector import DatasetCollector

            self._collector = DatasetCollector(
                {
                    "enabled": True,
                    "root": str(self.root / "lerobot"),
                    "repo_id": "benchmark/collect",
                    "task": "navigate",
                    "fps": self.fps,
                    "episode_frames": 0,
                    "include_image": False,
                    "include_laser": True,
                }
            )
            _LOG.info("LeRobot collector enabled → %s", self.root / "lerobot")
        except Exception as exc:  # noqa: BLE001
            _LOG.info("LeRobot unavailable (%s); JSONL only", exc)
            self._collector = None

    def start_episode(self, episode_id: int) -> None:
        self.episode_id = episode_id
        self.frames = []
        self.elapsed = 0.0

    def maybe_record(
        self,
        dt: float,
        *,
        pose: Pose2D,
        twist: Twist2D,
        command: Twist2D,
        laser: Optional[np.ndarray] = None,
    ) -> None:
        self.elapsed += dt
        if self.elapsed < self.period:
            return
        self.elapsed = 0.0
        frame = {
            "t": time.time(),
            "pose": [pose.x, pose.y, pose.yaw],
            "velocity": [twist.linear, twist.angular],
            "action": [command.linear, command.angular],
            "laser": None if laser is None else np.asarray(laser, dtype=np.float32).tolist(),
        }
        self.frames.append(frame)
        if self._collector is not None:
            self._collector.record_frame(
                self.period,
                timestamp=frame["t"],
                pose=(pose.x, pose.y, pose.yaw),
                velocity=(twist.linear, twist.angular),
                action=(command.linear, command.angular),
                laser=None if laser is None else np.asarray(laser, dtype=np.float32),
            )

    def flush(self) -> Path:
        path = self.root / f"episode_{self.episode_id:04d}.jsonl"
        with path.open("w", encoding="utf-8") as f:
            for frame in self.frames:
                f.write(json.dumps(frame) + "\n")
        if self._collector is not None:
            try:
                self._collector.close()
            except Exception as exc:  # noqa: BLE001
                _LOG.debug("collector close: %s", exc)
        _LOG.info("wrote %s (%s frames)", path, len(self.frames))
        return path


def load_jsonl_episode(path: Path) -> List[Dict[str, Any]]:
    frames: List[Dict[str, Any]] = []
    with Path(path).open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    return frames
