# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Local episode recorder (JSONL) with optional autosim LeRobot export."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any

import numpy as np

from benchmark.common.types import Pose2D, Twist2D

_LOG = logging.getLogger(__name__)


class EpisodeRecorder:
    """Buffer frames and flush to ``episode_XXXX.jsonl`` (+ optional LeRobot)."""

    def __init__(
        self,
        root: Path,
        *,
        fps: float = 10.0,
        use_lerobot: bool = False,
        channels: dict[str, bool] | None = None,
    ) -> None:
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.period = 1.0 / max(fps, 1e-3)
        self.elapsed = 0.0
        self.frames: list[dict[str, Any]] = []
        self.episode_id = 0
        self._collector: Any = None
        ch = channels or {}
        self.channels = {
            "pose": bool(ch.get("pose", True)),
            "velocity": bool(ch.get("velocity", True)),
            "action": bool(ch.get("action", True)),
            "include_laser": bool(ch.get("include_laser", True)),
            "include_image": bool(ch.get("include_image", False)),
            "lerobot": bool(ch.get("lerobot", use_lerobot)),
        }
        if self.channels["lerobot"] or use_lerobot:
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
                    "include_image": self.channels["include_image"],
                    "include_laser": self.channels["include_laser"],
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
        laser: np.ndarray | None = None,
        image: Any = None,
    ) -> None:
        self.elapsed += dt
        if self.elapsed < self.period:
            return
        self.elapsed = 0.0
        frame: dict[str, Any] = {"t": time.time()}
        if self.channels["pose"]:
            frame["pose"] = [pose.x, pose.y, pose.yaw]
        if self.channels["velocity"]:
            frame["velocity"] = [twist.linear, twist.angular]
        if self.channels["action"]:
            frame["action"] = [command.linear, command.angular]
        if self.channels["include_laser"]:
            frame["laser"] = (
                None if laser is None else np.asarray(laser, dtype=np.float32).tolist()
            )
        if self.channels["include_image"] and image is not None:
            # Keep compact: store shape + dtype hint; full pixels only if small list.
            arr = np.asarray(image)
            if arr.size and arr.size <= 64 * 64 * 4:
                frame["image"] = arr.tolist()
            else:
                frame["image_meta"] = {
                    "shape": list(arr.shape),
                    "dtype": str(arr.dtype),
                }
        self.frames.append(frame)
        if self._collector is not None:
            self._collector.record_frame(
                self.period,
                timestamp=frame["t"],
                pose=(pose.x, pose.y, pose.yaw) if self.channels["pose"] else (0.0, 0.0, 0.0),
                velocity=(twist.linear, twist.angular)
                if self.channels["velocity"]
                else (0.0, 0.0),
                action=(command.linear, command.angular)
                if self.channels["action"]
                else (0.0, 0.0),
                laser=None
                if (not self.channels["include_laser"] or laser is None)
                else np.asarray(laser, dtype=np.float32),
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


def load_jsonl_episode(path: Path) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    with Path(path).open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    return frames
