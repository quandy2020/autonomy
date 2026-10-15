# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sample demonstration frames at ``fps`` and flush LeRobot episodes."""

from __future__ import annotations

import logging
from typing import Any, Mapping, Optional, Tuple

import numpy as np

from autosim.dataset.episode import Episode
from autosim.dataset.frame import Frame
from autosim.dataset.lerobot import LeRobotWriter, maybe_create_writer
from autosim.dataset.schema import config_from_habitat

logger = logging.getLogger(__name__)


class DatasetCollector:
    """Buffer demonstration frames and write LeRobot episodes on flush.

    Typical wiring from :class:`~autosim.runtime.loop.Runner`::

        collector.record_frame(dt, timestamp=..., pose=..., velocity=..., action=...)
        ...
        collector.close()
    """

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.config = dict(config)
        self.enabled = bool(config.get("enabled", False))
        self.period = 1.0 / max(float(config.get("fps", 10.0)), 1e-3)
        self.elapsed = 0.0
        self.episode = Episode(task=str(config.get("task", "navigate")))
        self.episodes_written = 0
        self.writer: Optional[LeRobotWriter] = None
        self.closed = False
        if self.enabled:
            try:
                self.writer = maybe_create_writer(config)
            except Exception as exc:
                logger.warning("dataset writer disabled: %s", exc)
                self.enabled = False
                self.writer = None

    @classmethod
    def create_when_enabled(
        cls, habitat: Mapping[str, Any]
    ) -> Optional["DatasetCollector"]:
        """Return a collector when ``habitat.dataset.enabled`` is true."""
        config = config_from_habitat(habitat)
        if not config.get("enabled"):
            return None
        return cls(config)

    maybe_create = create_when_enabled

    def record_frame(
        self,
        dt: float,
        *,
        timestamp: float,
        pose: Tuple[float, float, float],
        velocity: Tuple[float, float],
        action: Tuple[float, float],
        rgb: Optional[np.ndarray] = None,
        laser: Optional[np.ndarray] = None,
    ) -> None:
        """Accumulate time and append a frame when the record period elapses."""
        if not self.enabled or self.writer is None or self.closed:
            return
        self.elapsed += float(dt)
        if self.elapsed < self.period:
            return
        self.elapsed = 0.0
        x, y, yaw = pose
        vx, wz = velocity
        command_vx, command_wz = action
        frame = Frame(
            timestamp=float(timestamp),
            state=np.array([x, y, yaw, vx, wz], dtype=np.float32),
            action=np.array([command_vx, command_wz], dtype=np.float32),
            rgb=None if rgb is None else np.asarray(rgb),
            laser=None if laser is None else np.asarray(laser, dtype=np.float32),
        )
        self.episode.append(frame)
        limit = int(self.config.get("episode_frames", 0))
        if limit > 0 and len(self.episode) >= limit:
            self.flush_episode()

    tick = record_frame

    def flush_episode(self) -> None:
        """Write the current episode to disk and start a new buffer."""
        if self.writer is None or not self.episode.frames:
            self.episode = Episode(task=str(self.config.get("task", "navigate")))
            return
        episode_index = self.writer.write_episode(self.episode)
        self.episodes_written += 1
        logger.info(
            "wrote LeRobot episode %s (%d frames) → %s",
            episode_index,
            len(self.episode),
            self.writer.root,
        )
        self.episode = Episode(task=str(self.config.get("task", "navigate")))
        max_episodes = int(self.config.get("max_episodes", 0))
        if max_episodes > 0 and self.episodes_written >= max_episodes:
            self.close()

    def close(self) -> None:
        """Flush the open episode and finalize dataset metadata."""
        if self.closed:
            return
        self.closed = True
        if self.writer is not None and self.episode.frames:
            self.flush_episode()
        if self.writer is not None:
            self.writer.close()
