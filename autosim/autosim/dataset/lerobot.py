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

"""Write AutoSim episodes in HuggingFace LeRobot v2.1 on-disk layout.

Layout::

    <root>/
      meta/info.json
      meta/episodes.jsonl
      meta/tasks.jsonl
      meta/stats.json          # optional (filled on close)
      data/chunk-XXX/episode_YYYYYY.parquet
      videos/chunk-XXX/observation.images.rgb/episode_YYYYYY.mp4

Requires ``pyarrow`` for parquet. Video encoding uses OpenCV when available;
otherwise PNG frame sequences are written under ``images/`` and ``info.json``
notes ``image_fallback``.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from autosim.dataset.episode import Episode
from autosim.dataset.frame import Frame
from autosim.dataset.schema import dataset_info, feature_dict

logger = logging.getLogger(__name__)


class LeRobotWriter:
    """Append-only LeRobot v2.1 dataset writer."""

    def __init__(
        self,
        root: str | Path,
        *,
        fps: float = 10.0,
        repo_id: str = "autosim/demo",
        robot_type: str = "differential",
        task: str = "navigate",
        image_hw: tuple[int, int] | None = (480, 640),
        laser_bins: int = 0,
        include_image: bool = True,
        chunks_size: int = 1000,
        video_codec: str = "mp4v",
    ) -> None:
        self.root = Path(root).expanduser().resolve()
        self.fps = float(fps)
        self.repo_id = str(repo_id)
        self.robot_type = str(robot_type)
        self.default_task = str(task)
        self.include_image = bool(include_image)
        self.laser_bins = int(laser_bins)
        self.chunks_size = max(1, int(chunks_size))
        self.video_codec = str(video_codec)
        self.image_hw = image_hw
        self.features = feature_dict(
            image_hw=image_hw if include_image else None,
            laser_bins=laser_bins,
            include_image=include_image,
        )
        self.tasks: dict[str, int] = {}
        self.episodes: list[dict[str, Any]] = []
        self.global_index = 0
        self.image_fallback = False
        self.ensure_directories()
        self.load_existing_meta()

    def ensure_directories(self) -> None:
        (self.root / "meta").mkdir(parents=True, exist_ok=True)
        (self.root / "data").mkdir(parents=True, exist_ok=True)
        if self.include_image:
            (self.root / "videos").mkdir(parents=True, exist_ok=True)

    def load_existing_meta(self) -> None:
        tasks_path = self.root / "meta" / "tasks.jsonl"
        if tasks_path.is_file():
            for line in tasks_path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                row = json.loads(line)
                self.tasks[str(row["task"])] = int(row["task_index"])
        episodes_path = self.root / "meta" / "episodes.jsonl"
        if episodes_path.is_file():
            for line in episodes_path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                row = json.loads(line)
                self.episodes.append(row)
                self.global_index += int(row.get("length", 0))

    def resolve_task_index(self, task: str) -> int:
        key = str(task)
        if key not in self.tasks:
            self.tasks[key] = len(self.tasks)
        return self.tasks[key]

    def resolve_chunk_index(self, episode_index: int) -> int:
        return int(episode_index) // self.chunks_size

    def write_episode(self, episode: Episode) -> int:
        """Flush one episode to disk; return ``episode_index``."""
        if not episode.frames:
            raise ValueError("cannot write empty episode")
        episode.mark_done()
        episode_index = len(self.episodes)
        chunk = self.resolve_chunk_index(episode_index)
        task_index = self.resolve_task_index(episode.task or self.default_task)
        data_dir = self.root / "data" / f"chunk-{chunk:03d}"
        data_dir.mkdir(parents=True, exist_ok=True)
        parquet_path = data_dir / f"episode_{episode_index:06d}.parquet"
        self.write_parquet(
            parquet_path,
            episode.frames,
            episode_index=episode_index,
            task_index=task_index,
            global_start=self.global_index,
        )
        if self.include_image:
            self.write_video(episode.frames, episode_index=episode_index, chunk=chunk)
        length = len(episode.frames)
        self.episodes.append(
            {
                "episode_index": episode_index,
                "tasks": [episode.task or self.default_task],
                "length": length,
            }
        )
        self.global_index += length
        self.flush_meta()
        return episode_index

    def write_parquet(
        self,
        path: Path,
        frames: Sequence[Frame],
        *,
        episode_index: int,
        task_index: int,
        global_start: int,
    ) -> None:
        try:
            import pyarrow as pa
            import pyarrow.parquet as pq
        except ImportError as exc:
            raise ImportError(
                "pyarrow is required to write LeRobot parquet "
                "(pip install pyarrow)"
            ) from exc

        n = len(frames)
        t0 = float(frames[0].timestamp)
        states = np.stack([f.state for f in frames], axis=0)
        actions = np.stack([f.action for f in frames], axis=0)
        columns: dict[str, Any] = {
            "observation.state": states.tolist(),
            "action": actions.tolist(),
            "timestamp": [float(f.timestamp - t0) for f in frames],
            "frame_index": list(range(n)),
            "episode_index": [episode_index] * n,
            "index": list(range(global_start, global_start + n)),
            "task_index": [task_index] * n,
            "next.done": [bool(f.done) for f in frames],
        }
        if self.laser_bins > 0:
            lasers = []
            for f in frames:
                if f.laser is None or f.laser.size == 0:
                    lasers.append([float("nan")] * self.laser_bins)
                else:
                    arr = np.asarray(f.laser, dtype=np.float32).reshape(-1)
                    if arr.size < self.laser_bins:
                        pad = np.full(self.laser_bins - arr.size, np.nan, dtype=np.float32)
                        arr = np.concatenate([arr, pad])
                    lasers.append(arr[: self.laser_bins].tolist())
            columns["observation.laser"] = lasers
        table = pa.Table.from_pydict(columns)
        pq.write_table(table, path)

    def write_video(
        self, frames: Sequence[Frame], *, episode_index: int, chunk: int
    ) -> None:
        key = "observation.images.rgb"
        rgbs = [f.rgb for f in frames if f.rgb is not None]
        if not rgbs:
            return
        video_dir = self.root / "videos" / f"chunk-{chunk:03d}" / key
        video_dir.mkdir(parents=True, exist_ok=True)
        mp4 = video_dir / f"episode_{episode_index:06d}.mp4"
        if self.encode_mp4(rgbs, mp4):
            return
        # Fallback: PNG sequence (still usable for IL pipelines).
        self.image_fallback = True
        img_dir = (
            self.root
            / "images"
            / f"chunk-{chunk:03d}"
            / key
            / f"episode_{episode_index:06d}"
        )
        img_dir.mkdir(parents=True, exist_ok=True)
        for i, rgb in enumerate(rgbs):
            self.save_png(img_dir / f"frame_{i:06d}.png", rgb)

    def encode_mp4(self, frames: Sequence[np.ndarray], path: Path) -> bool:
        try:
            import cv2  # type: ignore
        except ImportError:
            return False
        h, w = frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.video_codec[:4].ljust(4, "0"))
        writer = cv2.VideoWriter(str(path), fourcc, self.fps, (w, h))
        if not writer.isOpened():
            return False
        try:
            for rgb in frames:
                bgr = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
                if bgr.shape[0] != h or bgr.shape[1] != w:
                    bgr = cv2.resize(bgr, (w, h))
                writer.write(bgr)
        finally:
            writer.release()
        return path.is_file() and path.stat().st_size > 0

    @staticmethod
    def save_png(path: Path, rgb: np.ndarray) -> None:
        try:
            import cv2  # type: ignore

            bgr = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(path), bgr)
            return
        except ImportError:
            pass
        # Minimal uncompressed PPM as last resort (always available).
        arr = np.asarray(rgb, dtype=np.uint8)
        if arr.ndim != 3 or arr.shape[2] < 3:
            raise ValueError("rgb must be HxWx3")
        h, w = arr.shape[:2]
        ppm = path.with_suffix(".ppm")
        header = f"P6\n{w} {h}\n255\n".encode("ascii")
        ppm.write_bytes(header + arr[:, :, :3].tobytes())

    def flush_meta(self) -> None:
        features = dict(self.features)
        if self.image_fallback and "observation.images.rgb" in features:
            features["observation.images.rgb"] = dict(features["observation.images.rgb"])
            features["observation.images.rgb"]["dtype"] = "image"
            features["observation.images.rgb"]["info"] = {
                "note": "mp4 unavailable; PNG/PPM under images/"
            }
        info = dataset_info(
            fps=self.fps,
            features=features,
            robot_type=self.robot_type,
            total_episodes=len(self.episodes),
            total_frames=self.global_index,
            total_tasks=max(1, len(self.tasks)),
            chunks_size=self.chunks_size,
        )
        info["autosim"] = {"repo_id": self.repo_id, "image_fallback": self.image_fallback}
        (self.root / "meta" / "info.json").write_text(
            json.dumps(info, indent=2) + "\n", encoding="utf-8"
        )
        with (self.root / "meta" / "tasks.jsonl").open("w", encoding="utf-8") as fh:
            for task, idx in sorted(self.tasks.items(), key=lambda kv: kv[1]):
                fh.write(json.dumps({"task_index": idx, "task": task}) + "\n")
        with (self.root / "meta" / "episodes.jsonl").open("w", encoding="utf-8") as fh:
            for row in self.episodes:
                fh.write(json.dumps(row) + "\n")

    def close(self) -> None:
        """Rewrite meta (idempotent)."""
        if self.episodes or self.tasks:
            self.flush_meta()


def maybe_create_writer(config: Mapping[str, Any]) -> Optional[LeRobotWriter]:
    """Factory from :func:`schema.config_from_habitat` dict."""
    if not config.get("enabled"):
        return None
    root = Path(str(config["root"]))
    return LeRobotWriter(
        root,
        fps=float(config.get("fps", 10.0)),
        repo_id=str(config.get("repo_id", "autosim/demo")),
        robot_type=str(config.get("robot_type", "differential")),
        task=str(config.get("task", "navigate")),
        image_hw=tuple(config["image_hw"]) if config.get("image_hw") else None,
        laser_bins=int(config.get("laser_bins", 0)),
        include_image=bool(config.get("include_image", True)),
        chunks_size=int(config.get("chunks_size", 1000)),
        video_codec=str(config.get("video_codec", "mp4v")),
    )
