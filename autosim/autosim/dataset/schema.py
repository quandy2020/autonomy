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

"""LeRobot v2.1 feature schema for differential-drive AutoSim demos."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

# observation.state layout (meters / radians / m/s / rad/s)
STATE_NAMES: tuple[str, ...] = ("x", "y", "yaw", "vx", "wz")
ACTION_NAMES: tuple[str, ...] = ("vx", "wz")

CODEBASE_VERSION = "v2.1"
DEFAULT_ROBOT_TYPE = "differential"


def feature_dict(
    *,
    image_hw: tuple[int, int] | None = (480, 640),
    laser_bins: int = 0,
    include_image: bool = True,
) -> dict[str, Any]:
    """Build ``meta/info.json`` ``features`` mapping."""
    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32",
            "shape": [len(STATE_NAMES)],
            "names": list(STATE_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": [len(ACTION_NAMES)],
            "names": list(ACTION_NAMES),
        },
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
        "next.done": {"dtype": "bool", "shape": [1], "names": None},
    }
    if include_image and image_hw is not None:
        h, w = int(image_hw[0]), int(image_hw[1])
        features["observation.images.rgb"] = {
            "dtype": "video",
            "shape": [h, w, 3],
            "names": ["height", "width", "channels"],
        }
    if laser_bins > 0:
        features["observation.laser"] = {
            "dtype": "float32",
            "shape": [int(laser_bins)],
            "names": None,
        }
    return features


def dataset_info(
    *,
    fps: float,
    features: Mapping[str, Any],
    robot_type: str = DEFAULT_ROBOT_TYPE,
    total_episodes: int = 0,
    total_frames: int = 0,
    total_tasks: int = 1,
    chunks_size: int = 1000,
) -> dict[str, Any]:
    """LeRobot ``meta/info.json`` body."""
    return {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": robot_type,
        "total_episodes": int(total_episodes),
        "total_frames": int(total_frames),
        "total_tasks": int(total_tasks),
        "total_videos": int(total_episodes) if "observation.images.rgb" in features else 0,
        "total_chunks": max(1, (int(total_episodes) + int(chunks_size) - 1) // int(chunks_size))
        if total_episodes
        else 0,
        "chunks_size": int(chunks_size),
        "fps": float(fps),
        "splits": {"train": f"0:{int(total_episodes)}"} if total_episodes else {"train": "0:0"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": (
            "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
        ),
        "features": dict(features),
    }


def config_from_habitat(habitat: Mapping[str, Any]) -> dict[str, Any]:
    """Read ``habitat.dataset`` / ``habitat.record`` with defaults."""
    block: Mapping[str, Any] = {}
    for key in ("dataset", "record"):
        raw = habitat.get(key)
        if isinstance(raw, Mapping):
            block = raw
            break
    camera = (habitat.get("sensors") or {}).get("camera") or {}
    lidar = (habitat.get("sensors") or {}).get("lidar_2d") or {}
    width = int(camera.get("width", 640))
    height = int(camera.get("height", 480))
    return {
        "enabled": bool(block.get("enabled", False)),
        "root": str(block.get("root") or block.get("path") or "datasets/autosim").strip(),
        "repo_id": str(block.get("repo_id") or "autosim/demo").strip(),
        "task": str(block.get("task") or "navigate").strip(),
        "fps": float(block.get("fps", 10.0)),
        "episode_frames": int(block.get("episode_frames", 0)),  # 0 = whole run
        "max_episodes": int(block.get("max_episodes", 0)),  # 0 = unlimited
        "include_image": bool(block.get("include_image", True)),
        "include_laser": bool(block.get("include_laser", True)),
        "include_people": bool(block.get("include_people", False)),
        "image_hw": (height, width),
        "laser_bins": int(lidar.get("num_beams", 360))
        if bool(block.get("include_laser", True))
        else 0,
        "robot_type": str(block.get("robot_type") or DEFAULT_ROBOT_TYPE),
        "chunks_size": int(block.get("chunks_size", 1000)),
        "video_codec": str(block.get("video_codec") or "mp4v"),
    }
