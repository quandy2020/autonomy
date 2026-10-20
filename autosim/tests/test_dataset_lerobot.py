# Copyright 2026 The Openbot Authors (duyongquan)

"""LeRobot dataset writer + collector unit tests."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest


def test_feature_schema_and_config():
    from autosim.dataset.schema import config_from_habitat, feature_dict

    feats = feature_dict(image_hw=(48, 64), laser_bins=16, include_image=True)
    assert feats["observation.state"]["shape"] == [5]
    assert feats["action"]["shape"] == [2]
    assert feats["observation.images.rgb"]["shape"] == [48, 64, 3]
    assert feats["observation.laser"]["shape"] == [16]

    cfg = config_from_habitat(
        {
            "dataset": {
                "enabled": True,
                "root": "/tmp/ds",
                "fps": 5,
                "include_laser": True,
                "include_people": True,
            },
            "sensors": {
                "camera": {"width": 64, "height": 48},
                "lidar_2d": {"num_beams": 16},
            },
        }
    )
    assert cfg["enabled"] is True
    assert cfg["fps"] == pytest.approx(5.0)
    assert cfg["laser_bins"] == 16
    assert cfg["image_hw"] == (48, 64)
    assert cfg["include_people"] is True


def test_lerobot_writer_roundtrip(tmp_path: Path):
    pytest.importorskip("pyarrow")
    from autosim.dataset.episode import Episode
    from autosim.dataset.frame import Frame
    from autosim.dataset.lerobot import LeRobotWriter

    writer = LeRobotWriter(
        tmp_path / "demo",
        fps=10.0,
        include_image=True,
        laser_bins=8,
        image_hw=(24, 32),
        task="navigate",
    )
    ep = Episode(task="navigate")
    for i in range(5):
        rgb = np.zeros((24, 32, 3), dtype=np.uint8)
        rgb[..., 0] = i * 40
        ep.append(
            Frame(
                timestamp=0.1 * i,
                state=np.array([i, 0.0, 0.0, 0.1, 0.0], dtype=np.float32),
                action=np.array([0.1, 0.0], dtype=np.float32),
                rgb=rgb,
                laser=np.full(8, 1.5, dtype=np.float32),
                meta={
                    "people": [
                        {"id": 1, "x": float(i), "y": 0.0, "yaw": 0.0, "behavior": "regular"}
                    ]
                },
            )
        )
    idx = writer.write_episode(ep)
    writer.close()
    assert idx == 0

    root = tmp_path / "demo"
    info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["codebase_version"] == "v2.1"
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 5
    assert (root / "data" / "chunk-000" / "episode_000000.parquet").is_file()
    people_sidecar = root / "data" / "chunk-000" / "episode_000000_people.jsonl"
    assert people_sidecar.is_file()
    assert '"behavior":"regular"' in people_sidecar.read_text(encoding="utf-8")
    episodes = (root / "meta" / "episodes.jsonl").read_text(encoding="utf-8").strip()
    assert '"length": 5' in episodes


def test_collector_fps_and_flush(tmp_path: Path):
    pytest.importorskip("pyarrow")
    from autosim.dataset.collector import DatasetCollector

    collector = DatasetCollector(
        {
            "enabled": True,
            "root": str(tmp_path / "run"),
            "fps": 10.0,
            "episode_frames": 3,
            "include_image": False,
            "include_laser": False,
            "laser_bins": 0,
            "image_hw": (48, 64),
            "task": "navigate",
            "repo_id": "autosim/test",
            "robot_type": "differential",
            "chunks_size": 1000,
            "video_codec": "mp4v",
        }
    )
    assert collector.enabled
    for i in range(6):
        collector.tick(
            0.1,
            timestamp=float(i) * 0.1,
            pose=(float(i), 0.0, 0.0),
            velocity=(0.2, 0.0),
            action=(0.2, 0.0),
        )
    collector.close()
    root = tmp_path / "run"
    info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
    # 6 ticks at 10 Hz → 6 frames; episode_frames=3 → two episodes.
    assert info["total_episodes"] == 2
    assert info["total_frames"] == 6


def test_config_validate_dataset():
    from autosim.runtime import Config

    with pytest.raises(ValueError, match="dataset.root"):
        Config.validate_dataset({"enabled": True, "root": ""})
    Config.validate_dataset({"enabled": False})
    Config.validate_dataset({"enabled": True, "root": "datasets/x", "fps": 10})
