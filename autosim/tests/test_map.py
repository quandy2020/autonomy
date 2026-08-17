"""Tests for Map PLY cloud and grid projection (no Habitat)."""

from pathlib import Path

import numpy as np

from autosim.map import Map


def test_project_grid_carves_free_and_marks_occupied():
    builder = Map(
        {
            "ply": {
                "file": "",
                "channel": "/map/points",
                "range_max": 10.0,
                "horizontal": {"angle_min": -1, "angle_max": 1, "num_beams": 2},
                "vertical": {"angle_min": -0.1, "angle_max": 0.1, "num_rings": 2},
            },
            "grid": {
                "channel": "/map",
                "resolution": 1.0,
                "z_min": 0.0,
                "z_max": 2.0,
            },
        }
    )
    cloud = np.array([[2.5, 0.5, 1.0]], dtype=np.float32)
    grid, resolution, ox, oy, width, height = builder.project_grid(
        cloud, builder.settings["grid"], origin=(0.5, 0.5, 0.5)
    )
    assert resolution == 1.0
    assert np.any(grid == 100)
    assert np.any(grid == 0)


def test_sample_writes_ply(tmp_path):
    from tests.fake_simulator import FakeSimulator

    ply_path = tmp_path / "scene.ply"
    builder = Map(
        {
            "ply": {
                "channel": "",
                "file": str(ply_path),
                "range_max": 10.0,
                "horizontal": {"angle_min": -np.pi, "angle_max": np.pi, "num_beams": 8},
                "vertical": {"angle_min": -0.2, "angle_max": 0.2, "num_rings": 2},
            },
            "grid": {
                "channel": "/map",
                "resolution": 0.5,
                "z_min": 0.0,
                "z_max": 2.0,
            },
        }
    )
    cloud, grid, *_ = builder.sample(FakeSimulator(), (0.0, 0.0))
    assert cloud.shape[0] == 16
    assert grid.dtype == np.int8
    text = Path(ply_path).read_text(encoding="utf-8")
    assert text.startswith("ply")
    assert "element vertex 16" in text
