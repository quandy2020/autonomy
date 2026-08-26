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

"""Tests for Map PLY cloud and grid projection (no Habitat)."""

from pathlib import Path
from types import SimpleNamespace

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


class _FakePathFinder:
    is_loaded = True

    def get_random_navigable_point(self):
        return (0.0, 1.25, 0.0)

    def get_bounds(self):
        return ((0.0, 0.0, 0.0), (2.0, 0.0, 2.0))

    def get_topdown_view(self, resolution, floor_height, island_radius):
        assert resolution == 0.5
        assert floor_height == 1.25
        assert island_radius == 0.5
        return np.array(
            [
                [True, False, True, False],
                [True, True, False, False],
            ],
            dtype=bool,
        )


def test_sample_prefers_navmesh_grid_when_available():
    from tests.fake_simulator import FakeSimulator

    simulator = FakeSimulator()
    simulator.session = SimpleNamespace(pathfinder=_FakePathFinder())
    builder = Map(
        {
            "ply": {
                "channel": "/map/points",
                "file": "",
                "range_max": 10.0,
                "horizontal": {"angle_min": -np.pi, "angle_max": np.pi, "num_beams": 8},
                "vertical": {"angle_min": -0.2, "angle_max": 0.2, "num_rings": 2},
            },
            "grid": {
                "channel": "/map",
                "resolution": 0.5,
                "z_min": 0.1,
                "z_max": 1.8,
            },
        }
    )

    _, grid, resolution, ox, oy, width, height = builder.sample(simulator, (0.0, 0.0))
    assert resolution == 0.5
    assert (width, height) == (4, 2)
    assert (ox, oy) == (0.0, 0.0)
    np.testing.assert_array_equal(
        grid,
        np.array(
            [
                [0, 100, 0, 100],
                [0, 0, 100, 100],
            ],
            dtype=np.int8,
        ),
    )
    # Habitat (x=0.25, z=0.25) → ROS cell (col=0, row=0), which is free.
    assert grid[0, 0] == 0
    # Habitat (x=0.75, z=0.25) → occupied; robot +X stays along columns.


def test_occupancy_range_stops_at_first_wall():
    grid = np.array(
        [
            [0, 0, 100, 0],
            [0, 0, 100, 0],
        ],
        dtype=np.int8,
    )
    # Origin in free cell (0.25, 0.25); shoot +X into occupied column 2.
    distance = Map.occupancy_range(
        origin_x=0.25,
        origin_y=0.25,
        yaw=0.0,
        grid=grid,
        resolution=1.0,
        grid_origin_x=0.0,
        grid_origin_y=0.0,
        range_max=10.0,
    )
    assert distance is not None
    assert 1.5 <= distance <= 2.5


def test_clip_laser_ranges_blocks_through_wall_hits():
    builder = Map({"grid": {"resolution": 1.0, "z_min": 0.1, "z_max": 2.0}})
    builder.cached_grid = (
        np.array([[0, 0, 100, 0]], dtype=np.int8),
        1.0,
        0.0,
        0.0,
        4,
        1,
    )
    # Fake Habitat hit beyond the wall (3.5 m); clip must shrink to wall.
    ranges = np.array([3.5], dtype=np.float32)
    clipped = builder.clip_laser_ranges(
        ranges,
        np.array([0.0]),
        origin_x=0.25,
        origin_y=0.25,
        yaw=0.0,
        range_max=10.0,
    )
    assert clipped[0] < 3.0
    assert clipped[0] >= 1.5
