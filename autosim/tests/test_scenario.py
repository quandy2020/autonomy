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

"""Procedural scenario maps (mockamap types) without Habitat."""

import numpy as np
import pytest

from autosim.config import Config
from autosim.map import Map
from autosim.scenario import Scenario, Traffic
from autosim.scenario.field import Field
from autosim.simulator import Simulator


SMALL = {
    "seed": 7,
    "resolution": 0.5,
    "x_length": 6,
    "y_length": 6,
    "z_length": 2,
    "obstacle_number": 4,
    "road_width": 1.0,
    "add_wall_x": 1,
    "add_wall_y": 1,
    "num_nodes": 6,
    "connectivity": 0.5,
    "road_radius": 2,
    "spawn_radius": 0.6,
    "spawn_height": 1.0,
    "tree_dist": 2.0,
    "room_number": 2,
    "max_windows": 1,
    "window_size_min": 0.2,
    "window_size_max": 0.4,
    "add_ceiling": 0,
}


def _world(kind: str, mesh: str) -> Scenario:
    settings = dict(SMALL)
    settings["type"] = kind
    settings["mesh"] = mesh
    return Scenario(settings).build()


def test_perlin_posts_and_mazes_stay_inside_the_box(tmp_path):
    kinds = (
        "perlin",
        "perlin2d",
        "posts",
        "maze",
        "maze3d",
        "random_forest",
        "random_room",
    )
    for kind in kinds:
        world = _world(kind, str(tmp_path / f"{kind}.obj"))
        assert world.points.ndim == 2 and world.points.shape[1] == 3
        assert world.points.shape[0] > 0
        assert np.isfinite(world.points).all()
        assert world.mesh is not None and world.mesh.exists()
        text = world.mesh.read_text(encoding="utf-8")
        assert text.startswith("# autosim scenario")
        assert "\nv " in "\n" + text
        grid, resolution, ox, oy, width, height = world.grid
        assert grid.shape == (height, width)
        assert resolution == 0.5
        assert (grid == 0).any() and (grid == 100).any()
        assert abs(ox) > 0.0 or abs(oy) > 0.0


def test_forest_and_building_load_ply(tmp_path):
    ply = tmp_path / "scene.ply"
    ply.write_text(
        "ply\nformat ascii 1.0\n"
        "element vertex 4\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n"
        "0 0 0\n2 0 1\n0 2 1\n2 2 2\n",
        encoding="utf-8",
    )
    for kind in ("forest", "building"):
        settings = dict(SMALL)
        settings.update(type=kind, mesh=str(tmp_path / f"{kind}.obj"), ply=str(ply))
        world = Scenario(settings).build()
        assert world.points.shape[0] > 0
        assert world.mesh is not None and world.mesh.exists()


def test_forest_requires_ply():
    try:
        Scenario({"type": "forest", "resolution": 0.5, "x_length": 2, "y_length": 2, "z_length": 1, "ply": ""}).build()
    except ValueError as exc:
        assert "ply" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_spawn_column_is_free():
    world = _world("maze", "")
    grid, resolution, ox, oy, width, height = world.grid
    col = int(np.floor((0.0 - ox) / resolution))
    row = int(np.floor((0.0 - oy) / resolution))
    assert 0 <= row < height and 0 <= col < width
    assert int(grid[row, col]) == 0


def test_empty_path_installs_scenario_mesh(tmp_path):
    class FakeConfig:
        def __init__(self):
            self.gpu_device_id = -1
            self.enable_physics = False
            self.scene_id = ""

    class FakeHabitat:
        SimulatorConfiguration = FakeConfig

    mesh = tmp_path / "stage.obj"
    simulator = Simulator(
        backend="minimal",
        width=16,
        height=12,
        settings={
            "habitat": {
                "path": "",
                "gpu": 0,
                "scenario": {
                    "enabled": True,
                    "type": "posts",
                    "mesh": str(mesh),
                    **SMALL,
                },
            }
        },
        open_session=False,
    )
    configuration = simulator.habitat_configuration(FakeHabitat)
    assert configuration.scene_id == str(mesh)
    assert simulator.scenario_points.shape[0] > 0
    cloud, grid, _, _, _, _, _ = Map({"grid": {}}).sample(simulator, (0.0, 0.0))
    assert cloud.shape == simulator.scenario_points.shape
    assert grid.shape[0] > 0


def test_obstacle_stays_inside_and_moves():
    from autosim.scenario import Obstacle

    agent = Obstacle(
        {
            "enabled": True,
            "count": 4,
            "radius": 0.3,
            "speed_min": 0.5,
            "speed_max": 0.5,
            "x_length": 8,
            "y_length": 8,
            "seed": 3,
        }
    )
    start = agent.position.copy()
    agent.step(0.2)
    assert agent.points().shape == (4, 3)
    assert not np.allclose(agent.position, start)
    limit = 4.0 - 0.3
    assert np.all(np.abs(agent.position) <= limit + 1e-6)


def test_pedestrian_walks_toward_a_goal():
    from autosim.scenario import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "count": 3,
            "radius": 0.25,
            "speed": 1.0,
            "x_length": 10,
            "y_length": 10,
            "seed": 5,
            "goal_tolerance": 0.2,
        }
    )
    start = agent.position.copy()
    agent.step(0.1)
    poses = agent.poses()
    assert poses.shape == (3, 3)
    assert np.linalg.norm(agent.position - start, axis=1).min() > 0.0
    limit = 5.0 - 0.25
    assert np.all(np.abs(agent.position) <= limit + 1e-6)


def test_obstacle_reverses_at_the_boundary():
    from autosim.scenario import Obstacle

    agent = Obstacle(
        {
            "enabled": True,
            "count": 1,
            "radius": 0.3,
            "speed_min": 1.0,
            "speed_max": 1.0,
            "x_length": 4.0,
            "y_length": 4.0,
            "spawn_radius": 0.0,
            "seed": 0,
        }
    )
    agent.position[:] = [1.6, 0.0]
    agent.velocity[:] = [1.0, 0.0]
    agent.step(0.5)
    assert agent.velocity[0, 0] < 0.0
    assert agent.position[0, 0] <= 1.7 + 1e-6


def test_obstacle_bounces_off_an_occupied_cell():
    from autosim.scenario import Obstacle

    agent = Obstacle(
        {
            "enabled": True,
            "count": 1,
            "radius": 0.2,
            "speed_min": 1.0,
            "speed_max": 1.0,
            "x_length": 10.0,
            "y_length": 10.0,
            "spawn_radius": 0.0,
            "seed": 0,
        }
    )
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[4, 4] = 100
    agent.bind_field(Field(grid, 0.5, -2.0, -2.0))
    agent.position[:] = [-0.3, 0.0]
    agent.velocity[:] = [1.0, 0.0]
    agent.step(0.3)
    assert agent.velocity[0, 0] < 0.0
    assert agent.position[0, 0] <= -0.3 + 1e-6


def test_pedestrian_replaces_a_reached_goal():
    from autosim.scenario import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "count": 2,
            "radius": 0.2,
            "speed": 1.0,
            "x_length": 10.0,
            "y_length": 10.0,
            "spawn_radius": 0.0,
            "goal_tolerance": 0.4,
            "seed": 1,
        }
    )
    agent.position[:] = [1.0, 1.0]
    agent.goal[:] = [1.0, 1.0]
    previous = agent.goal.copy()
    agent.step(0.05)
    assert not np.allclose(agent.goal, previous)


def test_map_smaller_than_an_agent_is_rejected():
    from autosim.scenario import Obstacle, Pedestrian

    with pytest.raises(ValueError, match="smaller than an obstacle"):
        Obstacle(
            {
                "enabled": True,
                "count": 1,
                "radius": 1.0,
                "x_length": 1.0,
                "y_length": 4.0,
            }
        )
    with pytest.raises(ValueError, match="smaller than a pedestrian"):
        Pedestrian(
            {
                "enabled": True,
                "count": 1,
                "radius": 1.0,
                "x_length": 4.0,
                "y_length": 1.0,
            }
        )


def test_startup_rejects_bad_fractions_and_speeds():
    with pytest.raises(ValueError, match="fill"):
        Config.validate_scenario({"enabled": True, "type": "maze", "fill": 0.0})
    with pytest.raises(ValueError, match="connectivity"):
        Config.validate_scenario({"enabled": True, "type": "maze", "connectivity": 1.2})
    with pytest.raises(ValueError, match="speed_min"):
        Config.validate_scenario(
            {
                "enabled": True,
                "type": "maze",
                "obstacle": {
                    "enabled": True,
                    "count": 1,
                    "radius": 0.3,
                    "speed_min": 2.0,
                    "speed_max": 0.5,
                },
            }
        )
    with pytest.raises(ValueError, match="height"):
        Config.validate_scenario(
            {
                "enabled": True,
                "type": "maze",
                "obstacle": {"enabled": True, "count": 1, "radius": 0.3, "height": 0.0},
            }
        )


def test_agents_inherit_map_extent_unless_overridden():
    block = {
        "x_length": 20.0,
        "y_length": 18.0,
        "spawn_radius": 1.1,
        "obstacle": {"enabled": False},
        "pedestrian": {"enabled": False, "x_length": 6.0},
    }
    traffic = Traffic.from_block(block, None)
    assert traffic.obstacle.settings["x_length"] == 20.0
    assert traffic.obstacle.settings["y_length"] == 18.0
    assert traffic.obstacle.settings["spawn_radius"] == 1.1
    assert traffic.pedestrian.settings["x_length"] == 6.0
    assert traffic.pedestrian.settings["y_length"] == 18.0
    assert traffic.pedestrian.settings["spawn_radius"] == 1.1


def test_unknown_generator_is_rejected():
    world = Scenario(
        {"type": "maze", "resolution": 0.5, "x_length": 2, "y_length": 2, "z_length": 1}
    )
    world.kind = "missing"
    with pytest.raises(ValueError, match="no generator"):
        world.make_occupancy()


def test_lidar_keeps_the_nearer_dynamic_hit():
    traffic = Traffic.from_block(
        {
            "x_length": 20.0,
            "y_length": 20.0,
            "spawn_radius": 0.0,
            "obstacle": {
                "enabled": True,
                "count": 1,
                "radius": 0.5,
                "height": 2.0,
                "speed_min": 0.0,
                "speed_max": 0.0,
                "seed": 1,
            },
            "pedestrian": {
                "enabled": True,
                "count": 1,
                "radius": 0.3,
                "height": 1.7,
                "speed": 1.0,
                "x_length": 20.0,
                "y_length": 20.0,
                "spawn_radius": 0.0,
                "seed": 1,
            },
        },
        None,
    )
    traffic.obstacle.position[:] = [[2.0, 0.0]]
    distance = traffic.nearest((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 30.0)
    assert distance is not None and abs(distance - 1.5) < 1e-6

    traffic.obstacle.position[:] = [[20.0, 20.0]]
    traffic.pedestrian.position[:] = [[3.0, 0.0]]
    distance = traffic.nearest((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 30.0)
    assert distance is not None and abs(distance - 2.7) < 1e-6

    traffic.pedestrian.position[:] = [[0.0, 0.0]]
    cap = traffic.nearest((0.0, 0.0, 3.0), (0.0, 0.0, -1.0), 10.0)
    assert cap is not None and abs(cap - (3.0 - 1.7)) < 1e-6


def test_pedestrian_follows_the_free_direction():
    from autosim.scenario import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "count": 1,
            "radius": 0.2,
            "speed": 1.0,
            "x_length": 10.0,
            "y_length": 10.0,
            "spawn_radius": 0.0,
            "goal_tolerance": 0.05,
            "seed": 0,
        }
    )
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[:, 4:] = 100
    agent.bind_field(Field(grid, 0.5, -2.0, -2.0))
    agent.position[:] = [-0.25, 0.0]
    agent.goal[:] = [2.0, 0.0]
    agent.step(0.2)
    assert agent.position[0, 1] > 0.0
    assert agent.position[0, 0] < 0.0


def test_pedestrian_replaces_a_goal_when_stuck():
    from autosim.scenario import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "count": 1,
            "radius": 0.2,
            "speed": 1.0,
            "x_length": 10.0,
            "y_length": 10.0,
            "spawn_radius": 0.0,
            "goal_tolerance": 0.05,
            "stuck_steps": 2,
            "seed": 0,
        }
    )
    grid = np.full((8, 8), 100, dtype=np.int8)
    grid[4, 4] = 0
    agent.bind_field(Field(grid, 0.5, -2.0, -2.0))
    agent.position[:] = [0.25, 0.25]
    agent.goal[:] = [4.0, 4.0]
    previous = agent.goal.copy()
    agent.step(0.3)
    agent.step(0.3)
    assert not np.allclose(agent.goal, previous)


def test_refreshed_goal_stays_off_walls():
    from autosim.scenario import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "count": 1,
            "radius": 0.2,
            "speed": 1.0,
            "x_length": 8.0,
            "y_length": 8.0,
            "spawn_radius": 0.0,
            "goal_tolerance": 0.4,
            "seed": 2,
        }
    )
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[2:6, 2:6] = 100
    agent.bind_field(Field(grid, 1.0, -4.0, -4.0))
    agent.position[:] = [-3.2, -3.2]
    agent.goal[:] = [-3.2, -3.2]
    agent.step(0.05)
    assert not agent.field.blocked(agent.goal, 0.2)


def test_pedestrian_stays_outside_the_robot_disk():
    from autosim.scenario import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "count": 1,
            "radius": 0.2,
            "speed": 1.0,
            "x_length": 10.0,
            "y_length": 10.0,
            "spawn_radius": 1.0,
            "goal_tolerance": 0.05,
            "seed": 0,
        }
    )
    agent.position[:] = [0.2, 0.0]
    agent.goal[:] = [0.0, 0.0]
    agent.step(0.2)
    assert np.linalg.norm(agent.position[0]) >= 1.2 - 1e-6


def test_pedestrian_does_not_enter_an_obstacle():
    from autosim.scenario import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "count": 1,
            "radius": 0.2,
            "speed": 1.0,
            "x_length": 10.0,
            "y_length": 10.0,
            "spawn_radius": 0.0,
            "goal_tolerance": 0.05,
            "seed": 0,
        }
    )
    agent.position[:] = [0.0, 0.0]
    agent.goal[:] = [0.0, 3.0]
    agent.step(0.1, bodies=np.array([[0.25, 0.0]]), body_radius=0.2)
    assert agent.position[0, 0] < 0.0
