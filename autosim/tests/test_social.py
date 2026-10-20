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

"""Social force, robot reactions, agents YAML, and World."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from autosim.scene.agents.roster import Agents
from autosim.scene.agents.behavior import Behavior
from autosim.scene.agents.force import Force


ROOT = Path(__file__).resolve().parents[1]


def test_behavior_ids():
    assert Behavior.kind_name(1) == "regular"
    assert Behavior.kind_name("scared") == "scared"
    assert Behavior.kind_name({"type": 5}) == "curious"
    with pytest.raises(ValueError, match="behavior"):
        Behavior.kind_name("dance")


def test_scared_flees_and_curious_approaches():
    behavior = Behavior(notice=5.0, scare=3.0, approach=0.5)
    position = np.array([[0.0, 0.0], [0.0, 0.0]], dtype=np.float64)
    goal = np.array([[10.0, 0.0], [10.0, 0.0]], dtype=np.float64)
    speed = np.array([1.0, 1.0], dtype=np.float64)
    kinds = np.array(["scared", "curious"], dtype=object)
    preferred, impassive = behavior.preferred(
        kinds, position, goal, speed, robot=np.array([1.0, 0.0]), robot_yaw=0.0
    )
    assert preferred[0, 0] < 0.0
    assert preferred[1, 0] > 0.0
    assert not np.any(impassive)


def test_impassive_marks_robot_as_body():
    behavior = Behavior()
    preferred, impassive = behavior.preferred(
        np.array(["impassive"], dtype=object),
        np.array([[0.0, 0.0]]),
        np.array([[1.0, 0.0]]),
        np.array([1.0]),
        robot=np.array([0.5, 0.0]),
    )
    assert preferred[0, 0] > 0.0
    assert bool(impassive[0])


def test_social_force_repels_neighbors():
    force = Force({"A": 5.0, "B": 0.3, "tau": 0.5})
    position = np.array([[0.0, 0.0], [0.3, 0.0]], dtype=np.float64)
    velocity = np.zeros_like(position)
    preferred = np.array([[1.0, 0.0], [-1.0, 0.0]], dtype=np.float64)
    accel = force.acceleration(position, velocity, preferred, radius=0.2)
    assert accel[0, 0] < preferred[0, 0] / 0.5
    assert accel[1, 0] > preferred[1, 0] / 0.5


def test_agents_yaml_loads_native_layout():
    agents = Agents.load(ROOT / "config/scenarios/agents_isaac.yaml")
    assert agents.count == 4
    assert set(agents.behaviors().tolist()) >= {
        "regular",
        "curious",
        "scared",
        "impassive",
    }
    assert agents.positions().shape == (4, 2)
    assert all(len(goals) >= 2 for goals in agents.goal_lists())


def test_world_fills_extents_from_grid():
    from autosim.scene import World

    grid = np.zeros((20, 40), dtype=np.int8)
    packed = (grid, 0.5, -2.0, -1.0, 40, 20)
    world = World.from_block(
        {
            "obstacle": {"enabled": False},
            "pedestrian": {"enabled": False},
        },
        packed,
    )
    ped = world.pedestrian.settings
    assert ped["x_length"] == pytest.approx(20.0)
    assert ped["y_length"] == pytest.approx(10.0)
    assert ped["center_x"] == pytest.approx(8.0)
    assert ped["center_y"] == pytest.approx(4.0)


def test_pedestrian_social_reacts_to_robot():
    from autosim.scene import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "social": True,
            "count": 1,
            "radius": 0.2,
            "speed": 1.0,
            "behavior": "scared",
            "x_length": 20.0,
            "y_length": 20.0,
            "spawn_radius": 0.0,
            "scare": 4.0,
            "notice": 5.0,
            "seed": 0,
        }
    )
    agent.position[:] = [[-2.0, 0.0]]
    agent.goal[:] = [[5.0, 0.0]]
    agent.velocity[:] = [[0.0, 0.0]]
    start = agent.position.copy()
    agent.step(0.1, robot=(0.0, 0.0, 0.0))
    assert agent.position[0, 0] < start[0, 0]


def test_pedestrian_loads_agents_file():
    from autosim.scene import Pedestrian

    agent = Pedestrian(
        {
            "enabled": True,
            "agents": str(ROOT / "config/scenarios/agents_isaac.yaml"),
            "x_length": 30.0,
            "y_length": 30.0,
            "spawn_radius": 0.5,
            "center_x": 8.0,
            "center_y": 0.0,
            "spawn_x": 7.93,
            "spawn_y": -0.39,
        }
    )
    assert agent.social
    assert agent.position.shape[0] == 4
    assert "curious" in agent.kinds.tolist()
    people = agent.people()
    assert len(people) == 4
    assert "behavior" in people[0]


def test_pedestrian_hit_radius_and_goal_relocate():
    from autosim.scene import Pedestrian
    from autosim.scene.field import Field

    agent = Pedestrian(
        {
            "enabled": True,
            "count": 1,
            "radius": 0.2,
            "laser_radius": 0.3,
            "x_length": 10.0,
            "y_length": 10.0,
            "spawn_radius": 0.0,
            "seed": 1,
        }
    )
    assert agent.hit_radius() == pytest.approx(0.3)
    grid = np.zeros((20, 20), dtype=np.int8)
    grid[:, 10:] = 100  # right half occupied
    field = Field(grid, 0.5, -5.0, -5.0)
    agent.position[:] = [[2.0, 0.0]]  # on occupied side
    agent.goal[:] = [[3.0, 0.0]]
    agent.goal_lists = [[(3.0, 0.0), (4.0, 0.0)]]
    agent.goal_index[:] = [0]
    agent.bind_field(field)
    assert agent.position[0, 0] < 0.0
    assert agent.goal_lists[0][0][0] < 0.0
