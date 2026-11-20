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

import numpy as np
import pytest
import math
from pathlib import Path

from autosim.bridge import Sensors
from autosim.backends.habitat.simulator import Simulator
from tests.fake_simulator import FakeSimulator


def test_cone_directions_cover_the_spot():
    """ToF cone samples stay inside the half-angle and reach the rim."""
    half = math.radians(5.0)
    directions = Simulator.cone_directions(np.eye(3), half, 9)
    assert len(directions) == 9
    angles = [math.acos(max(-1.0, min(1.0, float(direction[0])))) for direction in directions]
    assert max(angles) <= half + 1e-9
    assert max(angles) > half * 0.5, "samples must fill the cone, not hug the axis"
    assert all(abs(float(np.linalg.norm(d)) - 1.0) < 1e-9 for d in directions)


def test_cone_directions_follow_the_link_rotation():
    yaw = math.pi / 2.0
    rotation = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    )
    axis = Simulator.cone_directions(rotation, 0.0, 4)
    assert len(axis) == 1
    assert np.allclose(axis[0], [0.0, 1.0, 0.0], atol=1e-9)


def test_cone_directions_reject_a_degenerate_axis():
    assert Simulator.cone_directions(np.zeros((3, 3)), math.radians(5.0), 9) == []


def test_fake_laser_ranges_and_points():
    simulator = FakeSimulator(width=64, height=48)
    simulator.reset(x=0.0, y=0.0, yaw=0.0)
    ranges = simulator.laser_ranges(
        angle_min=-np.pi, angle_max=np.pi, num_beams=36, range_max=10.0
    )
    assert ranges.shape == (36,)
    assert np.allclose(ranges, 5.0)

    points, rings = simulator.lidar_points(
        h_min=-np.pi,
        h_max=np.pi,
        h_beams=8,
        v_min=-0.2,
        v_max=0.2,
        v_rings=2,
        range_max=10.0,
    )
    assert points.ndim == 2 and points.shape[1] == 3
    assert points.shape[0] == 16
    assert rings.shape == (16,)
    assert set(rings.tolist()) == {0.0, 1.0}

    sensors = Sensors(
        angle_min=-np.pi,
        angle_max=np.pi,
        num_beams=36,
        range_min=0.1,
        range_max=10.0,
        lidar_3d={
            "horizontal": {"angle_min": -np.pi, "angle_max": np.pi, "num_beams": 8},
            "vertical": {"angle_min": -0.2, "angle_max": 0.2, "num_rings": 2},
            "range_min": 0.1,
            "range_max": 10.0,
            "noise": 0.0,
        },
    )
    clipped = sensors.sample_laser(simulator)
    assert clipped.shape == (36,)
    cloud, cloud_rings = sensors.sample_points(simulator)
    assert cloud.shape == (16, 3)
    assert cloud_rings.shape == (16,)
    color, depth = sensors.sample_camera(simulator)
    assert color.shape[2] == 3
    assert depth.shape == color.shape[:2]


def test_constrain_motion_blocks_navmesh_and_occupancy():
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={"habitat": {"path": "", "spawn": [0.0, 0.0, 0.0], "robot": {"urdf": ""}}},
        open_session=False,
    )

    class Pathfinder:
        is_loaded = True

        def try_step(self, start, end):
            del start, end
            return (1.0, 0.0, 0.0)

    class Session:
        pathfinder = Pathfinder()

    simulator.session = Session()
    simulator.floor_y = 0.0
    x, y = simulator.constrain_motion(0.0, 0.0, 4.0, 0.0)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0)

    simulator.session = None
    from autosim.scene.map import Map

    mapper = Map({"grid": {"resolution": 1.0, "z_min": 0.1, "z_max": 2.0}})
    mapper.cached_grid = (
        np.array([[0, 0, 100]], dtype=np.int8),
        1.0,
        0.0,
        0.0,
        3,
        1,
    )
    simulator.map_builder = mapper
    x, y = simulator.constrain_motion(0.5, 0.5, 2.5, 0.5)
    assert x < 2.0
    assert y == pytest.approx(0.5)
    assert mapper.disc_occupied(x, y, radius=0.16) is False
    simulator.block_motion = False
    x, y = simulator.constrain_motion(0.5, 0.5, 2.5, 0.5)
    assert x == pytest.approx(2.5)


def test_constrain_motion_nudges_out_of_occupied_start():
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={"habitat": {"path": "", "spawn": [0.0, 0.0, 0.0], "robot": {"urdf": ""}}},
        open_session=False,
    )
    from autosim.scene.map import Map

    mapper = Map({"grid": {"resolution": 1.0, "z_min": 0.1, "z_max": 2.0}})
    mapper.cached_grid = (
        np.array([[0, 100, 0]], dtype=np.int8),
        1.0,
        0.0,
        0.0,
        3,
        1,
    )
    simulator.map_builder = mapper
    simulator.session = None
    # Occupied cell center; cmd_vel stays inside the wall.
    x, y = simulator.constrain_motion(1.5, 0.5, 1.5, 0.5)
    assert mapper.disc_occupied(x, y, radius=0.16) is False


def test_simulator_has_no_mock_backend():
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={"habitat": {"path": "", "spawn": [0.0, 0.0, 0.0], "robot": {"urdf": ""}}},
        open_session=False,
    )
    assert not hasattr(simulator, "use_mock")
    assert simulator.session is None
    assert simulator.snap_to_navmesh() == (0.0, 0.0, 0.0)
    assert Simulator.finite_xyz((1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)
    assert Simulator.finite_xyz((float("nan"), 0.0, 0.0)) is None


def test_refine_floor_y_from_mesh_pulls_down_to_glb():
    """Navmesh bias above the GLB floor must lower floor_y for lidar/TF."""
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={"habitat": {"path": "", "spawn": [0.0, 0.0, 0.0], "robot": {"urdf": ""}}},
        open_session=False,
    )
    simulator.session = object()  # mark live without opening Habitat
    simulator.floor_y = 0.12
    simulator.x = 1.0
    simulator.y = 2.0
    simulator.yaw = 0.3

    def fake_cast(ox, oy, oz, yaw, pitch, range_max, *, ignore_object_id=None):
        assert pitch < 0.0  # downward
        assert oy > simulator.floor_y  # probe above agent
        assert ignore_object_id is None  # no articulated robot in this fixture
        return (ox, -oz, 0.01)  # map z_up = geometric floor

    simulator.cast_from_origin = fake_cast  # type: ignore[method-assign]
    assert simulator.refine_floor_y_from_mesh() == pytest.approx(0.01)
    assert simulator.floor_y == pytest.approx(0.01)


def test_refine_floor_y_ignores_articulated_robot():
    """The floor probe must skip the robot hull sitting over the spawn point."""
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={"habitat": {"path": "", "spawn": [0.0, 0.0, 0.0], "robot": {"urdf": ""}}},
        open_session=False,
    )
    simulator.session = object()
    simulator.floor_y = 0.0

    class Robot:
        object_id = 42

    simulator.articulated = Robot()
    seen = {}

    def fake_cast(ox, oy, oz, yaw, pitch, range_max, *, ignore_object_id=None):
        seen["ignore_object_id"] = ignore_object_id
        return (ox, -oz, 0.0)

    simulator.cast_from_origin = fake_cast  # type: ignore[method-assign]
    assert simulator.refine_floor_y_from_mesh() == pytest.approx(0.0)
    assert seen["ignore_object_id"] == 42


def test_navmesh_height_candidates_include_bounds_and_samples():
    """Elevated scenes need height probes beyond the old 0…1.5 list."""
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={"habitat": {"path": "", "spawn": [0.0, 0.0, 0.0], "robot": {"urdf": ""}}},
        open_session=False,
    )
    simulator.floor_y = 0.05

    class Pathfinder:
        def get_bounds(self):
            return ((0.0, 2.4, 0.0), (10.0, 5.2, 10.0))

        def get_random_navigable_point(self):
            return (1.0, 3.1, -2.0)

    heights = simulator.navmesh_height_candidates(Pathfinder())
    assert 0.05 in heights
    assert 2.4 in heights
    assert 5.2 in heights
    assert 3.1 in heights
    assert max(heights) >= 5.0


def test_snap_to_navmesh_uses_elevated_height_probe():
    """Spawn at z=0 must still snap when only an elevated navmesh island exists."""
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={"habitat": {"path": "", "spawn": [0.0, 0.0, 0.0], "robot": {"urdf": ""}}},
        open_session=False,
    )

    class Pathfinder:
        is_loaded = True

        def get_bounds(self):
            return ((-1.0, 4.0, -1.0), (1.0, 4.5, 1.0))

        def get_random_navigable_point(self):
            return (0.2, 4.2, -0.1)

        def snap_point(self, point):
            y = float(point[1])
            if abs(y - 4.2) > 0.6:
                return None
            return (0.0, 4.2, 0.0)

    class Session:
        pathfinder = Pathfinder()

        def get_agent(self, _index):
            raise RuntimeError("unused")

    simulator.session = Session()
    simulator.x = 0.0
    simulator.y = 0.0
    simulator.yaw = 0.0
    simulator.floor_y = 0.0
    # Avoid Habitat agent set_state; only verify floor selection.
    simulator.set_pose = lambda x, y, yaw: (
        setattr(simulator, "x", float(x)),
        setattr(simulator, "y", float(y)),
        setattr(simulator, "yaw", float(yaw)),
    )  # type: ignore[method-assign]
    simulator.refine_floor_y_from_mesh = lambda **_kwargs: float(simulator.floor_y)  # type: ignore[method-assign]

    pose = simulator.snap_to_navmesh()
    assert simulator.floor_y == pytest.approx(4.2)
    assert pose[0] == pytest.approx(0.0)
    assert pose[1] == pytest.approx(0.0)


def test_husky_camera_offset_is_above_agent():
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": "",
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": "robot/husky/husky.urdf"},
                "sensors": {"camera": {"sensor_height": 0.6}},
            }
        },
        open_session=False,
    )
    x, y, z = simulator.camera_local_offset()
    assert abs(x) < 1e-9
    assert abs(y - 0.6) < 1e-9
    assert abs(z + 0.20) < 1e-9
    assert not simulator.should_instance_urdf()


def test_habitat_plus_half_pi_looks_rearward():
    look_x, _, look_z = Simulator.habitat_look_direction(np.pi / 2.0)
    assert look_x < -0.99
    assert abs(look_z) < 1e-9


def test_habitat_agent_yaw_matches_ros_right_hand():
    """ROS +90° (CCW) faces map +Y = Habitat -Z (map_y = -Habitat_Z)."""
    x0, y0, z0 = Simulator.habitat_agent_forward(0.0)
    assert x0 > 0.99 and abs(y0) < 1e-9 and abs(z0) < 1e-9
    x, y, z = Simulator.habitat_agent_forward(np.pi / 2.0)
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and z < -0.99
    _, qy, _, qw = Simulator.habitat_agent_quaternion(np.pi / 2.0)
    assert abs(qy) < 1e-9 and qw > 0.99


def test_physics_enabled_without_urdf_instance():
    class FakeConfig:
        def __init__(self):
            self.gpu_device_id = -1
            self.enable_physics = False
            self.scene_id = ""

    class FakeHabitat:
        SimulatorConfiguration = FakeConfig
        STAGE_EMPTY_SCENE = "NONE"

    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": "",
                "gpu": 0,
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": "robot/turtlebot3_burger/turtlebot3_burger.urdf"},
            }
        },
        open_session=False,
    )
    assert not simulator.should_instance_urdf()
    configuration = simulator.habitat_configuration(FakeHabitat)
    assert configuration.enable_physics is True
    assert configuration.gpu_device_id == 0


def test_image_height_is_not_used_as_sensor_height():
    simulator = Simulator(
        backend="minimal",
        width=640,
        height=480,
        settings={
            "habitat": {
                "path": "",
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": ""},
                "sensors": {"camera": {"height": 480, "sensor_height": 0.6}},
            }
        },
        open_session=False,
    )
    _, y, _ = simulator.camera_local_offset()
    assert abs(y - 0.6) < 1e-9


def test_turtlebot_camera_offset_uses_urdf_height():
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": "",
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": "robot/turtlebot3_burger/turtlebot3_burger.urdf"},
            }
        },
        open_session=False,
    )
    x, y, z = simulator.camera_local_offset()
    assert abs(x) < 1e-6
    assert abs(y - 0.21) < 1e-6
    assert abs(z + 0.03) < 1e-6


def test_rgb_to_uint8_handles_float_and_rgba():
    rgba = np.zeros((2, 2, 4), dtype=np.float32)
    rgba[..., 0] = 1.0
    rgb = Simulator.rgb_to_uint8(rgba)
    assert rgb.shape == (2, 2, 3)
    assert rgb.dtype == np.uint8
    assert rgb[0, 0, 0] == 255


def test_align_camera_image_passthrough():
    color = np.zeros((2, 3, 3), dtype=np.uint8)
    color[0, 0] = [1, 2, 3]
    color[0, 2] = [7, 8, 9]
    aligned = Simulator.align_camera_image(color)
    np.testing.assert_array_equal(aligned[0, 0], [1, 2, 3])
    np.testing.assert_array_equal(aligned[0, 2], [7, 8, 9])
    depth = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    assert Simulator.align_camera_image(depth)[0, 0] == 1.0


def test_hide_articulated_sets_visible_false():
    class FakeNode:
        def __init__(self):
            self.is_visible = True
            self.children = []

    class FakeRobot:
        visible = True
        num_links = 0
        root_scene_node = FakeNode()

        def get_link_visual_nodes(self, link_id):
            return [self.root_scene_node] if link_id == -1 else []

    robot = FakeRobot()
    Simulator.hide_articulated(robot)
    assert robot.visible is False
    assert robot.root_scene_node.is_visible is False


def test_raycast_hit_distance_reads_habitat_hits_list():
    class Hit:
        ray_distance = 4.25

    class Results:
        hits = [Hit()]

        def has_hits(self):
            return True

    assert Simulator.raycast_hit_distance(Results()) == 4.25


def test_raycast_hit_distance_picks_closest_hit():
    class Hit:
        def __init__(self, distance):
            self.ray_distance = distance

    class Results:
        hits = [Hit(8.0), Hit(1.5), Hit(3.0)]

        def has_hits(self):
            return True

    assert Simulator.raycast_hit_distance(Results()) == 1.5


def test_resolve_mp3d_scene_dataset_from_glb_layout(tmp_path):
    scene_dir = tmp_path / "8WUmhLawc2A"
    scene_dir.mkdir()
    glb = scene_dir / "8WUmhLawc2A.glb"
    glb.write_bytes(b"glb")
    (scene_dir / "8WUmhLawc2A_semantic.ply").write_text("ply")
    (scene_dir / "8WUmhLawc2A.house").write_text("house")

    dataset_file, scene_id = Simulator.resolve_mp3d_scene_dataset(glb)
    assert scene_id == "8WUmhLawc2A/8WUmhLawc2A.glb"
    assert Path(dataset_file).name == "mp3d.scene_dataset_config.json"
    assert Path(dataset_file).parent == tmp_path
    assert Path(dataset_file).is_file()


def test_resolve_mp3d_scene_dataset_requires_semantic_assets(tmp_path):
    scene_dir = tmp_path / "sceneA"
    scene_dir.mkdir()
    glb = scene_dir / "sceneA.glb"
    glb.write_bytes(b"glb")
    assert Simulator.resolve_mp3d_scene_dataset(glb) is None


def test_semantic_ids_to_bgr_packs_object_ids():
    ids = np.array([[0, 1], [256, 65536]], dtype=np.uint32)
    bgr = Simulator.semantic_ids_to_bgr(ids)
    assert bgr.shape == (2, 2, 3)
    assert tuple(bgr[0, 1]) == (1, 0, 0)
    assert tuple(bgr[1, 0]) == (0, 1, 0)
    assert tuple(bgr[1, 1]) == (0, 0, 1)


def test_resolve_scene_dataset_prefers_path_when_config_in_wrong_dir(tmp_path):
    scene_dir = tmp_path / "sceneA"
    scene_dir.mkdir()
    glb = scene_dir / "sceneA.glb"
    glb.write_bytes(b"glb")
    (scene_dir / "sceneA_semantic.ply").write_text("ply")
    (scene_dir / "sceneA.house").write_text("house")
    wrong_config = tmp_path / "wrong" / "mp3d.scene_dataset_config.json"
    wrong_config.parent.mkdir()
    wrong_config.write_text("{}")

    hab_cfg = {
        "dataset_config": str(wrong_config),
        "scene_id": "sceneA/sceneA.glb",
        "path": str(glb),
    }
    resolved = Simulator.resolve_scene_dataset(hab_cfg)
    assert resolved is not None
    dataset_file, scene_id = resolved
    assert scene_id == "sceneA/sceneA.glb"
    assert Path(dataset_file).parent == tmp_path


def test_habitat_configuration_auto_upgrades_semantic_glb(tmp_path):
    scene_dir = tmp_path / "sceneA"
    scene_dir.mkdir()
    glb = scene_dir / "sceneA.glb"
    glb.write_bytes(b"glb")
    (scene_dir / "sceneA_semantic.ply").write_text("ply")
    (scene_dir / "sceneA.house").write_text("house")

    class FakeConfig:
        def __init__(self):
            self.gpu_device_id = -1
            self.enable_physics = False
            self.allow_sliding = False
            self.requires_textures = False
            self.scene_id = ""
            self.scene_dataset_config_file = ""
            self.load_semantic_mesh = False

    class FakeHabitat:
        SimulatorConfiguration = FakeConfig

    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": str(glb),
                "gpu": 0,
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": ""},
                "sensors": {"camera": {"semantic_enabled": True}},
            }
        },
        open_session=False,
    )
    configuration = simulator.habitat_configuration(FakeHabitat)
    assert configuration.load_semantic_mesh is True
    assert configuration.scene_id == "sceneA/sceneA.glb"
    assert Path(configuration.scene_dataset_config_file).is_file()
    class Results:
        hits = []

        def has_hits(self):
            return False

    assert Simulator.raycast_hit_distance(Results()) is None
    assert Simulator.raycast_hit_distance(None) is None


def wheeled_simulator(**robot) -> Simulator:
    """Minimal wheeled model with a stub articulation for wheel tests."""
    from types import SimpleNamespace

    from autosim.robot.joints import Joint

    settings = {
        "habitat": {
            "path": "",
            "spawn": [0.0, 0.0, 0.0],
            "robot": {
                "urdf": "robot/turtlebot3_burger/turtlebot3_burger.urdf",
                **robot,
            },
            "sensors": {"camera": {"sensor_height": 0.6}},
        }
    }
    simulator = Simulator(
        backend="minimal", width=64, height=48, settings=settings, open_session=False
    )
    calls: list[tuple[dict, float | None]] = []
    simulator.articulation = SimpleNamespace(
        joints={
            "wheel_left_joint": Joint(
                "wheel_left_joint", "continuous", "wheel_left_link"
            ),
            "wheel_right_joint": Joint(
                "wheel_right_joint", "continuous", "wheel_right_link"
            ),
        },
        positions=lambda: {"wheel_left_joint": 0.0, "wheel_right_joint": 0.0},
        set_positions=lambda updates, dt=None: calls.append((dict(updates), dt)),
    )
    return simulator, calls


def test_spin_wheels_splits_the_twist_between_sides():
    simulator, calls = wheeled_simulator(wheel_radius=0.033, wheel_separation=0.16)
    simulator.spin_wheels(0.3, 0.0, 0.5)
    updates, dt = calls[-1]
    assert dt == pytest.approx(0.5)
    # Straight line: both sides roll the same way at v / r.
    assert updates["wheel_left_joint"] == pytest.approx(0.3 / 0.033 * 0.5)
    assert updates["wheel_right_joint"] == pytest.approx(0.3 / 0.033 * 0.5)

    simulator.spin_wheels(0.0, 0.6, 0.5)
    updates, _ = calls[-1]
    # Turning in place: the left wheel rolls back, the right wheel forward.
    assert updates["wheel_left_joint"] == pytest.approx(-0.6 * 0.08 / 0.033 * 0.5)
    assert updates["wheel_right_joint"] == pytest.approx(0.6 * 0.08 / 0.033 * 0.5)


def test_spin_wheels_is_opt_in():
    simulator, calls = wheeled_simulator()
    assert simulator.wheel_mounts() == [
        ("wheel_left_joint", 1.0),
        ("wheel_right_joint", -1.0),
    ]
    simulator.spin_wheels(0.3, 0.0, 0.5)
    assert calls == []


def test_spin_wheels_uses_per_joint_radius_and_track():
    """Mixed wheel sizes (Google: 64 mm front, 38 mm rear) roll correctly."""
    simulator, calls = wheeled_simulator(
        wheel_radii={"wheel_left_joint": 0.05, "wheel_right_joint": 0.1},
        wheel_separations={"wheel_left_joint": 0.4, "wheel_right_joint": 0.2},
    )
    simulator.spin_wheels(0.0, 1.0, 1.0)
    updates, _ = calls[-1]
    # Left: v - w * track / 2 = -0.2 m/s over a 50 mm wheel.
    assert updates["wheel_left_joint"] == pytest.approx(-0.2 / 0.05)
    # Right: v + w * track / 2 = +0.1 m/s over a 100 mm wheel.
    assert updates["wheel_right_joint"] == pytest.approx(0.1 / 0.1)
