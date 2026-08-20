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

from autosim.sensors import Sensors
from autosim.simulator import Simulator
from tests.fake_simulator import FakeSimulator


def test_fake_laser_ranges_and_points():
    simulator = FakeSimulator(width=64, height=48)
    simulator.reset(x=0.0, y=0.0, yaw=0.0)
    ranges = simulator.laser_ranges(
        angle_min=-np.pi, angle_max=np.pi, num_beams=36, range_max=10.0
    )
    assert ranges.shape == (36,)
    assert np.allclose(ranges, 5.0)

    points = simulator.lidar_points(
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
    cloud = sensors.sample_points(simulator)
    assert cloud.shape == (16, 3)
    color, depth = sensors.sample_camera(simulator)
    assert color.shape[2] == 3
    assert depth.shape == color.shape[:2]


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


def test_husky_camera_offset_is_above_agent():
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": "",
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": "urdf/husky.urdf"},
                "sensors": {"camera": {"sensor_height": 0.6}},
            }
        },
        open_session=False,
    )
    x, y, z = simulator.camera_local_offset()
    assert abs(x) < 1e-9
    assert abs(y - 0.6) < 1e-9
    assert abs(z) < 1e-9
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
                "robot": {"urdf": "urdf/turtlebot3_burger.urdf"},
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
                "robot": {"urdf": "urdf/turtlebot3_burger.urdf"},
            }
        },
        open_session=False,
    )
    x, y, z = simulator.camera_local_offset()
    assert abs(x) < 1e-6
    assert abs(y - 0.21) < 1e-6
    assert abs(z) < 1e-6


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


def test_raycast_hit_distance_miss_on_has_hits_false():
    class Results:
        hits = []

        def has_hits(self):
            return False

    assert Simulator.raycast_hit_distance(Results()) is None
    assert Simulator.raycast_hit_distance(None) is None
