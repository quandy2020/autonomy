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

import math

from autosim.robot import Robot


def test_clamp_and_integrate():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5)
    robot.set_twist(2.0, 3.0, t=0.0)
    x, y, yaw = robot.step(dt=0.1, t=0.1)
    assert abs(x - 0.05) < 1e-9
    assert abs(y) < 1e-9
    assert abs(yaw - 0.1) < 1e-9


def test_watchdog_zeros_velocity():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.2)
    robot.set_twist(0.5, 0.0, t=0.0)
    robot.step(dt=0.1, t=0.1)
    x0, _, _ = robot.pose()
    robot.step(dt=0.1, t=0.35)
    x1, _, _ = robot.pose()
    assert abs(x1 - x0) < 1e-9


def test_yaw_wrap():
    robot = Robot(max_linear=0.0, max_angular=2.0, watchdog_sec=10.0)
    robot.set_twist(0.0, 2.0, t=0.0)
    robot.step(dt=math.pi, t=0.1)
    _, _, yaw = robot.pose()
    assert abs(yaw) < 1e-6 or abs(abs(yaw) - math.pi) < 1e-6 or abs(yaw) <= math.pi


def test_odometry_integrates_matching_gt_when_noise_zero():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5, odometry_noise=0.0)
    robot.set_twist(0.5, 0.0, t=0.0)
    robot.step(dt=0.1, t=0.1)
    ox, oy, oyaw = robot.odometry_pose()
    gx, gy, gyaw = robot.pose()
    assert abs(ox - gx) < 1e-9
    assert abs(oy - gy) < 1e-9
    assert abs(oyaw - gyaw) < 1e-9


def test_odometry_noise_drifts_from_gt():
    robot = Robot(
        max_linear=1.0,
        max_angular=1.0,
        watchdog_sec=10.0,
        odometry_noise=0.2,
        seed=3,
    )
    robot.set_twist(1.0, 0.0, t=0.0)
    for step in range(20):
        robot.step(dt=0.1, t=0.1 * (step + 1))
    ox, oy, _ = robot.odometry_pose()
    gx, gy, _ = robot.pose()
    assert abs(ox - gx) + abs(oy - gy) > 1e-3


def test_pure_rotation_odometry_keeps_xy():
    """Spin-in-place must not invent translational odometry noise."""
    robot = Robot(
        max_linear=0.5,
        max_angular=1.0,
        watchdog_sec=10.0,
        odometry_noise=0.2,
        seed=7,
    )
    robot.set_twist(0.0, 1.0, t=0.0)
    for step in range(50):
        robot.step(dt=0.05, t=0.05 * (step + 1))
    ox, oy, oyaw = robot.odometry_pose()
    gx, gy, _ = robot.pose()
    assert abs(ox) < 1e-9
    assert abs(oy) < 1e-9
    assert abs(gx) < 1e-9
    assert abs(gy) < 1e-9
    assert abs(oyaw) > 0.1


def test_inertial_finite_difference():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5)
    robot.reset_inertial(yaw=0.0, t=0.0, speed=1.0)
    gyro_z, accel_x, accel_y = robot.update_inertial(yaw=0.1, speed=1.0, t=0.1)
    assert abs(gyro_z - 1.0) < 1e-6
    assert abs(accel_x) < 1e-6
    assert abs(accel_y) < 1e-9


def test_inertial_gaussian_noise_changes_reading():
    robot = Robot(
        max_linear=0.5,
        max_angular=1.0,
        watchdog_sec=0.5,
        gyro_noise=0.5,
        accel_noise=0.5,
        seed=1,
    )
    robot.reset_inertial(yaw=0.0, t=0.0, speed=1.0)
    gyro_z, accel_x, accel_y = robot.update_inertial(yaw=0.1, speed=1.0, t=0.1)
    assert abs(gyro_z - 1.0) > 1e-3
    assert abs(accel_x) > 1e-3 or abs(accel_y) > 1e-3


def test_teleport_resets_odometry():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=1.0, x=1.0, y=2.0, yaw=0.3)
    robot.odometry_x = 9.0
    robot.teleport(3.0, 4.0, math.pi)
    assert robot.pose() == (3.0, 4.0, math.pi)
    assert robot.odometry_pose() == (3.0, 4.0, math.pi)


def test_map_to_odom_keeps_ground_truth_on_map():
    ground_truth = (1.5, -0.5, 0.4)
    odometry = (0.2, 0.1, -0.15)
    parent = Robot.map_to_odom(ground_truth, odometry)
    px, py, pyaw = parent
    ox, oy, oyaw = odometry
    cosine = math.cos(pyaw)
    sine = math.sin(pyaw)
    map_x = px + cosine * ox - sine * oy
    map_y = py + sine * ox + cosine * oy
    map_yaw = Robot.wrap_yaw(pyaw + oyaw)
    assert abs(map_x - ground_truth[0]) < 1e-9
    assert abs(map_y - ground_truth[1]) < 1e-9
    assert abs(map_yaw - ground_truth[2]) < 1e-9
    assert Robot.map_to_odom(ground_truth, ground_truth) == (0.0, 0.0, 0.0)
