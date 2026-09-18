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

# Instant dynamics for tests that assert kinematic-only integration.
_SNAP = dict(max_linear_accel=0.0, max_linear_decel=0.0, max_angular_accel=0.0)


def test_pose_at_scan_fraction_matches_backward_unicycle():
    from autosim.simulator import Simulator

    x_e, y_e, yaw_e = 1.0, 0.5, 0.3
    linear, angular, period = 0.4, 0.8, 0.1
    # Start of scan (fraction=0) is age=period before end.
    x0, y0, yaw0 = Simulator.pose_at_scan_fraction(
        x_e, y_e, yaw_e, linear, angular, period, 0.0
    )
    # Forward from start with same twist for full period should recover end.
    from autosim.robot import Robot

    xf, yf, yawf = Robot.integrate_unicycle(x0, y0, yaw0, linear, angular, period)
    assert abs(xf - x_e) < 1e-9
    assert abs(yf - y_e) < 1e-9
    assert abs(Robot.wrap_yaw(yawf - yaw_e)) < 1e-9
    # End fraction is identity.
    xe2, ye2, yawe2 = Simulator.pose_at_scan_fraction(
        x_e, y_e, yaw_e, linear, angular, period, 1.0
    )
    assert (xe2, ye2, yawe2) == (x_e, y_e, yaw_e)


def test_map_hit_to_sensor_roundtrip_identity_forward():
    from autosim.simulator import Simulator

    sim = Simulator(backend="minimal", width=16, height=12, open_session=False)
    sim.x, sim.y, sim.yaw = 0.0, 0.0, 0.0
    sim.floor_y = 0.0
    # Point 2 m forward in sensor frame → map hit at (2,0,laser_z)
    ox, oy, oz = sim.map_laser_origin_at(0.0, 0.0, 0.0)
    hit = (ox + 2.0, oy, oz)
    sx, sy, sz = sim.map_hit_to_sensor(hit, 0.0, 0.0, 0.0)
    assert abs(sx - 2.0) < 1e-9
    assert abs(sy) < 1e-9
    assert abs(sz) < 1e-9


def test_clamp_and_integrate():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5, **_SNAP)
    robot.set_twist(2.0, 3.0, t=0.0)
    x, y, yaw = robot.step(dt=0.1, t=0.1)
    # Clamped to v=0.5, w=1.0; exact unicycle arc (R=v/w).
    assert abs(robot.linear - 0.5) < 1e-12
    assert abs(robot.angular - 1.0) < 1e-12
    assert abs(yaw - 0.1) < 1e-9
    assert abs(x - 0.5 * math.sin(0.1)) < 1e-9
    assert abs(y + 0.5 * (math.cos(0.1) - 1.0)) < 1e-9


def test_accel_ramps_toward_command():
    robot = Robot(
        max_linear=1.0,
        max_angular=2.0,
        watchdog_sec=10.0,
        max_linear_accel=0.5,
        max_linear_decel=1.0,
        max_angular_accel=1.0,
    )
    robot.set_twist(1.0, 1.0, t=0.0)
    robot.step(dt=0.1, t=0.1)
    assert abs(robot.linear - 0.05) < 1e-9
    assert abs(robot.angular - 0.1) < 1e-9
    assert abs(robot.body_accel_x - 0.5) < 1e-9


def test_decel_coasts_when_command_drops():
    robot = Robot(
        max_linear=1.0,
        max_angular=1.0,
        watchdog_sec=10.0,
        max_linear_accel=2.0,
        max_linear_decel=1.0,
        max_angular_accel=2.0,
    )
    robot.set_twist(1.0, 0.0, t=0.0)
    robot.step(dt=1.0, t=1.0)  # reach 1.0 m/s
    assert abs(robot.linear - 1.0) < 1e-9
    robot.set_twist(0.0, 0.0, t=1.0)
    robot.step(dt=0.2, t=1.2)
    assert abs(robot.linear - 0.8) < 1e-9  # braked at 1.0 m/s²


def test_watchdog_zeros_command_then_coasts():
    robot = Robot(
        max_linear=0.5,
        max_angular=1.0,
        watchdog_sec=0.2,
        max_linear_accel=10.0,
        max_linear_decel=1.0,
        max_angular_accel=10.0,
    )
    robot.set_twist(0.5, 0.0, t=0.0)
    robot.step(dt=0.1, t=0.1)
    assert robot.linear > 0.4
    # Watchdog trips; command→0 but velocity coasts under decel.
    robot.step(dt=0.1, t=0.35)
    assert abs(robot.cmd_linear) < 1e-12
    assert robot.linear > 0.0
    assert robot.linear < 0.5


def test_unicycle_arc_matches_closed_form():
    robot = Robot(max_linear=1.0, max_angular=2.0, watchdog_sec=10.0, **_SNAP)
    robot.set_twist(0.5, 1.0, t=0.0)
    robot.step(dt=0.2, t=0.2)
    x, y, yaw = robot.pose()
    # R=v/w=0.5; Δθ=0.2; x=R(sinθ'-sin0), y=-R(cosθ'-cos0)
    assert abs(yaw - 0.2) < 1e-9
    assert abs(x - 0.5 * math.sin(0.2)) < 1e-9
    assert abs(y + 0.5 * (math.cos(0.2) - 1.0)) < 1e-9


def test_yaw_wrap():
    robot = Robot(max_linear=0.0, max_angular=2.0, watchdog_sec=10.0, **_SNAP)
    robot.set_twist(0.0, 2.0, t=0.0)
    robot.step(dt=math.pi, t=0.1)
    _, _, yaw = robot.pose()
    assert abs(yaw) < 1e-6 or abs(abs(yaw) - math.pi) < 1e-6 or abs(yaw) <= math.pi


def test_odometry_integrates_matching_gt_when_noise_zero():
    robot = Robot(
        max_linear=0.5, max_angular=1.0, watchdog_sec=0.5, odometry_noise=0.0, **_SNAP
    )
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
        **_SNAP,
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
        **_SNAP,
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
    gyro_z, accel_x, accel_y, accel_z = robot.update_inertial(yaw=0.1, speed=1.0, t=0.1)
    assert abs(gyro_z - 1.0) < 1e-6
    assert abs(accel_x) < 1e-6
    assert abs(accel_y) < 1e-9
    assert abs(accel_z - Robot.GRAVITY) < 1e-9


def test_inertial_specific_force_without_fake_hardcode():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5, gravity=9.80665)
    fx, fy, fz = robot.specific_force(0.5, -0.2, 0.0)
    assert abs(fx - 0.5) < 1e-12
    assert abs(fy + 0.2) < 1e-12
    assert abs(fz - 9.80665) < 1e-12
    robot_no_g = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5, gravity=0.0)
    _, _, fz0 = robot_no_g.specific_force(0.0, 0.0, 0.0)
    assert abs(fz0) < 1e-12


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
    gyro_z, accel_x, accel_y, accel_z = robot.update_inertial(yaw=0.1, speed=1.0, t=0.1)
    assert abs(gyro_z - 1.0) > 1e-3
    assert abs(accel_x) > 1e-3 or abs(accel_y) > 1e-3
    assert abs(accel_z - Robot.GRAVITY) > 1e-3


def test_teleport_resets_odometry():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=1.0, x=1.0, y=2.0, yaw=0.3)
    robot.odometry_x = 9.0
    robot.teleport(3.0, 4.0, math.pi)
    assert robot.pose() == (3.0, 4.0, math.pi)
    assert robot.odometry_pose() == (3.0, 4.0, math.pi)
    assert robot.linear == 0.0
    assert robot.cmd_linear == 0.0


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
