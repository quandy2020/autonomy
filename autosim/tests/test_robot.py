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


def test_odometry_tracks_pose_with_zero_noise():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5, odometry_noise=0.0)
    x, y, yaw = robot.update_odometry(gt_x=1.0, gt_y=0.0, gt_yaw=0.1)
    assert (x, y, yaw) == (1.0, 0.0, 0.1)


def test_inertial_finite_difference():
    robot = Robot(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5)
    robot.reset_inertial(yaw=0.0, t=0.0, speed=1.0)
    gyro_z, accel_x, accel_y = robot.update_inertial(yaw=0.1, speed=1.0, t=0.1)
    assert abs(gyro_z - 1.0) < 1e-6
    assert abs(accel_x) < 1e-6
    assert abs(accel_y) < 1e-9
