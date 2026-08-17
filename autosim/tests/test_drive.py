import math

from autosim.drive import Drive


def test_clamp_and_integrate():
    drive = Drive(max_linear=0.5, max_angular=1.0, watchdog_sec=0.5)
    drive.set_twist(2.0, 3.0, t=0.0)
    x, y, yaw = drive.step(dt=0.1, t=0.1)
    assert abs(x - 0.05) < 1e-9  # 0.5 * 0.1
    assert abs(y) < 1e-9
    assert abs(yaw - 0.1) < 1e-9  # 1.0 * 0.1


def test_watchdog_zeros_velocity():
    drive = Drive(max_linear=0.5, max_angular=1.0, watchdog_sec=0.2)
    drive.set_twist(0.5, 0.0, t=0.0)
    drive.step(dt=0.1, t=0.1)
    x0, _, _ = drive.pose()
    drive.step(dt=0.1, t=0.35)  # timeout
    x1, _, _ = drive.pose()
    assert abs(x1 - x0) < 1e-9


def test_yaw_wrap():
    drive = Drive(max_linear=0.0, max_angular=2.0, watchdog_sec=10.0)
    drive.set_twist(0.0, 2.0, t=0.0)
    drive.step(dt=math.pi, t=0.1)  # delta yaw ~ 2*pi
    _, _, yaw = drive.pose()
    assert abs(yaw) < 1e-6 or abs(abs(yaw) - math.pi) < 1e-6 or abs(yaw) <= math.pi
