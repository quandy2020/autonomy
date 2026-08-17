"""Planar robot motion: diff-drive, wheel odometry, inertial estimate, and ground truth."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


class Robot:
    """Diff-drive kinematics with odometry, inertial finite differences, and ground-truth pose."""

    def __init__(
        self,
        max_linear: float,
        max_angular: float,
        watchdog_sec: float,
        odometry_noise: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        seed: int = 0,
    ) -> None:
        """Construct robot state.

        Args:
            max_linear: Linear speed limit (m/s).
            max_angular: Angular speed limit (rad/s).
            watchdog_sec: Seconds without a command before velocities are zeroed.
            odometry_noise: Gaussian stddev on odometry XY; ``0`` disables noise.
            x: Initial planar x (m).
            y: Initial planar y (m).
            yaw: Initial heading (rad).
            seed: RNG seed for noise.
        """
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.watchdog_sec = float(watchdog_sec)
        self.odometry_noise = float(odometry_noise)
        self.rng = np.random.default_rng(seed)

        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.linear = 0.0
        self.angular = 0.0
        self.last_command_time = 0.0

        self.odometry_x = float(x)
        self.odometry_y = float(y)
        self.odometry_yaw = float(yaw)

        self.inertial_yaw: Optional[float] = None
        self.inertial_speed: Optional[float] = None
        self.inertial_time: Optional[float] = None

    @staticmethod
    def wrap_yaw(yaw: float) -> float:
        """Wrap heading into ``(-π, π]``.

        Args:
            yaw: Raw heading (rad).

        Returns:
            Wrapped heading.
        """
        return math.atan2(math.sin(yaw), math.cos(yaw))

    def set_twist(self, linear_x: float, angular_z: float, t: float) -> None:
        """Accept a velocity command and apply limits.

        Args:
            linear_x: Desired linear speed (m/s).
            angular_z: Desired angular speed (rad/s).
            t: Command timestamp (sim seconds) for the watchdog.
        """
        self.linear = max(-self.max_linear, min(self.max_linear, float(linear_x)))
        self.angular = max(-self.max_angular, min(self.max_angular, float(angular_z)))
        self.last_command_time = float(t)

    def step(self, dt: float, t: float) -> Tuple[float, float, float]:
        """Integrate one planar motion step; zero velocities if the watchdog expires.

        Args:
            dt: Integration step (s).
            t: Step end time (sim seconds) for watchdog checks.

        Returns:
            Updated ``(x, y, yaw)``.
        """
        if t - self.last_command_time > self.watchdog_sec:
            self.linear = 0.0
            self.angular = 0.0
        self.x += self.linear * math.cos(self.yaw) * dt
        self.y += self.linear * math.sin(self.yaw) * dt
        self.yaw = self.wrap_yaw(self.yaw + self.angular * dt)
        return self.pose()

    def pose(self) -> Tuple[float, float, float]:
        """Return the current ground-truth planar pose.

        Returns:
            ``(x, y, yaw)`` in m / m / rad.
        """
        return self.x, self.y, self.yaw

    def velocity(self) -> Tuple[float, float]:
        """Return the current clamped command velocities.

        Returns:
            ``(linear, angular)`` in m/s and rad/s.
        """
        return self.linear, self.angular

    def update_odometry(
        self, gt_x: float, gt_y: float, gt_yaw: float
    ) -> Tuple[float, float, float]:
        """Update wheel odometry from ground truth (optional XY noise).

        Args:
            gt_x: Ground-truth x (m).
            gt_y: Ground-truth y (m).
            gt_yaw: Ground-truth yaw (rad).

        Returns:
            Odometry pose ``(odometry_x, odometry_y, odometry_yaw)``.
        """
        self.odometry_x = float(gt_x)
        self.odometry_y = float(gt_y)
        self.odometry_yaw = float(gt_yaw)
        if self.odometry_noise > 0.0:
            self.odometry_x += float(self.rng.normal(0.0, self.odometry_noise))
            self.odometry_y += float(self.rng.normal(0.0, self.odometry_noise))
        return self.odometry_x, self.odometry_y, self.odometry_yaw

    def reset_inertial(self, yaw: float, t: float, speed: float = 0.0) -> None:
        """Reset inertial estimator history.

        Args:
            yaw: Current heading (rad).
            t: Current simulation time (s).
            speed: Current linear speed (m/s); default ``0``.
        """
        self.inertial_yaw = float(yaw)
        self.inertial_speed = float(speed)
        self.inertial_time = float(t)

    def update_inertial(
        self, yaw: float, speed: float, t: float
    ) -> Tuple[float, float, float]:
        """Estimate yaw rate and body linear acceleration via finite differences.

        On the first call or when ``dt <= 0``, returns zeros and refreshes internal state.

        Args:
            yaw: Current heading (rad).
            speed: Current linear speed (m/s).
            t: Current simulation time (s).

        Returns:
            ``(gyro_z, accel_x, accel_y)`` in rad/s and m/s².
        """
        if self.inertial_time is None or self.inertial_yaw is None or self.inertial_speed is None:
            self.reset_inertial(yaw, t, speed)
            return 0.0, 0.0, 0.0
        dt = t - self.inertial_time
        if dt <= 0.0:
            return 0.0, 0.0, 0.0
        gyro_z = (yaw - self.inertial_yaw) / dt
        accel_x = (speed - self.inertial_speed) / dt
        accel_y = 0.0
        self.inertial_yaw = float(yaw)
        self.inertial_speed = float(speed)
        self.inertial_time = float(t)
        return float(gyro_z), float(accel_x), float(accel_y)

    def ground_truth(self) -> Tuple[float, float, float]:
        """Return the simulated ground-truth pose (same as :meth:`pose`).

        Returns:
            ``(x, y, yaw)``.
        """
        return self.pose()
