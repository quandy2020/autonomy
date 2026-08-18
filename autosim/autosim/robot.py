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

"""Planar differential-drive plant: twist limits, odometry, IMU, ground truth."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


class Robot:
    """Diff-drive kinematics with integrated odometry noise and biased IMU."""

    def __init__(
        self,
        max_linear: float,
        max_angular: float,
        watchdog_sec: float,
        odometry_noise: float = 0.0,
        gyro_noise: float = 0.0,
        accel_noise: float = 0.0,
        gyro_bias: float = 0.0,
        accel_bias: float = 0.0,
        wheel_separation: float = 0.5,
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
            odometry_noise: Integrated σ scale on path length (m/√m); ``0`` disables.
            gyro_noise: Gaussian stddev on yaw rate (rad/s).
            accel_noise: Gaussian stddev on body accel x/y (m/s²).
            gyro_bias: Stddev used to draw a constant gyro bias at start.
            accel_bias: Stddev used to draw a constant accel bias at start.
            wheel_separation: Track width (m); scales yaw odometry noise.
            x, y, yaw: Initial ground-truth pose.
            seed: RNG seed for noise.
        """
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.watchdog_sec = float(watchdog_sec)
        self.odometry_noise = float(odometry_noise)
        self.gyro_noise = float(gyro_noise)
        self.accel_noise = float(accel_noise)
        self.wheel_separation = max(float(wheel_separation), 1e-3)
        self.rng = np.random.default_rng(seed)
        self.gyro_bias_z = float(self.rng.normal(0.0, float(gyro_bias))) if gyro_bias > 0 else 0.0
        self.accel_bias_x = float(self.rng.normal(0.0, float(accel_bias))) if accel_bias > 0 else 0.0
        self.accel_bias_y = float(self.rng.normal(0.0, float(accel_bias))) if accel_bias > 0 else 0.0

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
        """Wrap heading into ``(-π, π]``."""
        return math.atan2(math.sin(yaw), math.cos(yaw))

    def set_twist(self, linear_x: float, angular_z: float, t: float) -> None:
        """Accept a velocity command and apply limits."""
        self.linear = max(-self.max_linear, min(self.max_linear, float(linear_x)))
        self.angular = max(-self.max_angular, min(self.max_angular, float(angular_z)))
        self.last_command_time = float(t)

    def step(self, dt: float, t: float) -> Tuple[float, float, float]:
        """Integrate ground-truth pose; also advance noisy wheel odometry."""
        if t - self.last_command_time > self.watchdog_sec:
            self.linear = 0.0
            self.angular = 0.0
        self.x += self.linear * math.cos(self.yaw) * dt
        self.y += self.linear * math.sin(self.yaw) * dt
        self.yaw = self.wrap_yaw(self.yaw + self.angular * dt)
        self.integrate_odometry(dt)
        return self.pose()

    def integrate_odometry(self, dt: float) -> Tuple[float, float, float]:
        """Integrate body twist into odometry with path-length noise.

        Args:
            dt: Step (s).

        Returns:
            Updated ``(odometry_x, odometry_y, odometry_yaw)``.
        """
        ds = self.linear * float(dt)
        dth = self.angular * float(dt)
        if self.odometry_noise > 0.0 and abs(ds) + abs(dth) > 0.0:
            path = abs(ds) + 0.5 * self.wheel_separation * abs(dth)
            sigma = self.odometry_noise * math.sqrt(max(path, 1e-9))
            ds += float(self.rng.normal(0.0, sigma))
            dth += float(self.rng.normal(0.0, sigma / self.wheel_separation))
        self.odometry_x += ds * math.cos(self.odometry_yaw)
        self.odometry_y += ds * math.sin(self.odometry_yaw)
        self.odometry_yaw = self.wrap_yaw(self.odometry_yaw + dth)
        return self.odometry_x, self.odometry_y, self.odometry_yaw

    def pose(self) -> Tuple[float, float, float]:
        """Return ground-truth planar pose."""
        return self.x, self.y, self.yaw

    def odometry_pose(self) -> Tuple[float, float, float]:
        """Return integrated wheel-odometry pose."""
        return self.odometry_x, self.odometry_y, self.odometry_yaw

    def velocity(self) -> Tuple[float, float]:
        """Return clamped command velocities."""
        return self.linear, self.angular

    def update_odometry(
        self, gt_x: float, gt_y: float, gt_yaw: float
    ) -> Tuple[float, float, float]:
        """Return current integrated odometry (``gt_*`` kept for API compatibility)."""
        del gt_x, gt_y, gt_yaw
        return self.odometry_pose()

    def reset_inertial(self, yaw: float, t: float, speed: float = 0.0) -> None:
        """Reset inertial estimator history."""
        self.inertial_yaw = float(yaw)
        self.inertial_speed = float(speed)
        self.inertial_time = float(t)

    def update_inertial(
        self, yaw: float, speed: float, t: float
    ) -> Tuple[float, float, float]:
        """Estimate yaw rate / body accel; add bias and white noise."""
        if self.inertial_time is None or self.inertial_yaw is None or self.inertial_speed is None:
            self.reset_inertial(yaw, t, speed)
            return 0.0, 0.0, 0.0
        dt = t - self.inertial_time
        if dt <= 0.0:
            return 0.0, 0.0, 0.0
        gyro_z = (yaw - self.inertial_yaw) / dt + self.gyro_bias_z
        accel_x = (speed - self.inertial_speed) / dt + self.accel_bias_x
        accel_y = 0.0 + self.accel_bias_y
        if self.gyro_noise > 0.0:
            gyro_z += float(self.rng.normal(0.0, self.gyro_noise))
        if self.accel_noise > 0.0:
            accel_x += float(self.rng.normal(0.0, self.accel_noise))
            accel_y += float(self.rng.normal(0.0, self.accel_noise))
        self.inertial_yaw = float(yaw)
        self.inertial_speed = float(speed)
        self.inertial_time = float(t)
        return float(gyro_z), float(accel_x), float(accel_y)

    def ground_truth(self) -> Tuple[float, float, float]:
        """Return ground-truth pose."""
        return self.pose()
