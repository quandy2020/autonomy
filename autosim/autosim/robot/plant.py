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

"""Planar differential-drive plant: dynamics, kinematics, odometry, IMU."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


class Robot:
    """Unicycle / differential-drive plant with first-order velocity dynamics.

    ``cmd_vel`` sets a *command*; actual ``linear`` / ``angular`` slew under
    acceleration limits, then pose integrates with exact circular-arc
    kinematics (smooth, no instantaneous velocity jumps).
    """

    # Standard gravity (m/s²). Habitat has no physics IMU; we compose REP-145
    # specific force kinematically for a level (pitch=roll=0) ground robot.
    GRAVITY = 9.80665

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
        gravity: float = GRAVITY,
        wheel_separation: float = 0.5,
        max_linear_accel: float = 0.8,
        max_linear_decel: float = 1.2,
        max_angular_accel: float = 2.0,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        seed: int = 0,
    ) -> None:
        """Construct robot state.

        Args:
            max_linear: Linear speed limit (m/s).
            max_angular: Angular speed limit (rad/s).
            watchdog_sec: Seconds without a command before the *command* is
                zeroed; actual speed then coasts down under decel limits.
            odometry_noise: Integrated σ scale on path length (m/√m); ``0`` disables.
            gyro_noise: Gaussian stddev on yaw rate (rad/s).
            accel_noise: Gaussian stddev on body specific-force (m/s²).
            gyro_bias: Stddev used to draw a constant gyro bias at start.
            accel_bias: Stddev used to draw a constant accel bias at start.
            gravity: World gravity magnitude (m/s²); ``0`` disables gravity in
                specific force (kinematic accel only). Default matches Atlas.
            wheel_separation: Track width (m); scales yaw odometry noise.
            max_linear_accel: Forward/back accel limit (m/s²); ``≤0`` = unlimited.
            max_linear_decel: Braking limit (m/s²); ``≤0`` uses ``max_linear_accel``.
            max_angular_accel: Yaw accel/decel limit (rad/s²); ``≤0`` = unlimited.
            x, y, yaw: Initial ground-truth pose.
            seed: RNG seed for noise.
        """
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.watchdog_sec = float(watchdog_sec)
        self.odometry_noise = float(odometry_noise)
        self.gyro_noise = float(gyro_noise)
        self.accel_noise = float(accel_noise)
        self.gravity = max(0.0, float(gravity))
        self.wheel_separation = max(float(wheel_separation), 1e-3)
        self.max_linear_accel = float(max_linear_accel)
        decel = float(max_linear_decel)
        self.max_linear_decel = (
            decel if decel > 0.0 else max(self.max_linear_accel, 0.0)
        )
        self.max_angular_accel = float(max_angular_accel)
        self.rng = np.random.default_rng(seed)
        self.gyro_bias_z = float(self.rng.normal(0.0, float(gyro_bias))) if gyro_bias > 0 else 0.0
        self.accel_bias_x = float(self.rng.normal(0.0, float(accel_bias))) if accel_bias > 0 else 0.0
        self.accel_bias_y = float(self.rng.normal(0.0, float(accel_bias))) if accel_bias > 0 else 0.0
        self.accel_bias_z = float(self.rng.normal(0.0, float(accel_bias))) if accel_bias > 0 else 0.0

        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        # Commanded twist (from /cmd_vel); actual twist slews toward these.
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.body_accel_x = 0.0
        self.body_accel_yaw = 0.0
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

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        """Clamp ``value`` into ``[lo, hi]``."""
        return max(lo, min(hi, float(value)))

    @staticmethod
    def slew_rate(
        current: float,
        target: float,
        accel_limit: float,
        decel_limit: float,
        dt: float,
    ) -> float:
        """First-order rate limit toward ``target``.

        Speeding up (``|target| > |current|`` along the same sign, or crossing
        zero toward a farther magnitude) uses ``accel_limit``; slowing uses
        ``decel_limit``. ``accel_limit ≤ 0`` disables limiting (snap to target).
        """
        if dt <= 0.0:
            return float(current)
        if accel_limit <= 0.0:
            return float(target)
        decel = decel_limit if decel_limit > 0.0 else accel_limit
        cur = float(current)
        tgt = float(target)
        # Slowing down if |tgt| < |cur| while same sign, or braking through zero.
        speeding_up = abs(tgt) > abs(cur) + 1e-12 and (cur * tgt > 0.0 or abs(cur) < 1e-12)
        limit = accel_limit if speeding_up else decel
        max_delta = limit * float(dt)
        delta = tgt - cur
        if abs(delta) <= max_delta:
            return tgt
        return cur + math.copysign(max_delta, delta)

    def set_twist(self, linear_x: float, angular_z: float, t: float) -> None:
        """Accept a velocity *command* (clamped); actual speed updates in ``step``."""
        self.cmd_linear = self.clamp(float(linear_x), -self.max_linear, self.max_linear)
        self.cmd_angular = self.clamp(float(angular_z), -self.max_angular, self.max_angular)
        self.last_command_time = float(t)

    @staticmethod
    def integrate_unicycle(
        x: float,
        y: float,
        yaw: float,
        linear: float,
        angular: float,
        dt: float,
    ) -> Tuple[float, float, float]:
        """Exact planar unicycle step (circular arc when ``|ω|`` is significant).

        Args:
            x, y, yaw: Pose before the step.
            linear, angular: Body twist used over ``dt`` (constant).
            dt: Step (s).

        Returns:
            Updated ``(x, y, yaw)``.
        """
        v = float(linear)
        w = float(angular)
        h = float(dt)
        if abs(w) < 1e-9:
            x = float(x) + v * math.cos(yaw) * h
            y = float(y) + v * math.sin(yaw) * h
            yaw = Robot.wrap_yaw(float(yaw) + w * h)
            return x, y, yaw
        # ICR / circular-arc closed form.
        yaw_new = Robot.wrap_yaw(float(yaw) + w * h)
        radius = v / w
        x = float(x) + radius * (math.sin(yaw_new) - math.sin(yaw))
        y = float(y) - radius * (math.cos(yaw_new) - math.cos(yaw))
        return x, y, yaw_new

    def step(self, dt: float, t: float) -> Tuple[float, float, float]:
        """Apply dynamics then integrate kinematics + wheel odometry.

        Args:
            dt: Control period (s).
            t: Simulation time at end of period (s).

        Returns:
            Updated ground-truth ``(x, y, yaw)``.
        """
        h = max(0.0, float(dt))
        if float(t) - self.last_command_time > self.watchdog_sec:
            self.cmd_linear = 0.0
            self.cmd_angular = 0.0

        v0 = self.linear
        w0 = self.angular
        self.linear = self.slew_rate(
            v0, self.cmd_linear, self.max_linear_accel, self.max_linear_decel, h
        )
        self.angular = self.slew_rate(
            w0, self.cmd_angular, self.max_angular_accel, self.max_angular_accel, h
        )
        if h > 1e-12:
            self.body_accel_x = (self.linear - v0) / h
            self.body_accel_yaw = (self.angular - w0) / h
        else:
            self.body_accel_x = 0.0
            self.body_accel_yaw = 0.0

        # Unlimited accel → velocity is already at command for the whole step.
        # Otherwise trapezoidal average for smooth ramp integration.
        if self.max_linear_accel <= 0.0 and self.max_angular_accel <= 0.0:
            v_avg = self.linear
            w_avg = self.angular
        else:
            v_avg = 0.5 * (v0 + self.linear)
            w_avg = 0.5 * (w0 + self.angular)
        self.x, self.y, self.yaw = self.integrate_unicycle(
            self.x, self.y, self.yaw, v_avg, w_avg, h
        )
        self.integrate_odometry(h, linear=v_avg, angular=w_avg)
        return self.pose()

    def integrate_odometry(
        self,
        dt: float,
        linear: Optional[float] = None,
        angular: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """Integrate body twist into odometry with path-length noise.

        Args:
            dt: Step (s).
            linear: Body linear speed; defaults to current ``self.linear``.
            angular: Body yaw rate; defaults to current ``self.angular``.

        Returns:
            Updated ``(odometry_x, odometry_y, odometry_yaw)``.
        """
        v = self.linear if linear is None else float(linear)
        w = self.angular if angular is None else float(angular)
        h = float(dt)
        ds = v * h
        dth = w * h
        if self.odometry_noise > 0.0 and abs(ds) + abs(dth) > 0.0:
            path = abs(ds) + 0.5 * self.wheel_separation * abs(dth)
            sigma = self.odometry_noise * math.sqrt(max(path, 1e-9))
            # Pure rotation must not inject fake translation noise into Δs —
            # that makes Cartographer map→odom jump while spinning in place.
            if abs(ds) > 1e-9:
                ds += float(self.rng.normal(0.0, sigma))
            if abs(dth) > 1e-9:
                dth += float(self.rng.normal(0.0, sigma / self.wheel_separation))
        if abs(h) > 1e-12:
            self.odometry_x, self.odometry_y, self.odometry_yaw = Robot.integrate_unicycle(
                self.odometry_x,
                self.odometry_y,
                self.odometry_yaw,
                ds / h,
                dth / h,
                h,
            )
        return self.odometry_x, self.odometry_y, self.odometry_yaw

    def pose(self) -> Tuple[float, float, float]:
        """Return ground-truth planar pose."""
        return self.x, self.y, self.yaw

    def teleport(self, x: float, y: float, yaw: float) -> None:
        """Set ground-truth and odometry to the same planar pose; zero twist."""
        self.x = self.odometry_x = float(x)
        self.y = self.odometry_y = float(y)
        heading = self.wrap_yaw(float(yaw))
        self.yaw = self.odometry_yaw = heading
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.body_accel_x = 0.0
        self.body_accel_yaw = 0.0

    @staticmethod
    def map_to_odom(
        ground_truth: Tuple[float, float, float],
        odometry: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """``map→odom`` so ``map→base`` equals ground truth.

        ``odom→base`` stays the integrated wheel pose; this parent transform
        absorbs odometry drift so Autoviz heading matches the Habitat camera.
        """
        gx, gy, gyaw = ground_truth
        ox, oy, oyaw = odometry
        dyaw = Robot.wrap_yaw(float(gyaw) - float(oyaw))
        cosine = math.cos(dyaw)
        sine = math.sin(dyaw)
        return (
            float(gx) - (cosine * float(ox) - sine * float(oy)),
            float(gy) - (sine * float(ox) + cosine * float(oy)),
            dyaw,
        )

    def odometry_pose(self) -> Tuple[float, float, float]:
        """Return integrated wheel-odometry pose."""
        return self.odometry_x, self.odometry_y, self.odometry_yaw

    def velocity(self) -> Tuple[float, float]:
        """Return actual (dynamics-filtered) body velocities."""
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

    def specific_force(
        self, accel_x: float, accel_y: float, accel_z: float = 0.0
    ) -> Tuple[float, float, float]:
        """Compose REP-145 specific force for a level robot (pitch=roll=0).

        Habitat has no physics engine IMU. Kinematic body accel ``a`` plus
        world gravity ``g_w=(0,0,-G)`` become ``f = a - R^T g_w``. With yaw-only
        attitude, ``R^T g_w = (0,0,-G)``, so ``f_z = a_z + G``.

        Args:
            accel_x, accel_y, accel_z: Body-frame kinematic acceleration (m/s²).

        Returns:
            Specific force ``(fx, fy, fz)`` in the IMU / body frame (m/s²).
        """
        return (
            float(accel_x),
            float(accel_y),
            float(accel_z) + self.gravity,
        )

    def update_inertial(
        self, yaw: float, speed: float, t: float
    ) -> Tuple[float, float, float, float]:
        """Estimate yaw rate and body specific force; add bias and white noise.

        Prefer dynamics ``body_accel_x`` when the call coincides with the plant
        step (smooth); otherwise finite-difference speed.

        Returns:
            ``(ω_z, f_x, f_y, f_z)`` — gyro (rad/s) and REP-145 specific force.
        """
        if self.inertial_time is None or self.inertial_yaw is None or self.inertial_speed is None:
            self.reset_inertial(yaw, t, speed)
            fx, fy, fz = self.specific_force(
                self.accel_bias_x, self.accel_bias_y, self.accel_bias_z
            )
            return 0.0, fx, fy, fz
        dt = t - self.inertial_time
        if dt <= 0.0:
            fx, fy, fz = self.specific_force(
                self.accel_bias_x, self.accel_bias_y, self.accel_bias_z
            )
            return 0.0, fx, fy, fz
        dyaw = self.wrap_yaw(float(yaw) - float(self.inertial_yaw))
        gyro_z = dyaw / dt + self.gyro_bias_z
        # Use plant accel when available (same-step); else FD on speed.
        if abs(float(speed) - self.linear) < 1e-6 and abs(self.body_accel_x) > 0.0:
            accel_x = self.body_accel_x + self.accel_bias_x
        else:
            accel_x = (float(speed) - float(self.inertial_speed)) / dt + self.accel_bias_x
        accel_y = 0.0 + self.accel_bias_y
        accel_z = 0.0 + self.accel_bias_z
        if self.gyro_noise > 0.0:
            gyro_z += float(self.rng.normal(0.0, self.gyro_noise))
        if self.accel_noise > 0.0:
            accel_x += float(self.rng.normal(0.0, self.accel_noise))
            accel_y += float(self.rng.normal(0.0, self.accel_noise))
            accel_z += float(self.rng.normal(0.0, self.accel_noise))
        self.inertial_yaw = float(yaw)
        self.inertial_speed = float(speed)
        self.inertial_time = float(t)
        fx, fy, fz = self.specific_force(accel_x, accel_y, accel_z)
        return float(gyro_z), fx, fy, fz

    def ground_truth(self) -> Tuple[float, float, float]:
        """Return ground-truth pose."""
        return self.pose()
