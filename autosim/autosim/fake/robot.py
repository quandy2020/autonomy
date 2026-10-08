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

"""Differential-drive plant for the fake (no-Habitat) backend."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


class Robot:
    """Unicycle plant with first-order velocity dynamics and wheel odometry.

    Same planar kinematics as the Habitat ``Robot``, without IMU.
    """

    def __init__(
        self,
        max_linear: float,
        max_angular: float,
        watchdog_sec: float,
        odometry_noise: float = 0.0,
        wheel_separation: float = 0.5,
        max_linear_accel: float = 0.8,
        max_linear_decel: float = 1.2,
        max_angular_accel: float = 2.0,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        seed: int = 0,
    ) -> None:
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.watchdog_sec = float(watchdog_sec)
        self.odometry_noise = float(odometry_noise)
        self.wheel_separation = max(float(wheel_separation), 1e-3)
        self.max_linear_accel = float(max_linear_accel)
        decel = float(max_linear_decel)
        self.max_linear_decel = (
            decel if decel > 0.0 else max(self.max_linear_accel, 0.0)
        )
        self.max_angular_accel = float(max_angular_accel)
        self.rng = np.random.default_rng(seed)

        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
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

    @staticmethod
    def wrap_yaw(yaw: float) -> float:
        return math.atan2(math.sin(yaw), math.cos(yaw))

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(value)))

    @staticmethod
    def slew_rate(
        current: float,
        target: float,
        accel_limit: float,
        decel_limit: float,
        dt: float,
    ) -> float:
        if dt <= 0.0:
            return float(current)
        if accel_limit <= 0.0:
            return float(target)
        decel = decel_limit if decel_limit > 0.0 else accel_limit
        cur = float(current)
        tgt = float(target)
        speeding_up = abs(tgt) > abs(cur) + 1e-12 and (cur * tgt > 0.0 or abs(cur) < 1e-12)
        limit = accel_limit if speeding_up else decel
        max_delta = limit * float(dt)
        delta = tgt - cur
        if abs(delta) <= max_delta:
            return tgt
        return cur + math.copysign(max_delta, delta)

    def set_twist(self, linear_x: float, angular_z: float, t: float) -> None:
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
        v = float(linear)
        w = float(angular)
        h = float(dt)
        if abs(w) < 1e-9:
            x = float(x) + v * math.cos(yaw) * h
            y = float(y) + v * math.sin(yaw) * h
            yaw = Robot.wrap_yaw(float(yaw) + w * h)
            return x, y, yaw
        yaw_new = Robot.wrap_yaw(float(yaw) + w * h)
        radius = v / w
        x = float(x) + radius * (math.sin(yaw_new) - math.sin(yaw))
        y = float(y) - radius * (math.cos(yaw_new) - math.cos(yaw))
        return x, y, yaw_new

    def step(self, dt: float, t: float) -> Tuple[float, float, float]:
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
        v = self.linear if linear is None else float(linear)
        w = self.angular if angular is None else float(angular)
        h = float(dt)
        ds = v * h
        dth = w * h
        if self.odometry_noise > 0.0 and abs(ds) + abs(dth) > 0.0:
            path = abs(ds) + 0.5 * self.wheel_separation * abs(dth)
            sigma = self.odometry_noise * math.sqrt(max(path, 1e-9))
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
        return self.x, self.y, self.yaw

    def teleport(self, x: float, y: float, yaw: float) -> None:
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
        return self.odometry_x, self.odometry_y, self.odometry_yaw

    def velocity(self) -> Tuple[float, float]:
        return self.linear, self.angular

    def update_odometry(
        self, gt_x: float, gt_y: float, gt_yaw: float
    ) -> Tuple[float, float, float]:
        del gt_x, gt_y, gt_yaw
        return self.odometry_pose()

    def ground_truth(self) -> Tuple[float, float, float]:
        return self.pose()
