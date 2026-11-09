# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""automsgs <-> numpy conversions shared by the autosim runtime."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from vntpa.simulator.action import Twist


def depth(message) -> Optional[np.ndarray]:
    """sensor_msgs.Image -> (H, W) metric float32, or None for unsupported encodings."""
    height, width = int(message.height), int(message.width)
    if height <= 0 or width <= 0:
        return None
    data = bytes(message.data)
    if message.encoding == "32FC1":
        values = np.frombuffer(data, dtype=np.float32)
    elif message.encoding in ("16UC1", "mono16"):
        values = np.frombuffer(data, dtype=np.uint16).astype(np.float32) * 0.001
    else:
        return None
    if values.size < height * width:
        return None
    return values[: height * width].reshape(height, width)


def odom(message) -> np.ndarray:
    """nav_msgs.Odometry -> (x, y, yaw)."""
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return np.array([position.x, position.y, yaw], dtype=np.float32)


def make_twist_stamped(message_type, stamp_ns: int, frame_id: str, twist: Twist):
    """Build a geometry_msgs.TwistStamped for one control tick."""
    message = message_type()
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    message.twist.linear.x = float(twist.linear_x)
    message.twist.linear.y = float(twist.linear_y)
    message.twist.angular.z = float(twist.angular_z)
    return message
