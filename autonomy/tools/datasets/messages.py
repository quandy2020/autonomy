# Copyright 2026 The Openbot Authors (duyongquan)
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

"""Minimal automsgs encoders for dataset publish (no autosim dependency)."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.nav_msgs.path_pb2 import Path
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2
from automsgs.msgs.sensor_msgs.point_field_pb2 import PointField


class Messages:
    """Stateless helpers: NumPy arrays ↔ sensor_msgs protobufs."""

    @staticmethod
    def set_header(header: object, stamp: tuple[int, int], frame_id: str) -> None:
        header.stamp.sec = int(stamp[0])
        header.stamp.nanosec = int(stamp[1])
        header.frame_id = frame_id

    @staticmethod
    def stamp_from_nanoseconds(timestamp_ns: int) -> tuple[int, int]:
        """Split an integer nanosecond timestamp without losing precision."""
        sec, nanosec = divmod(int(timestamp_ns), 1_000_000_000)
        return sec, nanosec

    @staticmethod
    def stamp_from_seconds(t: float) -> tuple[int, int]:
        sec = int(t)
        nanosec = round((float(t) - sec) * 1e9)
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        if nanosec < 0:
            sec -= 1
            nanosec += 1_000_000_000
        return sec, nanosec

    @staticmethod
    def diagonal_covariance(values: Sequence[float], size: int = 9) -> list:
        dim = round(float(size) ** 0.5)
        cov = [0.0] * int(size)
        for index, value in enumerate(values):
            if index >= dim:
                break
            cov[index * dim + index] = float(value)
        return cov

    @classmethod
    def encode_inertial(
        cls,
        stamp: tuple[int, int],
        frame_id: str,
        orientation: tuple[float, float, float, float],
        angular_velocity: tuple[float, float, float],
        linear_acceleration: tuple[float, float, float],
        gyro_variance: float = 0.0,
        accel_variance: float = 0.0,
    ) -> Imu:
        message = Imu()
        cls.set_header(message.header, stamp, frame_id)
        message.orientation.x = orientation[0]
        message.orientation.y = orientation[1]
        message.orientation.z = orientation[2]
        message.orientation.w = orientation[3]
        message.angular_velocity.x = angular_velocity[0]
        message.angular_velocity.y = angular_velocity[1]
        message.angular_velocity.z = angular_velocity[2]
        message.linear_acceleration.x = linear_acceleration[0]
        message.linear_acceleration.y = linear_acceleration[1]
        message.linear_acceleration.z = linear_acceleration[2]
        message.orientation_covariance.extend([-1.0] + [0.0] * 8)
        gv = float(gyro_variance)
        av = float(accel_variance)
        message.angular_velocity_covariance.extend(
            cls.diagonal_covariance([gv, gv, gv], 9)
        )
        message.linear_acceleration_covariance.extend(
            cls.diagonal_covariance([av, av, av], 9)
        )
        return message

    @classmethod
    def encode_point_cloud2(
        cls,
        points: np.ndarray,
        stamp: tuple[int, int],
        frame_id: str,
        *,
        intensity: np.ndarray | None = None,
        time_ms: np.ndarray | None = None,
        scan_period_ms: float = 100.0,
    ) -> PointCloud2:
        array = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        count = int(array.shape[0])
        has_intensity = intensity is not None and count > 0
        # `time` is relative ms within the sweep (Argus stores it in curvature).
        dtype_fields: list[tuple[str, str]] = [
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
            ("intensity", "f4"),
            ("time", "f4"),
        ]
        structured = np.zeros(count, dtype=np.dtype(dtype_fields))
        if count > 0:
            structured["x"] = array[:, 0]
            structured["y"] = array[:, 1]
            structured["z"] = array[:, 2]
            if has_intensity:
                structured["intensity"] = np.asarray(
                    intensity, dtype=np.float32
                ).reshape(-1)
            if time_ms is not None and len(time_ms) == count:
                structured["time"] = np.asarray(time_ms, dtype=np.float32)
            elif count == 1:
                structured["time"] = 0.0
            else:
                structured["time"] = np.linspace(
                    0.0, float(scan_period_ms), count, dtype=np.float32
                )
        message = PointCloud2()
        cls.set_header(message.header, stamp, frame_id)
        message.height = 1
        message.width = count
        message.is_bigendian = False
        message.point_step = int(structured.dtype.itemsize) if count > 0 else 20
        message.row_step = int(message.point_step * message.width)
        message.is_dense = False
        offset = 0
        for name, _dtype in dtype_fields:
            field = message.fields.add()
            field.name = name
            field.offset = offset
            field.datatype = PointField.FLOAT32
            field.count = 1
            offset += 4
        message.data = structured.tobytes() if count > 0 else b""
        return message

    @classmethod
    def encode_image(
        cls,
        image: np.ndarray,
        stamp: tuple[int, int],
        frame_id: str,
        encoding: str = "rgb8",
    ) -> Image:
        message = Image()
        cls.set_header(message.header, stamp, frame_id)
        array = np.asarray(image)
        if (
            array.ndim == 3
            and array.shape[2] == 4
            and encoding in ("rgb8", "bgr8", "8UC3")
        ):
            array = np.ascontiguousarray(array[:, :, :3])
        if array.ndim == 3 and array.shape[2] == 1:
            array = np.ascontiguousarray(array[:, :, 0])
        if array.ndim == 2:
            height, width = array.shape
            channels = 1
        else:
            height, width, channels = array.shape
        message.height = int(height)
        message.width = int(width)
        message.encoding = encoding
        message.is_bigendian = False
        message.step = int(width * channels * array.dtype.itemsize)
        message.data = np.ascontiguousarray(array).tobytes()
        return message

    @classmethod
    def encode_odometry(
        cls,
        stamp: tuple[int, int],
        frame_id: str,
        child_frame_id: str,
        xyz: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
    ) -> Odometry:
        # Odometry.pose is PoseWithCovariance{ PoseStamped pose; covariance }
        # → pose.pose.pose.{position,orientation}
        message = Odometry()
        cls.set_header(message.header, stamp, frame_id)
        message.child_frame_id = child_frame_id
        stamped = message.pose.pose
        cls.set_header(stamped.header, stamp, frame_id)
        stamped.pose.position.x = float(xyz[0])
        stamped.pose.position.y = float(xyz[1])
        stamped.pose.position.z = float(xyz[2])
        stamped.pose.orientation.x = float(quat_xyzw[0])
        stamped.pose.orientation.y = float(quat_xyzw[1])
        stamped.pose.orientation.z = float(quat_xyzw[2])
        stamped.pose.orientation.w = float(quat_xyzw[3])
        message.pose.covariance.extend([0.0] * 36)
        message.twist.covariance.extend([0.0] * 36)
        return message

    @classmethod
    def encode_pose_stamped(
        cls,
        stamp: tuple[int, int],
        frame_id: str,
        xyz: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
    ) -> PoseStamped:
        message = PoseStamped()
        cls.set_header(message.header, stamp, frame_id)
        message.pose.position.x = float(xyz[0])
        message.pose.position.y = float(xyz[1])
        message.pose.position.z = float(xyz[2])
        message.pose.orientation.x = float(quat_xyzw[0])
        message.pose.orientation.y = float(quat_xyzw[1])
        message.pose.orientation.z = float(quat_xyzw[2])
        message.pose.orientation.w = float(quat_xyzw[3])
        return message

    @classmethod
    def encode_path(
        cls,
        stamp: tuple[int, int],
        frame_id: str,
        poses: Sequence[tuple[tuple[float, float, float], tuple[float, float, float, float]]],
        *,
        stride: int = 1,
    ) -> Path:
        message = Path()
        cls.set_header(message.header, stamp, frame_id)
        step = max(1, int(stride))
        for xyz, quat in list(poses)[::step]:
            ps = message.poses.add()
            cls.set_header(ps.header, stamp, frame_id)
            ps.pose.position.x = float(xyz[0])
            ps.pose.position.y = float(xyz[1])
            ps.pose.position.z = float(xyz[2])
            ps.pose.orientation.x = float(quat[0])
            ps.pose.orientation.y = float(quat[1])
            ps.pose.orientation.z = float(quat[2])
            ps.pose.orientation.w = float(quat[3])
        return message
