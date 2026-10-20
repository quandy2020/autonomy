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

from typing import Sequence, Tuple

import numpy as np

from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2
from automsgs.msgs.sensor_msgs.point_field_pb2 import PointField


class Messages:
    """Stateless helpers: NumPy arrays ↔ sensor_msgs protobufs."""

    @staticmethod
    def set_header(header: object, stamp: Tuple[int, int], frame_id: str) -> None:
        header.stamp.sec = int(stamp[0])
        header.stamp.nanosec = int(stamp[1])
        header.frame_id = frame_id

    @staticmethod
    def stamp_from_seconds(t: float) -> Tuple[int, int]:
        sec = int(t)
        nanosec = int(round((float(t) - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        if nanosec < 0:
            sec -= 1
            nanosec += 1_000_000_000
        return sec, nanosec

    @staticmethod
    def diagonal_covariance(values: Sequence[float], size: int = 9) -> list:
        dim = int(round(float(size) ** 0.5))
        cov = [0.0] * int(size)
        for index, value in enumerate(values):
            if index >= dim:
                break
            cov[index * dim + index] = float(value)
        return cov

    @classmethod
    def encode_inertial(
        cls,
        stamp: Tuple[int, int],
        frame_id: str,
        orientation: Tuple[float, float, float, float],
        angular_velocity: Tuple[float, float, float],
        linear_acceleration: Tuple[float, float, float],
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
        stamp: Tuple[int, int],
        frame_id: str,
        *,
        intensity: np.ndarray | None = None,
    ) -> PointCloud2:
        array = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        count = int(array.shape[0])
        has_intensity = intensity is not None and count > 0
        dtype_fields: list[tuple[str, str]] = [("x", "f4"), ("y", "f4"), ("z", "f4")]
        if has_intensity:
            dtype_fields.append(("intensity", "f4"))
        structured = np.zeros(count, dtype=np.dtype(dtype_fields))
        if count > 0:
            structured["x"] = array[:, 0]
            structured["y"] = array[:, 1]
            structured["z"] = array[:, 2]
            if has_intensity:
                structured["intensity"] = np.asarray(
                    intensity, dtype=np.float32
                ).reshape(-1)
        message = PointCloud2()
        cls.set_header(message.header, stamp, frame_id)
        message.height = 1
        message.width = count
        message.is_bigendian = False
        message.point_step = int(structured.dtype.itemsize) if count > 0 else 12
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
        stamp: Tuple[int, int],
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
