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

"""Encode and decode automsgs protobufs for the sensor–actuator bridge."""

from __future__ import annotations

import math
from typing import Sequence, Tuple, Union

import numpy as np

from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.builtin_interfaces.time_pb2 import Time
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2
from automsgs.msgs.sensor_msgs.point_field_pb2 import PointField
from automsgs.msgs.tf2_msgs.tf_message_pb2 import TFMessage


class Messages:
    """Stateless helpers: NumPy / geometry quantities ↔ automsgs protobufs."""

    @staticmethod
    def set_header(header: object, stamp: Tuple[int, int], frame_id: str) -> None:
        """Fill standard ``Header`` fields.

        Args:
            header: Protobuf header with ``stamp`` and ``frame_id``.
            stamp: ``(sec, nanosec)``.
            frame_id: Frame name.
        """
        header.stamp.sec = int(stamp[0])
        header.stamp.nanosec = int(stamp[1])
        header.frame_id = frame_id

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
        """Convert planar heading to a quaternion (rotation about z).

        Args:
            yaw: Heading (rad).

        Returns:
            ``(x, y, z, w)``.
        """
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))

    @staticmethod
    def diagonal_covariance(values: Sequence[float], size: int = 36) -> list:
        """Build a row-major covariance with diagonal ``values`` (rest zero)."""
        dim = int(round(math.sqrt(int(size))))
        cov = [0.0] * int(size)
        for index, value in enumerate(values):
            if index >= dim:
                break
            cov[index * dim + index] = float(value)
        return cov

    @staticmethod
    def camera_intrinsics(width: int, height: int, hfov_deg: float) -> list:
        """Pinhole ``K`` from horizontal FOV (degrees)."""
        hfov = math.radians(float(hfov_deg))
        fx = 0.5 * float(width) / max(math.tan(0.5 * hfov), 1e-6)
        fy = fx
        cx = 0.5 * float(width)
        cy = 0.5 * float(height)
        return [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

    @classmethod
    def encode_twist_stamped(
        cls,
        linear_x: float,
        angular_z: float,
        stamp: Tuple[int, int],
        frame_id: str = "base_link",
    ) -> TwistStamped:
        """Build a planar ``geometry_msgs.TwistStamped`` for ``cmd_vel``.

        Args:
            linear_x: Forward speed (m/s).
            angular_z: Yaw rate (rad/s).
            stamp: Timestamp ``(sec, nanosec)``.
            frame_id: Twist frame (usually ``base_link``).

        Returns:
            Populated :class:`TwistStamped`.
        """
        message = TwistStamped()
        cls.set_header(message.header, stamp, frame_id)
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        return message

    @classmethod
    def encode_laser_scan(
        cls,
        ranges: np.ndarray,
        stamp: Tuple[int, int],
        frame_id: str,
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        scan_time: float,
        intensities: np.ndarray | None = None,
    ) -> LaserScan:
        """Build a ``sensor_msgs.LaserScan``.

        Args:
            ranges: Range array (m).
            stamp: Timestamp ``(sec, nanosec)``.
            frame_id: Laser frame.
            angle_min: Start angle (rad).
            angle_max: End angle (rad).
            angle_increment: Angular resolution (rad).
            range_min: Minimum valid range (m).
            range_max: Maximum valid range (m).
            scan_time: Full-scan duration (s).
            intensities: Optional intensity array.

        Returns:
            Populated :class:`LaserScan`.
        """
        message = LaserScan()
        cls.set_header(message.header, stamp, frame_id)
        message.angle_min = float(angle_min)
        message.angle_max = float(angle_max)
        message.angle_increment = float(angle_increment)
        message.time_increment = 0.0
        message.scan_time = float(scan_time)
        message.range_min = float(range_min)
        message.range_max = float(range_max)
        message.ranges.extend(float(value) for value in ranges.reshape(-1))
        if intensities is not None:
            message.intensities.extend(float(value) for value in intensities.reshape(-1))
        return message

    @staticmethod
    def pack_rgb_uint32(colors: np.ndarray) -> np.ndarray:
        """Pack ``Nx3`` uint8 RGB rows into packed ``uint32`` (rviz / PCL convention).

        Args:
            colors: ``Nx3`` RGB in ``[0, 255]``.

        Returns:
            ``(N,)`` uint32 array.
        """
        array = np.asarray(colors, dtype=np.uint8).reshape(-1, 3)
        r = array[:, 0].astype(np.uint32)
        g = array[:, 1].astype(np.uint32)
        b = array[:, 2].astype(np.uint32)
        return (r << 16) | (g << 8) | b

    @classmethod
    def encode_point_cloud2(
        cls,
        points: np.ndarray,
        stamp: Tuple[int, int],
        frame_id: str,
        *,
        intensity: np.ndarray | None = None,
        rgb: np.ndarray | None = None,
    ) -> PointCloud2:
        """Build a ``sensor_msgs.PointCloud2`` with optional intensity / rgb fields.

        Args:
            points: ``Nx3`` float array in the sensor frame; ``N`` may be 0.
            stamp: Timestamp ``(sec, nanosec)``.
            frame_id: Point-cloud frame.
            intensity: Optional per-point float32 intensity (length ``N``).
            rgb: Optional ``Nx3`` uint8 RGB or ``(N,)`` packed uint32.

        Returns:
            Populated :class:`PointCloud2` with ``is_dense=False``.
        """
        array = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        count = int(array.shape[0])
        has_intensity = intensity is not None and count > 0
        has_rgb = rgb is not None and count > 0

        dtype_fields: list[tuple[str, str]] = [
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
        ]
        if has_intensity:
            dtype_fields.append(("intensity", "f4"))
        if has_rgb:
            dtype_fields.append(("rgb", "u4"))

        structured = np.zeros(count, dtype=np.dtype(dtype_fields))
        if count > 0:
            structured["x"] = array[:, 0]
            structured["y"] = array[:, 1]
            structured["z"] = array[:, 2]
            if has_intensity:
                structured["intensity"] = np.asarray(intensity, dtype=np.float32).reshape(-1)
            if has_rgb:
                rgb_array = np.asarray(rgb)
                if rgb_array.ndim == 2 and rgb_array.shape[1] == 3:
                    structured["rgb"] = cls.pack_rgb_uint32(rgb_array)
                else:
                    structured["rgb"] = rgb_array.reshape(-1).astype(np.uint32)

        message = PointCloud2()
        cls.set_header(message.header, stamp, frame_id)
        message.height = 1
        message.width = count
        message.is_bigendian = False
        message.point_step = int(structured.dtype.itemsize) if count > 0 else 12
        message.row_step = int(message.point_step * message.width)
        message.is_dense = False

        offset = 0
        for name, _ in dtype_fields:
            field = message.fields.add()
            field.name = name
            field.offset = offset
            field.datatype = (
                PointField.UINT32 if name == "rgb" else PointField.FLOAT32
            )
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
        encoding: str,
    ) -> Image:
        """Build a ``sensor_msgs.Image``.

        Args:
            image: ``HxW`` or ``HxWxC`` array.
            stamp: Timestamp.
            frame_id: Camera frame.
            encoding: e.g. ``rgb8``, ``32FC1``.

        Returns:
            Populated :class:`Image`.
        """
        message = Image()
        cls.set_header(message.header, stamp, frame_id)
        if image.ndim == 2:
            height, width = image.shape
            channels = 1
        else:
            height, width, channels = image.shape
        message.height = int(height)
        message.width = int(width)
        message.encoding = encoding
        message.is_bigendian = False
        message.step = int(width * channels * image.dtype.itemsize)
        message.data = np.ascontiguousarray(image).tobytes()
        return message

    @classmethod
    def encode_camera_info(
        cls,
        stamp: Tuple[int, int],
        frame_id: str,
        width: int,
        height: int,
        camera_matrix: Sequence[float],
        projection: Sequence[float] | None = None,
    ) -> CameraInfo:
        """Build a ``sensor_msgs.CameraInfo``.

        Args:
            stamp: Timestamp.
            frame_id: Camera frame.
            width: Image width in pixels.
            height: Image height in pixels.
            camera_matrix: Row-major intrinsic ``K`` of length 9.
            projection: Optional projection ``P`` of length 12; derived from ``K`` if omitted.

        Returns:
            Populated :class:`CameraInfo`.

        Raises:
            ValueError: If ``camera_matrix`` length is not 9.
        """
        message = CameraInfo()
        cls.set_header(message.header, stamp, frame_id)
        message.width = int(width)
        message.height = int(height)
        message.distortion_model = "plumb_bob"
        message.d.extend([0.0, 0.0, 0.0, 0.0, 0.0])
        matrix = [float(value) for value in camera_matrix]
        if len(matrix) != 9:
            raise ValueError("camera matrix K must have length 9")
        message.k.extend(matrix)
        message.r.extend([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        if projection is None:
            fx, fy, cx, cy = matrix[0], matrix[4], matrix[2], matrix[5]
            projection = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        message.p.extend(float(value) for value in projection)
        return message

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
        """Build a ``sensor_msgs.Imu``.

        Args:
            stamp: Timestamp.
            frame_id: IMU frame.
            orientation: Quaternion ``(x, y, z, w)``.
            angular_velocity: Angular velocity ``(wx, wy, wz)`` (rad/s).
            linear_acceleration: Linear acceleration ``(ax, ay, az)`` (m/s²).
            gyro_variance: Diagonal angular-velocity variance (rad²/s²).
            accel_variance: Diagonal linear-acceleration variance ((m/s²)²).

        Returns:
            Populated :class:`Imu`.
        """
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
        message.angular_velocity_covariance.extend(cls.diagonal_covariance([gv, gv, gv], 9))
        message.linear_acceleration_covariance.extend(cls.diagonal_covariance([av, av, av], 9))
        return message

    @classmethod
    def encode_odometry(
        cls,
        x: float,
        y: float,
        yaw: float,
        linear: float,
        angular: float,
        stamp: Tuple[int, int],
        frame_id: str,
        child_frame_id: str,
        pose_variance: float = 0.0,
        twist_variance: float = 0.0,
    ) -> Odometry:
        """Build a planar ``nav_msgs.Odometry``.

        Args:
            x: Odometry x (m).
            y: Odometry y (m).
            yaw: Heading (rad).
            linear: Linear speed (m/s).
            angular: Angular speed (rad/s).
            stamp: Timestamp.
            frame_id: Parent frame (usually ``odom``).
            child_frame_id: Child frame (usually ``base_link``).
            pose_variance: Diagonal pose variance for x/y/yaw.
            twist_variance: Diagonal twist variance for vx/wz.

        Returns:
            Populated :class:`Odometry`.
        """
        message = Odometry()
        cls.set_header(message.header, stamp, frame_id)
        message.child_frame_id = child_frame_id
        qx, qy, qz, qw = cls.yaw_to_quaternion(yaw)
        pose = message.pose.pose
        cls.set_header(pose.header, stamp, frame_id)
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        pv = float(pose_variance)
        tv = float(twist_variance)
        message.pose.covariance.extend(
            cls.diagonal_covariance([pv, pv, 0.0, 0.0, 0.0, pv], 36)
        )
        message.twist.twist.linear.x = float(linear)
        message.twist.twist.angular.z = float(angular)
        message.twist.covariance.extend(
            cls.diagonal_covariance([tv, 0.0, 0.0, 0.0, 0.0, tv], 36)
        )
        return message

    @classmethod
    def encode_transform(
        cls,
        stamp: Tuple[int, int],
        parent: str,
        child: str,
        xyz: Tuple[float, float, float],
        yaw: float = 0.0,
    ):
        """Build one ``TransformStamped`` (used inside ``TFMessage``)."""
        from automsgs.msgs.geometry_msgs.transform_stamped_pb2 import TransformStamped

        message = TransformStamped()
        cls.set_header(message.header, stamp, parent)
        message.child_frame_id = child
        message.transform.translation.x = float(xyz[0])
        message.transform.translation.y = float(xyz[1])
        message.transform.translation.z = float(xyz[2])
        qx, qy, qz, qw = cls.yaw_to_quaternion(yaw)
        message.transform.rotation.x = qx
        message.transform.rotation.y = qy
        message.transform.rotation.z = qz
        message.transform.rotation.w = qw
        return message

    @classmethod
    def encode_tf_message(cls, transforms: Sequence[object]) -> TFMessage:
        """Pack transforms into a ``TFMessage``."""
        message = TFMessage()
        for item in transforms:
            slot = message.transforms.add()
            slot.header.stamp.sec = item.header.stamp.sec
            slot.header.stamp.nanosec = item.header.stamp.nanosec
            slot.header.frame_id = item.header.frame_id
            slot.child_frame_id = item.child_frame_id
            slot.transform.translation.x = item.transform.translation.x
            slot.transform.translation.y = item.transform.translation.y
            slot.transform.translation.z = item.transform.translation.z
            slot.transform.rotation.x = item.transform.rotation.x
            slot.transform.rotation.y = item.transform.rotation.y
            slot.transform.rotation.z = item.transform.rotation.z
            slot.transform.rotation.w = item.transform.rotation.w
        return message

    @classmethod
    def encode_clock(cls, stamp: Tuple[int, int]) -> Time:
        """Build simulation time for ``/clock``."""
        message = Time()
        message.sec = int(stamp[0])
        message.nanosec = int(stamp[1])
        return message

    @classmethod
    def encode_pose_stamped(
        cls,
        x: float,
        y: float,
        yaw: float,
        stamp: Tuple[int, int],
        frame_id: str,
    ) -> PoseStamped:
        """Build a planar ``geometry_msgs.PoseStamped``.

        Args:
            x: Position x (m).
            y: Position y (m).
            yaw: Heading (rad).
            stamp: Timestamp.
            frame_id: Frame (often ``map`` for ground truth).

        Returns:
            Populated :class:`PoseStamped`.
        """
        message = PoseStamped()
        cls.set_header(message.header, stamp, frame_id)
        qx, qy, qz, qw = cls.yaw_to_quaternion(yaw)
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.position.z = 0.0
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        return message

    @classmethod
    def encode_occupancy_grid(
        cls,
        grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        stamp: Tuple[int, int],
        frame_id: str,
    ) -> OccupancyGrid:
        """Build a ``map_msgs.OccupancyGrid`` (row-major, unknown=-1, occ=100).

        Args:
            grid: ``HxW`` int array.
            resolution: Meters per cell.
            origin_x: World x of cell (0,0) lower-left.
            origin_y: World y of cell (0,0) lower-left.
            stamp: Timestamp.
            frame_id: Usually ``map``.

        Returns:
            Populated :class:`OccupancyGrid`.
        """
        array = np.asarray(grid)
        height, width = array.shape
        message = OccupancyGrid()
        cls.set_header(message.header, stamp, frame_id)
        message.info.map_load_time.sec = int(stamp[0])
        message.info.map_load_time.nanosec = int(stamp[1])
        message.info.resolution = float(resolution)
        message.info.width = int(width)
        message.info.height = int(height)
        message.info.origin.position.x = float(origin_x)
        message.info.origin.position.y = float(origin_y)
        message.info.origin.position.z = 0.0
        message.info.origin.orientation.w = 1.0
        flat = array.reshape(-1).astype(np.int32)
        message.data.extend(int(value) for value in flat.tolist())
        return message

    @staticmethod
    def parse_command_velocity(message: Union[Twist, TwistStamped]) -> Tuple[float, float]:
        """Extract planar velocity commands from ``Twist`` or ``TwistStamped``.

        Args:
            message: Velocity message; also accepts duck-typed objects with ``twist``.

        Returns:
            ``(linear_x, angular_z)``.
        """
        if isinstance(message, TwistStamped):
            twist = message.twist
        elif isinstance(message, Twist):
            twist = message
        else:
            twist = getattr(message, "twist", message)
        return float(twist.linear.x), float(twist.angular.z)
