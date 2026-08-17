"""automsgs protobuf encode/decode helpers."""

from __future__ import annotations

import math
from typing import Sequence, Tuple, Union

import numpy as np

from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2
from automsgs.msgs.sensor_msgs.point_field_pb2 import PointField


class Messages:
    """Stateless helpers: NumPy / geometry quantities ↔ automsgs messages."""

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

    @classmethod
    def encode_point_cloud2(
        cls,
        points: np.ndarray,
        stamp: Tuple[int, int],
        frame_id: str,
    ) -> PointCloud2:
        """Build a ``sensor_msgs.PointCloud2`` with ``x,y,z`` float32 fields.

        Args:
            points: ``Nx3`` float array in the sensor frame; ``N`` may be 0.
            stamp: Timestamp ``(sec, nanosec)``.
            frame_id: Point-cloud frame.

        Returns:
            Populated :class:`PointCloud2` with ``is_dense=False``.
        """
        array = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        message = PointCloud2()
        cls.set_header(message.header, stamp, frame_id)
        message.height = 1
        message.width = int(array.shape[0])
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = int(message.point_step * message.width)
        message.is_dense = False
        for name, offset in (("x", 0), ("y", 4), ("z", 8)):
            field = message.fields.add()
            field.name = name
            field.offset = offset
            field.datatype = PointField.FLOAT32
            field.count = 1
        message.data = array.tobytes()
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
    ) -> Imu:
        """Build a ``sensor_msgs.Imu``.

        Args:
            stamp: Timestamp.
            frame_id: IMU frame.
            orientation: Quaternion ``(x, y, z, w)``.
            angular_velocity: Angular velocity ``(wx, wy, wz)`` (rad/s).
            linear_acceleration: Linear acceleration ``(ax, ay, az)`` (m/s²).

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
        message.angular_velocity_covariance.extend([0.0] * 9)
        message.linear_acceleration_covariance.extend([0.0] * 9)
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
        message.pose.covariance.extend([0.0] * 36)
        message.twist.twist.linear.x = float(linear)
        message.twist.twist.angular.z = float(angular)
        message.twist.covariance.extend([0.0] * 36)
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
