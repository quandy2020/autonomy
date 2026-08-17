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

Stamp = Tuple[int, int]


def _set_header(header, stamp: Stamp, frame_id: str) -> None:
    header.stamp.sec = int(stamp[0])
    header.stamp.nanosec = int(stamp[1])
    header.frame_id = frame_id


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


def encode_laser_scan(
    ranges: np.ndarray,
    stamp: Stamp,
    frame_id: str,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    scan_time: float,
    intensities: np.ndarray | None = None,
) -> LaserScan:
    msg = LaserScan()
    _set_header(msg.header, stamp, frame_id)
    msg.angle_min = float(angle_min)
    msg.angle_max = float(angle_max)
    msg.angle_increment = float(angle_increment)
    msg.time_increment = 0.0
    msg.scan_time = float(scan_time)
    msg.range_min = float(range_min)
    msg.range_max = float(range_max)
    msg.ranges.extend(float(r) for r in ranges.reshape(-1))
    if intensities is not None:
        msg.intensities.extend(float(i) for i in intensities.reshape(-1))
    return msg


def encode_image(
    image: np.ndarray,
    stamp: Stamp,
    frame_id: str,
    encoding: str,
) -> Image:
    msg = Image()
    _set_header(msg.header, stamp, frame_id)
    if image.ndim == 2:
        height, width = image.shape
        channels = 1
    else:
        height, width, channels = image.shape
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(width * channels * image.dtype.itemsize)
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg


def encode_camera_info(
    stamp: Stamp,
    frame_id: str,
    width: int,
    height: int,
    k: Sequence[float],
    p: Sequence[float] | None = None,
) -> CameraInfo:
    msg = CameraInfo()
    _set_header(msg.header, stamp, frame_id)
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = "plumb_bob"
    msg.d.extend([0.0, 0.0, 0.0, 0.0, 0.0])
    k9 = [float(v) for v in k]
    if len(k9) != 9:
        raise ValueError("camera matrix K must have length 9")
    msg.k.extend(k9)
    msg.r.extend([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    if p is None:
        fx, fy, cx, cy = k9[0], k9[4], k9[2], k9[5]
        p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.p.extend(float(v) for v in p)
    return msg


def encode_imu(
    stamp: Stamp,
    frame_id: str,
    orient_xyzw: Tuple[float, float, float, float],
    gyro_xyz: Tuple[float, float, float],
    accel_xyz: Tuple[float, float, float],
) -> Imu:
    msg = Imu()
    _set_header(msg.header, stamp, frame_id)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = orient_xyzw
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro_xyz
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel_xyz
    msg.orientation_covariance.extend([-1.0] + [0.0] * 8)
    msg.angular_velocity_covariance.extend([0.0] * 9)
    msg.linear_acceleration_covariance.extend([0.0] * 9)
    return msg


def encode_odometry(
    x: float,
    y: float,
    yaw: float,
    v: float,
    w: float,
    stamp: Stamp,
    frame_id: str,
    child_frame_id: str,
) -> Odometry:
    msg = Odometry()
    _set_header(msg.header, stamp, frame_id)
    msg.child_frame_id = child_frame_id
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    pose = msg.pose.pose
    _set_header(pose.header, stamp, frame_id)
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    msg.pose.covariance.extend([0.0] * 36)
    msg.twist.twist.linear.x = float(v)
    msg.twist.twist.angular.z = float(w)
    msg.twist.covariance.extend([0.0] * 36)
    return msg


def encode_pose_stamped(
    x: float,
    y: float,
    yaw: float,
    stamp: Stamp,
    frame_id: str,
) -> PoseStamped:
    msg = PoseStamped()
    _set_header(msg.header, stamp, frame_id)
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def parse_cmd_vel(msg: Union[Twist, TwistStamped]) -> Tuple[float, float]:
    if isinstance(msg, TwistStamped):
        twist = msg.twist
    elif isinstance(msg, Twist):
        twist = msg
    else:
        twist = getattr(msg, "twist", msg)
    return float(twist.linear.x), float(twist.angular.z)
