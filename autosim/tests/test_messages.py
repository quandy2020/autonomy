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

import numpy as np

from autosim.messages import Messages


def test_encode_laser_scan_fields():
    ranges = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    message = Messages.encode_laser_scan(
        ranges=ranges,
        stamp=(1, 500000000),
        frame_id="laser_link",
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=1.0,
        range_min=0.1,
        range_max=30.0,
        scan_time=0.1,
    )
    assert message.header.frame_id == "laser_link"
    assert message.header.stamp.sec == 1
    assert list(message.ranges) == [1.0, 2.0, 3.0]


def test_encode_image_rgb8():
    image = np.zeros((2, 3, 3), dtype=np.uint8)
    image[0, 0] = [1, 2, 3]
    message = Messages.encode_image(
        image, stamp=(0, 0), frame_id="camera_link", encoding="rgb8"
    )
    assert message.height == 2
    assert message.width == 3
    assert message.encoding == "rgb8"
    assert message.step == 9
    assert message.data[0:3] == bytes([1, 2, 3])


def test_rgb_and_depth_images_share_identical_stamp():
    """RGB-D pair must carry the same header.stamp for Atlas sync."""
    stamp = (42, 123456789)
    rgb = Messages.encode_image(
        np.zeros((4, 4, 3), dtype=np.uint8),
        stamp=stamp,
        frame_id="camera_link",
        encoding="rgb8",
    )
    depth = Messages.encode_image(
        np.ones((4, 4), dtype=np.float32),
        stamp=stamp,
        frame_id="camera_link",
        encoding="32FC1",
    )
    assert rgb.header.stamp.sec == depth.header.stamp.sec == stamp[0]
    assert rgb.header.stamp.nanosec == depth.header.stamp.nanosec == stamp[1]


def test_parse_command_velocity_twist_and_stamped():
    from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
    from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped

    twist = Twist()
    twist.linear.x = 0.2
    twist.angular.z = -0.1
    assert Messages.parse_command_velocity(twist) == (0.2, -0.1)

    stamped = TwistStamped()
    stamped.twist.linear.x = 0.3
    stamped.twist.angular.z = 0.4
    assert Messages.parse_command_velocity(stamped) == (0.3, 0.4)


def test_encode_odometry_frames():
    message = Messages.encode_odometry(
        x=1.0,
        y=2.0,
        yaw=0.0,
        linear=0.1,
        angular=0.0,
        stamp=(0, 0),
        frame_id="odom",
        child_frame_id="base_link",
    )
    assert message.header.frame_id == "odom"
    assert message.child_frame_id == "base_link"
    assert abs(message.pose.pose.pose.position.x - 1.0) < 1e-9


def test_encode_point_cloud2_xyz():
    points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
    message = Messages.encode_point_cloud2(points, (1, 0), "lidar_link")
    assert message.height == 1
    assert message.width == 2
    assert message.point_step == 12
    assert message.row_step == 24
    assert message.is_dense is False
    assert [field.name for field in message.fields] == ["x", "y", "z"]
    data = np.frombuffer(message.data, dtype=np.float32).reshape(2, 3)
    np.testing.assert_allclose(data, points)


def test_encode_point_cloud2_intensity_and_rgb():
    points = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]], dtype=np.float32)
    intensity = np.array([3.0, 4.0], dtype=np.float32)
    rgb = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)
    message = Messages.encode_point_cloud2(
        points, (0, 0), "map", intensity=intensity, rgb=rgb
    )
    assert message.point_step == 20
    names = [field.name for field in message.fields]
    assert names == ["x", "y", "z", "intensity", "rgb"]
    structured = np.frombuffer(
        message.data,
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4"), ("rgb", "u4")],
    )
    np.testing.assert_allclose(structured["intensity"], intensity)
    assert structured["rgb"][0] == (255 << 16)
    assert structured["rgb"][1] == (255 << 8)


def test_pack_rgb_uint32():
    packed = Messages.pack_rgb_uint32(np.array([[1, 2, 3]], dtype=np.uint8))
    assert packed[0] == (1 << 16) | (2 << 8) | 3
