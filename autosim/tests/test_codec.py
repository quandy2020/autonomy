import numpy as np

from autosim.codec import (
    encode_laser_scan,
    encode_image,
    encode_camera_info,
    encode_imu,
    encode_odometry,
    encode_pose_stamped,
    parse_cmd_vel,
)


def test_encode_laser_scan_fields():
    ranges = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    msg = encode_laser_scan(
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
    assert msg.header.frame_id == "laser_link"
    assert msg.header.stamp.sec == 1
    assert list(msg.ranges) == [1.0, 2.0, 3.0]


def test_encode_image_rgb8():
    img = np.zeros((2, 3, 3), dtype=np.uint8)
    img[0, 0] = [1, 2, 3]
    msg = encode_image(img, stamp=(0, 0), frame_id="camera_link", encoding="rgb8")
    assert msg.height == 2
    assert msg.width == 3
    assert msg.encoding == "rgb8"
    assert msg.step == 9
    assert msg.data[0:3] == bytes([1, 2, 3])


def test_parse_cmd_vel_twist_and_stamped():
    from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
    from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped

    t = Twist()
    t.linear.x = 0.2
    t.angular.z = -0.1
    assert parse_cmd_vel(t) == (0.2, -0.1)

    ts = TwistStamped()
    ts.twist.linear.x = 0.3
    ts.twist.angular.z = 0.4
    assert parse_cmd_vel(ts) == (0.3, 0.4)


def test_encode_odometry_frames():
    msg = encode_odometry(
        x=1.0,
        y=2.0,
        yaw=0.0,
        v=0.1,
        w=0.0,
        stamp=(0, 0),
        frame_id="odom",
        child_frame_id="base_link",
    )
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"
    assert abs(msg.pose.pose.pose.position.x - 1.0) < 1e-9
