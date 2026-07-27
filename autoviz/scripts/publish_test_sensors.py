#!/usr/bin/env python3
"""Publish test PointCloud2 + LaserScan for Autoviz Ogre pick GUI verification."""

from __future__ import annotations

import math
import struct
import sys
import time

try:
    from automsgs.msgs.nav_msgs import path_pb2
    from automsgs.msgs.sensor_msgs import laser_scan_pb2, point_cloud2_pb2, point_field_pb2
    from automsgs.msgs.std_msgs import header_pb2
    from automsgs.msgs.visualization_msgs import marker_pb2
except ImportError as exc:
    print("Run from workspace with automsgs Python on PYTHONPATH", file=sys.stderr)
    raise SystemExit(1) from exc

try:
    import autolink
except ImportError:
    print("autolink Python bindings required for publishing", file=sys.stderr)
    raise SystemExit(1)


def make_header(frame_id: str, stamp_ns: int) -> header_pb2.Header:
    header = header_pb2.Header()
    header.frame_id = frame_id
    header.stamp.sec = stamp_ns // 1_000_000_000
    header.stamp.nanosec = stamp_ns % 1_000_000_000
    return header


def build_point_cloud2() -> point_cloud2_pb2.PointCloud2:
    msg = point_cloud2_pb2.PointCloud2()
    msg.header.CopyFrom(make_header("base_link", time.time_ns()))
    msg.height = 1
    msg.width = 5
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    for name, offset, datatype in (
        ("x", 0, point_field_pb2.PointField.FLOAT32),
        ("y", 4, point_field_pb2.PointField.FLOAT32),
        ("z", 8, point_field_pb2.PointField.FLOAT32),
        ("rgb", 12, point_field_pb2.PointField.UINT32),
    ):
        field = msg.fields.add()
        field.name = name
        field.offset = offset
        field.datatype = datatype
        field.count = 1

    buf = bytearray()
    colors = [(255, 80, 80), (255, 160, 80), (80, 220, 80), (80, 160, 255), (200, 80, 255)]
    for i in range(5):
        rgb = (colors[i][0] << 16) | (colors[i][1] << 8) | colors[i][2]
        buf.extend(struct.pack("<fffI", float(i) * 0.5, 0.0, 0.5, rgb))
    msg.data = bytes(buf)
    return msg


def build_laser_scan() -> laser_scan_pb2.LaserScan:
    msg = laser_scan_pb2.LaserScan()
    msg.header.CopyFrom(make_header("base_scan", time.time_ns()))
    msg.angle_min = -math.pi / 2
    msg.angle_max = math.pi / 2
    msg.angle_increment = math.pi / 180
    msg.range_min = 0.1
    msg.range_max = 10.0
    count = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
    for i in range(count):
        angle = msg.angle_min + i * msg.angle_increment
        msg.ranges.append(2.0 + 0.5 * math.sin(angle * 3))
        msg.intensities.append(float(i) / count)
    return msg


def build_path() -> path_pb2.Path:
    msg = path_pb2.Path()
    msg.header.CopyFrom(make_header("base_link", time.time_ns()))
    for i in range(6):
        pose = msg.poses.add()
        pose.header.CopyFrom(msg.header)
        pose.pose.position.x = float(i) * 0.4
        pose.pose.position.y = 0.3 * math.sin(i * 0.8)
        pose.pose.position.z = 0.2
        pose.pose.orientation.w = 1.0
    return msg


def build_marker() -> marker_pb2.Marker:
    msg = marker_pb2.Marker()
    msg.header.CopyFrom(make_header("base_link", time.time_ns()))
    msg.ns = "autoviz_test"
    msg.id = 1
    msg.type = marker_pb2.Marker.ARROW
    msg.action = marker_pb2.Marker.ADD
    msg.pose.position.x = 0.5
    msg.pose.position.y = 0.5
    msg.pose.position.z = 0.5
    msg.pose.orientation.w = 1.0
    msg.scale.x = 1.0
    msg.scale.y = 0.1
    msg.scale.z = 0.1
    msg.color.r = 1.0
    msg.color.g = 0.4
    msg.color.b = 0.1
    msg.color.a = 1.0
    return msg


def build_cube_marker() -> marker_pb2.Marker:
    msg = marker_pb2.Marker()
    msg.header.CopyFrom(make_header("base_link", time.time_ns()))
    msg.ns = "autoviz_test"
    msg.id = 2
    msg.type = marker_pb2.Marker.CUBE
    msg.action = marker_pb2.Marker.ADD
    msg.pose.position.x = -0.5
    msg.pose.position.y = 0.5
    msg.pose.position.z = 0.35
    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.4
    msg.scale.y = 0.4
    msg.scale.z = 0.4
    msg.color.r = 0.2
    msg.color.g = 0.7
    msg.color.b = 1.0
    msg.color.a = 0.9
    return msg


def build_sphere_marker() -> marker_pb2.Marker:
    msg = marker_pb2.Marker()
    msg.header.CopyFrom(make_header("base_link", time.time_ns()))
    msg.ns = "autoviz_test"
    msg.id = 3
    msg.type = marker_pb2.Marker.SPHERE
    msg.action = marker_pb2.Marker.ADD
    msg.pose.position.x = 0.0
    msg.pose.position.y = -0.5
    msg.pose.position.z = 0.35
    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.35
    msg.scale.y = 0.35
    msg.scale.z = 0.35
    msg.color.r = 0.9
    msg.color.g = 0.3
    msg.color.b = 0.9
    msg.color.a = 0.9
    return msg


def build_text_marker() -> marker_pb2.Marker:
    msg = marker_pb2.Marker()
    msg.header.CopyFrom(make_header("base_link", time.time_ns()))
    msg.ns = "autoviz_test"
    msg.id = 4
    msg.type = marker_pb2.Marker.TEXT_VIEW_FACING
    msg.action = marker_pb2.Marker.ADD
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 1.2
    msg.pose.orientation.w = 1.0
    msg.scale.z = 0.25
    msg.text = "Autoviz Ogre Text"
    msg.color.r = 1.0
    msg.color.g = 1.0
    msg.color.b = 1.0
    msg.color.a = 1.0
    return msg


def main() -> None:
    node = autolink.Node("autoviz_test_sensor_publisher")
    pc_writer = node.create_writer("/test/pointcloud", point_cloud2_pb2.PointCloud2)
    scan_writer = node.create_writer("/test/laserscan", laser_scan_pb2.LaserScan)
    path_writer = node.create_writer("/test/path", path_pb2.Path)
    marker_writer = node.create_writer("/test/marker", marker_pb2.Marker)
    print("Publishing /test/pointcloud, /test/laserscan, /test/path, /test/marker at 2 Hz")
    print("Marker: ARROW + CUBE + SPHERE + TEXT (ids 1-4) each cycle on /test/marker")
    print("Use Ogre viewport; Fixed Frame=base_link")
    while True:
        pc_writer.write(build_point_cloud2())
        scan_writer.write(build_laser_scan())
        path_writer.write(build_path())
        marker_writer.write(build_marker())
        marker_writer.write(build_cube_marker())
        marker_writer.write(build_sphere_marker())
        marker_writer.write(build_text_marker())
        time.sleep(0.5)


if __name__ == "__main__":
    main()
