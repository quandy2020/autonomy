#!/usr/bin/env python3
"""Publish common sensor_msgs on /fake/* for Autoviz.

  imu / image / camera_info / depth / scan / point_cloud / point_cloud2 / range

  /usr/bin/python3 examples/python/04_tutorial_sensor.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import math
import struct
import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.sensor_msgs import (
    camera_info_pb2,
    image_pb2,
    imu_pb2,
    laser_scan_pb2,
    point_cloud_pb2,
    point_cloud2_pb2,
    point_field_pb2,
    range_pb2,
)
from automsgs.msgs.tf2_msgs import tf_message_pb2

RW, RH = 160, 120
DW, DH = 80, 60


def stamp(h, frame='map'):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def make_imu(t):
    msg = imu_pb2.Imu()
    stamp(msg.header)
    yaw = 0.3 * math.sin(t)
    msg.orientation.z = math.sin(yaw * 0.5)
    msg.orientation.w = math.cos(yaw * 0.5)
    msg.angular_velocity.z = 0.3 * math.cos(t)
    msg.linear_acceleration.z = 9.81
    return msg


def make_rgb(phase=0):
    msg = image_pb2.Image()
    stamp(msg.header, 'camera')
    msg.height, msg.width, msg.encoding, msg.step = RH, RW, 'rgb8', RW * 3
    buf = bytearray(RH * RW * 3)
    for y in range(RH):
        for x in range(RW):
            i = (y * RW + x) * 3
            buf[i] = (x + phase) % 256
            buf[i + 1] = (y + phase) % 256
            buf[i + 2] = (x + y + phase) % 256
    msg.data = bytes(buf)
    return msg


def make_depth():
    msg = image_pb2.Image()
    stamp(msg.header, 'camera')
    msg.height, msg.width, msg.encoding, msg.step = DH, DW, '16UC1', DW * 2
    cx, cy = DW // 2, DH // 2
    buf = bytearray()
    for y in range(DH):
        for x in range(DW):
            mm = 1200 if (x - cx) ** 2 + (y - cy) ** 2 < 80 else 2000
            buf += struct.pack('<H', mm)
    msg.data = bytes(buf)
    return msg


def make_camera_info():
    msg = camera_info_pb2.CameraInfo()
    stamp(msg.header, 'camera')
    msg.height, msg.width = DH, DW
    msg.distortion_model = 'plumb_bob'
    fx = fy = 60.0
    cx, cy = DW / 2.0, DH / 2.0
    msg.d.extend([0.0] * 5)
    msg.k.extend([fx, 0, cx, 0, fy, cy, 0, 0, 1])
    msg.r.extend([1, 0, 0, 0, 1, 0, 0, 0, 1])
    msg.p.extend([fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0])
    return msg


def make_scan(t):
    msg = laser_scan_pb2.LaserScan()
    stamp(msg.header, 'laser')
    n = 180
    msg.angle_min = -math.pi / 2
    msg.angle_max = math.pi / 2
    msg.angle_increment = math.pi / (n - 1)
    msg.range_min, msg.range_max, msg.scan_time = 0.1, 10.0, 0.1
    for i in range(n):
        a = msg.angle_min + i * msg.angle_increment
        msg.ranges.append(2.0 + 0.5 * math.sin(a * 3 + t))
    return msg


def make_pc(t):
    msg = point_cloud_pb2.PointCloud()
    stamp(msg.header)
    for i in range(40):
        p = msg.points.add()
        p.x, p.y, p.z = 0.05 * i, math.sin(0.2 * i + t), 0.05 * (i % 5)
    return msg


def make_pc2(t):
    msg = point_cloud2_pb2.PointCloud2()
    stamp(msg.header)
    n = 80
    msg.height, msg.width, msg.is_dense = 1, n, True
    msg.point_step, msg.row_step = 12, 12 * n
    for name, off in (('x', 0), ('y', 4), ('z', 8)):
        f = msg.fields.add()
        f.name, f.offset, f.count = name, off, 1
        f.datatype = point_field_pb2.PointField.FLOAT32
    buf = bytearray()
    for i in range(n):
        buf += struct.pack('<fff', 0.05 * i, math.sin(0.2 * i + t), 0.05 * (i % 5))
    msg.data = bytes(buf)
    return msg


def make_range(t):
    msg = range_pb2.Range()
    stamp(msg.header, 'sonar')
    msg.radiation_type = range_pb2.Range.ULTRASOUND
    msg.field_of_view = 0.3
    msg.min_range, msg.max_range = 0.05, 4.0
    msg.range = 1.5 + 0.5 * math.sin(t)
    return msg


def _add_static_tf(msg, parent, child, x, y, z):
    ts = msg.transforms.add()
    stamp(ts.header, parent)
    ts.child_frame_id = child
    ts.transform.translation.x = x
    ts.transform.translation.y = y
    ts.transform.translation.z = z
    ts.transform.rotation.w = 1.0


def make_sensor_tf():
    """Static map→sensor frames so LaserScan works with Fixed Frame=map."""
    msg = tf_message_pb2.TFMessage()
    _add_static_tf(msg, 'map', 'base_link', 0.0, 0.0, 0.0)
    _add_static_tf(msg, 'base_link', 'laser', 0.2, 0.0, 0.15)
    _add_static_tf(msg, 'base_link', 'camera', 0.1, 0.0, 0.3)
    _add_static_tf(msg, 'base_link', 'sonar', 0.15, 0.0, 0.1)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    autolink.init('sensor')
    n = autolink.Node('/autoviz/sensor')
    w = {
        'imu': n.create_writer('/fake/imu', imu_pb2.Imu, qos_depth=2),
        'rgb': n.create_writer('/fake/image', image_pb2.Image, qos_depth=2),
        'info': n.create_writer(
            '/fake/camera_info', camera_info_pb2.CameraInfo, qos_depth=2),
        'depth': n.create_writer('/fake/depth', image_pb2.Image, qos_depth=2),
        'scan': n.create_writer('/fake/scan', laser_scan_pb2.LaserScan, qos_depth=2),
        'pc': n.create_writer(
            '/fake/point_cloud', point_cloud_pb2.PointCloud, qos_depth=2),
        'pc2': n.create_writer(
            '/fake/point_cloud2', point_cloud2_pb2.PointCloud2, qos_depth=2),
        'rng': n.create_writer('/fake/range', range_pb2.Range, qos_depth=2),
        'tf': n.create_writer('/tf', tf_message_pb2.TFMessage, qos_depth=2),
    }
    rate = autolink.Rate(args.rate)
    print(
        f'sensor @ {args.rate} Hz → /fake/{{imu,image,camera_info,depth,scan,'
        f'point_cloud,point_cloud2,range}} + /tf'
    )

    t0, phase = time.time(), 0
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            w['imu'].write(make_imu(t))
            w['rgb'].write(make_rgb(phase))
            w['info'].write(make_camera_info())
            w['depth'].write(make_depth())
            w['scan'].write(make_scan(t))
            w['pc'].write(make_pc(t))
            w['pc2'].write(make_pc2(t))
            w['rng'].write(make_range(t))
            w['tf'].write(make_sensor_tf())
            phase = (phase + 3) % 256
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
