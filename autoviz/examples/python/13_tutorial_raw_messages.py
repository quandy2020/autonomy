#!/usr/bin/env python3
"""Publish nested protobufs for Autoviz Raw Messages panel.

  /fake/raw/odom   nav_msgs/Odometry
  /fake/raw/poses  geometry_msgs/PoseArray
  /fake/raw/imu    sensor_msgs/Imu

Raw Messages: select channel → full DebugString; optional Path e.g. pose.pose.pose.position.x

  /usr/bin/python3 examples/python/13_tutorial_raw_messages.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import math
import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.geometry_msgs import pose_array_pb2
from automsgs.msgs.nav_msgs import odometry_pb2
from automsgs.msgs.sensor_msgs import imu_pb2


def stamp(h, frame='map'):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def set_pose(pose, x, y, yaw, z=0.05):
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)


def make_odom(t):
    x, y = 2.0 * math.cos(t), 1.5 * math.sin(t)
    yaw = t + math.pi / 2
    msg = odometry_pb2.Odometry()
    stamp(msg.header)
    msg.child_frame_id = 'base_link'
    stamp(msg.pose.pose.header)
    set_pose(msg.pose.pose.pose, x, y, yaw)
    msg.pose.covariance.extend([0.05 if i % 7 == 0 else 0.0 for i in range(36)])
    msg.twist.twist.linear.x = 0.8
    msg.twist.twist.angular.z = 0.4
    msg.twist.covariance.extend([0.01 if i % 7 == 0 else 0.0 for i in range(36)])
    return msg


def make_poses(t):
    msg = pose_array_pb2.PoseArray()
    stamp(msg.header)
    for i in range(5):
        a = t + i * (2 * math.pi / 5)
        set_pose(msg.poses.add(), math.cos(a), math.sin(a), a, z=0.1 * i)
    return msg


def make_imu(t):
    msg = imu_pb2.Imu()
    stamp(msg.header, 'imu_link')
    msg.orientation.z = math.sin(t * 0.5)
    msg.orientation.w = math.cos(t * 0.5)
    msg.angular_velocity.z = 0.2 * math.cos(t)
    msg.linear_acceleration.x = 0.1 * math.sin(t)
    msg.linear_acceleration.z = 9.81
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    autolink.init('raw')
    n = autolink.Node('/autoviz/raw')
    w_odom = n.create_writer('/fake/raw/odom', odometry_pb2.Odometry, qos_depth=5)
    w_poses = n.create_writer(
        '/fake/raw/poses', pose_array_pb2.PoseArray, qos_depth=5)
    w_imu = n.create_writer('/fake/raw/imu', imu_pb2.Imu, qos_depth=5)
    rate = autolink.Rate(args.rate)
    print(f'raw @ {args.rate} Hz → /fake/raw/{{odom,poses,imu}}')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            w_odom.write(make_odom(t))
            w_poses.write(make_poses(t))
            w_imu.write(make_imu(t))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
