#!/usr/bin/env python3
"""Publish Odometry / Path / Pose / PoseArray / PoseWithCovariance.

  /usr/bin/python3 examples/python/01_tutorial_poses.py
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
from automsgs.msgs.geometry_msgs import (
    pose_array_pb2,
    pose_stamped_pb2,
    pose_with_covariance_stamped_pb2,
)
from automsgs.msgs.nav_msgs import odometry_pb2, path_pb2

N = 40


def stamp(h, frame='map'):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def set_pose(pose, x, y, yaw, z=0.05):
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)


def on_path(u, phase=0.0):
    """u in [0, 1] → x, y, yaw on a sine path."""
    x = u * 8.0 - 4.0
    y = 2.0 * math.sin(u * 2 * math.pi + phase)
    dy = 4 * math.pi * math.cos(u * 2 * math.pi + phase)
    return x, y, math.atan2(dy, 8.0)


def cov_xy(sx, sy):
    c = [0.0] * 36
    c[0], c[7] = sx * sx, sy * sy
    c[14] = c[21] = c[28] = c[35] = 1e-4
    return c


def make_path(phase):
    msg = path_pb2.Path()
    stamp(msg.header)
    for i in range(N):
        x, y, yaw = on_path(i / (N - 1), phase)
        ps = msg.poses.add()
        stamp(ps.header)
        set_pose(ps.pose, x, y, yaw)
    return msg


def make_odom(phase, u):
    x, y, yaw = on_path(u, phase)
    msg = odometry_pb2.Odometry()
    stamp(msg.header)
    msg.child_frame_id = 'base_link'
    stamp(msg.pose.pose.header)
    set_pose(msg.pose.pose.pose, x, y, yaw)
    msg.pose.covariance.extend(cov_xy(0.05, 0.05))
    msg.twist.twist.linear.x = 0.8
    return msg


def make_pose(phase):
    x, y, yaw = on_path(1.0, phase)
    msg = pose_stamped_pb2.PoseStamped()
    stamp(msg.header)
    set_pose(msg.pose, x, y, yaw, z=0.1)
    return msg


def make_pose_array(phase):
    msg = pose_array_pb2.PoseArray()
    stamp(msg.header)
    for i in range(8):
        x, y, yaw = on_path(i / 7.0, phase)
        set_pose(msg.poses.add(), x, y + 0.4, yaw)
    return msg


def make_pose_cov(phase, u):
    x, y, yaw = on_path(u, phase)
    msg = pose_with_covariance_stamped_pb2.PoseWithCovarianceStamped()
    stamp(msg.header)
    stamp(msg.pose.pose.header)
    set_pose(msg.pose.pose.pose, x, y - 0.5, yaw)
    msg.pose.covariance.extend(cov_xy(0.3, 0.15))
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--static', action='store_true')
    args = ap.parse_args()

    autolink.init('poses')
    n = autolink.Node('/autoviz/poses')
    w = {
        'odom': n.create_writer('/fake/odom', odometry_pb2.Odometry, qos_depth=10),
        'path': n.create_writer('/fake/path', path_pb2.Path, qos_depth=10),
        'pose': n.create_writer(
            '/fake/pose', pose_stamped_pb2.PoseStamped, qos_depth=10),
        'arr': n.create_writer(
            '/fake/pose_array', pose_array_pb2.PoseArray, qos_depth=10),
        'cov': n.create_writer(
            '/fake/pose_with_covariance',
            pose_with_covariance_stamped_pb2.PoseWithCovarianceStamped,
            qos_depth=10),
    }
    rate = autolink.Rate(args.rate)
    print(f'poses @ {args.rate} Hz → /fake/{{odom,path,pose,pose_array,pose_with_covariance}}')

    phase, u = 0.0, 0.0
    try:
        while not autolink.is_shutdown():
            w['path'].write(make_path(phase))
            w['odom'].write(make_odom(phase, u))
            w['pose'].write(make_pose(phase))
            w['arr'].write(make_pose_array(phase))
            w['cov'].write(make_pose_cov(phase, u))
            if not args.static:
                u = (u + 0.01) % 1.0
                phase += 0.02
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
