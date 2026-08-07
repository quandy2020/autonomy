#!/usr/bin/env python3
"""Publish nav_msgs/Path over autolink for Autoviz Path display.

Publishes ``automsgs.msgs.nav_msgs.Path`` on ``/fake/path`` (default), matching
``config/default.autoviz`` with Fixed Frame ``map``.

Usage:
  Terminal 1:
    /usr/bin/python3 examples/python/01_tutorial_show_path.py

  Terminal 2 (verify with autolink channel echo):
    ./build/autonomy/bin/autolink channel echo /fake/path --once

  Terminal 3 (optional, visualize in Autoviz):
    ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import math
import sys
import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.nav_msgs import path_pb2
from automsgs.msgs.std_msgs import header_pb2


def make_header(frame_id: str, stamp_ns: int) -> header_pb2.Header:
    header = header_pb2.Header()
    header.frame_id = frame_id
    header.stamp.sec = stamp_ns // 1_000_000_000
    header.stamp.nanosec = stamp_ns % 1_000_000_000
    return header


def build_path(frame_id: str, phase: float, point_count: int) -> path_pb2.Path:
    """Build a sine-shaped path in the XY plane."""
    count = max(point_count, 2)
    stamp_ns = time.time_ns()
    header = make_header(frame_id, stamp_ns)

    msg = path_pb2.Path()
    msg.header.CopyFrom(header)

    for index in range(count):
        t = index / (count - 1)
        x = t * 8.0 - 4.0
        y = 2.0 * math.sin(t * 2 * math.pi + phase)

        pose = msg.poses.add()
        pose.header.CopyFrom(header)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.05

        if index + 1 < count:
            t_next = (index + 1) / (count - 1)
            x_next = t_next * 8.0 - 4.0
            y_next = 2.0 * math.sin(t_next * 2 * math.pi + phase)
            yaw = math.atan2(y_next - y, x_next - x)
            half = yaw * 0.5
            pose.pose.orientation.z = math.sin(half)
            pose.pose.orientation.w = math.cos(half)
        else:
            pose.pose.orientation.w = 1.0

    return msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Publish automsgs Path messages for Autoviz.')
    parser.add_argument('--channel', default='/fake/path')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--points', type=int, default=40)
    parser.add_argument('--static', action='store_true')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rate <= 0:
        sys.exit('--rate must be > 0')

    autolink.init('autoviz_tutorial_show_path')
    node = autolink.Node('path_publisher')
    writer = node.create_writer(args.channel, path_pb2.Path, qos_depth=10)
    rate = autolink.Rate(args.rate)

    mode = 'static' if args.static else 'animated sine'
    print(
        'Autoviz Path tutorial publisher\n'
        f'  channel : {args.channel}\n'
        f'  frame   : {args.frame}\n'
        f'  rate    : {args.rate} Hz\n'
        f'  mode    : {mode}\n\n'
        'Verify in another terminal:\n'
        f'  ./build/autonomy/bin/autolink channel echo {args.channel} --once\n'
        'Visualize in Autoviz:\n'
        '  ./build/autonomy/bin/autoviz'
    )

    phase = 0.0
    try:
        while not autolink.is_shutdown():
            writer.write(build_path(args.frame, phase, args.points))
            if not args.static:
                phase += 0.15
            rate.sleep()
    except KeyboardInterrupt:
        print('\nStopping publisher.')
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
