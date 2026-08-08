#!/usr/bin/env python3
"""Publish TF: map → base_link → laser / camera.

  /usr/bin/python3 examples/python/05_tutorial_transform_tree.py
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
from automsgs.msgs.tf2_msgs import tf_message_pb2


def stamp(h):
    now = time.time_ns()
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def add_tf(msg, parent, child, xyz, yaw=0.0):
    ts = msg.transforms.add()
    stamp(ts.header)
    ts.header.frame_id = parent
    ts.child_frame_id = child
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = xyz
    ts.transform.rotation.z = math.sin(yaw * 0.5)
    ts.transform.rotation.w = math.cos(yaw * 0.5)


def make_tree(t):
    msg = tf_message_pb2.TFMessage()
    add_tf(msg, 'map', 'base_link',
           (2 * math.cos(t), 2 * math.sin(t), 0), yaw=t + math.pi / 2)
    add_tf(msg, 'base_link', 'laser', (0.2, 0, 0.15))
    add_tf(msg, 'base_link', 'camera', (0.1, 0, 0.3))
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    autolink.init('tf')
    n = autolink.Node('/autoviz/tf')
    w = n.create_writer('/tf', tf_message_pb2.TFMessage, qos_depth=10)
    rate = autolink.Rate(args.rate)
    print(f'tf @ {args.rate} Hz → /tf (map→base_link→laser/camera)')
    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            w.write(make_tree(time.time() - t0))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
