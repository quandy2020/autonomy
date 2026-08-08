#!/usr/bin/env python3
"""Publish repeated arrays for Autoviz Table panel.

  /fake/table/status  diagnostic_msgs/DiagnosticArray  → array path status
  /fake/table/poses   geometry_msgs/PoseArray          → array path poses

Table: Channel + Array path (or auto-detect first repeated field).

  /usr/bin/python3 examples/python/14_tutiorial_table.py
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
from automsgs.msgs.diagnostic_msgs import (
    diagnostic_array_pb2,
    diagnostic_status_pb2,
)
from automsgs.msgs.geometry_msgs import pose_array_pb2

DS = diagnostic_status_pb2.DiagnosticStatus


def stamp(h, frame='map'):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def set_pose(pose, x, y, yaw, z=0.05):
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)


def add_kv(st, key, value):
    kv = st.values.add()
    kv.key, kv.value = key, value


def make_status(t):
    msg = diagnostic_array_pb2.DiagnosticArray()
    stamp(msg.header)
    rows = (
        (DS.OK, 'battery', 'nominal', 'batt0', f'{12.4 + 0.1 * math.sin(t):.2f} V'),
        (DS.WARN, 'cpu', 'hot', 'cpu0', f'{65 + 10 * math.sin(t * 0.7):.0f} C'),
        (DS.OK, 'lidar', 'streaming', 'lidar0', f'{int(20 + 5 * math.sin(t))} Hz'),
        (DS.ERROR if math.sin(t) > 0.8 else DS.OK, 'gps', 'fix', 'gps0',
         f'{4 + int(2 * abs(math.sin(t)))} sats'),
        (DS.STALE, 'camera', 'no frame', 'cam0', '0 Hz'),
    )
    for level, name, message, hw, val in rows:
        st = msg.status.add()
        st.level, st.name, st.message, st.hardware_id = level, name, message, hw
        add_kv(st, 'value', val)
    return msg


def make_poses(t):
    msg = pose_array_pb2.PoseArray()
    stamp(msg.header)
    for i in range(8):
        a = t + i * (2 * math.pi / 8)
        set_pose(msg.poses.add(), 2 * math.cos(a), 2 * math.sin(a), a, z=0.05 * i)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=5.0)
    args = ap.parse_args()

    autolink.init('table')
    n = autolink.Node('/autoviz/table')
    w_st = n.create_writer(
        '/fake/table/status', diagnostic_array_pb2.DiagnosticArray, qos_depth=5)
    w_poses = n.create_writer(
        '/fake/table/poses', pose_array_pb2.PoseArray, qos_depth=5)
    rate = autolink.Rate(args.rate)
    print(f'table @ {args.rate} Hz → /fake/table/{{status,poses}}')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            w_st.write(make_status(t))
            w_poses.write(make_poses(t))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
