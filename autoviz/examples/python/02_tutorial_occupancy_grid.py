#!/usr/bin/env python3
"""Publish OccupancyGrid on /fake/occupancy_grid.

  /usr/bin/python3 examples/python/02_tutorial_occupancy_grid.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.map_msgs import occupancy_grid_pb2

W, H, RES = 40, 30, 0.1


def make_grid(frame='map'):
    msg = occupancy_grid_pb2.OccupancyGrid()
    now = time.time_ns()
    msg.header.frame_id = frame
    msg.header.stamp.sec = now // 1_000_000_000
    msg.header.stamp.nanosec = now % 1_000_000_000
    msg.info.resolution, msg.info.width, msg.info.height = RES, W, H
    msg.info.origin.position.x = -0.5 * W * RES
    msg.info.origin.position.y = -0.5 * H * RES
    msg.info.origin.orientation.w = 1.0
    data = []
    for y in range(H):
        for x in range(W):
            edge = x in (0, W - 1) or y in (0, H - 1)
            data.append(100 if edge else (-1 if (x + y) % 9 == 0 else 0))
    msg.data.extend(data)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=1.0)
    args = ap.parse_args()

    autolink.init('grid')
    n = autolink.Node('/autoviz/grid')
    w = n.create_writer(
        '/fake/occupancy_grid', occupancy_grid_pb2.OccupancyGrid, qos_depth=2)
    rate = autolink.Rate(args.rate)
    print(f'grid @ {args.rate} Hz → /fake/occupancy_grid ({W}x{H})')
    try:
        while not autolink.is_shutdown():
            w.write(make_grid())
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
