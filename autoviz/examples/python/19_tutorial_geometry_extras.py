#!/usr/bin/env python3
"""Publish extra geometry displays for Autoviz.

  /fake/grid_cells  map_msgs/GridCells
  /fake/point       geometry_msgs/PointStamped
  /fake/polygon     geometry_msgs/PolygonStamped
  /fake/twist       geometry_msgs/TwistStamped
  /fake/accel       geometry_msgs/AccelStamped

  /usr/bin/python3 examples/python/19_tutorial_geometry_extras.py
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
    accel_stamped_pb2,
    point_stamped_pb2,
    polygon_stamped_pb2,
    twist_stamped_pb2,
)
from automsgs.msgs.map_msgs import grid_cells_pb2


def stamp(h, frame='map'):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def make_grid_cells(t):
    msg = grid_cells_pb2.GridCells()
    stamp(msg.header)
    msg.cell_width = msg.cell_height = 0.2
    for i in range(20):
        a = t * 0.5 + i * 0.3
        p = msg.cells.add()
        p.x, p.y, p.z = math.cos(a) * 2.0, math.sin(a) * 2.0, 0.0
    return msg


def make_point(t):
    msg = point_stamped_pb2.PointStamped()
    stamp(msg.header)
    msg.point.x = 1.5 * math.cos(t)
    msg.point.y = 1.5 * math.sin(t)
    msg.point.z = 0.3
    return msg


def make_polygon(t):
    msg = polygon_stamped_pb2.PolygonStamped()
    stamp(msg.header)
    r = 1.2 + 0.2 * math.sin(t)
    for i in range(6):
        a = t * 0.3 + i * math.pi / 3
        p = msg.polygon.points.add()
        p.x, p.y, p.z = r * math.cos(a), r * math.sin(a), 0.0
    return msg


def make_twist(t):
    msg = twist_stamped_pb2.TwistStamped()
    stamp(msg.header, 'base_link')
    msg.twist.linear.x = 0.5 + 0.3 * math.sin(t)
    msg.twist.angular.z = 0.4 * math.cos(t)
    return msg


def make_accel(t):
    msg = accel_stamped_pb2.AccelStamped()
    stamp(msg.header, 'base_link')
    msg.accel.linear.x = 0.2 * math.sin(t * 1.5)
    msg.accel.linear.z = 0.1 * math.cos(t)
    msg.accel.angular.y = 0.05 * math.sin(t)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    autolink.init('geometry')
    n = autolink.Node('/autoviz/geometry')
    w_gc = n.create_writer('/fake/grid_cells', grid_cells_pb2.GridCells, qos_depth=5)
    w_pt = n.create_writer('/fake/point', point_stamped_pb2.PointStamped, qos_depth=5)
    w_poly = n.create_writer(
        '/fake/polygon', polygon_stamped_pb2.PolygonStamped, qos_depth=5)
    w_tw = n.create_writer(
        '/fake/twist', twist_stamped_pb2.TwistStamped, qos_depth=5)
    w_ac = n.create_writer(
        '/fake/accel', accel_stamped_pb2.AccelStamped, qos_depth=5)
    rate = autolink.Rate(args.rate)
    print(f'geometry @ {args.rate} Hz → /fake/{{grid_cells,point,polygon,twist,accel}}')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            w_gc.write(make_grid_cells(t))
            w_pt.write(make_point(t))
            w_poly.write(make_polygon(t))
            w_tw.write(make_twist(t))
            w_ac.write(make_accel(t))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
