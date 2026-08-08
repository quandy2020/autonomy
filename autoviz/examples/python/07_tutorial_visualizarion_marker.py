#!/usr/bin/env python3
"""Publish Marker / MarkerArray / InteractiveMarker.

  /fake/marker  /fake/marker_array  /fake/init  /fake/update

  /usr/bin/python3 examples/python/07_tutorial_visualizarion_marker.py
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
from automsgs.msgs.visualization_msgs import (
    interactive_marker_control_pb2 as imc_pb2,
    interactive_marker_init_pb2 as imi_pb2,
    interactive_marker_pb2 as im_pb2,
    interactive_marker_update_pb2 as imu_pb2,
    marker_array_pb2,
    marker_pb2,
)

M = marker_pb2.Marker


def stamp(h):
    now = time.time_ns()
    h.frame_id = 'map'
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def make_marker(mid, mtype, xyz, scale, rgb, ns='demo'):
    m = marker_pb2.Marker(ns=ns, id=mid, type=mtype, action=M.ADD)
    stamp(m.header)
    m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
    return m


def make_arrow(t):
    m = make_marker(0, M.ARROW, (math.cos(t), math.sin(t), 0.2),
                    (0.6, 0.1, 0.1), (1.0, 0.4, 0.1))
    yaw = t + math.pi / 2
    m.pose.orientation.z = math.sin(yaw * 0.5)
    m.pose.orientation.w = math.cos(yaw * 0.5)
    return m


def make_array():
    specs = (
        (1, M.CUBE, (-1.5, 0, 0.2), (0.3, 0.3, 0.3), (0.9, 0.3, 0.3)),
        (2, M.SPHERE, (-1.5, 1, 0.2), (0.3, 0.3, 0.3), (0.3, 0.9, 0.4)),
        (3, M.CYLINDER, (-1.5, -1, 0.25), (0.25, 0.25, 0.5), (0.4, 0.4, 0.95)),
    )
    arr = marker_array_pb2.MarkerArray()
    arr.markers.extend(make_marker(*s, ns='shapes') for s in specs)
    text = make_marker(4, M.TEXT_VIEW_FACING, (-1.5, 0, 0.6), (0, 0, 0.2),
                       (1, 1, 1), ns='shapes')
    text.text = 'MarkerArray'
    arr.markers.append(text)
    return arr


def make_interactive():
    im = im_pb2.InteractiveMarker(name='drag_me', description='MOVE_3D', scale=0.5)
    stamp(im.header)
    im.pose.position.x, im.pose.position.z = 1.5, 0.25
    im.pose.orientation.w = 1.0
    ctrl = im.controls.add()
    ctrl.name = 'move'
    ctrl.interaction_mode = imc_pb2.InteractiveMarkerControl.MOVE_3D
    ctrl.always_visible = True
    vis = ctrl.markers.add()
    vis.type = M.CUBE
    vis.scale.x = vis.scale.y = vis.scale.z = 0.25
    vis.color.r, vis.color.g, vis.color.b, vis.color.a = 1.0, 0.85, 0.2, 1.0
    vis.pose.orientation.w = 1.0
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    autolink.init('marker')
    n = autolink.Node('/autoviz/marker')
    w_m = n.create_writer('/fake/marker', marker_pb2.Marker, qos_depth=10)
    w_a = n.create_writer(
        '/fake/marker_array', marker_array_pb2.MarkerArray, qos_depth=10)
    w_i = n.create_writer('/fake/init', imi_pb2.InteractiveMarkerInit, qos_depth=1)
    w_u = n.create_writer(
        '/fake/update', imu_pb2.InteractiveMarkerUpdate, qos_depth=10)
    rate = autolink.Rate(args.rate)
    shapes, im = make_array(), make_interactive()
    print(f'marker @ {args.rate} Hz (Interact → drag yellow cube)')

    t0, seq = time.time(), 0
    try:
        while not autolink.is_shutdown():
            w_m.write(make_arrow(time.time() - t0))
            w_a.write(shapes)
            init = imi_pb2.InteractiveMarkerInit(server_id='tutorial', seq_num=seq)
            init.markers.append(im)
            w_i.write(init)
            upd = imu_pb2.InteractiveMarkerUpdate(
                server_id='tutorial', seq_num=seq,
                type=imu_pb2.InteractiveMarkerUpdate.UPDATE)
            upd.markers.append(im)
            w_u.write(upd)
            seq += 1
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
