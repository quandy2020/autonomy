#!/usr/bin/env python3
"""Spin a mini Autolink topology for Channel Graph panel.

  Nodes:  /demo/sensor → /demo/perception → /demo/planner → /demo/controller
  + /demo/map_server  service /fake/graph/get_map

  Channels under /fake/graph/* (use Group=/fake/graph filter).

  /usr/bin/python3 examples/python/12_tutorial_channel_grpah.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.std_msgs import string_pb2


def noop(_msg):
    pass


def echo(req):
    return string_pb2.String(data=f'map:{req.data}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=5.0)
    args = ap.parse_args()

    autolink.init('channel_graph')

    sensor = autolink.Node('/demo/sensor')
    perception = autolink.Node('/demo/perception')
    planner = autolink.Node('/demo/planner')
    controller = autolink.Node('/demo/controller')
    map_server = autolink.Node('/demo/map_server')

    w_scan = sensor.create_writer('/fake/graph/scan', string_pb2.String, qos_depth=5)
    w_imu = sensor.create_writer('/fake/graph/imu', string_pb2.String, qos_depth=5)
    w_obj = perception.create_writer(
        '/fake/graph/objects', string_pb2.String, qos_depth=5)
    w_path = planner.create_writer('/fake/graph/path', string_pb2.String, qos_depth=5)
    w_cmd = controller.create_writer('/fake/graph/cmd', string_pb2.String, qos_depth=5)

    perception.create_reader('/fake/graph/scan', noop, string_pb2.String)
    perception.create_reader('/fake/graph/imu', noop, string_pb2.String)
    planner.create_reader('/fake/graph/objects', noop, string_pb2.String)
    controller.create_reader('/fake/graph/path', noop, string_pb2.String)

    map_server.create_service('/fake/graph/get_map', echo, req_type=string_pb2.String)
    client = planner.create_client('/fake/graph/get_map', req_type=string_pb2.String)

    rate = autolink.Rate(args.rate)
    print(f'channel graph @ {args.rate} Hz → /fake/graph/* (+ get_map service)')

    i = 0
    try:
        while not autolink.is_shutdown():
            w_scan.write(string_pb2.String(data=f'scan:{i}'))
            w_imu.write(string_pb2.String(data=f'imu:{i}'))
            w_obj.write(string_pb2.String(data=f'objects:{i}'))
            w_path.write(string_pb2.String(data=f'path:{i}'))
            w_cmd.write(string_pb2.String(data=f'cmd:{i}'))
            if i % 10 == 0:
                try:
                    client.send_request(string_pb2.String(data='floor1'), timeout_sec=0.5)
                except Exception:
                    pass
            i += 1
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
