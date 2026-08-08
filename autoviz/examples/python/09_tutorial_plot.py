#!/usr/bin/env python3
"""Publish numeric series for Autoviz Plot panel.

  /fake/plot/sine   std_msgs/Float64   → field data
  /fake/plot/cosine std_msgs/Float64   → field data
  /fake/plot/cmd    geometry_msgs/Twist → linear.x / angular.z

In Autoviz Plot: Add series, channel + field_path as above.

  /usr/bin/python3 examples/python/09_tutorial_plot.py
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
from automsgs.msgs.geometry_msgs import twist_pb2
from automsgs.msgs.std_msgs import float64_pb2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=50.0)
    args = ap.parse_args()

    autolink.init('plot')
    n = autolink.Node('/autoviz/plot')
    w_sin = n.create_writer('/fake/plot/sine', float64_pb2.Float64, qos_depth=10)
    w_cos = n.create_writer('/fake/plot/cosine', float64_pb2.Float64, qos_depth=10)
    w_cmd = n.create_writer('/fake/plot/cmd', twist_pb2.Twist, qos_depth=10)
    rate = autolink.Rate(args.rate)
    print(f'plot @ {args.rate} Hz → /fake/plot/{{sine,cosine,cmd}}')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            w_sin.write(float64_pb2.Float64(data=math.sin(t)))
            w_cos.write(float64_pb2.Float64(data=math.cos(t * 1.5)))
            cmd = twist_pb2.Twist()
            cmd.linear.x = 0.5 + 0.5 * math.sin(t * 0.7)
            cmd.angular.z = 0.3 * math.cos(t)
            w_cmd.write(cmd)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
