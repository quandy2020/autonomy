#!/usr/bin/env python3
"""Publish scalar env sensors + Float64 for Gauge / Indicator.

  /fake/illuminance        sensor_msgs/Illuminance
  /fake/fluid_pressure     sensor_msgs/FluidPressure
  /fake/relative_humidity  sensor_msgs/RelativeHumidity
  /fake/gauge              std_msgs/Float64  → field data

  /usr/bin/python3 examples/python/20_tutorial_scalar_env.py
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
from automsgs.msgs.sensor_msgs import (
    fluid_pressure_pb2,
    illuminance_pb2,
    relative_humidity_pb2,
)
from automsgs.msgs.std_msgs import float64_pb2


def stamp(h, frame='map'):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    autolink.init('scalar')
    n = autolink.Node('/autoviz/scalar')
    w_lux = n.create_writer(
        '/fake/illuminance', illuminance_pb2.Illuminance, qos_depth=5)
    w_p = n.create_writer(
        '/fake/fluid_pressure', fluid_pressure_pb2.FluidPressure, qos_depth=5)
    w_h = n.create_writer(
        '/fake/relative_humidity', relative_humidity_pb2.RelativeHumidity,
        qos_depth=5)
    w_g = n.create_writer('/fake/gauge', float64_pb2.Float64, qos_depth=10)
    rate = autolink.Rate(args.rate)
    print(f'scalar @ {args.rate} Hz → /fake/{{illuminance,fluid_pressure,relative_humidity,gauge}}')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            lux = illuminance_pb2.Illuminance()
            stamp(lux.header)
            lux.illuminance = 200 + 150 * (0.5 + 0.5 * math.sin(t * 0.4))
            lux.variance = 1.0
            w_lux.write(lux)

            press = fluid_pressure_pb2.FluidPressure()
            stamp(press.header)
            press.fluid_pressure = 101325 + 200 * math.sin(t * 0.2)
            press.variance = 10.0
            w_p.write(press)

            hum = relative_humidity_pb2.RelativeHumidity()
            stamp(hum.header)
            hum.relative_humidity = 0.45 + 0.2 * math.sin(t * 0.15)
            hum.variance = 0.01
            w_h.write(hum)

            w_g.write(float64_pb2.Float64(data=50 + 40 * math.sin(t)))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
