#!/usr/bin/env python3
"""Publish NavSatFix track for Autoviz Map panel.

  /gps/fix  sensor_msgs/NavSatFix  (circular track around Shanghai)

Map panel: add topic layer /gps/fix, Follow channel=/gps/fix.

  /usr/bin/python3 examples/python/23_tutorial_map_gps.py
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
from automsgs.msgs.sensor_msgs import nav_sat_fix_pb2, nav_sat_status_pb2

# Shanghai approx
LAT0, LON0 = 31.2304, 121.4737
# ~111m per deg lat; lon scale by cos(lat)
D_LAT, D_LON = 0.001, 0.001 / math.cos(math.radians(LAT0))


def make_fix(t):
    msg = nav_sat_fix_pb2.NavSatFix()
    now = time.time_ns()
    msg.header.frame_id = 'gps'
    msg.header.stamp.sec = now // 1_000_000_000
    msg.header.stamp.nanosec = now % 1_000_000_000
    msg.status.status = nav_sat_status_pb2.NavSatStatus.STATUS_FIX
    msg.status.service = nav_sat_status_pb2.NavSatStatus.SERVICE_GPS
    a = t * 0.15
    msg.latitude = LAT0 + D_LAT * math.sin(a)
    msg.longitude = LON0 + D_LON * math.cos(a)
    msg.altitude = 12.0 + math.sin(a)
    msg.position_covariance.extend([4.0, 0, 0, 0, 4.0, 0, 0, 0, 16.0])
    msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=5.0)
    args = ap.parse_args()

    autolink.init('map_gps')
    n = autolink.Node('/autoviz/map_gps')
    w = n.create_writer('/gps/fix', nav_sat_fix_pb2.NavSatFix, qos_depth=5)
    rate = autolink.Rate(args.rate)
    print(f'map_gps @ {args.rate} Hz → /gps/fix')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            w.write(make_fix(time.time() - t0))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
