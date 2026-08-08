#!/usr/bin/env python3
"""Publish diagnostic_msgs/DiagnosticArray for status inspection.

  /diagnostics  diagnostic_msgs/DiagnosticArray

View in Table: Channel=/diagnostics, Array path=status
(columns: level / name / message / hardware_id).

  /usr/bin/python3 examples/python/15_tutorial_diagnostics.py
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

DS = diagnostic_status_pb2.DiagnosticStatus


def stamp(h, frame=''):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def kv(st, key, value):
    item = st.values.add()
    item.key, item.value = key, str(value)


def add(msg, level, name, message, hw, pairs):
    st = msg.status.add()
    st.level, st.name, st.message, st.hardware_id = level, name, message, hw
    for k, v in pairs:
        kv(st, k, v)


def make_diagnostics(t):
    msg = diagnostic_array_pb2.DiagnosticArray()
    stamp(msg.header)

    batt = 12.6 + 0.4 * math.sin(t * 0.3)
    batt_lvl = DS.OK if batt > 12.0 else (DS.WARN if batt > 11.5 else DS.ERROR)
    add(msg, batt_lvl, 'power:battery', 
        'nominal' if batt_lvl == DS.OK else 'low voltage', 'batt0',
        [('voltage', f'{batt:.2f}'), ('current', f'{2.1 + 0.5 * math.sin(t):.2f}'),
         ('soc', f'{int(70 + 20 * math.sin(t * 0.2))}%')])

    cpu = 55 + 35 * (0.5 + 0.5 * math.sin(t * 0.8))
    cpu_lvl = DS.OK if cpu < 75 else (DS.WARN if cpu < 90 else DS.ERROR)
    add(msg, cpu_lvl, 'system:cpu',
        'ok' if cpu_lvl == DS.OK else 'thermal', 'cpu0',
        [('load', f'{cpu:.0f}%'), ('temp_c', f'{40 + cpu * 0.3:.0f}')])

    hz = 20 + 5 * math.sin(t)
    drop = max(0, int(3 * math.sin(t * 1.3)))
    lidar_lvl = DS.OK if drop == 0 else (DS.WARN if drop < 3 else DS.ERROR)
    add(msg, lidar_lvl, 'sensor:lidar',
        'streaming' if drop == 0 else f'dropout x{drop}', 'lidar0',
        [('rate_hz', f'{hz:.1f}'), ('points', int(18000 + 1000 * math.sin(t))),
         ('drop_frames', drop)])

    sats = 4 + int(3 * abs(math.sin(t * 0.5)))
    fix = sats >= 5
    gps_lvl = DS.OK if fix else DS.WARN
    add(msg, gps_lvl, 'sensor:gps',
        '3d fix' if fix else 'no fix', 'gps0',
        [('satellites', sats), ('hdop', f'{1.2 + 0.8 * (1 - fix):.2f}')])

    # Periodically go STALE to demo all four levels
    cam_lvl = DS.STALE if int(t) % 12 >= 9 else DS.OK
    add(msg, cam_lvl, 'sensor:camera',
        'no frame' if cam_lvl == DS.STALE else 'ok', 'cam0',
        [('fps', 0 if cam_lvl == DS.STALE else 30), ('exposure_ms', 8)])

    ctrl_ok = math.sin(t * 0.6) > -0.7
    add(msg, DS.OK if ctrl_ok else DS.ERROR, 'nav:controller',
        'tracking' if ctrl_ok else 'cmd_vel timeout', 'ctrl0',
        [('tracking_err_m', f'{0.03 + 0.2 * (not ctrl_ok):.3f}'),
         ('mode', 'AUTO' if ctrl_ok else 'HOLD')])

    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=2.0)
    args = ap.parse_args()

    autolink.init('diagnostics')
    n = autolink.Node('/autoviz/diagnostics')
    w = n.create_writer(
        '/diagnostics', diagnostic_array_pb2.DiagnosticArray, qos_depth=5)
    rate = autolink.Rate(args.rate)
    print(f'diagnostics @ {args.rate} Hz → /diagnostics')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            w.write(make_diagnostics(time.time() - t0))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
