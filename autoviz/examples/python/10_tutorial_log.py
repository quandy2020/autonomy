#!/usr/bin/env python3
"""Emit foxglove.Log for Autoviz Log panel (level / name / search).

  /fake/log  foxglove.Log (JSON payload)

Log panel: Topic=/fake/log. Capture glog = Autoviz process only.

  /usr/bin/python3 examples/python/10_tutorial_log.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import json
import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink

# foxglove.Log Level enum
DEBUG, INFO, WARN, ERROR = 1, 2, 3, 4

ENTRIES = (
    (DEBUG, 'sensor', 'imu sample ok'),
    (INFO, 'planner', 'path length=12.4 m'),
    (INFO, 'controller', 'tracking error=0.03 m'),
    (WARN, 'sensor', 'lidar dropout 2 frames'),
    (WARN, 'planner', 'goal near obstacle'),
    (ERROR, 'controller', 'cmd_vel timeout'),
    (ERROR, 'autolink', 'channel /cmd_vel no reader'),
)


def make_log(level: int, name: str, message: str, line: int) -> bytes:
    now = time.time_ns()
    return json.dumps(
        {
            'timestamp': {'sec': now // 1_000_000_000, 'nsec': now % 1_000_000_000},
            'level': level,
            'message': message,
            'name': name,
            'file': '10_tutorial_log.py',
            'line': line,
        },
        separators=(',', ':'),
    ).encode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=2.0)
    args = ap.parse_args()

    autolink.init('log')
    n = autolink.Node('/autoviz/log')
    w = n.create_writer('/fake/log', 'foxglove.Log', qos_depth=20)
    rate = autolink.Rate(args.rate)
    print(f'log @ {args.rate} Hz → /fake/log (foxglove.Log JSON)')

    i = 0
    try:
        while not autolink.is_shutdown():
            level, name, msg = ENTRIES[i % len(ENTRIES)]
            w.write(make_log(level, name, f'{msg} #{i}', 40 + (i % 7)))
            i += 1
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
