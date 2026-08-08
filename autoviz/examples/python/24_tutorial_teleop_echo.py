#!/usr/bin/env python3
"""Subscribe /cmd_vel to verify Autoviz Teleop panel.

Teleop panel publishes geometry_msgs/Twist on /cmd_vel (default).
This script prints received commands.

  Terminal A: /usr/bin/python3 examples/python/24_tutorial_teleop_echo.py
  Terminal B: ./build/autonomy/bin/autoviz  → open Teleop, drive
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.geometry_msgs import twist_pb2


def on_cmd(msg):
    print(f'cmd_vel  lin=({msg.linear.x:.3f},{msg.linear.y:.3f},{msg.linear.z:.3f})  '
          f'ang=({msg.angular.x:.3f},{msg.angular.y:.3f},{msg.angular.z:.3f})')


def main():
    autolink.init('teleop_echo')
    n = autolink.Node('/autoviz/teleop_echo')
    n.create_reader('/cmd_vel', on_cmd, twist_pb2.Twist)
    print('listening /cmd_vel — open Autoviz Teleop panel')
    try:
        while not autolink.is_shutdown():
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
