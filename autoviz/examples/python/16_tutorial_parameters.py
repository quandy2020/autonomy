#!/usr/bin/env python3
"""Host an Autolink ParameterServer for Autoviz Parameters panel.

  Node: /autoviz/params
  Params: max_speed (double), enable_estop (bool), waypoint_count (int),
          robot_name (string), goal_tol (double)

Parameters panel: select Node=/autoviz/params → view / edit values.

  /usr/bin/python3 examples/python/16_tutorial_parameters.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink

NODE = '/autoviz/params'

SEED = (
    autolink.Parameter('max_speed', 1.5),
    autolink.Parameter('goal_tol', 0.05),
    autolink.Parameter('enable_estop', False),
    autolink.Parameter('waypoint_count', 8),
    autolink.Parameter('robot_name', 'demo_bot'),
)


def main():
    autolink.init('parameters')
    n = autolink.Node(NODE)
    srv = autolink.ParameterServer(n)
    time.sleep(0.3)
    for p in SEED:
        srv.set_parameter(p)

    listed = srv.list_parameters()
    print(f'ParameterServer {NODE} ({len(listed)} params):')
    for p in listed:
        print(f'  {p.name()}: {p.debug_string()}')
    print('Open Autoviz Parameters → select this node → edit Value')

    try:
        while not autolink.is_shutdown():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
