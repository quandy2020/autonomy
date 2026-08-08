#!/usr/bin/env python3
"""Publish discrete states for Autoviz State Transitions panel.

  /fake/state/nav   std_msgs/String  → field data
  /fake/state/ctrl  std_msgs/String  → field data
  /fake/state/mode  std_msgs/String  → field data

Add series: channel + Message path `data`.

  /usr/bin/python3 examples/python/11_tutorial_state_transitions.py
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

# (hold_ticks, state) — each step holds for hold_ticks at --rate
NAV = (
    (4, 'IDLE'),
    (3, 'PLANNING'),
    (6, 'NAVIGATING'),
    (2, 'RECOVERY'),
    (5, 'NAVIGATING'),
    (3, 'IDLE'),
)
CTRL = (
    (5, 'IDLE'),
    (8, 'TRACKING'),
    (3, 'HOLD'),
    (6, 'TRACKING'),
    (2, 'IDLE'),
)
MODE = (
    (10, 'MANUAL'),
    (12, 'AUTO'),
    (4, 'ESTOP'),
    (8, 'AUTO'),
)


def advance(seq, i, hold):
    """Return (state, next_i, next_hold). `hold` includes the current tick."""
    state = seq[i][1]
    hold -= 1
    if hold <= 0:
        i = (i + 1) % len(seq)
        hold = seq[i][0]
    return state, i, hold


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=2.0)
    args = ap.parse_args()

    autolink.init('state')
    n = autolink.Node('/autoviz/state')
    w_nav = n.create_writer('/fake/state/nav', string_pb2.String, qos_depth=10)
    w_ctrl = n.create_writer('/fake/state/ctrl', string_pb2.String, qos_depth=10)
    w_mode = n.create_writer('/fake/state/mode', string_pb2.String, qos_depth=10)
    rate = autolink.Rate(args.rate)
    print(f'state @ {args.rate} Hz → /fake/state/{{nav,ctrl,mode}}')

    ni = ci = mi = 0
    nh, ch, mh = NAV[0][0], CTRL[0][0], MODE[0][0]
    try:
        while not autolink.is_shutdown():
            nav, ni, nh = advance(NAV, ni, nh)
            ctrl, ci, ch = advance(CTRL, ci, ch)
            mode, mi, mh = advance(MODE, mi, mh)
            w_nav.write(string_pb2.String(data=nav))
            w_ctrl.write(string_pb2.String(data=ctrl))
            w_mode.write(string_pb2.String(data=mode))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
