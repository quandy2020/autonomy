#!/usr/bin/env python3
"""Publish PR2 URDF + JointState + TF for RobotModel.

  /robot_description  /joint_states  /tf

  /usr/bin/python3 examples/python/06_tutorial_urdf.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import math
import time
from pathlib import Path

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.sensor_msgs import joint_state_pb2
from automsgs.msgs.std_msgs import string_pb2
from automsgs.msgs.tf2_msgs import tf_message_pb2

URDF = Path(__file__).resolve().parent / 'urdf' / 'pr2_simple.urdf'
JOINTS = (
    'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint',
    'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_elbow_flex_joint',
    'l_wrist_flex_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint',
    'r_elbow_flex_joint', 'r_wrist_flex_joint',
)


def stamp(h):
    now = time.time_ns()
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def make_js(t):
    msg = joint_state_pb2.JointState()
    stamp(msg.header)
    msg.name.extend(JOINTS)
    msg.position.extend([
        0.15 + 0.1 * math.sin(t * 0.5),
        0.6 * math.sin(t * 0.4),
        0.2 * math.sin(t * 0.3),
        0.5 + 0.4 * math.sin(t * 0.5),
        0.4 + 0.3 * math.sin(t * 0.6),
        -0.8 + 0.4 * math.sin(t * 0.7),
        0.3 * math.sin(t),
        -0.5 + 0.4 * math.sin(t * 0.5 + 1),
        0.4 + 0.3 * math.sin(t * 0.6 + 0.5),
        -0.8 + 0.4 * math.sin(t * 0.7 + 0.5),
        0.3 * math.sin(t + 0.5),
    ])
    return msg


def make_tf(t):
    msg = tf_message_pb2.TFMessage()
    ts = msg.transforms.add()
    stamp(ts.header)
    ts.header.frame_id = 'map'
    ts.child_frame_id = 'base_link'
    ts.transform.translation.x = 0.5 * math.cos(t * 0.2)
    ts.transform.translation.y = 0.5 * math.sin(t * 0.2)
    ts.transform.rotation.w = 1.0
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    desc = string_pb2.String(data=URDF.read_text(encoding='utf-8'))
    autolink.init('urdf')
    n = autolink.Node('/autoviz/urdf')
    w_desc = n.create_writer('/robot_description', string_pb2.String, qos_depth=1)
    w_js = n.create_writer('/joint_states', joint_state_pb2.JointState, qos_depth=10)
    w_tf = n.create_writer('/tf', tf_message_pb2.TFMessage, qos_depth=10)
    rate = autolink.Rate(args.rate)
    print(f'urdf @ {args.rate} Hz → /robot_description /joint_states /tf')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            w_desc.write(desc)
            w_js.write(make_js(t))
            w_tf.write(make_tf(t))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
