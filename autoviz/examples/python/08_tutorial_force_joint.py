#!/usr/bin/env python3
"""Publish Effort / Wrench / Temperature for Autoviz Displays.

  /robot_description + /joint_states (+ effort) + /tf  → Effort (+ RobotModel)
  /fake/wrench       geometry_msgs/WrenchStamped       → Wrench
  /fake/temperature  sensor_msgs/Temperature           → Temperature

  /usr/bin/python3 examples/python/08_tutorial_force_joint.py
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
from automsgs.msgs.geometry_msgs import wrench_stamped_pb2
from automsgs.msgs.sensor_msgs import joint_state_pb2, temperature_pb2
from automsgs.msgs.std_msgs import string_pb2
from automsgs.msgs.tf2_msgs import tf_message_pb2

URDF = Path(__file__).resolve().parent / 'urdf' / 'pr2_simple.urdf'
JOINTS = (
    'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint',
    'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_elbow_flex_joint',
    'l_wrist_flex_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint',
    'r_elbow_flex_joint', 'r_wrist_flex_joint',
)


def stamp(h, frame=''):
    now = time.time_ns()
    if frame:
        h.frame_id = frame
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
    msg.effort.extend([
        20 * math.sin(t * 0.5 + i * 0.4) for i in range(len(JOINTS))
    ])
    return msg


def make_tf(t):
    msg = tf_message_pb2.TFMessage()
    ts = msg.transforms.add()
    stamp(ts.header, 'map')
    ts.child_frame_id = 'base_link'
    ts.transform.translation.x = 0.5 * math.cos(t * 0.2)
    ts.transform.translation.y = 0.5 * math.sin(t * 0.2)
    ts.transform.rotation.w = 1.0
    return msg


def make_wrench(t):
    msg = wrench_stamped_pb2.WrenchStamped()
    stamp(msg.header, 'base_link')
    msg.wrench.force.x = 5.0 * math.cos(t)
    msg.wrench.force.z = 2.0 + math.sin(t * 1.5)
    msg.wrench.torque.y = 1.5 * math.sin(t * 0.8)
    msg.wrench.torque.z = 0.8 * math.cos(t * 0.6)
    return msg


def make_temp(t):
    msg = temperature_pb2.Temperature()
    stamp(msg.header, 'base_link')
    msg.temperature = 25.0 + 15.0 * (0.5 + 0.5 * math.sin(t * 0.4))
    msg.variance = 0.05
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    desc = string_pb2.String(data=URDF.read_text(encoding='utf-8'))
    autolink.init('force')
    n = autolink.Node('/autoviz/force')
    w_desc = n.create_writer('/robot_description', string_pb2.String, qos_depth=1)
    w_js = n.create_writer('/joint_states', joint_state_pb2.JointState, qos_depth=10)
    w_tf = n.create_writer('/tf', tf_message_pb2.TFMessage, qos_depth=10)
    w_w = n.create_writer(
        '/fake/wrench', wrench_stamped_pb2.WrenchStamped, qos_depth=10)
    w_t = n.create_writer(
        '/fake/temperature', temperature_pb2.Temperature, qos_depth=10)
    rate = autolink.Rate(args.rate)
    print(f'force @ {args.rate} Hz → joint_states(+effort) /fake/wrench /fake/temperature')

    t0 = time.time()
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            w_desc.write(desc)
            w_js.write(make_js(t))
            w_tf.write(make_tf(t))
            w_w.write(make_wrench(t))
            w_t.write(make_temp(t))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
