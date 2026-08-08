#!/usr/bin/env python3
"""Publish vision_msgs (+ Image) for Autoviz Image annotations / Table.

  /fake/vision/image         sensor_msgs/Image
  /fake/vision/detections2d  vision_msgs/Detection2DArray  → Image annotations
  /fake/vision/boxes2d       vision_msgs/BoundingBox2DArray
  /fake/vision/detections3d  vision_msgs/Detection3DArray  → Table / Raw Messages

  /usr/bin/python3 examples/python/17_tutorial_visions.py
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
from automsgs.msgs.sensor_msgs import image_pb2
from automsgs.msgs.vision_msgs import (
    bounding_box2d_array_pb2,
    detection2d_array_pb2,
    detection3d_array_pb2,
)

W, H = 320, 240
CLASSES = ('person', 'car', 'bike', 'dog')


def stamp(h, frame='camera'):
    now = time.time_ns()
    h.frame_id = frame
    h.stamp.sec = now // 1_000_000_000
    h.stamp.nanosec = now % 1_000_000_000


def make_image(phase):
    msg = image_pb2.Image()
    stamp(msg.header)
    msg.height, msg.width, msg.encoding, msg.step = H, W, 'rgb8', W * 3
    buf = bytearray(H * W * 3)
    for y in range(H):
        for x in range(W):
            i = (y * W + x) * 3
            buf[i] = (40 + (x + phase) // 2) % 256
            buf[i + 1] = (60 + y // 2) % 256
            buf[i + 2] = 90
    msg.data = bytes(buf)
    return msg


def set_box2d(box, cx, cy, sx, sy, theta=0.0):
    box.center.position.x = cx
    box.center.position.y = cy
    box.center.theta = theta
    box.size_x, box.size_y = sx, sy


def add_hyp(det, class_id, score):
    r = det.results.add()
    r.hypothesis.class_id = class_id
    r.hypothesis.score = score


def make_detections2d(t):
    msg = detection2d_array_pb2.Detection2DArray()
    stamp(msg.header)
    specs = (
        (80 + 40 * math.sin(t), 70, 60, 100, 'person', 0.92),
        (200 + 30 * math.cos(t * 0.7), 140, 90, 50, 'car', 0.81),
        (160, 50 + 20 * math.sin(t * 1.3), 40, 40, 'bike', 0.66),
    )
    for i, (cx, cy, sx, sy, cls, score) in enumerate(specs):
        det = msg.detections.add()
        stamp(det.header)
        det.id = f'{cls}_{i}'
        set_box2d(det.bbox, cx, cy, sx, sy, 0.1 * math.sin(t + i))
        add_hyp(det, cls, score)
    return msg


def make_boxes2d(t):
    msg = bounding_box2d_array_pb2.BoundingBox2DArray()
    stamp(msg.header)
    for i in range(3):
        a = t + i * 2.1
        set_box2d(msg.boxes.add(),
                  60 + 80 * i + 10 * math.sin(a),
                  180 + 8 * math.cos(a),
                  50, 30, 0.05 * i)
    return msg


def make_detections3d(t):
    msg = detection3d_array_pb2.Detection3DArray()
    stamp(msg.header, 'map')
    for i, cls in enumerate(CLASSES[:3]):
        a = t * 0.5 + i * 2.0
        det = msg.detections.add()
        stamp(det.header, 'map')
        det.id = f'{cls}_3d_{i}'
        det.bbox.center.position.x = 2.0 * math.cos(a)
        det.bbox.center.position.y = 2.0 * math.sin(a)
        det.bbox.center.position.z = 0.5 + 0.2 * i
        det.bbox.center.orientation.w = 1.0
        det.bbox.size.x = 0.6 + 0.2 * i
        det.bbox.size.y = 0.4
        det.bbox.size.z = 0.5 + 0.3 * (cls == 'person')
        add_hyp(det, cls, 0.7 + 0.1 * i)
        # Optional pose on hypothesis (PoseWithCovariance → PoseStamped)
        pose = det.results[0].pose.pose
        stamp(pose.header, 'map')
        pose.pose.CopyFrom(det.bbox.center)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    autolink.init('visions')
    n = autolink.Node('/autoviz/visions')
    w_img = n.create_writer('/fake/vision/image', image_pb2.Image, qos_depth=2)
    w_d2 = n.create_writer(
        '/fake/vision/detections2d', detection2d_array_pb2.Detection2DArray,
        qos_depth=5)
    w_b2 = n.create_writer(
        '/fake/vision/boxes2d', bounding_box2d_array_pb2.BoundingBox2DArray,
        qos_depth=5)
    w_d3 = n.create_writer(
        '/fake/vision/detections3d', detection3d_array_pb2.Detection3DArray,
        qos_depth=5)
    rate = autolink.Rate(args.rate)
    print(f'visions @ {args.rate} Hz → /fake/vision/{{image,detections2d,boxes2d,detections3d}}')

    t0 = time.time()
    phase = 0
    try:
        while not autolink.is_shutdown():
            t = time.time() - t0
            w_img.write(make_image(phase))
            w_d2.write(make_detections2d(t))
            w_b2.write(make_boxes2d(t))
            w_d3.write(make_detections3d(t))
            phase = (phase + 2) % 256
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
