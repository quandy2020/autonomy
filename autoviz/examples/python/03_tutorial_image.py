#!/usr/bin/env python3
"""Publish rgb8 Image on /fake/image.

  /usr/bin/python3 examples/python/03_tutorial_image.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.sensor_msgs import image_pb2

W, H = 320, 240


def make_image(phase=0):
    msg = image_pb2.Image()
    now = time.time_ns()
    msg.header.frame_id = 'camera'
    msg.header.stamp.sec = now // 1_000_000_000
    msg.header.stamp.nanosec = now % 1_000_000_000
    msg.height, msg.width, msg.encoding, msg.step = H, W, 'rgb8', W * 3
    buf = bytearray(H * W * 3)
    for y in range(H):
        for x in range(W):
            i = (y * W + x) * 3
            buf[i] = (x + phase) % 256
            buf[i + 1] = (y + phase) % 256
            buf[i + 2] = (x + y + phase) % 256
    msg.data = bytes(buf)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--static', action='store_true')
    args = ap.parse_args()

    autolink.init('image')
    n = autolink.Node('/autoviz/image')
    w = n.create_writer('/fake/image', image_pb2.Image, qos_depth=2)
    rate = autolink.Rate(args.rate)
    print(f'image @ {args.rate} Hz → /fake/image ({W}x{H} rgb8)')
    phase = 0
    try:
        while not autolink.is_shutdown():
            w.write(make_image(0 if args.static else phase))
            phase = (phase + 3) % 256
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
