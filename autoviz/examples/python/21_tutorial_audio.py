#!/usr/bin/env python3
"""Publish foxglove.RawAudio (pcm-s16) for Autoviz Audio panel.

  /fake/audio  foxglove.RawAudio JSON

  /usr/bin/python3 examples/python/21_tutorial_audio.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import argparse
import base64
import json
import math
import struct
import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink

SR = 16000
CHUNK = 1600  # 100 ms @ 16 kHz
FREQ = 440.0


def make_pcm(phase0, n, freq=FREQ):
    """Return (pcm_bytes, next_phase)."""
    buf = bytearray()
    phase = phase0
    step = 2 * math.pi * freq / SR
    for _ in range(n):
        s = int(max(-32767, min(32767, 8000 * math.sin(phase))))
        buf += struct.pack('<h', s)
        phase += step
    return bytes(buf), phase


def make_msg(pcm, phase_unused=None):
    del phase_unused
    now = time.time_ns()
    return json.dumps(
        {
            'timestamp': {'sec': now // 1_000_000_000, 'nsec': now % 1_000_000_000},
            'format': 'pcm-s16',
            'sample_rate': SR,
            'number_of_channels': 1,
            'data': base64.b64encode(pcm).decode('ascii'),
        },
        separators=(',', ':'),
    ).encode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    autolink.init('audio')
    n = autolink.Node('/autoviz/audio')
    w = n.create_writer('/fake/audio', 'foxglove.RawAudio', qos_depth=10)
    rate = autolink.Rate(args.rate)
    print(f'audio @ {args.rate} Hz → /fake/audio (pcm-s16 {SR} Hz)')

    phase = 0.0
    try:
        while not autolink.is_shutdown():
            pcm, phase = make_pcm(phase, CHUNK)
            w.write(make_msg(pcm))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
