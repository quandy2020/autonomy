#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
"""Run Beni teleop action sequences through the ONNX policy (offline).

Examples::

  # Offline policy smoke (no Isaac)
  python3 src/autonomy/autosim/scripts/beni_action_sequences.py

  # Live: publish /cmd_vel into a running autosim (autolink)
  python3 src/autonomy/autosim/scripts/beni_action_sequences.py --live

  # One sequence only
  python3 src/autonomy/autosim/scripts/beni_action_sequences.py --seq forward
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autosim.robot.policy.beni_sequences import BENI_SEQUENCES, SequenceStep  # noqa: E402

Twist = Tuple[float, float]


def run_offline(sequences: dict[str, List[SequenceStep]]) -> int:
    from autosim.robot.policy.driver import PolicyDriver

    onnx = ROOT / "robot" / "beni" / "models" / "policy.onnx"
    if not onnx.is_file():
        print(f"MISSING {onnx}", flush=True)
        return 2
    driver = PolicyDriver.create_when_enabled(
        {
            "policy": {
                "enabled": True,
                "action_space": "beni",
                "model_path": str(onnx),
                "rate_hz": 100.0,
                "drive_from_wheels": False,
            }
        }
    )
    if driver is None or driver.beni is None:
        print("FAILED to load Beni policy", flush=True)
        return 1

    failures = 0
    kwargs = dict(pose=(0.0, 0.0, 0.0), velocity=(0.0, 0.0), scan=None)
    for name, steps in sequences.items():
        driver.beni.reset()
        driver._hold_cmd = (0.0, 0.0)
        driver.last_command = (0.0, 0.0)
        print(f"\n=== {name} ===", flush=True)
        ok = True
        for label, vx, wz, dur in steps:
            steps_n = max(1, int(round(dur / 0.01)))
            last = (0.0, 0.0)
            for i in range(steps_n):
                twist: Optional[Twist] = (vx, wz) if i == 0 else None
                cmd = driver.compute_command(0.01, twist_cmd=twist, **kwargs)
                assert cmd is not None
                last = (float(cmd[0]), float(cmd[1]))
            status = "ok"
            if abs(vx) < 1e-9 and abs(wz) < 1e-9:
                if last != (0.0, 0.0):
                    status = f"FAIL stop→{last}"
                    ok = False
            elif abs(last[0] - vx) > 0.08 and abs(last[1] - wz) > 0.08:
                # Still ramping is fine for short segments; flag only if near-zero.
                if abs(last[0]) < 1e-3 and abs(last[1]) < 1e-3:
                    status = f"FAIL no-motion cmd=({vx},{wz}) out={last}"
                    ok = False
            print(
                f"  {label:12s} cmd=({vx:+.2f},{wz:+.2f}) "
                f"plant=({last[0]:+.3f},{last[1]:+.3f})  {status}",
                flush=True,
            )
        # Latch silence after final stop.
        for _ in range(10):
            cmd = driver.compute_command(0.01, twist_cmd=None, **kwargs)
            if cmd != (0.0, 0.0) and abs(driver._hold_cmd[0]) < 1e-9:
                print(f"  FAIL latch revived {cmd}", flush=True)
                ok = False
                break
        print(f"  → {'PASS' if ok else 'FAIL'}", flush=True)
        failures += 0 if ok else 1
    print(f"\n{len(sequences) - failures}/{len(sequences)} sequences passed", flush=True)
    return 0 if failures == 0 else 1


def run_live(sequences: dict[str, List[SequenceStep]], *, channel: str, rate_hz: float) -> int:
    """Publish TwistStamped sequences on autolink for a running autosim."""
    try:
        from autolink import Node
    except ImportError:
        print("autolink not available; use offline mode", flush=True)
        return 2
    from autosim.bridge.messages import Messages
    from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped

    node = Node("beni_action_seq")
    writer = node.create_writer(channel, TwistStamped, qos_depth=10)
    dt = 1.0 / max(rate_hz, 1.0)
    print(f"live publish → {channel} @ {rate_hz:.0f} Hz", flush=True)
    try:
        for name, steps in sequences.items():
            print(f"\n=== live:{name} ===", flush=True)
            for label, vx, wz, dur in steps:
                print(f"  {label}: ({vx:+.2f},{wz:+.2f}) × {dur:.2f}s", flush=True)
                end = time.time() + dur
                while time.time() < end:
                    stamp = (int(time.time()), int((time.time() % 1) * 1e9))
                    writer.write(
                        Messages.encode_twist_stamped(vx, wz, stamp, "base_link")
                    )
                    time.sleep(dt)
            # Hard stop + settle
            for _ in range(max(3, int(0.3 / dt))):
                stamp = (int(time.time()), int((time.time() % 1) * 1e9))
                writer.write(Messages.encode_twist_stamped(0.0, 0.0, stamp, "base_link"))
                time.sleep(dt)
            time.sleep(0.2)
    finally:
        stamp = (int(time.time()), 0)
        writer.write(Messages.encode_twist_stamped(0.0, 0.0, stamp, "base_link"))
        node.shutdown()
    print("\nlive sequences done (stopped)", flush=True)
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seq",
        action="append",
        choices=sorted(BENI_SEQUENCES.keys()),
        help="Run only these sequences (repeatable). Default: all.",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Publish /cmd_vel to a running autosim instead of offline ONNX.",
    )
    parser.add_argument("--channel", default="/cmd_vel")
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args(argv)
    selected = {
        k: BENI_SEQUENCES[k]
        for k in (args.seq or sorted(BENI_SEQUENCES.keys()))
    }
    if args.live:
        return run_live(selected, channel=args.channel, rate_hz=args.rate)
    return run_offline(selected)


if __name__ == "__main__":
    raise SystemExit(main())
