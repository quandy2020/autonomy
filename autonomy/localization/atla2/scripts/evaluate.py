#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
# SPDX-License-Identifier: Apache-2.0
"""Evaluate Atla2 trajectories (ATE / RPE wrapper around evo when available)."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--est", required=True, help="estimated trajectory (TUM/KITTI)")
    p.add_argument("--gt", required=True, help="ground-truth trajectory")
    p.add_argument("--mode", choices=("ate", "rpe"), default="ate")
    args = p.parse_args()

    evo = shutil.which("evo_ape" if args.mode == "ate" else "evo_rpe")
    if not evo:
        print(
            "evo not found; install with: pip install evo\n"
            f"would run: evo_* tum {args.gt} {args.est} -a --plot",
            file=sys.stderr,
        )
        return 2

    cmd = [evo, "tum", args.gt, args.est, "-a", "--plot"]
    print(" ".join(cmd))
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
