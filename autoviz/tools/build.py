#!/usr/bin/env python3
"""Build Autoviz target.

Usage:
    ./tools/build.py
    ./tools/build.py --jobs 8
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

from _bootstrap import ensure_tools_dir

ensure_tools_dir(__file__)

from common import autoviz_build_dir, find_autoviz_root, log_ok, log_step


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=max(1, (os.cpu_count() or 2) // 2),
        help="Parallel build jobs",
    )
    args = parser.parse_args()

    build_dir = autoviz_build_dir(find_autoviz_root())
    if not build_dir.is_dir():
        print("Build directory missing; run tools/configure.py first.", file=sys.stderr)
        return 1

    log_step(f"Building autoviz ({args.jobs} jobs)")
    subprocess.run(
        ["cmake", "--build", str(build_dir), "--target", "autoviz", "-j", str(args.jobs)],
        check=True,
    )
    log_ok("Build complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
