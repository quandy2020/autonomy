#!/usr/bin/env python3
"""Configure Autoviz as a standalone CMake project.

Usage:
    ./tools/configure.py
    ./tools/configure.py --release
    ./tools/configure.py --ogre --qml
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

from _bootstrap import ensure_tools_dir

ensure_tools_dir(__file__)

from common import autoviz_build_dir, find_autoviz_root, log_ok, log_step


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release", action="store_true", help="Release build type")
    parser.add_argument("--ogre", action="store_true", help="Enable AUTOVIZ_USE_OGRE")
    parser.add_argument("--qml", action="store_true", help="Enable AUTOVIZ_USE_QML_VEHICLE")
    parser.add_argument("--tests", action="store_true", help="Enable BUILD_AUTOVIZ_TESTS")
    parser.add_argument(
        "-G",
        "--generator",
        default=os.environ.get("CMAKE_GENERATOR", "Ninja"),
        help="CMake generator (default: Ninja)",
    )
    parser.add_argument("extra", nargs="*", help="Extra -D flags passed to CMake")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    autoviz_root = find_autoviz_root()
    build_dir = autoviz_build_dir(autoviz_root)

    cmake_args = [
        "cmake",
        "-S",
        str(autoviz_root),
        "-B",
        str(build_dir),
        "-G",
        args.generator,
        f"-DCMAKE_BUILD_TYPE={'Release' if args.release else 'Debug'}",
    ]
    if args.ogre:
        cmake_args.append("-DAUTOVIZ_USE_OGRE=ON")
    if args.qml:
        cmake_args.append("-DAUTOVIZ_USE_QML_VEHICLE=ON")
    if args.tests:
        cmake_args.append("-DBUILD_AUTOVIZ_TESTS=ON")
    cmake_args.extend(args.extra)

    log_step(f"Configure {build_dir}")
    subprocess.run(cmake_args, check=True)
    log_ok("Configure complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
