#!/usr/bin/env python3
"""Build Autoviz target.

Usage:
    ./tools/build.py
    ./tools/build.py --jobs 8
    ./tools/build.py --test

Environment:
    AUTOVIZ_BUILD_DIR  Override the CMake build tree (default: <package>/build).
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

from _bootstrap import ensure_tools_dir

ensure_tools_dir(__file__)

from common import autoviz_build_dir, find_autoviz_root, log_ok, log_step

TEST_TARGETS = ("autoviz_interaction_test", "autoviz_mosaic_restore_test")
TEST_FILTER = r"^autoviz_(interaction|mosaic_restore)_test$"


def _resolve_build_dir(root: Path) -> Path:
    """Honor AUTOVIZ_BUILD_DIR so colcon trees work without reconfiguring."""
    override = os.environ.get("AUTOVIZ_BUILD_DIR")
    return Path(override).resolve() if override else autoviz_build_dir(root)


def _cmake_version() -> tuple[int, int, int]:
    out = subprocess.run(
        ["cmake", "--version"], check=True, capture_output=True, text=True
    )
    found = re.search(r"(\d+)\.(\d+)\.(\d+)", out.stdout)
    return tuple(int(part) for part in found.groups()) if found else (0, 0, 0)


def _cache_value(cache_text: str, key: str) -> str | None:
    match = re.search(rf"^{re.escape(key)}:\w+=(.*)$", cache_text, re.MULTILINE)
    return match.group(1).strip() if match else None


def _enable_tests(root: Path, build_dir: Path) -> bool:
    """Ensure the build tree has AUTOVIZ_BUILD_CORE_TESTS=ON."""
    cache = build_dir / "CMakeCache.txt"
    text = cache.read_text(errors="ignore") if cache.exists() else ""
    if _cache_value(text, "AUTOVIZ_BUILD_CORE_TESTS") == "ON":
        return True

    home = _cache_value(text, "CMAKE_HOME_DIRECTORY")
    if home and Path(home).resolve() != root.resolve():
        print(
            "This build tree belongs to another CMake project "
            f"({home}).\n"
            "Enable tests when configuring it, e.g.\n"
            "  colcon build --packages-up-to autonomy --cmake-args "
            "-DAUTOVIZ_BUILD_CORE_TESTS=ON",
            file=sys.stderr,
        )
        return False

    log_step("Enabling AUTOVIZ_BUILD_CORE_TESTS")
    subprocess.run(
        [
            "cmake",
            "-S",
            str(root),
            "-B",
            str(build_dir),
            "-DAUTOVIZ_BUILD_CORE_TESTS=ON",
        ],
        check=True,
    )
    return True


def _run_tests(build_dir: Path) -> None:
    result_dir = build_dir / "autoviz-test-results"
    result_dir.mkdir(parents=True, exist_ok=True)
    junit = result_dir / "junit.xml"
    junit.unlink(missing_ok=True)

    command = [
        "ctest",
        "--test-dir",
        str(build_dir),
        "-R",
        TEST_FILTER,
        "--output-on-failure",
    ]
    if _cmake_version() >= (3, 21, 0):
        command += ["--output-junit", str(junit)]
    else:
        log_step("ctest --output-junit needs CMake >= 3.21; JUnit file skipped")

    log_step("Running Autoviz regression tests")
    subprocess.run(command, check=True)
    if junit.exists():
        log_ok(f"Tests passed · {junit}")
    else:
        log_ok("Tests passed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=max(1, (os.cpu_count() or 2) // 2),
        help="Parallel build jobs",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Build and run the core / headless-UI regression tests",
    )
    args = parser.parse_args()

    root = find_autoviz_root()
    build_dir = _resolve_build_dir(root)
    if not build_dir.is_dir():
        print("Build directory missing; run tools/configure.py first.", file=sys.stderr)
        return 1

    if args.test and not _enable_tests(root, build_dir):
        return 1

    targets = ["autoviz_app"]
    if args.test:
        targets.extend(TEST_TARGETS)

    log_step(f"Building {' + '.join(targets)} ({args.jobs} jobs)")
    subprocess.run(
        [
            "cmake",
            "--build",
            str(build_dir),
            "--target",
            *targets,
            "-j",
            str(args.jobs),
        ],
        check=True,
    )
    log_ok("Build complete")

    if args.test:
        _run_tests(build_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())

