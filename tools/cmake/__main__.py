#!/usr/bin/env python3
"""Run all unittest suites under tools.cmake.

Usage (from autonomy repo root):
  python3 -m tools.cmake
  python3 -m tools.cmake -v
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # Discover tests next to this package (test_*.py).
    start_dir = Path(__file__).resolve().parent
    suite = unittest.defaultTestLoader.discover(
        start_dir=str(start_dir),
        pattern="test_*.py",
        top_level_dir=str(start_dir.parents[1]),  # autonomy root for imports
    )
    runner = unittest.TextTestRunner(
        verbosity=2 if "-v" in argv or "--verbose" in argv else 1
    )
    result = runner.run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
