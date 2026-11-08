"""python -m vntpa.cli.collect: delegate collection tasks to VNT-PA."""

from __future__ import annotations

import argparse
import subprocess
import sys


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("task", choices=["explore", "tokens", "bank"])
    parser.add_argument("trailing", nargs=argparse.REMAINDER)
    args, unknown = parser.parse_known_args(argv)
    command = [sys.executable, "-m", f"thirdparty.pan.collection.{args.task}",
               *(unknown + args.trailing)]
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
