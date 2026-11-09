# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""``python -m vntpa.cli.collect``: offline collection task launcher.

Tasks delegate to the vendored VNT-PA collection modules:

    explore  frontier exploration -> tours
    tokens   tours -> frozen DeFM token stores
    bank     token stores -> evaluation bank
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from typing import Sequence

COLLECTION_TASKS = ("explore", "tokens", "bank", "synthetic")


def command(task: str, arguments: Sequence[str]) -> list[str]:
    """Return the module command for one collection stage."""
    if task not in COLLECTION_TASKS:
        raise ValueError(f"unknown collection task: {task}")
    if task == "synthetic":
        return [sys.executable, "-m", "vntpa.data.synthetic", *arguments]
    return [sys.executable, "-m", f"thirdparty.pan.collection.{task}", *arguments]


def main(argv: Sequence[str] | None = None) -> int:
    """Parse the stage and forward every remaining flag unchanged."""
    parser = argparse.ArgumentParser(
        prog="vntpa.cli.collect",
        description="Run VNT-PA data collection stages",
    )
    parser.add_argument("task", choices=COLLECTION_TASKS, help="collection stage")
    parser.add_argument(
        "trailing", nargs=argparse.REMAINDER, help="flags forwarded to the selected stage"
    )
    args, unknown = parser.parse_known_args(argv)
    trailing = unknown + args.trailing
    if trailing and trailing[0] == "--":
        trailing = trailing[1:]
    return subprocess.run(command(args.task, trailing), check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
