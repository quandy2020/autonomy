#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Deprecated: use ``download_humanoids.py`` (assets ship under autosim).

Kept as a thin wrapper so older docs/commands still work.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def main() -> int:
    print(
        "note: link_trackvla_humanoids.py is deprecated; "
        "downloading into autosim/data/humanoids/ (no TrackVLA path needed).",
        file=sys.stderr,
    )
    target = Path(__file__).resolve().parent / "download_humanoids.py"
    # Forward argv but drop this script name; strip leftover TrackVLA path args.
    forwarded = [str(target)]
    for arg in sys.argv[1:]:
        if arg.startswith("-"):
            forwarded.append(arg)
        elif Path(arg).is_file() and arg.endswith(".zip"):
            forwarded.extend(["--zip", arg])
        # Ignore bare TrackVLA roots — assets are no longer linked from there.
    sys.argv = forwarded
    runpy.run_path(str(target), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
