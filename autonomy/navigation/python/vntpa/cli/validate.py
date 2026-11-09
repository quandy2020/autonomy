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

"""``python -m vntpa.cli.validate``: validate HDF5 token stores."""

from __future__ import annotations

import argparse
import json
from typing import Sequence

from vntpa.data.dataset import validate_root


def build_parser() -> argparse.ArgumentParser:
    """Build the dataset validation parser."""
    parser = argparse.ArgumentParser(
        prog="vntpa.cli.validate",
        description="Validate VNT-PA token stores",
    )
    parser.add_argument("--root", required=True, help="directory with .h5 stores")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Validate and return non-zero when any store is invalid."""
    args = build_parser().parse_args(argv)
    report = validate_root(args.root)
    print(json.dumps({"files": report.files, "errors": report.errors}, indent=2))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
