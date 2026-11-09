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

"""``python -m vntpa.cli.export``: checkpoint -> ONNX."""

from __future__ import annotations

import argparse
from typing import Sequence

from vntpa.engine.model import VNT


def build_parser() -> argparse.ArgumentParser:
    """Build the export parser."""
    parser = argparse.ArgumentParser(
        prog="vntpa.cli.export",
        description="Export a VNT-PA checkpoint to ONNX",
    )
    parser.add_argument("--checkpoint", required=True, help="VNT-PA .pt checkpoint")
    parser.add_argument("--out", required=True, help="output ONNX path")
    parser.add_argument("--context-size", type=int, default=64, help="fixed keyframe context K")
    parser.add_argument(
        "--format", default="onnx", choices=["onnx"], help="export format (ONNX only)"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Export and write the metadata sidecar."""
    args = build_parser().parse_args(argv)
    return VNT.export(args.checkpoint, args.out, context_size=args.context_size, format=args.format)


if __name__ == "__main__":
    raise SystemExit(main())
