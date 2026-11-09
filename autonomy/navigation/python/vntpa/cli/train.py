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

"""``python -m vntpa.cli.train``: VNT-PA training launcher.

The wrapper consumes product options (logs, checkpoint promotion, TensorBoard)
and forwards every remaining flag to the vendored trainer.
"""

from __future__ import annotations

import argparse
from typing import Sequence

from vntpa.engine.model import VNT
from vntpa.engine.trainer import (
    build_checkpoint_hook,
    build_jsonl_end_hook,
    build_jsonl_start_hook,
)


def build_parser() -> argparse.ArgumentParser:
    """Build the training parser."""
    parser = argparse.ArgumentParser(
        prog="vntpa.cli.train",
        description="Train VNT-PA via torchrun",
    )
    parser.add_argument("--nproc", type=int, default=1, help="torchrun processes")
    parser.add_argument("--log", default=None, help="JSONL lifecycle log")
    parser.add_argument(
        "--checkpoint-in",
        default="logs/pan/checkpoints/final.pt",
        help="checkpoint to promote after success",
    )
    parser.add_argument("--checkpoint-out", default=None, help="stable promoted checkpoint path")
    parser.add_argument("--manifest", default=None, help="optional promotion manifest JSON")
    parser.add_argument("--tensorboard-dir", default=None, help="TensorBoard run directory")
    parser.add_argument(
        "trailing", nargs=argparse.REMAINDER, help="flags forwarded to thirdparty.pan.planner.train"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Forward training flags and attach lifecycle callbacks."""
    args, unknown = build_parser().parse_known_args(argv)
    if args.tensorboard_dir:
        args.trailing.extend(["--tensorboard-dir", args.tensorboard_dir])
    trailing = unknown + args.trailing
    start_hooks = []
    end_hooks = []
    if args.log:
        start_hooks.append(build_jsonl_start_hook(args.log))
        end_hooks.append(build_jsonl_end_hook(args.log))
    if args.checkpoint_out:
        end_hooks.append(
            build_checkpoint_hook(args.checkpoint_in, args.checkpoint_out, args.manifest)
        )
    return VNT.train(
        trailing,
        nproc=args.nproc,
        start_hooks=start_hooks,
        end_hooks=end_hooks,
    )


if __name__ == "__main__":
    raise SystemExit(main())
