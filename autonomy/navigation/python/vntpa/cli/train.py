# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""``python -m vntpa.cli.train``: VNT-PA imitation training.

Modes:

* ``--synthetic`` — write a Habitat-free corpus and train the frozen-bank loop.
* default with ``--token-root`` + ``--train-bank`` — same loop on real/frozen banks.
* ``--distributed`` — paper trainer (NCCL, Habitat TourDataset, 30k steps).
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path
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
        description="Train VNT-PA (frozen-bank loop or paper distributed trainer)",
    )
    parser.add_argument("--nproc", type=int, default=1, help="torch.distributed processes")
    parser.add_argument("--log", default=None, help="JSONL lifecycle log")
    parser.add_argument(
        "--checkpoint-in",
        default="logs/pan/checkpoints/final.pt",
        help="checkpoint to promote after success",
    )
    parser.add_argument("--checkpoint-out", default=None, help="stable promoted checkpoint path")
    parser.add_argument("--manifest", default=None, help="optional promotion manifest JSON")
    parser.add_argument(
        "--tensorboard-dir",
        default=None,
        help="TensorBoard directory; default logs/<run-name>/tb. Empty string disables.",
    )
    parser.add_argument("--synthetic", action="store_true", help="write and train on a synthetic corpus")
    parser.add_argument("--distributed", action="store_true", help="use the paper NCCL trainer")
    parser.add_argument("--tiny", action="store_true", help="small planner for CPU smoke tests")
    parser.add_argument("--full", action="store_true", help="paper-size planner (74M)")
    parser.add_argument("--data-root", default="dataset/synthetic", help="synthetic corpus root")
    parser.add_argument("--token-root", default="", help="training token stores")
    parser.add_argument("--train-bank", default="", help="frozen training bank npz")
    parser.add_argument("--val-token-root", default="", help="validation token stores")
    parser.add_argument("--val-bank", default="", help="frozen validation bank npz")
    parser.add_argument("--run-name", default="vntpa", help="logs/<run-name>/")
    parser.add_argument("--steps", type=int, default=None, help="optimizer steps (paper default 30000)")
    parser.add_argument("--batch-scenes", type=int, default=None)
    parser.add_argument("--accum", type=int, default=None)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument(
        "trailing", nargs=argparse.REMAINDER, help="extra flags for the paper trainer"
    )
    return parser


def _strip_remainder(trailing: Sequence[str]) -> list[str]:
    items = list(trailing)
    if items and items[0] == "--":
        items = items[1:]
    return items


def _local(args: argparse.Namespace) -> int:
    from vntpa.data.synthetic import write_corpus
    from vntpa.engine.loop import LoopConfig, run

    token_root = args.token_root
    train_bank = args.train_bank
    val_token_root = args.val_token_root
    val_bank = args.val_bank
    tiny = True if args.tiny else False
    if args.synthetic and not args.full and not args.tiny:
        tiny = True
    if args.full:
        tiny = False
    steps = args.steps if args.steps is not None else (20 if tiny else 30000)
    if args.synthetic:
        paths = write_corpus(args.data_root, tours=4 if tiny else 8, frames=16 if tiny else 32)
        token_root = paths["tokens_train"]
        train_bank = paths["bank_train"]
        val_token_root = paths["tokens_val"]
        val_bank = paths["bank_val"]
        print(f"wrote synthetic corpus under {args.data_root}", flush=True)
    if not token_root or not train_bank:
        raise SystemExit(
            "need --token-root and --train-bank, or pass --synthetic to generate them"
        )
    final = run(
        LoopConfig(
            token_root=token_root,
            train_bank=train_bank,
            val_token_root=val_token_root or token_root,
            val_bank=val_bank or train_bank,
            run_name=args.run_name,
            steps=steps,
            batch_scenes=args.batch_scenes or (2 if tiny else 4),
            accum=args.accum or 1,
            tiny=tiny,
            device=args.device,
            workers=args.workers,
            log_every=max(1, min(50, steps)),
            val_every=max(1, min(100, steps)),
            tensorboard_dir=args.tensorboard_dir,
        )
    )
    if args.checkpoint_out:
        destination = Path(args.checkpoint_out)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(final, destination)
    print(f"checkpoint {final}", flush=True)
    return 0


def _distributed(args: argparse.Namespace, trailing: list[str]) -> int:
    if args.tensorboard_dir:
        trailing.extend(["--tensorboard-dir", args.tensorboard_dir])
    if not any(item == "--steps" or item.startswith("--steps=") for item in trailing):
        trailing.extend(["--steps", str(args.steps if args.steps is not None else 30000)])
    if args.run_name and "--run-name" not in trailing:
        trailing.extend(["--run-name", args.run_name])
    if args.token_root and "--token-root" not in trailing:
        trailing.extend(["--token-root", args.token_root])
    if args.val_token_root and "--val-token-root" not in trailing:
        trailing.extend(["--val-token-root", args.val_token_root])
    if args.val_bank and "--val-bank" not in trailing:
        trailing.extend(["--val-bank", args.val_bank])
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


def main(argv: Sequence[str] | None = None) -> int:
    """Dispatch synthetic/local bank training or the paper distributed trainer."""
    args, unknown = build_parser().parse_known_args(argv)
    trailing = _strip_remainder(unknown + args.trailing)
    if args.distributed:
        return _distributed(args, trailing)
    return _local(args)


if __name__ == "__main__":
    raise SystemExit(main())
