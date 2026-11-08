"""python -m vntpa.cli.train."""

from __future__ import annotations

import argparse

from ..runtime.inference.predictor import VNT
from ..training.trainer import CallbackList, CheckpointCallback, JsonlCallback


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nproc", type=int, default=1)
    parser.add_argument("--log")
    parser.add_argument("--checkpoint-in", default="logs/pan/checkpoints/final.pt")
    parser.add_argument("--checkpoint-out")
    parser.add_argument("--manifest")
    parser.add_argument("--tensorboard-dir", default=None)
    parser.add_argument("trailing", nargs=argparse.REMAINDER)
    args, unknown = parser.parse_known_args(argv)
    if args.tensorboard_dir:
        args.trailing.extend(["--tensorboard-dir", args.tensorboard_dir])
    args.trailing = unknown + args.trailing
    callbacks = []
    if args.log:
        callbacks.append(JsonlCallback(args.log))
    if args.checkpoint_out:
        callbacks.append(CheckpointCallback(
            args.checkpoint_in, args.checkpoint_out, args.manifest))
    return VNT.train(args.trailing, nproc=args.nproc,
                     callbacks=CallbackList(callbacks) if callbacks else None)


if __name__ == "__main__":
    raise SystemExit(main())
