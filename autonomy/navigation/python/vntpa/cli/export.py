"""python -m vntpa.cli.export."""

from __future__ import annotations

import argparse

from ..runtime.inference.predictor import VNT


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--context-size", type=int, default=64)
    parser.add_argument("--format", default="onnx", choices=["onnx"])
    args = parser.parse_args(argv)
    return VNT.export(args.checkpoint, args.out,
                      context_size=args.context_size, format=args.format)


if __name__ == "__main__":
    raise SystemExit(main())
