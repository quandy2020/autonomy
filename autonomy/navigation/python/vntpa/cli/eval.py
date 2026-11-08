"""python -m vntpa.cli.eval."""

from __future__ import annotations

import argparse

from ..runtime.inference.predictor import VNT


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trailing", nargs=argparse.REMAINDER)
    args, unknown = parser.parse_known_args(argv)
    return VNT.val(unknown + args.trailing)


if __name__ == "__main__":
    raise SystemExit(main())
