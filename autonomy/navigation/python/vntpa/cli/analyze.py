# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""``python -m vntpa.cli.analyze``: corpus / bank statistics for TensorBoard."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from typing import Sequence

from vntpa.data.analyze import analyze


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="vntpa.cli.analyze",
        description="Summarize VNT-PA token stores and frozen banks",
    )
    parser.add_argument("--tokens", default="", help="token or traj root with <scene>/seed*.h5")
    parser.add_argument("--bank", default="", help="frozen bank npz")
    parser.add_argument("--tensorboard-dir", default="", help="TensorBoard run directory")
    parser.add_argument("--out", default="", help="JSON report path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.tokens and not args.bank:
        raise SystemExit("need --tokens and/or --bank")
    report = analyze(
        args.tokens,
        args.bank,
        tensorboard_dir=args.tensorboard_dir or None,
        out=args.out or None,
    )
    print(json.dumps(asdict(report), indent=2))
    return 1 if report.errors and report.n_tours == 0 and report.bank is None else 0


if __name__ == "__main__":
    raise SystemExit(main())
