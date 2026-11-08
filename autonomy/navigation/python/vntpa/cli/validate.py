"""python -m vntpa.cli.validate: validate token stores."""

from __future__ import annotations

import argparse
import json

from ..data.dataset import validate_root


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    args = parser.parse_args()
    report = validate_root(args.root)
    print(json.dumps({"files": report.files, "errors": report.errors}, indent=2))
    raise SystemExit(0 if report.ok else 1)


if __name__ == "__main__":
    main()
