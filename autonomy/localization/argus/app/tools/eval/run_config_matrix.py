#!/usr/bin/env python3
"""Run one voxel_slam binary across a set of YAML configs.

Each config is launched as:

    <binary> --config <file> --dataset <dataset> --output <run_dir>

The run directory is <output>/<config stem>/. A non-zero exit is recorded in
status.txt. The process exits 1 if any run failed.
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path


def collect_configs(paths: list[Path]) -> list[Path]:
    configs = []
    for path in paths:
        if path.is_dir():
            configs.extend(sorted(path.glob("*.yaml")))
            configs.extend(sorted(path.glob("*.yml")))
        else:
            configs.append(path)
    unique = []
    seen = set()
    for config in configs:
        resolved = config.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if config.suffix not in {".yaml", ".yml"}:
            raise SystemExit(f"config must be YAML: {config}")
        unique.append(config)
    if not unique:
        raise SystemExit("no YAML configs")
    return unique


def command(binary: Path, config: Path, dataset: Path, run_dir: Path) -> list[str]:
    return [
        str(binary),
        "--config",
        str(config),
        "--dataset",
        str(dataset),
        "--output",
        str(run_dir),
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--config", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--timeout", type=float, default=None)
    args = parser.parse_args(argv)

    configs = collect_configs(args.config)
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.dry_run and not args.binary.is_file():
        raise SystemExit(f"binary not found: {args.binary}")
    if not args.dataset.exists():
        raise SystemExit(f"dataset not found: {args.dataset}")

    failures = 0
    for config in configs:
        run_dir = args.output / config.stem
        run_dir.mkdir(parents=True, exist_ok=True)
        argv_run = command(args.binary, config, args.dataset, run_dir)
        (run_dir / "command.txt").write_text(shlex.join(argv_run) + "\n")
        if args.dry_run:
            print(shlex.join(argv_run))
            (run_dir / "status.txt").write_text("dry-run\n")
            continue
        print(shlex.join(argv_run), flush=True)
        completed = subprocess.run(argv_run, cwd=run_dir, timeout=args.timeout, check=False)
        (run_dir / "status.txt").write_text(f"{completed.returncode}\n")
        if completed.returncode != 0:
            failures += 1
    if failures:
        print(f"{failures} run(s) failed", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
