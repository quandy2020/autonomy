#!/usr/bin/env python3
"""Clean Autoviz build artifacts.

Examples:
    ./tools/clean.py
    ./tools/clean.py --dry-run
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

from _bootstrap import ensure_tools_dir

ensure_tools_dir(__file__)

from common import autoviz_build_dir, find_autoviz_root, log_info, log_ok, log_warn


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="Show actions only")
    return parser.parse_args()


def remove_path(path: Path, desc: str, *, dry_run: bool) -> None:
    if not path.exists() and not path.is_symlink():
        return
    if dry_run:
        log_info(f"Would remove: {desc}")
        return
    log_info(f"Removing: {desc}")
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path, ignore_errors=True)
    else:
        path.unlink(missing_ok=True)


def main() -> int:
    args = parse_args()
    autoviz_root = find_autoviz_root()
    if args.dry_run:
        log_warn("Dry run — nothing will be deleted")

    remove_path(autoviz_build_dir(autoviz_root), "Autoviz build directory", dry_run=args.dry_run)
    for cache_dir in autoviz_root.rglob("__pycache__"):
        remove_path(cache_dir, f"Python cache: {cache_dir.relative_to(autoviz_root)}", dry_run=args.dry_run)

    log_ok("Clean complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
