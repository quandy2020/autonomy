#!/usr/bin/env python3
"""package_ota_delta.py — build a file-level OTA delta package for OtaAgent.

Compares two install-tree roots (base → target) and writes:
  package_manifest.json
  delta/file_index.json
  delta/payloads/…   (changed/added files)
  delta/deletes.txt  (paths present in base but missing in target)
  full.tar.gz        (optional; for delta_fallback_to_full)

Usage:
  python3 tools/package_ota_delta.py \\
    --base /opt/autonomy-slots/a \\
    --target /opt/autonomy-slots/b \\
    --base-version v1 --target-version v2 \\
    --output dist/ota-delta-v1-v2

  # Also attach a full tarball for fallback:
  python3 tools/package_ota_delta.py ... --full-tar dist/autonomy-v2.tar.gz
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def iter_files(root: Path) -> dict[str, Path]:
    out: dict[str, Path] = {}
    if not root.is_dir():
        return out
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            full = Path(dirpath) / name
            if full.is_symlink():
                continue
            if not full.is_file():
                continue
            rel = full.relative_to(root).as_posix()
            out[rel] = full
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, type=Path, help="Base install root")
    parser.add_argument(
        "--target", required=True, type=Path, help="Target install root"
    )
    parser.add_argument("--base-version", required=True)
    parser.add_argument("--target-version", required=True)
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Output package directory",
    )
    parser.add_argument(
        "--full-tar",
        type=Path,
        default=None,
        help="Optional full.tar.gz to copy into package for fallback",
    )
    args = parser.parse_args(argv)

    base_files = iter_files(args.base.resolve())
    target_files = iter_files(args.target.resolve())

    replaces: list[dict[str, str]] = []
    adds: list[dict[str, str]] = []
    deletes: list[str] = []

    out = args.output.resolve()
    payloads = out / "delta" / "payloads"
    if out.exists():
        shutil.rmtree(out)
    payloads.mkdir(parents=True)

    for rel, tpath in sorted(target_files.items()):
        bpath = base_files.get(rel)
        if bpath is None:
            op = "add"
        elif file_sha256(bpath) != file_sha256(tpath):
            op = "replace"
        else:
            continue
        dest = payloads / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(tpath, dest)
        entry = {"op": op, "path": rel, "sha256": file_sha256(tpath)}
        if op == "add":
            adds.append(entry)
        else:
            replaces.append(entry)

    for rel in sorted(set(base_files) - set(target_files)):
        deletes.append(rel)

    index = {
        "schema_version": 1,
        "base_version": args.base_version,
        "target_version": args.target_version,
        "replace": replaces,
        "add": adds,
        "delete": deletes,
        "built_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    (out / "delta" / "file_index.json").write_text(
        json.dumps(index, indent=2) + "\n", encoding="utf-8"
    )
    (out / "delta" / "deletes.txt").write_text(
        "\n".join(deletes) + ("\n" if deletes else ""), encoding="utf-8"
    )

    manifest = {
        "type": "delta",
        "base_version": args.base_version,
        "target_version": args.target_version,
        "schema_version": 1,
        "built_at": index["built_at"],
        "stats": {
            "replace": len(replaces),
            "add": len(adds),
            "delete": len(deletes),
        },
    }
    (out / "package_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )

    if args.full_tar is not None:
        src = args.full_tar.resolve()
        if not src.is_file():
            print(f"full-tar missing: {src}", file=sys.stderr)
            return 1
        shutil.copy2(src, out / "full.tar.gz")

    print(
        f"[ota-delta] wrote {out} "
        f"replace={len(replaces)} add={len(adds)} delete={len(deletes)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
