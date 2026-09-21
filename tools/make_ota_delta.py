#!/usr/bin/env python3
"""Build a file-level delta OTA package between two install-tree directories."""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def rel_files(root: Path) -> dict[str, Path]:
    out: dict[str, Path] = {}
    for p in root.rglob("*"):
        if p.is_file():
            out[str(p.relative_to(root))] = p
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", type=Path, required=True, help="Base install-tree")
    ap.add_argument("--target", type=Path, required=True, help="Target install-tree")
    ap.add_argument("--output", type=Path, required=True, help="Output package dir")
    ap.add_argument("--base-version", default="")
    ap.add_argument("--target-version", default="")
    args = ap.parse_args()

    base = rel_files(args.base)
    target = rel_files(args.target)
    out = args.output
    payloads = out / "delta" / "payloads"
    payloads.mkdir(parents=True, exist_ok=True)

    index = []
    deletes = []
    for rel, path in target.items():
        b = base.get(rel)
        if b is None or file_sha256(b) != file_sha256(path):
            dest = payloads / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, dest)
            index.append({"op": "replace" if b else "add", "path": rel, "sha256": file_sha256(path)})
    for rel in base:
        if rel not in target:
            deletes.append(rel)
            index.append({"op": "delete", "path": rel})

    (out / "delta" / "file_index.json").write_text(json.dumps(index, indent=2) + "\n")
    (out / "delta" / "deletes.txt").write_text("\n".join(deletes) + ("\n" if deletes else ""))
    (out / "delta" / "base_version").write_text(args.base_version + "\n")
    (out / "delta" / "target_version").write_text(args.target_version + "\n")
    manifest = {
        "schema_version": 1,
        "type": "delta",
        "base_version": args.base_version,
        "target_version": args.target_version,
    }
    (out / "package_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"delta package written: {out} files={len(index)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
