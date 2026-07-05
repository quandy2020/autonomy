#!/usr/bin/env python3
"""Verify Bridge RPC documentation matches external_command_service.proto.

Checks:
  1. RPC count in proto == rows in rpcs/02_service_overview.md §2.1 table
  2. RPC names match between proto and doc index
  3. Handler table in grpc/07_handlers.md covers the same RPC names

Usage (from repo root):
  python3 docs/scripts/check_bridge_rpc_docs.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTO = REPO_ROOT / "autonomy/bridge/proto/external_command_service.proto"
RPC_INDEX = REPO_ROOT / "docs/source/15_Bridge/rpcs/02_service_overview.md"
HANDLERS = REPO_ROOT / "docs/source/15_Bridge/grpc/07_handlers.md"

RPC_PROTO_RE = re.compile(r"^\s*rpc\s+(\w+)", re.MULTILINE)
RPC_DOC_RE = re.compile(r"`(\w+)`\s*\|\s*(?:Stream|Query|System|Command)")
HANDLER_RPC_RE = re.compile(
    r"^\|\s*`((?:Send|Receive|Get|Emergency|Cancel)\w+)`\s*\|",
    re.MULTILINE,
)
STALE_RECOVERY_RE = re.compile(
    r"SendRecoveryCommand|TASK_TYPE_RECOVERY|ROBOT_TASK_RECOVERY|supports_recovery|cmd/recovery",
    re.IGNORECASE,
)
BRIDGE_DOCS = REPO_ROOT / "docs/source/15_Bridge"


def extract_proto_rpcs(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    return RPC_PROTO_RE.findall(text)


def extract_index_rpcs(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    # Limit to §2.1 table (before ## 2.2)
    section = text.split("## 2.2")[0]
    return RPC_DOC_RE.findall(section)


def extract_handler_rpcs(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    section = text.split("## 7.3")[0]
    return HANDLER_RPC_RE.findall(section)


def scan_stale_recovery_refs(root: Path) -> list[str]:
    hits: list[str] = []
    for path in sorted(root.rglob("*")):
        if path.suffix not in {".md", ".rst"}:
            continue
        for i, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if STALE_RECOVERY_RE.search(line):
                rel = path.relative_to(REPO_ROOT)
                hits.append(f"{rel}:{i}: {line.strip()[:80]}")
    return hits


def main() -> int:
    errors: list[str] = []

    if not PROTO.is_file():
        errors.append(f"Missing proto: {PROTO}")
    if not RPC_INDEX.is_file():
        errors.append(f"Missing index doc: {RPC_INDEX}")
    if not HANDLERS.is_file():
        errors.append(f"Missing handlers doc: {HANDLERS}")
    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    proto_rpcs = extract_proto_rpcs(PROTO)
    index_rpcs = extract_index_rpcs(RPC_INDEX)
    handler_rpcs = extract_handler_rpcs(HANDLERS)

    if len(proto_rpcs) != len(index_rpcs):
        errors.append(
            f"Index table has {len(index_rpcs)} RPCs, proto has {len(proto_rpcs)}"
        )

    proto_set = set(proto_rpcs)
    index_set = set(index_rpcs)
    if proto_set != index_set:
        missing_in_doc = proto_set - index_set
        extra_in_doc = index_set - proto_set
        if missing_in_doc:
            errors.append(f"In proto but not in 02 §2.1: {sorted(missing_in_doc)}")
        if extra_in_doc:
            errors.append(f"In 02 §2.1 but not in proto: {sorted(extra_in_doc)}")

    handler_set = set(handler_rpcs)
    if proto_set != handler_set:
        missing_in_handlers = proto_set - handler_set
        extra_in_handlers = handler_set - proto_set
        if missing_in_handlers:
            errors.append(
                f"In proto but not in grpc/07_handlers: {sorted(missing_in_handlers)}"
            )
        if extra_in_handlers:
            errors.append(
                f"In grpc/07_handlers but not in proto: {sorted(extra_in_handlers)}"
            )

    stale = scan_stale_recovery_refs(BRIDGE_DOCS)
    if stale:
        errors.append(f"Stale Recovery references in 15_Bridge ({len(stale)}):")
        errors.extend(f"  {h}" for h in stale[:5])
        if len(stale) > 5:
            errors.append(f"  ... and {len(stale) - 5} more")

    if errors:
        print("Bridge RPC docs check FAILED:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    print(
        f"OK: {len(proto_rpcs)} RPCs aligned "
        f"(proto, 02_service_overview §2.1, grpc/07_handlers)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
