#!/usr/bin/env python3
"""Verify Bridge RPC documentation matches automsgs/proto/rpcs/*.proto.

Checks:
  1. Every rpc in automsgs/proto/rpcs/*.proto appears in
     rpcs/02_service_overview.md (service name or method name)
  2. No stale AutonomyService / external_command_service references in
     15_Bridge docs (except explicit "已移除" notes)
  3. No stale Recovery references

Usage (from repo root):
  python3 docs/scripts/check_bridge_rpc_docs.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTO_DIR = REPO_ROOT / "automsgs/proto/rpcs"
RPC_INDEX = REPO_ROOT / "docs/source/15_Bridge/rpcs/02_service_overview.md"
HANDLERS = REPO_ROOT / "docs/source/15_Bridge/grpc/07_handlers.md"
BRIDGE_DOCS = REPO_ROOT / "docs/source/15_Bridge"

RPC_PROTO_RE = re.compile(r"^\s*rpc\s+(\w+)", re.MULTILINE)
SERVICE_PROTO_RE = re.compile(r"^\s*service\s+(\w+)", re.MULTILINE)
STALE_RECOVERY_RE = re.compile(
    r"SendRecoveryCommand|TASK_TYPE_RECOVERY|ROBOT_TASK_RECOVERY|supports_recovery|cmd/recovery",
    re.IGNORECASE,
)
# Allowed only when documenting removal.
STALE_AUTONOMY_RE = re.compile(
    r"external_command_service\.proto|"
    r"autonomy\.bridge\.proto\.AutonomyService|"
    r"(?<![已移除 `/`])AutonomyService(?!\s*/\s*`external_command)",
)


def extract_proto_rpcs(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    return RPC_PROTO_RE.findall(text)


def extract_proto_services(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    return SERVICE_PROTO_RE.findall(text)


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


def scan_stale_autonomy_refs(root: Path) -> list[str]:
    """Flag dual-surface leftovers; allow explicit removal notes."""
    hits: list[str] = []
    allow = re.compile(r"已移除|不再|历史|removed", re.IGNORECASE)
    for path in sorted(root.rglob("*")):
        if path.suffix not in {".md", ".rst"}:
            continue
        for i, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if "external_command_service" in line or "AutonomyService" in line:
                if allow.search(line):
                    continue
                # index / overview may mention removal in same sentence
                if "已移除" in line or "removed" in line.lower():
                    continue
                rel = path.relative_to(REPO_ROOT)
                hits.append(f"{rel}:{i}: {line.strip()[:100]}")
    return hits


def main() -> int:
    errors: list[str] = []

    if not PROTO_DIR.is_dir():
        errors.append(f"Missing proto dir: {PROTO_DIR}")
    if not RPC_INDEX.is_file():
        errors.append(f"Missing index doc: {RPC_INDEX}")
    if not HANDLERS.is_file():
        errors.append(f"Missing handlers doc: {HANDLERS}")
    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    proto_files = sorted(PROTO_DIR.glob("*.proto"))
    services: list[str] = []
    methods: list[str] = []
    for p in proto_files:
        services.extend(extract_proto_services(p))
        methods.extend(extract_proto_rpcs(p))

    index_text = RPC_INDEX.read_text(encoding="utf-8")
    handlers_text = HANDLERS.read_text(encoding="utf-8")
    combined = index_text + "\n" + handlers_text

    missing_services = [s for s in services if s not in combined]
    if missing_services:
        errors.append(
            f"Services in proto but not mentioned in 02/07 docs: "
            f"{sorted(set(missing_services))}"
        )

    # Method names may collide across services (Pause/Cancel/GetStatus); require
    # that each unique method appears at least once in overview or handlers.
    unique_methods = sorted(set(methods))
    missing_methods = [m for m in unique_methods if m not in combined]
    if missing_methods:
        errors.append(
            f"RPC methods in proto but not mentioned in 02/07 docs: "
            f"{missing_methods}"
        )

    stale = scan_stale_recovery_refs(BRIDGE_DOCS)
    if stale:
        errors.append(f"Stale Recovery references in 15_Bridge ({len(stale)}):")
        errors.extend(f"  {h}" for h in stale[:5])
        if len(stale) > 5:
            errors.append(f"  ... and {len(stale) - 5} more")

    autonomy_stale = scan_stale_autonomy_refs(BRIDGE_DOCS)
    if autonomy_stale:
        errors.append(
            f"Stale AutonomyService / external_command_service refs "
            f"({len(autonomy_stale)}):"
        )
        errors.extend(f"  {h}" for h in autonomy_stale[:12])
        if len(autonomy_stale) > 12:
            errors.append(f"  ... and {len(autonomy_stale) - 12} more")

    if errors:
        print("Bridge RPC docs check FAILED:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    print(
        f"OK: {len(services)} services / {len(unique_methods)} unique RPCs "
        f"aligned with docs; no stale AutonomyService dual-surface refs"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
