#!/usr/bin/env python3
# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""grpcurl wrapper for all automsgs/proto/rpcs services (single-file CLI)."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

PACKAGE_RE = re.compile(r"^\s*package\s+([\w.]+)\s*;", re.MULTILINE)
SERVICE_RE = re.compile(
    r"^\s*service\s+(\w+)\s*\{([^}]*)\}", re.MULTILINE | re.DOTALL
)
RPC_RE = re.compile(r"^\s*rpc\s+(\w+)\s*\(", re.MULTILINE)


# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Paths:
    """Repo layout derived from this script's location."""

    cli_dir: Path
    automsgs_root: Path
    proto_src: Path
    rpcs_src: Path
    default_include: Path

    @staticmethod
    def from_script(script_file: str | Path | None = None) -> Paths:
        cli_dir = Path(script_file or __file__).resolve().parent
        root = cli_dir.parents[1]
        proto = root / "proto"
        return Paths(
            cli_dir=cli_dir,
            automsgs_root=root,
            proto_src=proto,
            rpcs_src=proto / "rpcs",
            default_include=cli_dir / ".proto_include",
        )


@dataclass(frozen=True)
class RpcMethod:
    package: str
    service: str
    method: str
    proto_file: str

    @property
    def full_service(self) -> str:
        return f"{self.package}.{self.service}"

    @property
    def full_method(self) -> str:
        return f"{self.full_service}/{self.method}"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def die(msg: str, code: int = 1) -> None:
    print(msg, file=sys.stderr)
    raise SystemExit(code)


def default_target() -> str:
    return os.environ.get("AUTOMSGS_RPC_TARGET", "localhost:50051")


def find_grpcurl() -> str:
    path = shutil.which("grpcurl")
    if not path:
        die(
            "grpcurl not found in PATH.\n"
            "Install: https://github.com/fullstorydev/grpcurl#installation"
        )
    return path


def load_request_data(data: str) -> str:
    """Return JSON text; `@path` loads from file."""
    if not data.startswith("@"):
        return data
    path = Path(data[1:])
    if not path.is_file():
        die(f"data file not found: {path}")
    return path.read_text(encoding="utf-8")


def run_cmd(cmd: list[str], *, verbose: bool = False) -> int:
    if verbose:
        print("+", " ".join(cmd), file=sys.stderr)
    try:
        return subprocess.run(cmd, check=False).returncode
    except FileNotFoundError:
        die(f"failed to execute: {cmd[0]}")


# ---------------------------------------------------------------------------
# Proto include layout (matches CMake proto_include/automsgs/...)
# ---------------------------------------------------------------------------


def link_or_replace(dst: Path, src: Path) -> None:
    if dst.is_symlink() and dst.resolve() == src.resolve():
        return
    if dst.is_symlink() or dst.is_file():
        dst.unlink()
    elif dst.is_dir():
        shutil.rmtree(dst)
    dst.symlink_to(src, target_is_directory=True)


def ensure_proto_include(paths: Paths, include_dir: Path) -> Path:
    env = os.environ.get("AUTOMSGS_PROTO_INCLUDE")
    if env:
        root = Path(env).resolve()
        if not (root / "automsgs" / "rpcs").is_dir():
            die(f"AUTOMSGS_PROTO_INCLUDE missing automsgs/rpcs: {root}")
        return root

    if not paths.proto_src.is_dir():
        die(f"proto source not found: {paths.proto_src}")

    root = include_dir.resolve()
    am = root / "automsgs"
    am.mkdir(parents=True, exist_ok=True)
    for name in ("msgs", "srvs", "rpcs", "actions"):
        src = paths.proto_src / name
        if src.is_dir():
            link_or_replace(am / name, src)
    return root


def rpc_proto_flags(include_dir: Path) -> list[str]:
    rpcs = include_dir / "automsgs" / "rpcs"
    protos = sorted(rpcs.glob("*.proto"))
    if not protos:
        die(f"no rpcs protos under {rpcs}")
    flags: list[str] = []
    for proto in protos:
        flags += ["-proto", proto.relative_to(include_dir).as_posix()]
    return flags


def grpcurl_base(
    grpcurl: str, include_dir: Path, *, plaintext: bool
) -> list[str]:
    cmd = [grpcurl, "-import-path", str(include_dir), *rpc_proto_flags(include_dir)]
    if plaintext:
        cmd.append("-plaintext")
    return cmd


# ---------------------------------------------------------------------------
# Catalog
# ---------------------------------------------------------------------------


def parse_proto_methods(proto: Path) -> list[RpcMethod]:
    text = re.sub(r"//.*?$", "", proto.read_text(encoding="utf-8"), flags=re.M)
    pkg = PACKAGE_RE.search(text)
    if not pkg:
        return []
    package = pkg.group(1)
    out: list[RpcMethod] = []
    for svc in SERVICE_RE.finditer(text):
        service, body = svc.group(1), svc.group(2)
        for rpc in RPC_RE.finditer(body):
            out.append(
                RpcMethod(package, service, rpc.group(1), proto.name)
            )
    return out


def scan_catalog(rpcs_dir: Path) -> list[RpcMethod]:
    if not rpcs_dir.is_dir():
        die(f"rpcs dir not found: {rpcs_dir}")
    catalog = [
        m for p in sorted(rpcs_dir.glob("*.proto")) for m in parse_proto_methods(p)
    ]
    if not catalog:
        die(f"no service methods found under {rpcs_dir}")
    return catalog


def filter_service(catalog: list[RpcMethod], symbol: str) -> list[RpcMethod]:
    matched = [
        m
        for m in catalog
        if m.full_service == symbol
        or m.service == symbol
        or m.full_service.endswith("." + symbol)
    ]
    if not matched:
        die(f"unknown service: {symbol}")
    return matched


def resolve_method(raw: str, catalog: list[RpcMethod]) -> str:
    """Resolve full | Service/Method | unique Method → package.Service/Method."""
    raw = raw.strip()
    tip = f"Tip: run `{Path(sys.argv[0]).name} list`"

    def pick(matches: list[RpcMethod], label: str) -> str:
        if len(matches) == 1:
            return matches[0].full_method
        if not matches:
            die(f"unknown method: {label}\n{tip}")
        names = "\n  ".join(m.full_method for m in matches)
        die(f"ambiguous method {label!r}, matches:\n  {names}")

    if any(m.full_method == raw for m in catalog):
        return raw
    if "/" in raw:
        svc, method = raw.split("/", 1)
        return pick(
            [
                m
                for m in catalog
                if m.method == method
                and (m.service == svc or m.full_service == svc)
            ],
            raw,
        )
    return pick([m for m in catalog if m.method == raw], raw)


def print_catalog(catalog: list[RpcMethod]) -> None:
    current = ""
    for m in catalog:
        if m.full_service != current:
            current = m.full_service
            print(f"{current}  ({m.proto_file})")
        print(f"  {m.full_method}")
    n_svc = len({m.full_service for m in catalog})
    print(f"\n# {n_svc} services, {len(catalog)} methods")


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------


def cmd_list(args: argparse.Namespace, paths: Paths) -> int:
    catalog = scan_catalog(paths.rpcs_src)
    if args.symbol:
        for m in filter_service(catalog, args.symbol):
            print(m.full_method)
        return 0

    print_catalog(catalog)
    if not args.grpcurl:
        return 0

    include = ensure_proto_include(paths, Path(args.import_path))
    cmd = grpcurl_base(find_grpcurl(), include, plaintext=args.plaintext)
    cmd.append("list")
    print("\n# grpcurl list:", file=sys.stderr)
    return run_cmd(cmd, verbose=args.verbose)


def cmd_describe(args: argparse.Namespace, paths: Paths) -> int:
    symbol = args.symbol
    if "/" in symbol:
        symbol = resolve_method(symbol, scan_catalog(paths.rpcs_src))

    include = ensure_proto_include(paths, Path(args.import_path))
    cmd = grpcurl_base(find_grpcurl(), include, plaintext=args.plaintext)
    cmd.extend(["describe", symbol])
    return run_cmd(cmd, verbose=args.verbose)


def cmd_call(args: argparse.Namespace, paths: Paths) -> int:
    method = resolve_method(args.method, scan_catalog(paths.rpcs_src))
    include = ensure_proto_include(paths, Path(args.import_path))
    cmd = grpcurl_base(find_grpcurl(), include, plaintext=args.plaintext)
    cmd.extend(["-d", load_request_data(args.data)])
    if args.max_time is not None:
        cmd.extend(["-max-time", str(args.max_time)])
    cmd.extend([args.target, method])
    return run_cmd(cmd, verbose=args.verbose)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser(paths: Paths) -> argparse.ArgumentParser:
    target = default_target()
    p = argparse.ArgumentParser(
        prog="rpc-cli.py",
        description=(
            "Call all automsgs.rpcs.* via grpcurl "
            "(Navigation/Mapping/Localization/Charge/Follow/Sensor/System/Teleop/Exploration)."
        ),
    )
    p.add_argument("-t", "--target", default=target, help=f"host:port (default {target})")
    p.add_argument(
        "--import-path",
        default=str(paths.default_include),
        help="grpcurl -import-path root",
    )
    p.add_argument("--tls", action="store_true", help="use TLS (default: plaintext)")
    p.add_argument("-v", "--verbose", action="store_true", help="print grpcurl command")

    sub = p.add_subparsers(dest="command", required=True)

    list_p = sub.add_parser("list", help="list services/methods from local protos")
    list_p.add_argument("symbol", nargs="?", help="optional service filter")
    list_p.add_argument("--grpcurl", action="store_true", help="also run grpcurl list")
    list_p.set_defaults(func=cmd_list)

    desc_p = sub.add_parser("describe", help="describe a symbol via grpcurl")
    desc_p.add_argument("symbol", help="service, method, or message type")
    desc_p.set_defaults(func=cmd_describe)

    call_p = sub.add_parser("call", help="invoke an RPC method")
    call_p.add_argument("method", help="package.Service/Method | Service/Method | Method")
    call_p.add_argument("-d", "--data", default="{}", help='JSON or @file (default "{}")')
    call_p.add_argument("--max-time", type=float, default=None, help="grpcurl -max-time")
    call_p.set_defaults(func=cmd_call)

    return p


def main(argv: list[str] | None = None) -> int:
    paths = Paths.from_script()
    args = build_parser(paths).parse_args(argv)
    args.plaintext = not args.tls
    return args.func(args, paths)


if __name__ == "__main__":
    raise SystemExit(main())
