#!/usr/bin/env python3
"""
Install dependencies for autonomy workspace.

This script mirrors dependencies declared in:
docker/dockerfile/autonomy.x86_64.dockerfile
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List


APT_PACKAGES: List[str] = [
    "sudo",
    "software-properties-common",
    "pkg-config",
    "autoconf",
    "automake",
    "cmake",
    "curl",
    "git",
    "unzip",
    "vim",
    "wget",
    "bc",
    "gdb",
    "libsdl2-dev",
    "libblas-dev",
    "liblapack-dev",
    "libtinyxml2-dev",
    "liblua5.3-dev",
    "ninja-build",
    "sphinx",
    "python3-pip",
    "python3-dev",
    "python3-sphinx",
    "uuid-dev",
    "libcivetweb-dev",
    "libsuitesparse-dev",
    "lsb-release",
    "libompl-dev",
    "libcairo2-dev",
    "libboost-all-dev",
    "libasio-dev",
    "libncurses5-dev",
    "libavcodec-dev",
    "libswscale-dev",
    "libpoco-dev",
    "libflann-dev",
    "libqhull-dev",
    "libpcap0.8",
    "libpcap0.8-dev",
    "libusb-1.0-0",
    "libusb-1.0-0-dev",
    "libmetis-dev",
    "libyaml-cpp-dev",
    "libpcl-dev",
    "liboctomap-dev",
    "libgraphicsmagick++-dev",
    "libfltk1.3-dev",
    "libtool",
    "libtiff-dev",
    "libcurl4-openssl-dev",
    "libwebsocketpp-dev",
    "libeigen3-dev",
    "libsqlite3-dev",
    "libzmq3-dev",
    "liburdfdom-dev",
    "libgtk2.0-dev",
    "sqlite3",
    "stow",
]

THIRDPARTY_SCRIPTS: List[str] = [
    "install_gtest.sh",
    "install_glog.sh",
    "install_gflags.sh",
    "install_grpc.sh",
    "install_gperftools.sh",
    "install_opencv.sh",
    "install_ceres_solver.sh",
    "install_nlohmann.sh",
    "install_osqp.sh",
    "install_behaviortree_cpp.sh",
    "install_python_modules.sh",
    "install_assimp.sh",
    "install_adolc.sh",
    "install_ipopt.sh",
]


def run_command(command: Iterable[str], *, dry_run: bool, env: dict | None = None) -> None:
    printable = " ".join(shlex.quote(c) for c in command)
    print(f"[RUN] {printable}")
    if dry_run:
        return
    subprocess.run(list(command), check=True, env=env)


def check_ubuntu() -> None:
    if not Path("/etc/os-release").exists():
        print("Warning: /etc/os-release not found, skip platform check.", file=sys.stderr)
        return
    os_release = Path("/etc/os-release").read_text(encoding="utf-8", errors="ignore")
    if "ID=ubuntu" not in os_release:
        print("Warning: this script is designed for Ubuntu-based systems.", file=sys.stderr)


def install_apt_dependencies(*, dry_run: bool) -> None:
    run_command(["sudo", "apt-get", "update"], dry_run=dry_run)
    cmd = ["sudo", "apt-get", "install", "-y"] + sorted(set(APT_PACKAGES))
    run_command(cmd, dry_run=dry_run)


def install_thirdparty(*, repo_root: Path, dry_run: bool) -> None:
    install_dir = repo_root / "docker" / "install"
    if not install_dir.exists():
        raise FileNotFoundError(f"Install directory not found: {install_dir}")

    for script in THIRDPARTY_SCRIPTS:
        script_path = install_dir / script
        if not script_path.exists():
            raise FileNotFoundError(f"Missing dependency installer: {script_path}")
        run_command(["bash", str(script_path)], dry_run=dry_run)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Install autonomy dependencies from dockerfile definition.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Path to autonomy repository root (default: auto detect).",
    )
    parser.add_argument(
        "--apt-only",
        action="store_true",
        help="Only install apt dependencies.",
    )
    parser.add_argument(
        "--thirdparty-only",
        action="store_true",
        help="Only run third-party installer scripts under docker/install.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands without executing.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()

    if args.apt_only and args.thirdparty_only:
        print("Error: --apt-only and --thirdparty-only cannot be used together.", file=sys.stderr)
        return 2

    if os.geteuid() == 0:
        print("Warning: running as root. 'sudo' commands may be redundant.", file=sys.stderr)

    check_ubuntu()

    try:
        if not args.thirdparty_only:
            print("==> Installing apt dependencies")
            install_apt_dependencies(dry_run=args.dry_run)

        if not args.apt_only:
            print("==> Installing third-party dependencies")
            install_thirdparty(repo_root=repo_root, dry_run=args.dry_run)
    except subprocess.CalledProcessError as exc:
        print(f"Command failed with exit code {exc.returncode}", file=sys.stderr)
        return exc.returncode
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Installation failed: {exc}", file=sys.stderr)
        return 1

    print("Dependency installation finished.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
