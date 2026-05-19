#!/usr/bin/env python3
"""
Install dependencies for the autonomy workspace.

APT packages mirror docker/dockerfile/autonomy.x86_64.dockerfile (plus a few
extras for standalone CMake builds without ROS base images).

Third-party installers mirror the dockerfile RUN steps under docker/install/.
Boost / Autolink / Bazel are no longer required (C++17 + CMake-only libautonomy).
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, List

# ---------------------------------------------------------------------------
# APT packages (keep in sync with docker/dockerfile/autonomy.x86_64.dockerfile)
# ---------------------------------------------------------------------------
_APT_BUILD_TOOLS: List[str] = [
    "sudo",
    "software-properties-common",
    "pkg-config",
    "autoconf",
    "automake",
    "cmake",
    "ninja-build",
    "libtool",
    "curl",
    "git",
    "unzip",
    "vim",
    "wget",
    "bc",
    "gdb",
    "clang-format",
    "stow",
    "lsb-release",
]

_APT_PYTHON: List[str] = [
    "python3-pip",
    "python3-dev",
    "python3-sphinx",
    "sphinx",
]

_APT_CMAKE_LIBS: List[str] = [
    # find_package / link targets used in CMakeLists.txt (system or third-party)
    "libeigen3-dev",
    "liblua5.3-dev",
    "libyaml-cpp-dev",
    "libprotobuf-dev",
    "protobuf-compiler",
    "libcairo2-dev",
    "libsuitesparse-dev",  # Ceres
    "libblas-dev",
    "liblapack-dev",
    "libmetis-dev",
    "libflann-dev",
    "libqhull-dev",
    "libtinyxml2-dev",
    "libsqlite3-dev",
    "libzmq3-dev",
    "liburdfdom-dev",
]

_APT_MULTIMEDIA_GUI: List[str] = [
    "libsdl2-dev",
    "libavcodec-dev",
    "libswscale-dev",
    "libgtk2.0-dev",
    "libfltk1.3-dev",
    "libtiff-dev",
]

_APT_NETWORK_MISC: List[str] = [
    "libcivetweb-dev",
    "libasio-dev",
    "libncurses5-dev",
    "libpoco-dev",
    "libpcap0.8",
    "libpcap0.8-dev",
    "libusb-1.0-0",
    "libusb-1.0-0-dev",
    "libcurl4-openssl-dev",
    "libwebsocketpp-dev",
    "uuid-dev",
    "sqlite3",
]

_APT_TESTING: List[str] = [
    "libgtest-dev",
    "libgmock-dev",
]

APT_PACKAGES: List[str] = sorted(
    set(
        _APT_BUILD_TOOLS
        + _APT_PYTHON
        + _APT_CMAKE_LIBS
        + _APT_MULTIMEDIA_GUI
        + _APT_NETWORK_MISC
        + _APT_TESTING
    )
)

# Order matches dockerfile third-party RUN steps (docker/install/*.sh).
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
    "install_behaviortree_cpp.sh",  # required when BUILD_TASKS=ON
    "install_python_modules.sh",
    "install_assimp.sh",
    "install_ogre.sh",
    "install_adolc.sh",
    "install_ipopt.sh",
]

# Optional skip detection for --skip-installed (any path match is treated as installed).
_SCRIPT_CHECK_PATHS: Dict[str, List[str]] = {
    "install_gtest.sh": [
        "/usr/local/lib/libgtest.a",
        "/usr/lib/x86_64-linux-gnu/libgtest.a",
    ],
    "install_glog.sh": [
        "/usr/local/lib/libglog.so",
        "/usr/lib/x86_64-linux-gnu/libglog.so",
    ],
    "install_gflags.sh": [
        "/usr/local/lib/libgflags.so",
        "/usr/lib/x86_64-linux-gnu/libgflags.so",
    ],
    "install_grpc.sh": [
        "/usr/local/lib/libgrpc++.so",
        "/usr/lib/x86_64-linux-gnu/libgrpc++.so",
    ],
    "install_gperftools.sh": [
        "/usr/local/lib/libtcmalloc.so",
        "/usr/lib/x86_64-linux-gnu/libtcmalloc.so",
    ],
    "install_opencv.sh": [
        "/usr/local/lib/libopencv_core.so",
        "/usr/lib/x86_64-linux-gnu/libopencv_core.so",
    ],
    "install_ceres_solver.sh": [
        "/usr/local/lib/libceres.so",
        "/usr/lib/x86_64-linux-gnu/libceres.so",
    ],
    "install_nlohmann.sh": [
        "/usr/local/include/nlohmann/json.hpp",
        "/usr/include/nlohmann/json.hpp",
    ],
    "install_osqp.sh": [
        "/usr/local/lib/libosqp.so",
        "/usr/lib/x86_64-linux-gnu/libosqp.so",
    ],
    "install_behaviortree_cpp.sh": [
        "/usr/local/lib/libbehaviortree_cpp.so",
        "/usr/lib/x86_64-linux-gnu/libbehaviortree_cpp.so",
    ],
    "install_adolc.sh": [
        "/usr/local/lib/libadolc.so",
        "/usr/lib/x86_64-linux-gnu/libadolc.so",
    ],
    "install_ipopt.sh": [
        "/usr/local/lib/libipopt.so",
        "/usr/lib/x86_64-linux-gnu/libipopt.so",
    ],
    "install_assimp.sh": [
        "/usr/local/lib/libassimp.so",
        "/usr/lib/x86_64-linux-gnu/libassimp.so",
    ],
    "install_ogre.sh": [
        "/usr/local/lib/libOgreMain.so",
        "/usr/lib/x86_64-linux-gnu/libOgreMain.so",
    ],
}


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
    run_command(["sudo", "apt-get", "-y", "--fix-broken", "install"], dry_run=dry_run)
    cmd = ["sudo", "apt-get", "install", "-y"] + APT_PACKAGES
    try:
        run_command(cmd, dry_run=dry_run)
    except subprocess.CalledProcessError:
        if dry_run:
            raise
        print("Retry apt install after dependency repair...", file=sys.stderr)
        run_command(["sudo", "apt-get", "-y", "--fix-broken", "install"], dry_run=dry_run)
        run_command(cmd, dry_run=dry_run)


def _can_detect_installed(script_name: str) -> bool:
    return script_name in _SCRIPT_CHECK_PATHS


def _is_script_dependency_installed(script_name: str) -> bool:
    return any(Path(p).exists() for p in _SCRIPT_CHECK_PATHS.get(script_name, []))


def install_thirdparty(
    *,
    repo_root: Path,
    dry_run: bool,
    resume_from: str | None,
    skip_installed: bool,
) -> None:
    install_dir = repo_root / "docker" / "install"
    if not install_dir.exists():
        raise FileNotFoundError(f"Install directory not found: {install_dir}")

    if resume_from is not None and resume_from not in THIRDPARTY_SCRIPTS:
        raise ValueError(
            f"--resume-from={resume_from} is invalid, choose one of: "
            f"{', '.join(THIRDPARTY_SCRIPTS)}"
        )

    start = resume_from is None
    for script in THIRDPARTY_SCRIPTS:
        if not start:
            if script == resume_from:
                start = True
            else:
                continue

        script_path = install_dir / script
        if not script_path.exists():
            raise FileNotFoundError(f"Missing dependency installer: {script_path}")

        if skip_installed and _can_detect_installed(script):
            if _is_script_dependency_installed(script):
                print(f"[SKIP] {script}: dependency already detected")
                continue

        run_command(["bash", str(script_path)], dry_run=dry_run)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Install autonomy dependencies (apt + docker/install scripts).",
        epilog=(
            "CMake expects third-party libs under /usr/local from docker/install "
            "(glog, gflags, Ceres, OpenCV, OSQP, BehaviorTree.CPP, etc.). "
            "Boost is not required."
        ),
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
    parser.add_argument(
        "--list-apt",
        action="store_true",
        help="Print apt package names and exit.",
    )
    parser.add_argument(
        "--resume-from",
        type=str,
        default=None,
        help="Resume third-party installation from a script name (e.g. install_opencv.sh).",
    )
    parser.add_argument(
        "--skip-installed",
        action="store_true",
        help="Skip third-party installers when known libraries/headers already exist.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list_apt:
        for pkg in APT_PACKAGES:
            print(pkg)
        return 0

    repo_root = args.repo_root.resolve()

    if args.apt_only and args.thirdparty_only:
        print("Error: --apt-only and --thirdparty-only cannot be used together.", file=sys.stderr)
        return 2

    if os.geteuid() == 0:
        print("Warning: running as root. 'sudo' commands may be redundant.", file=sys.stderr)

    check_ubuntu()

    try:
        if not args.thirdparty_only:
            print(f"==> Installing {len(APT_PACKAGES)} apt packages")
            install_apt_dependencies(dry_run=args.dry_run)

        if not args.apt_only:
            print(f"==> Installing {len(THIRDPARTY_SCRIPTS)} third-party scripts")
            install_thirdparty(
                repo_root=repo_root,
                dry_run=args.dry_run,
                resume_from=args.resume_from,
                skip_installed=args.skip_installed,
            )
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
