#!/usr/bin/env python3
"""
Install dependencies for the autonomy workspace.

Sources of truth:
  - docker/dockerfile/autonomy.aarch64.dockerfile  (apt + docker/install order)
  - cmake/autonomy_deps.cmake / CMakeLists.txt     (required find_package)

Packages that CMake expects as CONFIG under /usr/local (glog/gflags/protobuf/
ceres/…) are installed ONLY via docker/install/*.sh and are force-run so apt
stubs cannot win the ABI race (see Firefly glog 0.4 vs 0.6).

Usage:
  python3 scripts/install_dependency.py --profile board
  python3 scripts/install_dependency.py --profile full --skip-installed
  python3 scripts/install_dependency.py --thirdparty-only --resume-from install_ceres_solver.sh
"""

from __future__ import annotations

import argparse
import os
import platform
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


# ---------------------------------------------------------------------------
# Apt: dockerfile system packages MINUS ones superseded by docker/install
# ---------------------------------------------------------------------------

# From autonomy.aarch64.dockerfile apt-get install, plus board extras.
# Intentionally OMIT (installed from source instead):
#   libgoogle-glog-dev, libgflags-dev, libceres-dev,
#   nlohmann-json3-dev, libgtest-dev (optional),
#   libgrpc*-dev / protobuf-compiler* (use install_protobuf/grpc.sh → 3.19).
_APT_DOCKERFILE_BASE: List[str] = [
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
    "python3-pip",
    "python3-dev",
    "uuid-dev",
    "libsuitesparse-dev",
    "lsb-release",
    "libcairo2-dev",
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
    "libtool",
    "libtiff-dev",
    "libcurl4-openssl-dev",
    "libwebsocketpp-dev",
    "libeigen3-dev",
    "libsqlite3-dev",
    "libzmq3-dev",
    "liburdfdom-dev",
    "liburdfdom-headers-dev",
    "clang-format",
    "sqlite3",
    "stow",
    "build-essential",
    "libunwind-dev",
    # CMake map / autodriver common
    "libpcl-dev",
    "libtbb-dev",
]

APT_PACKAGES_FULL: List[str] = _APT_DOCKERFILE_BASE + [
    "libcivetweb-dev",
    "libgtk2.0-dev",
    "libfltk1.3-dev",
    "python3-sphinx",
    "sphinx",
    "libgmock-dev",
]

# Headless aarch64 / Firefly: no GTK/FLTK/Sphinx; keep NFS client.
APT_PACKAGES_BOARD: List[str] = _APT_DOCKERFILE_BASE + [
    "nfs-common",
    "libgmock-dev",
]

# Apt packages that fight docker/install (remove if present before thirdparty).
APT_CONFLICT_PACKAGES: List[str] = [
    "libgoogle-glog-dev",
    "libgoogle-glog0v5",
    "libgflags-dev",
    "libceres-dev",
    "libceres2",
    "nlohmann-json3-dev",
    "libgrpc++-dev",
    "libgrpc-dev",
    "libgrpc10",
    "libgrpc++1",
    "protobuf-compiler",
    "protobuf-compiler-grpc",
    "libprotobuf-dev",
    "libprotoc-dev",
]


# ---------------------------------------------------------------------------
# Third-party: dockerfile install order (cmake CONFIG under /usr/local)
# ---------------------------------------------------------------------------

# Exact order from autonomy.aarch64.dockerfile RUN bash /tmp/install/...
THIRDPARTY_SCRIPTS_DOCKERFILE: List[str] = [
    "install_gtest.sh",
    "install_glog.sh",
    "install_gflags.sh",
    "install_protobuf.sh",
    "install_grpc.sh",
    "install_gperftools.sh",
    "install_opencv.sh",
    "install_ceres_solver.sh",
    "install_g2o.sh",
    "install_fbow.sh",
    "install_nlohmann.sh",
    "install_osqp.sh",
    "install_behaviortree_cpp.sh",
    "install_python_modules.sh",
    "install_adolc.sh",
    "install_ipopt.sh",
]

THIRDPARTY_SCRIPTS_FULL: List[str] = list(THIRDPARTY_SCRIPTS_DOCKERFILE) + [
    "install_taskflow.sh",
    "install_assimp.sh",
    "install_ogre.sh",
]

# Board: dockerfile set (cmake core + localization + BT + control QUIET deps).
# Skip Ogre/Assimp/Taskflow (GUI / optional).
THIRDPARTY_SCRIPTS_BOARD: List[str] = list(THIRDPARTY_SCRIPTS_DOCKERFILE)

# Always run via docker/install (ignore --skip-installed and apt stubs).
# Heavy/optional (opencv/gperftools) and apt wrappers (adolc/ipopt) are not
# forced so board can reuse jammy packages when present.
FORCE_THIRDPARTY_SCRIPTS: frozenset[str] = frozenset(
    {
        "install_gtest.sh",
        "install_glog.sh",
        "install_gflags.sh",
        "install_protobuf.sh",
        "install_grpc.sh",
        "install_ceres_solver.sh",
        "install_nlohmann.sh",
        "install_osqp.sh",
        "install_g2o.sh",
        "install_fbow.sh",
        "install_behaviortree_cpp.sh",
    }
)


def _usr_local(*rel: str) -> List[str]:
    return [str(Path("/usr/local").joinpath(*rel))]


# Detection paths for --skip-installed (non-forced only). Forced scripts
# never skip; checks below are /usr/local-only for documentation / list.
SCRIPT_INSTALL_CHECKS: Dict[str, List[str]] = {
    "install_gtest.sh": _usr_local("lib", "libgtest.a"),
    "install_gflags.sh": _usr_local("lib", "libgflags.so"),
    "install_glog.sh": _usr_local("lib", "libglog.so"),
    "install_protobuf.sh": _usr_local("bin", "protoc"),
    "install_grpc.sh": _usr_local("lib", "libgrpc++.so"),
    "install_gperftools.sh": _usr_local("lib", "libtcmalloc.so"),
    "install_opencv.sh": [
        "/usr/local/lib/libopencv_core.so",
        "/usr/lib/aarch64-linux-gnu/libopencv_core.so",
        "/usr/lib/x86_64-linux-gnu/libopencv_core.so",
    ],
    "install_ceres_solver.sh": _usr_local("lib", "libceres.so"),
    "install_nlohmann.sh": [
        "/usr/local/include/nlohmann/json.hpp",
        "/usr/include/nlohmann/json.hpp",
    ],
    "install_osqp.sh": _usr_local("lib", "libosqp.so"),
    "install_g2o.sh": _usr_local("lib", "cmake", "g2o", "g2oConfig.cmake"),
    "install_fbow.sh": _usr_local("lib", "libfbow.so"),
    "install_taskflow.sh": _usr_local("include", "taskflow", "taskflow.hpp"),
    "install_behaviortree_cpp.sh": _usr_local("lib", "libbehaviortree_cpp.so"),
    "install_adolc.sh": [
        "/usr/include/adolc/adolc.h",
        "/usr/local/include/adolc/adolc.h",
    ],
    "install_ipopt.sh": [
        "/usr/include/coin/IpIpoptApplication.hpp",
        "/usr/include/coin-or/IpIpoptApplication.hpp",
        "/usr/local/include/coin-or/IpIpoptApplication.hpp",
    ],
    "install_assimp.sh": _usr_local("lib", "libassimp.so"),
    "install_ogre.sh": _usr_local("lib", "libOgreMain.so"),
}


def run_command(
    command: Iterable[str], *, dry_run: bool, env: dict | None = None
) -> None:
    printable = " ".join(shlex.quote(c) for c in command)
    print(f"[RUN] {printable}")
    if dry_run:
        return
    subprocess.run(list(command), check=True, env=env)


def check_ubuntu() -> None:
    if not Path("/etc/os-release").exists():
        print(
            "Warning: /etc/os-release not found, skip platform check.",
            file=sys.stderr,
        )
        return
    os_release = Path("/etc/os-release").read_text(
        encoding="utf-8", errors="ignore"
    )
    if "ID=ubuntu" not in os_release and "ID=debian" not in os_release:
        print(
            "Warning: this script is designed for Ubuntu/Debian-based systems.",
            file=sys.stderr,
        )


def select_lists(profile: str) -> tuple[Sequence[str], Sequence[str]]:
    if profile == "board":
        return APT_PACKAGES_BOARD, THIRDPARTY_SCRIPTS_BOARD
    if profile == "full":
        return APT_PACKAGES_FULL, THIRDPARTY_SCRIPTS_FULL
    raise ValueError(f"unknown profile: {profile}")


def purge_apt_conflicts(*, dry_run: bool) -> None:
    """Remove apt packages that shadow /usr/local docker/install builds."""
    print("==> Purging apt packages that conflict with docker/install")
    # --allow-change-held-packages: board may have held glog/ceres.
    cmd = [
        "sudo",
        "apt-get",
        "remove",
        "-y",
        "--purge",
        "--allow-change-held-packages",
    ] + APT_CONFLICT_PACKAGES
    printable = " ".join(shlex.quote(c) for c in cmd)
    print(f"[RUN] {printable}")
    if dry_run:
        return
    # Non-zero if already absent is OK.
    subprocess.run(cmd, check=False)
    run_command(
        ["sudo", "apt-get", "-y", "--fix-broken", "install"], dry_run=False
    )
    # Drop leftover apt cmake configs / headers that confuse find_package.
    leftovers = [
        "/usr/lib/aarch64-linux-gnu/cmake/glog",
        "/usr/lib/x86_64-linux-gnu/cmake/glog",
        "/usr/lib/aarch64-linux-gnu/cmake/gflags",
        "/usr/lib/x86_64-linux-gnu/cmake/gflags",
        "/usr/lib/aarch64-linux-gnu/cmake/Ceres",
        "/usr/lib/x86_64-linux-gnu/cmake/Ceres",
        "/usr/include/glog",
        "/usr/include/gflags",
    ]
    for path in leftovers:
        p = Path(path)
        if p.exists():
            run_command(["sudo", "rm", "-rf", str(p)], dry_run=False)


def install_apt_dependencies(
    packages: Sequence[str], *, dry_run: bool
) -> None:
    run_command(["sudo", "apt-get", "update"], dry_run=dry_run)
    run_command(
        ["sudo", "apt-get", "-y", "--fix-broken", "install"], dry_run=dry_run
    )
    if "libunwind-dev" in packages:
        run_command(
            ["sudo", "apt-get", "install", "-y", "libunwind-dev"],
            dry_run=dry_run,
        )
    cmd = ["sudo", "apt-get", "install", "-y"] + sorted(set(packages))
    try:
        run_command(cmd, dry_run=dry_run)
    except subprocess.CalledProcessError:
        if dry_run:
            raise
        print("Retry apt install after dependency repair...", file=sys.stderr)
        run_command(
            ["sudo", "apt-get", "-y", "--fix-broken", "install"],
            dry_run=dry_run,
        )
        run_command(cmd, dry_run=dry_run)


def _can_detect_installed(script_name: str) -> bool:
    return script_name in SCRIPT_INSTALL_CHECKS


def _is_script_dependency_installed(script_name: str) -> bool:
    check_paths = SCRIPT_INSTALL_CHECKS.get(script_name, [])
    return any(Path(p).exists() for p in check_paths)


def install_thirdparty(
    *,
    repo_root: Path,
    scripts: Sequence[str],
    dry_run: bool,
    resume_from: str | None,
    skip_installed: bool,
    force_all: bool,
) -> None:
    install_dir = repo_root / "docker" / "install"
    if not install_dir.exists():
        raise FileNotFoundError(f"Install directory not found: {install_dir}")

    if resume_from is not None and resume_from not in scripts:
        raise ValueError(
            f"--resume-from={resume_from} is invalid, choose one of: "
            + ", ".join(scripts)
        )

    # Prefer /thirdparty (dockerfile convention) when writable.
    cache = os.environ.get("AUTONOMY_THIRDPARTY")
    if not cache:
        for candidate in (
            Path("/thirdparty"),
            Path.home() / ".cache" / "autonomy" / "thirdparty",
        ):
            try:
                candidate.mkdir(parents=True, exist_ok=True)
                if os.access(candidate, os.W_OK):
                    os.environ["AUTONOMY_THIRDPARTY"] = str(candidate)
                    break
            except OSError:
                continue
        else:
            cache_path = Path.home() / ".cache" / "autonomy" / "thirdparty"
            cache_path.mkdir(parents=True, exist_ok=True)
            os.environ["AUTONOMY_THIRDPARTY"] = str(cache_path)

    # docker/install scripts expect writable /usr/local (or sudo).
    print(
        f"==> AUTONOMY_THIRDPARTY={os.environ.get('AUTONOMY_THIRDPARTY')}"
    )

    start = resume_from is None
    for script in scripts:
        if not start:
            if script == resume_from:
                start = True
            else:
                continue

        script_path = install_dir / script
        if not script_path.exists():
            raise FileNotFoundError(
                f"Missing dependency installer: {script_path}"
            )

        force = force_all or script in FORCE_THIRDPARTY_SCRIPTS
        if (
            skip_installed
            and not force
            and _can_detect_installed(script)
            and _is_script_dependency_installed(script)
        ):
            print(f"[SKIP] {script}: dependency already detected")
            continue

        if force:
            print(f"[FORCE] {script}: docker/install (CMAKE /usr/local)")

        env = os.environ.copy()
        if force:
            env["AUTONOMY_FORCE_THIRDPARTY"] = "1"
        run_command(["bash", str(script_path)], dry_run=dry_run, env=env)


def parse_args() -> argparse.Namespace:
    default_profile = (
        "board"
        if platform.machine() in ("aarch64", "arm64")
        else "full"
    )
    parser = argparse.ArgumentParser(
        description=(
            "Install autonomy deps from autonomy.aarch64.dockerfile + "
            "cmake/autonomy_deps.cmake via apt + docker/install/*.sh."
        ),
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Path to autonomy repository root (default: parent of scripts/).",
    )
    parser.add_argument(
        "--profile",
        choices=("full", "board"),
        default=default_profile,
        help=(
            "full=desktop/CI (+Ogre/Assimp); board=headless aarch64 "
            f"(default on this host: {default_profile})."
        ),
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
        "--list",
        action="store_true",
        help="Print apt packages and third-party scripts for the profile, then exit.",
    )
    parser.add_argument(
        "--resume-from",
        type=str,
        default=None,
        help=(
            "Resume third-party installation from a script name "
            "(e.g. install_ceres_solver.sh)."
        ),
    )
    parser.add_argument(
        "--skip-installed",
        action="store_true",
        help=(
            "Skip non-forced third-party installers when /usr/local already "
            "has the artifact. Forced dockerfile scripts always re-run."
        ),
    )
    parser.add_argument(
        "--force-thirdparty",
        action="store_true",
        help="Force every third-party script in the profile (set AUTONOMY_FORCE_THIRDPARTY).",
    )
    parser.add_argument(
        "--no-purge-conflicts",
        action="store_true",
        help="Do not apt-purge glog/gflags/ceres/protobuf/grpc apt packages.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()

    if args.apt_only and args.thirdparty_only:
        print(
            "Error: --apt-only and --thirdparty-only cannot be used together.",
            file=sys.stderr,
        )
        return 2

    try:
        apt_packages, thirdparty_scripts = select_lists(args.profile)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    if args.list:
        print(f"# profile={args.profile}")
        print("# apt (system; no glog/gflags/ceres/protobuf/grpc)")
        for pkg in sorted(set(apt_packages)):
            print(pkg)
        print("# apt conflicts (purged before thirdparty)")
        for pkg in APT_CONFLICT_PACKAGES:
            print(pkg)
        print("# thirdparty (docker/install, forced)")
        for script in thirdparty_scripts:
            forced = (
                " FORCE"
                if args.force_thirdparty or script in FORCE_THIRDPARTY_SCRIPTS
                else ""
            )
            print(f"{script}{forced}")
        return 0

    if os.geteuid() == 0:
        print(
            "Warning: running as root. 'sudo' commands may be redundant.",
            file=sys.stderr,
        )

    check_ubuntu()
    print(f"==> profile={args.profile} arch={platform.machine()}")
    print(f"==> repo_root={repo_root}")
    print(
        "==> source: autonomy.aarch64.dockerfile + cmake/autonomy_deps.cmake"
    )

    try:
        if not args.thirdparty_only:
            print(f"==> Installing {len(set(apt_packages))} apt dependencies")
            install_apt_dependencies(apt_packages, dry_run=args.dry_run)

        if not args.apt_only:
            if not args.no_purge_conflicts:
                purge_apt_conflicts(dry_run=args.dry_run)
            print(
                f"==> Installing {len(thirdparty_scripts)} third-party scripts "
                "(docker/install → /usr/local)"
            )
            install_thirdparty(
                repo_root=repo_root,
                scripts=thirdparty_scripts,
                dry_run=args.dry_run,
                resume_from=args.resume_from,
                skip_installed=args.skip_installed,
                force_all=args.force_thirdparty,
            )
            # Ensure dynamic linker finds /usr/local
            if not args.dry_run:
                try:
                    subprocess.run(
                        [
                            "sudo",
                            "bash",
                            "-c",
                            "echo /usr/local/lib > /etc/ld.so.conf.d/usr-local.conf && ldconfig",
                        ],
                        check=False,
                    )
                except Exception as exc:  # pylint: disable=broad-except
                    print(f"Warning: ldconfig setup: {exc}", file=sys.stderr)
    except subprocess.CalledProcessError as exc:
        print(
            f"Command failed with exit code {exc.returncode}", file=sys.stderr
        )
        return exc.returncode
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Installation failed: {exc}", file=sys.stderr)
        return 1

    print("Dependency installation finished.")
    print(
        "CMake tip: prefer /usr/local\n"
        "  cmake -S $PWD -B ${HOME}/autonomy_ws/build "
        "-DCMAKE_PREFIX_PATH=/usr/local "
        "-DCMAKE_BUILD_TYPE=Release"
    )
    if args.profile == "board":
        print(
            "Board extras: -DBUILD_AUTOVIZ=OFF -DBUILD_ORBISVIEW=OFF "
            "-DBUILD_DOCS=OFF -DBUILD_ONNXRUNTIME=OFF"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
