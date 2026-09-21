#!/usr/bin/env python3
"""
Install dependencies for the autonomy workspace.

Sources of truth:
  - docker/dockerfile/autonomy.aarch64.dockerfile  (apt + docker/install order)
  - cmake/autonomy_deps.cmake / CMakeLists.txt     (required find_package)

Packages that CMake expects as CONFIG under a single prefix (default
/usr/local: glog/gflags/protobuf/ceres/…) are installed ONLY via
docker/install/*.sh and are force-run so apt stubs cannot win the ABI race
(see Firefly glog 0.4 vs 0.6). Do not mix ~/.local with /usr/local.

Usage:
  python3 scripts/install_dependencies.py --profile board
  python3 scripts/install_dependencies.py --profile full --skip-installed
  python3 scripts/install_dependencies.py --prefix /opt/autonomy --skip-installed
  python3 scripts/install_dependencies.py --thirdparty-only --resume-from install_ceres_solver.sh
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
APT_DOCKERFILE_BASE: List[str] = [
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
    "libpcap-dev",
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

APT_PACKAGES_FULL: List[str] = APT_DOCKERFILE_BASE + [
    "libcivetweb-dev",
    "libgtk2.0-dev",
    "libfltk1.3-dev",
    "python3-sphinx",
    "sphinx",
    "libgmock-dev",
]

# Headless aarch64 / Firefly: no GTK/FLTK/Sphinx; keep NFS client.
APT_PACKAGES_BOARD: List[str] = APT_DOCKERFILE_BASE + [
    "nfs-common",
    "libgmock-dev",
]

# Soft apt packages: skip if unavailable (mirror/DNS flaky boards).
APT_OPTIONAL_PACKAGES: frozenset[str] = frozenset(
    {
        "clang-format",
        "stow",
        "sqlite3",  # runtime CLI; libsqlite3-dev is enough for build
        "libpcap0.8-dev",  # jammy may only expose libpcap-dev / libpcap0.8
        "libmetis-dev",
        "libpoco-dev",
    }
)

# Prefer these when primary name is missing.
APT_PACKAGE_FALLBACKS: Dict[str, List[str]] = {
    "libpcap0.8-dev": ["libpcap-dev", "libpcap0.8"],
    "libncurses5-dev": ["libncurses-dev"],
}

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


DEFAULT_INSTALL_PREFIX = "/usr/local"


# Relative artifact paths for --skip-installed (joined with install prefix).
# Extra system paths (apt OpenCV etc.) are appended per-script where useful.
SCRIPT_INSTALL_RELS: Dict[str, List[tuple[str, ...]]] = {
    "install_gtest.sh": [("lib", "libgtest.so"), ("lib", "libgtest.a")],
    "install_gflags.sh": [("lib", "libgflags.so")],
    "install_glog.sh": [("lib", "libglog.so")],
    "install_protobuf.sh": [("bin", "protoc")],
    "install_grpc.sh": [("lib", "libgrpc++.so")],
    "install_gperftools.sh": [("lib", "libtcmalloc.so")],
    "install_opencv.sh": [("lib", "libopencv_core.so")],
    "install_ceres_solver.sh": [("lib", "libceres.so")],
    "install_nlohmann.sh": [("include", "nlohmann", "json.hpp")],
    "install_osqp.sh": [("lib", "libosqp.so")],
    "install_g2o.sh": [("lib", "cmake", "g2o", "g2oConfig.cmake")],
    "install_fbow.sh": [("lib", "libfbow.so")],
    "install_behaviortree_cpp.sh": [("lib", "libbehaviortree_cpp.so")],
    "install_adolc.sh": [("include", "adolc", "adolc.h")],
    "install_ipopt.sh": [
        ("include", "coin-or", "IpIpoptApplication.hpp"),
        ("include", "coin", "IpIpoptApplication.hpp"),
    ],
    "install_assimp.sh": [("lib", "libassimp.so")],
    "install_ogre.sh": [("lib", "libOgreMain.so")],
}

# Optional fallbacks outside the install prefix (apt / distro packages).
SCRIPT_INSTALL_SYSTEM_FALLBACKS: Dict[str, List[str]] = {
    "install_gperftools.sh": [
        "/usr/lib/libtcmalloc.so",
        "/usr/lib/aarch64-linux-gnu/libtcmalloc.so",
        "/usr/lib/x86_64-linux-gnu/libtcmalloc.so",
    ],
    "install_opencv.sh": [
        "/usr/lib/aarch64-linux-gnu/libopencv_core.so",
        "/usr/lib/x86_64-linux-gnu/libopencv_core.so",
    ],
    "install_nlohmann.sh": ["/usr/include/nlohmann/json.hpp"],
    "install_adolc.sh": ["/usr/include/adolc/adolc.h"],
    "install_ipopt.sh": [
        "/usr/include/coin/IpIpoptApplication.hpp",
        "/usr/include/coin-or/IpIpoptApplication.hpp",
    ],
}


def script_install_check_paths(script_name: str, prefix: str) -> List[str]:
    paths = [
        str(Path(prefix).joinpath(*rel))
        for rel in SCRIPT_INSTALL_RELS.get(script_name, [])
    ]
    paths.extend(SCRIPT_INSTALL_SYSTEM_FALLBACKS.get(script_name, []))
    return paths


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


def apt_package_available(package: str) -> bool:
    """True if apt-cache knows a candidate for package (non-zero = missing)."""
    try:
        result = subprocess.run(
            ["apt-cache", "policy", package],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return True  # best-effort; let apt-get decide
    # "Candidate: (none)" means not installable from current indexes.
    for line in result.stdout.splitlines():
        if "Candidate:" in line:
            return "(none)" not in line
    return False


def resolve_apt_packages(packages: Sequence[str]) -> List[str]:
    """Apply fallbacks and drop unavailable optional packages."""
    resolved: List[str] = []
    skipped: List[str] = []
    for pkg in sorted(set(packages)):
        candidates = [pkg] + APT_PACKAGE_FALLBACKS.get(pkg, [])
        chosen = None
        for candidate in candidates:
            if apt_package_available(candidate):
                chosen = candidate
                break
        if chosen is None:
            if pkg in APT_OPTIONAL_PACKAGES or pkg in APT_PACKAGE_FALLBACKS:
                skipped.append(pkg)
                continue
            # Required but unknown to cache (often stale indexes) — still try.
            chosen = pkg
        if chosen not in resolved:
            resolved.append(chosen)
    if skipped:
        print(
            f"==> Skipping unavailable optional apt packages: {', '.join(skipped)}"
        )
    return resolved


def install_apt_dependencies(
    packages: Sequence[str], *, dry_run: bool
) -> None:
    # update may partially fail on flaky DNS; continue with cached indexes.
    try:
        run_command(["sudo", "apt-get", "update"], dry_run=dry_run)
    except subprocess.CalledProcessError as exc:
        print(
            f"Warning: apt-get update failed ({exc.returncode}); "
            "continuing with existing indexes. Fix board DNS/NAT if packages missing.",
            file=sys.stderr,
        )
    run_command(
        ["sudo", "apt-get", "-y", "--fix-broken", "install"], dry_run=dry_run
    )
    if "libunwind-dev" in packages:
        run_command(
            ["sudo", "apt-get", "install", "-y", "libunwind-dev"],
            dry_run=dry_run,
        )

    to_install = (
        list(sorted(set(packages)))
        if dry_run
        else resolve_apt_packages(packages)
    )
    if not to_install:
        print("==> No apt packages to install")
        return

    cmd = ["sudo", "apt-get", "install", "-y"] + to_install
    try:
        run_command(cmd, dry_run=dry_run)
    except subprocess.CalledProcessError:
        if dry_run:
            raise
        print(
            "Retry apt install package-by-package after repair...",
            file=sys.stderr,
        )
        run_command(
            ["sudo", "apt-get", "-y", "--fix-broken", "install"],
            dry_run=dry_run,
        )
        failed: List[str] = []
        for pkg in to_install:
            try:
                run_command(
                    ["sudo", "apt-get", "install", "-y", pkg], dry_run=dry_run
                )
            except subprocess.CalledProcessError:
                if pkg in APT_OPTIONAL_PACKAGES or any(
                    pkg == f or pkg in fs
                    for f, fs in APT_PACKAGE_FALLBACKS.items()
                ):
                    print(f"[SKIP] optional/unavailable apt: {pkg}")
                    continue
                failed.append(pkg)
        if failed:
            raise RuntimeError(
                "Required apt packages failed (check DNS/NAT + apt update): "
                + ", ".join(failed)
            ) from None


def can_detect_installed(script_name: str) -> bool:
    return (
        script_name in SCRIPT_INSTALL_RELS
        or script_name in SCRIPT_INSTALL_SYSTEM_FALLBACKS
    )


def is_script_dependency_installed(script_name: str, prefix: str) -> bool:
    check_paths = script_install_check_paths(script_name, prefix)
    return any(Path(path).exists() for path in check_paths)


def install_thirdparty(
    *,
    repo_root: Path,
    scripts: Sequence[str],
    dry_run: bool,
    resume_from: str | None,
    skip_installed: bool,
    force_all: bool,
    prefix: str,
) -> None:
    install_dir = repo_root / "docker" / "install"
    if not install_dir.exists():
        raise FileNotFoundError(f"Install directory not found: {install_dir}")

    if resume_from is not None and resume_from not in scripts:
        raise ValueError(
            f"--resume-from={resume_from} is invalid, choose one of: "
            + ", ".join(scripts)
        )

    # Prefer a user-writable cache. Avoid root-owned /thirdparty on boards
    # (causes git "dubious ownership" when install scripts run as firefly).
    cache = os.environ.get("AUTONOMY_THIRDPARTY")
    if not cache:
        candidates = [
            Path.home() / ".cache" / "autonomy" / "thirdparty",
            Path("/thirdparty"),
            Path(f"/tmp/autonomy-thirdparty-{os.getuid()}"),
        ]
        for candidate in candidates:
            try:
                candidate.mkdir(parents=True, exist_ok=True)
                if os.access(candidate, os.W_OK):
                    # Skip /thirdparty if not owned by current user.
                    if candidate == Path("/thirdparty"):
                        st = candidate.stat()
                        if st.st_uid != os.getuid() and os.getuid() != 0:
                            continue
                    os.environ["AUTONOMY_THIRDPARTY"] = str(candidate)
                    break
            except OSError:
                continue
        else:
            cache_path = Path.home() / ".cache" / "autonomy" / "thirdparty"
            cache_path.mkdir(parents=True, exist_ok=True)
            os.environ["AUTONOMY_THIRDPARTY"] = str(cache_path)

    os.environ["AUTONOMY_INSTALL_PREFIX"] = prefix
    print(f"==> AUTONOMY_THIRDPARTY={os.environ.get('AUTONOMY_THIRDPARTY')}")
    print(f"==> AUTONOMY_INSTALL_PREFIX={prefix}")

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

        force_rebuild = force_all  # only --force-thirdparty rebuilds everything
        prefer_local = script in FORCE_THIRDPARTY_SCRIPTS
        if (
            skip_installed
            and not force_rebuild
            and can_detect_installed(script)
            and is_script_dependency_installed(script, prefix)
        ):
            print(
                f"[SKIP] {script}: dependency already detected under {prefix}"
            )
            continue

        if prefer_local or force_rebuild:
            print(
                f"[FORCE] {script}: docker/install → {prefix} (ignore apt stubs)"
            )

        env = os.environ.copy()
        env["AUTONOMY_INSTALL_PREFIX"] = prefix
        if prefer_local or force_rebuild:
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
        "--prefix",
        type=str,
        default=DEFAULT_INSTALL_PREFIX,
        help=(
            "Single install prefix for docker/install/*.sh "
            f"(default: {DEFAULT_INSTALL_PREFIX}). "
            "Exported as AUTONOMY_INSTALL_PREFIX; do not mix with ~/.local."
        ),
    )
    parser.add_argument(
        "--skip-installed",
        action="store_true",
        help=(
            "Skip third-party installers when the artifact is already under "
            "--prefix (or a known system fallback). "
            "Use --force-thirdparty to rebuild."
        ),
    )
    parser.add_argument(
        "--force-thirdparty",
        action="store_true",
        help="Rebuild every third-party script (ignore --skip-installed).",
    )
    parser.add_argument(
        "--no-purge-conflicts",
        action="store_true",
        help="Do not apt-purge glog/gflags/ceres/protobuf/grpc apt packages.",
    )
    return parser.parse_args()


def ensure_prefix_ldconfig(prefix: str, *, dry_run: bool) -> None:
    """Register ${prefix}/lib with the dynamic linker when possible."""
    lib_dir = Path(prefix) / "lib"
    if not lib_dir.is_dir():
        return
    conf_name = "autonomy-prefix.conf"
    if prefix == DEFAULT_INSTALL_PREFIX:
        conf_name = "usr-local.conf"
    conf_path = f"/etc/ld.so.conf.d/{conf_name}"
    cmd = (
        f'echo "{lib_dir}" > {shlex.quote(conf_path)} && ldconfig'
    )
    if dry_run:
        print(f"[DRY] sudo bash -c {shlex.quote(cmd)}")
        return
    try:
        subprocess.run(
            ["sudo", "bash", "-c", cmd],
            check=False,
        )
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Warning: ldconfig setup: {exc}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    prefix = str(Path(args.prefix).expanduser().resolve())

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
        print(f"# prefix={prefix}")
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
    print(f"==> install_prefix={prefix}")
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
                f"(docker/install → {prefix})"
            )
            install_thirdparty(
                repo_root=repo_root,
                scripts=thirdparty_scripts,
                dry_run=args.dry_run,
                resume_from=args.resume_from,
                skip_installed=args.skip_installed,
                force_all=args.force_thirdparty,
                prefix=prefix,
            )
            ensure_prefix_ldconfig(prefix, dry_run=args.dry_run)
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
        f"CMake tip: keep a single prefix ({prefix})\n"
        f"  cmake -S $PWD -B ${{HOME}}/autonomy_ws/build "
        f"-DCMAKE_PREFIX_PATH={prefix} "
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
