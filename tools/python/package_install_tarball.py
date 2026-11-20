#!/usr/bin/env python3
"""package_install_tarball.py — build and pack an install-tree tarball.

Used for Ansible artifact deployment.

Usage:
  python3 tools/python/package_install_tarball.py
  python3 tools/python/package_install_tarball.py --output /tmp/autonomy.tar.gz
  BUNDLE_CONFIG=OFF python3 tools/python/package_install_tarball.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def bundle_configuration(root: Path, staged_prefix: Path) -> None:
    """Keep the deployment layout aligned with the component-owned conf trees."""
    destination = staged_prefix / "share" / "autonomy" / "config"
    for source in sorted((root / "autonomy").glob("*/conf")):
        shutil.copytree(source, destination / source.parent.name, dirs_exist_ok=True)


def git_describe(root: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(root), "describe", "--tags", "--always", "--dirty"],
            check=True,
            capture_output=True,
            text=True,
        )
        return out.stdout.strip() or "dev"
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "dev"


def git_sha(root: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
        return out.stdout.strip() or "unknown"
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def default_jobs() -> int:
    try:
        return len(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        return os.cpu_count() or 4


def run(cmd: list[str], *, env: dict[str, str] | None = None) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True, env=env)


def main(argv: list[str] | None = None) -> int:
    root = repository_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Package an already configured and built CMake tree without rebuilding",
    )
    parser.add_argument(
        "--build-dir",
        default=os.environ.get("BUILD_DIR", str(root / "build")),
    )
    parser.add_argument(
        "--output",
        default=os.environ.get(
            "OUTPUT",
            str(root / "dist" / f"autonomy-{git_describe(root)}.tar.gz"),
        ),
    )
    parser.add_argument(
        "--prefix",
        default=os.environ.get("INSTALL_PREFIX", "/opt/autonomy"),
    )
    parser.add_argument(
        "--no-config",
        action="store_true",
        help="Do not bundle autonomy/*/conf into the tarball",
    )
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=int(os.environ.get("JOBS", default_jobs())),
    )
    args = parser.parse_args(argv)

    build_dir = Path(args.build_dir)
    output = Path(args.output)
    install_prefix = Path(args.prefix)
    if not install_prefix.is_absolute() or install_prefix == Path("/") or ".." in install_prefix.parts:
        parser.error("--prefix must be an absolute non-root install path")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    cmake_build_type = os.environ.get("CMAKE_BUILD_TYPE", "Release")
    build_grpc = os.environ.get("BUILD_GRPC", "ON")
    build_test = os.environ.get("BUILD_TEST", "OFF")
    bundle_config = (
        False
        if args.no_config
        else os.environ.get("BUNDLE_CONFIG", "ON").upper() in ("1", "ON", "TRUE", "YES")
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    build_dir.mkdir(parents=True, exist_ok=True)

    stage = Path(tempfile.mkdtemp(prefix="autonomy-pkg-"))
    try:
        if not args.skip_build:
            print(f"[package] configure: {build_dir}")
            run(
                [
                    "cmake",
                    "-G",
                    "Ninja",
                    "-S",
                    str(root),
                    "-B",
                    str(build_dir),
                    f"-DCMAKE_BUILD_TYPE={cmake_build_type}",
                    f"-DCMAKE_INSTALL_PREFIX={install_prefix}",
                    f"-DBUILD_GRPC={build_grpc}",
                    f"-DBUILD_TEST={build_test}",
                ]
            )
            run(["cmake", "--build", str(build_dir), "--parallel", str(args.jobs)])

        print(f"[package] build + stage install under {stage}")
        env = os.environ.copy()
        env["DESTDIR"] = str(stage)
        run(["cmake", "--install", str(build_dir), "--prefix", str(install_prefix)], env=env)

        staged_prefix = stage / install_prefix.relative_to("/")
        if bundle_config:
            print("[package] bundle autonomy/*/conf -> share/autonomy/config")
            bundle_configuration(root, staged_prefix)

        manifest_path = staged_prefix / "share" / "autonomy" / "MANIFEST.json"
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        cache = {}
        for line in (build_dir / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
            if not line.startswith(("#", "//")) and "=" in line and ":" in line.split("=", 1)[0]:
                key, value = line.split("=", 1)
                cache[key.split(":", 1)[0]] = value
        manifest = {
            "git_sha": git_sha(root),
            "git_describe": git_describe(root),
            "install_prefix": str(install_prefix),
            "architecture": platform.machine(),
            "cmake_build_type": cache.get("CMAKE_BUILD_TYPE", "unknown"),
            "build_grpc": cache.get("BUILD_GRPC", "unknown"),
            "build_test": cache.get("BUILD_TEST", "unknown"),
            "config_bundled": bundle_config,
            "packaged_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

        tar_name = install_prefix.name
        tar_parent = stage / install_prefix.parent.relative_to("/")
        run(
            [
                "tar",
                "-C",
                str(tar_parent),
                "-czf",
                str(output),
                tar_name,
            ]
        )
        digest = hashlib.sha256()
        with output.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        output.with_name(output.name + ".sha256").write_text(
            f"{digest.hexdigest()}  {output.name}\n", encoding="utf-8"
        )
        print(f"[package] wrote {output}")
    finally:
        shutil.rmtree(stage, ignore_errors=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
