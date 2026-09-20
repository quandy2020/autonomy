#!/usr/bin/env python3
"""package_install_tarball.py — build and pack an install-tree tarball.

Used for Ansible artifact deployment.

Usage:
  python3 tools/package_install_tarball.py
  python3 tools/package_install_tarball.py --output /tmp/autonomy.tar.gz
  BUNDLE_CONFIG=OFF python3 tools/package_install_tarball.py
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path


def repository_root() -> Path:
    return Path(__file__).resolve().parent.parent


def git_describe(root: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(root), "describe", "--tags", "--always"],
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
        help="Do not bundle config/ into the tarball",
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

        print(f"[package] build + stage install under {stage}")
        env = os.environ.copy()
        env["DESTDIR"] = str(stage)
        run(["ninja", "-C", str(build_dir), f"-j{args.jobs}", "install"], env=env)

        staged_prefix = stage / install_prefix.relative_to("/")
        if bundle_config:
            config_dest = staged_prefix / "share" / "autonomy" / "config"
            print("[package] bundle config/ -> share/autonomy/config")
            config_dest.mkdir(parents=True, exist_ok=True)
            # Prefer rsync when available; fall back to copytree.
            src_config = root / "config"
            if shutil.which("rsync"):
                run(
                    [
                        "rsync",
                        "-a",
                        "--delete",
                        f"{src_config}/",
                        f"{config_dest}/",
                    ]
                )
            else:
                if config_dest.exists():
                    shutil.rmtree(config_dest)
                shutil.copytree(src_config, config_dest)

        manifest_path = staged_prefix / "share" / "autonomy" / "MANIFEST.json"
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest = {
            "git_sha": git_sha(root),
            "git_describe": git_describe(root),
            "install_prefix": str(install_prefix),
            "cmake_build_type": cmake_build_type,
            "build_grpc": build_grpc,
            "build_test": build_test,
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
        print(f"[package] wrote {output}")
    finally:
        shutil.rmtree(stage, ignore_errors=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
