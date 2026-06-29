#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan). All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

"""Common utilities for Docker operations."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

try:
    from print_color import print_error, print_info, print_warning
except ImportError:
    def print_error(msg): print(f"ERROR: {msg}", file=sys.stderr)
    def print_warning(msg): print(f"WARNING: {msg}", file=sys.stderr)
    def print_info(msg): print(f"INFO: {msg}")


def resolve_autonomy_env_dir(script_dir: Path) -> Path:
    """Resolve host directory to mount at /workspace/autonomy in the container.

    Priority:
    1. AUTONOMY_ENV environment variable
    2. Repo root containing src/autonomy/CMakeLists.txt
    3. Standalone src/autonomy checkout (CMakeLists.txt + docker/ in same dir)
    4. Current working directory
    """
    env_path = os.environ.get("AUTONOMY_ENV", "").strip()
    if env_path:
        return Path(env_path).resolve()

    for parent in [script_dir, *script_dir.parents]:
        if (parent / "src" / "autonomy" / "CMakeLists.txt").is_file():
            return parent.resolve()

    for parent in [script_dir, *script_dir.parents]:
        if (parent / "CMakeLists.txt").is_file() and (parent / "docker").is_dir():
            return parent.resolve()

    return Path.cwd().resolve()


def check_image_exists(image_name: str) -> bool:
    """Check if a Docker image exists locally."""
    try:
        for tag in [image_name, f"{image_name}:latest"]:
            result = subprocess.run(
                ["docker", "images", "-q", tag],
                capture_output=True,
                text=True,
                check=False
            )
            if result.stdout.strip():
                return True
        return False
    except Exception:
        return False


def get_image_platform(image_name: str) -> str:
    """Get the platform architecture of a Docker image."""
    try:
        for tag in [image_name, f"{image_name}:latest"]:
            result = subprocess.run(
                ["docker", "inspect", "--format={{.Architecture}}", tag],
                capture_output=True,
                text=True,
                check=False
            )
            if result.returncode == 0 and result.stdout.strip():
                arch = result.stdout.strip()
                # Normalize architecture names
                if arch in ("arm64", "aarch64"):
                    return "arm64"
                elif arch in ("amd64", "x86_64"):
                    return "x86_64"
                return arch
        return None
    except Exception:
        return None


def check_docker_available() -> bool:
    """Check if Docker is available and accessible."""
    try:
        result = subprocess.run(
            ["docker", "ps"],
            capture_output=True,
            check=False
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False


def check_nvidia_available() -> bool:
    """Check if NVIDIA GPU driver is available on the host."""
    return shutil.which("nvidia-smi") is not None


def check_docker_gpu_support() -> bool:
    """Return True if Docker can pass NVIDIA GPUs into containers."""
    if not check_nvidia_available():
        return False

    cdi_specs = (
        Path("/etc/cdi/nvidia.yaml"),
        Path("/var/run/cdi/nvidia.yaml"),
    )
    if any(path.is_file() for path in cdi_specs):
        return True

    try:
        result = subprocess.run(
            ["docker", "info", "--format", "{{json .Runtimes}}"],
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )
        if result.returncode == 0 and "nvidia" in result.stdout:
            return True
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        pass

    return False


def is_gpu_docker_error(output: str) -> bool:
    """Return True if docker output indicates a GPU/CDI configuration problem."""
    lowered = output.lower()
    markers = (
        "cdi",
        "gpu vendor",
        "nvidia.com/gpu",
        "could not select device driver",
        "unknown or invalid runtime name: nvidia",
    )
    return any(marker in lowered for marker in markers)


def normalize_path(path: str, base_dir: Path) -> Path:
    """Normalize a path relative to base directory."""
    p = Path(path)
    if p.is_absolute():
        return p
    return base_dir / p


def run_command(cmd: list, check: bool = True, **kwargs) -> subprocess.CompletedProcess:
    """Run a command and handle errors."""
    try:
        return subprocess.run(cmd, check=check, **kwargs)
    except subprocess.CalledProcessError as e:
        print_error(f"Command failed with exit code {e.returncode}: {' '.join(cmd)}")
        raise
    except KeyboardInterrupt:
        print_info("\nOperation interrupted")
        sys.exit(1)

