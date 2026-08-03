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
import time
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


def parse_volume_spec(spec: str) -> tuple[str, str, str]:
    """Parse a Docker volume spec: HOST, HOST:CONTAINER, or HOST:CONTAINER:MODE."""
    spec = spec.strip()
    if not spec:
        raise ValueError("empty volume spec")

    parts = spec.split(":")
    if len(parts) == 1:
        host = parts[0]
        return host, host, "rw"
    if len(parts) == 2:
        return parts[0], parts[1], "rw"
    if len(parts) == 3:
        return parts[0], parts[1], parts[2]
    raise ValueError(
        f"invalid volume spec '{spec}': expected HOST, HOST:CONTAINER, or HOST:CONTAINER:MODE"
    )


def data_dir_layout(volume_root: Path) -> list[Path]:
    """Subdirs under volume_root used by autonomy_lerobot/data_paths.yaml."""
    data = volume_root / "data"
    return [
        data,
        data / "collection",
        data / "lerobot",
        data / "lerobot" / "collection",
        data / "lerobot" / "habitat_nav2",
    ]


def _chown_path(path: Path, uid: int, gid: int) -> None:
    os.chown(path, uid, gid)
    if path.is_dir():
        for child in path.iterdir():
            if child.is_symlink():
                continue
            _chown_path(child, uid, gid)


def prepare_host_data_volume(host: str, container: str) -> None:
    """Create data subdirs on the host and set owner to the current user."""
    host_path = Path(host).resolve()
    container_path = (container or host).rstrip("/")

    if container_path.endswith("/data"):
        dirs = [
            host_path,
            host_path / "collection",
            host_path / "lerobot",
            host_path / "lerobot" / "collection",
            host_path / "lerobot" / "habitat_nav2",
        ]
    else:
        dirs = data_dir_layout(host_path)

    uid, gid = os.getuid(), os.getgid()
    user = os.environ.get("USER", str(uid))

    created: list[Path] = []
    for directory in dirs:
        try:
            directory.mkdir(parents=True, exist_ok=True)
            created.append(directory)
        except PermissionError:
            print_warning(
                f"无法创建 {directory}（父目录可能为 root/nobody 所有）。请在宿主机执行：\n"
                f"  sudo chown -R {user}:{user} {directory.parent}\n"
                f"  sudo chmod -R u+rwX {directory.parent}\n"
                f"  mkdir -p {directory}"
            )
            break

    targets = sorted({d for d in created if d.is_dir()}, key=lambda p: len(p.parts))
    for target in targets:
        try:
            _chown_path(target, uid, gid)
        except PermissionError:
            print_warning(
                f"无法修改 {target} 属主（当前可能为 root/nobody）。请在宿主机执行：\n"
                f"  sudo chown -R {user}:{user} {target}\n"
                f"  sudo chmod -R u+rwX {target}"
            )
            break


def resolve_data_volumes(cli_volumes: list[str] | None = None) -> list[tuple[str, str, str]]:
    """Resolve host data volumes to mount into the container.

    Priority:
    1. CLI ``--data-volume`` (when any value is passed)
    2. ``AUTONOMY_DATA_VOLUMES`` environment variable (comma- or space-separated)
    3. No data volumes
    """
    if cli_volumes is not None:
        specs = cli_volumes
    else:
        env_value = os.environ.get("AUTONOMY_DATA_VOLUMES", "").strip()
        if not env_value:
            return []
        specs = [part for part in env_value.replace(",", " ").split() if part.strip()]

    volumes = []
    for spec in specs:
        volumes.append(parse_volume_spec(spec))
    return volumes


def resolve_container_name() -> str:
    """Resolve Docker container name from ``AUTONOMY_CONTAINER_NAME``."""
    name = os.environ.get("AUTONOMY_CONTAINER_NAME", "SpaceHero").strip()
    return name or "SpaceHero"


def container_exists(name: str) -> bool:
    """Return True if a Docker container with ``name`` exists (any state)."""
    result = subprocess.run(
        ["docker", "container", "inspect", name],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.returncode == 0


def get_container_state(name: str) -> dict | None:
    """Return container state dict, or None if the container does not exist."""
    result = subprocess.run(
        [
            "docker", "inspect", name,
            "--format", "{{.State.Status}}|{{.State.Running}}|{{.State.Pid}}|{{.Id}}",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return None
    parts = result.stdout.strip().split("|", 3)
    if len(parts) != 4:
        return None
    status, running, pid_text, container_id = parts
    return {
        "status": status,
        "running": running.lower() == "true",
        "pid": int(pid_text) if pid_text.isdigit() else 0,
        "id": container_id[:12] if container_id else "",
    }


def container_responds(name: str, *, timeout_s: float = 8.0) -> bool:
    """Return True when ``docker exec`` into the container succeeds."""
    try:
        result = subprocess.run(
            ["docker", "exec", name, "echo", "ok"],
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0 and "ok" in result.stdout


def find_available_container_name(base: str, *, max_tries: int = 50) -> str:
    """Pick the first unused container name: base, base-2, base-3, ..."""
    if not container_exists(base):
        return base
    for index in range(2, max_tries + 2):
        candidate = f"{base}-{index}"
        if not container_exists(candidate):
            return candidate
    return f"{base}-{int(time.time())}"


def print_stuck_container_recovery(name: str) -> None:
    """Print manual recovery steps for Docker cgroup / shim stuck containers."""
    state = get_container_state(name) or {}
    pid = state.get("pid", 0)
    short_id = state.get("id", "")
    print_warning(
        f"Container {name} is stuck in Docker (cgroup freeze / kill timeout).\n"
        "Recovery options on the host:\n"
        f"  1) sudo systemctl restart docker && docker rm -f {name}\n"
        + (
            f"  2) sudo kill -9 {pid} $(pgrep -f 'containerd-shim.*{short_id}' | head -1)"
            f" && docker rm -f {name}\n"
            if pid
            else f"  2) docker rm -f {name}   # after restarting Docker\n"
        )
        + "Until then, autonomy will start a new container with a different name."
    )


def remove_container_if_exists(name: str, *, timeout_s: float = 120.0) -> bool:
    """Force-remove a container and wait until Docker no longer reports it.

    Escalates stop -> kill -> rm -f. Returns False when removal fails (e.g. cgroup
    freeze on heavy Isaac Sim workloads).
    """
    if not container_exists(name):
        return True

    print_info(f"Container {name} exists. Stopping and removing it...")
    state = get_container_state(name)
    if state and state["running"]:
        try:
            subprocess.run(
                ["docker", "stop", "-t", "15", name],
                capture_output=True,
                text=True,
                check=False,
                timeout=min(timeout_s, 30.0),
            )
        except subprocess.TimeoutExpired:
            print_warning(f"docker stop {name} timed out; trying docker kill...")
        if container_exists(name) and (get_container_state(name) or {}).get("running"):
            subprocess.run(
                ["docker", "kill", name],
                capture_output=True,
                text=True,
                check=False,
                timeout=30.0,
            )

    try:
        result = subprocess.run(
            ["docker", "rm", "-f", name],
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        print_error(f"Timed out while removing container {name}.")
        return False

    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        lowered = detail.lower()
        if detail:
            print_warning(
                f"docker rm -f {name} returned {result.returncode}: {detail}"
            )
        if "cgroup" in lowered or "exit event" in lowered or "freeze" in lowered:
            print_stuck_container_recovery(name)

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not container_exists(name):
            print_info(f"Container {name} has been stopped and removed.")
            return True
        time.sleep(0.25)

    print_error(f"Container {name} still exists after removal attempt.")
    return False


def resolve_publish_ports() -> list[str]:
    """Resolve port mappings from ``AUTONOMY_PORTS`` (comma- or space-separated)."""
    env_value = os.environ.get("AUTONOMY_PORTS", "8765:8765").strip()
    if not env_value:
        return []
    return [part for part in env_value.replace(",", " ").split() if part.strip()]


def resolve_network_mode() -> str:
    """Resolve Docker network mode from ``AUTONOMY_NETWORK``."""
    return os.environ.get("AUTONOMY_NETWORK", "host").strip() or "host"


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


def get_nvidia_driver_version() -> str | None:
    """Return the installed NVIDIA driver version, or None if unavailable."""
    if not check_nvidia_available():
        return None
    try:
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=driver_version",
                "--format=csv,noheader",
            ],
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return None
    if result.returncode != 0:
        return None
    version = result.stdout.strip().splitlines()
    return version[0].strip() if version else None


def is_isaac_sim_driver_supported(driver_version: str | None) -> bool:
    """Return True when the driver is within Isaac Sim 5.x validated range."""
    if not driver_version:
        return True
    major_text = driver_version.split(".", 1)[0]
    if not major_text.isdigit():
        return True
    return int(major_text) < 590


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


def ensure_git_submodules(repo_root: Path) -> None:
    """Initialize git submodules required for the CMake build."""
    autolink_cmake = repo_root / "autolink" / "CMakeLists.txt"
    if autolink_cmake.is_file():
        return
    if not (repo_root / ".gitmodules").is_file():
        return

    print_info("autolink submodule missing; running git submodule update --init --recursive autolink")
    git_env = {**os.environ, "GIT_HTTP_VERSION": "HTTP/1.1"}
    result = subprocess.run(
        ["git", "submodule", "update", "--init", "--recursive", "autolink"],
        cwd=repo_root,
        env=git_env,
        check=False,
    )
    if result.returncode != 0 or not autolink_cmake.is_file():
        print_error("Failed to initialize autolink submodule.")
        print_error(
            "From the repository root, run:\n"
            "  git submodule update --init --recursive autolink"
        )
        sys.exit(1)


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

