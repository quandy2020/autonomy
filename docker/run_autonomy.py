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

"""Run Autonomy Docker container with platform and GPU support.

Examples:
    # Auto-detect platform and prefer NVIDIA image if both exist
    python3 run_autonomy.py

    # Use x86_64 platform with auto image selection
    python3 run_autonomy.py -p x86_64

    # Force NVIDIA image on x86_64 (short form)
    python3 run_autonomy.py -p x86_64 -n yes

    # Force NVIDIA image on x86_64 (long form)
    python3 run_autonomy.py --platform x86_64 --nvidia yes

    # Force standard image on x86_64
    python3 run_autonomy.py -p x86_64 -n no

    # Use aarch64 platform
    python3 run_autonomy.py --platform aarch64

    # Pass additional arguments to docker run
    python3 run_autonomy.py -- --rm

    # NVIDIA / Isaac-Lab 衍生镜像：默认用 --entrypoint /bin/bash，避免继承镜像 ENTRYPOINT 自动拉起 Kit/流式

    # 旧行为（保留镜像自带 ENTRYPOINT，可能再次自动启动 Kit / 流式服务）— CLI：
    python3 run_autonomy.py -p x86_64 -n yes --keep-isaac-entrypoint
    python3 run_autonomy.py --platform x86_64 --nvidia yes --keep-isaac-entrypoint

    # 旧行为 — 环境变量（等价于 --keep-isaac-entrypoint）：
    AUTONOMY_KEEP_ISAAC_ENTRYPOINT=1 python3 run_autonomy.py -p x86_64 -n yes

    # 以宿主机 uid:gid 进入（bind mount 文件属主与宿主机一致，但 /thirdparty、colcon 可能遇权限问题）：
    python3 run_autonomy.py --as-host-user
    AUTONOMY_RUN_AS_HOST_USER=1 python3 run_autonomy.py

    # 挂载宿主机数据卷（环境变量，逗号或空格分隔多个路径）：
    AUTONOMY_DATA_VOLUMES=/mnt/data4t python3 run_autonomy.py

    # 挂载宿主机数据卷（CLI，可重复指定；指定后忽略环境变量）：
    python3 run_autonomy.py --data-volume /mnt/data4t
    python3 run_autonomy.py --data-volume /mnt/data4t:/mnt/data4t:rw

    # 使用 YAML 平台配置（config/ 目录）：
    python3 run_autonomy.py --list-configs
    python3 run_autonomy.py --platform nvidia --profile webrtc
    python3 run_autonomy.py -p x86_64 -n yes --profile navrl
"""

import argparse
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.absolute()
sys.path.insert(0, str(SCRIPT_DIR / "scripts"))
from docker_utils import (
    check_image_exists, get_image_platform, check_docker_available, 
    check_nvidia_available, check_docker_gpu_support,
    get_nvidia_driver_version, is_isaac_sim_driver_supported,
    normalize_path, resolve_autonomy_env_dir, resolve_data_volumes,
    prepare_host_data_volume,
    resolve_container_name, resolve_publish_ports, resolve_network_mode,
    remove_container_if_exists, get_container_state, container_responds,
    find_available_container_name, print_stuck_container_recovery,
    run_command, ensure_git_submodules,
    print_error, print_info, print_warning
)
from config_loader import (
    PLATFORMS_FILE,
    PlatformRunConfig,
    format_config_catalog,
    platform_id_for_image,
    resolve_platform_id,
)


class ColoredHelpFormatter(argparse.RawTextHelpFormatter):
    """Custom help formatter that supports ANSI color codes."""
    
    # ANSI color codes
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    RESET = '\033[0m'
    
    def _format_action_invocation(self, action):
        """Format action invocation with colors."""
        if not sys.stdout.isatty():
            return super()._format_action_invocation(action)
        
        # Colorize option strings
        parts = []
        for option_string in action.option_strings:
            if option_string.startswith('--'):
                parts.append(f"{self.GREEN}{option_string}{self.RESET}")
            else:
                parts.append(f"{self.GREEN}{option_string}{self.RESET}")
        return ', '.join(parts)
    
    def _format_usage(self, usage, actions, groups, prefix):
        """Format usage with colors."""
        if not sys.stdout.isatty():
            return super()._format_usage(usage, actions, groups, prefix)
        
        formatted = super()._format_usage(usage, actions, groups, prefix)
        # Colorize the usage line
        return formatted.replace('usage: ', f'{self.GREEN}usage:{self.RESET} ')


class AutonomyRunner:
    """Main class for running Autonomy Docker containers."""

    def __init__(self):
        self.root = SCRIPT_DIR
        self.base_name = "autonomy:latest"
        self.build_script = ""
        self.dockerfile = ""
        self.platform_arch = ""
        self.use_nvidia = "auto"
        self.docker_args = []
        self.remaining_args = []
        self.autonomy_dev_dir = str(resolve_autonomy_env_dir(SCRIPT_DIR))
        self.data_volumes: list[tuple[str, str, str]] = []
        self.container_name = resolve_container_name()
        self.publish_ports = resolve_publish_ports()
        self.network_mode = resolve_network_mode()
        self.keep_isaac_entrypoint = False
        self.use_gpu_in_container = False
        self.gpu_docker_strategies = []
        self.run_as_host_user = False
        self.host_uid = os.getuid()
        self.force_recreate = False
        self.requested_container_name = resolve_container_name()
        self.profile = 'default'
        self.platform_id = ''
        self.platform_config: PlatformRunConfig | None = None
        self.config_env: dict[str, str] = {}
        self.docker_platform: str | None = None

    def load_platform_config(self, platform_id: str, profile: str = 'default') -> None:
        """Load platform + profile from config/platforms.yaml."""
        self.platform_config = PlatformRunConfig.load(platform_id, profile=profile)
        self.platform_id = platform_id
        self.profile = profile
        cfg = self.platform_config

        self.base_name = cfg.image_name
        if cfg.build_script:
            self.build_script = str(self.root / cfg.build_script)
        if cfg.dockerfile:
            self.dockerfile = cfg.dockerfile

        if not os.environ.get('AUTONOMY_CONTAINER_NAME', '').strip():
            self.container_name = cfg.container_name
            self.requested_container_name = cfg.container_name
        if not os.environ.get('AUTONOMY_PORTS', '').strip():
            self.publish_ports = list(cfg.ports)
        if not os.environ.get('AUTONOMY_NETWORK', '').strip():
            self.network_mode = 'host'

        self.config_env = dict(cfg.environment)
        self.docker_platform = cfg.docker_platform
        self.platform_arch = cfg.arch

        if cfg.keep_entrypoint:
            self.keep_isaac_entrypoint = True

        print_info(f"Platform: {platform_id}  profile={profile}  image={cfg.image_name}")

    def apply_config_gpu_preference(self) -> None:
        """Use gpu flag from YAML when CLI nvidia mode is auto."""
        if self.use_nvidia != 'auto' or not self.platform_config:
            return
        if self.platform_config.gpu:
            self.use_nvidia = 'yes'
            print_info('GPU enabled from platform config')
        else:
            self.use_nvidia = 'no'

    @staticmethod
    def _user_specified_entrypoint(remaining_args):
        for a in remaining_args:
            if a == "--entrypoint" or a.startswith("--entrypoint="):
                return True
        return False

    def _use_plain_shell_for_nvidia_image(self):
        """Isaac-Lab 基础镜像自带 ENTRYPOINT；覆盖为 /bin/bash 可避免容器启动即跑仿真/GUI。"""
        if self.platform_config and self.platform_config.keep_entrypoint:
            return False
        if "nvidia" not in self.base_name:
            return False
        if self.keep_isaac_entrypoint:
            return False
        if os.environ.get("AUTONOMY_KEEP_ISAAC_ENTRYPOINT", "").strip() == "1":
            return False
        return True

    def setup_x11(self):
        """Configure X11 server permissions."""
        if sys.platform == "linux" and shutil.which("xhost"):
            uid = os.getuid()
            for cmd in [
                ["xhost", f"+local:{uid}"],
                ["xhost", "+local:docker"],
                ["xhost", "+local:root"],
            ]:
                subprocess.run(cmd, check=False, capture_output=True)

    def select_x86_64_image(self):
        """Select appropriate x86_64 Docker image."""
        self.build_script = str(self.root / "build_docker_x86_64.py")
        standard = "autonomy.platform.x86_64"
        nvidia = "autonomy.platform.x86_64.nvidia"
        standard_exists = check_image_exists(standard)
        nvidia_exists = check_image_exists(nvidia)

        if self.use_nvidia == "yes":
            self.base_name = nvidia
            self.dockerfile = "dockerfile/autonomy.x86_64.nvidia.dockerfile"
            print_info("Using NVIDIA version (forced by --nvidia yes).")
        elif self.use_nvidia == "no":
            self.base_name = standard
            self.dockerfile = "dockerfile/autonomy.x86_64.dockerfile"
            print_info("Using standard x86_64 version (forced by --nvidia no).")
        else:  # auto
            if nvidia_exists and standard_exists:
                self.base_name = nvidia
                self.dockerfile = "dockerfile/autonomy.x86_64.nvidia.dockerfile"
                print_info("Both images exist, using NVIDIA version (preferred).")
            elif nvidia_exists:
                self.base_name = nvidia
                self.dockerfile = "dockerfile/autonomy.x86_64.nvidia.dockerfile"
                print_info("Using NVIDIA version (only NVIDIA image available).")
            elif standard_exists:
                self.base_name = standard
                self.dockerfile = "dockerfile/autonomy.x86_64.dockerfile"
                print_info("Using standard x86_64 version (only standard image available).")
            else:
                self.base_name = standard
                self.dockerfile = "dockerfile/autonomy.x86_64.dockerfile"
                print_info("No images found, will build standard x86_64 version.")

    def show_interactive_menu(self):
        """Show interactive menu for platform selection."""
        if not sys.stdin.isatty():
            # Not an interactive terminal, skip menu
            return False
        
        print("\n" + "=" * 60)
        print("  Autonomy Docker Container Platform Selection")
        print("=" * 60)
        print("  1. x86_64 (ROS2-Humble, Standard)")
        print("  2. x86_64 (ROS2-Humble, NVIDIA GPU, Isaac-Sim/Isaac-lab)")
        print("  3. aarch64 (ROS2-Humble, Standard)")
        print("=" * 60)
        
        while True:
            try:
                choice = input("\nPlease select the platform (1/2/3): ").strip()
                if choice == "1":
                    self.platform_arch = "x86_64"
                    self.use_nvidia = "no"
                    print_info("Selected: x86_64 (Standard)")
                    return True
                elif choice == "2":
                    self.platform_arch = "x86_64"
                    self.use_nvidia = "yes"
                    print_info("Selected: x86_64 (NVIDIA GPU, Isaac-Sim/Isaac-lab)")
                    return True
                elif choice == "3":
                    self.platform_arch = "aarch64"
                    self.use_nvidia = "auto"
                    print_info("Selected: aarch64")
                    return True
                else:
                    print_warning(f"Invalid choice: {choice}. Please enter 1, 2, or 3.")
            except (EOFError, KeyboardInterrupt):
                print_info("\nSelection cancelled.")
                sys.exit(0)
            except Exception as e:
                print_error(f"输入错误: {e}")

    def check_available_images(self):
        """Check which Docker images are available."""
        images = {
            "x86_64_standard": check_image_exists("autonomy.platform.x86_64"),
            "x86_64_nvidia": check_image_exists("autonomy.platform.x86_64.nvidia"),
            "aarch64": check_image_exists("autonomy.platform.aarch64")
        }
        return images

    def detect_platform(self):
        """Detect or use specified platform architecture."""
        # Check available images
        available_images = self.check_available_images()
        has_any_image = any(available_images.values())
        
        if not self.platform_arch:
            if has_any_image:
                # At least one image exists, use auto-detection or command line args
                self.platform_arch = platform.machine()
                print_info(f"Auto-detected platform: {self.platform_arch}")
                if self.platform_arch not in ("x86_64", "aarch64", "arm64"):
                    print_warning(f"Detected platform {self.platform_arch} not supported, defaulting to x86_64")
                    self.platform_arch = "x86_64"
            else:
                # No images exist, show interactive menu for selection
                print_info("No Docker images found. Please select a platform to build:")
                if not self.show_interactive_menu():
                    # Not an interactive terminal, use auto-detection
                    self.platform_arch = platform.machine()
                    print_info(f"Auto-detected platform: {self.platform_arch}")
                    if self.platform_arch not in ("x86_64", "aarch64", "arm64"):
                        print_warning(f"Detected platform {self.platform_arch} not supported, defaulting to x86_64")
                        self.platform_arch = "x86_64"

        if self.platform_arch == "x86_64":
            print_info("Using 64-bit x86 architecture.")
            self.select_x86_64_image()
        elif self.platform_arch in ("aarch64", "arm64"):
            print_info("Using 64-bit ARM architecture.")
            self.base_name = "autonomy.platform.aarch64"
            self.build_script = str(self.root / "build_docker_aarch64.py")
            self.dockerfile = "dockerfile/autonomy.aarch64.dockerfile"
        else:
            print_error(f"Unsupported platform: {self.platform_arch}")
            print_error("Supported: x86_64, aarch64, arm64")
            sys.exit(1)

    def check_requirements(self):
        """Validate system requirements."""
        if os.geteuid() == 0:
            print_error("Cannot run as root. Configure Docker for non-root user.")
            sys.exit(1)
        if not check_docker_available():
            print_error("Docker not available. Check installation.")
            sys.exit(1)

    def build_image_if_needed(self):
        """Build Docker image if it doesn't exist."""
        if check_image_exists(self.base_name):
            return

        print_info("Build conditions not met, starting Docker image build...")
        build_script = Path(self.build_script)
        dockerfile = self.root / self.dockerfile

        if not build_script.exists():
            print_error(f"Build script not found: {build_script}")
            sys.exit(1)
        if not dockerfile.exists():
            print_error(f"Dockerfile not found: {dockerfile}")
            sys.exit(1)

        os.chdir(self.root)
        if self.platform_config and self.platform_config.build_nvidia:
            cmd = ["python3", str(build_script), "-n"]
        elif "nvidia" in self.base_name:
            cmd = ["python3", str(build_script), "-n"]
        else:
            cmd = ["python3", str(build_script), "-f", self.dockerfile]
        if run_command(cmd, check=False).returncode != 0:
            print_error("Docker image build failed")
            sys.exit(1)

    def configure_display(self):
        """Configure X11 display forwarding."""
        display = os.environ.get("DISPLAY", ":0")
        print_info(f"Using DISPLAY: {display}")
        self.docker_args.extend(["-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw", "-e", f"DISPLAY={display}"])
        xauth = Path.home() / ".Xauthority"
        if xauth.exists():
            container_xauth = f"/tmp/.Xauthority-{self.host_uid}"
            self.docker_args.extend([
                "-v", f"{xauth}:{container_xauth}:rw",
                "-e", f"XAUTHORITY={container_xauth}",
            ])

    def configure_environment(self):
        """Configure environment variables from platform config."""
        for key, value in self.config_env.items():
            self.docker_args.extend(["-e", f"{key}={value}"])
        runtime_dir = f"/tmp/runtime-{self.host_uid}"
        self.docker_args.extend(["-e", f"XDG_RUNTIME_DIR={runtime_dir}"])
        xdg = os.environ.get("XDG_RUNTIME_DIR")
        if xdg and Path(xdg).exists():
            self.docker_args.extend(["-v", f"{xdg}:{xdg}:ro"])

    def configure_container_user(self):
        """Run as root by default; optional host uid:gid for bind-mount ownership."""
        if not self.run_as_host_user:
            print_info("Container runs as root")
            return
        uid, gid = os.getuid(), os.getgid()
        user = os.environ.get("USER", "user")
        self.docker_args.extend(["-u", f"{uid}:{gid}"])
        self.docker_args.extend(["-e", f"USER={user}", "-e", f"HOME=/tmp/home-{uid}"])
        print_info(f"Container runs as host user {user} ({uid}:{gid})")

    def _configure_software_rendering(self):
        """Configure software rendering when GPU passthrough is unavailable."""
        print_info("Using software rendering (Mesa llvmpipe)")
        self.docker_args.extend([
            "-e", "LIBGL_ALWAYS_SOFTWARE=1",
            "-e", "AUTOVIZ_SOFTWARE_GL=1",
            "-e", "__GLX_VENDOR_LIBRARY_NAME=mesa",
            "-e", "QT_OPENGL=software",
            "-e", "GALLIUM_DRIVER=llvmpipe",
            "-e", "MESA_GL_VERSION_OVERRIDE=3.3",
            "-e", "MESA_GLSL_VERSION_OVERRIDE=330",
            "-e", "LIBGL_DRI3_DISABLE=1",
        ])

    def configure_gpu(self):
        """Configure GPU support."""
        nvidia_driver = check_nvidia_available()
        docker_gpu = check_docker_gpu_support()
        wants_gpu = (
            self.use_nvidia == "yes" or
            (self.use_nvidia == "auto" and nvidia_driver)
        )

        if wants_gpu and docker_gpu:
            print_info("Enabling NVIDIA GPU support")
            driver_version = get_nvidia_driver_version()
            if driver_version and not is_isaac_sim_driver_supported(driver_version):
                print_warning(
                    f"NVIDIA driver {driver_version} is newer than Isaac Sim 5.x "
                    "validated range (~580.x)."
                )
                print_warning(
                    "Isaac Lab RGBD training may segfault in librtx.scenedb.plugin.so. "
                    "Install nvidia-driver-580-open on the host and reboot."
                )
            self.use_gpu_in_container = True
            self.gpu_docker_strategies = [
                (
                    ["--gpus", "all", "-e", "NVIDIA_VISIBLE_DEVICES=all",
                     "-e", "NVIDIA_DRIVER_CAPABILITIES=all"],
                    "--gpus all",
                ),
                (
                    ["--device", "nvidia.com/gpu=all", "-e", "NVIDIA_VISIBLE_DEVICES=all",
                     "-e", "NVIDIA_DRIVER_CAPABILITIES=all"],
                    "CDI device nvidia.com/gpu=all",
                ),
                (
                    ["--runtime", "nvidia", "-e", "NVIDIA_VISIBLE_DEVICES=all",
                     "-e", "NVIDIA_DRIVER_CAPABILITIES=all"],
                    "nvidia container runtime",
                ),
            ]
            return

        if wants_gpu and nvidia_driver and not docker_gpu:
            print_warning(
                "NVIDIA driver is available, but Docker GPU/CDI is not configured."
            )
            print_warning(
                "Configure it with: bash docker/install/install_nvidia_container_toolkit.sh"
            )
            print_warning("Starting container without GPU passthrough.")
        elif wants_gpu and not nvidia_driver:
            print_warning("NVIDIA GPU requested but nvidia-smi not found.")

        self.use_gpu_in_container = False
        self._configure_software_rendering()

    def configure_gui_gl(self):
        """Configure OpenGL/GLX for RViz and autoviz inside the container."""
        check_script = "src/autonomy/autoviz/scripts/check_gui_env.sh"
        if self.use_gpu_in_container:
            self.docker_args.extend([
                "-e", "AUTOVIZ_USE_HARDWARE_GL=1",
                "-e", "AUTOVIZ_ALLOW_ROOT_GUI=1",
                "-e", "__GLX_VENDOR_LIBRARY_NAME=nvidia",
            ])
            print_info(
                "GUI GLX: NVIDIA hardware GL enabled for RViz/autoviz (root in container)."
            )
            print_info(
                f"If GUI still fails, run: bash {check_script} "
                "or restart with --as-host-user."
            )
            return
        print_info(
            f"GUI GLX: software rendering (Mesa). Verify with: bash {check_script}"
        )

    def configure_shared_volumes(self):
        """Mount configured host data volumes into the container."""
        if not self.data_volumes:
            return

        for host, container, mode in self.data_volumes:
            host_path = Path(host)
            if not host_path.is_dir():
                print_warning(
                    f"Host path {host} not found; skipping volume {host}:{container}:{mode}"
                )
                continue
            prepare_host_data_volume(host, container)
            self.docker_args.extend(["-v", f"{host}:{container}:{mode}"])
            print_info(f"Mounting host data volume: {host} -> {container} ({mode})")

    def configure_network(self):
        """Configure network settings."""
        image_exists = check_image_exists(self.base_name)
        platform_flag = self.docker_platform

        if platform_flag is None and self.platform_arch in ("aarch64", "arm64"):
            platform_flag = "linux/arm64"

        if platform_flag:
            if image_exists:
                image_platform = get_image_platform(self.base_name)
                if image_platform and platform_flag.endswith(image_platform):
                    pass
                elif image_platform and image_platform not in ("arm64", "aarch64") and platform_flag == "linux/arm64":
                    print_warning(
                        f"Local image '{self.base_name}' is {image_platform}, "
                        f"not arm64. Using image as-is without --platform flag."
                    )
                    platform_flag = None
            if platform_flag:
                self.docker_args.extend(["--platform", platform_flag])
        
        if sys.platform != "darwin" and self.network_mode:
            self.docker_args.append(f"--net={self.network_mode}")

    def _collect_exec_env_args(self) -> list[str]:
        """Extract -e KEY=VAL pairs from docker run args for docker exec."""
        env_args: list[str] = []
        index = 0
        while index < len(self.docker_args):
            if self.docker_args[index] == "-e" and index + 1 < len(self.docker_args):
                env_args.extend(["-e", self.docker_args[index + 1]])
                index += 2
            else:
                index += 1
        return env_args

    def _exec_existing_container(self) -> None:
        """Attach an interactive shell to an already running container."""
        cmd = ["docker", "exec"]
        if sys.stdin.isatty():
            cmd.append("-it")
        else:
            cmd.append("-i")
        cmd.extend(self._collect_exec_env_args())
        cmd.extend(["-w", "/workspace/autonomy", self.container_name, "/bin/bash"])
        print_info(f"Attaching to running container {self.container_name} ...")
        result = subprocess.run(cmd, check=False)
        sys.exit(result.returncode)

    def _use_fallback_container_name(self, reason: str) -> None:
        """Switch to an unused container name when the requested one is stuck."""
        fallback = find_available_container_name(self.requested_container_name)
        if fallback != self.container_name:
            print_warning(f"{reason} Using fallback container name: {fallback}")
            self.container_name = fallback

    def prepare_container(self) -> None:
        """Reuse, remove, or replace the target container before docker run."""
        state = get_container_state(self.container_name)
        if state is None:
            return

        if state["running"] and not self.force_recreate:
            if container_responds(self.container_name):
                self._exec_existing_container()
            print_warning(
                f"Container {self.container_name} is running but not responding "
                "(likely cgroup freeze after Isaac Sim / GPU load)."
            )
            self._use_fallback_container_name(
                f"Cannot attach to stuck container {self.container_name}."
            )
            return

        if not remove_container_if_exists(self.container_name):
            self._use_fallback_container_name(
                f"Could not remove existing container {self.container_name}."
            )

    def _ports_for_docker_run(self) -> list[str]:
        """Return -p mappings; skip when host network (Docker ignores them anyway)."""
        if self.network_mode == 'host':
            return []
        return list(self.publish_ports)

    def _log_host_network_ports(self) -> None:
        """Explain WebRTC/service ports when using --net=host."""
        if self.network_mode != 'host' or not self.publish_ports:
            return
        service_ports = []
        for spec in self.publish_ports:
            host_part = spec.split('/')[0].split(':')[0]
            if host_part.isdigit():
                service_ports.append(host_part)
        if not service_ports:
            return
        ports_text = ', '.join(sorted(set(service_ports)))
        print_info(
            f"network=host：服务端口 {ports_text} 直接在宿主机监听，无需 -p 映射。"
        )
        if '49100' in service_ports or any('49100' in p for p in self.publish_ports):
            print_info(
                "WebRTC：宿主机安装 Isaac Sim Streaming Client，连接 127.0.0.1:49100 "
                "(LIVESTREAM=2 局域网模式)。"
            )

    def container_exist(self):
        """Backward-compatible alias for GPU retry cleanup."""
        if not remove_container_if_exists(self.container_name):
            self._use_fallback_container_name(
                f"Could not remove container {self.container_name} after docker run failure."
            )

    def run(self):
        """Main execution function."""
        self.setup_x11()
        if self.platform_config and not self.platform_arch:
            self.platform_arch = self.platform_config.arch
        print_info(f"Running {self.base_name}")
        self.check_requirements()
        ensure_git_submodules(Path(self.autonomy_dev_dir))
        self.build_image_if_needed()
        self.configure_display()
        self.configure_environment()
        self.configure_container_user()
        self.configure_shared_volumes()
        self.configure_network()
        self.configure_gpu()
        self.configure_gui_gl()

        print_info(f"Mounting host path as /workspace/autonomy: {self.autonomy_dev_dir}")

        print_info(f"Starting docker container: {self.container_name} ...")
        self._log_host_network_ports()
        self.prepare_container()
        print_info(f"Running container: {self.base_name}")

        try:
            result = self._run_docker_container()
            if result.returncode == 0:
                print_info("Docker run completed")
            else:
                sys.exit(result.returncode)
        except KeyboardInterrupt:
            print_info("\nDocker run interrupted")
            sys.exit(0)

    def _build_docker_run_cmd(self, gpu_args=None):
        """Build the docker run command."""
        use_local_only = check_image_exists(self.base_name)
        cmd = [
            "docker", "run", "-it" if sys.stdin.isatty() else "-i",
            "--name", self.container_name,
            *self.docker_args,
        ]
        for port in self._ports_for_docker_run():
            cmd.extend(["-p", port])
        if gpu_args:
            cmd.extend(gpu_args)
        cmd.extend([
            "-v", f"{self.autonomy_dev_dir}:/workspace/autonomy",
            "-v", "/dev:/dev", "-v", "/etc/localtime:/etc/localtime:ro",
            "--workdir", "/workspace/autonomy",
            "--privileged",
        ])
        if use_local_only:
            cmd.append("--pull=never")

        use_plain_shell = self._use_plain_shell_for_nvidia_image()
        user_entrypoint = self._user_specified_entrypoint(self.remaining_args)
        if use_plain_shell and not user_entrypoint:
            print_info(
                "NVIDIA 镜像：使用 --entrypoint /bin/bash。"
                "保留 Isaac ENTRYPOINT 请加 --profile isaac-kit 或 --keep-isaac-entrypoint。"
            )
            cmd.extend(["--entrypoint", "/bin/bash"])
        cmd.extend([*self.remaining_args, self.base_name])
        if not (use_plain_shell and not user_entrypoint):
            cmd.append("/bin/bash")
        return cmd

    def _run_docker_container(self):
        """Run the container, trying GPU strategies before falling back."""
        strategies = self.gpu_docker_strategies if self.use_gpu_in_container else [(None, "no GPU")]
        last_result = None

        for gpu_args, label in strategies:
            if gpu_args and label != "no GPU":
                print_info(f"Trying Docker GPU mode: {label}")
            cmd = self._build_docker_run_cmd(gpu_args)
            result = subprocess.run(cmd, check=False, capture_output=not sys.stdin.isatty(), text=True)
            last_result = result
            if result.returncode == 0:
                return result

            if gpu_args:
                output = f"{result.stderr or ''}\n{result.stdout or ''}"
                detail = output.strip() or f"exit code {result.returncode}"
                print_warning(f"Docker GPU mode failed ({label}): {detail}")
                self.container_exist()
                continue

            print_error(f"Docker run failed with exit code {result.returncode}.")
            if result.stderr:
                print_error(result.stderr.strip())
            return result

        print_warning("All Docker GPU modes failed; retrying without GPU passthrough.")
        self.container_exist()
        self.use_gpu_in_container = False
        fallback_cmd = self._build_docker_run_cmd()
        fallback_cmd.extend(["-e", "LIBGL_ALWAYS_SOFTWARE=1"])
        last_result = subprocess.run(
            fallback_cmd, check=False, capture_output=not sys.stdin.isatty(), text=True
        )
        if last_result.returncode != 0:
            print_error(f"Docker run failed with exit code {last_result.returncode}.")
            if last_result.stderr:
                print_error(last_result.stderr.strip())
        return last_result


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Run Autonomy Docker container with platform and GPU support",
        formatter_class=ColoredHelpFormatter,
        epilog="""
Examples:
  \033[91m# Auto-detect platform\033[0m
  \033[92mpython3 %(prog)s\033[0m
  \033[92mpython3 %(prog)s --platform x86_64\033[0m

  \033[91m# Use x86_64 platform (short and long form)\033[0m
  \033[92mpython3 %(prog)s -p x86_64\033[0m
  \033[92mpython3 %(prog)s --platform x86_64\033[0m

  \033[91m# Force NVIDIA image (short and long form)\033[0m
  \033[92mpython3 %(prog)s -p x86_64 -n yes\033[0m
  \033[92mpython3 %(prog)s --platform x86_64 --nvidia yes\033[0m

  \033[91m# Force standard image (short and long form)\033[0m
  \033[92mpython3 %(prog)s -p x86_64 -n no\033[0m
  \033[92mpython3 %(prog)s --platform x86_64 --nvidia no\033[0m

  \033[91m# Auto-select image (short and long form)\033[0m
  \033[92mpython3 %(prog)s -p x86_64 -n auto\033[0m
  \033[92mpython3 %(prog)s --platform x86_64 --nvidia auto\033[0m

  \033[91m# Use aarch64 platform (short and long form)\033[0m
  \033[92mpython3 %(prog)s -p aarch64\033[0m
  \033[92mpython3 %(prog)s --platform aarch64\033[0m

  \033[91m# Mixed short and long form\033[0m
  \033[92mpython3 %(prog)s -p x86_64 --nvidia yes\033[0m
  \033[92mpython3 %(prog)s --platform x86_64 -n yes\033[0m

  \033[91m# Pass additional arguments to docker run\033[0m
  \033[92mpython3 %(prog)s -- --rm\033[0m
  \033[93mpython3 %(prog)s -p x86_64 -- --detach\033[0m

  \033[91m# NVIDIA 镜像：保留 Isaac-Lab 默认 ENTRYPOINT（可能自动启动 Kit/流式）\033[0m
  \033[92mpython3 %(prog)s -p x86_64 -n yes --keep-isaac-entrypoint\033[0m

  \033[91m# 以宿主机用户进入（bind mount 属主一致；colcon/第三方库安装可能需额外处理权限）\033[0m
  \033[92mpython3 %(prog)s --as-host-user\033[0m
  \033[92mAUTONOMY_RUN_AS_HOST_USER=1 python3 %(prog)s\033[0m

  \033[91m# 挂载宿主机数据卷\033[0m
  \033[92mAUTONOMY_DATA_VOLUMES=/mnt/data4t python3 %(prog)s\033[0m
  \033[92mpython3 %(prog)s --data-volume /mnt/data4t\033[0m
  \033[92mpython3 %(prog)s --data-volume /mnt/data4t:/mnt/data4t:rw\033[0m

  \033[91m# YAML 平台配置（config/ 目录）\033[0m
  \033[92mpython3 %(prog)s --list-configs\033[0m
  \033[92mpython3 %(prog)s --platform nvidia --profile webrtc\033[0m
  \033[92mpython3 %(prog)s -p x86_64 -n yes --profile navrl\033[0m
        """
    )
    parser.add_argument(
        "--profile",
        default="default",
        metavar="NAME",
        help="场景 profile：webrtc | navrl | isaac-kit（见 config/platforms.yaml）",
    )
    parser.add_argument(
        "--list-configs",
        action="store_true",
        help="列出 platforms.yaml 中的平台与 profile，然后退出。",
    )
    parser.add_argument(
        "-p", "--platform",
        metavar="ID",
        help=(
            "平台 ID 或架构：\n"
            "  nvidia   — Isaac Lab / GPU（推荐训练）\n"
            "  x86_64   — 标准 x86\n"
            "  aarch64  — ARM64\n"
            "也可写 x86_64 + -n yes/no 组合选择镜像。"
        ),
    )
    parser.add_argument(
        "-n", "--nvidia",
        choices=["auto", "yes", "no"],
        default="auto",
        metavar="MODE",
        help="NVIDIA GPU mode: auto (default), yes, or no.\n"
             "  \033[91mauto: Auto-select image (prefers NVIDIA if both exist)\033[0m\n"
             "  \033[91myes:  Force use NVIDIA image\033[0m\n"
             "  \033[91mno:   Force use standard image\033[0m"
    )
    parser.add_argument(
        "--force-recreate",
        action="store_true",
        help=(
            "Always stop/remove the named container and create a new one. "
            "If removal fails (cgroup stuck), fall back to SpaceHero-2, etc. "
            "Equivalent env: AUTONOMY_FORCE_RECREATE=1."
        ),
    )
    parser.add_argument(
        "--as-host-user",
        action="store_true",
        help=(
            "以宿主机 uid:gid 进入容器（默认 root）。"
            "适用于希望 workspace 内新建文件在宿主机上属主为自己；"
            "colcon build、install_*.sh 等可能需要 root 或额外权限处理。"
            "等价环境变量：AUTONOMY_RUN_AS_HOST_USER=1。"
        ),
    )
    parser.add_argument(
        "--keep-isaac-entrypoint",
        action="store_true",
        help=(
            "仅对 NVIDIA / Isaac-Lab 衍生镜像有效：不要改用 /bin/bash 覆盖 ENTRYPOINT，"
            "与旧版 docker run 行为一致（可能随镜像逻辑自动启动 Omniverse Kit / 流式服务）。"
            "也可设置环境变量 AUTONOMY_KEEP_ISAAC_ENTRYPOINT=1。"
        ),
    )
    parser.add_argument(
        "--data-volume",
        action="append",
        dest="data_volumes",
        metavar="SPEC",
        help=(
            "宿主机数据卷，格式 HOST、HOST:CONTAINER 或 HOST:CONTAINER:MODE（可重复）。"
            "指定后优先于环境变量 AUTONOMY_DATA_VOLUMES。"
        ),
    )
    return parser.parse_known_args()


def resolve_force_recreate(cli_force_recreate: bool) -> bool:
    """Return True when the user wants a fresh container instead of attach/reuse."""
    if cli_force_recreate:
        return True
    return os.environ.get("AUTONOMY_FORCE_RECREATE", "").strip().lower() in (
        "1", "true", "yes",
    )


def resolve_run_as_host_user(cli_as_host_user: bool) -> bool:
    """Default root; opt in to host uid:gid via CLI or AUTONOMY_RUN_AS_HOST_USER."""
    if cli_as_host_user:
        return True
    return os.environ.get("AUTONOMY_RUN_AS_HOST_USER", "").strip().lower() in (
        "1", "true", "yes",
    )


def normalize_platform_arg(value: str) -> str:
    """Normalize CLI platform token to platform id or arch."""
    token = value.strip().lower()
    if token in ('nvidia', 'x86_64', 'aarch64'):
        return token
    if token in ('arm64', 'aarch64(arm64)'):
        return 'aarch64'
    if token in ('amd64', 'x64'):
        return 'x86_64'
    return token


def resolve_runner_platform(runner: AutonomyRunner, platform_arg: str | None) -> str:
    """Set runner arch/gpu flags and return platform id for platforms.yaml."""
    if platform_arg == 'nvidia':
        runner.platform_arch = 'x86_64'
        runner.use_nvidia = 'yes'
        return 'nvidia'
    if platform_arg == 'aarch64':
        runner.platform_arch = 'aarch64'
        return 'aarch64'
    if platform_arg == 'x86_64':
        runner.platform_arch = 'x86_64'
        if runner.use_nvidia == 'yes':
            return 'nvidia'
        if runner.use_nvidia == 'no':
            return 'x86_64'
        runner.detect_platform()
        return platform_id_for_image(runner.base_name)

    runner.detect_platform()
    runner.apply_config_gpu_preference()
    return resolve_platform_id(runner.platform_arch, runner.use_nvidia)


def main():
    """Script entry point."""
    args, remaining = parse_arguments()

    if args.list_configs:
        print(format_config_catalog())
        return

    runner = AutonomyRunner()
    runner.profile = args.profile or 'default'
    runner.use_nvidia = args.nvidia
    runner.keep_isaac_entrypoint = bool(args.keep_isaac_entrypoint)
    runner.force_recreate = resolve_force_recreate(bool(args.force_recreate))
    runner.data_volumes = resolve_data_volumes(args.data_volumes)
    runner.run_as_host_user = resolve_run_as_host_user(bool(args.as_host_user))
    runner.remaining_args = remaining

    platform_arg = normalize_platform_arg(args.platform) if args.platform else None
    try:
        platform_id = resolve_runner_platform(runner, platform_arg)
        runner.load_platform_config(platform_id, runner.profile)
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print_error(str(exc))
        sys.exit(1)

    runner.run()


if __name__ == "__main__":
    main()
