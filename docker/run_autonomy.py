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
    check_nvidia_available, normalize_path, run_command, 
    print_error, print_info, print_warning
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
        self.autonomy_dev_dir = os.environ.get("AUTONOMY_ENV", os.getcwd())

    def setup_x11(self):
        """Configure X11 server permissions."""
        if sys.platform == "linux" and shutil.which("xhost"):
            for cmd in [["xhost", "+local:root"], ["xhost", "+local:docker"], ["xhost", "+"]]:
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
        if "nvidia" in self.base_name:
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
            self.docker_args.extend(["-v", f"{xauth}:/root/.Xauthority:rw", "-e", "XAUTHORITY=/root/.Xauthority"])

    def configure_environment(self):
        """Configure environment variables."""
        self.docker_args.extend(["-e", "QT_X11_NO_MITSHM=1", "-e", "AUTONOMY_DEV_DIR=/workspace/autonomy"])
        if self.platform_arch == "x86_64":
            self.docker_args.extend(["-e", "ACCEPT_EULA=Y"])
            print_info("NVIDIA Isaac Sim EULA accepted (force enabled on x86_64)")
        xdg = os.environ.get("XDG_RUNTIME_DIR")
        self.docker_args.extend(["-e", "XDG_RUNTIME_DIR=/tmp/runtime-root"])
        if xdg and Path(xdg).exists():
            self.docker_args.extend(["-v", f"{xdg}:{xdg}:ro"])

    def configure_gpu(self):
        """Configure GPU support."""
        nvidia_available = check_nvidia_available()
        should_use_nvidia = (
            self.use_nvidia == "yes" or
            (self.use_nvidia == "auto" and nvidia_available)
        )

        if self.use_nvidia == "yes" and not nvidia_available:
            print_warning("NVIDIA GPU forced but nvidia-smi not found. GPU support may not work.")

        if should_use_nvidia:
            print_info("Enabling NVIDIA GPU support")
            self.docker_args.extend(["--gpus", "all", "-e", "NVIDIA_VISIBLE_DEVICES=all", "-e", "NVIDIA_DRIVER_CAPABILITIES=all"])
        else:
            print_info("Using software rendering (Mesa)")
            if sys.platform == "linux" and Path("/dev/dri").exists():
                self.docker_args.append("--device=/dev/dri")
            self.docker_args.append("-e")
            self.docker_args.append("LIBGL_ALWAYS_SOFTWARE=1")

    def configure_network(self):
        """Configure network settings."""
        # Only specify platform if image doesn't exist locally or if we're on macOS
        # On Linux with existing image, let Docker use the image's native platform
        image_exists = check_image_exists(self.base_name)
        
        if self.platform_arch in ("aarch64", "arm64"):
            # If image exists locally, check its actual platform
            if image_exists:
                image_platform = get_image_platform(self.base_name)
                if image_platform and image_platform not in ("arm64", "aarch64"):
                    # Image exists but is not arm64, skip platform specification
                    print_warning(f"Local image '{self.base_name}' is {image_platform}, "
                                f"not arm64. Using image as-is without --platform flag.")
                else:
                    # Image is arm64 or platform unknown, use platform flag
                    self.docker_args.extend(["--platform", "linux/arm64"])
            else:
                # Image doesn't exist, will be pulled - specify platform
                self.docker_args.extend(["--platform", "linux/arm64"])
        
        if sys.platform != "darwin":
            self.docker_args.append("--net=host")

    def container_exist(self):
        """Check and remove existing SpaceHero container."""
        result = subprocess.run(["docker", "ps", "-a", "--format", "{{.Names}}"], capture_output=True, text=True, check=False)
        if "SpaceHero" in result.stdout:
            print_info("Container SpaceHero exists. Stopping and removing it...")
            subprocess.run(["docker", "stop", "SpaceHero"], capture_output=True, check=False)
            subprocess.run(["docker", "rm", "SpaceHero"], capture_output=True, check=False)
            print_info("Container /SpaceHero has been stopped and removed.")

    def run(self):
        """Main execution function."""
        self.setup_x11()
        self.detect_platform()
        print_info(f"Running {self.base_name}")
        self.check_requirements()
        self.build_image_if_needed()
        self.configure_display()
        self.configure_environment()
        self.configure_network()
        self.configure_gpu()

        print_info("Starting docker container: SpaceHero ...")
        self.container_exist()
        print_info(f"Running container: {self.base_name}")

        # Check if image exists locally, if yes use --pull=never to avoid remote pull
        use_local_only = check_image_exists(self.base_name)
        cmd = [
            "docker", "run", "-it" if sys.stdin.isatty() else "-i",
            "--name", "SpaceHero", "-p", "8765:8765",
            *self.docker_args,
            "-v", f"{self.autonomy_dev_dir}:/workspace/autonomy",
            "-v", "/dev:/dev", "-v", "/etc/localtime:/etc/localtime:ro",
            "--workdir", "/workspace/autonomy", 
            "--privileged",
        ]
        # Use local image only if it exists, avoid remote pull attempts
        if use_local_only:
            cmd.append("--pull=never")
        cmd.extend([*self.remaining_args, self.base_name, "/bin/bash"])

        try:
            subprocess.run(cmd, check=False)
            print_info("Docker run completed")
        except KeyboardInterrupt:
            print_info("\nDocker run interrupted")
            sys.exit(0)


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
        """
    )
    parser.add_argument(
        "-p", "--platform",
        metavar="ARCH",
        help=(
            "Platform architecture. Supported values:\n"
            "  - x86_64\n"
            "  - aarch64\n"
            "  - arm64\n"
            "  - aarch64(arm64) (alias for aarch64)\n"
            "If not specified, platform will be auto-detected."
        )
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
    return parser.parse_known_args()


def main():
    """Script entry point."""
    args, remaining = parse_arguments()
    runner = AutonomyRunner()
    
    # Only use command line arguments if platform is explicitly specified
    # Otherwise, show interactive menu or auto-detect
    if args.platform:
        # Normalize platform aliases
        platform_arg = args.platform
        if platform_arg in ("aarch64(arm64)", "arm64", "aarch64"):
            platform_arg = "aarch64"
        # Accept common docker/CI values
        if platform_arg in ("amd64", "x64"):
            platform_arg = "x86_64"

        runner.platform_arch = platform_arg
        runner.use_nvidia = args.nvidia
    else:
        # No platform specified, will trigger interactive menu or auto-detect
        runner.platform_arch = ""
        runner.use_nvidia = args.nvidia
    
    runner.remaining_args = remaining
    runner.run()


if __name__ == "__main__":
    main()
