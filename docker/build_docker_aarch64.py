#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The AOpenRobotic Beginner Authors. All Rights Reserved.
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

"""Build Docker images for aarch64/arm64 platform."""

import argparse
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.absolute()
sys.path.insert(0, str(SCRIPT_DIR / "scripts"))
from docker_utils import normalize_path, run_command, print_error, print_info


class DockerBuilder:
    """Docker image builder for aarch64 platform."""

    def __init__(self):
        self.script_dir = SCRIPT_DIR
        self.dockerfile = ""
        self.use_cache = True
        self.tag = "autonomy.platform.aarch64"

    def build_image(self):
        """Build the Docker image."""
        if not self.dockerfile:
            print_error("Dockerfile is required. Use -f or --dockerfile option.")
            sys.exit(1)

        self.dockerfile = str(normalize_path(self.dockerfile, self.script_dir))
        if not Path(self.dockerfile).exists():
            print_error(f"Dockerfile not found: {self.dockerfile}")
            sys.exit(1)

        extra_args = ["--squash"]
        if not self.use_cache:
            extra_args.append("--no-cache=true")
            print_info("Docker use --no-cache flag build")

        print_info("Building Docker image...")
        print_info(f"  Tag: {self.tag}")
        print_info(f"  Dockerfile: {self.dockerfile}")
        print_info(f"  Context: {self.script_dir}")

        cmd = [
            "docker", "build", "--network=host", *extra_args,
            "-t", self.tag, "-f", self.dockerfile, str(self.script_dir)
        ]

        run_command(cmd)
        print_info(f"Built new image {self.tag}")


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Build Docker image for aarch64 platform",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -f dockerfile/autonomy.aarch64.dockerfile
  %(prog)s -c -f dockerfile/autonomy.aarch64.dockerfile
        """
    )
    parser.add_argument("-c", "--clean", action="store_true", help="Build without docker cache")
    parser.add_argument("-f", "--dockerfile", required=True, help="Dockerfile path (required)")
    return parser.parse_args()


def main():
    """Script entry point."""
    args = parse_arguments()
    builder = DockerBuilder()
    builder.use_cache = not args.clean
    builder.dockerfile = args.dockerfile
    builder.build_image()


if __name__ == "__main__":
    main()
