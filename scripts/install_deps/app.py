# Copyright 2025 The Openbot Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Application entry point for dependency installation."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

from install_deps.apt import Apt
from install_deps.config import Config
from install_deps.detector import Detector
from install_deps.thirdparty import Thirdparty

_EXIT_USAGE_ERROR = 2


class App:
    """Orchestrates APT and third-party dependency installation."""

    def __init__(self, package_dir: Path | None = None) -> None:
        self._config = Config.load(package_dir)
        detector = Detector(self._config)
        self._apt = Apt(self._config)
        self._thirdparty = Thirdparty(self._config, detector)

    def main(self, argv: list[str] | None = None) -> int:
        """Parses argv and runs installation."""
        parser = self._build_parser()
        args = parser.parse_args(argv)

        if args.list_apt:
            for package in self._config.apt_packages:
                print(package)
            return 0

        if args.apt_only and args.thirdparty_only:
            print(
                "Error: --apt-only and --thirdparty-only cannot be used together.",
                file=sys.stderr,
            )
            return _EXIT_USAGE_ERROR

        self._warn_platform()

        try:
            if not args.thirdparty_only:
                count = len(self._config.apt_packages)
                print(f"==> Installing {count} apt packages")
                self._apt.install(dry_run=args.dry_run)

            if not args.apt_only:
                count = len(self._config.thirdparty_scripts)
                print(f"==> Installing {count} third-party scripts")
                self._thirdparty.install(
                    repo_root=args.repo_root.resolve(),
                    dry_run=args.dry_run,
                    resume_from=args.resume_from,
                    skip_installed=args.skip_installed,
                )
        except subprocess.CalledProcessError as exc:
            print(f"Command failed with exit code {exc.returncode}", file=sys.stderr)
            return exc.returncode
        except (FileNotFoundError, ValueError, OSError) as exc:
            print(f"Installation failed: {exc}", file=sys.stderr)
            return 1

        print("Dependency installation finished.")
        return 0

    def _build_parser(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser(
            description="Install autonomy dependencies (apt + docker/install scripts).",
            epilog=(
                "CMake expects third-party libs under /usr/local from docker/install "
                "(glog, gflags, Ceres, OpenCV, OSQP, BehaviorTree.CPP, etc.). "
                "Ansible roles/dependencies/ calls this entry point; do not duplicate "
                "install logic in ansible/."
            ),
        )
        parser.add_argument(
            "--repo-root",
            type=Path,
            default=self._config.default_repo_root(),
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
            help=(
                "Resume third-party installation from a script name "
                "(e.g. install_opencv.sh)."
            ),
        )
        parser.add_argument(
            "--skip-installed",
            action="store_true",
            help="Skip third-party installers when known libraries/headers already exist.",
        )
        return parser

    def _warn_platform(self) -> None:
        if os.geteuid() == 0:
            print(
                "Warning: running as root. 'sudo' commands may be redundant.",
                file=sys.stderr,
            )

        os_release = Path(self._config.os_release_file)
        if not os_release.is_file():
            print(
                f"Warning: {os_release} not found, skip platform check.",
                file=sys.stderr,
            )
            return

        content = os_release.read_text(encoding="utf-8", errors="ignore")
        if self._config.ubuntu_id_token not in content:
            print(
                "Warning: this script is designed for Ubuntu-based systems.",
                file=sys.stderr,
            )


def main(argv: list[str] | None = None) -> int:
    """Module-level entry point."""
    return App().main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
