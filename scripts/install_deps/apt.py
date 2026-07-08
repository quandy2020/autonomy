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

"""APT package installation."""

from __future__ import annotations

import subprocess
import sys

from install_deps.config import Config


class Apt:
    """Installs APT packages required for autonomy CMake builds."""

    def __init__(self, config: Config) -> None:
        self._config = config

    def install(self, *, dry_run: bool) -> None:
        """Runs apt-get update, fix-broken, and package install."""
        self._config.run_command(self._config.apt_update_command, dry_run=dry_run)
        self._config.run_command(self._config.apt_fix_broken_command, dry_run=dry_run)

        install_cmd = [*self._config.apt_install_prefix, *self._config.apt_packages]
        try:
            self._config.run_command(install_cmd, dry_run=dry_run)
        except subprocess.CalledProcessError:
            if dry_run:
                raise
            print("Retry apt install after dependency repair...", file=sys.stderr)
            self._config.run_command(self._config.apt_fix_broken_command, dry_run=dry_run)
            self._config.run_command(install_cmd, dry_run=dry_run)
