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

"""Configuration loaded from data/*.json."""

from __future__ import annotations

import json
import os
import shlex
import subprocess
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any


class Config:
    """Loads install_deps data files and resolves repository paths."""

    def __init__(self, package_dir: Path) -> None:
        data_dir = package_dir / "data"
        with (data_dir / "settings.json").open(encoding="utf-8") as handle:
            self._settings = json.load(handle)
        with (data_dir / "apt_packages.json").open(encoding="utf-8") as handle:
            apt_groups = json.load(handle)["groups"]
        with (data_dir / "thirdparty.json").open(encoding="utf-8") as handle:
            thirdparty = json.load(handle)

        self._package_dir = package_dir
        self._apt_packages = tuple(
            package for group in apt_groups.values() for package in group
        )
        self._thirdparty_scripts = tuple(thirdparty["scripts"])
        self._skip_checks: dict[str, dict[str, Any]] = thirdparty["skip_checks"]

    @classmethod
    def load(cls, package_dir: Path | None = None) -> Config:
        """Loads configuration from the install_deps package directory."""
        resolved = package_dir or Path(__file__).resolve().parent
        return cls(resolved)

    @property
    def apt_packages(self) -> tuple[str, ...]:
        return self._apt_packages

    @property
    def thirdparty_scripts(self) -> tuple[str, ...]:
        return self._thirdparty_scripts

    def skip_check(self, script_name: str) -> dict[str, Any] | None:
        return self._skip_checks.get(script_name)

    def can_detect_installed(self, script_name: str) -> bool:
        return script_name in self._skip_checks

    def default_repo_root(self) -> Path:
        levels = int(self._settings["layout"]["scripts_parent_levels_from_package"])
        return self._package_dir.resolve().parents[levels]

    def docker_install_dir(self, repo_root: Path) -> Path:
        relative = self._settings["layout"]["docker_install_relative"]
        return repo_root / relative

    def prepare_thirdparty_cache_dir(self) -> Path:
        thirdparty = self._settings["thirdparty"]
        env_var = str(thirdparty["env_var"])
        override = os.environ.get(env_var)
        if override:
            path = Path(override)
            path.mkdir(parents=True, exist_ok=True)
            return path

        docker_path = Path(str(thirdparty["system_dir"]))
        if docker_path.is_dir():
            return docker_path

        if os.geteuid() == 0:
            try:
                docker_path.mkdir(parents=True, exist_ok=True)
                return docker_path
            except OSError:
                pass

        cache_path = Path.home() / self._settings["layout"]["user_cache_relative"]
        cache_path.mkdir(parents=True, exist_ok=True)
        return cache_path

    @property
    def user_lib_relative(self) -> str:
        return str(self._settings["layout"]["user_lib_relative"])

    @property
    def thirdparty_env_var(self) -> str:
        return str(self._settings["thirdparty"]["env_var"])

    @property
    def apt_update_command(self) -> list[str]:
        return list(self._settings["apt"]["update_command"])

    @property
    def apt_fix_broken_command(self) -> list[str]:
        return list(self._settings["apt"]["fix_broken_command"])

    @property
    def apt_install_prefix(self) -> list[str]:
        return list(self._settings["apt"]["install_prefix"])

    @property
    def shell_interpreter(self) -> str:
        return str(self._settings["shell"]["interpreter"])

    @property
    def run_log_prefix(self) -> str:
        return str(self._settings["shell"]["log_prefix"])

    @property
    def skip_log_prefix(self) -> str:
        return str(self._settings["shell"]["skip_prefix"])

    @property
    def os_release_file(self) -> str:
        return str(self._settings["platform"]["os_release_file"])

    @property
    def ubuntu_id_token(self) -> str:
        return str(self._settings["platform"]["ubuntu_id_token"])

    def run_command(
        self,
        command: Iterable[str],
        *,
        dry_run: bool,
        env: Mapping[str, str] | None = None,
    ) -> None:
        """Runs command after printing a quoted preview."""
        argv = list(command)
        printable = " ".join(shlex.quote(arg) for arg in argv)
        print(f"{self.run_log_prefix} {printable}")
        if dry_run:
            return
        subprocess.run(argv, check=True, env=env)
