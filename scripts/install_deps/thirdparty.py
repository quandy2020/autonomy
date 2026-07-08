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

"""Third-party shell installer orchestration."""

from __future__ import annotations

import os
from pathlib import Path

from install_deps.config import Config
from install_deps.detector import Detector


class Thirdparty:
    """Runs docker/install/*.sh scripts in dependency order."""

    def __init__(self, config: Config, detector: Detector) -> None:
        self._config = config
        self._detector = detector

    def install(
        self,
        *,
        repo_root: Path,
        dry_run: bool,
        resume_from: str | None,
        skip_installed: bool,
    ) -> None:
        """Executes third-party installer scripts."""
        install_dir = self._config.docker_install_dir(repo_root)
        if not install_dir.is_dir():
            raise FileNotFoundError(f"Install directory not found: {install_dir}")

        scripts = self._config.thirdparty_scripts
        if resume_from is not None and resume_from not in scripts:
            valid = ", ".join(scripts)
            raise ValueError(
                f"--resume-from={resume_from} is invalid, choose one of: {valid}"
            )

        started = resume_from is None
        thirdparty_dir = self._config.prepare_thirdparty_cache_dir()
        script_env = os.environ.copy()
        script_env[self._config.thirdparty_env_var] = str(thirdparty_dir)

        for script_name in scripts:
            if not started:
                if script_name == resume_from:
                    started = True
                else:
                    continue

            script_path = install_dir / script_name
            if not script_path.is_file():
                raise FileNotFoundError(f"Missing dependency installer: {script_path}")

            if (
                skip_installed
                and self._config.can_detect_installed(script_name)
                and self._detector.is_installed(script_name)
            ):
                print(
                    f"{self._config.skip_log_prefix} {script_name}: "
                    "dependency already detected"
                )
                continue

            self._config.run_command(
                [self._config.shell_interpreter, str(script_path)],
                dry_run=dry_run,
                env=script_env,
            )
