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

"""Detect whether third-party dependencies are already installed."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path
from typing import Any

from install_deps.config import Config


class Detector:
    """Checks configured artifact paths before running installers."""

    def __init__(self, config: Config) -> None:
        self._config = config

    def is_installed(self, script_name: str) -> bool:
        """Returns True when known artifacts for script_name are present."""
        check = self._config.skip_check(script_name)
        if check is None:
            return False

        if "protoc_version" in check:
            return self._is_protoc_version_installed(check["protoc_version"])

        paths = check.get("paths", ())
        if not self._library_found(tuple(paths)):
            return False

        executable = check.get("executable")
        if executable:
            return shutil.which(str(executable)) is not None
        return True

    def _extra_lib_dirs(self) -> list[Path]:
        home_local = Path.home() / self._config.user_lib_relative
        return [home_local] if home_local.is_dir() else []

    def _path_or_soname_exists(self, pattern: str) -> bool:
        path = Path(pattern)
        if path.exists():
            return True
        if not path.name.endswith(".so"):
            return False
        if not path.parent.is_dir():
            return False
        stem = path.name[: -len(".so")]
        return any(path.parent.glob(f"{stem}*.so*"))

    def _library_found(self, patterns: tuple[str, ...]) -> bool:
        for pattern in patterns:
            if self._path_or_soname_exists(pattern):
                return True

        for pattern in patterns:
            path = Path(pattern)
            if not path.name.endswith(".so"):
                continue
            stem = path.name[: -len(".so")]
            for lib_dir in self._extra_lib_dirs():
                if any(lib_dir.glob(f"{stem}*.so*")):
                    return True
        return False

    def _is_protoc_version_installed(self, protoc_config: dict[str, Any]) -> bool:
        protoc = Path(str(protoc_config["protoc"]))
        if not protoc.is_file():
            return False

        lib_dir = Path(str(protoc_config["lib_dir"]))
        lib_glob = str(protoc_config["lib_glob"])
        if not any(lib_dir.glob(lib_glob)):
            return False

        try:
            completed = subprocess.run(
                [str(protoc), "--version"],
                capture_output=True,
                text=True,
                check=True,
            )
        except subprocess.CalledProcessError:
            return False

        version_contains = str(protoc_config["version_contains"])
        return version_contains in f"{completed.stdout}{completed.stderr}"
