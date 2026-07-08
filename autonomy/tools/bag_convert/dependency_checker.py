# Copyright 2026 The Openbot Authors (duyongquan)
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

"""Runtime dependency checks for bag_convert."""

from __future__ import annotations

import re

from autonomy.tools.bag_convert.bag_convert_config import BagConvertConfig


class DependencyChecker:
    """Verify third-party packages required by bag_convert."""

    def __init__(self, config: BagConvertConfig | None = None) -> None:
        self._config = config or BagConvertConfig.create_default()

    def check_runtime_dependencies(self) -> None:
        requirements: dict[str, str] = {}
        if self._config.requirements_file.is_file():
            for line in self._config.requirements_file.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                match = re.match(r"^(?P<name>[A-Za-z0-9_.-]+)(?P<spec>.*)$", line)
                if match:
                    requirements[match.group("name").replace("_", "-").lower()] = (
                        match.group("spec").strip()
                    )

        def label(package: str) -> str:
            key = package.lower().replace("_", "-")
            spec = requirements.get(key, "")
            return f"{key}{spec}" if spec else key

        missing: list[str] = []
        for module, package in (("rosbags", "rosbags"), ("grpc_tools", "grpcio-tools")):
            try:
                __import__(module)
            except ImportError:
                missing.append(label(package))

        try:
            from google.protobuf.internal import builder  # noqa: F401
            import google.protobuf

            min_match = re.search(r">=(\d+)\.(\d+)", requirements.get("protobuf", ""))
            if min_match:
                parts: list[int] = []
                for part in google.protobuf.__version__.split(".")[:3]:
                    try:
                        parts.append(int(part))
                    except ValueError:
                        break
                while len(parts) < 2:
                    parts.append(0)
                min_version = (int(min_match.group(1)), int(min_match.group(2)))
                if tuple(parts[:2]) < min_version:
                    missing.append(label("protobuf"))
        except ImportError:
            missing.append(label("protobuf"))

        if not missing:
            return

        req = self._config.requirements_file.relative_to(self._config.repo_root)
        raise SystemExit(
            f"error: missing or incompatible Python packages: {' '.join(missing)}\n"
            f"Conda:  python -m pip install -r {req}\n"
            f"        python -m autonomy.tools.bag_convert ...\n"
            f"Other:  python3 -m pip install --user -r {req}"
        )
