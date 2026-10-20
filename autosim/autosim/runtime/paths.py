# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Project-root discovery for config / urdf / usd paths."""

from __future__ import annotations

from pathlib import Path


def project_root() -> Path:
    """Return the autosim project root (contains ``config/``, ``robot/``).

    Walks upward from this file so nested packages (``robot/``, ``backends/``, …)
    resolve the same root without hard-coded ``parents[N]`` depths.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        # Prefer the repo project root (YAML configs), not the Python package
        # which also has config/ and robot/ subpackages.
        if (parent / "config" / "default.yaml").is_file() or (
            parent / "config" / "isaac.yaml"
        ).is_file():
            return parent
        if (parent / "config").is_dir() and (parent / "usd").is_dir():
            return parent
    # Fallback: autosim/paths.py → package → project
    return here.parents[1]
