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

"""Isaac Sim (USD) WorldBackend."""

from __future__ import annotations

from typing import Any, Mapping

from autosim.backend.registry import register


def create(settings: Mapping[str, Any]) -> Any:
    """Build an Isaac Sim WorldBackend from full config data.

    Raises:
        ImportError: Isaac Sim / Omniverse Python packages are unavailable.
        FileNotFoundError: Configured USD world path does not exist.
    """
    from autosim.backends.isaac.simulator import IsaacSimulator

    return IsaacSimulator.create(dict(settings))


register("isaac", create)
