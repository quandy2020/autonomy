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

"""TrackVLA-style kinematic SMPL-X humanoids for Habitat (Traffic-driven)."""

from __future__ import annotations

from typing import Any

__all__ = [
    "HumanoidSync",
    "humanoid_config_from_habitat",
    "maybe_create_humanoid_sync",
    "resolve_humanoid_asset",
    "discover_humanoid_root",
    "list_humanoid_assets",
]


def __getattr__(name: str) -> Any:
    if name in (
        "HumanoidSync",
        "humanoid_config_from_habitat",
        "maybe_create_humanoid_sync",
        "resolve_humanoid_asset",
    ):
        from autosim.backends.habitat.humanoids import sync as _sync

        return getattr(_sync, name)
    if name in ("discover_humanoid_root", "list_humanoid_assets"):
        from autosim.backends.habitat.humanoids import assets as _assets

        return getattr(_assets, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
