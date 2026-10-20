# Copyright 2026 The Openbot Authors (duyongquan)
#
# Port of Hunav_isaac_wrapper.world_builder (ROS → Autolink / autosim paths).
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

"""Load a USD stage (map) from disk into the open Isaac Kit context."""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class WorldBuilder:
    """Open ``{base}/worlds/{map_name}.usd`` (Hunav_isaac_wrapper layout)."""

    def __init__(self, base_path: str | Path) -> None:
        self.base_path = Path(base_path).expanduser()
        self.usd_context: Any = None
        try:
            import omni

            self.usd_context = omni.usd.get_context()
        except Exception as exc:
            logger.warning("WorldBuilder: omni unavailable (%s)", exc)

    def load_map(self, map_name: str) -> str | None:
        """Open ``worlds/{map_name}.usd``; return path or ``None``."""
        name = str(map_name).removesuffix(".usd")
        map_path = self.base_path / "worlds" / f"{name}.usd"
        if not map_path.is_file():
            # Also accept a direct USD path under autosim/usd/worlds.
            alt = self.base_path / f"{name}.usd"
            if alt.is_file():
                map_path = alt
            else:
                logger.error("WorldBuilder: map not found at %s", map_path)
                return None
        if self.usd_context is None:
            logger.error("WorldBuilder: no USD context")
            return None
        self.usd_context.open_stage(os.fspath(map_path))
        logger.info("WorldBuilder: loaded %s", map_path)
        return os.fspath(map_path)
