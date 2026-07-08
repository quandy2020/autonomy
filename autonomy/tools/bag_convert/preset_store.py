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

"""Named conversion presets loaded from JSON files."""

from __future__ import annotations

import json
from typing import Dict, Tuple

from autonomy.tools.bag_convert.bag_convert_config import BagConvertConfig


class PresetStore:
    """Load topic filters and remaps from ``presets/<name>.json``."""

    def __init__(self, config: BagConvertConfig | None = None) -> None:
        self._config = config or BagConvertConfig.create_default()

    def load_preset(self, name: str) -> Tuple[Tuple[str, ...], Dict[str, str]]:
        path = self._config.presets_dir / f"{name}.json"
        if not path.is_file():
            raise FileNotFoundError(f"preset not found: {path}")

        data = json.loads(path.read_text(encoding="utf-8"))
        topics = tuple(str(topic) for topic in data.get("topics", ()))
        topic_remap = {str(src): str(dst) for src, dst in data.get("topic_remap", {}).items()}
        return topics, topic_remap
