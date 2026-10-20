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

"""Load HuNavSim / Hunav_isaac_wrapper agent YAML into autosim :class:`Agents`."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from autosim.scene.agents.roster import Agents


def unwrap_hunav_params(data: Mapping[str, Any]) -> Mapping[str, Any]:
    """Strip ``hunav_loader.ros__parameters`` nesting when present."""
    loader = data.get("hunav_loader")
    if isinstance(loader, Mapping):
        params = loader.get("ros__parameters")
        if isinstance(params, Mapping):
            return params
    return data


def load_hunav_scenario(path: str | Path) -> Agents:
    """Parse a HuNav-style agents YAML (warehouse / hospital / office).

    Accepts both native autosim ``agents: [...]`` and HuNav
    ``hunav_loader.ros__parameters`` layouts (named agent keys + global_goals).
    """
    file = Path(path).expanduser()
    if not file.is_file():
        raise FileNotFoundError(f"hunav scenario missing: {file}")
    with file.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, Mapping):
        raise ValueError(f"hunav scenario must be a mapping: {file}")
    params = dict(unwrap_hunav_params(raw))
    global_goals = params.get("global_goals") or {}
    agent_names = params.get("agents") or []
    if agent_names and all(isinstance(name, str) for name in agent_names):
        records = []
        for name in agent_names:
            entry = dict(params.get(name) or {})
            entry.setdefault("name", name)
            goals = entry.get("goals")
            if goals and global_goals and all(
                not isinstance(g, Mapping) for g in goals
            ):
                resolved = []
                for goal_id in goals:
                    g = global_goals.get(goal_id) or global_goals.get(str(goal_id))
                    if isinstance(g, Mapping):
                        resolved.append({"x": float(g["x"]), "y": float(g["y"])})
                entry["goals"] = resolved
            records.append(entry)
        return Agents.from_mapping({"agents": records})
    return Agents.from_mapping(params)
