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

"""HuNav-compatible agent messages ↔ autosim SFM (Autolink).

See ``automsgs/msgs/hunav_msgs`` and ``docs/12_hunav_isaac.md``.
"""

from autosim.scene.hunav.client import call_compute_agents, create_compute_agents_client
from autosim.scene.hunav.codec import AgentCodec, BEH_NAME_TO_TYPE, BEH_TYPE_TO_NAME
from autosim.scene.hunav.compute import ComputeAgentsEngine
from autosim.scene.hunav.loader import load_hunav_scenario, unwrap_hunav_params
from autosim.scene.hunav.service import HunavServiceHost

__all__ = [
    "AgentCodec",
    "BEH_NAME_TO_TYPE",
    "BEH_TYPE_TO_NAME",
    "ComputeAgentsEngine",
    "HunavServiceHost",
    "call_compute_agents",
    "create_compute_agents_client",
    "load_hunav_scenario",
    "unwrap_hunav_params",
]
