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

"""Isaac-side HuNav wrapper (faithful Autolink port of Hunav_isaac_wrapper).

Organization mirrors ``hunav_isaac_wrapper``:

* :class:`WorldBuilder` — open a USD map
* :class:`HuNavManager` — SkelRoot spawn, AnimGraph, ``/compute_agents``, pose apply
* :mod:`animation_utils` — SkelRoot / AnimationGraph / retarget helpers
"""

from autosim.backends.isaac.hunav import animation_utils
from autosim.backends.isaac.hunav.animation_utils import (
    apply_animation_graph,
    create_agent_animation_graph,
    create_animation,
    find_skelroot_path,
    find_skeleton_path,
    set_anim_graph_speed,
    setup_anim_retargeting,
)
from autosim.backends.isaac.hunav.manager import HuNavManager
from autosim.backends.isaac.hunav.world_builder import WorldBuilder

__all__ = [
    "HuNavManager",
    "WorldBuilder",
    "animation_utils",
    "apply_animation_graph",
    "create_agent_animation_graph",
    "create_animation",
    "find_skelroot_path",
    "find_skeleton_path",
    "set_anim_graph_speed",
    "setup_anim_retargeting",
]
