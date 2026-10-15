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

"""Engine backends — each package is self-contained and self-registers.

| Package | WorldBackend | Notes |
|---------|--------------|-------|
| ``habitat`` | ``Simulator`` | Habitat-Sim + GLB / MP3D |
| ``fake`` | ``FakeWorld`` | Scenario occupancy + raycast |
| ``isaac`` | ``IsaacSimulator`` | Isaac Sim + USD / PhysX |

Shared geometry (scenario traffic, occupancy tools) lives outside this tree.
Backends must not import sibling backends.
"""
