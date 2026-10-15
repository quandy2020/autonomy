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

"""autosim: multi-backend sensor–actuator bridge + LeRobot IL recording.

Canonical packages::

    runtime/    Runner orchestration
    backend/    WorldBackend contract + registry
    backends/   habitat | fake | isaac
    scenario/   shared geometry / traffic
    sensing/    sampler, cameras, stitch
    io/         bridge, messages, clock, teleop
    dataset/    LeRobot collector
    policy/     ONNX twist driver
    robot/      plant, profiles, urdf
    map/        ground-truth occupancy helpers
    config/     YAML load + validation
    paths.py    project-root discovery

Style: no leading-underscore names; attributes are nouns; functions are
verb + noun (``poll_command``, ``flush_episode``, ``build_feeds``).
"""

__version__ = "0.1.0"
