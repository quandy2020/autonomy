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

"""Runtime orchestration: control loop + encode/publish worker.

Data flow (one tick)::

    cmd_vel / PolicyDriver
            │
            ▼
         Robot.step  ──►  WorldBackend.set_pose / step
            │                      │
            │                      ▼
            │              Traffic.step (shared scenario)
            │                      │
            ▼                      ▼
         Sensors.sample_*  ◄── WorldBackend sensing APIs
            │
            ▼
         Messages.encode_*  →  Bridge  →  Autolink
"""

from autosim.runtime.loop import Runner
from autosim.runtime.worker import SensorWorker

__all__ = ["Runner", "SensorWorker"]
