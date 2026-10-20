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

"""Runtime orchestration: config, control loop, encode/publish worker.

Data flow (one tick)::

    cmd_vel / PolicyDriver
            │
            ▼
         Robot.step  ──►  WorldBackend.set_pose / step
            │                      │
            │                      ▼
            │              World.step (scene agents)
            │                      │
            ▼                      ▼
         Sensors.sample_*  ◄── WorldBackend sensing APIs
            │
            ▼
         Messages.encode_*  →  Bridge  →  Autolink
"""

from __future__ import annotations

from typing import Any

from autosim.runtime.settings import Config

__all__ = ["Config", "Runner", "SensorWorker"]


def __getattr__(name: str) -> Any:
    if name == "Runner":
        from autosim.runtime.runner import Runner

        return Runner
    if name == "SensorWorker":
        from autosim.runtime.worker import SensorWorker

        return SensorWorker
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
