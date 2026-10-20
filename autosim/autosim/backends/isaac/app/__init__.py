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

"""Kit app bootstrap — mirrors ``isaaclab.app`` (AppLauncher / SimulationApp)."""

from __future__ import annotations

from autosim.backends.isaac.app.launcher import (
    apply_rtx_sensor_settings,
    close_app,
    ensure_simulation_app,
    get_simulation_app,
    pause_timeline,
    play_timeline,
    set_simulation_seed,
    start_viewport_recording,
    stop_timeline,
    suppress_play_on_open,
    update_app,
)

__all__ = [
    "apply_rtx_sensor_settings",
    "close_app",
    "ensure_simulation_app",
    "get_simulation_app",
    "pause_timeline",
    "play_timeline",
    "set_simulation_seed",
    "start_viewport_recording",
    "stop_timeline",
    "suppress_play_on_open",
    "update_app",
]
