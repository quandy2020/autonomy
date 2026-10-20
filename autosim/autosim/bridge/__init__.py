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

"""Autolink bridge + sensor sampling helpers.

``Bridge`` / ``Messages`` / ``Clock`` need ``automsgs`` and load lazily.
"""

from __future__ import annotations

from typing import Any

from autosim.bridge.sensors import Sensors

__all__ = ["Bridge", "Clock", "Messages", "Sensors"]


def __getattr__(name: str) -> Any:
    if name == "Bridge":
        from autosim.bridge.bridge import Bridge

        return Bridge
    if name == "Clock":
        from autosim.bridge.clock import Clock

        return Clock
    if name == "Messages":
        from autosim.bridge.messages import Messages

        return Messages
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
