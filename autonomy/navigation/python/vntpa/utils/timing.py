# Copyright 2026 The Navigation Authors (duyongquan)
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

"""Wall-clock timing helpers."""

from __future__ import annotations

import time


def start() -> float:
    """Return a monotonic start timestamp in seconds."""
    return time.perf_counter()


def elapsed(start_time: float) -> float:
    """Return milliseconds elapsed since ``start_time``."""
    return (time.perf_counter() - start_time) * 1000.0
