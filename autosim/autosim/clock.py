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

"""Monotonic simulation clock for message ``Header.stamp``."""

from __future__ import annotations


class Clock:
    """Monotonic simulation clock that emits ``(sec, nanosec)`` stamps."""

    def __init__(self, start_sec: float = 0.0) -> None:
        """Initialize the clock.

        Args:
            start_sec: Starting simulation time in seconds.
        """
        self.time = float(start_sec)

    def tick(self, dt: float) -> None:
        """Advance simulation time.

        Args:
            dt: Time step in seconds; must be non-negative.

        Raises:
            ValueError: If ``dt < 0``.
        """
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        self.time += dt

    def now(self) -> float:
        """Return the current simulation time in seconds.

        Returns:
            Accumulated simulation time.
        """
        return self.time

    def stamp(self) -> tuple[int, int]:
        """Split the current time into seconds and nanoseconds for ``Header.stamp``.

        Returns:
            ``(sec, nanosec)`` with ``nanosec`` in ``[0, 1e9)``.
        """
        sec = int(self.time)
        nanosec = int(round((self.time - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return sec, nanosec
