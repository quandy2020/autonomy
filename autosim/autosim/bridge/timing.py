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

"""Per-sensor publish latency and jitter.

Habitat hands back a frame the instant the control loop asks for it, so every
consumer sees perfect timing. Real drivers buffer a frame, stamp it at capture
and deliver it a few milliseconds later with some spread. These helpers add
that delay without touching the message: a held frame keeps its capture stamp,
which is exactly what a consumer needs to test time sync.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any, Mapping

# A held frame is useless to a live controller long before these bounds, and
# they keep a typo from stalling the whole stream.
MAX_LATENCY_SEC = 2.0
MAX_JITTER_SEC = 1.0


@dataclass(frozen=True)
class SensorTiming:
    """Delivery delay for one sensor stream."""

    latency_sec: float = 0.0
    jitter_sec: float = 0.0

    @classmethod
    def from_block(cls, block: Mapping[str, Any] | None, label: str) -> "SensorTiming":
        """Read ``latency_sec`` / ``jitter_sec`` from a sensor block.

        Args:
            block: ``habitat.sensors.<name>`` mapping; ``None`` means "no
                timing keys", which yields a zero delay.
            label: Dotted path used in error messages.

        Returns:
            Parsed timing.

        Raises:
            ValueError: Missing, non-finite, negative or oversized value.
        """
        if not isinstance(block, Mapping):
            return cls()
        latency = float(block.get("latency_sec", 0.0) or 0.0)
        jitter = float(block.get("jitter_sec", 0.0) or 0.0)
        if not math.isfinite(latency) or not 0.0 <= latency <= MAX_LATENCY_SEC:
            raise ValueError(f"{label}.latency_sec must be in [0, {MAX_LATENCY_SEC}]")
        if not math.isfinite(jitter) or not 0.0 <= jitter <= MAX_JITTER_SEC:
            raise ValueError(f"{label}.jitter_sec must be in [0, {MAX_JITTER_SEC}]")
        return cls(latency, jitter)

    @property
    def enabled(self) -> bool:
        """True when this stream needs the delay queue."""
        return self.latency_sec > 0.0 or self.jitter_sec > 0.0

    def delay(self, rng: random.Random | None = None) -> float:
        """Sample this frame's delay in seconds.

        Args:
            rng: Generator for the jitter draw; ``None`` uses the module RNG.

        Returns:
            Non-negative delay; jitter alone can never arrive before capture.
        """
        if not self.enabled:
            return 0.0
        spread = 0.0
        if self.jitter_sec > 0.0:
            generator = rng if rng is not None else random
            spread = generator.uniform(-self.jitter_sec, self.jitter_sec)
        return max(0.0, self.latency_sec + spread)


class DelayQueue:
    """Held frames ordered by release time, then arrival."""

    def __init__(self) -> None:
        self._items: list[tuple[float, int, str, Any]] = []
        self._sequence = 0

    def push(self, release_sec: float, key: str, message: Any) -> None:
        """Hold ``message`` until ``release_sec`` (simulation clock)."""
        self._items.append((float(release_sec), self._sequence, key, message))
        self._sequence += 1

    def pop_ready(self, now_sec: float) -> list[tuple[str, Any]]:
        """Remove and return every frame whose release time has passed.

        Args:
            now_sec: Current simulation time.

        Returns:
            ``(channel key, message)`` pairs, oldest release first.
        """
        ready: list[tuple[float, int, str, Any]] = []
        waiting: list[tuple[float, int, str, Any]] = []
        for item in self._items:
            if item[0] <= now_sec:
                ready.append(item)
            else:
                waiting.append(item)
        self._items = waiting
        ready.sort(key=lambda item: (item[0], item[1]))
        return [(key, message) for _, _, key, message in ready]

    def clear(self) -> None:
        """Drop every held frame."""
        self._items.clear()

    def __len__(self) -> int:
        return len(self._items)
