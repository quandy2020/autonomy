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

"""Runtime health counters for the sensor bridge and control loop.

The bridge drops frames by design: a single-slot worker queue keeps a slow
encoder from stalling physics, and ``emit`` swallows publisher exceptions so
one bad stream cannot kill the run. Both behaviours are invisible without
counters, which makes "the camera looks slow" impossible to diagnose. This
module keeps the numbers and turns them into ``diagnostic_msgs`` rows.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class StreamHealth:
    """Counters for one logical stream (``rgb``, ``scan``, ``odom``...)."""

    name: str
    submitted: int = 0
    dropped: int = 0
    completed: int = 0
    errors: int = 0
    last_error: str = ""
    last_stamp: Optional[float] = None
    last_ms: float = 0.0
    max_ms: float = 0.0
    total_ms: float = 0.0
    rate_hz: float = 0.0

    def record_submit(self) -> None:
        """Count a frame handed to the encoder."""
        self.submitted += 1

    def record_drop(self) -> None:
        """Count a frame replaced before the encoder picked it up."""
        self.dropped += 1

    def record(self, elapsed_ms: float, now: Optional[float] = None) -> None:
        """Count a published frame and refresh the measured rate.

        Args:
            elapsed_ms: Encode + publish duration in milliseconds.
            now: Monotonic timestamp; defaults to ``time.monotonic()``.
        """
        stamp = time.monotonic() if now is None else float(now)
        if self.last_stamp is not None:
            span = stamp - self.last_stamp
            if span > 0.0:
                instant = 1.0 / span
                self.rate_hz = (
                    instant
                    if self.rate_hz <= 0.0
                    else 0.8 * self.rate_hz + 0.2 * instant
                )
        self.last_stamp = stamp
        self.completed += 1
        self.last_ms = float(elapsed_ms)
        self.total_ms += float(elapsed_ms)
        self.max_ms = max(self.max_ms, float(elapsed_ms))

    def record_error(self, error: BaseException | str) -> None:
        """Count a failed encode/publish and keep the latest reason."""
        self.errors += 1
        text = error if isinstance(error, str) else repr(error)
        self.last_error = text[:200]

    def age_sec(self, now: Optional[float] = None) -> float:
        """Seconds since the last successful publish (``inf`` before one)."""
        if self.last_stamp is None:
            return float("inf")
        return (time.monotonic() if now is None else float(now)) - self.last_stamp

    def mean_ms(self) -> float:
        """Average encode + publish duration in milliseconds."""
        if self.completed <= 0:
            return 0.0
        return self.total_ms / float(self.completed)

    def level(self, stale_sec: float = 2.0, now: Optional[float] = None) -> str:
        """``OK`` / ``WARN`` / ``ERROR`` / ``STALE`` for this stream."""
        if self.errors > 0:
            return "ERROR"
        if self.dropped > 0:
            return "WARN"
        if self.completed > 0 and self.age_sec(now) > stale_sec:
            return "STALE"
        return "OK"


@dataclass
class HealthMonitor:
    """Per-stream counters plus control-loop timing."""

    streams: Dict[str, StreamHealth] = field(default_factory=dict)
    cycles: int = 0
    overruns: int = 0
    last_cycle_ms: float = 0.0
    max_cycle_ms: float = 0.0
    started: float = field(default_factory=time.monotonic)

    def stream(self, name: str) -> StreamHealth:
        """Return (and lazily create) the counters for ``name``."""
        entry = self.streams.get(name)
        if entry is None:
            entry = StreamHealth(name=str(name))
            self.streams[name] = entry
        return entry

    def record_cycle(self, elapsed_ms: float, budget_ms: float) -> None:
        """Count one control cycle and flag a missed deadline."""
        self.cycles += 1
        self.last_cycle_ms = float(elapsed_ms)
        self.max_cycle_ms = max(self.max_cycle_ms, float(elapsed_ms))
        if budget_ms > 0.0 and elapsed_ms > budget_ms:
            self.overruns += 1

    def rows(self, hardware_id: str = "", stale_sec: float = 2.0) -> List[Dict[str, Any]]:
        """Flatten every counter into ``DiagnosticStatus`` shaped dicts.

        Args:
            hardware_id: Robot profile name reported on each row.
            stale_sec: Age after which a silent stream is reported stale.

        Returns:
            One dict per stream plus a ``control`` row.
        """
        now = time.monotonic()
        rows: List[Dict[str, Any]] = []
        for name in sorted(self.streams):
            entry = self.streams[name]
            rows.append(
                {
                    "name": f"autosim/{name}",
                    "hardware_id": hardware_id,
                    "level": entry.level(stale_sec, now),
                    "message": entry.last_error or "ok",
                    "values": {
                        "rate_hz": round(entry.rate_hz, 2),
                        "submitted": entry.submitted,
                        "dropped": entry.dropped,
                        "completed": entry.completed,
                        "errors": entry.errors,
                        "last_ms": round(entry.last_ms, 3),
                        "mean_ms": round(entry.mean_ms(), 3),
                        "max_ms": round(entry.max_ms, 3),
                        "age_sec": round(min(entry.age_sec(now), 9999.0), 3),
                    },
                }
            )
        rows.append(
            {
                "name": "autosim/control",
                "hardware_id": hardware_id,
                "level": "WARN" if self.overruns > 0 else "OK",
                "message": "ok",
                "values": {
                    "cycles": self.cycles,
                    "overruns": self.overruns,
                    "last_cycle_ms": round(self.last_cycle_ms, 3),
                    "max_cycle_ms": round(self.max_cycle_ms, 3),
                    "uptime_sec": round(now - self.started, 1),
                },
            }
        )
        return rows
