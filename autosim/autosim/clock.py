from __future__ import annotations


class SimClock:
    """Monotonic simulation clock used for message headers."""

    def __init__(self, start_sec: float = 0.0) -> None:
        self._t = float(start_sec)

    def tick(self, dt: float) -> None:
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        self._t += dt

    def now(self) -> float:
        return self._t

    def stamp(self) -> tuple[int, int]:
        sec = int(self._t)
        nanosec = int(round((self._t - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return sec, nanosec
