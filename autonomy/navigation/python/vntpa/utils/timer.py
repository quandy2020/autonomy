"""Simple wall-clock timer."""

from __future__ import annotations

import time


class Timer:
    def __init__(self) -> None:
        self.start = time.perf_counter()

    def elapsed_ms(self) -> float:
        return (time.perf_counter() - self.start) * 1000.0
