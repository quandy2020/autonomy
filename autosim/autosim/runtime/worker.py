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

"""Background encode/publish worker for the control loop."""

from __future__ import annotations

import queue
import threading
import time
import warnings
from typing import Any, Callable, Optional, Tuple

from autosim.runtime.health import HealthMonitor


class SensorWorker:
    """Drain a single-slot queue of encode/publish callables off the main thread.

    Engine render and ray-cast stay on the control thread (GL / Kit context).
    After numpy buffers are ready, the main thread enqueues ``(callable, args)``
    so protobuf I/O does not stall the loop.

    Queue size is 1: a newer frame replaces a stale one when the worker lags.
    The replacement is counted per stream so a slow encoder shows up as
    ``dropped`` on ``/autosim/status`` instead of silently losing frames.
    """

    def __init__(self, monitor: Optional[HealthMonitor] = None) -> None:
        """Create the worker.

        Args:
            monitor: Optional health registry that receives per-stream
                submit/drop/complete/error counters.
        """
        self.monitor = monitor
        self.task_queue: queue.Queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.worker_thread = threading.Thread(
            target=self.drain_queue, daemon=True, name="autosim-sensor"
        )
        self.submitted = 0
        self.dropped = 0
        self.completed = 0
        self.errors = 0
        self.last_error = ""

    def start(self) -> None:
        """Start the background worker thread."""
        self.worker_thread.start()

    def stop(self) -> None:
        """Signal shutdown and join the worker thread."""
        self.stop_event.set()
        try:
            self.task_queue.put_nowait(None)
        except queue.Full:
            pass
        self.worker_thread.join(timeout=2.0)

    def submit(
        self,
        action: Callable[..., Any],
        *args: Any,
        label: Optional[str] = None,
    ) -> None:
        """Enqueue an encode/publish task, dropping the previous pending one.

        Args:
            action: Callable that encodes and publishes one frame.
            *args: Forwarded to ``action``.
            label: Stream name used for the health counters.
        """
        payload: Optional[Tuple[Callable[..., Any], Tuple[Any, ...], Optional[str]]] = (
            action,
            args,
            label,
        )
        self.submitted += 1
        if self.monitor is not None and label:
            self.monitor.stream(label).record_submit()
        try:
            self.task_queue.put_nowait(payload)
        except queue.Full:
            try:
                stale = self.task_queue.get_nowait()
            except queue.Empty:
                stale = None
            self.dropped += 1
            if (
                self.monitor is not None
                and isinstance(stale, tuple)
                and len(stale) == 3
                and stale[2]
            ):
                self.monitor.stream(stale[2]).record_drop()
            try:
                self.task_queue.put_nowait(payload)
            except queue.Full:
                self.dropped += 1
                if self.monitor is not None and label:
                    self.monitor.stream(label).record_drop()

    def drain_queue(self) -> None:
        """Worker body: pop tasks until ``stop_event`` is set."""
        while not self.stop_event.is_set():
            try:
                item = self.task_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            action, args, label = item
            start = time.perf_counter()
            try:
                action(*args)
            except Exception as exc:
                self.errors += 1
                self.last_error = repr(exc)[:200]
                if self.monitor is not None and label:
                    self.monitor.stream(label).record_error(exc)
                warnings.warn(f"autosim sensor worker error: {exc!r}", stacklevel=2)
            else:
                self.completed += 1
                if self.monitor is not None and label:
                    self.monitor.stream(label).record(
                        (time.perf_counter() - start) * 1000.0
                    )
