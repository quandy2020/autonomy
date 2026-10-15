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
import warnings
from typing import Any, Callable, Optional, Tuple


class SensorWorker:
    """Drain a single-slot queue of encode/publish callables off the main thread.

    Engine render and ray-cast stay on the control thread (GL / Kit context).
    After numpy buffers are ready, the main thread enqueues ``(callable, args)``
    so protobuf I/O does not stall the loop.

    Queue size is 1: a newer frame replaces a stale one when the worker lags.
    """

    def __init__(self) -> None:
        self.task_queue: queue.Queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.worker_thread = threading.Thread(
            target=self.drain_queue, daemon=True, name="autosim-sensor"
        )

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

    def submit(self, action: Callable[..., Any], *args: Any) -> None:
        """Enqueue an encode/publish task, dropping the previous pending one."""
        payload: Optional[Tuple[Callable[..., Any], Tuple[Any, ...]]] = (action, args)
        try:
            self.task_queue.put_nowait(payload)
        except queue.Full:
            try:
                self.task_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.task_queue.put_nowait(payload)
            except queue.Full:
                pass

    def drain_queue(self) -> None:
        """Worker body: pop tasks until ``stop_event`` is set."""
        while not self.stop_event.is_set():
            try:
                item = self.task_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            action, args = item
            try:
                action(*args)
            except Exception as exc:
                warnings.warn(f"autosim sensor worker error: {exc!r}", stacklevel=2)
