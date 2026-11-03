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

"""Container-wide single-instance guard for AutoSim publishers."""

from __future__ import annotations

import fcntl
import json
import os
import stat
import time
from pathlib import Path
from typing import IO, Optional


DEFAULT_INSTANCE_LOCK_PATH = Path("/tmp/autosim.lock")
INSTANCE_LOCK_PATH_ENV = "AUTOSIM_INSTANCE_LOCK_PATH"


class InstanceLockError(RuntimeError):
    """Raised when AutoSim cannot become the container's active publisher."""


class InstanceLock:
    """Hold an advisory file lock for the complete AutoSim process lifetime."""

    def __init__(self, path: Path, config_path: Path) -> None:
        self.path = Path(path)
        self.config_path = Path(config_path).resolve(strict=False)
        self._stream: Optional[IO[str]] = None

    @classmethod
    def from_environment(cls, config_path: Path) -> "InstanceLock":
        """Build the default lock, allowing an explicit multi-domain override."""
        override = os.environ.get(INSTANCE_LOCK_PATH_ENV, "").strip()
        path = Path(override).expanduser() if override else DEFAULT_INSTANCE_LOCK_PATH
        return cls(path, config_path)

    @staticmethod
    def holder_description(stream: IO[str]) -> str:
        """Return concise metadata for the process currently holding the lock."""
        try:
            stream.seek(0)
            raw = stream.read().strip()
        except OSError:
            return ""
        if not raw:
            return ""
        try:
            metadata = json.loads(raw)
        except (TypeError, ValueError):
            return raw
        pid = metadata.get("pid")
        config = metadata.get("config")
        fields = []
        if pid is not None:
            fields.append(f"pid={pid}")
        if config:
            fields.append(f"config={config}")
        return ", ".join(fields)

    def acquire(self) -> "InstanceLock":
        """Acquire the lock without waiting and record owner metadata."""
        if self._stream is not None:
            return self

        flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
        try:
            descriptor = os.open(self.path, flags, 0o600)
        except OSError as error:
            raise InstanceLockError(
                f"unable to open AutoSim instance lock {self.path}: {error}"
            ) from error

        stream = os.fdopen(descriptor, "r+", encoding="utf-8")
        try:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                raise InstanceLockError(
                    f"AutoSim instance lock is not a regular file: {self.path}"
                )
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                holder = self.holder_description(stream)
                detail = f" ({holder})" if holder else ""
                raise InstanceLockError(
                    "another AutoSim instance is already running" + detail
                ) from error

            metadata = {
                "pid": os.getpid(),
                "config": str(self.config_path),
                "started_at_unix_sec": time.time(),
            }
            stream.seek(0)
            stream.truncate()
            json.dump(metadata, stream, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
            except OSError:
                pass
            stream.close()
            raise

        self._stream = stream
        return self

    def release(self) -> None:
        """Release this process's lock, if held."""
        stream = self._stream
        self._stream = None
        if stream is None:
            return
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        finally:
            stream.close()

    def __enter__(self) -> "InstanceLock":
        return self.acquire()

    def __exit__(self, _exc_type, _exc_value, _traceback) -> bool:
        self.release()
        return False
