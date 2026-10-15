# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Launch / attach autonomy.control ControllerServer processes."""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional

_LOG = logging.getLogger(__name__)


def autonomy_root() -> Path:
    env = os.environ.get("AUTONOMY_ROOT")
    if env:
        return Path(env)
    # benchmark/src/benchmark/bridge → …/src/autonomy
    return Path(__file__).resolve().parents[4]


def find_controller_binary() -> Optional[Path]:
    root = autonomy_root()
    candidates = [
        root / "install" / "autonomy" / "bin" / "autonomy.control",
        root / "build" / "autonomy" / "bin" / "autonomy.control",
        Path("/workspace/autonomy/install/autonomy/bin/autonomy.control"),
        Path("/workspace/autonomy/build/autonomy/bin/autonomy.control"),
    ]
    for path in candidates:
        if path.is_file() and os.access(path, os.X_OK):
            return path
    which = shutil.which("autonomy.control")
    return Path(which) if which else None


class AutonomyControlBridge:
    """Optional subprocess wrapper around ``autonomy.control``."""

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = dict(config or {})
        self.proc: Optional[subprocess.Popen[Any]] = None
        self.binary = find_controller_binary()

    @property
    def available(self) -> bool:
        return self.binary is not None

    def start(self) -> bool:
        if not self.binary:
            _LOG.warning("autonomy.control binary not found; using local planners")
            return False
        if self.proc and self.proc.poll() is None:
            return True
        dag = self.config.get("dag") or ""
        cmd: List[str] = [str(self.binary)]
        if dag:
            cmd.extend(["--dag", str(dag)])
        _LOG.info("starting autonomy.control: %s", " ".join(cmd))
        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=os.environ.copy(),
            )
            return True
        except OSError as exc:
            _LOG.warning("failed to start autonomy.control: %s", exc)
            self.proc = None
            return False

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
