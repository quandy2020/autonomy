# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Subprocess helpers to launch autosim / Isaac backends."""

from __future__ import annotations

import logging
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_LOG = logging.getLogger(__name__)


def resolve_autosim_root() -> Path:
    env = os.environ.get("AUTOSIM_ROOT")
    if env:
        return Path(env)
    # …/benchmark/src/benchmark/simulators → …/autosim
    return Path(__file__).resolve().parents[4] / "autosim"


def resolve_run_sh() -> Optional[Path]:
    script = resolve_autosim_root() / "scripts" / "run.sh"
    return script if script.is_file() else None


class AutosimProcess:
    """Managed ``autosim/scripts/run.sh sim`` child process."""

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = dict(config or {})
        self.proc: Optional[subprocess.Popen[Any]] = None

    def start(self) -> bool:
        script = resolve_run_sh()
        if script is None:
            _LOG.warning("autosim scripts/run.sh not found")
            return False
        autosim_cfg = self.config.get("autosim_config")
        env = os.environ.copy()
        if autosim_cfg:
            env["CONFIG"] = str(Path(autosim_cfg).expanduser())
        if self.config.get("isaaclab_path"):
            env["ISAACLAB_PATH"] = str(self.config["isaaclab_path"])
        if self.config.get("headless", True):
            env["AUTOSIM_ISAAC_HEADLESS"] = "1"
            env["HEADLESS"] = "1"
        cmd: List[str] = ["bash", str(script), "sim"]
        _LOG.info("launching autosim: CONFIG=%s %s", env.get("CONFIG"), " ".join(cmd))
        try:
            self.proc = subprocess.Popen(
                cmd,
                cwd=str(script.parent.parent),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            _LOG.warning("autosim launch failed: %s", exc)
            self.proc = None
            return False
        # Brief settle; Isaac may take longer — callers can poll `alive`.
        time.sleep(float(self.config.get("startup_wait_s", 1.0)))
        if self.proc.poll() is not None:
            _LOG.warning("autosim exited early (code=%s)", self.proc.returncode)
            return False
        return True

    @property
    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
