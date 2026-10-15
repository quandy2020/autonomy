# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Subprocess helpers to launch autosim / Isaac / Habitat backends."""

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


def resolve_autosim_config(config: Dict[str, Any], *, backend: str = "") -> Optional[Path]:
    """Pick autosim YAML for habitat / isaac / fake."""
    explicit = config.get("autosim_config")
    if explicit:
        p = Path(str(explicit)).expanduser()
        if not p.is_absolute():
            # try relative to autosim root and CWD
            for base in (resolve_autosim_root(), Path.cwd(), Path(__file__).resolve().parents[3]):
                cand = (base / p).resolve()
                if cand.is_file():
                    return cand
            # also try rewriting ../autosim/config/…
            name = p.name
            cand = resolve_autosim_root() / "config" / name
            if cand.is_file():
                return cand
        if p.is_file():
            return p
    key = (backend or str(config.get("backend") or config.get("name") or "")).lower()
    root = resolve_autosim_root() / "config"
    mapping = {
        "isaac": "isaac.yaml",
        "habitat": "default.yaml",
        "unity": "default.yaml",
        "gazebo": "fake.yaml",
        "fake": "fake.yaml",
        "flatland": "fake.yaml",
    }
    fname = mapping.get(key, "default.yaml")
    cand = root / fname
    return cand if cand.is_file() else None


class AutosimProcess:
    """Managed ``autosim/scripts/run.sh sim`` child process."""

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = dict(config or {})
        self.proc: Optional[subprocess.Popen[Any]] = None
        self.last_log: str = ""
        self.config_path: Optional[Path] = None

    def start(self) -> bool:
        script = resolve_run_sh()
        if script is None:
            _LOG.warning("autosim scripts/run.sh not found under %s", resolve_autosim_root())
            return False
        backend = str(self.config.get("backend") or self.config.get("name") or "")
        cfg_path = resolve_autosim_config(self.config, backend=backend)
        self.config_path = cfg_path
        env = os.environ.copy()
        if cfg_path is not None:
            env["CONFIG"] = str(cfg_path)
        if self.config.get("isaaclab_path"):
            env["ISAACLAB_PATH"] = str(self.config["isaaclab_path"])
        if self.config.get("headless", True):
            env["AUTOSIM_ISAAC_HEADLESS"] = "1"
            env["HEADLESS"] = "1"
        # Habitat / Isaac scene overrides
        if self.config.get("scene"):
            env["AUTOSIM_SCENE"] = str(self.config["scene"])
        cmd: List[str] = ["bash", str(script), "sim"]
        _LOG.info(
            "launching autosim backend=%s CONFIG=%s %s",
            backend or "?",
            env.get("CONFIG"),
            " ".join(cmd),
        )
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
        wait_s = float(self.config.get("startup_wait_s", 2.0))
        ready = self.wait_ready(timeout_s=wait_s)
        if not ready:
            self._capture_log_tail()
            _LOG.warning(
                "autosim not ready (code=%s) log_tail=%s",
                None if self.proc is None else self.proc.returncode,
                self.last_log[-400:] if self.last_log else "<empty>",
            )
            return False
        return True

    def wait_ready(self, timeout_s: float = 2.0) -> bool:
        """Process still alive after settle period."""
        deadline = time.monotonic() + max(0.1, timeout_s)
        while time.monotonic() < deadline:
            if self.proc is None:
                return False
            if self.proc.poll() is not None:
                self._capture_log_tail()
                return False
            time.sleep(0.1)
        return self.alive

    def _capture_log_tail(self, max_bytes: int = 8000) -> None:
        if self.proc is None or self.proc.stdout is None:
            return
        try:
            # Non-blocking-ish read of whatever is buffered
            data = self.proc.stdout.read(max_bytes) if self.proc.poll() is not None else b""
            if data:
                self.last_log = data.decode("utf-8", errors="replace")
        except Exception:  # noqa: BLE001
            pass

    @property
    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def status(self) -> Dict[str, Any]:
        return {
            "alive": self.alive,
            "pid": None if self.proc is None else self.proc.pid,
            "config": str(self.config_path) if self.config_path else None,
            "returncode": None if self.proc is None else self.proc.poll(),
        }

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
