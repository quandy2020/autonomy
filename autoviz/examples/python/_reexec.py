"""Re-exec with a clean system Python before Isaac Sim pollutes stdlib imports."""

from __future__ import annotations

import os
import sys


def _path_mentions_isaac(text: str) -> bool:
    lowered = text.lower()
    return "isaac" in lowered or "isaaclab" in lowered


def python_env_is_polluted() -> bool:
    if _path_mentions_isaac(os.path.realpath(sys.executable)):
        return True
    prefix = getattr(sys, "base_prefix", sys.prefix)
    if _path_mentions_isaac(prefix):
        return True
    for key in ("PYTHONHOME", "PYTHONPATH"):
        if _path_mentions_isaac(os.environ.get(key, "")):
            return True
    for entry in sys.path[:8]:
        if entry and _path_mentions_isaac(entry):
            return True
    return False


def system_python() -> str | None:
    override = os.environ.get("AUTOVIZ_PYTHON")
    if override and os.path.isfile(override):
        return override
    for candidate in ("/usr/bin/python3.10", "/usr/bin/python3"):
        if os.path.isfile(candidate):
            return candidate
    return None


def clean_exec_env() -> dict[str, str]:
    env = os.environ.copy()
    env["AUTOVIZ_PY_REEXEC"] = "1"
    for key in ("PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV", "CONDA_PREFIX"):
        env.pop(key, None)
    env["PATH"] = "/usr/bin:/bin:/usr/local/bin:" + env.get("PATH", "")
    return env


def early_reexec_if_needed() -> None:
    """Switch to system Python with a clean env (fixes SRE module mismatch)."""
    if os.environ.get("AUTOVIZ_PY_REEXEC") == "1":
        return
    if not python_env_is_polluted():
        return
    candidate = system_python()
    if candidate is None:
        return
    os.execve(candidate, [candidate, *sys.argv], clean_exec_env())
