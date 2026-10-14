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

"""Bootstrap Isaac Sim / Isaac Lab SimulationApp (required before omni.usd)."""

from __future__ import annotations

import os
from typing import Any, Optional

_APP: Any = None


def ensure_simulation_app(
    *,
    headless: Optional[bool] = None,
    enable_cameras: bool = True,
) -> Any:
    """Launch a process-wide SimulationApp once; return the live app.

    ``omni.usd`` is only importable after the Kit app has started. Prefer Isaac
    Lab's :class:`AppLauncher` when available (``/workspace/isaaclab``); fall
    back to ``isaacsim.SimulationApp``.

    Args:
        headless: Override GUI. Default: env ``HEADLESS`` / ``AUTOSIM_ISAAC_HEADLESS``
            or ``True`` when no display.
        enable_cameras: Load rendering extensions (needed for RGB-D).

    Returns:
        The ``SimulationApp`` instance.

    Raises:
        ImportError: Neither Isaac Lab nor isaacsim is available.
    """
    global _APP
    if _APP is not None:
        return _APP

    if headless is None:
        env = os.environ.get("AUTOSIM_ISAAC_HEADLESS") or os.environ.get("HEADLESS")
        if env is not None:
            headless = str(env).strip().lower() in ("1", "true", "yes", "on")
        elif os.path.exists("/.dockerenv"):
            # Docker SpaceHero usually has DISPLAY for streaming; still prefer
            # headless unless the user opts into a GUI (AUTOSIM_ISAAC_HEADLESS=0).
            headless = True
        else:
            headless = not bool(os.environ.get("DISPLAY"))

    try:
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(
            {
                "headless": bool(headless),
                "enable_cameras": bool(enable_cameras),
            }
        )
        _APP = launcher.app
        return _APP
    except ImportError:
        pass

    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        raise ImportError(
            "habitat.backend=isaac requires Isaac Sim / Isaac Lab. "
            "Run via: ISAACLAB_PATH=/workspace/isaaclab "
            "CONFIG=.../isaac.yaml src/autonomy/autosim/scripts/run.sh sim "
            "(uses Isaac Lab python.sh so omni.* is available)."
        ) from exc

    _APP = SimulationApp(
        {
            "headless": bool(headless),
            "anti_aliasing": 0,
        }
    )
    return _APP


def simulation_app() -> Any:
    """Return the launched app, or ``None`` if not started."""
    return _APP


def update_app() -> None:
    """Advance one Kit frame when an app is running."""
    if _APP is not None:
        _APP.update()


def close_app() -> None:
    """Close the SimulationApp if we own one."""
    global _APP
    if _APP is None:
        return
    try:
        _APP.close()
    except Exception:
        pass
    _APP = None
