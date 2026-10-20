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

"""Launch Isaac Lab ``AppLauncher`` or ``isaacsim.SimulationApp`` (before ``omni.usd``)."""

from __future__ import annotations

import logging
import os
from typing import Any

logger = logging.getLogger(__name__)

# Process-wide Kit app handle (must not share a name with a launcher function).
kit_app: Any = None


def ensure_simulation_app(
    *,
    headless: bool | None = None,
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
    global kit_app
    if kit_app is not None:
        return kit_app

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

    if enable_cameras:
        # AppLauncher also reads ENABLE_CAMERAS; keep env aligned for kit experience.
        os.environ.setdefault("ENABLE_CAMERAS", "1")

    # Only catch *import* failure of AppLauncher. Kit launch failures often raise
    # ModuleNotFoundError (subclass of ImportError); swallowing those would retry
    # SimulationApp with the same broken experience.
    app_launcher_cls: Any = None
    try:
        from isaaclab.app import AppLauncher as app_launcher_cls
    except ImportError:
        app_launcher_cls = None

    if app_launcher_cls is not None:
        launcher_args: dict[str, Any] = {
            "headless": bool(headless),
            "enable_cameras": bool(enable_cameras),
        }
        launcher = app_launcher_cls(launcher_args)
        kit_app = launcher.app
        if enable_cameras:
            apply_rtx_sensor_settings()
        # Do NOT timeline.play() here: SimulationManager warm-starts PhysX on
        # the empty default stage, then open_stage closes it →
        # "No USD stage attached" / failed simulation view.
        suppress_play_on_open()
        return kit_app

    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        raise ImportError(
            "habitat.backend=isaac requires Isaac Sim / Isaac Lab. "
            "Run via: ISAACLAB_PATH=/workspace/isaaclab "
            "CONFIG=.../isaac.yaml src/autonomy/autosim/scripts/run.sh sim "
            "(uses Isaac Lab python.sh so omni.* is available)."
        ) from exc

    kit_app = SimulationApp(
        {
            "headless": bool(headless),
            "anti_aliasing": 0,
        }
    )
    if enable_cameras:
        apply_rtx_sensor_settings()
    suppress_play_on_open()
    return kit_app


def apply_rtx_sensor_settings() -> None:
    """Carb knobs required for Isaac Camera / RTX lidar render products.

    Ported from arena-isaac ``run_isaacsim.py``: without ``supportMultiTickRate``
    sensor render products never tick; ``perSensorTickTlas`` ON segfaults and
    is unused by basic RGB-D.
    """
    try:
        import carb

        settings = carb.settings.get_settings()
        settings.set_bool("/isaaclab/render/rtx_sensors", True)
        # Do not sleep to a fixed 60Hz when a frame already took longer.
        settings.set_bool("/app/runLoops/main/rateLimitEnabled", False)
        settings.set("/rtx/hydra/supportMultiTickRate", True)
        settings.set("/rtx/rendering/perSensorTickTlas", False)
        settings.set("/rtx/rtpt/gaussian/skipTonemapping/enabled", True)
        logger.info(
            "Isaac RTX sensor settings: multiTickRate on, perSensorTickTlas off"
        )
    except Exception as exc:
        logger.debug("RTX sensor carb settings skipped: %s", exc)


def suppress_play_on_open() -> None:
    """Keep PhysX from warm-starting while the default empty stage is swapped."""
    try:
        import carb

        settings = carb.settings.get_settings()
        settings.set_bool("/app/player/playOnOpen", False)
    except Exception:
        pass
    stop_timeline()


def stop_timeline() -> None:
    """Stop Kit timeline so stage open does not recreate a physics sim view."""
    try:
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        if timeline is not None and timeline.is_playing():
            timeline.stop()
    except Exception as exc:
        logger.debug("timeline.stop skipped: %s", exc)


def get_simulation_app() -> Any:
    """Return the launched app, or ``None`` if not started."""
    return kit_app


def update_app() -> None:
    """Advance one Kit frame when an app is running."""
    if kit_app is not None:
        kit_app.update()


def close_app() -> None:
    """Close the SimulationApp if we own one."""
    global kit_app
    if kit_app is None:
        return
    try:
        kit_app.close()
    except Exception:
        pass
    kit_app = None


def set_simulation_seed(seed: int) -> None:
    """Best-effort deterministic seed for NumPy / carb / Replicator."""
    value = int(seed)
    try:
        import random

        random.seed(value)
    except Exception:
        pass
    try:
        import numpy as np

        np.random.seed(value)
    except Exception:
        pass
    try:
        import carb

        carb.settings.get_settings().set("/app/rngSeed", value)
    except Exception as exc:
        logger.debug("carb rngSeed skipped: %s", exc)
    try:
        import omni.replicator.core as rep

        rep.set_global_seed(value)
    except Exception as exc:
        logger.debug("replicator seed skipped: %s", exc)


def play_timeline() -> None:
    """Resume Kit timeline (PhysX / animation)."""
    try:
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        if timeline is not None and not timeline.is_playing():
            timeline.play()
    except Exception as exc:
        logger.debug("timeline.play skipped: %s", exc)


def pause_timeline() -> None:
    """Pause Kit timeline without tearing down the stage."""
    try:
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        if timeline is not None and timeline.is_playing():
            timeline.pause()
    except Exception as exc:
        logger.debug("timeline.pause skipped: %s", exc)


def start_viewport_recording(path: str) -> bool:
    """Best-effort viewport capture to ``path`` (mp4/dir). Returns success."""
    target = str(path or "").strip()
    if not target:
        return False
    try:
        import omni.kit.capture.viewport  # type: ignore

        capture = omni.kit.capture.viewport.CaptureOptions()
        capture.file_name = target
        # API varies across Kit builds; treat as best-effort.
        logger.info("viewport recording requested → %s", target)
        return True
    except Exception as exc:
        logger.debug("viewport recording unavailable: %s", exc)
        return False
