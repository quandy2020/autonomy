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

"""UsdSkel walk/idle helpers for Isaac People characters.

CPU Traffic still owns planar motion. This module binds SkelAnimation clips and
advances a shared stage/timeline clock (all agents share phase).
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

WALK_TOKENS = ("walk", "walking", "locomotion", "jog")
IDLE_TOKENS = ("idle", "stand", "standing", "breath")


def classify_clip_name(name: str) -> str | None:
    """Return ``walk`` / ``idle`` / ``None`` from a file or prim name."""
    text = str(name or "").lower()
    if any(token in text for token in WALK_TOKENS):
        return "walk"
    if any(token in text for token in IDLE_TOKENS):
        return "idle"
    return None


def list_animation_usds(root: str | Path) -> dict[str, str]:
    """Find walk/idle animation USD files under ``root`` (first match wins)."""
    base = Path(root).expanduser()
    found: dict[str, str] = {}
    if not base.is_dir():
        return found
    for path in sorted(base.rglob("*")):
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
            continue
        kind = classify_clip_name(path.name)
        if kind and kind not in found:
            found[kind] = str(path.resolve())
        if "walk" in found and "idle" in found:
            break
    return found


def discover_animation_root(character_root: str, explicit: str = "") -> str:
    """Resolve an Animations folder next to People/Characters."""
    if explicit.strip():
        path = Path(explicit).expanduser()
        if path.is_dir():
            return str(path.resolve())
    char = Path(character_root).expanduser() if character_root else None
    if char is None:
        return ""
    candidates = [
        char / "Animations",
        char.parent / "Animations",
        char.parent / "People" / "Animations",
        char / "animations",
    ]
    for path in candidates:
        if path.is_dir() and list_animation_usds(path):
            return str(path.resolve())
    # Animations may live beside character subfolders.
    clips = list_animation_usds(char)
    if clips:
        return str(char.resolve())
    return ""


def anim_rate(speed_mps: float, *, walk_mps: float, moving: bool) -> float:
    """Playback rate for the shared walk clock (0 when idle)."""
    if not moving:
        return 0.0
    ref = max(float(walk_mps), 1e-3)
    return float(clamp_rate(speed_mps / ref, 0.25, 2.0))


def clamp_rate(value: float, low: float, high: float) -> float:
    """Clamp value into [low, high]."""
    return max(low, min(high, float(value)))


def clip_span_seconds(anim_prim: Any, time_codes_per_second: float) -> tuple[float, float] | None:
    """Return ``(start_sec, span_sec)`` for a UsdSkel.Animation prim."""
    if anim_prim is None:
        return None
    try:
        from pxr import UsdSkel
    except ImportError:
        return None
    samples = UsdSkel.Animation(anim_prim).GetRotationsAttr().GetTimeSamples()
    if not samples:
        return None
    tps = float(time_codes_per_second) if time_codes_per_second > 0 else 24.0
    start = float(min(samples)) / tps
    span = (float(max(samples)) - float(min(samples))) / tps
    if span <= 1e-6:
        return None
    return start, span


def bind_animation_source(stage: Any, root_prim: Any, anim_prim: Any) -> bool:
    """Point SkelRoot/Skeleton under ``root_prim`` at ``anim_prim``."""
    if root_prim is None or anim_prim is None:
        return False
    try:
        from pxr import Usd, UsdSkel
    except ImportError:
        return False
    bound = False
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdSkel.Root) or prim.IsA(UsdSkel.Skeleton):
            api = UsdSkel.BindingAPI.Apply(prim)
            api.CreateAnimationSourceRel().SetTargets([anim_prim.GetPath()])
            bound = True
    return bound


def find_embedded_animations(stage: Any, root_prim: Any) -> dict[str, Any]:
    """SkelAnimation prims already under a character reference."""
    found: dict[str, Any] = {}
    if root_prim is None:
        return found
    try:
        from pxr import Usd, UsdSkel
    except ImportError:
        return found
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsA(UsdSkel.Animation):
            continue
        kind = classify_clip_name(prim.GetName()) or (
            "walk" if "walk" not in found else None
        )
        if kind and kind not in found:
            found[kind] = prim
    return found


def set_stage_time(stage: Any, seconds: float) -> None:
    """Scrub USD stage / Kit timeline to ``seconds``."""
    tps = float(stage.GetTimeCodesPerSecond() or 24.0)
    code = float(seconds) * tps
    try:
        stage.SetTimeCode(code)
    except Exception:
        pass
    try:
        import omni.timeline  # type: ignore

        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        timeline.set_current_time(float(seconds))
    except Exception:
        pass


class PeopleAnimator:
    """Shared walk/idle clock for all Traffic character instances."""

    def __init__(
        self,
        *,
        enabled: bool = True,
        walk_mps: float = 1.2,
        clip_assets: dict[str, str] | None = None,
    ) -> None:
        self.enabled = bool(enabled)
        self.walk_mps = float(walk_mps)
        self.clip_assets = dict(clip_assets or {})
        self.anim_time = 0.0
        self.mode = "idle"
        self.walk_span: tuple[float, float] | None = None
        self.idle_span: tuple[float, float] | None = None
        self.walk_prim: Any = None
        self.idle_prim: Any = None
        self.bound_paths: set[str] = set()
        self.warned = False

    def ensure_clips(self, stage: Any, character_prim: Any) -> None:
        """Load / discover walk+idle clips once."""
        if not self.enabled or stage is None:
            return
        if self.walk_prim is not None or self.idle_prim is not None:
            return
        embedded = find_embedded_animations(stage, character_prim)
        if "walk" in embedded:
            self.walk_prim = embedded["walk"]
        if "idle" in embedded:
            self.idle_prim = embedded["idle"]
        # Reference external animation USDs under a stage library path.
        library = "/Root/AutosimTraffic/AnimLibrary"
        for kind, asset in self.clip_assets.items():
            if kind == "walk" and self.walk_prim is not None:
                continue
            if kind == "idle" and self.idle_prim is not None:
                continue
            prim_path = f"{library}/{kind}"
            if not stage.GetPrimAtPath(library).IsValid():
                from pxr import UsdGeom

                UsdGeom.Xform.Define(stage, library)
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                from pxr import UsdGeom

                xform = UsdGeom.Xform.Define(stage, prim_path)
                try:
                    xform.GetPrim().GetReferences().AddReference(asset)
                except Exception as exc:
                    logger.warning("animation reference failed %s: %s", asset, exc)
                    continue
                prim = xform.GetPrim()
            embedded = find_embedded_animations(stage, prim)
            chosen = embedded.get(kind) or next(iter(embedded.values()), None)
            if kind == "walk":
                self.walk_prim = chosen or prim
            else:
                self.idle_prim = chosen or prim
        tps = float(stage.GetTimeCodesPerSecond() or 24.0)
        self.walk_span = clip_span_seconds(self.walk_prim, tps)
        self.idle_span = clip_span_seconds(self.idle_prim, tps)
        if self.walk_prim is None and self.idle_prim is None and not self.warned:
            self.warned = True
            logger.warning(
                "pedestrian_anim enabled but no walk/idle SkelAnimation found "
                "(set animation_root or use People assets with clips)"
            )

    def bind_character(self, stage: Any, character_prim: Any) -> None:
        """Bind the active clip to one character instance."""
        if not self.enabled or character_prim is None:
            return
        self.ensure_clips(stage, character_prim)
        path = str(character_prim.GetPath())
        if path in self.bound_paths:
            return
        anim = self.walk_prim or self.idle_prim
        if anim is not None and bind_animation_source(stage, character_prim, anim):
            self.bound_paths.add(path)

    def rebind_mode(self, stage: Any, mode: str) -> None:
        """Switch all bound characters to walk or idle clip."""
        anim = self.walk_prim if mode == "walk" else self.idle_prim
        if anim is None:
            anim = self.walk_prim or self.idle_prim
        if anim is None:
            return
        for path in list(self.bound_paths):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                bind_animation_source(stage, prim, anim)

    def advance(
        self,
        stage: Any,
        *,
        dt: float,
        mean_speed: float,
        any_moving: bool,
    ) -> None:
        """Advance shared animation clock and scrub the stage."""
        if not self.enabled or stage is None:
            return
        mode = "walk" if any_moving and self.walk_prim is not None else "idle"
        if mode != self.mode:
            self.mode = mode
            self.rebind_mode(stage, mode)
            self.anim_time = 0.0
        rate = anim_rate(mean_speed, walk_mps=self.walk_mps, moving=mode == "walk")
        if mode == "idle":
            rate = 1.0 if self.idle_span else 0.0
        self.anim_time += max(0.0, float(dt)) * rate
        span_info = self.walk_span if mode == "walk" else self.idle_span
        if span_info is None:
            # Still nudge stage time so embedded default clips can loop.
            set_stage_time(stage, self.anim_time)
            return
        start, span = span_info
        set_stage_time(stage, start + math.fmod(self.anim_time, span))
