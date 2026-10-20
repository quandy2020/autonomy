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

"""UsdSkel / AnimationGraph helpers for Isaac People characters.

CPU Traffic still owns planar motion. Clips are retargeted from the People
biped onto each character (Hunav-style), then driven by AnimationGraph speed.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

WALK_TOKENS = ("walk", "walking", "locomotion", "jog")
IDLE_TOKENS = ("idle", "stand", "standing", "breath")
# Prefer in-place loops so root motion does not fight Traffic poses.
WALK_PREFER = ("loop_in_place", "in_place", "loop")


def classify_clip_name(name: str) -> str | None:
    """Return ``walk`` / ``idle`` / ``None`` from a file or prim name."""
    text = str(name or "").lower()
    if any(token in text for token in WALK_TOKENS):
        return "walk"
    if any(token in text for token in IDLE_TOKENS):
        return "idle"
    return None


def list_animation_usds(root: str | Path) -> dict[str, str]:
    """Find walk/idle animation USD files under ``root`` (best match wins)."""
    base = Path(root).expanduser()
    found: dict[str, str] = {}
    if not base.is_dir():
        return found
    candidates: dict[str, list[Path]] = {"walk": [], "idle": []}
    for path in sorted(base.rglob("*")):
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
            continue
        if "mirror" in path.name.lower():
            continue
        kind = classify_clip_name(path.name)
        if kind:
            candidates[kind].append(path)
    for kind, paths in candidates.items():
        if not paths:
            continue
        if kind == "walk":

            def score(p: Path) -> tuple[int, str]:
                name = p.name.lower()
                pref = 0
                for i, token in enumerate(WALK_PREFER):
                    if token in name:
                        pref = len(WALK_PREFER) - i
                        break
                return (-pref, name)

            paths = sorted(paths, key=score)
        found[kind] = str(paths[0].resolve())
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
    clips = list_animation_usds(char)
    if clips:
        return str(char.resolve())
    return ""


def anim_rate(speed_mps: float, *, walk_mps: float, moving: bool) -> float:
    """Playback rate / blend weight for walk (0 when idle)."""
    if not moving:
        return 0.0
    ref = max(float(walk_mps), 1e-3)
    return float(clamp_rate(speed_mps / ref, 0.25, 2.0))


def clamp_rate(value: float, low: float, high: float) -> float:
    """Clamp value into [low, high]."""
    return max(low, min(high, float(value)))


def _make_invisible(prim: Any) -> None:
    """Hide retarget / clip library prims (Biped_Setup white bust, etc.)."""
    if prim is None or not getattr(prim, "IsValid", lambda: True)():
        return
    try:
        from pxr import UsdGeom

        UsdGeom.Imageable(prim).MakeInvisible()
    except Exception:
        pass
    try:
        attr = prim.GetAttribute("visibility")
        if attr and attr.IsValid():
            attr.Set("invisible")
    except Exception:
        pass
    try:
        from pxr import Sdf

        for name, value in (
            ("physxRigidBody:disableGravity", True),
            ("physxContact:collisionEnabled", False),
        ):
            attr = prim.GetAttribute(name)
            if attr and attr.IsValid():
                attr.Set(value)
            else:
                prim.CreateAttribute(name, Sdf.ValueTypeNames.Bool).Set(value)
    except Exception:
        pass


def _enable_anim_extensions() -> None:
    """Best-effort enable Kit retarget (AnimGraph is optional / often broken in Lab)."""
    try:
        from isaacsim.core.utils.extensions import enable_extension  # type: ignore
    except Exception:
        try:
            from omni.isaac.core.utils.extensions import enable_extension  # type: ignore
        except Exception:
            return
    # Do NOT enable *.ui / graph.schema mid-run — Lab often lacks them and
    # partial enable breaks AnimGraphSchema registration (T-pose forever).
    for ext in ("omni.anim.retarget.core",):
        try:
            enable_extension(ext)
        except Exception:
            pass


def _resolve_biped_agent_path(stage: Any, source_path: str) -> str:
    """Find the biped agent prim that actually contains a Skeleton."""
    from autosim.backends.isaac.hunav.animation_utils import find_skeleton_path

    candidates = [
        f"{source_path}/biped_demo_meters",
        source_path,
    ]
    root = stage.GetPrimAtPath(source_path)
    if root and root.IsValid():
        try:
            from pxr import Usd

            for child in Usd.PrimRange(root):
                name = child.GetName().lower()
                if "biped" in name or child.GetTypeName() in ("SkelRoot", "Skeleton"):
                    candidates.append(child.GetPath().pathString)
        except Exception:
            pass
    seen: set[str] = set()
    for path in candidates:
        if path in seen:
            continue
        seen.add(path)
        prim = stage.GetPrimAtPath(path)
        if prim is None or not prim.IsValid():
            continue
        if find_skeleton_path(prim) is not None:
            return path
    return source_path


def _retargeted_clip_paths(stage: Any, parent: str, kind: str) -> Any | None:
    """Pick a retargeted SkelAnimation under ``parent`` matching walk/idle."""
    root = stage.GetPrimAtPath(parent)
    if root is None or not root.IsValid():
        return None
    try:
        from pxr import Usd, UsdSkel
    except ImportError:
        return None
    tokens = WALK_TOKENS if kind == "walk" else IDLE_TOKENS
    # Prefer exact Hunav names first.
    prefer = ("WalkLoop", "walk") if kind == "walk" else ("IdleLoop", "idle")
    for name in prefer:
        prim = stage.GetPrimAtPath(f"{parent}/{name}")
        if prim and prim.IsValid() and prim.IsA(UsdSkel.Animation):
            return prim.GetPath()
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdSkel.Animation):
            continue
        text = prim.GetName().lower()
        if any(token in text for token in tokens):
            return prim.GetPath()
    return None


def find_skelroot_prim(root_prim: Any) -> Any | None:
    """Return the first SkelRoot under ``root_prim`` (inclusive)."""
    if root_prim is None:
        return None
    try:
        from pxr import Usd, UsdSkel
    except ImportError:
        return None
    if root_prim.IsA(UsdSkel.Root):
        return root_prim
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdSkel.Root):
            return prim
        path = prim.GetPath().pathString
        if "ManRoot" in path:
            for child in prim.GetChildren():
                if child.IsA(UsdSkel.Root):
                    return child
    return None


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
    """SkelAnimation prims already under a character / library reference."""
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
            "walk" if "walk" not in found else ("idle" if "idle" not in found else None)
        )
        if kind and kind not in found:
            found[kind] = prim
    return found


def set_stage_time(stage: Any, seconds: float) -> None:
    """Scrub USD stage time (do not stop Kit timeline — that freezes skels)."""
    tps = float(stage.GetTimeCodesPerSecond() or 24.0)
    code = float(seconds) * tps
    try:
        stage.SetTimeCode(code)
    except Exception:
        pass
    try:
        import omni.timeline  # type: ignore

        timeline = omni.timeline.get_timeline_interface()
        # Keep playing so UsdSkel / AnimGraph evaluate; only scrub time.
        timeline.set_current_time(float(seconds))
        if hasattr(timeline, "is_playing") and not timeline.is_playing():
            timeline.play()
    except Exception:
        pass


class PeopleAnimator:
    """Walk/idle driver for Traffic character instances.

    Prefers AnimationGraph + retarget (Isaac People). Falls back to UsdSkel
    BindingAPI + stage-time scrub when Kit anim extensions are unavailable.
    """

    def __init__(
        self,
        *,
        enabled: bool = True,
        walk_mps: float = 1.2,
        clip_assets: dict[str, str] | None = None,
        character_root: str = "",
    ) -> None:
        self.enabled = bool(enabled)
        self.walk_mps = float(walk_mps)
        self.clip_assets = dict(clip_assets or {})
        self.character_root = str(character_root or "")
        self.anim_time = 0.0
        self.mode = "idle"
        self.walk_span: tuple[float, float] | None = None
        self.idle_span: tuple[float, float] | None = None
        self.walk_prim: Any = None
        self.idle_prim: Any = None
        self.walk_path: Any = None
        self.idle_path: Any = None
        # After retarget: paths under AnimLibrary/Retargeted (Hunav-style).
        self.retarget_walk_path: Any = None
        self.retarget_idle_path: Any = None
        self.bound_paths: set[str] = set()
        self.graph_paths: dict[str, Any] = {}
        self.use_anim_graph = False
        self.retarget_done = False
        self.clips_ready = False
        self.warned = False
        self.library = "/Root/AutosimTraffic/AnimLibrary"
        self.retarget_parent = f"{self.library}/Retargeted"
        # Hunav names so CreateRetargetAnimationsCommand outputs are predictable.
        self._clip_prim_names = {"walk": "WalkLoop", "idle": "IdleLoop"}

    def ensure_clips(self, stage: Any, character_prim: Any) -> None:
        """Load walk+idle SkelAnimation prims once (invisible library)."""
        if not self.enabled or stage is None or self.clips_ready:
            return
        from pxr import Sdf, UsdGeom, UsdSkel

        _enable_anim_extensions()
        if not stage.GetPrimAtPath(self.library).IsValid():
            lib = UsdGeom.Xform.Define(stage, self.library)
            _make_invisible(lib.GetPrim())
        else:
            _make_invisible(stage.GetPrimAtPath(self.library))

        embedded = find_embedded_animations(stage, character_prim)
        if "walk" in embedded:
            self.walk_prim = embedded["walk"]
        if "idle" in embedded:
            self.idle_prim = embedded["idle"]

        for kind, asset in self.clip_assets.items():
            if kind == "walk" and self.walk_prim is not None:
                continue
            if kind == "idle" and self.idle_prim is not None:
                continue
            prim_name = self._clip_prim_names.get(kind, kind)
            prim_path = f"{self.library}/{prim_name}"
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                # SkelAnimation only — never Xform (avoids visible default meshes).
                prim = stage.DefinePrim(Sdf.Path(prim_path), "SkelAnimation")
                try:
                    prim.GetReferences().AddReference(asset)
                except Exception as exc:
                    logger.warning("animation reference failed %s: %s", asset, exc)
                    continue
            _make_invisible(prim)
            nested = find_embedded_animations(stage, prim)
            chosen = nested.get(kind)
            if chosen is None and prim.IsA(UsdSkel.Animation):
                chosen = prim
            if chosen is None:
                chosen = next(iter(nested.values()), prim)
            if kind == "walk":
                self.walk_prim = chosen
                self.walk_path = chosen.GetPath() if chosen is not None else None
            else:
                self.idle_prim = chosen
                self.idle_path = chosen.GetPath() if chosen is not None else None

        tps = float(stage.GetTimeCodesPerSecond() or 24.0)
        self.walk_span = clip_span_seconds(self.walk_prim, tps)
        self.idle_span = clip_span_seconds(self.idle_prim, tps)
        if self.walk_path is None and self.walk_prim is not None:
            self.walk_path = self.walk_prim.GetPath()
        if self.idle_path is None and self.idle_prim is not None:
            self.idle_path = self.idle_prim.GetPath()
        self.clips_ready = True
        if self.walk_prim is None and self.idle_prim is None and not self.warned:
            self.warned = True
            logger.warning(
                "pedestrian_anim enabled but no walk/idle SkelAnimation found "
                "(set animation_root or use People assets with clips)"
            )

    def _resolve_biped_usd(self) -> str:
        """Prefer ``biped_demo_meters.usd`` (clean skeleton); else ``Biped_Setup.usd``."""
        root = Path(self.character_root).expanduser() if self.character_root else None
        if root is None:
            return ""
        for candidate in (
            root / "biped_demo" / "biped_demo_meters.usd",
            root / "Biped_Setup.usd",
        ):
            if candidate.is_file() and candidate.stat().st_size > 0:
                return str(candidate.resolve())
        return ""

    def _spawn_hidden_biped(self, stage: Any) -> str:
        """Reference retarget-source biped off-stage and invisible.

        Uses ``biped_demo_meters.usd`` directly so we do not pull Biped_Setup's
        CharacterAnimation payloads (many missing / unused for Traffic gait).
        """
        from pxr import Sdf, UsdGeom

        biped = self._resolve_biped_usd()
        if not biped:
            return ""
        # Prefer a World path like Hunav so it is never under Traffic spawn.
        source_path = "/World/AutosimBiped_Setup"
        if not stage.GetPrimAtPath(source_path).IsValid():
            if not stage.GetPrimAtPath("/World").IsValid():
                source_path = f"{self.library}/Biped_Setup"
            src = stage.DefinePrim(Sdf.Path(source_path), "SkelRoot")
            try:
                src.GetReferences().AddReference(biped)
            except Exception as exc:
                logger.warning("biped reference failed (%s): %s", biped, exc)
                return ""
            try:
                xform = UsdGeom.Xformable(src)
                xform.AddTranslateOp().Set((-1000.0, -1000.0, -1000.0))
            except Exception:
                pass
            logger.info("retarget source biped → %s at %s", Path(biped).name, source_path)
        _make_invisible(stage.GetPrimAtPath(source_path))
        # Hide leftover library biped from older runs (may still hold Biped_Setup).
        legacy = stage.GetPrimAtPath(f"{self.library}/Biped_Setup")
        if legacy and legacy.IsValid():
            _make_invisible(legacy)
            try:
                UsdGeom.Xformable(legacy).AddTranslateOp().Set(
                    (-1000.0, -1000.0, -1000.0)
                )
            except Exception:
                pass
        return source_path

    def _ensure_retarget(self, stage: Any, character_prim: Any) -> None:
        """Retarget source biped clips onto the first character skeleton once."""
        if self.retarget_done:
            return
        self.retarget_done = True
        if self.walk_path is None and self.idle_path is None:
            return
        try:
            from autosim.backends.isaac.hunav.animation_utils import (
                setup_anim_retargeting,
            )
        except Exception as exc:
            logger.debug("retarget imports failed: %s", exc)
            return
        source_path = self._spawn_hidden_biped(stage)
        if not source_path:
            logger.warning(
                "no biped_demo_meters.usd / Biped_Setup.usd under character_root=%r "
                "— characters stay T-pose",
                self.character_root,
            )
            return
        biped_agent = _resolve_biped_agent_path(stage, source_path)
        source_dict = {}
        if self.idle_path is not None:
            source_dict[0] = self.idle_path
        if self.walk_path is not None:
            source_dict[1] = self.walk_path
        if not source_dict:
            return
        try:
            from pxr import UsdGeom
            from autosim.backends.isaac.hunav.animation_utils import find_skeleton_path

            biped_prim = stage.GetPrimAtPath(biped_agent)
            if find_skeleton_path(biped_prim) is None:
                logger.warning(
                    "Biped at %s has no Skeleton (need Characters/biped_demo/"
                    "biped_demo_meters.usd) — characters stay T-pose",
                    biped_agent,
                )
                return
            if not stage.GetPrimAtPath(self.retarget_parent).IsValid():
                UsdGeom.Xform.Define(stage, self.retarget_parent)
            _make_invisible(stage.GetPrimAtPath(self.retarget_parent))
            setup_anim_retargeting(
                stage,
                character_prim,
                source_dict,
                self.retarget_parent,
                source_biped_path=biped_agent,
            )
            self.retarget_idle_path = _retargeted_clip_paths(
                stage, self.retarget_parent, "idle"
            )
            self.retarget_walk_path = _retargeted_clip_paths(
                stage, self.retarget_parent, "walk"
            )
            if self.retarget_idle_path is None and self.retarget_walk_path is None:
                logger.warning(
                    "anim retarget produced no clips under %s (T-pose likely)",
                    self.retarget_parent,
                )
            else:
                tps = float(stage.GetTimeCodesPerSecond() or 24.0)
                walk_p = self._resolve_clip_prim(stage, self.retarget_walk_path)
                idle_p = self._resolve_clip_prim(stage, self.retarget_idle_path)
                if walk_p is not None:
                    self.walk_span = clip_span_seconds(walk_p, tps) or self.walk_span
                if idle_p is not None:
                    self.idle_span = clip_span_seconds(idle_p, tps) or self.idle_span
                logger.info(
                    "anim retarget ok idle=%s walk=%s spans=(%s,%s)",
                    self.retarget_idle_path,
                    self.retarget_walk_path,
                    self.idle_span,
                    self.walk_span,
                )
        except Exception as exc:
            logger.warning("anim retarget failed (T-pose likely): %s", exc)

    def _resolve_clip_prim(self, stage: Any, candidate: Any) -> Any | None:
        """Turn a path or prim into a valid SkelAnimation prim."""
        if candidate is None:
            return None
        if hasattr(candidate, "IsValid"):
            return candidate if candidate.IsValid() else None
        prim = stage.GetPrimAtPath(candidate)
        return prim if prim is not None and prim.IsValid() else None

    def _bind_anim_graph(self, stage: Any, character_prim: Any) -> bool:
        """Try AnimationGraph idle/walk blend; return False to use UsdSkel."""
        try:
            from autosim.backends.isaac.hunav.animation_utils import (
                apply_animation_graph,
                create_agent_animation_graph,
                find_skelroot_path,
            )
        except Exception:
            return False
        # Prefer retargeted clips (Hunav); fall back to source biped clips.
        idle = self.retarget_idle_path or self.idle_path or self.walk_path
        walk = self.retarget_walk_path or self.walk_path or self.idle_path
        if idle is None and walk is None:
            return False
        skel = find_skelroot_prim(character_prim)
        if skel is None:
            return False
        try:
            graph_path = create_agent_animation_graph(
                stage, skel, idle or walk, walk or idle
            )
            skel_path = find_skelroot_path(skel)
            target = (
                stage.GetPrimAtPath(skel_path)
                if skel_path is not None
                else skel
            )
            if not apply_animation_graph(
                target if target and target.IsValid() else skel, graph_path
            ):
                return False
            self.graph_paths[str(character_prim.GetPath())] = graph_path
            self.use_anim_graph = True
            return True
        except Exception as exc:
            logger.debug("AnimationGraph bind failed: %s", exc)
            return False

    def _bind_usd_skel(self, stage: Any, character_prim: Any) -> bool:
        """Bind retargeted SkelAnimation via UsdSkel BindingAPI.

        Source biped clips alone cannot drive People skins (different joints) —
        retarget must succeed first.
        """
        for candidate, kind in (
            (self.retarget_idle_path, "idle"),
            (self.retarget_walk_path, "walk"),
        ):
            anim = self._resolve_clip_prim(stage, candidate)
            if anim is not None and bind_animation_source(stage, character_prim, anim):
                tps = float(stage.GetTimeCodesPerSecond() or 24.0)
                span = clip_span_seconds(anim, tps)
                if kind == "walk":
                    self.walk_span = span or self.walk_span
                else:
                    self.idle_span = span or self.idle_span
                return True
        return False

    def bind_character(self, stage: Any, character_prim: Any) -> None:
        """Bind walk/idle to one character instance."""
        if not self.enabled or character_prim is None:
            return
        self.ensure_clips(stage, character_prim)
        path = str(character_prim.GetPath())
        if path in self.bound_paths:
            return
        self._ensure_retarget(stage, character_prim)
        # Prefer UsdSkel + retargeted clips: AnimGraph schema often broken in Lab.
        if self._bind_usd_skel(stage, character_prim):
            self.bound_paths.add(path)
            return
        if self._bind_anim_graph(stage, character_prim):
            self.bound_paths.add(path)
            return
        logger.warning("no animation bound for %s (T-pose)", path)

    def rebind_mode(self, stage: Any, mode: str) -> None:
        """Switch UsdSkel-bound characters to walk or idle clip."""
        if self.use_anim_graph:
            return
        if mode == "walk":
            candidates = (self.retarget_walk_path, self.walk_prim, self.idle_prim)
        else:
            candidates = (self.retarget_idle_path, self.idle_prim, self.walk_prim)
        anim = None
        for candidate in candidates:
            anim = self._resolve_clip_prim(stage, candidate)
            if anim is not None:
                break
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
        """Advance shared animation clock / AnimGraph speed."""
        if not self.enabled or stage is None:
            return
        mode = "walk" if any_moving and self.walk_prim is not None else "idle"
        speed = anim_rate(mean_speed, walk_mps=self.walk_mps, moving=mode == "walk")
        if self.use_anim_graph:
            try:
                from autosim.backends.isaac.hunav.animation_utils import (
                    find_skelroot_path,
                    set_anim_graph_speed,
                )
                import omni.anim.graph.core as ag  # type: ignore
            except Exception:
                ag = None
            weight = float(speed if mode == "walk" else 0.0)
            for char_path, graph_path in list(self.graph_paths.items()):
                char = None
                if ag is not None:
                    prim = stage.GetPrimAtPath(char_path)
                    if prim and prim.IsValid():
                        try:
                            skel_path = find_skelroot_path(prim)
                            char = ag.get_character(str(skel_path))
                        except Exception:
                            char = None
                set_anim_graph_speed(stage, char, graph_path, weight)
            # Keep Kit timeline running so the graph evaluates.
            self.anim_time += max(0.0, float(dt))
            set_stage_time(stage, self.anim_time)
            self.mode = mode
            return

        if mode != self.mode:
            self.mode = mode
            self.rebind_mode(stage, mode)
            self.anim_time = 0.0
        rate = speed
        if mode == "idle":
            rate = 1.0 if self.idle_span else 0.0
        self.anim_time += max(0.0, float(dt)) * rate
        span_info = self.walk_span if mode == "walk" else self.idle_span
        if span_info is None:
            set_stage_time(stage, self.anim_time)
            return
        start, span = span_info
        set_stage_time(stage, start + math.fmod(self.anim_time, span))


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
