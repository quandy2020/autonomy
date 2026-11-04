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

"""Mirror CPU Traffic pedestrians into Habitat kinematic SMPL-X humanoids."""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Mapping, Sequence

import numpy as np

from autosim.backends.habitat.humanoids.actor import HumanoidActor, load_actor
from autosim.backends.habitat.humanoids.assets import (
    HumanoidAsset,
    discover_humanoid_root,
    list_humanoid_assets,
)

logger = logging.getLogger(__name__)


def _parse_skin_map(raw: Any) -> dict[int, str]:
    """``{1: male_46, …}`` or ``{\"1\": \"male_46\"}`` → int→name."""
    if not isinstance(raw, Mapping):
        return {}
    out: dict[int, str] = {}
    for key, value in raw.items():
        name = str(value or "").strip()
        if not name:
            continue
        try:
            out[int(key)] = name
        except (TypeError, ValueError):
            continue
    return out


def humanoid_config_from_habitat(habitat: Mapping[str, Any] | None) -> dict[str, Any]:
    """Read ``habitat.humanoids`` with defaults."""
    block: dict[str, Any] = {}
    if isinstance(habitat, Mapping):
        raw = habitat.get("humanoids")
        if isinstance(raw, Mapping):
            block = dict(raw)
    names = block.get("names") or []
    if isinstance(names, str):
        names = [names]
    visual = str(block.get("visual") or "humanoid").strip().lower()
    return {
        "enabled": bool(block.get("enabled", False)),
        "root": str(block.get("root") or "").strip(),
        "visual": visual,
        "names": [str(n).strip() for n in names if str(n).strip()],
        "skin_map": _parse_skin_map(block.get("skin_map")),
        "sync_hz": float(block.get("sync_hz", 30.0)),
        "snap_navmesh": bool(block.get("snap_navmesh", True)),
        "base_offset_y": float(block.get("base_offset_y", 0.9)),
        "motion_fps": float(block.get("motion_fps", 30.0)),
    }


def maybe_create_humanoid_sync(
    session: Any,
    *,
    config: Mapping[str, Any],
    floor_y: float = 0.0,
    pathfinder: Any = None,
) -> "HumanoidSync | None":
    """Build a sync when enabled and assets exist; else None."""
    if not bool(config.get("enabled", False)):
        return None
    visual = str(config.get("visual") or "humanoid").strip().lower()
    if visual in ("none", "off", "disabled"):
        return None
    root = discover_humanoid_root(str(config.get("root") or ""))
    assets = list_humanoid_assets(root, names=config.get("names") or [])
    if not assets:
        logger.warning(
            "habitat.humanoids enabled but no URDF/motion assets under %s "
            "(set AUTOSIM_HUMANOID_ROOT or habitat.humanoids.root)",
            root or "(empty)",
        )
        return None
    return HumanoidSync(
        session,
        assets=assets,
        skin_map=dict(config.get("skin_map") or {}),
        sync_hz=float(config.get("sync_hz", 30.0)),
        snap_navmesh=bool(config.get("snap_navmesh", True)),
        floor_y=float(floor_y),
        pathfinder=pathfinder,
        base_offset_y=float(config.get("base_offset_y", 0.9)),
        motion_fps=float(config.get("motion_fps", 30.0)),
    )


def resolve_humanoid_asset(
    assets: Sequence[HumanoidAsset],
    *,
    index: int,
    skin: int = 0,
    avatar: str = "",
    skin_map: Mapping[int, str] | None = None,
) -> HumanoidAsset:
    """Pick an asset for one pedestrian slot.

    Priority: ``avatar`` name → ``skin_map[skin]`` → 1-based ``skin`` index
    into ``assets`` → round-robin by ``index``.
    """
    if not assets:
        raise ValueError("resolve_humanoid_asset requires a non-empty assets list")
    by_name = {a.name.lower(): a for a in assets}
    name = str(avatar or "").strip()
    if not name and skin_map and int(skin) > 0:
        name = str(skin_map.get(int(skin)) or "").strip()
    if name:
        hit = by_name.get(name.lower())
        if hit is not None:
            return hit
        logger.warning(
            "humanoid avatar %r not in asset list; falling back (known: %s)",
            name,
            ", ".join(a.name for a in assets),
        )
    if int(skin) > 0:
        return assets[(int(skin) - 1) % len(assets)]
    return assets[int(index) % len(assets)]


class HumanoidSync:
    """Spawn / update kinematic humanoids from ``Traffic.pedestrian`` poses."""

    def __init__(
        self,
        session: Any,
        *,
        assets: Sequence[HumanoidAsset],
        skin_map: Mapping[int, str] | None = None,
        sync_hz: float = 30.0,
        snap_navmesh: bool = True,
        floor_y: float = 0.0,
        pathfinder: Any = None,
        base_offset_y: float = 0.9,
        motion_fps: float = 30.0,
    ) -> None:
        self.session = session
        self.assets = list(assets)
        self.skin_map = dict(skin_map or {})
        self.sync_period = 1.0 / float(sync_hz) if float(sync_hz) > 0.0 else 0.0
        self.snap_navmesh = bool(snap_navmesh)
        self.floor_y = float(floor_y)
        self.pathfinder = pathfinder
        self.base_offset_y = float(base_offset_y)
        self.motion_fps = float(motion_fps)
        self.actors: list[HumanoidActor] = []
        self.last_sync_mono = 0.0
        self._spawn_warned = False
        self._habitat_sim: Any = None
        self._mn: Any = None

    def _ensure_imports(self) -> bool:
        if self._habitat_sim is not None and self._mn is not None:
            return True
        try:
            import habitat_sim
            import magnum as mn
        except ImportError as exc:
            if not self._spawn_warned:
                logger.warning("humanoid sync needs habitat-sim/magnum: %s", exc)
                self._spawn_warned = True
            return False
        self._habitat_sim = habitat_sim
        self._mn = mn
        return True

    def _asset_for(
        self,
        index: int,
        *,
        skin: int = 0,
        avatar: str = "",
    ) -> HumanoidAsset:
        return resolve_humanoid_asset(
            self.assets,
            index=index,
            skin=skin,
            avatar=avatar,
            skin_map=self.skin_map,
        )

    def _spawn_one(self, asset: HumanoidAsset) -> HumanoidActor | None:
        try:
            return load_actor(
                self.session,
                asset,
                habitat_sim=self._habitat_sim,
                mn=self._mn,
                base_offset_y=self.base_offset_y,
                motion_fps=self.motion_fps,
            )
        except Exception as exc:  # noqa: BLE001
            if not self._spawn_warned:
                logger.warning("humanoid spawn failed (%s): %s", asset.name, exc)
                self._spawn_warned = True
            return None

    def _ensure_actors(
        self,
        count: int,
        *,
        skins: np.ndarray,
        avatars: np.ndarray,
    ) -> None:
        if not self._ensure_imports():
            return
        while len(self.actors) > count:
            actor = self.actors.pop()
            actor.remove()
        for index in range(count):
            skin = int(skins[index]) if index < len(skins) else 0
            avatar = (
                str(avatars[index])
                if index < len(avatars) and avatars[index] is not None
                else ""
            )
            asset = self._asset_for(index, skin=skin, avatar=avatar)
            if index < len(self.actors):
                current = self.actors[index]
                if current.asset.name == asset.name:
                    continue
                replacement = self._spawn_one(asset)
                if replacement is None:
                    break
                current.remove()
                self.actors[index] = replacement
            else:
                actor = self._spawn_one(asset)
                if actor is None:
                    break
                self.actors.append(actor)

    def _snap_xy(self, x: float, y: float) -> tuple[float, float, float]:
        """Project onto navmesh free space; returns (x, y, floor_y).

        Keeps kinematic meshes on the navigable surface even when CPU Traffic
        briefly clips a wall (Field bounce is planar; snap is the Habitat GT).
        """
        floor_y = self.floor_y
        if not self.snap_navmesh or self.pathfinder is None:
            return float(x), float(y), floor_y
        try:
            mn = self._mn
            if mn is None:
                return float(x), float(y), floor_y
            # Habitat: (X, Y, Z) with map_y = -Z.
            query = mn.Vector3(float(x), float(floor_y), -float(y))
            navigable = getattr(self.pathfinder, "is_navigable", None)
            if callable(navigable):
                try:
                    if bool(navigable(query)):
                        # Still refresh floor height from snap when available.
                        snap = getattr(self.pathfinder, "snap_point", None)
                        if callable(snap):
                            snapped = snap(query)
                            if snapped is not None and np.isfinite(float(snapped[1])):
                                return float(x), float(y), float(snapped[1])
                        return float(x), float(y), floor_y
                except Exception:
                    pass
            snapped = self.pathfinder.snap_point(query)
            if snapped is None:
                return float(x), float(y), floor_y
            sx = float(snapped[0])
            sy = -float(snapped[2])
            fy = float(snapped[1])
            if not (np.isfinite(sx) and np.isfinite(sy) and np.isfinite(fy)):
                return float(x), float(y), floor_y
            return sx, sy, fy
        except Exception:
            return float(x), float(y), floor_y

    def sync(self, traffic: Any) -> None:
        """Mirror Traffic pedestrian poses (rate-limited)."""
        if traffic is None or self.session is None or not self.assets:
            return
        now = time.monotonic()
        dt = 0.0 if self.last_sync_mono <= 0.0 else max(0.0, now - self.last_sync_mono)
        if self.sync_period > 0.0 and dt < self.sync_period and self.last_sync_mono > 0.0:
            return
        self.last_sync_mono = now
        gait_dt = dt if dt > 0.0 else (self.sync_period or 1.0 / 30.0)

        pedestrian = getattr(traffic, "pedestrian", None)
        if pedestrian is None:
            self._ensure_actors(0, skins=np.zeros(0, dtype=np.int32), avatars=np.zeros(0, dtype=object))
            return
        poses = (
            pedestrian.poses()
            if hasattr(pedestrian, "poses")
            else np.zeros((0, 3), dtype=np.float64)
        )
        poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
        velocity = np.asarray(
            getattr(pedestrian, "velocity", []), dtype=np.float64
        ).reshape(-1, 2)
        skins = np.asarray(getattr(pedestrian, "skins", []), dtype=np.int32).reshape(-1)
        avatars = np.asarray(
            getattr(pedestrian, "avatars", []), dtype=object
        ).reshape(-1)
        count = int(poses.shape[0])
        self._ensure_actors(count, skins=skins, avatars=avatars)
        for i, actor in enumerate(self.actors):
            if i >= count:
                break
            mx, my, yaw = float(poses[i, 0]), float(poses[i, 1]), float(poses[i, 2])
            sx, sy, floor_y = self._snap_xy(mx, my)
            # Only pull Traffic back when clearly off-mesh; constant writeback
            # fights SFM and freezes walkers on the same snap cell.
            if math.hypot(sx - mx, sy - my) > 0.35:
                try:
                    if hasattr(pedestrian, "position") and i < int(
                        pedestrian.position.shape[0]
                    ):
                        pedestrian.position[i, 0] = sx
                        pedestrian.position[i, 1] = sy
                except Exception:
                    pass
                mx, my = sx, sy
            else:
                mx, my = sx, sy
            speed = 0.0
            if i < velocity.shape[0]:
                speed = float(np.linalg.norm(velocity[i]))
            # Keep gait cycling even at low SFM speed so meshes do not idle-Tpose.
            if speed < 0.12:
                speed = max(speed, 0.35)
            actor.apply_pose(
                mx, my, yaw, floor_y=floor_y, speed_mps=speed, dt=gait_dt
            )

    def close(self) -> None:
        """Remove all humanoid AOs."""
        for actor in self.actors:
            actor.remove()
        self.actors.clear()
