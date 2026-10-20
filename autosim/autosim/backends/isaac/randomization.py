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

"""Episode randomization (lights · People assets) — Isaac Lab–style events."""

from __future__ import annotations

import logging
import random
from typing import Any, Mapping

logger = logging.getLogger(__name__)


def randomization_config(isaac_block: Mapping[str, Any] | None) -> dict[str, Any]:
    """Parse ``habitat.isaac.domain_random`` with defaults (off)."""
    block = {}
    if isinstance(isaac_block, Mapping):
        raw = isaac_block.get("domain_random")
        if isinstance(raw, Mapping):
            block = dict(raw)
    return {
        "enabled": bool(block.get("enabled", False)),
        "light_intensity": tuple(
            float(v) for v in (block.get("light_intensity") or (800.0, 2500.0))
        )[:2],
        "light_color_jitter": float(block.get("light_color_jitter", 0.15)),
        "shuffle_characters": bool(block.get("shuffle_characters", True)),
        "seed": block.get("seed"),
    }


def apply_randomization(
    stage: Any,
    *,
    people_sync: Any = None,
    config: Mapping[str, Any] | None = None,
    rng: random.Random | None = None,
) -> dict[str, Any]:
    """Randomize lights and optionally reshuffle People character assets.

    Returns a small report dict (for logs / dataset meta). No-op when disabled
    or when Kit USD APIs are unavailable.
    """
    cfg = dict(config or {})
    report: dict[str, Any] = {"applied": False}
    if not cfg.get("enabled", False):
        return report
    rng = rng or random.Random(cfg.get("seed"))
    intensity_lo, intensity_hi = cfg.get("light_intensity", (800.0, 2500.0))
    jitter = float(cfg.get("light_color_jitter", 0.15))
    lights = 0
    try:
        from pxr import Usd, UsdLux

        usd_stage = getattr(stage, "stage", stage)
        if usd_stage is None:
            return report
        for prim in usd_stage.Traverse():
            if not (
                prim.IsA(UsdLux.DistantLight)
                or prim.IsA(UsdLux.DomeLight)
                or prim.IsA(UsdLux.SphereLight)
                or prim.IsA(UsdLux.DiskLight)
                or prim.IsA(UsdLux.RectLight)
            ):
                continue
            intensity = rng.uniform(float(intensity_lo), float(intensity_hi))
            try:
                light = UsdLux.LightAPI(prim)
                attr = light.GetIntensityAttr()
                if attr:
                    attr.Set(float(intensity))
                color_attr = light.GetColorAttr()
                if color_attr and jitter > 0.0:
                    base = color_attr.Get() or (1.0, 1.0, 1.0)
                    color_attr.Set(
                        tuple(
                            max(0.0, min(1.0, float(c) + rng.uniform(-jitter, jitter)))
                            for c in base[:3]
                        )
                    )
                lights += 1
            except Exception:
                continue
    except Exception as exc:
        logger.debug("domain_random lights skipped: %s", exc)

    shuffled = 0
    if cfg.get("shuffle_characters", True) and people_sync is not None:
        assets = list(getattr(people_sync, "character_assets", None) or [])
        if len(assets) > 1:
            rng.shuffle(assets)
            people_sync.character_assets = assets
            shuffled = len(assets)
            # Drop cached prims so next sync rebinds references.
            for attr in ("character_prims", "agents", "_prims", "prim_by_id"):
                if hasattr(people_sync, attr):
                    try:
                        setattr(people_sync, attr, {})
                    except Exception:
                        pass

    report.update({"applied": True, "lights": lights, "characters": shuffled})
    if lights or shuffled:
        logger.info(
            "domain_random: lights=%d character_pool=%d", lights, shuffled
        )
    return report


# Back-compat aliases
domain_random_config = randomization_config
apply_domain_random = apply_randomization
