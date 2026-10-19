# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Create human / crowd models from config (Arena SFM registry)."""

from __future__ import annotations

from typing import Any

from benchmark.simulators.humans.base import HumanModel
from benchmark.simulators.humans.behaviors import resolve_behavior
from benchmark.simulators.humans.crowd_flow import CrowdFlowHumans
from benchmark.simulators.humans.evacuation import EvacuationHumans
from benchmark.simulators.humans.hunav import HuNavHumans
from benchmark.simulators.humans.orca import OrcaHumans, Rvo2Humans
from benchmark.simulators.humans.pysocialforce import (
    DeepSocialHumans,
    LightSfmHumans,
    PedsimHumans,
    PySocialForceHumans,
)
from benchmark.simulators.humans.social_force.pedestrian import SocialForceHumans
from benchmark.simulators.humans.spinny import SpinnyHumans

_MODELS = {
    "social_force": SocialForceHumans,
    "crowd_flow": CrowdFlowHumans,
    "orca": OrcaHumans,
    "rvo2": Rvo2Humans,
    "evacuation": EvacuationHumans,
    "spinny": SpinnyHumans,
    "hunav": HuNavHumans,
    "pysocialforce": PySocialForceHumans,
    "deep_social": DeepSocialHumans,
    "lightsfm": LightSfmHumans,
    "pedsim": PedsimHumans,
}


def create_human_model(config: dict[str, Any] | None = None) -> HumanModel:
    cfg = dict(config or {})
    name = str(cfg.get("name") or cfg.get("model") or "social_force").strip().lower()
    behavior = cfg.get("behavior")
    if behavior:
        cfg.update(resolve_behavior(str(behavior)))
    factory = _MODELS.get(name)
    if factory is None:
        known = ", ".join(sorted(_MODELS))
        raise ValueError(f"unknown human model {name!r}; known: {known}")
    return factory(cfg)


def list_human_models() -> list[str]:
    return sorted(_MODELS)
