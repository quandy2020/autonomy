# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Create human / crowd models from config."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.humans.base import HumanModel
from benchmark.humans.behaviors import resolve_behavior
from benchmark.humans.crowd_flow import CrowdFlowHumans
from benchmark.humans.social_force.pedestrian import SocialForceHumans

_MODELS = {
    "social_force": SocialForceHumans,
    "crowd_flow": CrowdFlowHumans,
}


def create_human_model(config: Optional[Dict[str, Any]] = None) -> HumanModel:
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
