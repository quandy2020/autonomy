# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""RL training defaults."""

from __future__ import annotations

from typing import Any, Dict


DEFAULT_TRAIN: Dict[str, Any] = {
    "episodes": 50,
    "seed": 0,
    "curriculum": True,
    "learning_rate": 0.05,
    "gamma": 0.99,
}


DEFAULT_ENV: Dict[str, Any] = {
    "mode": "parametrized",
    "max_steps": 200,
    "mode_config": {"difficulty": 0.4},
}
