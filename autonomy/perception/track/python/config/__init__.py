# Copyright 2026 The Openbot Authors
"""YAML config loader for track training."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

_DEFAULT = Path(__file__).with_name("default.yaml")


def load_config(path: str | None = None) -> dict[str, Any]:
    config_path = Path(path) if path else _DEFAULT
    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    config["lattice_size"] = int(config["horizon_num"]) * int(
        config["vertical_num"]
    )
    config["segment_time_s"] = (
        2.0 * float(config["radio_range"]) / float(config["vel_max"])
    )
    return config
