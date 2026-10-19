# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

import pytest

from benchmark.common.config import package_root


@pytest.fixture
def repo_root() -> Path:
    return package_root()


@pytest.fixture
def default_config_path(repo_root: Path) -> Path:
    return repo_root / "configs" / "default.yaml"
