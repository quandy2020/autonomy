# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Union


def read_json(path: Union[str, Path]) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path: Union[str, Path], data: Any) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, indent=2), encoding="utf-8")
