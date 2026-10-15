# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field


class RunRequest(BaseModel):
    config: str = "configs/default.yaml"
    task: Optional[str] = None
    mode: Optional[str] = None
    overrides: List[str] = Field(default_factory=list)


class RunResponse(BaseModel):
    summary: Dict[str, Any]
