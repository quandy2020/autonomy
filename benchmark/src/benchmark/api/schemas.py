# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Pydantic request/response models for the HTTP API."""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field


class RunRequest(BaseModel):
    config: str = "configs/default.yaml"
    task: Optional[str] = None
    mode: Optional[str] = None
    overrides: List[str] = Field(default_factory=list)


class EvalRequest(BaseModel):
    config: str = "configs/default.yaml"
    episodes: Optional[int] = 5
    overrides: List[str] = Field(default_factory=list)


class TrainRequest(BaseModel):
    episodes: int = 30
    curriculum: bool = True
    seed: int = 0
    export: Optional[str] = None


class GenerateWorldRequest(BaseModel):
    name: Optional[str] = None
    prompt: Optional[str] = None
    seed: int = 0
