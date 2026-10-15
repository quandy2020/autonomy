# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Create and run a task from BenchmarkConfig."""

from __future__ import annotations

from typing import List

from benchmark.core.config import BenchmarkConfig
from benchmark.core.registry import get_registry
from benchmark.core.types import EpisodeResult
from benchmark.tasks.base import Task


def create_task(config: BenchmarkConfig) -> Task:
    hub = get_registry()
    hub.ensure_builtins()
    return hub.tasks.create(config.task_name, config)


def run_task(config: BenchmarkConfig) -> List[EpisodeResult]:
    task = create_task(config)
    return task.run()
