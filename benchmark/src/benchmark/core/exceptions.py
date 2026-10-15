# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Benchmark-specific exceptions."""

from __future__ import annotations


class BenchmarkError(Exception):
    """Base error for the benchmark package."""


class ConfigError(BenchmarkError):
    """Invalid or missing configuration."""


class RegistryError(BenchmarkError):
    """Unknown plugin / factory name."""


class TaskError(BenchmarkError):
    """Task execution failure."""


class SimulatorError(BenchmarkError):
    """Simulator adapter failure."""
