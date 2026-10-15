#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Convenience: run navigate task with default config."""

from __future__ import annotations

from benchmark.cli.main import main

if __name__ == "__main__":
    main(["run", "--task", "navigate"], standalone_mode=True)
