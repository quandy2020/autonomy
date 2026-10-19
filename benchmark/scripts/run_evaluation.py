#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Run navigate + write evaluation report."""

from __future__ import annotations

from benchmark.main import main

if __name__ == "__main__":
    main(["run", "--task", "navigate", "--report-dir", "data/eval_results"], standalone_mode=True)
