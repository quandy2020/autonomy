# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""``python -m benchmark``."""

from __future__ import annotations

import sys

from benchmark.main import main

if __name__ == "__main__":
    main(prog_name="benchmark")
    sys.exit(0)
