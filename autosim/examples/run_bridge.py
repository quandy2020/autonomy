#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autosim.runner import main

if __name__ == "__main__":
    main(["--config", str(ROOT / "config" / "default.yaml")])
