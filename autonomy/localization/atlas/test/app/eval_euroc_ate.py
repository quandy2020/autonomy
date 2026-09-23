#!/usr/bin/env python3
"""Backward-compatible wrapper → eval_ate.py --gt_format=euroc """
from eval_ate import main
import sys

if __name__ == "__main__":
    if "--gt_format" not in sys.argv:
        sys.argv[1:1] = ["--gt_format", "euroc"]
    main()
