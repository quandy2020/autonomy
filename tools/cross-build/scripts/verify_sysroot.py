#!/usr/bin/env python3
"""Structural checks only; does not execute target code."""
import sys
from cross_build import main

if __name__ == '__main__':
    sys.exit(main(['verify', *sys.argv[1:]]))
