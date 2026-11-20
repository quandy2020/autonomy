#!/usr/bin/env python3
"""Compatibility entry point for sysroot collection."""
import sys
from cross_build import main

if __name__ == '__main__':
    sys.exit(main(['sysroot', *sys.argv[1:]]))
