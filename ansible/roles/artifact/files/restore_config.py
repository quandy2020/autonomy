#!/usr/bin/env python3
"""Restore a configuration snapshot, preserving the failed tree if recovery errors."""
import os
from pathlib import Path
import sys


def restore_config(active, backup):
    saved = backup / 'config'
    failed = backup / 'failed-config'
    if (active.is_symlink() or not active.is_dir() or saved.is_symlink()
            or not saved.is_dir() or failed.exists() or failed.is_symlink()):
        raise ValueError('expected live and saved configuration directories and an unused recovery path')
    if active.stat().st_dev != saved.stat().st_dev:
        raise ValueError('configuration and snapshot must be on the same filesystem')
    os.rename(active, failed)
    try:
        os.rename(saved, active)
    except OSError:
        os.rename(failed, active)
        raise


if __name__ == '__main__':
    try:
        restore_config(Path(sys.argv[1]), Path(sys.argv[2]))
    except (OSError, ValueError) as error:
        sys.exit(str(error))
