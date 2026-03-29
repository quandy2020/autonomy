#!/usr/bin/env python3

"""Helper to import autolink_py3 in both install-tree and source-tree layouts."""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
from pathlib import Path


def _try_import() -> object | None:
    try:
        return importlib.import_module("autolink_py3")
    except ModuleNotFoundError:
        return None


def _load_from_source_tree(src_dir: Path) -> object | None:
    init_file = src_dir / "__init__.py"
    if not init_file.exists():
        return None

    spec = importlib.util.spec_from_file_location(
        "autolink_py3", str(init_file), submodule_search_locations=[str(src_dir)]
    )
    if spec is None or spec.loader is None:
        return None

    module = importlib.util.module_from_spec(spec)
    sys.modules["autolink_py3"] = module
    spec.loader.exec_module(module)
    return module


def import_autolink_py3():
    module = _try_import()
    if module is not None:
        return module

    # 1) install-tree via AUTOLINK_DISTRIBUTION_HOME
    home = os.environ.get("AUTOLINK_DISTRIBUTION_HOME")
    if home:
        candidate = Path(home) / "python"
        if candidate.exists():
            sys.path.insert(0, str(candidate))
            module = _try_import()
            if module is not None:
                return module

    # 2) common install prefixes
    for prefix in (Path("/usr/local/python"), Path("/usr/python")):
        if prefix.exists():
            sys.path.insert(0, str(prefix))
            module = _try_import()
            if module is not None:
                return module

    # 3) source-tree fallback: autolink/autolink/python/src
    # current: .../autolink/examples/python/_autolink_py_loader.py
    # fallback: .../autolink/autolink/python/src
    source_dir = Path(__file__).resolve().parents[2] / "autolink" / "python" / "src"
    module = _load_from_source_tree(source_dir)
    if module is not None:
        return module

    raise ModuleNotFoundError(
        "Cannot import autolink_py3. Set AUTOLINK_DISTRIBUTION_HOME/PYTHONPATH "
        "or run from a repository containing autolink/autolink/python/src."
    )
