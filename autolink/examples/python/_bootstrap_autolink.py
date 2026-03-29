#!/usr/bin/env python3

import importlib
import importlib.util
import os
import sys
from pathlib import Path


def _append_if_exists(path: Path) -> None:
    if path.exists():
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)


def setup_autolink_pythonpath() -> None:
    """Best-effort path bootstrap for running examples from source/docker."""
    cwd = Path.cwd()

    # Typical source-tree entry: /workspace/autonomy/src/autonomy/autolink/examples/python
    # Ensure `from autolink.python.autolink_py3 ...` fallback can work.
    workspace_src_root = cwd
    for _ in range(8):
        if (workspace_src_root / "src" / "autonomy").exists():
            break
        if workspace_src_root.parent == workspace_src_root:
            break
        workspace_src_root = workspace_src_root.parent
    _append_if_exists(workspace_src_root / "src" / "autonomy")

    # Installed/default prefixes for package + C-extension wrappers.
    autolink_home = Path(os.environ.get("AUTOLINK_DISTRIBUTION_HOME", "/usr/local"))
    _append_if_exists(autolink_home / "python")
    _append_if_exists(autolink_home / "lib" / "autolink" / "python" / "internal")

    # Common build/install locations in this repo.
    _append_if_exists(workspace_src_root / "build" / "autonomy" / "python")
    _append_if_exists(workspace_src_root / "build" / "autonomy" / "lib" / "autolink" / "python" / "internal")
    _append_if_exists(workspace_src_root / "build" / "autolink" / "python")
    _append_if_exists(workspace_src_root / "build" / "autolink" / "lib" / "autolink" / "python" / "internal")
    _append_if_exists(workspace_src_root / "install" / "python")
    _append_if_exists(workspace_src_root / "install" / "lib" / "autolink" / "python" / "internal")

    # Source-tree fallback:
    # map autolink/autolink/python/src/__init__.py as package "autolink_py3"
    # so examples can run without installation.
    try:
        importlib.import_module("autolink_py3")
        return
    except ModuleNotFoundError:
        pass

    source_pkg_dir = workspace_src_root / "src" / "autonomy" / "autolink" / "autolink" / "python" / "src"
    init_file = source_pkg_dir / "__init__.py"
    if not init_file.exists():
        return

    spec = importlib.util.spec_from_file_location(
        "autolink_py3", str(init_file), submodule_search_locations=[str(source_pkg_dir)]
    )
    if spec is None or spec.loader is None:
        return

    module = importlib.util.module_from_spec(spec)
    sys.modules["autolink_py3"] = module
    spec.loader.exec_module(module)
