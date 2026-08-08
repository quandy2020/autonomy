#!/usr/bin/env python3
"""Bootstrap PYTHONPATH for autoviz Python examples (autolink + automsgs)."""

from __future__ import annotations

import os
import sys

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import subprocess
from pathlib import Path


def _append(path: Path) -> None:
    if path.is_dir():
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def _append_lib(path: Path) -> None:
    if not path.is_dir():
        return
    current = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [part for part in current.split(os.pathsep) if part]
    text = str(path)
    if text not in parts:
        parts.insert(0, text)
        os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(parts)


def _find_repo_root(start: Path) -> Path | None:
    current = start.resolve()
    for candidate in [current, *current.parents]:
        if (candidate / "build" / "autonomy").is_dir():
            return candidate
        if (candidate / "src" / "autonomy" / "autoviz").is_dir():
            return candidate
    return None


def autolink_python_dirs(repo_root: Path | None) -> list[Path]:
    if repo_root is None:
        return []
    override = os.environ.get("AUTOLINK_PYTHON_DIR")
    dirs: list[Path] = []
    if override:
        dirs.append(Path(override) / "python")
    dirs.extend(
        [
            repo_root / "build" / "autonomy" / "python",
            repo_root / "build" / "autolink-python" / "python",
            repo_root / "install" / "autonomy" / "lib" / "python",
            Path(os.environ.get("AUTOLINK_DISTRIBUTION_HOME", "/usr/local")) / "python",
        ]
    )
    return dirs


def find_autolink_python_dir(repo_root: Path | None) -> Path | None:
    for base in autolink_python_dirs(repo_root):
        if any(base.glob("autolink/_core*.so")):
            return base
    return None


def has_autolink_python(repo_root: Path | None) -> bool:
    return find_autolink_python_dir(repo_root) is not None


def system_python_candidates() -> list[Path]:
    override = os.environ.get("AUTOVIZ_PYTHON")
    if override:
        return [Path(override)]
    return [
        Path("/usr/bin/python3.10"),
        Path("/usr/bin/python3"),
    ]


def _python_is_usable() -> bool:
    from _reexec import python_env_is_polluted

    if python_env_is_polluted():
        return False
    try:
        import google.protobuf  # noqa: F401
    except ImportError:
        return False
    return True


def reexec_with_preferred_python() -> None:
    """Re-run with system Python when Isaac Sim / venv pollutes the interpreter."""
    from _reexec import clean_exec_env, early_reexec_if_needed, python_env_is_polluted, system_python

    if os.environ.get("AUTOVIZ_PY_REEXEC") == "1":
        return
    if _python_is_usable():
        return

    candidate = system_python()
    if candidate is None:
        return
    if python_env_is_polluted() or Path(candidate).resolve() != Path(sys.executable).resolve():
        os.execve(candidate, [candidate, *sys.argv], clean_exec_env())


def setup_example_pythonpath() -> Path | None:
    """Add autolink + automsgs generated modules to ``sys.path``."""
    script_dir = Path(__file__).resolve().parent
    repo_root = _find_repo_root(script_dir)

    build_root = Path(os.environ.get("AUTONOMY_BUILD_DIR", ""))
    if not build_root.is_dir() and repo_root is not None:
        build_root = repo_root / "build" / "autonomy"

    autolink_py = find_autolink_python_dir(repo_root)
    autolink_py_root = Path(os.environ.get("AUTOLINK_PYTHON_DIR", ""))
    if not autolink_py_root.is_dir() and autolink_py is not None:
        autolink_py_root = autolink_py.parent

    install_root = Path(os.environ.get("AUTONOMY_INSTALL_PREFIX", ""))
    if not install_root.is_dir() and repo_root is not None:
        install_root = repo_root / "install" / "autonomy"

    candidates = [
        build_root / "automsgs" / "proto" / "gen" / "python",
        install_root / "lib" / "python",
    ]
    if autolink_py is not None:
        candidates.insert(0, autolink_py)
    if repo_root is not None:
        candidates.append(
            repo_root / "build" / "autonomy" / "automsgs" / "proto" / "gen" / "python"
        )

    lib_candidates = [
        autolink_py_root / "lib",
        build_root / "lib",
        install_root / "lib",
    ]
    if repo_root is not None:
        lib_candidates.extend(
            [
                repo_root / "build" / "autolink-python" / "lib",
                repo_root / "build" / "autonomy" / "lib",
                repo_root / "install" / "autonomy" / "lib",
            ]
        )

    for path in candidates:
        _append(path)
    for path in lib_candidates:
        _append_lib(path)

    return repo_root


def _clean_build_env() -> dict[str, str]:
    env = os.environ.copy()
    env["PATH"] = "/usr/bin:/bin:/usr/local/bin:" + env.get("PATH", "")
    for key in ("PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV", "CONDA_PREFIX"):
        env.pop(key, None)
    env.setdefault("AUTOVIZ_PYTHON", "/usr/bin/python3")
    return env


def build_autolink_python(repo_root: Path | None) -> None:
    if repo_root is None:
        raise RuntimeError("Could not locate autonomy repository root.")

    setup_script = (
        repo_root / "src" / "autonomy" / "autoviz" / "examples" / "python" /
        "setup_autolink_python.sh")
    if not setup_script.is_file():
        raise RuntimeError(f"Missing setup script: {setup_script}")

    print("Autolink Python bindings not found; building (first run only)...",
          flush=True)
    subprocess.run(["bash", str(setup_script)], check=True, env=_clean_build_env())


def ensure_autolink_python(repo_root: Path | None) -> None:
    """Ensure autolink Python bindings exist (optional auto-build)."""
    if has_autolink_python(repo_root):
        return
    if os.environ.get("AUTOVIZ_SKIP_AUTOLINK_BUILD") == "1":
        return
    if os.environ.get("AUTOVIZ_AUTO_BUILD_AUTOLINK") != "1":
        return
    try:
        build_autolink_python(repo_root)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            autolink_python_build_hint(repo_root) +
            "\n\nAuto-build failed (often protobuf mismatch in standalone cmake). "
            "Use colcon build instead."
        ) from exc


def autolink_python_build_hint(repo_root: Path | None) -> str:
    root = repo_root or Path("/path/to/autonomy")
    return (
        "Autolink Python bindings are unavailable.\n"
        "Expected after colcon build (AUTOLINK_BUILD_PYTHON=ON by default):\n"
        f"  {root / 'build/autonomy/python/autolink/_core*.so'}\n\n"
        "If you built before this default changed, reconfigure once:\n"
        "  colcon build --cmake-args -DAUTOLINK_BUILD_PYTHON=ON\n\n"
        "Or use standalone build (uses /usr/bin/python3, not Isaac Sim Python):\n"
        f"  {root / 'src/autonomy/autoviz/examples/python/setup_autolink_python.sh'}\n\n"
        "Run the tutorial with system Python:\n"
        "  /usr/bin/python3 examples/python/01_tutorial_poses.py\n\n"
        "Manual cmake:\n"
        f"  cmake -S {root / 'src/autonomy/autolink'} "
        f"-B {root / 'build/autolink-python'} -DAUTOLINK_BUILD_PYTHON=ON\n"
        f"  cmake --build {root / 'build/autolink-python'} -j$(nproc)"
    )


def prepare_example_environment() -> Path:
    """Configure paths and verify protobuf plus autolink bindings."""
    reexec_with_preferred_python()
    repo_root = setup_example_pythonpath()
    ensure_autolink_python(repo_root)

    try:
        import google.protobuf  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "Missing protobuf. Install it, then re-run:\n"
            "  pip3 install protobuf"
        ) from exc

    if not has_autolink_python(repo_root):
        raise SystemExit(autolink_python_build_hint(repo_root))

    return repo_root or Path.cwd()
