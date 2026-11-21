#!/usr/bin/env python3
"""package_install_tarball.py — build and pack an install-tree tarball.

Used for Ansible artifact deployment.

Usage:
  python3 tools/python/package_install_tarball.py
  python3 tools/python/package_install_tarball.py --output /tmp/autonomy.tar.gz
  BUNDLE_CONFIG=OFF python3 tools/python/package_install_tarball.py
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
from datetime import datetime, timezone
from pathlib import Path


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def bundle_configuration(root: Path, staged_prefix: Path) -> None:
    """Keep the deployment layout aligned with the component-owned conf trees."""
    destination = staged_prefix / "share" / "autonomy" / "config"
    for source in sorted((root / "autonomy").glob("*/conf")):
        shutil.copytree(source, destination / source.parent.name, dirs_exist_ok=True)


def bundle_runtime_libraries(build_dir: Path, staged_prefix: Path) -> list[str]:
    """Collect linked ELF dependencies; libc, C++ runtime and GPU drivers stay on the target."""
    target_libraries = {'libc.so.6', 'libm.so.6', 'libpthread.so.0', 'libdl.so.2', 'librt.so.1',
                        'libresolv.so.2', 'libutil.so.1', 'libanl.so.1', 'libstdc++.so.6',
                        'libgcc_s.so.1', 'libcuda.so.1'}
    destination = staged_prefix / 'lib'
    destination.mkdir(exist_ok=True)
    if (staged_prefix / 'bin/autoviz').exists():
        query = subprocess.run(['qmake6', '-query', 'QT_INSTALL_PLUGINS'],
                               check=True, capture_output=True, text=True)
        plugin_root = Path(query.stdout.strip())
        if not (plugin_root / 'platforms/libqxcb.so').is_file():
            raise ValueError('Qt 6 XCB platform plugin is required for the Autoviz runtime bundle')
        for category in ('platforms', 'xcbglintegrations', 'egldeviceintegrations', 'imageformats', 'iconengines'):
            source = plugin_root / category
            if source.is_dir():
                target = destination / 'qt6/plugins' / category
                shutil.copytree(source, target, dirs_exist_ok=True)
        (staged_prefix / 'bin/qt.conf').write_text('[Paths]\nPrefix=..\nPlugins=lib/qt6/plugins\n')
    copied = set()
    plugins = sorted((destination / 'qt6/plugins').rglob('*.so')) + sorted((destination / 'OGRE').glob('*.so*'))
    for installed in sorted((staged_prefix / 'bin').glob('*')) + plugins:
        if not installed.is_file():
            continue
        with installed.open('rb') as stream:
            if stream.read(4) != b'\x7fELF':
                continue
        # Build binaries retain the SDK search paths that install-time RPATH stripping removes.
        executable = build_dir / 'bin' / installed.name
        if installed.parent != staged_prefix / 'bin' or not executable.exists():
            executable = installed
        env = os.environ.copy()
        if executable == installed:
            env['LD_LIBRARY_PATH'] = os.pathsep.join((str(installed.parent), str(destination), env.get('LD_LIBRARY_PATH', '')))
        result = subprocess.run(['ldd', str(executable)], capture_output=True, text=True, env=env)
        if 'not a dynamic executable' in result.stdout + result.stderr:
            continue
        if result.returncode != 0 or '=> not found' in result.stdout:
            raise ValueError(f'Cannot resolve runtime libraries for {installed.name}')
        for line in result.stdout.splitlines():
            match = re.fullmatch(r'\s*(\S+)\s+=>\s+(/.*?)\s+\(0x[0-9a-fA-F]+\)\s*', line)
            if not match:
                continue
            name, source = match.groups()
            if name in target_libraries or name.startswith(('libnvidia-', 'libGLX_nvidia')):
                continue
            if not (destination / name).exists():
                shutil.copy2(Path(source).resolve(), destination / name)
                copied.add(name)
    return sorted(copied)


def configure_runtime_rpaths(staged_prefix: Path) -> None:
    """Keep real executables; each ELF resolves its own dependencies relative to lib/."""
    if not shutil.which('patchelf'):
        raise ValueError('Runtime bundling requires patchelf on the build machine')
    library_dir = staged_prefix / 'lib'
    for directory in (staged_prefix / 'bin', library_dir):
        for executable in sorted(directory.rglob('*')):
            if executable.is_symlink() or not executable.is_file():
                continue
            with executable.open('rb') as stream:
                if stream.read(4) != b'\x7fELF':
                    continue
            result = subprocess.run(['patchelf', '--print-rpath', str(executable)],
                                    capture_output=True, text=True)
            if result.returncode and 'statically linked' in result.stderr:
                continue
            result.check_returncode()
            relative = os.path.relpath(library_dir, executable.parent)
            rpath = '$ORIGIN' if relative == '.' else '$ORIGIN:$ORIGIN/' + relative
            subprocess.run(['patchelf', '--set-rpath', rpath, str(executable)], check=True)


def deduplicate_runtime_libraries(library_dir: Path) -> int:
    """Hardlink byte-identical ELF libraries while retaining every loader path and SONAME alias."""
    library_dir = library_dir.resolve(strict=True)
    seen = {}
    saved = 0
    for path in sorted(library_dir.rglob('*.so*')):
        if path.is_symlink() or not path.is_file():
            continue
        with path.open('rb') as stream:
            if stream.read(4) != b'\x7fELF':
                continue
            stream.seek(0)
            digest = hashlib.sha256()
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        metadata = path.stat()
        key = (metadata.st_size, metadata.st_mode, metadata.st_uid, metadata.st_gid, digest.digest())
        original = seen.setdefault(key, path)
        if original.stat().st_ino == metadata.st_ino:
            continue
        with tempfile.TemporaryDirectory(prefix='.deduplicate-', dir=path.parent) as temporary:
            replacement = Path(temporary) / path.name
            os.link(original, replacement)
            os.replace(replacement, path)
        if metadata.st_nlink == 1:
            saved += metadata.st_size
    return saved


def git_describe(root: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(root), "describe", "--tags", "--always", "--dirty"],
            check=True,
            capture_output=True,
            text=True,
        )
        return out.stdout.strip() or "dev"
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "dev"


def git_sha(root: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
        return out.stdout.strip() or "unknown"
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def default_jobs() -> int:
    try:
        return len(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        return os.cpu_count() or 4


def run(cmd: list[str], *, env: dict[str, str] | None = None) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True, env=env)


def write_archive(staged_prefix: Path, output: Path, manifest: dict, *, reproducible: bool = False) -> str:
    """Package a staged install without running any of its executables."""
    manifest_path = staged_prefix / "share/autonomy/MANIFEST.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if reproducible:
        def normalize(member):
            member.mtime = member.uid = member.gid = 0
            member.uname = member.gname = ''
            return member

        with output.open('wb') as stream, gzip.GzipFile(filename='', mode='wb', fileobj=stream, mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode='w') as archive:
                archive.add(staged_prefix, arcname=staged_prefix.name, filter=normalize)
    else:
        run(["tar", "-C", str(staged_prefix.parent), "-czf", str(output), staged_prefix.name],
            env={**os.environ, "COPYFILE_DISABLE": "1"})
    digest = hashlib.sha256()
    with output.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    root = repository_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Package an already configured and built CMake tree without rebuilding",
    )
    parser.add_argument(
        "--build-dir",
        default=os.environ.get("BUILD_DIR", str(root / "build")),
    )
    parser.add_argument(
        "--output",
        default=os.environ.get(
            "OUTPUT",
            str(Path.home() / ".autonomy/artifacts" / f"autonomy-{git_describe(root)}.tar.gz"),
        ),
    )
    parser.add_argument(
        "--prefix",
        default=os.environ.get("INSTALL_PREFIX", "/opt/autonomy"),
    )
    parser.add_argument(
        "--no-config",
        action="store_true",
        help="Do not bundle autonomy/*/conf into the tarball",
    )
    parser.add_argument('--bundle-runtime', action='store_true',
                        help='Bundle linked libraries and GUI plugins with relative ELF search paths; exclude libc/C++ runtime and GPU drivers')
    parser.add_argument('--reproducible', action='store_true',
                        help='Keep archive identity stable for identical staged files and source metadata')
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=int(os.environ.get("JOBS", default_jobs())),
    )
    args = parser.parse_args(argv)

    build_dir = Path(args.build_dir)
    output = Path(args.output).expanduser()
    install_prefix = Path(args.prefix)
    if not install_prefix.is_absolute() or install_prefix == Path("/") or ".." in install_prefix.parts:
        parser.error("--prefix must be an absolute non-root install path")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    cmake_build_type = os.environ.get("CMAKE_BUILD_TYPE", "Release")
    build_grpc = os.environ.get("BUILD_GRPC", "ON")
    build_test = os.environ.get("BUILD_TEST", "OFF")
    bundle_config = (
        False
        if args.no_config
        else os.environ.get("BUNDLE_CONFIG", "ON").upper() in ("1", "ON", "TRUE", "YES")
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    build_dir.mkdir(parents=True, exist_ok=True)

    stage = Path(tempfile.mkdtemp(prefix="autonomy-pkg-"))
    try:
        if not args.skip_build:
            run([sys.executable, str(root / 'tools/python/clang_format_sources.py'), '--check'])
            print(f"[package] configure: {build_dir}")
            run(
                [
                    "cmake",
                    "-G",
                    "Ninja",
                    "-S",
                    str(root),
                    "-B",
                    str(build_dir),
                    f"-DCMAKE_BUILD_TYPE={cmake_build_type}",
                    f"-DCMAKE_INSTALL_PREFIX={install_prefix}",
                    f"-DBUILD_GRPC={build_grpc}",
                    f"-DBUILD_TEST={build_test}",
                ]
            )
        cache = {}
        for line in (build_dir / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
            if not line.startswith(("#", "//")) and "=" in line and ":" in line.split("=", 1)[0]:
                key, value = line.split("=", 1)
                cache[key.split(":", 1)[0]] = value
        if cache.get('AUTONOMY_BUILD_MODULES'):
            parser.error('the release archive requires a full build directory')
        if not args.skip_build:
            run(["cmake", "--build", str(build_dir), "--parallel", str(args.jobs)])

        print(f"[package] build + stage install under {stage}")
        env = os.environ.copy()
        env["DESTDIR"] = str(stage)
        run(["cmake", "--install", str(build_dir), "--prefix", str(install_prefix)], env=env)

        staged_prefix = stage / install_prefix.relative_to("/")
        if bundle_config:
            print("[package] bundle autonomy/*/conf -> share/autonomy/config")
            bundle_configuration(root, staged_prefix)
        runtime_libraries = bundle_runtime_libraries(build_dir, staged_prefix) if args.bundle_runtime else []
        deduplicated_bytes = 0
        if args.bundle_runtime:
            configure_runtime_rpaths(staged_prefix)
            deduplicated_bytes = deduplicate_runtime_libraries(staged_prefix / 'lib')
            print(f'[package] deduplicated runtime libraries: {deduplicated_bytes} bytes saved')

        manifest = {
            "schema_version": 1,
            "git_sha": git_sha(root),
            "git_describe": git_describe(root),
            "install_prefix": str(install_prefix),
            "architecture": platform.machine(),
            "cmake_build_type": cache.get("CMAKE_BUILD_TYPE", "unknown"),
            "build_grpc": cache.get("BUILD_GRPC", "unknown"),
            "build_test": cache.get("BUILD_TEST", "unknown"),
            "config_bundled": bundle_config,
            "bundled_runtime_libraries": runtime_libraries,
            "deduplicated_runtime_bytes": deduplicated_bytes,
        }
        if not args.reproducible:
            manifest["packaged_at"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        digest = write_archive(staged_prefix, output, manifest, reproducible=args.reproducible)
        output.with_name(output.name + ".sha256").write_text(
            f"{digest}  {output.name}\n", encoding="utf-8"
        )
        print(f"[package] wrote {output}")
    finally:
        shutil.rmtree(stage, ignore_errors=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
