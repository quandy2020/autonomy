#!/usr/bin/env python3
"""Build/package once in Docker; return public deployment variables as JSON."""
import argparse
from contextlib import redirect_stdout
from datetime import datetime, timezone
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import tarfile
import tempfile
import uuid

import yaml

from inspect_artifact import inspect_artifact

ROOT = Path(__file__).resolve().parents[1]


def run(*arguments):
    subprocess.run(list(arguments), check=True, stdout=sys.stderr)


def prepare(config, *, incremental=False):
    required = {'container', 'workspace', 'build_dir', 'build_args', 'artifact_path', 'install_prefix'}
    if not isinstance(config, dict) or not required <= set(config) or set(config) - required - {'bundle_runtime'}:
        raise ValueError('Release configuration requires: ' + ', '.join(sorted(required)))
    if not isinstance(config.get('bundle_runtime', False), bool):
        raise ValueError('bundle_runtime must be a boolean')
    for key in required - {'build_args'}:
        if not isinstance(config[key], str) or not config[key].strip():
            raise ValueError(f'{key} must be a nonempty string')
    if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]*', config['container']):
        raise ValueError('container must be a Docker container name or ID')
    for key in ['workspace', 'build_dir', 'install_prefix']:
        path = PurePosixPath(config[key])
        if not path.is_absolute() or '..' in path.parts or str(path) == '/':
            raise ValueError(f'{key} must be an absolute non-root path without parent traversal')
    if not isinstance(config['build_args'], list) or not all(isinstance(arg, str) for arg in config['build_args']):
        raise ValueError('build_args must be a list of strings')

    artifact = (ROOT / Path(config['artifact_path']).expanduser()).resolve()
    if not artifact.name.endswith('.tar.gz') or artifact.is_dir():
        raise ValueError('artifact_path must name a .tar.gz file')
    artifact.parent.mkdir(parents=True, exist_ok=True)
    container = config['container']
    workspace = config['workspace']
    stage = '/tmp/autonomy-release-' + uuid.uuid4().hex
    try:
        run('docker', 'exec', '--workdir', workspace, container, 'colcon', 'build', *config['build_args'])
        run('docker', 'exec', '--workdir', workspace, container, 'python3',
            workspace + '/src/autonomy/tools/python/package_install_tarball.py',
            '--skip-build', '--build-dir', config['build_dir'], '--prefix', config['install_prefix'],
            '--output', stage + '/' + artifact.name,
            *(['--reproducible'] if incremental else []),
            *(['--bundle-runtime'] if config.get('bundle_runtime', False) else []))
        with tempfile.TemporaryDirectory(prefix='.autonomy-release-', dir=artifact.parent) as temporary:
            staged = Path(temporary) / artifact.name
            run('docker', 'cp', container + ':' + stage + '/' + artifact.name, str(staged))
            metadata = inspect_artifact(staged, config['install_prefix'])
            checksum = Path(str(staged) + '.sha256')
            checksum.write_text(f"{metadata['sha256']}  {artifact.name}\n")
            os.replace(staged, artifact)
            os.replace(checksum, Path(str(artifact) + '.sha256'))
    finally:
        subprocess.run(['docker', 'exec', container, 'rm', '-rf', '--', stage],
                       check=False, stdout=sys.stderr)
    return {'autonomy_artifact_path': str(artifact), 'autonomy_artifact_sha256': metadata['sha256'],
            'autonomy_install_prefix': config['install_prefix']}


def prepare_cross(config, *, incremental=False):
    root = ROOT.parent
    cross_scripts = root / 'tools/cross-build/scripts'
    sys.path.insert(0, str(cross_scripts))
    sys.path.insert(0, str(root / 'tools/python'))
    from cross_build import check_elf, load_profile
    from package_install_tarball import bundle_configuration, git_describe, git_sha, write_archive

    allowed = {'cross_profile', 'artifact_dir', 'install_prefix', 'build_args',
               'sysroot', 'image', 'docker_platform'}
    if not isinstance(config, dict) or not config.get('cross_profile') or config.keys() - allowed:
        raise ValueError('cross release requires cross_profile and supported release settings')
    for field in allowed - {'sysroot', 'build_args'}:
        if field in config and (not isinstance(config[field], str) or not config[field].strip()):
            raise ValueError(f'{field} must be a nonempty string')
    meta = load_profile(config['cross_profile'])
    profile = meta['platform']
    prefix = config.get('install_prefix', '/opt/autonomy')
    install_prefix = PurePosixPath(prefix)
    if (not install_prefix.is_absolute() or install_prefix.anchor != '/' or not install_prefix.name
            or '..' in install_prefix.parts or str(install_prefix) != prefix
            or any(char.isspace() for char in prefix)):
        raise ValueError('install_prefix must be an absolute non-root path')
    build_args = config.get('build_args', [])
    protected = {'CMAKE_TOOLCHAIN_FILE', 'CMAKE_INSTALL_PREFIX', 'CMAKE_SYSROOT', 'CROSS_SYSROOT'}
    if not isinstance(build_args, list) or any(
            not isinstance(arg, str) or not re.fullmatch(r'-D[A-Za-z_][A-Za-z0-9_]*(?::[A-Z]+)?=.*', arg)
            or arg[2:].split('=', 1)[0].split(':', 1)[0] in protected for arg in build_args):
        raise ValueError('build_args must be CMake -D arguments and preserve the toolchain, sysroot and install prefix')

    command = (sys.executable, str(cross_scripts / 'cross_build.py'))
    options = []
    if 'sysroot' in config:
        source = config['sysroot']
        if not isinstance(source, dict) or not {'host', 'user'} <= source.keys() or source.keys() - {'host', 'user', 'extra_dirs'}:
            raise ValueError('sysroot needs host and user, with optional extra_dirs')
        if any(not isinstance(source[key], str) or not source[key] for key in ('host', 'user')):
            raise ValueError('sysroot host and user must be nonempty strings')
        extra_dirs = source.get('extra_dirs', [])
        if not isinstance(extra_dirs, list) or any(not isinstance(value, str) for value in extra_dirs):
            raise ValueError('sysroot.extra_dirs must be a list of paths')
        options = ['--refresh-sysroot', '--host', source['host'], '--user', source['user']]
        for path in extra_dirs:
            options += ['--extra-dir', path]

    for field in ('image', 'docker_platform'):
        if field in config:
            options += ['--' + field.replace('_', '-'), config[field]]
    if incremental:
        options.append('--incremental')

    artifact_dir = (ROOT / Path(config.get('artifact_dir', '~/.autonomy/artifacts')).expanduser()).resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.autonomy-cross-', dir=artifact_dir) as temporary:
        stage = Path(temporary)
        run(*command, 'install', '--profile', profile, '--source', str(root),
            '--output-dir', str(stage), '--install-prefix', prefix, *options, '--', *build_args)
        installed = stage / 'install' / install_prefix.relative_to('/')
        executables = 0
        for path in installed.rglob('*'):
            if path.is_symlink() or not path.is_file():
                continue
            with path.open('rb') as stream:
                header = stream.read(18)
            if not header.startswith(b'\x7fELF'):
                continue
            byte_order = 'little' if header[5:6] == b'\x01' else 'big'
            if int.from_bytes(header[16:18], byte_order) in (2, 3):
                check_elf(path, meta)
                if path.parent == installed / 'bin' and path.stat().st_mode & 0o111:
                    executables += 1
        if not executables:
            raise ValueError('cross install did not produce a target ELF executable under bin/')
        bundle_configuration(root, installed)
        aarch64 = meta['arch'] == 'aarch64'
        manifest = {
            'cross_profile': profile, 'architecture': 'aarch64' if aarch64 else 'armv7l',
            'target_dpkg_arch': 'arm64' if aarch64 else 'armhf',
            'target_os': meta['distro'], 'target_version': meta['distro_version'],
            'git_sha': git_sha(root), 'git_describe': git_describe(root), 'install_prefix': prefix,
            'config_bundled': True,
        }
        if not incremental:
            manifest['packaged_at'] = datetime.now(timezone.utc).isoformat()
        staged = stage / 'autonomy.tar.gz'
        with redirect_stdout(sys.stderr):
            write_archive(installed, staged, manifest, reproducible=incremental)
        metadata = inspect_artifact(staged, prefix)
        artifact = artifact_dir / f"autonomy-{profile}-{metadata['sha256']}.tar.gz"
        checksum = stage / 'autonomy.sha256'
        checksum.write_text(f"{metadata['sha256']}  {artifact.name}\n")
        os.replace(staged, artifact)
        os.replace(checksum, artifact.with_name(artifact.name + '.sha256'))
    print(f'[cross-release] Artifact: {artifact}', file=sys.stderr)
    return {'autonomy_artifact_path': str(artifact), 'autonomy_artifact_sha256': metadata['sha256'],
            'autonomy_install_prefix': prefix, 'autonomy_cross_release_profile': profile}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--cross', action='store_true')
    parser.add_argument('--incremental', action='store_true')
    arguments = parser.parse_args()
    try:
        config = yaml.safe_load(arguments.config.read_text())
        build_release = prepare_cross if arguments.cross else prepare
        result = build_release(config, incremental=arguments.incremental)
    except (OSError, ValueError, KeyError, tarfile.TarError, yaml.YAMLError, subprocess.CalledProcessError) as error:
        # Configuration and tool output may contain private values; do not dump them.
        print(f'Release preparation failed ({type(error).__name__}); check the build configuration and preceding tool output. Deployment was not started.', file=sys.stderr)
        if arguments.cross and isinstance(error, ValueError):
            print(str(error), file=sys.stderr)
        return 1
    print(json.dumps(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
