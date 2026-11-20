#!/usr/bin/env python3
"""Build/package once in Docker; return public deployment variables as JSON."""
import argparse
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


def prepare(config):
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    arguments = parser.parse_args()
    try:
        result = prepare(yaml.safe_load(arguments.config.read_text()))
    except (OSError, ValueError, tarfile.TarError, yaml.YAMLError, subprocess.CalledProcessError) as error:
        # Configuration and tool output may contain private values; do not dump them.
        print(f'Release preparation failed ({type(error).__name__}); check the build configuration and preceding tool output. Deployment was not started.', file=sys.stderr)
        return 1
    print(json.dumps(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
