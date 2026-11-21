#!/usr/bin/env python3
"""Validate an install tarball before any target is changed; print public metadata."""
import argparse
import hashlib
import json
import posixpath
import re
import tarfile
from pathlib import Path, PurePosixPath


CROSS_ABIS = {
    'rockchip-rk3506-debian12': ('armv7l', 'armhf', 'debian', '12', 'arm-linux-gnueabihf'),
    **{profile: ('aarch64', 'arm64', 'ubuntu', '22.04', 'aarch64-linux-gnu') for profile in (
        'rockchip-rk3588-ubuntu2204', 'nvidia-agx-orin-jetpack6',
        'nvidia-orin-nx-jetpack6', 'nvidia-orin-nano-jetpack6',
    )},
}


def inspect_artifact(path: Path, prefix: str) -> dict:
    root = PurePosixPath(prefix).name
    if (not prefix.startswith('/') or prefix == '/' or not root
            or posixpath.normpath(prefix) != prefix or any(char.isspace() for char in prefix)):
        raise ValueError('install prefix must be an absolute non-root path')
    with path.open('rb') as stream, tarfile.open(fileobj=stream, mode='r:gz') as archive:
        members = archive.getmembers()
        links = set()
        names = set()
        for member in members:
            name = PurePosixPath(member.name)
            if name.is_absolute() or '..' in name.parts or not name.parts or name.parts[0] != root:
                raise ValueError('archive path escapes the install prefix')
            if str(name) in names:
                raise ValueError('duplicate archive entry')
            names.add(str(name))
            if not (member.isdir() or member.isfile() or member.issym() or member.islnk()):
                raise ValueError('archive contains a special device or unsupported entry')
            if member.issym() or member.islnk():
                target = member.linkname
                if member.issym():
                    target = posixpath.join(str(name.parent), target)
                normalized = PurePosixPath(posixpath.normpath(target))
                if (PurePosixPath(member.linkname).is_absolute() or not normalized.parts
                        or normalized.parts[0] != root):
                    raise ValueError('archive link escapes the install prefix')
                links.add(str(name))
        for name in names:
            if any(str(parent) in links for parent in PurePosixPath(name).parents):
                raise ValueError('archive writes through a link')
        manifest_name = f'{root}/share/autonomy/MANIFEST.json'
        member = archive.getmember(manifest_name)
        if not member.isfile() or member.size > 65536:
            raise ValueError('invalid or oversized manifest')
        manifest = json.load(archive.extractfile(member))
        if not isinstance(manifest, dict) or type(manifest.get('schema_version', 0)) is not int:
            raise ValueError('invalid artifact manifest')
        schema = manifest.get('schema_version', 0)
        if schema not in (0, 1):
            raise ValueError('unsupported artifact manifest schema')
        if manifest.get('install_prefix') != prefix:
            raise ValueError('artifact install prefix does not match the deployment prefix')
        architecture = manifest.get('architecture')
        target = {}
        if 'cross_profile' in manifest:
            target = {key: manifest.get(key) for key in
                      ('cross_profile', 'target_dpkg_arch', 'target_os', 'target_version')}
            profile = target['cross_profile']
            abi = CROSS_ABIS.get(profile) if isinstance(profile, str) else None
            if (abi is None or
                    (architecture, target['target_dpkg_arch'], target['target_os'], target['target_version']) != abi[:4]
                    or (schema == 1 and manifest.get('target_abi') != abi[4])):
                raise ValueError('invalid cross target in artifact manifest')
            target['target_abi'] = abi[4]
            for key, pattern in (('sysroot_sha256', r'[0-9a-f]{64}'), ('image_id', r'sha256:[0-9a-f]{64}')):
                value = manifest.get(key)
                if value is not None:
                    if not isinstance(value, str) or not re.fullmatch(pattern, value):
                        raise ValueError(f'invalid artifact {key}')
                    target[key] = value
        elif architecture not in ('x86_64', 'aarch64'):
            raise ValueError('artifact needs an x86_64 or aarch64 architecture manifest')
        if 'deployment_base_sha256' in manifest or 'modules' in manifest:
            base, modules = manifest.get('deployment_base_sha256'), manifest.get('modules')
            if (not target.get('sysroot_sha256') or not target.get('image_id') or schema != 1
                    or not isinstance(base, str) or not re.fullmatch('[0-9a-f]{64}', base)
                    or not isinstance(modules, list) or not modules
                    or any(not isinstance(name, str) or not re.fullmatch(
                        r'[a-z][a-z0-9_]*(/[a-z][a-z0-9_]*)?', name) for name in modules)):
                raise ValueError('invalid modular artifact metadata')
            target.update(deployment_base_sha256=base, modules=modules)
        digest = hashlib.sha256()
        stream.seek(0)
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    checksum = digest.hexdigest()
    return {'sha256': checksum, 'architecture': architecture, 'schema_version': schema,
            'git_sha': manifest.get('git_sha', 'unknown'), 'install_prefix': prefix, **target}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('artifact', type=Path)
    parser.add_argument('--prefix', required=True)
    args = parser.parse_args()
    try:
        print(json.dumps(inspect_artifact(args.artifact, args.prefix)))
    except (ValueError, KeyError, OSError, tarfile.TarError) as error:
        parser.exit(1, f'Invalid artifact: {error}\n')


if __name__ == '__main__':
    main()
