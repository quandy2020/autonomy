#!/usr/bin/env python3
"""Validate an install tarball before any target is changed; print public metadata."""
import argparse
import hashlib
import json
import posixpath
import tarfile
from pathlib import Path, PurePosixPath


def inspect_artifact(path: Path, prefix: str) -> dict:
    root = PurePosixPath(prefix).name
    if (not prefix.startswith('/') or prefix == '/' or not root
            or posixpath.normpath(prefix) != prefix or any(char.isspace() for char in prefix)):
        raise ValueError('install prefix must be an absolute non-root path')
    with tarfile.open(path, 'r:gz') as archive:
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
        if manifest.get('install_prefix') != prefix:
            raise ValueError('artifact install prefix does not match the deployment prefix')
        if manifest.get('architecture') not in ('x86_64', 'aarch64'):
            raise ValueError('artifact needs an x86_64 or aarch64 architecture manifest')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    checksum = digest.hexdigest()
    return {'sha256': checksum, 'architecture': manifest['architecture'],
            'git_sha': manifest.get('git_sha', 'unknown'), 'install_prefix': prefix}


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
