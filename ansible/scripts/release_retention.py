#!/usr/bin/env python3
"""List managed release cleanup candidates without deleting anything."""
import argparse
import json
from pathlib import Path
import re


def retention(prefix, directory, keep):
    digest = re.compile(r'[0-9a-f]{64}')
    if directory.is_symlink() or not directory.is_dir() or keep < 0:
        raise ValueError('expected a real release directory and nonnegative keep count')
    directory = directory.resolve()
    entries = {path.name: path for path in directory.iterdir()
               if digest.fullmatch(path.name) and not path.is_symlink() and path.is_dir()}
    protected = {}

    def protect(path, reason):
        if path.parent == directory and path.name in entries:
            protected.setdefault(path.name, []).append(reason)

    for path, reason in ((prefix, 'active'), (prefix.with_name(prefix.name + '.previous'), 'previous')):
        if path.is_symlink():
            protect(path.resolve(strict=True), reason)
    checkpoint = directory / '.last-verified'
    if checkpoint.exists():
        if not checkpoint.is_file() or checkpoint.is_symlink():
            raise ValueError('invalid verification checkpoint')
        protect(Path(json.loads(checkpoint.read_text())['release']), 'verified')
    newest = sorted(entries, key=lambda name: (entries[name].stat().st_mtime_ns, name), reverse=True)
    for name in newest[:keep]:
        protect(entries[name], 'recent')

    pending = list(protected)
    seen = set()
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        seen.add(name)
        manifest = entries[name] / 'share/autonomy/MANIFEST.json'
        if not manifest.is_file() or manifest.resolve() != manifest:
            raise ValueError(f'cannot verify the ancestry of {name}')
        parent = json.loads(manifest.read_text()).get('deployment_base_sha256')
        if parent:
            if not isinstance(parent, str) or not digest.fullmatch(parent) or parent not in entries:
                raise ValueError(f'missing or invalid module base for {name}')
            protect(entries[parent], 'module base')
            pending.append(parent)

    lock = directory / '.deployment-lock'
    busy = lock.exists() or lock.is_symlink()
    candidates = [] if busy else sorted(entries.keys() - protected.keys())
    return {'release_directory': str(directory), 'deployment_locked': busy,
            'protected': protected, 'cleanup_candidates': candidates}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path)
    parser.add_argument('release_directory', type=Path)
    parser.add_argument('--keep', type=int, default=3)
    args = parser.parse_args()
    try:
        print(json.dumps(retention(args.prefix, args.release_directory, args.keep)))
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'Release retention: {error}\n')


if __name__ == '__main__':
    main()
