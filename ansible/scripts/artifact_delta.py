#!/usr/bin/env python3
"""Transfer an artifact as a verified delta against files already on the target."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import shutil
import tarfile
import tempfile


def digest_stream(source, destination=None):
    digest = hashlib.sha256()
    for chunk in iter(lambda: source.read(1024 * 1024), b''):
        digest.update(chunk)
        if destination is not None:
            destination.write(chunk)
    return digest.hexdigest()


def file_digest(path):
    with path.open('rb') as stream:
        return digest_stream(stream)


def inventory(basis):
    if basis.is_symlink() or not basis.is_dir():
        raise ValueError('incremental basis is not a release directory')
    files = {}
    for directory, _, names in os.walk(basis):
        for name in names:
            path = Path(directory) / name
            info = path.lstat()
            if stat.S_ISREG(info.st_mode):
                files[path.relative_to(basis).as_posix()] = [info.st_size, file_digest(path)]
    return files


def prepare(artifact, prefix, expected_sha256, files, work):
    root = PurePosixPath(prefix).name
    payload = work / 'delta.tar.gz'
    metadata = work / 'reuse.json'
    reuse = {}
    # The role has already inspected the archive. Hash and read the same open
    # file here so replacing the path cannot switch the input after validation.
    with artifact.open('rb') as raw:
        if digest_stream(raw) != expected_sha256:
            raise ValueError('artifact changed while preparing its delta')
        full_bytes = raw.tell()
        raw.seek(0)
        with tarfile.open(fileobj=raw, mode='r|gz') as original, tarfile.open(payload, 'w:gz') as output:
            for member in original:
                if member.sparse is not None:
                    return {'use_delta': False}
                if not member.isfile():
                    output.addfile(member)
                    continue
                relative = PurePosixPath(member.name).relative_to(root).as_posix()
                previous = files.get(relative)
                with original.extractfile(member) as source:
                    if (not previous or previous[0] != member.size
                            or member.pax_headers.keys() - {'path', 'size', 'mtime'}):
                        output.addfile(member, source)
                        continue
                    # The gzip reader is forward-only. Spool same-size candidates once.
                    with tempfile.TemporaryFile(dir=work) as candidate:
                        checksum = digest_stream(source, candidate)
                        if checksum == previous[1]:
                            reuse[relative] = [checksum, member.mode, member.mtime]
                        else:
                            candidate.seek(0)
                            output.addfile(member, candidate)
    metadata.write_text(json.dumps({'root': root, 'files': reuse}))
    wire_bytes = payload.stat().st_size + metadata.stat().st_size
    if not reuse or wire_bytes >= full_bytes:
        return {'use_delta': False}
    return {
        'use_delta': True,
        'reused': len(reuse),
        'wire_bytes': wire_bytes,
        'full_bytes': full_bytes,
        'payload_sha256': file_digest(payload),
        'reuse_sha256': file_digest(metadata),
    }


def prepare_cached(artifact, prefix, checksum, files, cache):
    # The basis is derived from actual target file hashes, never from its release label alone.
    identity = json.dumps([checksum, prefix, files], sort_keys=True, separators=(',', ':')).encode()
    key = hashlib.sha256(identity).hexdigest()
    payload = cache / key
    with (cache / f'{key}.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        marker = payload / 'plan.json'
        if marker.is_file():
            plan = json.loads(marker.read_text())
            if not plan['use_delta'] or all((payload / name).is_file() for name in ('reuse.json', 'delta.tar.gz')):
                return {**plan, 'payload_dir': str(payload), 'cache_hit': True}
        if payload.exists():
            shutil.rmtree(payload)
        with tempfile.TemporaryDirectory(prefix='.delta-', dir=cache) as staging:
            staged = Path(staging) / 'payload'
            staged.mkdir()
            plan = prepare(artifact, prefix, checksum, files, staged)
            (staged / 'plan.json').write_text(json.dumps(plan))
            os.replace(staged, payload)
        return {**plan, 'payload_dir': str(payload), 'cache_hit': False}


def relative_path(name):
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or '..' in path.parts:
        raise ValueError('unsafe incremental path')
    return path


def restore(basis, stage, metadata, expected_sha256):
    data = metadata.read_bytes()
    if hashlib.sha256(data).hexdigest() != expected_sha256:
        raise ValueError('incremental metadata checksum mismatch')
    plan = json.loads(data)
    basis = basis.resolve(strict=True)
    destination = stage / relative_path(plan['root'])
    for name, (checksum, mode, mtime) in plan['files'].items():
        relative = relative_path(name)
        source = basis / relative
        if source.resolve(strict=True) != source or not source.is_file():
            raise ValueError(f'invalid basis file: {name}')
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open('rb') as old, target.open('xb') as new:
            if digest_stream(old, new) != checksum:
                raise ValueError(f'basis file changed: {name}')
        target.chmod(mode & 0o7777)
        os.utime(target, (mtime, mtime))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    scan = commands.add_parser('inventory')
    scan.add_argument('basis', type=Path)
    generate = commands.add_parser('prepare')
    generate.add_argument('artifact', type=Path)
    generate.add_argument('prefix')
    generate.add_argument('checksum')
    generate.add_argument('inventory', type=Path)
    generate.add_argument('work', type=Path)
    generate.add_argument('--cache', type=Path)
    apply = commands.add_parser('restore')
    apply.add_argument('basis', type=Path)
    apply.add_argument('stage', type=Path)
    apply.add_argument('metadata', type=Path)
    apply.add_argument('checksum')
    args = parser.parse_args()
    try:
        if args.command == 'inventory':
            print(json.dumps(inventory(args.basis)))
        elif args.command == 'prepare':
            files = json.loads(args.inventory.read_text())
            if args.cache:
                plan = prepare_cached(args.artifact, args.prefix, args.checksum, files, args.cache)
            else:
                plan = prepare(args.artifact, args.prefix, args.checksum, files, args.work)
            print(json.dumps(plan))
        else:
            restore(args.basis, args.stage, args.metadata, args.checksum)
    except (KeyError, OSError, ValueError, tarfile.TarError) as error:
        parser.exit(1, f'Incremental deployment: {error}\n')


if __name__ == '__main__':
    main()
