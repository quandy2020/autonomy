#!/usr/bin/env python3
"""Apply an explicit native-source manifest with drift checks and backups.

The manifest is prepared from inspected live bytes. This tool never expands a
directory into an implicit write set, and never writes map or TaskServer files.
It retains original bytes and modes and refuses to roll back a concurrent edit.
Build/runtime validation is a separate step after this source synchronization.
"""

import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import sys


REMOTE_PROGRAM = r'''
from pathlib import Path
import base64, fcntl, hashlib, json, os, stat, tempfile

manifest = __MANIFEST__
payload = __PAYLOAD__
root = Path(manifest['remote_root']).resolve(strict=True)
report = {'deployed': False, 'written': [], 'rollback_conflicts': []}

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None

def target(name):
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise RuntimeError('invalid destination: ' + name)
    if name.startswith('autonomy/map/') or name in (
            'autonomy/task/task_server.cpp', 'autonomy/task/task_server.hpp'):
        raise RuntimeError('protected destination: ' + name)
    result = root / relative
    if result.is_symlink() or root not in result.resolve().parents:
        raise RuntimeError('symlink or escaping destination: ' + name)
    return result

def map_hashes():
    return {str(p.relative_to(root)): digest(p)
            for p in sorted((root / 'autonomy/map').rglob('*')) if p.is_file()}

def protect():
    if map_hashes() != manifest['protected_map_sha256']:
        raise RuntimeError('protected map changed; no map write is authorized')
    for name, expected in manifest['protected_task_server_sha256'].items():
        if digest(root / name) != expected:
            raise RuntimeError('live TaskServer changed: ' + name)

def replace(path, data, mode):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.opt-sync-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as output:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(name, mode)
        os.replace(name, path)
    finally:
        if os.path.exists(name): os.unlink(name)

lock_path = '/tmp/opt-native-sync-' + hashlib.sha256(str(root).encode()).hexdigest()[:16] + '.lock'
with open(lock_path, 'a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    try:
        if set(payload) != {entry['destination'] for entry in manifest['entries']}:
            raise RuntimeError('payload is not the exact manifest allowlist')
        protect()
        entries = manifest['entries']
        # Preflight every file before the first mutation.
        for entry in entries:
            path = target(entry['destination'])
            if path.exists() and not path.is_file():
                raise RuntimeError('destination is not a regular file')
            if digest(path) != entry['expected_remote_sha256']:
                raise RuntimeError('remote drift: ' + entry['destination'])
            incoming = base64.b64decode(payload[entry['destination']], validate=True)
            if hashlib.sha256(incoming).hexdigest() != entry['sha256']:
                raise RuntimeError('payload hash mismatch: ' + entry['destination'])

        stage = Path(tempfile.mkdtemp(prefix='opt-native-sync-'))
        report['backup'] = str(stage / 'backup')
        report['expected_sha256'] = {e['destination']: e['expected_remote_sha256'] for e in entries}
        originals = {}
        for entry in entries:
            name = entry['destination']; path = target(name)
            data = path.read_bytes() if path.exists() else None
            mode = stat.S_IMODE(path.stat().st_mode) if data is not None else 0o644
            originals[name] = (data, mode)
            if data is not None:
                backup = stage / 'backup' / name
                backup.parent.mkdir(parents=True, exist_ok=True)
                backup.write_bytes(data); os.chmod(backup, mode)
        (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

        for entry in entries:
            protect()
            path = target(entry['destination'])
            if digest(path) != entry['expected_remote_sha256']:
                raise RuntimeError('remote drift during commit: ' + entry['destination'])
            replace(path, base64.b64decode(payload[entry['destination']]), originals[entry['destination']][1])
            report['written'].append(entry['destination'])
        protect()
        report['deployed_sha256'] = {e['destination']: digest(target(e['destination'])) for e in entries}
        if report['deployed_sha256'] != {e['destination']: e['sha256'] for e in entries}:
            raise RuntimeError('post-sync source mismatch')
        report['map_file_count'] = len(manifest['protected_map_sha256'])
        report['map_unchanged'] = True
        report['task_server_preserved'] = True
        report['deployed'] = True
    except Exception as error:
        report['error'] = str(error)
        expected_written = {e['destination']: e['sha256'] for e in manifest['entries']}
        for name in reversed(report['written']):
            path = target(name)
            if digest(path) != expected_written[name]:
                report['rollback_conflicts'].append(name)
                continue
            original, mode = originals[name]
            if original is None: path.unlink()
            else: replace(path, original, mode)
        report['rolled_back'] = bool(report['written']) and not report['rollback_conflicts']
print(json.dumps(report))
'''


def main():
    """Check exact local fingerprints and preserve one immutable sync attempt."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='quandy@192.168.66.124')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if args.report.exists(): parser.error('report already exists')
    if args.host.startswith('-'): parser.error('invalid host')
    manifest = json.loads(args.manifest.read_text())
    if not manifest.get('ready_to_sync'): parser.error('manifest is not ready')
    payload = {}
    for entry in manifest['entries']:
        data = (args.root / entry['source']).read_bytes()
        if hashlib.sha256(data).hexdigest() != entry['sha256']:
            parser.error('local source drift: ' + entry['source'])
        if entry['destination'] in payload: parser.error('duplicate destination')
        payload[entry['destination']] = base64.b64encode(data).decode()
    program = REMOTE_PROGRAM.replace('__MANIFEST__', repr(manifest)).replace('__PAYLOAD__', repr(payload))
    try:
        result = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                                 args.host, 'python3', '-'], input=program,
                                capture_output=True, text=True, timeout=180)
        report = {'ssh_returncode': result.returncode, 'ssh_error': result.stderr}
        if result.returncode == 0: report.update(json.loads(result.stdout))
        else: report['output'] = result.stdout
    except (subprocess.TimeoutExpired, ValueError) as error:
        # A connection timeout is not evidence that a remote transaction failed.
        report = {'deployed': None, 'attempt_error': str(error), 'remote_state_requires_reinspection': True}
    report['manifest_sha256'] = hashlib.sha256(args.manifest.read_bytes()).hexdigest()
    report['scope'] = 'Native source sync only; no full build, robot or physical stopping evidence.'
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x') as output:
        json.dump(report, output, indent=2); output.write('\n')
    print(json.dumps({'report': str(args.report), 'deployed': report.get('deployed'),
                      'files': len(report.get('written', [])), 'error': report.get('error', report.get('attempt_error'))}))
    return 0 if report.get('deployed') is True else 1


if __name__ == '__main__':
    sys.exit(main())
