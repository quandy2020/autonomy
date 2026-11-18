#!/usr/bin/env python3
"""Build real native application targets without starting shared task endpoints.

Domain-library CTests do not verify the three application entry points. This
check uses the existing container CMake cache, verifies the synchronized source
inventory before and after building, and records application --help results.
It does not start exploration, send commands, or certify a simulator run.
"""

import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


REMOTE_PROGRAM = r'''
from pathlib import Path
import base64, fcntl, hashlib, json, shlex, subprocess, tempfile

manifest = __MANIFEST__
container = __CONTAINER__
jobs = __JOBS__
with_localization = __WITH_LOCALIZATION__
opencv_config = __OPENCV_CONFIG__
root = Path(manifest['remote_root']).resolve(strict=True)
build = '/workspace/autonomy/build/autonomy'
targets = ('autonomy.task', 'autonomy.planning', 'autonomy.control')
if with_localization:
    targets += ('autonomy.localization',)
logs = Path(tempfile.mkdtemp(prefix='opt-native-applications-'))

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None

def check_sources():
    expected = manifest['complete_source_inventory']
    drift = [name for name, value in expected.items() if digest(root / name) != value]
    if drift:
        raise RuntimeError('synchronized source drift: ' + ', '.join(drift))
    current = {str(p.relative_to(root)): digest(p)
               for p in sorted((root / 'autonomy/map').rglob('*')) if p.is_file()}
    if current != manifest['protected_map_sha256']:
        raise RuntimeError('protected map changed')
    if any(digest(root / name) != value for name, value in
           manifest['protected_task_server_sha256'].items()):
        raise RuntimeError('protected TaskServer or CMake changed')

def emit(event):
    print(json.dumps(event), flush=True)

def run(stage, inner, seconds):
    command = ['docker', 'exec', '-w', build, container, 'timeout',
               '--signal=TERM', '--kill-after=10', str(seconds),
               '/bin/bash', '-lc', shlex.join(inner)]
    path = logs / (stage + '.log')
    emit({'event': 'start', 'stage': stage, 'remote_log': str(path)})
    with path.open('xb') as output:
        result = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT)
    emit({'event': 'step', 'stage': stage, 'returncode': result.returncode,
          'command': command, 'remote_log': str(path),
          'log_base64': base64.b64encode(path.read_bytes()).decode()})
    return result.returncode == 0

try:
    with open('/tmp/opt-native-applications-validation.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        check_sources()
        passed = True
        if with_localization:
            # Preserve the existing dependency/toolchain cache. Enabling this
            # domain constructs its actual Atlas/Argus targets; no mock SLAM is
            # linked and no source CMake file is rewritten.
            configure = ['cmake', '-S',
                '/workspace/autonomy/src/autonomy', '-B', build,
                '-DAUTONOMY_BUILD_LOCALIZATION=ON', '-DATLAS_BUILD_EXAMPLES=OFF',
                '-DATLAS_BUILD_TESTS=OFF', '-DARGUS_BUILD_APPS=OFF',
                '-DARGUS_BUILD_TESTS=OFF']
            if opencv_config:
                configure.append('-DOpenCV_DIR=' + opencv_config)
            passed = run('configure-localization', configure, 180)
        if passed:
            passed = run('build', ['cmake', '--build', build, '--target',
                                  *targets, '-j', str(jobs)], 1200)
        if passed:
            for target in targets:
                if not run(target + '-help', [build + '/bin/' + target, '--help'], 30):
                    passed = False
        check_sources()
        emit({'event': 'finished', 'applications_verified': passed,
              'map_unchanged': True, 'task_server_preserved': True,
              'protected_map_file_count': len(manifest['protected_map_sha256'])})
except Exception as error:
    emit({'event': 'finished', 'applications_verified': False,
          'error': f'{type(error).__name__}: {error}'})
'''


def main():
    """Retain each application log and a bounded statement of verified behavior."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='quandy@192.168.66.124')
    parser.add_argument('--container', default='SpaceHero')
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--with-localization', action='store_true',
                        help='Enable the existing localization domain and verify its real application too')
    parser.add_argument('--opencv-config', type=str, default='',
                        help='Explicit container OpenCV config directory for localization reconfiguration')
    args = parser.parse_args()
    if args.report.exists():
        parser.error('report already exists')
    if args.host.startswith('-') or args.container.startswith('-') or not 1 <= args.jobs <= 16:
        parser.error('invalid host, container or job count')
    if args.opencv_config and (not args.with_localization or
            not Path(args.opencv_config).is_absolute() or '..' in Path(args.opencv_config).parts):
        parser.error('OpenCV config requires localization and an absolute container path')
    manifest = json.loads(args.manifest.read_text())
    program = REMOTE_PROGRAM.replace('__MANIFEST__', repr(manifest))
    program = program.replace('__CONTAINER__', repr(args.container)).replace('__JOBS__', repr(args.jobs))
    program = program.replace('__WITH_LOCALIZATION__', repr(args.with_localization))
    program = program.replace('__OPENCV_CONFIG__', repr(args.opencv_config))
    report = {'applications_verified': False, 'steps': [],
              'manifest_sha256': hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
              'validator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'localization_requested': args.with_localization,
              'opencv_config': args.opencv_config,
              'exploration_task_started': False, 'coverage_comparison_performed': False,
              'scope': 'Production applications build and CLI only; no exploration acceptance.'}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryFile(mode='w+') as input_file:
        input_file.write(program)
        input_file.seek(0)
        process = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                                    args.host, 'python3', '-'], stdin=input_file,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in process.stdout:
            try:
                event = json.loads(line)
            except ValueError:
                report.setdefault('transport_output', []).append(line.rstrip())
                continue
            kind = event.pop('event')
            if kind == 'step':
                data = base64.b64decode(event.pop('log_base64'), validate=True)
                log = args.report.with_name(args.report.stem + '-' + event['stage'] + '.log')
                with log.open('xb') as output:
                    output.write(data)
                event['log'] = str(log)
                event['log_sha256'] = hashlib.sha256(data).hexdigest()
                report['steps'].append(event)
                print(event['stage'], 'returncode', event['returncode'], flush=True)
            elif kind == 'start':
                print(event['stage'], 'started;', event['remote_log'], flush=True)
            else:
                report.update(event)
        report['ssh_returncode'] = process.wait()
    report['applications_verified'] = bool(report['applications_verified'] and
        report['ssh_returncode'] == 0 and len(report['steps']) == (6 if args.with_localization else 4) and
        all(step['returncode'] == 0 for step in report['steps']))
    with args.report.open('x') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps({'report': str(args.report),
                      'applications_verified': report['applications_verified'],
                      'error': report.get('error')}), flush=True)
    return 0 if report['applications_verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
